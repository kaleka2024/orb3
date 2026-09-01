/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */
#include "Tracking.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "Converter.h"
#include "FrameDrawer.h"
#include "G2oTypes.h"
#include "GeometricTools.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Pinhole.h"

namespace ORB_SLAM3 {

/**
 * @brief Tracking类构造函数，负责初始化跟踪模块所有成员变量、加载配置参数、注册相机、初始化计时统计容器
 * @param pSys 系统顶层System共享指针
 * @param pVoc ORB词袋模型共享指针
 * @param pFrameDrawer 帧可视化绘制器
 * @param pMapDrawer 地图可视化绘制器
 * @param pAtlas 多地图Atlas管理器
 * @param pKFDB 关键帧数据库，用于回环检测、重定位
 * @param settings 配置参数管理器，读取yaml配置
 * @param _nameSeq 当前数据集序列名称
 */
Tracking::Tracking(const std::shared_ptr<System>& pSys,
                   const std::shared_ptr<ORBVocabulary>& pVoc,
                   const std::shared_ptr<FrameDrawer>& pFrameDrawer,
                   const std::shared_ptr<MapDrawer>& pMapDrawer,
                   const std::shared_ptr<Atlas>& pAtlas,
                   const std::shared_ptr<KeyFrameDatabase>& pKFDB,
                   const std::shared_ptr<Settings>& settings,
                   const string& _nameSeq)
    : std::enable_shared_from_this<Tracking>(), // 允许this生成shared_from_this共享指针
      mState(NO_IMAGES_YET),                    // 跟踪状态：还未收到任何图像
      mSensor(settings->sensor_),               // 传感器类型：单目/双目/RGBD/带IMU各类传感器
      mTrackedFr(0),                            // 已经成功跟踪的帧计数
      mbStep(false),                            // 单步调试模式标志，一步执行一帧
      mbOnlyTracking(false),                    // true=仅定位模式，关闭局部建图；false=SLAM正常建图+跟踪
      mbMapUpdated(false),                      // 标记地图是否被局部建图线程更新过
      mbVO(false),                              // 视觉里程计标志：true代表当前帧大多跟踪临时VO点，地图点匹配少
      mpORBVocabulary(pVoc),                    // ORB词袋模型指针
      mpKeyFrameDB(pKFDB),                      // 关键帧数据库指针
      mbReadyToInitializate(false),             // 单目初始化标记：是否已经保存参考初始化帧
      mpSystem(pSys),                           // System顶层系统指针
      mpViewer(nullptr),                        // 可视化Viewer指针，初始为空
      bStepByStep(false),                       // 是否开启单步调试模式
      mpFrameDrawer(pFrameDrawer),              // 帧绘制器，绘制当前帧特征点跟踪情况
      mpMapDrawer(pMapDrawer),                  // 地图绘制器，绘制三维地图、相机位姿
      mpAtlas(pAtlas),                          // Atlas多地图管理器
      mnLastRelocFrameId(0),                    // 上一次重定位成功的帧ID
      time_recently_lost(5.0),                  // RECENTLY_LOST状态超时时间，超过该时间直接置LOST
      mnInitialFrameId(0),                      // 地图初始化帧ID
      mbCreatedMap(false),                      // 是否刚刚新建地图标记
      mnFirstFrameId(0),                        // 当前地图第一帧ID
      mpCamera2(nullptr),                       // 第二个相机指针，双目/鱼眼双目使用
      mpLastKeyFrame(),                         // 上一个关键帧共享指针
      mThDepth(1.0),                            // 深度阈值，区分远近点，用于关键帧创建
      initID(0),                                // 初始化成功时关键帧ID
      lastID(0),                                // 上一帧Frame的ID
      mbInitWith3KFs(false),                    // IMU初始化是否使用3个关键帧标记
      mnNumDataset(0)                           // 数据集序号，支持多数据集连续运行
{
    // 从Settings配置类加载全部参数，相机、ORB、IMU参数全部在这里解析
    newParameterLoader(settings);

    {
        // 获取Atlas中全部注册的相机，打印相机类型调试日志
        vector<std::shared_ptr<GeometricCamera>> vpCams = mpAtlas->GetAllCameras();
        oslog::debug("There are {} cameras in the atlas", vpCams.size());

        // 遍历所有相机，打印ID和相机模型类型：针孔/鱼眼/未知
        for (auto const& pCam : vpCams) {
            if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
                oslog::debug("Camera {} is pinhole", pCam->GetId());
            } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
                oslog::debug("Camera {} is fisheye", pCam->GetId());
            } else {
                oslog::debug("Camera {} is unknown", pCam->GetId());
            }
        }
    }

#ifdef REGISTER_TIMES
    // 如果开启计时统计宏，清空所有耗时统计vector容器，用于记录各模块ms耗时
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif
}

#ifdef REGISTER_TIMES
/**
 * @brief 计算double类型vector容器的算术平均值
 * @param v_times 存储耗时的double数组
 * @return 平均值double
 */
double calcAverage(vector<double> v_times) {
    double accum = 0;
    for (double value : v_times) {
        accum += value;
    }

    return accum / v_times.size();
}

/**
 * @brief 计算double数组标准差
 * @param v_times 原始数据数组
 * @param average 已经计算好的平均值
 * @return 标准差
 */
double calcDeviation(vector<double> v_times, double average) {
    double accum = 0;
    for (double value : v_times) {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

/**
 * @brief int数组求平均值，跳过值为0的无效统计项
 * @param v_values int数组
 * @return 有效元素平均值
 */
double calcAverage(vector<int> v_values) {
    double accum = 0;
    int total = 0;
    for (double value : v_values) {
        if (value == 0) continue;
        accum += value;
        total++;
    }

    return accum / total;
}

/**
 * @brief int数组求标准差，跳过0无效项
 * @param v_values int数组
 * @param average 平均值
 * @return 标准差
 */
double calcDeviation(vector<int> v_values, double average) {
    double accum = 0;
    int total = 0;
    for (double value : v_values) {
        if (value == 0) continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

/**
 * @brief 将局部建图LocalMapper各项时间统计输出写入本地文本文件
 */
void Tracking::LocalMapStats2File() {
    ofstream f;

    try {
        f.open("LocalMapTimeStats.txt");
        f << fixed << setprecision(6);
        // 写入csv格式表头：立体校正、地图点剔除、地图点生成、局部BA、关键帧剔除、总耗时
        f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF "
             "culling[ms], Total[ms]"
          << endl;
        // 循环把LocalMapper内部各个时间统计数组写入csv每一行
        for (size_t i = 0; i < mpLocalMapper->vdLMTotal_ms.size(); ++i) {
            f << mpLocalMapper->vdKFInsert_ms.at(i) << ","
              << mpLocalMapper->vdMPCulling_ms.at(i) << ","
              << mpLocalMapper->vdMPCreation_ms.at(i) << ","
              << mpLocalMapper->vdLBASync_ms.at(i) << ","
              << mpLocalMapper->vdKFCullingSync_ms.at(i) << ","
              << mpLocalMapper->vdLMTotal_ms.at(i) << endl;

            f.close();
        }
    } catch (const std::out_of_range& ex) {
        // 捕获数组越界异常，打印日志
        oslog::error("[Tracking::LocalMapStats2File] Caught exception: {}",
                     ex.what());
        f.close();
    }

    try {
        f.open("LBA_Stats.txt");
        f << fixed << setprecision(6);
        // LBA统计表头：LBA耗时、优化关键帧数量、固定关键帧数量、地图点数量、图边数量
        f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
        for (size_t i = 0; i < mpLocalMapper->vdLBASync_ms.size(); ++i) {
            f << mpLocalMapper->vdLBASync_ms.at(i) << ","
              << mpLocalMapper->vnLBA_KFopt.at(i) << ","
              << mpLocalMapper->vnLBA_KFfixed.at(i) << ","
              << mpLocalMapper->vnLBA_MPs.at(i) << ","
              << mpLocalMapper->vnLBA_edges.at(i) << endl;
        }
        f.close();
    } catch (const std::out_of_range& ex) {
        oslog::error("[Tracking::LocalMapStats2File] Caught exception: {}",
                     ex.what());
        f.close();
    }
}

/**
 * @brief 将Tracking跟踪模块耗时统计写入磁盘文件
 */
void Tracking::TrackStats2File() {
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    // 会话信息：总关键帧数量、地图点数量、OpenCV版本
    f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
    f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    // csv表头：图像校正、图像缩放、ORB提取、立体匹配、IMU预积分、位姿预测、局部地图跟踪、新关键帧决策、总跟踪耗时
    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU "
         "preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]"
      << endl;

    // 遍历每一帧耗时，兼容部分传感器不存在的模块，缺失模块填0
    for (size_t i = 0; i < vdTrackTotal_ms.size(); ++i) {
        double stereo_rect = 0.0;
        if (!vdRectStereo_ms.empty()) {
            stereo_rect = vdRectStereo_ms[i];
        }

        double resize_image = 0.0;
        if (!vdResizeImage_ms.empty()) {
            resize_image = vdResizeImage_ms[i];
        }

        double stereo_match = 0.0;
        if (!vdStereoMatch_ms.empty()) {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if (!vdIMUInteg_ms.empty()) {
            imu_preint = vdIMUInteg_ms[i];
        }

        try {
            f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms.at(i)
              << "," << stereo_match << "," << imu_preint << ","
              << vdPosePred_ms.at(i) << "," << vdLMTrack_ms.at(i) << ","
              << vdNewKF_ms.at(i) << "," << vdTrackTotal_ms.at(i) << endl;
        } catch (const std::out_of_range& ex) {
            oslog::error("[Tracking::TrackStats2File] Caught exception: {}",
                         ex.what());
        }
    }

    f.close();
}

/**
 * @brief 打印全部时间统计，控制台输出同时写入ExecMean.txt文件；输出各模块均值±标准差
 */
void Tracking::PrintTimeStats() {
    // 先调用函数把统计数据落地到磁盘
    TrackStats2File();
    LocalMapStats2File();

    ofstream f;
    f.open("ExecMean.txt");
    f << fixed;
    // 控制台与文件同时输出时间统计标题
    std::cout << std::endl << " TIME STATS in ms (mean +/- std)" << std::endl;
    f << " TIME STATS in ms (mean +/- std)" << std::endl;
    cout << "OpenCV version: " << CV_VERSION << endl;
    f << "OpenCV version: " << CV_VERSION << endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;

    // 双目校正耗时统计
    if (!vdRectStereo_ms.empty()) {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        std::cout << "Stereo Rectification: " << average << " +/- " << deviation
                  << std::endl;
        f << "Stereo Rectification: " << average << " +/- " << deviation
          << std::endl;
    }

    // 图像缩放耗时
    if (!vdResizeImage_ms.empty()) {
        average = calcAverage(vdResizeImage_ms);
        deviation = calcDeviation(vdResizeImage_ms, average);
        std::cout << "Image Resize: " << average << " +/- " << deviation
                  << std::endl;
        f << "Image Resize: " << average << " +/- " << deviation << std::endl;
    }

    // ORB特征提取耗时
    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    std::cout << "ORB Extraction: " << average << " +/- " << deviation
              << std::endl;
    f << "ORB Extraction: " << average << " +/- " << deviation << std::endl;

    // 双目匹配耗时
    if (!vdStereoMatch_ms.empty()) {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        std::cout << "Stereo Matching: " << average << " +/- " << deviation
                  << std::endl;
        f << "Stereo Matching: " << average << " +/- " << deviation << std::endl;
    }

    // IMU预积分耗时
    if (!vdIMUInteg_ms.empty()) {
        average = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        std::cout << "IMU Preintegration: " << average << " +/- " << deviation
                  << std::endl;
        f << "IMU Preintegration: " << average << " +/- " << deviation << std::endl;
    }

    // 位姿预测耗时
    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    std::cout << "Pose Prediction: " << average << " +/- " << deviation
              << std::endl;
    f << "Pose Prediction: " << average << " +/- " << deviation << std::endl;

    // LM局部地图跟踪耗时
    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    std::cout << "LM Track: " << average << " +/- " << deviation << std::endl;
    f << "LM Track: " << average << " +/- " << deviation << std::endl;

    // 新关键帧决策耗时
    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    std::cout << "New KF decision: " << average << " +/- " << deviation
              << std::endl;
    f << "New KF decision: " << average << " +/- " << deviation << std::endl;

    // Tracking线程总耗时
    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    std::cout << "Total Tracking: " << average << " +/- " << deviation
              << std::endl;
    f << "Total Tracking: " << average << " +/- " << deviation << std::endl;

    // ------------------------ LocalMapper统计部分 ------------------------
    std::cout << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    // 关键帧插入耗时
    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    std::cout << "KF Insertion: " << average << " +/- " << deviation << std::endl;
    f << "KF Insertion: " << average << " +/- " << deviation << std::endl;

    // 地图点剔除耗时
    average = calcAverage(mpLocalMapper->vdMPCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
    std::cout << "MP Culling: " << average << " +/- " << deviation << std::endl;
    f << "MP Culling: " << average << " +/- " << deviation << std::endl;

    // 地图点生成耗时
    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    std::cout << "MP Creation: " << average << " +/- " << deviation << std::endl;
    f << "MP Creation: " << average << " +/- " << deviation << std::endl;

    // LBA局部BA耗时
    average = calcAverage(mpLocalMapper->vdLBA_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
    std::cout << "LBA: " << average << " +/- " << deviation << std::endl;
    f << "LBA: " << average << " +/- " << deviation << std::endl;

    // 关键帧剔除耗时
    average = calcAverage(mpLocalMapper->vdKFCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
    std::cout << "KF Culling: " << average << " +/- " << deviation << std::endl;
    f << "KF Culling: " << average << " +/- " << deviation << std::endl;

    // LocalMapper总耗时
    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    std::cout << "Total Local Mapping: " << average << " +/- " << deviation
              << std::endl;
    f << "Total Local Mapping: " << average << " +/- " << deviation << std::endl;

    // LBA优化复杂度统计
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean +/- std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean +/- std)" << std::endl;

    // LBA图边数量
    average = calcAverage(mpLocalMapper->vnLBA_edges);
    deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
    std::cout << "LBA Edges: " << average << " +/- " << deviation << std::endl;
    f << "LBA Edges: " << average << " +/- " << deviation << std::endl;

    // LBA参与优化的关键帧数目
    average = calcAverage(mpLocalMapper->vnLBA_KFopt);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
    std::cout << "LBA KF optimized: " << average << " +/- " << deviation
              << std::endl;
    f << "LBA KF optimized: " << average << " +/- " << deviation << std::endl;

    // LBA固定不动的关键帧数目
    average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
    std::cout << "LBA KF fixed: " << average << " +/- " << deviation << std::endl;
    f << "LBA KF fixed: " << average << " +/- " << deviation << std::endl;

    // LBA地图点数量
    average = calcAverage(mpLocalMapper->vnLBA_MPs);
    deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
    std::cout << "LBA MP: " << average << " +/- " << deviation << std::endl
              << std::endl;
    f << "LBA MP: " << average << " +/- " << deviation << std::endl << std::endl;

    // LBA执行总次数、提前终止次数
    std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

    // 地图整体信息
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::cout << "Num maps  : " << mpAtlas->CountMaps() << std::endl;
    std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
    std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();
    // 找出地图集合中关键帧最多的那张地图作为主地图
    std::shared_ptr<Map> pBestMap = vpMaps[0];
    for (int i = 1; i < vpMaps.size(); ++i) {
        if (pBestMap->GetAllKeyFrames().size() <
            vpMaps[i]->GetAllKeyFrames().size()) {
            pBestMap = vpMaps[i];
        }
    }

    f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

    // ---------------- 回环检测/重定位 Place Recognition 统计 ----------------
    f << "---------------------------" << std::endl;
    f << std::endl << "## Place Recognition (mean +/- std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "## Place Recognition (mean +/- std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdDataQuery_ms);
    deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
    f << "Database Query: " << average << " +/- " << deviation << std::endl;
    std::cout << "Database Query: " << average << " +/- " << deviation
              << std::endl;
    average = calcAverage(mpLoopClosing->vdEstSim3_ms);
    deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
    f << "SE3 estimation: " << average << " +/- " << deviation << std::endl;
    std::cout << "SE3 estimation: " << average << " +/- " << deviation
              << std::endl;
    average = calcAverage(mpLoopClosing->vdPRTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
    f << "Total Place Recognition: " << average << " +/- " << deviation
      << std::endl
      << std::endl;
    std::cout << "Total Place Recognition: " << average << " +/- " << deviation
              << std::endl
              << std::endl;

    // ---------------- Loop Closing回环融合统计 ----------------
    f << std::endl << "## Loop Closing (mean +/- std)" << std::endl;
    std::cout << std::endl << "## Loop Closing (mean +/- std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
    f << "Loop Fusion: " << average << " +/- " << deviation << std::endl;
    std::cout << "Loop Fusion: " << average << " +/- " << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
    f << "Essential Graph: " << average << " +/- " << deviation << std::endl;
    std::cout << "Essential Graph: " << average << " +/- " << deviation
              << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
    f << "Total Loop Closing: " << average << " +/- " << deviation << std::endl
      << std::endl;
    std::cout << "Total Loop Closing: " << average << " +/- " << deviation
              << std::endl
              << std::endl;

    f << "Num exec: " << mpLoopClosing->nLoop << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
    average = calcAverage(mpLoopClosing->vnLoopKFs);
    deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
    f << "Number of KFs: " << average << " +/- " << deviation << std::endl;
    std::cout << "Number of KFs: " << average << " +/- " << deviation
              << std::endl;

    // ---------------- Map Merging地图融合统计（多地图Atlas模式） ----------------
    f << std::endl << "## Map Merging (mean +/- std)" << std::endl;
    std::cout << std::endl << "## Map Merging (mean +/- std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
    f << "Merge Maps: " << average << " +/- " << deviation << std::endl;
    std::cout << "Merge Maps: " << average << " +/- " << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
    f << "Welding BA: " << average << " +/- " << deviation << std::endl;
    std::cout << "Welding BA: " << average << " +/- " << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
    f << "Optimization Ess.: " << average << " +/- " << deviation << std::endl;
    std::cout << "Optimization Ess.: " << average << " +/- " << deviation
              << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
    f << "Total Map Merging: " << average << " +/- " << deviation << std::endl
      << std::endl;
    std::cout << "Total Map Merging: " << average << " +/- " << deviation
              << std::endl
              << std::endl;

    f << "Num exec: " << mpLoopClosing->nMerges << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeKFs);
    deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
    f << "Number of KFs: " << average << " +/- " << deviation << std::endl;
    std::cout << "Number of KFs: " << average << " +/- " << deviation
              << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeMPs);
    deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
    f << "Number of MPs: " << average << " +/- " << deviation << std::endl;
    std::cout << "Number of MPs: " << average << " +/- " << deviation
              << std::endl;

    // ---------------- Full GBA 全局BA统计 ----------------
    f << std::endl << "## Full GBA (mean +/- std)" << std::endl;
    std::cout << std::endl << "## Full GBA (mean +/- std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdGBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
    f << "GBA: " << average << " +/- " << deviation << std::endl;
    std::cout << "GBA: " << average << " +/- " << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
    deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
    f << "Map Update: " << average << " +/- " << deviation << std::endl;
    std::cout << "Map Update: " << average << " +/- " << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
    f << "Total Full GBA: " << average << " +/- " << deviation << std::endl
      << std::endl;
    std::cout << "Total Full GBA: " << average << " +/- " << deviation
              << std::endl
              << std::endl;

    f << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    f << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAKFs);
    deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
    f << "Number of KFs: " << average << " +/- " << deviation << std::endl;
    std::cout << "Number of KFs: " << average << " +/- " << deviation
              << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAMPs);
    deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
    f << "Number of MPs: " << average << " +/- " << deviation << std::endl;
    std::cout << "Number of MPs: " << average << " +/- " << deviation
              << std::endl;

    f.close();
}

#endif

/**
 * @brief Tracking析构函数
 */
Tracking::~Tracking() {
    // f_track_stats.close();
}

/**
 * @brief 从Settings配置对象加载Tracking全部参数：相机、畸变、ORB提取器、IMU标定参数
 * @param settings Settings配置智能指针，读取yaml配置
 */
void Tracking::newParameterLoader(const std::shared_ptr<Settings>& settings) {
    // 获取主相机，注册到Atlas多相机管理
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    // 判断是否需要去畸变，读取畸变系数；不需要畸变则填充0矩阵
    if (settings->needToUndistort()) {
        mDistCoef = settings->camera1DistortionCoef();

        // std::cout << "[Tracking::newParameterLoader] " << mDistCoef << endl;
    } else {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    }

    // TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    // OpenCV cv::Mat形式内参矩阵K
    mK = cv::Mat::eye(3, 3, CV_32F);
    mK.at<float>(0, 0) = mpCamera->getParameter(0);
    mK.at<float>(1, 1) = mpCamera->getParameter(1);
    mK.at<float>(0, 2) = mpCamera->getParameter(2);
    mK.at<float>(1, 2) = mpCamera->getParameter(3);

    // Eigen::Matrix3f形式内参矩阵K_
    mK_.setIdentity();
    mK_(0, 0) = mpCamera->getParameter(0);
    mK_(1, 1) = mpCamera->getParameter(1);
    mK_(0, 2) = mpCamera->getParameter(2);
    mK_(1, 2) = mpCamera->getParameter(3);

    // 双目鱼眼相机：加载第二个相机，加载外参Tlr，开启双绘制窗口
    if ((mSensor == SensorType::STEREO || mSensor == SensorType::IMU_STEREO ||
         mSensor == SensorType::IMU_RGBD) &&
        settings->cameraType() == Settings::KannalaBrandt) {
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();

        mpFrameDrawer->both = true;
    }

    // 双目/RGBD传感器：读取bf基线*焦距，计算深度阈值mThDepth
    if (mSensor.isStereo() || mSensor.isRGBD()) {
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    // RGBD传感器，读取深度图缩放因子
    if (mSensor.isRGBD()) {
        mDepthMapFactor = settings->depthMapFactor();
        if (fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }

    mMinFrames = 0;
    mbRGB = settings->rgb();

    {
        // 读取ORB特征提取器配置参数
        const int nFeatures = settings->nFeatures();
        const int nLevels = settings->nLevels();
        const int fIniThFAST = settings->initThFAST();
        const int fMinThFAST = settings->minThFAST();
        const float fScaleFactor = settings->scaleFactor();

        // 左目ORB提取器
        mpORBextractorLeft = std::make_shared<ORBextractor>(
            nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // 双目模式创建右目ORB提取器
        if (mSensor.isStereo())
            mpORBextractorRight = std::make_shared<ORBextractor>(
                nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // 单目初始化阶段使用5倍特征点数量，提高初始化成功率
        if (mSensor.isMonocular())
            mpIniORBextractor = std::make_shared<ORBextractor>(
                5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }

    // IMU参数加载：Tbc相机到IMU外参，噪声、游走噪声，IMU频率
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuPer = 0.001;  // 1.0 / (double) mImuFreq;

    //TODO: ESTO ESTA BIEN?
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    // 根据IMU频率换算预积分使用的噪声系数
    const float sf = sqrt(mImuFreq);
    mImuCalib.Set(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    // 创建从最后一个关键帧开始的IMU预积分对象，初始bias为0
    mpImuPreintegratedFromLastKF =
        std::make_shared<IMU::Preintegrated>(IMU::Bias(), mImuCalib);
}

/**
 * @brief 双目传感器图像输入接口，接收左右校正后双目图像，时间戳，文件名；构建Frame，调用Track()执行跟踪，返回相机位姿Tcw
 * @param imRectLeft 校正后左目图像
 * @param imRectRight 校正后右目图像
 * @param timestamp 当前帧时间戳
 * @param filename 当前图像文件名，用于日志调试
 * @return Sophus::SE3f Tcw 世界到相机的位姿
 */
Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat& imRectLeft,
                                       const cv::Mat& imRectRight,
                                       const double& timestamp,
                                       string filename) {
    // 记录函数开始时间戳
    std::chrono::steady_clock::time_point t_start =
        std::chrono::steady_clock::now();

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    mImRight = imRectRight;

    // 如果输入是彩色图(RGB/BGR/RGBA/BGRA)，转灰度图；区分RGB开关
    if (mImGray.channels() == 3) {
        // cout << "Image with 3 channels" << endl;
        if (mbRGB) {
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
        } else {
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
        }
    } else if (mImGray.channels() == 4) {
        // cout << "Image with 4 channels" << endl;
        if (mbRGB) {
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
        } else {
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
        }
    }

    std::chrono::steady_clock::time_point t_post_image_conversion =
        std::chrono::steady_clock::now();

    // cout << "Incoming frame creation" << endl;

    // 根据传感器类型、是否存在第二相机，调用不同Frame构造函数
    if (mSensor == SensorType::STEREO && !mpCamera2)
        mCurrentFrame = std::make_shared<Frame>(
            mImGray, imGrayRight, timestamp, mpORBextractorLeft,
            mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth,
            mpCamera);
    else if (mSensor == SensorType::STEREO && mpCamera2)
        mCurrentFrame = std::make_shared<Frame>(
            mImGray, imGrayRight, timestamp, mpORBextractorLeft,
            mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth,
            mpCamera, mpCamera2, mTlr);
    else if (mSensor == SensorType::IMU_STEREO && !mpCamera2)
        mCurrentFrame = std::make_shared<Frame>(
            mImGray, imGrayRight, timestamp, mpORBextractorLeft,
            mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth,
            mpCamera, mLastFrame, mImuCalib);
    else if (mSensor == SensorType::IMU_STEREO && mpCamera2)
        mCurrentFrame = std::make_shared<Frame>(
            mImGray, imGrayRight, timestamp, mpORBextractorLeft,
            mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth,
            mpCamera, mpCamera2, mTlr, mLastFrame, mImuCalib);

    std::chrono::steady_clock::time_point t_post_frame_creation =
        std::chrono::steady_clock::now();

    // 保存文件名、数据集编号到Frame对象
    mCurrentFrame->mNameFile = filename;
    mCurrentFrame->mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    // 记录ORB提取耗时、双目匹配耗时，从Frame内部统计变量取出
    vdORBExtract_ms.push_back(mCurrentFrame->mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame->mTimeStereoMatch);
#endif

    // 核心跟踪主函数，执行整个跟踪逻辑
    Track();

    std::chrono::steady_clock::time_point t_post_track =
        std::chrono::steady_clock::now();

    // spdlog打印各阶段耗时
    oslog::info(
        "[Tracking::GrabImageStereo] Elapsed time (ms): image_conversion {:.4}, "
        "frame_creation {:.4}, track {:.4}",
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t_post_image_conversion - t_start)
            .count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t_post_frame_creation - t_post_image_conversion)
            .count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t_post_track - t_post_frame_creation)
            .count());

    // 返回当前帧估计的相机位姿Tcw
    return mCurrentFrame->GetPose();
}

/**
 * @brief RGBD传感器输入接口，接收RGB彩色图、深度图、时间戳、文件名；构建Frame调用Track跟踪，返回Tcw位姿
 * @param imRGB RGB彩色图像
 * @param imD 深度图
 * @param timestamp 时间戳
 * @param filename 图像文件名
 * @return Sophus::SE3f Tcw世界到相机位姿
 */
Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat& imRGB, const cv::Mat& imD,
                                     const double& timestamp, string filename) {
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    // 彩色图转灰度图，区分RGB/BGR/RGBA/BGRA
    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    } else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    // 深度图缩放转换为CV_32F浮点深度
    if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
        imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

    // RGBD普通模式 / IMU‑RGBD模式分别构造Frame对象
    if (mSensor == SensorType::RGBD)
        mCurrentFrame = std::make_shared<Frame>(
            mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK,
            mDistCoef, mbf, mThDepth, mpCamera);
    else if (mSensor == SensorType::IMU_RGBD)
        mCurrentFrame = std::make_shared<Frame>(
            mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK,
            mDistCoef, mbf, mThDepth, mpCamera, mLastFrame, mImuCalib);

    mCurrentFrame->mNameFile = filename;
    mCurrentFrame->mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame->mTimeORB_Ext);
#endif

    // 执行跟踪主逻辑
    Track();

    return mCurrentFrame->GetPose();
}

/**
 * @brief 单目传感器图像输入接口，接收单张图像，时间戳，文件名；构造Frame，调用Track，返回Tcw位姿
 * @param im 输入图像
 * @param timestamp 时间戳
 * @param filename 图像文件名
 * @return Sophus::SE3f Tcw世界到相机位姿
 */
Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat& im,
                                           const double& timestamp,
                                           string filename) {
    mImGray = im;
    // 彩色图转灰度
    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    } else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    // 单目模式：未初始化时使用高特征数量mpIniORBextractor；初始化完成使用普通提取器mpORBextractorLeft
    if (mSensor == SensorType::MONOCULAR) {
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET ||
            (lastID - initID) < mMaxFrames) {
            mCurrentFrame = std::make_shared<Frame>(
                mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera,
                mDistCoef, mbf, mThDepth);
        } else {
            mCurrentFrame = std::make_shared<Frame>(
                mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera,
                mDistCoef, mbf, mThDepth);
        }
    } else if (mSensor == SensorType::IMU_MONOCULAR) {
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET) {
            mCurrentFrame = std::make_shared<Frame>(
                mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera,
                mDistCoef, mbf, mThDepth, mLastFrame, mImuCalib);
        } else {
            mCurrentFrame = std::make_shared<Frame>(
                mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera,
                mDistCoef, mbf, mThDepth, mLastFrame, mImuCalib);
        }
    }

    // 第一帧记录t0时间戳
    if (mState == NO_IMAGES_YET) t0 = timestamp;

    mCurrentFrame->mNameFile = filename;
    mCurrentFrame->mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame->mTimeORB_Ext);
#endif

    lastID = mCurrentFrame->mnId;
    // 执行跟踪主逻辑
    Track();

    return mCurrentFrame->GetPose();
}

/**
 * @brief IMU数据接收接口，外部调用把IMU测量点存入队列mlQueueImuData，线程安全加锁
 * @param imuMeasurement IMU加速度、角速度、时间戳测量值
 */
void Tracking::GrabImuData(const IMU::Point& imuMeasurement) {
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

/**
 * @brief IMU预积分主函数：从IMU队列取出两帧图像之间的IMU数据，做时间插值，执行预积分计算
 */
void Tracking::PreintegrateIMU() {   // IMU预积分入口：从前一帧到当前帧提取IMU队列数据，做预积分
    if (!mCurrentFrame->mpPrevFrame) {     // 当前帧没有上一帧(第一帧)，无需预积分
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame->setIntegrated();     // 标记该帧IMU预积分已完成
        return;
    }

    mvImuFromLastFrame.clear();   // 清空存储[上一帧~当前帧]之间IMU测量值的容器
    mvImuFromLastFrame.reserve(mlQueueImuData.size());   // 预分配内存，避免频繁realloc
    if (mlQueueImuData.size() == 0) {   // IMU消息队列为空，没有IMU数据可积分
        Verbose::PrintMess("No IMU data in mlQueueImuData!!",
                           Verbose::VERBOSITY_NORMAL);
        mCurrentFrame->setIntegrated();
        return;
    }

    while (true) {     // 循环从全局IMU队列mlQueueImuData截取两帧图像时间戳之间的IMU样本
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);   // 加互斥锁保护IMU队列，多线程读写安全
            if (!mlQueueImuData.empty()) {   // IMU队列不为空
                IMU::Point* m = &mlQueueImuData.front();  // 获取队列头部IMU数据

                if (m->t < mCurrentFrame->mpPrevFrame->mTimeStamp - mImuPer) {   // IMU时间早于上一帧时间戳，属于过期数据直接丢弃
                    mlQueueImuData.pop_front();
                } else if (m->t < mCurrentFrame->mTimeStamp - mImuPer) {   // IMU时间介于上一帧~当前帧之间，存入本帧IMU列表并从全局队列弹出
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                } else {   // IMU时间>=当前帧时间戳，属于下一帧数据，停止截取，保留在全局队列
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            } else {   // IMU队列为空，跳出循环，标记sleep
                break;
                bSleep = true;
            }
        }
        if (bSleep) usleep(500);   // 队列空，休眠500微秒让出CPU
    }

    const int n = mvImuFromLastFrame.size() - 1;   // IMU样本数量减1，用于循环遍历
    if (n == 0) {     // 有效IMU测量点不足，无法做预积分
        oslog::warn("Empty IMU measurements vector!!!");
        return;
    }

    // 创建预积分对象：bias来自上一关键帧，IMU标定参数来自当前帧
    std::shared_ptr<IMU::Preintegrated> pImuPreintegratedFromLastFrame =
        std::make_shared<IMU::Preintegrated>(mLastFrame->mImuBias,
                                             mCurrentFrame->mImuCalib);

    // 遍历IMU样本，执行预积分，对首尾IMU做时间插值对齐图像时间戳
    for (int i = 0; i < n; i++) {
        float tstep;                 // 积分时间步长
        Eigen::Vector3f acc, angVel; // 插值后的加速度、角速度

        // 第一个IMU点：需要插值对齐【上一图像帧时间戳】
        if ((i == 0) && (i < (n - 1))) {
            float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t; // 当前IMU与下一个IMU时间差
            float tini =
                mvImuFromLastFrame[i].t - mCurrentFrame->mpPrevFrame->mTimeStamp; // IMU点距离上一图像帧的时间偏移
            // 线性插值，得到图像帧时刻的加速度
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                   (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) *
                       (tini / tab)) *
                  0.5f;
            // 线性插值，得到图像帧时刻的角速度
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                      (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) *
                          (tini / tab)) *
                     0.5f;
            // 积分时间：上一图像帧时间到下一个IMU采样时刻
            tstep =
                mvImuFromLastFrame[i + 1].t - mCurrentFrame->mpPrevFrame->mTimeStamp;
        } else if (i < (n - 1)) {   // 中间普通IMU样本：前后两个IMU取平均，时间步长为IMU采样间隔
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
            tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
        } else if ((i > 0) && (i == (n - 1))) {   // 最后一组IMU：插值对齐【当前图像帧时间戳】
            float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t; // 当前IMU与下一个IMU时间差
            float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame->mTimeStamp; // IMU点距离当前图像帧时间偏移
            // 插值得到当前图像帧时刻加速度
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                   (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) *
                       (tend / tab)) *
                  0.5f;
            // 插值得到当前图像帧时刻角速度
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                      (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) *
                          (tend / tab)) *
                     0.5f;
            // 积分时间：当前IMU采样时刻到当前图像帧时间戳
            tstep = mCurrentFrame->mTimeStamp - mvImuFromLastFrame[i].t;
        } else if ((i == 0) && (i == (n - 1))) {   // 边界情况：只有1个IMU样本，直接使用该样本原始值，时间步长等于两图像帧时间差
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep =
                mCurrentFrame->mTimeStamp - mCurrentFrame->mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            oslog::warn("mpImuPreintegratedFromLastKF does not exist");
        // 1. 从上个关键帧开始的预积分对象，累积IMU测量
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
        // 2. 从上一普通帧开始的预积分对象，累积IMU测量
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
    }

    // 将生成的预积分对象挂载到当前帧
    mCurrentFrame->mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame->mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame->mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame->setIntegrated();   // 标记当前帧IMU预积分完成
}

// 使用IMU预积分结果预测当前帧的位姿、速度，作为视觉位姿求解初值
bool Tracking::PredictStateIMU() {
    if (!mCurrentFrame->mpPrevFrame) {   // 当前帧不存在上一帧，无法IMU预测
        oslog::debug("[Tracking::PredictStateIMU] No last frame");
        return false;
    }

    if (mbMapUpdated && mpLastKeyFrame) {   // 地图被局部BA更新过，使用【上一关键帧】+关键帧间预积分做预测
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();  // 上关键帧IMU系位置
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();  // 上关键帧IMU系旋转
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();     // 上关键帧IMU系速度

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);  // 重力向量(IMU坐标系下)
        const float t12 = mpImuPreintegratedFromLastKF->dT;   // 预积分总时间

        // 预测当前IMU系旋转：上关键帧旋转 * 预积分delta旋转，做旋转归一化
        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(
                       mpLastKeyFrame->GetImuBias()));
        // 预测IMU系位置：传播公式 p2 = p1 + v1*dt + 0.5*g*dt² + R1*delta_p
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(
                       mpLastKeyFrame->GetImuBias());
        // 预测IMU系速度：v2 = v1 + g*dt + R1*delta_v
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(
                       mpLastKeyFrame->GetImuBias());
        // 将预测得到的IMU位姿速度设置到当前帧
        mCurrentFrame->SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame->mImuBias = mpLastKeyFrame->GetImuBias();   // 复制上关键帧bias
        mCurrentFrame->mPredBias = mCurrentFrame->mImuBias;       // 保存预测bias
        return true;
    } else if (!mbMapUpdated) {   // 地图没有被BA修改，使用【上一普通帧】+帧间预积分做预测
        const Eigen::Vector3f twb1 = mLastFrame->GetImuPosition();  // 上一帧IMU位置
        const Eigen::Matrix3f Rwb1 = mLastFrame->GetImuRotation();  // 上一帧IMU旋转
        const Eigen::Vector3f Vwb1 = mLastFrame->GetVelocity();     // 上一帧IMU速度
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);        // 重力向量
        const float t12 = mCurrentFrame->mpImuPreintegratedFrame->dT; // 帧间预积分总时间

        // IMU旋转传播
        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
            Rwb1 * mCurrentFrame->mpImuPreintegratedFrame->GetDeltaRotation(
                       mLastFrame->mImuBias));
        // IMU位置传播
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mCurrentFrame->mpImuPreintegratedFrame->GetDeltaPosition(
                       mLastFrame->mImuBias);
        // IMU速度传播
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * mCurrentFrame->mpImuPreintegratedFrame->GetDeltaVelocity(
                       mLastFrame->mImuBias);

        mCurrentFrame->SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame->mImuBias = mLastFrame->mImuBias;
        mCurrentFrame->mPredBias = mCurrentFrame->mImuBias;
        return true;
    } else {
        cout << "not IMU prediction!!" << endl;
    }

    return false;
}

// IMU帧重置，待实现
void Tracking::ResetFrameIMU() {
    // TODO To implement...
}

// Tracking主循环：每一帧图像进来执行此函数，完成IMU预积分、初始化、位姿跟踪、局部地图跟踪、关键帧生成
void Tracking::Track() {
    std::chrono::steady_clock::time_point t_start =
        std::chrono::steady_clock::now();   // 记录Track函数开始时刻

    mMaxFrames = mpSystem->fps();          // 最大帧间隔，取自系统帧率
    mnFramesToResetIMU = mMaxFrames;       // IMU重置保护帧数

    if (bStepByStep) {   // 单步调试模式，等待外部触发下一步
        oslog::trace("Tracking: Waiting to the next step");
        while (!mbStep && bStepByStep) usleep(500);
        mbStep = false;
    }

    if (mpLocalMapper->mbBadImu) {   // 局部映射器标记IMU异常，重置活跃地图
        oslog::info("TRACK: Reset map because local mapper set the bad imu flag ");
        mpSystem->ResetActiveMap();
        return;
    }

    auto pCurrentMap = mpAtlas->GetCurrentMap();   // 获取Atlas中当前正在使用的地图
    if (!pCurrentMap) {
        oslog::error("ERROR: There is not an active map in the atlas");
    }

    if (mState != NO_IMAGES_YET) {   // 系统已经接收过图像
        if (mLastFrame->mTimeStamp > mCurrentFrame->mTimeStamp) {   // 时间戳回退，异常，清空IMU队列，新建地图
            oslog::error(
                "ERROR: Frame with a timestamp older than previous frame detected!");
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        } else if (mCurrentFrame->mTimeStamp > mLastFrame->mTimeStamp + 1.0) {   // 前后帧时间戳跳变>1秒
            // cout << mCurrentFrame->mTimeStamp << ", " << mLastFrame->mTimeStamp <<
            // endl; cout << "id last: " << mLastFrame->mnId << "    id curr: " <<
            // mCurrentFrame->mnId << endl;
            if (mpAtlas->isInertial()) {   // 当前是IMU模式
                if (mpAtlas->isImuInitialized()) {   // IMU已经完成初始化
                    oslog::warn(
                        "Timestamp jump detected. State set to LOST. resetting IMU "
                        "integration...");

                    if (!pCurrentMap->GetInertialBA2()) {
                        mpSystem->ResetActiveMap();
                    } else {
                        CreateMapInAtlas();
                    }
                } else {   // IMU还未初始化，直接重置地图
                    oslog::warn(
                        "Timestamp jump detected, before IMU initialization. "
                        "resetting...");
                    mpSystem->ResetActiveMap();
                }
                return;
            }
        }
    }

    if (mSensor.isImu() && mpLastKeyFrame) {   // IMU传感器，设置当前帧bias来自上一个关键帧
        mCurrentFrame->SetNewBias(mpLastKeyFrame->GetImuBias());
    }

    if (mState == NO_IMAGES_YET) {   // 还没有处理过任何图像，切换状态为未初始化
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;   // 保存处理前系统状态

    if (mSensor.isImu() && !mbCreatedMap) {   // IMU模式，且不是刚新建地图，执行IMU预积分
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU =
            std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU =
            std::chrono::steady_clock::now();

        double timePreImu =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                time_EndPreIMU - time_StartPreIMU)
                .count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif
    }
    mbCreatedMap = false;   // 清除新建地图标记

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);   // 加地图更新锁，防止地图在跟踪时被修改

    mbMapUpdated = false;   // 地图是否被局部BA更新标记，初始false

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();   // 获取地图变更计数
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();        // 获取上一次跟踪时记录的地图变更计数
    if (nCurMapChangeIndex > nMapChangeIndex) {   // 计数变大，说明地图发生修改
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }

    if (mState == NOT_INITIALIZED) {   // 系统未初始化，执行地图初始化
        if (mSensor.isStereo() || mSensor.isRGBD()) {   // 双目/RGBD初始化
            StereoInitialization();
        } else {   // 单目初始化
            MonocularInitialization();
        }

        if (mpFrameDrawer) mpFrameDrawer->Update(shared_from_this());   // 更新可视化界面

        // If rightly initialized, mState=OK
        if (mState != OK) {   // 初始化失败，保存当前帧作为上一帧，直接返回
            mLastFrame = std::make_shared<Frame>(*mCurrentFrame);
            return;
        }

        if (mpAtlas->GetAllMaps().size() == 1) {
            mnFirstFrameId = mCurrentFrame->mnId;
        }
    } else {   // 系统已经初始化完成，执行正常跟踪流程
        // System is initialized. Track Frame.
        bool bOK = false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPosePred =
            std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization (if
        // tracking is lost)
        if (!mbOnlyTracking) {   // 正常SLAM模式：开启局部建图，不是纯定位模式
            std::chrono::steady_clock::time_point t_before_track =
                std::chrono::steady_clock::now();

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if (mState == OK) {   // 跟踪状态正常
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();   // 检查上一帧地图点是否被融合替换

                if ((!mbVelocity && !pCurrentMap->isImuInitialized()) ||
                    mCurrentFrame->mnId < mnLastRelocFrameId + 2) {   // 没有速度模型 / IMU未初始化 /刚重定位完，参考关键帧跟踪
                    oslog::info("TRACK: Track with respect to the reference KF");
                    bOK = TrackReferenceKeyFrame();
                } else {   // 使用运动模型跟踪
                    oslog::info("TRACK: Track with motion model");
                    bOK = TrackWithMotionModel();
                    if (!bOK) {   // 运动模型跟踪失败，回退到参考关键帧跟踪
                        oslog::info(
                            "TRACK:  ... motion model failed, falling back to reference "
                            "key frame");
                        bOK = TrackReferenceKeyFrame();
                    }
                }

                if (!bOK) {   // 跟踪仍然失败，标记丢失
                    oslog::info("TRACK:   Tracking still bad, I am lost!");
                    if (mCurrentFrame->mnId <=
                            (mnLastRelocFrameId + mnFramesToResetIMU) &&
                        mSensor.isImu()) {
                        mState = LOST;
                    } else if (pCurrentMap->KeyFramesInMap() > 10) {
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame->mTimeStamp;
                    } else {
                        mState = LOST;
                    }
                }
            } else {   // 状态不是OK，可能是RECENTLY_LOST / LOST
                if (mState == RECENTLY_LOST) {   // 短暂丢失状态，尝试恢复
                    oslog::info("Lost for a short time");

                    bOK = true;
                    if (mSensor.isImu()) {
                        if (pCurrentMap->isImuInitialized())
                            PredictStateIMU();   // IMU已初始化，用IMU预测位姿
                        else
                            bOK = false;

                        if (mCurrentFrame->mTimeStamp - mTimeStampLost >
                            time_recently_lost) {   // 超过短暂丢失时间阈值，彻底丢失
                            mState = LOST;
                            oslog::info("Track Lost...");
                            bOK = false;
                        }
                    } else {   // 纯视觉模式，执行重定位
                        bOK = Relocalization();
                        // std::cout << "mCurrentFrame->mTimeStamp:" <<
                        // to_string(mCurrentFrame->mTimeStamp) << std::endl; std::cout <<
                        // "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
                        if (mCurrentFrame->mTimeStamp - mTimeStampLost > 3.0f && !bOK) {
                            mState = LOST;
                            oslog::info("Track Lost...");
                            bOK = false;
                        }
                    }
                } else if (mState == LOST) {   // 彻底丢失，新建子地图
                    oslog::info("A new map is started...");

                    if (pCurrentMap->KeyFramesInMap() < 10) {
                        mpSystem->ResetActiveMap();
                        oslog::info("resetting current map...");
                    } else {
                        CreateMapInAtlas();
                    }

                    if (mpLastKeyFrame) mpLastKeyFrame.reset();

                    return;
                }
            }

            oslog::info(
                "[Tracking::Track] !mbOnlyTracking  {} ms ",
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                    std::chrono::steady_clock::now() - t_before_track)
                    .count());
        } else {   // mbOnlyTracking=true：纯定位模式，局部建图关闭
            std::chrono::steady_clock::time_point t_before_track =
                std::chrono::steady_clock::now();

            // Localization Mode: Local Mapping is deactivated (TODO Not available in
            // inertial mode)
            if (mState == LOST) {   // 丢失状态，执行重定位
                if (mSensor.isImu())
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            } else {   // 未丢失
                if (!mbVO) {   // 上一帧匹配到足够地图点，不是纯视觉里程计
                    oslog::info("[Track::Track] Tracked enough map points");
                    // In last frame we tracked enough MapPoints in the map
                    if (mbVelocity) {
                        bOK = TrackWithMotionModel();
                    } else {
                        bOK = TrackReferenceKeyFrame();
                    }

                } else {   // mbVO=true：上一帧大多是临时VO点，匹配地图点很少
                    oslog::info("[Track::Track] Tracked mostly VO points");

                    // In last frame we tracked mainly "visual odometry" points.
                    // We compute two camera poses, one from motion model and one doing
                    // relocalization. If relocalization is sucessfull we choose that
                    // solution, otherwise we retain the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    Sophus::SE3f TcwMM;
                    if (mbVelocity) {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame->mvpMapPoints;
                        vbOutMM = mCurrentFrame->mvbOutlier;
                        TcwMM = mCurrentFrame->GetPose();
                    }
                    bOKReloc = Relocalization();   // 同时跑重定位

                    if (bOKMM && !bOKReloc) {   // 运动模型成功，重定位失败，保留运动模型结果
                        mCurrentFrame->SetPose(TcwMM);
                        mCurrentFrame->mvpMapPoints = vpMPsMM;
                        mCurrentFrame->mvbOutlier = vbOutMM;

                        if (mbVO) {
                            for (int i = 0; i < mCurrentFrame->N; i++) {
                                if (mCurrentFrame->mvpMapPoints[i] &&
                                    !mCurrentFrame->mvbOutlier[i]) {
                                    mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    } else if (bOKReloc) {   // 重定位成功，清除VO标记
                        mbVO = false;
                    }

                    oslog::info(
                        "[Track::Track] Tracking status Reloc {}, Motion model {}",
                        (bOKReloc ? "GOOD" : "BAD"), (bOKMM ? "GOOD" : "BAD"));

                    bOK = bOKReloc || bOKMM;   // 任一成功即跟踪ok
                }
            }

            oslog::info(
                "[Track::Track]  {} ms ",
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                    std::chrono::steady_clock::now() - t_before_track)
                    .count());
        }

        if (!mCurrentFrame->mpReferenceKF)
            mCurrentFrame->mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred =
            std::chrono::steady_clock::now();

        double timePosePred =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                time_EndPosePred - time_StartPosePred)
                .count();
        vdPosePred_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack =
            std::chrono::steady_clock::now();
#endif
        // If we have an initial estimation of the camera pose and matching. Track
        // the local map.
        if (!mbOnlyTracking) {   // 正常SLAM模式，执行局部地图跟踪
            if (bOK) {
                bOK = TrackLocalMap();
            }
            if (!bOK) oslog::warn("Fail to track local map!");
        } else {   // 纯定位模式，只有非VO状态才执行局部地图跟踪
            // mbVO true means that there are few matches to MapPoints in the map. We
            // cannot retrieve a local map and therefore we do not perform
            // TrackLocalMap(). Once the system relocalizes the camera we will use the
            // local map again.
            if (bOK && !mbVO) bOK = TrackLocalMap();
        }

#ifdef REGISTER_TIMES
        oslog::info(
            "[Tracking::Track] TrackLocalMap took {} ms ",
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                std::chrono::steady_clock::now() - time_StartLMTrack)
                .count());
#endif

        if (bOK) {   // 局部地图跟踪成功，状态恢复OK
            mState = OK;
        } else if (mState == OK) {   // 原本正常，局部地图跟踪失败，进入短暂丢失
            oslog::info("[Tracking::Track] TrackLocalMap() not OK");
            if (mSensor.isImu()) {
                oslog::info("Track lost for less than one tick...");
                if (!pCurrentMap->isImuInitialized() ||
                    !pCurrentMap->GetInertialBA2()) {
                    oslog::info(
                        "IMU is not or recently initialized. resetting active map...");
                    mpSystem->ResetActiveMap();
                }

                mState = RECENTLY_LOST;
            } else {
                mState = RECENTLY_LOST;  // visual to lost
            }

            /*if(mCurrentFrame->mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
            mTimeStampLost = mCurrentFrame->mTimeStamp;
            //}
        }

        // Save frame if recent relocalization, since they are used for IMU reset
        // (as we are making copy, it shluld be once mCurrFrame is completely
        // modified)
        if ((mCurrentFrame->mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) &&
            (mCurrentFrame->mnId > mnFramesToResetIMU) && mSensor.isImu() &&
            pCurrentMap->isImuInitialized()) {   // 刚重定位不久，备份帧用于IMU复位
            oslog::info("Saving pointer to frame. imu needs reset...");

            // \todo{} check this situation.  This is a deep copy
            std::shared_ptr<Frame> pF = std::make_shared<Frame>(*mCurrentFrame);
            pF->mpPrevFrame = std::make_shared<Frame>(*mLastFrame);

            // Load preintegration
            //
            // \todo{AMM}.  I believe this was correct, want to make a deep copy of
            // the Preintegration?
            pF->mpImuPreintegratedFrame = std::make_shared<IMU::Preintegrated>();
            pF->mpImuPreintegratedFrame->CopyFrom(
                mCurrentFrame->mpImuPreintegratedFrame);
        }

        if (pCurrentMap->isImuInitialized()) {
            if (bOK) {
                if (mCurrentFrame->mnId == (mnLastRelocFrameId + mnFramesToResetIMU)) {
                    oslog::warn("resetting FRAME!!!");
                    ResetFrameIMU();
                } else if (mCurrentFrame->mnId > (mnLastRelocFrameId + 30)) {
                    mLastBias = mCurrentFrame->mImuBias;
                }
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack =
            std::chrono::steady_clock::now();

        double timeLMTrack =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                time_EndLMTrack - time_StartLMTrack)
                .count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        if (mpFrameDrawer) mpFrameDrawer->Update(shared_from_this());
        if (mCurrentFrame->isSet() && mpMapDrawer)
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->GetPose());

        if (bOK || mState == RECENTLY_LOST) {   // 跟踪成功或者短暂丢失，更新运动模型
            // Update motion model
            if (mLastFrame->isSet() && mCurrentFrame->isSet()) {
                Sophus::SE3f LastTwc = mLastFrame->GetPose().inverse();
                mVelocity = mCurrentFrame->GetPose() * LastTwc;   // 计算两帧之间相对位姿作为运动模型
                mbVelocity = true;
            } else {
                mbVelocity = false;
            }

            if (mSensor.isImu())
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->GetPose());

            // Clean VO matches
            for (int i = 0; i < mCurrentFrame->N; i++) {
                MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];
                if (pMP) {
                    if (pMP->Observations() < 1) {
                        mCurrentFrame->mvbOutlier[i] = false;
                        mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                    }
                }
            }

            // Delete temporal MapPoints  删除临时VO地图点
            for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(),
                                           lend = mlpTemporalPoints.end();
                 lit != lend; lit++) {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF =
                std::chrono::steady_clock::now();
#endif
            bool bNeedKF = NeedNewKeyFrame();   // 判断是否需要生成新关键帧

            // Check if we need to insert a new keyframe
            // if(bNeedKF && bOK)
            if (bNeedKF && (bOK || (mInsertKFsLost && mState == RECENTLY_LOST &&
                                    mSensor.isImu())))
                CreateNewKeyFrame();   // 创建新关键帧送入局部建图

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF =
                std::chrono::steady_clock::now();

            double timeNewKF =
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                    time_EndNewKF - time_StartNewKF)
                    .count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the Huber
            // Function) pass to the new keyframe, so that bundle adjustment will
            // finally decide if they are outliers or not. We don't want next frame to
            // estimate its position with those points so we discard them in the
            // frame. Only has effect if lastframe is tracked
            for (int i = 0; i < mCurrentFrame->N; i++) {
                if (mCurrentFrame->mvpMapPoints[i] && mCurrentFrame->mvbOutlier[i])
                    mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST) {
            if (pCurrentMap->KeyFramesInMap() <= 10) {
                mpSystem->ResetActiveMap();
                return;
            }

            if (mSensor.isImu()) {
                if (!pCurrentMap->isImuInitialized()) {
                    oslog::warn("Track lost before IMU initialisation, resetting...");
                    mpSystem->ResetActiveMap();
                    return;
                }
            }
            CreateMapInAtlas();

            return;
        }

        if (!mCurrentFrame->mpReferenceKF)
            mCurrentFrame->mpReferenceKF = mpReferenceKF;

        mLastFrame = std::make_shared<Frame>(*mCurrentFrame);   // 将当前帧拷贝保存为上一帧
    }

    if (mState == OK || mState == RECENTLY_LOST) {   // 正常或者短暂丢失，记录轨迹信息
        // Store frame pose information to retrieve the complete camera trajectory
        // afterwards.
        if (mCurrentFrame->isSet()) {
            Sophus::SE3f Tcr_ = mCurrentFrame->GetPose() *
                                mCurrentFrame->mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame->mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame->mTimeStamp);
            mlbLost.push_back(mState == LOST);
        } else {   // 当前帧位姿无效，复制上一条轨迹记录填充
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }
    }

#ifdef REGISTER_LOOP
    if (Stop()) {
        // Safe area to stop
        while (isStopped()) {
            usleep(3000);
        }
    }
#endif
}

// 双目/RGBD模式系统初始化，生成第一个关键帧与地图点
void Tracking::StereoInitialization() {
    oslog::info("[Tracking::StereoInitialization]");

    if (mCurrentFrame->N > 500) {   // 当前帧特征点数量大于500，满足初始化条件
        oslog::info(
            "[Tracking::StereoInitialization] Initializing map with {} points",
            mCurrentFrame->N);

        if (mSensor == SensorType::IMU_STEREO || mSensor == SensorType::IMU_RGBD) {   // IMU双目/RGBD模式
            if (!mCurrentFrame->mpImuPreintegrated ||
                !mLastFrame->mpImuPreintegrated) {   // 缺少IMU预积分数据，初始化失败
                oslog::warn("No IMU measurements to initialize this map");
                return;
            }

            if (!mFastInit && (mCurrentFrame->mpImuPreintegratedFrame->avgA -
                               mLastFrame->mpImuPreintegratedFrame->avgA)
                                      .norm() < 0.5) {   // 加速度变化太小，IMU无法观测重力，初始化失败
                oslog::warn("not enough acceleration to initialize");
                return;
            }

            mpImuPreintegratedFromLastKF =
                std::make_shared<IMU::Preintegrated>(IMU::Bias(), mImuCalib);
            mCurrentFrame->mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if (mSensor == SensorType::IMU_STEREO || mSensor == SensorType::IMU_RGBD) {   // IMU模式设置初始IMU系位姿速度
            Eigen::Matrix3f Rwb0 = mCurrentFrame->mImuCalib.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 = mCurrentFrame->mImuCalib.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            mCurrentFrame->SetImuPoseVelocity(Rwb0, twb0, Vwb0);
        } else {   // 普通双目/RGBD，相机位姿设为单位矩阵(世界坐标系原点)
            mCurrentFrame->SetPose(Sophus::SE3f());
        }

        // Create KeyFrame 基于当前帧构造第一个关键帧
        std::shared_ptr<KeyFrame> pKFini = std::make_shared<KeyFrame>(
            mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        // Insert KeyFrame in the map 将关键帧加入地图
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame 生成地图点
        if (!mpCamera2) {   // 没有第二个相机，RGBD模式
            oslog::debug(
                "[Tracking::StereoInitialization] No camera2, checking {} points",
                mCurrentFrame->N);

            for (int i = 0; i < mCurrentFrame->N; i++) {
                const float z = mCurrentFrame->mvDepth[i];
                if (z > 0) {   // 深度有效
                    Eigen::Vector3f x3D;
                    mCurrentFrame->UnprojectStereo(i, x3D);   // 反投影得到世界3D点
                    MapPoint* pNewMP =
                        new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame->mvpMapPoints[i] = pNewMP;
                }
            }
        } else {   // 双目相机，有camera2
            oslog::info(
                "[Tracking::StereoInitialization] Have camera2, checking {} points",
                mCurrentFrame->Nleft);
            for (int i = 0; i < mCurrentFrame->Nleft; i++) {
                int rightIndex = mCurrentFrame->mvLeftToRightMatch[i];
                oslog::info("{} {}", i, rightIndex);
                if (rightIndex != -1) {   // 存在左右匹配
                    Eigen::Vector3f x3D = mCurrentFrame->mvStereo3Dpoints[i];

                    MapPoint* pNewMP =
                        new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini, i);
                    pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame->Nleft);

                    pKFini->AddMapPoint(pNewMP, i);
                    pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame->Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame->mvpMapPoints[i] = pNewMP;
                    mCurrentFrame->mvpMapPoints[rightIndex + mCurrentFrame->Nleft] =
                        pNewMP;
                }
            }
        }

        Verbose::PrintMess(
            "[Tracking::StereoInitialization] New Map created with " +
                to_string(mpAtlas->MapPointsInMap()) + " points",
            Verbose::VERBOSITY_QUIET);

        // cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;
        mpLocalMapper->InsertKeyFrame(pKFini);   // 将首关键帧送入局部建图线程
        mLastFrame = std::shared_ptr<Frame>(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame->mnId;
        mpLastKeyFrame = pKFini;
        // mnLastRelocFrameId = mCurrentFrame->mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame->mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->GetPose());

        mState = OK;   // 初始化完成，跟踪状态置OK
    } else {
        oslog::info(
            "[Tracking::StereoInitialization] Frame has {} points, not enough to "
            "initialize map");
    }
}

// 单目模式初始化，两帧三角化生成初始地图
void Tracking::MonocularInitialization() {
    if (!mbReadyToInitializate) {   // 还没有设置初始化参考帧
        // Set Reference Frame
        if (mCurrentFrame->mvKeys.size() > 100) {   // 当前帧特征点足够，选做初始化参考帧
            mInitialFrame = std::make_shared<Frame>(*mCurrentFrame);
            mLastFrame = std::make_shared<Frame>(*mCurrentFrame);

            mvbPrevMatched.resize(mCurrentFrame->mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame->mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame->mvKeysUn[i].pt;

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            if (mSensor == SensorType::IMU_MONOCULAR) {
                mpImuPreintegratedFromLastKF =
                    std::make_shared<IMU::Preintegrated>(IMU::Bias(), mImuCalib);
                mCurrentFrame->mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            mbReadyToInitializate = true;

            return;
        }
    } else {   // 已经存在初始化参考帧，拿当前帧和参考帧做初始化
        if ((static_cast<int>(mCurrentFrame->mvKeys.size()) <= 100) ||
            ((mSensor == SensorType::IMU_MONOCULAR) &&
             (mLastFrame->mTimeStamp - mInitialFrame->mTimeStamp > 1.0))) {   // 特征点太少或者时间间隔过大，放弃初始化
            mbReadyToInitializate = false;

            return;
        }

        // Find correspondences 匹配参考帧与当前帧特征
        ORBmatcher matcher(0.9, true);
        int nmatches = matcher.SearchForInitialization(
            mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

        // Check if there are enough correspondences
        if (nmatches < 100) {   // 匹配数不足，初始化失败
            mbReadyToInitializate = false;
            return;
        }

        Sophus::SE3f Tcw;
        vector<bool> vbTriangulated;  // Triangulated Correspondences (mvIniMatches)

        // 两视图恢复运动，求解基础矩阵/单应矩阵，三角化得到3D点
        if (mpCamera->ReconstructWithTwoViews(mInitialFrame->mvKeysUn,
                                             mCurrentFrame->mvKeysUn, mvIniMatches,
                                             Tcw, mvIniP3D, vbTriangulated)) {
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses 参考帧位姿原点，当前帧位姿为求解得到Tcw
            mInitialFrame->SetPose(Sophus::SE3f());
            mCurrentFrame->SetPose(Tcw);

            CreateInitialMapMonocular();   // 基于三角化结果创建初始地图
        }
    }
}

// 单目初始化：根据两帧三角化结果创建地图、关键帧、地图点，做全局BA，尺度恢复
void Tracking::CreateInitialMapMonocular() {   // Create KeyFrames 创建单目初始地图
    // 构造初始帧关键帧，传入初始帧、当前地图、关键帧数据库
    std::shared_ptr<KeyFrame> pKFini = std::make_shared<KeyFrame>(
        mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
    // 构造当前帧关键帧，传入当前帧、当前地图、关键帧数据库
    std::shared_ptr<KeyFrame> pKFcur = std::make_shared<KeyFrame>(
        mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    // 如果是IMU+单目传感器，重置初始关键帧的IMU预积分对象
    if (mSensor == SensorType::IMU_MONOCULAR) pKFini->mpImuPreintegrated.reset();
    // 计算关键帧的BoW词袋向量，用于回环、重定位、特征匹配加速
    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map 将两个关键帧插入地图集
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    // 遍历初始化阶段得到的匹配对，生成地图点
    for (size_t i = 0; i < mvIniMatches.size(); i++) {
        // mvIniMatches[i]为-1代表该特征点无匹配，跳过
        if (mvIniMatches[i] < 0) continue;

        // Create MapPoint. 构造3D世界坐标
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        // 新建地图点，传入世界坐标、当前关键帧、当前地图
        MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

        // 给初始关键帧添加该地图点关联，i是初始帧特征索引
        pKFini->AddMapPoint(pMP, i);
        // 给当前关键帧添加该地图点关联，mvIniMatches[i]是当前帧匹配特征索引
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        // 地图点添加观测：记录哪一个关键帧、对应哪个特征看到该点
        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        // 计算地图点的代表性描述子，从所有观测描述子里选出最具代表性的
        pMP->ComputeDistinctiveDescriptors();
        // 更新地图点法向量与观测深度范围
        pMP->UpdateNormalAndDepth();

        // Fill Current Frame structure 填充当前帧的地图点指针，标记该点不是外点
        mCurrentFrame->mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame->mvbOutlier[mvIniMatches[i]] = false;

        // Add to Map 将地图点加入地图集管理
        mpAtlas->AddMapPoint(pMP);
    }

    // Update Connections 更新两个关键帧的共视关系、连接权重
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // 获取初始关键帧观测到的全部地图点集合
    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment 全局光束平差，对初始地图做BA优化，迭代20次
    Verbose::PrintMess("New Map created with " +
                           to_string(mpAtlas->MapPointsInMap()) + " points",
                       Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

    // 计算初始关键帧场景中值深度，参数2代表忽略前2个最小值做统计
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    // IMU单目模式使用4.0缩放系数，普通单目使用1.0
    if (mSensor == SensorType::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth;  // 4.0f
    else
        invMedianDepth = 1.0f / medianDepth;

    // TODO Check, originally 100 tracks
    // 初始化校验：中值深度非法，或者当前关键帧有效跟踪地图点不足50，则判定初始化失败，重置活跃地图
    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50) {
        Verbose::PrintMess("Wrong initialization, resetting...",
                           Verbose::VERBOSITY_QUIET);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline 缩放两帧之间基线，修正单目尺度不确定性
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points 对所有地图点的世界坐标做同样尺度缩放
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
        if (vpAllMapPoints[iMP]) {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    // IMU单目模式，建立关键帧时序链表，设置预积分
    if (mSensor == SensorType::IMU_MONOCULAR) {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        // 基于当前关键帧更新后的bias，生成新的IMU预积分对象，用于下一帧
        mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(
            pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
    }

    // 将两个初始化关键帧送入局部建图线程
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    // 记录当前关键帧时间戳，作为局部建图初始时间
    mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

    // 当前帧位姿同步为刚刚初始化完成的当前关键帧位姿
    mCurrentFrame->SetPose(pKFcur->GetPose());
    // 更新上一次关键帧帧ID、上一关键帧指针
    mnLastKeyFrameId = mCurrentFrame->mnId;
    mpLastKeyFrame = pKFcur;
    // mnLastRelocFrameId = mInitialFrame->mnId;

    // 局部关键帧列表存入两个初始化KF
    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    // 局部地图点取地图全部点
    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    // 设置参考关键帧
    mpReferenceKF = pKFcur;
    mCurrentFrame->mpReferenceKF = pKFcur;

    // Compute here initial velocity 获取全部关键帧，计算初始速度相关旋转量
    auto vKFs = mpAtlas->GetAllKeyFrames();

    // 计算两个关键帧之间位姿变换 deltaT = T_cur * T_init^{-1}
    Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity = false;
    // 李群转李代数，获取旋转向量phi
    Eigen::Vector3f phi = deltaT.so3().log();

    // 时间比例系数：(当前帧-上帧时间)/(当前帧-初始帧时间)
    double aux = (mCurrentFrame->mTimeStamp - mLastFrame->mTimeStamp) /
                 (mCurrentFrame->mTimeStamp - mInitialFrame->mTimeStamp);
    // 旋转向量按时间比例缩放
    phi *= aux;

    // 拷贝当前帧构造为mLastFrame，作为下一帧的上一帧
    mLastFrame = std::make_shared<Frame>(*mCurrentFrame);

    // 设置可视化参考地图点
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // 设置可视化相机位姿
    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    // 记录该地图的起源关键帧（初始化第一帧）
    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    // 跟踪状态置为正常OK
    mState = OK;
    // 保存初始化对应的关键帧ID
    initID = pKFcur->mnId;
}

void Tracking::CreateMapInAtlas() {
    // 记录本次初始化开始对应的帧ID
    mnLastInitFrameId = mCurrentFrame->mnId;
    // 在Atlas中创建一块全新地图
    mpAtlas->CreateNewMap();
    // 如果传感器带IMU，标记该地图为惯性地图
    if (mSensor.isImu()) mpAtlas->SetInertialSensor();
    mbSetInit = false;

    // 下一帧作为初始化候选帧ID
    mnInitialFrameId = mCurrentFrame->mnId + 1;
    // 跟踪状态置为还没有有效图像
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF 重置速度标志
    mbVelocity = false;
    // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the
    // current id, because it is the new starting point for new map
    oslog::debug("First frame id in map: {}", to_string(mnLastInitFrameId + 1));
    mbVO = false;  // Init value for know if there are enough MapPoints in the
                   // last KF
    // 单目传感器置初始化就绪标志为false，等待满足条件再初始化
    if (mSensor.isMonocular()) mbReadyToInitializate = false;

    // IMU传感器，重置预积分对象，bias置零
    if (mSensor.isImu() && mpImuPreintegratedFromLastKF) {
        mpImuPreintegratedFromLastKF =
            std::make_shared<IMU::Preintegrated>(IMU::Bias(), mImuCalib);
    }

    // 清空上一关键帧、参考关键帧智能指针
    if (mpLastKeyFrame) mpLastKeyFrame.reset();
    if (mpReferenceKF) mpReferenceKF.reset();

    // 构造空的上一帧、当前帧Frame对象
    mLastFrame = std::make_shared<Frame>();
    mCurrentFrame = std::make_shared<Frame>();
    // 清空初始化匹配数组
    mvIniMatches.clear();

    // 标记地图已经被创建
    mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame() {
    // 遍历上一帧全部特征点
    for (int i = 0; i < mLastFrame->N; i++) {
        MapPoint* pMP = mLastFrame->mvpMapPoints[i];

        if (pMP) {
            // 获取该地图点被替换后的新地图点
            MapPoint* pRep = pMP->GetReplaced();
            // 如果发生替换，把帧内地图点指针更新为替换后的点
            if (pRep) {
                mLastFrame->mvpMapPoints[i] = pRep;
            }
        }
    }
}

bool Tracking::TrackReferenceKeyFrame() {
    // Compute Bag of Words vector 计算当前帧BoW词袋，用于词袋匹配
    mCurrentFrame->ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    // ORB匹配器：阈值0.7，开启检查旋转
    ORBmatcher matcher(0.7, true);
    vector<MapPoint*> vpMapPointMatches;

    // 通过BoW在参考关键帧和当前帧之间做特征匹配，得到匹配地图点
    int nmatches =
        matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    oslog::debug("Tracking::TrackReferenceKeyFrame:  Got {} matches", nmatches);

    // 匹配数小于15，跟踪参考关键帧失败返回false
    if (nmatches < 15) {
        oslog::warn("TRACK_REF_KF: Less than 15 matches!!");
        return false;
    }

    // 将匹配到的地图点赋值给当前帧
    mCurrentFrame->mvpMapPoints = vpMapPointMatches;
    // 初始位姿猜测用上一帧的位姿
    mCurrentFrame->SetPose(mLastFrame->GetPose());

    // mCurrentFrame->PrintPointDistribution();
    // cout << " TrackReferenceKeyFrame mLastFrame->mTcw:  " << mLastFrame->mTcw
    // << endl;
    // 位姿优化：只优化相机位姿，地图点固定
    Optimizer::PoseOptimization(mCurrentFrame);

    // Discard outliers 剔除优化标记出来的外点
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame->N; i++) {
        // if(i >= mCurrentFrame->Nleft) break;
        if (mCurrentFrame->mvpMapPoints[i]) {
            // 如果该点被标记外点
            if (mCurrentFrame->mvbOutlier[i]) {
                MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];

                // 当前帧清空该地图点指针，外点标志复位
                mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                mCurrentFrame->mvbOutlier[i] = false;
                // 区分左右目，清除该地图点的跟踪可见标记
                if (i < mCurrentFrame->Nleft) {
                    pMP->mbTrackInView = false;
                } else {
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                // 记录该地图点最后被哪一帧观测
                pMP->mnLastFrameSeen = mCurrentFrame->mnId;
                // 总匹配计数减一
                nmatches--;
            } else if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0) {
                // 有效地图点，计数+1
                nmatchesMap++;
            }
        }
    }

    // IMU模式不做匹配数校验直接返回true，靠IMU约束
    if (mSensor.isImu()) return true;

    // 视觉模式要求有效地图点匹配>=10才算跟踪成功
    return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame() {
    // Update pose according to reference keyframe 根据参考关键帧更新上一帧位姿
    auto pRef = mLastFrame->mpReferenceKF;
    // 获取存储的相对位姿 T_lr = T_last * T_ref^{-1}
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    // T_last = T_lr * T_ref，恢复上一帧世界坐标系位姿
    mLastFrame->SetPose(Tlr * pRef->GetPose());

    // 如果上帧本身就是关键帧，或者单目，或者不是纯定位模式，直接返回，不需要生成临时VO地图点
    if (mnLastKeyFrameId == mLastFrame->mnId || mSensor.isMonocular() ||
        !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // 构建<深度,特征索引>pair数组，用于按深度排序
    vector<pair<float, int>> vDepthIdx;
    // 获取有效特征数量，区分鱼眼双目和普通单目
    const int Nfeat = mLastFrame->Nleft == -1 ? mLastFrame->N : mLastFrame->Nleft;
    vDepthIdx.reserve(Nfeat);
    for (int i = 0; i < Nfeat; i++) {
        float z = mLastFrame->mvDepth[i];
        // 深度>0代表深度有效
        if (z > 0) {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    // 没有有效深度点直接返回
    if (vDepthIdx.empty()) return;

    // 按照深度从小到大排序，优先处理近处点
    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame->mvpMapPoints[i];
        // 当前特征没有地图点，或者已有地图点观测数小于1，需要新建临时VO地图点
        if (!pMP)
            bCreateNew = true;
        else if (pMP->Observations() < 1)
            bCreateNew = true;

        if (bCreateNew) {
            Eigen::Vector3f x3D;

            // 根据相机类型反投影2D像素到3D世界点
            if (mLastFrame->Nleft == -1) {
                mLastFrame->UnprojectStereo(i, x3D);
            } else {
                x3D = mLastFrame->UnprojectStereoFishEye(i);
            }

            // 创建临时视觉里程计地图点，只在本帧生效，不进入全局地图
            MapPoint* pNewMP =
                new MapPoint(x3D, mpAtlas->GetCurrentMap(), mLastFrame, i);
            mLastFrame->mvpMapPoints[i] = pNewMP;

            // 存入临时点列表，后续会被清理，不参与局部建图
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        } else {
            nPoints++;
        }

        // 深度超过阈值并且已经生成100个点，停止生成
        if (vDepthIdx[j].first > mThDepth && nPoints > 100) break;
    }
}

bool Tracking::TrackWithMotionModel() {
    // 记录函数开始时间，统计耗时
    std::chrono::steady_clock::time_point t_start =
        std::chrono::steady_clock::now();

    // ORB匹配器，阈值0.9，开启旋转检查
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    // 更新上帧位姿，纯定位模式生成VO临时地图点
    UpdateLastFrame();

    // 如果IMU已经初始化，并且距离上次重定位帧超过重置IMU帧数，则使用IMU预测位姿
    if (mpAtlas->isImuInitialized() &&
        (mCurrentFrame->mnId > mnLastRelocFrameId + mnFramesToResetIMU)) {
        // Predict state with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    } else {
        // 没有可用IMU预测，用运动模型预测当前帧位姿：速度 * 上帧位姿
        mCurrentFrame->SetPose(mVelocity * mLastFrame->GetPose());
    }

    // 清空当前帧所有地图点指针
    fill(mCurrentFrame->mvpMapPoints.begin(), mCurrentFrame->mvpMapPoints.end(),
         static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame 将上一帧地图点投影到当前帧做匹配
    int th;
    // 双目模式搜索阈值7像素，单目15像素
    if (mSensor == SensorType::STEREO)
        th = 7;
    else
        th = 15;

    // 投影匹配，用上帧地图点匹配当前帧特征
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th,
                                               mSensor.isMonocular());

    oslog::debug(
        "[Tracking::TrackWithMotionModel] found {} matches in first pass",
        nmatches);
    // If few matches, uses a wider window search 第一轮匹配不足20，扩大搜索窗口重新匹配
    if (nmatches < 20) {
        oslog::info(
            "[Tracking::TrackWithMotionModel] Not enough matches, wider window "
            "search!!");
        fill(mCurrentFrame->mvpMapPoints.begin(), mCurrentFrame->mvpMapPoints.end(),
             static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th,
                                               mSensor.isMonocular());
        oslog::debug(
            "[Tracking::TrackWithMotionModel] Matches with wider search: {}",
            to_string(nmatches));
    }

    // 扩大窗口后仍然不足20匹配
    if (nmatches < 20) {
        oslog::info(
            "[Tracking::TrackWithMotionModel] Not enough matches after wider "
            "search!!");

        // IMU传感器不直接判定失败，交给IMU约束；纯视觉返回false运动模型跟踪失败
        if (mSensor.isImu())
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches 基于匹配点做位姿优化
    Optimizer::PoseOptimization(mCurrentFrame);

    // Discard outliers 剔除外点，统计有效地图点匹配数
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame->N; i++) {
        if (mCurrentFrame->mvpMapPoints[i]) {
            if (mCurrentFrame->mvbOutlier[i]) {
                MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];

                mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                mCurrentFrame->mvbOutlier[i] = false;
                if (i < mCurrentFrame->Nleft) {
                    pMP->mbTrackInView = false;
                } else {
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame->mnId;
                nmatches--;
            } else if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0) {
                nmatchesMap++;
            }
        }
    }

    oslog::debug(
        "[Tracking::TrackWithMotionModel] this led to {} matches against the map",
        nmatchesMap);

    oslog::info(
        "[Tracking::TrackWithMotionModel]  {} ms ",
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - t_start)
            .count());

    // 纯定位模式：标记VO状态，总匹配>20返回true
    if (mbOnlyTracking) {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    // IMU传感器直接返回true；视觉模式判断有效地图点匹配>=10
    if (mSensor.isImu()) {
        return true;
    } else {
        const bool goodTracking = (nmatchesMap >= 10);
        oslog::info("[Tracking::TrackWithMotionModel] tracking is {}",
                    (goodTracking ? "GOOD" : "BAD"));
        return goodTracking;
    }
}

bool Tracking::TrackLocalMap() {
    // 计时起点
    std::chrono::steady_clock::time_point t_start =
        std::chrono::steady_clock::now();

    // We have an estimation of the camera pose and some map points tracked in the
    // frame. We retrieve the local map and try to find matches to points in the
    // local map.
    // 跟踪帧计数+1
    mTrackedFr++;

    // 更新局部关键帧、局部地图点
    UpdateLocalMap();

    std::chrono::steady_clock::time_point t_post_update_local_map =
        std::chrono::steady_clock::now();

    // 将局部地图点投影到当前帧，寻找更多匹配
    SearchLocalPoints();

    std::chrono::steady_clock::time_point t_post_search_local_points =
        std::chrono::steady_clock::now();

    // TOO check outliers before PO 统计优化前地图点数量与外点数量，用于日志打印
    int aux1 = 0, aux2 = 0;
    for (int i = 0; i < mCurrentFrame->N; i++) {
        if (mCurrentFrame->mvpMapPoints[i]) {
            aux1++;
            if (mCurrentFrame->mvbOutlier[i]) aux2++;
        }
    }

    oslog::info("Before optimization, {} map points, {} outliers", aux1, aux2);

    int inliers;
    // 根据IMU初始化状态选择不同的位姿优化器
    if (!mpAtlas->isImuInitialized()) {
        oslog::trace("[Tracking::TrackLocalMap] No IMU, conventional optimization");
        // 无IMU：普通视觉位姿优化
        Optimizer::PoseOptimization(mCurrentFrame);
    } else {
        // IMU已初始化，但距离重定位很近，仍然只用视觉位姿优化
        if (mCurrentFrame->mnId <= mnLastRelocFrameId + mnFramesToResetIMU) {
            oslog::info("[Tracking::TrackLocalMap] PoseOptimization");
            Optimizer::PoseOptimization(mCurrentFrame);
        } else {
            // 距离重定位较远，使用视觉+IMU联合位姿优化
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            //       //  && (mnMatchesInliers>30))
            if (!mbMapUpdated) {
                oslog::info(
                    "[Tracking::TrackLocalMap] PoseInertialOptimizationLastFrame");
                // 使用上一帧IMU预积分做惯性位姿优化
                inliers = Optimizer::PoseInertialOptimizationLastFrame(
                    mCurrentFrame);  // ,
                                     // !mpLastKeyFrame->GetMap()->GetInertialBA1());
            } else {
                oslog::info(
                    "[Tracking::TrackLocalMap] PoseInertialOptimizationLastKeyFrame");
                // 使用上一关键帧IMU预积分做惯性位姿优化
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(
                    mCurrentFrame);  // ,
                                     // !mpLastKeyFrame->GetMap()->GetInertialBA1());
            }
        }
    }

    std::chrono::steady_clock::time_point t_post_pose_optimization =
        std::chrono::steady_clock::now();

    // 统计优化后地图点与外点数量
    aux1 = 0, aux2 = 0;
    for (int i = 0; i < mCurrentFrame->N; i++) {
        if (mCurrentFrame->mvpMapPoints[i]) {
            aux1++;
            if (mCurrentFrame->mvbOutlier[i]) aux2++;
        }
    }

    oslog::info("After optimization, {} map points, {} outliers", aux1, aux2);

    mnMatchesInliers = 0;

    // Update MapPoints Statistics 更新地图点统计信息，统计内点数目
    for (int i = 0; i < mCurrentFrame->N; i++) {
        if (mCurrentFrame->mvpMapPoints[i]) {
            if (!mCurrentFrame->mvbOutlier[i]) {
                // 该地图点被成功观测，增加found计数
                mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking) {
                    // if( i < 10 )
                    // oslog::debug("{}, observations {}", i ,
                    // mCurrentFrame->mvpMapPoints[i]->Observations() );

                    // 正常建图模式，只有存在历史观测才算有效内点
                    if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0) {
                        mnMatchesInliers++;
                    }
                } else {
                    // 纯定位模式直接计数
                    mnMatchesInliers++;
                }
            } else if (mSensor == SensorType::STEREO) {
                // 双目模式外点直接清空帧内地图点指针
                mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            }
        }
    }

    oslog::debug("[Tracking::TrackLocalMap] mnMatchesInliers: {}",
                 mnMatchesInliers);

    std::chrono::steady_clock::time_point t_end =
        std::chrono::steady_clock::now();

    // 打印各阶段耗时
    oslog::info(
        "[Tracking::TrackLocalMap] Elapsed time: Update local map {:.4}, search "
        "local points {:.4}, pose optimization {:.4} everything else {:.4}",
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t_post_update_local_map - t_start)
            .count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t_post_search_local_points - t_post_update_local_map)
            .count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t_post_pose_optimization - t_post_search_local_points)
            .count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t_end - t_post_pose_optimization)
            .count());

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    // 将当前内点数目传给局部建图线程
    mpLocalMapper->mnMatchesInliers = mnMatchesInliers;
    // 如果刚发生过重定位，内点不足50直接跟踪失败
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames &&
        mnMatchesInliers < 50)
        return false;

    // 刚丢失恢复状态，只要大于10个内点就成功
    if ((mnMatchesInliers > 10) && (mState == RECENTLY_LOST)) return true;

    // 分传感器类型设置跟踪成功阈值
    if (mSensor == SensorType::IMU_MONOCULAR) {
        if ((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) ||
            (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized()))
            return false;
        else
            return true;
    } else if (mSensor == SensorType::IMU_STEREO ||
               mSensor == SensorType::IMU_RGBD) {
        if (mnMatchesInliers < 15)
            return false;
        else
            return true;
    } else {
        if (mnMatchesInliers < 15)
            return false;
        else
            return true;
    }
}

bool Tracking::NeedNewKeyFrame() {
    // IMU还未完成初始化时，强制按时间间隔插入关键帧
    if (mSensor.isImu() && !mpAtlas->GetCurrentMap()->isImuInitialized()) {
        if (mSensor == SensorType::IMU_MONOCULAR &&
            (mCurrentFrame->mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
            return true;
        else if ((mSensor == SensorType::IMU_STEREO ||
                  mSensor == SensorType::IMU_RGBD) &&
                 (mCurrentFrame->mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
            return true;
        else
            return false;
    }

    // 纯定位模式，不需要插入关键帧
    if (mbOnlyTracking) return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // 如果局部建图被回环检测停止，则不插入关键帧
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == SensorType::MONOCULAR)
        {*/
        oslog::info(
            "[Tracking::NeedNewKeyFrame] Not adding KF because localmap is "
            "stopped");

        return false;
    }

    // 获取当前地图关键帧总数
    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last
    // relocalisation
    // 刚重定位不久，地图关键帧已经很多，暂时不插入关键帧
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames &&
        nKFs > mMaxFrames) {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    // if(nKFs<=2)
    nMinObs = 2;
    // 获取参考关键帧中观测数>=nMinObs的地图点数量
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes? 查询局部建图线程是否空闲，可以接收关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be
    // potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;

    // 非单目传感器，统计近处深度点：已跟踪、未跟踪的数量
    if (!mSensor.isMonocular()) {
        const size_t N =
            (mCurrentFrame->Nleft == -1) ? mCurrentFrame->N : mCurrentFrame->Nleft;
        for (size_t i = 0; i < N; i++) {
            if (mCurrentFrame->mvDepth[i] > 0 &&
                mCurrentFrame->mvDepth[i] < mThDepth) {
                if (mCurrentFrame->mvpMapPoints[i] && !mCurrentFrame->mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
        oslog::info(
            "[Tracking::NeedNewKeyFrame] tracked close points: {}; non tracked "
            "close points: {}",
            to_string(nTrackedClose), to_string(nNonTrackedClose));
    }

    // 近处有效跟踪点不足100，且有大于70个近处未跟踪特征，需要生成新关键帧来补充地图点
    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds 参考关键帧匹配比例阈值
    float thRefRatio = 0.75f;
    // 地图关键帧很少，放宽阈值
    if (nKFs < 2) thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==SensorType::STEREO ||
    mSensor==SensorType::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident
    about the stereo observations. thRefRatio = 0.9f;
    }*/

    // 单目模式阈值提高到0.9，对关键帧插入更保守
    if (mSensor == SensorType::MONOCULAR) thRefRatio = 0.9f;

    if (mpCamera2) thRefRatio = 0.75f;

    // IMU单目，根据当前内点数量动态调整阈值
    if (mSensor == SensorType::IMU_MONOCULAR) {
        if (mnMatchesInliers > 350)  // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    oslog::info(
        "[Tracking::NeedNewKeyFrame] mnMatchesInliers: {}; nRefMatches: {}; "
        "thRefRatio: {}",
        mnMatchesInliers, nRefMatches, thRefRatio);

    // Condition 1a: More than "MaxFrames" have passed from last keyframe
    // insertion 距离上一个关键帧超过最大帧数
    const bool c1a = mCurrentFrame->mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    // 距离上关键帧超过最小帧数，并且局部建图线程空闲
    const bool c1b =
        ((mCurrentFrame->mnId >= mnLastKeyFrameId + mMinFrames) &&
         bLocalMappingIdle);  // mpLocalMapper->KeyframesInQueue() < 2);
    // Condition 1c: tracking is weak
    // 非单目/IMU传感器，跟踪匹配数显著下降或者近处大量点没有地图点
    const bool c1c =
        mSensor != SensorType::MONOCULAR &&
        mSensor != SensorType::IMU_MONOCULAR &&
        mSensor != SensorType::IMU_STEREO && mSensor != SensorType::IMU_RGBD &&
        (mnMatchesInliers < nRefMatches * 0.50 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of
    // visual odometry compared to map matches.
    // 当前跟踪点相比参考关键帧比例下降，或者近处大量点缺失地图点，同时保证至少15个内点
    const bool c2 =
        (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) &&
         mnMatchesInliers > 15);

    // Temporal condition for Inertial cases IMU传感器时间间隔条件
    bool c3 = false;
    if (mpLastKeyFrame) {
        if (mSensor == SensorType::IMU_MONOCULAR) {
            if ((mCurrentFrame->mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                c3 = true;
        } else if (mSensor == SensorType::IMU_STEREO ||
                   mSensor == SensorType::IMU_RGBD) {
            if ((mCurrentFrame->mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                c3 = true;
        }
    }

    // IMU单目，跟踪偏弱或者刚刚从丢失恢复，需要插入关键帧
    bool c4 = false;
    if ((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) ||
         mState == RECENTLY_LOST) &&
        (mSensor ==
         SensorType::IMU_MONOCULAR))  // MODIFICATION_2, originally
                                     // ((((mnMatchesInliers<75) &&
                                     // (mnMatchesInliers>15)) ||
                                     // mState==RECENTLY_LOST) && ((mSensor ==
                                     // SensorType::IMU_MONOCULAR)))
        c4 = true;
    else
        c4 = false;

    oslog::debug(
        "[Tracking::NeedNewKeyFrame] c1a={}; c1b={}; c1c={}; c2={}; c3={}; c4={}",
        c1a, c1b, c1c, c2, c3, c4);

    // 判断是否满足新建关键帧条件
    if (((c1a || c1b || c1c) && c2) || c3 || c4) {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        // 如果局部建图空闲或者正在初始化，则直接新建关键帧
        if (bLocalMappingIdle || mpLocalMapper->IsInitializing()) {
            oslog::info("[Tracking::NeedNewKeyFrame] !!! Create a new keyframe!");
            return true;
        } else {
            // 局部建图繁忙，打断正在进行的BA
            oslog::info(
                "[Tracking::NeedNewKeyFrame] !!! Want to create a new keyframe, "
                "interrupting BA");
            mpLocalMapper->InterruptBA();
            // 双目/RGBD，队列未满3个允许插入，否则不插入
            if (mSensor != SensorType::MONOCULAR &&
                mSensor != SensorType::IMU_MONOCULAR) {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            } else {
                oslog::info(
                    "[Tracking::NeedNewKeyFrame] NeedNewKeyFrame: localmap is busy");
                return false;
            }
        }
    } else {
        return false;
    }
}

void Tracking::CreateNewKeyFrame() {
    // 如果局部建图正在初始化，且IMU还没初始化完成，不生成新关键帧
    if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized()) return;

    // 设置局部建图不允许被停止，准备插入关键帧
    if (!mpLocalMapper->SetNotStop(true)) return;

    // 使用当前帧构造新关键帧
    std::shared_ptr<KeyFrame> pKF = std::make_shared<KeyFrame>(
        mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    // IMU已经初始化，标记该关键帧带IMU信息
    if (mpAtlas->isImuInitialized())  //  || mpLocalMapper->IsInitializing())
        pKF->bImu = true;

    // 将当前帧IMU bias赋值给新关键帧
    pKF->SetNewBias(mCurrentFrame->mImuBias);
    // 更新参考关键帧为新建的KF
    mpReferenceKF = pKF;
    mCurrentFrame->mpReferenceKF = pKF;

    // 维护关键帧双向时序链表 mPrevKF / mNextKF
    if (mpLastKeyFrame) {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    } else {
        oslog::info("[Tracking::CreateNewKeyFrame] No last KF in KF creation!!");
    }
    // Reset preintegration from last KF (Create new object)
    // IMU传感器，基于新关键帧bias创建新的IMU预积分对象
    if (mSensor.isImu()) {
        mpImuPreintegratedFromLastKF =
            std::make_shared<IMU::Preintegrated>(pKF->GetImuBias(), pKF->mImuCalib);
    }

    // TODO check if incluide imu_stereo
    // 双目/RGBD传感器，利用深度生成新地图点
    if (!mSensor.isMonocular()) {
        mCurrentFrame->UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if (mSensor.isImu()) maxPoint = 100;

        vector<pair<float, int>> vDepthIdx;
        int N =
            (mCurrentFrame->Nleft != -1) ? mCurrentFrame->Nleft : mCurrentFrame->N;
        vDepthIdx.reserve(mCurrentFrame->N);
        // 收集有效深度的特征点，保存<深度，特征索引>
        for (int i = 0; i < N; i++) {
            float z = mCurrentFrame->mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty()) {
            // 按深度从小到大排序，优先近处点
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];
                // 当前特征没有地图点，或者地图点观测数小于1，需要新建地图点
                if (!pMP) {
                    bCreateNew = true;
                } else if (pMP->Observations() < 1) {
                    bCreateNew = true;
                    mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if (bCreateNew) {
                    Eigen::Vector3f x3D;

                    // 根据相机类型反投影2D像素得到3D世界坐标
                    if (mCurrentFrame->Nleft == -1) {
                        mCurrentFrame->UnprojectStereo(i, x3D);
                    } else {
                        x3D = mCurrentFrame->UnprojectStereoFishEye(i);
                    }

                    // 创建新地图点，关联当前新建关键帧
                    MapPoint* pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF, i);

                    // Check if it is a stereo observation in order to not
                    // duplicate mappoints
                    // 双目相机，如果存在左右匹配，右目特征也关联同一个地图点，避免重复建点
                    if (mCurrentFrame->Nleft != -1 &&
                        mCurrentFrame->mvLeftToRightMatch[i] >= 0) {
                        mCurrentFrame->mvpMapPoints[mCurrentFrame->Nleft +
                                                    mCurrentFrame->mvLeftToRightMatch[i]] =
                            pNewMP;
                        pNewMP->AddObservation(
                            pKF,
                            mCurrentFrame->Nleft + mCurrentFrame->mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP, mCurrentFrame->Nleft +
                                                     mCurrentFrame->mvLeftToRightMatch[i]);
                    }

                    // 关键帧记录该地图点，地图点计算描述子、法向深度，加入地图集
                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame->mvpMapPoints[i] = pNewMP;
                    nPoints++;
                } else {
                    nPoints++;
                }

                // 深度超过阈值，并且已经生成足够点，停止创建
                if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
                    break;
                }
            }
            oslog::info("[Tracking::CreateNewKeyFrame] New stereo KF with {} points",
                        nPoints);
        }
    }

    // 将新建关键帧插入局部建图线程队列
    mpLocalMapper->InsertKeyFrame(pKF);
    // 恢复局部建图可以被停止
    mpLocalMapper->SetNotStop(false);

    // 更新上一关键帧ID与指针
    mnLastKeyFrameId = mCurrentFrame->mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
    // Do not search map points already matched
    // 遍历当前帧已经匹配的地图点，标记可见，清除跟踪标记，坏点置空
    for (vector<MapPoint*>::iterator vit = mCurrentFrame->mvpMapPoints.begin(),
                                     vend = mCurrentFrame->mvpMapPoints.end();
         vit != vend; vit++) {
        MapPoint* pMP = *vit;
        if (pMP) {
            if (pMP->isBad()) {
                *vit = static_cast<MapPoint*>(NULL);
            } else {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame->mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    // 遍历全部局部地图点，判断是否在当前帧视锥内，统计待匹配点数量
    for (vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(),
                                     vend = mvpLocalMapPoints.end();
         vit != vend; vit++) {
        MapPoint* pMP = *vit;

        // 已经在本帧处理过，跳过
        if (pMP->mnLastFrameSeen == mCurrentFrame->mnId) continue;
        // 坏点跳过
        if (pMP->isBad()) continue;
        // Project (this fills MapPoint variables for matching)
        // 判断地图点是否在相机视锥内，0.5阈值，会填充mTrackProjX/Y投影坐标
        if (mCurrentFrame->isInFrustum(pMP, 0.5)) {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        // 如果标记在视野内，保存投影点，用于匹配
        if (pMP->mbTrackInView) {
            mCurrentFrame->mmProjectPoints[pMP->mnId] =
                cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    // 如果存在待匹配局部地图点，执行投影匹配
    if (nToMatch > 0) {
        ORBmatcher matcher(0.8);
        int th = 1;
        // RGBD传感器搜索像素阈值3
        if (mSensor == SensorType::RGBD || mSensor == SensorType::IMU_RGBD) th = 3;
        // IMU初始化不同阶段设置不同搜索阈值
        if (mpAtlas->isImuInitialized()) {
            if (mpAtlas->GetCurrentMap()->GetInertialBA2())
                th = 2;
            else
                th = 6;
        } else if (!mpAtlas->isImuInitialized() && mSensor.isImu()) {
            th = 10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        // 刚刚重定位，扩大搜索窗口
        if (mCurrentFrame->mnId < mnLastRelocFrameId + 2) th = 5;

        // 跟踪丢失状态，进一步放大搜索阈值
        if (mState == LOST ||
            mState == RECENTLY_LOST)  // Lost for less than 1 second
            th = 15;                    // 15
        // 局部地图点投影匹配，把局部地图点匹配到当前帧特征
        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints,
                                                 th, mpLocalMapper->mbFarPoints,
                                                 mpLocalMapper->mThFarPoints);
    }
}

void Tracking::UpdateLocalMap() {
    // This is for visualization
    // 设置可视化模块的参考地图点
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update 更新局部关键帧集合、局部地图点集合
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
    // 清空局部地图点容器，准备重新收集当前帧对应的局部地图点
    mvpLocalMapPoints.clear();
    // 统计有效局部地图点的数量
    int count_pts = 0;
    // 遍历所有局部关键帧
    for (auto pKF : mvpLocalKeyFrames) {
        // 获取该关键帧中所有匹配到的地图点
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
        // 遍历该关键帧的每一个地图点
        for (auto pMP : vpMPs) {
            // 空指针直接跳过
            if (!pMP) continue;
            // 如果该地图点已经标记为当前帧引用过，跳过避免重复加入
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame->mnId) continue;
            // 过滤掉坏的、失效的地图点
            if (!pMP->isBad()) {
                // 有效点计数自增
                count_pts++;
                // 将该地图点加入局部地图点集合
                mvpLocalMapPoints.push_back(pMP);
                // 标记该地图点被当前帧引用，防止本帧重复添加
                pMP->mnTrackReferenceForFrame = mCurrentFrame->mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames() {
    // 每个地图点对观测过它的关键帧进行投票，统计每个关键帧被多少地图点观测
    map<std::shared_ptr<KeyFrame>, int> keyframeCounter;

    // 判断IMU是否完成初始化 或者 当前帧距离上一次重定位帧间隔小于2帧
    if (!mpAtlas->isImuInitialized() ||
        (mCurrentFrame->mnId < mnLastRelocFrameId + 2)) {
        // 遍历当前帧所有特征点
        for (size_t i = 0; i < mCurrentFrame->N; i++) {
            // 获取当前帧该特征点关联的地图点
            MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];
            if (pMP) {
                // 地图点有效，没有被标记为坏点
                if (!pMP->isBad()) {
                    // 获取该地图点所有观测记录 <关键帧, <特征点索引, 其他信息>>
                    const auto observations = pMP->GetObservations();
                    // 遍历该地图点全部观测关键帧，进行投票计数
                    for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::const_iterator
                             it = observations.begin(),
                             itend = observations.end();
                         it != itend; it++)
                        keyframeCounter[it->first]++;
                } else {
                    // 地图点已经损坏，清空当前帧该位置地图点指针
                    mCurrentFrame->mvpMapPoints[i] = NULL;
                }
            }
        }
    } else {
        // IMU已经初始化，且距离上次重定位帧数足够，使用上一帧的地图点做投票
        for (size_t i = 0; i < mLastFrame->N; i++) {
            // 使用上一帧，因为当前帧此时还没有匹配关系
            if (mLastFrame->mvpMapPoints[i]) {
                // 获取上一帧该特征点关联的地图点
                MapPoint* pMP = mLastFrame->mvpMapPoints[i];
                if (!pMP) continue;
                // 地图点不为坏点
                if (!pMP->isBad()) {
                    // 获取该地图点全部观测记录
                    const auto observations = pMP->GetObservations();
                    // 遍历观测，给对应关键帧投票计数
                    for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::const_iterator
                             it = observations.begin(),
                             itend = observations.end();
                         it != itend; it++)
                        keyframeCounter[it->first]++;
                } else {
                    // MODIFICATION
                    // 地图点损坏，置空上一帧该位置地图点指针
                    mLastFrame->mvpMapPoints[i] = NULL;
                }
            }
        }
    }

    // 记录最高投票数
    int max = 0;
    // 记录投票最多的关键帧，作为参考关键帧
    std::shared_ptr<KeyFrame> pKFmax;

    // 清空局部关键帧容器
    mvpLocalKeyFrames.clear();
    // 预分配容器空间，3倍投票关键帧数量，预留邻居、父子关键帧空间
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // 将所有被地图点观测的关键帧加入局部地图；同时统计共视最多的关键帧
    for (auto const& [pKF, count] : keyframeCounter) {
        // 跳过已经损坏的关键帧
        if (pKF->isBad()) continue;

        // 更新最大投票计数与对应关键帧
        if (count > max) {
            max = count;
            pKFmax = pKF;
        }

        // 加入局部关键帧列表
        mvpLocalKeyFrames.push_back(pKF);
        // 标记该关键帧被当前帧引用，避免本帧重复加入
        pKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
    }

    // 补充添加：已经加入局部地图的关键帧的邻居、子、父关键帧（还未加入的）
    for (auto const& pKF : mvpLocalKeyFrames) {
        // 局部关键帧总数超过80则停止扩充，控制局部地图规模
        if (mvpLocalKeyFrames.size() > 80) break;

        // 获取该关键帧共视程度最高的10个邻居关键帧
        const auto vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        // 遍历邻居关键帧
        for (auto pNeighKF : vNeighs) {
            if (!pNeighKF->isBad()) {
                // 判断该邻居关键帧还没有被当前帧标记引用
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame->mnId) {
                    // 加入局部关键帧
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
                    break;
                }
            }
        }

        // 获取该关键帧的子关键帧集合（生成树子节点）
        const set<std::shared_ptr<KeyFrame>> spChilds = pKF->GetChilds();
        for (auto pChildKF : spChilds) {
            if (!pChildKF->isBad()) {
                // 子关键帧未被当前帧引用，则加入局部关键帧
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame->mnId) {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
                    break;
                }
            }
        }

        // 获取该关键帧生成树的父关键帧
        auto pParent = pKF->GetParent();
        if (pParent) {
            // 父关键帧未被当前帧引用，则加入局部关键帧
            if (pParent->mnTrackReferenceForFrame != mCurrentFrame->mnId) {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrame->mnId;
                break;
            }
        }
    }

    // IMU模式下，局部关键帧不足80，补充最近时序上的历史关键帧
    if (mSensor.isImu() && mvpLocalKeyFrames.size() < 80) {
        // 从最近的上一个关键帧开始向前遍历
        std::shared_ptr<KeyFrame> tempKeyFrame = mCurrentFrame->mpLastKeyFrame;

        // 最多向前追溯20帧时序关键帧
        const int Nd = 20;
        for (int i = 0; i < Nd; i++) {
            // 为空直接退出循环
            if (!tempKeyFrame) break;
            // 如果该时序关键帧尚未被当前帧引用
            if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame->mnId) {
                // 添加到局部关键帧列表
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame->mnId;
                // 迭代到上一个时序关键帧
                tempKeyFrame = tempKeyFrame->mPrevKF;
            }
        }
    }

    // 如果找到了投票最多的关键帧，设置为当前帧的参考关键帧
    if (pKFmax) {
        mpReferenceKF = pKFmax;
        mCurrentFrame->mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization() {
    oslog::debug("[Tracking::Relocalization] Starting relocalization");
    // 计算当前帧的词袋向量，用于回环/重定位候选帧检索
    mCurrentFrame->ComputeBoW();

    // 跟踪丢失时执行重定位：向关键帧数据库查询和当前帧相似的候选关键帧
    auto vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(
        mCurrentFrame, mpAtlas->GetCurrentMap());

    // 没有检索到候选关键帧，重定位直接失败返回false
    if (vpCandidateKFs.empty()) {
        oslog::warn("[Tracking::Relocalization] There are no candidates keyframes");
        return false;
    }

    oslog::info("[Tracking::Relocalization] Found {} candidate KFs",
                vpCandidateKFs.size());

    // 对每一个候选关键帧做ORB词袋匹配；匹配数足够则构建ML‑PnP求解器
    ORBmatcher matcher(0.75, true);

    // 候选关键帧总数量
    const int nKFs = vpCandidateKFs.size();
    // 每个候选关键帧对应的ML‑PnP求解器实例
    vector<std::shared_ptr<MLPnPsolver>> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    // 每个候选关键帧与当前帧的地图点匹配对
    vector<vector<MapPoint*>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    // 标记对应候选关键帧是否被丢弃
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    // 经过词袋匹配后合格的候选数量
    int nCandidates = 0;

    // 遍历全部候选关键帧，做BoW匹配过滤
    for (int i = 0; i < nKFs; i++) {
        std::shared_ptr<KeyFrame> pKF = vpCandidateKFs[i];
        // 关键帧本身已经损坏，标记丢弃
        if (pKF->isBad()) {
            vbDiscarded[i] = true;
        } else {
            // 使用词袋做特征匹配，得到匹配点数量，结果存入vvpMapPointMatches[i]
            int nmatches =
                matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            oslog::info("[Tracking::Relocalization]   kf {};  {} matches", i,
                        nmatches);
            // 匹配数小于15，候选不达标，直接丢弃
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            } else {
                // 匹配充足，构造ML‑PnP求解器，输入当前帧和匹配的地图点
                std::shared_ptr<MLPnPsolver> pSolver =
                    std::make_shared<MLPnPsolver>(mCurrentFrame, vvpMapPointMatches[i]);
                // 设置RANSAC参数：置信度0.99，最小样本数10，最大迭代300，最小6点，重投影阈值0.5，卡方阈值5.991
                pSolver->SetRansacParameters(
                    0.99, 10, 300, 6, 0.5,
                    5.991);  // This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                // 合格候选计数+1
                nCandidates++;
            }
        }
    }

    oslog::info("[Tracking::Relocalization] ... {} candidates are plausible",
                nCandidates);

    // 交替执行ML‑PnP的RANSAC迭代，直到找到一个内点足够多的相机位姿
    bool bMatch = false;
    // 第二个匹配器，投影搜索模式，阈值更宽松
    ORBmatcher matcher2(0.9, true);

    // 还有可用候选并且还没有匹配成功
    while (nCandidates > 0 && !bMatch) {
        // 遍历所有候选关键帧
        for (int i = 0; i < nKFs; i++) {
            // 已经被丢弃，跳过
            if (vbDiscarded[i]) continue;

            // 执行5次RANSAC迭代
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            // 获取该候选对应的PnP求解器
            std::shared_ptr<MLPnPsolver>& pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            // 执行迭代求解位姿；bNoMore标记是否达到最大迭代次数；输出内点标记、内点数量、求解得到的位姿矩阵Tcw
            bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

            // 如果RANSAC耗尽迭代次数，丢弃该候选
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // 如果成功解算出相机位姿，进行位姿优化
            if (bTcw) {
                // Eigen矩阵包装为Sophus SE3位姿
                Sophus::SE3f Tcw(eigTcw);
                // 将求解得到的位姿设置给当前帧
                mCurrentFrame->SetPose(Tcw);
                // Tcw.copyTo(mCurrentFrame->mTcw);

                // 保存本次求解成功匹配到的地图点集合
                set<MapPoint*> sFound;

                // 内点标记数组总长度
                const int np = vbInliers.size();

                // 根据RANSAC内点结果填充当前帧的地图点指针
                for (int j = 0; j < np; j++) {
                    if (vbInliers[j]) {
                        // 内点：赋值匹配的地图点
                        mCurrentFrame->mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    } else {
                        // 外点：置空地图点
                        mCurrentFrame->mvpMapPoints[j] = NULL;
                    }
                }

                // 执行帧位姿优化，返回优化后有效内点数量
                int nGood = Optimizer::PoseOptimization(mCurrentFrame);

                // 优化后有效内点不足10，这个位姿不可信，跳过该候选
                if (nGood < 10) continue;

                // 将优化标记为外点的特征点，清空关联地图点
                for (size_t io = 0; io < mCurrentFrame->N; io++)
                    if (mCurrentFrame->mvbOutlier[io])
                        mCurrentFrame->mvpMapPoints[io] = static_cast<MapPoint*>(NULL);

                // 如果当前有效内点不足50，使用投影搜索在较大窗口寻找更多匹配点，再次优化
                if (nGood < 50) {
                    // 投影搜索，窗口10像素，最多找100个新增匹配
                    int nadditional = matcher2.SearchByProjection(
                        mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    // 原有+新增匹配达到50，再次做位姿优化
                    if (nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(mCurrentFrame);

                        // 内点30~50之间，再次缩小投影窗口继续找更多匹配
                        if (nGood > 30 && nGood < 50) {
                            // 清空已找到集合，重新收集当前帧现存地图点
                            sFound.clear();
                            for (size_t ip = 0; ip < mCurrentFrame->N; ip++) {
                                if (mCurrentFrame->mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame->mvpMapPoints[ip]);
                            }

                            // 缩小投影搜索窗口3像素，最多找64个新增匹配
                            nadditional = matcher2.SearchByProjection(
                                mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                            // 总和满足阈值，执行最终位姿优化
                            if (nGood + nadditional >= 50) {
                                nGood = Optimizer::PoseOptimization(mCurrentFrame);

                                // 再次清除优化标记的外点地图点
                                for (size_t io = 0; io < mCurrentFrame->N; io++) {
                                    if (mCurrentFrame->mvbOutlier[io])
                                        mCurrentFrame->mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }
                }

                // 有效内点达到50，重定位成功，退出循环
                if (nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    // 没有找到合格位姿，重定位失败
    if (!bMatch) {
        return false;
    } else {
        // 记录本次重定位成功的帧ID
        mnLastRelocFrameId = mCurrentFrame->mnId;
        oslog::warn("[Tracking::Relocalization] ... Successful relocalization!!");
        return true;
    }
}

void Tracking::Reset(bool bLocMap) {
    oslog::info("[Tracking::Reset] System resetting");

    // 如果可视化查看器存在，请求停止查看器线程，等待完全停止
    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped()) usleep(3000);
    }

    // bLocMap为false则重置局部建图模块
    if (!bLocMap) {
        oslog::debug("[Tracking::Reset] !! resetting Local Mapper...");
        mpLocalMapper->RequestReset();
    }

    // 请求重置回环检测线程
    oslog::debug("[Tracking::Reset] !! resetting Loop Closing...");
    mpLoopClosing->RequestReset();

    // 清空词袋关键帧数据库
    oslog::debug("[Tracking::Reset] !! resetting Database...");
    mpKeyFrameDB->clear();

    // 清空整个Atlas地图集，销毁所有地图点与关键帧
    mpAtlas->clearAtlas();
    // 创建全新的空地图
    mpAtlas->CreateNewMap();
    // IMU传感器模式，设置惯性传感器标记
    if (mSensor.isImu()) mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    // 重置关键帧、帧的全局ID计数器
    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    // 设置跟踪状态：还没有收到图像
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit = false;

    // 清空帧位姿、参考帧、时间戳、丢失标记的历史列表
    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    // 重置当前帧、上一帧为全新空Frame对象
    mCurrentFrame = std::make_shared<Frame>();
    mnLastRelocFrameId = 0;
    mLastFrame = std::make_shared<Frame>();
    // 重置参考关键帧、上一个关键帧智能指针
    mpReferenceKF.reset();
    mpLastKeyFrame.reset();
    // 清空初始化阶段匹配
    mvIniMatches.clear();

    // 释放查看器资源
    if (mpViewer) mpViewer->Release();

    oslog::info("[Tracking::Reset] !! End resetting!");
}

void Tracking::ResetActiveMap(bool bLocMap) {
    oslog::info("[Tracking::ResetActiveMap] !! Active map resetting");
    // 如果可视化模块存在，请求停止并等待停止
    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped()) usleep(3000);
    }

    // 获取当前正在使用的地图
    std::shared_ptr<Map> pMap(mpAtlas->GetCurrentMap());

    // bLocMap为false，重置局部建图，传入当前活跃地图
    if (!bLocMap) {
        oslog::info("[Tracking::ResetActiveMap] !! resetting Local Mapper...");
        mpLocalMapper->RequestResetActiveMap(pMap);
    }

    // 请求回环检测针对当前活跃地图做重置
    oslog::info("[Tracking::ResetActiveMap] !! resetting Loop Closing...");
    mpLoopClosing->RequestResetActiveMap(pMap);

    // 词袋数据库只清除当前活跃地图相关的记录，保留其他地图
    oslog::info("[Tracking::ResetActiveMap] !! resetting Database");
    mpKeyFrameDB->clearMap(pMap);  // Only clear the active map references

    // 清空当前活跃地图内部所有关键帧、地图点
    mpAtlas->clearMap();

    // KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    // Frame::nNextId = mnLastInitFrameId;
    // 记录重置瞬间帧ID，作为后续新地图起始ID参考
    mnLastInitFrameId = Frame::nNextId;
    // mnLastRelocFrameId = mnLastInitFrameId;
    // 跟踪状态置为还没有图像输入
    mState = NO_IMAGES_YET;  // NOT_INITIALIZED;

    mbReadyToInitializate = false;

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    // 遍历Atlas全部地图，拿到所有地图中最小关键帧ID
    for (auto const& pMap : mpAtlas->GetAllMaps()) {
        if (pMap->GetAllKeyFrames().size() > 0) {
            if (index > pMap->GetLowerKFID()) index = pMap->GetLowerKFID();
        }
    }

    int num_lost = 0;

    // 重建帧丢失标记列表：旧地图的帧标记为丢失，新地图起始帧保留原有状态
    for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end();
         ilbL++) {
        if (index < mnInitialFrameId) {
            lbLost.push_back(*ilbL);
        } else {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    oslog::warn("[Tracking::ResetActiveMap] {} frames set to lost", num_lost);

    // 将新构建的丢失标记列表替换旧列表
    mlbLost = lbLost;

    // 更新初始帧ID、上一次重定位帧ID为当前帧ID
    mnInitialFrameId = mCurrentFrame->mnId;
    mnLastRelocFrameId = mCurrentFrame->mnId;

    // 重置当前帧、上一帧为全新空帧对象
    mCurrentFrame = std::make_shared<Frame>();
    mLastFrame = std::make_shared<Frame>();
    // 清空参考关键帧、上一关键帧
    mpReferenceKF.reset();
    mpLastKeyFrame.reset();
    // 清空初始化匹配
    mvIniMatches.clear();

    mbVelocity = false;

    // 释放可视化模块资源
    if (mpViewer) mpViewer->Release();

    oslog::warn("[Tracking::ResetActiveMap] !! End resetting!");
}

vector<MapPoint*> Tracking::GetLocalMapMPS() { return mvpLocalMapPoints; }

void Tracking::ChangeCalibration(const string& strSettingPath) {
    // 读取yaml配置文件
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    // 读取相机内参
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    // Eigen形式内参矩阵赋值
    mK_.setIdentity();
    mK_(0, 0) = fx;
    mK_(1, 1) = fy;
    mK_(0, 2) = cx;
    mK_(1, 2) = cy;

    // OpenCV Mat形式内参矩阵赋值
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    // 读取畸变系数k1,k2,p1,p2
    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    // 如果存在k3，则畸变矩阵扩充为5维
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    // 读取双目基线*fx参数
    mbf = fSettings["Camera.bf"];

    // 标记Frame需要重新执行初始化计算
    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool& flag) { mbOnlyTracking = flag; }

void Tracking::UpdateFrameIMU(
    const float s, const IMU::Bias& b,
    const std::shared_ptr<KeyFrame>& pCurrentKeyFrame) {
    // 获取当前关键帧所属地图
    std::shared_ptr<Map> pMap = pCurrentKeyFrame->GetMap();
    // unsigned int index = mnFirstFrameId;
    // 获取相对位姿列表迭代器、参考关键帧迭代器、帧丢失标记迭代器
    list<std::shared_ptr<KeyFrame>>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();

    // 遍历所有历史相对帧位姿，对属于当前地图的位姿做尺度缩放s
    for (auto lit = mlRelativeFramePoses.begin(),
             lend = mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lbL++) {
        // 该帧标记为丢失，跳过处理
        if (*lbL) continue;

        std::shared_ptr<KeyFrame> pKF = *lRit;

        // 如果参考关键帧是坏点，沿着生成树向上找父关键帧直到有效
        while (pKF->isBad()) {
            pKF = pKF->GetParent();
        }

        // 如果该参考关键帧属于当前地图，平移部分乘以尺度因子s
        if (pKF->GetMap() == pMap) {
            (*lit).translation() *= s;
        }
    }

    // 更新保存最新IMU偏置
    mLastBias = b;

    // 更新上一个关键帧指针
    mpLastKeyFrame = pCurrentKeyFrame;
    // 将最新IMU偏置设置给上一帧、当前帧
    mLastFrame->SetNewBias(mLastBias);
    mCurrentFrame->SetNewBias(mLastBias);

    // 等待当前帧IMU预积分完成，自旋等待
    while (!mCurrentFrame->imuIsPreintegrated()) {
        usleep(500);
    }

    // 判断上一帧是否就是上一个关键帧
    if (mLastFrame->mnId == mLastFrame->mpLastKeyFrame->mnFrameId) {
        // 直接复用关键帧IMU姿态、位置、速度赋值给上一帧
        mLastFrame->SetImuPoseVelocity(mLastFrame->mpLastKeyFrame->GetImuRotation(),
                                       mLastFrame->mpLastKeyFrame->GetImuPosition(),
                                       mLastFrame->mpLastKeyFrame->GetVelocity());
    } else {
        // 上一帧不是关键帧，通过IMU预积分推算上一帧IMU姿态、位置、速度
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = mLastFrame->mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame->mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame->mpLastKeyFrame->GetVelocity();
        // 预积分时间间隔
        float t12 = mLastFrame->mpImuPreintegrated->dT;

        // 旋转：上一KF旋转 * 预积分更新后的delta旋转，做归一化保证旋转矩阵正交
        // 位置：位置 + 速度*dt + 0.5*g*dt² + R*预积分delta位置
        // 速度：速度 + g*dt + R*预积分delta速度
        mLastFrame->SetImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 * mLastFrame->mpImuPreintegrated->GetUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 * mLastFrame->mpImuPreintegrated->GetUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 +
                Rwb1 * mLastFrame->mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    // 当前帧存在IMU预积分，推算当前帧IMU状态
    if (mCurrentFrame->mpImuPreintegrated) {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 =
            mCurrentFrame->mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 =
            mCurrentFrame->mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mCurrentFrame->mpLastKeyFrame->GetVelocity();
        float t12 = mCurrentFrame->mpImuPreintegrated->dT;

        // 使用IMU预积分结果推算当前帧IMU姿态、位置、速度
        mCurrentFrame->SetImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 *
                mCurrentFrame->mpImuPreintegrated->GetUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 * mCurrentFrame->mpImuPreintegrated->GetUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 +
                Rwb1 *
                    mCurrentFrame->mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    // 记录第一个有效IMU帧的ID
    mnFirstImuFrameId = mCurrentFrame->mnId;
}

void Tracking::NewDataset() { mnNumDataset++; }

int Tracking::GetNumberDataset() { return mnNumDataset; }

int Tracking::GetMatchesInliers() { return mnMatchesInliers; }

void Tracking::SaveSubTrajectory(string strNameFile_frames,
                                  string strNameFile_kf, string strFolder) {
    // 保存普通帧EuRoC格式轨迹，拼接文件夹路径
    mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
    // mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames,
                                  string strNameFile_kf,
                                  const std::shared_ptr<Map>& pMap) {
    // 保存指定地图的普通帧轨迹
    mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
    // 如果关键帧文件名非空，保存该地图关键帧轨迹
    if (!strNameFile_kf.empty())
        mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::GetImageScale() { return mImageScale; }

#ifdef REGISTER_LOOP
void Tracking::RequestStop() {
    // 获取停止互斥锁
    unique_lock<mutex> lock(mMutexStop);
    // 设置停止请求标记
    mbStopRequested = true;
}

bool Tracking::Stop() {
    unique_lock<mutex> lock(mMutexStop);
    // 收到停止请求，并且没有禁止停止标记，则置为已停止，返回true
    if (mbStopRequested && !mbNotStop) {
        mbStopped = true;
        oslog::info("Tracking STOP");
        return true;
    }

    return false;
}

bool Tracking::stopRequested() {
    unique_lock<mutex> lock(mMutexStop);
    // 返回是否收到停止请求
    return mbStopRequested;
}

bool Tracking::isStopped() {
    unique_lock<mutex> lock(mMutexStop);
    // 返回跟踪线程是否已经处于停止状态
    return mbStopped;
}

void Tracking::Release() {
    unique_lock<mutex> lock(mMutexStop);
    // 清除停止标记，恢复运行
    mbStopped = false;
    mbStopRequested = false;
}
#endif

}  // namespace ORB_SLAM3

