/**
 * This file is part of ORB‑SLAM3
 *
 * Copyright (C) 2017‑2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014‑2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB‑SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB‑SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB‑SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */
#include "System.h"
#include <pangolin/pangolin.h>
#include <algorithm>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Converter.h"

namespace ORB_SLAM3 {

// 全局日志输出等级，默认NORMAL级别
Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

/**
 * @brief System构造函数，SLAM系统顶层入口，只做成员初始化，真正初始化在initialize函数
 * @param settings 配置参数智能指针，包含相机、传感器、路径、开关等全部配置
 * @param initFr 是否启用第一帧IMU初始化，针对IMU模式
 * @param strSequence 数据集序列名称，用于日志、保存统计信息
 */
System::System(const std::shared_ptr<Settings> &settings, bool initFr,
               const string &strSequence)
    : enable_shared_from_this<System>(),
      mpViewer(),                     // 可视化Viewer智能指针，初始为空
      mbReset(false),                 // 全局地图复位请求标记
      mbResetActiveMap(false),        // 当前活跃子地图复位请求标记
      mbActivateLocalizationMode(false),  // 开启纯定位模式请求标记
      mbDeactivateLocalizationMode(false),// 关闭纯定位模式请求标记
      mbShutDown(false),              // 系统关闭标记
      settings_(settings)             // 保存配置对象
{
    // 打印版权banner信息
    printBanner();
}

/**
 * @brief 打印ORB‑SLAM3版权、开源协议欢迎信息
 */
void System::printBanner() {
    // Output welcome message
    cout << endl
         << "ORB‑SLAM3 Copyright (C) 2017‑2020 Carlos Campos, Richard Elvira, "
            "Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of "
            "Zaragoza."
         << endl
         << "ORB‑SLAM2 Copyright (C) 2014‑2016 Raúl Mur‑Artal, José M.M. Montiel "
            "and Juan D. Tardós, University of Zaragoza."
         << endl
         << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
         << "This is free software, and you are welcome to redistribute it"
         << endl
         << "under certain conditions. See LICENSE.txt." << endl
         << endl;

    // 日志输出当前传感器类型（单目/双目/RGBD/带IMU版本）
    oslog::info("Input sensor is type {}", sensorType().toString());
}

/**
 * @brief SLAM系统真正初始化函数：加载词袋、构建Atlas、启动Tracking/LocalMapping/LoopClosing/Viewer线程
 * @param initFr IMU模式下是否使用第一帧做初始化
 * @param strSequence 数据集序列名字
 * @return true初始化成功，false失败（词袋加载失败、地图文件读取失败）
 */
bool System::initialize(bool initFr, const string &strSequence) {
    // 从配置读取Atlas地图加载文件路径，为空代表不从文件加载，新建地图
    const string mStrLoadAtlasFromFile = settings_->atlasLoadFile();

    // 打印全部配置参数
    cout << (*settings_) << endl;

    // 是否开启回环检测
    const bool activeLC = settings_->loopClosing_;
    // ORB词袋txt文件路径
    const string vocabularyFilePath = settings_->strVocFile_;

    // Load ORB Vocabulary
    oslog::info("Loading ORB Vocabulary. This could take a while...");

    // 创建ORB词袋对象
    mpVocabulary = std::make_shared<ORBVocabulary>();
    // 从文本文件加载词袋
    bool bVocLoad = mpVocabulary->loadFromTextFile(vocabularyFilePath);
    if (!bVocLoad) {
        oslog::error("Wrong path to vocabulary.  Failed to open {}",
                     vocabularyFilePath);
        return false;
    }
    oslog::info("Vocabulary loaded!");

    // Create KeyFrame Database，关键帧数据库，用于回环、重定位词袋检索
    mpKeyFrameDatabase = std::make_shared<KeyFrameDatabase>(mpVocabulary);

    if (mStrLoadAtlasFromFile.empty()) {
        // Create the Atlas，路径为空：全新初始化Atlas多地图管理器
        oslog::info("Initializing Atlas from scratch ");
        mpAtlas = std::make_shared<Atlas>(0);
    } else {
        // Load the file with an earlier session，从二进制文件加载历史保存的Atlas地图
        // clock_t start = clock();
        oslog::info("Initializing Atlas from file: {}", mStrLoadAtlasFromFile);
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if (!isRead) {
            oslog::error(
                "Unable to load Atlas file, please try with other session file or "
                "vocabulary file");
            return false;
        }

        // 加载完成后新建一个子Map，用于后续运行
        mpAtlas->CreateNewMap();
    }

    // 如果传感器带IMU，Atlas设置惯性传感器标记
    if (sensorType().isImu()) mpAtlas->SetInertialSensor();

    // Only draw right image in stereo modes，双目模式才绘制右图
    const bool frame_drawer_both = sensorType().isStereo();

    // Create Drawers. These are used by the Viewer，图像帧绘制器、地图绘制器，供可视化线程使用
    mpFrameDrawer = std::make_shared<FrameDrawer>(mpAtlas, frame_drawer_both);
    mpMapDrawer = std::make_shared<MapDrawer>(mpAtlas, settings_);

    // Initialize the Tracking thread
    // (it will live in the main thread of execution, the one that called this
    // constructor)
    oslog::info("Seq. Name: {}", strSequence);
    // Tracking运行在主线程，不新建线程；每一帧调用TrackXXX函数执行跟踪逻辑
    mpTracker = std::make_shared<Tracking>(
        shared_from_this(), mpVocabulary, mpFrameDrawer, mpMapDrawer, mpAtlas,
        mpKeyFrameDatabase, settings_, strSequence);

    // Initialize the Local Mapping thread and launch，局部建图对象，处理关键帧、新增地图点、局部BA
    mpLocalMapper = std::make_shared<LocalMapping>(
        shared_from_this(), mpAtlas, sensorType().isMonocular(),
        sensorType().isImu(), strSequence);
    mpLocalMapper->mInitFr = initFr;
    // 读取远地点阈值
    mpLocalMapper->mThFarPoints = settings_->thFarPoints();

    // 如果阈值非0，开启过滤远地点功能
    if (mpLocalMapper->mThFarPoints != 0) {
        oslog::info(
            "LocalMapping will discard points further than {} m from current "
            "camera",
            mpLocalMapper->mThFarPoints);
        mpLocalMapper->mbFarPoints = true;
    } else {
        oslog::info("LocalMapping will _not_ discard far points");
        mpLocalMapper->mbFarPoints = false;
    }

    // 启动LocalMapping后台线程，执行LocalMapping::Run循环
    mptLocalMapping =
        std::make_unique<thread>(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);

    // Initialize the Loop Closing thread and launch，回环检测线程，回环检测、Sim3求解、全局BA
    mpLoopCloser = std::make_shared<LoopClosing>(
        mpAtlas, mpKeyFrameDatabase, mpVocabulary,
        sensorType() != SensorType::MONOCULAR, activeLC);
    mptLoopClosing =
        std::make_unique<thread>(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    // Set pointers between threads，各个模块互相设置指针，实现模块之间互相调用
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // Initialize the Viewer thread and launch，可视化线程
    oslog::info("Viewer enabled: {}", settings_->useViewer_ ? "YES" : "NO");
    if (settings_->useViewer_) {
        mpViewer = std::make_shared<Viewer>(this, mpFrameDrawer, mpMapDrawer,
                                            mpTracker, settings_);
        // 启动Viewer线程
        mptViewer = std::make_unique<thread>(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    // Fix verbosity，设置日志输出等级DEBUG
    Verbose::SetTh(Verbose::VERBOSITY_DEBUG);

    return true;
}

/**
 * @brief 双目/双目IMU模式跟踪接口，输入左右图像、时间戳、IMU测量，返回相机位姿Tcw
 * @param imLeft 左目输入图像
 * @param imRight 右目输入图像
 * @param timestamp 图像时间戳
 * @param vImuMeas 该帧之前累积的IMU测量数据，IMU模式传入
 * @param filename 文件名，用于日志记录
 * @return Sophus::SE3f Tcw：世界到相机的变换
 */
Sophus::SE3f System::TrackStereo(cv::InputArray imLeft, cv::InputArray imRight,
                                 double timestamp,
                                 const vector<IMU::Point> &vImuMeas,
                                 string filename) {
    // 校验传感器类型，如果不是双目直接退出
    if (!sensorType().isStereo()) {
        oslog::error(
            "ERROR: you called TrackStereo but input sensor was not set to "
            "Stereo nor Stereo‑Inertial.");
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    // 判断是否需要立体校正，执行remap校正
    if (settings_ && settings_->needToRectify()) {
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft.getMat(), imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight.getMat(), imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    } else if (settings_ && settings_->needToResize()) {
        // 需要缩放图像，缩放到配置指定尺寸
        cv::resize(imLeft.getMat(), imLeftToFeed, settings_->newImSize());
        cv::resize(imRight.getMat(), imRightToFeed, settings_->newImSize());
    } else {
        // 无需校正缩放，clone原图
        imLeftToFeed = imLeft.getMat().clone();
        imRightToFeed = imRight.getMat().clone();
    }

    // 处理纯定位模式切换请求
    processLocalizationModeChange();
    // 处理复位请求
    processReset();

    // 送入时间戳，估算当前FPS
    fps_estimator_.pushTimestamp(timestamp);
    oslog::debug("[System] Current FPS estimate {:2f}", fps_estimator_.fps());

    // IMU模式，把IMU测量数据喂给Tracker
    if (sensorType().isImu()) {
        for (auto const &imuMeas : vImuMeas) {
            mpTracker->GrabImuData(imuMeas);
        }
    }

    // 调用Tracker双目图像跟踪入口
    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed,
                                                  timestamp, filename);

    // 更新系统层保存的跟踪状态、地图点、特征点
    updateTrackingState();

    return Tcw;
}

/**
 * @brief RGBD/RGBD‑Inertial模式跟踪接口，输入彩色图、深度图、时间戳、IMU数据
 * @param im 彩色输入图像
 * @param depthmap 深度图
 * @param timestamp 图像时间戳
 * @param vImuMeas IMU测量序列
 * @param filename 文件名用于日志
 * @return Tcw世界到相机位姿
 */
Sophus::SE3f System::TrackRGBD(cv::InputArray im, cv::InputArray depthmap,
                               double timestamp,
                               const vector<IMU::Point> &vImuMeas,
                               string filename) {
    // 校验传感器类型，非RGBD直接退出
    if (!sensorType().isRGBD()) {
        oslog::error(
            "ERROR: you called TrackRGBD but input sensor was not set to RGBD.");
        exit(-1);
    }

    cv::Mat imToFeed;
    cv::Mat imDepthToFeed;
    // 是否需要缩放图像
    if (settings_ && settings_->needToResize()) {
        cv::resize(im.getMat(), imToFeed, settings_->newImSize());
        cv::resize(depthmap.getMat(), imDepthToFeed, settings_->newImSize());
    } else {
        imToFeed = im.getMat().clone();
        imDepthToFeed = depthmap.getMat().clone();
    }

    // 处理定位模式切换、复位请求
    processLocalizationModeChange();
    processReset();

    fps_estimator_.pushTimestamp(timestamp);

    // IMU模式喂入IMU数据
    if (sensorType().isImu()) {
        for (auto const &imuMeas : vImuMeas) {
            mpTracker->GrabImuData(imuMeas);
        }
    }

    // Tracker RGBD跟踪入口
    Sophus::SE3f Tcw =
        mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);

    updateTrackingState();
    return Tcw;
}

/**
 * @brief 单目/单目IMU跟踪接口
 * @param im 输入图像
 * @param timestamp 时间戳
 * @param vImuMeas IMU测量数据
 * @param filename 文件名字符串
 * @return Tcw世界到相机位姿
 */
Sophus::SE3f System::TrackMonocular(cv::InputArray im, double timestamp,
                                    const vector<IMU::Point> &vImuMeas,
                                    string filename) {
    {
        unique_lock<mutex> lock(mMutexReset);
        // 如果系统已经shutdown，返回空位姿
        if (mbShutDown) return Sophus::SE3f();
    }

    // 校验传感器类型
    if (!sensorType().isMonocular()) {
        oslog::error(
            "ERROR: you called TrackMonocular but input sensor was not set to "
            "Monocular nor Monocular‑Inertial.");
        exit(-1);
    }

    cv::Mat imToFeed;
    // 判断是否缩放图像
    if (settings_ && settings_->needToResize()) {
        cv::Mat resizedIm;
        cv::resize(im.getMat(), resizedIm, settings_->newImSize());
        imToFeed = resizedIm;
    } else {
        imToFeed = im.getMat().clone();
    }

    // 处理模式切换、复位
    processLocalizationModeChange();
    processReset();

    fps_estimator_.pushTimestamp(timestamp);

    // IMU模式送入IMU测量
    if (sensorType().isImu()) {
        for (auto const &imuMeas : vImuMeas) {
            mpTracker->GrabImuData(imuMeas);
        }
    }

    // 调用Tracker单目跟踪
    Sophus::SE3f Tcw =
        mpTracker->GrabImageMonocular(imToFeed, timestamp, filename);

    updateTrackingState();
    return Tcw;
}

//===============================================

/**
 * @brief 请求开启纯定位模式：停止局部建图，只做跟踪，不新增地图点与关键帧
 */
void System::ActivateLocalizationMode() {
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

/**
 * @brief 请求关闭纯定位模式，恢复完整SLAM模式，重新开启局部建图
 */
void System::DeactivateLocalizationMode() {
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

/**
 * @brief 实际执行定位模式切换，每帧TrackXXX内部调用，处理请求标记
 */
void System::processLocalizationModeChange() {
    unique_lock<mutex> lock(mMutexMode);
    // 处理开启纯定位请求
    if (mbActivateLocalizationMode) {
        // 请求LocalMapping线程停止
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped，轮询等待线程真正停止
        while (!mpLocalMapper->isStopped()) {
            usleep(1000);
        }

        // 设置Tracker只做跟踪，不生成新关键帧
        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
    }
    // 处理关闭纯定位请求
    if (mbDeactivateLocalizationMode) {
        mpTracker->InformOnlyTracking(false);
        // 释放LocalMapping，恢复运行
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
    }
}

//===============================================

/**
 * @brief 请求系统全局复位，清空全部地图，重新初始化
 */
void System::Reset() {
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

/**
 * @brief 请求复位当前活跃子地图，其他子地图保留，Atlas多地图模式使用
 */
void System::ResetActiveMap() {
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

/**
 * @brief 执行复位操作，每帧TrackXXX内部调用，消费复位标记
 */
void System::processReset() {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
        mpTracker->Reset();
        mbReset = false;
        mbResetActiveMap = false;
    } else if (mbResetActiveMap) {
        mpTracker->ResetActiveMap();
        mbResetActiveMap = false;
    }
}

//===============================================

/**
 * @brief 更新系统层保存的跟踪状态、当前帧地图点、去畸变关键点，加锁保护
 */
void System::updateTrackingState() {
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
}

//===============================================

/**
 * @brief 判断Atlas地图是否发生重大变更（新建地图、地图复位等）
 * @return true发生重大变更；false无变更
 */
bool System::MapChanged() {
    static int n = 0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if (n < curn) {
        n = curn;
        return true;
    } else {
        return false;
    }
}

/**
 * @brief 系统关闭函数，优雅退出各个后台线程，可选保存Atlas地图文件
 */
void System::Shutdown() {
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    oslog::warn("Shutdown");

    // 请求局部建图、回环线程结束循环
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    /*if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }*/

    // Wait until all thread have effectively stopped，等待线程真正退出，注意等待GBA全局BA结束
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
           mpLoopCloser->isRunningGBA()) {
        if (!mpLocalMapper->isFinished()) {
            oslog::warn("mpLocalMapper is not finished");
        }

        if (!mpLoopCloser->isFinished()) {
            oslog::warn("mpLoopCloser is not finished");
        }

        if (mpLoopCloser->isRunningGBA()) {
            oslog::warn("mpLoopCloser is running GBA");
        }

        oslog::warn(" .... waiting");
        usleep(5000);
    }

    oslog::warn("All threads finished");

    // 如果配置设置了保存Atlas路径，二进制序列化保存地图
    const string mStrSaveAtlasToFile = settings_->atlasSaveFile();
    if (!mStrSaveAtlasToFile.empty()) {
        Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile,
                           Verbose::VERBOSITY_NORMAL);
        SaveAtlas(FileType::BINARY_FILE);
    }

    if (mpViewer) pangolin::BindToContext("ORB‑SLAM3: Map Viewer");
#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif
}

/**
 * @brief 查询系统是否已经调用Shutdown关闭
 * @return true已经关闭
 */
bool System::isShutDown() {
    unique_lock<mutex> lock(mMutexReset);
    return mbShutDown;
}

/**
 * @brief 获取跟踪状态：OK / LOST / NOT_INITIALIZED等
 * @return int 对应Tracking枚举值
 */
int System::GetTrackingState() {
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

/**
 * @brief 获取当前帧跟踪到的地图点
 * @return vector<MapPoint*> 地图点数组
 */
vector<MapPoint *> System::GetTrackedMapPoints() {
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

/**
 * @brief 获取当前帧去畸变关键点
 * @return vector<cv::KeyPoint>
 */
vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

/**
 * @brief 获取IMU初始化完成后流逝的时间，IMU模式使用
 * @return double 时间，未初始化返回0
 */
double System::GetTimeFromIMUInit() {
    double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    if ((aux > 0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

/**
 * @brief 判断系统是否丢失跟踪，IMU初始化完成之后才有效
 * @return true跟踪丢失
 */
bool System::isLost() {
    if (!mpAtlas->isImuInitialized()) {
        return false;
    } else {
        if ((mpTracker->mState ==
             Tracking::LOST))  // ||(mpTracker‑>mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}

/**
 * @brief 判断IMU初始化是否完成（简单通过时间是否大于0.1s判断）
 * @return true IMU初始化完成
 */
bool System::isFinished() { return (GetTimeFromIMUInit() > 0.1); }

/**
 * @brief 切换数据集，多地图模式；当前地图关键帧少则重置当前地图，否则新建子地图
 */
void System::ChangeDataset() {
    if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12) {
        mpTracker->ResetActiveMap();
    } else {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

/**
 * @brief 获取图像缩放比例
 * @return float scale
 */
float System::GetImageScale() { return mpTracker->GetImageScale(); }

#ifdef REGISTER_TIMES
/**
 * @brief 记录双目校正耗时，调试统计时间用
 */
void System::InsertRectTime(double &time) {
    mpTracker->vdRectStereo_ms.push_back(time);
}

/**
 * @brief 记录图像resize耗时
 */
void System::InsertResizeTime(double &time) {
    mpTracker->vdResizeImage_ms.push_back(time);
}

/**
 * @brief 记录完整跟踪耗时
 */
void System::InsertTrackTime(double &time) {
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

}  // namespace ORB_SLAM3
