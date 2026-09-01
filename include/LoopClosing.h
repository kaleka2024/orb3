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
#pragma once // 头文件保护，防止头文件重复包含导致编译重定义
#include <g2o/types/sim3/types_seven_dof_expmap.h> // g2o的Sim3(7自由度)位姿图优化类型，用于回环的相似变换
#include <boost/algorithm/string.hpp>             // boost字符串工具库
#include <functional>                              // std::function函数对象
#include <list>                                    // std::list双向链表，存储待处理回环关键帧队列
#include <memory>                                  // std::shared_ptr智能指针
#include <mutex>                                   // std::mutex互斥锁，多线程共享数据同步
#include <set>                                     // std::set有序集合，存储关键帧、地图点集合
#include <string>                                  // std::string字符串，调试输出路径
#include <thread>                                  // std::thread线程，用于全局BA子线程
#include <utility>                                 // std::pair等工具
#include <vector>                                  // std::vector动态数组容器

#include "Atlas.h"               // 多地图集Atlas，管理所有子地图
#include "KeyFrame.h"            // 关键帧类定义
#include "KeyFrameDatabase.h"    // 关键帧词袋数据库，检索回环/融合候选帧
#include "LocalMapping.h"        // 局部建图模块，回环需要与其交互、中断局部BA
#include "ORBVocabulary.h"       // ORB词袋字典
#include "Tracking.h"            // 跟踪线程模块

namespace ORB_SLAM3 {

class Tracking;        // 跟踪线程前向声明
class LocalMapping;    // 局部建图线程前向声明
class KeyFrameDatabase;// 关键帧数据库前向声明
class Map;             // 子地图类前向声明

/**
 * @brief 回环闭合线程类，继承enable_shared_from_this支持this的shared_ptr获取
 * @details ORB‑SLAM3三大核心线程之一；负责回环检测、地图融合(Merge)、Sim3求解、回环校正、全局GBA；支持多地图Atlas模式
 */
class LoopClosing : public enable_shared_from_this<LoopClosing>
{
public:
    /// @brief 一致性组：一组具备共视一致性的候选关键帧集合 + 一致性计数
    typedef pair<set<std::shared_ptr<KeyFrame> >, int> ConsistentGroup;
    /**
     * @brief 关键帧‑Sim3位姿映射表
     * key：关键帧共享指针；value：g2o::Sim3相似变换；使用Eigen对齐分配器，适配Sim3内部Eigen成员内存对齐
     */
    typedef map<std::shared_ptr<KeyFrame>, g2o::Sim3,
                std::less<std::shared_ptr<KeyFrame> >,
                Eigen::aligned_allocator<
                    std::pair<const std::shared_ptr<KeyFrame>, g2o::Sim3> > >
        KeyFrameAndPose;

public:
    LoopClosing() = delete;                  // 删除默认构造，禁止无参构造实例
    LoopClosing(const LoopClosing &) = delete;// 删除拷贝构造，禁止拷贝对象

    /**
     * @brief 回环闭合模块构造函数
     * @param pAtlas 全局地图集Atlas指针
     * @param pDB 关键帧词袋数据库指针
     * @param pVoc ORB词袋字典指针
     * @param bFixScale true固定尺度(双目/RGBD)；false尺度可变(单目+IMU)
     * @param bActiveLC 是否启用回环检测功能
     */
    LoopClosing(const std::shared_ptr<Atlas> &pAtlas,
                const std::shared_ptr<KeyFrameDatabase> &pDB,
                const std::shared_ptr<ORBVocabulary> &pVoc, const bool bFixScale,
                const bool bActiveLC);

    /**
     * @brief 设置跟踪模块指针，回环校正后需要同步更新Tracking状态
     * @param pTracker Tracking共享指针
     */
    void SetTracker(const std::shared_ptr<Tracking> &pTracker);

    /**
     * @brief 设置局部建图模块指针，回环触发时中断局部BA、交互关键帧
     * @param pLocalMapper LocalMapping共享指针
     */
    void SetLocalMapper(const std::shared_ptr<LocalMapping> &pLocalMapper);

    // Main function 线程主函数，LoopClosing线程启动后循环运行该函数
    void Run();

    /**
     * @brief 插入新关键帧到回环待处理队列，由LocalMapping调用
     * @param pKF 新传入的关键帧智能指针
     */
    void InsertKeyFrame(const std::shared_ptr<KeyFrame> &pKF);

    /**
     * @brief 请求全局重置，清空全部地图与回环状态
     */
    void RequestReset();

    /**
     * @brief 请求仅重置当前激活地图，多地图Atlas模式使用
     * @param pMap 需要重置的目标地图
     */
    void RequestResetActiveMap(const std::shared_ptr<Map> &pMap);

    /**
     * @brief 在独立子线程中执行全局光束平差GBA
     * @param pActiveMap 需要做全局BA的激活地图
     * @param nLoopKF 触发本次GBA的回环关键帧ID
     * @note 该函数运行在单独GBA子线程
     */
    void RunGlobalBundleAdjustment(const std::shared_ptr<Map> &pActiveMap,
                                   unsigned long nLoopKF);

    /**
     * @brief 查询是否正在执行全局GBA优化
     * @return true GBA正在运行
     */
    bool isRunningGBA()
    {
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    /**
     * @brief 查询全局GBA是否已经执行完毕
     * @return true GBA已完成
     */
    bool isFinishedGBA()
    {
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }

    /**
     * @brief 请求回环线程正常结束，系统退出时调用
     */
    void RequestFinish();

    /**
     * @brief 查询回环线程是否已经结束退出
     * @return true线程已结束
     */
    bool isFinished();

    // \amm Unused? 视图器指针，注释标记为未使用
    std::shared_ptr<Viewer> mpViewer;

#ifdef REGISTER_TIMES // 编译宏开关，开启各步骤性能耗时统计
    vector<double> vdDataQuery_ms;    // 词袋查询候选耗时(ms)
    vector<double> vdEstSim3_ms;      // Sim3相似变换估计耗时(ms)
    vector<double> vdPRTotal_ms;      // 地点识别总耗时(ms)

    vector<double> vdMergeMaps_ms;    // 地图融合整体耗时(ms)
    vector<double> vdWeldingBA_ms;    // 融合拼接BA耗时(ms)
    vector<double> vdMergeOptEss_ms;  // 融合本质图优化耗时(ms)
    vector<double> vdMergeTotal_ms;   // 单次地图融合总耗时(ms)
    vector<int> vnMergeKFs;           // 融合涉及关键帧数量
    vector<int> vnMergeMPs;           // 融合涉及地图点数量
    int nMerges;                      // 地图融合执行总次数

    vector<double> vdLoopFusion_ms;   // 回环地图点融合耗时(ms)
    vector<double> vdLoopOptEss_ms;   // 回环本质图优化耗时(ms)
    vector<double> vdLoopTotal_ms;    // 单次回环校正总耗时(ms)
    vector<int> vnLoopKFs;            // 回环涉及关键帧数目
    int nLoop;                        // 回环检测成功总次数

    vector<double> vdGBA_ms;          // 全局BA优化耗时(ms)
    vector<double> vdUpdateMap_ms;    // GBA后地图更新耗时(ms)
    vector<double> vdFGBATotal_ms;    // 完整GBA流程总耗时(ms)
    vector<int> vnGBAKFs;             // GBA参与关键帧数量
    vector<int> vnGBAMPs;             // GBA参与地图点数量
    int nFGBA_exec;                   // GBA执行总次数
    int nFGBA_abort;                  // GBA被中断放弃次数
#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Eigen内存对齐宏，类存在Eigen/g2o Sim3成员必须添加
protected:
    /**
     * @brief 检查回环待处理队列是否存在新关键帧
     * @return true队列有新关键帧；false队列为空
     */
    bool CheckNewKeyFrames();

    // Methods to implement the new place recognition algorithm 新版地点识别算法接口
    /**
     * @brief 核心：检测公共区域，区分回环检测或者多地图融合Merge
     * @return true检测到回环/融合候选；false未检测到
     */
    bool NewDetectCommonRegions();

    /**
     * @brief 从上一帧关键帧出发，检测并精修Sim3相似变换
     * @param pCurrentKF 当前查询关键帧
     * @param pMatchedKF 输出匹配到的回环/融合关键帧
     * @param gScw 输出世界到当前帧Sim3变换
     * @param nNumProjMatches 输出投影匹配得到的匹配点数量
     * @param vpMPs 输入地图点集合
     * @param vpMatchedMPs 输出匹配成功的地图点集合
     * @return true求解Sim3成功；false失败
     */
    bool DetectAndReffineSim3FromLastKF(
        const std::shared_ptr<KeyFrame> &pCurrentKF,
        std::shared_ptr<KeyFrame> &pMatchedKF, g2o::Sim3 &gScw,
        int &nNumProjMatches, std::vector<MapPoint *> &vpMPs,
        std::vector<MapPoint *> &vpMatchedMPs);

    /**
     * @brief 基于词袋BoW候选帧检测公共区域，回环/融合入口
     * @param vpBowCand 词袋检索得到候选关键帧数组
     * @param pMatchedKF 输出匹配成功的关键帧
     * @param pLastCurrentKF 上一轮当前关键帧
     * @param g2oScw 输出Sim3世界到当前帧变换
     * @param nNumCoincidences 输出重合匹配点数量
     * @param vpMPs 输入地图点
     * @param vpMatchedMPs 输出匹配地图点
     * @return true检测到公共区域；false失败
     */
    bool DetectCommonRegionsFromBoW(
        std::vector<std::shared_ptr<KeyFrame> > &vpBowCand,
        std::shared_ptr<KeyFrame> &pMatchedKF,
        std::shared_ptr<KeyFrame> &pLastCurrentKF, g2o::Sim3 &g2oScw,
        int &nNumCoincidences, std::vector<MapPoint *> &vpMPs,
        std::vector<MapPoint *> &vpMatchedMPs);

    /**
     * @brief 从上一次匹配关键帧检测公共区域，用于跟踪延续
     * @param pCurrentKF 当前查询关键帧
     * @param pMatchedKF 匹配的历史关键帧
     * @param gScw 输出Sim3变换
     * @param nNumProjMatches 输出投影匹配数目
     * @param vpMPs 输入地图点
     * @param vpMatchedMPs 输出匹配地图点
     * @return true检测成功
     */
    bool DetectCommonRegionsFromLastKF(
        const std::shared_ptr<KeyFrame> &pCurrentKF,
        const std::shared_ptr<KeyFrame> &pMatchedKF, g2o::Sim3 &gScw,
        int &nNumProjMatches, std::vector<MapPoint *> &vpMPs,
        std::vector<MapPoint *> &vpMatchedMPs);

    /**
     * @brief 根据Sim3投影，在两个关键帧之间寻找地图点匹配
     * @param pCurrentKF 当前关键帧
     * @param pMatchedKFw 匹配历史关键帧
     * @param g2oScw Sim3世界到当前帧变换
     * @param spMatchedMPinOrigin 原始匹配点集合
     * @param vpMapPoints 输入待投影地图点
     * @param vpMatchedMapPoints 输出匹配成功地图点
     * @return 返回成功匹配点数量
     */
    int FindMatchesByProjection(const std::shared_ptr<KeyFrame> &pCurrentKF,
                                const std::shared_ptr<KeyFrame> &pMatchedKFw,
                                g2o::Sim3 &g2oScw,
                                set<MapPoint *> &spMatchedMPinOrigin,
                                vector<MapPoint *> &vpMapPoints,
                                vector<MapPoint *> &vpMatchedMapPoints);

    /**
     * @brief 根据校正后的位姿，搜索并融合地图点，重复地图点进行替换合并
     * @param CorrectedPosesMap 校正后关键帧‑Sim3位姿集合
     * @param vpMapPoints 输入需要融合处理的地图点
     */
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap,
                      vector<MapPoint *> &vpMapPoints);

    /**
     * @brief 指定共视关键帧集合，做地图点搜索融合
     * @param vConectedKFs 共视连接关键帧数组
     * @param vpMapPoints 待融合地图点数组
     */
    void SearchAndFuse(const vector<std::shared_ptr<KeyFrame> > &vConectedKFs,
                      vector<MapPoint *> &vpMapPoints);

    /**
     * @brief 回环校正主流程：地图点融合、本质图优化、触发GBA
     */
    void CorrectLoop();

    /**
     * @brief 多地图融合逻辑版本1，Atlas模式下两个地图合并
     */
    void MergeLocal();

    /**
     * @brief 多地图融合逻辑版本2，改进版地图合并
     */
    void MergeLocal2();

    /**
     * @brief 检查两组关键帧集合的地图点观测关系，用于地图融合校验
     * @param spKFsMap1 地图1关键帧集合
     * @param spKFsMap2 地图2关键帧集合
     */
    void CheckObservations(set<std::shared_ptr<KeyFrame> > &spKFsMap1,
                          set<std::shared_ptr<KeyFrame> > &spKFsMap2);

    /**
     * @brief 如果收到重置请求，则执行重置逻辑，清空队列、缓存变量
     */
    void ResetIfRequested();
    bool mbResetRequested;             // 全局重置请求标记
    bool mbResetActiveMapRequested;    // 仅重置激活地图请求标记
    std::shared_ptr<Map> mpMapToReset;// 需要重置的目标地图
    std::mutex mMutexReset;           // 重置状态互斥锁

    /**
     * @brief 检查是否收到线程结束请求
     * @return true收到结束请求
     */
    bool CheckFinish();

    /**
     * @brief 设置线程已经完成结束标记
     */
    void SetFinish();
    bool mbFinishRequested; // 请求线程结束标记
    bool mbFinished;        // 线程已经结束标记
    std::mutex mMutexFinish;// 结束状态变量互斥锁

    std::shared_ptr<Atlas> mpAtlas;               // 全局地图集Atlas指针
    std::shared_ptr<Tracking> mpTracker;           // 跟踪模块指针

    std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;// 关键帧词袋数据库
    std::shared_ptr<ORBVocabulary> mpORBVocabulary;// ORB词袋字典

    std::shared_ptr<LocalMapping> mpLocalMapper;   // 局部建图模块指针

    std::list<std::shared_ptr<KeyFrame> > mlpLoopKeyFrameQueue; // 回环待处理关键帧队列
    std::mutex mMutexLoopQueue; // 保护回环队列的互斥锁

    // Loop detector parameters 回环检测器参数
    float mnCovisibilityConsistencyTh; // 共视一致性阈值，用于筛选稳定回环候选

    // Loop detector variables 回环检测临时变量
    std::shared_ptr<KeyFrame> mpCurrentKF;               // 当前待检测回环的关键帧
    std::shared_ptr<KeyFrame> mpLastCurrentKF;            // 上一轮检测使用的当前关键帧
    std::shared_ptr<KeyFrame> mpMatchedKF;                // 匹配到的历史回环关键帧
    std::vector<ConsistentGroup> mvConsistentGroups;      // 一致性候选组集合
    std::vector<std::shared_ptr<KeyFrame> > mvpEnoughConsistentCandidates; // 达到一致性阈值的候选帧
    std::vector<std::shared_ptr<KeyFrame> > mvpCurrentConnectedKFs;       // 当前帧的共视连接关键帧
    std::vector<MapPoint *> mvpCurrentMatchedPoints;      // 当前帧匹配成功地图点
    std::vector<MapPoint *> mvpLoopMapPoints;             // 回环侧地图点
    cv::Mat mScw;                                         // cv::Mat格式Sim3变换矩阵(世界到当前帧)
    g2o::Sim3 mg2oScw;                                    // g2o Sim3格式世界到当前帧相似变换

    //-------
    std::shared_ptr<Map> mpLastMap; // 上一次处理的地图

    bool mbLoopDetected;                     // 是否检测到回环标记
    int mnLoopNumCoincidences;               // 回环匹配重合点数量
    int mnLoopNumNotFound;                   // 回环匹配失败计数
    std::shared_ptr<KeyFrame> mpLoopLastCurrentKF; // 回环分支保存上一当前关键帧
    g2o::Sim3 mg2oLoopSlw;                   // 回环分支Sim3变换
    g2o::Sim3 mg2oLoopScw;                   // 回环分支世界到当前帧Sim3
    std::shared_ptr<KeyFrame> mpLoopMatchedKF; // 回环匹配到的历史关键帧
    std::vector<MapPoint *> mvpLoopMPs;      // 回环分支地图点
    std::vector<MapPoint *> mvpLoopMatchedMPs;// 回环分支匹配地图点

    bool mbMergeDetected;                    // 是否检测到地图融合(Merge)标记
    int mnMergeNumCoincidences;              // 融合匹配重合点计数
    int mnMergeNumNotFound;                  // 融合匹配失败计数
    std::shared_ptr<KeyFrame> mpMergeLastCurrentKF; // 融合分支保存上一当前关键帧
    g2o::Sim3 mg2oMergeSlw;                  // 融合分支Sim3变换
    g2o::Sim3 mg2oMergeSmw;                  // 融合分支Sim3变换
    g2o::Sim3 mg2oMergeScw;                  // 融合分支世界到当前帧Sim3
    std::shared_ptr<KeyFrame> mpMergeMatchedKF; // 融合匹配到的另一张地图关键帧
    std::vector<MapPoint *> mvpMergeMPs;     // 融合分支地图点
    std::vector<MapPoint *> mvpMergeMatchedMPs; // 融合分支匹配地图点
    std::vector<std::shared_ptr<KeyFrame> > mvpMergeConnectedKFs; // 融合分支共视连接关键帧

    g2o::Sim3 mSold_new; // Merge时旧地图到新地图Sim3变换
    //-------

    long unsigned int mLastLoopKFid; // 上一次成功回环的关键帧ID

    // Variables related to Global Bundle Adjustment 全局GBA相关变量
    bool mbRunningGBA;        // GBA正在运行标记
    bool mbFinishedGBA;      // GBA执行完毕标记
    bool mbStopGBA;           // 请求停止GBA标记
    std::mutex mMutexGBA;    // GBA状态互斥锁
    std::thread *mpThreadGBA;// GBA独立子线程指针

    // Fix scale in the stereo/RGB‑D case 双目/RGBD模式固定尺度；单目IMU尺度可变
    bool mbFixScale;

    int mnFullBAIdx; // 全局BA执行计数索引

    vector<double> vdPR_CurrentTime;  // 地点识别：当前帧时间戳
    vector<double> vdPR_MatchedTime;  // 地点识别：匹配帧时间戳
    vector<int> vnPR_TypeRecogn;     // 地点识别类型标记：回环 / Merge融合

    // DEBUG 调试输出参数
    string mstrFolderSubTraj; // 子轨迹保存输出文件夹路径
    int mnNumCorrection;      // 校正执行总次数
    int mnCorrectionGBA;      // GBA校正次数

    // To (de)activate LC 回环检测功能开关
    bool mbActiveLC = true;

#ifdef REGISTER_LOOP // 编译宏，开启回环数据记录，保存回环相关文件
    string mstrFolderLoop;
#endif
};

} // namespace ORB_SLAM3
