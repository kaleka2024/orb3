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

// 局部建图模块头文件声明
#include "LocalMapping.h"
// STL标准库：通用算法函数
#include <algorithm>
// STL标准库：时间高精度计时
#include <chrono>
// STL标准库：标准输入输出
#include <iostream>
// STL标准库：双向链表容器
#include <list>
// STL标准库：智能指针管理
#include <memory>
// STL标准库：互斥锁，线程同步
#include <mutex>
// STL标准库：集合容器
#include <set>
// STL标准库：字符串
#include <string>
// STL标准库：元组结构
#include <tuple>
// STL标准库：成对结构
#include <utility>
// STL标准库：动态数组
#include <vector>

// ORB-SLAM3内部：类型转换工具集
#include "Converter.h"
// ORB-SLAM3内部：几何计算工具集
#include "GeometricTools.h"
// ORB-SLAM3内部：回环检测模块
#include "LoopClosing.h"
// ORB-SLAM3内部：ORB特征匹配器
#include "ORBmatcher.h"
// ORB-SLAM3内部：优化器（BA、IMU优化等）
#include "Optimizer.h"

namespace ORB_SLAM3 {

// ==============================================
// 局部建图模块构造函数
// 输入：pSys 系统指针，pAtlas 图集指针，bMonocular 是否单目模式，bInertial 是否启用IMU，_strSeqName 序列名
// 功能：初始化所有成员变量、标志位、计数器与状态
// ==============================================
LocalMapping::LocalMapping(const std::shared_ptr<System> &pSys,
                           const std::shared_ptr<Atlas> &pAtlas,
                           const float bMonocular, bool bInertial,
                           const string &_strSeqName)
    : mpSystem(pSys),             // 系统对象指针
      mbMonocular(bMonocular),     // 单目模式标志
      mbInertial(bInertial),       // 惯性/IMU模式标志
      mbResetRequested(false),     // 全局重置请求标志
      mbResetRequestedActiveMap(false), // 活动地图重置请求标志
      mbFinishRequested(false),    // 结束线程请求标志
      mbFinished(true),           // 线程已完成标志
      mpAtlas(pAtlas),            // 图集管理器指针
      bInitializing(false),       // IMU初始化中标志
      mbAbortBA(false),           // 中断BA优化标志
      mbStopped(false),           // 线程已停止标志
      mbStopRequested(false),     // 停止线程请求标志
      mbNotStop(false),           // 禁止停止标志
      mbAcceptKeyFrames(true),    // 可接受关键帧标志（跟踪线程判断局部建图是否忙）
      mIdxInit(0),                // 初始化次数计数
      mScale(1.0),                // 尺度因子（IMU初始化用）
      mInitSect(0),               // 初始化段计数
      mbNotBA1(true),             // 未执行第一阶段惯性BA标志
      mbNotBA2(true),             // 未执行第二阶段惯性BA标志
      mIdxIteration(0),           // 迭代次数索引
      infoInertial(Eigen::MatrixXd::Zero(9, 9)) { // IMU信息矩阵
  mnMatchesInliers = 0;       // 内点匹配数

  mbBadImu = false;           // IMU异常标志

  mTinit = 0.f;              // IMU初始化累计时间

  mNumLM = 0;                 // 局部建图次数
  mNumKFCulling = 0;          // 关键帧剔除次数

#ifdef REGISTER_TIMES
  nLBA_exec = 0;              // 局部BA执行次数
  nLBA_abort = 0;             // 局部BA中断次数
#endif
}

// ==============================================
// 设置回环检测模块指针
// ==============================================
void LocalMapping::SetLoopCloser(
    const std::shared_ptr<LoopClosing> &pLoopCloser) {
  mpLoopCloser = pLoopCloser;
}

// ==============================================
// 设置跟踪模块指针
// ==============================================
void LocalMapping::SetTracker(const std::shared_ptr<Tracking> &pTracker) {
  mpTracker = pTracker;
}

// ==============================================
// 局部建图主线程主循环
// 核心流程：新关键帧处理 → 地图点剔除 → 新地图点三角化 → 邻域融合 → 局部BA → 关键帧剔除 → 送回环检测
// ==============================================
void LocalMapping::Run() {
  mbFinished = false;

  while (true) {
    oslog::trace("@@ LocalMapping: loop running...");

    // Tracking will see that Local Mapping is busy
    // 标记为忙，不接受新关键帧，避免处理过程中被插入
    SetAcceptKeyFrames(false);

    // Check if there are keyframes in the queue
    // 队列中有新关键帧且IMU正常，则进入处理流程
    if (CheckNewKeyFrames() && !mbBadImu) {
#ifdef REGISTER_TIMES
      // 计时：关键帧处理开始
      double timeLBA_ms = 0;
      double timeKFCulling_ms = 0;

      std::chrono::steady_clock::time_point time_StartProcessKF =
          std::chrono::steady_clock::now();
#endif

      // BoW conversion and insertion in Map
      // 第一步：处理新关键帧：计算BoW、更新地图点观测、插入地图
      ProcessNewKeyFrame();

#ifdef REGISTER_TIMES
      // 计时：关键帧插入结束
      std::chrono::steady_clock::time_point time_EndProcessKF =
          std::chrono::steady_clock::now();

      double timeProcessKF =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndProcessKF - time_StartProcessKF)
              .count();
      vdKFInsert_ms.push_back(timeProcessKF);
#endif

      // Check recent MapPoints
      // 第二步：剔除质量差的新建地图点
      MapPointCulling();

#ifdef REGISTER_TIMES
      // 计时：地图点剔除结束
      std::chrono::steady_clock::time_point time_EndMPCulling =
          std::chrono::steady_clock::now();

      double timeMPCulling =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndMPCulling - time_EndProcessKF)
              .count();
      vdMPCulling_ms.push_back(timeMPCulling);
#endif

      // Triangulate new MapPoints
      // 第三步：与邻域关键帧三角化，生成新的地图点
      CreateNewMapPoints();

      mbAbortBA = false;

      // 没有新关键帧插入的话，执行邻域搜索融合
      if (!CheckNewKeyFrames()) {
        // Find more matches in neighbor keyframes and fuse point duplications
        SearchInNeighbors();
      }

#ifdef REGISTER_TIMES
      // 计时：新地图点创建结束
      std::chrono::steady_clock::time_point time_EndMPCreation =
          std::chrono::steady_clock::now();

      double timeMPCreation =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndMPCreation - time_EndMPCulling)
              .count();
      vdMPCreation_ms.push_back(timeMPCreation);
#endif

      bool b_doneLBA = false;
      int num_FixedKF_BA = 0;    // BA中固定的关键帧数量
      int num_OptKF_BA = 0;      // BA中优化的关键帧数量
      int num_MPs_BA = 0;        // BA中优化的地图点数量
      int num_edges_BA = 0;       // BA中的边数量

      // 没有新关键帧且未请求停止，则执行局部BA
      if (!CheckNewKeyFrames() && !stopRequested()) {
        // 至少2个关键帧才做BA
        if (mpAtlas->KeyFramesInMap() > 2) {
          // 惯性模式且地图已完成IMU初始化，执行惯性局部BA
          if (mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized()) {
            // 计算最近两段基线的位移和，判断运动幅度
            float dist =
                (mpCurrentKeyFrame->mPrevKF->GetCameraCenter() -
                 mpCurrentKeyFrame->GetCameraCenter())
                    .norm() +
                (mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() -
                 mpCurrentKeyFrame->mPrevKF->GetCameraCenter())
                    .norm();

            // 位移足够大则累加初始化时间
            if (dist > 0.05)
              mTinit += mpCurrentKeyFrame->mTimeStamp -
                         mpCurrentKeyFrame->mPrevKF->mTimeStamp;

            // 还没到第二阶段BA时，检查运动是否足够
            if (!mpCurrentKeyFrame->GetMap()->GetInertialBA2()) {
              // 初始化时间不足10s且位移太小，判定IMU异常，请求重置
              if ((mTinit < 10.f) && (dist < 0.02)) {
                cout << "Not enough motion for initializing. Reseting..."
                     << endl;
                unique_lock<mutex> lock(mMutexReset);
                mbResetRequestedActiveMap = true;
                mpMapToReset = mpCurrentKeyFrame->GetMap();
                mbBadImu = true;
              }
            }

            // 判断是否为大场景：单目内点>75，双目>100
            bool bLarge =
                ((mpTracker->GetMatchesInliers() > 75) && mbMonocular) ||
                ((mpTracker->GetMatchesInliers() > 100) && !mbMonocular);

            // 执行惯性局部BA优化
            Optimizer::LocalInertialBA(
                mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
                num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA, bLarge,
                !mpCurrentKeyFrame->GetMap()->GetInertialBA2());
            b_doneLBA = true;
          } else {
            // 纯视觉模式：执行普通局部束调整
            Optimizer::LocalBundleAdjustment(
                mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
                num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
            b_doneLBA = true;
          }
        }

#ifdef REGISTER_TIMES
        // 计时：局部BA结束
        std::chrono::steady_clock::time_point time_EndLBA =
            std::chrono::steady_clock::now();

        if (b_doneLBA) {
          timeLBA_ms = std::chrono::duration_cast<
                             std::chrono::duration<double, std::milli>>(
                             time_EndLBA - time_EndMPCreation)
                             .count();
          vdLBA_ms.push_back(timeLBA_ms);

          nLBA_exec += 1;
          if (mbAbortBA) {
            nLBA_abort += 1;
          }

          vnLBA_edges.push_back(num_edges_BA);
          vnLBA_KFopt.push_back(num_OptKF_BA);
          vnLBA_KFfixed.push_back(num_FixedKF_BA);
          vnLBA_MPs.push_back(num_MPs_BA);
        }

#endif

        // Initialize IMU here
        // 未初始化IMU且是惯性模式，执行IMU初始化
        if (!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial) {
          if (mbMonocular)
            InitializeIMU(1e2, 1e10, true);  // 单目初始化参数
          else
            InitializeIMU(1e2, 1e5, true);  // 双目初始化参数
        }

        // Check redundant local Keyframes
        // 第四步：关键帧剔除，删除冗余关键帧
        KeyFrameCulling();

#ifdef REGISTER_TIMES
        // 计时：关键帧剔除结束
        std::chrono::steady_clock::time_point time_EndKFCulling =
            std::chrono::steady_clock::now();

        timeKFCulling_ms = std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(
                                time_EndKFCulling - time_EndLBA)
                                .count();
        vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif

        // 初始化时间不足50s且是惯性模式，执行BA阶段升级与尺度精化
        if ((mTinit < 50.0f) && mbInertial) {
          // Enter here everytime local-mapping is called
          // IMU已初始化且跟踪正常
          if (mpCurrentKeyFrame->GetMap()->isImuInitialized() &&
              mpTracker->mState == Tracking::OK) {
            // 第一阶段BA：初始化时间超过5s触发
            if (!mpCurrentKeyFrame->GetMap()->GetInertialBA1()) {
              if (mTinit > 5.0f) {
                cout << "start VIBA 1" << endl;
                mpCurrentKeyFrame->GetMap()->SetInertialBA1();
                if (mbMonocular)
                  InitializeIMU(1.f, 1e5, true);
                else
                  InitializeIMU(1.f, 1e5, true);

                cout << "end VIBA 1" << endl;
              }
            }
            // 第二阶段BA：初始化时间超过15s触发
            else if (!mpCurrentKeyFrame->GetMap()->GetInertialBA2()) {
              if (mTinit > 15.0f) {
                cout << "start VIBA 2" << endl;
                mpCurrentKeyFrame->GetMap()->SetInertialBA2();
                if (mbMonocular)
                  InitializeIMU(0.f, 0.f, true);
                else
                  InitializeIMU(0.f, 0.f, true);

                cout << "end VIBA 2" << endl;
              }
            }

            // scale refinement
            // 尺度精化：每隔10s左右触发一次，仅单目
            if (((mpAtlas->KeyFramesInMap()) <= 200) &&
                ((mTinit > 25.0f && mTinit < 25.5f) ||
                 (mTinit > 35.0f && mTinit < 35.5f) ||
                 (mTinit > 45.0f && mTinit < 45.5f) ||
                 (mTinit > 55.0f && mTinit < 55.5f) ||
                 (mTinit > 65.0f && mTinit < 65.5f) ||
                 (mTinit > 75.0f && mTinit < 75.5f))) {
              if (mbMonocular) ScaleRefinement();
            }
          }
        }
      }

#ifdef REGISTER_TIMES
      vdLBASync_ms.push_back(timeKFCulling_ms);
      vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif

      // 将当前关键帧插入回环检测队列
      mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

#ifdef REGISTER_TIMES
      // 计时：局部建图整轮结束
      std::chrono::steady_clock::time_point time_EndLocalMap =
          std::chrono::steady_clock::now();

      double timeLocalMap =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndLocalMap - time_StartProcessKF)
              .count();
      vdLMTotal_ms.push_back(timeLocalMap);
#endif
    }
    // 没有新关键帧，检查停止请求
    else if (Stop() && !mbBadImu) {
      // Safe area to stop
      // 停止状态且未结束，循环等待
      while (isStopped() && !CheckFinish()) {
        usleep(300);
      }
      // 收到结束请求则退出循环
      if (CheckFinish()) break;
    }

    // 处理重置请求
    ResetIfRequested();

    // Tracking will see that Local Mapping is busy
    // 处理完一轮，标记为空闲，可以接受新关键帧
    SetAcceptKeyFrames(true);

    // 结束请求则退出
    if (CheckFinish()) break;

    // 睡眠3ms，避免空转占满CPU
    usleep(300);
  }

  // 标记线程结束
  SetFinish();
}

// ==============================================
// 向局部建图队列插入一个新关键帧
// 说明：由跟踪线程调用，同时设置中断BA标志，让新关键帧优先处理
// ==============================================
void LocalMapping::InsertKeyFrame(const std::shared_ptr<KeyFrame> &pKF) {
  unique_lock<mutex> lock(mMutexNewKFs);
  mlNewKeyFrames.push_back(pKF);
  mbAbortBA = true;  // 中断正在进行的BA优化
}

// ==============================================
// 检查新关键帧队列是否非空
// ==============================================
bool LocalMapping::CheckNewKeyFrames() {
  unique_lock<mutex> lock(mMutexNewKFs);
  return (!mlNewKeyFrames.empty());
}

// ==============================================
// 处理队首的新关键帧
// 功能：计算BoW、更新地图点观测、更新共视连接、加入地图
// ==============================================
void LocalMapping::ProcessNewKeyFrame() {
  {
    unique_lock<mutex> lock(mMutexNewKFs);
    // 取出队首关键帧
    mpCurrentKeyFrame = mlNewKeyFrames.front();
    mlNewKeyFrames.pop_front();
  }

  // Compute Bags of Words structures
  // 计算关键帧的词袋向量，用于后续回环、重定位
  mpCurrentKeyFrame->ComputeBoW();

  // Associate MapPoints to the new keyframe and update normal and descriptor
  // 获取当前帧的地图点匹配数组
  const vector<MapPoint *> vpMapPointMatches =
      mpCurrentKeyFrame->GetMapPointMatches();

  // 遍历所有匹配的地图点
  for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
    MapPoint *pMP = vpMapPointMatches[i];
    if (pMP) {
      if (!pMP->isBad()) {
        // 地图点未在该关键帧观测过，则添加观测
        if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
          pMP->AddObservation(mpCurrentKeyFrame, i);
          // 更新地图点的平均法向量与深度
          pMP->UpdateNormalAndDepth();
          // 更新地图点的代表性描述子
          pMP->ComputeDistinctiveDescriptors();
        } else {
          // this can only happen for new stereo points inserted by the
          // Tracking
          // 跟踪插入的新立体点，加入新建列表
          mlpRecentAddedMapPoints.push_back(pMP);
        }
      }
    }
  }

  // Update links in the Covisibility Graph
  // 更新关键帧的共视连接图
  mpCurrentKeyFrame->UpdateConnections();

  // Insert Keyframe in Map
  // 将关键帧加入所属地图
  mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

// ==============================================
// 清空新关键帧队列，全部处理完
// ==============================================
void LocalMapping::EmptyQueue() {
  while (CheckNewKeyFrames()) ProcessNewKeyFrame();
}

// ==============================================
// 新建地图点剔除策略
// 原理：对最近新增的地图点进行质量检查，剔除观测少、找到率低、质量差的点
// 规则：坏点直接删；找到率<25%删；超过2帧且观测≤阈值删；超过3帧移出监控列表
// ==============================================
void LocalMapping::MapPointCulling() {
  // Check Recent Added MapPoints
  // 遍历最近新增的地图点列表
  list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
  const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

  int nThObs;
  // 观测次数阈值：单目2次，双目3次
  if (mbMonocular)
    nThObs = 2;
  else
    nThObs = 3;
  const int cnThObs = nThObs;

  int borrar = mlpRecentAddedMapPoints.size();

  while (lit != mlpRecentAddedMapPoints.end()) {
    MapPoint *pMP = *lit;

    // 已经是坏点，直接删除
    if (pMP->isBad()) {
      lit = mlpRecentAddedMapPoints.erase(lit);
    }
    // 找到率低于25%，标记为坏点并删除
    else if (pMP->GetFoundRatio() < 0.25f) {
      pMP->SetBadFlag();
      lit = mlpRecentAddedMapPoints.erase(lit);
    }
    // 存在超过2帧且观测次数≤阈值，标记为坏点
    else if ((static_cast<int>(nCurrentKFid) -
                static_cast<int>(pMP->mnFirstKFid)) >= 2 &&
               pMP->Observations() <= cnThObs) {
      pMP->SetBadFlag();
      lit = mlpRecentAddedMapPoints.erase(lit);
    }
    // 存在超过3帧，移出监控列表（认为稳定）
    else if ((static_cast<int>(nCurrentKFid) -
                static_cast<int>(pMP->mnFirstKFid)) >= 3) {
      lit = mlpRecentAddedMapPoints.erase(lit);
    } else {
      lit++;
      borrar--;
    }
  }
}

// ==============================================
// 三角化创建新地图点
// 原理：取当前帧的共视邻域关键帧，特征匹配后用DLT三角化生成3D点，经多重校验后加入地图
// 校验项：基线长度、视差角、正深度、重投影误差、尺度一致性
// ==============================================
void LocalMapping::CreateNewMapPoints() {
  // Retrieve neighbor keyframes in covisibility graph
  // 取前N个最佳共视关键帧
  int nn = 10;
  // For stereo inertial case
  // 单目取30个邻域
  if (mbMonocular) nn = 30;
  auto vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

  // 惯性模式下，补充时间上相邻的前序关键帧
  if (mbInertial) {
    auto pKF = mpCurrentKeyFrame;
    int count = 0;
    while ((vpNeighKFs.size() <= nn) && (pKF->mPrevKF) && (count++ < nn)) {
      auto it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
      if (it == vpNeighKFs.end()) vpNeighKFs.push_back(pKF->mPrevKF);
      pKF = pKF->mPrevKF;
    }
  }

  // ORB匹配阈值
  const float th = 0.6f;

  ORBmatcher matcher(th, false);

  // 获取当前帧的位姿、旋转、平移、相机中心
  Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
  Eigen::Matrix<float, 3, 4> eigTcw1 = sophTcw1.matrix3x4();
  Eigen::Matrix<float, 3, 3> Rcw1 = eigTcw1.block<3, 3>(0, 0);
  Eigen::Matrix<float, 3, 3> Rwc1 = Rcw1.transpose();
  Eigen::Vector3f tcw1 = sophTcw1.translation();
  Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

  // 当前帧相机内参与倒数
  const float &fx1 = mpCurrentKeyFrame->fx;
  const float &fy1 = mpCurrentKeyFrame->fy;
  const float &cx1 = mpCurrentKeyFrame->cx;
  const float &cy1 = mpCurrentKeyFrame->cy;
  const float &invfx1 = mpCurrentKeyFrame->invfx;
  const float &invfy1 = mpCurrentKeyFrame->invfy;

  // 尺度比例因子
  const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;

  int countStereo = 0;
  int countStereoGoodProj = 0;
  int countStereoAttempt = 0;
  int totalStereoPts = 0;

  // Search matches with epipolar restriction and triangulate
  // 遍历每个邻域关键帧
  for (size_t i = 0; i < vpNeighKFs.size(); i++) {
    // 中途有新关键帧则返回，优先处理
    if (i > 0 && CheckNewKeyFrames()) return;

    auto pKF2 = vpNeighKFs[i];

    std::shared_ptr<GeometricCamera> pCamera1 = mpCurrentKeyFrame->mpCamera,
                                     pCamera2 = pKF2->mpCamera;

    // Check first that baseline is not too short
    // 计算两帧基线长度
    Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
    Eigen::Vector3f vBaseline = Ow2 - Ow1;
    const float baseline = vBaseline.norm();

    // 双目模式：基线小于像素基线则跳过（太近无法三角化）
    if (!mbMonocular) {
      if (baseline < pKF2->mb) continue;
    } else {
      // 单目模式：基线深度比太小则跳过
      const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
      const float ratioBaselineDepth = baseline / medianDepthKF2;

      if (ratioBaselineDepth < 0.01) continue;
    }

    // Search matches that fullfil epipolar constraint
    // 搜索满足对极约束的特征匹配
    vector<pair<size_t, size_t>> vMatchedIndices;
    // 粗匹配标志：惯性模式+最近丢失+第二阶段BA时用粗匹配
    bool bCoarse = mbInertial && mpTracker->mState == Tracking::RECENTLY_LOST &&
                   mpCurrentKeyFrame->GetMap()->GetInertialBA2();

    matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, vMatchedIndices,
                                   false, bCoarse);

    // 获取邻域帧的位姿、内参
    Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
    Eigen::Matrix<float, 3, 4> eigTcw2 = sophTcw2.matrix3x4();
    Eigen::Matrix<float, 3, 3> Rcw2 = eigTcw2.block<3, 3>(0, 0);
    Eigen::Matrix<float, 3, 3> Rwc2 = Rcw2.transpose();
    Eigen::Vector3f tcw2 = sophTcw2.translation();

    const float &fx2 = pKF2->fx;
    const float &fy2 = pKF2->fy;
    const float &cx2 = pKF2->cx;
    const float &cy2 = pKF2->cy;
    const float &invfx2 = pKF2->invfx;
    const float &invfy2 = pKF2->invfy;

    // Triangulate each match
    // 遍历每一对匹配点，逐个三角化
    const int nmatches = vMatchedIndices.size();
    for (int ikp = 0; ikp < nmatches; ikp++) {
      const int &idx1 = vMatchedIndices[ikp].first;
      const int &idx2 = vMatchedIndices[ikp].second;

      // 获取当前帧的特征点，区分左目/右目
      const cv::KeyPoint &kp1 =
          (mpCurrentKeyFrame->NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
          : (idx1 < mpCurrentKeyFrame->NLeft)
              ? mpCurrentKeyFrame->mvKeys[idx1]
              : mpCurrentKeyFrame->mvKeysRight[idx1 - mpCurrentKeyFrame->NLeft];
      const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
      bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur >= 0);
      const bool bRight1 =
          (mpCurrentKeyFrame->NLeft == -1 || idx1 < mpCurrentKeyFrame->NLeft)
              ? false
              : true;

      // 获取邻域帧的特征点
      const cv::KeyPoint &kp2 = (pKF2->NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                 : (idx2 < pKF2->NLeft)
                                     ? pKF2->mvKeys[idx2]
                                     : pKF2->mvKeysRight[idx2 - pKF2->NLeft];

      const float kp2_ur = pKF2->mvuRight[idx2];
      bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur >= 0);
      const bool bRight2 =
          (pKF2->NLeft == -1 || idx2 < pKF2->NLeft) ? false : true;

      // 双目模式：根据左右目匹配情况选择对应的相机和位姿
      if (mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2) {
        if (bRight1 && bRight2) {
          // 都用右目
          sophTcw1 = mpCurrentKeyFrame->GetRightPose();
          Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

          sophTcw2 = pKF2->GetRightPose();
          Ow2 = pKF2->GetRightCameraCenter();

          pCamera1 = mpCurrentKeyFrame->mpCamera2;
          pCamera2 = pKF2->mpCamera2;
        } else if (bRight1 && !bRight2) {
          // 左1右2
          sophTcw1 = mpCurrentKeyFrame->GetRightPose();
          Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

          sophTcw2 = pKF2->GetPose();
          Ow2 = pKF2->GetCameraCenter();

          pCamera1 = mpCurrentKeyFrame->mpCamera2;
          pCamera2 = pKF2->mpCamera;
        } else if (!bRight1 && bRight2) {
          // 右1左2
          sophTcw1 = mpCurrentKeyFrame->GetPose();
          Ow1 = mpCurrentKeyFrame->GetCameraCenter();

          sophTcw2 = pKF2->GetRightPose();
          Ow2 = pKF2->GetRightCameraCenter();

          pCamera1 = mpCurrentKeyFrame->mpCamera;
          pCamera2 = pKF2->mpCamera2;
        } else {
          // 都用左目
          sophTcw1 = mpCurrentKeyFrame->GetPose();
          Ow1 = mpCurrentKeyFrame->GetCameraCenter();

          sophTcw2 = pKF2->GetPose();
          Ow2 = pKF2->GetCameraCenter();

          pCamera1 = mpCurrentKeyFrame->mpCamera;
          pCamera2 = pKF2->mpCamera;
        }

        // 更新位姿矩阵
        eigTcw1 = sophTcw1.matrix3x4();
        Rcw1 = eigTcw1.block<3, 3>(0, 0);
        Rwc1 = Rcw1.transpose();
        tcw1 = sophTcw1.translation();

        eigTcw2 = sophTcw2.matrix3x4();
        Rcw2 = eigTcw2.block<3, 3>(0, 0);
        Rwc2 = Rcw2.transpose();
        tcw2 = sophTcw2.translation();
      }

      // Check parallax between rays
      // 计算两视角射线的夹角余弦
      Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
      Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

      Eigen::Vector3f ray1 = Rwc1 * xn1;
      Eigen::Vector3f ray2 = Rwc2 * xn2;
      const float cosParallaxRays =
          ray1.dot(ray2) / (ray1.norm() * ray2.norm());

      // 立体视差角余弦
      float cosParallaxStereo = cosParallaxRays + 1;
      float cosParallaxStereo1 = cosParallaxStereo;
      float cosParallaxStereo2 = cosParallaxStereo;

      // 当前帧有立体深度，计算立体视差
      if (bStereo1)
        cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mb / 2,
                                            mpCurrentKeyFrame->mvDepth[idx1]));
      else if (bStereo2)
        cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

      if (bStereo1 || bStereo2) totalStereoPts++;

      cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

      Eigen::Vector3f x3D;

      bool goodProj = false;
      bool bPointStereo = false;

      // 对极视差足够小，用双帧三角化
      if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
          (bStereo1 || bStereo2 || (cosParallaxRays < 0.996 && mbInertial) ||
           (cosParallaxRays < 0.998 && !mbInertial))) {
        goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
        if (!goodProj) continue;
      }
      // 优先用立体深度反投影
      else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
        countStereoAttempt++;
        bPointStereo = true;
        goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
      } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
        countStereoAttempt++;
        bPointStereo = true;
        goodProj = pKF2->UnprojectStereo(idx2, x3D);
      } else {
        continue;  // No stereo and very low parallax
      }

      if (goodProj && bPointStereo) countStereoGoodProj++;

      if (!goodProj) continue;

      // Check triangulation in front of cameras
      // 检查两个相机前深度都为正
      float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
      if (z1 <= 0) continue;

      float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
      if (z2 <= 0) continue;

      // Check reprojection error in first keyframe
      // 检查第一帧重投影误差
      const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
      const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
      const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);
      const float invz1 = 1.0 / z1;

      if (!bStereo1) {
        // 单目：像素重投影误差
        cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1, y1, z1));
        float errX1 = uv1.x - kp1.pt.x;
        float errY1 = uv1.y - kp1.pt.y;

        // 卡方检验：5.991对应自由度2的95%置信
        if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1) continue;
      } else {
        // 双目：左目+右目重投影误差
        float u1 = fx1 * x1 * invz1 + cx1;
        float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
        float v1 = fy1 * y1 * invz1 + cy1;
        float errX1 = u1 - kp1.pt.x;
        float errY1 = v1 - kp1.pt.y;
        float errX1_r = u1_r - kp1_ur;
        // 卡方7.8对应自由度3
        if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) >
            7.8 * sigmaSquare1)
          continue;
      }

      // Check reprojection error in second keyframe
      // 检查第二帧重投影误差
      const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
      const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
      const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
      const float invz2 = 1.0 / z2;
      if (!bStereo2) {
        cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2, y2, z2));
        float errX2 = uv2.x - kp2.pt.x;
        float errY2 = uv2.y - kp2.pt.y;

        if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2) continue;
      } else {
        float u2 = fx2 * x2 * invz2 + cx2;
        float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
        float v2 = fy2 * y2 * invz2 + cy2;
        float errX2 = u2 - kp2.pt.x;
        float errY2 = v2 - kp2.pt.y;
        float errX2_r = u2_r - kp2_ur;
        if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) >
            7.8 * sigmaSquare2)
          continue;
      }

      // Check scale consistency
      // 检查尺度一致性：距离比与尺度金字塔比例匹配
      Eigen::Vector3f normal1 = x3D - Ow1;
      float dist1 = normal1.norm();

      Eigen::Vector3f normal2 = x3D - Ow2;
      float dist2 = normal2.norm();

      if (dist1 == 0 || dist2 == 0) continue;

      // 远点过滤
      if (mbFarPoints &&
          (dist1 >= mThFarPoints || dist2 >= mThFarPoints))  // MODIFICATION
        continue;

      const float ratioDist = dist2 / dist1;
      const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave] /
                              pKF2->mvScaleFactors[kp2.octave];

      // 距离比例与尺度比例相差超过1.5倍则剔除
      if (ratioDist * ratioFactor < ratioOctave ||
          ratioDist > ratioOctave * ratioFactor)
        continue;

      // Triangulation is succesfull
      // 三角化成功，创建新地图点
      MapPoint *pMP =
          new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
      if (bPointStereo) countStereo++;

      // 给两个关键帧都添加观测
      pMP->AddObservation(mpCurrentKeyFrame, idx1);
      pMP->AddObservation(pKF2, idx2);

      mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
      pKF2->AddMapPoint(pMP, idx2);

      // 计算代表性描述子、法向量深度
      pMP->ComputeDistinctiveDescriptors();

      pMP->UpdateNormalAndDepth();

      // 加入地图和新建列表
      mpAtlas->AddMapPoint(pMP);
      mlpRecentAddedMapPoints.push_back(pMP);
    }
  }
}

// ==============================================
// 在邻域关键帧中搜索更多匹配，融合重复地图点
// 功能：扩展邻域范围，双向投影匹配，融合重复的地图点，更新描述子与法向量
// ==============================================
void LocalMapping::SearchInNeighbors() {
  // Retrieve neighbor keyframes
  // 获取一级共视邻域
  int nn = 10;
  if (mbMonocular) nn = 30;
  const auto vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

  list<std::shared_ptr<KeyFrame>> vpTargetKFs;

  // 一级邻域加入目标列表
  for (auto const &pKFi : vpNeighKFs) {
    if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
      continue;

    vpTargetKFs.push_back(pKFi);
    pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
  }

  // Add some covisible of covisible
  // Extend to some second neighbors if abort is not requested
  //   // Note this code extends the list while iterating through it...
  // Ensure we are only going over the initial set
  // 扩展二级共视邻域
  list<std::shared_ptr<KeyFrame>> vpInitialTargetKFs(vpTargetKFs.begin(),
                                                      vpTargetKFs.end());
  for (auto const &pTargetKF : vpInitialTargetKFs) {
    const auto vpSecondNeighKFs = pTargetKF->GetBestCovisibilityKeyFrames(20);

    for (auto const &pKFi2 : vpSecondNeighKFs) {
      if (pKFi2->isBad() ||
          pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
          pKFi2->mnId == mpCurrentKeyFrame->mnId)
        continue;
      vpTargetKFs.push_back(pKFi2);
      pKFi2->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }

    if (mbAbortBA) break;
  }

  // Extend to temporal neighbors
  // 惯性模式下补充时间邻域
  if (mbInertial) {
    auto pKFi = mpCurrentKeyFrame->mPrevKF;
    while (vpTargetKFs.size() < 20 && pKFi) {
      if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId) {
        pKFi = pKFi->mPrevKF;
        continue;
      }
      vpTargetKFs.push_back(pKFi);
      pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
      pKFi = pKFi->mPrevKF;
    }
  }

  // Search matches by projection from current KF in target KFs
  // 匹配器：将当前帧的地图点投影到邻域帧匹配
  ORBmatcher matcher;
  vector<MapPoint *> vpMapPointMatches =
      mpCurrentKeyFrame->GetMapPointMatches();

  // 正向投影匹配：当前帧地图点 → 邻域帧
  for (auto pKFi : vpTargetKFs) {
    matcher.Fuse(pKFi, vpMapPointMatches);
    // 双目模式同时匹配右目
    if (pKFi->NLeft != -1) matcher.Fuse(pKFi, vpMapPointMatches, true);
  }

  if (mbAbortBA) return;

  // Search matches by projection from target KFs in current KF
  // 反向投影匹配：邻域帧地图点 → 当前帧
  vector<MapPoint *> vpFuseCandidates;
  vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

  for (auto pKFi : vpTargetKFs) {
    auto vpMapPointsKFi = pKFi->GetMapPointMatches();
    for (auto pMP : vpMapPointsKFi) {
      if (!pMP) continue;
      if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
        continue;
      pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
      vpFuseCandidates.push_back(pMP);
    }
  }

  matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);
  if (mpCurrentKeyFrame->NLeft != -1)
    matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates, true);

  // Update points
  // 更新所有地图点的描述子和法向量
  vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
  for (auto pMP : vpMapPointMatches) {
    if (pMP) {
      if (!pMP->isBad()) {
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
      }
    }
  }

  // Update connections in covisibility graph
  // 更新共视图连接
  mpCurrentKeyFrame->UpdateConnections();
}

// ==============================================
// 请求停止局部建图线程
// ==============================================
void LocalMapping::RequestStop() {
  unique_lock<mutex> lock(mMutexStop);
  mbStopRequested = true;
  unique_lock<mutex> lock2(mMutexNewKFs);
  mbAbortBA = true;  // 中断BA
}

// ==============================================
// 检查是否应该停止
// ==============================================
bool LocalMapping::Stop() {
  unique_lock<mutex> lock(mMutexStop);
  if (mbStopRequested && !mbNotStop) {
    mbStopped = true;
    cout << "Local Mapping STOP" << endl;
    return true;
  }

  return false;
}

// ==============================================
// 检查是否已停止
// ==============================================
bool LocalMapping::isStopped() {
  unique_lock<mutex> lock(mMutexStop);
  return mbStopped;
}

// ==============================================
// 检查是否请求停止
// ==============================================
bool LocalMapping::stopRequested() {
  unique_lock<mutex> lock(mMutexStop);
  return mbStopRequested;
}

// ==============================================
// 释放停止状态，重新运行
// ==============================================
void LocalMapping::Release() {
  unique_lock<mutex> lock(mMutexStop);
  unique_lock<mutex> lock2(mMutexFinish);
  if (mbFinished) return;
  mbStopped = false;
  mbStopRequested = false;

  // for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(),
  //                                 lend = mlNewKeyFrames.end();
  //      lit != lend; lit++)
  //   delete *lit;

  mlNewKeyFrames.clear();

  cout << "Local Mapping RELEASE" << endl;
}

// ==============================================
// 检查是否可以接受关键帧（跟踪线程判断局部建图是否忙）
// ==============================================
bool LocalMapping::AcceptKeyFrames() {
  unique_lock<mutex> lock(mMutexAccept);
  return mbAcceptKeyFrames;
}

// ==============================================
// 设置是否接受关键帧标志
// ==============================================
void LocalMapping::SetAcceptKeyFrames(bool flag) {
  unique_lock<mutex> lock(mMutexAccept);
  mbAcceptKeyFrames = flag;
}

// ==============================================
// 设置不停止标志，返回是否设置成功
// ==============================================
bool LocalMapping::SetNotStop(bool flag) {
  unique_lock<mutex> lock(mMutexStop);

  if (flag && mbStopped) return false;

  mbNotStop = flag;

  return true;
}

// ==============================================
// 中断当前正在进行的BA优化
// ==============================================
void LocalMapping::InterruptBA() { mbAbortBA = true; }

// ==============================================
// 关键帧剔除策略
// 原理：如果一个关键帧90%的地图点能被其他至少3个关键帧观测到，则认为冗余可删除
// 说明：保证地图不过度增长，同时保留足够约束；IMU模式下更严格，还要维护预积分连续性
// ==============================================
void LocalMapping::KeyFrameCulling() {
  // Check redundant keyframes (only local keyframes)
  // A keyframe is considered redundant if the 90% of the MapPoints it sees, are
  // seen in at least other 3 keyframes (in the same or finer scale) We only
  // consider close stereo points
  // 检测窗口大小：最近21个关键帧
  const int Nd = 21;
  // 更新共视排序
  mpCurrentKeyFrame->UpdateBestCovisibles();
  auto vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

  float redundant_th;
  // 冗余阈值：纯视觉0.9，单目惯性0.9，双目惯性0.5
  if (!mbInertial)
    redundant_th = 0.9;
  else if (mbMonocular)
    redundant_th = 0.9;
  else
    redundant_th = 0.5;

  const bool bInitImu = mpAtlas->isImuInitialized();
  int count = 0;

  // Compoute last KF from optimizable window:
  // 惯性模式下找到优化窗口的最早关键帧ID
  unsigned int last_ID;
  if (mbInertial) {
    int count = 0;
    auto aux_KF = mpCurrentKeyFrame;
    while (count < Nd && aux_KF->mPrevKF) {
      aux_KF = aux_KF->mPrevKF;
      count++;
    }
    last_ID = aux_KF->mnId;
  }

  // 遍历共视关键帧，检查是否冗余
  for (auto pKF : vpLocalKeyFrames) {
    count++;

    // 初始关键帧、坏帧不删
    if ((pKF->mnId == pKF->GetMap()->GetInitKFid()) || pKF->isBad()) continue;

    const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

    int nObs = 3;
    const int thObs = nObs;
    int nRedundantObservations = 0;
    int nMPs = 0;

    // 遍历该关键帧的每个地图点
    for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
      MapPoint *pMP = vpMapPoints[i];
      if (pMP) {
        if (!pMP->isBad()) {
          // 双目模式只考虑近景点
          if (!mbMonocular) {
            if (pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0)
              continue;
          }

          nMPs++;
          // 观测次数超过阈值，进一步检查是否被其他足够多关键帧看到
          if (pMP->Observations() > thObs) {
            // 当前点的尺度层级
            const int &scaleLevel = (pKF->NLeft == -1) ? pKF->mvKeysUn[i].octave
                                        : (i < pKF->NLeft)
                                              ? pKF->mvKeys[i].octave
                                              : pKF->mvKeysRight[i].octave;

            const auto observations = pMP->GetObservations();
            int nObs = 0;

            // 遍历所有观测关键帧
            for (auto const &mit : observations) {
              std::shared_ptr<KeyFrame> pKFi = mit.first;
              if (pKFi == pKF) continue;

              tuple<int, int> indexes = mit.second;
              int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
              int scaleLeveli = -1;

              // 获取观测帧对应点的尺度层级
              if (pKFi->NLeft == -1) {
                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
              } else {
                if (leftIndex != -1) {
                  scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                }
                if (rightIndex != -1) {
                  int rightLevel =
                      pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                  scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel)
                                      ? rightLevel
                                      : scaleLeveli;
                }
              }

              // 尺度层级不低于当前-1，认为有效观测
              if (scaleLeveli <= scaleLevel + 1) {
                nObs++;
                if (nObs > thObs) break;
              }
            }

            // 超过阈值则计数冗余观测
            if (nObs > thObs) nRedundantObservations++;
          }
        }
      }
    }

    // 冗余观测比例超过阈值，判定为冗余关键帧
    if (nRedundantObservations > redundant_th * nMPs) {
      if (mbInertial) {
        // 惯性模式：关键帧太少不删
        if (mpAtlas->KeyFramesInMap() <= Nd) continue;

        // 最近2帧不删
        if (pKF->mnId > (mpCurrentKeyFrame->mnId - 2)) continue;

        // 前后都有关键帧才可以删
        if (pKF->mPrevKF && pKF->mNextKF) {
          const float t = pKF->mNextKF->mTimeStamp - pKF->mPrevKF->mTimeStamp;

          // IMU初始化中且时间间隔小，或时间间隔<0.5s，可以删
          if ((bInitImu && (pKF->mnId < last_ID) && t < 3.) || (t < 0.5)) {
            // 合并预积分：下一帧的预积分合并当前帧的预积分
            pKF->mNextKF->mpImuPreintegrated->MergePrevious(
                pKF->mpImuPreintegrated);
            // 链表重接：绕过当前帧
            pKF->mNextKF->mPrevKF = pKF->mPrevKF;
            pKF->mPrevKF->mNextKF = pKF->mNextKF;
            pKF->mNextKF = NULL;
            pKF->mPrevKF = NULL;
            // 标记为坏帧
            pKF->SetBadFlag();
          } else if (!mpCurrentKeyFrame->GetMap()->GetInertialBA2() &&
                     ((pKF->GetImuPosition() - pKF->mPrevKF->GetImuPosition())
                          .norm() < 0.02) &&
                     (t < 3)) {
            // 位移太小也可以删
            pKF->mNextKF->mpImuPreintegrated->MergePrevious(
                pKF->mpImuPreintegrated);
            pKF->mNextKF->mPrevKF = pKF->mPrevKF;
            pKF->mPrevKF->mNextKF = pKF->mNextKF;
            pKF->mNextKF = NULL;
            pKF->mPrevKF = NULL;
            pKF->SetBadFlag();
          }
        }
      } else {
        // 纯视觉模式直接标记为坏帧
        pKF->SetBadFlag();
      }
    }

    // 超过20个且BA中断，或超过100个，停止检查
    if ((count > 20 && mbAbortBA) || count > 100) {
      break;
    }
  }
}

// ==============================================
// 请求全局重置地图集
// ==============================================
void LocalMapping::RequestReset() {
  {
    unique_lock<mutex> lock(mMutexReset);
    cout << "LM: Map reset recieved" << endl;
    mbResetRequested = true;
  }
  cout << "LM: Map reset, waiting..." << endl;

  // 等待重置完成
  while (1) {
    {
      unique_lock<mutex> lock2(mMutexReset);
      if (!mbResetRequested) break;
    }
    usleep(300);
  }
  cout << "LM: Map reset, Done!!!" << endl;
}

// ==============================================
// 请求重置指定的活动地图
// ==============================================
void LocalMapping::RequestResetActiveMap(const std::shared_ptr<Map> &pMap) {
  {
    unique_lock<mutex> lock(mMutexReset);
    cout << "LM: Active map reset recieved" << endl;
    mbResetRequestedActiveMap = true;
    mpMapToReset = pMap;
  }
  cout << "LM: Active map reset, waiting..." << endl;

  while (1) {
    {
      unique_lock<mutex> lock2(mMutexReset);
      if (!mbResetRequestedActiveMap) break;
    }
    usleep(300);
  }
  cout << "LM: Active map reset, Done!!!" << endl;
}

// ==============================================
// 如果有重置请求则执行重置
// ==============================================
void LocalMapping::ResetIfRequested() {
  bool executed_reset = false;
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbResetRequested) {
      executed_reset = true;

      cout << "LM: Reseting Atlas in Local Mapping..." << endl;
      // 清空关键帧队列
      mlNewKeyFrames.clear();
      // 清空新建地图点列表
      mlpRecentAddedMapPoints.clear();
      mbResetRequested = false;
      mbResetRequestedActiveMap = false;

      // Inertial parameters
      // 重置IMU相关参数
      mTinit = 0.f;
      mbNotBA2 = true;
      mbNotBA1 = true;
      mbBadImu = false;

      mIdxInit = 0;

      cout << "LM: End reseting Local Mapping..." << endl;
    }

    if (mbResetRequestedActiveMap) {
      executed_reset = true;
      cout << "LM: Reseting current map in Local Mapping..." << endl;
      mlNewKeyFrames.clear();
      mlpRecentAddedMapPoints.clear();

      // Inertial parameters
      mTinit = 0.f;
      mbNotBA2 = true;
      mbNotBA1 = true;
      mbBadImu = false;

      mbResetRequested = false;
      mbResetRequestedActiveMap = false;
      cout << "LM: End reseting Local Mapping..." << endl;
    }
  }
  if (executed_reset) cout << "LM: Reset free the mutex" << endl;
}

// ==============================================
// 请求结束线程
// ==============================================
void LocalMapping::RequestFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

// ==============================================
// 检查是否请求结束
// ==============================================
bool LocalMapping::CheckFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

// ==============================================
// 设置线程结束标志
// ==============================================
void LocalMapping::SetFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinished = true;
  unique_lock<mutex> lock2(mMutexStop);
  mbStopped = true;
}

// ==============================================
// 检查线程是否已结束
// ==============================================
bool LocalMapping::isFinished() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinished;
}

// ==============================================
// IMU初始化函数
// 功能：估计重力方向、IMU零偏、尺度，执行全局惯性BA，完成IMU初始化
// 输入：priorG 陀螺零偏先验权重，priorA 加表零偏先验权重，bFIBA 是否执行全量惯性BA
// ==============================================
void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA) {
  if (mbResetRequested) return;

  float minTime;
  int nMinKF;
  // 最小时间和最少关键帧数：单目2s/10帧，双目1s/10帧
  if (mbMonocular) {
    minTime = 2.0;
    nMinKF = 10;
  } else {
    minTime = 1.0;
    nMinKF = 10;
  }

  // 关键帧数量不足则返回
  if (mpAtlas->KeyFramesInMap() < nMinKF) return;

  // Retrieve all keyframe in temporal order
  // 按时间顺序获取所有关键帧
  list<std::shared_ptr<KeyFrame>> lpKF;
  auto pKF = mpCurrentKeyFrame;
  while (pKF->mPrevKF) {
    lpKF.push_front(pKF);
    pKF = pKF->mPrevKF;
  }
  lpKF.push_front(pKF);
  vector<std::shared_ptr<KeyFrame>> vpKF(lpKF.begin(), lpKF.end());

  if (vpKF.size() < nMinKF) return;

  mFirstTs = vpKF.front()->mTimeStamp;
  // 时间跨度不足则返回
  if (mpCurrentKeyFrame->mTimeStamp - mFirstTs < minTime) return;

  bInitializing = true;

  // 先把队列里的关键帧都处理完
  while (CheckNewKeyFrames()) {
    ProcessNewKeyFrame();
    vpKF.push_back(mpCurrentKeyFrame);
    lpKF.push_back(mpCurrentKeyFrame);
  }

  const int N = vpKF.size();
  IMU::Bias b(0, 0, 0, 0, 0, 0);

  // Compute and KF velocities mRwg estimation
  // 未初始化时，估计重力方向与初始零偏
  if (!mpCurrentKeyFrame->GetMap()->isImuInitialized()) {
    Eigen::Matrix3f Rwg;
    Eigen::Vector3f dirG;
    dirG.setZero();

    // 遍历所有关键帧，由位置差计算速度，累加重力方向
    for (auto pKFi : vpKF) {
      if (!pKFi->mpImuPreintegrated) continue;
      if (!pKFi->mPrevKF) continue;

      // 速度方向累加（负重力方向）
      dirG -= pKFi->mPrevKF->GetImuRotation() *
                  pKFi->mpImuPreintegrated->GetUpdatedDeltaVelocity();
      // 由位置差计算速度
      Eigen::Vector3f _vel =
          (pKFi->GetImuPosition() - pKFi->mPrevKF->GetImuPosition()) /
          pKFi->mpImuPreintegrated->dT;
      pKFi->SetVelocity(_vel);
      pKFi->mPrevKF->SetVelocity(_vel);
    }

    // 归一化重力方向
    dirG = dirG / dirG.norm();
    // 世界系重力向下
    Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
    // 计算旋转向量，将估计重力对齐到世界重力
    Eigen::Vector3f v = gI.cross(dirG);
    const float nv = v.norm();
    const float cosg = gI.dot(dirG);
    const float ang = acos(cosg);
    Eigen::Vector3f vzg = v * ang / nv;
    Rwg = Sophus::SO3f::exp(vzg).matrix();

    mRwg = Rwg.cast<double>();
    mTinit = mpCurrentKeyFrame->mTimeStamp - mFirstTs;
  } else {
    // 已初始化则用现有零偏
    mRwg = Eigen::Matrix3d::Identity();
    mbg = mpCurrentKeyFrame->GetGyroBias().cast<double>();
    mba = mpCurrentKeyFrame->GetAccBias().cast<double>();
  }

  mScale = 1.0;

  mInitTime = mpTracker->mLastFrame->mTimeStamp - vpKF.front()->mTimeStamp;

  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  // 执行惯性优化：估计重力、零偏、尺度
  Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg,
                                  mba, mbMonocular, infoInertial, false, false,
                                  priorG, priorA);

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  // 尺度太小则初始化失败
  if (mScale < 1e-1) {
    cout << "scale too small" << endl;
    bInitializing = false;
    return;
  }

  // Before this line we are not changing the map
  // 应用尺度与旋转到整个地图
  {
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    // 尺度变化明显或双目模式，应用缩放旋转
    if ((fabs(mScale - 1.f) > 0.0001) || !mbMonocular) {
      Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
      mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);
      // 更新跟踪帧的IMU状态
      mpTracker->UpdateFrameIMU(mScale, vpKF[0]->GetImuBias(),
                                 mpCurrentKeyFrame);
    }

    // Check if initialization OK
    // 首次初始化则标记所有关键帧为IMU模式
    if (!mpAtlas->isImuInitialized()) {
      for (int i = 0; i < N; i++) {
        std::shared_ptr<KeyFrame> pKF2(vpKF[i]);
        pKF2->bImu = true;
      }
    }
  }

  // 更新跟踪器帧IMU
  mpTracker->UpdateFrameIMU(1.0, vpKF[0]->GetImuBias(), mpCurrentKeyFrame);

  // 首次初始化，设置地图IMU初始化标志
  if (!mpAtlas->isImuInitialized()) {
    mpAtlas->SetImuInitialized();
    mpTracker->t0IMU = mpTracker->mCurrentFrame->mTimeStamp;
    mpCurrentKeyFrame->bImu = true;
  }

  std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();

  // 执行全量惯性全局BA
  if (bFIBA) {
    if (priorA != 0.f)
      Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false,
                                 mpCurrentKeyFrame->mnId, NULL, true, priorG,
                                 priorA);
    else
      Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false,
                                 mpCurrentKeyFrame->mnId, NULL, false);
  }

  std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

  oslog::info("Global Bundle Adjustment finished.  Updating map ...");

  // Get Map Mutex
  unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

  unsigned long GBAid = mpCurrentKeyFrame->mnId;

  // Process keyframes in the queue
  // 处理队列中剩余关键帧
  while (CheckNewKeyFrames()) {
    ProcessNewKeyFrame();
    vpKF.push_back(mpCurrentKeyFrame);
    lpKF.push_back(mpCurrentKeyFrame);
  }

  // Correct keyframes starting at map first keyframe
  // 按生成树层级校正所有关键帧位姿
  list<std::shared_ptr<KeyFrame>> lpKFtoCheck(
      mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.begin(),
      mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.end());

  while (!lpKFtoCheck.empty()) {
    auto pKF = lpKFtoCheck.front();
    const auto sChilds = pKF->GetChilds();
    Sophus::SE3f Twc = pKF->GetPoseInverse();

    // 遍历子节点，计算GBA校正后的位姿
    for (auto pChild : sChilds) {
      if (!pChild || pChild->isBad()) continue;

      if (pChild->mnBAGlobalForKF != GBAid) {
        // 子帧相对父帧的位姿 × 父帧GBA位姿 = 子帧GBA位姿
        Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
        pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

        // 校正速度
        Sophus::SO3f Rcor =
            pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
        if (pChild->isVelocitySet()) {
          pChild->mVwbGBA = Rcor * pChild->GetVelocity();
        } else {
          oslog::warn("Child velocity empty!! ");
        }

        // 保存零偏
        pChild->mBiasGBA = pChild->GetImuBias();
        pChild->mnBAGlobalForKF = GBAid;
      }
      lpKFtoCheck.push_back(pChild);
    }

    // 保存GBA前的位姿，应用GBA位姿
    pKF->mTcwBefGBA = pKF->GetPose();
    pKF->SetPose(pKF->mTcwGBA);

    // 更新速度和零偏
    if (pKF->bImu) {
      pKF->mVwbBefGBA = pKF->GetVelocity();
      pKF->SetVelocity(pKF->mVwbGBA);
      pKF->SetNewBias(pKF->mBiasGBA);
    } else {
      cout << "KF " << pKF->mnId << " not set to inertial!! \n";
    }

    lpKFtoCheck.pop_front();
  }

  // Correct MapPoints
  // 校正所有地图点位置
  const vector<MapPoint *> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();

  for (size_t i = 0; i < vpMPs.size(); i++) {
    MapPoint *pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    if (pMP->mnBAGlobalForKF == GBAid) {
      // If optimized by Global BA, just update
      // GBA直接优化的点直接更新
      pMP->SetWorldPos(pMP->mPosGBA);
    } else {
      // Update according to the correction of its reference keyframe
      // 未直接优化的点，按参考关键帧的校正量相对校正
      auto pRefKF = pMP->GetReferenceKeyFrame();

      if (pRefKF->mnBAGlobalForKF != GBAid) continue;

      // Map to non-corrected camera
      // 转到参考帧相机坐标系
      Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

      // Backproject using corrected camera
      // 用校正后的相机位姿反投影回世界系
      pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
    }
  }

  oslog::info("Map updated!");

  mnKFs = vpKF.size();
  mIdxInit++;

  // 清空队列里的关键帧，标记为坏帧
  for (auto pKFi : mlNewKeyFrames) {
    pKFi->SetBadFlag();
    // delete *lit;
  }
  mlNewKeyFrames.clear();

  // 跟踪状态设为OK
  mpTracker->mState = Tracking::OK;
  bInitializing = false;

  // 增加地图变化索引
  mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

  return;
}

// ==============================================
// 尺度精化函数
// 功能：纯旋转优化，进一步精化IMU旋转与尺度
// ==============================================
void LocalMapping::ScaleRefinement() {
  // Minimum number of keyframes to compute a solution
  // Minimum time (seconds) between first and last keyframe to compute a
  // solution. Make the difference between monocular and stereo
  // unique_lock<mutex> lock0(mMutexImuInit);
  if (mbResetRequested) return;

  // Retrieve all keyframes in temporal order
  // 按时间顺序获取所有关键帧
  list<std::shared_ptr<KeyFrame>> lpKF;
  auto pKF = mpCurrentKeyFrame;
  while (pKF->mPrevKF) {
    lpKF.push_front(pKF);
    pKF = pKF->mPrevKF;
  }
  lpKF.push_front(pKF);
  vector<std::shared_ptr<KeyFrame>> vpKF(lpKF.begin(), lpKF.end());

  // 处理队列中所有关键帧
  while (CheckNewKeyFrames()) {
    ProcessNewKeyFrame();
    vpKF.push_back(mpCurrentKeyFrame);
    lpKF.push_back(mpCurrentKeyFrame);
  }

  const int N = vpKF.size();

  mRwg = Eigen::Matrix3d::Identity();
  mScale = 1.0;

  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  // 仅惯性优化（尺度与旋转）
  Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  if (mScale < 1e-1) {
    cout << "scale too small" << endl;
    bInitializing = false;
    return;
  }

  Sophus::SO3d so3wg(mRwg);

  // Before this line we are not changing the map
  // 应用尺度旋转到地图
  unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  if ((fabs(mScale - 1.f) > 0.002) || !mbMonocular) {
    Sophus::SE3f Tgw(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
    mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw, mScale, true);
    mpTracker->UpdateFrameIMU(mScale, mpCurrentKeyFrame->GetImuBias(),
                               mpCurrentKeyFrame);
  }
  std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

  // 清空队列
  for (auto pKFi : mlNewKeyFrames) {
    pKFi->SetBadFlag();
    // delete *lit;
  }
  mlNewKeyFrames.clear();

  double t_inertial_only =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
          .count();

  // To perform pose-inertial opt w.r.t. last keyframe
  // 增加地图变化索引
  mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

  return;
}

// ==============================================
// 检查是否正在IMU初始化
// ==============================================
bool LocalMapping::IsInitializing() { return bInitializing; }

// ==============================================
// 获取当前关键帧的时间戳
// ==============================================
double LocalMapping::GetCurrKFTime() {
  if (mpCurrentKeyFrame) {
    return mpCurrentKeyFrame->mTimeStamp;
  } else {
    return 0.0;
  }
}

// ==============================================
// 获取当前关键帧指针
// ==============================================
std::shared_ptr<KeyFrame> LocalMapping::GetCurrKF() {
  return mpCurrentKeyFrame;
}

}  // namespace ORB_SLAM3
