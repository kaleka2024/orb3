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

// 回环检测与闭合模块头文件
#include "LoopClosing.h"
// STL 标准库：通用算法
#include <algorithm>
// STL 标准库：标准输入输出
#include <iostream>
// STL 标准库：双向链表
#include <list>
// STL 标准库：智能指针
#include <memory>
// STL 标准库：互斥锁，线程同步
#include <mutex>
// STL 标准库：集合容器
#include <set>
// STL 标准库：线程
#include <thread>
// STL 标准库：元组
#include <tuple>
// STL 标准库：成对结构
#include <utility>
// STL 标准库：动态数组
#include <vector>

// ORB-SLAM3 内部：类型转换工具集
#include "Converter.h"
// ORB-SLAM3 内部：g2o 图优化类型定义
#include "G2oTypes.h"
// ORB-SLAM3 内部：ORB 特征匹配器
#include "ORBmatcher.h"
// ORB-SLAM3 内部：优化器（BA、Sim3 优化、IMU 优化等）
#include "Optimizer.h"
// ORB-SLAM3 内部：相似变换求解器（RANSAC+优化）
#include "Sim3Solver.h"

namespace ORB_SLAM3 {

// ==============================================
// 回环检测模块构造函数
// ==============================================
LoopClosing::LoopClosing(const std::shared_ptr<Atlas>& pAtlas,
                         const std::shared_ptr<KeyFrameDatabase>& pDB,
                         const std::shared_ptr<ORBVocabulary>& pVoc,
                         const bool bFixScale, const bool bActiveLC)
    : mbResetRequested(false),         // 全局重置请求标志
      mbResetActiveMapRequested(false),// 活动地图重置请求标志
      mbFinishRequested(false),        // 线程结束请求标志
      mbFinished(true),               // 线程已完成标志
      mpAtlas(pAtlas),               // 图集管理器指针
      mpKeyFrameDB(pDB),             // 关键帧数据库指针
      mpORBVocabulary(pVoc),         // ORB 词典指针
      mpMatchedKF(NULL),              // 匹配到的回环/合并关键帧
      mLastLoopKFid(0),               // 上一次回环的关键帧 ID
      mbRunningGBA(false),            // 全局 BA 正在运行标志
      mbFinishedGBA(true),            // 全局 BA 已完成标志
      mbStopGBA(false),               // 停止全局 BA 标志
      mpThreadGBA(NULL),              // 全局 BA 线程指针
      mbFixScale(bFixScale),          // 是否固定尺度
      mnFullBAIdx(0),                // 全局 BA 索引计数
      mnLoopNumCoincidences(0),       // 回环连续命中次数
      mnMergeNumCoincidences(0),      // 合并连续命中次数
      mbLoopDetected(false),          // 回环检测成功标志
      mbMergeDetected(false),         // 合并检测成功标志
      mnLoopNumNotFound(0),           // 回环连续未命中次数
      mnMergeNumNotFound(0),          // 合并连续未命中次数
      mbActiveLC(bActiveLC),          // 回环检测激活标志
      mpLastCurrentKF() {             // 上一个处理的当前关键帧
  mnCovisibilityConsistencyTh = 3;    // 共视一致性阈值：连续 3 次命中才确认

#ifdef REGISTER_TIMES
  // 各类耗时统计容器初始化
  vdDataQuery_ms.clear();
  vdEstSim3_ms.clear();
  vdPRTotal_ms.clear();

  vdMergeMaps_ms.clear();
  vdWeldingBA_ms.clear();
  vdMergeOptEss_ms.clear();
  vdMergeTotal_ms.clear();
  vnMergeKFs.clear();
  vnMergeMPs.clear();
  nMerges = 0;

  vdLoopFusion_ms.clear();
  vdLoopOptEss_ms.clear();
  vdLoopTotal_ms.clear();
  vnLoopKFs.clear();
  nLoop = 0;

  vdGBA_ms.clear();
  vdUpdateMap_ms.clear();
  vdFGBATotal_ms.clear();
  vnGBAKFs.clear();
  vnGBAMPs.clear();
  nFGBA_exec = 0;
  nFGBA_abort = 0;
#endif

  mstrFolderSubTraj = "SubTrajectories/";
  mnNumCorrection = 0;              // 回环校正次数计数
  mnCorrectionGBA = 0;             // 全局 BA 校正次数计数
}

// ==============================================
// 设置跟踪模块指针
// ==============================================
void LoopClosing::SetTracker(const std::shared_ptr<Tracking>& pTracker) {
  mpTracker = pTracker;
}

// ==============================================
// 设置局部建图模块指针
// ==============================================
void LoopClosing::SetLocalMapper(
    const std::shared_ptr<LocalMapping>& pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

// ==============================================
// 回环检测主线程主循环
// ==============================================
void LoopClosing::Run() {
  mbFinished = false;

  while (true) {
    oslog::trace("@@ LoopClosing: loop running...");

    // 队列中有新关键帧则处理
    if (CheckNewKeyFrames()) {
      oslog::info("LoopClosing: Have new frames to check");

      // 清空上一个关键帧的候选列表
      if (mpLastCurrentKF) {
        mpLastCurrentKF->mvpLoopCandKFs.clear();
        mpLastCurrentKF->mvpMergeCandKFs.clear();
      }

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_StartPR =
          std::chrono::steady_clock::now();
#endif

      // 检测公共区域：区分回环（同地图）和合并（跨地图）
      bool bFindedRegion = NewDetectCommonRegions();

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_EndPR =
          std::chrono::steady_clock::now();

      double timePRTotal =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndPR - time_StartPR)
              .count();
      vdPRTotal_ms.push_back(timePRTotal);
#endif

      if (bFindedRegion) {
        // ========== 处理合并检测 ==========
        if (mbMergeDetected) {
          if ((mpTracker->mSensor.isImu()) &&
              (!mpCurrentKF->GetMap()->isImuInitialized())) {
            cout << "IMU is not initilized, merge is aborted" << endl;
          } else {
            // 计算合并两边的相似变换
            Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
            g2o::Sim3 gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
            Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
            g2o::Sim3 gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
            g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();

            // 计算当前帧到匹配帧的相似变换
            mSold_new = (gSw2c * gScw1);

            // 惯性模式下做尺度和旋转校验
            if (mpCurrentKF->GetMap()->IsInertial() &&
                mpMergeMatchedKF->GetMap()->IsInertial()) {
              cout << "Merge check transformation with IMU" << endl;
              // 尺度偏差超过 10% 则判定合并错误
              if (mSold_new.scale() < 0.90 || mSold_new.scale() > 1.1) {
                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
                mbMergeDetected = false;
                oslog::warn("scale bad estimated. Abort merging");
                continue;
              }

              // 惯性模式且已过第一阶段 BA，只优化偏航角
              if ((mpTracker->mSensor.isImu()) &&
                  mpCurrentKF->GetMap()->GetInertialBA1()) {
                Eigen::Vector3d phi =
                    LogSO3(mSold_new.rotation().toRotationMatrix());
                phi(0) = 0;
                phi(1) = 0;
                mSold_new =
                    g2o::Sim3(ExpSO3(phi), mSold_new.translation(), 1.0);
              }
            }

            // 更新合并相似变换
            mg2oMergeSmw = gSmw2 * gSw2c * gScw1;
            mg2oMergeScw = mg2oMergeSlw;

            oslog::info("*Merge detected");

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartMerge =
                std::chrono::steady_clock::now();
            nMerges += 1;
#endif

            // IMU 模式调用 MergeLocal2，纯视觉调用 MergeLocal
            if (mpTracker->mSensor.isImu())
              MergeLocal2();
            else
              MergeLocal();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMerge =
                std::chrono::steady_clock::now();
            double timeMergeTotal =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(time_EndMerge -
                                                                time_StartMerge)
                    .count();
            vdMergeTotal_ms.push_back(timeMergeTotal);
#endif

            oslog::info("Merge finished!");
          }

          // 记录检测时间
          vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
          vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
          vnPR_TypeRecogn.push_back(1);

          // 重置合并相关变量
          mpMergeLastCurrentKF->SetErase();
          mpMergeMatchedKF->SetErase();
          mnMergeNumCoincidences = 0;
          mvpMergeMatchedMPs.clear();
          mvpMergeMPs.clear();
          mnMergeNumNotFound = 0;
          mbMergeDetected = false;

          if (mbLoopDetected) {
            mpLoopLastCurrentKF->SetErase();
            mpLoopMatchedKF->SetErase();
            mnLoopNumCoincidences = 0;
            mvpLoopMatchedMPs.clear();
            mvpLoopMPs.clear();
            mnLoopNumNotFound = 0;
            mbLoopDetected = false;
          }
        }

        // ========== 处理回环检测 ==========
        if (mbLoopDetected) {
          bool bGoodLoop = true;
          vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
          vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
          vnPR_TypeRecogn.push_back(0);

          oslog::debug("*Loop detected");

          // 保存回环相似变换
          mg2oLoopScw = mg2oLoopSlw;

          // 惯性模式下做旋转校验
          if (mpCurrentKF->GetMap()->IsInertial()) {
            Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
            g2o::Sim3 g2oTwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
            g2o::Sim3 g2oSww_new = g2oTwc * mg2oLoopScw;

            Eigen::Vector3d phi =
                LogSO3(g2oSww_new.rotation().toRotationMatrix());
            cout << "phi = " << phi.transpose() << endl;

            if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f &&
                fabs(phi(2)) < 0.349f) {
              if (mpCurrentKF->GetMap()->IsInertial()) {
                if ((mpTracker->mSensor.isImu()) &&
                    mpCurrentKF->GetMap()->GetInertialBA2()) {
                  phi(0) = 0;
                  phi(1) = 0;
                  g2oSww_new =
                      g2o::Sim3(ExpSO3(phi), g2oSww_new.translation(), 1.0);
                  mg2oLoopScw = g2oTwc.inverse() * g2oSww_new;
                }
              }
            } else {
              cout << "BAD LOOP!!!" << endl;
              bGoodLoop = false;
            }
          }

          if (bGoodLoop) {
            mvpLoopMapPoints = mvpLoopMPs;

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartLoop =
                std::chrono::steady_clock::now();
            nLoop += 1;
#endif

            CorrectLoop();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLoop =
                std::chrono::steady_clock::now();
            double timeLoopTotal =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(time_EndLoop -
                                                                time_StartLoop)
                    .count();
            vdLoopTotal_ms.push_back(timeLoopTotal);
#endif

            mnNumCorrection += 1;
          }

          mpLoopLastCurrentKF->SetErase();
          mpLoopMatchedKF->SetErase();
          mnLoopNumCoincidences = 0;
          mvpLoopMatchedMPs.clear();
          mvpLoopMPs.clear();
          mnLoopNumNotFound = 0;
          mbLoopDetected = false;
        }
      }

      mpLastCurrentKF = mpCurrentKF;
    }

    ResetIfRequested();

    if (CheckFinish()) {
      break;
    }

    usleep(500);
  }

  SetFinish();
}

// ==============================================
// 向回环检测队列插入关键帧
// ==============================================
void LoopClosing::InsertKeyFrame(const std::shared_ptr<KeyFrame>& pKF) {
  unique_lock<mutex> lock(mMutexLoopQueue);
  if (pKF->mnId != 0) {
    mlpLoopKeyFrameQueue.push_back(pKF);
    oslog::info("LoopClosing: Pushing KF {} into queue, which is length {}",
                pKF->mnId, mlpLoopKeyFrameQueue.size());
  }
}

// ==============================================
// 检查队列中是否有新关键帧
// ==============================================
bool LoopClosing::CheckNewKeyFrames() {
  unique_lock<mutex> lock(mMutexLoopQueue);
  return (!mlpLoopKeyFrameQueue.empty());
}

// ==============================================
// 新公共区域检测算法
// ==============================================
bool LoopClosing::NewDetectCommonRegions() {
  if (!mbActiveLC) {
    return false;
  }

  {
    unique_lock<mutex> lock(mMutexLoopQueue);
    mpCurrentKF = mlpLoopKeyFrameQueue.front();
    mlpLoopKeyFrameQueue.pop_front();
    mpCurrentKF->SetNotErase();
    mpCurrentKF->mbCurrentPlaceRecognition = true;

    mpLastMap = mpCurrentKF->GetMap();
  }

  // 惯性模式且未到第二阶段 BA，不做回环检测
  if (mpLastMap->IsInertial() && !mpLastMap->GetInertialBA2()) {
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  // 双目模式且关键帧太少，不检测
  if (mpTracker->mSensor == SensorType::STEREO &&
      mpLastMap->GetAllKeyFrames().size() < 5) {
    oslog::info(
        "[LoopClosing::NewDetectCommonRegions] Stereo KF inserted without "
        "check: {}",
        mpCurrentKF->mnId);
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  // 地图关键帧太少，不检测
  if (mpLastMap->GetAllKeyFrames().size() < 12) {
    oslog::info(
        "[LoopClosing::NewDetectCommonRegions] Stereo KF inserted without "
        "check, map is small: {}",
        mpCurrentKF->mnId);
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  oslog::info("[LoopClosing::NewDetectCommonRegions] Checking KF: {}",
              mpCurrentKF->mnId);

  bool bLoopDetectedInKF = false;

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartEstSim3_1 =
      std::chrono::steady_clock::now();
#endif

  // ========== 回环候选：几何跟踪验证 ==========
  if (mnLoopNumCoincidences > 0) {
    Sophus::SE3d mTcl =
        (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse())
            .cast<double>();
    g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
    g2o::Sim3 gScw = gScl * mg2oLoopSlw;

    int numProjMatches = 0;
    vector<MapPoint*> vpMatchedMPs;
    bool bCommonRegion = DetectAndReffineSim3FromLastKF(
        mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs,
        vpMatchedMPs);

    if (bCommonRegion) {
      bLoopDetectedInKF = true;
      mnLoopNumCoincidences++;
      mpLoopLastCurrentKF->SetErase();
      mpLoopLastCurrentKF = mpCurrentKF;
      mg2oLoopSlw = gScw;
      mvpLoopMatchedMPs = vpMatchedMPs;

      mbLoopDetected = mnLoopNumCoincidences >= 3;
      mnLoopNumNotFound = 0;

      if (!mbLoopDetected) {
        oslog::info("PR: Loop detected with Reffine Sim3");
      }
    } else {
      bLoopDetectedInKF = false;
      mnLoopNumNotFound++;
      if (mnLoopNumNotFound >= 2) {
        mpLoopLastCurrentKF->SetErase();
        mpLoopMatchedKF->SetErase();
        mnLoopNumCoincidences = 0;
        mvpLoopMatchedMPs.clear();
        mvpLoopMPs.clear();
        mnLoopNumNotFound = 0;
      }
    }
  }

  // ========== 合并候选：几何跟踪验证 ==========
  bool bMergeDetectedInKF = false;
  if (mnMergeNumCoincidences > 0) {
    Sophus::SE3d mTcl =
        (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse())
            .cast<double>();

    g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
    g2o::Sim3 gScw = gScl * mg2oMergeSlw;

    int numProjMatches = 0;
    vector<MapPoint*> vpMatchedMPs;
    bool bCommonRegion = DetectAndReffineSim3FromLastKF(
        mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs,
        vpMatchedMPs);

    if (bCommonRegion) {
      bMergeDetectedInKF = true;
      mnMergeNumCoincidences++;
      mpMergeLastCurrentKF->SetErase();
      mpMergeLastCurrentKF = mpCurrentKF;
      mg2oMergeSlw = gScw;
      mvpMergeMatchedMPs = vpMatchedMPs;

      mbMergeDetected = mnMergeNumCoincidences >= 3;
    } else {
      mbMergeDetected = false;
      bMergeDetectedInKF = false;
      mnMergeNumNotFound++;
      if (mnMergeNumNotFound >= 2) {
        mpMergeLastCurrentKF->SetErase();
        mpMergeMatchedKF->SetErase();
        mnMergeNumCoincidences = 0;
        mvpMergeMatchedMPs.clear();
        mvpMergeMPs.clear();
        mnMergeNumNotFound = 0;
      }
    }
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndEstSim3_1 =
      std::chrono::steady_clock::now();
  double timeEstSim3 =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndEstSim3_1 - time_StartEstSim3_1)
          .count();
#endif

  if (mbMergeDetected || mbLoopDetected) {
#ifdef REGISTER_TIMES
    vdEstSim3_ms.push_back(timeEstSim3);
#endif
    mpKeyFrameDB->add(mpCurrentKF);
    return true;
  }

  const vector<std::shared_ptr<KeyFrame>> vpConnectedKeyFrames =
      mpCurrentKF->GetVectorCovisibleKeyFrames();

  vector<std::shared_ptr<KeyFrame>> vpMergeBowCand, vpLoopBowCand;
  if (!bMergeDetectedInKF || !bLoopDetectedInKF) {
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartQuery =
        std::chrono::steady_clock::now();
#endif
    mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand,
                                        vpMergeBowCand, 3);
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndQuery =
        std::chrono::steady_clock::now();
    double timeDataQuery =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndQuery - time_StartQuery)
            .count();
    vdDataQuery_ms.push_back(timeDataQuery);
#endif
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartEstSim3_2 =
      std::chrono::steady_clock::now();
#endif

  if (!bLoopDetectedInKF && !vpLoopBowCand.empty()) {
    mbLoopDetected = DetectCommonRegionsFromBoW(
        vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw,
        mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
  }
  if (!bMergeDetectedInKF && !vpMergeBowCand.empty()) {
    mbMergeDetected = DetectCommonRegionsFromBoW(
        vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw,
        mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndEstSim3_2 =
      std::chrono::steady_clock::now();
  timeEstSim3 +=
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndEstSim3_2 - time_StartEstSim3_2)
          .count();
  vdEstSim3_ms.push_back(timeEstSim3);
#endif

  mpKeyFrameDB->add(mpCurrentKF);

  if (mbMergeDetected || mbLoopDetected) {
    return true;
  }

  mpCurrentKF->SetErase();
  mpCurrentKF->mbCurrentPlaceRecognition = false;

  return false;
}

// ==============================================
// 基于上一帧的 Sim3，投影匹配并优化
// ==============================================
bool LoopClosing::DetectAndReffineSim3FromLastKF(
    const std::shared_ptr<KeyFrame>& pCurrentKF,
    std::shared_ptr<KeyFrame>& pMatchedKF, g2o::Sim3& gScw,
    int& nNumProjMatches, std::vector<MapPoint*>& vpMPs,
    std::vector<MapPoint*>& vpMatchedMPs) {
  set<MapPoint*> spAlreadyMatchedMPs;
  nNumProjMatches = FindMatchesByProjection(
      pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

  int nProjMatches = 30;
  int nProjOptMatches = 50;
  int nProjMatchesRep = 100;

  if (nNumProjMatches >= nProjMatches) {
    Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
    g2o::Sim3 gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
    g2o::Sim3 gScm = gScw * gSwm;
    Eigen::Matrix<double, 7, 7> mHessian7x7;

    bool bFixedScale = mbFixScale;
    if (mpTracker->mSensor == SensorType::IMU_MONOCULAR &&
        !pCurrentKF->GetMap()->GetInertialBA2())
      bFixedScale = false;

    int numOptMatches =
        Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10,
                                bFixedScale, mHessian7x7, true);

    if (numOptMatches > nProjOptMatches) {
      g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(), 1.0);

      vector<MapPoint*> vpMatchedMP;
      vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(),
                         static_cast<MapPoint*>(NULL));

      nNumProjMatches =
          FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation,
                                  spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

      if (nNumProjMatches >= nProjMatchesRep) {
        gScw = gScw_estimation;
        return true;
      }
    }
  }
  return false;
}

// ==============================================
// 从词袋候选中检测公共区域
// ==============================================
bool LoopClosing::DetectCommonRegionsFromBoW(
    std::vector<std::shared_ptr<KeyFrame>>& vpBowCand,
    std::shared_ptr<KeyFrame>& pMatchedKF2,
    std::shared_ptr<KeyFrame>& pLastCurrentKF, g2o::Sim3& g2oScw,
    int& nNumCoincidences, std::vector<MapPoint*>& vpMPs,
    std::vector<MapPoint*>& vpMatchedMPs) {
  int nBoWMatches = 20;
  int nBoWInliers = 15;
  int nSim3Inliers = 20;
  int nProjMatches = 50;
  int nProjOptMatches = 80;

  set<std::shared_ptr<KeyFrame>> spConnectedKeyFrames =
      mpCurrentKF->GetConnectedKeyFrames();

  int nNumCovisibles = 10;

  ORBmatcher matcherBoW(0.9, true);
  ORBmatcher matcher(0.75, true);

  std::shared_ptr<KeyFrame> pBestMatchedKF;
  int nBestMatchesReproj = 0;
  int nBestNumCoindicendes = 0;
  g2o::Sim3 g2oBestScw;
  std::vector<MapPoint*> vpBestMapPoints;
  std::vector<MapPoint*> vpBestMatchedMapPoints;

  int numCandidates = vpBowCand.size();
  vector<int> vnStage(numCandidates, 0);
  vector<int> vnMatchesStage(numCandidates, 0);

  int index = 0;

  for (auto const& pKFi : vpBowCand) {
    if (!pKFi || pKFi->isBad()) continue;

    std::vector<std::shared_ptr<KeyFrame>> vpCovKFi =
        pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
    if (vpCovKFi.empty()) {
      std::cout << "Covisible list empty" << std::endl;
      vpCovKFi.push_back(pKFi);
    } else {
      vpCovKFi.push_back(vpCovKFi[0]);
      vpCovKFi[0] = pKFi;
    }

    bool bAbortByNearKF = false;
    for (size_t j = 0; j < vpCovKFi.size(); ++j) {
      if (spConnectedKeyFrames.find(vpCovKFi[j]) !=
          spConnectedKeyFrames.end()) {
        bAbortByNearKF = true;
        break;
      }
    }
    if (bAbortByNearKF) {
      continue;
    }

    std::vector<std::vector<MapPoint*>> vvpMatchedMPs;
    vvpMatchedMPs.resize(vpCovKFi.size());
    std::set<MapPoint*> spMatchedMPi;
    int numBoWMatches = 0;

    std::shared_ptr<KeyFrame> pMostBoWMatchesKF = pKFi;
    int nMostBoWNumMatches = 0;

    std::vector<MapPoint*> vpMatchedPoints = std::vector<MapPoint*>(
        mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
    std::vector<std::shared_ptr<KeyFrame>> vpKeyFrameMatchedMP =
        std::vector<std::shared_ptr<KeyFrame>>(
            mpCurrentKF->GetMapPointMatches().size(), nullptr);

    for (size_t j = 0; j < vpCovKFi.size(); ++j) {
      if (!vpCovKFi[j] || vpCovKFi[j]->isBad()) continue;

      int num =
          matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
      if (num > nMostBoWNumMatches) {
        nMostBoWNumMatches = num;
      }
    }

    for (size_t j = 0; j < vpCovKFi.size(); ++j) {
      for (size_t k = 0; k < vvpMatchedMPs[j].size(); ++k) {
        MapPoint* pMPi_j = vvpMatchedMPs[j][k];
        if (!pMPi_j || pMPi_j->isBad()) continue;

        if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end()) {
          spMatchedMPi.insert(pMPi_j);
          numBoWMatches++;

          vpMatchedPoints[k] = pMPi_j;
          vpKeyFrameMatchedMP[k] = vpCovKFi[j];
        }
      }
    }

    if (numBoWMatches >= nBoWMatches) {
      bool bFixedScale = mbFixScale;
      if (mpTracker->mSensor == SensorType::IMU_MONOCULAR &&
          !mpCurrentKF->GetMap()->GetInertialBA2())
        bFixedScale = false;

      Sim3Solver solver =
          Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints,
                     bFixedScale, vpKeyFrameMatchedMP);
      solver.SetRansacParameters(0.99, nBoWInliers, 300);

      bool bNoMore = false;
      vector<bool> vbInliers;
      int nInliers;
      bool bConverge = false;
      Eigen::Matrix4f mTcm;

      while (!bConverge && !bNoMore) {
        mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
      }

      if (bConverge) {
        vpCovKFi.clear();
        vpCovKFi =
            pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
        vpCovKFi.push_back(pMostBoWMatchesKF);
        set<std::shared_ptr<KeyFrame>> spCheckKFs(vpCovKFi.begin(),
                                                   vpCovKFi.end());

        set<MapPoint*> spMapPoints;
        vector<MapPoint*> vpMapPoints;
        vector<std::shared_ptr<KeyFrame>> vpKeyFrames;
        for (auto pCovKFi : vpCovKFi) {
          for (MapPoint* pCovMPij : pCovKFi->GetMapPointMatches()) {
            if (!pCovMPij || pCovMPij->isBad()) continue;

            if (spMapPoints.find(pCovMPij) == spMapPoints.end()) {
              spMapPoints.insert(pCovMPij);
              vpMapPoints.push_back(pCovMPij);
              vpKeyFrames.push_back(pCovKFi);
            }
          }
        }

        g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),
                       solver.GetEstimatedTranslation().cast<double>(),
                       static_cast<double>(solver.GetEstimatedScale()));
        g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),
                       pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
        g2o::Sim3 gScw = gScm * gSmw;
        Sophus::Sim3f mScw = Converter::toSophus(gScw);

        vector<MapPoint*> vpMatchedMP;
        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size());
        vector<std::shared_ptr<KeyFrame>> vpMatchedKF;
        vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size());
        int numProjMatches = matcher.SearchByProjection(
            mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP,
            vpMatchedKF, 8, 1.5);

        if (numProjMatches >= nProjMatches) {
          Eigen::Matrix<double, 7, 7> mHessian7x7;

          int numOptMatches =
              Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10,
                                      mbFixScale, mHessian7x7, true);

          if (numOptMatches >= nSim3Inliers) {
            g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),
                           pMostBoWMatchesKF->GetTranslation().cast<double>(),
                           1.0);
            g2o::Sim3 gScw = gScm * gSmw;
            Sophus::Sim3f mScw = Converter::toSophus(gScw);

            vector<MapPoint*> vpMatchedMP;
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(),
                                static_cast<MapPoint*>(NULL));
            int numProjOptMatches = matcher.SearchByProjection(
                mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

            if (numProjOptMatches >= nProjOptMatches) {
              int max_x = -1, min_x = 100000;
              int max_y = -1, min_y = 100000;
              for (MapPoint* pMPi : vpMatchedMP) {
                if (!pMPi || pMPi->isBad()) {
                  continue;
                }

                tuple<size_t, size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
                int index = get<0>(indexes);
                if (index >= 0) {
                  int coord_x = pKFi->mvKeysUn[index].pt.x;
                  if (coord_x < min_x) min_x = coord_x;
                  if (coord_x > max_x) max_x = coord_x;
                  int coord_y = pKFi->mvKeysUn[index].pt.y;
                  if (coord_y < min_y) min_y = coord_y;
                  if (coord_y > max_y) max_y = coord_y;
                }
              }

              int nNumKFs = 0;
              vector<std::shared_ptr<KeyFrame>> vpCurrentCovKFs =
                  mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

              size_t j = 0;
              while (nNumKFs < 3 && j < vpCurrentCovKFs.size()) {
                std::shared_ptr<KeyFrame> pKFj = vpCurrentCovKFs[j];
                Sophus::SE3d mTjc =
                    (pKFj->GetPose() * mpCurrentKF->GetPoseInverse())
                        .cast<double>();
                g2o::Sim3 gSjc(mTjc.unit_quaternion(), mTjc.translation(), 1.0);
                g2o::Sim3 gSjw = gSjc * gScw;
                int numProjMatches_j = 0;
                vector<MapPoint*> vpMatchedMPs_j;
                bool bValid = DetectCommonRegionsFromLastKF(
                    pKFj, pMostBoWMatchesKF, gSjw, numProjMatches_j,
                    vpMapPoints, vpMatchedMPs_j);

                if (bValid) {
                  Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
                  Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
                  Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                  Eigen::Vector3f vector_dist = Tc_cj.translation();
                  nNumKFs++;
                }
                j++;
              }

              if (nBestMatchesReproj < numProjOptMatches) {
                nBestMatchesReproj = numProjOptMatches;
                nBestNumCoindicendes = nNumKFs;
                pBestMatchedKF = pMostBoWMatchesKF;
                g2oBestScw = gScw;
                vpBestMapPoints = vpMapPoints;
                vpBestMatchedMapPoints = vpMatchedMP;
              }
            }
          }
        }
      }
    }
    index++;
  }

  if (nBestMatchesReproj > 0) {
    pLastCurrentKF = mpCurrentKF;
    nNumCoincidences = nBestNumCoindicendes;
    pMatchedKF2 = pBestMatchedKF;
    pMatchedKF2->SetNotErase();
    g2oScw = g2oBestScw;
    vpMPs = vpBestMapPoints;
    vpMatchedMPs = vpBestMatchedMapPoints;

    return nNumCoincidences >= 3;
  }
  return false;
}

// ==============================================
// 基于上一关键帧的 Sim3，投影匹配验证是否同一区域
// ==============================================
bool LoopClosing::DetectCommonRegionsFromLastKF(
    const std::shared_ptr<KeyFrame>& pCurrentKF,
    const std::shared_ptr<KeyFrame>& pMatchedKF, g2o::Sim3& gScw,
    int& nNumProjMatches, std::vector<MapPoint*>& vpMPs,
    std::vector<MapPoint*>& vpMatchedMPs) {
  set<MapPoint*> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
  nNumProjMatches = FindMatchesByProjection(
      pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

  const int nProjMatches = 30;
  if (nNumProjMatches >= nProjMatches) {
    return true;
  }

  return false;
}

// ==============================================
// 通过 Sim3 投影查找匹配的地图点
// ==============================================
int LoopClosing::FindMatchesByProjection(
    const std::shared_ptr<KeyFrame>& pCurrentKF,
    const std::shared_ptr<KeyFrame>& pMatchedKFw, g2o::Sim3& g2oScw,
    set<MapPoint*>& spMatchedMPinOrigin, vector<MapPoint*>& vpMapPoints,
    vector<MapPoint*>& vpMatchedMapPoints) {
  int nNumCovisibles = 10;
  vector<std::shared_ptr<KeyFrame>> vpCovKFm =
      pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
  int nInitialCov = vpCovKFm.size();
  vpCovKFm.push_back(pMatchedKFw);
  set<std::shared_ptr<KeyFrame>> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
  set<std::shared_ptr<KeyFrame>> spCurrentCovisbles =
      pCurrentKF->GetConnectedKeyFrames();

  if (nInitialCov < nNumCovisibles) {
    for (int i = 0; i < nInitialCov; ++i) {
      vector<std::shared_ptr<KeyFrame>> vpKFs =
          vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
      int nInserted = 0;

      size_t j = 0;
      while (j < vpKFs.size() && nInserted < nNumCovisibles) {
        if (spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() &&
            spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end()) {
          spCheckKFs.insert(vpKFs[j]);
          ++nInserted;
        }
        ++j;
      }
      vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
    }
  }

  set<MapPoint*> spMapPoints;
  vpMapPoints.clear();
  vpMatchedMapPoints.clear();
  for (auto pKFi : vpCovKFm) {
    for (MapPoint* pMPij : pKFi->GetMapPointMatches()) {
      if (!pMPij || pMPij->isBad()) continue;

      if (spMapPoints.find(pMPij) == spMapPoints.end()) {
        spMapPoints.insert(pMPij);
        vpMapPoints.push_back(pMPij);
      }
    }
  }

  Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
  ORBmatcher matcher(0.9, true);

  vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(),
                             static_cast<MapPoint*>(NULL));
  int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints,
                                                vpMatchedMapPoints, 3, 1.5);

  return num_matches;
}

// ==============================================
// 执行回环校正
// ==============================================
void LoopClosing::CorrectLoop() {
  mpLocalMapper->RequestStop();
  mpLocalMapper->EmptyQueue();

  if (isRunningGBA()) {
    cout << "Stoping Global Bundle Adjustment...";
    unique_lock<mutex> lock(mMutexGBA);
    mbStopGBA = true;

    mnFullBAIdx++;

    if (mpThreadGBA) {
      mpThreadGBA->detach();
      delete mpThreadGBA;
    }
    cout << "  Done!!" << endl;
  }

  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }

  mpCurrentKF->UpdateConnections();

  mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
  mvpCurrentConnectedKFs.push_back(mpCurrentKF);

  KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
  CorrectedSim3[mpCurrentKF] = mg2oLoopScw;
  Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
  Sophus::SE3f Tcw = mpCurrentKF->GetPose();
  g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(),
                  Tcw.translation().cast<double>(), 1.0);
  NonCorrectedSim3[mpCurrentKF] = g2oScw;

  Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(),
                             mg2oLoopScw.translation() / mg2oLoopScw.scale());
  mpCurrentKF->SetPose(correctedTcw.cast<float>());

  std::shared_ptr<Map> pLoopMap = mpCurrentKF->GetMap();

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartFusion =
      std::chrono::steady_clock::now();
#endif

  {
    unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

    const bool bImuInit = pLoopMap->isImuInitialized();

    for (auto pKFi : mvpCurrentConnectedKFs) {
      if (pKFi != mpCurrentKF) {
        Sophus::SE3f Tiw = pKFi->GetPose();
        Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
        g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
        g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;
        CorrectedSim3[pKFi] = g2oCorrectedSiw;

        Sophus::SE3d correctedTiw(
            g2oCorrectedSiw.rotation(),
            g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
        pKFi->SetPose(correctedTiw.cast<float>());

        g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(),
                          Tiw.translation().cast<double>(), 1.0);
        NonCorrectedSim3[pKFi] = g2oSiw;
      }
    }

    for (auto const& [pKFi, g2oCorrectedSiw] : CorrectedSim3) {
      g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

      g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

      vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
      for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++) {
        MapPoint* pMPi = vpMPsi[iMP];
        if (!pMPi) continue;
        if (pMPi->isBad()) continue;
        if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId) continue;

        Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
        Eigen::Vector3d eigCorrectedP3Dw =
            g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

        pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
        pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
        pMPi->mnCorrectedReference = pKFi->mnId;
        pMPi->UpdateNormalAndDepth();
      }

      if (bImuInit) {
        Eigen::Quaternionf Rcor =
            (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation())
                .cast<float>();
        pKFi->SetVelocity(Rcor * pKFi->GetVelocity());
      }

      pKFi->UpdateConnections();
    }

    mpAtlas->GetCurrentMap()->IncreaseChangeIndex();

    for (size_t i = 0; i < mvpLoopMatchedMPs.size(); i++) {
      if (mvpLoopMatchedMPs[i]) {
        MapPoint* pLoopMP = mvpLoopMatchedMPs[i];
        MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
        if (pCurMP) {
          pCurMP->Replace(pLoopMP);
        } else {
          mpCurrentKF->AddMapPoint(pLoopMP, i);
          pLoopMP->AddObservation(mpCurrentKF, i);
          pLoopMP->ComputeDistinctiveDescriptors();
        }
      }
    }
  }

  SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

  map<std::shared_ptr<KeyFrame>, set<std::shared_ptr<KeyFrame>>>
      LoopConnections;

  for (auto pKFi : mvpCurrentConnectedKFs) {
    auto vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

    pKFi->UpdateConnections();
    LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
    for (auto vit_prev : vpPreviousNeighbors) {
      LoopConnections[pKFi].erase(vit_prev);
    }
    for (auto vit2 : mvpCurrentConnectedKFs) {
      LoopConnections[pKFi].erase(vit2);
    }
  }

  bool bFixedScale = mbFixScale;
  if (mpTracker->mSensor == SensorType::IMU_MONOCULAR &&
      !mpCurrentKF->GetMap()->GetInertialBA2())
    bFixedScale = false;

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndFusion =
      std::chrono::steady_clock::now();
  double timeFusion =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndFusion - time_StartFusion)
          .count();
  vdLoopFusion_ms.push_back(timeFusion);
#endif

  // 惯性模式用 4 自由度本质图优化，纯视觉用 7 自由度
  if (pLoopMap->IsInertial() && pLoopMap->isImuInitialized()) {
    Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF,
                                          mpCurrentKF, NonCorrectedSim3,
                                          CorrectedSim3, LoopConnections);
  } else {
    Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF,
                                      NonCorrectedSim3, CorrectedSim3,
                                      LoopConnections, bFixedScale);
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndOpt =
      std::chrono::steady_clock::now();
  double timeOptEss =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndOpt - time_EndFusion)
          .count();
  vdLoopOptEss_ms.push_back(timeOptEss);
#endif

  mpAtlas->InformNewBigChange();

  mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
  mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

  if (!pLoopMap->isImuInitialized() ||
      (pLoopMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1)) {
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mnCorrectionGBA = mnNumCorrection;

    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,
                               shared_from_this(), pLoopMap, mpCurrentKF->mnId);
  }

  mpLocalMapper->Release();

  mLastLoopKFid = mpCurrentKF->mnId;
}

// ==============================================
// 纯视觉模式下的地图合并
// ==============================================
void LoopClosing::MergeLocal() {
  const int numTemporalKFs = 25;

  std::shared_ptr<KeyFrame> pNewChild;
  std::shared_ptr<KeyFrame> pNewParent;

  vector<std::shared_ptr<KeyFrame>> vpLocalCurrentWindowKFs;
  vector<std::shared_ptr<KeyFrame>> vpMergeConnectedKFs;

  bool bRelaunchBA = false;

  if (isRunningGBA()) {
    unique_lock<mutex> lock(mMutexGBA);
    mbStopGBA = true;

    mnFullBAIdx++;

    if (mpThreadGBA) {
      mpThreadGBA->detach();
      delete mpThreadGBA;
    }
    bRelaunchBA = true;
  }

  mpLocalMapper->RequestStop();
  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }

  mpLocalMapper->EmptyQueue();

  std::shared_ptr<Map> pCurrentMap = mpCurrentKF->GetMap();
  std::shared_ptr<Map> pMergeMap = mpMergeMatchedKF->GetMap();

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartMerge =
      std::chrono::steady_clock::now();
#endif

  mpCurrentKF->UpdateConnections();

  set<std::shared_ptr<KeyFrame>> spLocalWindowKFs;
  set<MapPoint*> spLocalWindowMPs;

  if (pCurrentMap->IsInertial() && pMergeMap->IsInertial()) {
    std::shared_ptr<KeyFrame> pKFi = mpCurrentKF;
    int nInserted = 0;
    while (pKFi && nInserted < numTemporalKFs) {
      spLocalWindowKFs.insert(pKFi);
      pKFi = mpCurrentKF->mPrevKF;
      nInserted++;

      set<MapPoint*> spMPi = pKFi->GetMapPoints();
      spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
    }

    pKFi = mpCurrentKF->mNextKF;
    while (pKFi) {
      spLocalWindowKFs.insert(pKFi);
      set<MapPoint*> spMPi = pKFi->GetMapPoints();
      spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
      pKFi = mpCurrentKF->mNextKF;
    }
  } else {
    spLocalWindowKFs.insert(mpCurrentKF);
  }

  vector<std::shared_ptr<KeyFrame>> vpCovisibleKFs =
      mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
  spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
  spLocalWindowKFs.insert(mpCurrentKF);

  const int nMaxTries = 5;
  int nNumTries = 0;
  while (spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries) {
    vector<std::shared_ptr<KeyFrame>> vpNewCovKFs;
    for (auto pKFi : spLocalWindowKFs) {
      vector<std::shared_ptr<KeyFrame>> vpKFiCov =
          pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);
      for (auto pKFcov : vpKFiCov) {
        if (pKFcov && !pKFcov->isBad() &&
            spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end()) {
          vpNewCovKFs.push_back(pKFcov);
        }
      }
    }
    spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
    nNumTries++;
  }

  for (auto pKFi : spLocalWindowKFs) {
    if (!pKFi || pKFi->isBad()) continue;
    set<MapPoint*> spMPs = pKFi->GetMapPoints();
    spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
  }

  set<std::shared_ptr<KeyFrame>> spMergeConnectedKFs;
  if (pCurrentMap->IsInertial() && pMergeMap->IsInertial()) {
    auto pKFi = mpMergeMatchedKF;
    int nInserted = 0;
    while (pKFi && nInserted < numTemporalKFs / 2) {
      spMergeConnectedKFs.insert(pKFi);
      pKFi = mpCurrentKF->mPrevKF;
      nInserted++;
    }

    pKFi = mpMergeMatchedKF->mNextKF;
    while (pKFi && nInserted < numTemporalKFs) {
      spMergeConnectedKFs.insert(pKFi);
      pKFi = mpCurrentKF->mNextKF;
    }
  } else {
    spMergeConnectedKFs.insert(mpMergeMatchedKF);
  }
  vpCovisibleKFs =
      mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
  spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
  spMergeConnectedKFs.insert(mpMergeMatchedKF);
  nNumTries = 0;
  while (spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries) {
    vector<std::shared_ptr<KeyFrame>> vpNewCovKFs;
    for (auto pKFi : spMergeConnectedKFs) {
      vector<std::shared_ptr<KeyFrame>> vpKFiCov =
          pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);
      for (auto pKFcov : vpKFiCov) {
        if (pKFcov && !pKFcov->isBad() &&
            spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end()) {
          vpNewCovKFs.push_back(pKFcov);
        }
      }
    }
    spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
    nNumTries++;
  }

  set<MapPoint*> spMapPointMerge;
  for (auto pKFi : spMergeConnectedKFs) {
    set<MapPoint*> vpMPs = pKFi->GetMapPoints();
    spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
  }

  vector<MapPoint*> vpCheckFuseMapPoint;
  vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
  std::copy(spMapPointMerge.begin(), spMapPointMerge.end(),
              std::back_inserter(vpCheckFuseMapPoint));

  // ==========================================================================
  //  修复点 1：Twc 类型统一为 SE3d
  //  --------------------------------------------------------------------------
  //  原代码：Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
  //  问题：与后续 Sophus::SE3d Tiw 相乘时类型不匹配，报错
  //      error: no match for 'operator*' (SE3d * SE3f)
  //  修复：直接转成 double，后续所有 Tiw * Twc 都是 SE3d * SE3d
  // ==========================================================================
  Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();

  g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(),
                               Twc.translation(),
                               1.0);
  g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
  g2o::Sim3 g2oCorrectedScw = mg2oMergeScw;

  KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
  vCorrectedSim3[mpCurrentKF] = g2oCorrectedScw;
  vNonCorrectedSim3[mpCurrentKF] = g2oNonCorrectedScw;

#ifdef REGISTER_TIMES
  vnMergeKFs.push_back(spLocalWindowKFs.size() + spMergeConnectedKFs.size());
  vnMergeMPs.push_back(spLocalWindowMPs.size() + spMapPointMerge.size());
#endif

  // 校正当前窗口所有关键帧的位姿
  for (auto pKFi : spLocalWindowKFs) {
    if (!pKFi || pKFi->isBad()) {
      oslog::debug("Bad KF in correction");
      continue;
    }

    if (pKFi->GetMap() != pCurrentMap)
      oslog::debug("Other map KF, this should't happen");

    g2o::Sim3 g2oCorrectedSiw;

    if (pKFi != mpCurrentKF) {
      Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
      g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
      vNonCorrectedSim3[pKFi] = g2oSiw;

      // 修复点 2：Tiw 和 Twc 都是 SE3d，类型匹配
      Sophus::SE3d Tic = Tiw * Twc;
      g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
      g2oCorrectedSiw = g2oSic * mg2oMergeScw;
      vCorrectedSim3[pKFi] = g2oCorrectedSiw;
    } else {
      g2oCorrectedSiw = g2oCorrectedScw;
    }

    pKFi->mTcwMerge = pKFi->GetPose();

    double s = g2oCorrectedSiw.scale();
    pKFi->mfScale = s;
    Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),
                               g2oCorrectedSiw.translation() / s);
    pKFi->mTcwMerge = correctedTiw.cast<float>();

    if (pCurrentMap->isImuInitialized()) {
      Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() *
                                  vNonCorrectedSim3[pKFi].rotation())
                                     .cast<float>();
      pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
    }
  }

  int numPointsWithCorrection = 0;

  set<MapPoint*>::iterator itMP = spLocalWindowMPs.begin();
  while (itMP != spLocalWindowMPs.end()) {
    MapPoint* pMPi = *itMP;
    if (!pMPi || pMPi->isBad()) {
      itMP = spLocalWindowMPs.erase(itMP);
      continue;
    }

    auto pKFref = pMPi->GetReferenceKeyFrame();
    if (vCorrectedSim3.find(pKFref) == vCorrectedSim3.end()) {
      itMP = spLocalWindowMPs.erase(itMP);
      numPointsWithCorrection++;
      continue;
    }
    g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
    g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

    Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
    Eigen::Vector3d eigCorrectedP3Dw =
        g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
    Eigen::Quaterniond Rcor =
        g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

    pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
    pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

    itMP++;
  }

  {
    unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate);
    unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate);

    for (auto pKFi : spLocalWindowKFs) {
      if (!pKFi || pKFi->isBad()) {
        continue;
      }

      pKFi->mTcwBefMerge = pKFi->GetPose();
      pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
      pKFi->SetPose(pKFi->mTcwMerge);

      pKFi->UpdateMap(pMergeMap);
      pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;
      pMergeMap->AddKeyFrame(pKFi);
      pCurrentMap->EraseKeyFrame(pKFi);

      if (pCurrentMap->isImuInitialized()) {
        pKFi->SetVelocity(pKFi->mVwbMerge);
      }
    }

    for (MapPoint* pMPi : spLocalWindowMPs) {
      if (!pMPi || pMPi->isBad()) continue;

      pMPi->SetWorldPos(pMPi->mPosMerge);
      pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
      pMPi->UpdateMap(pMergeMap);
      pMergeMap->AddMapPoint(pMPi);
      pCurrentMap->EraseMapPoint(pMPi);
    }

    mpAtlas->ChangeMap(pMergeMap);
    mpAtlas->SetMapBad(pCurrentMap);
    pMergeMap->IncreaseChangeIndex();
    pMergeMap->ChangeId(pCurrentMap->GetId());
  }

  pCurrentMap->GetOriginKF()->SetFirstConnection(false);
  pNewChild = mpCurrentKF->GetParent();
  pNewParent = mpCurrentKF;
  mpCurrentKF->ChangeParent(mpMergeMatchedKF);

  while (pNewChild) {
    pNewChild->EraseChild(pNewParent);
    auto pOldParent = pNewChild->GetParent();
    pNewChild->ChangeParent(pNewParent);
    pNewParent = pNewChild;
    pNewChild = pOldParent;
  }

  mpMergeMatchedKF->UpdateConnections();

  vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
  vpMergeConnectedKFs.push_back(mpMergeMatchedKF);

  SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

  for (auto pKFi : spLocalWindowKFs) {
    if (!pKFi || pKFi->isBad()) continue;
    pKFi->UpdateConnections();
  }
  for (auto pKFi : spMergeConnectedKFs) {
    if (!pKFi || pKFi->isBad()) continue;
    pKFi->UpdateConnections();
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartWeldingBA =
      std::chrono::steady_clock::now();
  double timeMergeMaps =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_StartWeldingBA - time_StartMerge)
          .count();
  vdMergeMaps_ms.push_back(timeMergeMaps);
#endif

  bool bStop = false;
  vpLocalCurrentWindowKFs.clear();
  vpMergeConnectedKFs.clear();
  std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(),
              std::back_inserter(vpLocalCurrentWindowKFs));
  std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(),
              std::back_inserter(vpMergeConnectedKFs));

  if (mpTracker->mSensor.isImu()) {
    Optimizer::MergeInertialBA(mpCurrentKF, mpMergeMatchedKF, &bStop,
                                pCurrentMap, vCorrectedSim3);
  } else {
    Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs,
                                      vpMergeConnectedKFs, &bStop);
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndWeldingBA =
      std::chrono::steady_clock::now();
  double timeWeldingBA =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndWeldingBA - time_StartWeldingBA)
          .count();
  vdWeldingBA_ms.push_back(timeWeldingBA);
#endif

  mpLocalMapper->Release();

  auto vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
  auto vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

  if (vpCurrentMapKFs.size() == 0) {
  } else {
    if (mpTracker->mSensor == SensorType::MONOCULAR) {
      unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate);

      for (auto pKFi : vpCurrentMapKFs) {
        if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
          continue;
        }

        g2o::Sim3 g2oCorrectedSiw;

        Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        vNonCorrectedSim3[pKFi] = g2oSiw;

        // 修复点 3：Tiw 和 Twc 都是 SE3d
        Sophus::SE3d Tic = Tiw * Twc;
        g2o::Sim3 g2oSim(Tic.unit_quaternion(), Tic.translation(), 1.0);
        g2oCorrectedSiw = g2oSim * mg2oMergeScw;
        vCorrectedSim3[pKFi] = g2oCorrectedSiw;

        double s = g2oCorrectedSiw.scale();
        pKFi->mfScale = s;

        Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),
                                   g2oCorrectedSiw.translation() / s);
        pKFi->mTcwBefMerge = pKFi->GetPose();
        pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
        pKFi->SetPose(correctedTiw.cast<float>());

        if (pCurrentMap->isImuInitialized()) {
          Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() *
                                      vNonCorrectedSim3[pKFi].rotation())
                                         .cast<float>();
          pKFi->SetVelocity(Rcor * pKFi->GetVelocity());
        }
      }

      for (MapPoint* pMPi : vpCurrentMapMPs) {
        if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pCurrentMap) continue;

        auto pKFref = pMPi->GetReferenceKeyFrame();
        g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

        Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
        Eigen::Vector3d eigCorrectedP3Dw =
            g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
        pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
        pMPi->UpdateNormalAndDepth();
      }
    }

    mpLocalMapper->RequestStop();
    while (!mpLocalMapper->isStopped()) {
      usleep(1000);
    }

    if (mpTracker->mSensor != SensorType::MONOCULAR) {
      Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs,
                                      vpLocalCurrentWindowKFs,
                                      vpCurrentMapKFs, vpCurrentMapMPs);
    }

    {
      unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate);
      unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate);

      for (auto pKFi : vpCurrentMapKFs) {
        if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
          continue;
        }
        pKFi->UpdateMap(pMergeMap);
        pMergeMap->AddKeyFrame(pKFi);
        pCurrentMap->EraseKeyFrame(pKFi);
      }

      for (MapPoint* pMPi : vpCurrentMapMPs) {
        if (!pMPi || pMPi->isBad()) continue;
        pMPi->UpdateMap(pMergeMap);
        pMergeMap->AddMapPoint(pMPi);
        pCurrentMap->EraseMapPoint(pMPi);
      }
    }
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndOptEss =
      std::chrono::steady_clock::now();
  double timeOptEss =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndOptEss - time_EndWeldingBA)
          .count();
  vdMergeOptEss_ms.push_back(timeOptEss);
#endif

  mpLocalMapper->Release();

  if (bRelaunchBA &&
      (!pCurrentMap->isImuInitialized() ||
       (pCurrentMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1))) {
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this,
                               pMergeMap, mpCurrentKF->mnId);
  }

  mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
  mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

  pCurrentMap->IncreaseChangeIndex();
  pMergeMap->IncreaseChangeIndex();

  mpAtlas->RemoveBadMaps();
}

// ==============================================
// IMU 模式下的地图合并
// ==============================================
void LoopClosing::MergeLocal2() {
  std::shared_ptr<KeyFrame> pNewChild;
  std::shared_ptr<KeyFrame> pNewParent;

  vector<std::shared_ptr<KeyFrame>> vpLocalCurrentWindowKFs;
  vector<std::shared_ptr<KeyFrame>> vpMergeConnectedKFs;

  KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

  if (isRunningGBA()) {
    unique_lock<mutex> lock(mMutexGBA);
    mbStopGBA = true;

    mnFullBAIdx++;

    if (mpThreadGBA) {
      mpThreadGBA->detach();
      delete mpThreadGBA;
    }
  }

  mpLocalMapper->RequestStop();
  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }

  std::shared_ptr<Map> pCurrentMap = mpCurrentKF->GetMap();
  std::shared_ptr<Map> pMergeMap = mpMergeMatchedKF->GetMap();

  {
    float s_on = mSold_new.scale();
    Sophus::SE3f T_on(mSold_new.rotation().cast<float>(),
                       mSold_new.translation().cast<float>());

    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    mpLocalMapper->EmptyQueue();

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    bool bScaleVel = false;
    if (s_on != 1) bScaleVel = true;

    mpAtlas->GetCurrentMap()->ApplyScaledRotation(T_on, s_on, bScaleVel);
    mpTracker->UpdateFrameIMU(s_on, mpCurrentKF->GetImuBias(),
                               mpTracker->GetLastKeyFrame());

    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
  }

  const int numKFnew = pCurrentMap->KeyFramesInMap();

  if ((mpTracker->mSensor.isImu()) && !pCurrentMap->GetInertialBA2()) {
    Eigen::Vector3d bg, ba;
    bg << 0., 0., 0.;
    ba << 0., 0., 0.;
    Optimizer::InertialOptimization(pCurrentMap, bg, ba);
    IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    mpTracker->UpdateFrameIMU(1.0f, b, mpTracker->GetLastKeyFrame());

    pCurrentMap->SetInertialBA2();
    pCurrentMap->SetInertialBA1();
    pCurrentMap->SetImuInitialized();
  }

  {
    unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate);
    unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate);

    vector<std::shared_ptr<KeyFrame>> vpMergeMapKFs =
        pMergeMap->GetAllKeyFrames();
    vector<MapPoint*> vpMergeMapMPs = pMergeMap->GetAllMapPoints();

    for (auto pKFi : vpMergeMapKFs) {
      if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap) {
        continue;
      }

      pKFi->UpdateMap(pCurrentMap);
      pCurrentMap->AddKeyFrame(pKFi);
      pMergeMap->EraseKeyFrame(pKFi);
    }

    for (MapPoint* pMPi : vpMergeMapMPs) {
      if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap) continue;

      pMPi->UpdateMap(pCurrentMap);
      pCurrentMap->AddMapPoint(pMPi);
      pMergeMap->EraseMapPoint(pMPi);
    }

    auto vpKFs = pCurrentMap->GetAllKeyFrames();
    for (auto pKFi : vpKFs) {
      Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
      g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
      NonCorrectedSim3[pKFi] = g2oSiw;
    }
  }

  pMergeMap->GetOriginKF()->SetFirstConnection(false);
  pNewChild = mpMergeMatchedKF->GetParent();
  pNewParent = mpMergeMatchedKF;
  mpMergeMatchedKF->ChangeParent(mpCurrentKF);
  while (pNewChild) {
    pNewChild->EraseChild(pNewParent);
    auto pOldParent = pNewChild->GetParent();
    pNewChild->ChangeParent(pNewParent);
    pNewParent = pNewChild;
    pNewChild = pOldParent;
  }

  vector<MapPoint*> vpCheckFuseMapPoint;
  vector<std::shared_ptr<KeyFrame>> vpCurrentConnectedKFs;

  mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
  auto aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
  mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(),
                               aux.end());
  if (mvpMergeConnectedKFs.size() > 6)
    mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin() + 6,
                                mvpMergeConnectedKFs.end());

  mpCurrentKF->UpdateConnections();
  vpCurrentConnectedKFs.push_back(mpCurrentKF);
  aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
  vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(),
                                aux.end());
  if (vpCurrentConnectedKFs.size() > 6)
    vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6,
                                 vpCurrentConnectedKFs.end());

  set<MapPoint*> spMapPointMerge;
  for (auto pKFi : mvpMergeConnectedKFs) {
    set<MapPoint*> vpMPs = pKFi->GetMapPoints();
    spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
    if (spMapPointMerge.size() > 1000) break;
  }

  vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
  std::copy(spMapPointMerge.begin(), spMapPointMerge.end(),
              std::back_inserter(vpCheckFuseMapPoint));

  SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

  for (auto pKFi : vpCurrentConnectedKFs) {
    if (!pKFi || pKFi->isBad()) continue;

    pKFi->UpdateConnections();
  }

  for (auto pKFi : mvpMergeConnectedKFs) {
    if (!pKFi || pKFi->isBad()) continue;

    pKFi->UpdateConnections();
  }

  if (numKFnew < 10) {
    mpLocalMapper->Release();
    return;
  }

  bool bStopFlag = false;
  auto pCurrKF = mpTracker->GetLastKeyFrame();
  Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap,
                              CorrectedSim3);

  mpLocalMapper->Release();

  return;
}

// ==============================================
// 调试函数：检查两个地图关键帧之间的共视观测数
// ==============================================
void LoopClosing::CheckObservations(set<std::shared_ptr<KeyFrame>>& spKFsMap1,
                                    set<std::shared_ptr<KeyFrame>>& spKFsMap2) {
  cout << "----------------------" << endl;
  for (auto pKFi1 : spKFsMap1) {
    map<std::shared_ptr<KeyFrame>, int> mMatchedMP;
    set<MapPoint*> spMPs = pKFi1->GetMapPoints();

    for (MapPoint* pMPij : spMPs) {
      if (!pMPij || pMPij->isBad()) {
        continue;
      }

      map<std::shared_ptr<KeyFrame>, tuple<int, int>> mMPijObs =
          pMPij->GetObservations();
      for (auto pKFi2 : spKFsMap2) {
        if (mMPijObs.find(pKFi2) != mMPijObs.end()) {
          if (mMatchedMP.find(pKFi2) != mMatchedMP.end()) {
            mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
          } else {
            mMatchedMP[pKFi2] = 1;
          }
        }
      }
    }

    if (mMatchedMP.size() == 0) {
      cout << "CHECK-OBS: KF " << pKFi1->mnId
           << " has not any matched MP with the other map" << endl;
    } else {
      cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with "
           << mMatchedMP.size() << " KF from the other map" << endl;
      for (pair<std::shared_ptr<KeyFrame>, int> matchedKF : mMatchedMP) {
        cout << "   -KF: " << matchedKF.first->mnId
             << ", Number of matches: " << matchedKF.second << endl;
      }
    }
  }
  cout << "----------------------" << endl;
}

// ==============================================
// 搜索并融合重复地图点（按关键帧-位姿映射）
// ==============================================
void LoopClosing::SearchAndFuse(const KeyFrameAndPose& CorrectedPosesMap,
                                vector<MapPoint*>& vpMapPoints) {
  ORBmatcher matcher(0.8);

  int total_replaces = 0;

  for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(),
                                       mend = CorrectedPosesMap.end();
       mit != mend; mit++) {
    int num_replaces = 0;
    std::shared_ptr<KeyFrame> pKFi = mit->first;
    std::shared_ptr<Map> pMap = pKFi->GetMap();

    g2o::Sim3 g2oScw = mit->second;
    Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

    vector<MapPoint*> vpReplacePoints(vpMapPoints.size(),
                                       static_cast<MapPoint*>(NULL));
    matcher.Fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    auto const nLP = vpMapPoints.size();
    for (size_t i = 0; i < nLP; i++) {
      MapPoint* pRep = vpReplacePoints[i];
      if (pRep) {
        num_replaces += 1;
        pRep->Replace(vpMapPoints[i]);
      }
    }

    total_replaces += num_replaces;
  }
}

// ==============================================
// 搜索并融合重复地图点（按关键帧向量）
// ==============================================
void LoopClosing::SearchAndFuse(
    const vector<std::shared_ptr<KeyFrame>>& vConectedKFs,
    vector<MapPoint*>& vpMapPoints) {
  ORBmatcher matcher(0.8);

  for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end(); mit != mend;
       mit++) {
    int num_replaces = 0;
    std::shared_ptr<KeyFrame> pKF = (*mit);
    std::shared_ptr<Map> pMap = pKF->GetMap();
    Sophus::SE3f Tcw = pKF->GetPose();
    Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
    Scw.setScale(1.f);

    vector<MapPoint*> vpReplacePoints(vpMapPoints.size(),
                                       static_cast<MapPoint*>(NULL));
    matcher.Fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

    // ========================================================================
    //  修复点 4：mMutexUpdate → mMutexMapUpdate
    //  ------------------------------------------------------------------------
    //  Map 类里实际成员名是 mMutexMapUpdate，原代码写成 mMutexUpdate 导致：
    //      error: 'class ORB_SLAM3::Map' has no member named 'mMutexUpdate';
    //              did you mean 'mMutexMapUpdate'?
    // ========================================================================
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    const int nLP = vpMapPoints.size();
    for (int i = 0; i < nLP; i++) {
      MapPoint* pRep = vpReplacePoints[i];
      if (pRep) {
        num_replaces += 1;
        pRep->Replace(vpMapPoints[i]);
      }
    }
  }
}

// ==============================================
// 请求全局重置
// ==============================================
void LoopClosing::RequestReset() {
  {
    unique_lock<mutex> lock(mMutexReset);
    mbResetRequested = true;
  }

  while (1) {
    {
      unique_lock<mutex> lock2(mMutexReset);
      if (!mbResetRequested) break;
    }
    usleep(500);
  }
}

// ==============================================
// 请求重置活动地图
// ==============================================
void LoopClosing::RequestResetActiveMap(const std::shared_ptr<Map>& pMap) {
  {
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMapRequested = true;
    mpMapToReset = pMap;
  }

  while (1) {
    {
      unique_lock<mutex> lock2(mMutexReset);
      if (!mbResetActiveMapRequested) break;
    }
    usleep(300);
  }
}

// ==============================================
// 如果有重置请求则执行重置
// ==============================================
void LoopClosing::ResetIfRequested() {
  unique_lock<mutex> lock(mMutexReset);
  if (mbResetRequested) {
    cout << "Loop closer reset requested..." << endl;
    mlpLoopKeyFrameQueue.clear();
    mLastLoopKFid = 0;
    mbResetRequested = false;
    mbResetActiveMapRequested = false;
  } else if (mbResetActiveMapRequested) {
    for (list<std::shared_ptr<KeyFrame>>::const_iterator it =
             mlpLoopKeyFrameQueue.begin();
         it != mlpLoopKeyFrameQueue.end();) {
      std::shared_ptr<KeyFrame> pKFi = *it;
      if (pKFi->GetMap() == mpMapToReset) {
        it = mlpLoopKeyFrameQueue.erase(it);
      } else {
        ++it;
      }
    }

    mLastLoopKFid = mpAtlas->GetLastInitKFid();
    mbResetActiveMapRequested = false;
  }
}

// ==============================================
// 全局 BA 线程函数
// ==============================================
void LoopClosing::RunGlobalBundleAdjustment(
    const std::shared_ptr<Map>& pActiveMap, unsigned long nLoopKF) {
  oslog::debug("Starting Global Bundle Adjustment");

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartFGBA =
      std::chrono::steady_clock::now();
  nFGBA_exec += 1;
  vnGBAKFs.push_back(pActiveMap->GetAllKeyFrames().size());
  vnGBAMPs.push_back(pActiveMap->GetAllMapPoints().size());
#endif

  const bool bImuInit = pActiveMap->isImuInitialized();

  if (!bImuInit)
    Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF,
                                       false);
  else
    Optimizer::FullInertialBA(pActiveMap, 7, false, nLoopKF, &mbStopGBA);

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndGBA =
      std::chrono::steady_clock::now();
  double timeGBA =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndGBA - time_StartFGBA)
          .count();
  vdGBA_ms.push_back(timeGBA);

  if (mbStopGBA) {
    nFGBA_abort += 1;
  }
#endif

  int idx = mnFullBAIdx;

  {
    unique_lock<mutex> lock(mMutexGBA);
    if (idx != mnFullBAIdx) return;

    if (!bImuInit && pActiveMap->isImuInitialized()) return;

    if (!mbStopGBA) {
      oslog::info("Global Bundle Adjustment finished");
      oslog::info("Updating map ...");

      mpLocalMapper->RequestStop();

      while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()) {
        usleep(1000);
      }

      unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);

      list<std::shared_ptr<KeyFrame>> lpKFtoCheck(
          pActiveMap->mvpKeyFrameOrigins.begin(),
          pActiveMap->mvpKeyFrameOrigins.end());

      while (!lpKFtoCheck.empty()) {
        std::shared_ptr<KeyFrame> pKF = lpKFtoCheck.front();
        const set<std::shared_ptr<KeyFrame>> sChilds = pKF->GetChilds();
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        for (auto pChild : sChilds) {
          if (!pChild || pChild->isBad()) continue;

          if (pChild->mnBAGlobalForKF != nLoopKF) {
            Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

            Sophus::SO3f Rcor =
                pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
            if (pChild->isVelocitySet()) {
              pChild->mVwbGBA = Rcor * pChild->GetVelocity();
            } else {
              oslog::info("Child velocity empty!!");
            }

            pChild->mBiasGBA = pChild->GetImuBias();

            pChild->mnBAGlobalForKF = nLoopKF;
          }
          lpKFtoCheck.push_back(pChild);
        }

        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA);

        if (pKF->bImu) {
          pKF->mVwbBefGBA = pKF->GetVelocity();
          pKF->SetVelocity(pKF->mVwbGBA);
          pKF->SetNewBias(pKF->mBiasGBA);
        }

        lpKFtoCheck.pop_front();
      }

      const vector<MapPoint*> vpMPs = pActiveMap->GetAllMapPoints();

      for (size_t i = 0; i < vpMPs.size(); i++) {
        MapPoint* pMP = vpMPs[i];

        if (pMP->isBad()) continue;

        if (pMP->mnBAGlobalForKF == nLoopKF) {
          pMP->SetWorldPos(pMP->mPosGBA);
        } else {
          auto pRefKF = pMP->GetReferenceKeyFrame();

          if (pRefKF->mnBAGlobalForKF != nLoopKF) continue;

          Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

          pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
      }

      pActiveMap->InformNewBigChange();
      pActiveMap->IncreaseChangeIndex();

      mpLocalMapper->Release();

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_EndUpdateMap =
          std::chrono::steady_clock::now();
      double timeUpdateMap =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndUpdateMap - time_EndGBA)
              .count();
      vdUpdateMap_ms.push_back(timeUpdateMap);

      double timeFGBA =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndUpdateMap - time_StartFGBA)
              .count();
      vdFGBATotal_ms.push_back(timeFGBA);
#endif

      oslog::info("Map updated!");
    }

    mbFinishedGBA = true;
    mbRunningGBA = false;
  }
}

// ==============================================
// 请求结束线程
// ==============================================
void LoopClosing::RequestFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

// ==============================================
// 检查是否请求结束
// ==============================================
bool LoopClosing::CheckFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

// ==============================================
// 设置线程结束标志
// ==============================================
void LoopClosing::SetFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinished = true;
}

// ==============================================
// 检查线程是否已结束
// ==============================================
bool LoopClosing::isFinished() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinished;
}

}  // namespace ORB_SLAM3