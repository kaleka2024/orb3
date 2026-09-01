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

// 地图点类头文件：定义3D地图点的属性、观测管理、几何计算等核心接口
#include "MapPoint.h"
// STL标准库：通用算法，用于排序、查找等操作
#include <algorithm>
// STL标准库：标准输入输出流，用于日志、调试打印
#include <iostream>
// STL标准库：映射容器，存储关键帧到观测索引的映射关系
#include <map>
// STL标准库：智能指针管理
#include <memory>
// STL标准库：互斥锁，保证地图点多线程操作的线程安全
#include <mutex>
// STL标准库：集合容器，用于去重、快速查找
#include <set>
// STL标准库：元组结构，存储左右目两个观测索引
#include <tuple>
// STL标准库：动态数组容器
#include <vector>

// ORB-SLAM3内部：ORB特征匹配器，提供描述子距离计算函数
#include "ORBmatcher.h"

namespace ORB_SLAM3 {

// 全局静态计数器：地图点全局唯一ID自增分配器，所有地图点共享
long unsigned int MapPoint::nNextId = 0;
// 全局互斥锁：保护地图点ID分配、全局操作的线程安全
mutex MapPoint::mGlobalMutex;

// ==============================================
// 默认构造函数：初始化所有成员变量为默认值
// ==============================================
MapPoint::MapPoint()
    : mnFirstKFid(0),              // 首次观测到该点的关键帧ID
      mnFirstFrame(0),             // 首次观测到该点的普通帧ID
      nObs(0),                   // 总观测次数（双目点左右目各算1次，合计+2）
      mnTrackReferenceForFrame(0),  // 作为跟踪参考点的次数
      mnLastFrameSeen(0),        // 最后一次被观测到的帧ID
      mnBALocalForKF(0),         // 局部束调整中被优化的次数
      mnFuseCandidateForKF(0),    // 作为融合候选点的次数
      mnLoopPointForKF(0),       // 作为回环匹配点的次数
      mnCorrectedByKF(0),        // 执行校正的关键帧ID
      mnCorrectedReference(0),    // 校正参考索引
      mnBAGlobalForKF(0),       // 全局束调整中被优化的次数
      mnVisible(1),              // 可见计数：投影在图像范围内的总次数
      mnFound(1),                // 匹配成功计数：实际匹配到的次数
      mbBad(false),              // 坏点标志：true表示该点已废弃/无效
      mpReplaced(static_cast<MapPoint*>(NULL)) { // 被融合后指向的替代地图点
  mpReplaced = static_cast<MapPoint*>(NULL);
}

// ==============================================
// 构造函数：由3D世界坐标、参考关键帧、所属地图创建地图点
// 输入：Pos 世界坐标系3D位置，pRefKF 参考关键帧智能指针，pMap 所属地图智能指针
// 场景：三角化成功、新生成地图点时调用
// ==============================================
MapPoint::MapPoint(const Eigen::Vector3f& Pos,
                   const std::shared_ptr<KeyFrame>& pRefKF,
                   const std::shared_ptr<Map>& pMap)
    : mnFirstKFid(pRefKF->mnId),    // 首次观测关键帧ID设为参考帧ID
      mnFirstFrame(pRefKF->mnFrameId), // 首次观测帧ID设为参考帧对应帧ID
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(pRefKF),        // 存储参考关键帧指针
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mfMinDistance(0),       // 最小不变距离（对应最精细金字塔层级）
      mfMaxDistance(0),       // 最大不变距离（对应观测所在金字塔层级）
      mpMap(pMap),            // 所属地图指针
      mnOriginMapId(pMap->GetId()) { // 起源地图ID
  // 设置世界坐标位置
  SetWorldPos(Pos);

  // 平均观测法向量初始化为零向量
  mNormalVector.setZero();

  mbTrackInViewR = false;
  mbTrackInView = false;

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  // 加地图点创建锁，保证ID分配的线程安全，避免ID冲突
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  // 分配全局唯一ID，计数器自增
  mnId = nNextId++;
}

// ==============================================
// 构造函数：由逆深度、初始像素坐标创建临时地图点
// 输入：invDepth 逆深度值，uv_init 初始像素坐标，pRefKF 参考关键帧，pHostKF 宿主关键帧，pMap 所属地图
// 场景：单目初始化、临时匹配生成候选点时使用，暂不计算完整世界坐标
// ==============================================
MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init,
                   const std::shared_ptr<KeyFrame>& pRefKF,
                   const std::shared_ptr<KeyFrame>& pHostKF,
                   const std::shared_ptr<Map>& pMap)
    : mnFirstKFid(pRefKF->mnId),
      mnFirstFrame(pRefKF->mnFrameId),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(pRefKF),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mfMinDistance(0),
      mfMaxDistance(0),
      mpMap(pMap),
      mnOriginMapId(pMap->GetId()) {
  // 存储逆深度值
  mInvDepth = invDepth;
  // 存储初始像素坐标u、v（转双精度）
  mInitU = static_cast<double>(uv_init.x);
  mInitV = static_cast<double>(uv_init.y);
  // 存储宿主关键帧指针
  mpHostKF = pHostKF;

  mNormalVector.setZero();

  // Worldpos is not set
  // 世界坐标暂未设置，待后续三角化计算

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

// ==============================================
// 构造函数：由3D位置、普通帧、特征索引创建地图点
// 输入：Pos 世界坐标，pMap 所属地图，pFrame 观测普通帧，idxF 特征点索引
// 功能：自动计算观测法向量、距离不变性范围、提取对应描述子
// ==============================================
MapPoint::MapPoint(const Eigen::Vector3f& Pos, const std::shared_ptr<Map>& pMap,
                   const std::shared_ptr<Frame>& pFrame, const int& idxF)
    : mnFirstKFid(-1),       // 无关键帧，初始化为-1
      mnFirstFrame(pFrame->mnId), // 首次观测的普通帧ID
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(nullptr),    // 无参考关键帧
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mpMap(pMap),
      mnOriginMapId(pMap->GetId()) {
  // 设置世界坐标
  SetWorldPos(Pos);

  Eigen::Vector3f Ow;
  // 判断特征点属于左目还是右目，获取对应相机中心
  if (pFrame->Nleft == -1 || idxF < pFrame->Nleft) {
    Ow = pFrame->GetCameraCenter();
  } else {
    // 右目特征：通过左目旋转与左右相对位姿计算右目相机中心
    Eigen::Matrix3f Rwl = pFrame->GetRwc();
    Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
    Eigen::Vector3f twl = pFrame->GetOw();

    Ow = Rwl * tlr + twl;
  }

  // 计算观测方向向量：从相机指向地图点
  mNormalVector = mWorldPos - Ow;
  // 单位化法向量
  mNormalVector = mNormalVector / mNormalVector.norm();

  // 计算地图点到相机的距离
  Eigen::Vector3f PC = mWorldPos - Ow;
  const float dist = PC.norm();

  // 获取特征点所在的图像金字塔层级
  const int level = (pFrame->Nleft == -1)
    ? pFrame->mvKeysUn[idxF].octave
                    : (idxF < pFrame->Nleft) ? pFrame->mvKeys[idxF].octave
                                              : pFrame->mvKeysRight[idxF - pFrame->Nleft].octave;

  // 获取该金字塔层级对应的尺度因子
  const float levelScaleFactor = pFrame->mvScaleFactors[level];
  const int nLevels = pFrame->mnScaleLevels;

  // 计算距离不变性范围：
  // 最大距离 = 相机到点的距离 × 该层尺度因子
  // 最小距离 = 最大距离 / 最粗层尺度因子
  mfMaxDistance = dist * levelScaleFactor;
  mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

  // 复制对应特征点的ORB描述子作为初始描述子
  pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

// ==============================================
// 设置地图点的世界坐标
// 线程安全：同时加全局锁和位置互斥锁，保证多线程下位置修改安全
// ==============================================
void MapPoint::SetWorldPos(const Eigen::Vector3f& Pos) {
  unique_lock<mutex> lock2(mGlobalMutex);
  unique_lock<mutex> lock(mMutexPos);
  mWorldPos = Pos;
}

// ==============================================
// 获取地图点的世界坐标
// 线程安全：加位置互斥锁
// ==============================================
Eigen::Vector3f MapPoint::GetWorldPos() {
  unique_lock<mutex> lock(mMutexPos);
  return mWorldPos;
}

// ==============================================
// 获取地图点的平均观测法向量
// 线程安全：加位置互斥锁
// ==============================================
Eigen::Vector3f MapPoint::GetNormal() {
  unique_lock<mutex> lock(mMutexPos);
  return mNormalVector;
}

// ==============================================
// 获取参考关键帧指针
// 线程安全：加特征互斥锁
// ==============================================
std::shared_ptr<KeyFrame> MapPoint::GetReferenceKeyFrame() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mpRefKF;
}

// ==============================================
// 添加一个关键帧对该地图点的观测
// 输入：pKF 观测关键帧智能指针，idx 特征点索引（左目或右目）
// 说明：自动区分左右目；单目观测计数+1，有立体匹配的双目点计数+2
// ==============================================
void MapPoint::AddObservation(const std::shared_ptr<KeyFrame>& pKF, int idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  tuple<int, int> indexes;

  // 已存在该关键帧的观测则读取原有索引，否则初始化为(-1,-1)
  if (mObservations.count(pKF)) {
    indexes = mObservations[pKF];
  } else {
    indexes = tuple<int, int>(-1, -1);
  }

  // 索引大于等于左目总数，说明是右目观测
  if (pKF->NLeft != -1 && idx >= pKF->NLeft) {
    get<1>(indexes) = idx;  // 存入右目索引
  } else {
    get<0>(indexes) = idx;  // 存入左目索引
  }

  // 更新观测映射表
  mObservations[pKF] = indexes;

  // 无第二相机但该点有右目立体匹配，观测次数+2
  if (!pKF->mpCamera2 && pKF->mvuRight[idx] >= 0)
    nObs += 2;
  else
    nObs++;  // 普通单目观测次数+1
}

// ==============================================
// 擦除指定关键帧对该地图点的观测
// 输入：pKF 待擦除观测的关键帧智能指针
// 说明：擦除后自动更新观测计数；观测过少则标记为坏点；参考帧被擦除则自动更换
// ==============================================
void MapPoint::EraseObservation(const std::shared_ptr<KeyFrame>& pKF) {
  bool bBad = false;
  {
    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF)) {
      tuple<int, int> indexes = mObservations[pKF];
      int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

      // 左目有观测，对应减少观测次数
      if (leftIndex != -1) {
        if (!pKF->mpCamera2 && pKF->mvuRight[leftIndex] >= 0)
          nObs -= 2;  // 双目点减2次
        else
          nObs--;     // 单目点减1次
      }
      // 右目有观测，观测次数再减1
      if (rightIndex != -1) {
        nObs--;
      }

      // 从观测映射中移除该关键帧
      mObservations.erase(pKF);

      // 观测为空，标记为坏点
      if (mObservations.size() == 0) {
        bBad = true;
      } else if (mpRefKF == pKF) {
        // 被擦除的是参考帧，更换为第一个观测关键帧作为新参考
        mpRefKF = mObservations.begin()->first;
      }

      // If only 2 observations or less, discard point
      // 总观测次数≤2，判定为质量差，标记为坏点
      if (nObs <= 2) bBad = true;
    }
  }

  // 标记为坏点则执行坏点清理流程
  if (bBad) SetBadFlag();
}

// ==============================================
// 获取所有观测的映射表：关键帧 → 左右目索引元组
// 线程安全：加特征互斥锁
// ==============================================
std::map<std::shared_ptr<KeyFrame>, std::tuple<int, int>> MapPoint::GetObservations() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mObservations;
}

// ==============================================
// 获取总观测次数
// 线程安全：加特征互斥锁
// ==============================================
int MapPoint::Observations() {
  unique_lock<mutex> lock(mMutexFeatures);
  return nObs;
}

// ==============================================
// 标记地图点为坏点，并清理所有关联
// 功能：清空自身观测，通知所有观测关键帧擦除对应匹配，最后从所属地图移除
// ==============================================
void MapPoint::SetBadFlag() {
  map<std::shared_ptr<KeyFrame>, tuple<int, int>> obs;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    mbBad = true;
    // 备份观测列表用于后续遍历清理
    obs = mObservations;
    mObservations.clear();
  }

  // 遍历所有观测关键帧，通知其擦除对应位置的地图点匹配
  for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::iterator
           mit = obs.begin(),
           mend = obs.end();
       mit != mend; mit++) {
    std::shared_ptr<KeyFrame> pKF = mit->first;
    int leftIndex = get<0>(mit->second), rightIndex = get<1>(mit->second);

    // 擦除左目对应匹配
    if (leftIndex != -1) {
      pKF->EraseMapPointMatch(leftIndex);
    }
    // 擦除右目对应匹配
    if (rightIndex != -1) {
      pKF->EraseMapPointMatch(rightIndex);
    }
  }

  // 从所属地图中移除该地图点
  mpMap->EraseMapPoint(this);
}

// ==============================================
// 获取替代该点的地图点指针（融合后指向的有效点）
// 线程安全：同时加特征锁和位置锁
// ==============================================
MapPoint* MapPoint::GetReplaced() {
  unique_lock<mutex> lock1(mMutexFeatures);
  unique_lock<mutex> lock2(mMutexPos);
  return mpReplaced;
}

// ==============================================
// 地图点融合：将当前点替换为目标点pMP
// 原理：将当前点的所有观测转移给目标点，标记自身为坏点，完成点的合并
// 输入：pMP 目标融合地图点指针
// ==============================================
void MapPoint::Replace(MapPoint* pMP) {
  // 同一个点直接返回
  if (pMP->mnId == this->mnId) return;

  int nvisible, nfound;
  map<std::shared_ptr<KeyFrame>, tuple<int, int>> obs;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    // 备份当前点的所有观测
    obs = mObservations;
    mObservations.clear();
    mbBad = true;
    // 备份可见计数、匹配计数
    nvisible = mnVisible;
    nfound = mnFound;
    // 记录替代指针
    mpReplaced = pMP;
  }

  // 遍历所有观测关键帧，将观测转移到目标点
  for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::iterator
           mit = obs.begin(),
           mend = obs.end();
       mit != mend; mit++) {
    // Replace measurement in keyframe
    std::shared_ptr<KeyFrame> pKF = mit->first;

    tuple<int, int> indexes = mit->second;
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

    // 目标点无该关键帧观测则替换匹配，否则直接擦除当前点的匹配
    if (!pMP->IsInKeyFrame(pKF)) {
      if (leftIndex != -1) {
        pKF->ReplaceMapPointMatch(leftIndex, pMP);
        pMP->AddObservation(pKF, leftIndex);
      }
      if (rightIndex != -1) {
        pKF->ReplaceMapPointMatch(rightIndex, pMP);
        pMP->AddObservation(pKF, rightIndex);
      }
    } else {
      if (leftIndex != -1) {
        pKF->EraseMapPointMatch(leftIndex);
      }
      if (rightIndex != -1) {
        pKF->EraseMapPointMatch(rightIndex);
      }
    }
  }

  // 目标点累加可见、匹配次数
  pMP->IncreaseFound(nfound);
  pMP->IncreaseVisible(nvisible);
  // 重新计算目标点的代表性描述子
  pMP->ComputeDistinctiveDescriptors();

  // 从地图中移除当前点
  mpMap->EraseMapPoint(this);
}

// ==============================================
// 判断是否为坏点
// 线程安全：同时锁定特征互斥量和位置互斥量，保证数据一致
// ==============================================
bool MapPoint::isBad() {
  unique_lock<mutex> lock1(mMutexFeatures, std::defer_lock);
  unique_lock<mutex> lock2(mMutexPos, std::defer_lock);
  lock(lock1, lock2);

  return mbBad;
}

// ==============================================
// 增加可见计数
// ==============================================
void MapPoint::IncreaseVisible(int n) {
  unique_lock<mutex> lock(mMutexFeatures);
  mnVisible += n;
}

// ==============================================
// 增加匹配成功计数
// ==============================================
void MapPoint::IncreaseFound(int n) {
  unique_lock<mutex> lock(mMutexFeatures);
  mnFound += n;
}

// ==============================================
// 获取找到率：匹配成功次数 / 可见次数
// 用途：评估地图点质量，找到率过低的点会被剔除
// ==============================================
float MapPoint::GetFoundRatio() {
  unique_lock<mutex> lock(mMutexFeatures);
  return static_cast<float>(mnFound) / mnVisible;
}

// ==============================================
// 计算地图点的代表性描述子
// 原理：计算所有观测描述子两两之间的汉明距离，取中位数距离最小的描述子作为代表
// 目的：提升匹配鲁棒性，避免单视角描述子的偶然性
// ==============================================
void MapPoint::ComputeDistinctiveDescriptors() {
  // Retrieve all observed descriptors
  vector<cv::Mat> vDescriptors;

  map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations;

  {
    unique_lock<mutex> lock1(mMutexFeatures);
    if (mbBad) return;
    observations = mObservations;
  }

  if (observations.empty()) return;

  vDescriptors.reserve(observations.size());

  // 收集所有有效观测的描述子
  for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::iterator
           mit = observations.begin(),
           mend = observations.end();
       mit != mend; mit++) {
    std::shared_ptr<KeyFrame> pKF = mit->first;

    if (!pKF->isBad()) {
      tuple<int, int> indexes = mit->second;
      int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

      // 左目描述子
      if (leftIndex != -1) {
        vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
      }
      // 右目描述子
      if (rightIndex != -1) {
        vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
      }
    }
  }

  if (vDescriptors.empty()) return;

  // Compute distances between them
  // 计算描述子两两之间的汉明距离矩阵
  const size_t N = vDescriptors.size();

  float Distances[N][N];
  for (size_t i = 0; i < N; i++) {
    Distances[i][i] = 0;
    for (size_t j = i + 1; j < N; j++) {
      int distij =
          ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
      Distances[i][j] = distij;
      Distances[j][i] = distij;
    }
  }

  // Take the descriptor with least median distance to the rest
  // 寻找中位数距离最小的描述子作为代表性描述子
  int BestMedian = INT_MAX;
  int BestIdx = 0;
  for (size_t i = 0; i < N; i++) {
    vector<int> vDists(Distances[i], Distances[i] + N);
    // 对第i个描述子到其他所有描述子的距离排序
    sort(vDists.begin(), vDists.end());
    // 取中位数
    int median = vDists[0.5 * (N - 1)];

    // 更新最佳中位数对应的索引
    if (median < BestMedian) {
      BestMedian = median;
      BestIdx = i;
    }
  }

  {
    unique_lock<mutex> lock(mMutexFeatures);
    // 保存代表性描述子
    mDescriptor = vDescriptors[BestIdx].clone();
  }
}

// ==============================================
// 获取地图点的代表性描述子
// 线程安全：加特征互斥锁
// ==============================================
cv::Mat MapPoint::GetDescriptor() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mDescriptor.clone();
}

// ==============================================
// 获取指定关键帧中对应的特征点索引（左右目）
// 返回：tuple<左目索引, 右目索引>，不存在则返回(-1,-1)
// ==============================================
tuple<int, int> MapPoint::GetIndexInKeyFrame(
    const std::shared_ptr<KeyFrame>& pKF) {
  unique_lock<mutex> lock(mMutexFeatures);
  if (mObservations.count(pKF))
    return mObservations[pKF];
  else
    return tuple<int, int>(-1, -1);
}

// ==============================================
// 判断是否被指定关键帧观测到
// ==============================================
bool MapPoint::IsInKeyFrame(const std::shared_ptr<KeyFrame>& pKF) {
  unique_lock<mutex> lock(mMutexFeatures);
  return (mObservations.count(pKF));
}

// ==============================================
// 更新地图点的平均法向量与距离不变性范围
// 原理：根据所有观测相机位置，计算平均观测方向，更新最大/最小不变距离
// ==============================================
void MapPoint::UpdateNormalAndDepth() {
  map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations;
  std::shared_ptr<KeyFrame> pRefKF;
  Eigen::Vector3f Pos;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    if (mbBad) return;
    observations = mObservations;
    pRefKF = mpRefKF;
    Pos = mWorldPos;
  }

  if (observations.empty()) return;

  Eigen::Vector3f normal;
  normal.setZero();
  int n = 0;

  // 遍历所有观测，累加单位观测方向向量
  for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::iterator
           mit = observations.begin(),
           mend = observations.end();
       mit != mend; mit++) {
    std::shared_ptr<KeyFrame> pKF = mit->first;

    tuple<int, int> indexes = mit->second;
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

    // 左目相机的观测方向
    if (leftIndex != -1) {
      Eigen::Vector3f Owi = pKF->GetCameraCenter();
      Eigen::Vector3f normali = Pos - Owi;
      normal = normal + normali / normali.norm();
      n++;
    }
    // 右目相机的观测方向
    if (rightIndex != -1) {
      Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
      Eigen::Vector3f normali = Pos - Owi;
      normal = normal + normali / normali.norm();
      n++;
    }
  }

  // 计算参考帧到地图点的距离
  Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
  const float dist = PC.norm();

  // 获取参考帧中对应特征点的金字塔层级
  tuple<int, int> indexes = observations[pRefKF];
  int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
  int level;
  if (pRefKF->NLeft == -1) {
    level = pRefKF->mvKeysUn[leftIndex].octave;
  } else if (leftIndex != -1) {
    level = pRefKF->mvKeys[leftIndex].octave;
  } else {
    level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
  }

  // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
  const float levelScaleFactor = pRefKF->mvScaleFactors[level];
  const int nLevels = pRefKF->mnScaleLevels;

  {
    unique_lock<mutex> lock3(mMutexPos);
    // 更新最大/最小不变距离
    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
    // 平均法向量归一化
    mNormalVector = normal / n;
  }
}

// ==============================================
// 直接设置法向量
// ==============================================
void MapPoint::SetNormalVector(const Eigen::Vector3f& normal) {
  unique_lock<mutex> lock3(mMutexPos);
  mNormalVector = normal;
}

// ==============================================
// 获取最小不变距离（带0.8安全系数）
// 用于投影匹配时的尺度范围下限
// ==============================================
float MapPoint::GetMinDistanceInvariance() {
  unique_lock<mutex> lock(mMutexPos);
  return 0.8f * mfMinDistance;
}

// ==============================================
// 获取最大不变距离（带1.2安全系数）
// 用于投影匹配时的尺度范围上限
// ==============================================
float MapPoint::GetMaxDistanceInvariance() {
  unique_lock<mutex> lock(mMutexPos);
  return 1.2f * mfMaxDistance;
}

// ==============================================
// 根据当前距离预测特征所在的金字塔层级
// 输入：currentDist 当前距离，pKF 目标关键帧
// 返回：预测的金字塔层级索引
// ==============================================
int MapPoint::PredictScale(const float& currentDist,
                           const std::shared_ptr<KeyFrame>& pKF) {
  float ratio;
  {
    unique_lock<mutex> lock(mMutexPos);
    // 距离比 = 最大不变距离 / 当前距离
    ratio = mfMaxDistance / currentDist;
  }

  // 对数计算预测层级：n = ceil( log(ratio) / log(scaleFactor) )
  int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
  // 边界限制，不小于0，不超过最大层级
  if (nScale < 0)
    nScale = 0;
  else if (nScale >= pKF->mnScaleLevels)
    nScale = pKF->mnScaleLevels - 1;

  return nScale;
}

// ==============================================
// 重载：根据普通帧预测金字塔层级
// ==============================================
int MapPoint::PredictScale(const float& currentDist,
                           const std::shared_ptr<Frame>& pF) {
  float ratio;
  {
    unique_lock<mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
  if (nScale < 0)
    nScale = 0;
  else if (nScale >= pF->mnScaleLevels)
    nScale = pF->mnScaleLevels - 1;

  return nScale;
}

// ==============================================
// 打印所有观测信息（调试用）
// ==============================================
void MapPoint::PrintObservations() {
  oslog::debug("MP_OBS: MP {}", mnId);
  for (auto const& [pKFi, indexes] : mObservations) {
    // int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    oslog::debug("--OBS in KF {} in map {}", pKFi->mnId,
                 pKFi->GetMap()->GetId());
  }
}

// ==============================================
// 获取所属地图指针
// 线程安全：加地图互斥锁
// ==============================================
std::shared_ptr<Map> MapPoint::GetMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mpMap;
}

// ==============================================
// 更新所属地图指针
// ==============================================
void MapPoint::UpdateMap(const std::shared_ptr<Map>& pMap) {
  unique_lock<mutex> lock(mMutexMap);
  mpMap = pMap;
}

// ==============================================
// 保存前预处理：指针转ID，为序列化做准备
// 输入：spKF 所有待存关键帧集合，spMP 所有待存地图点集合
// ==============================================
void MapPoint::PreSave(set<std::shared_ptr<KeyFrame>>& spKF,
                       set<MapPoint*>& spMP) {
  // 备份替代点ID，不存在则为-1
  mBackupReplacedId = -1;
  if (mpReplaced && spMP.find(mpReplaced) != spMP.end())
    mBackupReplacedId = mpReplaced->mnId;

  mBackupObservationsId1.clear();
  mBackupObservationsId2.clear();

  // Save the id and position in each KF who view it
  // 遍历所有观测，保存关键帧ID和对应左右目索引
  for (std::map<std::shared_ptr<KeyFrame>, std::tuple<int, int>>::const_iterator
           it = mObservations.begin(),
           end = mObservations.end();
       it != end; ++it) {
    std::shared_ptr<KeyFrame> pKFi = it->first;
    // 关键帧在待存集合中则保存ID，否则擦除该无效观测
    if (spKF.find(pKFi) != spKF.end()) {
      mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
      mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
    } else {
      EraseObservation(pKFi);
    }
  }

  // Save the id of the reference KF
  // 保存参考关键帧ID
  if (spKF.find(mpRefKF) != spKF.end()) {
    mBackupRefKFId = mpRefKF->mnId;
  }
}

// ==============================================
// 加载后重建：从ID恢复指针引用，重建所有观测关联
// 输入：mpKFid 关键帧ID-指针对射表，mpMPid 地图点ID-指针对射表
// ==============================================
void MapPoint::PostLoad(
    map<long unsigned int, std::shared_ptr<KeyFrame>>& mpKFid,
    map<long unsigned int, MapPoint*>& mpMPid) {
  // 恢复参考关键帧指针
  mpRefKF = mpKFid[mBackupRefKFId];
  if (!mpRefKF) {
    oslog::warn("ERROR: MP without KF reference {}; Num obs: {}",
                mBackupRefKFId, nObs);
  }

  // 恢复替代点指针
  mpReplaced = static_cast<MapPoint*>(NULL);
  if (mBackupReplacedId >= 0) {
    map<long unsigned int, MapPoint*>::iterator it =
        mpMPid.find(mBackupReplacedId);
    if (it != mpMPid.end()) mpReplaced = it->second;
  }

  // 重建观测映射表
  mObservations.clear();

  for (map<long unsigned int, int>::const_iterator
           it = mBackupObservationsId1.begin(),
           end = mBackupObservationsId1.end();
       it != end; ++it) {
    std::shared_ptr<KeyFrame> pKFi = mpKFid[it->first];
    map<long unsigned int, int>::const_iterator it2 =
        mBackupObservationsId2.find(it->first);
    std::tuple<int, int> indexes = tuple<int, int>(it->second, it2->second);
    if (pKFi) {
      mObservations[pKFi] = indexes;
    }
  }

  // 清空备份容器
  mBackupObservationsId1.clear();
  mBackupObservationsId2.clear();
}

}  // namespace ORB_SLAM3
