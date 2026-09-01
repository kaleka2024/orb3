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

// 地图类头文件：管理单张子地图的核心元素，包括关键帧集合、地图点集合、状态标记、IMU属性等
#include "Map.h"
// STL标准库：通用算法函数，用于排序等操作
#include <algorithm>
// STL标准库：标准输入输出流
#include <iostream>
// STL标准库：智能指针管理
#include <memory>
// STL标准库：互斥锁，保证地图多线程操作的线程安全
#include <mutex>
// STL标准库：有序集合容器，存储不重复的关键帧、地图点指针
#include <set>
// STL标准库：元组结构
#include <tuple>
// STL标准库：动态数组容器
#include <vector>

namespace ORB_SLAM3 {

// 全局静态计数器：地图ID自增分配器，每个新建Map自动分配唯一的全局ID
unsigned long int Map::nNextId = 0;

// ==============================================
// 默认构造函数：初始化地图所有成员变量的默认值
// ==============================================
Map::Map()
    : mnMaxKFid(0),            // 地图内最大关键帧ID
      mnBigChangeIdx(0),       // 地图大结构变化索引（回环/合并/全局BA后递增）
      mbImuInitialized(false),  // IMU是否完成初始化标志
      mnMapChange(0),         // 地图局部变化计数
      mpFirstRegionKF(nullptr), // 第一个区域关键帧指针
      mbFail(false),          // 地图构建失败标志
      mIsInUse(false),        // 地图是否为当前活动地图标志
      mHasTumbnail(false),    // 是否生成缩略图标志
      mbBad(false),           // 地图是否为无效（坏）地图标志
      mnMapChangeNotified(0), // 已通知的地图变化索引
      mbIsInertial(false),    // 是否为惯性（IMU融合）模式地图
      mbIMU_BA1(false),      // 是否已完成第一阶段惯性束调整
      mbIMU_BA2(false) {     // 是否已完成第二阶段惯性束调整
  // 分配唯一地图ID，全局计数器自增
  mnId = nNextId++;
}

// ==============================================
// 带初始关键帧ID的构造函数：指定地图的起始关键帧ID
// 输入：initKFid 地图第一个关键帧的ID
// ==============================================
Map::Map(int initKFid)
    : mnInitKFid(initKFid),     // 地图初始关键帧ID
      mnMaxKFid(initKFid),     // 最大关键帧ID初始化为起始ID
      /*mnLastLoopKFid(initKFid),*/
      mnBigChangeIdx(0),
      mIsInUse(false),
      mHasTumbnail(false),
      mbBad(false),
      mbImuInitialized(false),
      mpFirstRegionKF(nullptr),
      mnMapChange(0),
      mbFail(false),
      mnMapChangeNotified(0),
      mbIsInertial(false),
      mbIMU_BA1(false),
      mbIMU_BA2(false) {
  mnId = nNextId++;
}

// ==============================================
// 析构函数：释放地图资源
// 说明：当前实现仅清空集合容器，未释放堆内存（保留原代码TODO标记）
// ==============================================
Map::~Map() {
  // TODO: erase all points from memory
  // 清空地图点集合
  mspMapPoints.clear();

  // TODO: erase all keyframes from memory
  // 清空关键帧集合
  mspKeyFrames.clear();

  // 清空参考地图点向量
  mvpReferenceMapPoints.clear();
  // 清空关键帧起源（生成树根节点）列表
  mvpKeyFrameOrigins.clear();
}

// ==============================================
// 向地图中添加一个关键帧
// 输入：pKF 待添加的关键帧智能指针
// 线程安全：加地图互斥锁保护共享资源
// 说明：第一个添加的关键帧会被设为初始关键帧；同时维护最大/最小关键帧ID
// ==============================================
void Map::AddKeyFrame(const std::shared_ptr<KeyFrame>& pKF) {
  unique_lock<mutex> lock(mMutexMap);

  // 第一个关键帧：初始化地图起始信息
  if (mspKeyFrames.empty()) {
    oslog::info("[Map::AddKeyFrame] First KF: {}; Map init KF: {}", pKF->mnId,
                mnInitKFid);
    mnInitKFid = pKF->mnId;  // 记录地图初始关键帧ID
    mpKFinitial = pKF;       // 保存初始关键帧指针（生成树的根节点）
    mpKFlowerID = pKF;      // 记录最小ID关键帧指针
  }

  // 插入关键帧到集合
  mspKeyFrames.insert(pKF);

  // 更新最大关键帧ID
  if (pKF->mnId > mnMaxKFid) {
    mnMaxKFid = pKF->mnId;
  }

  // 更新最小ID关键帧指针
  if (pKF->mnId < mpKFlowerID->mnId) {
    mpKFlowerID = pKF;
  }
}

// ==============================================
// 向地图中添加一个地图点
// 输入：pMP 待添加的地图点指针
// 线程安全：加锁
// ==============================================
void Map::AddMapPoint(MapPoint* pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.insert(pMP);
}

// ==============================================
// 标记地图IMU初始化完成
// ==============================================
void Map::SetImuInitialized() {
  unique_lock<mutex> lock(mMutexMap);
  mbImuInitialized = true;
}

// ==============================================
// 查询地图IMU是否已初始化
// ==============================================
bool Map::isImuInitialized() {
  unique_lock<mutex> lock(mMutexMap);
  return mbImuInitialized;
}

// ==============================================
// 从地图中擦除指定地图点
// 输入：pMP 待擦除的地图点指针
// 说明：仅从集合中移除指针，未释放堆内存（保留原代码TODO标记）
// ==============================================
void Map::EraseMapPoint(MapPoint* pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

// ==============================================
// 从地图中擦除指定关键帧
// 输入：pKF 待擦除的关键帧智能指针
// 说明：擦除后若集合非空，重新定位最小ID关键帧；集合为空则重置最小ID指针
// ==============================================
void Map::EraseKeyFrame(const std::shared_ptr<KeyFrame>& pKF) {
  unique_lock<mutex> lock(mMutexMap);
  // 从关键帧集合中移除
  mspKeyFrames.erase(pKF);

  if (mspKeyFrames.size() > 0) {
    // 若擦除的是当前最小ID关键帧，重新查找最小ID
    if (pKF->mnId == mpKFlowerID->mnId) {
      vector<std::shared_ptr<KeyFrame>> vpKFs =
          vector<std::shared_ptr<KeyFrame>>(mspKeyFrames.begin(),
                                             mspKeyFrames.end());
      // 按关键帧ID升序排序
      sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
      // 第一个即为最小ID关键帧
      mpKFlowerID = vpKFs[0];
    }
  } else {
    // 集合为空，重置最小ID指针
    mpKFlowerID = 0;
  }

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

// ==============================================
// 设置参考地图点集合（重定位、跟踪初始化使用的参考点）
// ==============================================
void Map::SetReferenceMapPoints(const vector<MapPoint*>& vpMPs) {
  unique_lock<mutex> lock(mMutexMap);
  mvpReferenceMapPoints = vpMPs;
}

// ==============================================
// 通知地图发生了大结构变化（回环闭合、地图合并、全局BA等）
// 功能：大变化索引自增，供其他模块检测地图是否发生更新
// ==============================================
void Map::InformNewBigChange() {
  unique_lock<mutex> lock(mMutexMap);
  mnBigChangeIdx++;
}

// ==============================================
// 获取最后一次大结构变化的索引
// ==============================================
int Map::GetLastBigChangeIdx() {
  unique_lock<mutex> lock(mMutexMap);
  return mnBigChangeIdx;
}

// ==============================================
// 获取地图中所有关键帧的向量副本
// ==============================================
vector<std::shared_ptr<KeyFrame>> Map::GetAllKeyFrames() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<std::shared_ptr<KeyFrame>>(mspKeyFrames.begin(),
                                            mspKeyFrames.end());
}

// ==============================================
// 获取地图中所有地图点的向量副本
// ==============================================
vector<MapPoint*> Map::GetAllMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<MapPoint*>(mspMapPoints.begin(), mspMapPoints.end());
}

// ==============================================
// 获取地图中地图点的总数量
// ==============================================
unsigned long int Map::MapPointsInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspMapPoints.size();
}

// ==============================================
// 获取地图中关键帧的总数量
// ==============================================
unsigned long int Map::KeyFramesInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspKeyFrames.size();
}

// ==============================================
// 获取参考地图点向量
// ==============================================
vector<MapPoint*> Map::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return mvpReferenceMapPoints;
}

// ==============================================
// 获取地图的唯一ID
// ==============================================
unsigned long int Map::GetId() { return mnId; }

// ==============================================
// 获取地图初始关键帧的ID
// ==============================================
unsigned long int Map::GetInitKFid() {
  unique_lock<mutex> lock(mMutexMap);
  return mnInitKFid;
}

// ==============================================
// 设置地图初始关键帧ID
// ==============================================
void Map::SetInitKFid(unsigned long int initKFif) {
  unique_lock<mutex> lock(mMutexMap);
  mnInitKFid = initKFif;
}

// ==============================================
// 获取地图中最大的关键帧ID
// ==============================================
unsigned long int Map::GetMaxKFid() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMaxKFid;
}

// ==============================================
// 获取地图的初始（起源）关键帧（生成树的根节点）
// ==============================================
std::shared_ptr<KeyFrame> Map::GetOriginKF() { return mpKFinitial; }

// ==============================================
// 标记地图为当前活动地图
// ==============================================
void Map::SetCurrentMap() { mIsInUse = true; }

// ==============================================
// 标记地图为存储（非活动）地图
// ==============================================
void Map::SetStoredMap() { mIsInUse = false; }

// ==============================================
// 清空地图所有数据，重置到初始状态
// 说明：清空关键帧、地图点，重置IMU状态、BA阶段标记等所有属性
// ==============================================
void Map::clear() {
  //    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(),
  //    send=mspMapPoints.end(); sit!=send; sit++)
  //        delete *sit;

  // 所有关键帧的地图指针置空，断开关联
  for (auto pKF : mspKeyFrames) {
    pKF->UpdateMap(nullptr);
  }
  // 清空关键帧集合
  mspKeyFrames.clear();

  // 清空地图点集合
  mspMapPoints.clear();

  // 重置最大关键帧ID为初始ID
  mnMaxKFid = mnInitKFid;
  // 重置IMU初始化标志
  mbImuInitialized = false;
  // 清空参考地图点
  mvpReferenceMapPoints.clear();
  // 清空关键帧起源列表
  mvpKeyFrameOrigins.clear();
  // 重置惯性BA阶段标记
  mbIMU_BA1 = false;
  mbIMU_BA2 = false;
}

// ==============================================
// 查询地图是否为当前活动使用状态
// ==============================================
bool Map::IsInUse() { return mIsInUse; }

// ==============================================
// 标记地图为无效（坏）地图
// ==============================================
void Map::SetBad() { mbBad = true; }

// ==============================================
// 查询地图是否为无效地图
// ==============================================
bool Map::IsBad() { return mbBad; }

// ==============================================
// 对整个地图应用相似变换（旋转 + 平移 + 尺度缩放）
// 输入：T 世界坐标系的SE3变换，s 尺度因子，bScaledVel 是否同步缩放速度
// 场景：IMU初始化、尺度校正时，统一变换所有关键帧位姿、速度和地图点位置
// ==============================================
void Map::ApplyScaledRotation(const Sophus::SE3f& T, const float s,
                              const bool bScaledVel) {
  unique_lock<mutex> lock(mMutexMap);

  // Body position (IMU) of first keyframe is fixed to (0,0,0)
  // 世界到IMU体坐标系的变换
  Sophus::SE3f Tyw = T;
  Eigen::Matrix3f Ryw = Tyw.rotationMatrix(); // 旋转矩阵
  Eigen::Vector3f tyw = Tyw.translation();   // 平移向量

  // 遍历所有关键帧，应用位姿变换
  for (set<std::shared_ptr<KeyFrame>>::iterator sit = mspKeyFrames.begin();
       sit != mspKeyFrames.end(); sit++) {
    std::shared_ptr<KeyFrame> pKF = *sit;
    // 获取相机到世界的逆位姿
    Sophus::SE3f Twc = pKF->GetPoseInverse();
    // 逆位姿的平移部分按尺度缩放
    Twc.translation() *= s;
    // 世界→相机 变换 × 相机→原世界 变换 = 新的世界→相机位姿
    Sophus::SE3f Tyc = Tyw * Twc;
    Sophus::SE3f Tcy = Tyc.inverse();
    // 更新关键帧的位姿
    pKF->SetPose(Tcy);

    // 更新速度向量：旋转后根据决定是否缩放
    Eigen::Vector3f Vw = pKF->GetVelocity();
    if (!bScaledVel)
      pKF->SetVelocity(Ryw * Vw);
    else
      pKF->SetVelocity(Ryw * Vw * s);
  }

  // 遍历所有地图点，应用位置变换
  for (set<MapPoint*>::iterator sit = mspMapPoints.begin();
       sit != mspMapPoints.end(); sit++) {
    MapPoint* pMP = *sit;
    // 新位置 = s * R * 原世界位置 + 平移向量
    pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
    // 更新地图点的平均法向量与深度
    pMP->UpdateNormalAndDepth();
  }

  // 地图变化计数自增
  mnMapChange++;
}

// ==============================================
// 设置地图为惯性传感器模式
// ==============================================
void Map::SetInertialSensor() {
  unique_lock<mutex> lock(mMutexMap);
  mbIsInertial = true;
}

// ==============================================
// 查询是否为惯性模式地图
// ==============================================
bool Map::IsInertial() {
  unique_lock<mutex> lock(mMutexMap);
  return mbIsInertial;
}

// ==============================================
// 标记已完成第一阶段惯性束调整
// ==============================================
void Map::SetInertialBA1() {
  unique_lock<mutex> lock(mMutexMap);
  mbIMU_BA1 = true;
}

// ==============================================
// 标记已完成第二阶段惯性束调整
// ==============================================
void Map::SetInertialBA2() {
  unique_lock<mutex> lock(mMutexMap);
  mbIMU_BA2 = true;
}

// ==============================================
// 查询是否完成第一阶段惯性束调整
// ==============================================
bool Map::GetInertialBA1() {
  unique_lock<mutex> lock(mMutexMap);
  return mbIMU_BA1;
}

// ==============================================
// 查询是否完成第二阶段惯性束调整
// ==============================================
bool Map::GetInertialBA2() {
  unique_lock<mutex> lock(mMutexMap);
  return mbIMU_BA2;
}

// ==============================================
// 修改地图的ID
// ==============================================
void Map::ChangeId(unsigned long int nId) { mnId = nId; }

// ==============================================
// 获取地图中最小的关键帧ID
// ==============================================
unsigned int Map::GetLowerKFID() {
  unique_lock<mutex> lock(mMutexMap);
  if (mpKFlowerID) {
    return mpKFlowerID->mnId;
  }
  return 0;
}

// ==============================================
// 获取当前地图变化索引
// ==============================================
int Map::GetMapChangeIndex() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMapChange;
}

// ==============================================
// 地图变化索引自增
// ==============================================
void Map::IncreaseChangeIndex() {
  unique_lock<mutex> lock(mMutexMap);
  mnMapChange++;
}

// ==============================================
// 获取上一次通知的地图变化索引
// ==============================================
int Map::GetLastMapChange() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMapChangeNotified;
}

// ==============================================
// 设置已通知的地图变化索引
// ==============================================
void Map::SetLastMapChange(int currentChangeId) {
  unique_lock<mutex> lock(mMutexMap);
  mnMapChangeNotified = currentChangeId;
}

// ==============================================
// 保存前预处理：清理无效观测、备份ID映射，为序列化做准备
// 输入：spCams 所有相机对象集合
// ==============================================
void Map::PreSave(std::set<std::shared_ptr<GeometricCamera>>& spCams) {
  int nMPWithoutObs = 0;

  oslog::info("[Map::PreSave] for {} map points", mspMapPoints.size());

  // 遍历所有地图点，清理跨地图/坏关键帧的无效观测
  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad()) continue;

    // 统计无观测的地图点数量
    if (pMPi->GetObservations().size() == 0) {
      nMPWithoutObs++;
    }

    map<std::shared_ptr<KeyFrame>, std::tuple<int, int>> mpObs =
        pMPi->GetObservations();
    // 移除不属于本地图、或对应关键帧已坏的观测
    for (map<std::shared_ptr<KeyFrame>, std::tuple<int, int>>::iterator
             it = mpObs.begin(),
             end = mpObs.end();
         it != end; ++it) {
      if (it->first->GetMap().get() != this || it->first->isBad()) {
        pMPi->EraseObservation(it->first);
      }
    }
  }

  // Saves the id of KF origins
  // 备份关键帧起源的ID列表
  mvBackupKeyFrameOriginsId.clear();
  mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
  for (int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i) {
    mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
  }

  // Backup of MapPoints
  // 备份地图点列表，调用每个地图点的保存前处理
  mvpBackupMapPoints.clear();
  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad()) continue;

    mvpBackupMapPoints.push_back(pMPi);
    pMPi->PreSave(mspKeyFrames, mspMapPoints);
  }

  // Backup of KeyFrames
  // 备份关键帧列表，调用每个关键帧的保存前处理
  mvpBackupKeyFrames.clear();
  for (auto pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad()) continue;

    mvpBackupKeyFrames.push_back(pKFi);
    pKFi->PreSave(mspKeyFrames, mspMapPoints, spCams);
  }

  // 备份初始关键帧ID
  mnBackupKFinitialID = -1;
  if (mpKFinitial) {
    mnBackupKFinitialID = mpKFinitial->mnId;
  }

  // 备份最小ID关键帧ID
  mnBackupKFlowerID = -1;
  if (mpKFlowerID) {
    mnBackupKFlowerID = mpKFlowerID->mnId;
  }
}

// ==============================================
// 加载后重建：从ID恢复指针引用，重建所有对象关联关系
// 输入：pKFDB 关键帧数据库指针，pORBVoc ORB词典指针，mpCams 相机ID-指针对射表
// 说明：反序列化后，通过ID重建关键帧、地图点之间的所有引用与关联
// ==============================================
void Map::PostLoad(
    const std::shared_ptr<KeyFrameDatabase>& pKFDB,
    const std::shared_ptr<ORBVocabulary>&
        pORBVoc /*, map<unsigned long int, KeyFrame*>& mpKeyFrameId*/,
    map<unsigned int, std::shared_ptr<GeometricCamera>>& mpCams) {
  // 将备份的地图点、关键帧插入当前集合
  std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(),
              std::inserter(mspMapPoints, mspMapPoints.begin()));
  std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(),
              std::inserter(mspKeyFrames, mspKeyFrames.begin()));

  // 构建地图点ID到指针的映射表
  map<unsigned long int, MapPoint*> mpMapPointId;
  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad()) continue;

    pMPi->UpdateMap(shared_from_this());
    mpMapPointId[pMPi->mnId] = pMPi;
  }

  // 构建关键帧ID到指针的映射表
  map<unsigned long int, std::shared_ptr<KeyFrame>> mpKeyFrameId;
  for (auto pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad()) continue;

    pKFi->UpdateMap(shared_from_this());
    pKFi->SetORBVocabulary(pORBVoc);
    pKFi->SetKeyFrameDatabase(pKFDB);
    mpKeyFrameId[pKFi->mnId] = pKFi;
  }

  // References reconstruction between different instances
  // 重建所有地图点的观测引用
  for (MapPoint* pMPi : mspMapPoints) {
    if (!pMPi || pMPi->isBad()) continue;

    pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
  }

  // 重建所有关键帧的引用，加入关键帧数据库
  for (auto pKFi : mspKeyFrames) {
    if (!pKFi || pKFi->isBad()) continue;

    pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
    pKFDB->add(pKFi);
  }

  // 恢复初始关键帧指针
  if (mnBackupKFinitialID != -1) {
    mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
  }

  // 恢复最小ID关键帧指针
  if (mnBackupKFlowerID != -1) {
    mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
  }

  // 恢复关键帧起源列表
  mvpKeyFrameOrigins.clear();
  mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
  for (size_t i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i) {
    mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
  }

  // 清空备份容器
  mvpBackupMapPoints.clear();
}

}  // namespace ORB_SLAM3
