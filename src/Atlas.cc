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

// Atlas类自身头文件声明：多地图集管理器，负责管理多个子地图的创建、切换、存储与加载
#include "Atlas.h"
// STL标准库：通用算法函数，用于排序、查找等操作
#include <algorithm>
// STL标准库：标准输入输出流
#include <iostream>
// STL标准库：智能指针管理，支持shared_ptr等自动内存管理
#include <memory>
// STL标准库：有序集合容器，用于存储不重复的地图指针集合
#include <set>
// STL标准库：动态数组容器
#include <vector>

// ORB-SLAM3内部：几何相机基类头文件
#include "GeometricCamera.h"
// ORB-SLAM3内部：Kannala-Brandt8参数鱼眼相机模型头文件
#include "KannalaBrandt8.h"
// ORB-SLAM3内部：针孔相机模型头文件
#include "Pinhole.h"
// ORB-SLAM3内部：可视化查看器头文件
#include "Viewer.h"

namespace ORB_SLAM3 {

// ==============================================
// 默认构造函数：初始化图集核心成员
// ==============================================
Atlas::Atlas() : mpCurrentMap(nullptr), mpViewer(nullptr) {}

// ==============================================
// 带初始关键帧ID的构造函数
// 输入：initKFid 新地图起始关键帧ID
// 功能：指定初始ID并创建第一个子地图
// ==============================================
Atlas::Atlas(int initKFid)
    : mnLastInitKFidMap(initKFid), mpCurrentMap(nullptr), mpViewer(nullptr) {
  // 创建第一个子地图
  CreateNewMap();
}

// ==============================================
// 析构函数：清空所有存储的子地图集合
// ==============================================
Atlas::~Atlas() { mspMaps.clear(); }

// ==============================================
// 创建新的子地图
// 线程安全：加锁保护图集共享资源
// 逻辑：将当前地图标记为已存储，创建新地图并设为当前活动地图
// ==============================================
void Atlas::CreateNewMap() {
  // 加互斥锁，保证多线程下图集操作的线程安全
  unique_lock<mutex> lock(mMutexAtlas);

  oslog::info("Creation of new map with id: {}", Map::nNextId);

  // 若存在当前活动地图，先将其标记为存储状态
  if (mpCurrentMap) {
    // 更新下一个地图的起始关键帧ID：当前地图最大关键帧ID+1
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                          1;  // The init KF is the next of current maximum

    // 将当前地图标记为已存储状态（非活动）
    mpCurrentMap->SetStoredMap();
    oslog::info("Stored map with ID: {}", mpCurrentMap->GetId());

    // if(mpViewer)
    //     mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
  }

  oslog::info("Creation of new map with last KF id: {}", mnLastInitKFidMap);

  // 创建新的Map对象，指定起始关键帧ID
  mpCurrentMap = std::make_shared<Map>(mnLastInitKFidMap);
  // 将新地图标记为当前活动地图
  mpCurrentMap->SetCurrentMap();
  // 将新地图加入图集集合
  mspMaps.insert(mpCurrentMap);
}

// ==============================================
// 切换当前活动地图
// 输入：pMap 目标切换地图的智能指针
// ==============================================
void Atlas::ChangeMap(const std::shared_ptr<Map> &pMap) {
  unique_lock<mutex> lock(mMutexAtlas);

  oslog::info("Change to map with id: {}", pMap->GetId());

  // 将原当前地图标记为存储状态
  if (mpCurrentMap) {
    mpCurrentMap->SetStoredMap();
  }

  // 更新当前活动地图指针
  mpCurrentMap = pMap;
  // 将目标地图标记为当前活动状态
  mpCurrentMap->SetCurrentMap();
}

// ==============================================
// 获取上一个地图的最后一个初始化关键帧ID
// ==============================================
unsigned long int Atlas::GetLastInitKFid() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mnLastInitKFidMap;
}

// ==============================================
// 设置可视化查看器
// ==============================================
void Atlas::SetViewer(const std::shared_ptr<Viewer> &pViewer) {
  mpViewer = pViewer;
}

// ==============================================
// 向关键帧所属的地图中添加关键帧
// 输入：pKF 待添加的关键帧智能指针
// ==============================================
void Atlas::AddKeyFrame(const std::shared_ptr<KeyFrame> &pKF) {
  // 获取关键帧所属的地图
  std::shared_ptr<Map> pMapKF = pKF->GetMap();
  // 调用对应地图的添加关键帧接口
  pMapKF->AddKeyFrame(pKF);
}

// ==============================================
// 向地图点所属的地图中添加地图点
// 输入：pMP 待添加的地图点指针
// ==============================================
void Atlas::AddMapPoint(MapPoint *pMP) {
  // 获取地图点所属的地图
  std::shared_ptr<Map> pMapMP = pMP->GetMap();
  // 调用对应地图的添加地图点接口
  pMapMP->AddMapPoint(pMP);
}

// ==============================================
// 添加相机对象到图集，自动去重
// 输入：pCam 待添加的相机智能指针
// 返回：已存在的同相机指针或新添加的相机指针
// ==============================================
std::shared_ptr<GeometricCamera> Atlas::AddCamera(
    const std::shared_ptr<GeometricCamera> &pCam) {
  // 在相机列表中查找是否已存在相同相机
  auto const it = std::find(mvpCameras.begin(), mvpCameras.end(), pCam);
  // 已存在则直接返回原有指针
  if (it != mvpCameras.end()) {
    return *it;
  }

  // 不存在则加入相机列表
  mvpCameras.push_back(pCam);
  return pCam;
}

// ==============================================
// 获取图集中所有相机对象
// ==============================================
std::vector<std::shared_ptr<GeometricCamera>> Atlas::GetAllCameras() {
  return mvpCameras;
}

// ==============================================
// 设置当前地图的参考地图点
// 参考地图点用于重定位、跟踪等匹配基准
// ==============================================
void Atlas::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs) {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

// ==============================================
// 通知当前地图发生了大的结构变化（如回环、重定位）
// ==============================================
void Atlas::InformNewBigChange() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->InformNewBigChange();
}

// ==============================================
// 获取当前地图最后一次大变化的索引编号
// ==============================================
int Atlas::GetLastBigChangeIdx() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetLastBigChangeIdx();
}

// ==============================================
// 获取当前活动地图中的地图点总数
// ==============================================
long unsigned int Atlas::MapPointsInMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->MapPointsInMap();
}

// ==============================================
// 获取当前活动地图中的关键帧总数
// ==============================================
long unsigned Atlas::KeyFramesInMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->KeyFramesInMap();
}

// ==============================================
// 获取当前活动地图的所有关键帧列表
// ==============================================
std::vector<std::shared_ptr<KeyFrame>> Atlas::GetAllKeyFrames() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllKeyFrames();
}

// ==============================================
// 获取当前活动地图的所有地图点列表
// ==============================================
std::vector<MapPoint *> Atlas::GetAllMapPoints() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllMapPoints();
}

// ==============================================
// 获取当前活动地图的参考地图点列表
// ==============================================
std::vector<MapPoint *> Atlas::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetReferenceMapPoints();
}

// ==============================================
// 获取图集中所有子地图，按地图ID升序排序
// ==============================================
vector<std::shared_ptr<Map>> Atlas::GetAllMaps() {
  unique_lock<mutex> lock(mMutexAtlas);

  // 自定义比较仿函数：按地图ID从小到大排序
  struct compFunctor {
    inline bool operator()(const std::shared_ptr<Map> &elem1,
                           const std::shared_ptr<Map> &elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };

  // 将set集合转为vector容器
  vector<std::shared_ptr<Map>> vMaps(mspMaps.begin(), mspMaps.end());
  // 按ID排序
  sort(vMaps.begin(), vMaps.end(), compFunctor());
  return vMaps;
}

// ==============================================
// 获取图集中子地图的总数量
// ==============================================
int Atlas::CountMaps() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mspMaps.size();
}

// ==============================================
// 清空当前活动地图的所有数据
// ==============================================
void Atlas::clearMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->clear();
}

// ==============================================
// 清空整个图集的所有数据，重置状态
// ==============================================
void Atlas::clearAtlas() {
  unique_lock<mutex> lock(mMutexAtlas);

  /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end();
  it!=send; it++)
  {
      (*it)->clear();
      delete *it;
  }*/

  // 清空所有地图集合
  mspMaps.clear();
  // 释放当前地图指针
  mpCurrentMap.reset();
  // 重置初始关键帧ID计数
  mnLastInitKFidMap = 0;
}

// ==============================================
// 获取当前活动地图指针
// 逻辑：无地图则自动创建；地图状态异常则等待修复
// ==============================================
std::shared_ptr<Map> Atlas::GetCurrentMap() {
  unique_lock<mutex> lock(mMutexAtlas);

  // 无当前地图则创建新地图
  if (!mpCurrentMap) CreateNewMap();

  // 若当前地图标记为坏状态，循环等待300ms直到恢复
  while (mpCurrentMap->IsBad()) usleep(300);

  return mpCurrentMap;
}

// ==============================================
// 将指定地图标记为坏地图，移入坏地图集合
// 通常用于跟踪失败、无效的地图
// ==============================================
void Atlas::SetMapBad(const std::shared_ptr<Map> &pMap) {
  // 从正常地图集合中移除
  mspMaps.erase(pMap);
  // 标记地图为坏状态
  pMap->SetBad();

  // 加入坏地图集合
  mspBadMaps.insert(pMap);
}

// ==============================================
// 清空所有坏地图集合，释放无效地图
// ==============================================
void Atlas::RemoveBadMaps() { mspBadMaps.clear(); }

// ==============================================
// 判断当前地图是否为惯性模式（启用IMU融合）
// ==============================================
bool Atlas::isInertial() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->IsInertial();
}

// ==============================================
// 将当前地图设置为惯性传感器模式
// ==============================================
void Atlas::SetInertialSensor() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetInertialSensor();
}

// ==============================================
// 标记当前地图的IMU已完成初始化
// ==============================================
void Atlas::SetImuInitialized() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetImuInitialized();
}

// ==============================================
// 查询当前地图IMU是否已初始化
// ==============================================
bool Atlas::isImuInitialized() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->isImuInitialized();
}

// ==============================================
// 保存前预处理：备份地图、排序、过滤空地图
// ==============================================
void Atlas::PreSave() {
  // 更新下一个地图的起始关键帧ID
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                          1;  // The init KF is the next of current maximum
  }

  // 自定义比较仿函数：按地图ID排序
  struct compFunctor {
    inline bool operator()(const std::shared_ptr<Map> &elem1,
                           const std::shared_ptr<Map> &elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };

  // 将所有地图拷贝到备份向量
  std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
  // 备份地图按ID升序排序
  sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

  // 相机列表转set去重
  std::set<std::shared_ptr<GeometricCamera>> spCams(mvpCameras.begin(),
                                                    mvpCameras.end());

  // 遍历所有备份地图，执行保存前预处理
  for (auto pMi : mvpBackupMaps) {
    // 空指针或坏地图跳过
    if (!pMi || pMi->IsBad()) continue;

    // 无关键帧的空地图，标记为坏地图不保存
    if (pMi->GetAllKeyFrames().size() == 0) {
      // Empty map, erase before of save it.
      SetMapBad(pMi);
      continue;
    }
    // 执行地图保存前预处理，传入相机集合
    pMi->PreSave(spCams);
  }

  // 清理坏地图
  RemoveBadMaps();
}

// ==============================================
// 加载后处理：重建地图关联、恢复关键帧与词典
// ==============================================
void Atlas::PostLoad() {
  // 构建相机ID到相机对象的映射表
  map<unsigned int, std::shared_ptr<GeometricCamera>> mpCams;
  for (auto pCam : mvpCameras) {
    mpCams[pCam->GetId()] = pCam;
  }

  // 清空原地图集合
  mspMaps.clear();

  unsigned long int numKF = 0, numMP = 0;
  // 遍历所有备份地图，执行加载后重建
  for (auto pMi : mvpBackupMaps) {
    // 重新加入地图集合
    mspMaps.insert(pMi);
    // 调用地图加载后接口，传入关键帧数据库、ORB词典、相机映射
    pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
    // 统计总关键帧和地图点数量
    numKF += pMi->GetAllKeyFrames().size();
    numMP += pMi->GetAllMapPoints().size();
  }
  // 清空备份向量
  mvpBackupMaps.clear();
}

// ==============================================
// 设置关键帧数据库指针
// 注：原代码变量名Dababase为拼写保留，不做修改
// ==============================================
void Atlas::SetKeyFrameDababase(
    const std::shared_ptr<KeyFrameDatabase> &pKFDB) {
  mpKeyFrameDB = pKFDB;
}

// ==============================================
// 获取关键帧数据库指针
// ==============================================
std::shared_ptr<KeyFrameDatabase> Atlas::GetKeyFrameDatabase() {
  return mpKeyFrameDB;
}

// ==============================================
// 设置ORB特征词典指针
// ==============================================
void Atlas::SetORBVocabulary(const std::shared_ptr<ORBVocabulary> &pORBVoc) {
  mpORBVocabulary = pORBVoc;
}

// ==============================================
// 获取ORB特征词典指针
// ==============================================
std::shared_ptr<ORBVocabulary> Atlas::GetORBVocabulary() {
  return mpORBVocabulary;
}

// ==============================================
// 统计图集中所有存活地图的关键帧总数量
// ==============================================
long unsigned int Atlas::GetNumLivedKF() {
  unique_lock<mutex> lock(mMutexAtlas);

  long unsigned int num = 0;
  // 遍历所有地图累加关键帧数量
  for (auto const &pMap_i : mspMaps) {
    num += pMap_i->GetAllKeyFrames().size();
  }

  return num;
}

// ==============================================
// 统计图集中所有存活地图的地图点总数量
// ==============================================
long unsigned int Atlas::GetNumLivedMP() {
  unique_lock<mutex> lock(mMutexAtlas);

  long unsigned int num = 0;
  // 遍历所有地图累加地图点数量
  for (auto const &pMap_i : mspMaps) {
    num += pMap_i->GetAllMapPoints().size();
  }

  return num;
}

// ==============================================
// 获取图集中所有关键帧的ID-指针对应映射表
// 返回：map<关键帧ID, 关键帧智能指针>
// ==============================================
map<long unsigned int, std::shared_ptr<KeyFrame>> Atlas::GetAtlasKeyframes() {
  map<long unsigned int, std::shared_ptr<KeyFrame>> mpIdKFs;

  // 遍历所有备份地图
  for (auto const &pMap_i : mvpBackupMaps) {
    // 获取当前地图的所有关键帧
    vector<std::shared_ptr<KeyFrame>> vpKFs_Mi = pMap_i->GetAllKeyFrames();

    // 逐个加入ID-指针对射表
    for (auto const &pKF_j_Mi : vpKFs_Mi) {
      mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
    }
  }

  return mpIdKFs;
}

}  // namespace ORB_SLAM3
