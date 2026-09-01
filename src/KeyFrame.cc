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

// 关键帧类头文件声明
#include "KeyFrame.h"
// STL标准库：通用算法
#include <algorithm>
// STL标准库：标准输入输出
#include <iostream>
// STL标准库：双向链表容器
#include <list>
// STL标准库：映射容器
#include <map>
// STL标准库：智能指针
#include <memory>
// STL标准库：互斥锁，线程安全
#include <mutex>
// STL标准库：集合容器
#include <set>
// STL标准库：元组
#include <tuple>
// STL标准库：成对结构
#include <utility>
// STL标准库：动态数组
#include <vector>

// ORB-SLAM3内部：类型转换工具
#include "Converter.h"
// ORB-SLAM3内部：IMU类型定义
#include "ImuTypes.h"
// ORB-SLAM3内部：日志工具
#include "Logging.h"

namespace ORB_SLAM3 {

// 全局关键帧ID自增计数器，所有关键帧共享，分配唯一ID
unsigned long int KeyFrame::nNextId = 0;

// ==============================================
// 关键帧构造函数：从普通帧构造关键帧
// 输入：F 普通帧指针，pMap 所属地图，pKFDB 关键帧数据库指针
// 说明：完整拷贝帧的所有数据，初始化关键帧专属的连接、树结构、计数等成员
// ==============================================
KeyFrame::KeyFrame(const std::shared_ptr<Frame> &F,
                   const std::shared_ptr<Map> &pMap,
                   const std::shared_ptr<KeyFrameDatabase> &pKFDB)
    // IMU相关：标记地图是否已完成IMU初始化
    : bImu(pMap->isImuInitialized()),
      // 对应普通帧的帧ID
      mnFrameId(F->mnId),
      // 时间戳
      mTimeStamp(F->mTimeStamp),
      // 特征点网格列数
      mnGridCols(FRAME_GRID_COLS),
      // 特征点网格行数
      mnGridRows(FRAME_GRID_ROWS),
      // 网格单元宽度倒数
      mfGridElementWidthInv(F->mfGridElementWidthInv),
      // 网格单元高度倒数
      mfGridElementHeightInv(F->mfGridElementHeightInv),
      // 计数：作为参考帧被跟踪的次数
      mnTrackReferenceForFrame(0),
      // 计数：作为融合目标的次数
      mnFuseTargetForKF(0),
      // 计数：局部BA中被优化的次数
      mnBALocalForKF(0),
      // 计数：固定BA中作为固定帧的次数
      mnBAFixedForKF(0),
      // 计数：合并时局部BA次数
      mnBALocalForMerge(0),
      // 计数：回环检测查询次数
      mnLoopQuery(0),
      // 计数：回环词数
      mnLoopWords(0),
      // 计数：重定位查询次数
      mnRelocQuery(0),
      // 计数：重定位词数
      mnRelocWords(0),
      // 计数：全局BA次数
      mnBAGlobalForKF(0),
      // 计数：位置识别查询次数
      mnPlaceRecognitionQuery(0),
      // 计数：位置识别词数
      mnPlaceRecognitionWords(0),
      // 位置识别得分
      mPlaceRecognitionScore(0),
      // 相机内参：x方向焦距
      fx(F->fx),
      // 相机内参：y方向焦距
      fy(F->fy),
      // 相机内参：x方向主点
      cx(F->cx),
      // 相机内参：y方向主点
      cy(F->cy),
      // 焦距x倒数，预计算避免重复除法
      invfx(F->invfx),
      // 焦距y倒数
      invfy(F->invfy),
      // 物理基线×焦距 bf = b*f
      mbf(F->mbf),
      // 像素单位基线 mb = bf/fx
      mb(F->mb),
      // 近景深度阈值
      mThDepth(F->mThDepth),
      // 特征点总数量
      N(F->N),
      // 左目原始特征点向量
      mvKeys(F->mvKeys),
      // 去畸变后的特征点向量
      mvKeysUn(F->mvKeysUn),
      // 每个左目特征点对应的右目u坐标
      mvuRight(F->mvuRight),
      // 每个特征点对应的深度值
      mvDepth(F->mvDepth),
      // ORB描述子矩阵（深拷贝）
      mDescriptors(F->mDescriptors.clone()),
      // 词袋BoW向量
      mBowVec(F->mBowVec),
      // 词袋特征向量
      mFeatVec(F->mFeatVec),
      // 图像金字塔层数
      mnScaleLevels(F->mnScaleLevels),
      // 金字塔尺度因子
      mfScaleFactor(F->mfScaleFactor),
      // 尺度因子的对数值
      mfLogScaleFactor(F->mfLogScaleFactor),
      // 每层尺度因子数组
      mvScaleFactors(F->mvScaleFactors),
      // 每层尺度对应的方差平方
      mvLevelSigma2(F->mvLevelSigma2),
      // 每层尺度方差平方的倒数
      mvInvLevelSigma2(F->mvInvLevelSigma2),
      // 去畸变后图像最小X坐标
      mnMinX(F->mnMinX),
      // 去畸变后图像最小Y坐标
      mnMinY(F->mnMinY),
      // 去畸变后图像最大X坐标
      mnMaxX(F->mnMaxX),
      // 去畸变后图像最大Y坐标
      mnMaxY(F->mnMaxY),
      // Eigen格式的相机内参矩阵
      mK_(F->mK_),
      // 前一个关键帧指针
      mPrevKF(NULL),
      // 后一个关键帧指针
      mNextKF(NULL),
      // IMU预积分对象指针
      mpImuPreintegrated(F->mpImuPreintegrated),
      // IMU预积分备份指针对象
      mpBackupImuPreintegrated(std::make_shared<IMU::Preintegrated>()),
      // IMU标定参数
      mImuCalib(F->mImuCalib),
      // 特征点对应的地图点指针数组
      mvpMapPoints(F->mvpMapPoints),
      // 关键帧数据库指针
      mpKeyFrameDB(pKFDB),
      // ORB词典指针
      mpORBvocabulary(F->mpORBvocabulary),
      // 是否为首次连接标记（用于生成树）
      mbFirstConnection(true),
      // 生成树父节点指针
      mpParent(NULL),
      // 镜头畸变系数
      mDistCoef(F->mDistCoef),
      // 不可删除标记（参与回环/合并的帧不可删）
      mbNotErase(false),
      // 数据集编号
      mnDataset(F->mnDataset),
      // 待删除标记
      mbToBeErased(false),
      // 坏帧标记
      mbBad(false),
      // 半基线长度
      mHalfBaseline(F->mb / 2),
      // 所属地图指针
      mpMap(pMap),
      // 当前位置识别标记
      mbCurrentPlaceRecognition(false),
      // 图像文件名
      mNameFile(F->mNameFile),
      // 合并修正计数
      mnMergeCorrectedForKF(0),
      // 左相机对象指针
      mpCamera(F->mpCamera),
      // 右相机对象指针
      mpCamera2(F->mpCamera2),
      // 左目到右目的匹配索引数组
      mvLeftToRightMatch(F->mvLeftToRightMatch),
      // 右目到左目的匹配索引数组
      mvRightToLeftMatch(F->mvRightToLeftMatch),
      // 左目到右目的相对位姿Tlr
      mTlr(F->GetRelativePoseTlr()),
      // 右目原始特征点向量
      mvKeysRight(F->mvKeysRight),
      // 左目特征点数量
      NLeft(F->Nleft),
      // 右目特征点数量
      NRight(F->Nright),
      // 右目到左目的相对位姿Trl
      mTrl(F->GetRelativePoseTrl()),
      // 被优化次数计数
      mnNumberOfOpt(0),
      // 是否有有效速度标记
      mbHasVelocity(false) {
  // 分配全局唯一关键帧ID，计数器自增
  mnId = nNextId++;

  oslog::trace("New kF; map {}", mpMap ? "(exists)" : "NULL");

  // 初始化特征点网格：二维数组，每格存储对应特征点索引
  mGrid.resize(mnGridCols);
  // 双目模式下同时初始化右目网格
  if (F->Nleft != -1) mGridRight.resize(mnGridCols);

  // 逐列逐行拷贝帧的网格数据
  for (int i = 0; i < mnGridCols; i++) {
    mGrid[i].resize(mnGridRows);
    if (F->Nleft != -1) mGridRight[i].resize(mnGridRows);
    for (int j = 0; j < mnGridRows; j++) {
      mGrid[i][j] = F->mGrid[i][j];
      if (F->Nleft != -1) {
        mGridRight[i][j] = F->mGridRight[i][j];
      }
    }
  }

  // 处理速度：原帧无速度则置零，有则拷贝
  if (!F->HasVelocity()) {
    mVw.setZero();
    mbHasVelocity = false;
  } else {
    mVw = F->GetVelocity();
    mbHasVelocity = true;
  }

  // 拷贝IMU零偏
  mImuBias = F->mImuBias;
  // 设置关键帧位姿，同时更新派生旋转、平移矩阵
  SetPose(F->GetPose());

  // 记录创建时所属的原始地图ID
  mnOriginMapId = pMap->GetId();
}

// ==============================================
// 计算关键帧的词袋向量BoW
// 说明：仅在BoW为空时计算，避免重复计算；用于回环检测、重定位、位置识别
// ==============================================
void KeyFrame::ComputeBoW() {
  // BoW向量或特征向量为空时才计算
  if (mBowVec.empty() || mFeatVec.empty()) {
    // 将描述子矩阵转为向量格式，适配DBoW2输入
    vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // Feature vector associate features with nodes in the 4th level (from
    // leaves up) We assume the vocabulary tree has 6 levels, change the 4
    // otherwise
    // 调用词典转换，第4层分支深度
    mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
  }
}

// ==============================================
// 设置关键帧位姿（世界→相机SE3变换）
// 线程安全：位姿互斥锁保护
// ==============================================
void KeyFrame::SetPose(const Sophus::SE3f &Tcw) {
  unique_lock<mutex> lock(mMutexPose);

  // 保存世界到相机的SE3位姿
  mTcw = Tcw;
  // 提取世界到相机的旋转矩阵
  mRcw = mTcw.rotationMatrix();
  // 计算逆位姿：相机到世界
  mTwc = mTcw.inverse();
  // 提取相机到世界的旋转矩阵
  mRwc = mTwc.rotationMatrix();

  // TODO Use a flag instead of the OpenCV matrix
  // IMU标定有效时，计算IMU在世界坐标系下的位置
  if (mImuCalib.mbIsSet) {
    mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
  }
}

// ==============================================
// 设置世界坐标系下的相机速度
// ==============================================
void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw) {
  unique_lock<mutex> lock(mMutexPose);
  mVw = Vw;
  mbHasVelocity = true;
}

// ==============================================
// 获取位姿（世界→相机）
// ==============================================
Sophus::SE3f KeyFrame::GetPose() {
  unique_lock<mutex> lock(mMutexPose);
  return mTcw;
}

// ==============================================
// 获取逆位姿（相机→世界）
// ==============================================
Sophus::SE3f KeyFrame::GetPoseInverse() {
  unique_lock<mutex> lock(mMutexPose);
  return mTwc;
}

// ==============================================
// 获取相机光心在世界坐标系下的位置
// ==============================================
Eigen::Vector3f KeyFrame::GetCameraCenter() {
  unique_lock<mutex> lock(mMutexPose);
  return mTwc.translation();
}

// ==============================================
// 获取IMU在世界坐标系下的位置
// ==============================================
Eigen::Vector3f KeyFrame::GetImuPosition() {
  unique_lock<mutex> lock(mMutexPose);
  return mOwb;
}

// ==============================================
// 获取IMU在世界坐标系下的旋转矩阵
// ==============================================
Eigen::Matrix3f KeyFrame::GetImuRotation() {
  unique_lock<mutex> lock(mMutexPose);
  return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

// ==============================================
// 获取IMU在世界坐标系下的SE3位姿
// ==============================================
Sophus::SE3f KeyFrame::GetImuPose() {
  unique_lock<mutex> lock(mMutexPose);
  return mTwc * mImuCalib.mTcb;
}

// ==============================================
// 获取世界到相机的旋转矩阵
// ==============================================
Eigen::Matrix3f KeyFrame::GetRotation() {
  unique_lock<mutex> lock(mMutexPose);
  return mRcw;
}

// ==============================================
// 获取世界到相机的平移向量
// ==============================================
Eigen::Vector3f KeyFrame::GetTranslation() {
  unique_lock<mutex> lock(mMutexPose);
  return mTcw.translation();
}

// ==============================================
// 获取世界坐标系下的相机速度
// ==============================================
Eigen::Vector3f KeyFrame::GetVelocity() {
  unique_lock<mutex> lock(mMutexPose);
  return mVw;
}

// ==============================================
// 判断速度是否已设置
// ==============================================
bool KeyFrame::isVelocitySet() {
  unique_lock<mutex> lock(mMutexPose);
  return mbHasVelocity;
}

// ==============================================
// 添加与另一个关键帧的共视连接，指定权重（共视地图点数量）
// ==============================================
void KeyFrame::AddConnection(const std::shared_ptr<KeyFrame> &pKF,
                             const int &weight) {
  {
    unique_lock<mutex> lock(mMutexConnections);
    // 不存在该连接则添加；权重不同则更新；相同则直接返回
    if (!mConnectedKeyFrameWeights.count(pKF))
      mConnectedKeyFrameWeights[pKF] = weight;
    else if (mConnectedKeyFrameWeights[pKF] != weight)
      mConnectedKeyFrameWeights[pKF] = weight;
    else
      return;
  }

  // 连接变更后更新最佳共视排序列表
  UpdateBestCovisibles();
}

// ==============================================
// 更新共视关键帧的有序列表：按权重从高到低排序，过滤坏帧
// ==============================================
void KeyFrame::UpdateBestCovisibles() {
  unique_lock<mutex> lock(mMutexConnections);

  // 将<权重, 关键帧>对存入向量用于排序
  vector<pair<int, std::shared_ptr<KeyFrame>>> vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());

  // 遍历连接映射，转为（权重，关键帧）对
  for (map<std::shared_ptr<KeyFrame>, int>::iterator
           mit = mConnectedKeyFrameWeights.begin(),
           mend = mConnectedKeyFrameWeights.end();
       mit != mend; mit++)
    vPairs.push_back(make_pair(mit->second, mit->first));

  // 按权重升序排序
  sort(vPairs.begin(), vPairs.end());

  list<std::shared_ptr<KeyFrame>> lKFs;
  list<int> lWs;
  // 逆序遍历（权重从高到低），跳过坏帧，存入链表
  for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
    if (!vPairs[i].second->isBad()) {
      lKFs.push_front(vPairs[i].second);
      lWs.push_front(vPairs[i].first);
    }
  }

  // 转为有序向量存储：按共视权重从大到小排列
  mvpOrderedConnectedKeyFrames =
      vector<std::shared_ptr<KeyFrame>>(lKFs.begin(), lKFs.end());
  mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

// ==============================================
// 获取所有连接的关键帧集合
// ==============================================
set<std::shared_ptr<KeyFrame>> KeyFrame::GetConnectedKeyFrames() {
  unique_lock<mutex> lock(mMutexConnections);
  set<std::shared_ptr<KeyFrame>> s;
  // 遍历连接映射，插入集合
  for (map<std::shared_ptr<KeyFrame>, int>::iterator mit =
           mConnectedKeyFrameWeights.begin();
       mit != mConnectedKeyFrameWeights.end(); mit++)
    s.insert(mit->first);
  return s;
}

// ==============================================
// 获取按权重排序的共视关键帧向量
// ==============================================
vector<std::shared_ptr<KeyFrame>> KeyFrame::GetVectorCovisibleKeyFrames() {
  unique_lock<mutex> lock(mMutexConnections);
  return mvpOrderedConnectedKeyFrames;
}

// ==============================================
// 获取前N个共视权重最高的关键帧
// ==============================================
vector<std::shared_ptr<KeyFrame>> KeyFrame::GetBestCovisibilityKeyFrames(
    int N) {
  unique_lock<mutex> lock(mMutexConnections);
  // 总数不足N则返回全部
  if (mvpOrderedConnectedKeyFrames.size() < N)
    return mvpOrderedConnectedKeyFrames;
  else
    // 返回前N个
    return vector<std::shared_ptr<KeyFrame>>(
        mvpOrderedConnectedKeyFrames.begin(),
        mvpOrderedConnectedKeyFrames.begin() + N);
}

// ==============================================
// 获取共视权重大于等于w的所有关键帧
// ==============================================
vector<std::shared_ptr<KeyFrame>> KeyFrame::GetCovisiblesByWeight(int w) {
  unique_lock<mutex> lock(mMutexConnections);

  if (mvpOrderedConnectedKeyFrames.empty()) {
    return vector<std::shared_ptr<KeyFrame>>();
  }

  // 二分查找第一个大于w的权重位置
  vector<int>::iterator it =
      upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                  KeyFrame::weightComp);

  // 所有都小于w则返回空
  if (it == mvOrderedWeights.end() && mvOrderedWeights.back() < w) {
    return vector<std::shared_ptr<KeyFrame>>();
  } else {
    // 返回前n个（权重>=w）
    int n = it - mvOrderedWeights.begin();
    return vector<std::shared_ptr<KeyFrame>>(
        mvpOrderedConnectedKeyFrames.begin(),
        mvpOrderedConnectedKeyFrames.begin() + n);
  }
}

// ==============================================
// 获取与指定关键帧的共视权重
// ==============================================
int KeyFrame::GetWeight(const std::shared_ptr<KeyFrame> &pKF) {
  unique_lock<mutex> lock(mMutexConnections);
  if (mConnectedKeyFrameWeights.count(pKF))
    return mConnectedKeyFrameWeights[pKF];
  else
    return 0;
}

// ==============================================
// 统计关键帧中有效地图点的数量
// ==============================================
int KeyFrame::GetNumberMPs() {
  unique_lock<mutex> lock(mMutexFeatures);
  int numberMPs = 0;
  // 遍历所有特征点，统计非空且有效的地图点
  for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
    if (!mvpMapPoints[i]) continue;
    numberMPs++;
  }
  return numberMPs;
}

// ==============================================
// 在指定索引位置添加地图点匹配
// ==============================================
void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  mvpMapPoints[idx] = pMP;
}

// ==============================================
// 擦除指定索引处的地图点匹配（置空）
// ==============================================
void KeyFrame::EraseMapPointMatch(const int &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
}

// ==============================================
// 擦除指定地图点的匹配（按指针查找索引）
// ==============================================
void KeyFrame::EraseMapPointMatch(MapPoint *pMP) {
  // 获取地图点在本关键帧中的左右目索引
  tuple<size_t, size_t> indexes = pMP->GetIndexInKeyFrame(shared_from_this());
  size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
  // 左目有匹配则擦除
  if (leftIndex != -1) mvpMapPoints[leftIndex] = static_cast<MapPoint *>(NULL);
  // 右目有匹配则擦除
  if (rightIndex != -1)
    mvpMapPoints[rightIndex] = static_cast<MapPoint *>(NULL);
}

// ==============================================
// 替换指定索引处的地图点匹配
// ==============================================
void KeyFrame::ReplaceMapPointMatch(const int &idx, MapPoint *pMP) {
  mvpMapPoints[idx] = pMP;
}

// ==============================================
// 获取关键帧中所有有效且非坏的地图点集合
// ==============================================
set<MapPoint *> KeyFrame::GetMapPoints() {
  unique_lock<mutex> lock(mMutexFeatures);
  set<MapPoint *> s;
  for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
    if (!mvpMapPoints[i]) continue;
    MapPoint *pMP = mvpMapPoints[i];
    // 跳过坏的地图点
    if (!pMP->isBad()) s.insert(pMP);
  }
  return s;
}

// ==============================================
// 统计跟踪到的地图点数量，可指定最小观测次数阈值
// ==============================================
int KeyFrame::TrackedMapPoints(const int &minObs) {
  unique_lock<mutex> lock(mMutexFeatures);

  int nPoints = 0;
  const bool bCheckObs = minObs > 0;

  // 遍历所有特征点
  for (int i = 0; i < N; i++) {
    MapPoint *pMP = mvpMapPoints[i];
    if (pMP) {
      if (!pMP->isBad()) {
        // 需要检查观测次数
        if (bCheckObs) {
          if (mvpMapPoints[i]->Observations() >= minObs) nPoints++;
        } else {
          // 不检查则直接计数
          nPoints++;
        }
      }
    }
  }

  return nPoints;
}

// ==============================================
// 获取整个地图点匹配数组（含空指针）
// ==============================================
vector<MapPoint *> KeyFrame::GetMapPointMatches() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mvpMapPoints;
}

// ==============================================
// 按索引获取对应地图点指针
// ==============================================
MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  return mvpMapPoints[idx];
}

// ==============================================
// 更新关键帧的共视连接关系
// 原理：遍历所有地图点，统计每个其他关键帧的共视次数，超过阈值则建立连接
// upParent：是否更新生成树父节点
// ==============================================
void KeyFrame::UpdateConnections(bool upParent) {
  map<std::shared_ptr<KeyFrame>, int> KFcounter;

  vector<MapPoint *> vpMP;

  {
    unique_lock<mutex> lockMPs(mMutexFeatures);
    vpMP = mvpMapPoints;
  }

  // For all map points in keyframe check in which other keyframes are they seen
  // Increase counter for those keyframes
  // 遍历本帧所有地图点，统计每个关键帧的共视次数
  for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end();
       vit != vend; vit++) {
    MapPoint *pMP = *vit;

    if (!pMP) continue;
    if (pMP->isBad()) continue;

    // 获取该地图点在所有关键帧中的观测
    map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations =
        pMP->GetObservations();

    for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      // 跳过自身、坏帧、不同地图的关键帧
      if (mit->first->mnId == mnId || mit->first->isBad() ||
          mit->first->GetMap() != mpMap)
        continue;
      // 对应关键帧计数+1
      KFcounter[mit->first]++;
    }
  }

  // This should not happen
  // 无共视关键帧直接返回
  if (KFcounter.empty()) return;

  // If the counter is greater than threshold add connection
  // In case no keyframe counter is over threshold add the one with maximum
  // counter
  // 共视阈值，至少15个共视点才建立连接
  int nmax = 0;
  std::shared_ptr<KeyFrame> pKFmax = NULL;
  int th = 15;

  vector<pair<int, std::shared_ptr<KeyFrame>>> vPairs;
  vPairs.reserve(KFcounter.size());

  if (!upParent) cout << "UPDATE_CONN: current KF " << mnId << endl;

  // 遍历计数结果
  for (map<std::shared_ptr<KeyFrame>, int>::iterator mit = KFcounter.begin(),
                                                      mend = KFcounter.end();
       mit != mend; mit++) {
    if (!upParent)
      cout << "  UPDATE_CONN: KF " << mit->first->mnId
           << " ; num matches: " << mit->second << endl;

    // 记录最大共视的关键帧
    if (mit->second > nmax) {
      nmax = mit->second;
      pKFmax = mit->first;
    }

    // 超过阈值则加入连接列表，并双向添加连接
    if (mit->second >= th) {
      vPairs.push_back(make_pair(mit->second, mit->first));
      (mit->first)->AddConnection(shared_from_this(), mit->second);
    }
  }

  // 没有超过阈值的连接，则添加共视最多的那一个
  if (vPairs.empty()) {
    vPairs.push_back(make_pair(nmax, pKFmax));
    pKFmax->AddConnection(shared_from_this(), nmax);
  }

  // 按权重升序排序
  sort(vPairs.begin(), vPairs.end());

  list<std::shared_ptr<KeyFrame>> lKFs;
  list<int> lWs;
  // 逆序存入链表，得到权重从高到低的顺序
  for (size_t i = 0; i < vPairs.size(); i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  {
    unique_lock<mutex> lockCon(mMutexConnections);

    // 更新连接权重映射和有序列表
    mConnectedKeyFrameWeights = KFcounter;
    mvpOrderedConnectedKeyFrames =
        vector<std::shared_ptr<KeyFrame>>(lKFs.begin(), lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

    // 首次连接且不是初始关键帧时，设置生成树父节点
    if (mbFirstConnection && mnId != mpMap->GetInitKFid()) {
      mpParent = mvpOrderedConnectedKeyFrames.front();
      mpParent->AddChild(shared_from_this());
      mbFirstConnection = false;
    }
  }
}

// ==============================================
// 添加子关键帧（生成树结构）
// ==============================================
void KeyFrame::AddChild(const std::shared_ptr<KeyFrame> &pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.insert(pKF);
}

// ==============================================
// 删除子关键帧
// ==============================================
void KeyFrame::EraseChild(const std::shared_ptr<KeyFrame> &pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.erase(pKF);
}

// ==============================================
// 更换父关键帧
// ==============================================
void KeyFrame::ChangeParent(const std::shared_ptr<KeyFrame> &pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);

  // 父节点不能是自身
  if (pKF == shared_from_this()) {
    cout << "ERROR: Change parent KF, the parent and child are the same KF"
         << endl;
    throw std::invalid_argument("The parent and child can not be the same");
  }

  // 设置新父节点，并向新父节点添加自身为子节点
  mpParent = pKF;
  pKF->AddChild(shared_from_this());
}

// ==============================================
// 获取所有子关键帧集合
// ==============================================
set<std::shared_ptr<KeyFrame>> KeyFrame::GetChilds() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspChildrens;
}

// ==============================================
// 获取父关键帧
// ==============================================
std::shared_ptr<KeyFrame> KeyFrame::GetParent() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mpParent;
}

// ==============================================
// 判断是否包含指定子关键帧
// ==============================================
bool KeyFrame::hasChild(const std::shared_ptr<KeyFrame> &pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspChildrens.count(pKF);
}

// ==============================================
// 设置首次连接标记
// ==============================================
void KeyFrame::SetFirstConnection(bool bFirst) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mbFirstConnection = bFirst;
}

// ==============================================
// 添加回环边关键帧
// ==============================================
void KeyFrame::AddLoopEdge(const std::shared_ptr<KeyFrame> &pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mbNotErase = true;  // 有回环边的帧不可删除
  mspLoopEdges.insert(pKF);
}

// ==============================================
// 获取所有回环边关键帧集合
// ==============================================
set<std::shared_ptr<KeyFrame>> KeyFrame::GetLoopEdges() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspLoopEdges;
}

// ==============================================
// 添加合并边关键帧
// ==============================================
void KeyFrame::AddMergeEdge(const std::shared_ptr<KeyFrame> &pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mbNotErase = true;  // 有合并边的帧不可删除
  mspMergeEdges.insert(pKF);
}

// ==============================================
// 获取所有合并边关键帧集合
// ==============================================
set<std::shared_ptr<KeyFrame>> KeyFrame::GetMergeEdges() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspMergeEdges;
}

// ==============================================
// 设置为不可删除状态
// ==============================================
void KeyFrame::SetNotErase() {
  unique_lock<mutex> lock(mMutexConnections);
  mbNotErase = true;
}

// ==============================================
// 设置为可删除状态：回环边为空时才允许删除
// ==============================================
void KeyFrame::SetErase() {
  {
    unique_lock<mutex> lock(mMutexConnections);
    // 回环边为空时，才取消不可删除标记
    if (mspLoopEdges.empty()) {
      mbNotErase = false;
    }
  }

  // 已标记待删除则执行坏帧标记
  if (mbToBeErased) {
    SetBadFlag();
  }
}

// ==============================================
// 设置关键帧为坏帧：清除所有连接、观测，从地图和数据库移除
// 说明：关键帧剔除的核心函数，维护共视图、生成树的一致性
// ==============================================
void KeyFrame::SetBadFlag() {
  {
    unique_lock<mutex> lock(mMutexConnections);
    // 初始关键帧不能删
    if (mnId == mpMap->GetInitKFid()) {
      return;
    } else if (mbNotErase) {
      // 不可删除则标记待删除，延后处理
      mbToBeErased = true;
      return;
    }
  }

  // 遍历所有连接，通知对方删除与自身的连接
  for (map<std::shared_ptr<KeyFrame>, int>::iterator
           mit = mConnectedKeyFrameWeights.begin(),
           mend = mConnectedKeyFrameWeights.end();
       mit != mend; mit++) {
    mit->first->EraseConnection(shared_from_this());
  }

  // 遍历所有地图点，删除自身对它们的观测
  for (size_t i = 0; i < mvpMapPoints.size(); i++) {
    if (mvpMapPoints[i]) {
      mvpMapPoints[i]->EraseObservation(shared_from_this());
    }
  }

  {
    unique_lock<mutex> lock(mMutexConnections);
    unique_lock<mutex> lock1(mMutexFeatures);

    // 清空连接相关数据
    mConnectedKeyFrameWeights.clear();
    mvpOrderedConnectedKeyFrames.clear();

    // Update Spanning Tree
    // 更新生成树：处理子节点的父节点重连
    set<std::shared_ptr<KeyFrame>> sParentCandidates;
    if (mpParent) sParentCandidates.insert(mpParent);

    // Assign at each iteration one children with a parent (the pair with
    // highest covisibility weight) Include that children as new parent
    // candidate for the rest
    // 迭代为每个子节点寻找新的父节点
    while (!mspChildrens.empty()) {
      bool bContinue = false;

      int max = -1;
      std::shared_ptr<KeyFrame> pC;
      std::shared_ptr<KeyFrame> pP;

      // 遍历所有子节点，找与候选父节点共视最高的
      for (set<std::shared_ptr<KeyFrame>>::iterator sit = mspChildrens.begin(),
                                                     send = mspChildrens.end();
           sit != send; sit++) {
        std::shared_ptr<KeyFrame> pKF = *sit;
        if (pKF->isBad()) continue;

        // Check if a parent candidate is connected to the keyframe
        vector<std::shared_ptr<KeyFrame>> vpConnected =
            pKF->GetVectorCovisibleKeyFrames();

        // 遍历子节点的共视关键帧，找候选父节点
        for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
          for (set<std::shared_ptr<KeyFrame>>::iterator
                   spcit = sParentCandidates.begin(),
                   spcend = sParentCandidates.end();
               spcit != spcend; spcit++) {
            if (vpConnected[i]->mnId == (*spcit)->mnId) {
              int w = pKF->GetWeight(vpConnected[i]);
              // 记录最大权重的父子对
              if (w > max) {
                pC = pKF;
                pP = vpConnected[i];
                max = w;
                bContinue = true;
              }
            }
          }
        }
      }

      // 找到合适的父节点，重连
      if (bContinue) {
        pC->ChangeParent(pP);
        sParentCandidates.insert(pC);
        mspChildrens.erase(pC);
      } else {
        break;
      }
    }

    // If a children has no covisibility links with any parent candidate, assign
    // to the original parent of this KF
    // 剩余找不到父节点的子节点，挂到原父节点下
    if (!mspChildrens.empty()) {
      for (set<std::shared_ptr<KeyFrame>>::iterator sit =
               mspChildrens.begin();
           sit != mspChildrens.end(); sit++) {
        (*sit)->ChangeParent(mpParent);
      }
    }

    // 从原父节点删除自身
    if (mpParent) {
      mpParent->EraseChild(shared_from_this());
      // 记录与父节点的相对位姿
      mTcp = mTcw * mpParent->GetPoseInverse();
    }

    // 标记为坏帧
    mbBad = true;
  }

  // 从地图中移除
  mpMap->EraseKeyFrame(shared_from_this());
  // 从关键帧数据库中移除
  mpKeyFrameDB->erase(shared_from_this());
}

// ==============================================
// 判断是否为坏帧
// ==============================================
bool KeyFrame::isBad() {
  unique_lock<mutex> lock(mMutexConnections);
  return mbBad;
}

// ==============================================
// 擦除与指定关键帧的连接
// ==============================================
void KeyFrame::EraseConnection(const std::shared_ptr<KeyFrame> &pKF) {
  bool bUpdate = false;
  {
    unique_lock<mutex> lock(mMutexConnections);
    // 存在则删除
    if (mConnectedKeyFrameWeights.count(pKF)) {
      mConnectedKeyFrameWeights.erase(pKF);
      bUpdate = true;
    }
  }

  // 删除后更新排序
  if (bUpdate) UpdateBestCovisibles();
}

// ==============================================
// 获取指定圆形区域内的特征点索引
// x,y：区域中心，r：半径，bRight：是否查右目网格
// ==============================================
vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y,
                                           const float &r,
                                           const bool bRight) const {
  vector<size_t> vIndices;
  vIndices.reserve(N);

  float factorX = r;
  float factorY = r;

  // 计算区域左边界对应的网格列
  const int nMinCellX = max(0, static_cast<int>(floor((x - mnMinX - factorX) *
                                                      mfGridElementWidthInv)));
  if (nMinCellX >= mnGridCols) return vIndices;

  // 计算区域右边界对应的网格列
  const int nMaxCellX = min(
      static_cast<int>(mnGridCols) - 1,
      static_cast<int>(ceil((x - mnMinX + factorX) * mfGridElementWidthInv)));
  if (nMaxCellX < 0) return vIndices;

  // 计算区域上下边界对应的网格行
  const int nMinCellY = max(0, static_cast<int>(floor((y - mnMinY - factorY) *
                                                      mfGridElementHeightInv)));
  if (nMinCellY >= mnGridRows) return vIndices;

  const int nMaxCellY = min(
      static_cast<int>(mnGridRows) - 1,
      static_cast<int>(ceil((y - mnMinY + factorY) * mfGridElementHeightInv)));
  if (nMaxCellY < 0) return vIndices;

  // 遍历范围内的所有网格
  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      // 获取对应网格的特征点索引列表
      const vector<size_t> vCell =
          (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];

      // 遍历网格内每个特征点，检查是否在圆形区域内
      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                                    : (!bRight)   ? mvKeys[vCell[j]]
                                                  : mvKeysRight[vCell[j]];
        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        // 矩形内则加入结果
        if (fabs(distx) < r && fabs(disty) < r) vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

// ==============================================
// 判断像素坐标是否在有效图像范围内
// ==============================================
bool KeyFrame::IsInImage(const float &x, const float &y) const {
  return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
}

// ==============================================
// 立体反投影：将第i个特征点反投影为世界坐标系3D点
// ==============================================
bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D) {
  const float z = mvDepth[i];
  if (z > 0) {
    const float u = mvKeys[i].pt.x;
    const float v = mvKeys[i].pt.y;
    // 像素坐标反投影为相机坐标系3D点
    const float x = (u - cx) * z * invfx;
    const float y = (v - cy) * z * invfy;
    Eigen::Vector3f x3Dc(x, y, z);

    unique_lock<mutex> lock(mMutexPose);
    // 相机坐标转为世界坐标
    x3D = mRwc * x3Dc + mTwc.translation();
    return true;
  } else {
    return false;
  }
}

// ==============================================
// 计算场景深度的分位数（默认q=2为中位数）
// ==============================================
float KeyFrame::ComputeSceneMedianDepth(const int q) {
  if (N == 0) return -1.0;

  vector<MapPoint *> vpMapPoints;
  Eigen::Matrix3f Rcw;
  Eigen::Vector3f tcw;
  {
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPose);
    vpMapPoints = mvpMapPoints;
    tcw = mTcw.translation();
    Rcw = mRcw;
  }

  vector<float> vDepths;
  vDepths.reserve(N);

  // 取旋转矩阵第三行（z方向），用于快速计算深度
  Eigen::Matrix<float, 1, 3> Rcw2 = Rcw.row(2);
  float zcw = tcw(2);

  // 遍历所有地图点，计算相机坐标系下的z深度
  for (int i = 0; i < N; i++) {
    if (mvpMapPoints[i]) {
      MapPoint *pMP = mvpMapPoints[i];
      Eigen::Vector3f x3Dw = pMP->GetWorldPos();
      // 深度 = R第三行·P + tz
      float z = Rcw2.dot(x3Dw) + zcw;
      vDepths.push_back(z);
    }
  }

  // 深度排序
  sort(vDepths.begin(), vDepths.end());

  // 返回第q分位数
  return vDepths[(vDepths.size() - 1) / q];
}

// ==============================================
// 设置IMU新零偏，同步更新到预积分器
// ==============================================
void KeyFrame::SetNewBias(const IMU::Bias &b) {
  unique_lock<mutex> lock(mMutexPose);
  mImuBias = b;
  if (mpImuPreintegrated) mpImuPreintegrated->SetNewBias(b);
}

// ==============================================
// 获取陀螺仪零偏向量
// ==============================================
Eigen::Vector3f KeyFrame::GetGyroBias() {
  unique_lock<mutex> lock(mMutexPose);
  return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

// ==============================================
// 获取加速度计零偏向量
// ==============================================
Eigen::Vector3f KeyFrame::GetAccBias() {
  unique_lock<mutex> lock(mMutexPose);
  return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

// ==============================================
// 获取完整IMU零偏结构体
// ==============================================
IMU::Bias KeyFrame::GetImuBias() {
  unique_lock<mutex> lock(mMutexPose);
  return mImuBias;
}

// ==============================================
// 获取所属地图指针
// ==============================================
std::shared_ptr<Map> KeyFrame::GetMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mpMap;
}

// ==============================================
// 更新所属地图
// ==============================================
void KeyFrame::UpdateMap(const std::shared_ptr<Map> &pMap) {
  unique_lock<mutex> lock(mMutexMap);
  oslog::trace("Updating mpMap to {}", pMap ? "(exists)" : "NULL");
  mpMap = pMap;
}

// ==============================================
// 保存前预处理：将指针引用转为ID存储，用于序列化
// 输入：spKF 所有待存关键帧集合，spMP 所有待存地图点集合，spCam 所有待存相机集合
// ==============================================
void KeyFrame::PreSave(set<std::shared_ptr<KeyFrame>> &spKF,
                       set<MapPoint *> &spMP,
                       set<std::shared_ptr<GeometricCamera>> &spCam) {
  // Save the id of each MapPoint in this KF, there can be null pointer in the
  // vector
  // 备份每个特征点对应的地图点ID，空指针存-1
  mvBackupMapPointsId.clear();
  mvBackupMapPointsId.reserve(N);
  for (int i = 0; i < N; ++i) {
    if (mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) !=
                               spMP.end())  // Checks if the element is not null
      mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
    else  // If the element is null his value is -1 because all the id are
           // positives
      mvBackupMapPointsId.push_back(-1);
  }

  // Save the id of each connected KF with it weight
  // 备份连接关键帧的ID和权重
  mBackupConnectedKeyFrameIdWeights.clear();
  for (std::map<std::shared_ptr<KeyFrame>, int>::const_iterator
           it = mConnectedKeyFrameWeights.begin(),
           end = mConnectedKeyFrameWeights.end();
       it != end; ++it) {
    if (spKF.find(it->first) != spKF.end())
      mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
  }

  // Save the parent id
  // 备份父关键帧ID
  mBackupParentId = -1;
  if (mpParent && spKF.find(mpParent) != spKF.end())
    mBackupParentId = mpParent->mnId;

  // Save the id of the childrens KF
  // 备份子关键帧ID列表
  mvBackupChildrensId.clear();
  mvBackupChildrensId.reserve(mspChildrens.size());
  for (auto pKFi : mspChildrens) {
    if (spKF.find(pKFi) != spKF.end())
      mvBackupChildrensId.push_back(pKFi->mnId);
  }

  // Save the id of the loop edge KF
  // 备份回环边关键帧ID
  mvBackupLoopEdgesId.clear();
  mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
  for (auto pKFi : mspLoopEdges) {
    if (spKF.find(pKFi) != spKF.end())
      mvBackupLoopEdgesId.push_back(pKFi->mnId);
  }

  // Save the id of the merge edge KF
  // 备份合并边关键帧ID
  mvBackupMergeEdgesId.clear();
  mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
  for (auto pKFi : mspMergeEdges) {
    if (spKF.find(pKFi) != spKF.end())
      mvBackupMergeEdgesId.push_back(pKFi->mnId);
  }

  // Camera data
  // 备份主相机ID
  mnBackupIdCamera = -1;
  if (mpCamera && spCam.find(mpCamera) != spCam.end())
    mnBackupIdCamera = mpCamera->GetId();

  // 备份右相机ID
  mnBackupIdCamera2 = -1;
  if (mpCamera2 && spCam.find(mpCamera2) != spCam.end())
    mnBackupIdCamera2 = mpCamera2->GetId();

  // Inertial data
  // 备份前一关键帧ID
  mBackupPrevKFId = -1;
  if (mPrevKF && spKF.find(mPrevKF) != spKF.end())
    mBackupPrevKFId = mPrevKF->mnId;

  // 备份后一关键帧ID
  mBackupNextKFId = -1;
  if (mNextKF && spKF.find(mNextKF) != spKF.end())
    mBackupNextKFId = mNextKF->mnId;

  // 备份IMU预积分数据
  if (mpImuPreintegrated)
    mpBackupImuPreintegrated->CopyFrom(mpImuPreintegrated);
}

// ==============================================
// 加载后重建：从ID恢复指针引用，重建所有关联关系
// ==============================================
void KeyFrame::PostLoad(
    map<unsigned long int, std::shared_ptr<KeyFrame>> &mpKFid,
    map<unsigned long int, MapPoint *> &mpMPid,
    map<unsigned int, std::shared_ptr<GeometricCamera>> &mpCamId) {
  // Rebuild the empty variables

  // Pose
  // 恢复位姿及派生矩阵
  SetPose(mTcw);

  // 恢复右目到左目的相对位姿
  mTrl = mTlr.inverse();

  // Reference reconstruction
  // Each MapPoint sight from this KeyFrame
  // 重建地图点匹配数组
  mvpMapPoints.clear();
  mvpMapPoints.resize(N);
  for (int i = 0; i < N; ++i) {
    if (mvBackupMapPointsId[i] != -1)
      mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
    else
      mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
  }

  // Conected KeyFrames with him weight
  // 重建连接关键帧权重映射
  mConnectedKeyFrameWeights.clear();
  for (map<unsigned long int, int>::const_iterator
           it = mBackupConnectedKeyFrameIdWeights.begin(),
           end = mBackupConnectedKeyFrameIdWeights.end();
       it != end; ++it) {
    std::shared_ptr<KeyFrame> pKFi = mpKFid[it->first];
    mConnectedKeyFrameWeights[pKFi] = it->second;
  }

  // Restore parent KeyFrame
  // 恢复父关键帧
  if (mBackupParentId >= 0) mpParent = mpKFid[mBackupParentId];

  // KeyFrame childrens
  // 恢复子关键帧集合
  mspChildrens.clear();
  for (vector<unsigned long int>::const_iterator
           it = mvBackupChildrensId.begin(),
           end = mvBackupChildrensId.end();
       it != end; ++it) {
    mspChildrens.insert(mpKFid[*it]);
  }

  // Loop edge KeyFrame
  // 恢复回环边集合
  mspLoopEdges.clear();
  for (vector<unsigned long int>::const_iterator
           it = mvBackupLoopEdgesId.begin(),
           end = mvBackupLoopEdgesId.end();
       it != end; ++it) {
    mspLoopEdges.insert(mpKFid[*it]);
  }

  // Merge edge KeyFrame
  // 恢复合并边集合
  mspMergeEdges.clear();
  for (vector<unsigned long int>::const_iterator
           it = mvBackupMergeEdgesId.begin(),
           end = mvBackupMergeEdgesId.end();
       it != end; ++it) {
    mspMergeEdges.insert(mpKFid[*it]);
  }

  // Camera data
  // 恢复主相机指针
  if (mnBackupIdCamera >= 0) {
    mpCamera = mpCamId[mnBackupIdCamera];
  } else {
    cout << "ERROR: There is not a main camera in KF " << mnId << endl;
  }
  // 恢复右相机指针
  if (mnBackupIdCamera2 >= 0) {
    mpCamera2 = mpCamId[mnBackupIdCamera2];
  }

  // Inertial data
  // 恢复前一关键帧指针
  if (mBackupPrevKFId != -1) {
    mPrevKF = mpKFid[mBackupPrevKFId];
  }
  // 恢复后一关键帧指针
  if (mBackupNextKFId != -1) {
    mNextKF = mpKFid[mBackupNextKFId];
  }

  // \todo{} WAS:
  // mpImuPreintegrated = &mBackupImuPreintegrated;
  //   // Which doesn't work with shared ptrs.  Switched to:
  // 恢复IMU预积分指针
  mpImuPreintegrated = mpBackupImuPreintegrated;

  // Remove all backup container
  // 清空所有备份容器
  mvBackupMapPointsId.clear();
  mBackupConnectedKeyFrameIdWeights.clear();
  mvBackupChildrensId.clear();
  mvBackupLoopEdgesId.clear();

  // 更新共视排序
  UpdateBestCovisibles();
}

// ==============================================
// 投影地图点到图像，计算畸变后的像素坐标
// ==============================================
bool KeyFrame::ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u,
                                   float &v) {
  // 3D in absolute coordinates
  // 获取地图点世界坐标
  Eigen::Vector3f P = pMP->GetWorldPos();

  // 3D in camera coordinates
  // 转换到相机坐标系
  Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
  float &PcX = Pc(0);
  float &PcY = Pc(1);
  float &PcZ = Pc(2);

  // Check positive depth
  // 检查正深度
  if (PcZ < 0.0f) {
    cout << "Negative depth: " << PcZ << endl;
    return false;
  }

  // Project in image and check it is not outside
  // 针孔投影得到像素坐标
  float invz = 1.0f / PcZ;
  u = fx * PcX * invz + cx;
  v = fy * PcY * invz + cy;

  // cout << "c";

  // 检查是否在图像范围内
  if (u < mnMinX || u > mnMaxX) return false;
  if (v < mnMinY || v > mnMaxY) return false;

  // 转归一化坐标计算畸变
  float x = (u - cx) * invfx;
  float y = (v - cy) * invfy;
  float r2 = x * x + y * y;
  // 畸变系数
  float k1 = mDistCoef.at<float>(0);
  float k2 = mDistCoef.at<float>(1);
  float p1 = mDistCoef.at<float>(2);
  float p2 = mDistCoef.at<float>(3);
  float k3 = 0;
  if (mDistCoef.total() == 5) {
    k3 = mDistCoef.at<float>(4);
  }

  // Radial distorsion
  // 径向畸变
  float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
  float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

  // Tangential distorsion
  // 切向畸变
  x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
  y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

  // 转回像素坐标
  float u_distort = x_distort * fx + cx;
  float v_distort = y_distort * fy + cy;

  u = u_distort;
  v = v_distort;

  kp = cv::Point2f(u, v);

  return true;
}

// ==============================================
// 投影地图点到图像，返回未畸变的像素坐标
// ==============================================
bool KeyFrame::ProjectPointUnDistort(MapPoint *pMP, cv::Point2f &kp, float &u,
                                     float &v) {
  // 3D in absolute coordinates
  Eigen::Vector3f P = pMP->GetWorldPos();

  // 3D in camera coordinates
  Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
  float &PcX = Pc(0);
  float &PcY = Pc(1);
  float &PcZ = Pc(2);

  // Check positive depth
  if (PcZ < 0.0f) {
    cout << "Negative depth: " << PcZ << endl;
    return false;
  }

  // Project in image and check it is not outside
  const float invz = 1.0f / PcZ;
  u = fx * PcX * invz + cx;
  v = fy * PcY * invz + cy;

  if (u < mnMinX || u > mnMaxX) return false;
  if (v < mnMinY || v > mnMaxY) return false;

  kp = cv::Point2f(u, v);

  return true;
}

// ==============================================
// 获取右目到左目的相对位姿Trl
// ==============================================
Sophus::SE3f KeyFrame::GetRelativePoseTrl() {
  unique_lock<mutex> lock(mMutexPose);
  return mTrl;
}

// ==============================================
// 获取左目到右目的相对位姿Tlr
// ==============================================
Sophus::SE3f KeyFrame::GetRelativePoseTlr() {
  unique_lock<mutex> lock(mMutexPose);
  return mTlr;
}

// ==============================================
// 获取右相机的世界位姿
// ==============================================
Sophus::SE3<float> KeyFrame::GetRightPose() {
  unique_lock<mutex> lock(mMutexPose);

  // 右相机位姿 = 右→左相对位姿 × 左相机位姿
  return mTrl * mTcw;
}

// ==============================================
// 获取右相机的世界逆位姿
// ==============================================
Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
  unique_lock<mutex> lock(mMutexPose);

  // 右相机逆位姿 = 左相机逆位姿 × 左→右相对位姿
  return mTwc * mTlr;
}

// ==============================================
// 获取右相机光心世界坐标
// ==============================================
Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
  unique_lock<mutex> lock(mMutexPose);

  return (mTwc * mTlr).translation();
}

// ==============================================
// 获取右相机的世界旋转矩阵
// ==============================================
Eigen::Matrix<float, 3, 3> KeyFrame::GetRightRotation() {
  unique_lock<mutex> lock(mMutexPose);

  return (mTrl.so3() * mTcw.so3()).matrix();
}

// ==============================================
// 获取右相机的世界平移向量
// ==============================================
Eigen::Vector3f KeyFrame::GetRightTranslation() {
  unique_lock<mutex> lock(mMutexPose);
  return (mTrl * mTcw).translation();
}

// ==============================================
// 设置ORB词典指针
// ==============================================
void KeyFrame::SetORBVocabulary(const std::shared_ptr<ORBVocabulary> &pORBVoc) {
  mpORBvocabulary = pORBVoc;
}

// ==============================================
// 设置关键帧数据库指针
// ==============================================
void KeyFrame::SetKeyFrameDatabase(
    const std::shared_ptr<KeyFrameDatabase> &pKFDB) {
  mpKeyFrameDB = pKFDB;
}

}  // namespace ORB_SLAM3
