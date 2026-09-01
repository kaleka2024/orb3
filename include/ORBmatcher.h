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
#pragma once                          // 头文件保护，防止重复包含
#include <memory>                     // std::shared_ptr智能指针
#include <opencv2/core/core.hpp>      // OpenCV核心模块，Mat、Point等基础数据结构
#include <opencv2/features2d/features2d.hpp> // OpenCV特征点相关，KeyPoint、描述子
#include <set>                        // std::set有序集合容器
#include <utility>                    // std::pair键值对
#include <vector>                     // std::vector动态数组

#include "Frame.h"                    // 普通帧类定义
#include "KeyFrame.h"                 // 关键帧类定义
#include "MapPoint.h"                 // 地图点类定义
#include "sophus/sim3.hpp"            // Sophus库Sim3相似变换，包含旋转、平移、尺度

namespace ORB_SLAM3 {

/**
 * @brief ORB特征匹配器，封装SLAM各类匹配逻辑：投影匹配、词袋匹配、初始化匹配、三角化匹配、地图点融合等
 * @details 核心使用汉明距离做描述子匹配，支持方向一致性检查、比例测试过滤误匹配
 */
class ORBmatcher {
 public:
  /**
   * @brief ORB匹配器构造函数
   * @param nnratio 最近邻/次近邻汉明距离比例阈值，用于剔除歧义匹配，默认0.6
   * @param checkOri 是否开启特征点旋转方向一致性校验，默认开启
   */
  explicit ORBmatcher(float nnratio = 0.6, bool checkOri = true);

  /**
   * @brief 静态函数，计算两个ORB描述子之间的汉明距离，统计不同bit位数
   * @param a 第一个ORB描述子，cv::Mat类型(256bit/32字节)
   * @param b 第二个ORB描述子
   * @return int 返回汉明距离，数值越小代表描述子越相似
   */
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

  // Search matches between Frame keypoints and projected MapPoints. Returns
  // number of matches Used to track the local map (Tracking)
  /**
   * @brief 将局部地图点投影到当前普通帧图像上，在投影邻域内搜索匹配，Tracking局部地图跟踪使用
   * @param F 当前待跟踪普通帧
   * @param vpMapPoints 待投影的局部地图点集合
   * @param th 像素搜索半径阈值
   * @param bFarPoints 是否开启远点特殊处理
   * @param thFarPoints 远点距离阈值
   * @return int 返回成功匹配的数量
   */
  int SearchByProjection(const std::shared_ptr<Frame> &F,
                         const std::vector<MapPoint *> &vpMapPoints,
                         const float th = 3, const bool bFarPoints = false,
                         const float thFarPoints = 50.0f);

  // Project MapPoints tracked in last frame into the current frame and search
  // matches. Used to track from previous frame (Tracking)
  /**
   * @brief 将上一帧的地图点投影到当前帧，搜索匹配，用于帧间跟踪(运动模型跟踪)
   * @param CurrentFrame 当前帧
   * @param LastFrame 上一帧
   * @param th 像素搜索半径阈值
   * @param bMono 是否单目模式
   * @return int 返回匹配成功数目
   */
  int SearchByProjection(const std::shared_ptr<Frame> &CurrentFrame,
                         const std::shared_ptr<Frame> &LastFrame,
                         const float th, const bool bMono);

  // Project MapPoints seen in KeyFrame into the Frame and search matches.
  // Used in relocalisation (Tracking)
  /**
   * @brief 将关键帧关联的地图点投影到普通帧，搜索匹配，重定位Relocalisation时调用
   * @param CurrentFrame 当前待重定位普通帧
   * @param pKF 候选关键帧
   * @param sAlreadyFound 已经匹配成功的地图点集合，避免重复匹配
   * @param th 像素搜索半径阈值
   * @param ORBdist 汉明距离最大允许阈值
   * @return int 返回匹配数量
   */
  int SearchByProjection(const std::shared_ptr<Frame> &CurrentFrame,
                         const std::shared_ptr<KeyFrame> &pKF,
                         const std::set<MapPoint *> &sAlreadyFound,
                         const float th, const int ORBdist);

  // Project MapPoints using a Similarity Transformation and search matches.
  // Used in loop detection (Loop Closing)
  /**
   * @brief 使用Sim3相似变换把地图点变换到关键帧坐标系后投影，回环检测使用
   * @param pKF 目标关键帧
   * @param Scw Sim3变换矩阵，世界坐标系到关键帧坐标系的相似变换
   * @param vpPoints 待变换投影的地图点数组
   * @param vpMatched 输出，匹配成功的地图点数组，下标对应pKF关键点
   * @param th 像素搜索半径阈值
   * @param ratioHamming 汉明距离比例缩放系数
   * @return int 返回匹配成功数目
   */
  int SearchByProjection(const std::shared_ptr<KeyFrame> &pKF,
                         Sophus::Sim3<float> &Scw,
                         const std::vector<MapPoint *> &vpPoints,
                         std::vector<MapPoint *> &vpMatched, int th,
                         float ratioHamming = 1.0);

  // Project MapPoints using a Similarity Transformation and search matches.
  // Used in Place Recognition (Loop Closing and Merging)
  /**
   * @brief Sim3投影匹配重载版本，地图点来自多个源关键帧，用于回环、地图融合场景识别
   * @param pKF 目标关键帧
   * @param Scw 世界到pKF的Sim3相似变换
   * @param vpPoints 待投影地图点集合
   * @param vpPointsKFs 每个地图点对应的源关键帧
   * @param vpMatched 输出匹配成功地图点
   * @param vpMatchedKF 输出匹配成功地图点对应的源关键帧
   * @param th 像素搜索半径阈值
   * @param ratioHamming 汉明距离缩放系数
   * @return int 返回匹配数目
   */
  int SearchByProjection(
      const std::shared_ptr<KeyFrame> &pKF, Sophus::Sim3<float> &Scw,
      const std::vector<MapPoint *> &vpPoints,
      const std::vector<std::shared_ptr<KeyFrame>> &vpPointsKFs,
      std::vector<MapPoint *> &vpMatched,
      std::vector<std::shared_ptr<KeyFrame>> &vpMatchedKF, int th,
      float ratioHamming = 1.0);

  // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
  // Brute force constrained to ORB that belong to the same vocabulary node (at
  // a certain level) Used in Relocalisation and Loop Detection
  /**
   * @brief 基于词袋BoW匹配，关键帧地图点与普通帧关键点匹配，只在同一词袋节点内做匹配，重定位、回环检测使用
   * @param pKF 候选关键帧
   * @param F 当前普通帧
   * @param vpMapPointMatches 输出匹配结果，下标对应普通帧关键点，存储匹配的地图点
   * @return int 返回匹配数量
   */
  int SearchByBoW(const std::shared_ptr<KeyFrame> &pKF,
                  const std::shared_ptr<Frame> &F,
                  std::vector<MapPoint *> &vpMapPointMatches);

  /**
   * @brief 词袋BoW匹配，两个关键帧之间做匹配，回环检测中计算两关键帧匹配点
   * @param pKF1 关键帧1
   * @param pKF2 关键帧2
   * @param vpMatches12 输出匹配结果，下标对应pKF1关键点，存储pKF2匹配到的地图点
   * @return int 返回匹配数目
   */
  int SearchByBoW(const std::shared_ptr<KeyFrame> &pKF1,
                  const std::shared_ptr<KeyFrame> &pKF2,
                  std::vector<MapPoint *> &vpMatches12);

  // Matching for the Map Initialization (only used in the monocular case)
  /**
   * @brief 单目初始化专用匹配函数，对两帧图像关键点做窗口内匹配，用于单目初始化求基础矩阵
   * @param F1 参考帧
   * @param F2 当前帧
   * @param vbPrevMatched 输入输出，F1关键点坐标，作为匹配搜索中心
   * @param vnMatches12 输出匹配关系，下标F1关键点id，存储F2匹配关键点id，-1代表无匹配
   * @param windowSize 图像上搜索窗口半宽
   * @return int 返回匹配成功对数
   */
  int SearchForInitialization(const std::shared_ptr<Frame> &F1,
                              const std::shared_ptr<Frame> &F2,
                              std::vector<cv::Point2f> &vbPrevMatched,
                              std::vector<int> &vnMatches12,
                              int windowSize = 10);

  // Matching to triangulate new MapPoints. Check Epipolar Constraint.
  /**
   * @brief 用于三角化生成新地图点，两关键帧之间匹配，校验极线约束，筛选可三角化的匹配对
   * @param pKF1 关键帧1
   * @param pKF2 关键帧2
   * @param vMatchedPairs 输出匹配点对，存储pair<kf1_kpt_id, kf2_kpt_id>
   * @param bOnlyStereo 是否仅双目模式
   * @param bCoarse 是否粗匹配模式
   * @return int 返回匹配对数
   */
  int SearchForTriangulation(const std::shared_ptr<KeyFrame> &pKF1,
                             const std::shared_ptr<KeyFrame> &pKF2,
                             std::vector<pair<size_t, size_t>> &vMatchedPairs,
                             const bool bOnlyStereo,
                             const bool bCoarse = false);

  // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3
  // [s12*R12|t12] In the stereo and RGB‑D case, s12=1
  /**
   * @brief 根据已知Sim3相似变换，在两关键帧之间搜索地图点匹配，回环Sim3求解使用
   * @param pKF1 源关键帧
   * @param pKF2 目标关键帧
   * @param vpMatches12 输入输出匹配数组，下标对应pKF1关键点，存储pKF2地图点
   * @param S12 pKF1到pKF2的Sim3相似变换
   * @param th 像素搜索半径阈值
   * @return int 返回匹配数目
   */
  int SearchBySim3(const std::shared_ptr<KeyFrame> &pKF1,
                   const std::shared_ptr<KeyFrame> &pKF2,
                   std::vector<MapPoint *> &vpMatches12,
                   const Sophus::Sim3f &S12, const float th);

  // Project MapPoints into KeyFrame and search for duplicated MapPoints.
  /**
   * @brief 地图点融合Fuse，将传入地图点投影到目标关键帧，检测重复地图点，完成融合替换，局部建图线程调用
   * @param pKF 目标关键帧
   * @param vpMapPoints 待融合的地图点集合
   * @param th 像素搜索半径阈值
   * @param bRight 是否使用双目右图
   * @return int 返回发生融合的地图点数量
   */
  int Fuse(const std::shared_ptr<KeyFrame> &pKF,
           const vector<MapPoint *> &vpMapPoints, const float th = 3.0,
           const bool bRight = false);

  // Project MapPoints into KeyFrame using a given Sim3 and search for
  // duplicated MapPoints.
  /**
   * @brief 带Sim3变换的地图点融合，回环校正、地图融合时使用，先做Sim3变换再投影，检测重复地图点
   * @param pKF 目标关键帧
   * @param Scw 世界坐标系到pKF的Sim3变换
   * @param vpPoints 待融合地图点集合
   * @param th 像素搜索半径阈值
   * @param vpReplacePoint 输出需要被替换的地图点
   * @return int 返回融合成功数量
   */
  int Fuse(const std::shared_ptr<KeyFrame> &pKF, Sophus::Sim3f &Scw,
           const std::vector<MapPoint *> &vpPoints, float th,
           vector<MapPoint *> &vpReplacePoint);

 public:
  static const int TH_LOW;       ///< 低汉明距离阈值，严格匹配
  static const int TH_HIGH;      ///< 高汉明距离阈值，宽松匹配
  static const int HISTO_LENGTH; ///< 方向直方图的bin数目，用于方向一致性校验

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW ///< Eigen内存对齐宏，类中使用Sophus/Sim3需要

 protected:
  /**
   * @brief 根据观测夹角cos值计算投影匹配搜索半径，观测角度越小搜索半径越大
   * @param viewCos 观测方向余弦
   * @return float 返回像素搜索半径
   */
  [[nodiscard]] float RadiusByViewingCos(float viewCos) const;

  /**
   * @brief 计算方向直方图中三个最大峰值，用于方向一致性过滤，剔除偏离主方向的误匹配
   * @param histo 方向直方图数组指针
   * @param L 直方图bin总长度
   * @param ind1 输出第一大峰值下标
   * @param ind2 输出第二大峰值下标
   * @param ind3 输出第三大峰值下标
   */
  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1,
                          int &ind2, int &ind3);

  float mfNNratio;       ///< 最近邻次近邻距离比例阈值，用于Lowe比例测试
  bool mbCheckOrientation; ///< 是否开启特征点旋转方向一致性检查
};

}  // namespace ORB_SLAM3
