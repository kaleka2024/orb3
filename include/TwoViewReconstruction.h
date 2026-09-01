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
#pragma once
#include <Eigen/Core>                 // Eigen矩阵基础模块
#include <opencv2/opencv.hpp>         // OpenCV全套接口，关键点、点类型
#include <sophus/se3.hpp>             // Sophus SE3李代数，位姿T21
#include <unordered_set>              // 无序哈希集合
#include <utility>                    // std::pair
#include <vector>                     // std::vector动态数组

namespace ORB_SLAM3 {

/**
 * @brief TwoViewReconstruction 两视图重建类，单目初始化核心
 * @details 同时并行求解单应矩阵H与基础矩阵F，打分择优；
 * 分解矩阵得到两帧之间相对位姿T21，三角化恢复3D地图点；
 * 单目初始化时，参考帧为Frame1，当前帧为Frame2，输出T21为从帧1到帧2的变换。
 */
class TwoViewReconstruction {
  typedef std::pair<int, int> Match;  ///< Match：first为帧1关键点索引，second为帧2关键点索引

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Eigen内存对齐宏，类含有Eigen成员必须添加

  /**
   * @brief 构造TwoViewReconstruction，固定参考相机内参
   * @param k 相机内参矩阵K
   * @param sigma 重投影误差标准差，用于RANSAC判断内点阈值，默认1.0
   * @param iterations RANSAC最大迭代次数，默认200
   */
  TwoViewReconstruction(const Eigen::Matrix3f &k, float sigma = 1.0,
                        int iterations = 200);

  /**
   * @brief 两视图重建主入口，并行计算H单应、F基础矩阵，选择最优模型，恢复运动与3D结构
   * @param vKeys1 参考帧(Frame1)关键点集合
   * @param vKeys2 当前帧(Frame2)关键点集合
   * @param vMatches12 匹配关系，vMatches12[i]=j代表帧1第i个关键点匹配帧2第j个关键点，-1表示无匹配
   * @param T21 [out] 输出位姿 Sophus::SE3f，T21：将帧1坐标系下的点变换到帧2坐标系 P2 = T21 * P1
   * @param vP3D [out] 三角化得到的3D世界点（在帧1坐标系下）
   * @param vbTriangulated [out] bool标记，标记对应匹配点是否成功三角化
   * @return true 重建成功；false 重建失败，无法得到有效位姿与足够3D点
   */
  bool Reconstruct(const std::vector<cv::KeyPoint> &vKeys1,
                   const std::vector<cv::KeyPoint> &vKeys2,
                   const std::vector<int> &vMatches12, Sophus::SE3f &T21,
                   std::vector<cv::Point3f> &vP3D,
                   std::vector<bool> &vbTriangulated);

 private:
  /**
   * @brief RANSAC求解单应矩阵H21，帧1到帧2的单应变换
   * @param vbMatchesInliers [out] 匹配点是否为H模型内点标记
   * @param score [out] H模型得分，用于和F模型比较择优
   * @param H21 [out] 输出3×3单应矩阵H21
   */
  void FindHomography(std::vector<bool> &vbMatchesInliers, float &score,
                      Eigen::Matrix3f &H21);

  /**
   * @brief RANSAC求解基础矩阵F21，对极几何基础矩阵
   * @param vbInliers [out] F模型内点标记
   * @param score [out] F模型得分，用于和H模型比较择优
   * @param F21 [out] 输出3×3基础矩阵F21
   */
  void FindFundamental(std::vector<bool> &vbInliers, float &score,
                       Eigen::Matrix3f &F21);

  /**
   * @brief 直接计算单应矩阵H21，使用归一化DLT算法，输入归一化后的图像点
   * @param vP1 帧1归一化图像点
   * @param vP2 帧2归一化图像点
   * @return Eigen::Matrix3f H21 单应矩阵
   */
  Eigen::Matrix3f ComputeH21(const std::vector<cv::Point2f> &vP1,
                             const std::vector<cv::Point2f> &vP2);

  /**
   * @brief 直接计算基础矩阵F21，8点算法，输入归一化图像点
   * @param vP1 帧1归一化图像点
   * @param vP2 帧2归一化图像点
   * @return Eigen::Matrix3f F21 基础矩阵
   */
  Eigen::Matrix3f ComputeF21(const std::vector<cv::Point2f> &vP1,
                             const std::vector<cv::Point2f> &vP2);

  /**
   * @brief 评估单应矩阵H，计算得分，标记内点，利用对称转移误差
   * @param H21 帧1→帧2单应矩阵
   * @param H12 帧2→帧1单应矩阵（H21的逆）
   * @param vbMatchesInliers [out] 内点标记
   * @param sigma 误差标准差阈值
   * @return float 返回H模型总得分，分数越高模型越好
   */
  float CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12,
                        std::vector<bool> &vbMatchesInliers, float sigma);

  /**
   * @brief 评估基础矩阵F，计算对极误差，标记内点，计算模型得分
   * @param F21 基础矩阵
   * @param vbMatchesInliers [out] 内点标记
   * @param sigma 误差标准差阈值
   * @return float 返回F模型总得分
   */
  float CheckFundamental(const Eigen::Matrix3f &F21,
                         std::vector<bool> &vbMatchesInliers, float sigma);

  /**
   * @brief 使用基础矩阵F恢复两帧相对位姿R,t，三角化3D点，筛选有效解
   * @param vbMatchesInliers F模型内点标记
   * @param F21 基础矩阵
   * @param K 相机内参矩阵
   * @param T21 [out] 输出帧1到帧2的SE3位姿
   * @param vP3D [out] 三角化得到3D点（帧1坐标系）
   * @param vbTriangulated [out] 标记点是否三角化成功
   * @param minParallax 最小视差角度阈值，视差过小丢弃解
   * @param minTriangulated 最少成功三角化点数量阈值
   * @return true 得到合法位姿与足够3D点；false 无解
   */
  bool ReconstructF(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21,
                    Eigen::Matrix3f &K, Sophus::SE3f &T21,
                    std::vector<cv::Point3f> &vP3D,
                    std::vector<bool> &vbTriangulated, float minParallax,
                    int minTriangulated);

  /**
   * @brief 使用单应矩阵H恢复两帧相对位姿R,t，三角化3D点，筛选有效解
   * @param vbMatchesInliers H模型内点标记
   * @param H21 单应矩阵
   * @param K 相机内参矩阵
   * @param T21 [out] 输出帧1到帧2的SE3位姿
   * @param vP3D [out] 三角化3D点（帧1坐标系）
   * @param vbTriangulated [out] 标记点是否三角化成功
   * @param minParallax 最小视差角度阈值
   * @param minTriangulated 最少成功三角化点数量阈值
   * @return true 得到合法位姿与足够3D点；false 无解
   */
  bool ReconstructH(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21,
                    Eigen::Matrix3f &K, Sophus::SE3f &T21,
                    std::vector<cv::Point3f> &vP3D,
                    std::vector<bool> &vbTriangulated, float minParallax,
                    int minTriangulated);

  /**
   * @brief 图像点归一化，归一化DLT预处理：平移缩放使点分布中心化，提升H/F求解稳定性
   * @param vKeys 输入原始关键点
   * @param vNormalizedPoints [out] 输出归一化之后的2D像素点
   * @param T [out] 输出归一化变换矩阵（3×3），原始点左乘T得到归一化点
   */
  void Normalize(const std::vector<cv::KeyPoint> &vKeys,
                 std::vector<cv::Point2f> &vNormalizedPoints,
                 Eigen::Matrix3f &T);

  /**
   * @brief 校验R,t四组候选解；三角化匹配点，统计视差，筛选满足深度为正的有效解
   * @param R 候选旋转矩阵R21
   * @param t 候选平移向量t21
   * @param vKeys1 帧1关键点
   * @param vKeys2 帧2关键点
   * @param vMatches12 匹配对集合
   * @param vbMatchesInliers 当前模型内点标记
   * @param K 相机内参
   * @param vP3D [out] 输出三角化得到3D点
   * @param th2 重投影误差阈值平方
   * @param vbGood [out] 标记哪些匹配点三角化成功、深度为正
   * @param parallax [out] 输出所有成功点平均视差角度
   * @return int 返回成功三角化、深度为正的点数量
   */
  int CheckRT(const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
              const std::vector<cv::KeyPoint> &vKeys1,
              const std::vector<cv::KeyPoint> &vKeys2,
              const std::vector<Match> &vMatches12,
              std::vector<bool> &vbMatchesInliers, const Eigen::Matrix3f &K,
              std::vector<cv::Point3f> &vP3D, float th2,
              std::vector<bool> &vbGood, float &parallax);

  /**
   * @brief 本质矩阵E分解，得到两组候选旋转R1,R2和平移t；E = [t]_×R
   * @param E 输入3×3本质矩阵
   * @param R1 [out] 候选旋转解1
   * @param R2 [out] 候选旋转解2
   * @param t [out] 候选平移向量（有正负两种符号）
   */
  void DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1,
                  Eigen::Matrix3f &R2, Eigen::Vector3f &t);

  // Keypoints from Reference Frame (Frame 1)
  std::vector<cv::KeyPoint> mvKeys1;    ///< 参考帧Frame1关键点

  // Keypoints from Current Frame (Frame 2)
  std::vector<cv::KeyPoint> mvKeys2;    ///< 当前帧Frame2关键点

  // Current Matches from Reference to Current
  std::vector<Match> mvMatches12;       ///< 参考帧到当前帧匹配对，pair<int,int>
  std::vector<bool> mvbMatched1;        ///< 标记Frame1关键点是否存在匹配

  // Calibration
  Eigen::Matrix3f mK;                   ///< 相机内参矩阵K

  // Standard Deviation and Variance
  float mSigma, mSigma2;                ///< 误差标准差sigma，sigma平方，用于RANSAC阈值计算

  // Ransac max iterations
  int mMaxIterations;                   ///< RANSAC最大迭代次数

  // Ransac sets
  std::vector<std::vector<size_t> > mvSets; ///< RANSAC预生成随机采样集合，每组8个点索引
};

}  // namespace ORB_SLAM3
