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
#include <memory>                     // std::shared_ptr智能指针
#include <opencv2/opencv.hpp>         // OpenCV基础库
#include <vector>                     // std::vector容器

#include "KeyFrame.h"                 // 关键帧类定义

namespace ORB_SLAM3 {

/**
 * @brief Sim3求解器，用于计算两关键帧之间的相似变换Sim(3)：旋转+平移+尺度
 * @details 主要用于回环检测、地图融合；单目模式尺度未知，需要求解尺度；双目/RGBD深度已知，尺度固定为1
 * Sim3变换矩阵 T = [ sR  t; 0 0 0 1 ]，s为尺度因子，R旋转，t平移
 */
class Sim3Solver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Eigen内存对齐宏，用于Eigen成员变量内存对齐

  /**
   * @brief Sim3Solver构造函数
   * @param pKF1 参考关键帧KF1
   * @param pKF2 待匹配关键帧KF2
   * @param vpMatched12 KF1与KF2相匹配的地图点数组，vpMatched12[i]为KF1第i个地图点在KF2中的匹配点
   * @param bFixScale 是否固定尺度，双目/RGBD深度可信设置true(s=1)；单目false求解尺度
   * @param vpKeyFrameMatchedMP 匹配地图点对应的关键帧，可选参数
   */
  Sim3Solver(const std::shared_ptr<KeyFrame> &pKF1,
             const std::shared_ptr<KeyFrame> &pKF2,
             const std::vector<MapPoint *> &vpMatched12,
             const bool bFixScale = true,
             const vector<std::shared_ptr<KeyFrame>> vpKeyFrameMatchedMP =
                 vector<std::shared_ptr<KeyFrame>>());

  /**
   * @brief 设置RANSAC随机采样参数
   * @param probability RANSAC成功概率，一般0.99
   * @param minInliers 判定求解成功的最少内点数量，最少6对3D点
   * @param maxIterations RANSAC最大迭代次数
   */
  void SetRansacParameters(double probability = 0.99, int minInliers = 6,
                           int maxIterations = 300);

  /**
   * @brief 执行完整RANSAC流程，求解Sim3变换
   * @param vbInliers12 输出，标记每一对匹配点是否为内点
   * @param nInliers 输出，内点总个数
   * @return Eigen::Matrix4f 返回求得的T12：将KF2坐标系点变换到KF1坐标系 T12 = [sR t;0 1]
   */
  Eigen::Matrix4f find(std::vector<bool> &vbInliers12, int &nInliers);

  /**
   * @brief 执行指定次数RANSAC迭代，不做完整循环
   * @param nIterations 本次要执行的迭代次数
   * @param bNoMore 输出，true表示已经达到最大迭代次数，RANSAC结束
   * @param vbInliers 输出，内点标记数组
   * @param nInliers 输出，当前最优内点数目
   * @return Eigen::Matrix4f 当前最优T12变换矩阵
   */
  Eigen::Matrix4f iterate(int nIterations, bool &bNoMore,
                           std::vector<bool> &vbInliers, int &nInliers);

  /**
   * @brief iterate重载版本，额外输出收敛标志
   * @param nIterations 本次迭代次数
   * @param bNoMore 输出，是否迭代结束
   * @param vbInliers 输出内点标记
   * @param nInliers 输出内点数量
   * @param bConverge 输出，true表示已经收敛得到合格解
   * @return Eigen::Matrix4f 当前最优T12变换矩阵
   */
  Eigen::Matrix4f iterate(int nIterations, bool &bNoMore,
                          vector<bool> &vbInliers, int &nInliers,
                          bool &bConverge);

  Eigen::Matrix4f GetEstimatedTransformation(); ///< 获取当前估计的T12 Sim3矩阵
  Eigen::Matrix3f GetEstimatedRotation();        ///< 获取估计旋转矩阵R12
  Eigen::Vector3f GetEstimatedTranslation();    ///< 获取估计平移向量t12
  float GetEstimatedScale();                     ///< 获取估计尺度s12

 protected:
  /**
   * @brief 计算点集质心，并且对点做去中心化
   * @param P 输出，去中心化后的第一组3D点矩阵
   * @param Pr 输出，去中心化后的第二组3D点矩阵
   * @param C 输出，第一组点的质心
   */
  void ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr,
                       Eigen::Vector3f &C);

  /**
   * @brief 传入两组3D点，求解Sim3变换（SVD求解相似变换）
   * @param P1 KF1相机坐标系下3D点
   * @param P2 KF2相机坐标系下3D点
   * @note 结果存到成员变量 mR12i, mt12i, ms12i，得到T12i
   */
  void ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2);

  /**
   * @brief 使用当前Sim3模型，对全部匹配点做重投影误差计算，筛选内点外点
   * @note 结果存入mvbInliersi，mnInliersi
   */
  void CheckInliers();

  /**
   * @brief 将世界坐标系3D点，通过Tcw投影到图像得到2D像素坐标
   * @param vP3Dw 输入世界坐标系3D点
   * @param vP2D 输出图像2D归一化/像素点
   * @param Tcw 相机位姿世界到相机Tcw
   * @param pCamera 相机模型对象，支持针孔、鱼眼
   */
  void Project(const std::vector<Eigen::Vector3f> &vP3Dw,
               std::vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw,
               const std::shared_ptr<GeometricCamera> &pCamera);

  /**
   * @brief 将相机坐标系3D点投影到图像得到像素坐标
   * @param vP3Dc 相机坐标系下三维点
   * @param vP2D 输出图像二维像素点
   * @param pCamera 相机模型
   */
  void FromCameraToImage(const std::vector<Eigen::Vector3f> &vP3Dc,
                         std::vector<Eigen::Vector2f> &vP2D,
                         const std::shared_ptr<GeometricCamera> &pCamera);

 protected:
  // KeyFrames and matches 关键帧与匹配点相关成员
  std::shared_ptr<KeyFrame> mpKF1;    ///< 参考关键帧KF1
  std::shared_ptr<KeyFrame> mpKF2;    ///< 待求解关键帧KF2

  std::vector<Eigen::Vector3f> mvX3Dc1; ///< KF1相机坐标系下匹配地图点
  std::vector<Eigen::Vector3f> mvX3Dc2; ///< KF2相机坐标系下匹配地图点
  std::vector<MapPoint *> mvpMapPoints1; ///< KF1的地图点
  std::vector<MapPoint *> mvpMapPoints2; ///< KF2的地图点
  std::vector<MapPoint *> mvpMatches12;  ///< KF1->KF2匹配地图点
  std::vector<size_t> mvnIndices1;       ///< 有效匹配点在KF1中的索引
  std::vector<size_t> mvSigmaSquare1;   ///< KF1各点的误差方差平方
  std::vector<size_t> mvSigmaSquare2;   ///< KF2各点的误差方差平方
  std::vector<size_t> mvnMaxError1;     ///< KF1各点最大允许误差阈值
  std::vector<size_t> mvnMaxError2;     ///< KF2各点最大允许误差阈值

  int N;      ///< 有效匹配点总对数
  int mN1;    ///< KF1参与计算的点数量

  // Current Estimation 当前这一轮RANSAC采样得到的Sim3模型
  Eigen::Matrix3f mR12i;     ///< 当前迭代R12旋转
  Eigen::Vector3f mt12i;     ///< 当前迭代t12平移
  float ms12i;               ///< 当前迭代尺度s12
  Eigen::Matrix4f mT12i;     ///< 当前迭代T12：KF2→KF1变换
  Eigen::Matrix4f mT21i;     ///< 当前迭代T21：KF1→KF2变换
  std::vector<bool> mvbInliersi; ///< 当前迭代内点标记
  int mnInliersi;            ///< 当前迭代内点数量

  // Current Ransac State RANSAC最优解状态
  int mnIterations;                 ///< 已经执行过的RANSAC迭代次数
  std::vector<bool> mvbBestInliers; ///< 历史最优解的内点标记
  int mnBestInliers;                ///< 历史最大内点数目
  Eigen::Matrix4f mBestT12;         ///< 最优Sim3变换矩阵T12
  Eigen::Matrix3f mBestRotation;    ///< 最优旋转R12
  Eigen::Vector3f mBestTranslation; ///< 最优平移t12
  float mBestScale;                 ///< 最优尺度s12

  // Scale is fixed to 1 in the stereo/RGBD case
  bool mbFixScale; ///< true固定尺度为1(双目/RGBD)；false求解尺度(单目)

  // Indices for random selection 用于RANSAC随机采样的全部有效点索引
  std::vector<size_t> mvAllIndices;

  // Projections 预计算的图像像素坐标
  std::vector<Eigen::Vector2f> mvP1im1; ///< KF1地图点在KF1图像上像素坐标
  std::vector<Eigen::Vector2f> mvP2im2; ///< KF2地图点在KF2图像上像素坐标

  // RANSAC probability RANSAC参数
  double mRansacProb;      ///< RANSAC期望成功概率

  // RANSAC min inliers
  int mRansacMinInliers;   ///< 判定解有效的最少内点

  // RANSAC max iterations
  int mRansacMaxIts;       ///< RANSAC最大迭代次数

  // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
  float mTh;               ///< 重投影误差阈值
  float mSigma2;           ///< 特征点方差平方

  // Calibration 相机标定参数，旧版cv::Mat K已废弃，改用GeometricCamera对象
  // cv::Mat mK1;
  // cv::Mat mK2;

  std::shared_ptr<GeometricCamera> pCamera1, pCamera2; ///< KF1、KF2对应的相机模型
};

}  // namespace ORB_SLAM3
