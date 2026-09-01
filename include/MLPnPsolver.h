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
/******************************************************************************
 * Author:   Steffen Urban                                              *
 * Contact:  urbste@gmail.com                                          *
 * License:  Copyright (c) 2016 Steffen Urban, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/
#pragma once // 头文件保护，防止头文件重复包含造成编译重定义
#include <Eigen/Dense>          // Eigen稠密矩阵线性代数库
#include <Eigen/Sparse>         // Eigen稀疏矩阵库，MLPnP高斯牛顿求解使用
#include <memory>               // std::shared_ptr智能指针
#include <vector>               // std::vector动态数组容器

#include "Frame.h"              // Frame普通帧类定义
#include "MapPoint.h"           // MapPoint地图点类定义

namespace ORB_SLAM3 {

/**
 * @brief MLPnP求解器，结合RANSAC，从2D‑3D匹配对求解相机位姿Tcw
 * @details MLPnP：Maximum Likelihood PnP，最大似然PnP算法，考虑特征点噪声协方差；
 * Tracking模块初始化相机位姿时调用；内置RANSAC外点剔除，支持高斯牛顿非线性优化；
 * 原始代码来自Steffen Urban，ORB‑SLAM3做适配封装
 */
class MLPnPsolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类中存在Eigen对齐类型成员必须添加

  /**
   * @brief MLPnP求解器构造函数
   * @param F 输入普通帧Frame
   * @param vpMapPointMatches 和帧特征匹配的地图点数组，2D‑3D匹配对
   */
  MLPnPsolver(const std::shared_ptr<Frame>& F,
              const vector<MapPoint*>& vpMapPointMatches);

  ~MLPnPsolver(); // 析构函数

  /**
   * @brief 设置RANSAC求解参数
   * @param probability RANSAC成功概率
   * @param minInliers 最少内点数量阈值
   * @param maxIterations RANSAC最大迭代次数
   * @param minSet 每次随机采样最小样本集合大小
   * @param epsilon 预估内点占比
   * @param th2 卡方阈值，重投影误差平方阈值
   */
  void SetRansacParameters(double probability = 0.99, int minInliers = 8,
                           int maxIterations = 300, int minSet = 6,
                           float epsilon = 0.4, float th2 = 5.991);

  // Find metod is necessary?
  /**
   * @brief 执行RANSAC迭代求解位姿
   * @param nIterations 本次允许执行迭代次数
   * @param bNoMore 输出标记，true代表已经达到收敛条件，不需要继续迭代
   * @param vbInliers 输出布尔数组，标记每个匹配对是否为内点
   * @param nInliers 输出内点总个数
   * @param Tout 输出求解得到相机位姿Tcw 4×4矩阵
   * @return true求解成功，false求解失败
   */
  bool iterate(int nIterations, bool& bNoMore, vector<bool>& vbInliers,
               int& nInliers, Eigen::Matrix4f& Tout);

  // Type definitions needed by the original code
  /** A 3‑vector of unit length used to describe landmark observations/bearings
   *  in camera frames (always expressed in camera frames)
   */
  typedef Eigen::Vector3d bearingVector_t; ///< 观测方向单位向量（相机坐标系下归一化射线）

  /** An array of bearing‑vectors */
  typedef std::vector<bearingVector_t,
                      Eigen::aligned_allocator<bearingVector_t> >
      bearingVectors_t; ///< 观测射线向量数组，Eigen对齐分配器

  /** A 2‑matrix containing the 2D covariance information of a bearing vector
   */
  typedef Eigen::Matrix2d cov2_mat_t; ///< 二维协方差矩阵类型

  /** A 3‑matrix containing the 3D covariance information of a bearing vector */
  typedef Eigen::Matrix3d cov3_mat_t; ///< 三维协方差矩阵类型

  /** An array of 3D covariance matrices */
  typedef std::vector<cov3_mat_t, Eigen::aligned_allocator<cov3_mat_t> >
      cov3_mats_t; ///< 三维协方差矩阵数组，Eigen对齐分配器

  /** A 3‑vector describing a point in 3D‑space */
  typedef Eigen::Vector3d point_t; ///< 三维空间点向量类型

  /** An array of 3D‑points */
  typedef std::vector<point_t, Eigen::aligned_allocator<point_t> > points_t; ///< 三维点数组

  /** A homogeneous 3‑vector describing a point in 3D‑space */
  typedef Eigen::Vector4d point4_t; ///< 齐次四维3D点

  /** An array of homogeneous 3D‑points */
  typedef std::vector<point4_t, Eigen::aligned_allocator<point4_t> > points4_t; ///< 齐次点数组

  /** A 3‑vector containing the rodrigues parameters of a rotation matrix */
  typedef Eigen::Vector3d rodrigues_t; ///< Rodrigues旋转轴角参数向量

  /** A rotation matrix */
  typedef Eigen::Matrix3d rotation_t; ///< 3×3旋转矩阵

  /** A 3x4 transformation matrix containing rotation \f$ \mathbf{R} \f$ and
   *  translation \f$ \mathbf{t} \f$ as follows:
   *  \f$ \left( \begin{array}{cc} \mathbf{R} & \mathbf{t} \end{array} \right)
   * \f$
   */
  typedef Eigen::Matrix<double, 3, 4> transformation_t; ///< 3×4位姿矩阵 [R | t]

  /** A 3‑vector describing a translation/camera position */
  typedef Eigen::Vector3d translation_t; ///< 平移向量类型

 private:
  /**
   * @brief 根据当前估计位姿，检查所有匹配对，统计内点外点
   */
  void CheckInliers();

  /**
   * @brief 使用高斯牛顿对RANSAC得到的位姿做非线性精调优化
   * @return true优化收敛成功
   */
  bool Refine();

  // Functions from de original MLPnP code
  /*
   * Computes the camera pose given 3D points coordinates (in the camera
   * reference system), the camera rays and (optionally) the covariance matrix
   * of those camera rays. Result is stored in solution
   */
  /**
   * @brief MLPnP核心求解函数，根据3D点、观测射线、协方差求解相机位姿
   * @param f 归一化观测射线向量集合
   * @param p 世界坐标系三维点集合
   * @param covMats 观测噪声协方差矩阵数组
   * @param indices 当前采样子集点索引
   * @param result 输出3×4位姿矩阵[R|t]
   */
  void computePose(const bearingVectors_t& f, const points_t& p,
                   const cov3_mats_t& covMats, const std::vector<int>& indices,
                   transformation_t& result);

  /**
   * @brief MLPnP高斯牛顿迭代求解函数
   * @param x 待优化变量，Rodrigues旋转+平移共6维向量
   * @param pts 输入3D世界点
   * @param nullspaces 零空间矩阵数组
   * @param Kll 稀疏信息矩阵
   * @param use_cov 是否启用协方差加权
   */
  void mlpnp_gn(Eigen::VectorXd& x, const points_t& pts,
                const std::vector<Eigen::MatrixXd>& nullspaces,
                const Eigen::SparseMatrix<double> Kll, bool use_cov);

  /**
   * @brief 计算MLPnP残差与雅可比矩阵
   * @param x 当前优化变量(旋转+平移)
   * @param pts 3D世界点
   * @param nullspaces 零空间矩阵
   * @param r 输出残差向量
   * @param fjac 输出雅可比矩阵
   * @param getJacs true计算雅可比；false只算残差
   */
  void mlpnp_residuals_and_jacs(const Eigen::VectorXd& x, const points_t& pts,
                                const std::vector<Eigen::MatrixXd>& nullspaces,
                                Eigen::VectorXd& r, Eigen::MatrixXd& fjac,
                                bool getJacs);

  /**
   * @brief 计算单个点对应的雅可比矩阵
   * @param pt 输入3D世界点
   * @param nullspace_r 旋转零空间
   * @param nullspace_s 平移零空间
   * @param w Rodrigues旋转向量
   * @param t 平移向量
   * @param jacs 输出雅可比矩阵
   */
  void mlpnpJacs(const point_t& pt, const Eigen::Vector3d& nullspace_r,
                 const Eigen::Vector3d& nullspace_s, const rodrigues_t& w,
                 const translation_t& t, Eigen::MatrixXd& jacs);

  // Auxiliar methods
  /**
   * \brief Compute a rotation matrix from Rodrigues axis angle.
   *
   * \param[in] omega The Rodrigues‑parameters of a rotation.
   * \return The 3x3 rotation matrix.
   */
  /**
   * @brief Rodrigues轴角向量转为3×3旋转矩阵
   * @param omega Rodrigues轴角向量
   * @return 旋转矩阵R
   */
  Eigen::Matrix3d rodrigues2rot(const Eigen::Vector3d& omega);

  /**
   * \brief Compute the Rodrigues‑parameters of a rotation matrix.
   *
   * \param[in] R The 3x3 rotation matrix.
   * \return The Rodrigues‑parameters.
   */
  /**
   * @brief 旋转矩阵转换为Rodrigues轴角向量
   * @param R 3×3旋转矩阵
   * @return Rodrigues轴角向量
   */
  Eigen::Vector3d rot2rodrigues(const Eigen::Matrix3d& R);

  //----------------------------------------------------
  // Fields of the solver
  //----------------------------------------------------
  vector<MapPoint*> mvpMapPointMatches; ///< 输入2D‑3D匹配对应的地图点数组

  // 2D Points
  vector<cv::Point2f> mvP2D; ///< 图像上2D特征像素坐标
  // Substitued by bearing vectors
  bearingVectors_t mvBearingVecs; ///< 归一化相机观测射线向量（替代原始2D点）

  vector<float> mvSigma2; ///< 各特征点噪声方差，来自图像金字塔层级

  // 3D Points
  // vector<cv::Point3f> mvP3Dw;
  points_t mvP3Dw; ///< 地图点在世界坐标系下三维坐标

  // Index in Frame
  vector<size_t> mvKeyPointIndices; ///< 匹配点在Frame特征点数组内的索引

  // Current Estimation
  double mRi[3][3]; ///< 当前RANSAC样本解：旋转矩阵R
  double mti[3];    ///< 当前RANSAC样本解：平移向量t
  Eigen::Matrix4f mTcwi; ///< 当前迭代得到位姿Tcw 4×4矩阵
  vector<bool> mvbInliersi; ///< 当前迭代解的内点标记数组
  int mnInliersi; ///< 当前迭代解的内点数量

  // Current Ransac State
  int mnIterations; ///< 已经执行RANSAC迭代次数
  vector<bool> mvbBestInliers; ///< 历史最优解对应的内点标记数组
  int mnBestInliers; ///< 历史最优解内点数目
  Eigen::Matrix4f mBestTcw; ///< RANSAC得到最优位姿Tcw

  // Refined
  Eigen::Matrix4f mRefinedTcw; ///< 经过高斯牛顿精调后的位姿
  vector<bool> mvbRefinedInliers; ///< 精调后内点标记
  int mnRefinedInliers; ///< 精调之后内点数量

  // Number of Correspondences
  int N; ///< 2D‑3D匹配对总数量

  // Indices for random selection [0 .. N‑1]
  vector<size_t> mvAllIndices; ///< 全部匹配对索引，用于RANSAC随机采样

  // RANSAC probability
  double mRansacProb; ///< RANSAC期望成功概率

  // RANSAC min inliers
  int mRansacMinInliers; ///< RANSAC要求最少内点

  // RANSAC max iterations
  int mRansacMaxIts; ///< RANSAC最大迭代次数上限

  // RANSAC expected inliers/total ratio
  float mRansacEpsilon; ///< RANSAC预估内点比例epsilon

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh; ///< RANSAC重投影误差平方阈值

  // RANSAC Minimun Set used at each iteration
  int mRansacMinSet; ///< RANSAC每次随机采样最小样本数

  // Max square error associated with scale level. Max error =
  // th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError; ///< 每个金字塔层级对应的最大允许重投影误差平方

  std::shared_ptr<GeometricCamera> mpCamera; ///< 相机模型对象，获取内参、计算归一化射线
};

}  // namespace ORB_SLAM3
