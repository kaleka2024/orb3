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
#pragma once                          // 头文件保护，防止重复包含引发编译重定义
#include <cmath>                      // 标准数学库
#include <memory>                     // 智能指针 std::shared_ptr
#include <set>                        // std::set有序集合容器
#include <vector>                     // std::vector动态数组容器

#include "Frame.h"                    // 普通帧类定义
#include "KeyFrame.h"                 // 关键帧类定义
#include "LoopClosing.h"              // 回环检测模块类
#include "Map.h"                      // 地图类，管理关键帧与地图点
#include "MapPoint.h"                 // 地图点类

#include "g2o/core/block_solver.h"                     // g2o块求解器，BA稀疏求解核心
#include "g2o/core/optimization_algorithm_gauss_newton.h" // 高斯牛顿优化算法
#include "g2o/core/optimization_algorithm_levenberg.h"    // LM列文伯格‑马夸尔特优化算法
#include "g2o/core/robust_kernel_impl.h"               // g2o鲁棒核函数(Huber等)，抑制外点
#include "g2o/core/sparse_block_matrix.h"              // 稀疏块矩阵，用于Hessian矩阵存储
#include "g2o/solvers/dense/linear_solver_dense.h"     // 稠密线性求解器
#include "g2o/solvers/eigen/linear_solver_eigen.h"     // 基于Eigen的线性求解器
#include "g2o/types/sba/types_six_dof_expmap.h"        // SE3 6自由度李代数exp‑map顶点与边，用于光束平差
#include "g2o/types/sim3/types_seven_dof_expmap.h"     // Sim3 7自由度顶点边，回环相似变换优化

namespace ORB_SLAM3 {

class LoopClosing; // 前向声明回环类，避免循环头文件依赖

/**
 * @brief Optimizer优化器类，全部为静态函数，封装SLAM中各类g2o优化任务
 * @details 包含光束平差BA、位姿优化、EssentialGraph图优化、IMU惯性优化、Sim3求解、舒尔补边缘化等
 */
class Optimizer {
 public:
  /**
   * @brief 光束平差BA，对指定一批关键帧与地图点做联合优化
   * @param vpKF 待优化关键帧集合
   * @param vpMP 待优化地图点集合
   * @param nIterations g2o最大迭代次数
   * @param pbStopFlag 外部终止标志，可外部置true强制停止优化
   * @param nLoopKF 触发本次BA的回环关键帧ID，0代表非回环触发
   * @param bRobust 是否开启鲁棒核函数，抑制重投影外点
   */
  static void BundleAdjustment(
      const std::vector<std::shared_ptr<KeyFrame>> &vpKF,
      const std::vector<MapPoint *> &vpMP, int nIterations = 5,
      bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
      const bool bRobust = true);

  /**
   * @brief 全局光束平差，优化地图内全部关键帧与全部地图点
   * @param pMap 系统全局地图
   * @param nIterations 优化迭代次数
   * @param pbStopFlag 外部终止标志位
   * @param nLoopKF 回环触发关键帧ID，0表示普通全局BA
   * @param bRobust 是否启用鲁棒核
   */
  static void GlobalBundleAdjustemnt(const std::shared_ptr<Map> &pMap,
                                     int nIterations = 5,
                                     bool *pbStopFlag = NULL,
                                     const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);

  /**
   * @brief 完整惯性BA，IMU模式下，联合优化关键帧位姿、速度、IMU偏置、地图点
   * @param pMap 全局地图
   * @param its 最大迭代次数
   * @param bFixLocal 是否固定局部部分变量
   * @param nLoopKF 回环关键帧ID
   * @param pbStopFlag 外部终止标志
   * @param bInit 是否为初始化阶段调用
   * @param priorG gyro陀螺先验权重
   * @param priorA accel加速度计先验权重
   * @param vSingVal 输出奇异值，用于评估Hessian矩阵
   * @param bHess 输出标志，指示Hessian是否奇异
   */
  static void FullInertialBA(const std::shared_ptr<Map> &pMap, int its,
                             const bool bFixLocal = false,
                             const unsigned long nLoopKF = 0,
                             bool *pbStopFlag = NULL, bool bInit = false,
                             float priorG = 1e2, float priorA = 1e6,
                             Eigen::VectorXd *vSingVal = NULL,
                             bool *bHess = NULL);

  /**
   * @brief 局部光束平差，LocalMapping线程核心，优化当前关键帧邻域关键帧与关联地图点
   * @param pKF 当前处理的关键帧
   * @param pbStopFlag 外部终止标志
   * @param pMap 全局地图
   * @param num_fixedKF 输出：被固定不参与优化的关键帧数量
   * @param num_OptKF 输出：参与优化的关键帧数量
   * @param num_MPs 输出：参与优化的地图点数量
   * @param num_edges 输出：优化边(重投影约束)总数量
   */
  static void LocalBundleAdjustment(const std::shared_ptr<KeyFrame> &pKF,
                                    bool *pbStopFlag,
                                    const std::shared_ptr<Map> &pMap,
                                    int &num_fixedKF, int &num_OptKF,
                                    int &num_MPs, int &num_edges);

  /**
   * @brief 普通帧位姿优化，只优化Frame相机位姿，地图点固定，Tracking中用于跟踪
   * @param pFrame 当前普通帧
   * @return 返回有效内点数量
   */
  static int PoseOptimization(const std::shared_ptr<Frame> &pFrame);

  /**
   * @brief 惯性模式位姿优化，参考上一关键帧，联合视觉重投影+IMU预积分约束
   * @param pFrame 当前普通帧
   * @param bRecInit 是否允许重新初始化IMU状态
   * @return 返回内点数目
   */
  static int PoseInertialOptimizationLastKeyFrame(
      const std::shared_ptr<Frame> &pFrame, bool bRecInit = false);

  /**
   * @brief 惯性模式位姿优化，参考上一普通帧，视觉+IMU约束
   * @param pFrame 当前普通帧
   * @param bRecInit 是否允许IMU状态重初始化
   * @return 返回内点数目
   */
  static int PoseInertialOptimizationLastFrame(
      const std::shared_ptr<Frame> &pFrame, bool bRecInit = false);

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
  // (mono)
  /**
   * @brief 本质图优化，回环校正后执行，只优化位姿图，不优化地图点；双目/RGBD固定尺度做SE3，单目Sim3
   * @param pMap 全局地图
   * @param pLoopKF 回环匹配关键帧
   * @param pCurKF 当前闭环关键帧
   * @param NonCorrectedSim3 闭环未校正的Sim3位姿
   * @param CorrectedSim3 闭环校正后的Sim3位姿
   * @param LoopConnections 闭环新增的关键帧连接关系
   * @param bFixScale true固定尺度(双目/RGBD)做SE3；false单目优化Sim3带尺度
   */
  static void OptimizeEssentialGraph(
      const std::shared_ptr<Map> &pMap,
      const std::shared_ptr<KeyFrame> &pLoopKF,
      const std::shared_ptr<KeyFrame> &pCurKF,
      const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose &CorrectedSim3,
      const map<std::shared_ptr<KeyFrame>, set<std::shared_ptr<KeyFrame>>>
          &LoopConnections,
      const bool &bFixScale);

  /**
   * @brief 重载本质图优化，用于地图融合场景，区分固定帧、待优化帧
   * @param pCurKF 当前关键帧
   * @param vpFixedKFs 固定不动的关键帧
   * @param vpFixedCorrectedKFs 已经校正完毕的固定关键帧
   * @param vpNonFixedKFs 需要参与优化的关键帧
   * @param vpNonCorrectedMPs 未校正地图点集合
   */
  static void OptimizeEssentialGraph(
      const std::shared_ptr<KeyFrame> &pCurKF,
      vector<std::shared_ptr<KeyFrame>> &vpFixedKFs,
      vector<std::shared_ptr<KeyFrame>> &vpFixedCorrectedKFs,
      vector<std::shared_ptr<KeyFrame>> &vpNonFixedKFs,
      vector<MapPoint *> &vpNonCorrectedMPs);

  // For inertial loopclosing
  /**
   * @brief IMU回环专用本质图优化，4DoF优化（yaw航向自由，roll/pitch由IMU重力约束固定）
   * @param pMap 全局地图
   * @param pLoopKF 回环匹配帧
   * @param pCurKF 当前闭环帧
   * @param NonCorrectedSim3 未校正Sim3
   * @param CorrectedSim3 校正后Sim3
   * @param LoopConnections 闭环连接边
   */
  static void OptimizeEssentialGraph4DoF(
      const std::shared_ptr<Map> &pMap,
      const std::shared_ptr<KeyFrame> &pLoopKF,
      const std::shared_ptr<KeyFrame> &pCurKF,
      const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose &CorrectedSim3,
      const map<std::shared_ptr<KeyFrame>, set<std::shared_ptr<KeyFrame>>>
          &LoopConnections);

  // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
  // (NEW)
  /**
   * @brief 优化两关键帧之间Sim3变换，回环检测中计算两帧相似变换
   * @param pKF1 关键帧1
   * @param pKF2 关键帧2
   * @param vpMatches1 两帧匹配的地图点
   * @param g2oS12 输入初始Sim3，输出优化后的Sim3
   * @param th2 重投影误差平方阈值
   * @param bFixScale true固定尺度(双目RGBD)优化SE3；false单目优化7维Sim3含尺度
   * @param mAcumHessian 输出累加Hessian矩阵
   * @param bAllPoints false只用内点；true使用全部匹配点
   * @return 返回内点数量
   */
  static int OptimizeSim3(const std::shared_ptr<KeyFrame> &pKF1,
                          const std::shared_ptr<KeyFrame> &pKF2,
                          std::vector<MapPoint *> &vpMatches1,
                          g2o::Sim3 &g2oS12, const float th2,
                          const bool bFixScale,
                          Eigen::Matrix<double, 7, 7> &mAcumHessian,
                          const bool bAllPoints = false);

  // For inertial systems
  /**
   * @brief 局部惯性BA，IMU模式局部建图，联合视觉重投影+IMU预积分约束
   * @param pKF 当前关键帧
   * @param pbStopFlag 外部停止标志
   * @param pMap 全局地图
   * @param num_fixedKF 输出固定关键帧数目
   * @param num_OptKF 输出优化关键帧数目
   * @param num_MPs 输出优化地图点数目
   * @param num_edges 输出约束边总数
   * @param bLarge 是否大场景模式
   * @param bRecInit 是否允许IMU状态重初始化
   */
  static void LocalInertialBA(const std::shared_ptr<KeyFrame> &pKF,
                              bool *pbStopFlag,
                              const std::shared_ptr<Map> &pMap,
                              int &num_fixedKF, int &num_OptKF, int &num_MPs,
                              int &num_edges, bool bLarge = false,
                              bool bRecInit = false);

  /**
   * @brief 地图融合时的惯性BA，两张地图合并，校正位姿、IMU状态
   * @param pCurrKF 当前地图关键帧
   * @param pMergeKF 待合并地图关键帧
   * @param pbStopFlag 外部终止标志
   * @param pMap 合并之后的地图
   * @param corrPoses 输出校正后的位姿集合
   */
  static void MergeInertialBA(const std::shared_ptr<KeyFrame> &pCurrKF,
                              const std::shared_ptr<KeyFrame> &pMergeKF,
                              bool *pbStopFlag,
                              const std::shared_ptr<Map> &pMap,
                              LoopClosing::KeyFrameAndPose &corrPoses);

  // Local BA in welding area when two maps are merged
  /**
   * @brief 地图融合拼接区域局部BA，只优化拼接附近关键帧，其余帧固定
   * @param pMainKF 主地图基准关键帧
   * @param vpAdjustKF 需要参与优化的关键帧
   * @param vpFixedKF 固定不动的关键帧
   * @param pbStopFlag 外部终止标志
   */
  static void LocalBundleAdjustment(const std::shared_ptr<KeyFrame> &pMainKF,
                                    vector<shared_ptr<KeyFrame>> vpAdjustKF,
                                    vector<shared_ptr<KeyFrame>> vpFixedKF,
                                    bool *pbStopFlag);

  // Marginalize block element (start:end,start:end). Perform Schur complement.
  // Marginalized elements are filled with zeros.
  /**
   * @brief 矩阵边缘化，执行舒尔补Schur complement，把指定块变量消元，被消元区域置零
   * @param H 输入Hessian矩阵
   * @param start 起始行/列索引
   * @param end 结束行/列索引
   * @return 边缘化之后的矩阵
   */
  static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start,
                                     const int &end);

  // Inertial pose‑graph
  /**
   * @brief IMU位姿图优化，只优化位姿、速度、IMU偏置，不优化地图点；输出重力旋转、尺度、bg、ba、协方差
   * @param pMap 全局地图
   * @param Rwg 输出世界到IMU重力旋转矩阵
   * @param scale 输出单目尺度因子
   * @param bg 输出陀螺偏置
   * @param ba 输出加速度计偏置
   * @param bMono 是否单目模式
   * @param covInertial 输出惯性状态协方差矩阵
   * @param bFixedVel 是否固定速度变量
   * @param bGauss 是否高斯迭代模式
   * @param priorG 陀螺先验权重
   * @param priorA 加速度计先验权重
   */
  static void InertialOptimization(const std::shared_ptr<Map> &pMap,
                                   Eigen::Matrix3d &Rwg, double &scale,
                                   Eigen::Vector3d &bg, Eigen::Vector3d &ba,
                                   bool bMono, Eigen::MatrixXd &covInertial,
                                   bool bFixedVel = false, bool bGauss = false,
                                   float priorG = 1e2, float priorA = 1e6);

  /**
   * @brief IMU位姿图优化重载版本，只输出陀螺、加速度计偏置
   * @param pMap 全局地图
   * @param bg 输出陀螺偏置
   * @param ba 输出加速度计偏置
   * @param priorG 陀螺先验权重
   * @param priorA 加速度计先验权重
   */
  static void InertialOptimization(const std::shared_ptr<Map> &pMap,
                                   Eigen::Vector3d &bg, Eigen::Vector3d &ba,
                                   float priorG = 1e2, float priorA = 1e6);

  /**
   * @brief IMU位姿图优化重载版本，输出重力旋转与尺度
   * @param pMap 全局地图
   * @param Rwg 输出重力对齐旋转矩阵
   * @param scale 输出尺度因子
   */
  static void InertialOptimization(const std::shared_ptr<Map> &pMap,
                                   Eigen::Matrix3d &Rwg, double &scale);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类中存在Eigen对齐类型成员
};

}  // namespace ORB_SLAM3
