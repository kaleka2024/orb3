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

#pragma once

#include <cmath>
#include <memory>
#include <set>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

namespace ORB_SLAM3 {

class LoopClosing;

class Optimizer {
 public:
  static void BundleAdjustment(
      const std::vector<std::shared_ptr<KeyFrame>> &vpKF,
      const std::vector<MapPoint *> &vpMP, int nIterations = 5,
      bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
      const bool bRobust = true);
  static void GlobalBundleAdjustemnt(const std::shared_ptr<Map> &pMap,
                                     int nIterations = 5,
                                     bool *pbStopFlag = NULL,
                                     const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);
  static void FullInertialBA(const std::shared_ptr<Map> &pMap, int its,
                             const bool bFixLocal = false,
                             const unsigned long nLoopKF = 0,
                             bool *pbStopFlag = NULL, bool bInit = false,
                             float priorG = 1e2, float priorA = 1e6,
                             Eigen::VectorXd *vSingVal = NULL,
                             bool *bHess = NULL);

  static void LocalBundleAdjustment(const std::shared_ptr<KeyFrame> &pKF,
                                    bool *pbStopFlag,
                                    const std::shared_ptr<Map> &pMap,
                                    int &num_fixedKF, int &num_OptKF,
                                    int &num_MPs, int &num_edges);

  static int PoseOptimization(const std::shared_ptr<Frame> &pFrame);
  static int PoseInertialOptimizationLastKeyFrame(
      const std::shared_ptr<Frame> &pFrame, bool bRecInit = false);
  static int PoseInertialOptimizationLastFrame(
      const std::shared_ptr<Frame> &pFrame, bool bRecInit = false);

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
  // (mono)
  static void OptimizeEssentialGraph(
      const std::shared_ptr<Map> &pMap,
      const std::shared_ptr<KeyFrame> &pLoopKF,
      const std::shared_ptr<KeyFrame> &pCurKF,
      const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose &CorrectedSim3,
      const map<std::shared_ptr<KeyFrame>, set<std::shared_ptr<KeyFrame>>>
          &LoopConnections,
      const bool &bFixScale);

  static void OptimizeEssentialGraph(
      const std::shared_ptr<KeyFrame> &pCurKF,
      vector<std::shared_ptr<KeyFrame>> &vpFixedKFs,
      vector<std::shared_ptr<KeyFrame>> &vpFixedCorrectedKFs,
      vector<std::shared_ptr<KeyFrame>> &vpNonFixedKFs,
      vector<MapPoint *> &vpNonCorrectedMPs);

  // For inertial loopclosing
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
  static int OptimizeSim3(const std::shared_ptr<KeyFrame> &pKF1,
                          const std::shared_ptr<KeyFrame> &pKF2,
                          std::vector<MapPoint *> &vpMatches1,
                          g2o::Sim3 &g2oS12, const float th2,
                          const bool bFixScale,
                          Eigen::Matrix<double, 7, 7> &mAcumHessian,
                          const bool bAllPoints = false);

  // For inertial systems

  static void LocalInertialBA(const std::shared_ptr<KeyFrame> &pKF,
                              bool *pbStopFlag,
                              const std::shared_ptr<Map> &pMap,
                              int &num_fixedKF, int &num_OptKF, int &num_MPs,
                              int &num_edges, bool bLarge = false,
                              bool bRecInit = false);

  static void MergeInertialBA(const std::shared_ptr<KeyFrame> &pCurrKF,
                              const std::shared_ptr<KeyFrame> &pMergeKF,
                              bool *pbStopFlag,
                              const std::shared_ptr<Map> &pMap,
                              LoopClosing::KeyFrameAndPose &corrPoses);

  // Local BA in welding area when two maps are merged
  static void LocalBundleAdjustment(const std::shared_ptr<KeyFrame> &pMainKF,
                                    vector<shared_ptr<KeyFrame>> vpAdjustKF,
                                    vector<shared_ptr<KeyFrame>> vpFixedKF,
                                    bool *pbStopFlag);

  // Marginalize block element (start:end,start:end). Perform Schur complement.
  // Marginalized elements are filled with zeros.
  static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start,
                                     const int &end);

  // Inertial pose-graph
  static void InertialOptimization(const std::shared_ptr<Map> &pMap,
                                   Eigen::Matrix3d &Rwg, double &scale,
                                   Eigen::Vector3d &bg, Eigen::Vector3d &ba,
                                   bool bMono, Eigen::MatrixXd &covInertial,
                                   bool bFixedVel = false, bool bGauss = false,
                                   float priorG = 1e2, float priorA = 1e6);
  static void InertialOptimization(const std::shared_ptr<Map> &pMap,
                                   Eigen::Vector3d &bg, Eigen::Vector3d &ba,
                                   float priorG = 1e2, float priorA = 1e6);
  static void InertialOptimization(const std::shared_ptr<Map> &pMap,
                                   Eigen::Matrix3d &Rwg, double &scale);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ORB_SLAM3
