/**
 * ============================================================================
 *  Optimizer.cpp
 *  ---------------------------------------------------------------------------
 *  ORB-SLAM3 后端优化器实现
 *
 *  本文件是 ORB-SLAM3 的"后端大脑"，所有基于 g2o 的图优化都在这里完成。
 *
 *  主要功能模块：
 *    1. 全局 BA               GlobalBundleAdjustemnt / BundleAdjustment
 *    2. 局部 BA               LocalBundleAdjustment（两种重载）
 *    3. 惯性 BA               FullInertialBA / LocalInertialBA / MergeInertialBA
 *    4. 位姿优化              PoseOptimization
 *    5. 惯性位姿优化          PoseInertialOptimizationLastKeyFrame / LastFrame
 *    6. 本质图优化            OptimizeEssentialGraph（两种重载）
 *    7. Sim3 优化             OptimizeSim3
 *    8. 惯性初始化优化        InertialOptimization（三种重载）
 *    9. 4DoF 本质图优化       OptimizeEssentialGraph4DoF
 *   10. 边缘化工具            Marginalize
 *
 *  典型的优化问题结构（以 BA 为例）：
 *    - 顶点（Vertex）：待优化变量
 *        * 关键帧位姿  VertexSE3Expmap / VertexPose
 *        * 地图点      VertexPointXYZ
 *        * 速度        VertexVelocity
 *        * 陀螺零偏    VertexGyroBias
 *        * 加速度零偏  VertexAccBias
 *        * 重力方向    VertexGDir
 *        * 尺度        VertexScale
 *    - 边（Edge）：观测约束
 *        * 重投影误差  EdgeSE3ProjectXYZ / EdgeStereoSE3ProjectXYZ
 *        * 惯性残差    EdgeInertial / EdgeInertialGS
 *        * 随机游走    EdgeGyroRW / EdgeAccRW
 *        * 先验        EdgePriorAcc / EdgePriorGyro / EdgePriorPoseImu
 *        * Sim3 边     EdgeSim3 / Edge4DoF
 *    - 鲁棒核：Huber，抑制外点
 *    - 求解器：Levenberg-Marquardt / Gauss-Newton
 *
 *  文件采用 ORB_SLAM3 命名空间，所有对外函数均在 Optimizer.h 中声明。
 * ============================================================================
 */

#include "Optimizer.h"

// ---- Eigen 线性代数库 ----
#include <Eigen/Dense>          // 稠密矩阵/向量
#include <Eigen/StdVector>      // 支持 std::vector<Eigen::...> 对齐
#include <algorithm>            // std::sort / std::min / std::max
#include <complex>              // 复数（部分模板实例化需要）
#include <iostream>             // std::cout / std::cerr
#include <list>                 // std::list
#include <memory>               // std::shared_ptr / std::unique_ptr
#include <mutex>                // std::mutex / std::unique_lock
#include <set>                  // std::set
#include <tuple>                // std::tuple / std::get
#include <unordered_set>        // std::unordered_set
#include <unsupported/Eigen/MatrixFunctions>  // 矩阵函数（如 log/exp）
#include <utility>              // std::pair / std::make_pair
#include <vector>               // std::vector

// ---- ORB-SLAM3 内部依赖 ----
#include "Converter.h"          // 类型转换工具
#include "G2oTypes.h"           // 自定义 g2o 类型（VertexPose 等）
#include "OptimizableTypes.h"   // 自定义可优化边（EdgeMono/EdgeStereo 等）

// ---- g2o 图优化库 ----
#include "g2o/core/block_solver.h"                        // 块求解器
#include "g2o/core/optimization_algorithm_gauss_newton.h" // GN 算法
#include "g2o/core/optimization_algorithm_levenberg.h"    // LM 算法
#include "g2o/core/robust_kernel_impl.h"                  // Huber 核
#include "g2o/core/sparse_block_matrix.h"                 // 稀疏块矩阵
#include "g2o/solvers/dense/linear_solver_dense.h"        // 稠密线性求解
#include "g2o/solvers/eigen/linear_solver_eigen.h"        // Eigen 稀疏求解
#include "g2o/types/sba/types_six_dof_expmap.h"           // SE3 顶点/边
#include "g2o/types/sba/vertex_se3_expmap.h"              // SE3 顶点

namespace ORB_SLAM3 {

/**
 * 辅助排序函数：按 pair 的 second 升序排序
 * 用于 MergeInertialBA 中对地图点观测次数排序
 */
bool sortByVal(const pair<MapPoint*, int>& a, const pair<MapPoint*, int>& b) {
  return (a.second < b.second);
}

// ============================================================================
//  一、全局 BA
// ============================================================================

/**
 * 全局 BA 入口：从地图中取出所有关键帧和地图点，调用 BundleAdjustment
 *
 * @param pMap        地图指针
 * @param nIterations 优化迭代次数
 * @param pbStopFlag  外部停止标志（可为空）
 * @param nLoopKF     回环关键帧 id（0 表示普通全局 BA）
 * @param bRobust     是否使用鲁棒核
 */
void Optimizer::GlobalBundleAdjustemnt(const std::shared_ptr<Map>& pMap,
                                       int nIterations, bool* pbStopFlag,
                                       const unsigned long nLoopKF,
                                       const bool bRobust) {
  // 取出地图中所有活跃关键帧
  auto vpKFs = pMap->GetAllKeyFrames();
  // 取出地图中所有活跃地图点
  auto vpMP = pMap->GetAllMapPoints();
  // 调用核心 BA
  BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

/**
 * 全局 BA 核心实现
 *
 * 优化变量：
 *   - 关键帧位姿（VertexSE3Expmap），初始关键帧固定
 *   - 地图点位置（VertexPointXYZ），边缘化掉
 *
 * 观测边：
 *   - 单目：EdgeSE3ProjectXYZ（2 维重投影误差）
 *   - 双目：EdgeStereoSE3ProjectXYZ（3 维重投影误差）
 *   - 鱼眼右目：EdgeSE3ProjectXYZToBody（2 维）
 *
 * 使用 Levenberg-Marquardt + Eigen 稀疏求解 + Huber 鲁棒核
 */
void Optimizer::BundleAdjustment(const vector<std::shared_ptr<KeyFrame>>& vpKFs,
                                 const vector<MapPoint*>& vpMP, int nIterations,
                                 bool* pbStopFlag, const unsigned long nLoopKF,
                                 const bool bRobust) {
  // 记录未加入优化的地图点（观测边数为 0）
  std::unordered_set<MapPoint*> vbNotIncludedMP;

  // 取地图指针（从第一个关键帧获得）
  auto pMap = vpKFs[0]->GetMap();

  // ---- 1. 构建 g2o 优化器 ----
  g2o::SparseOptimizer optimizer;

  // 线性求解器：Eigen 稀疏求解，块大小 6x3（位姿 6 维 / 点 3 维）
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  // 块求解器 + LM 算法
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);   // 设置求解算法
  optimizer.setVerbose(false);      // 关闭 g2o 内部打印

  // 若外部提供停止标志，交给 g2o
  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;    // 记录最大关键帧 id

  // 预估边数 = 关键帧数 × 地图点数
  const int nExpectedSize = (vpKFs.size()) * vpMP.size();

  // ---- 2. 预分配各类边的容器 ----
  vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;   // 单目边
  vpEdgesMono.reserve(nExpectedSize);

  vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody; // 鱼眼右目边
  vpEdgesBody.reserve(nExpectedSize);

  vector<std::shared_ptr<KeyFrame>> vpEdgeKFMono;      // 单目边对应的 KF
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<std::shared_ptr<KeyFrame>> vpEdgeKFBody;      // 鱼眼边对应的 KF
  vpEdgeKFBody.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;                // 单目边对应的 MP
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeBody;                // 鱼眼边对应的 MP
  vpMapPointEdgeBody.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo; // 双目边
  vpEdgesStereo.reserve(nExpectedSize);

  vector<std::shared_ptr<KeyFrame>> vpEdgeKFStereo;    // 双目边对应的 KF
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;              // 双目边对应的 MP
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  // ---- 3. 添加关键帧位姿顶点 ----
  for (auto const& pKF : vpKFs) {
    if (pKF->isBad()) continue;                        // 跳过坏帧

    // 顶点类型：SE3 位姿（旋转 + 平移）
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    // 从关键帧获取 Tcw（世界→相机）
    Sophus::SE3<float> Tcw = pKF->GetPose();
    // 设置初值：旋转用四元数，平移用向量，均转 double
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKF->mnId);                            // 顶点 id = KF id
    // 第一个关键帧固定（规范自由度）
    vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);                         // 加入优化器
    if (pKF->mnId > maxKFid) maxKFid = pKF->mnId;      // 更新最大 id
  }

  // 鲁棒核阈值：卡方分布 95% 分位数开根号
  const float thHuber2D = sqrt(5.99);    // 2 自由度
  const float thHuber3D = sqrt(7.815);   // 3 自由度

  // ---- 4. 添加地图点顶点及其观测边 ----
  for (auto pMP : vpMP) {
    if (pMP->isBad()) continue;                        // 跳过坏点

    // 顶点类型：3D 点
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>()); // 世界坐标初值
    // 顶点 id：地图点 id + maxKFid + 1（避免与 KF id 冲突）
    const int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);                     // 边缘化（Schur 消元）
    optimizer.addVertex(vPoint);

    // 取出该地图点的所有观测：KF → (左目索引, 右目索引)
    const map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations =
        pMP->GetObservations();

    int nEdges = 0;                                    // 该点的有效观测数

    // ---- 遍历观测，为每个观测添加一条重投影边 ----
    for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::const_iterator mit =
             observations.begin();
         mit != observations.end(); mit++) {
      std::shared_ptr<KeyFrame> pKF = mit->first;      // 观测该点的关键帧
      if (pKF->isBad() || pKF->mnId > maxKFid) continue; // 跳过坏帧
      if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
        continue;                                      // 顶点不在图中
      nEdges++;

      const int leftIndex = get<0>(mit->second);       // 左目特征索引

      // ---- 4.1 单目观测：右目无匹配 ----
      if (leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)] < 0) {
        const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex]; // 去畸变关键点

        Eigen::Matrix<double, 2, 1> obs;               // 2D 观测
        obs << kpUn.pt.x, kpUn.pt.y;

        // 单目重投影边
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

        // 顶点 0：地图点；顶点 1：关键帧位姿
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);                        // 观测值

        // 信息矩阵 = I × invSigma2（按金字塔层级方差）
        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

        // 鲁棒核
        if (bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);
        }

        e->pCamera = pKF->mpCamera;                    // 相机模型

        optimizer.addEdge(e);                          // 加入优化器

        vpEdgesMono.push_back(e);                      // 记录边
        vpEdgeKFMono.push_back(pKF);
        vpMapPointEdgeMono.push_back(pMP);
      }
      // ---- 4.2 双目观测：右目有匹配 ----
      else if (leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0) {
        const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

        Eigen::Matrix<double, 3, 1> obs;               // 3D 观测 (uL, vL, uR)
        const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        // 双目重投影边
        g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);

        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
        e->setInformation(Info);

        if (bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber3D);
        }

        // 双目相机内参
        e->fx = pKF->fx; e->fy = pKF->fy;
        e->cx = pKF->cx; e->cy = pKF->cy;
        e->bf = pKF->mbf;                              // 基线 × 焦距

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(pKF);
        vpMapPointEdgeStereo.push_back(pMP);
      }

      // ---- 4.3 鱼眼右目观测（相机 2）----
      if (pKF->mpCamera2) {
        int rightIndex = get<1>(mit->second);

        if (rightIndex != -1 &&
            rightIndex < static_cast<int>(pKF->mvKeysRight.size())) {
          rightIndex -= pKF->NLeft;                    // 索引偏移

          Eigen::Matrix<double, 2, 1> obs;
          cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
          obs << kp.pt.x, kp.pt.y;

          // 相对位姿约束的右目边
          ORB_SLAM3::EdgeSE3ProjectXYZToBody* e =
              new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs);

          const float& invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);

          // 左目→右目的相对位姿
          Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
          e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                 Trl.translation().cast<double>());

          e->pCamera = pKF->mpCamera2;

          optimizer.addEdge(e);
          vpEdgesBody.push_back(e);
          vpEdgeKFBody.push_back(pKF);
          vpMapPointEdgeBody.push_back(pMP);
        }
      }
    }

    // ---- 4.4 若该点无任何有效观测，移除顶点 ----
    if (nEdges == 0) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP.insert(pMP);                     // 记录未参与优化
    }
  }

  // ---- 5. 执行优化 ----
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();                  // 初始化（构建 Hessian）
  optimizer.optimize(nIterations);                     // 迭代 nIterations 次
  oslog::debug("BA: End of the optimization");         // 日志

  // ---- 6. 恢复优化后的位姿 ----
  for (auto pKF : vpKFs) {
    if (pKF->isBad()) continue;

    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));

    g2o::SE3Quat SE3quat = vSE3->estimate();           // 取出优化结果

    // 普通全局 BA：直接写回位姿
    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(),
                                SE3quat.translation().cast<float>()));
    } else {
      // 回环后全局 BA：先存到 mTcwGBA，等后续融合
      pKF->mTcwGBA =
          Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
      pKF->mnBAGlobalForKF = nLoopKF;

      // 计算位姿变化量，若超过 1m 则统计内外点
      Sophus::SE3f mTwc = pKF->GetPoseInverse();
      Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
      Eigen::Vector3f vector_dist = mTcGBA_c.translation();
      const double dist = vector_dist.norm();
      if (dist > 1) {
        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;

        // 统计单目内外点
        for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++) {
          ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i2];
          MapPoint* pMP = vpMapPointEdgeMono[i2];
          std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i2];

          if (pKF != pKFedge) continue;                // 只统计当前 KF
          if (pMP->isBad()) continue;

          // 卡方 > 5.991 或深度为负 → 外点
          if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            numMonoBadPoints++;
          } else {
            numMonoOptPoints++;
            vpMonoMPsOpt.push_back(pMP);
          }
        }

        // 统计双目内外点
        for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend; i2++) {
          g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i2];
          MapPoint* pMP = vpMapPointEdgeStereo[i2];
          std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i2];

          if (pKF != pKFedge) continue;
          if (pMP->isBad()) continue;

          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            numStereoBadPoints++;
          } else {
            numStereoOptPoints++;
            vpStereoMPsOpt.push_back(pMP);
          }
        }
      }
    }
  }

  // ---- 7. 恢复优化后的地图点 ----
  for (auto pMP : vpMP) {
    if (vbNotIncludedMP.find(pMP) != vbNotIncludedMP.end()) continue;
    if (pMP->isBad()) continue;

    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));

    // 普通全局 BA：直接写回
    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();                     // 更新法向和深度
    } else {
      // 回环后全局 BA：暂存
      pMP->mPosGBA = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopKF;
    }
  }
}

// ============================================================================
//  二、全惯性 BA
// ============================================================================

/**
 * 全惯性 BA：同时优化位姿、速度、零偏、地图点，使用 IMU 预积分约束
 *
 * 顶点：
 *   - 位姿 VertexPose、速度 VertexVelocity、陀螺零偏 VertexGyroBias、加速度零偏 VertexAccBias
 *   - 地图点 VertexPointXYZ
 * 边：
 *   - 惯性残差 EdgeInertial、陀螺/加速度随机游走 EdgeGyroRW/EdgeAccRW
 *   - 视觉重投影 EdgeMono/EdgeStereo
 *   - 先验 EdgePriorAcc/EdgePriorGyro
 *
 * @param bInit   是否为初始化阶段（所有 KF 共享一组零偏）
 * @param bFixLocal 是否固定局部窗口外的 KF
 */
void Optimizer::FullInertialBA(const std::shared_ptr<Map>& pMap, int its,
                               const bool bFixLocal,
                               const long unsigned int nLoopId,
                               bool* pbStopFlag, bool bInit, float priorG,
                               float priorA, Eigen::VectorXd* vSingVal,
                               bool* bHess) {
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  // ---- 1. 构建优化器（块求解器 X 表示任意块大小）----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  solver->setUserLambdaInit(1e-5);                     // 初始阻尼因子
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);
  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  int nNonFixed = 0;                                   // 非固定 KF 数

  // ---- 2. 添加关键帧顶点（位姿 / 速度 / 零偏）----
  shared_ptr<KeyFrame> pIncKF;
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);             // 位姿顶点
    VP->setId(pKFi->mnId);
    pIncKF = pKFi;
    bool bFixed = false;
    // 局部固定模式：窗口外的 KF 固定
    if (bFixLocal) {
      bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) ||
               (pKFi->mnBAFixedForKF >= (maxKFid - 1));
      if (!bFixed) nNonFixed++;
      VP->setFixed(bFixed);
    }
    optimizer.addVertex(VP);

    // 惯性 KF：额外添加速度、零偏顶点
    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);       // id 偏移避免冲突
      VV->setFixed(bFixed);
      optimizer.addVertex(VV);

      // 初始化阶段所有 KF 共享零偏，不单独添加
      if (!bInit) {
        VertexGyroBias* VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
        VG->setFixed(bFixed);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pKFi);
        VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
        VA->setFixed(bFixed);
        optimizer.addVertex(VA);
      }
    }
  }

  // ---- 3. 初始化阶段：添加共享零偏顶点 ----
  if (bInit) {
    VertexGyroBias* VG = new VertexGyroBias(pIncKF);
    VG->setId(4 * maxKFid + 2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pIncKF);
    VA->setId(4 * maxKFid + 3);
    VA->setFixed(false);
    optimizer.addVertex(VA);
  }

  if (bFixLocal) {
    if (nNonFixed < 3) return;                         // 非固定帧太少，放弃
  }

  // ---- 4. 添加惯性边 ----
  for (auto pKFi : vpKFs) {
    if (!pKFi->mPrevKF) {                              // 无前序 KF
      oslog::warn("NOT INERTIAL LINK TO PREVIOUS FRAME!");
      continue;
    }

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
      if (pKFi->bImu && pKFi->mPrevKF->bImu) {
        // 用前序 KF 的零偏作为预积分初值
        pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());

        // 取各顶点指针
        g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
        g2o::HyperGraph::Vertex* VV1 =
            optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);

        g2o::HyperGraph::Vertex* VG1;
        g2o::HyperGraph::Vertex* VA1;
        g2o::HyperGraph::Vertex* VG2;
        g2o::HyperGraph::Vertex* VA2;
        if (!bInit) {                                  // 非初始化：各 KF 独立零偏
          VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
          VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
          VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
          VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
        } else {                                       // 初始化：共享零偏
          VG1 = optimizer.vertex(4 * maxKFid + 2);
          VA1 = optimizer.vertex(4 * maxKFid + 3);
        }

        g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
        g2o::HyperGraph::Vertex* VV2 =
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);

        // 检查顶点是否齐全
        if (!bInit) {
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
                 << endl;
            continue;
          }
        } else {
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2) {
            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                 << ", " << VP2 << ", " << VV2 << endl;
            continue;
          }
        }

        // 惯性残差边
        EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
        ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
        ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
        ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
        ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
        ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
        ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

        g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
        ei->setRobustKernel(rki);
        rki->setDelta(sqrt(16.92));                    // 鲁棒核阈值

        optimizer.addEdge(ei);

        // 非初始化阶段：添加零偏随机游走边
        if (!bInit) {
          // 陀螺零偏随机游走
          EdgeGyroRW* egr = new EdgeGyroRW();
          egr->setVertex(0, VG1);
          egr->setVertex(1, VG2);
          Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                      .cast<double>()
                                      .inverse();
          egr->setInformation(InfoG);
          egr->computeError();
          optimizer.addEdge(egr);

          // 加速度零偏随机游走
          EdgeAccRW* ear = new EdgeAccRW();
          ear->setVertex(0, VA1);
          ear->setVertex(1, VA2);
          Eigen::Matrix3d InfoA =
              pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                  .cast<double>()
                  .inverse();
          ear->setInformation(InfoA);
          ear->computeError();
          optimizer.addEdge(ear);
        }
      } else {
        cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu"
             << endl;
      }
    }
  }

  // ---- 5. 初始化阶段：添加零偏先验 ----
  if (bInit) {
    g2o::HyperGraph::Vertex* VG = optimizer.vertex(4 * maxKFid + 2);
    g2o::HyperGraph::Vertex* VA = optimizer.vertex(4 * maxKFid + 3);

    Eigen::Vector3f bprior;
    bprior.setZero();                                  // 零偏先验为 0

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    epa->setInformation(priorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);

    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    epg->setInformation(priorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);
  }

  // 视觉鲁棒核阈值
  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  const unsigned long iniMPid = maxKFid * 5;           // 地图点 id 偏移

  std::unordered_set<MapPoint*> vbNotIncludedMP;

  // ---- 6. 添加地图点顶点及视觉边 ----
  for (auto pMP : vpMPs) {
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<shared_ptr<KeyFrame>, tuple<int, int>> observations =
        pMP->GetObservations();

    bool bAllFixed = true;                             // 是否所有观测 KF 都固定

    // ---- 6.1 遍历观测，添加视觉边 ----
    for (auto const& [pKFi, vObs] : observations) {
      if (pKFi->mnId > maxKFid) continue;

      if (!pKFi->isBad()) {
        const int leftIndex = get<0>(vObs);
        cv::KeyPoint kpUn;

        // 单目观测
        if (leftIndex != -1 && pKFi->mvuRight[get<0>(vObs)] < 0) {
          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);               // 左目单目边

          g2o::OptimizableGraph::Vertex* VP =
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                  optimizer.vertex(pKFi->mnId));
          if (bAllFixed)
            if (!VP->fixed()) bAllFixed = false;

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
        }
        // 双目观测
        else if (leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0) {
          kpUn = pKFi->mvKeysUn[leftIndex];
          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);           // 双目边

          g2o::OptimizableGraph::Vertex* VP =
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                  optimizer.vertex(pKFi->mnId));
          if (bAllFixed)
            if (!VP->fixed()) bAllFixed = false;

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
        }

        // 鱼眼右目观测
        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(vObs);

          if (rightIndex != -1 &&
              rightIndex < static_cast<int>(pKFi->mvKeysRight.size())) {
            rightIndex -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            kpUn = pKFi->mvKeysRight[rightIndex];
            obs << kpUn.pt.x, kpUn.pt.y;

            EdgeMono* e = new EdgeMono(1);             // 右目单目边

            g2o::OptimizableGraph::Vertex* VP =
                dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                    optimizer.vertex(pKFi->mnId));
            if (bAllFixed)
              if (!VP->fixed()) bAllFixed = false;

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, VP);
            e->setMeasurement(obs);
            const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
          }
        }
      }
    }

    // ---- 6.2 若所有观测 KF 都固定，该点无优化价值 ----
    if (bAllFixed) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP.insert(pMP);
    }
  }

  if (pbStopFlag && *pbStopFlag) return;

  // ---- 7. 执行优化 ----
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // ---- 8. 恢复优化后的位姿、速度、零偏 ----
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    if (nLoopId == 0) {
      // 普通 BA：直接写回
      Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                       VP->estimate().tcw[0].cast<float>());
      pKFi->SetPose(Tcw);
    } else {
      // 回环后 BA：暂存
      pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(),
                                   VP->estimate().tcw[0].cast<float>());
      pKFi->mnBAGlobalForKF = nLoopId;
    }

    // 恢复速度与零偏
    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      if (nLoopId == 0) {
        pKFi->SetVelocity(VV->estimate().cast<float>());
      } else {
        pKFi->mVwbGBA = VV->estimate().cast<float>();
      }

      VertexGyroBias* VG;
      VertexAccBias* VA;
      if (!bInit) {
        VG = static_cast<VertexGyroBias*>(
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
        VA = static_cast<VertexAccBias*>(
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      } else {
        VG = static_cast<VertexGyroBias*>(optimizer.vertex(4 * maxKFid + 2));
        VA = static_cast<VertexAccBias*>(optimizer.vertex(4 * maxKFid + 3));
      }

      Vector6d vb;
      vb << VG->estimate(), VA->estimate();
      IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
      if (nLoopId == 0) {
        pKFi->SetNewBias(b);
      } else {
        pKFi->mBiasGBA = b;
      }
    }
  }

  // ---- 9. 恢复优化后的地图点 ----
  for (auto pMP : vpMPs) {
    if (vbNotIncludedMP.find(pMP) != vbNotIncludedMP.end()) continue;

    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));

    if (nLoopId == 0) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopId;
    }
  }

  pMap->IncreaseChangeIndex();                         // 地图变更计数 +1
}


// ============================================================================
//  三、帧位姿优化（仅优化单帧位姿，地图点固定）
// ============================================================================

/**
 * 单帧位姿优化 PoseOptimization
 *
 * 使用场景：
 *   - 跟踪线程中，每帧根据已有地图点估计当前帧位姿
 *   - 只优化 1 个位姿顶点（id=0），地图点全部固定
 *
 * 观测边：
 *   - 单目：EdgeSE3ProjectXYZOnlyPose
 *   - 双目：EdgeStereoSE3ProjectXYZOnlyPose
 *   - 鱼眼右目：EdgeSE3ProjectXYZOnlyPoseToBody
 *
 * 优化策略（4 轮迭代）：
 *   每轮先优化，再根据卡方阈值把外点降级（setLevel(1)），
 *   第 3 轮起去掉鲁棒核（此时外点已基本剔除），
 *   最后返回内点数量。
 *
 * @param pFrame 当前帧
 * @return 内点数量
 */
int Optimizer::PoseOptimization(const std::shared_ptr<Frame>& pFrame) {
  // ---- 1. 构建优化器（稠密求解，因为只有 1 个位姿顶点）----
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  oslog::trace("Enter PoseOptimization");

  // 稠密线性求解器，块 6x3
  std::unique_ptr<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>
      linearSolver(
          std::make_unique<
              g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>());
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(
          std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;                     // 初始匹配数

  // ---- 2. 添加当前帧位姿顶点（唯一可优化变量）----
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  {
    const Sophus::SE3<float> Tcw = pFrame->GetPose();  // 当前帧位姿初值
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
  }
  vSE3->setId(0);                                      // id 固定为 0
  vSE3->setFixed(false);                               // 待优化
  optimizer.addVertex(vSE3);

  // ---- 3. 遍历特征点，添加观测边 ----
  const int N = pFrame->N;                             // 特征点总数

  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;      // 单目边
  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody*> vpEdgesMono_FHR; // 鱼眼边
  vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;               // 对应索引
  vpEdgesMono.reserve(N);
  vpEdgesMono_FHR.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeRight.reserve(N);

  vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;    // 双目边
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  // 鲁棒核阈值
  const float deltaMono = sqrt(5.991);                 // 2 自由度 95%
  const float deltaStereo = sqrt(7.815);               // 3 自由度 95%

  {
    // 锁定地图点全局互斥锁（防止地图点被其他线程修改）
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];         // 第 i 个特征对应地图点
      if (pMP) {
        // ---- 3.1 普通 SLAM（单相机）----
        if (!pFrame->mpCamera2) {
          // 单目观测（右目无匹配）
          if (pFrame->mvuRight[i] < 0) {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;             // 初始认为非外点

            Eigen::Matrix<double, 2, 1> obs;           // 2D 观测
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            // 仅位姿的单目重投影边
            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;             // 相机模型
            e->Xw = pMP->GetWorldPos().cast<double>(); // 地图点世界坐标（固定）

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          }
          // 双目观测
          else {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 3, 1> obs;           // 3D 观测 (uL, vL, uR)
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            const float& kp_ur = pFrame->mvuRight[i];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            // 仅位姿的双目重投影边
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e =
                new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaStereo);

            // 双目内参
            e->fx = pFrame->fx; e->fy = pFrame->fy;
            e->cx = pFrame->cx; e->cy = pFrame->cy;
            e->bf = pFrame->mbf;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);
          }
        }
        // ---- 3.2 多相机 SLAM（刚体双相机）----
        else {
          nInitialCorrespondences++;

          cv::KeyPoint kpUn;

          // 左目观测
          if (i < pFrame->Nleft) {
            kpUn = pFrame->mvKeys[i];

            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          }
          // 右目观测
          else {
            kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            pFrame->mvbOutlier[i] = false;

            // 右目到刚体的边
            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera2;
            e->Xw = pMP->GetWorldPos().cast<double>();

            // 左目→右目的相对位姿
            e->mTrl = g2o::SE3Quat(
                pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(),
                pFrame->GetRelativePoseTrl().translation().cast<double>());

            optimizer.addEdge(e);

            vpEdgesMono_FHR.push_back(e);
            vnIndexEdgeRight.push_back(i);
          }
        }
      }
    }
  }

  // 匹配数太少，无法优化
  if (nInitialCorrespondences < 3) return 0;

  oslog::trace("   ... starting PoseOptimization");

  // ---- 4. 4 轮迭代优化 + 外点剔除 ----
  // 每轮优化后，卡方超过阈值的观测标记为外点，下轮不参与优化
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
  const int iterations[4] = {10, 10, 10, 10};

  int nBad = 0;                                        // 外点数
  for (int iter = 0; iter < 4; iter++) {
    oslog::trace("   ... Iter {}", iter);

    // 每轮用最新位姿重置顶点初值
    {
      const Sophus::SE3<float> Tcw = pFrame->GetPose();
      g2o::SE3Quat est = g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                      Tcw.translation().cast<double>());
      vSE3->setEstimate(est);
    }

    optimizer.initializeOptimization(0);               // level 0 的边才参与
    optimizer.optimize(iterations[iter]);

    nBad = 0;

    // ---- 4.1 单目边外点判断 ----
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();                             // 重新计算误差
      }

      const float chi2 = e->chi2();                    // 卡方

      if (chi2 > chi2Mono[iter]) {
        pFrame->mvbOutlier[idx] = true;                // 标记外点
        e->setLevel(1);                                // 降级，下轮不参与
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;               // 内点
        e->setLevel(0);
      }

      // 第 3 轮起去掉鲁棒核
      if (iter == 2) e->setRobustKernel(0);
    }

    // ---- 4.2 鱼眼右目边外点判断 ----
    for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

      const size_t idx = vnIndexEdgeRight[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[iter]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (iter == 2) e->setRobustKernel(0);
    }

    // ---- 4.3 双目边外点判断 ----
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[iter]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        e->setLevel(0);
        pFrame->mvbOutlier[idx] = false;
      }

      if (iter == 2) e->setRobustKernel(0);
    }

    // 有效边太少，提前退出
    if (optimizer.edges().size() < 10) break;
  }

  // ---- 5. 恢复优化后的位姿 ----
  const g2o::VertexSE3Expmap* vSE3_recov =
      static_cast<const g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  const g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  const Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                                SE3quat_recov.translation().cast<float>());
  pFrame->SetPose(pose);

  oslog::trace(" Exit PoseOptimization");
  return nInitialCorrespondences - nBad;               // 返回内点数
}

// ============================================================================
//  四、局部 BA（第一版：基于共视关系，标准 SLAM 局部 BA）
// ============================================================================

/**
 * 局部 BA（共视版）
 *
 * 优化范围：
 *   - 局部关键帧（当前 KF + 共视 KF）位姿可优化
 *   - 固定关键帧（能看到局部地图点但不在局部窗口内的 KF）位姿固定
 *   - 局部地图点位置可优化
 *
 * 用途：跟踪线程中每插入一个 KF 就调用一次，保持局部一致性。
 *
 * @param pKF         当前关键帧
 * @param pbStopFlag  停止标志
 * @param pMap        地图
 * @param num_fixedKF 输出：固定 KF 数
 * @param num_OptKF   输出：优化 KF 数
 * @param num_MPs     输出：优化地图点数
 * @param num_edges   输出：边数
 */
void Optimizer::LocalBundleAdjustment(const shared_ptr<KeyFrame>& pKF,
                                      bool* pbStopFlag,
                                      const std::shared_ptr<Map>& pMap,
                                      int& num_fixedKF, int& num_OptKF,
                                      int& num_MPs, int& num_edges) {
  // ---- 1. 收集局部关键帧（共视关系）----
  list<shared_ptr<KeyFrame>> lLocalKeyFrames;

  lLocalKeyFrames.push_back(pKF);                      // 当前 KF
  pKF->mnBALocalForKF = pKF->mnId;                     // 标记已加入局部 BA
  auto pCurrentMap = pKF->GetMap();

  // 共视 KF（与当前 KF 有共同观测）
  const auto vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
  for (auto pKFi : vNeighKFs) {
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
      lLocalKeyFrames.push_back(pKFi);
  }

  // ---- 2. 收集局部地图点（局部 KF 观测到的）----
  num_fixedKF = 0;
  list<MapPoint*> lLocalMapPoints;
  set<MapPoint*> sNumObsMP;
  for (auto pKFi : lLocalKeyFrames) {
    if (pKFi->mnId == pMap->GetInitKFid()) {
      num_fixedKF = 1;                                 // 初始 KF 固定
    }
    vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();

    for (auto pMP : vpMPs) {
      if (pMP && !pMP->isBad() && pMP->GetMap() == pCurrentMap) {
        if (pMP->mnBALocalForKF != pKF->mnId) {
          lLocalMapPoints.push_back(pMP);
          pMP->mnBALocalForKF = pKF->mnId;             // 标记已加入
        }
      }
    }
  }

  // ---- 3. 收集固定关键帧（能看到局部地图点但不在局部窗口内）----
  list<shared_ptr<KeyFrame>> lFixedCameras;
  for (auto pMP : lLocalMapPoints) {
    map<shared_ptr<KeyFrame>, tuple<int, int>> observations =
        pMP->GetObservations();

    for (auto mit = observations.begin(), mend = observations.end();
         mit != mend; mit++) {
      shared_ptr<KeyFrame> pKFi = mit->first;

      // 未标记为局部 KF 也未标记为固定 KF
      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
          lFixedCameras.push_back(pKFi);
      }
    }
  }
  num_fixedKF = lFixedCameras.size() + num_fixedKF;

  // 无固定 KF，BA 无意义
  if (num_fixedKF == 0) {
    oslog::info(
        "LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted");
    return;
  }

  // ---- 4. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

  // 惯性模式下增大初始阻尼
  if (pMap->IsInertial()) solver->setUserLambdaInit(100.0);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  // DEBUG：清空调试集合
  pCurrentMap->msOptKFs.clear();
  pCurrentMap->msFixedKFs.clear();

  // ---- 5. 添加局部 KF 位姿顶点（可优化）----
  for (auto pKFi : lLocalKeyFrames) {
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    {
      const Sophus::SE3<float> Tcw = pKFi->GetPose();
      vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                     Tcw.translation().cast<double>()));
    }
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid()); // 初始 KF 固定
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    pCurrentMap->msOptKFs.insert(pKFi->mnId);          // DEBUG
  }
  num_OptKF = lLocalKeyFrames.size();

  // ---- 6. 添加固定 KF 位姿顶点 ----
  for (auto pKFi : lFixedCameras) {
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(true);                              // 固定
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    pCurrentMap->msFixedKFs.insert(pKFi->mnId);        // DEBUG
  }

  // ---- 7. 预分配边的容器 ----
  const int nExpectedSize =
      (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

  vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  vector<shared_ptr<KeyFrame>> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<shared_ptr<KeyFrame>> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<shared_ptr<KeyFrame>> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  int nPoints = 0;
  int nEdges = 0;

  // ---- 8. 添加局部地图点及其观测边 ----
  for (auto pMP : lLocalMapPoints) {
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    int id = pMP->mnId + maxKFid + 1;                  // id 偏移
    vPoint->setId(id);
    vPoint->setMarginalized(true);                     // 边缘化
    optimizer.addVertex(vPoint);
    nPoints++;

    map<shared_ptr<KeyFrame>, tuple<int, int>> observations =
        pMP->GetObservations();

    // ---- 遍历观测，添加边 ----
    for (auto const& [pKFi, vObs] : observations) {
      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = get<0>(vObs);

        // ---- 8.1 单目观测 ----
        if (leftIndex != -1 && pKFi->mvuRight[get<0>(vObs)] < 0) {
          const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          e->pCamera = pKFi->mpCamera;

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);

          nEdges++;
        }
        // ---- 8.2 双目观测 ----
        else if (leftIndex != -1 && pKFi->mvuRight[get<0>(vObs)] >= 0) {
          const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKFi->mvuRight[get<0>(vObs)];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          e->fx = pKFi->fx; e->fy = pKFi->fy;
          e->cx = pKFi->cx; e->cy = pKFi->cy;
          e->bf = pKFi->mbf;

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);

          nEdges++;
        }

        // ---- 8.3 鱼眼右目观测 ----
        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(vObs);

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZToBody* e =
                new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            Sophus::SE3f Trl = pKFi->GetRelativePoseTrl();
            e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                   Trl.translation().cast<double>());

            e->pCamera = pKFi->mpCamera2;

            optimizer.addEdge(e);
            vpEdgesBody.push_back(e);
            vpEdgeKFBody.push_back(pKFi);
            vpMapPointEdgeBody.push_back(pMP);

            nEdges++;
          }
        }
      }
    }
  }
  num_edges = nEdges;

  if (pbStopFlag)
    if (*pbStopFlag) return;

  // ---- 9. 执行优化 ----
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // ---- 10. 收集外点，准备剔除 ----
  vector<pair<shared_ptr<KeyFrame>, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                   vpEdgesStereo.size());

  // 单目边外点
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      auto pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // 鱼眼边外点
  for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
    MapPoint* pMP = vpMapPointEdgeBody[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      auto pKFi = vpEdgeKFBody[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // 双目边外点
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      auto pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // ---- 11. 剔除外点观测 ----
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      shared_ptr<KeyFrame> pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);                  // 删除 KF→MP 匹配
      pMPi->EraseObservation(pKFi);                    // 删除 MP→KF 观测
    }
  }

  // ---- 12. 恢复优化后的 KF 位姿 ----
  for (auto pKFi : lLocalKeyFrames) {
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());
    pKFi->SetPose(Tiw);
  }

  // ---- 13. 恢复优化后的地图点位置 ----
  for (auto pMP : lLocalMapPoints) {
    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();                         // 地图变更计数 +1
}

/**
 * 局部 BA（地图合并版）
 *
 * 与上面共视版不同：
 *   - 不基于共视关系，而是显式传入可优化 KF 和固定 KF
 *   - 用于地图合并（Map Merging）后，把融合后的局部窗口做一次 BA
 *
 * @param pMainKF     主 KF
 * @param vpAdjustKF  可优化 KF 列表
 * @param vpFixedKF   固定 KF 列表
 * @param pbStopFlag  停止标志
 */
void Optimizer::LocalBundleAdjustment(const shared_ptr<KeyFrame>& pMainKF,
                                      vector<shared_ptr<KeyFrame>> vpAdjustKF,
                                      vector<shared_ptr<KeyFrame>> vpFixedKF,
                                      bool* pbStopFlag) {
  vector<MapPoint*> vpMPs;                             // 局部地图点

  // ---- 1. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;
  set<shared_ptr<KeyFrame>> spKeyFrameBA;              // 参与 BA 的 KF 集合

  std::shared_ptr<Map> pCurrentMap = pMainKF->GetMap();

  // ---- 2. 添加固定 KF 顶点 ----
  int numInsertedPoints = 0;
  for (auto pKFi : vpFixedKF) {
    if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
      Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }

    pKFi->mnBALocalForMerge = pMainKF->mnId;           // 标记合并 BA

    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(true);                              // 固定
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

    // 收集该 KF 观测的地图点
    set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
    for (MapPoint* pMPi : spViewMPs) {
      if (pMPi) {
        if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {
          if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(pMPi);
            pMPi->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
        }
      }
    }

    spKeyFrameBA.insert(pKFi);
  }

  // ---- 3. 添加可优化 KF 顶点 ----
  set<shared_ptr<KeyFrame>> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
  numInsertedPoints = 0;
  for (auto pKFi : vpAdjustKF) {
    if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) continue;

    pKFi->mnBALocalForMerge = pMainKF->mnId;

    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    optimizer.addVertex(vSE3);                         // 可优化（未 setFixed）
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

    // 收集该 KF 观测的地图点
    set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
    for (MapPoint* pMPi : spViewMPs) {
      if (pMPi) {
        if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {
          if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(pMPi);
            pMPi->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
        }
      }
    }

    spKeyFrameBA.insert(pKFi);
  }

  // ---- 4. 预分配边容器 ----
  const int nExpectedSize =
      (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

  vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<shared_ptr<KeyFrame>> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<shared_ptr<KeyFrame>> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuber2D = sqrt(5.99);
  const float thHuber3D = sqrt(7.815);

  // ---- 5. 添加地图点顶点及观测边 ----
  map<shared_ptr<KeyFrame>, int> mpObsKFs;             // 调试：KF 观测数
  map<shared_ptr<KeyFrame>, int> mpObsFinalKFs;
  map<MapPoint*, int> mpObsMPs;                        // 调试：MP 观测数
  for (auto pMPi : vpMPs) {
    if (pMPi->isBad()) continue;

    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
    const int id = pMPi->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<shared_ptr<KeyFrame>, tuple<int, int>> observations =
        pMPi->GetObservations();
    int nEdges = 0;

    // ---- 遍历观测 ----
    for (auto const& [pKF, tObs] : observations) {
      // 只处理参与本次 BA 的 KF
      if (pKF->isBad() || pKF->mnId > maxKFid ||
          pKF->mnBALocalForMerge != pMainKF->mnId ||
          !pKF->GetMapPoint(get<0>(tObs)))
        continue;

      nEdges++;

      const cv::KeyPoint& kpUn = pKF->mvKeysUn[get<0>(tObs)];

      // ---- 5.1 单目观测 ----
      if (pKF->mvuRight[get<0>(tObs)] < 0) {
        mpObsMPs[pMPi]++;
        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);
        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber2D);

        e->pCamera = pKF->mpCamera;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(pKF);
        vpMapPointEdgeMono.push_back(pMPi);

        mpObsKFs[pKF]++;
      }
      // ---- 5.2 双目/RGBD 观测 ----
      else {
        mpObsMPs[pMPi] += 2;
        Eigen::Matrix<double, 3, 1> obs;
        const float kp_ur = pKF->mvuRight[get<0>(tObs)];
        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);
        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
        e->setInformation(Info);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber3D);

        e->fx = pKF->fx; e->fy = pKF->fy;
        e->cx = pKF->cx; e->cy = pKF->cy;
        e->bf = pKF->mbf;

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(pKF);
        vpMapPointEdgeStereo.push_back(pMPi);

        mpObsKFs[pKF]++;
      }
    }
  }

  if (pbStopFlag && *pbStopFlag) return;

  // ---- 6. 两阶段优化 ----
  optimizer.initializeOptimization();
  optimizer.optimize(5);                               // 第一阶段：5 次

  bool bDoMore = true;                                 // 是否继续第二阶段
  if (pbStopFlag && *pbStopFlag) bDoMore = false;

  map<unsigned long int, int> mWrongObsKF;             // 记录各 KF 外点数
  if (bDoMore) {
    // 第一阶段后标记外点
    int badMonoMP = 0, badStereoMP = 0;

    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      MapPoint* pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        e->setLevel(1);                                // 降级
        badMonoMP++;
      }
      e->setRobustKernel(0);                           // 去掉鲁棒核
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
      MapPoint* pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        e->setLevel(1);
        badStereoMP++;
      }
      e->setRobustKernel(0);
    }

    Verbose::PrintMess("[BA]: First optimization(Huber), there are " +
                           to_string(badMonoMP) + " monocular and " +
                           to_string(badStereoMP) + " stereo bad edges",
                       Verbose::VERBOSITY_DEBUG);

    optimizer.initializeOptimization(0);               // 只用 level 0 的边
    optimizer.optimize(10);                            // 第二阶段：10 次
  }

  // ---- 7. 收集最终外点 ----
  vector<pair<shared_ptr<KeyFrame>, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
  set<MapPoint*> spErasedMPs;
  set<shared_ptr<KeyFrame>> spErasedKFs;

  int badMonoMP = 0, badStereoMP = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      const shared_ptr<KeyFrame> pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badMonoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      const shared_ptr<KeyFrame> pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badStereoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  Verbose::PrintMess("[BA]: Second optimization, there are " +
                         to_string(badMonoMP) + " monocular and " +
                         to_string(badStereoMP) + " sterero bad edges",
                     Verbose::VERBOSITY_DEBUG);

  // ---- 8. 剔除外点 ----
  unique_lock<mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      shared_ptr<KeyFrame> pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // 统计最终观测数（调试）
  for (auto const pMPi : vpMPs) {
    if (pMPi->isBad()) continue;

    const map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations =
        pMPi->GetObservations();
    for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::const_iterator mit =
             observations.begin();
         mit != observations.end(); mit++) {
      std::shared_ptr<KeyFrame> pKF = mit->first;
      if (pKF->isBad() || pKF->mnId > maxKFid ||
          pKF->mnBALocalForKF != pMainKF->mnId ||
          !pKF->GetMapPoint(get<0>(mit->second)))
        continue;

      if (pKF->mvuRight[get<0>(mit->second)] < 0) {
        mpObsFinalKFs[pKF]++;
      } else {
        mpObsFinalKFs[pKF]++;
      }
    }
  }

  // ---- 9. 恢复优化后的 KF 位姿 ----
  for (auto const& pKFi : vpAdjustKF) {
    if (pKFi->isBad()) continue;

    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());

    // 统计内外点（调试用）
    int numMonoBadPoints = 0, numMonoOptPoints = 0;
    int numStereoBadPoints = 0, numStereoOptPoints = 0;
    vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;
    vector<MapPoint*> vpMonoMPsBad, vpStereoMPsBad;

    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      MapPoint* pMP = vpMapPointEdgeMono[i];
      std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i];

      if (pKFi != pKFedge) continue;
      if (pMP->isBad()) continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        numMonoBadPoints++;
        vpMonoMPsBad.push_back(pMP);
      } else {
        numMonoOptPoints++;
        vpMonoMPsOpt.push_back(pMP);
      }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
      MapPoint* pMP = vpMapPointEdgeStereo[i];
      const std::shared_ptr<KeyFrame>& pKFedge = vpEdgeKFMono[i];

      if (pKFi != pKFedge) continue;
      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        numStereoBadPoints++;
        vpStereoMPsBad.push_back(pMP);
      } else {
        numStereoOptPoints++;
        vpStereoMPsOpt.push_back(pMP);
      }
    }

    pKFi->SetPose(Tiw);
  }

  // ---- 10. 恢复优化后的地图点位置 ----
  for (MapPoint* pMPi : vpMPs) {
    if (pMPi->isBad()) continue;

    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
        optimizer.vertex(pMPi->mnId + maxKFid + 1));
    pMPi->SetWorldPos(vPoint->estimate().cast<float>());
    pMPi->UpdateNormalAndDepth();
  }
}


// ============================================================================
//  五、本质图优化（回环闭合后使用，第一版：Sim3 位姿图）
// ============================================================================

/**
 * 本质图优化 OptimizeEssentialGraph（回环版）
 *
 * 使用场景：
 *   - 检测到回环后，在本质图（Essential Graph）上做位姿图优化
 *   - 本质图只保留：生成树边、回环边、共视权重 ≥ minFeat 的边
 *   - 使用 Sim3（含尺度）顶点，因为单目 SLAM 回环时存在尺度漂移
 *
 * 顶点：
 *   - Sim3 位姿 VertexSim3Expmap，初始 KF 固定
 * 边：
 *   - EdgeSim3：生成树边、回环边、共视边、惯性边
 *
 * @param pMap              地图
 * @param pLoopKF           回环关键帧
 * @param pCurKF            当前关键帧
 * @param NonCorrectedSim3  未校正的 Sim3 位姿（回环前）
 * @param CorrectedSim3     已校正的 Sim3 位姿（回环后）
 * @param LoopConnections   回环新建立的连接
 * @param bFixScale         是否固定尺度（双目/RGBD 时固定）
 */
void Optimizer::OptimizeEssentialGraph(
    const shared_ptr<Map>& pMap, const shared_ptr<KeyFrame>& pLoopKF,
    const shared_ptr<KeyFrame>& pCurKF,
    const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose& CorrectedSim3,
    const map<shared_ptr<KeyFrame>, set<shared_ptr<KeyFrame>>>& LoopConnections,
    const bool& bFixScale) {
  // ---- 1. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  // 块求解器 7x3：Sim3 顶点 7 维（四元数 4 + 平移 3），点 3 维
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolver_7_3>(std::move(linearSolver)));

  solver->setUserLambdaInit(1e-16);                    // 初始阻尼极小
  optimizer.setAlgorithm(solver);

  const vector<shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  // Sim3 位姿缓存：世界→相机 Scw / 相机→世界 Swc
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

  // 调试用：Z 轴方向
  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vZvectors(
      nMaxKFid + 1);
  Eigen::Vector3d z_vec;
  z_vec << 0.0, 0.0, 1.0;

  const int minFeat = 100;                             // 共视权重阈值

  // ---- 2. 添加关键帧 Sim3 顶点 ----
  for (auto pKF : vpKFs) {
    if (pKF->isBad()) continue;
    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKF->mnId;

    // 优先用回环校正后的 Sim3 作为初值
    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      VSim3->setEstimate(it->second);
    } else {
      // 否则用当前位姿（假设尺度 1）
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
      vScw[nIDi] = Siw;
      VSim3->setEstimate(Siw);
    }

    // 初始 KF 固定
    if (pKF->mnId == pMap->GetInitKFid()) VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);                     // Sim3 顶点不边缘化
    VSim3->_fix_scale = bFixScale;                     // 是否固定尺度

    optimizer.addVertex(VSim3);
    vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec;   // 调试

    vpVertices[nIDi] = VSim3;
  }

  // 记录已插入的边对（避免重复）
  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // Sim3 边信息矩阵 = I（各项同性）
  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  // ---- 3. 添加回环边（LoopConnections）----
  int count_loop = 0;
  for (auto const& [pKF, spConnections] : LoopConnections) {
    const long unsigned int nIDi = pKF->mnId;
    const g2o::Sim3 Siw = vScw[nIDi];
    const g2o::Sim3 Swi = Siw.inverse();

    for (auto const& pConnection : spConnections) {
      auto const nIDj = pConnection->mnId;

      // 只保留强连接（或回环本身的连接）
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(pConnection) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sji = Sjw * Swi;                 // 相对 Sim3

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);

      e->information() = matLambda;

      optimizer.addEdge(e);
      count_loop++;
      sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
    }
  }

  // ---- 4. 添加普通边（生成树 / 回环 / 共视 / 惯性）----
  for (auto pKF : vpKFs) {
    const int nIDi = pKF->mnId;

    g2o::Sim3 Swi;

    // 优先用未校正的 Sim3
    LoopClosing::KeyFrameAndPose::const_iterator iti =
        NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Swi = (iti->second).inverse();
    else
      Swi = vScw[nIDi].inverse();

    shared_ptr<KeyFrame> pParentKF = pKF->GetParent();

    // ---- 4.1 生成树边 ----
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Sjw;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Sjw = itj->second;
      else
        Sjw = vScw[nIDj];

      g2o::Sim3 Sji = Sjw * Swi;

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // ---- 4.2 回环边 ----
    const set<shared_ptr<KeyFrame>> sLoopEdges = pKF->GetLoopEdges();
    for (auto pLKF : sLoopEdges) {
      if (pLKF->mnId < pKF->mnId) {                    // 避免重复
        g2o::Sim3 Slw;

        LoopClosing::KeyFrameAndPose::const_iterator itl =
            NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Slw = itl->second;
        else
          Slw = vScw[pLKF->mnId];

        g2o::Sim3 Sli = Slw * Swi;
        g2o::EdgeSim3* el = new g2o::EdgeSim3();
        el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                             optimizer.vertex(pLKF->mnId)));
        el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                             optimizer.vertex(nIDi)));
        el->setMeasurement(Sli);
        el->information() = matLambda;
        optimizer.addEdge(el);
      }
    }

    // ---- 4.3 共视边 ----
    const vector<shared_ptr<KeyFrame>> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);

    for (auto pKFn : vpConnectedKFs) {
      if (pKFn && pKFn != pParentKF &&
          !pKF->hasChild(pKFn)) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          // 已在回环边中插入，跳过
          if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId),
                                             max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Snw;

          LoopClosing::KeyFrameAndPose::const_iterator itn =
              NonCorrectedSim3.find(pKFn);

          if (itn != NonCorrectedSim3.end())
            Snw = itn->second;
          else
            Snw = vScw[pKFn->mnId];

          g2o::Sim3 Sni = Snw * Swi;

          g2o::EdgeSim3* en = new g2o::EdgeSim3();
          en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(pKFn->mnId)));
          en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(nIDi)));
          en->setMeasurement(Sni);
          en->information() = matLambda;
          optimizer.addEdge(en);
        }
      }
    }

    // ---- 4.4 惯性边（若为惯性模式）----
    if (pKF->bImu && pKF->mPrevKF) {
      g2o::Sim3 Spw;
      LoopClosing::KeyFrameAndPose::const_iterator itp =
          NonCorrectedSim3.find(pKF->mPrevKF);
      if (itp != NonCorrectedSim3.end())
        Spw = itp->second;
      else
        Spw = vScw[pKF->mPrevKF->mnId];

      g2o::Sim3 Spi = Spw * Swi;
      g2o::EdgeSim3* ep = new g2o::EdgeSim3();
      ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                           optimizer.vertex(pKF->mPrevKF->mnId)));
      ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                           optimizer.vertex(nIDi)));
      ep->setMeasurement(Spi);
      ep->information() = matLambda;
      optimizer.addEdge(ep);
    }
  }

  // ---- 5. 执行优化 ----
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();                     // 计算初始误差
  optimizer.optimize(20);                              // 20 次迭代
  optimizer.computeActiveErrors();                     // 计算最终误差
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);      // 锁定地图

  // ---- 6. 恢复 KF 位姿（Sim3 → SE3）----
  for (auto pKFi : vpKFs) {
    const int nIDi = pKFi->mnId;

    g2o::VertexSim3Expmap* VSim3 =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    double s = CorrectedSiw.scale();                   // 尺度

    // Sim3:[sR t; 0 1] → SE3:[R t/s; 0 1]
    Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(),
                     CorrectedSiw.translation().cast<float>() / s);
    pKFi->SetPose(Tiw);
  }

  // ---- 7. 恢复地图点位置 ----
  // 把地图点从"未优化参考 KF 坐标系"变换到"优化后参考 KF 坐标系"
  for (auto pMP : vpMPs) {
    if (pMP->isBad()) continue;

    int nIDr;
    // 若该点被当前 KF 校正过，用校正参考 KF
    if (pMP->mnCorrectedByKF == pCurKF->mnId) {
      nIDr = pMP->mnCorrectedReference;
    } else {
      shared_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
      nIDr = pRefKF->mnId;
    }

    g2o::Sim3 Srw = vScw[nIDr];                        // 旧位姿
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];      // 新位姿

    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    // 先映射到旧 KF 坐标系，再映射到新 KF 坐标系
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();                         // 地图变更计数 +1
}

// ============================================================================
//  五（续）、本质图优化（第二版：地图合并专用）
// ============================================================================

/**
 * 本质图优化 OptimizeEssentialGraph（地图合并版）
 *
 * 使用场景：
 *   - 地图合并（Map Merging）后，对合并后的两组 KF 做一次位姿图优化
 *   - 显式区分固定 KF、已校正 KF、未固定 KF
 *
 * @param pCurKF              当前 KF
 * @param vpFixedKFs          固定 KF（合并地图中的）
 * @param vpFixedCorrectedKFs 固定 KF（旧地图中已校正的）
 * @param vpNonFixedKFs       非固定 KF（待优化）
 * @param vpNonCorrectedMPs   未校正的地图点
 */
void Optimizer::OptimizeEssentialGraph(
    const std::shared_ptr<KeyFrame>& pCurKF,
    vector<shared_ptr<KeyFrame>>& vpFixedKFs,
    vector<shared_ptr<KeyFrame>>& vpFixedCorrectedKFs,
    vector<shared_ptr<KeyFrame>>& vpNonFixedKFs,
    vector<MapPoint*>& vpNonCorrectedMPs) {
  // 打印各类 KF 数量（调试）
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpFixedKFs.size()) +
                         " KFs fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpFixedCorrectedKFs.size()) +
                         " KFs fixed in the old map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpNonFixedKFs.size()) +
                         " KFs non-fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpNonCorrectedMPs.size()) +
                         " MPs non-corrected in the merged map",
                     Verbose::VERBOSITY_DEBUG);

  // ---- 1. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolver_7_3>(std::move(linearSolver)));

  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  std::shared_ptr<Map> pMap = pCurKF->GetMap();
  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  // Sim3 缓存
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

  // 位姿好坏标记
  vector<bool> vpGoodPose(nMaxKFid + 1);               // 已校正位姿可用
  vector<bool> vpBadPose(nMaxKFid + 1);                // 未校正位姿可用

  const int minFeat = 100;

  // ---- 2. 添加固定 KF 顶点（合并地图）----
  for (auto const& pKFi : vpFixedKFs) {
    if (pKFi->isBad()) continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKFi->mnId;

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    VSim3->setFixed(true);                             // 固定

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = true;                          // 固定尺度

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = false;
  }
  Verbose::PrintMess("Opt_Essential: vpFixedKFs loaded",
                     Verbose::VERBOSITY_DEBUG);

  // ---- 3. 添加固定 KF 顶点（旧地图已校正）----
  set<unsigned long> sIdKF;
  for (auto const& pKFi : vpFixedCorrectedKFs) {
    if (pKFi->isBad()) continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKFi->mnId;

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    // 旧位姿（合并前）
    Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
    vScw[nIDi] =
        g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

    VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = true;                            // 两种位姿都可用
  }

  // ---- 4. 添加非固定 KF 顶点（待优化）----
  for (auto const& pKFi : vpNonFixedKFs) {
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;

    // 已在上面添加过，跳过
    if (sIdKF.count(nIDi)) continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vScw[nIDi] = Siw;
    VSim3->setEstimate(Siw);

    VSim3->setFixed(false);                            // 可优化

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = false;
    vpBadPose[nIDi] = true;
  }

  // ---- 5. 合并所有 KF 到统一容器 ----
  vector<shared_ptr<KeyFrame>> vpKFs;
  vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() +
                vpNonFixedKFs.size());
  vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
  vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(),
               vpFixedCorrectedKFs.end());
  vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
  set<shared_ptr<KeyFrame>> spKFs(vpKFs.begin(), vpKFs.end()); // 用于快速查找

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  // ---- 6. 添加各类边 ----
  for (auto const& pKFi : vpKFs) {
    int num_connections = 0;                           // 该 KF 的连接数
    const int nIDi = pKFi->mnId;

    g2o::Sim3 correctedSwi;
    g2o::Sim3 Swi;

    if (vpGoodPose[nIDi]) correctedSwi = vCorrectedSwc[nIDi];
    if (vpBadPose[nIDi]) Swi = vScw[nIDi].inverse();

    auto pParentKFi = pKFi->GetParent();

    // ---- 6.1 生成树边 ----
    if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
      int nIDj = pParentKFi->mnId;

      g2o::Sim3 Sjw;
      bool bHasRelation = false;

      // 两者位姿都可用才建立约束
      if (vpGoodPose[nIDi] && vpGoodPose[nIDj]) {
        Sjw = vCorrectedSwc[nIDj].inverse();
        bHasRelation = true;
      } else if (vpBadPose[nIDi] && vpBadPose[nIDj]) {
        Sjw = vScw[nIDj];
        bHasRelation = true;
      }

      if (bHasRelation) {
        g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3* e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);
        e->information() = matLambda;
        optimizer.addEdge(e);
        num_connections++;
      }
    }

    // ---- 6.2 回环边 ----
    const set<std::shared_ptr<KeyFrame>> sLoopEdges = pKFi->GetLoopEdges();
    for (auto const& pLKF : sLoopEdges) {
      if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId) {
        g2o::Sim3 Slw;
        bool bHasRelation = false;

        if (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId]) {
          Slw = vCorrectedSwc[pLKF->mnId].inverse();
          bHasRelation = true;
        } else if (vpBadPose[nIDi] && vpBadPose[pLKF->mnId]) {
          Slw = vScw[pLKF->mnId];
          bHasRelation = true;
        }

        if (bHasRelation) {
          g2o::Sim3 Sli = Slw * Swi;
          g2o::EdgeSim3* el = new g2o::EdgeSim3();
          el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(pLKF->mnId)));
          el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(nIDi)));
          el->setMeasurement(Sli);
          el->information() = matLambda;
          optimizer.addEdge(el);
          num_connections++;
        }
      }
    }

    // ---- 6.3 共视边 ----
    const vector<shared_ptr<KeyFrame>> vpConnectedKFs =
        pKFi->GetCovisiblesByWeight(minFeat);
    for (auto pKFn : vpConnectedKFs) {
      if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) &&
          !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end()) {
        if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {
          g2o::Sim3 Snw = vScw[pKFn->mnId];
          bool bHasRelation = false;

          if (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId]) {
            Snw = vCorrectedSwc[pKFn->mnId].inverse();
            bHasRelation = true;
          } else if (vpBadPose[nIDi] && vpBadPose[pKFn->mnId]) {
            Snw = vScw[pKFn->mnId];
            bHasRelation = true;
          }

          if (bHasRelation) {
            g2o::Sim3 Sni = Snw * Swi;

            g2o::EdgeSim3* en = new g2o::EdgeSim3();
            en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                 optimizer.vertex(pKFn->mnId)));
            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                 optimizer.vertex(nIDi)));
            en->setMeasurement(Sni);
            en->information() = matLambda;
            optimizer.addEdge(en);
            num_connections++;
          }
        }
      }
    }

    // 该 KF 无任何连接（孤立点）
    if (num_connections == 0) {
      Verbose::PrintMess(
          "Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections",
          Verbose::VERBOSITY_DEBUG);
    }
  }

  // ---- 7. 执行优化 ----
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // ---- 8. 恢复非固定 KF 位姿 ----
  for (auto pKFi : vpNonFixedKFs) {
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;

    g2o::VertexSim3Expmap* VSim3 =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    double s = CorrectedSiw.scale();
    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation() / s);

    // 保存合并前位姿，用于地图点校正
    pKFi->mTcwBefMerge = pKFi->GetPose();
    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
    pKFi->SetPose(Tiw.cast<float>());
  }

  // ---- 9. 恢复未校正地图点 ----
  for (auto pMPi : vpNonCorrectedMPs) {
    if (pMPi->isBad()) continue;

    auto pRefKF = pMPi->GetReferenceKeyFrame();
    // 跳过坏参考 KF
    while (pRefKF->isBad()) {
      if (!pRefKF) {
        Verbose::PrintMess(
            "MP " + to_string(pMPi->mnId) + " without a valid reference KF",
            Verbose::VERBOSITY_DEBUG);
        break;
      }

      pMPi->EraseObservation(pRefKF);
      pRefKF = pMPi->GetReferenceKeyFrame();
    }

    // 若参考 KF 是"旧地图"KF，需要重新校正
    if (vpBadPose[pRefKF->mnId]) {
      Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
      Sophus::SE3f Twr = pRefKF->GetPoseInverse();

      // 旧位姿 → 新位姿
      Eigen::Vector3f eigCorrectedP3Dw =
          Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
      pMPi->SetWorldPos(eigCorrectedP3Dw);

      pMPi->UpdateNormalAndDepth();
    } else {
      // 参考 KF 来自另一地图，无法校正
      cout << "ERROR: MapPoint has a reference KF from another map" << endl;
    }
  }
}

// ============================================================================
//  六、Sim3 优化（回环检测时估计两帧之间的 Sim3 变换）
// ============================================================================

/**
 * Sim3 优化 OptimizeSim3
 *
 * 使用场景：
 *   - 回环检测时，用两帧之间的匹配点估计 Sim3 变换
 *   - 同时估计尺度（单目 SLAM 必要）
 *
 * 顶点：
 *   - Sim3 位姿 VertexSim3Expmap（id=0）
 *   - 地图点 VertexPointXYZ（两组，均固定）
 * 边：
 *   - EdgeSim3ProjectXYZ：S12 * X2 投影到 KF1
 *   - EdgeInverseSim3ProjectXYZ：S21 * X1 投影到 KF2
 *
 * @param pKF1          关键帧 1
 * @param pKF2          关键帧 2
 * @param vpMatches1    匹配关系（in/out：被剔除外点置 NULL）
 * @param g2oS12        in/out：Sim3 变换
 * @param th2           卡方阈值
 * @param bFixScale     是否固定尺度
 * @param mAcumHessian  输出：累积 Hessian（用于回环不确定性）
 * @param bAllPoints    是否使用所有点
 * @return 内点数
 */
int Optimizer::OptimizeSim3(const shared_ptr<KeyFrame>& pKF1,
                            const shared_ptr<KeyFrame>& pKF2,
                            vector<MapPoint*>& vpMatches1, g2o::Sim3& g2oS12,
                            const float th2, const bool bFixScale,
                            Eigen::Matrix<double, 7, 7>& mAcumHessian,
                            const bool bAllPoints) {
  // ---- 1. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  // ---- 2. 取出两个 KF 的相机位姿 ----
  const Eigen::Matrix3f R1w = pKF1->GetRotation();     // 世界→KF1 旋转
  const Eigen::Vector3f t1w = pKF1->GetTranslation();  // 世界→KF1 平移
  const Eigen::Matrix3f R2w = pKF2->GetRotation();     // 世界→KF2 旋转
  const Eigen::Vector3f t2w = pKF2->GetTranslation();  // 世界→KF2 平移

  // ---- 3. 添加 Sim3 位姿顶点 ----
  ORB_SLAM3::VertexSim3Expmap* vSim3 = new ORB_SLAM3::VertexSim3Expmap();
  vSim3->_fix_scale = bFixScale;
  vSim3->setEstimate(g2oS12);
  vSim3->setId(0);
  vSim3->setFixed(false);
  vSim3->pCamera1 = pKF1->mpCamera;
  vSim3->pCamera2 = pKF2->mpCamera;
  optimizer.addVertex(vSim3);

  // ---- 4. 添加地图点顶点及双向边 ----
  const int N = vpMatches1.size();
  const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
  vector<ORB_SLAM3::EdgeSim3ProjectXYZ*> vpEdges12;    // S12 * X2 → KF1
  vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*> vpEdges21; // S21 * X1 → KF2
  vector<size_t> vnIndexEdge;
  vector<bool> vbIsInKF2;                              // 该点在 KF2 中是否有观测

  vnIndexEdge.reserve(2 * N);
  vpEdges12.reserve(2 * N);
  vpEdges21.reserve(2 * N);
  vbIsInKF2.reserve(2 * N);

  const float deltaHuber = sqrt(th2);                  // 鲁棒核阈值

  int nCorrespondences = 0;                            // 有效匹配数
  int nBadMPs = 0;                                     // 坏点数
  int nInKF2 = 0;                                      // 在 KF2 中有观测的点数
  int nOutKF2 = 0;                                     // 仅在 KF1 中的点数
  int nMatchWithoutMP = 0;                             // 无 MP 的匹配数

  vector<int> vIdsOnlyInKF2;                           // 仅在 KF2 中的顶点 id

  // ---- 遍历匹配对 ----
  for (int i = 0; i < N; i++) {
    if (!vpMatches1[i]) continue;

    MapPoint* pMP1 = vpMapPoints1[i];                  // KF1 中的点
    MapPoint* pMP2 = vpMatches1[i];                    // KF2 中的匹配点

    const int id1 = 2 * i + 1;                         // KF1 点顶点 id（奇）
    const int id2 = 2 * (i + 1);                       // KF2 点顶点 id（偶）

    // 点 pMP2 在 KF2 中的索引
    const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));

    Eigen::Vector3f P3D1c;                             // KF1 坐标系下的点
    Eigen::Vector3f P3D2c;                             // KF2 坐标系下的点

    if (pMP1 && pMP2) {
      if (!pMP1->isBad() && !pMP2->isBad()) {
        // 添加 KF1 点顶点
        g2o::VertexPointXYZ* vPoint1 = new g2o::VertexPointXYZ();
        Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
        P3D1c = R1w * P3D1w + t1w;                     // 世界→KF1
        vPoint1->setEstimate(P3D1c.cast<double>());
        vPoint1->setId(id1);
        vPoint1->setFixed(true);                       // 固定
        optimizer.addVertex(vPoint1);

        // 添加 KF2 点顶点
        g2o::VertexPointXYZ* vPoint2 = new g2o::VertexPointXYZ();
        Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
        P3D2c = R2w * P3D2w + t2w;                     // 世界→KF2
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);
      } else {
        nBadMPs++;
        continue;
      }
    } else {
      nMatchWithoutMP++;

      // 只有 pMP2 存在
      if (!pMP2->isBad()) {
        g2o::VertexPointXYZ* vPoint2 = new g2o::VertexPointXYZ();
        Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
        P3D2c = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);

        vIdsOnlyInKF2.push_back(id2);
      }
      continue;
    }

    // 若 pMP2 在 KF2 中无观测且不使用所有点，跳过
    if (i2 < 0 && !bAllPoints) {
      Verbose::PrintMess("    Remove point -> i2: " + to_string(i2) +
                             "; bAllPoints: " + to_string(bAllPoints),
                         Verbose::VERBOSITY_DEBUG);
      continue;
    }

    // 深度必须为正
    if (P3D2c(2) < 0) {
      Verbose::PrintMess("Sim3: Z coordinate is negative",
                         Verbose::VERBOSITY_DEBUG);
      continue;
    }

    nCorrespondences++;

    // ---- 4.1 添加正向边：x1 = S12 * X2 ----
    Eigen::Matrix<double, 2, 1> obs1;
    const cv::KeyPoint& kpUn1 = pKF1->mvKeysUn[i];
    obs1 << kpUn1.pt.x, kpUn1.pt.y;

    ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();

    e12->setVertex(
        0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
    e12->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e12->setMeasurement(obs1);
    const float& invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
    e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

    g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
    e12->setRobustKernel(rk1);
    rk1->setDelta(deltaHuber);
    optimizer.addEdge(e12);

    // ---- 4.2 添加反向边：x2 = S21 * X1 ----
    Eigen::Matrix<double, 2, 1> obs2;
    cv::KeyPoint kpUn2;
    bool inKF2;
    if (i2 >= 0) {
      // pMP2 在 KF2 中有观测
      kpUn2 = pKF2->mvKeysUn[i2];
      obs2 << kpUn2.pt.x, kpUn2.pt.y;
      inKF2 = true;
      nInKF2++;
    } else {
      // 用 pMP2 投影坐标作为观测
      float invz = 1 / P3D2c(2);
      float x = P3D2c(0) * invz;
      float y = P3D2c(1) * invz;

      obs2 << x, y;
      kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

      inKF2 = false;
      nOutKF2++;
    }

    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 =
        new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();

    e21->setVertex(
        0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
    e21->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e21->setMeasurement(obs2);
    float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
    e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

    g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
    e21->setRobustKernel(rk2);
    rk2->setDelta(deltaHuber);
    optimizer.addEdge(e21);

    vpEdges12.push_back(e12);
    vpEdges21.push_back(e21);
    vnIndexEdge.push_back(i);

    vbIsInKF2.push_back(inKF2);
  }

  // ---- 5. 第一次优化 ----
  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // ---- 6. 检查内点，剔除外点 ----
  int nBad = 0;                                        // 外点数
  int nBadOutKF2 = 0;                                  // 仅在 KF1 中的外点数
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
    if (!e12 || !e21) continue;

    // 双向卡方任一超阈值即剔除
    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint*>(NULL); // 标记为无效匹配
      optimizer.removeEdge(e12);
      optimizer.removeEdge(e21);
      vpEdges12[i] = static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ*>(NULL);
      vpEdges21[i] = static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*>(NULL);
      nBad++;

      if (!vbIsInKF2[i]) {
        nBadOutKF2++;
      }
      continue;
    }

    // 去掉鲁棒核，用纯二次误差再优化
    e12->setRobustKernel(0);
    e21->setRobustKernel(0);
  }

  // ---- 7. 第二次优化 ----
  int nMoreIterations;
  if (nBad > 0)
    nMoreIterations = 10;
  else
    nMoreIterations = 5;

  // 内点太少，放弃
  if (nCorrespondences - nBad < 10) return 0;

  optimizer.initializeOptimization();
  optimizer.optimize(nMoreIterations);

  // ---- 8. 统计最终内点 ----
  int nIn = 0;
  mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
    if (!e12 || !e21) continue;

    e12->computeError();
    e21->computeError();

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint*>(NULL);  // 剔除
    } else {
      nIn++;
    }
  }

  // ---- 9. 恢复优化后的 Sim3 ----
  g2o::VertexSim3Expmap* vSim3_recov =
      static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
  g2oS12 = vSim3_recov->estimate();

  return nIn;
}

// ============================================================================
//  七、惯性初始化优化（三种重载）
// ============================================================================

/**
 * 惯性优化（重载 1）：同时估计重力方向、尺度、零偏、速度
 *
 * 使用场景：
 *   - 单目惯性初始化阶段，视觉 SLAM 与 IMU 尚未对齐
 *   - 需要同时求解：重力方向 Rwg、尺度 scale、陀螺零偏 bg、加速度零偏 ba
 *
 * 顶点：
 *   - 位姿 VertexPose（固定）
 *   - 速度 VertexVelocity（可优化）
 *   - 零偏 VertexGyroBias/VertexAccBias（共享，可优化）
 *   - 重力方向 VertexGDir、尺度 VertexScale（可优化）
 * 边：
 *   - EdgeInertialGS：带重力与尺度的惯性残差
 *   - EdgePriorAcc/EdgePriorGyro：零偏先验
 *
 * @param bMono        是否单目（单目时尺度可优化）
 * @param bFixedVel    是否固定速度
 * @param bGauss       是否使用高斯牛顿
 * @param priorG/priorA 零偏先验权重
 */
void Optimizer::InertialOptimization(const std::shared_ptr<Map>& pMap,
                                     Eigen::Matrix3d& Rwg, double& scale,
                                     Eigen::Vector3d& bg, Eigen::Vector3d& ba,
                                     bool bMono, Eigen::MatrixXd& covInertial,
                                     bool bFixedVel, bool bGauss, float priorG,
                                     float priorA) {
  Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
  int its = 200;                                       // 迭代次数
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const auto vpKFs = pMap->GetAllKeyFrames();

  // ---- 1. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  if (priorG != 0.f) solver->setUserLambdaInit(1e3);   // 有先验时增大阻尼

  optimizer.setAlgorithm(solver);

  // ---- 2. 添加关键帧顶点（位姿固定，速度可优化）----
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);                                // 位姿固定
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + (pKFi->mnId) + 1);             // id 偏移
    if (bFixedVel)
      VV->setFixed(true);
    else
      VV->setFixed(false);

    optimizer.addVertex(VV);
  }

  // ---- 3. 添加共享零偏顶点 ----
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
  VG->setId(maxKFid * 2 + 2);
  if (bFixedVel)
    VG->setFixed(true);
  else
    VG->setFixed(false);
  optimizer.addVertex(VG);

  VertexAccBias* VA = new VertexAccBias(vpKFs.front());
  VA->setId(maxKFid * 2 + 3);
  if (bFixedVel)
    VA->setFixed(true);
  else
    VA->setFixed(false);
  optimizer.addVertex(VA);

  // ---- 4. 零偏先验（零均值）----
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorAcc* epa = new EdgePriorAcc(bprior);
  epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
  epa->setInformation(priorA * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epa);

  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  epg->setInformation(priorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  // ---- 5. 添加重力方向与尺度顶点 ----
  VertexGDir* VGDir = new VertexGDir(Rwg);
  VGDir->setId(maxKFid * 2 + 4);
  VGDir->setFixed(false);
  optimizer.addVertex(VGDir);

  VertexScale* VS = new VertexScale(scale);
  VS->setId(maxKFid * 2 + 5);
  VS->setFixed(!bMono);                                // 双目/RGBD 固定尺度
  optimizer.addVertex(VS);

  // ---- 6. 添加带重力与尺度的惯性边 ----
  vector<EdgeInertialGS*> vpei;
  vpei.reserve(vpKFs.size());
  vector<pair<shared_ptr<KeyFrame>, shared_ptr<KeyFrame>>> vppUsedKF;
  vppUsedKF.reserve(vpKFs.size());

  for (auto pKFi : vpKFs) {
    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
      if (!pKFi->mpImuPreintegrated)
        std::cout << "Not preintegrated measurement" << std::endl;

      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());

      // 取所有相关顶点
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid * 2 + 2);
      g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid * 2 + 3);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid * 2 + 5);

      // 检查顶点完整性
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA
             << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS
             << endl;
        continue;
      }

      // 带重力与尺度的惯性边
      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      vpei.push_back(ei);
      vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
      optimizer.addEdge(ei);
    }
  }

  // ---- 7. 执行优化 ----
  std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  scale = VS->estimate();                              // 恢复尺度

  // ---- 8. 恢复零偏、重力方向 ----
  VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
  VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
  Vector6d vb;
  vb << VG->estimate(), VA->estimate();
  bg << VG->estimate();
  ba << VA->estimate();
  scale = VS->estimate();

  IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
  Rwg = VGDir->estimate().Rwg;

  // ---- 9. 恢复各 KF 速度与零偏 ----
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;

    VertexVelocity* VV = static_cast<VertexVelocity*>(
        optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
    Eigen::Vector3d Vw = VV->estimate();
    pKFi->SetVelocity(Vw.cast<float>());

    // 零偏变化大则重新预积分
    if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
      pKFi->SetNewBias(b);
      if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
    } else {
      pKFi->SetNewBias(b);
    }
  }
}

/**
 * 惯性优化（重载 2）：仅估计零偏与速度（重力方向与尺度固定）
 *
 * 使用场景：
 *   - 已初始化完成，重力方向和尺度已知，只优化零偏
 */
void Optimizer::InertialOptimization(const std::shared_ptr<Map>& pMap,
                                     Eigen::Vector3d& bg, Eigen::Vector3d& ba,
                                     float priorG, float priorA) {
  int its = 200;
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const vector<shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();

  // ---- 1. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  solver->setUserLambdaInit(1e3);
  optimizer.setAlgorithm(solver);

  // ---- 2. 添加 KF 顶点（位姿固定，速度优化）----
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + (pKFi->mnId) + 1);
    VV->setFixed(false);

    optimizer.addVertex(VV);
  }

  // ---- 3. 共享零偏顶点 ----
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
  VG->setId(maxKFid * 2 + 2);
  VG->setFixed(false);
  optimizer.addVertex(VG);

  VertexAccBias* VA = new VertexAccBias(vpKFs.front());
  VA->setId(maxKFid * 2 + 3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // ---- 4. 零偏先验 ----
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorAcc* epa = new EdgePriorAcc(bprior);
  epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
  epa->setInformation(priorA * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epa);

  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  epg->setInformation(priorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  // ---- 5. 重力方向与尺度（固定）----
  VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
  VGDir->setId(maxKFid * 2 + 4);
  VGDir->setFixed(true);
  optimizer.addVertex(VGDir);

  VertexScale* VS = new VertexScale(1.0);
  VS->setId(maxKFid * 2 + 5);
  VS->setFixed(true);
  optimizer.addVertex(VS);

  // ---- 6. 添加惯性边 ----
  vector<EdgeInertialGS*> vpei;
  vpei.reserve(vpKFs.size());
  vector<pair<std::shared_ptr<KeyFrame>, std::shared_ptr<KeyFrame>>> vppUsedKF;
  vppUsedKF.reserve(vpKFs.size());

  for (auto pKFi : vpKFs) {
    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;

      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());

      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid * 2 + 2);
      g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid * 2 + 3);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid * 2 + 5);

      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA
             << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS
             << endl;
        continue;
      }

      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      vpei.push_back(ei);
      vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
      optimizer.addEdge(ei);
    }
  }

  // ---- 7. 执行优化 ----
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // ---- 8. 恢复零偏 ----
  VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
  VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
  Vector6d vb;
  vb << VG->estimate(), VA->estimate();
  bg << VG->estimate();
  ba << VA->estimate();

  IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

  // ---- 9. 恢复速度与零偏 ----
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;

    VertexVelocity* VV = static_cast<VertexVelocity*>(
        optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
    Eigen::Vector3d Vw = VV->estimate();
    pKFi->SetVelocity(Vw.cast<float>());

    if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
      pKFi->SetNewBias(b);
      if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
    } else {
      pKFi->SetNewBias(b);
    }
  }
}

/**
 * 惯性优化（重载 3）：仅估计重力方向与尺度（速度、零偏固定）
 *
 * 使用场景：
 *   - 地图已初始化完成，只需精化重力与尺度
 */
void Optimizer::InertialOptimization(const std::shared_ptr<Map>& pMap,
                                     Eigen::Matrix3d& Rwg, double& scale) {
  int its = 10;
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const vector<shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();

  // ---- 1. 构建优化器（高斯牛顿）----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  // ---- 2. 所有 KF 顶点固定 ----
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + 1 + (pKFi->mnId));
    VV->setFixed(true);
    optimizer.addVertex(VV);

    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
    VG->setFixed(true);
    optimizer.addVertex(VG);

    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
    VA->setFixed(true);
    optimizer.addVertex(VA);
  }

  // ---- 3. 重力方向与尺度（可优化）----
  VertexGDir* VGDir = new VertexGDir(Rwg);
  VGDir->setId(4 * (maxKFid + 1));
  VGDir->setFixed(false);
  optimizer.addVertex(VGDir);

  VertexScale* VS = new VertexScale(scale);
  VS->setId(4 * (maxKFid + 1) + 1);
  VS->setFixed(false);
  optimizer.addVertex(VS);

  // ---- 4. 添加惯性边 ----
  int count_edges = 0;
  for (auto pKFi : vpKFs) {
    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;

      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex((maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex((maxKFid + 1) + pKFi->mnId);
      g2o::HyperGraph::Vertex* VG =
          optimizer.vertex(2 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VA =
          optimizer.vertex(3 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(4 * (maxKFid + 1));
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(4 * (maxKFid + 1) + 1);

      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        Verbose::PrintMess(
            "Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) +
                ", " + to_string(VG->id()) + ", " + to_string(VA->id()) + ", " +
                to_string(VP2->id()) + ", " + to_string(VV2->id()) + ", " +
                to_string(VGDir->id()) + ", " + to_string(VS->id()),
            Verbose::VERBOSITY_NORMAL);

        continue;
      }
      count_edges++;

      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      ei->setRobustKernel(rk);
      rk->setDelta(1.f);
      optimizer.addEdge(ei);
    }
  }

  // ---- 5. 执行优化 ----
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.activeRobustChi2();                        // 初始误差
  optimizer.optimize(its);
  optimizer.computeActiveErrors();
  optimizer.activeRobustChi2();                        // 最终误差

  // ---- 6. 恢复重力方向与尺度 ----
  scale = VS->estimate();
  Rwg = VGDir->estimate().Rwg;
}

// ============================================================================
//  八、局部惯性 BA
// ============================================================================

/**
 * 局部惯性 BA
 *
 * 使用场景：
 *   - 跟踪线程中，局部窗口内的 KF 位姿、速度、零偏一起优化
 *   - 局部窗口 = 当前 KF 向前 Nd 个 KF，窗口外固定
 *   - 额外加入少量共视 KF 作为视觉约束
 *
 * @param bLarge    是否大窗口（更多迭代）
 * @param bRecInit  是否为重新初始化
 */
void Optimizer::LocalInertialBA(const shared_ptr<KeyFrame>& pKF,
                                bool* pbStopFlag,
                                const std::shared_ptr<Map>& pMap,
                                int& num_fixedKF, int& num_OptKF, int& num_MPs,
                                int& num_edges, bool bLarge, bool bRecInit) {
  std::shared_ptr<Map> pCurrentMap = pKF->GetMap();

  // 窗口大小与迭代次数
  int maxOpt = 10;
  int opt_it = 10;
  if (bLarge) {
    maxOpt = 25;
    opt_it = 4;
  }
  const int Nd =
      std::min(static_cast<int>(pCurrentMap->KeyFramesInMap()) - 2, maxOpt);
  const unsigned long maxKFid = pKF->mnId;

  // ---- 1. 收集可优化 KF（时间窗口内）----
  vector<shared_ptr<KeyFrame>> vpOptimizableKFs;
  const auto vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
  list<shared_ptr<KeyFrame>> lpOptVisKFs;

  vpOptimizableKFs.reserve(Nd);
  vpOptimizableKFs.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
    } else {
      break;
    }
  }

  // ---- 2. 收集局部地图点 ----
  list<MapPoint*> lLocalMapPoints;
  for (auto const& pKF : vpOptimizableKFs) {
    vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for (auto pMP : vpMPs) {
      if (pMP && !pMP->isBad()) {
        if (pMP->mnBALocalForKF != pKF->mnId) {
          lLocalMapPoints.push_back(pMP);
          pMP->mnBALocalForKF = pKF->mnId;
        }
      }
    }
  }

  // ---- 3. 固定 KF（窗口前一个）----
  list<shared_ptr<KeyFrame>> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
  } else {
    // 无前序 KF：把最后一个窗口 KF 固定
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // ---- 4. 共视可优化 KF（视觉约束）----
  const int maxCovKF = 0;                              // 默认不使用
  for (auto const& pKFi : vpNeighsKFs) {
    if (lpOptVisKFs.size() >= maxCovKF) break;

    if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
      continue;
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
      lpOptVisKFs.push_back(pKFi);

      vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
      for (auto pMP : vpMPs) {
        if (pMP && !pMP->isBad()) {
          if (pMP->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
        }
      }
    }
  }

  // ---- 5. 固定 KF（不共视但能看到局部点的）----
  const int maxFixKF = 200;

  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    map<shared_ptr<KeyFrame>, tuple<int, int>> observations =
        (*lit)->GetObservations();
    for (map<shared_ptr<KeyFrame>, tuple<int, int>>::iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      shared_ptr<KeyFrame> pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad()) {
          lFixedKeyFrames.push_back(pKFi);
          break;
        }
      }
    }
    if (lFixedKeyFrames.size() >= maxFixKF) break;
  }

  // ---- 6. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  if (bLarge) {
    solver->setUserLambdaInit(1e-2);                   // 大窗口用较小阻尼
  } else {
    solver->setUserLambdaInit(1e0);
  }
  optimizer.setAlgorithm(solver);

  // ---- 7. 添加可优化 KF 顶点 ----
  for (auto pKFi : vpOptimizableKFs) {
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    // 惯性 KF：添加速度、零偏
    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // ---- 8. 添加共视可优化 KF 顶点 ----
  for (auto pKFi : lpOptVisKFs) {
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);
  }

  // ---- 9. 添加固定 KF 顶点 ----
  for (auto pKFi : lFixedKeyFrames) {
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    // 固定 KF 的惯性变量也固定
    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // ---- 10. 添加惯性边 ----
  int N = vpOptimizableKFs.size();

  vector<EdgeInertial*> vei(N, nullptr);
  vector<EdgeGyroRW*> vegr(N, nullptr);
  vector<EdgeAccRW*> vear(N, nullptr);

  for (int i = 0; i < N; i++) {
    auto pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());

      // 取顶点指针
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
             << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
             << endl;
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      // 窗口最后一条边降权 + 加鲁棒核
      if (i == N - 1 || bRecInit) {
        g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
        vei[i]->setRobustKernel(rki);
        if (i == N - 1) vei[i]->setInformation(vei[i]->information() * 1e-2);
        rki->setDelta(sqrt(16.92));
      }
      optimizer.addEdge(vei[i]);

      // 陀螺随机游走
      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                  .cast<double>()
                                  .inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      // 加速度随机游走
      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                  .cast<double>()
                                  .inverse();
      vear[i]->setInformation(InfoA);

      optimizer.addEdge(vear[i]);
    } else {
      cout << "ERROR building inertial edge" << endl;
    }
  }

  // ---- 11. 添加地图点顶点及视觉边 ----
  const int nExpectedSize =
      (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  // 单目边
  vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);
  vector<shared_ptr<KeyFrame>> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);
  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  // 双目边
  vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);
  vector<shared_ptr<KeyFrame>> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);
  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float chi2Mono2 = 5.991;
  const float thHuberStereo = sqrt(7.815);
  const float chi2Stereo2 = 7.815;

  const unsigned long iniMPid = maxKFid * 5;

  // 统计每个 KF 的视觉边数
  map<int, int> mVisEdges;
  for (auto pKFi : vpOptimizableKFs) {
    mVisEdges[pKFi->mnId] = 0;
  }
  for (auto lit : lFixedKeyFrames) {
    mVisEdges[lit->mnId] = 0;
  }

  // ---- 遍历局部地图点 ----
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    auto const observations = pMP->GetObservations();

    // ---- 遍历观测，添加视觉边 ----
    for (auto const& [pKFi, vObs] : observations) {
      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId)
        continue;

      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = get<0>(vObs);

        cv::KeyPoint kpUn;

        // 单目左目
        if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
          mVisEdges[pKFi->mnId]++;

          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);

          // 考虑像素不确定性
          const float unc2 = pKFi->mpCamera->uncertainty2(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        }
        // 双目
        else if (leftIndex != -1) {
          kpUn = pKFi->mvKeysUn[leftIndex];
          mVisEdges[pKFi->mnId]++;

          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);

          const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }

        // 鱼眼右目
        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(vObs);

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;
            mVisEdges[pKFi->mnId]++;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            EdgeMono* e = new EdgeMono(1);

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);

            const float unc2 = pKFi->mpCamera->uncertainty2(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
          }
        }
      }
    }
  }

  // ---- 12. 执行优化 ----
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  float err = optimizer.activeRobustChi2();            // 优化前误差
  optimizer.optimize(opt_it);
  float err_end = optimizer.activeRobustChi2();        // 优化后误差
  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  // ---- 13. 收集外点 ----
  vector<pair<shared_ptr<KeyFrame>, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // 单目外点
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMono* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];
    bool bClose = pMP->mTrackDepth < 10.f;             // 近点放宽阈值

    if (pMP->isBad()) continue;

    if ((e->chi2() > chi2Mono2 && !bClose) ||
        (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
      shared_ptr<KeyFrame> pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // 双目外点
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Stereo2) {
      shared_ptr<KeyFrame> pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // ---- 14. 剔除外点 ----
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // 误差未下降，BA 失败
  if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) {
    oslog::error("FAIL LOCAL-INERTIAL BA!!!!");
    return;
  }

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      shared_ptr<KeyFrame> pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  for (auto pKFi : lFixedKeyFrames) pKFi->mnBAFixedForKF = 0;

  // ---- 15. 恢复优化结果 ----
  N = vpOptimizableKFs.size();
  for (auto pKFi : vpOptimizableKFs) {
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // 共视可优化 KF
  for (auto pKFi : lpOptVisKFs) {
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;
  }

  // 恢复地图点
  for (auto pMP : lLocalMapPoints) {
    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

// ============================================================================
//  九、合并惯性 BA（地图合并时）
// ============================================================================

/**
 * 合并惯性 BA
 *
 * 使用场景：
 *   - 两地图合并后，对合并区域做一次惯性 BA
 *   - 优化范围：当前 KF 时间窗口 + 合并 KF 时间窗口 + 少量共视 KF
 *
 * @param pCurrKF     当前 KF
 * @param pMergeKF    合并 KF
 * @param corrPoses   输出：各 KF 校正后的 Sim3
 */
void Optimizer::MergeInertialBA(const std::shared_ptr<KeyFrame>& pCurrKF,
                                const std::shared_ptr<KeyFrame>& pMergeKF,
                                bool* pbStopFlag,
                                const std::shared_ptr<Map>& pMap,
                                LoopClosing::KeyFrameAndPose& corrPoses) {
  const int Nd = 6;                                    // 每个窗口大小
  const unsigned long maxKFid = pCurrKF->mnId;

  // ---- 1. 收集可优化 KF ----
  vector<std::shared_ptr<KeyFrame>> vpOptimizableKFs;
  vpOptimizableKFs.reserve(2 * Nd);

  // 共视可优化 KF
  const int maxCovKF = 30;
  vector<std::shared_ptr<KeyFrame>> vpOptimizableCovKFs;
  vpOptimizableCovKFs.reserve(maxCovKF);

  // 当前 KF 窗口
  vpOptimizableKFs.push_back(pCurrKF);
  pCurrKF->mnBALocalForKF = pCurrKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else {
      break;
    }
  }

  // 固定 KF（窗口前一个）
  list<std::shared_ptr<KeyFrame>> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
  } else {
    vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // 合并 KF 加入
  vpOptimizableKFs.push_back(pMergeKF);
  pMergeKF->mnBALocalForKF = pCurrKF->mnId;

  // 合并 KF 的前序 KF
  for (int i = 1; i < (Nd / 2); i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else {
      break;
    }
  }

  // 固定旧地图
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // 合并 KF 的后序 KF
  if (pMergeKF->mNextKF) {
    vpOptimizableKFs.push_back(pMergeKF->mNextKF);
    vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
  }

  while (vpOptimizableKFs.size() < (2 * Nd)) {
    if (vpOptimizableKFs.back()->mNextKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else {
      break;
    }
  }

  int N = vpOptimizableKFs.size();

  // ---- 2. 收集局部地图点 ----
  list<MapPoint*> lLocalMapPoints;
  map<MapPoint*, int> mLocalObs;
  for (int i = 0; i < N; i++) {
    vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
    for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
         vit != vend; vit++) {
      MapPoint* pMP = *vit;
      if (pMP) {
        if (!pMP->isBad()) {
          if (pMP->mnBALocalForKF != pCurrKF->mnId) {
            mLocalObs[pMP] = 1;
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pCurrKF->mnId;
          } else {
            mLocalObs[pMP]++;
          }
        }
      }
    }
  }

  // 按观测次数排序
  std::vector<std::pair<MapPoint*, int>> pairs;
  pairs.reserve(mLocalObs.size());
  for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
    pairs.push_back(*itr);
  sort(pairs.begin(), pairs.end(), sortByVal);

  // ---- 3. 收集共视可优化 KF（只取观测次数多的点对应的 KF）----
  int i = 0;
  for (vector<pair<MapPoint*, int>>::iterator lit = pairs.begin(),
                                              lend = pairs.end();
       lit != lend; lit++, i++) {
    map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations =
        lit->first->GetObservations();

    if (i >= maxCovKF) break;

    for (auto mit : observations) {
      auto pKFi = mit.first;

      if (pKFi->mnBALocalForKF != pCurrKF->mnId &&
          pKFi->mnBAFixedForKF != pCurrKF->mnId) {
        pKFi->mnBALocalForKF = pCurrKF->mnId;
        if (!pKFi->isBad()) {
          vpOptimizableCovKFs.push_back(pKFi);
          break;
        }
      }
    }
  }

  // ---- 4. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  // ---- 5. 添加可优化 KF 顶点 ----
  N = vpOptimizableKFs.size();
  for (auto pKFi : vpOptimizableKFs) {
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // ---- 6. 添加共视可优化 KF 顶点 ----
  int Ncov = vpOptimizableCovKFs.size();
  for (auto pKFi : vpOptimizableCovKFs) {
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // ---- 7. 添加固定 KF 顶点 ----
  for (auto pKFi : lFixedKeyFrames) {
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // ---- 8. 添加惯性边 ----
  vector<EdgeInertial*> vei(N, nullptr);
  vector<EdgeGyroRW*> vegr(N, nullptr);
  vector<EdgeAccRW*> vear(N, nullptr);

  for (int i = 0; i < N; i++) {
    std::shared_ptr<KeyFrame> pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      Verbose::PrintMess("NO INERTIAL LINK TO PREVIOUS FRAME!!!!",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());

      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
             << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
             << endl;
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
      vei[i]->setRobustKernel(rki);
      rki->setDelta(sqrt(16.92));
      optimizer.addEdge(vei[i]);

      // 陀螺随机游走
      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                  .cast<double>()
                                  .inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      // 加速度随机游走
      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                  .cast<double>()
                                  .inverse();
      vear[i]->setInformation(InfoA);
      optimizer.addEdge(vear[i]);
    } else {
      Verbose::PrintMess("ERROR building inertial edge",
                         Verbose::VERBOSITY_NORMAL);
    }
  }

  Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);

  // ---- 9. 添加视觉边（单目 + 双目）----
  const int nExpectedSize =
      (N + Ncov + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);
  vector<std::shared_ptr<KeyFrame>> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);
  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);
  vector<std::shared_ptr<KeyFrame>> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);
  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float chi2Mono2 = 5.991;
  const float thHuberStereo = sqrt(7.815);
  const float chi2Stereo2 = 7.815;

  const unsigned long iniMPid = maxKFid * 5;

  // ---- 遍历局部地图点 ----
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    if (!pMP) continue;

    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations =
        pMP->GetObservations();

    // ---- 遍历观测，添加视觉边 ----
    for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      std::shared_ptr<KeyFrame> pKFi = mit->first;

      if (!pKFi) continue;

      if ((pKFi->mnBALocalForKF != pCurrKF->mnId) &&
          (pKFi->mnBAFixedForKF != pCurrKF->mnId))
        continue;

      if (pKFi->mnId > maxKFid) {
        continue;
      }

      if (optimizer.vertex(id) == NULL || optimizer.vertex(pKFi->mnId) == NULL)
        continue;

      if (!pKFi->isBad()) {
        const cv::KeyPoint& kpUn = pKFi->mvKeysUn[get<0>(mit->second)];

        // 单目
        if (pKFi->mvuRight[get<0>(mit->second)] < 0) {
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);
          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        }
        // 双目
        else {
          const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }
      }
    }
  }

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);
  if (pbStopFlag)
    if (*pbStopFlag) return;

  // ---- 10. 执行优化 ----
  optimizer.initializeOptimization();
  optimizer.optimize(8);

  // ---- 11. 收集外点 ----
  vector<pair<std::shared_ptr<KeyFrame>, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMono* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Mono2) {
      std::shared_ptr<KeyFrame> pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Stereo2) {
      std::shared_ptr<KeyFrame> pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // ---- 12. 剔除外点 ----
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);
  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      std::shared_ptr<KeyFrame> pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // ---- 13. 恢复优化结果 + 输出校正 Sim3 ----
  for (auto pKFi : vpOptimizableKFs) {
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);

    // 输出校正后的 Sim3
    Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
    g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
    corrPoses[pKFi] = g2oSiw;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  for (auto pKFi : vpOptimizableCovKFs) {
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);

    Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
    g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
    corrPoses[pKFi] = g2oSiw;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // 恢复地图点
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

// ============================================================================
//  十、帧惯性位姿优化（上一关键帧先验版）
// ============================================================================

/**
 * 帧惯性位姿优化（上一关键帧先验版）
 *
 * 使用场景：
 *   - 跟踪线程中，用上一关键帧的位姿/速度/零偏作为先验
 *   - 优化当前帧位姿、速度、零偏
 *   - 边缘化上一关键帧状态，生成新的先验 ConstraintPoseImu
 */
int Optimizer::PoseInertialOptimizationLastKeyFrame(
    const std::shared_ptr<Frame>& pFrame, bool bRecInit) {
  // ---- 1. 构建优化器（高斯牛顿，稠密）----
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  auto linearSolver = std::make_unique<
      g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  int nInitialMonoCorrespondences = 0;                 // 单目匹配数
  int nInitialStereoCorrespondences = 0;               // 双目匹配数
  int nInitialCorrespondences = 0;                     // 总匹配数

  // ---- 2. 添加当前帧顶点 ----
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);

  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);

  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);

  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // ---- 3. 添加观测边 ----
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;               // 单目边
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;           // 双目边
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;

        // ---- 3.1 左目单目观测 ----
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)                               // 双目左目
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // 考虑像素不确定性
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // ---- 3.2 双目观测 ----
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));
          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // ---- 3.3 右目单目观测 ----
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          const float unc2 = pFrame->mpCamera->uncertainty2(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }
  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // ---- 4. 添加上一关键帧顶点（固定）----
  auto pKF = pFrame->mpLastKeyFrame;
  VertexPose* VPk = new VertexPose(pKF);
  VPk->setId(4);
  VPk->setFixed(true);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pKF);
  VVk->setId(5);
  VVk->setFixed(true);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pKF);
  VGk->setId(6);
  VGk->setFixed(true);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pKF);
  VAk->setId(7);
  VAk->setFixed(true);
  optimizer.addVertex(VAk);

  // ---- 5. 添加惯性边（上一 KF → 当前帧）----
  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  // 陀螺随机游走
  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  // 加速度随机游走
  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  // ---- 6. 4 轮优化 + 外点剔除 ----
  float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
  float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};
  int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];

    // 单目外点判断
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    // 双目外点判断
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // ---- 7. 内点太少则尝试恢复部分外点 ----
  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  // ---- 8. 恢复优化后的位姿、速度、零偏 ----
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // ---- 9. 计算 Hessian，边缘化上一 KF 状态，生成先验 ----
  Eigen::Matrix<double, 15, 15> H;
  H.setZero();

  H.block<9, 9>(0, 0) += ei->GetHessian2();            // 惯性边 Hessian
  H.block<3, 3>(9, 9) += egr->GetHessian2();           // 陀螺随机游走
  H.block<3, 3>(12, 12) += ear->GetHessian2();         // 加速度随机游走

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else {
      tot_out++;
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else {
      tot_out++;
    }
  }

  // 生成先验约束
  pFrame->mpcpi =
      new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb,
                            VV->estimate(), VG->estimate(), VA->estimate(), H);

  return nInitialCorrespondences - nBad;
}

// ============================================================================
//  十一、帧惯性位姿优化（上一帧先验版）
// ============================================================================

/**
 * 帧惯性位姿优化（上一帧先验版）
 *
 * 使用场景：
 *   - 跟踪线程中，用上一帧的位姿/速度/零偏作为先验
 *   - 优化当前帧位姿、速度、零偏
 *   - 边缘化上一帧状态，生成当前帧的先验
 */
int Optimizer::PoseInertialOptimizationLastFrame(
    const std::shared_ptr<Frame>& pFrame, bool bRecInit) {
  // ---- 1. 构建优化器（高斯牛顿）----
  g2o::SparseOptimizer optimizer;

  auto linearSolver = std::make_unique<
      g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // ---- 2. 添加当前帧顶点 ----
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);

  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);

  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);

  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // ---- 3. 添加观测边（与上一函数相同）----
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // 左目单目
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          const float unc2 = pFrame->mpCamera->uncertainty2(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // 双目
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));
          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // 右目单目
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          const float unc2 = pFrame->mpCamera->uncertainty2(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }

  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // ---- 4. 添加上一帧顶点 ----
  std::shared_ptr<Frame> pFp = pFrame->mpPrevFrame;

  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(4);
  VPk->setFixed(false);                                // 上一帧也优化
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(false);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pFp);
  VGk->setId(6);
  VGk->setFixed(false);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pFp);
  VAk->setId(7);
  VAk->setFixed(false);
  optimizer.addVertex(VAk);

  // ---- 5. 添加帧间惯性边 ----
  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  // 陀螺随机游走
  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  // 加速度随机游走
  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  if (!pFp->mpcpi)
    Verbose::PrintMess(
        "pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
        Verbose::VERBOSITY_NORMAL);

  // ---- 6. 添加上一帧先验边 ----
  EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // ---- 7. 4 轮优化 + 外点剔除 ----
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];

    // 单目外点
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    // 双目外点
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // ---- 8. 内点少则恢复部分外点 ----
  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  nInliers = nInliersMono + nInliersStereo;

  // ---- 9. 恢复优化后的位姿、速度、零偏 ----
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // ---- 10. 计算 Hessian，边缘化上一帧状态 ----
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();

  H.block<24, 24>(0, 0) += ei->GetHessian();

  // 陀螺随机游走 Hessian 累加
  Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
  H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

  // 加速度随机游走 Hessian 累加
  Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
  H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

  // 上一帧先验
  H.block<15, 15>(0, 0) += ep->GetHessian();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else {
      tot_out++;
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else {
      tot_out++;
    }
  }

  // 边缘化上一帧状态（前 15 维）
  H = Marginalize(H, 0, 14);

  // 生成当前帧先验
  pFrame->mpcpi = new ConstraintPoseImu(
      VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
      VA->estimate(), H.block<15, 15>(15, 15));
  delete pFp->mpcpi;                                   // 释放上一帧先验
  pFp->mpcpi = NULL;

  return nInitialCorrespondences - nBad;
}

// ============================================================================
//  十二、4DoF 本质图优化（地图合并时，仅优化 x/y/z/yaw）
// ============================================================================

/**
 * 4DoF 本质图优化
 *
 * 使用场景：
 *   - 地图合并时，两地图之间的相对变换只需要 4 自由度（x/y/z/yaw）
 *   - 减少优化变量，提高鲁棒性
 *
 * 顶点：
 *   - VertexPose4DoF：4 自由度位姿
 * 边：
 *   - Edge4DoF：4 自由度相对位姿约束
 */
void Optimizer::OptimizeEssentialGraph4DoF(
    const std::shared_ptr<Map>& pMap, const std::shared_ptr<KeyFrame>& pLoopKF,
    const std::shared_ptr<KeyFrame>& pCurKF,
    const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose& CorrectedSim3,
    const map<std::shared_ptr<KeyFrame>, set<std::shared_ptr<KeyFrame>>>&
        LoopConnections) {
  // ---- 1. 构建优化器 ----
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  const vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);

  vector<VertexPose4DoF*> vpVertices(nMaxKFid + 1);    // 4DoF 顶点

  const int minFeat = 100;

  // ---- 2. 添加 4DoF 顶点 ----
  for (auto pKF : vpKFs) {
    if (pKF->isBad()) continue;

    VertexPose4DoF* V4DoF;

    const int nIDi = pKF->mnId;

    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      const g2o::Sim3 Swc = it->second.inverse();
      Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
      Eigen::Vector3d twc = Swc.translation();
      V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
    } else {
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

      vScw[nIDi] = Siw;
      V4DoF = new VertexPose4DoF(pKF);
    }

    // 回环 KF 固定
    if (pKF == pLoopKF) V4DoF->setFixed(true);

    V4DoF->setId(nIDi);
    V4DoF->setMarginalized(false);

    optimizer.addVertex(V4DoF);
    vpVertices[nIDi] = V4DoF;
  }
  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // 4DoF 边信息矩阵：roll/pitch 权重大（不可观测）
  Eigen::Matrix<double, 6, 6> matLambda =
      Eigen::Matrix<double, 6, 6>::Identity();
  matLambda(0, 0) = 1e3;
  matLambda(1, 1) = 1e3;
  matLambda(0, 0) = 1e3;

  // ---- 3. 添加回环边 ----
  for (auto const& [pKF, spConnections] : LoopConnections) {
    const long unsigned int nIDi = pKF->mnId;
    const g2o::Sim3 Siw = vScw[nIDi];

    for (auto pConnection : spConnections) {
      const long unsigned int nIDj = pConnection->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(pConnection) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sij = Siw * Sjw.inverse();
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));

      e->information() = matLambda;
      optimizer.addEdge(e);

      sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
    }
  }

  // ---- 4. 添加普通边 ----
  for (auto const& pKF : vpKFs) {
    const int nIDi = pKF->mnId;

    g2o::Sim3 Siw;

    LoopClosing::KeyFrameAndPose::const_iterator iti =
        NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Siw = iti->second;
    else
      Siw = vScw[nIDi];

    // ---- 4.1 生成树边 ----
    std::shared_ptr<KeyFrame> pParentKF;
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Swj;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Swj = (itj->second).inverse();
      else
        Swj = vScw[nIDj].inverse();

      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // ---- 4.2 惯性边 ----
    auto const& prevKF = pKF->mPrevKF;
    if (prevKF) {
      int nIDj = prevKF->mnId;

      g2o::Sim3 Swj;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(prevKF);

      if (itj != NonCorrectedSim3.end())
        Swj = (itj->second).inverse();
      else
        Swj = vScw[nIDj].inverse();

      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // ---- 4.3 回环边 ----
    auto const& sLoopEdges = pKF->GetLoopEdges();
    for (auto pLKF : sLoopEdges) {
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Swl;

        LoopClosing::KeyFrameAndPose::const_iterator itl =
            NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Swl = itl->second.inverse();
        else
          Swl = vScw[pLKF->mnId].inverse();

        g2o::Sim3 Sil = Siw * Swl;
        Eigen::Matrix4d Til;
        Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
        Til.block<3, 1>(0, 3) = Sil.translation();
        Til(3, 3) = 1.;

        Edge4DoF* e = new Edge4DoF(Til);
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDi)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pLKF->mnId)));
        e->information() = matLambda;
        optimizer.addEdge(e);
      }
    }

    // ---- 4.4 共视边 ----
    const vector<std::shared_ptr<KeyFrame>> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);

    for (auto const& pKFn : vpConnectedKFs) {
      if (pKFn && pKFn != pParentKF && pKFn != prevKF && pKFn != pKF->mNextKF &&
          !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId),
                                             max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Swn;

          LoopClosing::KeyFrameAndPose::const_iterator itn =
              NonCorrectedSim3.find(pKFn);

          if (itn != NonCorrectedSim3.end())
            Swn = itn->second.inverse();
          else
            Swn = vScw[pKFn->mnId].inverse();

          g2o::Sim3 Sin = Siw * Swn;
          Eigen::Matrix4d Tin;
          Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
          Tin.block<3, 1>(0, 3) = Sin.translation();
          Tin(3, 3) = 1.;
          Edge4DoF* e = new Edge4DoF(Tin);
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(nIDi)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFn->mnId)));
          e->information() = matLambda;
          optimizer.addEdge(e);
        }
      }
    }
  }

  // ---- 5. 执行优化 ----
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(20);

  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // ---- 6. 恢复 KF 位姿 ----
  for (auto pKFi : vpKFs) {
    const int nIDi = pKFi->mnId;

    VertexPose4DoF* Vi = static_cast<VertexPose4DoF*>(optimizer.vertex(nIDi));
    Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
    Eigen::Vector3d ti = Vi->estimate().tcw[0];

    g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
    pKFi->SetPose(Tiw.cast<float>());
  }

  // ---- 7. 恢复地图点位置 ----
  for (auto const& pMP : vpMPs) {
    if (pMP->isBad()) continue;

    std::shared_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
    const int nIDr = pRefKF->mnId;

    g2o::Sim3 Srw = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    pMP->UpdateNormalAndDepth();
  }
  pMap->IncreaseChangeIndex();
}

// ============================================================================
//  十三、边缘化工具（Schur 补）
// ============================================================================

/**
 * 边缘化工具 Marginalize
 *
 * 数学原理（Schur 补）：
 *   原始 Hessian：
 *     | a  | ab | ac |
 *     | ba | b  | bc |
 *     | ca | cb | c  |
 *   边缘化 b 后的 Hessian：
 *     | a*  | 0 | ac* |
 *     | 0   | 0 | 0   |
 *     | ca* | 0 | c*  |
 *   其中 a* = a - ab * b^-1 * ba
 *
 * 用途：
 *   - 惯性优化中，把上一帧状态从 Hessian 中消去，生成当前帧的先验
 *
 * @param H      输入 Hessian
 * @param start  待边缘化的块起始索引
 * @param end    待边缘化的块结束索引
 * @return 边缘化后的 Hessian
 */
Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd& H,
                                       const int& start, const int& end) {
  // 目标：
  // a  | ab | ac       a*  | 0 | ac*
  // ba | b  | bc  -->  0   | 0 | 0
  // ca | cb | c        ca* | 0 | c*

  const int a = start;                                 // 前块大小
  const int b = end - start + 1;                       // 待边缘化块大小
  const int c = H.cols() - (end + 1);                  // 后块大小

  // 重排：
  // a  | ab | ac       a  | ac | ab
  // ba | b  | bc  -->  ca | c  | cb
  // ca | cb | c        ba | bc | b

  Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
    Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
    Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
  }
  if (a > 0 && c > 0) {
    Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
    Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
  }
  if (c > 0) {
    Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
    Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
    Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
  }
  Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

  // ---- Schur 补 ----
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv =
      svd.singularValues();
  for (int i = 0; i < b; ++i) {
    if (singularValues_inv(i) > 1e-6)
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    else
      singularValues_inv(i) = 0;                       // 奇异值截断
  }
  Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() *
                          svd.matrixU().transpose();
  Hn.block(0, 0, a + c, a + c) =
      Hn.block(0, 0, a + c, a + c) -
      Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
  Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
  Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
  Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

  // 反向重排
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
    res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
    res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
  }
  if (a > 0 && c > 0) {
    res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
    res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
  }
  if (c > 0) {
    res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
    res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
    res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
  }

  res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

  return res;
}





}  // namespace ORB_SLAM3









