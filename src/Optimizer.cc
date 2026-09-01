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
 * ORB‑SLAM3. If not, see http://www.gnu.org/licenses/.
 */
#include "Optimizer.h"

// Eigen线性代数库，矩阵、向量运算
#include <Eigen/Dense>
// 支持Eigen对象放入std容器，内存对齐适配
#include <Eigen/StdVector>
#include <mutex>
#include <memory>
#include <algorithm>
#include <iostream>
#include <vector>
#include <map>
#include <unordered_set>
#include <set>
#include <list>
#include <tuple>
// Eigen矩阵指数运算模块，用于李群李代数
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Converter.h"
// g2o自定义类型，双目、IMU相关顶点边
#include "G2oTypes.h"
// ORB‑SLAM3自定义g2o顶点、边
#include "OptimizableTypes.h"
// g2o核心：图优化求解器、块求解器基类
#include "g2o/core/block_solver.h"
// g2o高斯‑牛顿优化算法
#include "g2o/core/optimization_algorithm_gauss_newton.h"
// g2o列文伯格‑马夸尔特LM优化算法，SLAM最常用
#include "g2o/core/optimization_algorithm_levenberg.h"
// g2o鲁棒核函数实现，Huber等，抑制外点
#include "g2o/core/robust_kernel_impl.h"
// g2o稀疏块矩阵，内部存储Hessian矩阵
#include "g2o/core/sparse_block_matrix.h"
// g2o稠密线性求解器，位姿优化小规模使用
#include "g2o/solvers/dense/linear_solver_dense.h"
// g2o基于Eigen的稀疏线性求解器，BA大规模图优化
#include "g2o/solvers/eigen/linear_solver_eigen.h"
// g2o标准SBA：SE3李代数exp映射顶点、重投影边
#include "g2o/types/sba/types_six_dof_expmap.h"
// g2o SE3Expmap顶点头文件，位姿顶点
#include "g2o/types/sba/vertex_se3_expmap.h"

namespace ORB_SLAM3 {

/**
 * @brief 排序比较函数，按照pair第二个int值从小到大排序
 * @param a pair<MapPoint*,int>
 * @param b pair<MapPoint*,int>
 * @return true a.second < b.second
 */
bool sortByVal(const pair<MapPoint*, int>& a, const pair<MapPoint*, int>& b)
{
    return (a.second < b.second);
}

/**
 * @brief 全局光束平差，对整个地图所有关键帧、地图点做BA
 * @param pMap 地图智能指针
 * @param nIterations 迭代次数
 * @param pbStopFlag 外部停止标志，多线程终止BA
 * @param nLoopKF 触发本次BA的回环关键帧ID，用于区分GBA是回环触发还是普通
 * @param bRobust 是否开启Huber鲁棒核，处理外点
 */
void Optimizer::GlobalBundleAdjustemnt(const std::shared_ptr<Map>& pMap, int nIterations, bool pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    // 获取地图全部关键帧
    auto vpKFs = pMap->GetAllKeyFrames();
    // 获取地图全部地图点
    auto vpMP = pMap->GetAllMapPoints();
    // 调用通用BA函数执行全局BA
    BundleAdjustment(vpKFs, vpMP, nIterations, &pbStopFlag, nLoopKF, bRobust);
}

/**
 * @brief 光束平差BA核心实现，输入指定一批关键帧与地图点，联合优化位姿+三维点
 * @param vpKFs 待优化关键帧集合
 * @param vpMP 待优化地图点集合
 * @param nIterations g2o迭代次数
 * @param pbStopFlag 外部终止标志指针，多线程
 * @param nLoopKF 回环触发关键帧ID，保存GBA临时结果mTcwGBA
 * @param bRobust 是否启用Huber鲁棒核
 */
void Optimizer::BundleAdjustment(const vector<std::shared_ptr<KeyFrame>>& vpKFs, const vector<MapPoint*>& vpMP, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    // 存储没有加入图优化的地图点，无观测的点
    std::unordered_set<MapPoint*> vbNotIncludedMP;
    // 获取地图指针，从第一个有效关键帧拿
    auto pMap = vpKFs[0]->GetMap();

    // g2o稀疏优化器，管理顶点、边、求解器
    g2o::SparseOptimizer optimizer;

    // 构建线性求解器：BlockSolver_6_3，位姿6维，路点3维；LinearSolverEigen基于Eigen稀疏求解
    auto linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    // LM算法，传入块求解器，std::move转移所有权
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
    // 设置求解器到优化器
    optimizer.setAlgorithm(solver);
    // 关闭g2o控制台打印信息
    optimizer.setVerbose(false);

    // 如果传入停止标志，设置g2o强制停止标记
    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    // 记录最大关键帧ID，用于地图点vertex id偏移，防止ID冲突
    long unsigned int maxKFid = 0;

    // 预估边总数量，预留内存，减少vector扩容开销
    const int nExpectedSize = (vpKFs.size()) * vpMP.size();

    // 单目重投影边集合
    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);
    // 右目相机刚体外参的重投影边（双目右相机，带Trl外参）
    vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    // 每条边对应的关键帧，后续BA结束恢复数据、剔除外点使用
    vector<std::shared_ptr<KeyFrame>> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);
    vector<std::shared_ptr<KeyFrame>> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    // 每条边对应的地图点
    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    // 双目重投影边（u,v,ur三维观测）
    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);
    vector<std::shared_ptr<KeyFrame>> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    // ========== 添加关键帧SE3位姿顶点 ==========
    for (auto const& pKF : vpKFs)
    {
        // 坏关键帧直接跳过，不参与优化
        if (pKF->isBad())
            continue;
        // g2o SE3顶点，使用exp映射李代数更新
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        // 获取关键帧Tcw：世界到相机变换 Sophus::SE3
        Sophus::SE3 Tcw = pKF->GetPose();
        // Sophus SE3 转换为g2o的SE3Quat(四元数+平移)，类型转换cast<double>
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
        // vertex id = 关键帧mnId
        vSE3->setId(pKF->mnId);
        // 地图初始关键帧固定，不优化，作为BA的参考坐标系
        vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
        // 将顶点加入g2o优化图
        optimizer.addVertex(vSE3);
        // 更新最大关键帧ID
        if (pKF->mnId > maxKFid)
            maxKFid = pKF->mnId;
    }

    // Huber鲁棒阈值，卡方分布，2自由度95%置信 chi2=5.99，开根号
    const float thHuber2D = sqrt(5.99);
    // 双目观测3自由度，chi2=7.815，Huber阈值
    const float thHuber3D = sqrt(7.815);

    // ========== 添加地图点顶点（三维路点） ==========
    for (auto pMP : vpMP)
    {
        // 坏地图点跳过
        if (pMP->isBad())
            continue;
        // g2o三维点顶点，存储世界坐标系下X,Y,Z
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        // 设置地图点初始世界坐标，转double
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        // 地图点vertex id偏移：maxKFid+1 + mpId，不和关键帧id冲突
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        // 地图点做边缘化，BA中先消去路点，只求解相机位姿，加速求解
        vPoint->setMarginalized(true);
        // 添加顶点到图
        optimizer.addVertex(vPoint);

        // 获取该地图点所有观测：key=关键帧，tuple(left_kpt_idx, right_kpt_idx)
        const map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations = pMP->GetObservations();
        int nEdges = 0; // 统计该地图点有效边数量

        // ========== 遍历该地图点全部观测，构建重投影边 ==========
        for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
        {
            std::shared_ptr<KeyFrame> pKF = mit->first;
            // 关键帧是坏的 / 关键帧id超出本次BA的最大KFid，跳过
            if (pKF->isBad() || pKF->mnId > maxKFid)
                continue;
            // 检查顶点是否成功加入优化器，为空说明被过滤
            if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
                continue;

            nEdges++;
            // tuple第0个元素：左图关键点索引
            const int leftIndex = get<0>(mit->second);

            // ---------------------- 单目观测：mvuRight <0代表无右目匹配点 ----------------------
            if (leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)] < 0)
            {
                // 获取去畸变关键点
                const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];
                // 观测值：图像像素u,v
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                // 构造单目重投影边，优化位姿+路点
                ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();
                // vertex0：地图点三维顶点
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                // vertex1：关键帧位姿顶点
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                // 设置像素观测
                e->setMeasurement(obs);
                // 信息矩阵：金字塔层级的逆方差，层级越高特征越模糊，信息越小
                const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                // 启用Huber鲁棒核
                if (bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }
                // 传入相机模型指针，自定义边project需要相机内参
                e->pCamera = pKF->mpCamera;
                // 将边加入优化图
                optimizer.addEdge(e);

                // 缓存边、对应KF、对应MP，BA结束后用来统计外点
                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMP);
            }
            // ---------------------- 双目观测 leftIndex有效，mvuRight>=0，存在右目匹配 ----------------------
            else if (leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0)
            {
                const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];
                Eigen::Matrix<double, 3, 1> obs;
                // 观测 (u, v, ur) ur是右图像素x坐标
                const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                // g2o内置双目重投影边
                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                // Huber鲁棒核
                if (bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }
                // 给边赋值双目相机参数 fx fy cx cy bf基线*焦距
                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;
                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMP);
            }

            // ---------------------- 双目右相机单独观测，mpCamera2存在（双相机刚体装配，Trl为左右相机外参） ----------------------
            if (pKF->mpCamera2)
            {
                // tuple第1个元素：右图像关键点索引
                int rightIndex = get<1>(mit->second);
                if (rightIndex != -1 && rightIndex < static_cast<int>(pKF->mvKeysRight.size()))
                {
                    // 右关键点容器存储做了偏移，减去NLeft得到真实下标
                    rightIndex -= pKF->NLeft;
                    Eigen::Matrix<double, 2, 1> obs;
                    cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
                    obs << kp.pt.x, kp.pt.y;

                    // 自定义边EdgeSE3ProjectXYZToBody：世界点→左相机系→Trl变换到右相机系再投影
                    ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float& invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    // 右目观测也加Huber鲁棒核
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);

                    // 左右相机之间固定外参Trl
                    Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
                    e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());
                    // 设置右相机模型
                    e->pCamera = pKF->mpCamera2;
                    optimizer.addEdge(e);

                    vpEdgesBody.push_back(e);
                    vpEdgeKFBody.push_back(pKF);
                    vpMapPointEdgeBody.push_back(pMP);
                }
            }
        } // end for observations

        // 如果这个地图点没有任何有效观测边，从图中移除该顶点，存入集合
        if (nEdges == 0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP.insert(pMP);
        }
    } // end for vpMP

    // ========== 执行图优化 ==========
    optimizer.setVerbose(false);
    // 初始化优化器，读取顶点边初始值，构建Hessian结构
    optimizer.initializeOptimization();
    // 执行nIterations次LM迭代
    optimizer.optimize(nIterations);

    oslog::debug("BA: End of the optimization");

    // ========== 从g2o提取优化结果，写回关键帧、地图点 ==========
    // 恢复关键帧位姿
    for (auto pKF : vpKFs)
    {
        if (pKF->isBad())
            continue;
        // 取出SE3Expmap顶点
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();

        // nLoopKF等于OriginKF说明这是真正生效的GBA，直接覆盖关键帧位姿
        if (nLoopKF == pMap->GetOriginKF()->mnId)
        {
            pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>()));
        }
        else
        {
            // 回环线程GBA，不直接修改原始位姿，保存到mTcwGBA临时变量，等待回环融合后再应用
            pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
            // 记录该关键帧GBA是由哪个回环KF触发
            pKF->mnBAGlobalForKF = nLoopKF;

            // 计算BA修正前后位姿平移差值，如果位移大于1米，统计内外点
            Sophus::SE3f mTwc = pKF->GetPoseInverse();
            Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
            Eigen::Vector3f vector_dist = mTcGBA_c.translation();
            const double dist = vector_dist.norm();
            if (dist > 1)
            {
                int numMonoBadPoints = 0, numMonoOptPoints = 0;
                int numStereoBadPoints = 0, numStereoOptPoints = 0;
                vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;

                // 遍历单目边，卡方检验chi2>5.991或者投影深度为负判定外点
                for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++)
                {
                    ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i2];
                    MapPoint* pMP = vpMapPointEdgeMono[i2];
                    std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i2];
                    if (pKF != pKFedge)
                        continue;
                    if (pMP->isBad())
                        continue;
                    if (e->chi2() > 5.991 || !e->isDepthPositive())
                    {
                        numMonoBadPoints++;
                    }
                    else
                    {
                        numMonoOptPoints++;
                        vpMonoMPsOpt.push_back(pMP);
                    }
                }
                // 遍历双目边，chi2>7.815或者深度负判定外点
                for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend; i2++)
                {
                    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i2];
                    MapPoint* pMP = vpMapPointEdgeStereo[i2];
                    std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i2];
                    if (pKF != pKFedge)
                        continue;
                    if (pMP->isBad())
                        continue;
                    if (e->chi2() > 7.815 || !e->isDepthPositive())
                    {
                        numStereoBadPoints++;
                    }
                    else
                    {
                        numStereoOptPoints++;
                        vpStereoMPsOpt.push_back(pMP);
                    }
                }
            }
        }
    }

    // 恢复地图点三维坐标
    for (auto pMP : vpMP)
    {
        // 没有加入优化图的点跳过
        if (vbNotIncludedMP.find(pMP) != vbNotIncludedMP.end())
            continue;
        if (pMP->isBad())
            continue;
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1));

        // 真正生效GBA直接覆盖世界坐标；回环GBA保存临时变量mPosGBA
        if (nLoopKF == pMap->GetOriginKF()->mnId)
        {
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA = vPoint->estimate().cast<float>();
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
}

/**
 * @brief 完整IMU‑视觉联合全局BA，所有关键帧：位姿、速度、陀螺仪bias、加速度bias、路点一起优化
 * @param pMap 地图指针
 * @param its 迭代次数
 * @param bFixLocal 是否固定部分局部窗口外关键帧
 * @param nLoopId 回环关键帧ID，用于保存GBA临时结果
 * @param pbStopFlag 外部停止标志
 * @param bInit 是否是IMU初始化阶段调用
 * @param priorG 陀螺仪bias先验权重
 * @param priorA 加速度bias先验权重
 * @param vSingVal SVD奇异值输出（未实际使用）
 * @param bHess 是否输出Hessian（未实际使用）
 */
void Optimizer::FullInertialBA(const std::shared_ptr<Map>& pMap, int its, const bool bFixLocal, const long unsigned int nLoopId, bool* pbStopFlag, bool bInit, float priorG, float priorA, Eigen::VectorXd* vSingVal, bool* bHess)
{
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    // 构建g2o优化器，BlockSolverX动态维度块求解器，IMU‑VIS联合BA顶点类型多，维度不固定
    g2o::SparseOptimizer optimizer;
    auto linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
    // LM初始lambda
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int nNonFixed = 0; // 统计非固定关键帧数量
    std::shared_ptr<KeyFrame> pIncKF;

    // ========= 添加关键帧位姿顶点VertexPose（IMU自定义顶点，包含Rcw、tcw） =========
    for (auto pKFi : vpKFs)
    {
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        pIncKF = pKFi;
        bool bFixed = false;
        // bFixLocal模式：部分关键帧固定，不参与优化
        if (bFixLocal)
        {
            bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) || (pKFi->mnBAFixedForKF >= (maxKFid - 1));
            if (!bFixed)
                nNonFixed++;
            VP->setFixed(bFixed);
        }
        optimizer.addVertex(VP);

        // 如果该关键帧带有IMU数据，增加速度、gyro bias、acc bias顶点
        if (pKFi->bImu)
        {
            // 速度顶点Vw，世界坐标系下速度
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(bFixed);
            optimizer.addVertex(VV);

            if (!bInit)
            {
                // 陀螺仪零偏顶点
                VertexGyroBias* VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                VG->setFixed(bFixed);
                optimizer.addVertex(VG);
                // 加速度计零偏顶点
                VertexAccBias* VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                VA->setFixed(bFixed);
                optimizer.addVertex(VA);
            }
        }
    }

    // IMU初始化阶段：所有关键帧共用同一组bias，单独添加全局bias顶点，不每个KF一套bias
    if (bInit)
    {
        VertexGyroBias* VG = new VertexGyroBias(pIncKF);
        VG->setId(4 * maxKFid + 2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pIncKF);
        VA->setId(4 * maxKFid + 3);
        VA->setFixed(false);
        optimizer.addVertex(VA);
    }

    // bFixLocal模式，可优化的关键帧少于3个，优化无意义直接返回
    if (bFixLocal)
    {
        if (nNonFixed < 3)
            return;
    }

    // ========= 添加IMU预积分边EdgeInertial，连接连续两个关键帧 =========
    for (auto pKFi : vpKFs)
    {
        // 没有上一帧，无法做预积分约束
        if (!pKFi->mPrevKF)
        {
            oslog::warn("NOT INERTIAL LINK TO PREVIOUS FRAME!");
            continue;
        }
        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid)
        {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                continue;
            // 当前帧与上一帧都有IMU
            if (pKFi->bImu && pKFi->mPrevKF->bImu)
            {
                // 根据上一帧bias更新预积分，bias变化需要重算预积分残差
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());

                // 获取各个顶点
                g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
                g2o::HyperGraph::Vertex* VG1;
                g2o::HyperGraph::Vertex* VA1;
                g2o::HyperGraph::Vertex* VG2;
                g2o::HyperGraph::Vertex* VA2;

                if (!bInit)
                {
                    // 非初始化，每个KF拥有自己bias顶点
                    VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
                    VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
                    VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
                    VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
                }
                else
                {
                    // IMU初始化阶段，共用全局bias顶点
                    VG1 = optimizer.vertex(4 * maxKFid + 2);
                    VA1 = optimizer.vertex(4 * maxKFid + 3);
                }

                g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);

                // 检查顶点有效性
                if (!bInit)
                {
                    if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
                    {
                        cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2 << endl;
                        continue;
                    }
                }
                else
                {
                    if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2)
                    {
                        cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2 << endl;
                        continue;
                    }
                }

                // EdgeInertial: 6个顶点：prev位姿、prev速度、prev gyro bias、prev acc bias、curr位姿、curr速度
                EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

                // IMU残差也加Huber鲁棒核，sqrt(16.92)对应6自由度chi2阈值
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));
                optimizer.addEdge(ei);

                // 随机游走RW边：gyro bias，bias随时间做随机游走，约束相邻帧bias变化
                if (!bInit)
                {
                    EdgeGyroRW* egr = new EdgeGyroRW();
                    egr->setVertex(0, VG1);
                    egr->setVertex(1, VG2);
                    // 从预积分协方差矩阵取出bias噪声，取逆作为信息矩阵
                    Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
                    egr->setInformation(InfoG);
                    egr->computeError();
                    optimizer.addEdge(egr);

                    // acc bias随机游走边
                    EdgeAccRW* ear = new EdgeAccRW();
                    ear->setVertex(0, VA1);
                    ear->setVertex(1, VA2);
                    Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
                    ear->setInformation(InfoA);
                    ear->computeError();
                    optimizer.addEdge(ear);
                }
            }
            else
            {
                cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu" << endl;
            }
        }
    }

    // IMU初始化阶段，给全局bias添加零bias先验约束
    if (bInit)
    {
        g2o::HyperGraph::Vertex* VG = optimizer.vertex(4 * maxKFid + 2);
        g2o::HyperGraph::Vertex* VA = optimizer.vertex(4 * maxKFid + 3);
        Eigen::Vector3f bprior;
        bprior.setZero();
        // 加速度bias先验边
        EdgePriorAcc* epa = new EdgePriorAcc(bprior);
        epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
        double infoPriorA = priorA;
        // epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);
        // 陀螺仪bias先验边
        EdgePriorGyro* epg = new EdgePriorGyro(bprior);
        epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
        double infoPriorG = priorG;
        // epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);
    }

    // 视觉重投影Huber阈值
    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
    // 地图点顶点ID偏移量，避开关键帧顶点ID
    const unsigned long iniMPid = maxKFid * 5;
    std::unordered_set<MapPoint*> vbNotIncludedMP;

    // ========= 添加地图点顶点 + 视觉重投影边 =========
    for (auto pMP : vpMPs)
    {
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        auto const observations = pMP->GetObservations();
        bool bAllFixed = true; // 该地图点所有观测的关键帧是否全部固定

        // 遍历该地图点所有观测
        for (auto const& [pKFi, vObs] : observations)
        {
            if (pKFi->mnId > maxKFid)
                continue;
            if (!pKFi->isBad())
            {
                const int leftIndex = get<0>(vObs);
                cv::KeyPoint kpUn;
                // ---------------- 单目左目观测 ----------------
                if (leftIndex != -1 && pKFi->mvuRight[get<0>(vObs)] < 0)
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;
                    // IMU‑VIS BA使用EdgeMono自定义边
                    EdgeMono* e = new EdgeMono(0);
                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    // 如果存在不固定的关键帧，bAllFixed置false
                    if (bAllFixed)
                        if (!VP->fixed())
                            bAllFixed = false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);
                    optimizer.addEdge(e);
                }
                // ---------------- 双目观测 ----------------
                else if (leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0)
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
                    EdgeStereo* e = new EdgeStereo(0);
                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if (bAllFixed)
                        if (!VP->fixed())
                            bAllFixed = false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);
                    optimizer.addEdge(e);
                }
                // ---------------- 双目右相机观测 mpCamera2 ----------------
                if (pKFi->mpCamera2)
                {
                    int rightIndex = get<1>(vObs);
                    if (rightIndex != -1 && rightIndex < static_cast<size_t>(pKFi->mvKeysRight.size()))
                    {
                        rightIndex -= pKFi->NLeft;
                        Eigen::Matrix<double, 2, 1> obs;
                        kpUn = pKFi->mvKeysRight[rightIndex];
                        obs << kpUn.pt.x, kpUn.pt.y;
                        EdgeMono* e = new EdgeMono(1);
                        g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                        if (bAllFixed)
                            if (!VP->fixed())
                                bAllFixed = false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
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
        // 如果所有观测的关键帧全部固定，该地图点无约束，移除顶点
        if (bAllFixed)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP.insert(pMP);
        }
    }

    // 外部停止标记有效直接返回，不做优化
    if (pbStopFlag && pbStopFlag)
        return;

    // 初始化优化器，执行迭代
    optimizer.initializeOptimization();
    optimizer.optimize(its);

    // ============ 提取优化结果，写回关键帧：位姿、速度、IMU bias ============
    for (auto pKFi : vpKFs)
    {
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        // nLoopId ==0，普通BA直接覆盖；回环GBA保存临时变量
        if (nLoopId == 0)
        {
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);
        }
        else
        {
            pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
            pKFi->mnBAGlobalForKF = nLoopId;
        }

        // IMU相关状态恢复
        if (pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            if (nLoopId == 0)
            {
                pKFi->SetVelocity(VV->estimate().cast<float>());
            }
            else
            {
                pKFi->mVwbGBA = VV->estimate().cast<float>();
            }

            VertexGyroBias* VG;
            VertexAccBias* VA;
            if (!bInit)
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            }
            else
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(4 * maxKFid + 2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(4 * maxKFid + 3));
            }
            // 组合6维bias：[gyro_bias, acc_bias]
            Vector6d vb;
            vb << VG->estimate(), VA->estimate();
            IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
            if (nLoopId == 0)
            {
                pKFi->SetNewBias(b);
            }
            else
            {
                pKFi->mBiasGBA = b;
            }
        }
    }

    // 恢复地图点世界坐标
    for (auto pMP : vpMPs)
    {
        if (vbNotIncludedMP.find(pMP) != vbNotIncludedMP.end())
            continue;
        if (pMP->isBad())
            continue;
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(pMP->mnId + iniMPid + 1));
        if (nLoopId == 0)
        {
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA = vPoint->estimate().cast<float>();
            pMP->mnBAGlobalForKF = nLoopId;
        }
    }
    // 地图修改计数器+1，其他线程感知地图发生变更
    pMap->IncreaseChangeIndex();
}

/**
 * @brief 普通帧位姿优化（仅优化相机位姿，地图点固定不动），跟踪阶段调用
 * @param pFrame 当前帧，普通Frame，非KeyFrame
 * @return 返回内点数量
 */
int Optimizer::PoseOptimization(const std::shared_ptr<Frame>& pFrame) {
    // 构造g2o稀疏优化器对象，用于仅优化帧位姿，路标点固定
    g2o::SparseOptimizer optimizer;
    // 关闭g2o控制台打印输出，不输出迭代调试信息
    optimizer.setVerbose(false);
    // 输出trace级别日志，标记进入位姿优化函数
    oslog::trace("Enter PoseOptimization");

    // 构建稠密线性求解器，针对BlockSolver_6_3的位姿矩阵类型(6维位姿，3维路标点)
    // unique_ptr接管内存，无需手动delete g2o对象
    std::unique_ptr<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>
        linearSolver(
            std::make_unique<
                g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>());
    // 构造Levenberg‑Marquardt优化算法，传入块求解器，求解器接管linearSolver所有权
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    // 将LM求解器设置给稀疏优化器
    optimizer.setAlgorithm(solver);

    // 记录初始有效匹配点数量，少于3个无法求解位姿直接返回
    int nInitialCorrespondences = 0;

    // ========= 设置帧位姿顶点 =========
    // SE3Expmap：流形上的SE3位姿顶点，使用李代数增量更新，6自由度
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    {
        // 获取当前帧的相机‑世界变换Tcw，Sophus::SE3<float>类型
        const Sophus::SE3<float> Tcw = pFrame->GetPose();
        // Sophus转g2o::SE3Quat，四元数+平移，float转double供g2o使用
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
    }
    // 设置顶点id为0，本图只有这一个位姿顶点
    vSE3->setId(0);
    // 不固定位姿，参与优化
    vSE3->setFixed(false);
    // 将位姿顶点加入优化图
    optimizer.addVertex(vSE3);

    // ========= 准备路标点观测相关容器 =========
    // N为当前帧特征点总数量
    const int N = pFrame->N;

    // 单目仅位姿边：普通单目相机观测，只优化位姿，路标点固定
    vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    // FHR刚体模式：第二个相机(右目)的投影边，外参Trl固定
    vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody*> vpEdgesMono_FHR;
    // 保存每条边对应的帧内特征点索引，方便优化后标记outlier
    vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
    // 预分配内存，避免vector频繁realloc
    vpEdgesMono.reserve(N);
    vpEdgesMono_FHR.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeRight.reserve(N);

    // 双目仅位姿投影边，输入观测是(u,v,ur)三维
    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    // Huber鲁棒核delta阈值：单目2自由度卡方95%阈值 chi2=5.991，开根号
    const float deltaMono = sqrt(5.991);
    // 双目3自由度卡方95%阈值 chi2=7.815，开根号
    const float deltaStereo = sqrt(7.815);

    {
        // 全局路标点互斥锁，防止多线程读写MapPoint
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        // 遍历帧所有特征点，构建观测边
        for (int i = 0; i < N; i++) {
            // 获取该特征点关联的世界路标点
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (pMP) {
                // 普通SLAM模式：没有第二个相机（非FHR刚体相机）
                if (!pFrame->mpCamera2) {
                    // 单目观测：mvuRight[i] < 0代表没有右目匹配，纯单目
                    if (pFrame->mvuRight[i] < 0) {
                        // 有效匹配计数+1
                        nInitialCorrespondences++;
                        // 初始标记该点不是外点
                        pFrame->mvbOutlier[i] = false;

                        // 观测值：图像像素坐标(u,v)
                        Eigen::Matrix<double, 2, 1> obs;
                        const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        // 创建仅位姿优化的单目投影边，路标点Xw固定，只优化顶点0(帧位姿)
                        ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e =
                            new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

                        // 绑定唯一可优化顶点id=0（帧位姿）
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                             optimizer.vertex(0)));
                        // 设置像素观测
                        e->setMeasurement(obs);
                        // 信息矩阵：由特征金字塔层级确定invSigma2，金字塔越高噪声越大
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        // 设置Huber鲁棒核，抑制外点影响
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        // 给边传入相机模型指针、路标点世界坐标
                        e->pCamera = pFrame->mpCamera;
                        e->Xw = pMP->GetWorldPos().cast<double>();

                        // 将边加入优化图
                        optimizer.addEdge(e);

                        // 保存边和对应特征点索引，用于迭代后检测外点
                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    } else {
                        // 双目观测：存在右目匹配，观测是(u,v,ur)
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        // 三维观测：左目像素u,v + 右目像素ur
                        Eigen::Matrix<double, 3, 1> obs;
                        const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
                        const float& kp_ur = pFrame->mvuRight[i];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        // 双目仅位姿投影边
                        g2o::EdgeStereoSE3ProjectXYZOnlyPose* e =
                            new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                        // 绑定待优化位姿顶点
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                             optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        // 3维信息矩阵
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        // Huber鲁棒核
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaStereo);

                        // 传入双目相机内参、基线bf、路标点世界坐标
                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        e->bf = pFrame->mbf;
                        e->Xw = pMP->GetWorldPos().cast<double>();

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }
                } else {
                    // FHR模式：存在第二个相机，机器人本体‑相机外参固定不变
                    nInitialCorrespondences++;

                    cv::KeyPoint kpUn;

                    if (i < pFrame->Nleft) {
                        // 左相机观测，使用普通单目投影边
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
                    } else {
                        // 右相机观测，使用带本体外参Trl的投影边EdgeSE3ProjectXYZOnlyPoseToBody
                        kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        pFrame->mvbOutlier[i] = false;

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

                        // 右目相机模型，路标点世界坐标，相机‑本体相对位姿Trl固定
                        e->pCamera = pFrame->mpCamera2;
                        e->Xw = pMP->GetWorldPos().cast<double>();
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

    // 有效匹配点小于3，无法求解6自由度位姿，直接返回0个内点
    if (nInitialCorrespondences < 3) return 0;

    oslog::trace("   ... starting PoseOptimization");

    // 执行4轮优化：每轮优化后做外点检测，设置edge level控制是否参与下一轮优化
    // 第3轮(iter==2)移除鲁棒核，使用纯最小二乘
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    // 每轮迭代最大迭代次数
    const int iterations[4] = {10, 10, 10, 10};

    // 统计外点数量
    int nBad = 0;
    for (int iter = 0; iter < 4; iter++) {
        oslog::trace("   ... Iter {}", iter);

        // 每轮开始，把帧当前位姿重新设置给顶点，用上一帧估计值作为初始值
        {
            const Sophus::SE3<float> Tcw = pFrame->GetPose();
            g2o::SE3Quat est = g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                            Tcw.translation().cast<double>());
            vSE3->setEstimate(est);
        }

        // 初始化优化器，level=0，只激活level=0的边
        optimizer.initializeOptimization(0);
        // 执行LM优化，最多iterations[iter]步
        optimizer.optimize(iterations[iter]);

        nBad = 0;
        // ========= 遍历普通单目边，判断外点 =========
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            // 如果上一轮标记为outlier，强制计算误差，更新chi2
            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            // 获取该边卡方误差
            const float chi2 = e->chi2();

            // 超过阈值标记为外点，设置level=1，下一轮优化不参与
            if (chi2 > chi2Mono[iter]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
            }

            // 第3轮迭代，移除鲁棒核，切换普通最小二乘
            if (iter == 2) e->setRobustKernel(0);
        }

        // ========= FHR右目边外点检测 =========
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

        // ========= 双目边外点检测 =========
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

        // 剩余有效边太少，提前终止四轮循环
        if (optimizer.edges().size() < 10) break;
    }

    // ========= 取回优化完成的位姿，写回Frame对象 =========
    const g2o::VertexSE3Expmap* vSE3_recov =
        static_cast<const g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    const g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    // g2o double转Sophus float SE3，赋值给帧
    const Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                                  SE3quat_recov.translation().cast<float>());
    pFrame->SetPose(pose);

    oslog::trace(" Exit PoseOptimization");
    // 返回内点数量 = 初始匹配数 - 外点数量
    return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(const shared_ptr<KeyFrame>& pKF,
                                      bool* pbStopFlag,
                                      const std::shared_ptr<Map>& pMap,
                                      int& num_fixedKF, int& num_OptKF,
                                      int& num_MPs, int& num_edges) {
    // ========= 局部BA：构建局部窗口 =========
    // lLocalKeyFrames：待优化局部关键帧集合，从当前关键帧广度优先搜索共视关键帧
    list<shared_ptr<KeyFrame>> lLocalKeyFrames;

    // 把当前关键帧加入局部优化集合，标记mnBALocalForKF，避免重复加入
    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    auto pCurrentMap = pKF->GetMap();

    // 获取与pKF共视的关键帧，加入局部待优化窗口
    const auto vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (auto pKFi : vNeighKFs) {
        pKFi->mnBALocalForKF = pKF->mnId;
        // 坏关键帧、不属于当前地图的跳过
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            lLocalKeyFrames.push_back(pKFi);
    }

    // ========= 收集局部窗口内所有被观测的路标点 =========
    num_fixedKF = 0;
    list<MapPoint*> lLocalMapPoints;
    set<MapPoint*> sNumObsMP;
    for (auto pKFi : lLocalKeyFrames) {
        // 如果是地图初始化第一帧，该帧后续会被固定
        if (pKFi->mnId == pMap->GetInitKFid()) {
            num_fixedKF = 1;
        }
        // 获取该关键帧所有关联路标点
        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();

        for (auto pMP : vpMPs) {
            if (pMP && !pMP->isBad() && pMP->GetMap() == pCurrentMap) {
                // mnBALocalForKF标记，防止同一个MapPoint重复入list
                if (pMP->mnBALocalForKF != pKF->mnId) {
                    lLocalMapPoints.push_back(pMP);
                    pMP->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }

    // ========= 收集固定关键帧：看到局部路标点，但不在局部优化窗口内的关键帧 =========
    // 这些关键帧位姿固定，只提供约束，不参与位姿优化
    list<shared_ptr<KeyFrame>> lFixedCameras;
    for (auto pMP : lLocalMapPoints) {
        // 获取该路标点所有观测（哪些关键帧看到它）
        map<shared_ptr<KeyFrame>, tuple<int, int>> observations =
            pMP->GetObservations();

        for (auto mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            shared_ptr<KeyFrame> pKFi = mit->first;

            // 不在局部优化窗口、也没有标记为固定BA，则加入固定相机列表
            if (pKFi->mnBALocalForKF != pKF->mnId &&
                pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    // 固定关键帧总数 = 初始化帧标记 + 外部固定相机数量
    num_fixedKF = lFixedCameras.size() + num_fixedKF;

    // 没有任何固定帧，BA缺少约束，直接退出
    if (num_fixedKF == 0) {
        oslog::info(
            "LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted");
        return;
    }

    // ========= g2o优化器初始化 =========
    g2o::SparseOptimizer optimizer;
    // 使用Eigen线性求解器，BlockSolver_6_3：6维位姿，3维路标点
    auto linearSolver = std::make_unique<
        g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    // IMU模式下设置LM初始lambda更大，增强阻尼
    if (pMap->IsInertial()) solver->setUserLambdaInit(100.0);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // 设置外部停止标志，多线程回环时可以中断BA
    if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // DEBUG：保存本次BA涉及的关键帧id，用于调试打印
    pCurrentMap->msOptKFs.clear();
    pCurrentMap->msFixedKFs.clear();

    // ========= 添加【待优化】局部关键帧顶点 =========
    for (auto pKFi : lLocalKeyFrames) {
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        {
            const Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                           Tcw.translation().cast<double>()));
        }
        vSE3->setId(pKFi->mnId);
        // 地图初始化第一帧固定住，防止尺度漂移
        vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
        optimizer.addVertex(vSE3);
        // 更新最大关键帧id，用于路标点vertex id偏移
        if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
        pCurrentMap->msOptKFs.insert(pKFi->mnId);
    }
    num_OptKF = lLocalKeyFrames.size();

    // ========= 添加【固定】关键帧顶点 =========
    for (auto pKFi : lFixedCameras) {
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        // fixed=true：位姿不优化，仅作为约束
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
        pCurrentMap->msFixedKFs.insert(pKFi->mnId);
    }

    // ========= 准备各类投影边容器，预分配内存 =========
    const int nExpectedSize =
        (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    // 普通单目投影边，顶点0路标点，顶点1关键帧位姿
    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    // FHR右目投影边，本体‑相机外参固定
    vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    // 记录每条边对应的关键帧和路标点，优化后用来检测外点
    vector<shared_ptr<KeyFrame>> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<shared_ptr<KeyFrame>> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    // 双目投影边，观测(u,v,ur)三维
    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<shared_ptr<KeyFrame>> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    // Huber阈值，卡方95%开根号
    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    int nPoints = 0;

    // 统计总边数量
    int nEdges = 0;

    // ========= 遍历局部路标点，添加路标点顶点 + 观测边 =========
    for (auto pMP : lLocalMapPoints) {
        // 路标点顶点VertexPointXYZ，三维世界坐标，设置边缘化
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        // id偏移：避免和关键帧id冲突，maxKFid+1开始分配路标点id
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        // 路标点设置可边缘化，BA结束后消去路标点变量，只保留位姿Hessian
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        nPoints++;

        // 获取该路标点全部观测记录：哪些关键帧、左右特征索引
        map<shared_ptr<KeyFrame>, tuple<int, int>> observations =
            pMP->GetObservations();

        // 遍历每一条观测，构建投影边
        for (auto const& [pKFi, vObs] : observations) {
            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                const int leftIndex = get<0>(vObs);

                // ========= 左目单目观测，没有右目匹配 =========
                if (leftIndex != -1 && pKFi->mvuRight[get<0>(vObs)] < 0) {
                    const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    // vertex0:路标点，vertex1:关键帧位姿
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                       optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                       optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    // Huber鲁棒核抑制外点
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->pCamera = pKFi->mpCamera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    nEdges++;
                } else if (leftIndex != -1 && pKFi->mvuRight[get<0>(vObs)] >= 0) {
                    // ========= 双目观测，存在右目匹配 =========
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

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    nEdges++;
                }

                // ========= FHR模式：右相机观测 =========
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

    // 如果外部停止标记被置true，直接退出，不执行优化
    if (pbStopFlag)
        if (*pbStopFlag) return;

    // 初始化图优化，执行10轮LM迭代
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // 保存需要删除的观测对<关键帧，路标点>，优化后剔除外点观测
    vector<pair<shared_ptr<KeyFrame>, MapPoint*>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                     vpEdgesStereo.size());

    // ========= 检查单目边外点：chi2超限 或者 投影深度为负 =========
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad()) continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            auto pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // ========= FHR右目边外点检测 =========
    for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
        ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
        MapPoint* pMP = vpMapPointEdgeBody[i];

        if (pMP->isBad()) continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            auto pKFi = vpEdgeKFBody[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // ========= 双目边外点检测 =========
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad()) continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            auto pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // 地图更新互斥锁，修改地图观测关系必须上锁
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // 剔除外点观测：关键帧删除该路标点匹配，路标点删除该关键帧观测
    if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
            shared_ptr<KeyFrame> pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // ========= 把优化结果回写到关键帧对象 =========
    for (auto pKFi : lLocalKeyFrames) {
        g2o::VertexSE3Expmap* vSE3 =
            static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                         SE3quat.translation().cast<float>());
        pKFi->SetPose(Tiw);
    }

    // ========= 把优化后的三维点回写到MapPoint，更新法向量和深度 =========
    for (auto pMP : lLocalMapPoints) {
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
            optimizer.vertex(pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    // 地图变更计数+1，用于多线程判断地图是否发生修改
    pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(
    const shared_ptr<Map>& pMap, const shared_ptr<KeyFrame>& pLoopKF,
    const shared_ptr<KeyFrame>& pCurKF,
    const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose& CorrectedSim3,
    const map<shared_ptr<KeyFrame>, set<shared_ptr<KeyFrame>>>& LoopConnections,
    const bool& bFixScale) {
  // Setup optimizer
  // 创建g2o稀疏优化器对象，用于执行位姿图优化
  g2o::SparseOptimizer optimizer;
  // 关闭g2o控制台打印输出，不输出优化过程日志
  optimizer.setVerbose(false);

  // 创建Eigen线性求解器，针对7维块(BlockSolver_7_3，顶点7维，边残差3维)
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
  // 构造L‑M列文伯格‑马夸尔特优化算法，传入7‑3分块求解器，接管linearSolver所有权
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolver_7_3>(std::move(linearSolver)));

  // 设置L‑M初始lambda阻尼系数，很小，信任高斯‑牛顿步
  solver->setUserLambdaInit(1e-16);
  // 将配置好的求解算法挂载到稀疏优化器
  optimizer.setAlgorithm(solver);

  // 获取地图中所有关键帧
  const vector<shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  // 获取地图中所有地图点
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  // 获取当前地图最大关键帧ID，用于数组下标分配
  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  // vScw[nID]：存储每个关键帧的Sim3位姿Tcw(世界到相机)
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  // vCorrectedSwc[nID]：优化后Twc(相机到世界)Sim3，用于后续修正地图点
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  // vpVertices[nID]：保存g2o顶点指针，方便后续按ID快速取回顶点对象
  vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

  // Debug专用数组，保存每个关键帧旋转后的z轴方向向量，用于调试可视化
  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vZvectors(
      nMaxKFid + 1);
  // 单位z轴向量(0,0,1)，相机前向方向
  Eigen::Vector3d z_vec;
  z_vec << 0.0, 0.0, 1.0;

  // 共视权重阈值，只有共视特征数>=100才构建约束边
  const int minFeat = 100;

  // Set KeyFrame vertices
  // 遍历所有关键帧，创建Sim3顶点，设置初始估计值，加入优化器
  for (auto pKF : vpKFs) {
    // 跳过已经标记为坏的关键帧
    if (pKF->isBad()) continue;
    // 新建Sim3指数映射顶点，7维变量：旋转+平移+尺度
    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    // 获取当前关键帧ID，作为g2o顶点ID
    const int nIDi = pKF->mnId;
    // 在回环修正后的位姿map查找该关键帧
    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);
    if (it != CorrectedSim3.end()) {
      // 存在回环修正结果，直接使用回环输出的Sim3作为顶点初始值
      vScw[nIDi] = it->second;
      VSim3->setEstimate(it->second);
    } else {
      // 没有回环修正，从关键帧原始SE3位姿构造Sim3，尺度s=1
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
      vScw[nIDi] = Siw;
      VSim3->setEstimate(Siw);
    }

    // 将地图初始化第一帧设为固定顶点，作为位姿图基准，不参与优化
    if (pKF->mnId == pMap->GetInitKFid()) VSim3->setFixed(true);

    // 设置顶点ID，和关键帧mnId保持一致
    VSim3->setId(nIDi);
    // 该顶点不做边缘化，位姿图中所有关键帧都保留在图内
    VSim3->setMarginalized(false);
    // 是否固定尺度；单目回环bFixScale=false，双目/RGBD=true尺度固定为1
    VSim3->_fix_scale = bFixScale;

    // 将顶点添加到g2o优化器
    optimizer.addVertex(VSim3);
    // Debug：记录旋转之后相机z轴方向
    vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec;

    // 缓存顶点指针，后续按ID快速访问
    vpVertices[nIDi] = VSim3;
  }

  // 集合，记录已经添加过的边，避免重复添加同一条双向约束边，存储pair(minId,maxId)
  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // 信息矩阵，单位阵，Pose‑Graph中每条边权重相同
  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  // Set Loop edges
  // 添加回环闭合约束边：LoopConnections是回环检测得到的匹配关键帧对
  int count_loop = 0;
  for (auto const& [pKF, spConnections] : LoopConnections) {
    // 当前关键帧ID
    const long unsigned int nIDi = pKF->mnId;
    // Siw：Tcw，世界→相机i的Sim3
    const g2o::Sim3 Siw = vScw[nIDi];
    // Swi：Twc，相机i→世界Sim3，取逆
    const g2o::Sim3 Swi = Siw.inverse();

    // 遍历与pKF构成回环的关联关键帧
    for (auto const& pConnection : spConnections) {
      // 关联关键帧ID
      auto const nIDj = pConnection->mnId;
      // 过滤条件：不是(curKF‑loopKF)这一对，并且两帧共视特征数小于阈值则跳过
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(pConnection) < minFeat)
        continue;

      // Sjw：世界→相机j Sim3
      const g2o::Sim3 Sjw = vScw[nIDj];
      // Sji = Sjw * Swi ：Ti_j，i坐标系到j坐标系的相对Sim3变换，作为边的测量值
      const g2o::Sim3 Sji = Sjw * Swi;

      // 创建Sim3位姿图边EdgeSim3，约束两个关键帧之间相对Sim3
      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      // vertex1：j帧；vertex0：i帧；边模型 e(z = Sji, xi, xj) 即 xj = Sji * xi
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      // 设置观测值，两帧之间相对Sim3
      e->setMeasurement(Sji);

      // 设置信息矩阵，单位阵
      e->information() = matLambda;

      // 将回环边加入优化器
      optimizer.addEdge(e);
      // 统计回环边数量
      count_loop++;
      // 记录这条边，防止后续普通共视边重复添加
      sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
    }
  }

  // Set normal edges
  // 添加普通约束边：生成树边、回环边、共视边、IMU惯性边
  for (auto pKF : vpKFs) {
    // 当前关键帧ID
    const int nIDi = pKF->mnId;

    g2o::Sim3 Swi;
    // 优先使用NonCorrectedSim3(未做回环修正的位姿)，否则用vScw取逆得到Twc
    LoopClosing::KeyFrameAndPose::const_iterator iti =
        NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Swi = (iti->second).inverse();
    else
      Swi = vScw[nIDi].inverse();

    // 获取生成树父关键帧
    shared_ptr<KeyFrame> pParentKF = pKF->GetParent();

    // Spanning tree edge
    // 添加共视生成树的父子关键帧约束边
    if (pParentKF) {
      // 父关键帧ID
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Sjw;
      // 优先取未修正的父帧位姿，否则从vScw读取
      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Sjw = itj->second;
      else
        Sjw = vScw[nIDj];

      // Sji = Sjw * Swi：i到j的相对Sim3，作为边测量
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

    // Loop edges
    // 添加该关键帧记录的其他回环边
    const set<shared_ptr<KeyFrame>> sLoopEdges = pKF->GetLoopEdges();
    for (auto pLKF : sLoopEdges) {
      // 只处理ID更小的帧，避免边重复双向添加
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Slw;
        // 优先使用NonCorrectedSim3，否则读取vScw
        LoopClosing::KeyFrameAndPose::const_iterator itl =
            NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Slw = itl->second;
        else
          Slw = vScw[pLKF->mnId];

        // Sli = Slw * Swi，i到回环帧l的相对Sim3
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

    // Covisibility graph edges
    // 添加共视图约束边，共视权重大于minFeat的邻居关键帧
    const vector<shared_ptr<KeyFrame>> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);

    for (auto pKFn : vpConnectedKFs) {
      // 过滤：有效帧、不是父帧、不是子帧；注释掉了排除回环边
      if (pKFn && pKFn != pParentKF &&
          !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/) {
        // 关键帧有效，并且pKFn ID更小，避免重复边
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          // 如果这条边已经被加入过，则跳过
          if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId),
                                             max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Snw;
          // 优先NonCorrectedSim3，否则读取vScw
          LoopClosing::KeyFrameAndPose::const_iterator itn =
              NonCorrectedSim3.find(pKFn);
          if (itn != NonCorrectedSim3.end())
            Snw = itn->second;
          else
            Snw = vScw[pKFn->mnId];

          // Sni = Snw * Swi，i到n的相对Sim3变换
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

    // Inertial edges if inertial
    // IMU模式：添加时序惯性相邻关键帧之间的Sim3约束边
    if (pKF->bImu && pKF->mPrevKF) {
      g2o::Sim3 Spw;
      // 优先NonCorrectedSim3，否则读取vScw
      LoopClosing::KeyFrameAndPose::const_iterator itp =
          NonCorrectedSim3.find(pKF->mPrevKF);
      if (itp != NonCorrectedSim3.end())
        Spw = itp->second;
      else
        Spw = vScw[pKF->mPrevKF->mnId];

      // Spi = Spw * Swi，i到上一帧prev的相对Sim3
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

  // g2o初始化优化结构，准备迭代
  optimizer.initializeOptimization();
  // 计算初始所有激活边的残差，得到初始总误差
  optimizer.computeActiveErrors();
  // 执行最多20轮L‑M迭代优化位姿图
  optimizer.optimize(20);
  // 优化结束后再次计算残差
  optimizer.computeActiveErrors();
  // 上锁地图更新互斥锁，防止多线程读写地图冲突
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  // 从优化后的Sim3恢复SE3位姿，更新关键帧位姿；Sim3平移除以尺度s得到SE3平移
  for (auto pKFi : vpKFs) {
    const int nIDi = pKFi->mnId;

    // 取出优化后的Sim3顶点
    g2o::VertexSim3Expmap* VSim3 =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    // 获取优化后估计值Sim3 Tiw(世界到相机i)
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    // 求逆得到Twc(相机i到世界)，保存用于地图点修正
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    // 获取优化得到的尺度因子s
    double s = CorrectedSiw.scale();

    // Sim3转SE3：旋转不变，平移除以尺度s；转float存入关键帧
    Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(),
                     CorrectedSiw.translation().cast<float>() / s);
    // 更新关键帧的位姿
    pKFi->SetPose(Tiw);
  }

  // Correct points. Transform to "non‑optimized" reference keyframe pose and
  // transform back with optimized pose
  // 根据位姿图优化结果修正所有地图点的世界坐标：参考帧原始位姿投影到相机，再用优化后参考帧位姿反投影回世界
  for (auto pMP : vpMPs) {
    // 跳过坏点
    if (pMP->isBad()) continue;

    int nIDr;
    // 如果该地图点是本次回环修正生成，使用记录的修正参考关键帧ID
    if (pMP->mnCorrectedByKF == pCurKF->mnId) {
      nIDr = pMP->mnCorrectedReference;
    } else {
      // 否则取地图点原始参考关键帧
      shared_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
      nIDr = pRefKF->mnId;
    }

    // Srw：回环优化前参考帧的Sim3 Twr(世界→参考相机)
    g2o::Sim3 Srw = vScw[nIDr];
    // correctedSwr：优化之后参考帧Twc(相机→世界)
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    // 获取地图点原始世界坐标，转为double
    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    // 坐标修正公式：P' = correctedSwr.map( Srw.map(Pw) )
    // Srw.map(Pw)：世界点投影到优化前参考相机坐标系；correctedSwr.map()：用优化后参考帧位姿反投影回世界坐标系
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    // 将修正后的坐标写回地图点，转float
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    // 更新地图点观测方向与深度信息
    pMP->UpdateNormalAndDepth();
  }

  // TODO Check this changeindex
  // 地图版本号自增，通知其他线程地图发生改动
  pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(
    const std::shared_ptr<KeyFrame>& pCurKF,
    vector<shared_ptr<KeyFrame>>& vpFixedKFs,
    vector<shared_ptr<KeyFrame>>& vpFixedCorrectedKFs,
    vector<shared_ptr<KeyFrame>>& vpNonFixedKFs,
    vector<MapPoint*>& vpNonCorrectedMPs) {
  // Debug打印输出：打印合并地图中各类关键帧与待修正地图点数量
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
                         " KFs non‑fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpNonCorrectedMPs.size()) +
                         " MPs non‑corrected in the merged map",
                     Verbose::VERBOSITY_DEBUG);

  // 新建g2o稀疏优化器对象
  g2o::SparseOptimizer optimizer;
  // 关闭g2o输出日志
  optimizer.setVerbose(false);

  // Eigen线性求解器，用于BlockSolverX通用块求解器
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
  // L‑M优化算法，搭配通用块求解器，接管linearSolver所有权
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  // 设置L‑M初始阻尼lambda
  solver->setUserLambdaInit(1e-16);
  // 挂载求解算法到优化器
  optimizer.setAlgorithm(solver);

  // 获取当前关键帧所属地图
  std::shared_ptr<Map> pMap = pCurKF->GetMap();
  // 获取地图最大关键帧ID，用于数组下标
  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  // vScw[nID]：未修正的Sim3 Tcw(世界→相机)
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  // vCorrectedSwc[nID]：优化完成后的Twc(相机→世界)Sim3
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  // 缓存g2o顶点指针，按关键帧ID索引
  vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

  // vpGoodPose：标记该关键帧位姿是可靠固定位姿
  vector<bool> vpGoodPose(nMaxKFid + 1);
  // vpBadPose：标记该关键帧位姿需要参与优化修正
  vector<bool> vpBadPose(nMaxKFid + 1);

  // 共视权重阈值，大于等于该值才构建约束边
  const int minFeat = 100;

  // 处理vpFixedKFs：合并地图内固定不动的关键帧，位姿固定不优化
  for (auto const& pKFi : vpFixedKFs) {
    // 跳过标记为坏的关键帧
    if (pKFi->isBad()) continue;

    // 创建Sim3指数映射顶点
    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    // 获取关键帧ID
    const int nIDi = pKFi->mnId;
    // 将关键帧SE3位姿转为double
    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    // 构造Sim3，尺度固定s=1
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    // Twc = Siw.inverse()存入vCorrectedSwc，固定帧优化前后不变
    vCorrectedSwc[nIDi] = Siw.inverse();
    // 设置顶点初始估计值
    VSim3->setEstimate(Siw);

    // 设置该顶点为固定，不参与优化更新
    VSim3->setFixed(true);
    // 设置顶点ID等于关键帧mnId
    VSim3->setId(nIDi);
    // 不做边缘化处理
    VSim3->setMarginalized(false);
    // 固定尺度，用于双目/RGBD
    VSim3->_fix_scale = true;

    // 将顶点加入g2o优化器
    optimizer.addVertex(VSim3);
    // 缓存顶点指针
    vpVertices[nIDi] = VSim3;
    // 标记位姿可靠，不需要优化
    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = false;
  }
  // Debug日志，固定帧加载完成
  Verbose::PrintMess("Opt_Essential: vpFixedKFs loaded",
                     Verbose::VERBOSITY_DEBUG);

  // sIdKF集合记录已经添加到优化图的关键帧ID，防止重复添加顶点
  set<unsigned long> sIdKF;
  // vpFixedCorrectedKFs：旧地图过来的固定关键帧，位姿固定，但保存合并前原始位姿用于地图点修正
  for (auto const& pKFi : vpFixedCorrectedKFs) {
    if (pKFi->isBad()) continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKFi->mnId;
    // 当前合并后的SE3位姿
    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    // 保存当前Twc，固定帧不会被优化改变
    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    // 获取合并之前旧地图的原始位姿mTcwBefMerge，转为double
    Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
    // 将合并前原始位姿存入vScw，后续地图点修正要用
    vScw[nIDi] =
        g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

    // 该类关键帧顶点固定，不优化
    VSim3->setFixed(true);
    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);
    vpVertices[nIDi] = VSim3;
    // 将ID加入已添加集合
    sIdKF.insert(nIDi);
    // 标记：位姿是固定可靠，但是原始位姿是旧地图的，标记badPose用于边构建逻辑
    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = true;
  }

  // vpNonFixedKFs：待优化的关键帧，位姿可变，参与图优化
  for (auto const& pKFi : vpNonFixedKFs) {
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;
    // 如果ID已经被加入图，跳过，防止重复创建顶点
    if (sIdKF.count(nIDi))  // It has already added in the corrected merge KFs
      continue;
    // 创建Sim3顶点
    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();
    // 获取当前位姿
    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    // 存入vScw作为初始未修正位姿
    vScw[nIDi] = Siw;
    VSim3->setEstimate(Siw);

    // 设置为非固定，允许优化更新
    VSim3->setFixed(false);
    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);
    vpVertices[nIDi] = VSim3;
    sIdKF.insert(nIDi);
    // 标记位姿不可靠，需要优化
    vpGoodPose[nIDi] = false;
    vpBadPose[nIDi] = true;
  }

  // 将三类关键帧合并到vpKFs容器，方便遍历构建边
  vector<shared_ptr<KeyFrame>> vpKFs;
  vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() +
                vpNonFixedKFs.size());
  vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
  vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(),
               vpFixedCorrectedKFs.end());
  vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
  // 转为set快速查找该关键帧是否在本次优化图中
  set<shared_ptr<KeyFrame>> spKFs(vpKFs.begin(), vpKFs.end());

  // 信息矩阵，单位阵，所有边权重相同
  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  // 遍历所有参与优化的关键帧，构建各类约束边
  for (auto const& pKFi : vpKFs) {
    // 记录该关键帧成功添加的约束边数量
    int num_connections = 0;
    // 当前关键帧ID
    const int nIDi = pKFi->mnId;

    g2o::Sim3 correctedSwi;
    g2o::Sim3 Swi;

    // 如果是goodPose，取修正后Twc
    if (vpGoodPose[nIDi]) correctedSwi = vCorrectedSwc[nIDi];
    // 如果是badPose，取未修正位姿的逆Twc
    if (vpBadPose[nIDi]) Swi = vScw[nIDi].inverse();

    // 获取生成树父关键帧
    auto pParentKFi = pKFi->GetParent();

    // Spanning tree edge
    // 添加生成树父子关键帧约束边，父帧必须也在本次优化集合内
    if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
      int nIDj = pParentKFi->mnId;

      g2o::Sim3 Sjw;
      bool bHasRelation = false;

      // 分支：i,j都是goodPose，使用correctedSwc取逆得到Sjw
      if (vpGoodPose[nIDi] && vpGoodPose[nIDj]) {
        Sjw = vCorrectedSwc[nIDj].inverse();
        bHasRelation = true;
      }
      // 分支：i,j都是badPose，使用原始未修正vScw
      else if (vpBadPose[nIDi] && vpBadPose[nIDj]) {
        Sjw = vScw[nIDj];
        bHasRelation = true;
      }

      // 满足关系才构建边
      if (bHasRelation) {
        // Sji = Sjw * Swi：i到j的相对Sim3变换，作为边观测
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

    // Loop edges
    // 添加该关键帧保存的回环约束边，pLKF必须在优化集合，ID更小避免重复边
    const set<std::shared_ptr<KeyFrame>> sLoopEdges = pKFi->GetLoopEdges();
    for (auto const& pLKF : sLoopEdges) {
      if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId) {
        g2o::Sim3 Slw;
        bool bHasRelation = false;

        // i和l都为goodPose，使用correctedSwc
        if (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId]) {
          Slw = vCorrectedSwc[pLKF->mnId].inverse();
          bHasRelation = true;
        }
        // i和l都为badPose，使用原始vScw
        else if (vpBadPose[nIDi] && vpBadPose[pLKF->mnId]) {
          Slw = vScw[pLKF->mnId];
          bHasRelation = true;
        }

        if (bHasRelation) {
          // Sli = Slw * Swi，i到l的相对Sim3
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

    // Covisibility graph edges
    // 添加共视图约束边，共视权重大于minFeat
    const vector<shared_ptr<KeyFrame>> vpConnectedKFs =
        pKFi->GetCovisiblesByWeight(minFeat);
    for (auto pKFn : vpConnectedKFs) {
      // 过滤条件：非父帧、非子帧、非回环边，并且在优化集合内
      if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) &&
          !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end()) {
        // 关键帧有效，ID更小避免双向重复边
        if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {
          g2o::Sim3 Snw = vScw[pKFn->mnId];
          bool bHasRelation = false;

          // i,n均goodPose，使用修正后位姿
          if (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId]) {
            Snw = vCorrectedSwc[pKFn->mnId].inverse();
            bHasRelation = true;
          }
          // i,n均badPose，使用原始未修正位姿
          else if (vpBadPose[nIDi] && vpBadPose[pKFn->mnId]) {
            Snw = vScw[pKFn->mnId];
            bHasRelation = true;
          }

          if (bHasRelation) {
            // Sni = Snw * Swi，i到n相对Sim3
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

    // Debug打印：该关键帧没有任何约束边，图结构存在风险
    if (num_connections == 0) {
      Verbose::PrintMess(
          "Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections",
          Verbose::VERBOSITY_DEBUG);
    }
  }

  // Optimize!
  // g2o初始化优化器
  optimizer.initializeOptimization();
  // 执行最多20轮L‑M迭代优化
  optimizer.optimize(20);

  // 上锁地图互斥锁，保护地图写操作
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  // 只更新非固定待优化关键帧：Sim3转SE3，平移除以尺度s，保存优化结果
  for (auto pKFi : vpNonFixedKFs) {
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;

    // 获取优化后的Sim3顶点
    g2o::VertexSim3Expmap* VSim3 =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    // 获取优化后的Tiw(世界到相机i)Sim3
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    // 求逆得到Twc，存入vCorrectedSwc用于地图点修正
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    // 获取尺度因子
    double s = CorrectedSiw.scale();
    // Sim3转SE3，平移除以尺度
    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation() / s);

    // 保存合并优化之前的原始位姿，用于地图点修正
    pKFi->mTcwBefMerge = pKFi->GetPose();
    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
    // 将优化后的位姿写入关键帧，转float
    pKFi->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non‑optimized" reference keyframe pose and
  // transform back with optimized pose
  // 修正非校正地图点的世界坐标，使用参考关键帧优化前后位姿做坐标变换
  for (auto pMPi : vpNonCorrectedMPs) {
    if (pMPi->isBad()) continue;

    // 获取地图点参考关键帧
    auto pRefKF = pMPi->GetReferenceKeyFrame();
    // 如果参考关键帧标记为坏，循环删除该观测，重新获取参考关键帧
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

    // 如果该参考关键帧是参与优化的badPose，执行坐标修正
    if (vpBadPose[pRefKF->mnId]) {
      // mTwcBefMerge：优化前参考帧Twc
      Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
      // GetPoseInverse()：优化之后参考帧Twc
      Sophus::SE3f Twr = pRefKF->GetPoseInverse();

      // 坐标变换公式：Pw_new = Twr * TNonCorrectedwr.inv() * Pw_old
      Eigen::Vector3f eigCorrectedP3Dw =
          Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
      // 将修正后坐标写入地图点
      pMPi->SetWorldPos(eigCorrectedP3Dw);

      // 更新地图点观测方向与深度
      pMPi->UpdateNormalAndDepth();
    } else {
      // 参考帧来自另一个地图，异常报错
      cout << "ERROR: MapPoint has a reference KF from another map" << endl;
    }
  }
}

int Optimizer::OptimizeSim3(const shared_ptr& pKF1, const shared_ptr& pKF2, vector<MapPoint*>& vpMatches1, g2o::Sim3& g2oS12, const float th2, const bool bFixScale, Eigen::Matrix<double, 7, 7>& mAcumHessian, const bool bAllPoints) {
    // 创建g2o稀疏优化器对象，用于Sim3位姿图优化
    g2o::SparseOptimizer optimizer;
    // 构建稠密线性求解器，针对BlockSolverX的PoseMatrixType矩阵类型
    auto linearSolver = std::make_unique< g2o::LinearSolverDenseg2o::BlockSolverX::PoseMatrixType>();
    // 构建列文伯格-马夸尔特优化算法，传入块求解器，转移线性求解器所有权
    auto solver = new g2o::OptimizationAlgorithmLevenberg( std::make_uniqueg2o::BlockSolverX(std::move(linearSolver)));
    // 将LM求解算法设置给稀疏优化器
    optimizer.setAlgorithm(solver);

    // 获取关键帧1的世界到相机旋转矩阵R1w，世界坐标系 -> KF1相机坐标系
    const Eigen::Matrix3f R1w = pKF1->GetRotation();
    // 获取关键帧1的世界到相机平移向量t1w，世界坐标系 -> KF1相机坐标系
    const Eigen::Vector3f t1w = pKF1->GetTranslation();
    // 获取关键帧2的世界到相机旋转矩阵R2w，世界坐标系 -> KF2相机坐标系
    const Eigen::Matrix3f R2w = pKF2->GetRotation();
    // 获取关键帧2的世界到相机平移向量t2w，世界坐标系 -> KF2相机坐标系
    const Eigen::Vector3f t2w = pKF2->GetTranslation();

    // 设置Sim3顶点，Sim3包含旋转R、平移t、尺度s，7维参数
    ORB_SLAM3::VertexSim3Expmap* vSim3 = new ORB_SLAM3::VertexSim3Expmap();
    // 是否固定尺度，单目为false，双目/深度相机为true尺度固定为1
    vSim3->_fix_scale = bFixScale;
    // 设置Sim3初始估计值，输入的g2oS12为KF1到KF2的Sim3变换
    vSim3->setEstimate(g2oS12);
    // 设置顶点id为0，Sim3顶点唯一id
    vSim3->setId(0);
    // 设置该顶点不固定，参与优化迭代更新
    vSim3->setFixed(false);
    // 绑定两个关键帧对应的相机模型指针，用于投影计算
    vSim3->pCamera1 = pKF1->mpCamera;
    vSim3->pCamera2 = pKF2->mpCamera;
    // 将Sim3顶点添加到g2o优化器图中
    optimizer.addVertex(vSim3);

    // 设置地图点顶点
    // 获取匹配对数量，vpMatches1[i]代表KF1第i个特征点匹配到KF2的地图点
    const int N = vpMatches1.size();
    // 获取KF1所有地图点，vpMapPoints1[i]为KF1第i个特征对应的地图点
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    // 存储边：将KF2的3D点通过S12投影到KF1图像的边
    vector<ORB_SLAM3::EdgeSim3ProjectXYZ*> vpEdges12;
    // 存储边：将KF1的3D点通过S21(S12逆)投影到KF2图像的边
    vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    // 存储边对应的原始索引i，用于优化后剔除外点时定位vpMatches1
    vector vnIndexEdge;
    // 标记该匹配点是否在KF2关键帧中存在观测
    vector vbIsInKF2;
    // 预分配容器内存，最多2*N条边
    vnIndexEdge.reserve(2 * N);
    vpEdges12.reserve(2 * N);
    vpEdges21.reserve(2 * N);
    vbIsInKF2.reserve(2 * N);
    // Huber鲁棒核阈值，输入th2为卡方阈值，开根号得到残差阈值
    const float deltaHuber = sqrt(th2);
    // 有效匹配对计数
    int nCorrespondences = 0;
    // 坏地图点计数（地图点标记为bad）
    int nBadMPs = 0;
    // 该地图点在KF2中存在观测的计数
    int nInKF2 = 0;
    // 该地图点不在KF2观测内，使用投影虚拟观测计数
    int nOutKF2 = 0;
    // 匹配对中地图点为空的计数
    int nMatchWithoutMP = 0;
    // 仅KF2存在地图点的顶点id集合
    vector vIdsOnlyInKF2;

    // 遍历所有匹配对，构建图顶点与边
    for (int i = 0; i < N; i++) {
        // 当前匹配为空，跳过
        if (!vpMatches1[i]) continue;
        // pMP1：KF1第i个特征对应的地图点
        MapPoint* pMP1 = vpMapPoints1[i];
        // pMP2：KF1第i个特征匹配到KF2的地图点
        MapPoint* pMP2 = vpMatches1[i];
        // KF1地图点顶点id，2*i+1，避免和Sim3顶点id=0冲突
        const int id1 = 2 * i + 1;
        // KF2地图点顶点id，2*(i+1)
        const int id2 = 2 * (i + 1);
        // 获取pMP2在KF2关键帧内部的特征索引，返回-1代表无观测
        const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));
        // 存储KF1相机坐标系下3D点坐标
        Eigen::Vector3f P3D1c;
        // 存储KF2相机坐标系下3D点坐标
        Eigen::Vector3f P3D2c;

        // KF1和KF2两边都存在有效地图点
        if (pMP1 && pMP2) {
            // 两个地图点都没有被标记为坏点
            if (!pMP1->isBad() && !pMP2->isBad()) {
                // 创建KF1相机坐标系下3D点顶点，该顶点固定不优化
                g2o::VertexPointXYZ* vPoint1 = new g2o::VertexPointXYZ();
                // 获取pMP1世界坐标系3D坐标
                Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
                // 将世界坐标转换到KF1相机坐标系
                P3D1c = R1w * P3D1w + t1w;
                // 设置顶点初始估计值，转换为double类型
                vPoint1->setEstimate(P3D1c.cast());
                // 设置顶点id
                vPoint1->setId(id1);
                // 设置顶点固定，地图点坐标固定，只优化Sim3变换
                vPoint1->setFixed(true);
                // 添加顶点到优化器
                optimizer.addVertex(vPoint1);

                // 创建KF2相机坐标系下3D点顶点，固定不优化
                g2o::VertexPointXYZ* vPoint2 = new g2o::VertexPointXYZ();
                // 获取pMP2世界坐标系3D坐标
                Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                // 世界坐标转换到KF2相机坐标系
                P3D2c = R2w * P3D2w + t2w;
                // 设置顶点初始估计值，转double
                vPoint2->setEstimate(P3D2c.cast());
                // 设置顶点id
                vPoint2->setId(id2);
                // 设置顶点固定，地图点坐标固定
                vPoint2->setFixed(true);
                // 添加顶点到优化器
                optimizer.addVertex(vPoint2);
            } else {
                // 存在坏地图点，计数+1，跳过该匹配
                nBadMPs++;
                continue;
            }
        } else {
            // 匹配对至少一侧地图点为空，计数+1
            nMatchWithoutMP++;
            // TODO The 3D position in KF1 doesn't exist
            // 如果KF2侧地图点有效，依然加入KF2的3D点顶点
            if (!pMP2->isBad()) {
                g2o::VertexPointXYZ* vPoint2 = new g2o::VertexPointXYZ();
                Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                P3D2c = R2w * P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
                // 将该id加入仅KF2存在的顶点id列表
                vIdsOnlyInKF2.push_back(id2);
            }
            continue;
        }

        // i2<0代表pMP2在KF2没有观测，并且bAllPoints=false，则跳过该点
        if (i2 < 0 && !bAllPoints) {
            Verbose::PrintMess(" Remove point -> i2: " + to_string(i2) + "; bAllPoints: " + to_string(bAllPoints), Verbose::VERBOSITY_DEBUG);
            continue;
        }
        // KF2相机坐标系下Z为负，点在相机后方，无效，跳过
        if (P3D2c(2) < 0) {
            Verbose::PrintMess("Sim3: Z coordinate is negative", Verbose::VERBOSITY_DEBUG);
            continue;
        }
        // 有效匹配对计数自增
        nCorrespondences++;

        // 设置边 x1 = S12 * X2 ：将KF2的相机坐标系3D点X2，经过S12变换投影得到KF1图像观测x1
        Eigen::Matrix<double, 2, 1> obs1;
        // 获取KF1第i个去畸变关键点
        const cv::KeyPoint& kpUn1 = pKF1->mvKeysUn[i];
        // 图像观测赋值给obs1，像素坐标u,v
        obs1 << kpUn1.pt.x, kpUn1.pt.y;
        // 创建Sim3投影边，输入X2，输出x1
        ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();
        // vertex0：KF2相机坐标系3D点
        e12->setVertex( 0, dynamic_castg2o::OptimizableGraph::Vertex*(optimizer.vertex(id2)));
        // vertex1：待优化Sim3顶点S12
        e12->setVertex( 1, dynamic_castg2o::OptimizableGraph::Vertex*(optimizer.vertex(0)));
        // 设置图像观测测量值obs1
        e12->setMeasurement(obs1);
        // 获取该关键点金字塔层级对应的逆方差
        const float& invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        // 设置信息矩阵，单位矩阵乘以逆方差
        e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);
        // 创建Huber鲁棒核，抑制外点残差
        g2o::RobustKernelHuber rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        // 设置Huber阈值
        rk1->setDelta(deltaHuber);
        // 将边加入优化器
        optimizer.addEdge(e12);

        // 设置边 x2 = S21 * X1 ：S21是S12的逆变换，KF1的3D点X1投影得到KF2图像观测x2
        Eigen::Matrix<double, 2, 1> obs2;
        cv::KeyPoint kpUn2;
        bool inKF2;
        if (i2 >= 0) {
            // i2>=0，pMP2在KF2存在真实观测，读取真实关键点
            kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;
            inKF2 = true;
            nInKF2++;
        } else {
            // pMP2在KF2没有观测，使用KF2相机坐标系3D点生成虚拟归一化平面观测
            float invz = 1 / P3D2c(2);
            float x = P3D2c(0) * invz;
            float y = P3D2c(1) * invz;
            obs2 << x, y;
            // 构造虚拟关键点，使用地图点跟踪尺度层级
            kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);
            inKF2 = false;
            nOutKF2++;
        }
        // 创建逆Sim3投影边，输入X1，输出x2
        ORB_SLAM3::EdgeInverseSim3ProjectXYZ e21 = new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();
        // vertex0：KF1相机坐标系3D点
        e21->setVertex( 0, dynamic_castg2o::OptimizableGraph::Vertex*(optimizer.vertex(id1)));
        // vertex1：待优化Sim3顶点S12
        e21->setVertex( 1, dynamic_castg2o::OptimizableGraph::Vertex*(optimizer.vertex(0)));
        // 设置观测测量值obs2
        e21->setMeasurement(obs2);
        // 获取关键点金字塔层级逆方差
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);
        // Huber鲁棒核
        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        // 保存边、原始索引、是否KF2内观测标记
        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
        vbIsInKF2.push_back(inKF2);
    }

    // 初始化优化器，准备迭代
    optimizer.initializeOptimization();
    // 第一轮优化，迭代5次，带鲁棒核
    optimizer.optimize(5);

    // 检查内点，剔除外点
    int nBad = 0;
    // 不在KF2真实观测内的坏边计数
    int nBadOutKF2 = 0;
    for (size_t i = 0; i < vpEdges12.size(); i++) {
        ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        // 边为空直接跳过
        if (!e12 || !e21) continue;
        // 任意一条边卡方残差大于阈值，判定为外点
        if (e12->chi2() > th2 || e21->chi2() > th2) {
            // 获取原始匹配索引
            size_t idx = vnIndexEdge[i];
            // 将匹配置空，剔除该匹配
            vpMatches1[idx] = static_cast<MapPoint*>(NULL);
            // 优化器移除两条边
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            // 置空容器内边指针，防止野指针
            vpEdges12[i] = static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i] = static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
            // 如果该点不是KF2真实观测，坏点计数+1
            if (!vbIsInKF2[i]) {
                nBadOutKF2++;
            }
            continue;
        }
        // 内点，移除鲁棒核，后续迭代使用纯最小二乘
        e12->setRobustKernel(0);
        e21->setRobustKernel(0);
    }

    // 根据外点数量决定迭代次数，存在外点迭代10次，无外点迭代5次
    int nMoreIterations;
    if (nBad > 0) nMoreIterations = 10;
    else nMoreIterations = 5;
    // 有效内点小于10，优化失败直接返回0
    if (nCorrespondences - nBad < 10) return 0;

    // 只使用筛选后的内点再次优化
    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    // 再次统计最终内点数量
    int nIn = 0;
    // 初始化7x7海森矩阵置零，Sim3共7维参数
    mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
    for (size_t i = 0; i < vpEdges12.size(); i++) {
        ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if (!e12 || !e21) continue;
        // 手动计算边残差
        e12->computeError();
        e21->computeError();
        // 残差超过阈值，剔除匹配
        if (e12->chi2() > th2 || e21->chi2() > th2) {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx] = static_cast<MapPoint*>(NULL);
        } else {
            // 有效内点计数+1
            nIn++;
        }
    }

    // 从优化器取出优化后的Sim3结果
    g2o::VertexSim3Expmap* vSim3_recov = static_castg2o::VertexSim3Expmap*(optimizer.vertex(0));
    g2oS12 = vSim3_recov->estimate();
    // 返回最终内点数量
    return nIn;
}

void Optimizer::LocalInertialBA(const shared_ptr& pKF, bool* pbStopFlag, const std::shared_ptr& pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit) {
    // 获取当前关键帧所属地图的智能指针
    std::shared_ptr pCurrentMap = pKF->GetMap();
    // 普通模式最大迭代次数
    int maxOpt = 10;
    // 普通模式每次initialize后迭代步数
    int opt_it = 10;
    // bLarge=true代表大场景局部BA，调参适配大规模场景
    if (bLarge) {
        maxOpt = 25;
        opt_it = 4;
    }
    // 滑动窗口大小，取地图总关键帧-2和maxOpt两者较小值
    const int Nd = std::min(static_cast(pCurrentMap->KeyFramesInMap()) - 2, maxOpt);
    // 当前关键帧id，作为窗口最大id标记
    const unsigned long maxKFid = pKF->mnId;
    // 存储需要优化的时序滑动窗口关键帧
    vector<shared_ptr> vpOptimizableKFs;
    // 获取和pKF共视的所有关键帧
    const auto vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
    // 存储额外可优化的共视关键帧（非时序滑动窗口）
    list<shared_ptr> lpOptVisKFs;
    // 预分配滑动窗口容器内存
    vpOptimizableKFs.reserve(Nd);
    // 将当前关键帧加入待优化列表
    vpOptimizableKFs.push_back(pKF);
    // mnBALocalForKF标记：标记该关键帧参与以pKF为主体的局部BA
    pKF->mnBALocalForKF = pKF->mnId;

    // 沿着mPrevKF向前回溯，构建时序滑动窗口
    for (int i = 1; i < Nd; i++) {
        // 如果上一关键帧存在
        if (vpOptimizableKFs.back()->mPrevKF) {
            // 把前一帧加入优化窗口
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            // 打上局部BA标记
            vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
        } else {
            // 没有前驱关键帧，停止回溯
            break;
        }
    }

    // 收集滑动窗口内所有关键帧观测到的局部地图点
    list<MapPoint*> lLocalMapPoints;
    for (auto const& pKF : vpOptimizableKFs) {
        // 获取该关键帧所有地图点匹配
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
        for (auto pMP : vpMPs) {
            if (pMP && !pMP->isBad()) {
                // 避免同一个地图点重复加入列表，判断BA标记
                if (pMP->mnBALocalForKF != pKF->mnId) {
                    lLocalMapPoints.push_back(pMP);
                    pMP->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }

    // 设置固定关键帧：滑动窗口最前一帧的前一帧，作为固定基准
    list<shared_ptr> lFixedKeyFrames;
    if (vpOptimizableKFs.back()->mPrevKF) {
        // 存在前驱，加入固定帧列表
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        // mnBAFixedForKF标记：标记该帧在本次BA中固定不优化
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
    } else {
        // 已经是地图第一帧，无前驱，把窗口最后一帧改为固定帧，从可优化列表移除
        vpOptimizableKFs.back()->mnBALocalForKF = 0;
        vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // 可优化的共视关键帧，maxCovKF=0，不加入额外共视帧
    const int maxCovKF = 0;
    for (auto const& pKFi : vpNeighsKFs) {
        if (lpOptVisKFs.size() >= maxCovKF) break;
        // 已经标记为可优化或者固定，跳过
        if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId) continue;
        pKFi->mnBALocalForKF = pKF->mnId;
        // 关键帧有效，并且属于当前地图
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
            lpOptVisKFs.push_back(pKFi);
            // 收集该共视帧观测到的地图点
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

    // 收集局部地图点观测到的其他关键帧，标记为固定帧，上限maxFixKF=200
    const int maxFixKF = 200;
    for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        // 获取该地图点所有观测 <关键帧,观测索引>
        map<shared_ptr, tuple<int, int>> observations = (lit)->GetObservations();
        for (map<shared_ptr, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
            shared_ptr pKFi = mit->first;
            // 该关键帧没有被标记为可优化、没有标记为固定
            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad()) {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        // 达到固定帧上限，停止收集
        if (lFixedKeyFrames.size() >= maxFixKF) break;
    }

    // bool bNonFixed = (lFixedKeyFrames.size() == 0);
    // 初始化g2o稀疏优化器
    g2o::SparseOptimizer optimizer;
    // 特征值求解器，用于BlockSolverX
    auto linearSolver = std::make_unique< g2o::LinearSolverEigeng2o::BlockSolverX::PoseMatrixType>();
    // LM优化算法，传入块求解器，转移求解器所有权
    auto solver = new g2o::OptimizationAlgorithmLevenberg( std::make_uniqueg2o::BlockSolverX(std::move(linearSolver)));
    // 大场景BA设置初始lambda，避免LM反复迭代寻找lambda
    if (bLarge) {
        solver->setUserLambdaInit( 1e-2); // to avoid iterating for finding optimal lambda
    } else {
        solver->setUserLambdaInit(1e0);
    }
    // 设置优化算法
    optimizer.setAlgorithm(solver);

    // 添加时序滑动窗口内可优化关键帧顶点
    // N = vpOptimizableKFs.size();
    for (auto pKFi : vpOptimizableKFs) {
        // 构造位姿顶点VertexPose，包含SE3位姿
        VertexPose VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        // 不固定，参与优化
        VP->setFixed(false);
        optimizer.addVertex(VP);
        // 如果该关键帧带IMU信息，增加速度、陀螺仪bias、加速度bias顶点
        if (pKFi->bImu) {
            // 速度顶点id构造：maxKFid + 3*id +1
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            // 陀螺仪零偏顶点
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            // 加速度计零偏顶点
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // 添加额外可优化共视关键帧顶点，只优化位姿，不优化IMU状态
    for (auto pKFi : lpOptVisKFs) {
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
    }

    // 添加固定关键帧顶点，位姿固定不优化
    for (auto pKFi : lFixedKeyFrames) {
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);
        // 固定帧如果有IMU，IMU状态同样固定
        // This should be done only for keyframe just before temporal window
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

    // 创建IMU惯性约束边容器
    int N = vpOptimizableKFs.size();
    // IMU预积分残差边
    vector<EdgeInertial*> vei(N, nullptr);
    // 陀螺仪随机游走残差边
    vector<EdgeGyroRW*> vegr(N, nullptr);
    // 加速度计随机游走残差边
    vector<EdgeAccRW*> vear(N, nullptr);
    for (int i = 0; i < N; i++) {
        auto pKFi = vpOptimizableKFs[i];
        // 当前帧没有前驱关键帧，无法构建IMU边，打印告警
        if (!pKFi->mPrevKF) {
            cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
            continue;
        }
        // 当前帧、前一帧都带IMU，并且存在预积分数据
        if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
            // 使用前一帧bias更新预积分内部bias
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            // 获取各个顶点指针：prevKF位姿、速度、gyro bias、acc bias；currKF位姿、速度、gyro bias、acc bias
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
            g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
            g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
            g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
            // 任意顶点为空，报错跳过
            if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
                cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2 << endl;
                continue;
            }
            // 构造IMU预积分残差边EdgeInertial
            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);
            // 绑定顶点：prevKF位姿、prev速度、prev gyro bias、prev acc bias、currKF位姿、curr速度
            vei[i]->setVertex(0, dynamic_castg2o::OptimizableGraph::Vertex*(VP1));
            vei[i]->setVertex(1, dynamic_castg2o::OptimizableGraph::Vertex*(VV1));
            vei[i]->setVertex(2, dynamic_castg2o::OptimizableGraph::Vertex*(VG1));
            vei[i]->setVertex(3, dynamic_castg2o::OptimizableGraph::Vertex*(VA1));
            vei[i]->setVertex(4, dynamic_castg2o::OptimizableGraph::Vertex*(VP2));
            vei[i]->setVertex(5, dynamic_castg2o::OptimizableGraph::Vertex*(VV2));
            // 窗口最后一条IMU边或者bRecInit，开启Huber鲁棒核，并且降低信息权重
            if (i == N - 1 || bRecInit) {
                // All inertial residuals are included without robust cost function, but
                // not that one linking the last optimizable keyframe inside of the
                // local window and the first fixed keyframe out. The information matrix
                // for this measurement is also downweighted. This is done to avoid
                // accumulating error due to fixing variables.
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                // 窗口最后一条IMU边信息矩阵乘以0.01降权，缓解固定帧带来的累积误差
                if (i == N - 1) vei[i]->setInformation(vei[i]->information() * 1e-2);
                rki->setDelta(sqrt(16.92));
            }
            // 将IMU残差边加入优化器
            optimizer.addEdge(vei[i]);

            // 陀螺仪随机游走残差边，约束前后帧gyro bias变化
            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0, VG1);
            vegr[i]->setVertex(1, VG2);
            // 从预积分协方差矩阵取出gyro随机游走部分，求逆作为信息矩阵
            Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9) .cast() .inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            // 加速度计随机游走残差边，约束前后帧acc bias变化
            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0, VA1);
            vear[i]->setVertex(1, VA2);
            Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12) .cast() .inverse();
            vear[i]->setInformation(InfoA);
            optimizer.addEdge(vear[i]);
        } else {
            cout << "ERROR building inertial edge" << endl;
        }
    }

    // 设置地图点顶点，预估边数量，预分配容器
    const int nExpectedSize = (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();
    // Mono单目边
    vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);
    vector<shared_ptr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);
    // Stereo双目边
    vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);
    vector<shared_ptr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    // Huber阈值，单目2自由度卡方0.05阈值5.991开根号
    const float thHuberMono = sqrt(5.991);
    // 单目卡方外点判断阈值
    const float chi2Mono2 = 5.991;
    // 双目3自由度卡方0.05阈值7.815开根号
    const float thHuberStereo = sqrt(7.815);
    // 双目卡方外点判断阈值
    const float chi2Stereo2 = 7.815;
    // 地图点顶点id偏移，避免和关键帧id冲突
    const unsigned long iniMPid = maxKFid * 5;
    // 统计每个关键帧视觉边数量
    map<int, int> mVisEdges;
    for (auto pKFi : vpOptimizableKFs) {
        mVisEdges[pKFi->mnId] = 0;
    }
    for (auto lit : lFixedKeyFrames) {
        mVisEdges[lit->mnId] = 0;
    }

    // 遍历所有局部地图点，添加地图点顶点与视觉投影边
    for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint* pMP = lit;
        // 创建地图点3D顶点
        g2o::VertexPointXYZ vPoint = new g2o::VertexPointXYZ();
        // 设置地图点世界坐标初始值，转double
        vPoint->setEstimate(pMP->GetWorldPos().cast());
        // 地图点顶点id
        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        // 设置该顶点可以被舒尔补边缘化，BA中地图点会被marginalize
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        // 获取该地图点全部观测
        auto const observations = pMP->GetObservations();

        // 遍历每一条观测，创建视觉投影残差边
        for (auto const& [pKFi, vObs] : observations) {
            // 该关键帧没有参与本次BA，跳过
            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) continue;
            // 关键帧有效，属于当前地图
            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                // 获取左图特征索引
                const int leftIndex = get<0>(vObs);
                cv::KeyPoint kpUn;
                // 左图观测，mvuRight[leftIndex]<0代表单目观测，没有右图匹配
                if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
                    mVisEdges[pKFi->mnId]++;
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;
                    // 构造单目投影边EdgeMono，0代表左相机
                    EdgeMono* e = new EdgeMono(0);
                    // vertex0：地图点3D顶点；vertex1：关键帧位姿顶点
                    e->setVertex(0, dynamic_castg2o::OptimizableGraph::Vertex*( optimizer.vertex(id)));
                    e->setVertex(1, dynamic_castg2o::OptimizableGraph::Vertex*( optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    // 获取相机不确定性，计算信息矩阵
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs);
                    const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
                    // Huber鲁棒核
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);
                    optimizer.addEdge(e);
                    // 保存边、对应关键帧、对应地图点，用于优化后外点剔除
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                } else if (leftIndex != -1) {
                    // 双目观测，存在右图匹配，观测包含u,v,ur
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    mVisEdges[pKFi->mnId]++;
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
                    // 双目投影边EdgeStereo
                    EdgeStereo* e = new EdgeStereo(0);
                    e->setVertex(0, dynamic_castg2o::OptimizableGraph::Vertex*( optimizer.vertex(id)));
                    e->setVertex(1, dynamic_castg2o::OptimizableGraph::Vertex*( optimizer.vertex(pKFi->mnId)));
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
                // 如果存在第二个相机（右目），处理右相机单目观测
                if (pKFi->mpCamera2) {
                    int rightIndex = get<1>(vObs);
                    if (rightIndex != -1) {
                        // 右目关键点索引，减去左图关键点总数偏移
                        rightIndex -= pKFi->NLeft;
                        mVisEdges[pKFi->mnId]++;
                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;
                        // EdgeMono参数1代表右相机
                        EdgeMono* e = new EdgeMono(1);
                        e->setVertex(0, dynamic_castg2o::OptimizableGraph::Vertex*( optimizer.vertex(id)));
                        e->setVertex(1, dynamic_castg2o::OptimizableGraph::Vertex*( optimizer.vertex(pKFi->mnId)));
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

    // cout << "Total map points: " << lLocalMapPoints.size() << endl;
    for (map<int, int>::iterator mit = mVisEdges.begin(), mend = mVisEdges.end(); mit != mend; mit++) {
        // AMM assert(mit->second>=3);
    }

    // 初始化优化器
    optimizer.initializeOptimization();
    // 计算全部激活边的残差
    optimizer.computeActiveErrors();
    // 获取优化前鲁棒卡方总残差
    float err = optimizer.activeRobustChi2();
    // 执行优化迭代opt_it次
    optimizer.optimize(opt_it);
    // 获取优化后总残差
    float err_end = optimizer.activeRobustChi2();
    // 如果外部停止标志有效，设置强制停止标志
    if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

    // 存储需要删除的观测对<关键帧，地图点>
    vector<pair<shared_ptr, MapPoint*>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // 检查单目观测外点
    // Mono
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        EdgeMono* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];
        // mTrackDepth较小代表刚生成的地图点，放宽卡方阈值
        bool bClose = pMP->mTrackDepth < 10.f;
        if (pMP->isBad()) continue;
        // 普通点残差>5.991，近点残差>1.5*5.991，或者深度为负，判定外点，准备删除观测
        if ((e->chi2() > chi2Mono2 && !bClose) || (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
            shared_ptr pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // 检查双目观测外点
    // Stereo
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];
        if (pMP->isBad()) continue;
        if (e->chi2() > chi2Stereo2) {
            shared_ptr pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // 获取地图更新互斥锁，删除外点观测
    unique_lock lock(pMap->mMutexMapUpdate);
    // TODO: Some convergence problems have been detected here
    // bGN)
    // 收敛判断：普通模式下，优化后残差没有下降或者出现nan，打印错误日志直接返回，不更新地图
    if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) {
        oslog::error("FAIL LOCAL‑INERTIAL BA!!!!");
        return;
    }
    // 存在外点观测，执行删除
    if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
            shared_ptr pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            // 关键帧删除该地图点匹配
            pKFi->EraseMapPointMatch(pMPi);
            // 地图点删除该关键帧观测
            pMPi->EraseObservation(pKFi);
        }
    }

    // 清空本次BA的标记，固定帧标记置0
    for (auto pKFi : lFixedKeyFrames) pKFi->mnBAFixedForKF = 0;

    // 恢复优化结果，写回关键帧位姿、速度、bias
    // Recover optimized data
    // Local temporal Keyframes
    N = vpOptimizableKFs.size();
    for (auto pKFi : vpOptimizableKFs) {
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        // 取出优化后的SE3位姿，转换为Sophus::SE3f，写回关键帧
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast(), VP->estimate().tcw[0].cast());
        pKFi->SetPose(Tcw);
        // 清除BA标记
        pKFi->mnBALocalForKF = 0;
        // IMU关键帧，恢复速度、bias
        if (pKFi->bImu) {
            VertexVelocity* VV = static_cast<VertexVelocity*>( optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            pKFi->SetVelocity(VV->estimate().cast());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>( optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
            VertexAccBias* VA = static_cast<VertexAccBias*>( optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            // 组合6维bias [gyro, acc]
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            // 将bias写回关键帧IMU bias成员
            pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
    }

    // 恢复可优化共视关键帧位姿
    // Local visual KeyFrame
    for (auto pKFi : lpOptVisKFs) {
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast(), VP->estimate().tcw[0].cast());
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF = 0;
    }

    // 恢复地图点世界坐标，更新法向和深度
    // Points
    for (auto pMP : lLocalMapPoints) {
        g2o::VertexPointXYZ* vPoint = static_castg2o::VertexPointXYZ*( optimizer.vertex(pMP->mnId + iniMPid + 1));
        pMP->SetWorldPos(vPoint->estimate().cast());
        pMP->UpdateNormalAndDepth();
    }
    // 地图修改计数器+1，标记地图发生修改
    pMap->IncreaseChangeIndex();
}

Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd& H,
                                        const int& start, const int& end) {
   // Goal
   // a  | ab | ac       a*  | 0 | ac*
   // ba | b  | bc  -->  0   | 0 | 0
   // ca | cb | c        ca* | 0 | c*
   /**
    * @brief 矩阵边缘化，舒尔补(Schur complement)，把[start,end]区间的变量块b消元
    * H 输入海森矩阵H=J^T*J
    * start 待边缘化块起始索引
    * end 待边缘化块结束索引
    * 返回边缘化之后的海森矩阵，b块置0，保留a,c块的舒尔补结果
    */

   // Size of block before block to marginalize
   const int a = start;                     ///< a块维度：待边缘化块b之前的变量维度
   // Size of block to marginalize
   const int b = end - start + 1;           ///< b块维度：需要被边缘化消去的变量块大小
   // Size of block after block to marginalize
   const int c = H.cols() - (end + 1);      ///< c块维度：待边缘化块b之后的变量维度

   // Reorder as follows:
   // a  | ab | ac       a  | ac | ab
   // ba | b  | bc  -->  ca | c  | cb
   // ca | cb | c        ba | bc | b
   /// 矩阵重排，把要边缘化的b块移动到矩阵右下角，方便做舒尔补计算
   Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());

   /// 拷贝左上角a块；拷贝a-b交叉块放到新矩阵右上角；拷贝b‑a交叉块放到新矩阵左下角
   if (a > 0) {
     Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
     Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
     Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
   }

   /// 拷贝a‑c、c‑a交叉块，a块与c块之间的关联项
   if (a > 0 && c > 0) {
     Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
     Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
   }

   /// 拷贝c块本身；拷贝c‑b、b‑c交叉块，c与待边缘化b块之间的关联
   if (c > 0) {
     Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
     Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
     Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
   }

   /// 将原始H中的b块拷贝到重排矩阵右下角，这是要求逆做舒尔补的子矩阵
   Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

   // Perform marginalization (Schur complement)
   /// SVD分解对b块求逆，防止矩阵奇异，小奇异值直接置零，数值稳定
   Eigen::JacobiSVD<Eigen::MatrixXd> svd(
       Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
   Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv =
       svd.singularValues();

   /// 对奇异值做阈值截断，小于1e‑6视为零空间，避免除以0
   for (int i = 0; i < b; ++i) {
     if (singularValues_inv(i) > 1e-6)
       singularValues_inv(i) = 1.0 / singularValues_inv(i);
     else
       singularValues_inv(i) = 0;
   }

   /// SVD伪逆：V * Σ⁻¹ * Uᵀ，得到b块的伪逆矩阵invHb
   Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() *
                           svd.matrixU().transpose();

   /// 舒尔补公式：H_ac* = H_ac − H_ab * Hb⁻¹ * H_ba，更新左上角a+c子块
   Hn.block(0, 0, a + c, a + c) =
       Hn.block(0, 0, a + c, a + c) -
       Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);

   /// 把已经边缘化消去的b块全部置零
   Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
   Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
   Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

   // Inverse reorder
   // a*  | ac* | 0       a*  | 0 | ac*
   // ca* | c*  | 0  -->  0   | 0 | 0
   // 0   | 0   | 0       ca* | 0 | c*
   /// 逆重排，恢复变量原始顺序，输出最终边缘化后的矩阵
   Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());

   /// 还原a块；还原a‑b交叉位置（此时值为0）；还原b‑a交叉位置（此时值为0）
   if (a > 0) {
     res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
     res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
     res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
   }

   /// 还原a‑c、c‑a交叉块，舒尔补之后的关联
   if (a > 0 && c > 0) {
     res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
     res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
   }

   /// 还原c块本体；还原c‑b、b‑c交叉块（值为0）
   if (c > 0) {
     res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
     res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
     res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
   }

   /// b块区域全部置零
   res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

   return res;
 }

/**
 * @brief 惯性优化函数1：同时优化重力方向Rwg、尺度scale、IMU零偏bg/ba，单目初始化阶段使用
 * @param pMap 地图指针
 * @param Rwg 输出：世界到IMU重力旋转矩阵
 * @param scale 输出：单目尺度因子
 * @param bg 输出：陀螺仪bias
 * @param ba 输出：加速度计bias
 * @param bMono true=单目模式，scale参与优化；false=双目，scale固定
 * @param covInertial 输出惯性协方差（本函数内部未实际填充）
 * @param bFixedVel true速度顶点固定；false速度参与优化
 * @param bGauss true使用高斯牛顿；false列文伯格马夸尔特
 * @param priorG 陀螺仪bias先验信息权重
 * @param priorA 加速度计bias先验信息权重
 */
void Optimizer::InertialOptimization(const std::shared_ptr<Map>& pMap,
                                      Eigen::Matrix3d& Rwg, double& scale,
                                      Eigen::Vector3d& bg, Eigen::Vector3d& ba,
                                      bool bMono, Eigen::MatrixXd& covInertial,
                                      bool bFixedVel, bool bGauss, float priorG,
                                      float priorA) {
   Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
   int its = 200;                                 ///< 最大迭代次数
   long unsigned int maxKFid = pMap->GetMaxKFid();///< 地图最大关键帧ID
   const auto vpKFs = pMap->GetAllKeyFrames();    ///< 获取全部关键帧

   // Setup optimizer
   /// 构造g2o稀疏优化器
   g2o::SparseOptimizer optimizer;
   /// 使用Eigen后端线性求解器，BlockSolverX，动态维度块求解器
   auto linearSolver = std::make_unique<
       g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
   /// 构造LM求解器，传入块求解器
   auto solver = new g2o::OptimizationAlgorithmLevenberg(
       std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

   /// 如果设置陀螺仪先验，设置LM初始lambda
   if (priorG != 0.f) solver->setUserLambdaInit(1e3);

   optimizer.setAlgorithm(solver);

   // Set KeyFrame vertices (fixed poses and optimizable velocities)
   /// 添加关键帧顶点：位姿固定，速度可优化
   for (auto pKFi : vpKFs) {
     if (pKFi->mnId > maxKFid) continue; ///< 只处理ID不超过maxKFid的关键帧

     /// 位姿顶点，位姿固定，视觉给出位姿，不优化位姿
     VertexPose* VP = new VertexPose(pKFi);
     VP->setId(pKFi->mnId);
     VP->setFixed(true);
     optimizer.addVertex(VP);

     /// 速度顶点，IMU速度状态
     VertexVelocity* VV = new VertexVelocity(pKFi);
     VV->setId(maxKFid + (pKFi->mnId) + 1);
     if (bFixedVel)
       VV->setFixed(true);
     else
       VV->setFixed(false);

     optimizer.addVertex(VV);
   }

   // Biases
   /// 陀螺仪零偏顶点
   VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
   VG->setId(maxKFid * 2 + 2);
   if (bFixedVel)
     VG->setFixed(true);
   else
     VG->setFixed(false);
   optimizer.addVertex(VG);

   /// 加速度计零偏顶点
   VertexAccBias* VA = new VertexAccBias(vpKFs.front());
   VA->setId(maxKFid * 2 + 3);
   if (bFixedVel)
     VA->setFixed(true);
   else
     VA->setFixed(false);

   optimizer.addVertex(VA);

   // prior acc bias
   Eigen::Vector3f bprior;
   bprior.setZero(); ///< bias先验，初始为0向量

   /// 加速度bias先验边
   EdgePriorAcc* epa = new EdgePriorAcc(bprior);
   epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
   double infoPriorA = priorA;
   epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
   optimizer.addEdge(epa);

   /// 陀螺仪bias先验边
   EdgePriorGyro* epg = new EdgePriorGyro(bprior);
   epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
   double infoPriorG = priorG;
   epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
   optimizer.addEdge(epg);

   // Gravity and scale
   /// 重力方向顶点，优化世界与IMU之间重力旋转
   VertexGDir* VGDir = new VertexGDir(Rwg);
   VGDir->setId(maxKFid * 2 + 4);
   VGDir->setFixed(false);
   optimizer.addVertex(VGDir);

   /// 尺度顶点，单目尺度，双目固定为1
   VertexScale* VS = new VertexScale(scale);
   VS->setId(maxKFid * 2 + 5);
   VS->setFixed(!bMono);  // Fixed for stereo case
   optimizer.addVertex(VS);

   // Graph edges
   // IMU links with gravity and scale
   vector<EdgeInertialGS*> vpei;  ///< IMU预积分边容器，带重力、尺度
   vpei.reserve(vpKFs.size());
   vector<pair<shared_ptr<KeyFrame>, shared_ptr<KeyFrame>>> vppUsedKF;
   vppUsedKF.reserve(vpKFs.size());
   // std::cout << "build optimization graph" << std::endl;

   /// 遍历关键帧，添加IMU预积分约束EdgeInertialGS
   for (auto pKFi : vpKFs) {
     if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
       if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
       if (!pKFi->mpImuPreintegrated)
         std::cout << "Not preintegrated measurement" << std::endl;

       /// 使用上一帧的bias更新预积分，bias变化需要重传播预积分
       pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());

       /// 获取各个顶点指针
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

       /// 空指针保护，顶点查找失败跳过这条边
       if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
         cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA
              << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS
              << endl;

         continue;
       }

       /// 实例化带重力、尺度的IMU预积分边
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

   // Compute error for different scales
   std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

   optimizer.setVerbose(false);
   optimizer.initializeOptimization();   ///< g2o初始化，复制estimate到cache
   optimizer.optimize(its);              ///< 执行LM迭代优化

   scale = VS->estimate();

   // Recover optimized data
   // Biases
   /// 取回优化之后bias顶点
   VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
   VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
   Vector6d vb;
   vb << VG->estimate(), VA->estimate();
   bg << VG->estimate();
   ba << VA->estimate();
   scale = VS->estimate();

   /// 组装IMU bias结构体，注意顺序：acc_bias(3), gyro_bias(3)
   IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
   Rwg = VGDir->estimate().Rwg;

   // Keyframes velocities and biases
   /// 将优化得到的速度、bias写回关键帧
   for (auto pKFi : vpKFs) {
     if (pKFi->mnId > maxKFid) continue;

     VertexVelocity* VV = static_cast<VertexVelocity*>(
         optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
     Eigen::Vector3d Vw = VV->estimate();  // Velocity is scaled after
     pKFi->SetVelocity(Vw.cast<float>());

     /// 如果bias变化大，设置新bias并且重新传播预积分
     if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
       pKFi->SetNewBias(b);
       if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
     } else {
       pKFi->SetNewBias(b);
     }
   }
 }

/**
 * @brief 惯性优化函数2：固定重力方向、固定尺度，仅优化IMU零偏bg ba，用于后续bias精细校准
 * @param pMap 地图
 * @param bg 输出陀螺仪bias
 * @param ba 输出加速度计bias
 * @param priorG 陀螺仪bias先验权重
 * @param priorA 加速度bias先验权重
 */
void Optimizer::InertialOptimization(const std::shared_ptr<Map>& pMap,
                                      Eigen::Vector3d& bg, Eigen::Vector3d& ba,
                                      float priorG, float priorA) {
   int its = 200;  // Check number of iterations
   long unsigned int maxKFid = pMap->GetMaxKFid();
   const vector<shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();

   // Setup optimizer
   g2o::SparseOptimizer optimizer;
   auto linearSolver = std::make_unique<
       g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
   auto solver = new g2o::OptimizationAlgorithmLevenberg(
       std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

   solver->setUserLambdaInit(1e3);
   optimizer.setAlgorithm(solver);

   // Set KeyFrame vertices (fixed poses and optimizable velocities)
   /// 位姿固定，速度参与优化
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

   // Biases
   VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
   VG->setId(maxKFid * 2 + 2);
   VG->setFixed(false);
   optimizer.addVertex(VG);

   VertexAccBias* VA = new VertexAccBias(vpKFs.front());
   VA->setId(maxKFid * 2 + 3);
   VA->setFixed(false);

   optimizer.addVertex(VA);

   // prior acc bias
   Eigen::Vector3f bprior;
   bprior.setZero();

   EdgePriorAcc* epa = new EdgePriorAcc(bprior);
   epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
   double infoPriorA = priorA;
   epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
   optimizer.addEdge(epa);

   EdgePriorGyro* epg = new EdgePriorGyro(bprior);
   epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
   double infoPriorG = priorG;
   epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
   optimizer.addEdge(epg);

   // Gravity and scale
   /// 重力方向固定为单位矩阵，不优化；尺度固定1.0
   VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
   VGDir->setId(maxKFid * 2 + 4);
   VGDir->setFixed(true);
   optimizer.addVertex(VGDir);

   VertexScale* VS = new VertexScale(1.0);
   VS->setId(maxKFid * 2 + 5);
   VS->setFixed(
       true);  // Fixed since scale is obtained from already well initialized map
   optimizer.addVertex(VS);

   // Graph edges
   // IMU links with gravity and scale
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

   // Compute error for different scales
   optimizer.setVerbose(false);
   optimizer.initializeOptimization();
   optimizer.optimize(its);

   // Recover optimized data
   // Biases
   VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
   VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
   Vector6d vb;
   vb << VG->estimate(), VA->estimate();
   bg << VG->estimate();
   ba << VA->estimate();

   IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

   // Keyframes velocities and biases
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
 * @brief 惯性优化函数3：固定位姿、速度、bias，只优化重力方向Rwg和单目尺度scale，Gauss‑Newton
 * @param pMap 地图
 * @param Rwg 输出重力旋转矩阵
 * @param scale 输出尺度
 */
void Optimizer::InertialOptimization(const std::shared_ptr<Map>& pMap,
                                      Eigen::Matrix3d& Rwg, double& scale) {
   int its = 10;                                  ///< 迭代次数少，只优化重力+尺度
   long unsigned int maxKFid = pMap->GetMaxKFid();
   const vector<shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();

   // Setup optimizer
   g2o::SparseOptimizer optimizer;
   auto linearSolver = std::make_unique<
       g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
   /// 使用高斯牛顿算法
   auto solver = new g2o::OptimizationAlgorithmGaussNewton(
       std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

   optimizer.setAlgorithm(solver);

   // Set KeyFrame vertices (all variables are fixed)
   /// 全部状态固定：位姿、速度、陀螺仪bias、加速度bias，仅重力+尺度放开
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

     // Vertex of fixed biases
     VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
     VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
     VG->setFixed(true);
     optimizer.addVertex(VG);
     VertexAccBias* VA = new VertexAccBias(vpKFs.front());
     VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
     VA->setFixed(true);
     optimizer.addVertex(VA);
   }

   // Gravity and scale
   VertexGDir* VGDir = new VertexGDir(Rwg);
   VGDir->setId(4 * (maxKFid + 1));
   VGDir->setFixed(false);
   optimizer.addVertex(VGDir);

   VertexScale* VS = new VertexScale(scale);
   VS->setId(4 * (maxKFid + 1) + 1);
   VS->setFixed(false);
   optimizer.addVertex(VS);

   // Graph edges
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
       /// Huber鲁棒核，抑制IMU外点
       g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
       ei->setRobustKernel(rk);
       rk->setDelta(1.f);
       optimizer.addEdge(ei);
     }
   }

   // Compute error for different scales
   optimizer.setVerbose(false);
   optimizer.initializeOptimization();
   optimizer.computeActiveErrors();
   // float err =   optimizer.activeRobustChi2();
   optimizer.optimize(its);
   optimizer.computeActiveErrors();
   // float err_end =   optimizer.activeRobustChi2();
   // Recover optimized data
   scale = VS->estimate();
   Rwg = VGDir->estimate().Rwg;
 }

/**
 * @brief 局部BA，Local Bundle Adjustment；优化部分关键帧位姿，地图点做边缘化，部分关键帧固定
 * @param pMainKF 触发本次局部BA的主关键帧
 * @param vpAdjustKF 需要优化位姿的关键帧集合
 * @param vpFixedKF 固定不动的关键帧集合
 * @param pbStopFlag 外部中断标志，可强制停止优化
 */
void Optimizer::LocalBundleAdjustment(const shared_ptr<KeyFrame>& pMainKF,
                                       vector<shared_ptr<KeyFrame>> vpAdjustKF,
                                       vector<shared_ptr<KeyFrame>> vpFixedKF,
                                       bool* pbStopFlag) {
   // bool bShowImages = false;
    vector<MapPoint*> vpMPs;    // 存储参与局部BA的所有地图点
    g2o::SparseOptimizer optimizer;  // g2o稀疏优化器对象
    // 构造线性求解器，使用Eigen实现，针对BlockSolver_6_3的位姿矩阵类型
    auto linearSolver = std::make_unique<
        g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    // 构造LM算法优化器，传入6自由度位姿+3自由度路标点的块求解器
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);   // 将求解算法设置给优化器
    optimizer.setVerbose(false);      // 关闭g2o内部打印输出

    // 如果外部传入停止标志位，设置给优化器，用于中途终止优化
    if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;                     // 记录关键帧最大ID，用于路标点顶点ID偏移
    set<shared_ptr<KeyFrame>> spKeyFrameBA;           // 参与BA的全部关键帧集合

    std::shared_ptr<Map> pCurrentMap = pMainKF->GetMap(); // 获取主关键帧所属地图

    // ===================== 设置固定不动的关键帧顶点 =====================
    int numInsertedPoints = 0; // 统计插入的地图点数量
    for (auto pKFi : vpFixedKF) { // 遍历需要固定位姿的关键帧
      // 跳过坏关键帧、不属于当前地图的关键帧
      if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
        Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map",
                           Verbose::VERBOSITY_NORMAL);
        continue;
      }

      pKFi->mnBALocalForMerge = pMainKF->mnId; // 标记该关键帧参与本次merge局部BA，标记为主关键帧ID

      g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap(); // SE3位姿顶点
      Sophus::SE3<float> Tcw = pKFi->GetPose(); // 获取关键帧相机到世界位姿Tcw(float)
      // 将Sophus::SE3<float>转换为g2o::SE3Quat(double)，设置顶点初始估计值
      vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                     Tcw.translation().cast<double>()));
      vSE3->setId(pKFi->mnId);   // 设置顶点ID等于关键帧ID
      vSE3->setFixed(true);      // 设置该顶点固定，优化时不更新位姿
      optimizer.addVertex(vSE3); // 将顶点加入g2o优化器

      // 更新最大关键帧ID
      if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

      set<MapPoint*> spViewMPs = pKFi->GetMapPoints(); // 获取该关键帧观测到的所有地图点
      for (MapPoint* pMPi : spViewMPs) {
        if (pMPi) {
          // 地图点有效，且属于当前地图，且没有被标记参与本次BA
          if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {
            if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
              vpMPs.push_back(pMPi);               // 加入BA地图点列表
              pMPi->mnBALocalForMerge = pMainKF->mnId; // 标记该地图点参与本次BA
              numInsertedPoints++;
            }
          }
        }
      }

      spKeyFrameBA.insert(pKFi); // 将该关键帧加入BA关键帧集合
    }

    // ===================== 设置待优化的关键帧顶点（位姿会被优化更新） =====================
    set<shared_ptr<KeyFrame>> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
    numInsertedPoints = 0; // 重置计数
    for (auto pKFi : vpAdjustKF) { // 遍历待优化关键帧
      // 跳过坏关键帧、不属于当前地图的关键帧
      if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) continue;

      pKFi->mnBALocalForMerge = pMainKF->mnId; // 标记参与本次merge局部BA

      g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap(); // SE3位姿顶点
      Sophus::SE3<float> Tcw = pKFi->GetPose(); // 获取关键帧位姿Tcw
      // sophus float转g2o SE3Quat double，设置初始估计
      vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                     Tcw.translation().cast<double>()));
      vSE3->setId(pKFi->mnId);
      optimizer.addVertex(vSE3); // 默认不setFixed，即待优化

      if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId; // 更新最大KF ID

      set<MapPoint*> spViewMPs = pKFi->GetMapPoints(); // 获取该关键帧观测的地图点
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

      spKeyFrameBA.insert(pKFi); // 加入BA关键帧集合
    }

    // 预估边的总数量，用于预分配容器内存，避免频繁realloc
    const int nExpectedSize =
        (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono; // 单目投影边集合
    vpEdgesMono.reserve(nExpectedSize);

    vector<shared_ptr<KeyFrame>> vpEdgeKFMono;         // 单目边对应的关键帧
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;              // 单目边对应的地图点
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo; // 双目/RGBD投影边集合
    vpEdgesStereo.reserve(nExpectedSize);

    vector<shared_ptr<KeyFrame>> vpEdgeKFStereo;        // 双目边对应的关键帧
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;             // 双目边对应的地图点
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuber2D = sqrt(5.99);   // 2自由度卡方0.95阈值5.99，huber核delta
    const float thHuber3D = sqrt(7.815);  // 3自由度卡方0.95阈值7.815，huber核delta

    // ===================== 创建地图点顶点 + 添加视觉投影边 =====================
    map<shared_ptr<KeyFrame>, int> mpObsKFs;      // 统计每个关键帧观测边数量
    map<shared_ptr<KeyFrame>, int> mpObsFinalKFs;
    map<MapPoint*, int> mpObsMPs;                 // 统计每个地图点观测次数
    for (auto pMPi : vpMPs) { // 遍历所有参与BA的地图点
      if (pMPi->isBad()) continue; // 跳过坏点

      g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ(); // 3D路标点顶点
      vPoint->setEstimate(pMPi->GetWorldPos().cast<double>()); // 设置初始世界坐标(double)
      const int id = pMPi->mnId + maxKFid + 1; // 路标点ID偏移，和关键帧ID不冲突
      vPoint->setId(id);
      vPoint->setMarginalized(true); // 路标点做边缘化，BA标准操作
      optimizer.addVertex(vPoint);   // 添加路标点顶点到优化器

      // 获取该地图点全部观测：key=关键帧，value=tuple<特征索引, 其他>
      const map<shared_ptr<KeyFrame>, tuple<int, int>> observations =
          pMPi->GetObservations();
      int nEdges = 0;
      // 遍历该地图点所有观测，构建投影边
      for (auto const& [pKF, tObs] : observations) {
        // 过滤条件：坏KF、KF ID超过最大KFID、未标记参与本次BA、该KF实际不持有该地图点，跳过
        if (pKF->isBad() || pKF->mnId > maxKFid ||
            pKF->mnBALocalForMerge != pMainKF->mnId ||
            !pKF->GetMapPoint(get<0>(tObs)))
          continue;

        nEdges++;

        const cv::KeyPoint& kpUn = pKF->mvKeysUn[get<0>(tObs)]; // 获取去畸变关键点

        // ---------- 单目观测：右目值mvuRight小于0代表单目 ----------
        if (pKF->mvuRight[get<0>(tObs)] < 0) {
          mpObsMPs[pMPi]++;
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y; // 2D像素观测

          ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

          // vertex0:路标点顶点；vertex1:关键帧位姿顶点
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs); // 设置观测值

          // 信息矩阵：根据特征金字塔层级设置逆方差
          const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber; // Huber鲁棒核
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D); // 设置huber阈值

          e->pCamera = pKF->mpCamera; // 传入相机模型指针，用于投影计算残差
          optimizer.addEdge(e);      // 将边加入优化器

          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKF);
          vpMapPointEdgeMono.push_back(pMPi);

          mpObsKFs[pKF]++;
        } else {
          // ---------- 双目 / RGBD观测，拥有右目匹配点 ----------
          mpObsMPs[pMPi] += 2;
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKF->mvuRight[get<0>(tObs)]; // 右目u坐标
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur; // 观测 (ul, vl, ur)

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

          // 给双目边设置相机内参和基线bf
          e->fx = pKF->fx;
          e->fy = pKF->fy;
          e->cx = pKF->cx;
          e->cy = pKF->cy;
          e->bf = pKF->mbf;

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKF);
          vpMapPointEdgeStereo.push_back(pMPi);

          mpObsKFs[pKF]++;
        }
      }
    }

    // 如果外部停止标志被置true，直接退出，不执行优化
    if (pbStopFlag && *pbStopFlag) return;

    optimizer.initializeOptimization(); // g2o初始化优化结构
    optimizer.optimize(5);              // 第一轮优化，迭代5次，带Huber鲁棒核

    bool bDoMore = true;
    if (pbStopFlag && *pbStopFlag) bDoMore = false; // 如果中途停止，不再做后续优化

    map<unsigned long int, int> mWrongObsKF;
    if (bDoMore) {
      // 第一轮优化结束，检测外点：chi2超过阈值或者深度为负，标记边level=1，下一轮不参与优化
      int badMonoMP = 0, badStereoMP = 0;
      for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad()) continue;

        // 单目卡方阈值5.991，同时检查投影深度是否为正
        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
          e->setLevel(1); // level=1的边不参与优化
          badMonoMP++;
        }
        e->setRobustKernel(0); // 移除鲁棒核，第二轮优化使用最小二乘
      }

      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad()) continue;

        // 双目卡方阈值7.815，检查深度是否为正
        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
          e->setLevel(1);
          badStereoMP++;
        }

        e->setRobustKernel(0); // 去掉鲁棒核
      }
      Verbose::PrintMess("[BA]: First optimization(Huber), there are " +
                             to_string(badMonoMP) + " monocular and " +
                             to_string(badStereoMP) + " stereo bad edges",
                         Verbose::VERBOSITY_DEBUG);

      optimizer.initializeOptimization(0); // 仅激活level=0的边
      optimizer.optimize(10);              // 第二轮优化迭代10次，无鲁棒核
    }

    vector<pair<shared_ptr<KeyFrame>, MapPoint*>> vToErase; // 需要删除的观测对<KF,MP>
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
    set<MapPoint*> spErasedMPs;
    set<shared_ptr<KeyFrame>> spErasedKFs;

    // 第二轮优化完成，再次检测外点，准备真正删除观测
    int badMonoMP = 0, badStereoMP = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      MapPoint* pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        const shared_ptr<KeyFrame> pKFi = vpEdgeKFMono[i];
        vToErase.push_back(make_pair(pKFi, pMP)); // 记录该观测需要删除
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

    // 获取地图更新互斥锁，修改地图数据必须加锁
    unique_lock<mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

    if (!vToErase.empty()) {
      for (size_t i = 0; i < vToErase.size(); i++) {
        shared_ptr<KeyFrame> pKFi = vToErase[i].first;
        MapPoint* pMPi = vToErase[i].second;
        pKFi->EraseMapPointMatch(pMPi); // 关键帧删除该地图点匹配
        pMPi->EraseObservation(pKFi);   // 地图点删除该关键帧观测
      }
    }
    for (auto const pMPi : vpMPs) {
      if (pMPi->isBad()) continue;

      const map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations =
          pMPi->GetObservations();
      for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::const_iterator mit =
               observations.begin();
           mit != observations.end(); mit++) {
        std::shared_ptr<KeyFrame> pKF = mit->first;
        // 过滤条件
        if (pKF->isBad() || pKF->mnId > maxKFid ||
            pKF->mnBALocalForKF != pMainKF->mnId ||
            !pKF->GetMapPoint(get<0>(mit->second)))
          continue;

        if (pKF->mvuRight[get<0>(mit->second)] < 0) {
          // Monocular
          mpObsFinalKFs[pKF]++;
        } else {
          // RGBD or Stereo
          mpObsFinalKFs[pKF]++;
        }
      }
    }

    // ===================== 从g2o优化器回写优化后的位姿到关键帧对象 =====================
    for (auto const& pKFi : vpAdjustKF) {
      if (pKFi->isBad()) continue;

      // 根据关键帧ID取出g2o中的SE3顶点
      g2o::VertexSE3Expmap* vSE3 =
          static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
      g2o::SE3Quat SE3quat = vSE3->estimate();
      // g2o double转Sophus::SE3<float>，得到Tcw
      Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                       SE3quat.translation().cast<float>());

      int numMonoBadPoints = 0, numMonoOptPoints = 0;
      int numStereoBadPoints = 0, numStereoOptPoints = 0;
      vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;
      vector<MapPoint*> vpMonoMPsBad, vpStereoMPsBad;

      // 遍历单目边，统计当前关键帧对应的内外点
      for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];
        std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i];

        if (pKFi != pKFedge) {
          continue;
        }

        if (pMP->isBad()) continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
          numMonoBadPoints++;
          vpMonoMPsBad.push_back(pMP);
        } else {
          numMonoOptPoints++;
          vpMonoMPsOpt.push_back(pMP);
        }
      }

      // 遍历双目边，统计当前关键帧对应的内外点
      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];
        const std::shared_ptr<KeyFrame>& pKFedge = vpEdgeKFMono[i];

        if (pKFi != pKFedge) {
          continue;
        }

        if (pMP->isBad()) continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
          numStereoBadPoints++;
          vpStereoMPsBad.push_back(pMP);
        } else {
          numStereoOptPoints++;
          vpStereoMPsOpt.push_back(pMP);
        }
      }

      pKFi->SetPose(Tiw); // 将优化后的位姿设置回关键帧
    }

    // ===================== 回写优化后的地图点3D坐标 =====================
    for (MapPoint* pMPi : vpMPs) {
      if (pMPi->isBad()) continue;

      // 根据偏移ID取出路标点顶点
      g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
          optimizer.vertex(pMPi->mnId + maxKFid + 1));
      pMPi->SetWorldPos(vPoint->estimate().cast<float>()); // 更新世界坐标
      pMPi->UpdateNormalAndDepth(); // 更新地图点法向、观测深度
    }
 }

 void Optimizer::MergeInertialBA(const std::shared_ptr<KeyFrame>& pCurrKF,
                                 const std::shared_ptr<KeyFrame>& pMergeKF,
                                 bool* pbStopFlag,
                                 const std::shared_ptr<Map>& pMap,
                                 LoopClosing::KeyFrameAndPose& corrPoses) {
   const int Nd = 6;   // 滑动窗口半长，用于选取参与BA的关键帧数量
   const unsigned long maxKFid = pCurrKF->mnId;    // 当前关键帧ID，用作g2o顶点ID偏移基准

   vector<std::shared_ptr<KeyFrame>> vpOptimizableKFs;
   vpOptimizableKFs.reserve(2 * Nd);    // 可优化关键帧容器，预分配2*Nd容量

   // For cov KFS, inertial parameters are not optimized
   const int maxCovKF = 30;   // 协视关键帧最大数量，这类帧位姿参与优化，IMU参数不参与
   vector<std::shared_ptr<KeyFrame>> vpOptimizableCovKFs;
   vpOptimizableCovKFs.reserve(maxCovKF);    // 协视关键帧容器，预分配容量

   // Add sliding window for current KF
   vpOptimizableKFs.push_back(pCurrKF);   // 将当前关键帧加入可优化列表
   pCurrKF->mnBALocalForKF = pCurrKF->mnId;   // 标记该关键帧参与以pCurrKF为基准的局部BA
   for (int i = 1; i < Nd; i++) {   // 向前遍历，依次取前序关键帧构建滑动窗口
     if (vpOptimizableKFs.back()->mPrevKF) {   // 如果当前帧存在上一关键帧
       vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);   // 加入可优化列表
       vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;   // 打上本次BA标记
     } else {
       break;   // 没有前序关键帧，终止循环
     }
   }

   list<std::shared_ptr<KeyFrame>> lFixedKeyFrames;   // 固定位姿的关键帧列表，位姿不参与优化，作为约束
   if (vpOptimizableKFs.back()->mPrevKF) {   // 滑动窗口最末尾帧还存在前驱关键帧
     vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);   // 前驱帧加入协视帧集合
     vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;  // 打上BA标记
   } else {   // 已经到达地图最开头，没有更早关键帧
     vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());   // 将当前末尾帧放入协视帧
     vpOptimizableKFs.pop_back();   // 从可优化集合移除
   }

   // Add temporal neighbours to merge KF (previous and next KFs)
   vpOptimizableKFs.push_back(pMergeKF);   // 将回环匹配待合并关键帧加入可优化集合
   pMergeKF->mnBALocalForKF = pCurrKF->mnId;   // 打上本次BA标记

   // Previous KFs
   for (int i = 1; i < (Nd / 2); i++) {   // 向前取mergeKF的历史关键帧，取Nd/2个
     if (vpOptimizableKFs.back()->mPrevKF) {
       vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
       vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
     } else {
       break;
     }
   }

   // We fix just once the old map
   if (vpOptimizableKFs.back()->mPrevKF) {   // 窗口最末尾帧还有前驱帧
     lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);   // 前驱帧设置为固定帧
     vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;  // 标记为BA固定帧
   } else {   // 已经到地图起点
     vpOptimizableKFs.back()->mnBALocalForKF = 0;   // 清除局部BA标记
     vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;   // 设置为固定帧标记
     lFixedKeyFrames.push_back(vpOptimizableKFs.back());   // 加入固定帧列表
     vpOptimizableKFs.pop_back();   // 移出可优化集合
   }

   // Next KFs
   if (pMergeKF->mNextKF) {   // 如果mergeKF存在后继关键帧
     vpOptimizableKFs.push_back(pMergeKF->mNextKF);   // 后继帧加入可优化集合
     vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
   }

   while (vpOptimizableKFs.size() < (2 * Nd)) {   // 持续向后取后继帧，凑够2*Nd个可优化关键帧
     if (vpOptimizableKFs.back()->mNextKF) {
       vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
       vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
     } else {
       break;   // 没有后继帧，退出循环
     }
   }

   int N = vpOptimizableKFs.size();   // 获取最终可优化关键帧数量

   // Optimizable points seen by optimizable keyframes
   list<MapPoint*> lLocalMapPoints;   // 局部地图点链表，存放参与BA的3D点
   map<MapPoint*, int> mLocalObs;     // map记录每个地图点被多少局部关键帧观测到
   for (int i = 0; i < N; i++) {   // 遍历全部可优化关键帧
     vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();   // 获取该帧所有匹配地图点
     for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
          vit != vend; vit++) {
       // Using mnBALocalForKF we avoid redundance here, one MP can not be added
       // several times to lLocalMapPoints
       MapPoint* pMP = *vit;
       if (pMP) {   // 地图点指针非空
         if (!pMP->isBad()) {   // 地图点未被标记为坏点
           if (pMP->mnBALocalForKF != pCurrKF->mnId) {   // 该点还未加入本次BA
             mLocalObs[pMP] = 1;   // 观测计数初始化为1
             lLocalMapPoints.push_back(pMP);   // 加入局部地图点列表
             pMP->mnBALocalForKF = pCurrKF->mnId;   // 标记该点参与本次BA
           } else {
             mLocalObs[pMP]++;   // 已经存在，观测计数+1
           }
         }
       }
     }
   }

   std::vector<std::pair<MapPoint*, int>> pairs;
   pairs.reserve(mLocalObs.size());
   for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
     pairs.push_back(*itr);
   sort(pairs.begin(), pairs.end(), sortByVal);   // 按照观测次数对地图点排序

   // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
   // Keyframes
   int i = 0;
   for (vector<pair<MapPoint*, int>>::iterator lit = pairs.begin(),
                                               lend = pairs.end();
        lit != lend; lit++, i++) {   // 遍历按观测数排序的地图点
     map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations =
         lit->first->GetObservations();   // 获取该地图点全部观测（关键帧，特征索引）

     if (i >= maxCovKF) break;   // 协视关键帧数量到达上限，停止选取

     for (auto mit : observations) {   // 遍历该地图点的每一个观测关键帧
       auto pKFi = mit.first;

       if (pKFi->mnBALocalForKF != pCurrKF->mnId &&
           pKFi->mnBAFixedForKF != pCurrKF->mnId) {   // 该帧既不是可优化帧，也不是固定帧
         // If optimizable or already included...
         pKFi->mnBALocalForKF = pCurrKF->mnId;   // 打上BA标记
         if (!pKFi->isBad()) {
           vpOptimizableCovKFs.push_back(pKFi);   // 加入协视关键帧集合
           break;   // 每个地图点只取一个外部协视帧
         }
       }
     }
   }

   g2o::SparseOptimizer optimizer;   // g2o稀疏优化器对象
   auto linearSolver = std::make_unique<
       g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();   // Eigen实现的线性求解器
   auto solver = new g2o::OptimizationAlgorithmLevenberg(
       std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));   // LM列文伯格‑马夸尔特求解器，BlockSolverX处理动态维度

   solver->setUserLambdaInit(1e3);   // 设置LM初始lambda值

   optimizer.setAlgorithm(solver);   // 为优化器配置求解算法
   optimizer.setVerbose(false);   // 关闭g2o内部打印输出

   // Set Local KeyFrame vertices
   N = vpOptimizableKFs.size();   // 重新赋值，构建顶点
   for (auto pKFi : vpOptimizableKFs) {   // 遍历所有可优化关键帧
     VertexPose* VP = new VertexPose(pKFi);   // 创建位姿顶点，封装关键帧位姿
     VP->setId(pKFi->mnId);   // 设置g2o顶点ID等于关键帧ID
     VP->setFixed(false);   // 位姿允许优化，不固定
     optimizer.addVertex(VP);   // 将顶点加入优化器图

     if (pKFi->bImu) {   // 如果该关键帧具备IMU数据
       VertexVelocity* VV = new VertexVelocity(pKFi);   // 速度顶点
       VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);   // 构造不与位姿冲突的顶点ID
       VV->setFixed(false);
       optimizer.addVertex(VV);
       VertexGyroBias* VG = new VertexGyroBias(pKFi);   // 陀螺零偏顶点
       VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
       VG->setFixed(false);
       optimizer.addVertex(VG);
       VertexAccBias* VA = new VertexAccBias(pKFi);   // 加速度计零偏顶点
       VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
       VA->setFixed(false);
       optimizer.addVertex(VA);
     }
   }

   // Set Local cov keyframes vertices
   int Ncov = vpOptimizableCovKFs.size();
   for (auto pKFi : vpOptimizableCovKFs) {   // 协视关键帧：位姿可优化，IMU状态同样构建顶点
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

   // Set Fixed KeyFrame vertices
   for (auto pKFi : lFixedKeyFrames) {   // 固定关键帧，所有状态全部固定不参与优化
     VertexPose* VP = new VertexPose(pKFi);
     VP->setId(pKFi->mnId);
     VP->setFixed(true);   // 位姿固定
     optimizer.addVertex(VP);

     if (pKFi->bImu) {
       VertexVelocity* VV = new VertexVelocity(pKFi);
       VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
       VV->setFixed(true);   // 速度固定
       optimizer.addVertex(VV);
       VertexGyroBias* VG = new VertexGyroBias(pKFi);
       VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
       VG->setFixed(true);   // 陀螺bias固定
       optimizer.addVertex(VG);
       VertexAccBias* VA = new VertexAccBias(pKFi);
       VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
       VA->setFixed(true);   // 加速度bias固定
       optimizer.addVertex(VA);
     }
   }

   // Create intertial constraints
   vector<EdgeInertial*> vei(N, nullptr);     // IMU预积分边容器
   vector<EdgeGyroRW*> vegr(N, nullptr);     // 陀螺bias随机游走边
   vector<EdgeAccRW*> vear(N, nullptr);       // 加速度bias随机游走边
   for (int i = 0; i < N; i++) {
     // cout << "inserting inertial edge " << i << endl;
     std::shared_ptr<KeyFrame> pKFi = vpOptimizableKFs[i];

     if (!pKFi->mPrevKF) {   // 当前关键帧没有前驱，无法构建IMU约束
       Verbose::PrintMess("NO INERTIAL LINK TO PREVIOUS FRAME!!!!",
                          Verbose::VERBOSITY_NORMAL);
       continue;
     }
     if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {   // 当前帧、前帧都有IMU，并且存在预积分对象
       pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());   // 使用前一帧bias更新预积分
       g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);   // 前帧位姿顶点
       g2o::HyperGraph::Vertex* VV1 =
           optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);   // 前帧速度顶点
       g2o::HyperGraph::Vertex* VG1 =
           optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);   // 前帧陀螺bias
       g2o::HyperGraph::Vertex* VA1 =
           optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);   // 前帧加速度bias
       g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);   // 当前帧位姿
       g2o::HyperGraph::Vertex* VV2 =
           optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);   // 当前帧速度
       g2o::HyperGraph::Vertex* VG2 =
           optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);   // 当前帧陀螺bias
       g2o::HyperGraph::Vertex* VA2 =
           optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);   // 当前帧加速度bias

       if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {   // 检查顶点是否全部有效
         cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
              << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
              << endl;
         continue;
       }

       vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);   // 实例化IMU预积分边

       vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
       vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
       vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
       vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
       vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
       vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

       // TODO Uncomment
       g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;   // Huber鲁棒核，抑制IMU异常
       vei[i]->setRobustKernel(rki);
       rki->setDelta(sqrt(16.92));
       optimizer.addEdge(vei[i]);   // IMU边加入图优化

       vegr[i] = new EdgeGyroRW();   // 陀螺bias随机游走边
       vegr[i]->setVertex(0, VG1);
       vegr[i]->setVertex(1, VG2);
       Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                   .cast<double>()
                                   .inverse();   // 从预积分协方差矩阵取陀螺bias信息矩阵
       vegr[i]->setInformation(InfoG);
       optimizer.addEdge(vegr[i]);

       vear[i] = new EdgeAccRW();   // 加速度bias随机游走边
       vear[i]->setVertex(0, VA1);
       vear[i]->setVertex(1, VA2);
       Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                   .cast<double>()
                                   .inverse();   // 从预积分协方差取加速度bias信息矩阵
       vear[i]->setInformation(InfoA);
       optimizer.addEdge(vear[i]);
     } else {
       Verbose::PrintMess("ERROR building inertial edge",
                          Verbose::VERBOSITY_NORMAL);
     }
   }

   Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);

   // Set MapPoint vertices
   const int nExpectedSize =
       (N + Ncov + lFixedKeyFrames.size()) * lLocalMapPoints.size();   // 预估视觉边数量，用于预分配内存

   // Mono
   vector<EdgeMono*> vpEdgesMono;
   vpEdgesMono.reserve(nExpectedSize);   // 单目视觉边容器

   vector<std::shared_ptr<KeyFrame>> vpEdgeKFMono;
   vpEdgeKFMono.reserve(nExpectedSize);   // 保存单目边对应的关键帧指针

   vector<MapPoint*> vpMapPointEdgeMono;
   vpMapPointEdgeMono.reserve(nExpectedSize);   // 保存单目边对应的地图点指针

   // Stereo
   vector<EdgeStereo*> vpEdgesStereo;
   vpEdgesStereo.reserve(nExpectedSize);   // 双目视觉边容器

   vector<std::shared_ptr<KeyFrame>> vpEdgeKFStereo;
   vpEdgeKFStereo.reserve(nExpectedSize);   // 双目边对应关键帧

   vector<MapPoint*> vpMapPointEdgeStereo;
   vpMapPointEdgeStereo.reserve(nExpectedSize);   // 双目边对应地图点

   const float thHuberMono = sqrt(5.991);   // 单目Huber阈值，卡方2自由度95%置信
   const float chi2Mono2 = 5.991;           // 单目卡方阈值，用于外点剔除
   const float thHuberStereo = sqrt(7.815); // 双目Huber阈值，卡方3自由度95%置信
   const float chi2Stereo2 = 7.815;          // 双目卡方阈值，外点剔除

   const unsigned long iniMPid = maxKFid * 5;   // 地图点顶点ID偏移，防止和关键帧ID冲突

   for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                  lend = lLocalMapPoints.end();
        lit != lend; lit++) {   // 遍历所有局部地图点
     MapPoint* pMP = *lit;
     if (!pMP) continue;

     g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();   // 3D点顶点
     vPoint->setEstimate(pMP->GetWorldPos().cast<double>());   // 设置初始世界坐标
     unsigned long id = pMP->mnId + iniMPid + 1;   // 生成地图点g2o顶点ID
     vPoint->setId(id);
     vPoint->setMarginalized(true);   // 开启边缘化，BA中对3D点进行舒尔补消元
     optimizer.addVertex(vPoint);

     const map<std::shared_ptr<KeyFrame>, tuple<int, int>> observations =
         pMP->GetObservations();   // 获取该地图点全部观测

     // Create visual constraints
     for (map<std::shared_ptr<KeyFrame>, tuple<int, int>>::const_iterator
              mit = observations.begin(),
              mend = observations.end();
          mit != mend; mit++) {   // 遍历每一条观测
       std::shared_ptr<KeyFrame> pKFi = mit->first;

       if (!pKFi) continue;

       if ((pKFi->mnBALocalForKF != pCurrKF->mnId) &&
           (pKFi->mnBAFixedForKF != pCurrKF->mnId))
         continue;   // 只处理参与本次BA（可优化/固定）的关键帧观测

       if (pKFi->mnId > maxKFid) {
         continue;   // ID超过当前基准，跳过
       }

       if (optimizer.vertex(id) == NULL || optimizer.vertex(pKFi->mnId) == NULL)
         continue;   // 检查顶点是否成功加入优化器

       if (!pKFi->isBad()) {
         const cv::KeyPoint& kpUn = pKFi->mvKeysUn[get<0>(mit->second)];   // 获取去畸变关键点

         if (pKFi->mvuRight[get<0>(mit->second)] < 0) {   // 右目无效，单目观测
           // Monocular observation
           Eigen::Matrix<double, 2, 1> obs;
           obs << kpUn.pt.x, kpUn.pt.y;

           EdgeMono* e = new EdgeMono();   // 创建单目重投影误差边
           e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(id)));      // 3D点顶点
           e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(pKFi->mnId))); // 关键帧位姿顶点
           e->setMeasurement(obs);   // 设置图像2D观测
           const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];   // 根据金字塔层级获取信息
           e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

           g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
           e->setRobustKernel(rk);
           rk->setDelta(thHuberMono);
           optimizer.addEdge(e);
           vpEdgesMono.push_back(e);
           vpEdgeKFMono.push_back(pKFi);
           vpMapPointEdgeMono.push_back(pMP);
         } else {   // 存在右目匹配，双目观测
           // stereo observation
           const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
           Eigen::Matrix<double, 3, 1> obs;
           obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

           EdgeStereo* e = new EdgeStereo();   // 双目重投影边，观测u,v,ur

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

   if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);   // 设置外部停止标志，用于多线程终止优化
   if (pbStopFlag)
     if (*pbStopFlag) return;   // 如果外部要求停止，直接返回不执行优化

   optimizer.initializeOptimization();   // g2o初始化图结构
   optimizer.optimize(8);   // 执行8轮LM迭代优化

   vector<pair<std::shared_ptr<KeyFrame>, MapPoint*>> vToErase;
   vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());   // 存储需要剔除的(关键帧，地图点)观测对

   // Check inlier observations
   // Mono
   for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
     EdgeMono* e = vpEdgesMono[i];
     MapPoint* pMP = vpMapPointEdgeMono[i];

     if (pMP->isBad()) continue;

     if (e->chi2() > chi2Mono2) {   // 重投影卡方大于阈值，判定外点
       std::shared_ptr<KeyFrame> pKFi = vpEdgeKFMono[i];
       vToErase.push_back(make_pair(pKFi, pMP));
     }
   }

   // Stereo
   for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
     EdgeStereo* e = vpEdgesStereo[i];
     MapPoint* pMP = vpMapPointEdgeStereo[i];

     if (pMP->isBad()) continue;

     if (e->chi2() > chi2Stereo2) {   // 双目重投影卡方超标，判定外点
       std::shared_ptr<KeyFrame> pKFi = vpEdgeKFStereo[i];
       vToErase.push_back(make_pair(pKFi, pMP));
     }
   }

   // Get Map Mutex and erase outliers
   unique_lock<mutex> lock(pMap->mMutexMapUpdate);   // 加地图更新互斥锁，防止多线程冲突
   if (!vToErase.empty()) {
     for (size_t i = 0; i < vToErase.size(); i++) {
       std::shared_ptr<KeyFrame> pKFi = vToErase[i].first;
       MapPoint* pMPi = vToErase[i].second;
       pKFi->EraseMapPointMatch(pMPi);   // 关键帧删除该地图点匹配
       pMPi->EraseObservation(pKFi);     // 地图点删除该关键帧观测
     }
   }

   // Recover optimized data
   // Keyframes
   for (auto pKFi : vpOptimizableKFs) {   // 从g2o顶点取回优化后的位姿、速度、bias，写回关键帧对象
     VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
     Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                      VP->estimate().tcw[0].cast<float>());
     pKFi->SetPose(Tcw);   // 更新相机到世界位姿Tcw

     Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
     g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
     corrPoses[pKFi] = g2oSiw;   // 将优化后位姿存入输出map，供回环修正使用

     if (pKFi->bImu) {
       VertexVelocity* VV = static_cast<VertexVelocity*>(
           optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
       pKFi->SetVelocity(VV->estimate().cast<float>());   // 更新速度
       VertexGyroBias* VG = static_cast<VertexGyroBias*>(
           optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
       VertexAccBias* VA = static_cast<VertexAccBias*>(
           optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
       Vector6d b;
       b << VG->estimate(), VA->estimate();
       pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));   // 更新IMU零偏
     }
   }

   for (auto pKFi : vpOptimizableCovKFs) {   // 协视关键帧同样取回优化状态
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

   // Points
   for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                  lend = lLocalMapPoints.end();
        lit != lend; lit++) {   // 取回优化后3D点坐标，更新地图点
     MapPoint* pMP = *lit;
     g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
         optimizer.vertex(pMP->mnId + iniMPid + 1));
     pMP->SetWorldPos(vPoint->estimate().cast<float>());   // 设置优化后世界坐标
     pMP->UpdateNormalAndDepth();   // 更新地图点法向与观测深度
   }

   pMap->IncreaseChangeIndex();   // 地图变更计数+1，通知其他模块地图发生改动
}

int Optimizer::PoseInertialOptimizationLastKeyFrame(
    const std::shared_ptr<Frame>& pFrame, bool bRecInit) {
    // 构造g2o稀疏优化器对象
    g2o::SparseOptimizer optimizer;
    // 关闭g2o控制台打印输出
    optimizer.setVerbose(false);

    // 创建稠密线性求解器，针对BlockSolverX的位姿矩阵类型
    auto linearSolver = std::make_unique<
        g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
    // 构造高斯‑牛顿优化算法，传入BlockSolverX块求解器，移动语义接管linearSolver所有权
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

    // 将求解算法设置给优化器
    optimizer.setAlgorithm(solver);

    // 统计单目观测初始匹配点数量
    int nInitialMonoCorrespondences = 0;
    // 统计双目观测初始匹配点数量
    int nInitialStereoCorrespondences = 0;
    // 总初始匹配点 = 单目+双目
    int nInitialCorrespondences = 0;

    // ========= 添加待优化帧顶点：位姿、速度、陀螺仪bias、加速度计bias =========
    // 位姿顶点，绑定当前帧pFrame，id=0，不固定，参与优化
    VertexPose* VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    // 速度顶点，绑定当前帧pFrame，id=1，不固定，参与优化
    VertexVelocity* VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);

    // 陀螺仪零偏顶点，绑定当前帧pFrame，id=2，不固定，参与优化
    VertexGyroBias* VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);

    // 加速度计零偏顶点，绑定当前帧pFrame，id=3，不固定，参与优化
    VertexAccBias* VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);

    // ========= 地图点相关观测边准备 =========
    // 当前帧总特征点数量
    const int N = pFrame->N;
    // 左目特征点数量；双目模式Nleft != -1
    const int Nleft = pFrame->Nleft;
    // bRight=true代表存在右目图像（双目）
    const bool bRight = (Nleft != -1);

    // 存储单目重投影边指针
    vector<EdgeMonoOnlyPose*> vpEdgesMono;
    // 存储双目重投影边指针
    vector<EdgeStereoOnlyPose*> vpEdgesStereo;
    // 单目边对应的帧内特征点索引
    vector<size_t> vnIndexEdgeMono;
    // 双目边对应的帧内特征点索引
    vector<size_t> vnIndexEdgeStereo;
    // 预分配内存避免多次realloc
    vpEdgesMono.reserve(N);
    vpEdgesStereo.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    // Huber鲁棒核阈值，单目2自由度，卡方分布0.95，sqrt(5.991)
    const float thHuberMono = sqrt(5.991);
    // Huber鲁棒核阈值，双目3自由度，卡方分布0.95，sqrt(7.815)
    const float thHuberStereo = sqrt(7.815);

    {
        // 加全局地图点互斥锁，防止多线程读写地图点
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        // 遍历帧内全部特征点
        for (int i = 0; i < N; i++) {
            // 获取该特征点关联的地图点
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            // 该特征点有效，存在关联地图点
            if (pMP) {
                cv::KeyPoint kpUn;

                // ---------------------- 左目单目观测分支 ----------------------
                // 非双目模式且无右视深度 或者 当前特征属于左目区域
                if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
                    // i<Nleft说明是双目模式下的左‑右配对点，取原始左目关键点
                    if (i < Nleft)  // pair left‑right
                        kpUn = pFrame->mvKeys[i];
                    // 单目模式，取去畸变后关键点
                    else
                        kpUn = pFrame->mvKeysUn[i];

                    // 单目匹配计数自增
                    nInitialMonoCorrespondences++;
                    // 先标记该点不是外点，后续优化迭代再更新
                    pFrame->mvbOutlier[i] = false;

                    // 构造观测2D像素坐标
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    // 构造仅优化位姿的单目重投影边；0代表左目相机
                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                    // 边绑定顶点0：当前帧位姿VP
                    e->setVertex(0, VP);
                    // 设置像素观测值
                    e->setMeasurement(obs);

                    // 计算像素不确定性，用于信息矩阵加权
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);
                    // 金字塔层级逆方差除以不确定性，得到信息权重
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                    // 设置2维信息矩阵
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    // 创建Huber鲁棒核对象
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    // 边挂载鲁棒核
                    e->setRobustKernel(rk);
                    // 设置Huber阈值
                    rk->setDelta(thHuberMono);

                    // 将边加入优化器
                    optimizer.addEdge(e);

                    // 缓存边指针与对应特征点索引
                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                // ---------------------- 双目观测分支（非双目模式下的双目深度） ----------------------
                else if (!bRight) {
                    // 双目观测匹配计数自增
                    nInitialStereoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    // 获取去畸变关键点与右视匹配的u坐标
                    kpUn = pFrame->mvKeysUn[i];
                    const float kp_ur = pFrame->mvuRight[i];
                    // 双目观测：u, v, ur
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    // 构造仅优化位姿的双目重投影边
                    EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                    // 绑定当前帧位姿顶点
                    e->setVertex(0, VP);
                    // 设置3维观测
                    e->setMeasurement(obs);

                    // 像素不确定性，取前两维uv
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));
                    // 计算信息权重
                    const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                    // 设置3维信息矩阵
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    // Huber鲁棒核
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);

                    // 缓存边与索引
                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }

                // ---------------------- 双目模式右目单目观测 ----------------------
                if (bRight && i >= Nleft) {
                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    // 取右目关键点
                    kpUn = pFrame->mvKeysRight[i - Nleft];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    // 单目重投影边，1表示右目相机
                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // 不确定性加权
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

    // 总初始匹配数 = 单目+双目
    nInitialCorrespondences =
        nInitialMonoCorrespondences + nInitialStereoCorrespondences;

    // ========= 添加上一关键帧顶点，全部固定，作为IMU预积分起点 =========
    auto pKF = pFrame->mpLastKeyFrame;
    // 上一关键帧位姿顶点 id=4，固定不优化
    VertexPose* VPk = new VertexPose(pKF);
    VPk->setId(4);
    VPk->setFixed(true);
    optimizer.addVertex(VPk);

    // 上一关键帧速度顶点 id=5，固定
    VertexVelocity* VVk = new VertexVelocity(pKF);
    VVk->setId(5);
    VVk->setFixed(true);
    optimizer.addVertex(VVk);

    // 上一关键帧gyro bias id=6，固定
    VertexGyroBias* VGk = new VertexGyroBias(pKF);
    VGk->setId(6);
    VGk->setFixed(true);
    optimizer.addVertex(VGk);

    // 上一关键帧acc bias id=7，固定
    VertexAccBias* VAk = new VertexAccBias(pKF);
    VAk->setId(7);
    VAk->setFixed(true);
    optimizer.addVertex(VAk);

    // IMU预积分边，输入当前帧的预积分对象，连接上一KF与当前帧IMU状态
    EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);
    // 顶点0：上一KF位姿
    ei->setVertex(0, VPk);
    // 顶点1：上一KF速度
    ei->setVertex(1, VVk);
    // 顶点2：上一KF gyro bias
    ei->setVertex(2, VGk);
    // 顶点3：上一KF acc bias
    ei->setVertex(3, VAk);
    // 顶点4：当前帧位姿
    ei->setVertex(4, VP);
    // 顶点5：当前帧速度
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);

    // gyro bias随机游走边：约束上一帧bias与当前帧bias
    EdgeGyroRW* egr = new EdgeGyroRW();
    egr->setVertex(0, VGk);
    egr->setVertex(1, VG);
    // 从预积分协方差矩阵取出gyro bias部分求逆得到信息矩阵
    Eigen::Matrix3d InfoG =
        pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    // acc bias随机游走边：约束上一帧bias与当前帧bias
    EdgeAccRW* ear = new EdgeAccRW();
    ear->setVertex(0, VAk);
    ear->setVertex(1, VA);
    // 从预积分协方差取出acc bias块求逆作为信息矩阵
    Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                .cast<double>()
                                .inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    // 四轮优化；每轮优化完成后做内外点判定；level=1的边下一轮不参与优化；第3轮移除鲁棒核
    // 单目每轮卡方阈值，逐步收紧
    float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
    // 双目每轮卡方阈值，逐步收紧
    float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};
    // 每轮迭代最大迭代次数10次
    int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    int nBadMono = 0;
    int nBadStereo = 0;
    int nInliersMono = 0;
    int nInliersStereo = 0;
    int nInliers = 0;
    // 四轮迭代循环
    for (size_t it = 0; it < 4; it++) {
        // 初始化优化器，0代表不重置所有边level
        optimizer.initializeOptimization(0);
        // 执行优化，最多its[it]步
        optimizer.optimize(its[it]);

        // 每轮重置统计变量
        nBad = 0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers = 0;
        nInliersMono = 0;
        nInliersStereo = 0;
        // 近点阈值放大系数，深度<10m的近点允许更大残差
        float chi2close = 1.5 * chi2Mono[it];

        // ---------- 遍历所有单目边，做内外点判别 ----------
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            EdgeMonoOnlyPose* e = vpEdgesMono[i];

            // 获取该边对应的帧特征点索引
            const size_t idx = vnIndexEdgeMono[i];

            // 如果上一轮标记为外点，强制计算当前残差
            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            // 获取边的chi2残差
            const float chi2 = e->chi2();
            // 判断地图点是否为近点，深度小于10米
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

            // 外点条件：远点残差超阈值 || 近点残差放大后超阈值 || 三角化深度非正
            if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
                !e->isDepthPositive()) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);     // level=1，下一轮优化不使用这条边
                nBadMono++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);     // level=0，参与下一轮优化
                nInliersMono++;
            }

            // 第3轮(it==2)之后移除鲁棒核，后续使用普通最小二乘
            if (it == 2) e->setRobustKernel(0);
        }

        // ---------- 遍历所有双目边，内外点判别 ----------
        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            // 上一轮是外点，强制计算残差
            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            // 残差大于双目阈值判定外点
            if (chi2 > chi2Stereo[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);  // not included in next optimization
                nBadStereo++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
                nInliersStereo++;
            }

            // 第3轮移除鲁棒核
            if (it == 2) e->setRobustKernel(0);
        }

        // 统计总内点、外点
        nInliers = nInliersMono + nInliersStereo;
        nBad = nBadMono + nBadStereo;

        // 有效边过少直接跳出循环
        if (optimizer.edges().size() < 10) {
            break;
        }
    }

    // 如果内点数量不足30，并且不是恢复初始化模式，尝试抢救部分残差不算特别大的点
    if ((nInliers < 30) && !bRecInit) {
        nBad = 0;
        const float chi2MonoOut = 18.f;
        const float chi2StereoOut = 24.f;
        EdgeMonoOnlyPose* e1;
        EdgeStereoOnlyPose* e2;
        // 抢救单目边
        for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
            const size_t idx = vnIndexEdgeMono[i];
            e1 = vpEdgesMono[i];
            e1->computeError();
            // 残差小于宽松阈值，重新标记为内点
            if (e1->chi2() < chi2MonoOut)
                pFrame->mvbOutlier[idx] = false;
            else
                nBad++;
        }
        // 抢救双目边
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

    // ========= 将优化结果写回Frame对象 =========
    // 把优化得到的Rwb、twb、速度赋值给帧IMU状态
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                                VP->estimate().twb.cast<float>(),
                                VV->estimate().cast<float>());
    // 组装bias向量：[gyro_bias, acc_bias]
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    // 构造IMU Bias对象赋值给帧
    pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

    // ========= 构造先验Hessian矩阵，为下一帧Marginalize做准备 =========
    // 15维Hessian：位姿6维 + 速度3维 + gyro bias3维 + acc bias3维
    Eigen::Matrix<double, 15, 15> H;
    H.setZero();

    // 累加IMU预积分边的Hessian到H矩阵
    H.block<9, 9>(0, 0) += ei->GetHessian2();
    // 累加gyro bias随机游走边Hessian
    H.block<3, 3>(9, 9) += egr->GetHessian2();
    // 累加acc bias随机游走边Hessian
    H.block<3, 3>(12, 12) += ear->GetHessian2();

    int tot_in = 0, tot_out = 0;
    // 累加单目内点重投影边的Hessian，只加位姿部分(6x6)
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        EdgeMonoOnlyPose* e = vpEdgesMono[i];

        const size_t idx = vnIndexEdgeMono[i];

        // 仅内点参与先验Hessian构建
        if (!pFrame->mvbOutlier[idx]) {
            H.block<6, 6>(0, 0) += e->GetHessian();
            tot_in++;
        } else {
            tot_out++;
        }
    }

    // 累加双目内点重投影边Hessian
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

    // 创建约束先验对象ConstraintPoseImu，保存当前帧全部IMU状态与Hessian，留给下一帧做marginalization先验
    pFrame->mpcpi =
        new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb,
                              VV->estimate(), VG->estimate(), VA->estimate(), H);

    // 返回有效匹配数量 = 初始总匹配 - 最终外点数量
    return nInitialCorrespondences - nBad;
}

int Optimizer::PoseInertialOptimizationLastFrame(
    const std::shared_ptr<Frame>& pFrame, bool bRecInit) {
    // 构造g2o稀疏优化器
    g2o::SparseOptimizer optimizer;

    // 创建稠密线性求解器
    auto linearSolver = std::make_unique<
        g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
    // 高斯‑牛顿求解器 + BlockSolverX块求解器
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);
    // 关闭g2o打印
    optimizer.setVerbose(false);

    // 单目初始匹配计数
    int nInitialMonoCorrespondences = 0;
    // 双目初始匹配计数
    int nInitialStereoCorrespondences = 0;
    // 总初始匹配计数
    int nInitialCorrespondences = 0;

    // ========= 当前帧待优化顶点：位姿、速度、gyro bias、acc bias =========
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

    // ========= 地图点观测边准备 =========
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

    // Huber鲁棒核阈值
    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    {
        // 地图点全局锁，保护多线程访问地图点
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        // 遍历全部特征点
        for (int i = 0; i < N; i++) {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (pMP) {
                cv::KeyPoint kpUn;
                // ----------------------左目单目观测----------------------
                if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
                    if (i < Nleft)  // pair left‑right
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

                    // 像素不确定性加权信息矩阵
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
                // ----------------------双目观测----------------------
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

                // ----------------------双目模式右目单目观测----------------------
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

    // ========= 上一普通帧（非关键帧）顶点，**不固定**，参与优化 =========
    std::shared_ptr<Frame> pFp = pFrame->mpPrevFrame;

    VertexPose* VPk = new VertexPose(pFp);
    VPk->setId(4);
    VPk->setFixed(false);
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

    // IMU预积分边，使用帧间预积分mpImuPreintegratedFrame，连接上一帧与当前帧IMU状态
    EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);
    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, VP);
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);

    // gyro bias随机游走边
    EdgeGyroRW* egr = new EdgeGyroRW();
    egr->setVertex(0, VGk);
    egr->setVertex(1, VG);
    Eigen::Matrix3d InfoG =
        pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    // acc bias随机游走边
    EdgeAccRW* ear = new EdgeAccRW();
    ear->setVertex(0, VAk);
    ear->setVertex(1, VA);
    Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                .cast<double>()
                                .inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    // 如果上一帧没有marginalize出来的先验约束，打印警告日志
    if (!pFp->mpcpi)
        Verbose::PrintMess(
            "pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
            Verbose::VERBOSITY_NORMAL);

    // 先验边：用上一帧marginalize输出的ConstraintPoseImu作为先验约束上一帧IMU状态
    EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);
    ep->setVertex(0, VPk);
    ep->setVertex(1, VVk);
    ep->setVertex(2, VGk);
    ep->setVertex(3, VAk);
    // 给先验边加Huber鲁棒核，delta=5
    g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
    ep->setRobustKernel(rkp);
    rkp->setDelta(5);
    optimizer.addEdge(ep);

    // 四轮优化；单目阈值全程5.991；双目阈值逐步收紧；每轮最多迭代10次
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
    const int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    int nBadMono = 0;
    int nBadStereo = 0;
    int nInliersMono = 0;
    int nInliersStereo = 0;
    int nInliers = 0;
    // 四轮迭代优化
    for (size_t it = 0; it < 4; it++) {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        // 每轮重置统计变量
        nBad = 0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers = 0;
        nInliersMono = 0;
        nInliersStereo = 0;
        float chi2close = 1.5 * chi2Mono[it];

        // ----------单目边内外点判断----------
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            EdgeMonoOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];
            // 判断地图点是否近点(<10m)
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

            // 上一轮标记外点，强制计算残差
            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            // 外点判定条件
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

            // it=2移除鲁棒核
            if (it == 2) e->setRobustKernel(0);
        }

        // ----------双目边内外点判断----------
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

        // 边数量太少直接退出
        if (optimizer.edges().size() < 10) {
            break;
        }
    }

    // 内点不足30且不是恢复初始化模式，抢救部分点
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

    // ========= 将优化结果回写到Frame对象 =========
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                                VP->estimate().twb.cast<float>(),
                                VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

    // ========= 构造30维Hessian矩阵：上一帧15维 + 当前帧15维，之后边缘化掉上一帧状态 =========
    Eigen::Matrix<double, 30, 30> H;
    H.setZero();

    // 累加IMU预积分边完整Hessian(24×24)，占据H左上角24×24
    H.block<24, 24>(0, 0) += ei->GetHessian();

    // gyro bias随机游走边Hessian(6×6)，跨上一帧gyro bias与当前帧gyro bias，填充对应分块
    Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
    H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
    H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
    H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

    // acc bias随机游走边Hessian(6×6)，跨上一帧acc bias与当前帧acc bias，填充分块
    Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
    H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
    H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
    H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
    H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

    // 累加来自上一帧的先验边Hessian(15×15)，左上角，对应上一帧状态
    H.block<15, 15>(0, 0) += ep->GetHessian();

    int tot_in = 0, tot_out = 0;
    // 单目内点重投影边，只加到当前帧位姿6×6分块，H[15~21,15~21]
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

    // 双目内点重投影边，加到当前帧位姿分块
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

    // 边缘化掉0‑14索引，即上一帧全部15维IMU状态，留下当前帧15维的先验Hessian
    H = Marginalize(H, 0, 14);

    // 把marginalize后得到的当前帧Hessian保存到mpcpi，作为下一帧的先验约束
    pFrame->mpcpi = new ConstraintPoseImu(
        VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
        VA->estimate(), H.block<15, 15>(15, 15));
    // 释放上一帧的先验约束，已经被marginalize，不再使用
    delete pFp->mpcpi;
    pFp->mpcpi = NULL;

    // 返回有效匹配点数
    return nInitialCorrespondences - nBad;
}

void Optimizer::OptimizeEssentialGraph4DoF(
    const std::shared_ptr<Map>& pMap, const std::shared_ptr<KeyFrame>& pLoopKF,
    const std::shared_ptr<KeyFrame>& pCurKF,
    const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose& CorrectedSim3,
    const map<std::shared_ptr<KeyFrame>, set<std::shared_ptr<KeyFrame>>>&
        LoopConnections) {
  // Setup optimizer
  // 构造g2o稀疏优化器实例，用于4DoF位姿图闭环优化
  g2o::SparseOptimizer optimizer;
  // 关闭g2o控制台打印输出，不输出迭代调试信息
  optimizer.setVerbose(false);

  // 创建Eigen实现的线性求解器，针对BlockSolverX的位姿矩阵类型
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  // 构造列文伯格‑马夸尔特优化算法，传入BlockSolverX块求解器，接管linearSolver所有权
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));

  // 将LM优化算法设置给稀疏优化器
  optimizer.setAlgorithm(solver);

  // 获取地图中全部关键帧集合
  const vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  // 获取地图中全部地图点集合
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  // 获取地图内最大关键帧ID，用于数组容器开辟下标空间
  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  // vScw[nID]: 关键帧nID的 Sim3 位姿 S_cw，世界到相机Sim3变换
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  // vCorrectedSwc[nID]: 优化完成后，相机到世界的Sim3逆变换 S_wc
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);

  // 保存每一个关键帧对应的4DoF位姿顶点指针，下标为关键帧mnId
  vector<VertexPose4DoF*> vpVertices(nMaxKFid + 1);

  // 共视权重阈值，共视特征点数量大于该值才会添加约束边
  const int minFeat = 100;
  // Set KeyFrame vertices
  // 遍历所有关键帧，向g2o优化器添加4DoF位姿顶点
  for (auto pKF : vpKFs) {
    // 跳过已经标记为失效的坏关键帧
    if (pKF->isBad()) continue;

    VertexPose4DoF* V4DoF;
    // 获取当前关键帧ID，作为g2o顶点id
    const int nIDi = pKF->mnId;
    // 在闭环校正后的Sim3容器查找该关键帧是否存在闭环修正后的位姿
    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);
    if (it != CorrectedSim3.end()) {
      // 存在闭环修正，使用闭环输出的Sim3 S_cw
      vScw[nIDi] = it->second;
      // 求逆得到S_wc 相机到世界Sim3变换
      const g2o::Sim3 Swc = it->second.inverse();
      // 提取旋转矩阵Rwc
      Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
      // 提取平移twc
      Eigen::Vector3d twc = Swc.translation();
      // 使用闭环修正后的Rwc、twc构造4DoF位姿顶点
      V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
    } else {
      // 无闭环修正，直接读取关键帧原始SE3位姿
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      // 将SE3转换为尺度s=1的Sim3，S_cw
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

      vScw[nIDi] = Siw;
      // 使用关键帧原始位姿构造4DoF顶点
      V4DoF = new VertexPose4DoF(pKF);
    }

    // 将闭环参考帧pLoopKF顶点固定，作为优化的基准，不参与迭代更新
    if (pKF == pLoopKF) V4DoF->setFixed(true);
    // 设置g2o顶点ID，与关键帧mnId保持一致
    V4DoF->setId(nIDi);
    // 关闭边缘化，位姿图优化不做Schur消元边缘化
    V4DoF->setMarginalized(false);

    // 将4DoF位姿顶点加入g2o稀疏优化器
    optimizer.addVertex(V4DoF);
    // 在vpVertices数组缓存顶点指针，后续快速索引
    vpVertices[nIDi] = V4DoF;
  }

  // set存储已经添加过的边，pair存储(minId,maxId)，避免重复添加同一对关键帧约束边
  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // Edge used in posegraph has still 6Dof, even if updates of camera poses are
  // just in 4DoF
  // 信息矩阵，虽然顶点是4DoF，但边残差使用6维，对旋转两个轴施加强约束，只允许yaw旋转
  Eigen::Matrix<double, 6, 6> matLambda =
      Eigen::Matrix<double, 6, 6>::Identity();
  // 对roll、pitch设置大权重1e3，抑制这两个方向旋转，只放开航向角yaw
  matLambda(0, 0) = 1e3;
  matLambda(1, 1) = 1e3;
  matLambda(0, 0) = 1e3;

  // Set Loop edges
  // Edge4DoF* e_loop;
  // 遍历闭环连接关系，添加闭环约束边
  for (auto const& [pKF, spConnections] : LoopConnections) {
    // 当前关键帧ID
    const long unsigned int nIDi = pKF->mnId;
    // 获取该关键帧的S_cw Sim3位姿
    const g2o::Sim3 Siw = vScw[nIDi];

    // 遍历该关键帧闭环连接的邻居关键帧
    for (auto pConnection : spConnections) {
      const long unsigned int nIDj = pConnection->mnId;
      // 过滤条件：不是(curKF,loopKF)配对，并且共视权重小于minFeat则跳过
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(pConnection) < minFeat)
        continue;

      // 获取邻居关键帧Sim3位姿
      const g2o::Sim3 Sjw = vScw[nIDj];
      // 计算两帧之间相对Sim3变换 S_ij = S_iw * S_jw^{-1}
      const g2o::Sim3 Sij = Siw * Sjw.inverse();
      // Sim3转4×4齐次变换矩阵Tij
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      // 实例化4DoF位姿图边，输入相对变换Tij作为观测值
      Edge4DoF* e = new Edge4DoF(Tij);
      // 设置边的两个顶点：顶点1为j帧，顶点0为i帧
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));

      // 设置边的信息矩阵
      e->information() = matLambda;
      // e_loop = e;
      // 将闭环约束边加入优化器
      optimizer.addEdge(e);

      // 将这条边的帧id对存入集合，标记已添加，防止重复建边
      sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
    }
  }

  // 1. Set normal edges
  // 遍历所有关键帧，添加生成树边、惯性相邻帧边、旧闭环边、共视图约束边
  for (auto const& pKF : vpKFs) {
    const int nIDi = pKF->mnId;

    g2o::Sim3 Siw;
    // Use noncorrected poses for posegraph edges
    // 位姿图边使用未经过闭环修正的原始位姿作为观测
    LoopClosing::KeyFrameAndPose::const_iterator iti =
        NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Siw = iti->second;
    else
      Siw = vScw[nIDi];

    // 1.1.0 Spanning tree edge
    // 生成树父关键帧，这里代码存在bug：pParentKF没有赋值，始终为空，该段永远不会执行
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

      // 计算i帧到父j帧相对Sim3 S_ij = S_iw * S_wj
      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      // 创建4DoF约束边，添加生成树约束
      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.1.1 Inertial edges
    // IMU惯性相邻帧，pKF的前一帧mPrevKF
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

      // 计算当前帧i到前一帧j的相对Sim3变换
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

    // 1.2 Loop edges
    // 获取该关键帧历史所有闭环关联关键帧
    auto const& sLoopEdges = pKF->GetLoopEdges();
    for (auto pLKF : sLoopEdges) {
      // 只处理id更小的闭环帧，单向建边，避免重复
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Swl;

        LoopClosing::KeyFrameAndPose::const_iterator itl =
            NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Swl = itl->second.inverse();
        else
          Swl = vScw[pLKF->mnId].inverse();

        // i帧到历史闭环帧l的相对Sim3
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

    // 1.3 Covisibility graph edges
    // 获取与当前帧共视权重大于minFeat的共视关键帧
    const vector<std::shared_ptr<KeyFrame>> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);

    // 遍历共视关键帧，添加共视图约束边
    for (auto const& pKFn : vpConnectedKFs) {
      // 过滤：不为空、不是父帧、不是IMU前帧、不是下一帧、不是生成树子节点、不是闭环帧
      if (pKFn && pKFn != pParentKF && pKFn != prevKF && pKFn != pKF->mNextKF &&
          !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
        // 跳过坏帧；只处理id更小帧，单向建边；检查该帧对是否已经添加过边
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

          // 当前帧i到共视帧n的相对Sim3变换
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

  // g2o初始化优化，准备顶点边结构、计算雅可比图
  optimizer.initializeOptimization();
  // 计算所有激活边的初始误差，用于调试查看初始残差
  optimizer.computeActiveErrors();
  // 执行LM迭代优化，最大迭代20次
  optimizer.optimize(20);

  // 加地图更新互斥锁，防止多线程同时读写地图数据
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  // 4DoF位姿图优化完毕，回写所有关键帧位姿到KeyFrame对象
  for (auto pKFi : vpKFs) {
    const int nIDi = pKFi->mnId;

    // 取出优化后的4DoF顶点
    VertexPose4DoF* Vi = static_cast<VertexPose4DoF*>(optimizer.vertex(nIDi));
    // 获取优化后的旋转Rcw、平移tcw
    Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
    Eigen::Vector3d ti = Vi->estimate().tcw[0];

    // 构造尺度s=1的Sim3，4DoF优化只优化旋转yaw与平移，尺度固定为1
    g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
    // 求逆，保存相机到世界S_wc，后续地图点校正使用
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

    // Sim3转SE3，尺度=1，构造Sophus SE3d世界到相机位姿Tiw
    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
    // 将优化后的位姿回写给关键帧，转float存储
    pKFi->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non‑optimized" reference keyframe pose and
  // transform back with optimized pose
  // 根据关键帧优化前后位姿变化，校正全部地图点的世界坐标
  for (auto const& pMP : vpMPs) {
    // 跳过坏地图点
    if (pMP->isBad()) continue;

    // 获取地图点的参考关键帧
    std::shared_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
    const int nIDr = pRefKF->mnId;

    // 优化前参考帧的S_rw(世界到相机)
    g2o::Sim3 Srw = vScw[nIDr];
    // 优化后参考帧的S_wr(相机到世界)
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    // 读取地图点当前世界坐标，转为double精度
    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    // 点校正公式：Pw' = S_wr * S_rw * Pw
    // 1. Srw.map(Pw): 将世界点投影到参考帧相机坐标系(旧位姿)
    // 2. correctedSwr.map(): 使用优化后参考帧位姿，把相机坐标系点重新投影回世界坐标系
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    // 回写校正后的地图点世界坐标，转float
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    // 更新地图点法向量、观测深度
    pMP->UpdateNormalAndDepth();
  }
  // 地图变更计数+1，标记地图发生修改，其他线程可以感知地图更新
  pMap->IncreaseChangeIndex();
}

}  // namespace ORB_SLAM3
