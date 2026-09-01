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
#include "Sim3Solver.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <vector>

#include "KeyFrame.h"
#include "ORBmatcher.h"
#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM3 {

/**
 * @brief Sim3求解器构造函数，求解两关键帧之间带尺度的相似变换Sim3(旋转R,平移t,尺度s)，用于回环检测、重定位
 * @param pKF1 参考关键帧1
 * @param pKF2 待匹配关键帧2
 * @param vpMatched12 匹配关系：vpMatched12[i]为KF1第i个地图点在KF2匹配到的地图点，nullptr代表无匹配
 * @param bFixScale 是否固定尺度，true为固定尺度=1（SE3），false求解尺度s（Sim3）；单目回环需要求解尺度，双目/RGBD固定尺度
 * @param vpKeyFrameMatchedMP 每个匹配地图点所属的关键帧，空则全部默认属于pKF2
 */
Sim3Solver::Sim3Solver(const std::shared_ptr<KeyFrame> &pKF1,
                       const std::shared_ptr<KeyFrame> &pKF2,
                       const vector<MapPoint *> &vpMatched12,
                       const bool bFixScale,
                       vector<std::shared_ptr<KeyFrame>> vpKeyFrameMatchedMP)
    : mnIterations(0),          // RANSAC已经执行迭代次数
      mnBestInliers(0),         // RANSAC得到最优模型的内点数量
      mbFixScale(bFixScale),    // 是否固定尺度开关
      pCamera1(pKF1->mpCamera), // KF1对应的相机模型
      pCamera2(pKF2->mpCamera)  // KF2对应的相机模型
{
    bool bDifferentKFs = false;
    // 如果传入的匹配点所属关键帧数组为空，则标记所有匹配点都来自pKF2
    if (vpKeyFrameMatchedMP.empty()) {
        bDifferentKFs = true;
        // 构造数组，全部元素填充pKF2
        vpKeyFrameMatchedMP =
            vector<std::shared_ptr<KeyFrame>>(vpMatched12.size(), pKF2);
    }

    mpKF1 = pKF1;
    mpKF2 = pKF2;

    // 获取KF1每个特征点对应的地图点
    vector<MapPoint *> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    // 匹配对总数量
    mN1 = vpMatched12.size();

    // 预分配容器内存，存储有效匹配的地图点、索引、相机坐标系3D点
    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    // 获取KF1、KF2的位姿 Rcw:世界到相机旋转，tcw：世界到相机平移
    Eigen::Matrix3f Rcw1 = pKF1->GetRotation();
    Eigen::Vector3f tcw1 = pKF1->GetTranslation();
    Eigen::Matrix3f Rcw2 = pKF2->GetRotation();
    Eigen::Vector3f tcw2 = pKF2->GetTranslation();

    // 保存有效匹配点的索引，用于RANSAC随机采样
    mvAllIndices.reserve(mN1);

    size_t idx = 0;

    std::shared_ptr<KeyFrame> pKFm = pKF2;  // Default variable，当前匹配点所属关键帧
    // 遍历所有匹配对，筛选有效地图点，计算相机坐标系三维坐标
    for (int i1 = 0; i1 < mN1; i1++) {
        // 当前索引存在匹配地图点
        if (vpMatched12[i1]) {
            // pMP1是KF1上的地图点，pMP2是KF2匹配到的地图点
            MapPoint *pMP1 = vpKeyFrameMP1[i1];
            MapPoint *pMP2 = vpMatched12[i1];

            // KF1该位置无地图点，跳过
            if (!pMP1) continue;

            // 任意一个地图点标记为坏点，舍弃该匹配对
            if (pMP1->isBad() || pMP2->isBad()) continue;

            // 如果匹配点来自不同关键帧，更新pKFm
            if (bDifferentKFs) pKFm = vpKeyFrameMatchedMP[i1];

            // 获取地图点在对应关键帧特征点容器中的下标
            int indexKF1 = get<0>(pMP1->GetIndexInKeyFrame(pKF1));
            int indexKF2 = get<0>(pMP2->GetIndexInKeyFrame(pKFm));

            // 下标为负说明地图点不在该关键帧中，无效匹配
            if (indexKF1 < 0 || indexKF2 < 0) continue;

            // 取出去畸变后的特征点
            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKFm->mvKeysUn[indexKF2];

            // 根据特征点金字塔层级获取方差，用于计算重投影误差阈值
            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKFm->mvLevelSigma2[kp2.octave];

            // 9.210是卡方分布chi2(2自由度,95%)阈值，保存每一对匹配点的最大允许误差平方
            mvnMaxError1.push_back(9.210 * sigmaSquare1);
            mvnMaxError2.push_back(9.210 * sigmaSquare2);

            // 保存有效匹配的地图点、原始索引
            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            // 地图点世界坐标转换到KF1相机坐标系 Xc1 = Rcw1*Xw + tcw1
            Eigen::Vector3f X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1 * X3D1w + tcw1);

            // 地图点世界坐标转换到KF2相机坐标系 Xc2 = Rcw2*Xw + tcw2
            Eigen::Vector3f X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2 * X3D2w + tcw2);

            // 记录有效匹配点的内部索引，RANSAC采样使用
            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    // 将相机坐标系三维点投影到图像平面，保存2D像素坐标
    FromCameraToImage(mvX3Dc1, mvP1im1, pCamera1);
    FromCameraToImage(mvX3Dc2, mvP2im2, pCamera2);

    // 设置RANSAC默认参数
    SetRansacParameters();
}

/**
 * @brief 设置RANSAC求解Sim3的参数，自动计算RANSAC迭代次数
 * @param probability 期望成功得到正确模型的概率
 * @param minInliers 最少内点阈值
 * @param maxIterations 允许最大迭代次数上限
 */
void Sim3Solver::SetRansacParameters(double probability, int minInliers,
                                     int maxIterations) {
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;

    // 有效匹配对数量，至少取1避免除零
    N = std::max<size_t>(1, mvpMapPoints1.size());  // number of correspondences

    // 存储单次迭代内点标记
    mvbInliersi.resize(N);

    // 根据匹配点数量计算外点占比epsilon
    float epsilon = static_cast<float>(mRansacMinInliers / N);

    // RANSAC迭代次数公式：n = log(1‑p)/log(1‑(inlier_ratio)^sample_size)，Sim3最小样本为3对点
    int nIterations;

    // 如果要求全部点都是内点，只迭代1次
    if (mRansacMinInliers == N)
        nIterations = 1;
    else
        nIterations = ceil(log(1 - mRansacProb) / log(1 - pow(epsilon, 3)));

    // 实际迭代次数取计算值与最大迭代上限两者的较小值，最小为1
    mRansacMaxIts = max(1, min(nIterations, mRansacMaxIts));

    // 重置已经迭代计数
    mnIterations = 0;
}

/**
 * @brief 执行指定轮数RANSAC迭代，求解Sim3变换，无收敛标记，找到满足最小内点直接返回
 * @param nIterations 本次调用允许执行的迭代轮数
 * @param bNoMore 输出标记，true代表达到最大迭代次数，RANSAC结束
 * @param vbInliers 输出，原始匹配数组的内点标记
 * @param nInliers 输出，当前最优模型内点数目
 * @return 4×4 Sim3变换矩阵 T12，将KF1坐标系点变换到KF2坐标系；失败返回单位矩阵
 */
Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore,
                                    vector<bool> &vbInliers, int &nInliers) {
    bNoMore = false;
    vbInliers = vector<bool>(mN1, false);
    nInliers = 0;

    // 有效匹配点数量小于最小内点要求，直接终止
    if (N < mRansacMinInliers) {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;
    Eigen::Matrix3f P3Dc1i; // 3列矩阵，保存随机采样3对点：KF1相机下3D点
    Eigen::Matrix3f P3Dc2i; // 3列矩阵，保存随机采样3对点：KF2相机下3D点

    int nCurrentIterations = 0;
    // 没有达到全局最大迭代，同时本次调用迭代次数没有耗尽
    while (mnIterations < mRansacMaxIts && nCurrentIterations < nIterations) {
        nCurrentIterations++;
        mnIterations++;

        // 复制全部可用点索引，用于不放回随机采样3对点
        vAvailableIndices = mvAllIndices;

        // Get min set of points，Sim3最小求解集合需要3对3D点
        for (short i = 0; i < 3; ++i) {
            // 随机取一个下标
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

            int idx = vAvailableIndices[randi];

            // 填入矩阵第i列
            P3Dc1i.col(i) = mvX3Dc1[idx];
            P3Dc2i.col(i) = mvX3Dc2[idx];

            // 不放回采样：把选中元素用末尾元素覆盖，弹出末尾，实现删除
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // 使用采样得到3对点计算Sim3变换 T12
        ComputeSim3(P3Dc1i, P3Dc2i);

        // 计算当前模型的内点
        CheckInliers();

        // 如果当前迭代内点数量优于历史最优，更新最优模型
        if (mnInliersi >= mnBestInliers) {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i;
            mBestRotation = mR12i;
            mBestTranslation = mt12i;
            mBestScale = ms12i;

            // 当前内点超过要求最小内点，直接提前返回，RANSAC提前终止
            if (mnInliersi > mRansacMinInliers) {
                nInliers = mnInliersi;
                // 将内部有效点的内点标记映射回原始匹配数组下标
                for (int i = 0; i < N; i++)
                    if (mvbInliersi[i]) vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            }
        }
    }

    // 已经达到全局RANSAC最大迭代次数，标记结束
    if (mnIterations >= mRansacMaxIts) bNoMore = true;

    return Eigen::Matrix4f::Identity();
}

/**
 * @brief RANSAC迭代重载版本，增加bConverge收敛标记
 * @param nIterations 本次调用可执行迭代轮数
 * @param bNoMore 输出，true代表达到最大迭代，RANSAC全部结束
 * @param vbInliers 输出原始匹配数组内点标记
 * @param nInliers 输出最优模型内点数目
 * @param bConverge 输出，true代表找到满足最小内点的合格模型
 * @return 4×4 Sim3矩阵T12；无合格模型返回当前bestSim3
 */
Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore,
                                    vector<bool> &vbInliers, int &nInliers,
                                    bool &bConverge) {
    bNoMore = false;
    bConverge = false;
    vbInliers = vector<bool>(mN1, false);
    nInliers = 0;

    // 有效匹配点不足最小内点数量，直接退出
    if (N < mRansacMinInliers) {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int nCurrentIterations = 0;
    Eigen::Matrix4f bestSim3;
    while (mnIterations < mRansacMaxIts && nCurrentIterations < nIterations) {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;

        // Get min set of points，随机采样3对点
        for (short i = 0; i < 3; ++i) {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

            int idx = vAvailableIndices[randi];

            P3Dc1i.col(i) = mvX3Dc1[idx];
            P3Dc2i.col(i) = mvX3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // 计算Sim3模型
        ComputeSim3(P3Dc1i, P3Dc2i);

        // 统计该模型内点
        CheckInliers();

        // 更新全局最优模型
        if (mnInliersi >= mnBestInliers) {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i;
            mBestRotation = mR12i;
            mBestTranslation = mt12i;
            mBestScale = ms12i;

            // 内点超过阈值，判定收敛，直接返回
            if (mnInliersi > mRansacMinInliers) {
                nInliers = mnInliersi;
                for (int i = 0; i < N; i++)
                    if (mvbInliersi[i]) vbInliers[mvnIndices1[i]] = true;
                bConverge = true;
                return mBestT12;
            } else {
                bestSim3 = mBestT12;
            }
        }
    }

    // 迭代耗尽，标记RANSAC结束
    if (mnIterations >= mRansacMaxIts) bNoMore = true;

    return bestSim3;
}

/**
 * @brief 直接执行完整全套RANSAC，封装接口，调用iterate跑满最大迭代
 * @param vbInliers12 [out]原始匹配数组内点标记
 * @param nInliers [out]最优模型内点数量
 * @return 4×4 Sim3矩阵T12
 */
Eigen::Matrix4f Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers) {
    bool bFlag;
    return iterate(mRansacMaxIts, bFlag, vbInliers12, nInliers);
}

/**
 * @brief 计算点集质心，并且得到每个点相对于质心的去中心坐标
 * @param P 输入点集3×N矩阵
 * @param Pr [out]每个点减去质心后的相对坐标
 * @param C [out]点集质心
 */
void Sim3Solver::ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr,
                                  Eigen::Vector3f &C) {
    // 按行求和，得到点集总和
    C = P.rowwise().sum();
    // 除以点数量，得到质心
    C = C / P.cols();
    // 每个点减去质心，得到去中心化坐标
    for (int i = 0; i < P.cols(); i++) Pr.col(i) = P.col(i) - C;
}

/**
 * @brief Horn1987闭式解法，输入两组3对3D点，求解Sim3变换 T12：P1 = s*R12*P2 + t12
 * @param P1 3列矩阵，KF1相机坐标系三维点
 * @param P2 3列矩阵，KF2相机坐标系三维点
 */
void Sim3Solver::ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2) {
    // Custom implementation of:
    // Horn 1987, Closed‑form solution of absolute orientataion using unit
    // quaternions

    // Step 1: Centroid and relative coordinates，计算两组点质心、去中心化坐标
    Eigen::Matrix3f Pr1;  // Relative coordinates to centroid (set 1)
    Eigen::Matrix3f Pr2;  // Relative coordinates to centroid (set 2)
    Eigen::Vector3f O1;   // Centroid of P1
    Eigen::Vector3f O2;   // Centroid of P2

    ComputeCentroid(P1, Pr1, O1);
    ComputeCentroid(P2, Pr2, O2);

    // Step 2: Compute M matrix，协方差矩阵
    Eigen::Matrix3f M = Pr2 * Pr1.transpose();

    // Step 3: Compute N matrix，4×4矩阵，用于四元数特征值求解旋转
    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    Eigen::Matrix4f N;

    N11 = M(0, 0) + M(1, 1) + M(2, 2);
    N12 = M(1, 2) - M(2, 1);
    N13 = M(2, 0) - M(0, 2);
    N14 = M(0, 1) - M(1, 0);
    N22 = M(0, 0) - M(1, 1) - M(2, 2);
    N23 = M(0, 1) + M(1, 0);
    N24 = M(2, 0) + M(0, 2);
    N33 = -M(0, 0) + M(1, 1) - M(2, 2);
    N34 = M(1, 2) + M(2, 1);
    N44 = -M(0, 0) - M(1, 1) + M(2, 2);

    // 填充4×4对称N矩阵
    N << N11, N12, N13, N14, N12, N22, N23, N24, N13, N23, N33, N34, N14, N24,
        N34, N44;

    // Step 4: Eigenvector of the highest eigenvalue，求特征值特征向量，最大特征向量对应旋转四元数
    Eigen::EigenSolver<Eigen::Matrix4f> eigSolver;
    eigSolver.compute(N);

    // 取出实部特征值、特征向量
    Eigen::Vector4f eval = eigSolver.eigenvalues().real();
    Eigen::Matrix4f evec =
        eigSolver.eigenvectors()
            .real();  // evec[0] is the quaternion of the desired rotation

    int maxIndex;  // should be zero，寻找最大特征值下标
    eval.maxCoeff(&maxIndex);

    // 提取四元数虚部
    Eigen::Vector3f vec = evec.block<3, 1>(
        1, maxIndex);  // extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    // 四元数转旋转角，atan2(虚部模长，实部)得到半角
    double ang = atan2(vec.norm(), evec(0, maxIndex));

    // 转换为轴角表示，乘以2恢复完整旋转角度
    vec = 2 * ang * vec /
          vec.norm();  // Angle‑axis representation. quaternion angle is the half
    // 轴角转旋转矩阵SO3
    mR12i = Sophus::SO3f::exp(vec).matrix();

    // Step 5: Rotate set 2，把去中心化后的P2点旋转
    Eigen::Matrix3f P3 = mR12i * Pr2;

    // Step 6: Scale，求解尺度s
    if (!mbFixScale) {
        // 分子：Pr1与旋转后P3点的点积之和
        double cvnom = Converter::toCvMat(Pr1).dot(Converter::toCvMat(P3));
        double nom = (Pr1.array() * P3.array()).sum();
        // 两套计算方式对比，打印数值差异用于调试
        if (abs(nom - cvnom) > 1e-3)
            std::cout << "sim3 solver: " << abs(nom - cvnom) << std::endl
                      << nom << std::endl;
        // 分母：旋转后P3各点模长平方和
        Eigen::Array<float, 3, 3> aux_P3;
        aux_P3 = P3.array() * P3.array();
        double den = aux_P3.sum();

        // 尺度s = nom / den
        ms12i = nom / den;
    } else {
        // 固定尺度，s强制等于1
        ms12i = 1.0f;
    }

    // Step 7: Translation，求解平移 t12 = O1 - s*R12*O2
    mt12i = O1 - ms12i * mR12i * O2;

    // Step 8: Transformation，组装Sim3矩阵 T12: 把P1坐标系点转到P2坐标系
    // Step 8.1 T12
    mT12i.setIdentity();
    Eigen::Matrix3f sR = ms12i * mR12i;
    mT12i.block<3, 3>(0, 0) = sR;
    mT12i.block<3, 1>(0, 3) = mt12i;

    // Step 8.2 T21，Sim3逆变换，P2转到P1坐标系
    mT21i.setIdentity();
    Eigen::Matrix3f sRinv = (1.0 / ms12i) * mR12i.transpose();

    // sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    mT21i.block<3, 3>(0, 0) = sRinv;

    Eigen::Vector3f tinv = -sRinv * mt12i;
    mT21i.block<3, 1>(0, 3) = tinv;
}

/**
 * @brief 使用当前ComputeSim3得到的T12/T21做双向重投影，统计内点，填充mvbInliersi、mnInliersi
 */
void Sim3Solver::CheckInliers() {
    vector<Eigen::Vector2f> vP1im2, vP2im1;
    // mvX3Dc2(KF2相机3D点) 通过T12变换到KF1坐标系，投影到KF1图像
    Project(mvX3Dc2, vP2im1, mT12i, pCamera1);
    // mvX3Dc1(KF1相机3D点) 通过T21变换到KF2坐标系，投影到KF2图像
    Project(mvX3Dc1, vP1im2, mT21i, pCamera2);

    mnInliersi = 0;

    // 遍历每一对匹配点，双向重投影误差校验
    for (size_t i = 0; i < mvP1im1.size(); i++) {
        // 预测像素与观测像素差向量
        Eigen::Vector2f dist1 = mvP1im1[i] - vP2im1[i];
        Eigen::Vector2f dist2 = vP1im2[i] - mvP2im2[i];

        // 误差平方
        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        // 双向误差都小于阈值，判定为内点
        if (err1 < mvnMaxError1[i] && err2 < mvnMaxError2[i]) {
            mvbInliersi[i] = true;
            mnInliersi++;
        } else {
            mvbInliersi[i] = false;
        }
    }
}

/**
 * @brief 获取RANSAC求解得到最优4×4 Sim3变换矩阵 T12
 * @return 4×4 Eigen矩阵 Sim3
 */
Eigen::Matrix4f Sim3Solver::GetEstimatedTransformation() { return mBestT12; }

/**
 * @brief 获取最优模型旋转矩阵R12
 * @return 3×3旋转矩阵
 */
Eigen::Matrix3f Sim3Solver::GetEstimatedRotation() { return mBestRotation; }

/**
 * @brief 获取最优模型平移向量t12
 * @return 3维平移向量
 */
Eigen::Vector3f Sim3Solver::GetEstimatedTranslation() {
    return mBestTranslation;
}

/**
 * @brief 获取最优模型尺度s12
 * @return float尺度因子
 */
float Sim3Solver::GetEstimatedScale() { return mBestScale; }

/**
 * @brief 将一组三维点，使用Tcw变换后投影到图像平面
 * @param vP3Dw 输入三维点（世界坐标系）
 * @param vP2D [out]输出像素坐标
 * @param Tcw 变换矩阵 world -> camera
 * @param pCamera 相机模型对象（针孔/KB鱼眼）
 */
void Sim3Solver::Project(const vector<Eigen::Vector3f> &vP3Dw,
                         vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw,
                         const std::shared_ptr<GeometricCamera> &pCamera) {
    // 从4×4矩阵提取旋转、平移
    Eigen::Matrix3f Rcw = Tcw.block<3, 3>(0, 0);
    Eigen::Vector3f tcw = Tcw.block<3, 1>(0, 3);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    // 遍历每个3D点，转到相机坐标系，调用相机模型project得到像素
    for (size_t i = 0, iend = vP3Dw.size(); i < iend; i++) {
        Eigen::Vector3f P3Dc = Rcw * vP3Dw[i] + tcw;
        Eigen::Vector2f pt2D = pCamera->project(P3Dc);
        vP2D.push_back(pt2D);
    }
}

/**
 * @brief 相机坐标系三维点直接投影到图像，不需要位姿变换
 * @param vP3Dc 输入相机坐标系三维点
 * @param vP2D [out]输出像素坐标
 * @param pCamera 相机模型
 */
void Sim3Solver::FromCameraToImage(
    const vector<Eigen::Vector3f> &vP3Dc, vector<Eigen::Vector2f> &vP2D,
    const std::shared_ptr<GeometricCamera> &pCamera) {
    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for (size_t i = 0, iend = vP3Dc.size(); i < iend; i++) {
        Eigen::Vector2f pt2D = pCamera->project(vP3Dc[i]);
        vP2D.push_back(pt2D);
    }
}

}  // namespace ORB_SLAM3
