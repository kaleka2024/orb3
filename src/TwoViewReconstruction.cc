/**
 * @file TwoViewReconstruction.cc
 * @brief ORB-SLAM3两视图几何重建模块：包含单应矩阵/基础矩阵RANSAC鲁棒求解、运动恢复、三角化校验
 *        用于单目初始化、重定位、回环检测中的两帧运动估计与地图点三维恢复
 */

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

#include "TwoViewReconstruction.h"
#include <algorithm>
#include <thread>
#include <utility>
#include <vector>

#include "Converter.h"
#include "GeometricTools.h"
#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM3 {

// 引入std常用名字空间标识符，简化代码书写
using std::make_pair;
using std::max;
using std::min;
using std::ref;
using std::thread;
using std::vector;

/**
 * @brief 两视图重建类构造函数
 * @param k 相机内参矩阵 3×3
 * @param sigma 像素重投影误差的高斯分布标准差，用于卡方检验阈值计算
 * @param iterations RANSAC最大迭代次数
 */
TwoViewReconstruction::TwoViewReconstruction(const Eigen::Matrix3f &k,
                                             float sigma, int iterations) {
    mK = k;                  // 保存相机内参矩阵到成员变量
    mSigma = sigma;            // 像素误差标准差
    mSigma2 = sigma * sigma;   // 误差方差 sigma²
    mMaxIterations = iterations; // RANSAC最大迭代轮数
}

/**
 * @brief 两视图重建主入口：自动选择单应/基础矩阵模型，RANSAC鲁棒估计两帧运动，三角化生成3D点
 * @param vKeys1 参考帧(帧1)特征点数组
 * @param vKeys2 当前帧(帧2)特征点数组
 * @param vMatches12 匹配关系：vMatches12[i]为帧1第i个特征匹配到帧2的特征索引，-1代表无匹配
 * @param T21 [out] 输出帧2到帧1的SE3变换：P₁ = T₂₁ · P₂
 * @param vP3D [out] 输出三角化得到的3D点数组，索引与帧1特征点索引对应
 * @param vbTriangulated [out] 输出对应特征点是否成功三角化的标记
 * @return true重建成功；false重建失败（匹配不足、无合理解、视差不足等）
 */
bool TwoViewReconstruction::Reconstruct(const std::vector<cv::KeyPoint> &vKeys1,
                                        const std::vector<cv::KeyPoint> &vKeys2,
                                        const vector<int> &vMatches12,
                                        Sophus::SE3f &T21,
                                        vector<cv::Point3f> &vP3D,
                                        vector<bool> &vbTriangulated) {
    // 清空成员特征点容器
    mvKeys1.clear();
    mvKeys2.clear();
    // 保存输入的两帧特征点到成员变量
    mvKeys1 = vKeys1;
    mvKeys2 = vKeys2;

    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    // 清空匹配对容器，重新整理有效匹配
    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    // 标记帧1每个特征点是否存在匹配
    mvbMatched1.resize(mvKeys1.size());

    // 遍历输入匹配数组，筛选有效匹配对存入mvMatches12
    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
        if (vMatches12[i] >= 0) { // 匹配索引有效
            // 保存<帧1特征索引, 帧2特征索引>配对
            mvMatches12.push_back(make_pair(i, vMatches12[i]));
            mvbMatched1[i] = true; // 标记帧1该点有有效匹配
        } else {
            mvbMatched1[i] = false; // 无匹配
        }
    }

    const int N = mvMatches12.size(); // 有效匹配对总数量

    // Indices for minimum set selection
    // 全部匹配索引数组，用于RANSAC随机采样
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    // 填充0~N-1全部索引，作为采样候选池
    for (int i = 0; i < N; i++) {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    // 预先生成所有RANSAC迭代的最小采样集（每组8点），避免迭代内重复随机计算
    mvSets = vector<vector<size_t> >(mMaxIterations, vector<size_t>(8, 0));

    DUtils::Random::SeedRandOnce(0); // 初始化随机数种子

    // 为每一次迭代生成8个不重复的随机采样索引
    for (int it = 0; it < mMaxIterations; it++) {
        vAvailableIndices = vAllIndices; // 重置可用索引池

        // Select a minimum set
        for (size_t j = 0; j < 8; j++) {
            // 随机生成[0, 可用索引数-1]范围内整数
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx; // 保存本次采样的匹配对索引

            // 不放回采样：用末尾元素覆盖选中位置，弹出末尾，实现O(1)删除
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    // 并行计算：两个线程分别计算单应矩阵H和基础矩阵F，提升速度
    vector<bool> vbMatchesInliersH, vbMatchesInliersF; // 单应/基础模型对应的内点标记
    float SH, SF; // 单应/基础模型的RANSAC得分
    Eigen::Matrix3f H, F; // 最优单应矩阵H₂₁、最优基础矩阵F₂₁

    // 线程1：计算最优单应矩阵
    thread threadH(&TwoViewReconstruction::FindHomography, this,
                 ref(vbMatchesInliersH), ref(SH), ref(H));
    // 线程2：计算最优基础矩阵
    thread threadF(&TwoViewReconstruction::FindFundamental, this,
                 ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    // 等待两个计算线程执行完毕
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    // 两个模型得分都为0，无法判断模型有效性，直接失败
    if (SH + SF == 0.f) return false;
    // 计算单应得分占总得分的比例RH，用于判断场景平面性
    float RH = SH / (SH + SF);

    float minParallax = 1.0; // 最小视差角阈值，单位：度

    // Try to reconstruct from homography or fundamental depending on the ratio
    // (0.40-0.45)
    // RH>0.5说明场景趋近平面，优先用单应矩阵重建
    if (RH > 0.50) {
        // if(RH>0.40)
        // cout << "Initialization from Homography" << endl;
        // 用单应矩阵方案重建运动与3D点
        return ReconstructH(vbMatchesInliersH, H, mK, T21, vP3D, vbTriangulated,
                            minParallax, 50);
    } else {
        // if(pF_HF>0.6)
        // cout << "Initialization from Fundamental" << endl;
        // 用基础矩阵方案重建运动与3D点
        return ReconstructF(vbMatchesInliersF, F, mK, T21, vP3D, vbTriangulated,
                            minParallax, 50);
    }
}

/**
 * @brief RANSAC求解最优单应矩阵H₂₁：满足x₁ = H₂₁ · x₂，描述平面场景两视图投影变换
 * @param vbMatchesInliers [out] 最优模型对应的内点标记数组，长度等于匹配对总数
 * @param score [out] 最优模型的RANSAC总得分
 * @param H21 [out] 最优单应矩阵（帧2 → 帧1）
 */
void TwoViewReconstruction::FindHomography(vector<bool> &vbMatchesInliers,
                                           float &score, Eigen::Matrix3f &H21) {
    // Number of putative matches
    const int N = mvMatches12.size(); // 匹配对总数

    // Normalize coordinates
    // 特征点坐标归一化，提升DLT直接线性变换求解的数值稳定性
    vector<cv::Point2f> vPn1, vPn2; // 归一化后的两帧特征点坐标
    Eigen::Matrix3f T1, T2; // 帧1、帧2的归一化变换矩阵
    Normalize(mvKeys1, vPn1, T1); // 对帧1特征点做归一化
    Normalize(mvKeys2, vPn2, T2); // 对帧2特征点做归一化
    Eigen::Matrix3f T2inv = T2.inverse(); // T2求逆，用于反归一化H矩阵

    // Best Results variables
    score = 0.0; // 初始化最高得分为0
    vbMatchesInliers = vector<bool>(N, false); // 内点标记初始全为外点

    // Iteration variables
    // 单次迭代临时变量
    vector<cv::Point2f> vPn1i(8); // 本次采样的帧1归一化点
    vector<cv::Point2f> vPn2i(8); // 本次采样的帧2归一化点
    Eigen::Matrix3f H21i, H12i; // 本次迭代求解的H₂₁、逆矩阵H₁₂
    vector<bool> vbCurrentInliers(N, false); // 本次迭代内点标记
    float currentScore; // 本次迭代得分

    // Perform all RANSAC iterations and save the solution with highest score
    // 遍历所有RANSAC迭代，保留得分最高的模型
    for (int it = 0; it < mMaxIterations; it++) {
        // Select a minimum set
        // 取出预生成的8个采样点，填入临时数组
        for (size_t j = 0; j < 8; j++) {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        // 用8对归一化点求解归一化空间下的单应矩阵Hn
        Eigen::Matrix3f Hn = ComputeH21(vPn1i, vPn2i);
        // 反归一化：还原到原始像素坐标系下的H₂₁
        H21i = T2inv * Hn * T1;
        // 求逆得到H₁₂，用于双向重投影误差校验
        H12i = H21i.inverse();

        // 计算当前H模型的内点数量与总得分
        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

        // 当前得分高于历史最高，更新最优解
        if (currentScore > score) {
            H21 = H21i;
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

/**
 * @brief RANSAC求解最优基础矩阵F₂₁：满足x₂ᵀ · F₂₁ · x₁ = 0，描述两视图对极几何约束
 * @param vbMatchesInliers [out] 最优模型对应的内点标记
 * @param score [out] 最优模型总得分
 * @param F21 [out] 最优基础矩阵（帧1 → 帧2）
 */
void TwoViewReconstruction::FindFundamental(vector<bool> &vbMatchesInliers,
                                            float &score,
                                            Eigen::Matrix3f &F21) {
    // Number of putative matches
    const int N = vbMatchesInliers.size(); // 匹配对总数

    // Normalize coordinates
    // 特征点归一化，提升8点法基础矩阵求解的数值稳定性
    vector<cv::Point2f> vPn1, vPn2;
    Eigen::Matrix3f T1, T2;
    Normalize(mvKeys1, vPn1, T1);
    Normalize(mvKeys2, vPn2, T2);
    Eigen::Matrix3f T2t = T2.transpose(); // T2转置，用于反归一化F

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N, false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    Eigen::Matrix3f F21i; // 本次迭代基础矩阵
    vector<bool> vbCurrentInliers(N, false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < mMaxIterations; it++) {
        // Select a minimum set
        // 取出8个采样匹配对
        for (int j = 0; j < 8; j++) {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        // 8点法计算归一化空间下的基础矩阵Fn
        Eigen::Matrix3f Fn = ComputeF21(vPn1i, vPn2i);

        // 反归一化：还原原始像素坐标系下的F₂₁：F = T₂ᵀ · Fn · T₁
        F21i = T2t * Fn * T1;

        // 计算当前F模型的内点与得分
        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        // 更新最优解
        if (currentScore > score) {
            F21 = F21i;
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

/**
 * @brief DLT直接线性变换法求解单应矩阵H：x₁ = H · x₂，标准8点算法
 * @param vP1 帧1归一化特征点
 * @param vP2 帧2归一化特征点
 * @return 3×3单应矩阵H
 */
Eigen::Matrix3f TwoViewReconstruction::ComputeH21(
    const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2) {
    const int N = vP1.size(); // 点对数，RANSAC采样固定为8

    // 构造2N×9的齐次线性方程组Ah=0，h为H矩阵拉直的9维向量
    Eigen::MatrixXf A(2 * N, 9);

    // 逐点填充A矩阵，每对特征点贡献两行约束
    for (int i = 0; i < N; i++) {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        // 第一行约束：[0, 0, 0, -u₁, -v₁, -1, v₂u₁, v₂v₁, v₂]
        A(2 * i, 0) = 0.0;
        A(2 * i, 1) = 0.0;
        A(2 * i, 2) = 0.0;
        A(2 * i, 3) = -u1;
        A(2 * i, 4) = -v1;
        A(2 * i, 5) = -1;
        A(2 * i, 6) = v2 * u1;
        A(2 * i, 7) = v2 * v1;
        A(2 * i, 8) = v2;

        // 第二行约束：[u₁, v₁, 1, 0, 0, 0, -u₂u₁, -u₂v₁, -u₂]
        A(2 * i + 1, 0) = u1;
        A(2 * i + 1, 1) = v1;
        A(2 * i + 1, 2) = 1;
        A(2 * i + 1, 3) = 0.0;
        A(2 * i + 1, 4) = 0.0;
        A(2 * i + 1, 5) = 0.0;
        A(2 * i + 1, 6) = -u2 * u1;
        A(2 * i + 1, 7) = -u2 * v1;
        A(2 * i + 1, 8) = -u2;
    }

    // 对A做SVD分解，V矩阵最小奇异值对应的列就是H的最小二乘解
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);

    // 取出V矩阵最后一列（第8列），reshape为3×3行优先矩阵，即为单应H
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> H(svd.matrixV().col(8).data());

    return H;
}

/**
 * @brief 8点法求解基础矩阵F，归一化8点算法，强制秩2约束
 * @param vP1 帧1归一化特征点
 * @param vP2 帧2归一化特征点
 * @return 3×3基础矩阵F
 */
Eigen::Matrix3f TwoViewReconstruction::ComputeF21(
    const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2) {
    const int N = vP1.size(); // 匹配点对数

    // 构造N×9的齐次线性方程组Af=0，f为F拉直的9维向量
    Eigen::MatrixXf A(N, 9);

    // 逐点填充A矩阵，每行对应一个对极约束：u₂u₁f₁₁ + u₂v₁f₁₂ + u₂f₁₃ + v₂u₁f₂₁ + v₂v₁f₂₂ + v₂f₂₃ + u₁f₃₁ + v₁f₃₂ + f₃₃ = 0
    for (int i = 0; i < N; i++) {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A(i, 0) = u2 * u1;
        A(i, 1) = u2 * v1;
        A(i, 2) = u2;
        A(i, 3) = v2 * u1;
        A(i, 4) = v2 * v1;
        A(i, 5) = v2;
        A(i, 6) = u1;
        A(i, 7) = v1;
        A(i, 8) = 1;
    }

    // 第一次SVD求F的初始解
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // V矩阵最后一列reshape为3×3，得到初始基础矩阵Fpre
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Fpre(svd.matrixV().col(8).data());

    // 第二次SVD，强制基础矩阵秩为2：将最小奇异值置0
    Eigen::JacobiSVD<Eigen::Matrix3f> svd2(
        Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f w = svd2.singularValues();
    w(2) = 0; // 第三个奇异值置0，满足基础矩阵秩为2的约束

    // 重构秩2的基础矩阵F = U · diag(w) · Vᵀ
    return svd2.matrixU() * Eigen::DiagonalMatrix<float, 3>(w) *
           svd2.matrixV().transpose();
}

/**
 * @brief 校验单应矩阵，双向重投影计算卡方误差，统计内点与总得分
 * @param H21 帧2到帧1的单应矩阵
 * @param H12 帧1到帧2的单应矩阵（H21的逆）
 * @param vbMatchesInliers [out] 每个匹配对是否为内点
 * @param sigma 像素误差标准差
 * @return 单应模型总得分，所有内点卡方距离余量之和
 */
float TwoViewReconstruction::CheckHomography(const Eigen::Matrix3f &H21,
                                             const Eigen::Matrix3f &H12,
                                             vector<bool> &vbMatchesInliers,
                                             float sigma) {
    const int N = mvMatches12.size(); // 匹配对总数

    // 提取H₂₁各元素，避免矩阵下标重复计算，提升性能
    const float h11 = H21(0, 0);
    const float h12 = H21(0, 1);
    const float h13 = H21(0, 2);
    const float h21 = H21(1, 0);
    const float h22 = H21(1, 1);
    const float h23 = H21(1, 2);
    const float h31 = H21(2, 0);
    const float h32 = H21(2, 1);
    const float h33 = H21(2, 2);

    // 提取H₁₂各元素
    const float h11inv = H12(0, 0);
    const float h12inv = H12(0, 1);
    const float h13inv = H12(0, 2);
    const float h21inv = H12(1, 0);
    const float h22inv = H12(1, 1);
    const float h23inv = H12(1, 2);
    const float h31inv = H12(2, 0);
    const float h32inv = H12(2, 1);
    const float h33inv = H12(2, 2);

    vbMatchesInliers.resize(N);
    float score = 0;
    const float th = 5.991; // 卡方分布2自由度95%置信阈值 5.991
    const float invSigmaSquare = 1.0 / (sigma * sigma); // 1/σ²

    // 遍历所有匹配对，双向计算重投影误差
    for (int i = 0; i < N; i++) {
        bool bIn = true;

        // 取出两帧对应特征点
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x₂in1 = H₁₂·x₂  将帧2点投影到帧1，计算与观测点的误差
        const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
        const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
        const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

        // 帧1上的重投影误差平方
        const float squareDist1 =
            (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

        // 卡方值 = 误差平方 / σ²
        const float chiSquare1 = squareDist1 * invSigmaSquare;

        // 超过阈值标记为外点
        if (chiSquare1 > th)
            bIn = false;
        else
            score += th - chiSquare1; // 得分累加：阈值-卡方值，误差越小得分越高

        // Reprojection error in second image
        // x₁in2 = H₂₁·x₁  将帧1点投影到帧2
        const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
        const float u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
        const float v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

        // 帧2上的重投影误差平方
        const float squareDist2 =
            (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += th - chiSquare2;

        // 保存该匹配对内点标记
        if (bIn)
            vbMatchesInliers[i] = true;
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}

/**
 * @brief 校验基础矩阵，计算点到对极线距离的卡方误差，统计内点与总得分
 * @param F21 基础矩阵 帧1→帧2
 * @param vbMatchesInliers [out] 内点标记数组
 * @param sigma 像素误差标准差
 * @return 基础模型总得分
 */
float TwoViewReconstruction::CheckFundamental(const Eigen::Matrix3f &F21,
                                            vector<bool> &vbMatchesInliers,
                                            float sigma) {
    const int N = mvMatches12.size();

    // 提取F₂₁各元素
    const float f11 = F21(0, 0);
    const float f12 = F21(0, 1);
    const float f13 = F21(0, 2);
    const float f21 = F21(1, 0);
    const float f22 = F21(1, 1);
    const float f23 = F21(1, 2);
    const float f31 = F21(2, 0);
    const float f32 = F21(2, 1);
    const float f33 = F21(2, 2);

    vbMatchesInliers.resize(N);
    float score = 0;
    const float th = 3.841; // 卡方分布1自由度95%置信阈值 3.841
    const float thScore = 5.991; // 得分基准阈值，与单应得分保持量级一致
    const float invSigmaSquare = 1.0 / (sigma * sigma);

    // 遍历所有匹配对
    for (int i = 0; i < N; i++) {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l₂=F₂₁x₁=(a₂,b₂,c₂)  帧1点在帧2上对应的极线
        const float a2 = f11 * u1 + f12 * v1 + f13;
        const float b2 = f21 * u1 + f22 * v1 + f23;
        const float c2 = f31 * u1 + f32 * v1 + f33;

        // 点到直线距离平方：(ax+by+c)² / (a²+b²)
        const float num2 = a2 * u2 + b2 * v2 + c2;
        const float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l₁ =x₂ᵀF₂₁=(a₁,b₁,c₁)  帧2点在帧1上对应的极线
        const float a1 = f11 * u2 + f21 * v2 + f31;
        const float b1 = f12 * u2 + f22 * v2 + f32;
        const float c1 = f13 * u2 + f23 * v2 + f33;

        const float num1 = a1 * u1 + b1 * v1 + c1;
        const float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if (bIn)
            vbMatchesInliers[i] = true;
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}

/**
 * @brief 从基础矩阵出发，分解本质矩阵得到4种运动假设，三角化验证选出正确解
 * @param vbMatchesInliers RANSAC筛选的内点标记
 * @param F21 基础矩阵F₂₁
 * @param K 相机内参矩阵
 * @param T21 [out] 输出帧2到帧1的SE3运动
 * @param vP3D [out] 输出三角化3D点数组
 * @param vbTriangulated [out] 点是否成功三角化标记
 * @param minParallax 最小视差角阈值，单位度
 * @param minTriangulated 最少有效三角化点数阈值
 * @return true重建成功；false失败
 */
bool TwoViewReconstruction::ReconstructF(
    vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21, Eigen::Matrix3f &K,
    Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,
    float minParallax, int minTriangulated) {
    int N = 0;
    // 统计内点总数量
    for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
        if (vbMatchesInliers[i]) N++;

    // Compute Essential Matrix from Fundamental Matrix
    // 基础矩阵转本质矩阵：E = Kᵀ · F · K
    Eigen::Matrix3f E21 = K.transpose() * F21 * K;

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t;
    // Recover the 4 motion hypotheses
    // 分解本质矩阵，得到2个旋转R1、R2和1个平移t，共4种运动组合
    DecomposeE(E21, R1, R2, t);

    Eigen::Vector3f t1 = t;
    Eigen::Vector3f t2 = -t;

    // Reconstruct with the 4 hyphoteses and check
    // 4种运动假设分别三角化验证
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1, vbTriangulated2, vbTriangulated3,
        vbTriangulated4;
    float parallax1, parallax2, parallax3, parallax4;

    // 4种组合：(R1,t1), (R2,t1), (R1,t2), (R2,t2)
    int nGood1 = CheckRT(R1, t1, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers,
                       K, vP3D1, 4.0 * mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2, t1, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers,
                       K, vP3D2, 4.0 * mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1, t2, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers,
                       K, vP3D3, 4.0 * mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2, t2, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers,
                       K, vP3D4, 4.0 * mSigma2, vbTriangulated4, parallax4);

    // 找出4种假设中有效三角化点最多的数量
    int maxGood = max(nGood1, max(nGood2, max(nGood3, nGood4)));
    // 最少有效点数阈值：总内点90% 与 minTriangulated 取较大值
    int nMinGood = max(static_cast<int>(0.9 * N), minTriangulated);

    int nsimilar = 0;
    // 统计和最优解差距在30%以内的解的数量，判断是否存在运动歧义
    if (nGood1 > 0.7 * maxGood) nsimilar++;
    if (nGood2 > 0.7 * maxGood) nsimilar++;
    if (nGood3 > 0.7 * maxGood) nsimilar++;
    if (nGood4 > 0.7 * maxGood) nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject
    // initialization
    // 最优解不满足最少点数，或者存在多个相近解（歧义），重建失败
    if (maxGood < nMinGood || nsimilar > 1) {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    // 根据最优解索引，输出对应结果
    if (maxGood == nGood1) {
        if (parallax1 > minParallax) { // 视差角足够
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            T21 = Sophus::SE3f(R1, t1); // 构造SE3变换
            return true;
        }
    } else if (maxGood == nGood2) {
        if (parallax2 > minParallax) {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            T21 = Sophus::SE3f(R2, t1);
            return true;
        }
    } else if (maxGood == nGood3) {
        if (parallax3 > minParallax) {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            T21 = Sophus::SE3f(R1, t2);
            return true;
        }
    } else if (maxGood == nGood4) {
        if (parallax4 > minParallax) {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            T21 = Sophus::SE3f(R2, t2);
            return true;
        }
    }

    return false;
}

/**
 * @brief 从单应矩阵出发，Faugeras平面运动分解法得到8种运动假设，三角化验证选出最优解
 * @param vbMatchesInliers RANSAC内点标记
 * @param H21 单应矩阵H₂₁
 * @param K 相机内参
 * @param T21 [out] 输出SE3运动
 * @param vP3D [out] 输出3D点数组
 * @param vbTriangulated [out] 三角化标记
 * @param minParallax 最小视差角
 * @param minTriangulated 最少有效点数
 * @return true重建成功
 */
bool TwoViewReconstruction::ReconstructH(
    vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21, Eigen::Matrix3f &K,
    Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,
    float minParallax, int minTriangulated) {
    int N = 0;
    // 统计内点总数
    for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
        if (vbMatchesInliers[i]) N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence,
    // 1988
    // 单应分解运动：先将H转换到归一化相机坐标系 A = K⁻¹ · H · K
    Eigen::Matrix3f invK = K.inverse();
    Eigen::Matrix3f A = invK * H21 * K;

    // 对A做SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(
        A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    Eigen::Matrix3f Vt = V.transpose();
    Eigen::Vector3f w = svd.singularValues(); // 奇异值 d₁≥d₂≥d₃

    float s = U.determinant() * Vt.determinant(); // 符号因子，保证旋转行列式为+1
    float d1 = w(0);
    float d2 = w(1);
    float d3 = w(2);

    // 奇异值过于接近，平面退化严重，无法分解运动
    if (d1 / d2 < 1.0001 || d2 / d3 < 1.0001) {
        return false;
    }

    vector<Eigen::Matrix3f> vR; // 8种旋转
    vector<Eigen::Vector3f> vt, vn; // 平移、平面法向量
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    // n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    // 计算第一组4个解（d'=d₂）
    float aux1 = sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    float aux3 = sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    float x1[] = {aux1, aux1, -aux1, -aux1};
    float x3[] = {aux3, -aux3, aux3, -aux3};

    // case d'=d2
    float aux_stheta =
        sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);
    float ctheta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    // 第一组4种旋转、平移、法向量
    for (int i = 0; i < 4; i++) {
        Eigen::Matrix3f Rp;
        Rp.setZero();
        Rp(0, 0) = ctheta;
        Rp(0, 2) = -stheta[i];
        Rp(1, 1) = 1.f;
        Rp(2, 0) = stheta[i];
        Rp(2, 2) = ctheta;

        // R = s · U · Rp · Vᵀ
        Eigen::Matrix3f R = s * U * Rp * Vt;
        vR.push_back(R);

        Eigen::Vector3f tp;
        tp(0) = x1[i];
        tp(1) = 0;
        tp(2) = -x3[i];
        tp *= d1 - d3;

        Eigen::Vector3f t = U * tp;
        vt.push_back(t / t.norm()); // 平移归一化

        Eigen::Vector3f np;
        np(0) = x1[i];
        np(1) = 0;
        np(2) = x3[i];

        Eigen::Vector3f n = V * np;
        if (n(2) < 0) n = -n; // 保证法向z分量为正，场景在相机前方
        vn.push_back(n);
    }

    // case d'=-d2  第二组4个解
    float aux_sphi =
        sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);
    float cphi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for (int i = 0; i < 4; i++) {
        Eigen::Matrix3f Rp;
        Rp.setZero();
        Rp(0, 0) = cphi;
        Rp(0, 2) = sphi[i];
        Rp(1, 1) = -1;
        Rp(2, 0) = sphi[i];
        Rp(2, 2) = -cphi;

        Eigen::Matrix3f R = s * U * Rp * Vt;
        vR.push_back(R);

        Eigen::Vector3f tp;
        tp(0) = x1[i];
        tp(1) = 0;
        tp(2) = x3[i];
        tp *= d1 + d3;

        Eigen::Vector3f t = U * tp;
        vt.push_back(t / t.norm());

        Eigen::Vector3f np;
        np(0) = x1[i];
        np(1) = 0;
        np(2) = x3[i];

        Eigen::Vector3f n = V * np;
        if (n(2) < 0) n = -n;
        vn.push_back(n);
    }

    int bestGood = 0;
    int secondBestGood = 0;
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras'
    // paper (which could fail for points seen with low parallax) We reconstruct
    // all hypotheses and check in terms of triangulated points and parallax
    // 遍历8种假设，三角化验证，选出最优解
    for (size_t i = 0; i < 8; i++) {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        // 检查当前R,t下的三角化有效点数
        int nGood =
            CheckRT(vR[i], vt[i], mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers,
                    K, vP3Di, 4.0 * mSigma2, vbTriangulatedi, parallaxi);

        // 更新最优、次优解
        if (nGood > bestGood) {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        } else if (nGood > secondBestGood) {
            secondBestGood = nGood;
        }
    }

    // 次优解远小于最优解、视差足够、有效点足够、有效点占比超过90%，判定重建成功
    if (secondBestGood < 0.75 * bestGood && bestParallax >= minParallax &&
        bestGood > minTriangulated && bestGood > 0.9 * N) {
        T21 = Sophus::SE3f(vR[bestSolutionIdx], vt[bestSolutionIdx]);
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

/**
 * @brief 特征点坐标归一化：去均值、单位平均偏差，提升DLT类算法数值稳定性
 * @param vKeys 输入特征点数组
 * @param vNormalizedPoints [out] 归一化后的点坐标
 * @param T [out] 归一化变换矩阵 3×3
 */
void TwoViewReconstruction::Normalize(const vector<cv::KeyPoint> &vKeys,
                                      vector<cv::Point2f> &vNormalizedPoints,
                                      Eigen::Matrix3f &T) {
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    // 计算x,y坐标均值
    for (int i = 0; i < N; i++) {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }
    meanX = meanX / N;
    meanY = meanY / N;

    float meanDevX = 0;
    float meanDevY = 0;

    // 去均值，计算平均绝对偏差
    for (int i = 0; i < N; i++) {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }
    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;

    // 缩放因子：使平均绝对偏差为1
    float sX = 1.0 / meanDevX;
    float sY = 1.0 / meanDevY;

    // 归一化坐标缩放
    for (int i = 0; i < N; i++) {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    // 构造归一化变换矩阵 T = diag(sX,sY,1) · T(-meanX,-meanY)
    T.setZero();
    T(0, 0) = sX;
    T(1, 1) = sY;
    T(0, 2) = -meanX * sX;
    T(1, 2) = -meanY * sY;
    T(2, 2) = 1.f;
}

/**
 * @brief 给定旋转R和平移t，线性三角化所有匹配点，校验重投影误差，统计有效点与视差角
 * @param R 旋转矩阵 帧2→帧1
 * @param t 平移向量 帧2→帧1
 * @param vKeys1 帧1特征点
 * @param vKeys2 帧2特征点
 * @param vMatches12 匹配对数组
 * @param vbMatchesInliers 内点标记
 * @param K 相机内参
 * @param vP3D [out] 三角化得到的3D点数组，索引对应帧1特征索引
 * @param th2 重投影误差平方阈值
 * @param vbGood [out] 每个点是否成功三角化且有效
 * @param parallax [out] 视差角，单位度
 * @return 有效三角化点的数量
 */
int TwoViewReconstruction::CheckRT(
    const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
    const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
    const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
    const Eigen::Matrix3f &K, vector<cv::Point3f> &vP3D, float th2,
    vector<bool> &vbGood, float &parallax) {
    // Calibration parameters
    // 提取内参参数
    const float fx = K(0, 0);
    const float fy = K(1, 1);
    const float cx = K(0, 2);
    const float cy = K(1, 2);

    vbGood = vector<bool>(vKeys1.size(), false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax; // 保存各有效点的视差余弦值
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    // 相机1投影矩阵 P1 = K·[I | 0]
    Eigen::Matrix<float, 3, 4> P1;
    P1.setZero();
    P1.block<3, 3>(0, 0) = K;

    Eigen::Vector3f O1; // 相机1光心，位于原点
    O1.setZero();

    // Camera 2 Projection Matrix K[R|t]
    // 相机2投影矩阵 P2 = K·[R | t]
    Eigen::Matrix<float, 3, 4> P2;
    P2.block<3, 3>(0, 0) = R;
    P2.block<3, 1>(0, 3) = t;
    P2 = K * P2;

    Eigen::Vector3f O2 = -R.transpose() * t; // 相机2光心在帧1坐标系下的位置

    int nGood = 0;

    // 遍历所有匹配对
    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
        if (!vbMatchesInliers[i]) continue; // 跳过外点

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

        Eigen::Vector3f p3dC1; // 三角化得到的3D点（相机1坐标系）
        Eigen::Vector3f x_p1(kp1.pt.x, kp1.pt.y, 1);
        Eigen::Vector3f x_p2(kp2.pt.x, kp2.pt.y, 1);

        // 线性三角化法求解3D点
        GeometricTools::Triangulate(x_p1, x_p2, P1, P2, p3dC1);

        // 结果含NaN/Inf，无效点跳过
        if (!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2))) {
            vbGood[vMatches12[i].first] = false;
            continue;
        }

        // Check parallax
        // 计算视差角：两个相机光心与3D点夹角的余弦值
        Eigen::Vector3f normal1 = p3dC1 - O1;
        float dist1 = normal1.norm();

        Eigen::Vector3f normal2 = p3dC1 - O2;
        float dist2 = normal2.norm();

        float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

        // Check depth in front of first camera (only if enough parallax, as
        // "infinite" points can easily go to negative depth)
        // 深度为正，或者视差接近0（无穷远点）时接受
        if (p3dC1(2) <= 0 && cosParallax < 0.9998) continue;

        // Check depth in front of second camera (only if enough parallax, as
        // "infinite" points can easily go to negative depth)
        // 变换到相机2坐标系，检查深度为正
        Eigen::Vector3f p3dC2 = R * p3dC1 + t;

        if (p3dC2(2) <= 0 && cosParallax < 0.9998) continue;

        // Check reprojection error in first image
        // 重投影到帧1，计算像素误差
        float im1x, im1y;
        float invZ1 = 1.0 / p3dC1(2);
        im1x = fx * p3dC1(0) * invZ1 + cx;
        im1y = fy * p3dC1(1) * invZ1 + cy;

        float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) +
                          (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

        if (squareError1 > th2) continue; // 超过误差阈值，无效

        // Check reprojection error in second image
        // 重投影到帧2
        float im2x, im2y;
        float invZ2 = 1.0 / p3dC2(2);
        im2x = fx * p3dC2(0) * invZ2 + cx;
        im2y = fy * p3dC2(1) * invZ2 + cy;

        float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) +
                          (im2y - kp2.pt.y) * (im2y - kp2.pt.y);

        if (squareError2 > th2) continue;

        // 有效点：保存视差余弦，3D点坐标
        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
        nGood++;

        // 视差足够大时标记为good点
        if (cosParallax < 0.9998) vbGood[vMatches12[i].first] = true;
    }

    if (nGood > 0) {
        // 视差余弦排序，取第50个（从小到大）对应的角度作为视差角，避免外点干扰
        sort(vCosParallax.begin(), vCosParallax.end());

        size_t idx = min(50, static_cast<int>(vCosParallax.size() - 1));
        parallax = acos(vCosParallax[idx]) * 180 / CV_PI; // 弧度转角度
    } else {
        parallax = 0;
    }

    return nGood;
}

/**
 * @brief 本质矩阵分解：得到2个可能旋转和1个平移，共4种运动组合
 * @param E 本质矩阵
 * @param R1 [out] 旋转解1
 * @param R2 [out] 旋转解2
 * @param t [out] 平移向量（单位长度）
 */
void TwoViewReconstruction::DecomposeE(const Eigen::Matrix3f &E,
                                       Eigen::Matrix3f &R1, Eigen::Matrix3f &R2,
                                       Eigen::Vector3f &t) {
    // 本质矩阵SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(
        E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f Vt = svd.matrixV().transpose();

    // 平移向量为U的第三列
    t = U.col(2);
    t = t / t.norm(); // 平移归一化

    // 构造反对称矩阵W，对应绕z轴旋转90度
    Eigen::Matrix3f W;
    W.setZero();
    W(0, 1) = -1;
    W(1, 0) = 1;
    W(2, 2) = 1;

    // 第一种旋转 R₁ = U · W · Vᵀ
    R1 = U * W * Vt;
    // 保证旋转行列式为+1（右手系旋转矩阵）
    if (R1.determinant() < 0) R1 = -R1;

    // 第二种旋转 R₂ = U · Wᵀ · Vᵀ
    R2 = U * W.transpose() * Vt;
    if (R2.determinant() < 0) R2 = -R2;
}

}  // namespace ORB_SLAM3
