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

/*===========================================================================
 * 文件名称：MLPnPsolver.cc  —— ORB-SLAM3 中的 MLPnP 位姿求解器（Maximum Likelihood PnP）
 * 作者来源：Steffen Urban（ANU），MLPnP 论文的参考实现，被集成进 ORB-SLAM3。
 * 核心作用：
 *   已知若干组 “2D 观测(像素/观测方向 bearing) <-> 3D 地图点(世界系)” 对应，
 *   用 RANSAC + MLPnP 闭式解 + 高斯-牛顿迭代，求出相机的世界->相机位姿 Tcw。
 * 与传统 PnP 的区别：
 *   - 直接使用相机模型反投影得到的单位观测方向(bearing vector)，天然适配鱼眼等任意相机模型；
 *   - 用每个观测方向的零空间(nullspace)构造设计矩阵，最小化角度(余弦)误差而非像素重投影；
 *   - 可融合每个观测的协方差(本实现默认不传协方差，按等权处理)。
 * 阅读主线：构造(收集对应) -> iterate(RANSAC 外循环) -> computePose(一次 MLPnP 求解)
 *           -> 平面/非平面分支恢复 R,t -> mlpnp_gn 高斯牛顿精修 -> CheckInliers/Refine。
 *===========================================================================*/
#include "MLPnPsolver.h"
#include <Eigen/Sparse>  // Eigen 稀疏矩阵：随机模型权重 P 是块对角稀疏阵
#include <algorithm>  // STL 算法：min/max/ceil/log/min_element 等
#include <limits>  // numeric_limits：罗德里格斯公式中判断角度是否接近 0
#include <memory>  // 智能指针支持
#include <vector>  // STL 动态数组

namespace ORB_SLAM3 {  // 进入 ORB-SLAM3 命名空间

/*===========================================================================
 * 构造函数：从当前帧 F 与 “特征索引->地图点” 匹配数组中收集有效 2D-3D 对应。
 * 同时计算每个特征的观测方向(bearing)、尺度方差 sigma^2，并初始化 RANSAC 参数。
 *===========================================================================*/
MLPnPsolver::MLPnPsolver(const std::shared_ptr<Frame> &F,
                         const vector<MapPoint *> &vpMapPointMatches)
    : mnInliersi(0),  // 本轮迭代内点数先清零
      mnIterations(0),  // RANSAC 累计迭代次数清零
      mnBestInliers(0),  // 历史最佳内点数清零
      N(0),  // 有效对应数量，稍后在 SetRansacParameters 中赋值
      mpCamera(F->mpCamera) {  // 保存几何相机模型(针孔/鱼眼)，用于 unproject/project
  mvpMapPointMatches = vpMapPointMatches;  // 保存“特征->地图点”匹配数组(可能含 nullptr)
  mvBearingVecs.reserve(F->mvpMapPoints.size());  // 预分配：观测方向向量数组
  mvP2D.reserve(F->mvpMapPoints.size());  // 预分配：2D 像素坐标数组
  mvSigma2.reserve(F->mvpMapPoints.size());  // 预分配：每点尺度方差(用于内点阈值加权)
  mvP3Dw.reserve(F->mvpMapPoints.size());  // 预分配：3D 世界点数组
  mvKeyPointIndices.reserve(F->mvpMapPoints.size());  // 预分配：对应在帧关键点中的原始索引
  mvAllIndices.reserve(F->mvpMapPoints.size());  // 预分配：在“有效对应数组”中的连续索引 0..N-1

  int idx = 0;  // 有效对应计数器(只统计非空且非坏点)
  // 遍历所有匹配槽位，筛出真正可用的 2D-3D 对应
  for (size_t i = 0, iend = mvpMapPointMatches.size(); i < iend; i++) {
    MapPoint *pMP = vpMapPointMatches[i];  // 取出第 i 个槽位对应的地图点(可能为空)

    if (pMP) {  // 该槽位必须有关联地图点
      if (!pMP->isBad()) {  // 且该地图点没有被标记为坏点(已剔除)
        if (i >= F->mvKeysUn.size()) continue;  // 越界保护：索引不能超过去畸变关键点数量
        const cv::KeyPoint &kp = F->mvKeysUn[i];  // 取去畸变后的第 i 个关键点

        mvP2D.push_back(kp.pt);  // 保存其像素坐标 (u,v)
        mvSigma2.push_back(F->mvLevelSigma2[kp.octave]);  // 按其金字塔层取尺度平方 sigma^2(层级越高容差越大)

        // Bearing vector should be normalized
        // 由相机模型把像素反投影为观测射线(bearing)，MLPnP 直接在射线上建模，故兼容鱼眼
        cv::Point3f cv_br = mpCamera->unproject(kp.pt);
        cv_br /= cv_br.z;  // 除以 z 做归一化尺度处理(使 z 分量为 1)
        bearingVector_t br(cv_br.x, cv_br.y, cv_br.z);  // 转成 Eigen 三维观测方向
        mvBearingVecs.push_back(br);  // 保存观测方向

        // 3D coordinates
        // 取该地图点的世界系 3D 坐标
        Eigen::Matrix<float, 3, 1> posEig = pMP->GetWorldPos();
        point_t pos(posEig(0), posEig(1), posEig(2));  // 转成 double 精度的 Eigen 点
        mvP3Dw.push_back(pos);  // 保存世界点

        mvKeyPointIndices.push_back(i);  // 记录它在帧关键点中的原始下标(回填内点要用)
        mvAllIndices.push_back(idx);  // 记录它在有效数组中的连续下标(RANSAC 抽样用)

        idx++;  // 有效对应计数 +1
      }
    }
  }

  SetRansacParameters();  // 依据有效对应数量设置 RANSAC 迭代次数与内点阈值
}

MLPnPsolver::~MLPnPsolver() {}  // 析构：成员均为 STL/Eigen 对象，可自动释放，函数体为空

// RANSAC methods
/*===========================================================================
 * iterate：RANSAC 主循环(可被 Tracking 分多次调用)。
 * 每次用最小集(mRansacMinSet 组对应)求一个位姿假设，统计内点；
 * 若出现更优假设则保存，并尝试用当前全部内点 Refine 精修后立即返回；
 * 达到最大迭代次数后返回历史最佳假设。
 * 输出：bNoMore(是否已穷尽迭代)、vbInliers(内点标记，按帧特征索引)、nInliers、Tout(位姿)。
 *===========================================================================*/
bool MLPnPsolver::iterate(int nIterations, bool &bNoMore,
                          vector<bool> &vbInliers, int &nInliers,
                          Eigen::Matrix4f &Tout) {
  Tout.setIdentity();  // 输出位姿先置单位阵
  bNoMore = false;
  vbInliers.clear();  // 清空内点输出
  nInliers = 0;

  if (N < mRansacMinInliers) {  // 有效对应总数连最低内点数都不够，直接判定无解
    bNoMore = true;  // 通知调用方 RANSAC 已穷尽
    return false;
  }

  vector<size_t> vAvailableIndices;

  int nCurrentIterations = 0;  // 本次函数调用内已执行的迭代计数
  // 循环条件：总迭代未超上限，或本次调用要求的 nIterations 尚未跑完
  while (mnIterations < mRansacMaxIts || nCurrentIterations < nIterations) {
    nCurrentIterations++;
    mnIterations++;

    vAvailableIndices = mvAllIndices;  // 复制一份可抽样索引池，每轮从中无放回抽样

    // Bearing vectors and 3D points used for this ransac iteration
    bearingVectors_t bearingVecs(mRansacMinSet);  // 本轮最小集的观测方向
    points_t p3DS(mRansacMinSet);  // 本轮最小集的 3D 点
    vector<int> indexes(mRansacMinSet);  // 最小集内部索引(0..mRansacMinSet-1)

    // Get min set of points
    // 无放回地随机抽取最小集(MLPnP 至少需要 6 组对应)
    for (short i = 0; i < mRansacMinSet; ++i) {
      int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);  // 在剩余索引池中均匀随机一个位置

      int idx = vAvailableIndices[randi];  // 取出对应的有效下标

      bearingVecs[i] = mvBearingVecs[idx];
      p3DS[i] = mvP3Dw[idx];
      indexes[i] = i;  // 最小集内部顺序索引

      // “与末尾交换后弹尾”实现 O(1) 无放回抽样
      vAvailableIndices[randi] = vAvailableIndices.back();
      vAvailableIndices.pop_back();  // 移除已抽到的末尾元素，避免重复
    }

    // By the moment, we are using MLPnP without covariance info
    cov3_mats_t covs(1);  // 协方差占位(长度1表示不提供逐点协方差，computePose 内按等权)

    // Result
    transformation_t result;  // 本次假设的 4x4 位姿结果

    // Compute camera pose
    computePose(bearingVecs, p3DS, covs, indexes, result);  // 用最小集跑一次完整 MLPnP，得到 R、t

    // Save result
    // 把结果旋转矩阵 R 拆到普通二维数组 mRi(供 CheckInliers 以 C 数组方式使用)
    mRi[0][0] = result(0, 0);
    mRi[0][1] = result(0, 1);
    mRi[0][2] = result(0, 2);

    mRi[1][0] = result(1, 0);
    mRi[1][1] = result(1, 1);
    mRi[1][2] = result(1, 2);

    mRi[2][0] = result(2, 0);
    mRi[2][1] = result(2, 1);
    mRi[2][2] = result(2, 2);

    // 把结果平移向量 t 拆到 mti
    mti[0] = result(0, 3);
    mti[1] = result(1, 3);
    mti[2] = result(2, 3);

    // Check inliers
    CheckInliers();  // 用本轮 R,t 把所有 3D 点投影，统计内点数 mnInliersi

    if (mnInliersi >= mRansacMinInliers) {  // 内点达到最低要求才认为该假设有效
      // If it is the best solution so far, save it
      if (mnInliersi > mnBestInliers) {  // 内点数刷新历史纪录 -> 保存为最佳假设
        mvbBestInliers = mvbInliersi;  // 记录最佳内点掩码
        mnBestInliers = mnInliersi;  // 更新最佳内点数

        cv::Mat Rcw(3, 3, CV_64F, mRi);  // 用 mRi 内存包装成 OpenCV 3x3 矩阵(不拷贝)
        cv::Mat tcw(3, 1, CV_64F, mti);  // 用 mti 包装成 3x1 平移向量
        Rcw.convertTo(Rcw, CV_32F);  // 旋转转成 float 精度
        tcw.convertTo(tcw, CV_32F);  // 平移转成 float 精度
        mBestTcw.setIdentity();  // 最佳位姿先置单位阵
        mBestTcw.block<3, 3>(0, 0) = Converter::toMatrix3f(Rcw);  // 写入旋转块 R
        mBestTcw.block<3, 1>(0, 3) = Converter::toVector3f(tcw);  // 写入平移块 t

        // 下面两行把结果再包成 Eigen 视图(本实现中未继续使用，属保留写法)
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> eigRcw(mRi[0]);
        Eigen::Vector3d eigtcw(mti);
      }

      if (Refine()) {  // 用最佳内点全集重新求解并精修；成功则直接返回精修结果
        nInliers = mnRefinedInliers;  // 输出精修后的内点数
        vbInliers = vector<bool>(mvpMapPointMatches.size(), false);  // 按“帧特征总数”建内点标记，先全 false
        // 把“有效数组内点掩码”映射回“帧特征索引”的输出掩码
        for (int i = 0; i < N; i++) {
          if (mvbRefinedInliers[i]) vbInliers[mvKeyPointIndices[i]] = true;  // 有效下标 i -> 帧原始索引
        }
        Tout = mRefinedTcw;  // 输出精修位姿
        return true;  // 精修成功
      }
    }
  }

  if (mnIterations >= mRansacMaxIts) {  // 迭代次数用尽：不再有新假设
    bNoMore = true;
    if (mnBestInliers >= mRansacMinInliers) {  // 历史上存在过合格假设，则输出最佳假设
      nInliers = mnBestInliers;
      vbInliers = vector<bool>(mvpMapPointMatches.size(), false);
      for (int i = 0; i < N; i++) {
        if (mvbBestInliers[i]) vbInliers[mvKeyPointIndices[i]] = true;  // 同样映射回帧特征索引
      }
      Tout = mBestTcw;  // 输出历史最佳位姿
      return true;
    }
  }

  return false;
}

/*===========================================================================
 * SetRansacParameters：依据对应数量自适应调整 RANSAC 参数。
 * 重点是用经典公式 N=log(1-p)/log(1-epsilon^s) 估计所需迭代次数，
 * 其中 p 为“至少抽到一次全内点集”的概率，epsilon 为内点比例，s=最小集大小(这里按 3 次方)。
 * 同时为每个点按 sigma^2*th2 计算各自的内点判定阈值。
 *===========================================================================*/
void MLPnPsolver::SetRansacParameters(double probability, int minInliers,
                                      int maxIterations, int minSet,
                                      float epsilon, float th2) {
  mRansacProb = probability;
  mRansacMinInliers = minInliers;
  mRansacMaxIts = maxIterations;
  mRansacEpsilon = epsilon;
  mRansacMinSet = minSet;

  N = mvP2D.size();  // number of correspondences  // 有效对应总数 number of correspondences

  mvbInliersi.resize(N);  // 本轮内点掩码按 N 分配

  // Adjust Parameters according to number of correspondences
  int nMinInliers = N * mRansacEpsilon;  // 按期望内点比例 epsilon 估算最低内点数
  if (nMinInliers < mRansacMinInliers) nMinInliers = mRansacMinInliers;  // 不低于构造时给定的下限
  if (nMinInliers < minSet) nMinInliers = minSet;  // 且不能少于最小求解集大小
  mRansacMinInliers = nMinInliers;

  // 若实际内点比例下限比传入 epsilon 还高，则抬高 epsilon，保证公式自洽
  if (mRansacEpsilon < static_cast<float>(mRansacMinInliers / N))
    mRansacEpsilon = static_cast<float>(mRansacMinInliers / N);

  // Set RANSAC iterations according to probability, epsilon, and max iterations
  int nIterations;

  if (mRansacMinInliers == N)  // 所有点都是内点时，一次迭代即可
    nIterations = 1;
  else
    nIterations = ceil(log(1 - mRansacProb) / log(1 - pow(mRansacEpsilon, 3)));  // RANSAC 迭代次数经典公式(指数 3 对应最小集)

  mRansacMaxIts = max(1, min(nIterations, mRansacMaxIts));  // 夹在 [1, 外部上限] 之间

  mvMaxError.resize(mvSigma2.size());  // 每个点一个内点误差阈值
  for (size_t i = 0; i < mvSigma2.size(); i++)
    mvMaxError[i] = mvSigma2[i] * th2;  // 阈值=该点尺度方差×基础卡方阈值 th2(尺度越大越宽松)
}

/*===========================================================================
 * CheckInliers：用当前假设 (mRi,mti) 把每个世界点变换到相机系并投影，
 * 与观测像素比较，误差平方小于该点阈值 mvMaxError 即判为内点，累计 mnInliersi。
 *===========================================================================*/
void MLPnPsolver::CheckInliers() {
  mnInliersi = 0;  // 内点计数清零

  for (int i = 0; i < N; i++) {
    point_t p = mvP3Dw[i];
    cv::Point3f P3Dw(p(0), p(1), p(2));  // 世界点转 OpenCV 坐标
    cv::Point2f P2D = mvP2D[i];  // 对应的观测像素

    // Pc = Rcw*Pw + tcw，逐行计算相机系坐标 xc
    float xc =
        mRi[0][0] * P3Dw.x + mRi[0][1] * P3Dw.y + mRi[0][2] * P3Dw.z + mti[0];
    float yc =  // 相机系 y 坐标
        mRi[1][0] * P3Dw.x + mRi[1][1] * P3Dw.y + mRi[1][2] * P3Dw.z + mti[1];
    float zc =  // 相机系 z(深度)坐标
        mRi[2][0] * P3Dw.x + mRi[2][1] * P3Dw.y + mRi[2][2] * P3Dw.z + mti[2];

    cv::Point3f P3Dc(xc, yc, zc);  // 组装相机系 3D 点
    cv::Point2f uv = mpCamera->project(P3Dc);  // 用相机模型投影回像素(兼容鱼眼)

    float distX = P2D.x - uv.x;  // x 方向重投影误差
    float distY = P2D.y - uv.y;  // y 方向重投影误差

    float error2 = distX * distX + distY * distY;  // 误差平方和

    if (error2 < mvMaxError[i]) {  // 小于该点(尺度相关)阈值 -> 内点
      mvbInliersi[i] = true;  // 标记为内点
      mnInliersi++;  // 内点计数 +1
    } else {
      mvbInliersi[i] = false;  // 否则标记为外点
    }
  }
}

/*===========================================================================
 * Refine：用当前最佳内点的全部对应重新跑一次 MLPnP(相当于用更多点做模型精修)，
 * 再统计内点；若仍满足最低内点数，保存精修位姿 mRefinedTcw 并返回 true。
 *===========================================================================*/
bool MLPnPsolver::Refine() {
  vector<int> vIndices;  // 收集最佳内点在有效数组中的下标
  vIndices.reserve(mvbBestInliers.size());  // 预留容量

  for (size_t i = 0; i < mvbBestInliers.size(); i++) {
    if (mvbBestInliers[i]) {  // 只收最佳内点
      vIndices.push_back(i);
    }
  }

  // Bearing vectors and 3D points used for this ransac iteration
  bearingVectors_t bearingVecs;  // 精修用观测方向(数量=内点数，不再是最小集)
  points_t p3DS;  // 精修用 3D 点
  vector<int> indexes;  // 精修集合内部顺序索引

  for (size_t i = 0; i < vIndices.size(); i++) {
    int idx = vIndices[i];

    bearingVecs.push_back(mvBearingVecs[idx]);  // 收集该内点观测方向
    p3DS.push_back(mvP3Dw[idx]);  // 收集该内点 3D 点
    indexes.push_back(i);  // 内部顺序号
  }

  // By the moment, we are using MLPnP without covariance info
  cov3_mats_t covs(1);

  // Result
  transformation_t result;

  // Compute camera pose
  computePose(bearingVecs, p3DS, covs, indexes, result);  // 用全部内点重新求位姿

  // Check inliers
  CheckInliers();

  mnRefinedInliers = mnInliersi;  // 记录精修后的内点统计
  mvbRefinedInliers = mvbInliersi;  // 记录精修后的内点掩码

  if (mnInliersi > mRansacMinInliers) {  // 精修后内点仍达标才接受
    cv::Mat Rcw(3, 3, CV_64F, mRi);
    cv::Mat tcw(3, 1, CV_64F, mti);
    Rcw.convertTo(Rcw, CV_32F);
    tcw.convertTo(tcw, CV_32F);
    mRefinedTcw.setIdentity();  // 精修位姿先置单位阵

    mRefinedTcw.block<3, 3>(0, 0) = Converter::toMatrix3f(Rcw);  // 写旋转 R
    mRefinedTcw.block<3, 1>(0, 3) = Converter::toVector3f(tcw);  // 写平移 t

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> eigRcw(mRi[0]);
    Eigen::Vector3d eigtcw(mti);

    return true;
  }
  return false;
}

// MLPnP methods
/*===========================================================================
 * computePose：MLPnP 核心求解(一次完整位姿估计)，五大步骤：
 *   (1) 求每个观测方向的二维零空间；判断场景是否平面(秩=2)，平面时旋转到特征坐标系；
 *   (2) 随机模型：组装权重阵 P(有协方差用马氏权重，否则单位阵等权)；
 *   (3) 填设计矩阵 A(平面 9 列 / 非平面 12 列)，列对应 R 的各元素与 t；
 *   (4) 解 A^T P A 的最小奇异值右奇异向量得到代数解，再恢复正交旋转(SVD 投影)
 *       与正确尺度、并在平面 4 解/非平面 2 解中按重投影角度误差选最优；
 *   (5) 以罗德里格斯参数化做高斯-牛顿(mlpnp_gn)非线性精修，输出 result。
 * 参数：f=观测方向数组, p=世界点数组, covMats=协方差, indices=本次使用的对应下标, result=输出位姿。
 *===========================================================================*/
void MLPnPsolver::computePose(const bearingVectors_t &f, const points_t &p,
                              const cov3_mats_t &covMats,
                              const std::vector<int> &indices,
                              transformation_t &result) {
  size_t numberCorrespondences = indices.size();  // 本次参与求解的对应数量
  assert(numberCorrespondences > 5);  // MLPnP 至少需要 6 组对应

  bool planar = false;  // 是否为平面场景标志
  // compute the nullspace of all vectors
  std::vector<Eigen::MatrixXd> nullspaces(numberCorrespondences);  // 每个观测方向对应一个 3x2 零空间基
  Eigen::MatrixXd points3(3, numberCorrespondences);  // 把所有 3D 点按列拼成 3xN 矩阵(平面判断用)
  points_t points3v(numberCorrespondences);  // vector 形式的同样 3D 点(后续逐点用)
  points4_t points4v(numberCorrespondences);  // 齐次 4D 点容器(本实现未实际使用，预留)
  for (size_t i = 0; i < numberCorrespondences; i++) {
    bearingVector_t f_current = f[indices[i]];  // 第 i 个观测方向
    points3.col(i) = p[indices[i]];  // 第 i 列填入对应世界点
    // nullspace of right vector
    // 对观测方向 f^T 做 SVD，其右奇异向量的后两列张成“与 f 垂直”的二维零空间：
    // 残差将被约束在该零空间内，等价于最小化方向角度误差
    Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner> svd_f(
        f_current.transpose(), Eigen::ComputeFullV);
    nullspaces[i] = svd_f.matrixV().block(0, 1, 3, 2);  // 取 V 的第 1、2 列作为 3x2 零空间基
    points3v[i] = p[indices[i]];  // 同步保存 vector 形式点
  }

  //////////////////////////////////////
  // 1. test if we have a planar scene
  //////////////////////////////////////

  // ===== 步骤1：平面场景检测 =====
  // 点云的散布矩阵(3x3 格拉姆矩阵)，其秩反映点是否共面
  Eigen::Matrix3d planarTest = points3 * points3.transpose();
  Eigen::FullPivHouseholderQR<Eigen::Matrix3d> rankTest(planarTest);  // 全主元 QR 分解用于求秩
  Eigen::Matrix3d eigenRot;  // 特征坐标系旋转(平面时把点旋转到该系)
  eigenRot.setIdentity();  // 默认单位旋转(非平面时不改变点)

  // if yes -> transform points to new eigen frame
  // if (minEigenVal < 1e-3 || minEigenVal == 0.0)
  // rankTest.setThreshold(1e-10);
  if (rankTest.rank() == 2) {  // 秩为 2 说明所有 3D 点近似共面，走平面分支
    planar = true;  // 置平面标志
    // self adjoint is faster and more accurate than general eigen solvers
    // also has closed form solution for 3x3 self-adjoint matrices
    // in addition this solver sorts the eigenvalues in increasing order
    // 自伴随(对称)特征分解更快更稳，且特征值按升序排列
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(planarTest);
    eigenRot = eigen_solver.eigenvectors().real();  // 特征向量构成旋转矩阵
    eigenRot.transposeInPlace();  // 转置以便左乘把点变换到特征坐标系
    for (size_t i = 0; i < numberCorrespondences; i++)
      points3.col(i) = eigenRot * points3.col(i);  // 逐点旋转到特征坐标系(消去一个自由度)
  }

  //////////////////////////////////////
  // 2. stochastic model
  //////////////////////////////////////

  // ===== 步骤2：随机(权重)模型 P，2N x 2N 的块对角稀疏阵 =====
  Eigen::SparseMatrix<double> P(2 * numberCorrespondences,
                                2 * numberCorrespondences);
  bool use_cov = false;  // 是否使用逐点协方差(默认否)
  P.setIdentity();  // standard  // 默认单位阵：所有观测等权(standard)

  // if we do have covariance information
  // -> fill covariance matrix
  if (covMats.size() == numberCorrespondences) {  // 仅当协方差数量与观测数一致才启用马氏权重
    use_cov = true;  // 启用协方差加权
    int l = 0;  // 块对角阵中的行/列游标，每次步进 2
    for (size_t i = 0; i < numberCorrespondences; ++i) {  // 逐观测填两行
      // invert matrix
      cov2_mat_t temp = nullspaces[i].transpose() * covMats[i] * nullspaces[i];  // 把 3D 协方差投影到二维零空间得到 2x2 协方差
      temp = temp.inverse().eval();  // 求逆得到信息(权重)矩阵
      // 把 2x2 权重块写入稀疏阵对角位置
      P.coeffRef(l, l) = temp(0, 0);
      P.coeffRef(l, l + 1) = temp(0, 1);
      P.coeffRef(l + 1, l) = temp(1, 0);
      P.coeffRef(l + 1, l + 1) = temp(1, 1);
      l += 2;  // 游标前进到下一观测块
    }
  }

  //////////////////////////////////////
  // 3. fill the design matrix A
  //////////////////////////////////////

  // ===== 步骤3：组装设计矩阵 A =====
  // 每个观测贡献 2 行(零空间两基向量各一行)
  const int rowsA = 2 * numberCorrespondences;
  int colsA = 12;  // 非平面未知量：R 的 9 元素 + t 的 3 元素 = 12 列
  Eigen::MatrixXd A;  // 法方程矩阵 J^T W J
  if (planar) {  // 平面场景第一列尺度方向退化，只需 9 列
    colsA = 9;  // 平面：R 仅估两行(6)+t(3)=9 列
    A = Eigen::MatrixXd(rowsA, 9);  // 分配 2N x 9
  } else {  // 非平面：分配 2N x 12
    A = Eigen::MatrixXd(rowsA, 12);  // 分配 2N x 12
  }
  A.setZero();  // 先清零再填

  // fill design matrix
  // ---- 平面分支：设计矩阵只含 R 的第2、3列(r*2,r*3)与 t ----
  if (planar) {
    for (size_t i = 0; i < numberCorrespondences; ++i) {
      point_t pt3_current = points3.col(i);  // 当前世界点(可能已旋到特征系)

      // r12
      A(2 * i, 0) = nullspaces[i](0, 0) * pt3_current[1];  // 平面 r12 项：零空间第1基 × 点y
      A(2 * i + 1, 0) = nullspaces[i](0, 1) * pt3_current[1];
      // r13
      A(2 * i, 1) = nullspaces[i](0, 0) * pt3_current[2];
      A(2 * i + 1, 1) = nullspaces[i](0, 1) * pt3_current[2];
      // r22
      A(2 * i, 2) = nullspaces[i](1, 0) * pt3_current[1];
      A(2 * i + 1, 2) = nullspaces[i](1, 1) * pt3_current[1];
      // r23
      A(2 * i, 3) = nullspaces[i](1, 0) * pt3_current[2];
      A(2 * i + 1, 3) = nullspaces[i](1, 1) * pt3_current[2];
      // r32
      A(2 * i, 4) = nullspaces[i](2, 0) * pt3_current[1];
      A(2 * i + 1, 4) = nullspaces[i](2, 1) * pt3_current[1];
      // r33
      A(2 * i, 5) = nullspaces[i](2, 0) * pt3_current[2];
      A(2 * i + 1, 5) = nullspaces[i](2, 1) * pt3_current[2];
      // t1
      A(2 * i, 6) = nullspaces[i](0, 0);
      A(2 * i + 1, 6) = nullspaces[i](0, 1);
      // t2
      A(2 * i, 7) = nullspaces[i](1, 0);
      A(2 * i + 1, 7) = nullspaces[i](1, 1);
      // t3
      A(2 * i, 8) = nullspaces[i](2, 0);
      A(2 * i + 1, 8) = nullspaces[i](2, 1);
    }
  // ---- 非平面分支：R 三列(r*1,r*2,r*3)共9项 + t 3项 = 12 列 ----
  } else {
    for (size_t i = 0; i < numberCorrespondences; ++i) {
      point_t pt3_current = points3.col(i);  // 当前世界点

      // r11
      A(2 * i, 0) = nullspaces[i](0, 0) * pt3_current[0];  // 非平面 r11 项：零空间第1基 × 点x
      A(2 * i + 1, 0) = nullspaces[i](0, 1) * pt3_current[0];
      // r12
      A(2 * i, 1) = nullspaces[i](0, 0) * pt3_current[1];
      A(2 * i + 1, 1) = nullspaces[i](0, 1) * pt3_current[1];
      // r13
      A(2 * i, 2) = nullspaces[i](0, 0) * pt3_current[2];
      A(2 * i + 1, 2) = nullspaces[i](0, 1) * pt3_current[2];
      // r21
      A(2 * i, 3) = nullspaces[i](1, 0) * pt3_current[0];
      A(2 * i + 1, 3) = nullspaces[i](1, 1) * pt3_current[0];
      // r22
      A(2 * i, 4) = nullspaces[i](1, 0) * pt3_current[1];
      A(2 * i + 1, 4) = nullspaces[i](1, 1) * pt3_current[1];
      // r23
      A(2 * i, 5) = nullspaces[i](1, 0) * pt3_current[2];
      A(2 * i + 1, 5) = nullspaces[i](1, 1) * pt3_current[2];
      // r31
      A(2 * i, 6) = nullspaces[i](2, 0) * pt3_current[0];
      A(2 * i + 1, 6) = nullspaces[i](2, 1) * pt3_current[0];
      // r32
      A(2 * i, 7) = nullspaces[i](2, 0) * pt3_current[1];
      A(2 * i + 1, 7) = nullspaces[i](2, 1) * pt3_current[1];
      // r33
      A(2 * i, 8) = nullspaces[i](2, 0) * pt3_current[2];
      A(2 * i + 1, 8) = nullspaces[i](2, 1) * pt3_current[2];
      // t1
      A(2 * i, 9) = nullspaces[i](0, 0);
      A(2 * i + 1, 9) = nullspaces[i](0, 1);
      // t2
      A(2 * i, 10) = nullspaces[i](1, 0);
      A(2 * i + 1, 10) = nullspaces[i](1, 1);
      // t3
      A(2 * i, 11) = nullspaces[i](2, 0);
      A(2 * i + 1, 11) = nullspaces[i](2, 1);
    }
  }

  //////////////////////////////////////
  // 4. solve least squares
  //////////////////////////////////////

  // ===== 步骤4：求解最小二乘 =====
  Eigen::MatrixXd AtPA;
  if (use_cov)  // 加权用 J^T Kll
    AtPA = A.transpose() * P *  // 有协方差：法方程 A^T P A(作者注：完整法方程有时不稳)
           A;  // setting up the full normal equations seems to be unstable
  else
    AtPA = A.transpose() * A;  // 等权：A^T A

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(AtPA, Eigen::ComputeFullV);  // 对法方程矩阵做 SVD
  Eigen::MatrixXd result1 = svd_A.matrixV().col(colsA - 1);  // 最小奇异值对应最后一列右奇异向量=代数最小二乘解

  ////////////////////////////////
  // now we treat the results differently,
  // depending on the scene (planar or not)
  ////////////////////////////////

  // 下面按平面/非平面分别从代数解恢复合法旋转 R 与平移 t
  rotation_t Rout;
  translation_t tout;  // 输出平移(待恢复)
  if (planar) {  // ================ 平面场景分支(代数解恢复) ================
    // planar case

    rotation_t tmp;  // 临时矩阵，用于从代数解拼装 R
    // until now, we only estimated
    // row one and two of the transposed rotation matrix
    // 平面时只估出了 R 转置的前两行，第一列留空(置0)，按解向量顺序填入
    tmp << 0.0, result1(0, 0), result1(1, 0), 0.0, result1(2, 0), result1(3, 0),
        0.0, result1(4, 0), result1(5, 0);
    // row 3
    tmp.col(0) = tmp.col(1).cross(tmp.col(2));  // 第1列由第2、3列叉乘得到(保证正交)
    tmp.transposeInPlace();  // 转置还原成旋转矩阵排布

    // 由两列范数几何均值估计未知尺度
    double scale =
        1.0 / std::sqrt(std::abs(tmp.col(1).norm() * tmp.col(2).norm()));  // 尺度=1/sqrt(列2范数×列3范数)
    // find best rotation matrix in frobenius sense
    // 在 Frobenius 范数意义下找最接近的合法旋转矩阵(正交 Procrustes：U V^T)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(
        tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
    rotation_t Rout1 = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();  // R* = U V^T，保证正交
    // test if we found a good rotation matrix
    if (Rout1.determinant() < 0) Rout1 *= -1.0;  // 行列式为负(含反射)时整体取负，保证 det=+1
    // rotate this matrix back using the eigen frame
    Rout1 = eigenRot.transpose() * Rout1;  // 把旋转从特征坐标系旋回原世界坐标系

    // 用尺度恢复平移向量 t
    translation_t t =
        scale * translation_t(result1(6, 0), result1(7, 0), result1(8, 0));  // 解向量最后3列即 t，乘尺度
    Rout1.transposeInPlace();  // 转置以统一约定方向
    Rout1 *= -1;  // 方向取负(配合 opengv 的逆约定)
    if (Rout1.determinant() < 0.0) Rout1.col(2) *= -1;  // 若又出现反射，翻转第3列保证 det=+1
    // now we have to find the best out of 4 combinations
    // 平面存在 4 种(R 的符号 × t 的符号)组合，需逐一比较选最优
    rotation_t R1, R2;
    R1.col(0) = Rout1.col(0);  // R1：三列保持
    R1.col(1) = Rout1.col(1);
    R1.col(2) = Rout1.col(2);
    R2.col(0) = -Rout1.col(0);  // R2：前两列取负、第三列保持(行列式仍为+1)
    R2.col(1) = -Rout1.col(1);
    R2.col(2) = Rout1.col(2);

    vector<transformation_t, Eigen::aligned_allocator<transformation_t>> Ts(4);  // 4 个候选 4x4 位姿(Eigen 对齐分配)
    Ts[0].block<3, 3>(0, 0) = R1;  // 候选0：R1 + t
    Ts[0].block<3, 1>(0, 3) = t;
    Ts[1].block<3, 3>(0, 0) = R1;  // 候选1：R1 - t
    Ts[1].block<3, 1>(0, 3) = -t;
    Ts[2].block<3, 3>(0, 0) = R2;  // 候选2：R2 + t
    Ts[2].block<3, 1>(0, 3) = t;
    Ts[3].block<3, 3>(0, 0) = R2;  // 候选3：R2 - t
    Ts[3].block<3, 1>(0, 3) = -t;

    vector<double> normVal(4);  // 每个候选的角度误差累计
    for (int i = 0; i < 4; ++i) {  // 遍历 4 个候选
      point_t reproPt;  // 重投影(变换)后的点
      double norms = 0.0;  // 该候选误差累计器
      for (int p = 0; p < 6; ++p) {  // 只用前 6 组对应快速判方向(省时)
        reproPt =  // 把世界点变换到相机系
            Ts[i].block<3, 3>(0, 0) * points3v[p] + Ts[i].block<3, 1>(0, 3);  // R*P + t
        reproPt = reproPt / reproPt.norm();  // 单位化，以便与观测方向比较余弦
        norms += (1.0 - reproPt.transpose() * f[indices[p]]);  // 角度误差=1-方向点积(越小越一致)
      }
      normVal[i] = norms;  // 记录该候选总误差
    }
    // 选误差最小的候选
    std::vector<double>::iterator findMinRepro =
        std::min_element(std::begin(normVal), std::end(normVal));
    int idx = std::distance(std::begin(normVal), findMinRepro);  // 最优候选下标
    Rout = Ts[idx].block<3, 3>(0, 0);  // 确定平面最优 R
    tout = Ts[idx].block<3, 1>(0, 3);  // 确定平面最优 t
  } else {  // ================ 非平面场景分支(代数解恢复) ================
    // non-planar

    rotation_t tmp;  // 临时矩阵装代数解的 9 个 R 元素
    // 解向量按列优先顺序重排为 3x3 矩阵(列1:索引0,1,2；列2:3,4,5；列3:6,7,8)
    tmp << result1(0, 0), result1(3, 0), result1(6, 0), result1(1, 0),
        result1(4, 0), result1(7, 0), result1(2, 0), result1(5, 0),
        result1(8, 0);
    // get the scale
    // 非平面尺度由三列范数的几何均值(立方根倒数)确定
    double scale =
        1.0 / std::pow(std::abs(tmp.col(0).norm() * tmp.col(1).norm() *  // 尺度=1/(三列范数乘积)^(1/3)
                                tmp.col(2).norm()),
                       1.0 / 3.0);
    // double scale = 1.0 / std::sqrt(std::abs(tmp.col(0).norm() *
    // tmp.col(1).norm()));
    //  find best rotation matrix in frobenius sense
    // 同样用 U V^T 求最近正交旋转(Frobenius 投影)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(
        tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Rout = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();  // 最近合法旋转
    // test if we found a good rotation matrix
    if (Rout.determinant() < 0) Rout *= -1.0;  // 反射则取负
    // scale translation
    // 用尺度恢复 t，并左乘 R 旋到正确方向
    tout = Rout * (scale * translation_t(result1(9, 0), result1(10, 0),
                                         result1(11, 0)));

    // find correct direction in terms of reprojection error, just take the
    // first 6 correspondences
    // 非平面只剩 t 的正负 2 种方向，用前 6 点判优
    vector<double> error(2);
    vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Ts(2);  // 2 个候选位姿
    for (int s = 0; s < 2; ++s) {  // 候选0用 +t，候选1用 -t
      error[s] = 0.0;  // 误差清零
      Ts[s] = Eigen::Matrix4d::Identity();  // 候选置单位阵
      Ts[s].block<3, 3>(0, 0) = Rout;  // 填 R
      if (s == 0)  // s=0 用 +t
        Ts[s].block<3, 1>(0, 3) = tout;  // +t
      else
        Ts[s].block<3, 1>(0, 3) = -tout;  // -t
      Ts[s] = Ts[s].inverse().eval();  // 求逆：统一到与观测方向相同的比较约定
      for (int p = 0; p < 6; ++p) {
        bearingVector_t v =  // 世界点变换到相机系
            Ts[s].block<3,3>(0, 0) * points3v[p] + Ts[s].block<3, 1>(0, 3);  // R*P+t
        v = v / v.norm();  // 单位化
        error[s] += (1.0 - v.transpose() * f[indices[p]]);  // 累计角度误差
      }
    }
    if (error[0] < error[1])  // 比较两方向，选误差小者的 t
      tout = Ts[0].block<3, 1>(0, 3);  // +t 更优
    else
      tout = Ts[1].block<3, 1>(0, 3);  // -t 更优
    Rout = Ts[0].block<3, 3>(0, 0);  // 两候选 R 相同，取出即可
  }

  //////////////////////////////////////
  // 5. gauss newton
  //////////////////////////////////////

  // ===== 步骤5：高斯-牛顿非线性精修 =====
  // 先把旋转矩阵转成罗德里格斯轴角向量(3维)，与 t(3维)拼成 6 维状态
  rodrigues_t omega = rot2rodrigues(Rout);
  Eigen::VectorXd minx(6);  // 6 维状态 [wx,wy,wz,tx,ty,tz]
  minx[0] = omega[0];
  minx[1] = omega[1];
  minx[2] = omega[2];
  minx[3] = tout[0];
  minx[4] = tout[1];
  minx[5] = tout[2];

  mlpnp_gn(minx, points3v, nullspaces, P, use_cov);  // 对该状态做高斯-牛顿迭代，原地更新 minx

  Rout = rodrigues2rot(rodrigues_t(minx[0], minx[1], minx[2]));  // 精修后的轴角转回旋转矩阵
  tout = translation_t(minx[3], minx[4], minx[5]);  // 精修后的平移

  // result inverse as opengv uses this convention
  // 写入最终结果(注意：opengv 约定下这里是其逆变换方向)
  result.block<3, 3>(0, 0) = Rout;
  result.block<3, 1>(0, 3) = tout;  // 写平移
}

/*===========================================================================
 * rodrigues2rot：罗德里格斯公式，把轴角向量 omega 转成旋转矩阵 R。
 * R = I + sin(θ)/θ·[ω]× + (1-cosθ)/θ²·[ω]×²，θ=|ω|；θ≈0 时 R≈I。
 *===========================================================================*/
Eigen::Matrix3d MLPnPsolver::rodrigues2rot(const Eigen::Vector3d &omega) {
  rotation_t R = Eigen::Matrix3d::Identity();  // R 初始化为单位阵 I

  Eigen::Matrix3d skewW;  // omega 的反对称(叉乘)矩阵 [ω]×
  skewW << 0.0, -omega(2), omega(1), omega(2), 0.0, -omega(0), -omega(1),  // 按叉乘矩阵定义填充
      omega(0), 0.0;

  double omega_norm = omega.norm();  // 旋转角 θ = 轴角向量模长

  if (omega_norm > std::numeric_limits<double>::epsilon())  // 角度非零才用级数公式，否则保持 I 避免除零
    R = R + sin(omega_norm) / omega_norm * skewW +  // 罗德里格斯第二项 sinθ/θ·[ω]×
        (1 - cos(omega_norm)) / (omega_norm * omega_norm) * (skewW * skewW);  // 第三项 (1-cosθ)/θ²·[ω]×²

  return R;  // 返回旋转矩阵
}

/*===========================================================================
 * rot2rodrigues：罗德里格斯逆变换，由旋转矩阵 R 求轴角向量 omega。
 * θ=arccos((tr(R)-1)/2)，omega = θ/(2sinθ)·[R32-R23, R13-R31, R21-R12]^T。
 *===========================================================================*/
Eigen::Vector3d MLPnPsolver::rot2rodrigues(const Eigen::Matrix3d &R) {
  rodrigues_t omega;  // 输出轴角向量
  omega << 0.0, 0.0, 0.0;  // 默认零向量(对应 R=I)

  double trace = R.trace() - 1.0;  // tr(R)-1，用于反算角度
  double wnorm = acos(trace / 2.0);  // 旋转角 θ=arccos((trR-1)/2)
  if (wnorm > std::numeric_limits<double>::epsilon()) {  // 角度非零才恢复轴向，否则保持零
    omega[0] = (R(2, 1) - R(1, 2));  // 轴向量 x 分量(反对称部分)
    omega[1] = (R(0, 2) - R(2, 0));  // y 分量
    omega[2] = (R(1, 0) - R(0, 1));  // z 分量
    double sc = wnorm / (2.0 * sin(wnorm));  // 缩放系数 θ/(2sinθ)
    omega *= sc;  // 缩放得到最终轴角
  }
  return omega;  // 返回轴角
}

/*===========================================================================
 * mlpnp_gn：固定最多 5 次的高斯-牛顿迭代，最小化“零空间残差”。
 * 每次：由当前状态算残差 r 与雅可比 J -> 法方程 (J^T W J) dx = J^T W r ->
 *       LDLT 解增量 dx -> x = x - dx；含发散保护与收敛判据。
 * 参数：x=6维状态(进出)、pts=世界点、nullspaces=零空间、Kll=权重阵、use_cov=是否加权。
 *===========================================================================*/
void MLPnPsolver::mlpnp_gn(Eigen::VectorXd &x, const points_t &pts,
                           const std::vector<Eigen::MatrixXd> &nullspaces,
                           const Eigen::SparseMatrix<double> Kll,
                           bool use_cov) {
  const int numObservations = pts.size();  // 观测数
  const int numUnknowns = 6;  // 待估参数：3 轴角 + 3 平移
  // check redundancy
  assert((2 * numObservations - numUnknowns) > 0);  // 检查冗余度：方程数必须多于未知量

  // =============
  // set all matrices up
  // =============

  Eigen::VectorXd r(2 * numObservations);  // 残差向量(每观测 2 维)
  Eigen::VectorXd rd(2 * numObservations);  // 残差备份(本实现未实际使用)
  Eigen::MatrixXd Jac(2 * numObservations, numUnknowns);  // 雅可比矩阵 2N x 6
  Eigen::VectorXd g(numUnknowns, 1);  // 法方程右端 J^T W r
  Eigen::VectorXd dx(numUnknowns, 1);  // result vector  // 每次迭代的状态增量

  Jac.setZero();
  r.setZero();
  dx.setZero();
  g.setZero();

  int it_cnt = 0;  // 迭代计数
  bool stop = false;  // 收敛标志
  const int maxIt = 5;  // 最多迭代 5 次(经验值，PnP 初值好时收敛很快)
  double epsP = 1e-5;  // 收敛阈值：增量最大值小于它即停

  Eigen::MatrixXd JacTSKll;  // J^T W(加权时)或 J^T
  Eigen::MatrixXd A;
  // solve simple gradient descent
  while (it_cnt < maxIt && !stop) {  // 未达上限且未收敛就继续
    mlpnp_residuals_and_jacs(x, pts, nullspaces, r, Jac, true);  // 计算当前状态下的残差 r 与雅可比 Jac

    if (use_cov)
      JacTSKll = Jac.transpose() * Kll;  // 加权左端转置
    else
      JacTSKll = Jac.transpose();  // 等权则仅转置

    A = JacTSKll * Jac;  // 法方程矩阵 (J^T W J)

    // get system matrix
    g = JacTSKll * r;  // 法方程右端 (J^T W r)

    // solve
    Eigen::LDLT<Eigen::MatrixXd> chol(A);  // LDLT 分解(对称正定，比完整 SVD 快)
    dx = chol.solve(g);  // 解出增量 dx
    // this is to prevent the solution from falling into a wrong minimum
    // if the linear estimate is spurious
    // 发散保护：增量异常大说明线性化不可信，直接终止以免陷入错误极小值
    if (dx.array().abs().maxCoeff() > 5.0 || dx.array().abs().minCoeff() > 1.0)
      break;  // 跳出循环
    // observation update
    Eigen::MatrixXd dl = Jac * dx;  // 观测空间增量 J*dx，用于判断收敛
    if (dl.array().abs().maxCoeff() < epsP) {  // 最大观测增量已小于阈值 -> 收敛
      stop = true;  // 置收敛标志
      x = x - dx;  // 更新状态(高斯-牛顿为减)
      break;
    } else {
      x = x - dx;  // 未收敛：正常更新状态继续迭代
    }

    ++it_cnt;  // 迭代计数 +1
  }  // while
  // result
}

/*===========================================================================
 * mlpnp_residuals_and_jacs：逐点计算残差，并在需要时调用 mlpnpJacs 填雅可比。
 * 残差定义：把世界点用当前 (R,t) 变到相机系并单位化，再投影到观测方向的零空间，
 *           得到 2 维残差(理想情况下变换方向与观测方向一致，残差为 0)。
 *===========================================================================*/
void MLPnPsolver::mlpnp_residuals_and_jacs(
    const Eigen::VectorXd &x, const points_t &pts,
    const std::vector<Eigen::MatrixXd> &nullspaces, Eigen::VectorXd &r,
    Eigen::MatrixXd &fjac, bool getJacs) {
  rodrigues_t w(x[0], x[1], x[2]);  // 从状态向量取轴角 w
  translation_t T(x[3], x[4], x[5]);  // 取平移 t

  rotation_t R = rodrigues2rot(w);  // 当前轴角 -> 当前旋转矩阵
  int ii = 0;  // 残差/雅可比行游标，每点步进 2

  Eigen::MatrixXd jacs(2, 6);  // 单点的 2x6 雅可比

  for (int i = 0; i < pts.size(); ++i) {  // 遍历所有观测点
    Eigen::Vector3d ptCam = R * pts[i] + T;  // 世界点 -> 相机系点
    ptCam /= ptCam.norm();  // 单位化(MLPnP 在单位球面方向上建模)

    r[ii] = nullspaces[i].col(0).transpose() * ptCam;  // 残差第1维：零空间第1基与相机方向点积
    r[ii + 1] = nullspaces[i].col(1).transpose() * ptCam;  // 残差第2维：零空间第2基点积
    if (getJacs) {  // 需要雅可比时才计算(省时)
      // jacs
      mlpnpJacs(pts[i], nullspaces[i].col(0), nullspaces[i].col(1), w, T, jacs);  // 解析求该点 2x6 雅可比

      // r
      // 第1行(对应零空间第1基 r)对 6 个状态量的偏导
      fjac(ii, 0) = jacs(0, 0);
      fjac(ii, 1) = jacs(0, 1);
      fjac(ii, 2) = jacs(0, 2);

      fjac(ii, 3) = jacs(0, 3);
      fjac(ii, 4) = jacs(0, 4);
      fjac(ii, 5) = jacs(0, 5);
      // s
      // 第2行(对应零空间第2基 s)对 6 个状态量的偏导
      fjac(ii + 1, 0) = jacs(1, 0);
      fjac(ii + 1, 1) = jacs(1, 1);
      fjac(ii + 1, 2) = jacs(1, 2);

      fjac(ii + 1, 3) = jacs(1, 3);
      fjac(ii + 1, 4) = jacs(1, 4);
      fjac(ii + 1, 5) = jacs(1, 5);
    }
    ii += 2;  // 行游标前进 2
  }
}

/*===========================================================================
 * mlpnpJacs：单个观测点的解析雅可比(2 行 x 6 列)，由符号计算软件(CAS)离线求导、
 * 再把展开式直接生成为 C++ 代码，因此出现大量 tXX 中间变量与超长表达式。
 * 阅读方法(不必逐行展开代数)：
 *   - r1..r3/s1..s3：该点二维零空间的两个基向量(对应两行残差)；
 *   - X1,Y1,Z1：世界点坐标；w1..w3 轴角；t1..t3 平移；
 *   - t5..t14：|w|²、sin/cos 及其倒数等公共三角量；
 *   - t15,t18,t23：当前 (R,t) 下的相机系点坐标(x,y,z)；t60..t65 为其模长相关量；
 *   - t66..t191/t193..t204：旋转部分对轴角偏导的各项代数展开；
 *   - t205..t216：平移部分(对 t 求导)的公共项；
 *   - 最终 jacs(0,0..5)、jacs(1,0..5) 即两行六列解析导数。
 * 这些 tXX 只做代数缓存，不改变任何数学含义。
 *===========================================================================*/
void MLPnPsolver::mlpnpJacs(const point_t &pt,
                            const Eigen::Vector3d &nullspace_r,
                            const Eigen::Vector3d &nullspace_s,
                            const rodrigues_t &w, const translation_t &t,
                            Eigen::MatrixXd &jacs) {
  // —— 输入：零空间第 1 基 r(对应第 1 行残差) ——
  double r1 = nullspace_r[0];
  double r2 = nullspace_r[1];
  double r3 = nullspace_r[2];

  // —— 输入：零空间第 2 基 s(对应第 2 行残差) ——
  double s1 = nullspace_s[0];
  double s2 = nullspace_s[1];
  double s3 = nullspace_s[2];

  // —— 输入：世界点坐标 X,Y,Z ——
  double X1 = pt[0];
  double Y1 = pt[1];
  double Z1 = pt[2];

  // —— 输入：轴角 w=(w1,w2,w3) ——
  double w1 = w[0];
  double w2 = w[1];
  double w3 = w[2];

  // —— 输入：平移 t=(t1,t2,t3) ——
  double t1 = t[0];
  double t2 = t[1];
  double t3 = t[2];

  // —— 公共三角量：t5..t14。t8=|w|^2, t9=|w|, t10=sin|w|, t12=cos|w|, t11/t14/t25/t26 为其倒数/幂 ——
  double t5 = w1 * w1;
  double t6 = w2 * w2;
  double t7 = w3 * w3;
  double t8 = t5 + t6 + t7;
  double t9 = sqrt(t8);
  double t10 = sin(t9);
  double t11 = 1.0 / sqrt(t8);
  double t12 = cos(t9);
  double t13 = t12 - 1.0;
  double t14 = 1.0 / t8;
  double t16 = t10 * t11 * w3;
  double t17 = t13 * t14 * w1 * w2;
  double t19 = t10 * t11 * w2;
  double t20 = t13 * t14 * w1 * w3;
  double t24 = t6 + t7;
  double t27 = t16 + t17;
  double t28 = Y1 * t27;
  double t29 = t19 - t20;
  double t30 = Z1 * t29;
  double t31 = t13 * t14 * t24;
  double t32 = t31 + 1.0;
  double t33 = X1 * t32;
  // —— 当前位姿下的相机系点坐标：t15=xc, t18=yc, t23=zc(即 R*P+t 的三个分量展开) ——
  double t15 = t1 - t28 + t30 + t33;
  double t21 = t10 * t11 * w1;
  double t22 = t13 * t14 * w2 * w3;
  double t45 = t5 + t7;
  double t53 = t16 - t17;
  double t54 = X1 * t53;
  double t55 = t21 + t22;
  double t56 = Z1 * t55;
  double t57 = t13 * t14 * t45;
  double t58 = t57 + 1.0;
  double t59 = Y1 * t58;
  double t18 = t2 + t54 - t56 + t59;  // 相机系 y 坐标 yc
  double t34 = t5 + t6;
  double t38 = t19 + t20;
  double t39 = X1 * t38;
  double t40 = t21 - t22;
  double t41 = Y1 * t40;
  double t42 = t13 * t14 * t34;
  double t43 = t42 + 1.0;
  double t44 = Z1 * t43;
  double t23 = t3 - t39 + t41 + t44;  // 相机系 z 坐标 zc
  // —— 三角函数求导所需的幂次/倒数项(t25、t26) 与旋转矩阵各元素的代数展开(t35..t59) ——
  double t25 = 1.0 / pow(t8, 3.0 / 2.0);
  double t26 = 1.0 / (t8 * t8);
  double t35 = t12 * t14 * w1 * w2;
  double t36 = t5 * t10 * t25 * w3;
  double t37 = t5 * t13 * t26 * w3 * 2.0;
  double t46 = t10 * t25 * w1 * w3;
  double t47 = t5 * t10 * t25 * w2;
  double t48 = t5 * t13 * t26 * w2 * 2.0;
  double t49 = t10 * t11;
  double t50 = t5 * t12 * t14;
  double t51 = t13 * t26 * w1 * w2 * w3 * 2.0;
  double t52 = t10 * t25 * w1 * w2 * w3;
  // —— t60..t65：相机系点模长平方 t63 与其倒数 t65(单位化时链式求导要用) ——
  double t60 = t15 * t15;
  double t61 = t18 * t18;
  double t62 = t23 * t23;
  double t63 = t60 + t61 + t62;  // |Pc|^2 = xc^2+yc^2+zc^2
  double t64 = t5 * t10 * t25;
  double t65 = 1.0 / sqrt(t63);  // 1/|Pc|，单位化因子
  // —— t66..t110 与 t93：第 1 行残差(基 r)对“旋转+点坐标”的长多项式累加项 ——
  double t66 = Y1 * r2 * t6;
  double t67 = Z1 * r3 * t7;
  double t68 = r1 * t1 * t5;
  double t69 = r1 * t1 * t6;
  double t70 = r1 * t1 * t7;
  double t71 = r2 * t2 * t5;
  double t72 = r2 * t2 * t6;
  double t73 = r2 * t2 * t7;
  double t74 = r3 * t3 * t5;
  double t75 = r3 * t3 * t6;
  double t76 = r3 * t3 * t7;
  double t77 = X1 * r1 * t5;
  double t78 = X1 * r2 * w1 * w2;
  double t79 = X1 * r3 * w1 * w3;
  double t80 = Y1 * r1 * w1 * w2;
  double t81 = Y1 * r3 * w2 * w3;
  double t82 = Z1 * r1 * w1 * w3;
  double t83 = Z1 * r2 * w2 * w3;
  double t84 = X1 * r1 * t6 * t12;
  double t85 = X1 * r1 * t7 * t12;
  double t86 = Y1 * r2 * t5 * t12;
  double t87 = Y1 * r2 * t7 * t12;
  double t88 = Z1 * r3 * t5 * t12;
  double t89 = Z1 * r3 * t6 * t12;
  double t90 = X1 * r2 * t9 * t10 * w3;
  double t91 = Y1 * r3 * t9 * t10 * w1;
  double t92 = Z1 * r1 * t9 * t10 * w2;
  double t102 = X1 * r3 * t9 * t10 * w2;
  double t103 = Y1 * r1 * t9 * t10 * w3;
  double t104 = Z1 * r2 * t9 * t10 * w1;
  double t105 = X1 * r2 * t12 * w1 * w2;
  double t106 = X1 * r3 * t12 * w1 * w3;
  double t107 = Y1 * r1 * t12 * w1 * w2;
  double t108 = Y1 * r3 * t12 * w2 * w3;
  double t109 = Z1 * r1 * t12 * w1 * w3;
  double t110 = Z1 * r2 * t12 * w2 * w3;
  double t93 = t66 + t67 + t68 + t69 + t70 + t71 + t72 + t73 + t74 + t75 + t76 +  // 第1行残差相关的大累加和(代数缓存)
               t77 + t78 + t79 + t80 + t81 + t82 + t83 + t84 + t85 + t86 + t87 +
               t88 + t89 + t90 + t91 + t92 - t102 - t103 - t104 - t105 - t106 -
               t107 - t108 - t109 - t110;
  // —— t94..t137：旋转矩阵对 w1 的偏导展开，用于 jacs(*,0) ——
  double t94 = t10 * t25 * w1 * w2;
  double t95 = t6 * t10 * t25 * w3;
  double t96 = t6 * t13 * t26 * w3 * 2.0;
  double t97 = t12 * t14 * w2 * w3;
  double t98 = t6 * t10 * t25 * w1;
  double t99 = t6 * t13 * t26 * w1 * 2.0;
  double t100 = t6 * t10 * t25;
  double t101 = 1.0 / pow(t63, 3.0 / 2.0);
  double t111 = t6 * t12 * t14;
  double t112 = t10 * t25 * w2 * w3;
  double t113 = t12 * t14 * w1 * w3;
  double t114 = t7 * t10 * t25 * w2;
  double t115 = t7 * t13 * t26 * w2 * 2.0;
  double t116 = t7 * t10 * t25 * w1;
  double t117 = t7 * t13 * t26 * w1 * 2.0;
  double t118 = t7 * t12 * t14;
  double t119 = t13 * t24 * t26 * w1 * 2.0;
  double t120 = t10 * t24 * t25 * w1;
  double t121 = t119 + t120;
  double t122 = t13 * t26 * t34 * w1 * 2.0;
  double t123 = t10 * t25 * t34 * w1;
  double t131 = t13 * t14 * w1 * 2.0;
  double t124 = t122 + t123 - t131;
  double t139 = t13 * t14 * w3;
  double t125 = -t35 + t36 + t37 + t94 - t139;
  double t126 = X1 * t125;
  double t127 = t49 + t50 + t51 + t52 - t64;
  double t128 = Y1 * t127;
  double t129 = t126 + t128 - Z1 * t124;
  double t130 = t23 * t129 * 2.0;
  double t132 = t13 * t26 * t45 * w1 * 2.0;
  double t133 = t10 * t25 * t45 * w1;
  double t138 = t13 * t14 * w2;
  double t134 = -t46 + t47 + t48 + t113 - t138;
  double t135 = X1 * t134;
  double t136 = -t49 - t50 + t51 + t52 + t64;
  double t137 = Z1 * t136;
  // —— t140..t191 与 t167：第 2 行残差(基 s)对应的同构累加项(与上面对称) ——
  double t140 = X1 * s1 * t5;
  double t141 = Y1 * s2 * t6;
  double t142 = Z1 * s3 * t7;
  double t143 = s1 * t1 * t5;
  double t144 = s1 * t1 * t6;
  double t145 = s1 * t1 * t7;
  double t146 = s2 * t2 * t5;
  double t147 = s2 * t2 * t6;
  double t148 = s2 * t2 * t7;
  double t149 = s3 * t3 * t5;
  double t150 = s3 * t3 * t6;
  double t151 = s3 * t3 * t7;
  double t152 = X1 * s2 * w1 * w2;
  double t153 = X1 * s3 * w1 * w3;
  double t154 = Y1 * s1 * w1 * w2;
  double t155 = Y1 * s3 * w2 * w3;
  double t156 = Z1 * s1 * w1 * w3;
  double t157 = Z1 * s2 * w2 * w3;
  double t158 = X1 * s1 * t6 * t12;
  double t159 = X1 * s1 * t7 * t12;
  double t160 = Y1 * s2 * t5 * t12;
  double t161 = Y1 * s2 * t7 * t12;
  double t162 = Z1 * s3 * t5 * t12;
  double t163 = Z1 * s3 * t6 * t12;
  double t164 = X1 * s2 * t9 * t10 * w3;
  double t165 = Y1 * s3 * t9 * t10 * w1;
  double t166 = Z1 * s1 * t9 * t10 * w2;
  double t183 = X1 * s3 * t9 * t10 * w2;
  double t184 = Y1 * s1 * t9 * t10 * w3;
  double t185 = Z1 * s2 * t9 * t10 * w1;
  double t186 = X1 * s2 * t12 * w1 * w2;
  double t187 = X1 * s3 * t12 * w1 * w3;
  double t188 = Y1 * s1 * t12 * w1 * w2;
  double t189 = Y1 * s3 * t12 * w2 * w3;
  double t190 = Z1 * s1 * t12 * w1 * w3;
  double t191 = Z1 * s2 * t12 * w2 * w3;
  double t167 = t140 + t141 + t142 + t143 + t144 + t145 + t146 + t147 + t148 +  // 第2行残差相关的大累加和(代数缓存)
                t149 + t150 + t151 + t152 + t153 + t154 + t155 + t156 + t157 +
                t158 + t159 + t160 + t161 + t162 + t163 + t164 + t165 + t166 -
                t183 - t184 - t185 - t186 - t187 - t188 - t189 - t190 - t191;
  // —— t168..t204：旋转矩阵对 w2、w3 的偏导展开，用于 jacs(*,1)、jacs(*,2) ——
  double t168 = t13 * t26 * t45 * w2 * 2.0;
  double t169 = t10 * t25 * t45 * w2;
  double t170 = t168 + t169;
  double t171 = t13 * t26 * t34 * w2 * 2.0;
  double t172 = t10 * t25 * t34 * w2;
  double t176 = t13 * t14 * w2 * 2.0;
  double t173 = t171 + t172 - t176;
  double t174 = -t49 + t51 + t52 + t100 - t111;
  double t175 = X1 * t174;
  double t177 = t13 * t24 * t26 * w2 * 2.0;
  double t178 = t10 * t24 * t25 * w2;
  double t192 = t13 * t14 * w1;
  double t179 = -t97 + t98 + t99 + t112 - t192;
  double t180 = Y1 * t179;
  double t181 = t49 + t51 + t52 - t100 + t111;
  double t182 = Z1 * t181;
  double t193 = t13 * t26 * t34 * w3 * 2.0;
  double t194 = t10 * t25 * t34 * w3;
  double t195 = t193 + t194;
  double t196 = t13 * t26 * t45 * w3 * 2.0;
  double t197 = t10 * t25 * t45 * w3;
  double t200 = t13 * t14 * w3 * 2.0;
  double t198 = t196 + t197 - t200;
  double t199 = t7 * t10 * t25;
  double t201 = t13 * t24 * t26 * w3 * 2.0;
  double t202 = t10 * t24 * t25 * w3;
  double t203 = -t49 + t51 + t52 - t118 + t199;
  double t204 = Y1 * t203;
  // —— t205..t216：对平移 t 求导的公共项(单位化链式)，用于 jacs(*,3..5) ——
  double t205 = t1 * 2.0;
  double t206 = Z1 * t29 * 2.0;
  double t207 = X1 * t32 * 2.0;
  double t208 = t205 + t206 + t207 - Y1 * t27 * 2.0;  // 第1个平移方向公共项(对应 tx)
  double t209 = t2 * 2.0;
  double t210 = X1 * t53 * 2.0;
  double t211 = Y1 * t58 * 2.0;
  double t212 = t209 + t210 + t211 - Z1 * t55 * 2.0;  // 第2个平移方向公共项(对应 ty)
  double t213 = t3 * 2.0;
  double t214 = Y1 * t40 * 2.0;
  double t215 = Z1 * t43 * 2.0;
  double t216 = t213 + t214 + t215 - X1 * t38 * 2.0;  // 第3个平移方向公共项(对应 tz)
  // =========================================================================
  // 最终输出 2x6 雅可比：
  //   行0=零空间基 r 对应的残差；行1=基 s 对应的残差；
  //   列0,1,2=对轴角(wx,wy,wz)的偏导；列3,4,5=对平移(tx,ty,tz)的偏导。
  // =========================================================================
  // 行0列0：残差r 对 wx 的偏导(符号求导展开式，勿手改)
  jacs(0, 0) =
      t14 * t65 *
          (X1 * r1 * w1 * 2.0 + X1 * r2 * w2 + X1 * r3 * w3 + Y1 * r1 * w2 +
           Z1 * r1 * w3 + r1 * t1 * w1 * 2.0 + r2 * t2 * w1 * 2.0 +
           r3 * t3 * w1 * 2.0 + Y1 * r3 * t5 * t12 + Y1 * r3 * t9 * t10 -
           Z1 * r2 * t5 * t12 - Z1 * r2 * t9 * t10 - X1 * r2 * t12 * w2 -
           X1 * r3 * t12 * w3 - Y1 * r1 * t12 * w2 + Y1 * r2 * t12 * w1 * 2.0 -
           Z1 * r1 * t12 * w3 + Z1 * r3 * t12 * w1 * 2.0 +
           Y1 * r3 * t5 * t10 * t11 - Z1 * r2 * t5 * t10 * t11 +
           X1 * r2 * t12 * w1 * w3 - X1 * r3 * t12 * w1 * w2 -
           Y1 * r1 * t12 * w1 * w3 + Z1 * r1 * t12 * w1 * w2 -
           Y1 * r1 * t10 * t11 * w1 * w3 + Z1 * r1 * t10 * t11 * w1 * w2 -
           X1 * r1 * t6 * t10 * t11 * w1 - X1 * r1 * t7 * t10 * t11 * w1 +
           X1 * r2 * t5 * t10 * t11 * w2 + X1 * r3 * t5 * t10 * t11 * w3 +
           Y1 * r1 * t5 * t10 * t11 * w2 - Y1 * r2 * t5 * t10 * t11 * w1 -
           Y1 * r2 * t7 * t10 * t11 * w1 + Z1 * r1 * t5 * t10 * t11 * w3 -
           Z1 * r3 * t5 * t10 * t11 * w1 - Z1 * r3 * t6 * t10 * t11 * w1 +
           X1 * r2 * t10 * t11 * w1 * w3 - X1 * r3 * t10 * t11 * w1 * w2 +
           Y1 * r3 * t10 * t11 * w1 * w2 * w3 +
           Z1 * r2 * t10 * t11 * w1 * w2 * w3) -
      t26 * t65 * t93 * w1 * 2.0 -
      t14 * t93 * t101 *
          (t130 +
           t15 *
               (-X1 * t121 +
                Y1 * (t46 + t47 + t48 - t13 * t14 * w2 - t12 * t14 * w1 * w3) +
                Z1 * (t35 + t36 + t37 - t13 * t14 * w3 - t10 * t25 * w1 * w2)) *
               2.0 +
           t18 * (t135 + t137 - Y1 * (t132 + t133 - t13 * t14 * w1 * 2.0)) *
               2.0) *
          (1.0 / 2.0);
  jacs(0, 1) =  // 行0列1：残差r 对 wy 的偏导
      t14 * t65 *
          (X1 * r2 * w1 + Y1 * r1 * w1 + Y1 * r2 * w2 * 2.0 + Y1 * r3 * w3 +
           Z1 * r2 * w3 + r1 * t1 * w2 * 2.0 + r2 * t2 * w2 * 2.0 +
           r3 * t3 * w2 * 2.0 - X1 * r3 * t6 * t12 - X1 * r3 * t9 * t10 +
           Z1 * r1 * t6 * t12 + Z1 * r1 * t9 * t10 + X1 * r1 * t12 * w2 * 2.0 -
           X1 * r2 * t12 * w1 - Y1 * r1 * t12 * w1 - Y1 * r3 * t12 * w3 -
           Z1 * r2 * t12 * w3 + Z1 * r3 * t12 * w2 * 2.0 -
           X1 * r3 * t6 * t10 * t11 + Z1 * r1 * t6 * t10 * t11 +
           X1 * r2 * t12 * w2 * w3 - Y1 * r1 * t12 * w2 * w3 +
           Y1 * r3 * t12 * w1 * w2 - Z1 * r2 * t12 * w1 * w2 -
           Y1 * r1 * t10 * t11 * w2 * w3 + Y1 * r3 * t10 * t11 * w1 * w2 -
           Z1 * r2 * t10 * t11 * w1 * w2 - X1 * r1 * t6 * t10 * t11 * w2 +
           X1 * r2 * t6 * t10 * t11 * w1 - X1 * r1 * t7 * t10 * t11 * w2 +
           Y1 * r1 * t6 * t10 * t11 * w1 - Y1 * r2 * t5 * t10 * t11 * w2 -
           Y1 * r2 * t7 * t10 * t11 * w2 + Y1 * r3 * t6 * t10 * t11 * w3 -
           Z1 * r3 * t5 * t10 * t11 * w2 + Z1 * r2 * t6 * t10 * t11 * w3 -
           Z1 * r3 * t6 * t10 * t11 * w2 + X1 * r2 * t10 * t11 * w2 * w3 +
           X1 * r3 * t10 * t11 * w1 * w2 * w3 +
           Z1 * r1 * t10 * t11 * w1 * w2 * w3) -
      t26 * t65 * t93 * w2 * 2.0 -
      t14 * t93 * t101 *
          (t18 *
               (Z1 * (-t35 + t94 + t95 + t96 - t13 * t14 * w3) - Y1 * t170 +
                X1 * (t97 + t98 + t99 - t13 * t14 * w1 - t10 * t25 * w2 * w3)) *
               2.0 +
           t15 * (t180 + t182 - X1 * (t177 + t178 - t13 * t14 * w2 * 2.0)) *
               2.0 +
           t23 *
               (t175 + Y1 * (t35 - t94 + t95 + t96 - t13 * t14 * w3) -
                Z1 * t173) *
               2.0) *
          (1.0 / 2.0);
  jacs(0, 2) =  // 行0列2：残差r 对 wz 的偏导
      t14 * t65 *
          (X1 * r3 * w1 + Y1 * r3 * w2 + Z1 * r1 * w1 + Z1 * r2 * w2 +
           Z1 * r3 * w3 * 2.0 + r1 * t1 * w3 * 2.0 + r2 * t2 * w3 * 2.0 +
           r3 * t3 * w3 * 2.0 + X1 * r2 * t7 * t12 + X1 * r2 * t9 * t10 -
           Y1 * r1 * t7 * t12 - Y1 * r1 * t9 * t10 + X1 * r1 * t12 * w3 * 2.0 -
           X1 * r3 * t12 * w1 + Y1 * r2 * t12 * w3 * 2.0 - Y1 * r3 * t12 * w2 -
           Z1 * r1 * t12 * w1 - Z1 * r2 * t12 * w2 + X1 * r2 * t7 * t10 * t11 -
           Y1 * r1 * t7 * t10 * t11 - X1 * r3 * t12 * w2 * w3 +
           Y1 * r3 * t12 * w1 * w3 + Z1 * r1 * t12 * w2 * w3 -
           Z1 * r2 * t12 * w1 * w3 + Y1 * r3 * t10 * t11 * w1 * w3 +
           Z1 * r1 * t10 * t11 * w2 * w3 - Z1 * r2 * t10 * t11 * w1 * w3 -
           X1 * r1 * t6 * t10 * t11 * w3 - X1 * r1 * t7 * t10 * t11 * w3 +
           X1 * r3 * t7 * t10 * t11 * w1 - Y1 * r2 * t5 * t10 * t11 * w3 -
           Y1 * r2 * t7 * t10 * t11 * w3 + Y1 * r3 * t7 * t10 * t11 * w2 +
           Z1 * r1 * t7 * t10 * t11 * w1 + Z1 * r2 * t7 * t10 * t11 * w2 -
           Z1 * r3 * t5 * t10 * t11 * w3 - Z1 * r3 * t6 * t10 * t11 * w3 -
           X1 * r3 * t10 * t11 * w2 * w3 + X1 * r2 * t10 * t11 * w1 * w2 * w3 +
           Y1 * r1 * t10 * t11 * w1 * w2 * w3) -
      t26 * t65 * t93 * w3 * 2.0 -
      t14 * t93 * t101 *
          (t18 *
               (Z1 * (t46 - t113 + t114 + t115 - t13 * t14 * w2) - Y1 * t198 +
                X1 * (t49 + t51 + t52 + t118 - t7 * t10 * t25)) *
               2.0 +
           t23 *
               (X1 * (-t97 + t112 + t116 + t117 - t13 * t14 * w1) +
                Y1 * (-t46 + t113 + t114 + t115 - t13 * t14 * w2) - Z1 * t195) *
               2.0 +
           t15 *
               (t204 + Z1 * (t97 - t112 + t116 + t117 - t13 * t14 * w1) -
                X1 * (t201 + t202 - t13 * t14 * w3 * 2.0)) *
               2.0) *
          (1.0 / 2.0);
  jacs(0, 3) = r1 * t65 - t14 * t93 * t101 * t208 * (1.0 / 2.0);  // 行0列3：残差r 对 tx 的偏导(形式简洁)
  jacs(0, 4) = r2 * t65 - t14 * t93 * t101 * t212 * (1.0 / 2.0);  // 行0列4：残差r 对 ty 的偏导
  jacs(0, 5) = r3 * t65 - t14 * t93 * t101 * t216 * (1.0 / 2.0);  // 行0列5：残差r 对 tz 的偏导
  jacs(1, 0) =  // 行1列0：残差s 对 wx 的偏导
      t14 * t65 *
          (X1 * s1 * w1 * 2.0 + X1 * s2 * w2 + X1 * s3 * w3 + Y1 * s1 * w2 +
           Z1 * s1 * w3 + s1 * t1 * w1 * 2.0 + s2 * t2 * w1 * 2.0 +
           s3 * t3 * w1 * 2.0 + Y1 * s3 * t5 * t12 + Y1 * s3 * t9 * t10 -
           Z1 * s2 * t5 * t12 - Z1 * s2 * t9 * t10 - X1 * s2 * t12 * w2 -
           X1 * s3 * t12 * w3 - Y1 * s1 * t12 * w2 + Y1 * s2 * t12 * w1 * 2.0 -
           Z1 * s1 * t12 * w3 + Z1 * s3 * t12 * w1 * 2.0 +
           Y1 * s3 * t5 * t10 * t11 - Z1 * s2 * t5 * t10 * t11 +
           X1 * s2 * t12 * w1 * w3 - X1 * s3 * t12 * w1 * w2 -
           Y1 * s1 * t12 * w1 * w3 + Z1 * s1 * t12 * w1 * w2 +
           X1 * s2 * t10 * t11 * w1 * w3 - X1 * s3 * t10 * t11 * w1 * w2 -
           Y1 * s1 * t10 * t11 * w1 * w3 + Z1 * s1 * t10 * t11 * w1 * w2 -
           X1 * s1 * t6 * t10 * t11 * w1 - X1 * s1 * t7 * t10 * t11 * w1 +
           X1 * s2 * t5 * t10 * t11 * w2 + X1 * s3 * t5 * t10 * t11 * w3 +
           Y1 * s1 * t5 * t10 * t11 * w2 - Y1 * s2 * t5 * t10 * t11 * w1 -
           Y1 * s2 * t7 * t10 * t11 * w1 + Z1 * s1 * t5 * t10 * t11 * w3 -
           Z1 * s3 * t5 * t10 * t11 * w1 - Z1 * s3 * t6 * t10 * t11 * w1 +
           Y1 * s3 * t10 * t11 * w1 * w2 * w3 +
           Z1 * s2 * t10 * t11 * w1 * w2 * w3) -
      t14 * t101 * t167 *
          (t130 +
           t15 *
               (Y1 * (t46 + t47 + t48 - t113 - t138) +
                Z1 * (t35 + t36 + t37 - t94 - t139) - X1 * t121) *
               2.0 +
           t18 * (t135 + t137 - Y1 * (-t131 + t132 + t133)) * 2.0) *
          (1.0 / 2.0) -
      t26 * t65 * t167 * w1 * 2.0;
  jacs(1, 1) =  // 行1列1：残差s 对 wy 的偏导
      t14 * t65 *
          (X1 * s2 * w1 + Y1 * s1 * w1 + Y1 * s2 * w2 * 2.0 + Y1 * s3 * w3 +
           Z1 * s2 * w3 + s1 * t1 * w2 * 2.0 + s2 * t2 * w2 * 2.0 +
           s3 * t3 * w2 * 2.0 - X1 * s3 * t6 * t12 - X1 * s3 * t9 * t10 +
           Z1 * s1 * t6 * t12 + Z1 * s1 * t9 * t10 + X1 * s1 * t12 * w2 * 2.0 -
           X1 * s2 * t12 * w1 - Y1 * s1 * t12 * w1 - Y1 * s3 * t12 * w3 -
           Z1 * s2 * t12 * w3 + Z1 * s3 * t12 * w2 * 2.0 -
           X1 * s3 * t6 * t10 * t11 + Z1 * s1 * t6 * t10 * t11 +
           X1 * s2 * t12 * w2 * w3 - Y1 * s1 * t12 * w2 * w3 +
           Y1 * s3 * t12 * w1 * w2 - Z1 * s2 * t12 * w1 * w2 +
           X1 * s2 * t10 * t11 * w2 * w3 - Y1 * s1 * t10 * t11 * w2 * w3 +
           Y1 * s3 * t10 * t11 * w1 * w2 - Z1 * s2 * t10 * t11 * w1 * w2 -
           X1 * s1 * t6 * t10 * t11 * w2 + X1 * s2 * t6 * t10 * t11 * w1 -
           X1 * s1 * t7 * t10 * t11 * w2 + Y1 * s1 * t6 * t10 * t11 * w1 -
           Y1 * s2 * t5 * t10 * t11 * w2 - Y1 * s2 * t7 * t10 * t11 * w2 +
           Y1 * s3 * t6 * t10 * t11 * w3 - Z1 * s3 * t5 * t10 * t11 * w2 +
           Z1 * s2 * t6 * t10 * t11 * w3 - Z1 * s3 * t6 * t10 * t11 * w2 +
           X1 * s3 * t10 * t11 * w1 * w2 * w3 +
           Z1 * s1 * t10 * t11 * w1 * w2 * w3) -
      t26 * t65 * t167 * w2 * 2.0 -
      t14 * t101 * t167 *
          (t18 *
               (X1 * (t97 + t98 + t99 - t112 - t192) +
                Z1 * (-t35 + t94 + t95 + t96 - t139) - Y1 * t170) *
               2.0 +
           t15 * (t180 + t182 - X1 * (-t176 + t177 + t178)) * 2.0 +
           t23 * (t175 + Y1 * (t35 - t94 + t95 + t96 - t139) - Z1 * t173) *
               2.0) *
          (1.0 / 2.0);
  jacs(1, 2) =  // 行1列2：残差s 对 wz 的偏导
      t14 * t65 *
          (X1 * s3 * w1 + Y1 * s3 * w2 + Z1 * s1 * w1 + Z1 * s2 * w2 +
           Z1 * s3 * w3 * 2.0 + s1 * t1 * w3 * 2.0 + s2 * t2 * w3 * 2.0 +
           s3 * t3 * w3 * 2.0 + X1 * s2 * t7 * t12 + X1 * s2 * t9 * t10 -
           Y1 * s1 * t7 * t12 - Y1 * s1 * t9 * t10 + X1 * s1 * t12 * w3 * 2.0 -
           X1 * s3 * t12 * w1 + Y1 * s2 * t12 * w3 * 2.0 - Y1 * s3 * t12 * w2 -
           Z1 * s1 * t12 * w1 - Z1 * s2 * t12 * w2 + X1 * s2 * t7 * t10 * t11 -
           Y1 * s1 * t7 * t10 * t11 - X1 * s3 * t12 * w2 * w3 +
           Y1 * s3 * t12 * w1 * w3 + Z1 * s1 * t12 * w2 * w3 -
           Z1 * s2 * t12 * w1 * w3 - X1 * s3 * t10 * t11 * w2 * w3 +
           Y1 * s3 * t10 * t11 * w1 * w3 + Z1 * s1 * t10 * t11 * w2 * w3 -
           Z1 * s2 * t10 * t11 * w1 * w3 - X1 * s1 * t6 * t10 * t11 * w3 -
           X1 * s1 * t7 * t10 * t11 * w3 + X1 * s3 * t7 * t10 * t11 * w1 -
           Y1 * s2 * t5 * t10 * t11 * w3 - Y1 * s2 * t7 * t10 * t11 * w3 +
           Y1 * s3 * t7 * t10 * t11 * w2 + Z1 * s1 * t7 * t10 * t11 * w1 +
           Z1 * s2 * t7 * t10 * t11 * w2 - Z1 * s3 * t5 * t10 * t11 * w3 -
           Z1 * s3 * t6 * t10 * t11 * w3 + X1 * s2 * t10 * t11 * w1 * w2 * w3 +
           Y1 * s1 * t10 * t11 * w1 * w2 * w3) -
      t26 * t65 * t167 * w3 * 2.0 -
      t14 * t101 * t167 *
          (t18 *
               (Z1 * (t46 - t113 + t114 + t115 - t138) - Y1 * t198 +
                X1 * (t49 + t51 + t52 + t118 - t199)) *
               2.0 +
           t23 *
               (X1 * (-t97 + t112 + t116 + t117 - t192) +
                Y1 * (-t46 + t113 + t114 + t115 - t138) - Z1 * t195) *
               2.0 +
           t15 *
               (t204 + Z1 * (t97 - t112 + t116 + t117 - t192) -
                X1 * (-t200 + t201 + t202)) *
               2.0) *
          (1.0 / 2.0);
  jacs(1, 3) = s1 * t65 - t14 * t101 * t167 * t208 * (1.0 / 2.0);  // 行1列3：残差s 对 tx 的偏导
  jacs(1, 4) = s2 * t65 - t14 * t101 * t167 * t212 * (1.0 / 2.0);  // 行1列4：残差s 对 ty 的偏导
  jacs(1, 5) = s3 * t65 - t14 * t101 * t167 * t216 * (1.0 / 2.0);  // 行1列5：残差s 对 tz 的偏导
}

}  // namespace ORB_SLAM3  // 命名空间结束
