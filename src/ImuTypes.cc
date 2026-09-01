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

// IMU类型定义头文件：包含IMU零偏、标定参数、预积分等核心数据结构声明
#include "ImuTypes.h"
// STL标准库：标准输入输出流，用于零偏格式化输出
#include <iostream>
// STL标准库：智能指针管理
#include <memory>
// STL标准库：动态数组容器，存储IMU测量序列
#include <vector>

// ORB-SLAM3内部：类型转换工具集
#include "Converter.h"
// ORB-SLAM3内部：几何计算工具集
#include "GeometricTools.h"

namespace ORB_SLAM3 {
namespace IMU {

// 数值小量阈值：用于判断旋转向量是否趋近于零，避免小角度下的数值不稳定
const float eps = 1e-4;

// ==============================================
// 功能：旋转矩阵正交归一化
// 原理：通过SVD分解找到与输入矩阵最接近的正交旋转矩阵，修正数值积分累积的正交性误差
// 输入：3×3待修正旋转矩阵
// 返回：满足正交性的标准旋转矩阵
// ==============================================
Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R) {
  // 对输入矩阵执行雅可比SVD分解，同时计算U和V矩阵
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
      R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // 正交化结果：R = U * Vᵀ，保证RᵀR=I且行列式为1
  return svd.matrixU() * svd.matrixV().transpose();
}

// ==============================================
// 功能：计算SO(3)李群的右雅可比矩阵
// 原理：BCH公式右侧近似，描述旋转向量小扰动对旋转矩阵的导数，用于协方差传播与零偏修正
// 输入：旋转向量的三个分量x,y,z
// 返回：3×3右雅可比矩阵
// ==============================================
Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y,
                                 const float &z) {
  // 3阶单位矩阵
  Eigen::Matrix3f I;
  I.setIdentity();

  // 旋转向量模长的平方
  const float d2 = x * x + y * y + z * z;
  // 旋转向量模长（转角大小）
  const float d = sqrt(d2);

  // 构造旋转向量的反对称矩阵
  Eigen::Vector3f v;
  v << x, y, z;
  Eigen::Matrix3f W = Sophus::SO3f::hat(v);

  // 转角趋近于0时，右雅可比近似为单位矩阵（一阶近似）
  if (d < eps) {
    return I;
  } else {
    // 完整右雅可比公式：J_r = I - W*(1-cosd)/d² + W²*(d-sind)/(d²*d)
    return I - W * (1.0f - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
  }
}

// ==============================================
// 功能：SO(3)右雅可比（向量输入重载）
// ==============================================
Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v) {
  return RightJacobianSO3(v(0), v(1), v(2));
}

// ==============================================
// 功能：计算SO(3)右雅可比矩阵的逆矩阵
// 原理：BCH公式的逆运算，用于从旋转误差反推李代数扰动
// ==============================================
Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y,
                                        const float &z) {
  Eigen::Matrix3f I;
  I.setIdentity();

  const float d2 = x * x + y * y + z * z;
  const float d = sqrt(d2);

  Eigen::Vector3f v;
  v << x, y, z;
  Eigen::Matrix3f W = Sophus::SO3f::hat(v);

  // 小角度下逆右雅可比近似为单位阵
  if (d < eps) {
    return I;
  } else {
    // 完整逆右雅可比公式：J_r⁻¹ = I + W/2 + W²*(1/d² - (1+cosd)/(2*d*sind))
    return I + W / 2 +
           W * W * (1.0f / d2 - (1.0f + cos(d)) / (2.0f * d * sin(d)));
  }
}

// ==============================================
// 功能：SO(3)逆右雅可比（向量输入重载）
// ==============================================
Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v) {
  return InverseRightJacobianSO3(v(0), v(1), v(2));
}

// ==============================================
// 积分旋转类：单步IMU旋转积分结果
// 存储一段积分时间内的旋转增量deltaR，以及对应的右雅可比矩阵rightJ
// ==============================================

// ==============================================
// 构造函数：由角速度、IMU零偏、积分时间，计算旋转增量与右雅可比
// ==============================================
IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel,
                                       const Bias &imuBias, const float &time) {
  // 减去陀螺零偏，得到真实角速度，乘以时间得到积分旋转向量（body系）
  const float x = (angVel(0) - imuBias.bwx) * time;
  const float y = (angVel(1) - imuBias.bwy) * time;
  const float z = (angVel(2) - imuBias.bwz) * time;

  // 旋转向量模长平方与模长
  const float d2 = x * x + y * y + z * z;
  const float d = sqrt(d2);

  // 构造旋转向量的反对称矩阵
  Eigen::Vector3f v;
  v << x, y, z;
  Eigen::Matrix3f W = Sophus::SO3f::hat(v);

  // 小角度近似
  if (d < eps) {
    // 旋转增量近似为 I + W（一阶罗德里格斯）
    deltaR = Eigen::Matrix3f::Identity() + W;
    // 右雅可比近似为单位阵
    rightJ = Eigen::Matrix3f::Identity();
  } else {
    // 罗德里格斯公式：Exp(v) = I + W*sin(d)/d + W²*(1-cosd)/d²
    deltaR = Eigen::Matrix3f::Identity() + W * sin(d) / d +
             W * W * (1.0f - cos(d)) / d2;
    // 计算对应旋转量的右雅可比矩阵
    rightJ = Eigen::Matrix3f::Identity() - W * (1.0f - cos(d)) / d2 +
             W * W * (d - sin(d)) / (d2 * d);
  }
}

// ==============================================
// IMU预积分器类：核心预积分运算
// 积分两个关键帧之间的所有IMU测量，得到旋转、速度、位置增量，以及对应协方差与零偏雅可比
// ==============================================

// ==============================================
// 构造函数：由IMU零偏与标定参数初始化预积分器
// ==============================================
Preintegrated::Preintegrated(const Bias &b_, const Calib &calib) {
  // IMU测量噪声协方差矩阵（高斯白噪声）
  Nga = calib.Cov;
  // IMU零偏随机游走协方差矩阵
  NgaWalk = calib.CovWalk;
  // 初始化预积分器状态
  Initialize(b_);
}

// ==============================================
// 拷贝构造函数：深拷贝所有预积分状态
// ==============================================
// Copy constructor
Preintegrated::Preintegrated(const std::shared_ptr<Preintegrated> &pImuPre)
    : dT(pImuPre->dT),          // 总积分时间
      C(pImuPre->C),            // 预积分误差协方差矩阵
      Info(pImuPre->Info),      // 信息矩阵（协方差逆）
      Nga(pImuPre->Nga),        // IMU测量噪声协方差
      NgaWalk(pImuPre->NgaWalk),// 零偏随机游走协方差
      b(pImuPre->b),            // 积分使用的原始零偏
      dR(pImuPre->dR),          // 旋转增量
      dV(pImuPre->dV),          // 速度增量
      dP(pImuPre->dP),          // 位置增量
      JRg(pImuPre->JRg),        // 旋转对陀螺零偏的雅可比
      JVg(pImuPre->JVg),        // 速度对陀螺零偏的雅可比
      JVa(pImuPre->JVa),        // 速度对加表零偏的雅可比
      JPg(pImuPre->JPg),        // 位置对陀螺零偏的雅可比
      JPa(pImuPre->JPa),        // 位置对加表零偏的雅可比
      avgA(pImuPre->avgA),      // 平均加速度
      avgW(pImuPre->avgW),      // 平均角速度
      bu(pImuPre->bu),          // 更新后的零偏
      db(pImuPre->db),          // 零偏更新量
      mvMeasurements(pImuPre->mvMeasurements) {} // IMU测量序列

// ==============================================
// 功能：从另一个预积分器复制所有状态
// ==============================================
void Preintegrated::CopyFrom(const std::shared_ptr<Preintegrated> &pImuPre) {
  dT = pImuPre->dT;
  C = pImuPre->C;
  Info = pImuPre->Info;
  Nga = pImuPre->Nga;
  NgaWalk = pImuPre->NgaWalk;
  b.CopyFrom(pImuPre->b);
  dR = pImuPre->dR;
  dV = pImuPre->dV;
  dP = pImuPre->dP;
  JRg = pImuPre->JRg;
  JVg = pImuPre->JVg;
  JVa = pImuPre->JVa;
  JPg = pImuPre->JPg;
  JPa = pImuPre->JPa;
  avgA = pImuPre->avgA;
  avgW = pImuPre->avgW;
  bu.CopyFrom(pImuPre->bu);
  db = pImuPre->db;
  mvMeasurements = pImuPre->mvMeasurements;
}

// ==============================================
// 功能：初始化/重置预积分器所有状态
// ==============================================
void Preintegrated::Initialize(const Bias &b_) {
  dR.setIdentity();       // 旋转增量初始化为单位阵
  dV.setZero();          // 速度增量初始化为0
  dP.setZero();          // 位置增量初始化为0
  JRg.setZero();         // 旋转对陀螺零偏雅可比初始化为0
  JVg.setZero();         // 速度对陀螺零偏雅可比初始化为0
  JVa.setZero();         // 速度对加表零偏雅可比初始化为0
  JPg.setZero();         // 位置对陀螺零偏雅可比初始化为0
  JPa.setZero();         // 位置对加表零偏雅可比初始化为0
  C.setZero();           // 协方差矩阵初始化为0
  Info.setZero();        // 信息矩阵初始化为0
  db.setZero();          // 零偏增量初始化为0
  b = b_;                // 设置积分使用的原始零偏
  bu = b_;               // 更新零偏初始化为原始零偏
  avgA.setZero();        // 平均加速度初始化为0
  avgW.setZero();        // 平均角速度初始化为0
  dT = 0.0f;             // 总积分时间初始化为0
  mvMeasurements.clear();// 清空测量序列
}

// ==============================================
// 功能：使用更新后的零偏重新积分所有测量
// 场景：零偏估计更新后，无需重新传播，直接重积分得到准确预积分量
// ==============================================
void Preintegrated::Reintegrate() {
  // 加锁保证线程安全
  std::unique_lock<std::mutex> lock(mMutex);
  // 备份当前所有IMU测量
  const std::vector<integrable> aux = mvMeasurements;
  // 用更新后的零偏bu重置预积分器
  Initialize(bu);
  // 遍历所有测量，重新执行积分
  for (size_t i = 0; i < aux.size(); i++)
    IntegrateNewMeasurement(aux[i].a, aux[i].w, aux[i].t);
}

// ==============================================
// 功能：单步IMU预积分核心函数
// 输入：body系加速度、body系角速度、积分时间步长
// 说明：积分顺序为 位置→速度→旋转，保证依赖关系正确；同时传播协方差与零偏雅可比
// ==============================================
void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &acceleration,
                                            const Eigen::Vector3f &angVel,
                                            const float &dt) {
  // 将当前IMU测量存入序列，用于后续重积分
  mvMeasurements.push_back(integrable(acceleration, angVel, dt));

  // Position is updated firstly, as it depends on previously computed velocity
  // and rotation. Velocity is updated secondly, as it depends on previously
  // computed rotation. Rotation is the last to be updated.
  // 积分顺序说明：位置先更新（依赖历史速度、旋转），速度次之（依赖历史旋转），旋转最后更新

  // Matrices to compute covariance
  // 状态转移矩阵A（9×9）：对应状态[φ, δv, δp]，用于协方差传播
  Eigen::Matrix<float, 9, 9> A;
  A.setIdentity();
  // 噪声输入矩阵B（9×6）：对应噪声[n_ω, n_a]，将IMU噪声映射到状态误差
  Eigen::Matrix<float, 9, 6> B;
  B.setZero();

  // 减去零偏，得到真实的body系加速度与角速度
  Eigen::Vector3f acc, accW;
  acc << acceleration(0) - b.bax, acceleration(1) - b.bay,
      acceleration(2) - b.baz;
  accW << angVel(0) - b.bwx, angVel(1) - b.bwy, angVel(2) - b.bwz;

  // 递推计算平均加速度与平均角速度（时间加权平均）
  avgA = (dT * avgA + dR * acc * dt) / (dT + dt);
  avgW = (dT * avgW + accW * dt) / (dT + dt);

  // Update delta position dP and velocity dV (rely on no-updated delta
  // rotation)
  // 更新位置增量：dP = dP + dV·dt + 0.5·dR·a·dt² （起始body系下）
  dP = dP + dV * dt + 0.5f * dR * acc * dt * dt;
  // 更新速度增量：dV = dV + dR·a·dt
  dV = dV + dR * acc * dt;

  // Compute velocity and position parts of matrices A and B (rely on
  // non-updated delta rotation)
  // 构造加速度的反对称矩阵
  Eigen::Matrix<float, 3, 3> Wacc = Sophus::SO3f::hat(acc);

  // 状态转移矩阵A：速度误差块（3-5行，0-2列）：δv_k+1 = δv_k - R·[a]×·δφ·dt
  A.block<3, 3>(3, 0) = -dR * dt * Wacc;
  // 状态转移矩阵A：位置误差块（6-8行，0-2列）：δp_k+1 = δp_k + δv_k·dt - 0.5·R·[a]×·δφ·dt²
  A.block<3, 3>(6, 0) = -0.5f * dR * dt * dt * Wacc;
  // 状态转移矩阵A：位置误差块（6-8行，3-5列）：δp对δv的传递，系数为dt
  A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<float, 3>(dt, dt, dt);

  // 噪声矩阵B：速度误差对加速度噪声（3-5行，3-5列）：δv 由 a_noise 激励
  B.block<3, 3>(3, 3) = dR * dt;
  // 噪声矩阵B：位置误差对加速度噪声（6-8行，3-5列）：δp 由 a_noise 二次激励
  B.block<3, 3>(6, 3) = 0.5f * dR * dt * dt;

  // Update position and velocity jacobians wrt bias correction
  // 更新位置对加表零偏的雅可比：JPa = JPa + JVa·dt - 0.5·R·dt²
  JPa = JPa + JVa * dt - 0.5f * dR * dt * dt;
  // 更新位置对陀螺零偏的雅可比：JPg = JPg + JVg·dt - 0.5·R·dt²·[a]×·JRg
  JPg = JPg + JVg * dt - 0.5f * dR * dt * dt * Wacc * JRg;
  // 更新速度对加表零偏的雅可比：JVa = JVa - R·dt
  JVa = JVa - dR * dt;
  // 更新速度对陀螺零偏的雅可比：JVg = JVg - R·dt·[a]×·JRg
  JVg = JVg - dR * dt * Wacc * JRg;

  // Update delta rotation
  // 计算单步旋转增量
  IntegratedRotation dRi(angVel, b, dt);
  // 更新总旋转增量：dR = dR · dRi，随后正交归一化修正误差
  dR = NormalizeRotation(dR * dRi.deltaR);

  // Compute rotation parts of matrices A and B
  // 状态转移矩阵A：旋转误差块（0-2行，0-2列）：δφ_k+1 = ΔRᵀ·δφ_k （右扰动模型）
  A.block<3, 3>(0, 0) = dRi.deltaR.transpose();
  // 噪声矩阵B：旋转误差对角速度噪声（0-2行，0-2列）：δφ 由 ω_noise 激励
  B.block<3, 3>(0, 0) = dRi.rightJ * dt;

  // Update covariance
  // 协方差传播：P_k+1 = A·P_k·Aᵀ + B·Q·Bᵀ
  C.block<9, 9>(0, 0) =
      A * C.block<9, 9>(0, 0) * A.transpose() + B * Nga * B.transpose();
  // 零偏随机游走噪声累积：加到协方差矩阵的零偏对应块
  C.block<6, 6>(9, 9) += NgaWalk;

  // Update rotation jacobian wrt bias correction
  // 更新旋转对陀螺零偏的雅可比：JRg = ΔRᵀ·JRg - J_r·dt
  JRg = dRi.deltaR.transpose() * JRg - dRi.rightJ * dt;

  // Total integrated time
  // 累加总积分时间
  dT += dt;
}

// ==============================================
// 功能：合并前一段预积分的所有测量，与当前测量一起重新积分
// 场景：两段预积分拼接时，合并为一段完整预积分
// ==============================================
void Preintegrated::MergePrevious(const std::shared_ptr<Preintegrated> &pPrev) {
  // 自身合并自身直接返回
  if (pPrev.get() == this) return;

  // 双加锁：同时锁定当前与前一个预积分器，避免死锁
  std::unique_lock<std::mutex> lock1(mMutex);
  std::unique_lock<std::mutex> lock2(pPrev->mMutex);

  // 构造合并使用的零偏（使用当前更新零偏）
  Bias bav;
  bav.bwx = bu.bwx;
  bav.bwy = bu.bwy;
  bav.bwz = bu.bwz;
  bav.bax = bu.bax;
  bav.bay = bu.bay;
  bav.baz = bu.baz;

  // 分别获取前一段与当前段的IMU测量序列
  const std::vector<integrable> aux1 = pPrev->mvMeasurements;
  const std::vector<integrable> aux2 = mvMeasurements;

  // 重置预积分器
  Initialize(bav);
  // 先积分前一段的所有测量
  for (size_t i = 0; i < aux1.size(); i++)
    IntegrateNewMeasurement(aux1[i].a, aux1[i].w, aux1[i].t);
  // 再积分当前段的所有测量
  for (size_t i = 0; i < aux2.size(); i++)
    IntegrateNewMeasurement(aux2[i].a, aux2[i].w, aux2[i].t);
}

// ==============================================
// 功能：设置新的更新零偏，并计算与原始零偏的差值db
// ==============================================
void Preintegrated::SetNewBias(const Bias &bu_) {
  std::unique_lock<std::mutex> lock(mMutex);
  // 保存更新后的零偏
  bu = bu_;

  // 计算零偏增量：新零偏 - 原始零偏，前3维陀螺，后3维加表
  db(0) = bu_.bwx - b.bwx;
  db(1) = bu_.bwy - b.bwy;
  db(2) = bu_.bwz - b.bwz;
  db(3) = bu_.bax - b.bax;
  db(4) = bu_.bay - b.bay;
  db(5) = bu_.baz - b.baz;
}

// ==============================================
// 功能：计算输入零偏与原始零偏的差值
// ==============================================
IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_) {
  std::unique_lock<std::mutex> lock(mMutex);
  // 返回零偏差值：输入零偏 - 原始积分零偏
  return IMU::Bias(b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz,
                   b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz);
}

// ==============================================
// 功能：给定零偏，计算修正后的旋转增量（一阶雅可比近似）
// 原理：利用零偏雅可比，近似计算零偏变化后的旋转增量，避免重积分
// ==============================================
Eigen::Matrix3f Preintegrated::GetDeltaRotation(const Bias &b_) {
  std::unique_lock<std::mutex> lock(mMutex);
  // 计算陀螺零偏变化量
  Eigen::Vector3f dbg;
  dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
  // 修正旋转：dR_corrected = dR · Exp(JRg · δb_g)，随后归一化
  return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
}

// ==============================================
// 功能：给定零偏，计算修正后的速度增量（一阶雅可比近似）
// ==============================================
Eigen::Vector3f Preintegrated::GetDeltaVelocity(const Bias &b_) {
  std::unique_lock<std::mutex> lock(mMutex);
  // 陀螺零偏变化量、加表零偏变化量
  Eigen::Vector3f dbg, dba;
  dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
  dba << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz;
  // 修正速度：dV_corrected = dV + JVg·δb_g + JVa·δb_a
  return dV + JVg * dbg + JVa * dba;
}

// ==============================================
// 功能：给定零偏，计算修正后的位置增量（一阶雅可比近似）
// ==============================================
Eigen::Vector3f Preintegrated::GetDeltaPosition(const Bias &b_) {
  std::unique_lock<std::mutex> lock(mMutex);
  Eigen::Vector3f dbg, dba;
  dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
  dba << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz;
  // 修正位置：dP_corrected = dP + JPg·δb_g + JPa·δb_a
  return dP + JPg * dbg + JPa * dba;
}

// ==============================================
// 功能：使用当前db计算修正后的旋转增量
// ==============================================
Eigen::Matrix3f Preintegrated::GetUpdatedDeltaRotation() {
  std::unique_lock<std::mutex> lock(mMutex);
  // 用存储的零偏增量db的前3维（陀螺）修正旋转
  return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * db.head(3)).matrix());
}

// ==============================================
// 功能：使用当前db计算修正后的速度增量
// ==============================================
Eigen::Vector3f Preintegrated::GetUpdatedDeltaVelocity() {
  std::unique_lock<std::mutex> lock(mMutex);
  // db前3维陀螺零偏增量，后3维加表零偏增量
  return dV + JVg * db.head(3) + JVa * db.tail(3);
}

// ==============================================
// 功能：使用当前db计算修正后的位置增量
// ==============================================
Eigen::Vector3f Preintegrated::GetUpdatedDeltaPosition() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dP + JPg * db.head(3) + JPa * db.tail(3);
}

// ==============================================
// 功能：获取原始未修正的旋转增量
// ==============================================
Eigen::Matrix3f Preintegrated::GetOriginalDeltaRotation() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dR;
}

// ==============================================
// 功能：获取原始未修正的速度增量
// ==============================================
Eigen::Vector3f Preintegrated::GetOriginalDeltaVelocity() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dV;
}

// ==============================================
// 功能：获取原始未修正的位置增量
// ==============================================
Eigen::Vector3f Preintegrated::GetOriginalDeltaPosition() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dP;
}

// ==============================================
// 功能：获取积分使用的原始零偏
// ==============================================
Bias Preintegrated::GetOriginalBias() {
  std::unique_lock<std::mutex> lock(mMutex);
  return b;
}

// ==============================================
// 功能：获取更新后的零偏
// ==============================================
Bias Preintegrated::GetUpdatedBias() {
  std::unique_lock<std::mutex> lock(mMutex);
  return bu;
}

// ==============================================
// 功能：获取零偏增量向量
// ==============================================
Eigen::Matrix<float, 6, 1> Preintegrated::GetDeltaBias() {
  std::unique_lock<std::mutex> lock(mMutex);
  return db;
}

// ==============================================
// IMU零偏结构体：包含3轴陀螺零偏 + 3轴加表零偏
// ==============================================

// ==============================================
// 功能：从另一个零偏对象复制值
// ==============================================
void Bias::CopyFrom(Bias &b) {
  bax = b.bax; // 加速度计X轴零偏
  bay = b.bay; // 加速度计Y轴零偏
  baz = b.baz; // 加速度计Z轴零偏
  bwx = b.bwx; // 陀螺仪X轴零偏
  bwy = b.bwy; // 陀螺仪Y轴零偏
  bwz = b.bwz; // 陀螺仪Z轴零偏
}

// ==============================================
// 功能：重载输出运算符，格式化打印零偏值
// ==============================================
std::ostream &operator<<(std::ostream &out, const Bias &b) {
  // 正数前补空格对齐
  if (b.bwx > 0) out << " ";
  out << b.bwx << ",";
  if (b.bwy > 0) out << " ";
  out << b.bwy << ",";
  if (b.bwz > 0) out << " ";
  out << b.bwz << ",";
  if (b.bax > 0) out << " ";
  out << b.bax << ",";
  if (b.bay > 0) out << " ";
  out << b.bay << ",";
  if (b.baz > 0) out << " ";
  out << b.baz;

  return out;
}

// ==============================================
// IMU标定参数结构体：存储IMU外参、噪声密度、随机游走等标定参数
// ==============================================

// ==============================================
// 功能：设置IMU标定参数
// 输入：
//   sophTbc：相机到IMU的外参SE3变换
//   ng：陀螺仪噪声密度（白噪声）
//   na：加速度计噪声密度（白噪声）
//   ngw：陀螺仪零偏随机游走
//   naw：加速度计零偏随机游走
// ==============================================
void Calib::Set(const Sophus::SE3<float> &sophTbc, const float &ng,
                const float &na, const float &ngw, const float &naw) {
  mbIsSet = true;

  // 噪声平方得到方差
  const float ng2 = ng * ng;
  const float na2 = na * na;
  const float ngw2 = ngw * ngw;
  const float naw2 = naw * naw;

  // Sophus/Eigen
  // 保存相机到IMU的外参Tbc
  mTbc = sophTbc;
  // 求逆得到IMU到相机的外参Tcb
  mTcb = mTbc.inverse();

  // IMU测量噪声协方差矩阵（对角阵）：前3维陀螺，后3维加表
  Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
  // IMU零偏随机游走协方差矩阵（对角阵）
  CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
}

// ==============================================
// 拷贝构造函数
// ==============================================
Calib::Calib(const Calib &calib) {
  mbIsSet = calib.mbIsSet;
  // Sophus/Eigen parameters
  mTbc = calib.mTbc;
  mTcb = calib.mTcb;
  Cov = calib.Cov;
  CovWalk = calib.CovWalk;
}

}  // namespace IMU
}  // namespace ORB_SLAM3
