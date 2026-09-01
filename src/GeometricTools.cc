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

// 几何工具集头文件：提供SLAM核心几何计算函数，包括基础矩阵计算、特征点三角化等
#include "GeometricTools.h"
// 关键帧类头文件：提供关键帧位姿、相机内参等数据接口
#include "KeyFrame.h"

namespace ORB_SLAM3 {

// ==============================================
// 功能：计算两个关键帧之间的基础矩阵 F₁₂
// 几何意义：描述两视图之间的对极几何约束，满足 x₁ᵀ F₁₂ x₂ = 0
// 推导公式：F = K₁⁻ᵀ · [t_c₁c₂]× · R_c₁c₂ · K₂⁻¹
// 输入：pKF1 关键帧1指针，pKF2 关键帧2指针
// 返回：3×3单精度基础矩阵F12
// ==============================================
Eigen::Matrix3f GeometricTools::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2) {
  // 获取关键帧1的世界→相机SE3位姿 T_c1w
  Sophus::SE3<float> Tc1w = pKF1->GetPose();
  // 提取关键帧1的旋转矩阵 R_c1w（世界坐标系到相机1坐标系的旋转）
  Sophus::Matrix3<float> Rc1w = Tc1w.rotationMatrix();
  // 提取关键帧1的平移向量 t_c1w（世界原点在相机1坐标系下的位置）
  Sophus::SE3<float>::TranslationMember tc1w = Tc1w.translation();

  // 获取关键帧2的世界→相机SE3位姿 T_c2w
  Sophus::SE3<float> Tc2w = pKF2->GetPose();
  // 提取关键帧2的旋转矩阵 R_c2w
  Sophus::Matrix3<float> Rc2w = Tc2w.rotationMatrix();
  // 提取关键帧2的平移向量 t_c2w
  Sophus::SE3<float>::TranslationMember tc2w = Tc2w.translation();

  // 计算相机2到相机1的相对旋转：R_c1c2 = R_c1w * R_c2wᵀ
  Sophus::Matrix3<float> Rc1c2 = Rc1w * Rc2w.transpose();
  // 计算相机2光心在相机1坐标系下的平移：t_c1c2 = t_c1w - R_c1c2 * t_c2w
  Eigen::Vector3f tc1c2 = -Rc1c2 * tc2w + tc1w;

  // 计算平移向量的反对称矩阵 [t]×，将叉乘运算转换为矩阵乘法：a×b = [a]× · b
  Eigen::Matrix3f tc1c2x = Sophus::SO3f::hat(tc1c2);

  // 获取关键帧1的相机内参矩阵K₁（Eigen格式）
  const Eigen::Matrix3f K1 = pKF1->mpCamera->toK_();
  // 获取关键帧2的相机内参矩阵K₂（Eigen格式）
  const Eigen::Matrix3f K2 = pKF2->mpCamera->toK_();

  // 代入基础矩阵公式并返回：F₁₂ = K₁⁻ᵀ · [t]× · R · K₂⁻¹
  return K1.transpose().inverse() * tc1c2x * Rc1c2 * K2.inverse();
}

// ==============================================
// 功能：线性DLT三角化，从两视图恢复三维空间点坐标
// 原理：构造齐次线性方程组 AX=0，通过SVD分解求解最小二乘解
// 输入：
//   x_c1：相机1下的归一化齐次图像点（去畸变后像素转归一化坐标，z=1）
//   x_c2：相机2下的归一化齐次图像点
//   Tc1w：相机1的3×4投影矩阵（世界→相机的位姿矩阵）
//   Tc2w：相机2的3×4投影矩阵（世界→相机的位姿矩阵）
// 输出：
//   x3D：输出世界坐标系下的三维欧氏点
// 返回值：三角化有效返回true；齐次项为0（点在无穷远）返回false
// ==============================================
bool GeometricTools::Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,
                                 Eigen::Matrix<float, 3, 4> &Tc1w,
                                 Eigen::Matrix<float, 3, 4> &Tc2w,
                                 Eigen::Vector3f &x3D) {
  // 构造4×4系数矩阵A，满足 A·X = 0，X为待求三维点的齐次坐标
  Eigen::Matrix4f A;

  // 第一行约束：由相机1点的x分量推导：x₁ * P₁第3行 - P₁第1行 = 0
  A.block<1, 4>(0, 0) =
      x_c1(0) * Tc1w.block<1, 4>(2, 0) - Tc1w.block<1, 4>(0, 0);
  // 第二行约束：由相机1点的y分量推导：y₁ * P₁第3行 - P₁第2行 = 0
  A.block<1, 4>(1, 0) =
      x_c1(1) * Tc1w.block<1, 4>(2, 0) - Tc1w.block<1, 4>(1, 0);
  // 第三行约束：由相机2点的x分量推导：x₂ * P₂第3行 - P₂第1行 = 0
  A.block<1, 4>(2, 0) =
      x_c2(0) * Tc2w.block<1, 4>(2, 0) - Tc2w.block<1, 4>(0, 0);
  // 第四行约束：由相机2点的y分量推导：y₂ * P₂第3行 - P₂第2行 = 0
  A.block<1, 4>(3, 0) =
      x_c2(1) * Tc2w.block<1, 4>(2, 0) - Tc2w.block<1, 4>(1, 0);

  // 对A矩阵执行雅可比SVD分解，指定计算完整的V矩阵用于求解零空间
  Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

  // SVD结果中V矩阵的最后一列，就是AX=0的最小二乘解（齐次形式的三维点）
  Eigen::Vector4f x3Dh = svd.matrixV().col(3);

  // 齐次项为0表示空间点在无穷远处，三角化无意义，返回失败
  if (x3Dh(3) == 0) return false;

  // 齐次坐标归一化：前3维除以第4维，得到欧氏空间三维坐标
  // Euclidean coordinates
  x3D = x3Dh.head(3) / x3Dh(3);

  return true;
}

}  // namespace ORB_SLAM3
