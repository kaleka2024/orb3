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
#include "Logging.h"          // ORB-SLAM3 日志模块头文件
#include "OptimizableTypes.h" // 可优化顶点、边自定义类型头文件，本文件实现重投影误差边 EdgeSE3ProjectXYZ

namespace ORB_SLAM3 {         // ORB-SLAM3 工程命名空间

/**
 * @brief g2o 二元边构造函数：SE3 位姿 + 3D 地图点，观测为 2 维像素重投影坐标
 * @details BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ, g2o::VertexSE3Expmap>
 *  2：观测维度，像素 u、v 二维；
 *  Eigen::Vector2d：观测值类型，像素坐标；
 *  g2o::VertexPointXYZ：顶点 0，3D 地图点顶点；
 *  g2o::VertexSE3Expmap：顶点 1，相机 SE3 位姿顶点，流形李代数 exp-map 表示
 */
EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ()
    : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                     g2o::VertexSE3Expmap>() {}

/**
 * @brief 从输入流读取边数据：读取观测像素值、信息矩阵，g2o 图保存加载回调接口
 * @param is 输入 istream 流，读取 .g2o 文件二进制/文本内容
 * @return true 读取成功
 */
bool EdgeSE3ProjectXYZ::read(std::istream& is) {
  // 循环读取 2 维像素观测 _measurement[0]=u，_measurement[1]=v
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  // 读取 2×2 信息矩阵（误差协方差矩阵的逆），只读取上三角，填充对称下三角
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      // 非对角线元素，复制赋值完成矩阵对称
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}

/**
 * @brief 将边数据写入输出流：保存观测、信息矩阵到 g2o 文件，g2o 序列化回调
 * @param os 输出 ostream 流，输出 .g2o 文本
 * @return os.good() 返回流状态，标识写入是否正常
 */
bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {
  // 输出二维像素观测值 u v
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  // 只输出信息矩阵上三角部分，g2o 文件存储约定
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

/**
 * @brief g2o 边核心函数，计算雅可比矩阵，BA 优化时调用
 * @details _jacobianOplusXi：边对顶点 0(3D 地图点)的雅可比；
 *          _jacobianOplusXj：边对顶点 1(相机 SE3 位姿)的雅可比
 * 重投影误差 e = u_obs - proj(T * Pw)，Pw 世界坐标系 3D 点，T 相机位姿 Tcw
 *
 * 数学推导：
 *   Pc = Tcw * Pw                    (世界 → 相机)
 *   e  = u_obs - π(Pc)               (π 为相机投影模型)
 *
 *   对世界点 Pw 的雅可比：
 *     ∂e/∂Pw = ∂e/∂Pc * ∂Pc/∂Pw = (-projectJac) * Rcw
 *   对位姿 ξ 的右扰动雅可比：
 *     Pc' = exp(ξ^) * Pc
 *     ∂Pc/∂ξ = [I_3x3 | -[Pc]_×]    (3×6)
 *     ∂e/∂ξ  = ∂e/∂Pc * ∂Pc/∂ξ = (-projectJac) * SE3deriv
 */
void EdgeSE3ProjectXYZ::linearizeOplus() {
  // 获取顶点 1：相机 SE3 位姿顶点，_vertices[1] 对应位姿顶点
  const g2o::VertexSE3Expmap* vj =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
  // 获取当前估计的相机 SE3 变换 Tcw
  const g2o::SE3Quat T(vj->estimate());
  // 获取顶点 0：世界坐标系下 3D 地图点顶点
  const g2o::VertexPointXYZ* vi =
      static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
  // 获取当前估计的世界坐标系三维点坐标 Pw
  const Eigen::Vector3d xyz = vi->estimate();
  // 世界点左乘 Tcw，变换到相机坐标系下 Pc = T * Pw
  const Eigen::Vector3d xyz_trans = T.map(xyz);

  // 相机坐标系三维点分量 x, y, z
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];

  // 获取相机投影模型对相机坐标系三维点的雅可比 ∂proj(Pc)/∂Pc，2×3 矩阵，带负号
  Eigen::Matrix<double, 2, 3> projectJac = -pCamera->projectJac(xyz_trans);

  // 雅可比：重投影误差对世界坐标系 3D 地图点 Pw 的导数；∂e/∂Pw = ∂proj/∂Pc * Rcw
  _jacobianOplusXi = projectJac * T.rotation().toRotationMatrix();

  // ==========================================================================
  //  关键修复：显式声明 SE3deriv
  //  --------------------------------------------------------------------------
  //  SE3deriv 描述「相机坐标系下的 3D 点 Pc 对 SE3 右扰动 ξ 的导数」，
  //  即 ∂(exp(ξ^) * Pc) / ∂ξ，是一个 3 行 6 列的矩阵。
  //  原代码直接使用 `SE3deriv << ...` 但从未声明，导致编译报错：
  //      error: 'SE3deriv' was not declared in this scope
  //  这里补上声明。
  // ==========================================================================
  Eigen::Matrix<double, 3, 6> SE3deriv;

  // SE3 扰动关于李代数增量 ξ 的雅可比矩阵 ∂(T⊙exp(ξ)·P)/∂ξ，3 行 6 列
  // 分块形式：[ I_3x3 | -[Pc]_× ]
  //   I_3x3 对应平移扰动 ρ
  //   -[Pc]_× 对应旋转扰动 φ
  // 其中 [Pc]_× 为 Pc 的反对称矩阵：
  //   [  0  -z   y ]
  //   [  z   0  -x ]
  //   [ -y   x   0 ]
  // 因此 -[Pc]_× 为：
  //   [  0   z  -y ]
  //   [ -z   0   x ]
  //   [  y  -x   0 ]
  SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
              -z, 0.f, x, 0.f, 1.f, 0.f,
               y, -x, 0.f, 0.f, 0.f, 1.f;

  // 雅可比：重投影误差对相机 SE3 李代数增量 ξ 的导数 ∂e/∂ξ = ∂proj/∂Pc * ∂(Pc)/∂ξ
  // projectJac 是 2×3，SE3deriv 是 3×6，相乘得 2×6
  _jacobianOplusXj = projectJac * SE3deriv;
}

}  // namespace ORB_SLAM3