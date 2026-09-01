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
#include "Logging.h"        // ORB‑SLAM3日志模块头文件
#include "OptimizableTypes.h"// 可优化顶点、边自定义类型头文件，本文件实现重投影误差边EdgeSE3ProjectXYZ

namespace ORB_SLAM3 {      // ORB‑SLAM3工程命名空间

/**
 * @brief g2o二元边构造函数：SE3位姿 + 3D地图点，观测为2维像素重投影坐标
 * @details BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ, g2o::VertexSE3Expmap>
 *  2：观测维度，像素u、v二维；
 *  Eigen::Vector2d：观测值类型，像素坐标；
 *  g2o::VertexPointXYZ：顶点0，3D地图点顶点；
 *  g2o::VertexSE3Expmap：顶点1，相机SE3位姿顶点，流形李代数exp‑map表示
 */
EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ()
    : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                     g2o::VertexSE3Expmap>() {}

/**
 * @brief 从输入流读取边数据：读取观测像素值、信息矩阵，g2o图保存加载回调接口
 * @param is 输入istream流，读取.g2o文件二进制/文本内容
 * @return true 读取成功
 */
bool EdgeSE3ProjectXYZ::read(std::istream& is) {
  // 循环读取2维像素观测 _measurement[0]=u，_measurement[1]=v
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  // 读取2×2信息矩阵（误差协方差矩阵的逆），只读取上三角，填充对称下三角
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      // 非对角线元素，复制赋值完成矩阵对称
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}

/**
 * @brief 将边数据写入输出流：保存观测、信息矩阵到g2o文件，g2o序列化回调
 * @param os 输出ostream流，输出.g2o文本
 * @return os.good() 返回流状态，标识写入是否正常
 */
bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {
  // 输出二维像素观测值 u v
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  // 只输出信息矩阵上三角部分，g2o文件存储约定
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

/**
 * @brief g2o边核心函数，计算雅可比矩阵，BA优化时调用
 * @details _jacobianOplusXi：边对顶点0(3D地图点)的雅可比；_jacobianOplusXj：边对顶点1(相机SE3位姿)的雅可比
 * 重投影误差 e = u_obs - proj(T * Pw)，Pw世界坐标系3D点，T相机位姿Tcw
 */
void EdgeSE3ProjectXYZ::linearizeOplus() {
  // 获取顶点1：相机SE3位姿顶点，_vertices[1]对应位姿顶点
  const g2o::VertexSE3Expmap* vj =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
  // 获取当前估计的相机SE3变换Tcw
  const g2o::SE3Quat T(vj->estimate());
  // 获取顶点0：世界坐标系下3D地图点顶点
  const g2o::VertexPointXYZ* vi =
      static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
  // 获取当前估计的世界坐标系三维点坐标Pw
  const Eigen::Vector3d xyz = vi->estimate();
  // 世界点左乘Tcw，变换到相机坐标系下 Pc = T * Pw
  const Eigen::Vector3d xyz_trans = T.map(xyz);

  // 相机坐标系三维点分量 x,y,z
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];

  // 获取相机投影模型对相机坐标系三维点的雅可比 ∂proj(Pc)/∂Pc，2×3矩阵，带负号
  Eigen::Matrix<double, 2, 3> projectJac = -pCamera->projectJac(xyz_trans);

  // 雅可比：重投影误差对世界坐标系3D地图点Pw的导数；∂e/∂Pw = ∂proj/∂Pc * Rcw
  _jacobianOplusXi = projectJac * T.rotation().toRotationMatrix();

  // SE3扰动关于李代数增量ξ的雅可比矩阵 ∂(T⊙exp(ξ)·P)/∂ξ，3行6列
  SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
              -z, 0.f, x, 0.f, 1.f, 0.f,
               y, -x, 0.f, 0.f, 0.f, 1.f;

  // 雅可比：重投影误差对相机SE3李代数增量ξ的导数 ∂e/∂ξ = ∂proj/∂Pc * ∂(Pc)/∂ξ
  _jacobianOplusXj = projectJac * SE3deriv;
}

}  // namespace ORB_SLAM3
