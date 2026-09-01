/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
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
#include "Logging.h"        // ORB‑SLAM3日志模块头文件
#include "OptimizableTypes.h"// g2o自定义顶点、边类型头文件，本文件实现EdgeSE3ProjectXYZToBody

namespace ORB_SLAM3 {      // ORB‑SLAM3工程命名空间

/**
 * @brief 二元重投影边构造函数，同时优化3D地图点与IMU本体位姿
 * @details BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ, g2o::VertexSE3Expmap>
 *  2：观测维度，像素u、v二维；
 *  Eigen::Vector2d：观测值类型，像素坐标；
 *  g2o::VertexPointXYZ：顶点0 vi，世界坐标系3D地图点；
 *  g2o::VertexSE3Expmap：顶点1 vj，IMU本体(body)SE3位姿，采用exp‑map李代数流形；
 *  mTrl为IMU‑相机外参T_cam_body，将本体坐标系变换到相机坐标系
 */
EdgeSE3ProjectXYZToBody::EdgeSE3ProjectXYZToBody()
    : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                     g2o::VertexSE3Expmap>() {}

/**
 * @brief g2o序列化读取接口：读取像素观测值与信息矩阵
 * @param is 输入文件流，读取g2o格式文件
 * @return true 读取完成返回真
 */
bool EdgeSE3ProjectXYZToBody::read(std::istream& is) {
  // 循环读取二维像素观测u、v，存入基类成员_measurement
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  // 读取2×2信息矩阵，仅读取上三角元素，手动填充对称下三角
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      // 非对角线元素复制赋值，保证信息矩阵对称
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}

/**
 * @brief g2o序列化写入接口：输出观测与信息矩阵到g2o文件
 * @param os 输出文件流
 * @return os.good() 返回流状态标识写入是否正常
 */
bool EdgeSE3ProjectXYZToBody::write(std::ostream& os) const {
  // 输出二维像素观测 u v
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  // 仅输出信息矩阵上三角，遵循g2o存储格式约定
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

/**
 * @brief g2o边线性化函数，计算两组雅可比矩阵
 * @details _jacobianOplusXi：误差对顶点0(世界坐标系3D地图点)的雅可比；
 *          _jacobianOplusXj：误差对顶点1(IMU本体SE3位姿李代数增量)的雅可比；
 *          完整相机位姿 T_rw = mTrl * T_lw，T_lw为IMU本体到世界变换；
 *          世界点Xw先到IMU本体坐标系，再经外参变换到相机坐标系后做投影
 */
void EdgeSE3ProjectXYZToBody::linearizeOplus() {
  // 获取顶点1：IMU本体(body)SE3位姿优化顶点
  g2o::VertexSE3Expmap* vj = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
  // 获取当前估计IMU本体到世界坐标系变换T_lw(Tbw)
  g2o::SE3Quat T_lw(vj->estimate());
  // IMU‑相机外参乘本体位姿，得到相机到世界坐标系完整变换T_rw(Tcw)
  g2o::SE3Quat T_rw = mTrl * T_lw;
  // 获取顶点0：世界坐标系3D地图点优化顶点
  g2o::VertexPointXYZ* vi = static_cast<g2o::VertexPointXYZ*>(_vertices[0]);
  // 获取当前估计世界坐标系三维点X_w
  Eigen::Vector3d X_w = vi->estimate();
  // 将世界点变换到IMU本体坐标系得到X_l
  Eigen::Vector3d X_l = T_lw.map(X_w);
  // 世界点经过IMU本体位姿+外参，变换到相机坐标系得到X_r
  Eigen::Vector3d X_r = mTrl.map(T_lw.map(X_w));

  // 重投影误差对世界坐标系3D地图点X_w的雅可比；链式求导：∂e/∂Xw = -∂proj/∂Pc * R_cw
  _jacobianOplusXi =
      -pCamera->projectJac(X_r) * T_rw.rotation().toRotationMatrix();

  // 取出IMU本体坐标系三维点X_l的x,y,z分量
  double x = X_l[0];
  double y = X_l[1];
  double z = X_l[2];

  // SE3李代数扰动对IMU本体坐标系三维点的雅可比矩阵，3行6列 ∂(Tbw·exp(ξ)Xw)/∂ξ
  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
              -z, 0.f, x, 0.f, 1.f, 0.f,
               y, -x, 0.f, 0.f, 0.f, 1.f;

  // 重投影误差对IMU本体SE3李代数增量ξ的雅可比；链式求导引入IMU‑相机外参旋转矩阵
  _jacobianOplusXj =
      -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
}

}  // namespace ORB_SLAM3
