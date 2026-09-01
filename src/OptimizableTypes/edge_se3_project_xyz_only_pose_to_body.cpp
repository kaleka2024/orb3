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
#include "OptimizableTypes.h"// g2o自定义边与顶点类型头文件，本文件实现EdgeSE3ProjectXYZOnlyPoseToBody

namespace ORB_SLAM3 {      // ORB‑SLAM3工程命名空间

/**
 * @brief g2o序列化读取接口：读取IMU本体坐标系位姿边的像素观测与信息矩阵
 * @param is 输入文件流，读取g2o格式文件内容
 * @return true 读取完成返回真
 * @details EdgeSE3ProjectXYZOnlyPoseToBody：仅优化IMU本体(body)SE3位姿；mTrl为IMU‑相机外参T_cam_body；地图点Xw固定不参与优化；一元边
 */
bool EdgeSE3ProjectXYZOnlyPoseToBody::read(std::istream& is) {
  // 循环读取二维像素观测u、v，存入基类成员_measurement
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  // 读取2×2信息矩阵，仅读取上三角元素，手动填充对称下三角
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      // 非对角线元素，复制赋值保证信息矩阵对称
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}

/**
 * @brief g2o序列化写入接口：将边的观测、信息矩阵输出写入g2o文件
 * @param os 输出文件流
 * @return os.good() 返回流状态标识写入是否正常
 */
bool EdgeSE3ProjectXYZOnlyPoseToBody::write(std::ostream& os) const {
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
 * @brief g2o边计算误差函数，计算重投影误差；优化IMU本体位姿，地图点Xw固定
 * @details v1->estimate()：IMU本体到世界坐标系变换Tbw；mTrl：相机相对于IMU本体的外参Tcb；
 *          完整相机位姿 Tcw = mTrl * Tbw；误差 e = 观测像素 - proj(Tcw * Xw)
 */
void EdgeSE3ProjectXYZOnlyPoseToBody::computeError() {
  // 获取唯一优化顶点：IMU本体(body)的SE3位姿顶点
  const g2o::VertexSE3Expmap* v1 =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  // 将基类存储的二维像素观测封装为Eigen::Vector2d
  Eigen::Vector2d obs(_measurement);
  // mTrl * v1->estimate()：IMU外参乘本体位姿得到相机到世界变换；map(Xw)将世界3D点变换到相机坐标系；投影得到预测像素；计算重投影误差存入_error
  _error = obs - pCamera->project((mTrl * v1->estimate()).map(Xw));
}

/**
 * @brief 校验变换后的3D地图点在相机坐标系下深度是否为正，过滤相机后方无效点
 * @return true 深度>0，点在相机前方；false 点落在相机后方，该约束失效
 */
bool EdgeSE3ProjectXYZOnlyPoseToBody::isDepthPositive() {
  // 获取IMU本体SE3位姿顶点
  const g2o::VertexSE3Expmap* v1 =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  // mTrl * v1->estimate()得到相机位姿；map(Xw)把世界点变换至相机坐标系；取z深度分量做大于0判断
  return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
}

/**
 * @brief g2o边线性化函数，计算重投影误差对IMU本体SE3李代数增量的雅可比矩阵
 * @note 顶点是IMU本体位姿，需要链式求导引入IMU‑相机外参mTrl
 */
void EdgeSE3ProjectXYZOnlyPoseToBody::linearizeOplus() {
  // 获取IMU本体(body)SE3位姿优化顶点
  const g2o::VertexSE3Expmap* vi =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  // 获取当前估计的IMU本体到世界坐标系变换T_lw(Tbw)
  g2o::SE3Quat T_lw(vi->estimate());
  // 将世界坐标系固定点Xw变换到IMU本体坐标系得到X_l
  Eigen::Vector3d X_l = T_lw.map(Xw);
  // 通过IMU‑相机外参mTrl，把本体坐标系点变换到相机坐标系得到X_r
  Eigen::Vector3d X_r = mTrl.map(T_lw.map(Xw));

  // 取出IMU本体坐标系三维点的x,y,z分量
  double x_w = X_l[0];
  double y_w = X_l[1];
  double z_w = X_l[2];

  // SE3李代数扰动对IMU本体坐标系三维点的雅可比矩阵，3行6列 ∂(Tbw·exp(ξ)Xw)/∂ξ
  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.f, z_w, -y_w, 1.f, 0.f, 0.f,
              -z_w, 0.f, x_w, 0.f, 1.f, 0.f,
               y_w, -x_w, 0.f, 0.f, 0.f, 1.f;

  // 链式求导：∂e/∂ξ = -∂proj/∂Pc * R_cam_body * ∂P_body/∂ξ
  // projectJac(X_r)：投影对相机坐标系点的雅可比；mTrl.rotation()：IMU到相机旋转矩阵；SE3deriv：本体位姿扰动雅可比
  _jacobianOplusXi =
      -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
}

}  // namespace ORB_SLAM3
