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
#include "OptimizableTypes.h"// g2o自定义边、顶点类型头文件，本文件实现EdgeSE3ProjectXYZOnlyPose

namespace ORB_SLAM3 {      // ORB‑SLAM3工程命名空间

/**
 * @brief g2o序列化读取接口：读取仅位姿边的观测像素与信息矩阵
 * @param is 输入文件流，读取g2o格式文件内容
 * @return true 读取操作完成返回真
 * @details EdgeSE3ProjectXYZOnlyPose：仅优化相机位姿，3D地图点Xw固定不作为优化顶点；一元边，仅SE3位姿顶点
 */
bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is) {
  // 循环读取二维像素观测u、v，存入基类成员_measurement
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  // 读取2×2信息矩阵，只读取上三角元素，手动填充对称下三角
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      // 非对角线元素，复制赋值保证信息矩阵对称
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}

/**
 * @brief g2o序列化写入接口：将边的观测、信息矩阵输出到g2o文件
 * @param os 输出文件流
 * @return os.good() 返回流状态，标识写入是否正常
 */
bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {
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
 * @brief g2o边计算误差函数，重投影误差；地图点Xw固定，只优化相机位姿
 * @details 误差公式 e = 观测像素 - 投影(相机位姿 * 世界3D点Xw)
 */
void EdgeSE3ProjectXYZOnlyPose::computeError() {
  // 获取唯一顶点：_vertices[0]为相机SE3位姿顶点，流形exp‑map形式
  const g2o::VertexSE3Expmap* v1 =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  // 将基类存储的二维观测值封装为Eigen::Vector2d
  Eigen::Vector2d obs(_measurement);
  // v1->estimate().map(Xw)：世界坐标系固定点Xw经相机位姿变换得到相机坐标系点，再调用相机模型投影得到预测像素；计算重投影误差存入_error
  _error = obs - pCamera->project(v1->estimate().map(Xw));
}

/**
 * @brief 判断变换后的3D点在相机坐标系下深度是否大于0，用于剔除相机后方无效点
 * @return true 深度>0，点在相机前方；false 点在相机后方，该约束无效
 */
bool EdgeSE3ProjectXYZOnlyPose::isDepthPositive() {
  // 获取相机SE3位姿顶点
  const g2o::VertexSE3Expmap* v1 =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  // map(Xw)把世界点变换至相机坐标系，取z轴深度分量判断是否大于0
  return (v1->estimate().map(Xw))(2) > 0.0;
}

/**
 * @brief g2o边线性化，计算重投影误差对相机SE3李代数增量的雅可比矩阵，仅位姿优化，地图点固定
 */
void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
  //  const g2o::VertexSE3Expmap* vi = static_cast<const
  //  g2o::VertexSE3Expmap*>(_vertices[0]); const Eigen::Vector3d xyz_trans =
  //  vi->estimate().map(Xw);

  // 获取当前待优化的相机SE3位姿顶点
  const g2o::VertexSE3Expmap* vi =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  // 获取当前估计的相机SE3变换T_lw，世界到相机的变换
  const g2o::SE3Quat T_lw = vi->estimate();
  // 将固定世界点Xw变换到相机坐标系，得到相机坐标系三维点X_l
  const Eigen::Vector3d X_l = T_lw.map(Xw);

  // 取出相机坐标系三维点的x y z分量
  const double x = X_l[0];
  const double y = X_l[1];
  const double z = X_l[2];

  // SE3李代数扰动对相机坐标系三维点的雅可比矩阵，3行6列，∂(T·exp(ξ)Xw)/∂ξ
  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
              -z, 0.f, x, 0.f, 1.f, 0.f,
               y, -x, 0.f, 0.f, 0.f, 1.f;

  // 2×6雅可比矩阵，存储重投影误差对SE3李代数增量的导数
  Eigen::Matrix<double, 2, 6> p;
  // noalias禁止临时内存，避免Eigen隐式临时对象；projectJac为投影对相机三维点雅可比，乘以SE3导数得到完整雅可比
  p.noalias() = (-pCamera->projectJac(X_l).eval() * SE3deriv);
  // 赋值给g2o一元边雅可比成员，_jacobianOplusXi：误差对唯一顶点(相机位姿)的雅可比
  _jacobianOplusXi = p;
}

}  // namespace ORB_SLAM3
