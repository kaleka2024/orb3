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
#include "OptimizableTypes.h"// g2o自定义顶点边类型头文件，实现Sim3尺度相似变换相关重投影边

namespace ORB_SLAM3 {      // ORB‑SLAM3工程命名空间

/**
 * @brief Sim3相似变换重投影二元边构造函数
 * @details BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ, VertexSim3Expmap>
 * 2：观测维度，二维像素u、v；
 * Eigen::Vector2d：观测值类型，像素坐标；
 * g2o::VertexPointXYZ：顶点0，世界坐标系3D地图点；
 * VertexSim3Expmap：顶点1，Sim3相似变换顶点，包含旋转+平移+尺度，使用exp‑map流形；
 * @note 用于回环闭合、地图融合，优化Sim3位姿与3D地图点，计算重投影误差
 */
EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ()
    : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                          VertexSim3Expmap>() {}

/**
 * @brief g2o序列化读取接口，读取EdgeSim3ProjectXYZ边的观测像素与信息矩阵
 * @param is 输入文件流，读取g2o格式文件
 * @return true 读取完成返回真
 */
bool EdgeSim3ProjectXYZ::read(std::istream& is) {
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
 * @brief g2o序列化写入接口，输出EdgeSim3ProjectXYZ边观测、信息矩阵到g2o文件
 * @param os 输出文件流
 * @return os.good() 返回流状态标识写入是否正常
 */
bool EdgeSim3ProjectXYZ::write(std::ostream& os) const {
  // 输出二维像素观测 u v
  for (int i = 0; i < 2; i++) {
    os << _measurement[i] << " ";
  }

  // 仅输出信息矩阵上三角，遵循g2o存储格式约定
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

/**
 * @brief 逆Sim3重投影二元边构造函数
 * @details BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ, VertexSim3Expmap>
 * 顶点0：3D地图点；顶点1：Sim3相似变换；
 * @note EdgeSim3ProjectXYZ使用T_sim3做投影；EdgeInverseSim3ProjectXYZ使用T_sim3的逆变换做投影；
 * 回环优化双向约束，两条边分别从两个地图坐标系互相投影构建误差约束
 */
EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ()
    : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                          VertexSim3Expmap>() {}

/**
 * @brief g2o序列化读取接口，读取EdgeInverseSim3ProjectXYZ边观测与信息矩阵
 * @param is 输入文件流，读取g2o格式文件
 * @return true 读取完成返回真
 */
bool EdgeInverseSim3ProjectXYZ::read(std::istream& is) {
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
 * @brief g2o序列化写入接口，输出EdgeInverseSim3ProjectXYZ边观测、信息矩阵到g2o文件
 * @param os 输出文件流
 * @return os.good() 返回流状态标识写入是否正常
 */
bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const {
  // 输出二维像素观测 u v
  for (int i = 0; i < 2; i++) {
    os << _measurement[i] << " ";
  }

  // 仅输出信息矩阵上三角，遵循g2o存储格式约定
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

}  // namespace ORB_SLAM3
