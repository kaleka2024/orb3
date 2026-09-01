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
#include "OptimizableTypes.h"// g2o自定义顶点边头文件，本文件实现VertexSim3Expmap顶点

namespace ORB_SLAM3 {      // ORB‑SLAM3工程命名空间

/**
 * @brief Sim3相似变换顶点构造函数
 * @details BaseVertex<7, g2o::Sim3>：7维流形，Sim3包含旋转3维+平移3维+尺度1维，共7个自由度；存储类型为g2o::Sim3
 * @var _marginalized 是否被边缘化标记
 * @var _fix_scale 是否固定尺度，闭环优化时可锁定尺度不做优化
 */
VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, g2o::Sim3>() {
  _marginalized = false;  // 初始化边缘化标记为false，代表尚未被边缘化
  _fix_scale = false;     // 初始化尺度不锁定，默认开启尺度优化
}

/**
 * @brief g2o序列化读取接口：从g2o文件读取Sim3顶点数据、两套相机内参
 * @param is 输入文件流
 * @return true 读取完成返回真
 */
bool VertexSim3Expmap::read(std::istream& is) {
  g2o::Vector7 cam2world; // 7维向量，存储log空间下cam2world的Sim3李代数参数
  // 读取前6维：旋转+平移的李代数参数
  for (int i = 0; i < 6; i++) {
    is >> cam2world[i];
  }
  is >> cam2world[6];     // 读取第7维：尺度对数参数

  float nextParam;
  // 读取第一个相机pCamera1的全部内参参数，并赋值给相机对象
  for (size_t i = 0; i < pCamera1->size(); i++) {
    is >> nextParam;
    pCamera1->setParameter(nextParam, i);
  }

  // 读取第二个相机pCamera2的全部内参参数，并赋值给相机对象
  for (size_t i = 0; i < pCamera2->size(); i++) {
    is >> nextParam;
    pCamera2->setParameter(nextParam, i);
  }

  // 由7维李代数向量构造Sim3，再取逆得到world‑to‑cam，设置为本顶点估计值
  setEstimate(g2o::Sim3(cam2world).inverse());
  return true;
}

/**
 * @brief g2o序列化写入接口：将Sim3顶点、两套相机内参输出写入g2o文件
 * @param os 输出文件流
 * @return os.good() 返回流状态标识写入是否正常
 */
bool VertexSim3Expmap::write(std::ostream& os) const {
  // 将顶点存储的world‑to‑cam取逆，得到cam‑to‑world的Sim3变换
  g2o::Sim3 cam2world(estimate().inverse());
  // 对Sim3取log，映射到7维李代数向量空间
  g2o::Vector7 lv = cam2world.log();
  // 输出7维Sim3李代数参数
  for (int i = 0; i < 7; i++) {
    os << lv[i] << " ";
  }

  // 输出pCamera1相机的全部内参
  for (size_t i = 0; i < pCamera1->size(); i++) {
    os << pCamera1->getParameter(i) << " ";
  }

  // 输出pCamera2相机的全部内参
  for (size_t i = 0; i < pCamera2->size(); i++) {
    os << pCamera2->getParameter(i) << " ";
  }

  return os.good();
}

}  // namespace ORB_SLAM3
