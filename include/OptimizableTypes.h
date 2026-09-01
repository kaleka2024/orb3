/**
 * This file is part of ORB‑SLAM3
 *
 * Copyright (C) 2017‑2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014‑2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
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
#pragma once // 头文件保护宏，避免头文件重复包含引发重定义编译错误
#include <g2o/core/base_unary_edge.h>           // g2o一元边基类，只有一个待优化顶点
#include <g2o/core/eigen_types.h>                // g2o内置Eigen类型别名定义
#include <g2o/types/sba/types_six_dof_expmap.h>  // 6自由度SE3李代数exp映射顶点与边（SBA光束平差）
#include <g2o/types/sba/vertex_se3_expmap.h>     // SE3位姿顶点，使用exp‑map李代数更新
#include <g2o/types/sim3/sim3.h>                 // Sim3相似变换（旋转+平移+尺度）类型定义
#include <include/CameraModels/GeometricCamera.h>// ORB‑SLAM3抽象相机模型基类，支持针孔/鱼眼

#include <Eigen/Geometry> // Eigen几何模块，四元数、变换矩阵等
// #include <Eigen/Version>   // This file only exists in current versions of
// Eigen...
#include <memory> // std::shared_ptr智能指针

namespace ORB_SLAM3 {

/**
 * @brief g2o一元边：仅优化相机SE3位姿，3D地图点Xw固定，重投影误差边
 * @note 顶点：VertexSE3Expmap(相机位姿)；测量值：2D图像像素观测；3D点Xw为边内常量，不参与优化
 * @usage Tracking模块位姿优化，只优化相机位姿，地图点保持固定
 */
class EdgeSE3ProjectXYZOnlyPose
    : public g2o::BaseUnaryEdge<2, g2o::Vector2, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类含有Eigen成员必须添加

  /**
   * @brief 从输入流读取边数据，g2o序列化接口
   * @param is 输入流
   * @return 读取成功返回true
   */
  bool read(std::istream& is);
  /**
   * @brief 将边数据写入输出流，g2o序列化接口
   * @param os 输出流
   * @return 写入成功返回true
   */
  bool write(std::ostream& os) const;
  /**
   * @brief 计算重投影误差，基类虚函数重写
   */
  void computeError();
  /**
   * @brief 判断投影之后相机坐标系下深度是否大于0，剔除负深度无效约束
   * @return 深度>0返回true
   */
  bool isDepthPositive();

  /**
   * @brief 计算雅可比矩阵，线性化，重写基类接口
   */
  void linearizeOplus() override;

  g2o::Vector3 Xw;                    ///< 固定的3D地图点世界坐标，不做优化变量
  std::shared_ptr<GeometricCamera> pCamera; ///< 相机模型，用于投影计算
};

/**
 * @brief g2o一元边：仅优化外相机SE3位姿，存在IMU本体‑相机外参Trl，3D点固定，计算到本体坐标系再投影
 * @note 顶点：外相机位姿；Trl为相机到IMU本体固定外参；测量2D像素；3D点Xw固定
 * @usage IMU模式，优化相机位姿，需要通过外参转到IMU本体坐标系
 */
class EdgeSE3ProjectXYZOnlyPoseToBody
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZOnlyPoseToBody() {}          // 默认构造
  virtual ~EdgeSE3ProjectXYZOnlyPoseToBody() {} // 虚析构

  /**
   * @brief g2o序列化读接口
   * @param is 输入流
   * @return true读取成功
   */
  bool read(std::istream& is);
  /**
   * @brief g2o序列化写接口
   * @param os 输出流
   * @return true写入成功
   */
  bool write(std::ostream& os) const;

  /**
   * @brief 计算重投影误差
   */
  void computeError();
  /**
   * @brief 判断本体坐标系下深度是否为正
   * @return 深度>0返回true
   */
  bool isDepthPositive();

  /**
   * @brief 计算雅可比矩阵线性化
   */
  void linearizeOplus() override;

  Eigen::Vector3d Xw;                    ///< 固定3D地图点世界坐标
  std::shared_ptr<GeometricCamera> pCamera; ///< 相机模型对象
  g2o::SE3Quat mTrl;                     ///< IMU本体到相机的固定外参Trl
};

/**
 * @brief g2o二元边：标准SBA重投影误差边，同时优化3D点XYZ和相机SE3位姿
 * @note 顶点0：VertexPointXYZ(3D地图点)；顶点1：VertexSE3Expmap(相机位姿)；测量值2D像素观测
 * @usage LocalMapping局部BA、全局BA，光束平差，点和位姿一起优化
 */
class EdgeSE3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                                  g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZ(); // 构造函数

  /**
   * @brief g2o序列化读取
   * @param is 输入流
   * @return true读取成功
   */
  bool read(std::istream& is);
  /**
   * @brief g2o序列化写入
   * @param os 输出流
   * @return true写入成功
   */
  bool write(std::ostream& os) const;
  /**
   * @brief 计算重投影误差：观测像素 − 投影得到像素
   */
  void computeError() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexPointXYZ* v2 =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs - pCamera->project(v1->estimate().map(v2->estimate()));
  }

  /**
   * @brief 判断相机坐标系下3D点深度是否大于0，过滤无效约束
   * @return z>0返回true
   */
  bool isDepthPositive() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexPointXYZ* v2 =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    return ((v1->estimate().map(v2->estimate()))(2) > 0.0);
  }

  /**
   * @brief 计算二元边对两个顶点的雅可比矩阵
   */
  virtual void linearizeOplus();

  std::shared_ptr<GeometricCamera> pCamera; ///< 相机模型，执行投影project
};

/**
 * @brief g2o二元边：IMU模式SBA重投影误差，同时优化3D点与相机位姿，带本体‑相机外参Trl
 * @note 顶点0：3D地图点；顶点1：相机SE3位姿；mTrl固定本体‑相机外参；先把相机位姿转到IMU本体坐标系再投影
 * @usage IMU融合下局部BA、全局BA
 */
class EdgeSE3ProjectXYZToBody
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                                  g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZToBody(); // 构造函数

  /**
   * @brief g2o序列化读接口
   * @param is 输入流
   * @return true读取成功
   */
  bool read(std::istream& is);
  /**
   * @brief g2o序列化写接口
   * @param os 输出流
   * @return true写入成功
   */
  bool write(std::ostream& os) const;
  /**
   * @brief 计算重投影误差，位姿经过Trl转换到IMU本体坐标系再投影
   */
  void computeError() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexPointXYZ* v2 =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error =
        obs - pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
  }

  /**
   * @brief 判断IMU本体坐标系下点深度是否大于0
   * @return z>0返回true
   */
  bool isDepthPositive() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexPointXYZ* v2 =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    return ((mTrl * v1->estimate()).map(v2->estimate()))(2) > 0.0;
  }

  /**
   * @brief 计算雅可比矩阵线性化
   */
  virtual void linearizeOplus();

  std::shared_ptr<GeometricCamera> pCamera; ///< 相机模型对象
  g2o::SE3Quat mTrl;                        ///< IMU本体到相机固定外参
};

/**
 * @brief g2o顶点：Sim3相似变换顶点，7维（旋转3+平移3+尺度1），exp‑map流形更新
 * @note 用于回环检测，闭环Sim3位姿优化；可选择固定尺度，单目场景尺度漂移校正
 */
class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexSim3Expmap(); // 构造函数
  virtual bool read(std::istream& is);  ///< g2o序列化读取顶点
  virtual bool write(std::ostream& os) const; ///< g2o序列化写入顶点

  /**
   * @brief 重置顶点估计值到单位Sim3（零旋转、零平移、尺度1）
   */
  virtual void setToOriginImpl() { _estimate = g2o::Sim3(); }

  /**
   * @brief 流形增量更新，oplus操作；使用Sim3 exp‑map；支持固定尺度
   * @param update_ 7维增量数组，[omega(3),tau(3),s(1)]
   */
  virtual void oplusImpl(const double* update_) {
    Eigen::Map<g2o::Vector7> update(const_cast<double*>(update_));

    if (_fix_scale) update[6] = 0; // 如果开启固定尺度，尺度增量置0

    g2o::Sim3 s(update);
    setEstimate(s * estimate()); // 左乘更新Sim3估计值
  }

  std::shared_ptr<GeometricCamera> pCamera1, pCamera2; ///< 左右两个相机模型，用于Sim3投影边
  bool _fix_scale; ///< true固定尺度不优化；false允许优化尺度（单目回环）
};

/**
 * @brief g2o二元边：Sim3变换下重投影误差边，使用pCamera1做投影
 * @note 顶点0：3D地图点；顶点1：VertexSim3Expmap相似变换；测量2D像素
 * @usage 回环检测Sim3求解，把世界点经过Sim3变换投影到相机1图像
 */
class EdgeSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                                  ORB_SLAM3::VertexSim3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSim3ProjectXYZ(); // 构造函数
  virtual bool read(std::istream& is);  ///< g2o序列化读取
  virtual bool write(std::ostream& os) const; ///< g2o序列化写入

  /**
   * @brief 计算重投影误差：观测像素 − Sim3变换后投影到camera1的像素
   */
  void computeError() {
    const ORB_SLAM3::VertexSim3Expmap* v1 =
        static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
    const g2o::VertexPointXYZ* v2 =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);

    Eigen::Vector2d obs(_measurement);
    _error = obs - v1->pCamera1->project(v1->estimate().map(v2->estimate()));
  }

  // virtual void linearizeOplus();
};

/**
 * @brief g2o二元边：逆Sim3变换重投影误差边，使用pCamera2投影
 * @note 顶点0：3D地图点；顶点1：VertexSim3Expmap；取Sim3.inverse()做变换，投影到camera2
 * @usage 回环Sim3优化，双向约束，构建Sim3双边投影误差
 */
class EdgeInverseSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                                  VertexSim3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeInverseSim3ProjectXYZ(); // 构造函数
  virtual bool read(std::istream& is);  ///< g2o序列化读取
  virtual bool write(std::ostream& os) const; ///< g2o序列化写入

  /**
   * @brief 计算逆Sim3变换下重投影误差
   */
  void computeError() {
    const ORB_SLAM3::VertexSim3Expmap* v1 =
        static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
    const g2o::VertexPointXYZ* v2 =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);

    Eigen::Vector2d obs(_measurement);
    _error = obs - v1->pCamera2->project(
                       (v1->estimate().inverse().map(v2->estimate())));
  }

  // virtual void linearizeOplus();
};

}  // namespace ORB_SLAM3
