/**  * This file is part of ORB‑SLAM3  *
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
#pragma once
#include <Frame.h>                          // 普通帧Frame类头文件
#include <KeyFrame.h>                       // 关键帧KeyFrame类头文件

#include <Eigen/Core>                       // Eigen基础矩阵向量
#include <Eigen/Dense>                      // Eigen稠密矩阵运算、SVD分解
#include <Eigen/Geometry>                   // Eigen几何模块，旋转、四元数
#include <cmath>                            // C++数学库，exp、log等
#include <memory>                           // C++智能指针shared_ptr
#include <opencv2/core/core.hpp>            // OpenCV核心Mat基础类型
#include <vector>                           // C++动态数组vector

#include "Converter.h"                      // 类型转换工具，OpenCV‑Eigen‑Sophus互转
#include "g2o/core/base_binary_edge.h"      // g2o二元边基类，连接两个顶点
#include "g2o/core/base_multi_edge.h"       // g2o多边基类，连接多于两个顶点
#include "g2o/core/base_unary_edge.h"       // g2o一元边基类，只连接一个顶点
#include "g2o/core/base_vertex.h"           // g2o顶点基类，待优化变量
#include "g2o/types/sba/types_sba.h"        // g2o自带SBA顶点，点顶点VertexPointXYZ

namespace ORB_SLAM3 {
class KeyFrame;         // 前向声明关键帧，避免头文件循环依赖
class Frame;            // 前向声明普通帧
class GeometricCamera;  // 前向声明相机模型类，支持针孔、鱼眼

typedef Eigen::Matrix<double, 6, 1> Vector6d;   // 6维向量：[旋转3维，平移3维]，SE3扰动
typedef Eigen::Matrix<double, 9, 1> Vector9d;   // 9维向量IMU残差：旋转3、速度3、位置3
typedef Eigen::Matrix<double, 12, 1> Vector12d; // 12维向量，位姿+速度+陀螺偏置
typedef Eigen::Matrix<double, 15, 1> Vector15d; // 15维向量：位姿6+速度3+bg3+ba3
typedef Eigen::Matrix<double, 12, 12> Matrix12d;// 12×12海森矩阵
typedef Eigen::Matrix<double, 15, 15> Matrix15d;// 15×15海森矩阵，IMU先验H矩阵
typedef Eigen::Matrix<double, 9, 9> Matrix9d;   // 9×9海森矩阵，IMU残差对应H

/**
 * @brief SO3指数映射，旋转向量转旋转矩阵
 * @param x,y,z 旋转向量三个分量
 * @return Eigen::Matrix3d 输出旋转矩阵R
 */
Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);
/**
 * @brief SO3指数映射，旋转向量转旋转矩阵
 * @param w 输入旋转向量
 * @return Eigen::Matrix3d 输出旋转矩阵R
 */
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w);

/**
 * @brief SO3对数映射，旋转矩阵转为旋转向量
 * @param R 输入旋转矩阵
 * @return Eigen::Vector3d 输出旋转向量
 */
Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R);

/**
 * @brief SO3右雅可比的逆矩阵
 * @param v 旋转向量
 * @return Eigen::Matrix3d Jr⁻¹
 */
Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v);
/**
 * @brief SO3右雅可比矩阵Jr
 * @param v 旋转向量
 * @return Eigen::Matrix3d Jr
 */
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v);
/**
 * @brief SO3右雅可比矩阵Jr，传入xyz三个分量
 * @param x,y,z 旋转向量分量
 * @return Eigen::Matrix3d Jr
 */
Eigen::Matrix3d RightJacobianSO3(const double x, const double y,
                                  const double z);

/**
 * @brief 反对称矩阵，由三维向量生成斜对称矩阵
 * @param w 三维向量
 * @return Eigen::Matrix3d 反对称矩阵[w]×
 */
Eigen::Matrix3d Skew(const Eigen::Vector3d& w);
/**
 * @brief SO3右雅可比的逆矩阵，传入xyz三个分量
 * @param x,y,z 旋转向量分量
 * @return Eigen::Matrix3d Jr⁻¹
 */
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y,
                                         const double z);

/**
 * @brief SVD正交化，把带数值噪声的3×3矩阵归一化为严格旋转矩阵
 * @tparam T 数值类型double/float
 * @param R 输入近似旋转矩阵，存在数值漂移
 * @return Eigen::Matrix<T,3,3> 严格正交旋转矩阵
 */
template <typename T = double> Eigen::Matrix<T, 3, 3> NormalizeRotation(const Eigen::Matrix<T, 3, 3>& R) {
  Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(
      R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.matrixU() * svd.matrixV().transpose();
}

/**
 * @brief IMU‑相机联合位姿结构体，g2o顶点存储类型；保存IMU位姿、多相机外参、相机模型
 */
class ImuCamPose {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Eigen内存对齐宏
  ImuCamPose() {}                   // 默认构造
  /**
   * @brief 根据关键帧构造ImuCamPose，提取KF的IMU位姿、相机外参
   * @param pKF 输入关键帧智能指针
   */
  explicit ImuCamPose(const std::shared_ptr<KeyFrame>& pKF);
  /**
   * @brief 根据普通帧Frame构造ImuCamPose
   * @param pF 输入普通帧智能指针
   */
  explicit ImuCamPose(const std::shared_ptr<Frame>& pF);
  /**
   * @brief 直接传入Rwc、twc和关键帧构造
   * @param _Rwc 相机到世界旋转
   * @param _twc 相机到世界平移
   * @param pKF 关联关键帧
   */
  ImuCamPose(Eigen::Matrix3d& _Rwc, Eigen::Vector3d& _twc,
             const std::shared_ptr<KeyFrame>& pKF);

  /**
   * @brief 设置多相机参数：Rcw、tcw、Rbc、tbc、bf基线*fx
   * @param _Rcw 世界到各相机旋转
   * @param _tcw 世界到各相机平移
   * @param _Rbc IMU到相机旋转外参
   * @param _tbc IMU到相机平移外参
   * @param _bf 基线乘fx
   */
  void SetParam(const std::vector<Eigen::Matrix3d>& _Rcw,
                 const std::vector<Eigen::Vector3d>& _tcw,
                 const std::vector<Eigen::Matrix3d>& _Rbc,
                 const std::vector<Eigen::Vector3d>& _tbc, const double& _bf);

  /**
   * @brief IMU坐标系下更新位姿，6维扰动，oplus更新，用于VertexPose
   * @param pu double数组，6维扰动[w1,w2,w3,u1,u2,u3]
   */
  void Update(const double* pu);
  // update in the imu reference
  /**
   * @brief 在世界坐标系下做位姿扰动更新，用于4DoF顶点
   * @param pu 6维扰动数组
   */
  void UpdateW(const double* pu);
  // update in the world reference
  /**
   * @brief 单目投影，世界点Xw投影到指定相机，返回归一化平面坐标(u,v)
   * @param Xw 世界坐标系三维点
   * @param cam_idx 相机索引，多相机系统使用，默认0主相机
   * @return Eigen::Vector2d 归一化平面u,v
   */
  Eigen::Vector2d Project(const Eigen::Vector3d& Xw,
                          int cam_idx = 0) const;  // Mono
  /**
   * @brief 双目投影，输出u,v,ur，ur = u‑bf/z
   * @param Xw 世界三维点
   * @param cam_idx 相机索引
   * @return Eigen::Vector3d (u,v,ur)
   */
  Eigen::Vector3d ProjectStereo(const Eigen::Vector3d& Xw,
                                 int cam_idx = 0) const;  // Stereo
  /**
   * @brief 判断世界点在相机坐标系深度是否大于0，用于剔除负深度异常
   * @param Xw 世界三维点
   * @param cam_idx 相机索引
   * @return true深度>0；false负深度
   */
  bool isDepthPositive(const Eigen::Vector3d& Xw, int cam_idx = 0) const;
public:
  // For IMU
  Eigen::Matrix3d Rwb;          /// IMU到世界旋转矩阵 Rwb
  Eigen::Vector3d twb;          /// IMU在世界坐标系平移 twb

  // For set of cameras
  std::vector<Eigen::Matrix3d> Rcw;    /// 世界到各个相机旋转 Rcw
  std::vector<Eigen::Vector3d> tcw;    /// 世界到各个相机平移 tcw
  std::vector<Eigen::Matrix3d> Rcb, Rbc; /// Rcb:相机‑IMU；Rbc:IMU‑相机
  std::vector<Eigen::Vector3d> tcb, tbc; /// tcb相机到IMU；tbc IMU到相机
  double bf;                            /// 基线 × fx
  std::vector<std::shared_ptr<GeometricCamera>> pCamera; /// 各个相机模型指针

  // For posegraph 4DoF
  Eigen::Matrix3d Rwb0;        /// 4DoF位姿图：初始IMU旋转，固定roll/pitch只优化yaw
  Eigen::Matrix3d DR;          /// 4DoF增量旋转
  int its;                     /// 迭代计数
};

/**
 * @brief 逆深度点结构体，g2o顶点存储；用逆深度ρ表示3D点，适合单目初始化
 */
class InvDepthPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InvDepthPoint() {}
  /**
   * @brief 逆深度点构造函数
   * @param _rho 逆深度ρ=1/z
   * @param _u host帧归一化u
   * @param _v host帧归一化v
   * @param pHostKF 观测该点的宿主关键帧
   */
  InvDepthPoint(double _rho, double _u, double _v,
                const std::shared_ptr<KeyFrame>& pHostKF);

  /**
   * @brief 逆深度扰动更新，oplus，仅更新rho
   * @param pu 扰动数组，第0元为delta_rho
   */
  void Update(const double* pu);

  double rho;               /// 逆深度 ρ=1/z，待优化变量
  double u, v;              /// they are not variables, observation in the host frame 宿主帧归一化观测，不优化
  double fx, fy, cx, cy, bf;/// 宿主帧相机内参
  int its;                  /// 迭代计数
};

// Optimizable parameters are IMU pose
/**
 * @brief g2o顶点：IMU‑相机联合6DoF位姿顶点，维度6，存储类型ImuCamPose
 */
class VertexPose : public g2o::BaseVertex<6, ImuCamPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPose() {}
  /**
   * @brief 由关键帧构造位姿顶点
   * @param pKF 关键帧
   */
  explicit VertexPose(const std::shared_ptr<KeyFrame>& pKF) {
    setEstimate(ImuCamPose(pKF));
  }
  /**
   * @brief 由普通帧构造位姿顶点
   * @param pF 普通帧
   */
  explicit VertexPose(const std::shared_ptr<Frame>& pF) {
    setEstimate(ImuCamPose(pF));
  }

  virtual bool read(std::istream& is);   // 从流读取顶点，本工程未实现
  virtual bool write(std::ostream& os) const; // 写入流，本工程未实现

  virtual void setToOriginImpl() {}      // 置零实现，此处空实现

  /**
   * @brief g2o增量更新oplus，6维扰动更新ImuCamPose
   * @param update_ 6维扰动数组
   */
  virtual void oplusImpl(const double* update_) {
    _estimate.Update(update_);
    updateCache();
  }
};

/**
 * @brief g2o顶点：4DoF位姿顶点；仅优化yaw航向角+平移，roll/pitch固定，用于位姿图
 */
class VertexPose4DoF : public g2o::BaseVertex<4, ImuCamPose> {
  // Translation and yaw are the only optimizable variables
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPose4DoF() {}
  explicit VertexPose4DoF(const std::shared_ptr<KeyFrame>& pKF) {
    setEstimate(ImuCamPose(pKF));
  }
  explicit VertexPose4DoF(const std::shared_ptr<Frame>& pF) {
    setEstimate(ImuCamPose(pF));
  }
  VertexPose4DoF(Eigen::Matrix3d& _Rwc, Eigen::Vector3d& _twc,
                 const std::shared_ptr<KeyFrame>& pKF) {
    setEstimate(ImuCamPose(_Rwc, _twc, pKF));
  }

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  virtual void setToOriginImpl() {}

  /**
   * @brief 4DoF oplus，扰动[tyaw,tx,ty,tz]，补零roll pitch，调用UpdateW
   * @param update_ 4维扰动
   */
  virtual void oplusImpl(const double* update_) {
    double update6DoF[6];
    update6DoF[0] = 0;
    update6DoF[1] = 0;
    update6DoF[2] = update_[0];
    update6DoF[3] = update_[1];
    update6DoF[4] = update_[2];
    update6DoF[5] = update_[3];
    _estimate.UpdateW(update6DoF);
    updateCache();
  }
};

/**
 * @brief g2o顶点：IMU速度顶点，维度3，Eigen::Vector3d世界坐标系速度
 */
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexVelocity() {}
  explicit VertexVelocity(const std::shared_ptr<KeyFrame>& pKF);
  explicit VertexVelocity(const std::shared_ptr<Frame>& pF);

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  virtual void setToOriginImpl() {}

  /**
   * @brief 速度oplus，直接向量加法
   * @param update_ 3维速度扰动
   */
  virtual void oplusImpl(const double* update_) {
    Eigen::Vector3d uv;
    uv << update_[0], update_[1], update_[2];
    setEstimate(estimate() + uv);
  }
};

/**
 * @brief g2o顶点：陀螺仪偏置bg，维度3
 */
class VertexGyroBias : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexGyroBias() {}
  explicit VertexGyroBias(const std::shared_ptr<KeyFrame>& pKF);
  explicit VertexGyroBias(const std::shared_ptr<Frame>& pF);

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  virtual void setToOriginImpl() {}

  /**
   * @brief 陀螺偏置oplus，向量加法
   * @param update_ 3维bg扰动
   */
  virtual void oplusImpl(const double* update_) {
    Eigen::Vector3d ubg;
    ubg << update_[0], update_[1], update_[2];
    setEstimate(estimate() + ubg);
  }
};

/**
 * @brief g2o顶点：加速度计偏置ba，维度3
 */
class VertexAccBias : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexAccBias() {}
  explicit VertexAccBias(const std::shared_ptr<KeyFrame>& pKF);
  explicit VertexAccBias(const std::shared_ptr<Frame>& pF);

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  virtual void setToOriginImpl() {}

  /**
   * @brief 加速度偏置oplus，向量加法
   * @param update_ 3维ba扰动
   */
  virtual void oplusImpl(const double* update_) {
    Eigen::Vector3d uba;
    uba << update_[0], update_[1], update_[2];
    setEstimate(estimate() + uba);
  }
};

// Gravity direction vertex
/**
 * @brief 重力方向结构体，待优化；仅优化绕水平两轴旋转，z轴为重力初始方向
 */
class GDirection {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GDirection() {}
  explicit GDirection(Eigen::Matrix3d pRwg) : Rwg(pRwg) {}

  /**
   * @brief 重力方向扰动更新，仅xy轴旋转，z=0，ExpSO3右乘更新Rwg
   * @param pu 扰动数组，pu[0] pu[1]为两轴扰动
   */
  void Update(const double* pu) { Rwg = Rwg * ExpSO3(pu[0], pu[1], 0.0); }

  Eigen::Matrix3d Rwg, Rgw; /// Rwg世界‑重力系；Rgw重力‑世界
  int its;                  /// 迭代计数
};

/**
 * @brief g2o顶点：重力方向顶点，维度2，存储GDirection
 */
class VertexGDir : public g2o::BaseVertex<2, GDirection> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexGDir() {}
  explicit VertexGDir(Eigen::Matrix3d pRwg) { setEstimate(GDirection(pRwg)); }

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  virtual void setToOriginImpl() {}

  /**
   * @brief 重力方向oplus，调用GDirection::Update
   * @param update_ 2维扰动
   */
  virtual void oplusImpl(const double* update_) {
    _estimate.Update(update_);
    updateCache();
  }
};

// scale vertex
/**
 * @brief g2o顶点：尺度顶点，维度1；对单目IMU，优化全局尺度，扰动使用指数更新
 */
class VertexScale : public g2o::BaseVertex<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexScale() { setEstimate(1.0); }
  explicit VertexScale(double ps) { setEstimate(ps); }

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  virtual void setToOriginImpl() { setEstimate(1.0); }

  /**
   * @brief 尺度oplus，指数更新 s = s * exp(delta)
   * @param update_ 1维尺度扰动delta
   */
  virtual void oplusImpl(const double* update_) {
    setEstimate(estimate() * exp(*update_));
  }
};

// Inverse depth point (just one parameter, inverse depth at the host frame)
/**
 * @brief g2o顶点：逆深度点顶点，维度1，存储InvDepthPoint，仅优化ρ逆深度
 */
class VertexInvDepth : public g2o::BaseVertex<1, InvDepthPoint> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexInvDepth() {}
  explicit VertexInvDepth(double invDepth, double u, double v,
                          const std::shared_ptr<KeyFrame>& pHostKF) {
    setEstimate(InvDepthPoint(invDepth, u, v, pHostKF));
  }

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  virtual void setToOriginImpl() {}

  /**
   * @brief 逆深度oplus，调用InvDepthPoint::Update
   * @param update_ 1维rho扰动
   */
  virtual void oplusImpl(const double* update_) {
    _estimate.Update(update_);
    updateCache();
  }
};

/**
 * @brief g2o二元边：单目重投影误差边；连接VertexPointXYZ(3D点) + VertexPose(IMU‑相机位姿)；残差2维[u‑u_obs, v‑v_obs]
 */
class EdgeMono : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,
                                             g2o::VertexPointXYZ, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EdgeMono(int cam_idx_ = 0) : cam_idx(cam_idx_) {}

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  /**
   * @brief 计算单目重投影误差
   */
  void computeError() {
    const g2o::VertexPointXYZ* VPoint =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const Eigen::Vector2d obs(_measurement);
    _error = obs - VPose->estimate().Project(VPoint->estimate(), cam_idx);
  }

  virtual void linearizeOplus(); /// 计算雅可比矩阵

  /**
   * @brief 判断该观测点相机坐标系深度是否>0
   * @return true深度为正
   */
  bool isDepthPositive() {
    const g2o::VertexPointXYZ* VPoint =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    return VPose->estimate().isDepthPositive(VPoint->estimate(), cam_idx);
  }

  /**
   * @brief 获取拼接后的雅可比矩阵J(2×9)，点3维+位姿6维
   * @return Eigen::Matrix<double,2,9> J矩阵
   */
  Eigen::Matrix<double, 2, 9> GetJacobian() {
    linearizeOplus();
    Eigen::Matrix<double, 2, 9> J;
    J.block<2, 3>(0, 0) = _jacobianOplusXi;
    J.block<2, 6>(0, 3) = _jacobianOplusXj;
    return J;
  }

  /**
   * @brief 计算该边对应的海森矩阵 H = JᵀΩJ
   * @return Eigen::Matrix<double,9,9> H
   */
  Eigen::Matrix<double, 9, 9> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 2, 9> J;
    J.block<2, 3>(0, 0) = _jacobianOplusXi;
    J.block<2, 6>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }
public:
  const int cam_idx; /// 使用哪个相机，多相机系统
};

/**
 * @brief g2o一元边：仅优化位姿的单目重投影边；3D点固定，只优化VertexPose；残差2维
 */
class EdgeMonoOnlyPose
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EdgeMonoOnlyPose(const Eigen::Vector3f& Xw_, int cam_idx_ = 0)
      : Xw(Xw_.cast<double>()), cam_idx(cam_idx_) {}

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  /**
   * @brief 计算单目重投影误差，3D点Xw固定
   */
  void computeError() {
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector2d obs(_measurement);
    _error = obs - VPose->estimate().Project(Xw, cam_idx);
  }

  virtual void linearizeOplus();

  /**
   * @brief 判断固定3D点深度是否为正
   * @return true深度>0
   */
  bool isDepthPositive() {
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
    return VPose->estimate().isDepthPositive(Xw, cam_idx);
  }

  /**
   * @brief 获取位姿6×6海森矩阵
   * @return Eigen::Matrix<double,6,6> H
   */
  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }
public:
  const Eigen::Vector3d Xw; /// 固定世界三维点
  const int cam_idx;       /// 相机索引
};

/**
 * @brief g2o二元边：双目重投影误差边；连接VertexPointXYZ + VertexPose；残差3维(u,v,ur)
 */
class EdgeStereo : public g2o::BaseBinaryEdge<3, Eigen::Vector3d,
                                               g2o::VertexPointXYZ, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EdgeStereo(int cam_idx_ = 0) : cam_idx(cam_idx_) {}

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  /**
   * @brief 计算双目重投影误差，残差(u‑u_obs, v‑v_obs, ur‑ur_obs)
   */
  void computeError() {
    const g2o::VertexPointXYZ* VPoint =
        static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const Eigen::Vector3d obs(_measurement);
    _error = obs - VPose->estimate().ProjectStereo(VPoint->estimate(), cam_idx);
  }

  virtual void linearizeOplus();

  /**
   * @brief 获取拼接雅可比3×9，点3维，位姿6维
   * @return Eigen::Matrix<double,3,9> J
   */
  Eigen::Matrix<double, 3, 9> GetJacobian() {
    linearizeOplus();
    Eigen::Matrix<double, 3, 9> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 6>(0, 3) = _jacobianOplusXj;
    return J;
  }

  /**
   * @brief 获取海森矩阵9×9
   * @return Eigen::Matrix<double,9,9> H
   */
  Eigen::Matrix<double, 9, 9> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 3, 9> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 6>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }
public:
  const int cam_idx; /// 相机索引
};

/**
 * @brief g2o一元边：仅优化位姿双目重投影边；3D点固定，残差3维
 */
class EdgeStereoOnlyPose
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EdgeStereoOnlyPose(const Eigen::Vector3f& Xw_, int cam_idx_ = 0)
      : Xw(Xw_.cast<double>()), cam_idx(cam_idx_) {}

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  /**
   * @brief 双目重投影误差计算，Xw固定
   */
  void computeError() {
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d obs(_measurement);
    _error = obs - VPose->estimate().ProjectStereo(Xw, cam_idx);
  }

  virtual void linearizeOplus();

  /**
   * @brief 获取位姿6×6海森矩阵
   * @return Eigen::Matrix<double,6,6> H
   */
  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }
public:
  const Eigen::Vector3d Xw;  // 3D point coordinates 固定世界三维点
  const int cam_idx;         // 相机索引
};

/**
 * @brief g2o多边：IMU预积分残差边；9维残差[dR,dv,dp]；连接两个VertexPose、两个VertexVelocity、VertexGyroBias、VertexAccBias，共6个顶点
 */
class EdgeInertial : public g2o::BaseMultiEdge<9, Vector9d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EdgeInertial(const std::shared_ptr<IMU::Preintegrated>& pInt);

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  void computeError();                 /// IMU残差计算
  virtual void linearizeOplus();       /// IMU残差雅可比求解

  /**
   * @brief 完整24维海森矩阵，两个位姿(6+6)+两速度(3+3)+bg(3)+ba(3)
   * @return Eigen::Matrix<double,24,24> H
   */
  Eigen::Matrix<double, 24, 24> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 24> J;
    J.block<9, 6>(0, 0) = _jacobianOplus[0];
    J.block<9, 3>(0, 6) = _jacobianOplus[1];
    J.block<9, 3>(0, 9) = _jacobianOplus[2];
    J.block<9, 3>(0, 12) = _jacobianOplus[3];
    J.block<9, 6>(0, 15) = _jacobianOplus[4];
    J.block<9, 3>(0, 21) = _jacobianOplus[5];
    return J.transpose() * information() * J;
  }

  /**
   * @brief 去掉第一个位姿，剩余18维变量的海森
   * @return Eigen::Matrix<double,18,18> H
   */
  Eigen::Matrix<double, 18, 18> GetHessianNoPose1() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 18> J;
    J.block<9, 3>(0, 0) = _jacobianOplus[1];
    J.block<9, 3>(0, 3) = _jacobianOplus[2];
    J.block<9, 3>(0, 6) = _jacobianOplus[3];
    J.block<9, 6>(0, 9) = _jacobianOplus[4];
    J.block<9, 3>(0, 15) = _jacobianOplus[5];
    return J.transpose() * information() * J;
  }

  /**
   * @brief 只取第二个位姿+加速度偏置，9维海森
   * @return Eigen::Matrix<double,9,9> H
   */
  Eigen::Matrix<double, 9, 9> GetHessian2() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 9> J;
    J.block<9, 6>(0, 0) = _jacobianOplus[4];
    J.block<9, 3>(0, 6) = _jacobianOplus[5];
    return J.transpose() * information() * J;
  }

  const Eigen::Matrix3d JRg, JVg, JPg; /// 预积分存储雅可比：旋转、速度、位置对bg
  const Eigen::Matrix3d JVa, JPa;      /// 预积分存储雅可比：速度、位置对ba
  std::shared_ptr<IMU::Preintegrated> mpInt; /// IMU预积分对象
  const double dt;                         /// 预积分总时间
  Eigen::Vector3d g;                       /// 重力向量(0,0,-9.81)
};

// Edge inertial whre gravity is included as optimizable variable and it is not
// supposed to be pointing in -z axis, as well as scale
/**
 * @brief g2o多边：IMU残差边GS版本；重力方向、尺度作为待优化变量；9维残差，共8个顶点
 */
class EdgeInertialGS : public g2o::BaseMultiEdge<9, Vector9d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EdgeInertialGS(const std::shared_ptr<IMU::Preintegrated>& pInt);

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  void computeError();
  virtual void linearizeOplus();

  const Eigen::Matrix3d JRg, JVg, JPg;
  const Eigen::Matrix3d JVa, JPa;
  std::shared_ptr<IMU::Preintegrated> mpInt;
  const double dt;
  Eigen::Vector3d g, gI;  /// g世界重力；gI初始重力

  /**
   * @brief 全部变量27维海森矩阵，含重力方向2维、尺度1维
   * @return Eigen::Matrix<double,27,27> H
   */
  Eigen::Matrix<double, 27, 27> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 27> J;
    J.block<9, 6>(0, 0) = _jacobianOplus[0];
    J.block<9, 3>(0, 6) = _jacobianOplus[1];
    J.block<9, 3>(0, 9) = _jacobianOplus[2];
    J.block<9, 3>(0, 12) = _jacobianOplus[3];
    J.block<9, 6>(0, 15) = _jacobianOplus[4];
    J.block<9, 3>(0, 21) = _jacobianOplus[5];
    J.block<9, 2>(0, 24) = _jacobianOplus[6];
    J.block<9, 1>(0, 26) = _jacobianOplus[7];
    return J.transpose() * information() * J;
  }

  Eigen::Matrix<double, 27, 27> GetHessian2() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 27> J;
    J.block<9, 3>(0, 0) = _jacobianOplus[2];
    J.block<9, 3>(0, 3) = _jacobianOplus[3];
    J.block<9, 2>(0, 6) = _jacobianOplus[6];
    J.block<9, 1>(0, 8) = _jacobianOplus[7];
    J.block<9, 3>(0, 9) = _jacobianOplus[1];
    J.block<9, 3>(0, 12) = _jacobianOplus[5];
    J.block<9, 6>(0, 15) = _jacobianOplus[0];
    J.block<9, 6>(0, 21) = _jacobianOplus[4];
    return J.transpose() * information() * J;
  }

  Eigen::Matrix<double, 9, 9> GetHessian3() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 9> J;
    J.block<9, 3>(0, 0) = _jacobianOplus[2];
    J.block<9, 3>(0, 3) = _jacobianOplus[3];
    J.block<9, 2>(0, 6) = _jacobianOplus[6];
    J.block<9, 1>(0, 8) = _jacobianOplus[7];
    return J.transpose() * information() * J;
  }

  /**
   * @brief 仅尺度维度1×1海森
   * @return Eigen::Matrix<double,1,1> H
   */
  Eigen::Matrix<double, 1, 1> GetHessianScale() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 1> J = _jacobianOplus[7];
    return J.transpose() * information() * J;
  }

  /**
   * @brief 仅陀螺偏置bg的3×3海森
   * @return Eigen::Matrix<double,3,3> H
   */
  Eigen::Matrix<double, 3, 3> GetHessianBiasGyro() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 3> J = _jacobianOplus[2];
    return J.transpose() * information() * J;
  }

  /**
   * @brief 仅加速度偏置ba的3×3海森
   * @return Eigen::Matrix<double,3,3> H
   */
  Eigen::Matrix<double, 3, 3> GetHessianBiasAcc() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 3> J = _jacobianOplus[3];
    return J.transpose() * information() * J;
  }

  /**
   * @brief 仅重力方向2×2海森
   * @return Eigen::Matrix<double,2,2> H
   */
  Eigen::Matrix<double, 2, 2> GetHessianGDir() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 2> J = _jacobianOplus[6];
    return J.transpose() * information() * J;
  }
};

/**
 * @brief g2o二元边：陀螺仪偏置随机游走；残差bg2‑bg1，3维
 */
class EdgeGyroRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d,
                                               VertexGyroBias, VertexGyroBias> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeGyroRW() {}

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  /**
   * @brief 陀螺偏置随机游走残差 e = bg2 − bg1
   */
  void computeError() {
    const VertexGyroBias* VG1 =
        static_cast<const VertexGyroBias*>(_vertices[0]);
    const VertexGyroBias* VG2 =
        static_cast<const VertexGyroBias*>(_vertices[1]);
    _error = VG2->estimate() - VG1->estimate();
  }

  /**
   * @brief 雅可比矩阵：d(e)/d(bg1)=‑I，d(e)/d(bg2)=I
   */
  virtual void linearizeOplus() {
    _jacobianOplusXi = -Eigen::Matrix3d::Identity();
    _jacobianOplusXj.setIdentity();
  }

  /**
   * @brief 获取6×6海森矩阵(bg1,bg2)
   * @return Eigen::Matrix<double,6,6> H
   */
  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 3, 6> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 3>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }

  /**
   * @brief 获取bg2单独的3×3海森
   * @return Eigen::Matrix3d H
   */
  Eigen::Matrix3d GetHessian2() {
    linearizeOplus();
    return _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
  }
};

/**
 * @brief g2o二元边：加速度计偏置随机游走；残差ba2‑ba1，3维
 */
class EdgeAccRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias,
                                              VertexAccBias> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeAccRW() {}

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  /**
   * @brief 加速度偏置随机游走残差 e = ba2 − ba1
   */
  void computeError() {
    const VertexAccBias* VA1 = static_cast<const VertexAccBias*>(_vertices[0]);
    const VertexAccBias* VA2 = static_cast<const VertexAccBias*>(_vertices[1]);
    _error = VA2->estimate() - VA1->estimate();
  }

  /**
   * @brief 雅可比矩阵 d(e)/d(ba1)=‑I，d(e)/d(ba2)=I
   */
  virtual void linearizeOplus() {
    _jacobianOplusXi = -Eigen::Matrix3d::Identity();
    _jacobianOplusXj.setIdentity();
  }

  /**
   * @brief 获取6×6海森矩阵(ba1,ba2)
   * @return Eigen::Matrix<double,6,6> H
   */
  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 3, 6> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 3>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }

  /**
   * @brief 获取ba2单独3×3海森
   * @return Eigen::Matrix3d H
   */
  Eigen::Matrix3d GetHessian2() {
    linearizeOplus();
    return _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
  }
};

/**
 * @brief IMU‑位姿先验约束结构体；存储IMU状态初始值与15×15先验H矩阵，用于边缘化
 */
class ConstraintPoseImu {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 构造IMU位姿先验约束；对H做对称化、特征值裁剪，消除零特征值
   * @param Rwb_ IMU世界旋转
   * @param twb_ IMU世界平移
   * @param vwb_ IMU世界速度
   * @param bg_ 陀螺偏置
   * @param ba_ 加速度偏置
   * @param H_ 输入15×15海森矩阵
   */
  ConstraintPoseImu(const Eigen::Matrix3d& Rwb_, const Eigen::Vector3d& twb_,
                     const Eigen::Vector3d& vwb_, const Eigen::Vector3d& bg_,
                     const Eigen::Vector3d& ba_, const Matrix15d& H_)
      : Rwb(Rwb_), twb(twb_), vwb(vwb_), bg(bg_), ba(ba_), H(H_) {
    H = (H + H) / 2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
    Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
    for (int i = 0; i < 15; i++)
      if (eigs[i] < 1e-12) eigs[i] = 0;
    H = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
  }

  Eigen::Matrix3d Rwb;   /// IMU到世界旋转
  Eigen::Vector3d twb;   /// IMU世界平移
  Eigen::Vector3d vwb;   /// IMU世界速度
  Eigen::Vector3d bg;    /// 陀螺偏置先验值
  Eigen::Vector3d ba;    /// 加速度偏置先验值
  Matrix15d H;           /// 15×15先验海森矩阵
};

/**
 * @brief g2o多边：IMU位姿先验边；15维残差，来自边缘化ConstraintPoseImu
 */
class EdgePriorPoseImu : public g2o::BaseMultiEdge<15, Vector15d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit EdgePriorPoseImu(ConstraintPoseImu* c);

  virtual bool read(std::istream& is) { return false; }
  virtual bool write(std::ostream& os) const { return false; }

  void computeError();           /// 计算15维先验残差
  virtual void linearizeOplus(); /// 计算先验雅可比

  /**
   * @brief 完整15×15海森矩阵
   * @return Eigen::Matrix<double,15,15> H
   */
  Eigen::Matrix<double, 15, 15> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 15, 15> J;
    J.block<15, 6>(0, 0) = _jacobianOplus[0];
    J.block<15, 3>(0, 6) = _jacobianOplus[1];
    J.block<15, 3>(0, 9) = _jacobianOplus[2];
    J.block<15, 3>(0, 12) = _jacobianOplus[3];
    return J.transpose() * information() * J;
  }

  /**
   * @brief 去掉位姿，剩余9维(速度+bg+ba)海森矩阵
   * @return Eigen::Matrix<double,9,9> H
   */
  Eigen::Matrix<double, 9, 9> GetHessianNoPose() {
    linearizeOplus();
    Eigen::Matrix<double, 15, 9> J;
    J.block<15, 3>(0, 0) = _jacobianOplus[1];
    J.block<15, 3>(0, 3) = _jacobianOplus[2];
    J.block<15, 3>(0, 6) = _jacobianOplus[3];
    return J.transpose() * information() * J;
  }

  Eigen::Matrix3d Rwb;    /// IMU‑世界旋转，先验值
  Eigen::Vector3d twb, vwb; /// IMU平移、速度先验
  Eigen::Vector3d bg, ba;  /// bg、ba先验值
};

// Priors for biases
/**
 * @brief g2o一元边：加速度计偏置先验；残差bprior‑ba，3维
 */
// 加速度计bias先验边：一元边，残差维度3，观测值Vector3d，连接顶点VertexAccBias(加速度计偏置顶点)
class EdgePriorAcc     : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexAccBias> {
public:
  // Eigen内存对齐宏，当类包含Eigen固定大小矩阵成员时必须添加，防止内存崩溃
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 构造函数：传入float类型的加速度bias先验值，内部转为double存储
  explicit EdgePriorAcc(const Eigen::Vector3f& bprior_)
      : bprior(bprior_.cast<double>()) {}

  // g2o序列化读接口，本项目不做磁盘序列化，直接返回false
  virtual bool read(std::istream& is) { return false; }
  // g2o序列化写接口，本项目不做磁盘序列化，直接返回false
  virtual bool write(std::ostream& os) const { return false; }

  // 计算残差：g2o边核心虚函数，每次迭代求解器会调用该函数计算残差_error
  void computeError() {
    // 将边绑定的第0号顶点强制转换为加速度bias顶点类型
    const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[0]);
    // 残差 = bias先验值 - 顶点当前估计的bias值
    _error = bprior - VA->estimate();
  }

  // 解析雅可比求导函数声明，外部cpp文件实现，计算残差对顶点状态的雅可比矩阵
  virtual void linearizeOplus();

  // 获取该边对应的Hessian子块：H = J^T * Ω * J，Ω为信息矩阵(information)
  Eigen::Matrix<double, 3, 3> GetHessian() {
    // 先执行线性化，填充成员变量_jacobianOplusXi雅可比矩阵
    linearizeOplus();
    // J转置 * 信息矩阵 * J，得到该一元边贡献的Hessian矩阵块
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }

  // 加速度计bias的先验常量，构造时初始化，求解过程不可修改
  const Eigen::Vector3d bprior;
};

// 陀螺仪bias先验边：一元边，残差维度3，观测Vector3d，连接顶点VertexGyroBias(陀螺仪偏置顶点)
class EdgePriorGyro     : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexGyroBias> {
public:
  // Eigen内存对齐宏，类内存在Eigen固定尺寸矩阵成员必须添加
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 构造函数：传入float陀螺仪bias先验，cast转为double保存
  explicit EdgePriorGyro(const Eigen::Vector3f& bprior_)
      : bprior(bprior_.cast<double>()) {}

  // g2o读取序列化，本工程不使用，返回false
  virtual bool read(std::istream& is) { return false; }
  // g2o写入序列化，本工程不使用，返回false
  virtual bool write(std::ostream& os) const { return false; }

  // g2o核心虚函数，计算残差
  void computeError() {
    // 将绑定的0号顶点强转为陀螺仪bias顶点
    const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[0]);
    // 残差 = 陀螺仪bias先验 - 当前顶点估计的bias
    _error = bprior - VG->estimate();
  }

  // 解析雅可比线性化函数声明，实现在cpp文件
  virtual void linearizeOplus();

  // 计算该边对Hessian矩阵的贡献子块 H = J^T * Ω * J
  Eigen::Matrix<double, 3, 3> GetHessian() {
    // 调用线性化，填充_jacobianOplusXi雅可比
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }

  // 陀螺仪bias先验常量，构造初始化，求解阶段只读
  const Eigen::Vector3d bprior;
};

// 4DoF位姿二元边：二元边，残差维度6，观测Vector6d，连接两个4自由度位姿顶点VertexPose4DoF
class Edge4DoF     : public g2o::BaseBinaryEdge<6, Vector6d, VertexPose4DoF, VertexPose4DoF> {
public:
  // Eigen内存对齐宏，类包含Eigen固定大小矩阵成员，必须开启对齐new
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 构造函数：输入4x4的相对变换矩阵deltaT，拆分旋转dRij和平移dtij缓存
  explicit Edge4DoF(const Eigen::Matrix4d& deltaT) {
    dTij = deltaT;                     // 完整4*4相对变换矩阵 T_i^j
    dRij = deltaT.block<3, 3>(0, 0);   // 提取左上角3*3旋转子块，相对旋转R_i^j
    dtij = deltaT.block<3, 1>(0, 3);   // 提取第0~2行第3列，3*1平移向量t_i^j
  }

  // g2o序列化读接口，项目不使用，返回false
  virtual bool read(std::istream& is) { return false; }
  // g2o序列化写接口，项目不使用，返回false
  virtual bool write(std::ostream& os) const { return false; }

  // 残差计算函数声明，cpp实现，二元边残差计算
  void computeError();

  // virtual void linearizeOplus(); //  Use numerical implementation
  // 注释：不手写解析雅可比，使用g2o内置数值求导方式完成线性化

  Eigen::Matrix4d dTij; // 完整4×4相对位姿观测 Tij
  Eigen::Matrix3d dRij; // 相对旋转观测 Rij，从dTij预提取缓存
  Eigen::Vector3d dtij; // 相对平移观测 tij，从dTij预提取缓存
};
}


  // namespace ORB_SLAM3
