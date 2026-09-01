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
// 头文件保护，防止重复包含
#pragma once

// Eigen几何模块，矩阵、向量、位姿运算
#include <Eigen/Geometry>
// boost序列化访问权限，用于序列化类私有成员
#include <boost/serialization/access.hpp>
// boost序列化标记抽象基类
#include <boost/serialization/assume_abstract.hpp>
// boost序列化基类对象
#include <boost/serialization/base_object.hpp>
// boost序列化导出类，用于存档多态派生类
#include <boost/serialization/export.hpp>
// boost序列化核心接口
#include <boost/serialization/serialization.hpp>
// boost序列化std::shared_ptr智能指针
#include <boost/serialization/shared_ptr.hpp>
// boost序列化std::vector容器
#include <boost/serialization/vector.hpp>
// C++标准智能指针
#include <memory>
// OpenCV核心模块，Mat、Point基础数据结构
#include <opencv2/core/core.hpp>
// OpenCV特征点模块，KeyPoint
#include <opencv2/features2d/features2d.hpp>
// OpenCV图像处理模块
#include <opencv2/imgproc/imgproc.hpp>
// Sophus SE3李群库，SE(3)位姿变换
#include <sophus/se3.hpp>
// C++标准动态数组容器
#include <vector>

// ORB‑SLAM3内部类型转换工具头文件
#include "Converter.h"
// ORB‑SLAM3几何计算工具头文件
#include "GeometricTools.h"

namespace ORB_SLAM3 {

/**
 * @brief 相机几何抽象基类
 * 定义针孔、鱼眼相机统一接口；封装投影、反投影、雅可比、三角化、极线约束等虚函数
 * 支持boost序列化，支持多态派生（Pinhole / Fisheye相机）
 */
class GeometricCamera {
  // Eigen内存对齐宏，保证Eigen对象内存对齐，避免崩溃
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // boost序列化友元，允许序列化访问本类私有成员
  friend class boost::serialization::access;

  /**
   * @brief boost序列化模板函数，存档/读档本类成员变量
   * @tparam Archive 序列化存档类型
   * @param ar 存档对象引用
   * @param version 版本号，用于兼容不同存档版本
   */
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & mnId;         // 序列化相机ID
    ar & mnType;       // 序列化相机类型标记
    ar & mvParameters; // 序列化相机参数数组
  }

public:
  // 默认构造函数，编译器生成默认行为
  GeometricCamera() = default;

  /**
   * @brief 带参数列表的显式构造函数
   * @param _vParameters 相机参数vector，fx,fy,cx,cy等
   */
  explicit GeometricCamera(const std::vector<float>& _vParameters)
      : mvParameters(_vParameters) {}

  // 析构函数，虚函数在派生类实现，这里空实现
  ~GeometricCamera() {}

  /**
   * @brief 3D点投影到图像平面，输出OpenCV Point2f
   * @param p3D 相机坐标系下三维点cv::Point3f
   * @return 图像像素坐标cv::Point2f
   */
  virtual cv::Point2f project(const cv::Point3f& p3D) = 0;

  /**
   * @brief Eigen::Vector3d三维点投影，返回Eigen::Vector2d像素
   * @param v3D 相机坐标系三维点double
   * @return 图像二维像素向量double
   */
  virtual Eigen::Vector2d project(const Eigen::Vector3d& v3D) = 0;

  /**
   * @brief Eigen::Vector3f三维点投影，返回Eigen::Vector2f像素
   * @param v3D 相机坐标系三维点float
   * @return 图像二维像素向量float
   */
  virtual Eigen::Vector2f project(const Eigen::Vector3f& v3D) = 0;

  /**
   * @brief 投影接口，输入cv::Point3f，输出Eigen::Vector2f像素
   * @param p3D 相机坐标系三维点cv::Point3f
   * @return Eigen::Vector2f像素坐标
   */
  virtual Eigen::Vector2f projectMat(const cv::Point3f& p3D) = 0;

  /**
   * @brief 计算图像点的不确定性，返回2阶协方差相关数值
   * @param p2D 图像二维像素点
   * @return float 不确定性数值，用于BA残差权重
   */
  virtual float uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D) = 0;

  /**
   * @brief 图像像素反投影，得到归一化平面三维点Eigen::Vector3f（Z=1）
   * @param p2D 图像像素坐标cv::Point2f
   * @return 归一化平面三维点Eigen::Vector3f
   */
  virtual Eigen::Vector3f unprojectEig(const cv::Point2f& p2D) = 0;

  /**
   * @brief 图像像素反投影，返回cv::Point3f归一化平面点(Z=1)
   * @param p2D 图像像素坐标cv::Point2f
   * @return cv::Point3f归一化三维点
   */
  virtual cv::Point3f unproject(const cv::Point2f& p2D) = 0;

  /**
   * @brief 投影函数对三维点的雅可比矩阵 d(u,v)/d(X,Y,Z)
   * @param v3D 相机坐标系三维点Eigen::Vector3d
   * @return 2×3雅可比矩阵
   */
  virtual Eigen::Matrix<double, 2, 3> projectJac(
      const Eigen::Vector3d& v3D) = 0;

  /**
   * @brief 两视图重构：匹配关键点，求解T21并三角化3D点
   * @param vKeys1 第一帧关键点
   * @param vKeys2 第二帧关键点
   * @param vMatches12 匹配对索引
   * @param T21 [out] 第二帧相对于第一帧的位姿 Sophus::SE3f
   * @param vP3D [out] 输出三角化得到三维点
   * @param vbTriangulated [out] 标记每个匹配是否成功三角化
   * @return bool 重构是否成功
   */
  virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1,
                                      const std::vector<cv::KeyPoint>& vKeys2,
                                      const std::vector<int>& vMatches12,
                                      Sophus::SE3f& T21,
                                      std::vector<cv::Point3f>& vP3D,
                                      std::vector<bool>& vbTriangulated) = 0;

  /**
   * @brief 获取相机内参K矩阵，返回OpenCV Mat
   * @return cv::Mat 3×3内参矩阵
   */
  virtual cv::Mat toK() = 0;

  /**
   * @brief 获取相机内参K矩阵，返回Eigen Matrix3f
   * @return Eigen::Matrix3f 3×3内参矩阵float
   */
  virtual Eigen::Matrix3f toK_() = 0;

  /**
   * @brief 极线约束校验，判断一对匹配点是否满足对极几何约束
   * @param otherCamera 另一台相机实例shared_ptr
   * @param kp1 相机1关键点
   * @param kp2 相机2关键点
   * @param R12 旋转矩阵：从相机1到相机2
   * @param t12 平移向量：从相机1到相机2
   * @param sigmaLevel 关键点尺度对应的sigma
   * @param unc 不确定性阈值
   * @return bool true满足极线约束，false外点
   */
  virtual bool epipolarConstrain(
      const std::shared_ptr<GeometricCamera>& otherCamera,
      const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
      const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12,
      const float sigmaLevel, const float unc) = 0;

  /**
   * @brief 获取第i个相机参数
   * @param i 参数下标索引
   * @return float 参数值
   */
  float getParameter(const int i) { return mvParameters[i]; }

  /**
   * @brief 设置第i个相机参数
   * @param p 待赋值参数值
   * @param i 参数下标索引
   */
  void setParameter(const float p, const size_t i) { mvParameters[i] = p; }

  /**
   * @brief 获取相机参数总个数
   * @return size_t 参数数量
   */
  size_t size() { return mvParameters.size(); }

  /**
   * @brief 匹配点校验+三角化，输入两帧关键点与位姿，输出三角化3D点
   * @param kp1 帧1关键点
   * @param kp2 帧2关键点
   * @param pOther 另一相机对象shared_ptr
   * @param Tcw1 帧1相机位姿世界到相机
   * @param Tcw2 帧2相机位姿世界到相机
   * @param sigmaLevel1 关键点1尺度sigma
   * @param sigmaLevel2 关键点2尺度sigma
   * @param x3Dtriangulated [out] 三角化输出三维点
   * @return bool 三角化是否成功
   */
  virtual bool matchAndtriangulate(
      const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
      const std::shared_ptr<GeometricCamera>& pOther, Sophus::SE3f& Tcw1,
      Sophus::SE3f& Tcw2, const float sigmaLevel1, const float sigmaLevel2,
      Eigen::Vector3f& x3Dtriangulated) = 0;

  /**
   * @brief 获取相机ID
   * @return unsigned int mnId
   */
  unsigned int GetId() { return mnId; }

  /**
   * @brief 获取相机类型标记
   * @return unsigned int mnType
   */
  unsigned int GetType() { return mnType; }

  // 相机类型常量：针孔相机
  static const unsigned int CAM_PINHOLE = 0;
  // 相机类型常量：鱼眼相机
  static const unsigned int CAM_FISHEYE = 1;

  // 全局相机ID计数器，生成下一个相机唯一ID
  static long unsigned int nNextId;

protected:
  /// 相机参数数组：针孔[fx,fy,cx,cy]；鱼眼[k1,k2,k3,k4,fx,fy,cx,cy]
  std::vector<float> mvParameters;

  /// 相机实例唯一ID
  unsigned int mnId;

  /// 相机类型，CAM_PINHOLE / CAM_FISHEYE
  unsigned int mnType;
};

}  // namespace ORB_SLAM3
