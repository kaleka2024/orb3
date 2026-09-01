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
// 头文件保护，防止头文件被重复包含
#pragma once

// assert断言，用于运行时参数合法性校验
#include <assert.h>
// boost序列化对std::shared_ptr的支持
#include <boost/serialization/shared_ptr.hpp>
// C++标准智能指针
#include <memory>
// STL动态数组容器
#include <vector>

// 相机几何抽象基类头文件
#include "GeometricCamera.h"
// 两视图重构模块，求解基础矩阵、位姿恢复、三角化
#include "TwoViewReconstruction.h"

namespace ORB_SLAM3 {

/**
 * @brief 针孔相机模型类，继承GeometricCamera抽象基类
 * 参数存储顺序：[fx, fy, cx, cy]，4参数针孔相机，无畸变系数
 * 实现投影、反投影、投影雅可比、两视图重构、极线约束等虚接口
 */
class Pinhole : public GeometricCamera {
  // Eigen内存对齐宏，保证Eigen相关对象内存对齐，避免运行时崩溃
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // boost序列化友元，允许序列化访问本类私有与保护成员
  friend class boost::serialization::access;

  /**
   * @brief boost序列化模板函数，完成本类存档/读档
   * @tparam Archive 序列化存档类型
   * @param ar 存档对象引用
   * @param version 存档版本号，用于版本兼容
   */
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    // 序列化父类GeometricCamera基类部分
    ar& boost::serialization::base_object<GeometricCamera>(*this);
  }

public:
  /**
   * @brief 默认构造函数
   */
  Pinhole();

  /**
   * @brief 带相机参数的显式构造函数
   * @param _vParameters 针孔相机4维参数 [fx, fy, cx, cy]
   */
  explicit Pinhole(const std::vector<float> _vParameters);

  /**
   * @brief 拷贝构造函数，从已有Pinhole对象复制相机参数
   * @param pinhole 源针孔相机对象
   */
  explicit Pinhole(const Pinhole& pinhole);

  /**
   * @brief 析构函数
   */
  ~Pinhole();

  /**
   * @brief 3D点投影到图像平面，cv::Point3f输入，输出cv::Point2f像素坐标
   * @param p3D 相机坐标系下三维点cv::Point3f
   * @return 图像像素点cv::Point2f
   */
  cv::Point2f project(const cv::Point3f& p3D);

  /**
   * @brief Eigen::Vector3d三维点投影，返回Eigen::Vector2d像素
   * @param v3D 相机坐标系double类型三维点
   * @return 图像二维像素向量double
   */
  Eigen::Vector2d project(const Eigen::Vector3d& v3D);

  /**
   * @brief Eigen::Vector3f三维点投影，返回Eigen::Vector2f像素
   * @param v3D 相机坐标系float类型三维点
   * @return 图像二维像素向量float
   */
  Eigen::Vector2f project(const Eigen::Vector3f& v3D);

  /**
   * @brief 投影接口，cv::Point3f输入，输出Eigen::Vector2f像素
   * @param p3D 相机坐标系三维点cv::Point3f
   * @return Eigen::Vector2f像素坐标
   */
  Eigen::Vector2f projectMat(const cv::Point3f& p3D);

  /**
   * @brief 计算图像特征点不确定性，用于BA优化残差权重
   * @param p2D 图像二维像素列向量
   * @return float 不确定性数值
   */
  float uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D);

  /**
   * @brief 图像像素反投影，得到归一化平面三维点Eigen::Vector3f(Z=1)
   * @param p2D 图像像素坐标cv::Point2f
   * @return 归一化平面三维点Eigen::Vector3f
   */
  Eigen::Vector3f unprojectEig(const cv::Point2f& p2D);

  /**
   * @brief 图像像素反投影，返回cv::Point3f格式归一化平面点(Z=1)
   * @param p2D 图像像素坐标cv::Point2f
   * @return cv::Point3f归一化三维点
   */
  cv::Point3f unproject(const cv::Point2f& p2D);

  /**
   * @brief 投影函数对三维点X,Y,Z的雅可比矩阵 d(u,v)/d(X,Y,Z)
   * @param v3D 相机坐标系三维点Eigen::Vector3d
   * @return 2×3雅可比矩阵double
   */
  Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D);

  /**
   * @brief 两视图重构，求解基础矩阵，恢复两帧之间位姿T21并三角化匹配点
   * @param vKeys1 第一帧关键点
   * @param vKeys2 第二帧关键点
   * @param vMatches12 匹配索引对
   * @param T21 [out] 第二帧相对于第一帧的位姿Sophus::SE3f
   * @param vP3D [out] 输出三角化三维点
   * @param vbTriangulated [out] 标记每个匹配点是否三角化成功
   * @return bool 两视图重构是否成功
   */
  bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1,
                                const std::vector<cv::KeyPoint>& vKeys2,
                                const std::vector<int>& vMatches12,
                                Sophus::SE3f& T21,
                                std::vector<cv::Point3f>& vP3D,
                                std::vector<bool>& vbTriangulated);

  /**
   * @brief 获取针孔相机内参K矩阵，返回OpenCV Mat
   * @return cv::Mat 3×3内参矩阵
   */
  cv::Mat toK();

  /**
   * @brief 获取Eigen格式内参K矩阵float
   * @return Eigen::Matrix3f 3×3内参矩阵
   */
  Eigen::Matrix3f toK_();

  /**
   * @brief 针孔相机极线约束校验，判断匹配点是否满足对极几何，筛选外点
   * @param pCamera2 另一台针孔相机shared_ptr
   * @param kp1 相机1关键点
   * @param kp2 相机2关键点
   * @param R12 1到2的旋转矩阵
   * @param t12 1到2的平移向量
   * @param sigmaLevel 关键点尺度对应的sigma
   * @param unc 不确定性阈值
   * @return bool true满足极线约束，false判定为外点
   */
  bool epipolarConstrain(const std::shared_ptr<GeometricCamera>& pCamera2,
                          const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                          const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12,
                          const float sigmaLevel, const float unc);

  /**
   * @brief 匹配校验并且三角化，针孔相机此处直接返回false，未实现该接口
   * @param kp1 帧1关键点
   * @param kp2 帧2关键点
   * @param pOther 另一相机对象shared_ptr
   * @param Tcw1 帧1世界到相机位姿
   * @param Tcw2 帧2世界到相机位姿
   * @param sigmaLevel1 关键点1尺度sigma
   * @param sigmaLevel2 关键点2尺度sigma
   * @param x3Dtriangulated [out] 输出三角化三维点
   * @return bool 固定返回false
   */
  bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                            const std::shared_ptr<GeometricCamera>& pOther,
                            Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                            const float sigmaLevel1, const float sigmaLevel2,
                            Eigen::Vector3f& x3Dtriangulated) {
    return false;
  }

  // 输出流重载，打印Pinhole相机参数
  friend std::ostream& operator<<(std::ostream& os, const Pinhole& ph);
  // 输入流重载，从流读取Pinhole相机参数
  friend std::istream& operator>>(std::istream& os, Pinhole& ph);

  /**
   * @brief 判断传入相机是否与本相机参数完全一致
   * @param pCam 待对比相机shared_ptr
   * @return true参数完全一致
   */
  bool IsEqual(const std::shared_ptr<GeometricCamera>& pCam);

private:
  // Parameters vector corresponds to
  //       [fx, fy, cx, cy]

  /// 两视图重构智能指针，用于求解基础矩阵、位姿恢复、三角化
  std::shared_ptr<TwoViewReconstruction> tvr;
};

}  // namespace ORB_SLAM3

// BOOST_CLASS_EXPORT_KEY(ORBSLAM2::Pinhole)
