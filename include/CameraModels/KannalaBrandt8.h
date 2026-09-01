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
// 头文件保护，避免头文件重复包含
#pragma once

// assert断言，用于参数合法性校验
#include <assert.h>
// C++智能指针
#include <memory>
// STL动态数组容器
#include <vector>

// 相机几何抽象基类头文件
#include "GeometricCamera.h"
// 两视图重构模块头文件，用于基础矩阵求解、三角化
#include "TwoViewReconstruction.h"

namespace ORB_SLAM3 {

/**
 * @brief Kannala‑Brandt鱼眼相机模型，8参数模型，继承GeometricCamera抽象基类
 * 参数顺序：[fx, fy, cx, cy, k0, k1, k2, k3]
 * 实现鱼眼相机的投影、反投影、雅可比、极线约束、三角化、两视图重构全部接口
 */
class KannalaBrandt8 : public GeometricCamera {
  // Eigen内存对齐宏，保证Eigen成员内存对齐，防止运行时崩溃
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // boost序列化友元，允许序列化访问类私有成员
  friend class boost::serialization::access;

  /**
   * @brief boost序列化函数，存档/读档本类对象
   * @tparam Archive 序列化存档类型
   * @param ar 存档对象引用
   * @param version 存档版本号
   */
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    // 序列化父类GeometricCamera基类部分
    ar& boost::serialization::base_object<GeometricCamera>(*this);
    // 序列化精度常量precision，const成员需要const_cast去除const修饰
    ar& const_cast<float&>(precision);
  }

public:
  /**
   * @brief 默认构造函数
   * precision默认1e‑6，参数容器resize为8，分配唯一相机ID，标记相机类型为鱼眼
   */
  KannalaBrandt8() : precision(1e-6) {
    mvParameters.resize(8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }

  /**
   * @brief 带相机参数的显式构造函数
   * @param _vParameters 鱼眼相机8个参数vector [fx,fy,cx,cy,k0,k1,k2,k3]
   */
  explicit KannalaBrandt8(const std::vector<float> _vParameters)
      : GeometricCamera(_vParameters),
        precision(1e-6),
        mvLappingArea(2, 0),
        tvr(nullptr) {
    // 强制校验参数数量必须等于8
    assert(mvParameters.size() == 8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }

  /**
   * @brief 带自定义迭代求解精度的构造函数
   * @param _vParameters 8维鱼眼相机参数
   * @param _precision 反投影迭代求解精度阈值
   */
  KannalaBrandt8(const std::vector<float> _vParameters, const float _precision)
      : GeometricCamera(_vParameters),
        precision(_precision),
        mvLappingArea(2, 0) {
    assert(mvParameters.size() == 8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }

  /**
   * @brief 拷贝构造函数，从已有KannalaBrandt8对象复制参数与精度
   * @param pKannala 源鱼眼相机对象指针
   */
  explicit KannalaBrandt8(KannalaBrandt8* pKannala)
      : GeometricCamera(pKannala->mvParameters),
        precision(pKannala->precision),
        mvLappingArea(2, 0),
        tvr(nullptr) {
    assert(mvParameters.size() == 8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }

  /**
   * @brief 3D点投影到图像平面，OpenCV Point3f输入，输出cv::Point2f像素坐标
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
   * @brief 计算图像特征点不确定性，用于BA残差权重
   * @param p2D 图像二维像素列向量
   * @return float 不确定性数值
   */
  float uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D);

  /**
   * @brief 图像像素反投影，迭代求解得到归一化平面三维点Eigen::Vector3f(Z不为1)
   * @param p2D 图像像素坐标cv::Point2f
   * @return 归一化球坐标系三维方向向量
   */
  Eigen::Vector3f unprojectEig(const cv::Point2f& p2D);

  /**
   * @brief 图像像素反投影，返回cv::Point3f格式方向向量
   * @param p2D 图像像素坐标cv::Point2f
   * @return cv::Point3f方向向量
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
   * @brief 获取内参K矩阵，返回OpenCV Mat，鱼眼只返回fx,fy,cx,cy组成的3*3矩阵，不含畸变系数
   * @return cv::Mat 3×3内参矩阵
   */
  cv::Mat toK();

  /**
   * @brief 获取Eigen格式内参K矩阵float
   * @return Eigen::Matrix3f 3×3内参矩阵
   */
  Eigen::Matrix3f toK_();

  /**
   * @brief 鱼眼相机极线约束校验，判断匹配点是否满足对极几何，筛选外点
   * @param pCamera2 另一台鱼眼相机shared_ptr
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
   * @brief 对匹配关键点做三角化，返回重投影误差，用于校验匹配质量
   * @param pCamera2 另一相机shared_ptr
   * @param kp1 关键点1
   * @param kp2 关键点2
   * @param R12 旋转1→2
   * @param t12 平移1→2
   * @param sigmaLevel 尺度sigma
   * @param unc 不确定性阈值
   * @param p3D [out] 输出三角化得到三维点
   * @return float 三角化后的重投影误差
   */
  float TriangulateMatches(const std::shared_ptr<GeometricCamera>& pCamera2,
                            const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                            const Eigen::Matrix3f& R12,
                            const Eigen::Vector3f& t12, const float sigmaLevel,
                            const float unc, Eigen::Vector3f& p3D);

  /// 双目重叠区域，存储[x_min, x_max]，用于双目匹配过滤，大小固定为2
  std::vector<int> mvLappingArea;

  /**
   * @brief 匹配校验并且三角化，输入两帧关键点、两帧世界‑相机位姿，输出三角化3D点
   * @param kp1 帧1关键点
   * @param kp2 帧2关键点
   * @param pOther 另一相机对象shared_ptr
   * @param Tcw1 帧1世界到相机位姿
   * @param Tcw2 帧2世界到相机位姿
   * @param sigmaLevel1 关键点1尺度sigma
   * @param sigmaLevel2 关键点2尺度sigma
   * @param x3Dtriangulated [out] 输出三角化三维点
   * @return bool 三角化是否成功
   */
  bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                            const std::shared_ptr<GeometricCamera>& pOther,
                            Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                            const float sigmaLevel1, const float sigmaLevel2,
                            Eigen::Vector3f& x3Dtriangulated);

  // 输出流重载，打印KannalaBrandt8相机参数
  friend std::ostream& operator<<(std::ostream& os, const KannalaBrandt8& kb);
  // 输入流重载，从流读取相机参数
  friend std::istream& operator>>(std::istream& is, KannalaBrandt8& kb);

  /**
   * @brief 获取反投影迭代求解精度
   * @return float precision精度阈值
   */
  float GetPrecision() { return precision; }

  /**
   * @brief 判断传入相机是否与本相机参数完全一致
   * @param pCam 待对比相机shared_ptr
   * @return true参数完全一致
   */
  bool IsEqual(const std::shared_ptr<GeometricCamera>& pCam);

private:
  /// 反投影迭代求解收敛精度阈值，const常量不可修改
  const float precision;

  // Parameter vector corresponds to
  // [fx, fy, cx, cy, k0, k1, k2, k3]

  /// 两视图重构对象指针，用于求解基础矩阵、位姿恢复
  TwoViewReconstruction* tvr;

  /**
   * @brief 私有三角化函数，DLT直接线性变换三角化两个归一化平面点
   * @param p1 图像点1
   * @param p2 图像点2
   * @param Tcw1 相机1投影矩阵3×4
   * @param Tcw2 相机2投影矩阵3×4
   * @param x3D [out] 输出三角化三维点
   */
  void Triangulate(const cv::Point2f& p1, const cv::Point2f& p2,
                    const Eigen::Matrix<float, 3, 4>& Tcw1,
                    const Eigen::Matrix<float, 3, 4>& Tcw2,
                    Eigen::Vector3f& x3D);
};

}  // namespace ORB_SLAM3
