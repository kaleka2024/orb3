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
// 针孔相机模型头文件
#include "Pinhole.h"
// boost序列化导出，用于相机对象序列化保存加载
#include <boost/serialization/export.hpp>
// 智能指针头文件
#include <memory>
// 动态数组容器
#include <vector>

// BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::Pinhole)

namespace ORB_SLAM3 {
// BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")

// 相机全局ID计数器，每新建一个GeometricCamera实例就自增
long unsigned int GeometricCamera::nNextId = 0;

/**
 * @brief Pinhole默认构造函数
 * mvParameters顺序：fx, fy, cx, cy，共4个参数
 */
Pinhole::Pinhole() : GeometricCamera(), tvr() {
  mvParameters.resize(4);      // 针孔相机参数固定4个：fx fy cx cy
  mnId = nNextId++;            // 分配唯一相机ID，全局计数器+1
  mnType = CAM_PINHOLE;        // 设置相机类型标记为针孔相机
}

/**
 * @brief 带参数向量的构造函数
 * @param _vParameters 输入相机参数vector，顺序fx fy cx cy
 */
Pinhole::Pinhole(const std::vector<float> _vParameters)
    : GeometricCamera(_vParameters), tvr() {
  assert(mvParameters.size() == 4);  // 强制校验参数数量必须等于4
  mnId = nNextId++;                  // 分配唯一相机ID
  mnType = CAM_PINHOLE;              // 设置相机类型
}

/**
 * @brief 拷贝构造函数
 * @param pinhole 源针孔相机对象
 */
Pinhole::Pinhole(const Pinhole &pinhole)
    : GeometricCamera(pinhole.mvParameters), tvr() {
  assert(mvParameters.size() == 4); // 校验参数个数
  mnId = nNextId++;                 // 新实例分配全新ID，不拷贝源ID
  mnType = CAM_PINHOLE;             // 设置相机类型
}

/**
 * @brief Pinhole析构函数，释放TwoViewReconstruction对象
 */
Pinhole::~Pinhole() {
  if (tvr) tvr.reset();
}

/**
 * @brief 针孔相机投影函数 cv::Point3f版本，相机坐标系3D点→像素坐标
 * @param p3D 相机坐标系三维点 cv::Point3f
 * @return cv::Point2f 输出像素u,v
 * @note 投影公式 u = fx * X/Z + cx ; v = fy * Y/Z + cy
 */
cv::Point2f Pinhole::project(const cv::Point3f &p3D) {
  return cv::Point2f(mvParameters[0] * p3D.x / p3D.z + mvParameters[2],
                     mvParameters[1] * p3D.y / p3D.z + mvParameters[3]);
}

/**
 * @brief 针孔投影 Eigen::Vector3d双精度版本
 * @param v3D 相机坐标系三维点 double
 * @return Eigen::Vector2d 像素坐标(u,v)
 */
Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &v3D) {
  return Eigen::Vector2d(mvParameters[0] * v3D[0] / v3D[2] + mvParameters[2],
                         mvParameters[1] * v3D[1] / v3D[2] + mvParameters[3]);
}

/**
 * @brief 针孔投影 Eigen::Vector3f单精度版本
 * @param v3D 相机坐标系三维点 float
 * @return Eigen::Vector2f 像素坐标(u,v)
 */
Eigen::Vector2f Pinhole::project(const Eigen::Vector3f &v3D) {
  return Eigen::Vector2f(mvParameters[0] * v3D[0] / v3D[2] + mvParameters[2],
                         mvParameters[1] * v3D[1] / v3D[2] + mvParameters[3]);
}

/**
 * @brief 投影封装接口，输入cv::Point3f输出Eigen::Vector2f
 * @param p3D 相机坐标系3D点
 * @return Eigen::Vector2f 像素坐标
 */
Eigen::Vector2f Pinhole::projectMat(const cv::Point3f &p3D) {
  cv::Point2f point = this->project(p3D);
  return Eigen::Vector2f(point.x, point.y);
}

/**
 * @brief 像素不确定性，图优化信息矩阵使用，针孔直接返回固定1.0
 * @param p2D 二维像素点
 * @return float 不确定性值
 */
float Pinhole::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) {
  return 1.0;
}

/**
 * @brief 反投影，像素点转为Eigen::Vector3f归一化相机射线 z=1
 * @param p2D 输入像素坐标
 * @return Eigen::Vector3f 归一化方向射线
 */
Eigen::Vector3f Pinhole::unprojectEig(const cv::Point2f &p2D) {
  return Eigen::Vector3f((p2D.x - mvParameters[2]) / mvParameters[0],
                         (p2D.y - mvParameters[3]) / mvParameters[1], 1.f);
}

/**
 * @brief 反投影 cv::Point2f版本，像素坐标求归一化射线 z=1
 * @param p2D 输入像素坐标
 * @return cv::Point3f 归一化方向射线
 */
cv::Point3f Pinhole::unproject(const cv::Point2f &p2D) {
  return cv::Point3f((p2D.x - mvParameters[2]) / mvParameters[0],
                     (p2D.y - mvParameters[3]) / mvParameters[1], 1.f);
}

/**
 * @brief 投影雅可比矩阵 d(u,v)/d(X,Y,Z)，2行3列
 * @param v3D 相机坐标系三维点 double
 * @return Eigen::Matrix<double,2,3> 雅可比矩阵
 */
Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &v3D) {
  Eigen::Matrix<double, 2, 3> Jac;
  // 填充雅可比：
  // du/dX, du/dY, du/dZ
  // dv/dX, dv/dY, dv/dZ
  Jac << mvParameters[0] / v3D[2], 0.f,
      -mvParameters[0] * v3D[0] / (v3D[2] * v3D[2]), 0.f,
      mvParameters[1] / v3D[2], -mvParameters[1] * v3D[1] / (v3D[2] * v3D[2]);

  return Jac;
}

/**
 * @brief 两视图重建，求解基础矩阵、位姿、三角化3D点
 * @param vKeys1 第一帧关键点
 * @param vKeys2 第二帧关键点
 * @param vMatches12 匹配对索引
 * @param T21 输出第二帧相对于第一帧位姿T21
 * @param vP3D 输出三角化得到3D点
 * @param vbTriangulated 标记哪些匹配成功三角化
 * @return bool 两视图重建是否成功
 */
bool Pinhole::ReconstructWithTwoViews(const std::vector<cv::KeyPoint> &vKeys1,
                                      const std::vector<cv::KeyPoint> &vKeys2,
                                      const std::vector<int> &vMatches12,
                                      Sophus::SE3f &T21,
                                      std::vector<cv::Point3f> &vP3D,
                                      std::vector<bool> &vbTriangulated) {
  // 如果两视图重建对象未初始化，构造对象并传入相机内参K
  if (!tvr) {
    const Eigen::Matrix3f K = this->toK_();
    tvr = std::make_shared<TwoViewReconstruction>(K);
  }
  // 调用TwoViewReconstruction执行两视图几何重建
  return tvr->Reconstruct(vKeys1, vKeys2, vMatches12, T21, vP3D,
                          vbTriangulated);
}

/**
 * @brief 获取OpenCV格式3阶针孔相机内参矩阵K
 * @return cv::Mat 3×3内参矩阵
 */
cv::Mat Pinhole::toK() {
  cv::Mat K = (cv::Mat_<float>(3, 3) << mvParameters[0], 0.f, mvParameters[2],
               0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
  return K;
}

/**
 * @brief 获取Eigen格式3阶针孔相机内参矩阵K
 * @return Eigen::Matrix3f 3×3内参矩阵
 */
Eigen::Matrix3f Pinhole::toK_() {
  Eigen::Matrix3f K;
  K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1],
      mvParameters[3], 0.f, 0.f, 1.f;
  return K;
}

/**
 * @brief 针孔相机极线约束校验，计算点到极线距离，判断匹配是否合法
 * @param pCamera2 第二个相机对象
 * @param kp1 帧1关键点
 * @param kp2 帧2关键点
 * @param R12 帧1到帧2旋转矩阵
 * @param t12 帧1到帧2平移向量
 * @param sigmaLevel 像素噪声sigma
 * @param unc 不确定性系数
 * @return bool true满足极线约束；false不满足
 */
bool Pinhole::epipolarConstrain(
    const std::shared_ptr<GeometricCamera> &pCamera2, const cv::KeyPoint &kp1,
    const cv::KeyPoint &kp2, const Eigen::Matrix3f &R12,
    const Eigen::Vector3f &t12, const float sigmaLevel, const float unc) {
  // Compute Fundamental Matrix 计算基础矩阵F12
  Eigen::Matrix3f t12x = Sophus::SO3f::hat(t12); // 平移向量反对称矩阵
  Eigen::Matrix3f K1 = this->toK_();              // 相机1内参
  Eigen::Matrix3f K2 = pCamera2->toK_();          // 相机2内参
  // F12 = K2^{-T} * [t12]_× * R12 * K2^{-1}
  Eigen::Matrix3f F12 = K1.transpose().inverse() * t12x * R12 * K2.inverse();

  // Epipolar line in second image l = x1'F12 = [a b c] 第二张图像上极线系数 a*x + b*y + c =0
  const float a = kp1.pt.x * F12(0, 0) + kp1.pt.y * F12(1, 0) + F12(2, 0);
  const float b = kp1.pt.x * F12(0, 1) + kp1.pt.y * F12(1, 1) + F12(2, 1);
  const float c = kp1.pt.x * F12(0, 2) + kp1.pt.y * F12(1, 2) + F12(2, 2);

  // 代入kp2像素，计算分子 a*u2 + b*v2 + c
  const float num = a * kp2.pt.x + b * kp2.pt.y + c;

  // 极线法向量模长平方 a²+b²
  const float den = a * a + b * b;

  // 分母为0，极线无效，直接返回false
  if (den == 0) return false;

  // 点到极线距离平方 d² = (a*u+b*v+c)^2/(a²+b²)
  const float dsqr = num * num / den;

  // 卡方检验自由度1，95%阈值3.84，距离平方小于阈值则通过极线约束
  return dsqr < 3.84 * unc;
}

/**
 * @brief 输出流重载，打印针孔相机4个参数 fx fy cx cy
 * @param os 输出流
 * @param ph Pinhole相机对象
 * @return std::ostream&
 */
std::ostream &operator<<(std::ostream &os, const Pinhole &ph) {
  os << ph.mvParameters[0] << " " << ph.mvParameters[1] << " "
     << ph.mvParameters[2] << " " << ph.mvParameters[3];
  return os;
}

/**
 * @brief 输入流重载，从流读取4个相机参数赋值给Pinhole对象
 * @param is 输入流
 * @param ph Pinhole相机对象
 * @return std::istream&
 */
std::istream &operator>>(std::istream &is, Pinhole &ph) {
  float nextParam;
  for (size_t i = 0; i < 4; i++) {
    assert(is.good());  // Make sure the input stream is good
    is >> nextParam;
    ph.mvParameters[i] = nextParam;
  }
  return is;
}

/**
 * @brief 判断两个相机对象参数是否相等
 * @param pCam 待比较相机智能指针
 * @return bool true参数一致；false不一致
 */
bool Pinhole::IsEqual(const std::shared_ptr<GeometricCamera> &pCam) {
  // 判断相机类型必须是针孔CAM_PINHOLE，否则直接false
  if (pCam->GetType() != GeometricCamera::CAM_PINHOLE) return false;

  // 基类指针向下转型为Pinhole引用
  Pinhole &PinholeCam = dynamic_cast<Pinhole &>(*pCam);

  // 参数数量不一致返回false
  if (size() != PinholeCam.size()) return false;

  bool is_same_camera = true;
  // 循环比对全部4个相机参数，容许1e‑6误差
  for (size_t i = 0; i < size(); ++i) {
    if (abs(mvParameters[i] - PinholeCam.getParameter(i)) > 1e-6) {
      is_same_camera = false;
      break;
    }
  }
  return is_same_camera;
}

}  // namespace ORB_SLAM3
