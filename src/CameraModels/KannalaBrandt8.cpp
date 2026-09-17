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
// Kannala‑Brandt8鱼眼相机模型头文件，8阶畸变模型
#include "KannalaBrandt8.h"
// boost序列化导出，用于对象序列化保存加载
#include <boost/serialization/export.hpp>
// 智能指针头文件
#include <memory>
// 动态数组容器
#include <vector>

// BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::KannalaBrandt8)

namespace ORB_SLAM3 {
// BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

/**
 * @brief 鱼眼相机投影函数，cv::Point3f版本，相机坐标系3D点投影到像素平面
 * @param p3D 相机坐标系下三维点，cv::Point3f格式
 * @return cv::Point2f 输出像素坐标(u,v)
 * @note mvParameters[0]:fx mvParameters[1]:fy mvParameters[2]:cx mvParameters[3]:cy
 * mvParameters[4~7]:k0 k1 k2 k3 鱼眼4个畸变系数
 */
cv::Point2f KannalaBrandt8::project(const cv::Point3f &p3D) {
  // 计算x²+y²，相机平面径向距离平方
  const float x2_plus_y2 = p3D.x * p3D.x + p3D.y * p3D.y;
  // theta:入射光线与相机光轴(z轴)夹角，极角
  const float theta = atan2f(sqrtf(x2_plus_y2), p3D.z);
  // psi:方位角，xy平面上与x轴夹角
  const float psi = atan2f(p3D.y, p3D.x);

  // 计算theta各次幂，用于Kannala‑Brandt畸变多项式
  const float theta2 = theta * theta;
  const float theta3 = theta * theta2;
  const float theta5 = theta3 * theta2;
  const float theta7 = theta5 * theta2;
  const float theta9 = theta7 * theta2;
  // r:畸变后的图像平面径向距离 r = θ + k0θ³ +k1θ⁵ +k2θ⁷ +k3θ⁹
  const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5 +
                  mvParameters[6] * theta7 + mvParameters[7] * theta9;

  // 像素坐标：u = fx*r*cosψ + cx ; v = fy*r*sinψ + cy
  return cv::Point2f(mvParameters[0] * r * cos(psi) + mvParameters[2],
                     mvParameters[1] * r * sin(psi) + mvParameters[3]);
}

/**
 * @brief 鱼眼投影函数，Eigen::Vector3d双精度版本
 * @param v3D 相机坐标系三维点，double精度
 * @return Eigen::Vector2d 像素坐标(u,v)
 */
Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &v3D) {
  // x²+y²，相机平面径向平方
  const double x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
  // theta 光线与光轴夹角
  const double theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
  // psi 方位角
  const double psi = atan2f(v3D[1], v3D[0]);

  // theta各阶幂次
  const double theta2 = theta * theta;
  const double theta3 = theta * theta2;
  const double theta5 = theta3 * theta2;
  const double theta7 = theta5 * theta2;
  const double theta9 = theta7 * theta2;
  // 畸变后径向距离r
  const double r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5 +
                   mvParameters[6] * theta7 + mvParameters[7] * theta9;

  Eigen::Vector2d res;
  res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
  res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

  return res;
}

/**
 * @brief 鱼眼投影函数，Eigen::Vector3f单精度版本
 * @param v3D 相机坐标系三维点float
 * @return Eigen::Vector2f 像素坐标
 */
Eigen::Vector2f KannalaBrandt8::project(const Eigen::Vector3f &v3D) {
  const float x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
  const float theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
  const float psi = atan2f(v3D[1], v3D[0]);

  const float theta2 = theta * theta;
  const float theta3 = theta * theta2;
  const float theta5 = theta3 * theta2;
  const float theta7 = theta5 * theta2;
  const float theta9 = theta7 * theta2;
  const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5 +
                  mvParameters[6] * theta7 + mvParameters[7] * theta9;

  Eigen::Vector2f res;
  res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
  res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

  return res;

  /*cv::Point2f cvres = this->project(cv::Point3f(v3D[0],v3D[1],v3D[2]));
  Eigen::Vector2d res;
  res[0] = cvres.x;
  res[1] = cvres.y;
  return res;*/
}

/**
 * @brief 投影封装接口：输入cv::Point3f，输出Eigen::Vector2f
 * @param p3D 相机坐标系3D点
 * @return Eigen::Vector2f 像素坐标
 */
Eigen::Vector2f KannalaBrandt8::projectMat(const cv::Point3f &p3D) {
  cv::Point2f point = this->project(p3D);
  return Eigen::Vector2f(point.x, point.y);
}

/**
 * @brief 像素点不确定性，用于图优化信息矩阵，当前直接返回固定1.f
 * @param p2D 二维像素点
 * @return float 不确定性值
 */
float KannalaBrandt8::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) {
  /*Eigen::Matrix<double,2,1> c;
  c << mvParameters[2], mvParameters[3];
  if ((p2D-c).squaredNorm()>57600) // 240*240 (256)
      return 100.f;
  else
      return 1.0f;*/
  return 1.f;
}

/**
 * @brief 反投影封装，cv::Point2f像素→Eigen::Vector3f相机归一化射线
 * @param p2D 输入像素坐标
 * @return Eigen::Vector3f 相机坐标系归一化射线方向向量
 */
Eigen::Vector3f KannalaBrandt8::unprojectEig(const cv::Point2f &p2D) {
  cv::Point3f ray = this->unproject(p2D);
  return Eigen::Vector3f(ray.x, ray.y, ray.z);
}

/**
 * @brief 鱼眼相机反投影，像素坐标求解相机归一化射线，使用牛顿迭代求解θ
 * @param p2D 输入像素坐标(u,v)
 * @return cv::Point3f 相机坐标系归一化方向射线，z=1
 */
cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &p2D) {
  // Use Newthon method to solve for theta with good precision (err ~ e-6)
  // pw：归一化平面坐标，扣除主点，除以fx fy
  cv::Point2f pw((p2D.x - mvParameters[2]) / mvParameters[0],
                 (p2D.y - mvParameters[3]) / mvParameters[1]);
  float scale = 1.f;
  // theta_d：畸变后的径向距离，作为牛顿迭代初值
  float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
  // 限制theta_d范围 [-π/2, π/2]
  theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

  // 径向距离大于极小值，执行牛顿迭代求解真实入射角theta
  if (theta_d > 1e-8) {
    // Compensate distortion iteratively
    // 迭代初值取theta_d
    float theta = theta_d;

    // 最多迭代10次牛顿法
    for (int j = 0; j < 10; j++) {
      // 计算theta各阶次幂
      float theta2 = theta * theta, theta4 = theta2 * theta2,
            theta6 = theta4 * theta2, theta8 = theta4 * theta4;
      // k0*θ²，k1*θ⁴，k2*θ⁶，k3*θ⁸
      float k0_theta2 = mvParameters[4] * theta2,
            k1_theta4 = mvParameters[5] * theta4;
      float k2_theta6 = mvParameters[6] * theta6,
            k3_theta8 = mvParameters[7] * theta8;
      // 牛顿迭代修正量，f(theta)/f'(theta)
      float theta_fix =
          (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) -
           theta_d) /
          (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
      // 更新theta
      theta = theta - theta_fix;
      // 修正量小于精度阈值，提前退出迭代
      if (fabsf(theta_fix) < precision) break;
    }
    // scale = theta - theta_d;
    // scale = tan(theta)/theta_d；恢复归一化射线xy分量
    scale = std::tan(theta) / theta_d;
  }

  // 返回归一化射线，z固定为1
  return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
}

/**
 * @brief 投影雅可比矩阵，d(u,v)/d(X,Y,Z)，2行3列
 * @param v3D 相机坐标系三维点double
 * @return Eigen::Matrix<double,2,3> 雅可比矩阵
 */
Eigen::Matrix<double, 2, 3> KannalaBrandt8::projectJac(
    const Eigen::Vector3d &v3D) {
  // x² y² z²
  double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
  // r² = x²+y²
  double r2 = x2 + y2;
  // r = sqrt(x²+y²)
  double r = sqrt(r2);
  // r³ = r²*r
  double r3 = r2 * r;
  // theta 光线与光轴夹角
  double theta = atan2(r, v3D[2]);

  // theta各阶幂次
  double theta2 = theta * theta, theta3 = theta2 * theta;
  double theta4 = theta2 * theta2, theta5 = theta4 * theta;
  double theta6 = theta2 * theta4, theta7 = theta6 * theta;
  double theta8 = theta4 * theta4, theta9 = theta8 * theta;

  // f(theta)=θ +k0θ³+k1θ⁵+k2θ⁷+k3θ⁹，畸变径向函数
  double f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] +
             theta7 * mvParameters[6] + theta9 * mvParameters[7];
  // f'(theta)导数 df/dθ
  double fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 +
              7 * mvParameters[6] * theta6 + 9 * mvParameters[7] * theta8;

  Eigen::Matrix<double, 2, 3> JacGood;
  // du / dX
  JacGood(0, 0) =
      mvParameters[0] * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
  // dv / dX
  JacGood(1, 0) =
      mvParameters[1] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) -
                         f * v3D[1] * v3D[0] / r3);

  // du / dY
  JacGood(0, 1) =
      mvParameters[0] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) -
                         f * v3D[1] * v3D[0] / r3);
  // dv / dY
  JacGood(1, 1) =
      mvParameters[1] * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

  // du / dZ
  JacGood(0, 2) = -mvParameters[0] * fd * v3D[0] / (r2 + z2);
  // dv / dZ
  JacGood(1, 2) = -mvParameters[1] * fd * v3D[1] / (r2 + z2);

  return JacGood;
}

/**
 * @brief 两视图重建，鱼眼相机，对关键点去畸变后调用TwoViewReconstruction求解基础矩阵、三角化
 * @param vKeys1 第一帧关键点
 * @param vKeys2 第二帧关键点
 * @param vMatches12 匹配对索引
 * @param T21 输出第二帧相对于第一帧位姿T21
 * @param vP3D 输出三角化得到3D点
 * @param vbTriangulated 标记哪些匹配成功三角化
 * @return bool 两视图重建是否成功
 */
bool KannalaBrandt8::ReconstructWithTwoViews(
    const std::vector<cv::KeyPoint> &vKeys1,
    const std::vector<cv::KeyPoint> &vKeys2, const std::vector<int> &vMatches12,
    Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D,
    std::vector<bool> &vbTriangulated) {
  // 如果两视图重建对象未初始化，构造对象，传入相机内参K
  if (!tvr) {
    Eigen::Matrix3f K = this->toK_();
    tvr = new TwoViewReconstruction(K);
  }

  // Correct FishEye distortion
  // 拷贝关键点，用于存储去畸变之后关键点
  std::vector<cv::KeyPoint> vKeysUn1 = vKeys1, vKeysUn2 = vKeys2;
  std::vector<cv::Point2f> vPts1(vKeys1.size()), vPts2(vKeys2.size());

  // 提取关键点像素坐标
  for (size_t i = 0; i < vKeys1.size(); i++) vPts1[i] = vKeys1[i].pt;
  for (size_t i = 0; i < vKeys2.size(); i++) vPts2[i] = vKeys2[i].pt;

  // 构造鱼眼畸变系数D [k0,k1,k2,k3]
  cv::Mat D = (cv::Mat_<float>(4, 1) << mvParameters[4], mvParameters[5],
               mvParameters[6], mvParameters[7]);
  // 单位旋转矩阵，去畸变不做旋转
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  // 获取相机内参矩阵K
  cv::Mat K = this->toK();
  // OpenCV鱼眼去畸变，输出仍然是像素坐标系
  cv::fisheye::undistortPoints(vPts1, vPts1, K, D, R, K);
  cv::fisheye::undistortPoints(vPts2, vPts2, K, D, R, K);

  // 将去畸变像素坐标回填关键点
  for (size_t i = 0; i < vKeys1.size(); i++) vKeysUn1[i].pt = vPts1[i];
  for (size_t i = 0; i < vKeys2.size(); i++) vKeysUn2[i].pt = vPts2[i];

  // 调用两视图重建，求解T21与三角化点
  return tvr->Reconstruct(vKeysUn1, vKeysUn2, vMatches12, T21, vP3D,
                          vbTriangulated);
}

/**
 * @brief 获取OpenCV格式3阶相机内参矩阵K
 * @return cv::Mat 3×3内参矩阵
 */
cv::Mat KannalaBrandt8::toK() {
  cv::Mat K = (cv::Mat_<float>(3, 3) << mvParameters[0], 0.f, mvParameters[2],
               0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
  return K;
}

/**
 * @brief 获取Eigen格式3阶相机内参矩阵K
 * @return Eigen::Matrix3f 3×3内参矩阵
 */
Eigen::Matrix3f KannalaBrandt8::toK_() {
  Eigen::Matrix3f K;
  K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1],
      mvParameters[3], 0.f, 0.f, 1.f;
  return K;
}

/**
 * @brief 极线约束判断，三角化匹配点，判断是否满足极几何约束
 * @param pCamera2 第二个相机对象
 * @param kp1 帧1关键点
 * @param kp2 帧2关键点
 * @param R12 帧1到帧2旋转矩阵
 * @param t12 帧1到帧2平移向量
 * @param sigmaLevel 像素噪声sigma
 * @param unc 不确定性
 * @return bool true满足极线约束；false不满足
 */
bool KannalaBrandt8::epipolarConstrain(
    const std::shared_ptr<GeometricCamera> &pCamera2, const cv::KeyPoint &kp1,
    const cv::KeyPoint &kp2, const Eigen::Matrix3f &R12,
    const Eigen::Vector3f &t12, const float sigmaLevel, const float unc) {
  Eigen::Vector3f p3D;
  // TriangulateMatches返回值大于极小阈值，代表三角化成功，满足约束
  return this->TriangulateMatches(pCamera2, kp1, kp2, R12, t12, sigmaLevel, unc,
                                  p3D) > 0.0001f;
}

/**
 * @brief 匹配点三角化完整流程：视差检查、三角化、深度合法性、重投影误差校验
 * @param kp1 帧1关键点
 * @param kp2 帧2关键点
 * @param pOther 第二个相机模型
 * @param Tcw1 帧1世界到相机位姿
 * @param Tcw2 帧2世界到相机位姿
 * @param sigmaLevel1 帧1噪声sigma
 * @param sigmaLevel2 帧2噪声sigma
 * @param x3Dtriangulated 输出三角化世界坐标系3D点
 * @return bool true三角化全部校验通过；false失败
 */
bool KannalaBrandt8::matchAndtriangulate(
    const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
    const std::shared_ptr<GeometricCamera> &pOther, Sophus::SE3f &Tcw1,
    Sophus::SE3f &Tcw2, const float sigmaLevel1, const float sigmaLevel2,
    Eigen::Vector3f &x3Dtriangulated) {
  // 获取Tcw1的3×4投影矩阵
  Eigen::Matrix<float, 3, 4> eigTcw1 = Tcw1.matrix3x4();
  // 提取Rcw1 世界到相机1旋转
  Eigen::Matrix3f Rcw1 = eigTcw1.block<3, 3>(0, 0);
  // Rwc1 相机1到世界旋转
  Eigen::Matrix3f Rwc1 = Rcw1.transpose();
  // 获取Tcw2的3×4投影矩阵
  Eigen::Matrix<float, 3, 4> eigTcw2 = Tcw2.matrix3x4();
  // Rcw2 世界到相机2旋转
  Eigen::Matrix3f Rcw2 = eigTcw2.block<3, 3>(0, 0);
  // Rwc2 相机2到世界旋转
  Eigen::Matrix3f Rwc2 = Rcw2.transpose();

  // 像素反投影，得到相机坐标系归一化射线
  cv::Point3f ray1c = this->unproject(kp1.pt);
  cv::Point3f ray2c = pOther->unproject(kp2.pt);

  Eigen::Vector3f r1(ray1c.x, ray1c.y, ray1c.z);
  Eigen::Vector3f r2(ray2c.x, ray2c.y, ray2c.z);

  // Check parallax between rays
  // 射线变换到世界坐标系
  Eigen::Vector3f ray1 = Rwc1 * r1;
  Eigen::Vector3f ray2 = Rwc2 * r2;

  // 两射线夹角余弦，衡量视差大小；越接近1视差越小
  const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

  // If parallax is lower than 0.9998, reject this match
  // cos>0.9998 视差太小，三角化不稳定，直接拒绝
  if (cosParallaxRays > 0.9998) {
    return false;
  }

  // Parallax is good, so we try to triangulate
  // 取出归一化平面坐标，z=1，仅xy
  cv::Point2f p11, p22;

  p11.x = ray1c.x;
  p11.y = ray1c.y;

  p22.x = ray2c.x;
  p22.y = ray2c.y;

  Eigen::Vector3f x3D;
  // SVD三角化求解世界点
  Triangulate(p11, p22, eigTcw1, eigTcw2, x3D);
  // Check triangulation in front of cameras
  // 检查点在相机1前方，z>0
  float z1 = Rcw1.row(2).dot(x3D) + Tcw1.translation()(2);
  if (z1 <= 0) {  // Point is not in front of the first camera
    return false;
  }

  // 检查点在相机2前方，z>0
  float z2 = Rcw2.row(2).dot(x3D) + Tcw2.translation()(2);
  if (z2 <= 0) {  // Point is not in front of the first camera
    return false;
  }

  // Check reprojection error in first keyframe
  //   -Transform point into camera reference system
  // 3D点转到相机1坐标系
  Eigen::Vector3f x3D1 = Rcw1 * x3D + Tcw1.translation();
  // 投影回像素
  Eigen::Vector2f uv1 = this->project(x3D1);

  // 计算重投影误差
  float errX1 = uv1(0) - kp1.pt.x;
  float errY1 = uv1(1) - kp1.pt.y;

  // 卡方检验，自由度2，95%阈值5.991；误差过大拒绝
  if ((errX1 * errX1 + errY1 * errY1) >
      5.991 * sigmaLevel1) {  // Reprojection error is high
    return false;
  }

  // Check reprojection error in second keyframe;
  //   -Transform point into camera reference system
  // 3D点转到相机2坐标系
  Eigen::Vector3f x3D2 = Rcw2 * x3D + Tcw2.translation();  // avoid using q
  Eigen::Vector2f uv2 = pOther->project(x3D2);

  float errX2 = uv2(0) - kp2.pt.x;
  float errY2 = uv2(1) - kp2.pt.y;

  if ((errX2 * errX2 + errY2 * errY2) >
      5.991 * sigmaLevel2) {  // Reprojection error is high
    return false;
  }

  // Since parallax is big enough and reprojection errors are low, this pair of
  // points can be considered as a match
  // 全部校验通过，输出三角化3D点
  x3Dtriangulated = x3D;

  return true;
}

/**
 * @brief 三角化匹配点，输入相对位姿R12 t12，第一相机作为单位坐标系
 * @param pCamera2 第二个相机
 * @param kp1 相机1关键点
 * @param kp2 相机2关键点
 * @param R12 相机1到相机2旋转
 * @param t12 相机1到相机2平移
 * @param sigmaLevel 噪声水平
 * @param unc 不确定性
 * @param p3D 输出三角化得到3D点（相机1坐标系）
 * @return float 成功返回z深度值；负数代表各类失败码-1-2-3-4-5
 */
float KannalaBrandt8::TriangulateMatches(
    const std::shared_ptr<GeometricCamera> &pCamera2, const cv::KeyPoint &kp1,
    const cv::KeyPoint &kp2, const Eigen::Matrix3f &R12,
    const Eigen::Vector3f &t12, const float sigmaLevel, const float unc,
    Eigen::Vector3f &p3D) {
  // 像素反投影得到相机归一化射线
  Eigen::Vector3f r1 = this->unprojectEig(kp1.pt);
  Eigen::Vector3f r2 = pCamera2->unprojectEig(kp2.pt);

  // Check parallax
  // r2变换到相机1坐标系下的射线
  Eigen::Vector3f r21 = R12 * r2;

  // 两射线夹角余弦，评估视差
  const float cosParallaxRays = r1.dot(r21) / (r1.norm() * r21.norm());

  // 视差过小直接返回-1
  if (cosParallaxRays > 0.9998) {
    return -1;
  }

  // Parallax is good, so we try to triangulate
  // 提取归一化平面xy坐标
  cv::Point2f p11, p22;

  p11.x = r1[0];
  p11.y = r1[1];

  p22.x = r2[0];
  p22.y = r2[1];

  Eigen::Vector3f x3D;
  // Tcw1：相机1投影矩阵，单位姿态
  Eigen::Matrix<float, 3, 4> Tcw1;
  Tcw1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

  // Tcw2：相机2在相机1坐标系下投影矩阵
  Eigen::Matrix<float, 3, 4> Tcw2;
  Eigen::Matrix3f R21 = R12.transpose();
  Tcw2 << R21, -R21 * t12;

  // SVD三角化
  Triangulate(p11, p22, Tcw1, Tcw2, x3D);
  // cv::Mat x3Dt = x3D.t();

  // 相机1坐标系深度z1，小于0返回-2
  float z1 = x3D(2);
  if (z1 <= 0) {
    return -2;
  }

  // 相机2坐标系深度z2，小于0返回-3
  float z2 = R21.row(2).dot(x3D) + Tcw2(2, 3);
  if (z2 <= 0) {
    return -3;
  }

  // Check reprojection error
  // 相机1重投影误差校验
  Eigen::Vector2f uv1 = this->project(x3D);

  float errX1 = uv1(0) - kp1.pt.x;
  float errY1 = uv1(1) - kp1.pt.y;

  // 重投影误差超限返回-4
  if ((errX1 * errX1 + errY1 * errY1) >
      5.991 * sigmaLevel) {  // Reprojection error is high
    return -4;
  }

  // 3D点转到相机2坐标系
  Eigen::Vector3f x3D2 = R21 * x3D + Tcw2.col(3);
  Eigen::Vector2f uv2 = pCamera2->project(x3D2);

  float errX2 = uv2(0) - kp2.pt.x;
  float errY2 = uv2(1) - kp2.pt.y;

  // 相机2重投影误差超限返回-5
  if ((errX2 * errX2 + errY2 * errY2) >
      5.991 * unc) {  // Reprojection error is high
    return -5;
  }

  // 全部校验通过，输出3D点，返回相机1深度z1
  p3D = x3D;

  return z1;
}

/**
 * @brief 输出流重载，打印KannalaBrandt8相机全部8个参数
 * @param os 输出流
 * @param kb 相机对象
 * @return std::ostream&
 */
std::ostream &operator<<(std::ostream &os, const KannalaBrandt8 &kb) {
  os << kb.mvParameters[0] << " " << kb.mvParameters[1] << " "
     << kb.mvParameters[2] << " " << kb.mvParameters[3] << " "
     << kb.mvParameters[4] << " " << kb.mvParameters[5] << " "
     << kb.mvParameters[6] << " " << kb.mvParameters[7];
  return os;
}

/**
 * @brief 输入流重载，从流读取8个相机参数赋值给KannalaBrandt8对象
 * @param is 输入流
 * @param kb 相机对象
 * @return std::istream&
 */
std::istream &operator>>(std::istream &is, KannalaBrandt8 &kb) {
  float nextParam;
  // 循环读取8个参数
  for (size_t i = 0; i < 8; i++) {
    assert(is.good());  // Make sure the input stream is good
    is >> nextParam;
    kb.mvParameters[i] = nextParam;
  }
  return is;
}

/**
 * @brief SVD三角化函数，DLT直接线性变换求解齐次4维点，归一化得到3D坐标
 * @param p1 归一化平面点1 z=1
 * @param p2 归一化平面点2 z=1
 * @param Tcw1 相机1的3×4投影矩阵
 * @param Tcw2 相机2的3×4投影矩阵
 * @param x3D 输出三角化三维点
 */
void KannalaBrandt8::Triangulate(const cv::Point2f &p1, const cv::Point2f &p2,
                                 const Eigen::Matrix<float, 3, 4> &Tcw1,
                                 const Eigen::Matrix<float, 3, 4> &Tcw2,
                                 Eigen::Vector3f &x3D) {
  Eigen::Matrix<float, 4, 4> A;
  // DLT构造4×4方程组矩阵A，两行来自相机1，两行来自相机2
  A.row(0) = p1.x * Tcw1.row(2) - Tcw1.row(0);
  A.row(1) = p1.y * Tcw1.row(2) - Tcw1.row(1);
  A.row(2) = p2.x * Tcw2.row(2) - Tcw2.row(0);
  A.row(3) = p2.y * Tcw2.row(2) - Tcw2.row(1);

  // SVD分解A，取V矩阵最后一列作为齐次解
  Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4f x3Dh = svd.matrixV().col(3);
  // 齐次坐标归一化，X = Xh / w
  x3D = x3Dh.head(3) / x3Dh(3);
}

/**
 * @brief 判断两个相机对象参数是否完全相等
 * @param pCam 待比较相机智能指针
 * @return bool true参数一致；false不一致
 */
bool KannalaBrandt8::IsEqual(const std::shared_ptr<GeometricCamera> &pCam) {
  // 判断相机类型必须是鱼眼CAM_FISHEYE，否则直接false
  if (pCam->GetType() != GeometricCamera::CAM_FISHEYE) return false;

  // 基类指针向下转型为KannalaBrandt8引用
  KannalaBrandt8 &pKBCam = dynamic_cast<KannalaBrandt8 &>(*pCam);

  // 比较迭代精度precision，差值大于1e-6判定不等
  if (abs(precision - pKBCam.GetPrecision()) > 1e-6) return false;

  // 参数数量不一致返回false
  if (size() != pKBCam.size()) return false;

  bool is_same_camera = true;
  // 循环比对全部8个相机参数，容许1e-6误差
  for (size_t i = 0; i < size(); ++i) {
    if (abs(mvParameters[i] - pKBCam.getParameter(i)) > 1e-6) {
      is_same_camera = false;
      break;
    }
  }
  return is_same_camera;
}

}  // namespace ORB_SLAM3