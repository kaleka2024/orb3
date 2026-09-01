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

// 转换器类头文件：提供OpenCV/Eigen/g2o/Sophus之间矩阵、向量、位姿的类型互转接口
#include "Converter.h"
// STL标准库：动态数组容器，用于存储描述子向量、四元数、欧拉角等序列数据
#include <vector>

namespace ORB_SLAM3 {

// ==============================================
// 功能：将OpenCV格式的描述子矩阵转换为vector<cv::Mat>格式
// 原理：把N行D列的描述子矩阵拆分为N个1行D列的单行描述子，依次存入vector
// 用途：适配DBoW2词袋库的输入格式要求，用于回环检测与重定位
// ==============================================
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors) {
  std::vector<cv::Mat> vDesc;
  // 预分配内存空间，避免多次扩容拷贝，提升性能
  vDesc.reserve(Descriptors.rows);
  // 逐行提取每个特征点的描述子，存入向量
  for (int j = 0; j < Descriptors.rows; j++)
    vDesc.push_back(Descriptors.row(j));

  return vDesc;
}

// ==============================================
// 功能：OpenCV 4×4位姿矩阵 → g2o::SE3Quat位姿
// 原理：提取cv::Mat中的旋转部分与平移部分，构造g2o优化使用的四元数+平移位姿
// 用途：后端图优化(g2o)中位姿顶点、约束边的类型转换
// ==============================================
g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT) {
  // 定义3×3双精度Eigen旋转矩阵
  Eigen::Matrix<double, 3, 3> R;
  // 逐元素从cv::Mat左上角3×3区域复制旋转矩阵值
  R << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2),
      cvT.at<float>(1, 0), cvT.at<float>(1, 1), cvT.at<float>(1, 2),
      cvT.at<float>(2, 0), cvT.at<float>(2, 1), cvT.at<float>(2, 2);

  // 定义3×1双精度Eigen平移向量，从cv::Mat最右列前三行提取
  Eigen::Matrix<double, 3, 1> t(cvT.at<float>(0, 3), cvT.at<float>(1, 3),
                                 cvT.at<float>(2, 3));

  // 用旋转矩阵+平移向量构造g2o::SE3Quat对象并返回
  return g2o::SE3Quat(R, t);
}

// ==============================================
// 功能：Sophus::SE3f李群位姿 → g2o::SE3Quat位姿
// 原理：提取Sophus的单位四元数与平移向量，精度从float提升为double后构造g2o位姿
// 用途：前端跟踪得到的Sophus位姿传入后端g2o优化时的类型转换
// ==============================================
g2o::SE3Quat Converter::toSE3Quat(const Sophus::SE3f &T) {
  // 提取单位四元数转双精度 + 平移向量转双精度，构造g2o SE3位姿
  return g2o::SE3Quat(T.unit_quaternion().cast<double>(),
                      T.translation().cast<double>());
}

// ==============================================
// 功能：g2o::SE3Quat位姿 → OpenCV cv::Mat位姿矩阵
// 原理：先转为4×4齐次Eigen矩阵，再调用重载函数转为cv::Mat
// ==============================================
cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3) {
  // SE3Quat转为4×4齐次变换矩阵（Eigen双精度）
  Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
  // 调用Eigen矩阵转cv::Mat的重载函数完成最终转换
  return toCvMat(eigMat);
}

// ==============================================
// 功能：g2o::Sim3相似变换 → OpenCV cv::Mat位姿矩阵
// 原理：提取Sim3的旋转、平移、尺度，将尺度融入旋转矩阵后构造3×4位姿
// 用途：闭环检测中Sim3尺度优化结果，转为常规SE3位姿格式
// ==============================================
cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3) {
  // 提取Sim3中的旋转矩阵（3×3双精度）
  Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
  // 提取Sim3中的平移向量（3×1双精度）
  Eigen::Vector3d eigt = Sim3.translation();
  // 提取Sim3中的尺度因子
  double s = Sim3.scale();
  // 构造带尺度的SE3矩阵：s*R作为旋转部分，t作为平移部分
  return toCvSE3(s * eigR, eigt);
}

// ==============================================
// 功能：Eigen 4×4双精度矩阵 → OpenCV cv::Mat矩阵
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &m) {
  // 创建4行4列单精度浮点型OpenCV矩阵
  cv::Mat cvMat(4, 4, CV_32F);
  // 逐元素复制矩阵数值
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) cvMat.at<float>(i, j) = m(i, j);

  // 返回深拷贝矩阵，避免临时内存释放导致野指针
  return cvMat.clone();
}

// ==============================================
// 功能：Eigen 4×4单精度矩阵 → OpenCV cv::Mat矩阵
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 4, 4> &m) {
  cv::Mat cvMat(4, 4, CV_32F);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

// ==============================================
// 功能：Eigen 3×4单精度矩阵 → OpenCV cv::Mat矩阵
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 3, 4> &m) {
  cv::Mat cvMat(3, 4, CV_32F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 4; j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

// ==============================================
// 功能：Eigen 3×3双精度矩阵 → OpenCV cv::Mat矩阵
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m) {
  cv::Mat cvMat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

// ==============================================
// 功能：Eigen 3×3单精度矩阵 → OpenCV cv::Mat矩阵
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::Matrix3f &m) {
  cv::Mat cvMat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

// ==============================================
// 功能：Eigen动态大小单精度矩阵 → OpenCV cv::Mat矩阵
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::MatrixXf &m) {
  // 根据Eigen矩阵的行列数创建对应尺寸的OpenCV矩阵
  cv::Mat cvMat(m.rows(), m.cols(), CV_32F);
  for (int i = 0; i < m.rows(); i++)
    for (int j = 0; j < m.cols(); j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

// ==============================================
// 功能：Eigen动态大小双精度矩阵 → OpenCV cv::Mat矩阵
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::MatrixXd &m) {
  cv::Mat cvMat(m.rows(), m.cols(), CV_32F);
  for (int i = 0; i < m.rows(); i++)
    for (int j = 0; j < m.cols(); j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

// ==============================================
// 功能：Eigen 3×1双精度向量 → OpenCV cv::Mat列向量
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &m) {
  cv::Mat cvMat(3, 1, CV_32F);
  for (int i = 0; i < 3; i++) cvMat.at<float>(i) = m(i);

  return cvMat.clone();
}

// ==============================================
// 功能：Eigen 3×1单精度向量 → OpenCV cv::Mat列向量
// ==============================================
cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 3, 1> &m) {
  cv::Mat cvMat(3, 1, CV_32F);
  for (int i = 0; i < 3; i++) cvMat.at<float>(i) = m(i);

  return cvMat.clone();
}

// ==============================================
// 功能：从旋转矩阵+平移向量构造OpenCV格式的4×4 SE3齐次位姿矩阵
// 原理：左上角填充旋转矩阵，最右列填充平移向量，右下角补1构成齐次形式
// ==============================================
cv::Mat Converter::toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                           const Eigen::Matrix<double, 3, 1> &t) {
  // 初始化4×4单位矩阵，保证右下角齐次项为1
  cv::Mat cvMat = cv::Mat::eye(4, 4, CV_32F);
  // 左上角3×3区域赋值为旋转矩阵
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cvMat.at<float>(i, j) = R(i, j);
    }
  }
  // 最右列前三行赋值为平移向量
  for (int i = 0; i < 3; i++) {
    cvMat.at<float>(i, 3) = t(i);
  }

  return cvMat.clone();
}

// ==============================================
// 功能：OpenCV 3×1列向量 → Eigen 3×1双精度向量
// ==============================================
Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Mat &cvVector) {
  Eigen::Matrix<double, 3, 1> v;
  // 依次复制x、y、z三个分量
  v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

  return v;
}

// ==============================================
// 功能：OpenCV 3×1列向量 → Eigen 3×1单精度向量
// ==============================================
Eigen::Matrix<float, 3, 1> Converter::toVector3f(const cv::Mat &cvVector) {
  Eigen::Matrix<float, 3, 1> v;
  v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

  return v;
}

// ==============================================
// 功能：OpenCV Point3f三维点 → Eigen 3×1双精度向量
// ==============================================
Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Point3f &cvPoint) {
  Eigen::Matrix<double, 3, 1> v;
  v << cvPoint.x, cvPoint.y, cvPoint.z;

  return v;
}

// ==============================================
// 功能：OpenCV 3×3矩阵 → Eigen 3×3双精度矩阵
// ==============================================
Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const cv::Mat &cvMat3) {
  Eigen::Matrix<double, 3, 3> M;

  // 逐行逐列复制矩阵元素
  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
      cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
      cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

  return M;
}

// ==============================================
// 功能：OpenCV 4×4矩阵 → Eigen 4×4双精度矩阵
// ==============================================
Eigen::Matrix<double, 4, 4> Converter::toMatrix4d(const cv::Mat &cvMat4) {
  Eigen::Matrix<double, 4, 4> M;

  M << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2),
      cvMat4.at<float>(0, 3), cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1),
      cvMat4.at<float>(1, 2), cvMat4.at<float>(1, 3), cvMat4.at<float>(2, 0),
      cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2), cvMat4.at<float>(2, 3),
      cvMat4.at<float>(3, 0), cvMat4.at<float>(3, 1), cvMat4.at<float>(3, 2),
      cvMat4.at<float>(3, 3);
  return M;
}

// ==============================================
// 功能：OpenCV 3×3矩阵 → Eigen 3×3单精度矩阵
// ==============================================
Eigen::Matrix<float, 3, 3> Converter::toMatrix3f(const cv::Mat &cvMat3) {
  Eigen::Matrix<float, 3, 3> M;

  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
      cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
      cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

  return M;
}

// ==============================================
// 功能：OpenCV 4×4矩阵 → Eigen 4×4单精度矩阵
// ==============================================
Eigen::Matrix<float, 4, 4> Converter::toMatrix4f(const cv::Mat &cvMat4) {
  Eigen::Matrix<float, 4, 4> M;

  M << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2),
      cvMat4.at<float>(0, 3), cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1),
      cvMat4.at<float>(1, 2), cvMat4.at<float>(1, 3), cvMat4.at<float>(2, 0),
      cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2), cvMat4.at<float>(2, 3),
      cvMat4.at<float>(3, 0), cvMat4.at<float>(3, 1), cvMat4.at<float>(3, 2),
      cvMat4.at<float>(3, 3);
  return M;
}

// ==============================================
// 功能：OpenCV旋转矩阵 → 四元数（vector<float>存储，顺序x,y,z,w）
// 用途：部分接口需要四元数形式存储、传输旋转信息
// ==============================================
std::vector<float> Converter::toQuaternion(const cv::Mat &M) {
  // 先转为Eigen 3×3双精度旋转矩阵
  Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(M);
  // 从旋转矩阵构造Eigen四元数
  Eigen::Quaterniond q(eigMat);

  // 构造4元素float向量，按x,y,z,w顺序存储四元数分量
  std::vector<float> v(4);
  v[0] = q.x();
  v[1] = q.y();
  v[2] = q.z();
  v[3] = q.w();

  return v;
}

// ==============================================
// 功能：从三维向量构造3×3反对称矩阵（叉乘矩阵）
// 原理：对于向量v=[v1,v2,v3]，生成反对称矩阵，满足 a×b = [a]× · b
// 用途：李代数运算、将叉乘运算转换为矩阵乘法
// ==============================================
cv::Mat Converter::tocvSkewMatrix(const cv::Mat &v) {
  // 直接初始化3×3反对称矩阵并返回
  return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1),
           v.at<float>(2), 0, -v.at<float>(0), -v.at<float>(1), v.at<float>(0),
           0);
}

// ==============================================
// 功能：验证输入矩阵是否为合法的旋转矩阵
// 原理：旋转矩阵满足正交性 R^T · R = I，通过计算转置乘自身与单位矩阵的误差判断
// ==============================================
bool Converter::isRotationMatrix(const cv::Mat &R) {
  cv::Mat Rt;
  // 计算输入矩阵的转置
  cv::transpose(R, Rt);
  // 转置矩阵 × 原矩阵，理论结果应为单位矩阵
  cv::Mat shouldBeIdentity = Rt * R;
  // 生成同数据类型的3×3单位矩阵
  cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

  // 计算两矩阵的F范数，误差小于1e-6则判定为有效旋转矩阵
  return cv::norm(I, shouldBeIdentity) < 1e-6;
}

// ==============================================
// 功能：旋转矩阵 → Z-Y-X顺序欧拉角（roll / pitch / yaw），单位：弧度
// 说明：处理万向锁奇异情况，保证数值稳定性
// ==============================================
std::vector<float> Converter::toEuler(const cv::Mat &R) {
  // 断言验证：输入必须是合法旋转矩阵
  assert(isRotationMatrix(R));
  // 计算sy：旋转矩阵第一列前两元素的模长，对应cos(pitch)的绝对值
  float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) +
                  R.at<float>(1, 0) * R.at<float>(1, 0));

  // 判断是否接近奇异状态（pitch≈±90°，出现万向锁）
  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular) {
    // 非奇异情况：标准ZYX欧拉角分解公式
    x = atan2(R.at<float>(2, 1), R.at<float>(2, 2)); // roll：绕X轴旋转角
    y = atan2(-R.at<float>(2, 0), sy);              // pitch：绕Y轴旋转角
    z = atan2(R.at<float>(1, 0), R.at<float>(0, 0)); // yaw：绕Z轴旋转角
  } else {
    // 奇异情况（万向锁）：roll与yaw耦合，令z=0，仅计算x和y
    x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
    y = atan2(-R.at<float>(2, 0), sy);
    z = 0;
  }

  // 构造欧拉角向量，顺序为 [roll, pitch, yaw]
  std::vector<float> v_euler(3);
  v_euler[0] = x;
  v_euler[1] = y;
  v_euler[2] = z;

  return v_euler;
}

// ==============================================
// 功能：OpenCV位姿矩阵 → Sophus::SE3f李群位姿
// 用途：将OpenCV存储的位姿转为Sophus格式，用于前端李代数位姿运算
// ==============================================
Sophus::SE3<float> Converter::toSophus(const cv::Mat &T) {
  // 提取左上角3×3旋转区域，转为Eigen双精度矩阵
  Eigen::Matrix<double, 3, 3> eigMat =
      toMatrix3d(T.rowRange(0, 3).colRange(0, 3));
  // 旋转矩阵转单精度四元数
  Eigen::Quaternionf q(eigMat.cast<float>());

  // 提取最右列平移向量，转为Eigen单精度向量
  Eigen::Matrix<float, 3, 1> t =
      toVector3d(T.rowRange(0, 3).col(3)).cast<float>();

  // 用四元数和平移向量构造Sophus SE3位姿并返回
  return Sophus::SE3<float>(q, t);
}

// ==============================================
// 功能：g2o::Sim3相似变换 → Sophus::Sim3f相似变换
// 用途：后端g2o优化得到的Sim3结果，转为Sophus格式用于前端位姿运算
// ==============================================
Sophus::Sim3f Converter::toSophus(const g2o::Sim3 &S) {
  // 构造Sophus::RxSO3（尺度+旋转），再拼接平移向量，整体转单精度
  return Sophus::Sim3f(
      Sophus::RxSO3d(static_cast<float>(S.scale()), S.rotation().matrix())
          .cast<float>(),
      S.translation().cast<float>());
}

}  // namespace ORB_SLAM3
