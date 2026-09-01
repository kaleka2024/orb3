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
#pragma once                          // 头文件保护，防止重复包含
#include <Eigen/Core>                 // Eigen线性代数库，矩阵、向量、四元数数据结构
#include <boost/serialization/serialization.hpp>   // boost序列化核心头文件，实现对象持久化保存/加载
#include <boost/serialization/vector.hpp>          // boost对std::vector容器的序列化支持
#include <opencv2/core/core.hpp>                   // OpenCV核心模块，cv::Mat基础矩阵
#include <opencv2/features2d/features2d.hpp>       // OpenCV特征点模块，cv::KeyPoint定义
#include <sophus/se3.hpp>                          // Sophus库SE3李群，位姿旋转平移变换
#include <vector>                                   // std::vector动态数组容器

namespace ORB_SLAM3 {

/**
 * @brief Sophus::SE3f位姿的boost序列化函数，实现SE3位姿保存到文件、从文件恢复
 * @tparam Archive boost序列化归档类型(输出归档保存 / 输入归档加载)
 * @param ar boost序列化归档对象
 * @param T 待序列化/反序列化的SE3位姿
 * @param version 序列化版本号
 */
template <class Archive> void serializeSophusSE3(Archive& ar, Sophus::SE3f& T,
                        const unsigned int version) {
  Eigen::Vector4f quat;   ///< 四元数(w,x,y,z)，存储旋转信息
  Eigen::Vector3f transl; ///< 平移向量(tx,ty,tz)

  // 如果是保存模式：把SE3分解为四元数和平移向量
  if (Archive::is_saving::value) {
    Eigen::Quaternionf q = T.unit_quaternion();
    quat << q.w(), q.x(), q.y(), q.z();
    transl = T.translation();
  }

  // 将四元数、平移数组交给boost序列化，以原始数组字节读写
  ar& boost::serialization::make_array(quat.data(), quat.size());
  ar& boost::serialization::make_array(transl.data(), transl.size());

  // 如果是加载模式：从读取到的四元数和平移重建SE3位姿
  if (Archive::is_loading::value) {
    Eigen::Quaternionf q(quat[0], quat[1], quat[2], quat[3]);
    T = Sophus::SE3f(q, transl);
  }
}

/*template <class Archive, size_t dim> void serializeDiagonalMatrix(Archive &ar, Eigen::DiagonalMatrix<float, dim> &D, const unsigned int version) {
    Eigen::Matrix<float,dim,dim> dense;
    if(Archive::is_saving::value)
    {
        dense = D.toDenseMatrix();
    }

    ar & boost::serialization::make_array(dense.data(), dense.size());

    if (Archive::is_loading::value)
    {
        D = dense.diagonal().asDiagonal();
    }
}*/

/**
 * @brief cv::Mat非const版本boost序列化函数，实现OpenCV矩阵保存与加载
 * @tparam Archive boost序列化归档类型
 * @param ar boost序列化归档对象
 * @param mat 待序列化cv::Mat矩阵
 * @param version 序列化版本号
 */
template <class Archive> void serializeMatrix(Archive& ar, cv::Mat& mat, const unsigned int version) {
  int cols, rows, type;
  bool continuous;

  // 保存模式：提取mat元信息：行列数、数据类型、内存是否连续
  if (Archive::is_saving::value) {
    cols = mat.cols;
    rows = mat.rows;
    type = mat.type();
    continuous = mat.isContinuous();
  }

  // 序列化矩阵元数据
  ar & cols & rows & type & continuous;

  // 加载模式：先创建空mat，分配对应尺寸与类型内存
  if (Archive::is_loading::value) mat.create(rows, cols, type);

  // 判断内存布局：连续整块内存直接一次性读写；非连续按行逐行读写
  if (continuous) {
    const unsigned int data_size = rows * cols * mat.elemSize();
    ar& boost::serialization::make_array(mat.ptr(), data_size);
  } else {
    const unsigned int row_size = cols * mat.elemSize();
    for (int i = 0; i < rows; i++) {
      ar& boost::serialization::make_array(mat.ptr(i), row_size);
    }
  }
}

/**
 * @brief cv::Mat const常量版本boost序列化重载，const对象不能直接修改，借助临时matAux中转
 * @tparam Archive boost序列化归档类型
 * @param ar boost序列化归档对象
 * @param mat const修饰待序列化cv::Mat
 * @param version 序列化版本号
 */
template <class Archive> void serializeMatrix(Archive& ar, const cv::Mat& mat,
                     const unsigned int version) {
  cv::Mat matAux = mat;

  // 调用上面非const版本序列化临时矩阵
  serializeMatrix(ar, matAux, version);

  // 加载完成后，绕过const限制，把临时矩阵赋值回原const mat
  if (Archive::is_loading::value) {
    cv::Mat* ptr;
    ptr = (cv::Mat*)(&mat);
    *ptr = matAux;
  }
}

/**
 * @brief std::vector<cv::KeyPoint>常量版本boost序列化，关键点容器持久化
 * @tparam Archive boost序列化归档类型
 * @param ar boost序列化归档对象
 * @param vKP const修饰关键点vector
 * @param version 序列化版本号
 */
template <class Archive> void serializeVectorKeyPoints(Archive& ar, const std::vector<cv::KeyPoint>& vKP,
                              const unsigned int version) {
  int NumEl;

  // 保存模式：获取关键点数量
  if (Archive::is_saving::value) {
    NumEl = vKP.size();
  }

  ar & NumEl;

  // 使用临时辅助容器vKPaux做读写中转，规避const不可直接修改限制
  std::vector<cv::KeyPoint> vKPaux = vKP;
  if (Archive::is_loading::value) vKPaux.reserve(NumEl);

  // 逐个序列化KeyPoint成员属性：角度、响应值、特征点尺寸、坐标、类别id、金字塔层数
  for (int i = 0; i < NumEl; ++i) {
    cv::KeyPoint KPi;

    if (Archive::is_loading::value) KPi = cv::KeyPoint();

    if (Archive::is_saving::value) KPi = vKPaux[i];

    ar & KPi.angle;
    ar & KPi.response;
    ar & KPi.size;
    ar & KPi.pt.x;
    ar & KPi.pt.y;
    ar & KPi.class_id;
    ar & KPi.octave;

    // 加载模式把解析完成的关键点存入临时容器
    if (Archive::is_loading::value) vKPaux.push_back(KPi);
  }

  // 加载完毕，绕过const，将临时容器赋值回原始const vector
  if (Archive::is_loading::value) {
    std::vector<cv::KeyPoint>* ptr;
    ptr = (std::vector<cv::KeyPoint>*)(&vKP);
    *ptr = vKPaux;
  }
}

}  // namespace ORB_SLAM3
