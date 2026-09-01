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
// 头文件保护，防止头文件重复包含
#pragma once

// Eigen线性代数库，矩阵、位姿数据结构
#include <Eigen/Dense>
// OpenCV核心模块，cv::Mat、cv::Point等数据结构
#include <opencv2/core/core.hpp>
// STL动态数组容器
#include <vector>

// g2o六自由度SE3位姿优化类型（旋转+平移）
#include "g2o/types/sba/types_six_dof_expmap.h"
// g2o七自由度Sim3相似变换类型（旋转+平移+尺度）
#include "g2o/types/sim3/types_seven_dof_expmap.h"
// Sophus几何库SE3位姿定义
#include "sophus/geometry.hpp"
// Sophus Sim3相似变换定义
#include "sophus/sim3.hpp"

namespace ORB_SLAM3 {

/**
 * @brief 数据格式转换工具类，全部为静态函数
 * 完成OpenCV cv::Mat、Eigen、g2o、Sophus之间位姿、矩阵、向量、描述子的互相转换
 */
class Converter {
public:
    // Eigen内存对齐宏，保证Eigen对象内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 将OpenCV的描述子Mat，转换为std::vector<cv::Mat>，每一个元素对应一个特征点描述子
     * @param Descriptors OpenCV描述子矩阵，行数=特征点数量，列数=描述子维度
     * @return std::vector<cv::Mat> 每个特征点单独的描述子向量
     */
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    /**
     * @brief cv::Mat格式T(4*4变换矩阵)转为g2o::SE3Quat
     * @param cvT OpenCV 4×4 SE3变换矩阵
     * @return g2o::SE3Quat g2o的SE3四元数+平移位姿对象
     */
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

    /**
     * @brief Sophus::SE3f(float版本SE3)转为g2o::SE3Quat(double版本)
     * @param T Sophus float类型SE3位姿
     * @return g2o::SE3Quat g2o SE3对象
     */
    static g2o::SE3Quat toSE3Quat(const Sophus::SE3f &T);

    /**
     * @brief g2o::Sim3(七自由度相似变换)提取旋转平移，转为g2o::SE3Quat（丢弃尺度）
     * @param gSim3 g2o Sim3相似变换
     * @return g2o::SE3Quat g2o SE3位姿
     */
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    // TODO templetize these functions
    /**
     * @brief g2o::SE3Quat转换为OpenCV 4×4 cv::Mat变换矩阵
     * @param SE3 g2o SE3位姿对象
     * @return cv::Mat 4行4列cv::Mat
     */
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

    /**
     * @brief g2o::Sim3七自由度相似变换转为OpenCV 4×4 cv::Mat矩阵（带尺度）
     * @param Sim3 g2o Sim3对象
     * @return cv::Mat 4×4相似变换矩阵
     */
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

    /**
     * @brief Eigen 4×4 double矩阵转为cv::Mat
     * @param m Eigen Matrix<double,4,4>
     * @return cv::Mat 4×4 OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);

    /**
     * @brief Eigen 4×4 float矩阵转为cv::Mat
     * @param m Eigen Matrix<float,4,4>
     * @return cv::Mat 4×4 OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::Matrix<float, 4, 4> &m);

    /**
     * @brief Eigen 3×4 float矩阵转为cv::Mat，常用于相机外参[R|t]
     * @param m Eigen Matrix<float,3,4>
     * @return cv::Mat 3×4 OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 4> &m);

    /**
     * @brief Eigen 3×3 double旋转矩阵转为cv::Mat
     * @param m Eigen Matrix3d
     * @return cv::Mat 3×3 OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);

    /**
     * @brief Eigen 3×1 double向量转为cv::Mat
     * @param m Eigen Matrix<double,3,1>
     * @return cv::Mat 3行1列OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);

    /**
     * @brief Eigen 3×1 float向量转为cv::Mat
     * @param m Eigen Matrix<float,3,1>
     * @return cv::Mat 3行1列OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 1> &m);

    /**
     * @brief Eigen 3×3 float矩阵转为cv::Mat
     * @param m Eigen Matrix<float,3,3>
     * @return cv::Mat 3×3 OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 3> &m);

    /**
     * @brief Eigen动态大小float矩阵MatrixXf转为cv::Mat
     * @param m Eigen MatrixXf
     * @return cv::Mat OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::MatrixXf &m);

    /**
     * @brief Eigen动态大小double矩阵MatrixXd转为cv::Mat
     * @param m Eigen MatrixXd
     * @return cv::Mat OpenCV矩阵
     */
    static cv::Mat toCvMat(const Eigen::MatrixXd &m);

    /**
     * @brief 给定旋转矩阵R和平移向量t，组合生成OpenCV SE3 4×4变换矩阵
     * @param R Eigen3d旋转矩阵
     * @param t Eigen3d平移向量
     * @return cv::Mat 4×4 SE3变换矩阵
     */
    static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                            const Eigen::Matrix<double, 3, 1> &t);

    /**
     * @brief 由三维向量v计算反对称矩阵（斜对称矩阵）
     * @param v OpenCV 3维向量
     * @return cv::Mat 3×3反对称矩阵
     */
    static cv::Mat tocvSkewMatrix(const cv::Mat &v);

    /**
     * @brief OpenCV cv::Mat(3×1)向量转为Eigen::Vector3d(double)
     * @param cvVector OpenCV 3行1列向量
     * @return Eigen::Matrix<double,3,1> Eigen三维double向量
     */
    static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);

    /**
     * @brief OpenCV cv::Mat(3×1)向量转为Eigen::Vector3f(float)
     * @param cvVector OpenCV 3行1列向量
     * @return Eigen::Matrix<float,3,1> Eigen三维float向量
     */
    static Eigen::Matrix<float, 3, 1> toVector3f(const cv::Mat &cvVector);

    /**
     * @brief OpenCV cv::Point3f点转为Eigen::Vector3d
     * @param cvPoint OpenCV三维点
     * @return Eigen::Matrix<double,3,1> Eigen三维double向量
     */
    static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);

    /**
     * @brief OpenCV 3×3 cv::Mat转为Eigen Matrix3d(double旋转矩阵)
     * @param cvMat3 OpenCV 3×3矩阵
     * @return Eigen::Matrix<double,3,3> Eigen3d矩阵
     */
    static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);

    /**
     * @brief OpenCV 4×4 cv::Mat转为Eigen Matrix4d(double)
     * @param cvMat4 OpenCV4×4矩阵
     * @return Eigen::Matrix<double,4,4> Eigen4d矩阵
     */
    static Eigen::Matrix<double, 4, 4> toMatrix4d(const cv::Mat &cvMat4);

    /**
     * @brief OpenCV 3×3 cv::Mat转为Eigen Matrix3f(float)
     * @param cvMat3 OpenCV3×3矩阵
     * @return Eigen::Matrix<float,3,3> Eigen3f矩阵
     */
    static Eigen::Matrix<float, 3, 3> toMatrix3f(const cv::Mat &cvMat3);

    /**
     * @brief OpenCV 4×4 cv::Mat转为Eigen Matrix4f(float)
     * @param cvMat4 OpenCV4×4矩阵
     * @return Eigen::Matrix<float,4,4> Eigen4f矩阵
     */
    static Eigen::Matrix<float, 4, 4> toMatrix4f(const cv::Mat &cvMat4);

    /**
     * @brief 旋转矩阵M转为四元数，返回std::vector<float> {x,y,z,w}
     * @param M OpenCV旋转矩阵
     * @return std::vector<float> 四元数向量
     */
    static std::vector<float> toQuaternion(const cv::Mat &M);

    /**
     * @brief 判断输入矩阵R是否为合法旋转矩阵（正交，行列式≈1）
     * @param R OpenCV3×3矩阵
     * @return true 是旋转矩阵；false 不是
     */
    static bool isRotationMatrix(const cv::Mat &R);

    /**
     * @brief 旋转矩阵转为欧拉角，返回std::vector<float> {roll,pitch,yaw}
     * @param R OpenCV3×3旋转矩阵
     * @return std::vector<float> 欧拉角
     */
    static std::vector<float> toEuler(const cv::Mat &R);

    // TODO: Sophus migration, to be deleted in the future
    /**
     * @brief OpenCV4×4变换矩阵T转为Sophus::SE3<float>
     * @param T OpenCV4×4 SE3变换矩阵
     * @return Sophus::SE3<float> Sophus float SE3位姿
     */
    static Sophus::SE3<float> toSophus(const cv::Mat &T);

    /**
     * @brief g2o::Sim3转为Sophus::Sim3f(float版本Sim3)
     * @param S g2o Sim3对象
     * @return Sophus::Sim3f Sophus float相似变换
     */
    static Sophus::Sim3f toSophus(const g2o::Sim3 &S);
};

}  // namespace ORB_SLAM3
