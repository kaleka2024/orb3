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
#pragma once // 头文件保护，防止该头文件被多次重复include引发重定义问题
#include <Eigen/Core>        // Eigen线性代数库基础头文件，矩阵、向量数据结构
#include <iostream>          // C++标准输入输出流，用于打印调试信息
#include <opencv2/core/core.hpp> // OpenCV核心模块，cv::Mat等数据结构
#include <sophus/se3.hpp>    // Sophus库，专门用于SO(3)/SE(3)李群李代数运算

namespace ORB_SLAM3 {

class KeyFrame; // 前向声明关键帧类，此处仅用指针，不需要完整类定义，减少头文件依赖

class GeometricTools {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类中使用Eigen固定大小矩阵时必须添加，避免内存未对齐崩溃

    // Compute the Fundamental matrix between KF1 and KF2
    // 静态函数：计算两个关键帧KF1、KF2之间的基础矩阵F12
    static Eigen::Matrix3f ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

    // Triangulate point with KF1 and KF2
    // 静态函数：对两个关键帧下的归一化平面观测点做三角化，恢复3D空间点坐标
    // x_c1：KF1归一化平面观测点；x_c2：KF2归一化平面观测点
    // Tc1w：KF1相机到世界的投影矩阵3×4；Tc2w：KF2相机到世界的投影矩阵3×4
    // x3D：输出三角化得到的世界坐标系下3D点坐标；返回true代表三角化成功
    static bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,
                            Eigen::Matrix<float, 3, 4> &Tc1w,
                            Eigen::Matrix<float, 3, 4> &Tc2w,
                            Eigen::Vector3f &x3D);

    // 模板函数：校验OpenCV cv::Mat矩阵与Eigen矩阵数据是否近似相等，用于调试核对矩阵转换结果
    template <int rows, int cols>
    static bool CheckMatrices(const cv::Mat &cvMat,
                              const Eigen::Matrix<float, rows, cols> &eigMat) {
        const float epsilon = 1e-3; // 允许的数值误差阈值，两个元素差值超过该值判定矩阵不一致
        // std::cout << cvMat.cols - cols << cvMat.rows - rows << std::endl;
        // 先校验矩阵行列尺寸是否匹配，尺寸不一样直接返回false
        if (rows != cvMat.rows || cols != cvMat.cols) {
            std::cout << "wrong cvmat size\n";
            return false;
        }
        // 双重循环遍历矩阵每一个元素
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                // 判断cvMat元素是否超出eigMat元素±epsilon容错区间
                if ((cvMat.at<float>(i, j) > (eigMat(i, j) + epsilon)) ||
                    (cvMat.at<float>(i, j) < (eigMat(i, j) - epsilon))) {
                    std::cout << "cv mat:\n" << cvMat << std::endl; // 打印opencv矩阵便于调试
                    std::cout << "eig mat:\n" << eigMat << std::endl; // 打印eigen矩阵便于调试
                    return false; // 存在元素误差超限，矩阵不一致返回false
                }
        return true; // 全部元素误差均在阈值内，矩阵数据一致
    }

    // 模板函数：校验两个Eigen矩阵数据是否近似相等，支持任意标量类型T，指定行列rows、cols
    template <typename T, int rows, int cols>
    static bool CheckMatrices(const Eigen::Matrix<T, rows, cols> &eigMat1,
                              const Eigen::Matrix<T, rows, cols> &eigMat2) {
        const float epsilon = 1e-3; // 数值比对容错阈值
        // 遍历矩阵全部元素
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                // 判断eigMat1元素是否超出eigMat2±epsilon容错区间
                if ((eigMat1(i, j) > (eigMat2(i, j) + epsilon)) ||
                    (eigMat1(i, j) < (eigMat2(i, j) - epsilon))) {
                    std::cout << "eig mat 1:\n" << eigMat1 << std::endl; // 输出第一个矩阵调试
                    std::cout << "eig mat 2:\n" << eigMat2 << std::endl; // 输出第二个矩阵调试
                    return false; // 元素不一致返回false
                }
        return true; // 全部元素误差符合要求，矩阵相等
    }
};

}  // namespace ORB_SLAM3
