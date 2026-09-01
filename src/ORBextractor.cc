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
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// ORB特征提取器类头文件
#include "ORBextractor.h"
// STL通用算法，包含min、max、排序等
#include <algorithm>
// 标准输入输出流，用于调试打印
#include <iostream>
// STL双向链表容器
#include <list>
// OpenCV核心模块，Mat矩阵、基础运算
#include <opencv2/core/core.hpp>
// OpenCV 2D特征模块，FAST角点、KeyPoint、特征过滤
#include <opencv2/features2d/features2d.hpp>
// OpenCV高层GUI模块，图像显示
#include <opencv2/highgui/highgui.hpp>
// OpenCV图像处理模块，缩放、模糊、边界填充
#include <opencv2/imgproc/imgproc.hpp>
// STL pair键值对工具
#include <utility>
// STL动态数组容器
#include <vector>

// ORB-SLAM3算法命名空间
namespace ORB_SLAM3 {

// 引入OpenCV常用类型，省略cv::前缀
using cv::InputArray;       // 输入数组参数封装类
using cv::KeyPoint;       // 2D特征点结构体（坐标、角度、响应、层级、尺寸）
using cv::KeyPointsFilter;// 特征点过滤工具类
using cv::Mat;            // OpenCV图像矩阵类
using cv::OutputArray;      // 输出数组参数封装类
using cv::Point;          // 整数像素坐标点
using cv::Point2f;        // 浮点像素坐标点
using cv::Rect;           // 矩形区域
using cv::Size;           // 尺寸结构（宽、高）

// 引入STL常用容器，省略std::前缀
using std::list;           // 双向链表
using std::make_pair;     // 构造pair对象
using std::pair;           // 键值对模板
using std::vector;         // 动态数组

// ORB描述子采样patch尺寸：31x31像素
const int PATCH_SIZE = 31;
// patch半径，15像素
const int HALF_PATCH_SIZE = 15;
// 边缘安全阈值，特征点距离图像边缘至少19像素，避免越界访问
const int EDGE_THRESHOLD = 19;

/**
 * @brief 计算特征点邻域灰度质心角度，实现ORB旋转不变性
 * @param image 输入单通道灰度图像
 * @param pt 特征点亚像素坐标
 * @param u_max 圆形邻域每行水平半宽数组
 * @return 特征点主方向（弧度）
 */
static float IC_Angle(const Mat& image, Point2f pt, const vector<int>& u_max)
{
    // 图像一阶矩：m_01为y方向矩，m_10为x方向矩
    int m_01 = 0, m_10 = 0;

    // 取特征点中心像素地址，cvRound将浮点坐标取整到整像素
    const uchar* center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    // 单独处理中心行（v=0），仅累加x方向矩
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circular patch
    // 图像每行字节步长，用于跨行指针偏移
    int step = static_cast<int>(image.step1());
    // 遍历v从1到半patch高度，同时处理正负v对称的两行
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        // 当前v对应的圆形水平半宽
        int d = u_max[v];
        // 遍历u从-d到d，同时计算+v行和-v行
        for (int u = -d; u <= d; ++u)
        {
            // 正v行像素、负v行像素
            int val_plus = center[u + v * step], val_minus = center[u - v * step];
            // 两行灰度差用于计算y方向矩m_01
            v_sum += (val_plus - val_minus);
            // 两行灰度和用于计算x方向矩m_10
            m_10 += u * (val_plus + val_minus);
        }
        // 累加y方向一阶矩，v为权重
        m_01 += v * v_sum;
    }

    // 快速反正切计算质心角度，即特征点主方向
    return cv::fastAtan2(static_cast<float>(m_01), static_cast<float>(m_10));
}

// 角度转弧度系数：π/180
const float factorPI = static_cast<float>(CV_PI / 180.f);

/**
 * @brief 计算单个特征点的rBRIEF二进制描述子
 * @param kpt 输入特征点
 * @param img 输入灰度图像
 * @param pattern 预定义采样点对数组首地址
 * @param desc 输出描述子指针（32字节）
 */
static void computeOrbDescriptor(const KeyPoint& kpt, const Mat& img,
                                 const Point* pattern, uchar* desc)
{
    // 特征点角度转弧度
    float angle = static_cast<float>(kpt.angle) * factorPI;
    // 旋转矩阵元素：a=cosθ，b=sinθ
    float a = static_cast<float>(cos(angle)), b = static_cast<float>(sin(angle));

    // 特征点中心像素地址
    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    // 图像行步长
    const int step = static_cast<int>(img.step);

    // 宏：获取第idx个采样点旋转后的灰度值
    // 旋转公式：x' = x*cosθ - y*sinθ；y' = x*sinθ + y*cosθ
    #define GET_VALUE(idx)                                             \
    center[cvRound(pattern[idx].x * b + pattern[idx].y * a) * step + \
        cvRound(pattern[idx].x * a - pattern[idx].y * b)]

    // 生成32字节描述子，每次处理16个点对（2字节）
    for (int i = 0; i < 32; ++i, pattern += 16)
    {
        int t0, t1, val;
        // 第0对点比较，第0位
        t0 = GET_VALUE(0);
        t1 = GET_VALUE(1);
        val = t0 < t1;
        // 第1对点比较，第1位
        t0 = GET_VALUE(2);
        t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        // 第2对点比较，第2位
        t0 = GET_VALUE(4);
        t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        // 第3对点比较，第3位
        t0 = GET_VALUE(6);
        t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        // 第4对点比较，第4位
        t0 = GET_VALUE(8);
        t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        // 第5对点比较，第5位
        t0 = GET_VALUE(10);
        t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        // 第6对点比较，第6位
        t0 = GET_VALUE(12);
        t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        // 第7对点比较，第7位
        t0 = GET_VALUE(14);
        t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        // 写入1字节描述子
        desc[i] = static_cast<uchar>(val);
    }

    // 取消宏定义，避免命名空间污染
    #undef GET_VALUE
}

// ORB rBRIEF预定义采样模式：256个点对，每个点对(x1,y1,x2,y2)
// 附带每组点对的灰度均值、相关性统计，用于学习得到高判别力模式
static int bit_pattern_31_[256 * 4] = {
    8,   -3,  9,   5 /*mean (0), correlation (0)*/,
    4,   2,   7,   -12 /*mean (1.12461e-05), correlation (0.0437584)*/,
    -11, 9,   -8,  2 /*mean (3.37382e-05), correlation (0.0617409)*/,
    7,   -12, 12,  -13 /*mean (5.62303e-05), correlation (0.0636977)*/,
    2,   -13, 2,   12 /*mean (0.00134953), correlation (0.085099)*/,
    1,   -7,  1,   6 /*mean (0.00528565), correlation (0.0857175)*/,
    -2,  -10, -2,  -4 /*mean (0.0188821), correlation (0.0985774)*/,
    -13, -13, -11, -8 /*mean (0.0363135), correlation (0.0899616)*/,
    -13, -3,  -12, -9 /*mean (0.121806), correlation (0.099849)*/,
    10,  4,   11,  9 /*mean (0.122065), correlation (0.093285)*/,
    -13, -8,  -8,  -9 /*mean (0.162787), correlation (0.0942748)*/,
    -11, 7,   -9,  12 /*mean (0.21561), correlation (0.0974438)*/,
    7,   7,   12,  6 /*mean (0.160583), correlation (0.130064)*/,
    -4,  -5,  -3,  0 /*mean (0.228171), correlation (0.132998)*/,
    -13, 2,   -12, -3 /*mean (0.00997526), correlation (0.145926)*/,
    -9,  0,   -7,  5 /*mean (0.198234), correlation (0.143636)*/,
    12,  -6,  12,  -1 /*mean (0.0676226), correlation (0.16689)*/,
    -3,  6,   -2,  12 /*mean (0.166847), correlation (0.171682)*/,
    -6,  -13, -4,  -8 /*mean (0.101215), correlation (0.179716)*/,
    11,  -13, 12,  -8 /*mean (0.200641), correlation (0.192279)*/,
    4,   7,   5,   1 /*mean (0.205106), correlation (0.186848)*/,
    5,   -3,  10,  -3 /*mean (0.234908), correlation (0.192319)*/,
    3,   -7,  6,   12 /*mean (0.0709964), correlation (0.210872)*/,
    -8,  -7,  -6,  -2 /*mean (0.0939834), correlation (0.212589)*/,
    -2,  11,  -1,  -10 /*mean (0.127778), correlation (0.20866)*/,
    -13, 12,  -8,  10 /*mean (0.14783), correlation (0.206356)*/,
    -7,  3,   -5,  -3 /*mean (0.182141), correlation (0.198942)*/,
    -4,  2,   -3,  7 /*mean (0.188237), correlation (0.21384)*/,
    -10, -12, -6,  11 /*mean (0.14865), correlation (0.23571)*/,
    5,   -12, 6,   -7 /*mean (0.22312), correlation (0.23324)*/,
    5,   -6,  7,   -1 /*mean (0.229082), correlation (0.23389)*/,
    1,   0,   4,   -5 /*mean (0.241577), correlation (0.215286)*/,
    9,   11,  11,  -13 /*mean (0.00338507), correlation (0.251373)*/,
    4,   7,   4,   12 /*mean (0.131005), correlation (0.257622)*/,
    2,   -1,  4,   4 /*mean (0.152755), correlation (0.255205)*/,
    -4,  -12, -2,  7 /*mean (0.182771), correlation (0.244867)*/,
    -8,  -5,  -7,  -10 /*mean (0.186898), correlation (0.23901)*/,
    4,   11,  9,   12 /*mean (0.226226), correlation (0.258255)*/,
    0,   -8,  1,   -13 /*mean (0.0897886), correlation (0.274827)*/,
    -13, -2,  -8,  2 /*mean (0.148774), correlation (0.28065)*/,
    -3,  -2,  -2,  3 /*mean (0.153048), correlation (0.283063)*/,
    -6,  9,   -4,  -9 /*mean (0.169523), correlation (0.278248)*/,
    8,   12,  10,  7 /*mean (0.225337), correlation (0.282851)*/,
    0,   9,   1,   3 /*mean (0.226687), correlation (0.278734)*/,
    7,   -5,  11,  -10 /*mean (0.00693882), correlation (0.305161)*/,
    -13, -6,  -11, 0 /*mean (0.0227283), correlation (0.300181)*/,
    10,  7,   12,  1 /*mean (0.125517), correlation (0.31089)*/,
    -6,  -3,  -6,  12 /*mean (0.131748), correlation (0.312779)*/,
    10,  -9,  12,  -4 /*mean (0.144827), correlation (0.292797)*/,
    -13, 8,   -8,  -12 /*mean (0.149202), correlation (0.308918)*/,
    -13, 0,   -8,  -4 /*mean (0.160909), correlation (0.310013)*/,
    3,   3,   7,   8 /*mean (0.177755), correlation (0.309394)*/,
    5,   7,   10,  -7 /*mean (0.212337), correlation (0.310315)*/,
    -1,  7,   1,   -12 /*mean (0.214429), correlation (0.311933)*/,
    3,   -10, 5,   6 /*mean (0.235807), correlation (0.313104)*/,
    2,   -4,  3,   -10 /*mean (0.00494827), correlation (0.344948)*/,
    -13, 0,   -13, 5 /*mean (0.0549145), correlation (0.344675)*/,
    -13, -7,  -12, 12 /*mean (0.103385), correlation (0.342715)*/,
    -13, 3,   -11, 8 /*mean (0.134222), correlation (0.322922)*/,
    -7,  12,  -4,  7 /*mean (0.153284), correlation (0.337061)*/,
    6,   -10, 12,  8 /*mean (0.154881), correlation (0.329257)*/,
    -9,  -1,  -7,  -6 /*mean (0.200967), correlation (0.33312)*/,
    -2,  -5,  0,   12 /*mean (0.201518), correlation (0.340635)*/,
    -12, 5,   -7,  5 /*mean (0.207805), correlation (0.335631)*/,
    3,   -10, 8,   -13 /*mean (0.224438), correlation (0.34504)*/,
    -7,  -7,  -4,  5 /*mean (0.239361), correlation (0.338053)*/,
    -3,  -2,  -1,  -7 /*mean (0.240744), correlation (0.344322)*/,
    2,   9,   5,   -11 /*mean (0.242949), correlation (0.34145)*/,
    -11, -13, -5,  -13 /*mean (0.244028), correlation (0.336861)*/,
    -1,  6,   0,   -1 /*mean (0.247571), correlation (0.343684)*/,
    5,   -3,  5,   2 /*mean (0.00697256), correlation (0.357265)*/,
    -4,  -13, -4,  12 /*mean (0.00213675), correlation (0.373827)*/,
    -9,  -6,  -9,  6 /*mean (0.0126856), correlation (0.373938)*/,
    -12, -10, -8,  -4 /*mean (0.0152497), correlation (0.364237)*/,
    10,  2,   12,  -3 /*mean (0.0299933), correlation (0.345292)*/,
    7,   12,  12,  12 /*mean (0.0307242), correlation (0.366299)*/,
    -7,  -13, -6,  5 /*mean (0.0534975), correlation (0.368357)*/,
    -4,  9,   -3,  4 /*mean (0.099865), correlation (0.372276)*/,
    7,   -1,  12,  2 /*mean (0.117083), correlation (0.364529)*/,
    -7,  6,   -5,  1 /*mean (0.126125), correlation (0.369606)*/,
    -13, 11,  -12, 5 /*mean (0.130364), correlation (0.358502)*/,
    -3,  7,   -2,  -6 /*mean (0.131691), correlation (0.375531)*/,
    7,   -8,  12,  -7 /*mean (0.160166), correlation (0.379508)*/,
    -13, -7,  -11, -12 /*mean (0.167848), correlation (0.353343)*/,
    1,   -3,  12,  12 /*mean (0.183378), correlation (0.371916)*/,
    2,   -6,  3,   0 /*mean (0.228711), correlation (0.371761)*/,
    -4,  3,   -2,  -13 /*mean (0.247211), correlation (0.364063)*/,
    -1,  -13, 1,   9 /*mean (0.249325), correlation (0.378139)*/,
    7,   1,   8,   -6 /*mean (0.00652272), correlation (0.411682)*/,
    1,   -1,  3,   12 /*mean (0.00248538), correlation (0.392988)*/,
    9,   1,   12,  6 /*mean (0.0206815), correlation (0.386106)*/,
    -1,  -9,  -1,  3 /*mean (0.0364485), correlation (0.410752)*/,
    -13, -13, -10, 5 /*mean (0.0376068), correlation (0.398374)*/,
    7,   7,   10,  12 /*mean (0.0424202), correlation (0.405663)*/,
    12,  -5,  12,  9 /*mean (0.0942645), correlation (0.410422)*/,
    6,   3,   7,   11 /*mean (0.1074), correlation (0.413224)*/,
    5,   -13, 6,   10 /*mean (0.109256), correlation (0.408646)*/,
    2,   -12, 2,   3 /*mean (0.131691), correlation (0.416076)*/,
    3,   8,   4,   -6 /*mean (0.165081), correlation (0.417569)*/,
    2,   6,   12,  -13 /*mean (0.171874), correlation (0.408471)*/,
    9,   -12, 10,  3 /*mean (0.175146), correlation (0.41296)*/,
    -8,  4,   -7,  9 /*mean (0.183682), correlation (0.402956)*/,
    -11, 12,  -4,  -6 /*mean (0.184672), correlation (0.416125)*/,
    1,   12,  2,   -8 /*mean (0.191487), correlation (0.386696)*/,
    6,   -9,  7,   -4 /*mean (0.192668), correlation (0.394771)*/,
    2,   3,   3,   -2 /*mean (0.200157), correlation (0.408303)*/,
    6,   3,   11,  0 /*mean (0.204588), correlation (0.411762)*/,
    3,   -3,  8,   -8 /*mean (0.205904), correlation (0.416294)*/,
    7,   8,   9,   3 /*mean (0.213237), correlation (0.409306)*/,
    -11, -5,  -6,  -4 /*mean (0.243444), correlation (0.395069)*/,
    -10, 11,  -5,  10 /*mean (0.247672), correlation (0.413392)*/,
    -5,  -8,  -3,  12 /*mean (0.24774), correlation (0.411416)*/,
    -10, 5,   -9,  0 /*mean (0.00213675), correlation (0.454003)*/,
    8,   -1,  12,  -6 /*mean (0.0293635), correlation (0.455368)*/,
    4,   -6,  6,  -11 /*mean (0.0404971), correlation (0.457393)*/,
    -10, 12,  -8,  7 /*mean (0.0481107), correlation (0.448364)*/,
    4,   -2,  6,   7 /*mean (0.050641), correlation (0.455019)*/,
    -2,  0,   -2,  12 /*mean (0.0525978), correlation (0.44338)*/,
    -5,  -8,  -5,  2 /*mean (0.0629667), correlation (0.457096)*/,
    7,   -6,  10,  12 /*mean (0.0653846), correlation (0.445623)*/,
    -9,  -13, -8,  -8 /*mean (0.0858749), correlation (0.449789)*/,
    -5,  -13, -5,  -2 /*mean (0.122402), correlation (0.450201)*/,
    8,   -8,  9,   -13 /*mean (0.125416), correlation (0.453224)*/,
    -9,  -11, -9,  0 /*mean (0.130128), correlation (0.458724)*/,
    1,   -8,  1,   -2 /*mean (0.132467), correlation (0.440133)*/,
    7,   -4,  9,   1 /*mean (0.132692), correlation (0.454)*/,
    -2,  1,   -1,  -4 /*mean (0.135695), correlation (0.455739)*/,
    11,  -6,  12,  -11 /*mean (0.142904), correlation (0.446114)*/,
    -12, -9,  -6,  4 /*mean (0.146165), correlation (0.451473)*/,
    3,   7,   7,   12 /*mean (0.147627), correlation (0.456643)*/,
    5,   5,   10,  8 /*mean (0.152901), correlation (0.455036)*/,
    0,   -4,  2,   8 /*mean (0.167083), correlation (0.459315)*/,
    -9,  12,  -5,  -13 /*mean (0.173234), correlation (0.454706)*/,
    0,   7,   2,   12 /*mean (0.18312), correlation (0.433855)*/,
    -1,  2,   1,   7 /*mean (0.185504), correlation (0.443838)*/,
    5,   11,  7,   -9 /*mean (0.185706), correlation (0.451123)*/,
    3,   5,   6,   -8 /*mean (0.188968), correlation (0.455808)*/,
    -13, -4,  -8,  9 /*mean (0.191667), correlation (0.459128)*/,
    -5,  9,   -3,  -3 /*mean (0.193196), correlation (0.458364)*/,
    -4,  -7,  -3,  -12 /*mean (0.196536), correlation (0.455782)*/,
    6,   5,   8,   0 /*mean (0.1972), correlation (0.450481)*/,
    -7,  6,   -6,  12 /*mean (0.199438), correlation (0.458156)*/,
    -13, 6,   -5,  -2 /*mean (0.211224), correlation (0.449548)*/,
    1,   -10, 3,   10 /*mean (0.211718), correlation (0.440606)*/,
    4,   1,   8,   -4 /*mean (0.213034), correlation (0.443177)*/,
    -2,  -2,  2,   -13 /*mean (0.234334), correlation (0.455304)*/,
    2,   -12, 12,  12 /*mean (0.235684), correlation (0.443436)*/,
    -2,  -13, 0,   -6 /*mean (0.237674), correlation (0.452525)*/,
    4,   1,   9,   3 /*mean (0.23962), correlation (0.444824)*/,
    -6,  -10, -3,  -5 /*mean (0.248459), correlation (0.439621)*/,
    -3,  -13, -1,  1 /*mean (0.249505), correlation (0.456666)*/,
    7,   5,   12,  -11 /*mean (0.00119208), correlation (0.495466)*/,
    4,   -2,  5,   -7 /*mean (0.00372245), correlation (0.484214)*/,
    -13, 9,   -9,  -5 /*mean (0.00741116), correlation (0.499854)*/,
    7,   1,   8,   6 /*mean (0.0208952), correlation (0.499773)*/,
    7,   -8,  7,   6 /*mean (0.0220085), correlation (0.501609)*/,
    -7,  -4,  -7,  1 /*mean (0.0233806), correlation (0.496568)*/,
    -8,  11,  -7,  -8 /*mean (0.0236505), correlation (0.489719)*/,
    -13, 6,   -12, -8 /*mean (0.0268781), correlation (0.503487)*/,
    2,   4,   3,   9 /*mean (0.0323324), correlation (0.501938)*/,
    10,  -5,  12,  3 /*mean (0.0399235), correlation (0.494029)*/,
    -6,  -5,  -6,  7 /*mean (0.0420153), correlation (0.486579)*/,
    8,   -3,  9,   -8 /*mean (0.0548021), correlation (0.484237)*/,
    2,   -12, 2,   8 /*mean (0.0616622), correlation (0.496642)*/,
    -11, -2,  -10, 3 /*mean (0.0627755), correlation (0.498563)*/,
    -12, -13, -7,  -9 /*mean (0.0829622), correlation (0.495491)*/,
    -11, 0,   -10, -5 /*mean (0.0843342), correlation (0.487146)*/,
    5,   -3,  11,  8 /*mean (0.0929937), correlation (0.502315)*/,
    -2,  -13, -1,  12 /*mean (0.113327), correlation (0.48941)*/,
    -1,  -8,  0,   9 /*mean (0.132119), correlation (0.467268)*/,
    -13, -11, -12, -5 /*mean (0.136269), correlation (0.498771)*/,
    -10, -2,  -10, 11 /*mean (0.142173), correlation (0.498714)*/,
    -3,  9,   -2,  -13 /*mean (0.144141), correlation (0.491973)*/,
    2,   -3,  3,   2 /*mean (0.14892), correlation (0.500782)*/,
    -9,  -13, -4,  0 /*mean (0.150371), correlation (0.498211)*/,
    -4,  6,   -3,  -10 /*mean (0.152159), correlation (0.495547)*/,
    -4,  12,  -2,  -7 /*mean (0.156152), correlation (0.496925)*/,
    -6,  -11, -4,  9 /*mean (0.15749), correlation (0.499222)*/,
    6,   -3,  6,   11 /*mean (0.159211), correlation (0.503821)*/,
    -13, 11,  -5,  5 /*mean (0.162427), correlation (0.501907)*/,
    11,  11,  12,  6 /*mean (0.16652), correlation (0.497632)*/,
    7,   -5,  12,  -2 /*mean (0.169141), correlation (0.484474)*/,
    -1,  12,  0,   7 /*mean (0.169456), correlation (0.495339)*/,
    -4,  -8,  -3,  -2 /*mean (0.171457), correlation (0.487251)*/,
    -7,  1,   -6,  7 /*mean (0.175), correlation (0.500024)*/,
    -13, -12, -8,  -13 /*mean (0.175866), correlation (0.497523)*/,
    -7,  -2,  -6,  -8 /*mean (0.178273), correlation (0.501854)*/,
    -8,  5,   -6,  -9 /*mean (0.181107), correlation (0.494888)*/,
    -5,  -1,  -4,  5 /*mean (0.190227), correlation (0.482557)*/,
    -13, 7,   -8,  10 /*mean (0.196739), correlation (0.496503)*/,
    1,   5,   5,   -13 /*mean (0.19973), correlation (0.499759)*/,
    1,   0,   10,  -13 /*mean (0.204465), correlation (0.49873)*/,
    9,   12,  10,  -1 /*mean (0.209334), correlation (0.49063)*/,
    5,   -8,  10,  -9 /*mean (0.211134), correlation (0.503011)*/,
    -1,  11,  1,   -13 /*mean (0.212), correlation (0.499414)*/,
    -9,  -3,  -6,  2 /*mean (0.212168), correlation (0.480739)*/,
    -1,  -10, 1,   12 /*mean (0.212731), correlation (0.502523)*/,
    -13, 1,   -8,  -10 /*mean (0.21327), correlation (0.489786)*/,
    8,   -11, 10,  -6 /*mean (0.214159), correlation (0.488246)*/,
    2,   -13, 3,   -6 /*mean (0.216993), correlation (0.50287)*/,
    7,   -13, 12,  -9 /*mean (0.223639), correlation (0.470502)*/,
    -10, -10, -5,  -7 /*mean (0.224089), correlation (0.500852)*/,
    -10, -8,  -8,  -13 /*mean (0.228666), correlation (0.502629)*/,
    4,   -6,  8,  5 /*mean (0.22906), correlation (0.498305)*/,
    3,   12,  8,  -13 /*mean (0.233378), correlation (0.503825)*/,
    -4,  2,  -3,  -3 /*mean (0.234323), correlation (0.476692)*/,
    5,   -13, 10,  -12 /*mean (0.236392), correlation (0.475462)*/,
    4,   -13, 5,   -1 /*mean (0.236842), correlation (0.504132)*/,
    -9,  9,   -4,  3 /*mean (0.236977), correlation (0.497739)*/,
    0,   3,   3,  -9 /*mean (0.24314), correlation (0.499398)*/,
    -12, 1,   -6,  1 /*mean (0.243297), correlation (0.489447)*/,
    3,   2,   4,  -8 /*mean (0.00155196), correlation (0.553496)*/,
    -10, -10, -10, 9 /*mean (0.00239541), correlation (0.54297)*/,
    8,   -13, 12,  12 /*mean (0.0034413), correlation (0.544361)*/,
    -8,  -12, -6,  -5 /*mean (0.003565), correlation (0.551225)*/,
    2,   2,   3,   7 /*mean (0.00835583), correlation (0.55285)*/,
    10,  6,   11,  -8 /*mean (0.00885065), correlation (0.540913)*/,
    6,   8,   8,   -12 /*mean (0.0101552), correlation (0.551085)*/,
    -7,  10,  -6,  5 /*mean (0.0102227), correlation (0.533635)*/,
    -3,  -9,  -3,  9 /*mean (0.0110211), correlation (0.543121)*/,
    -1,  -13, -1,  5 /*mean (0.0113473), correlation (0.550173)*/,
    -3,  -7,  -3,  4 /*mean (0.0140913), correlation (0.554774)*/,
    -8,  -2,  -8,  3 /*mean (0.017049), correlation (0.55461)*/,
    4,   2,   12,  12 /*mean (0.01778), correlation (0.546921)*/,
    2,   -5,  3,   11 /*mean (0.0224022), correlation (0.549667)*/,
    6,   -9,  11,  -13 /*mean (0.029161), correlation (0.546295)*/,
    3,   -1,  7,  12 /*mean (0.0303081), correlation (0.548599)*/,
    11,  -1,  12,  4 /*mean (0.0355151), correlation (0.523943)*/,
    -3,  0,   -3,  6 /*mean (0.0417904), correlation (0.543395)*/,
    4,   -11, 4,   12 /*mean (0.0487292), correlation (0.542818)*/,
    2,   -4,  2,   1 /*mean (0.0575124), correlation (0.554888)*/,
    -10, -6,  -8,  1 /*mean (0.0594242), correlation (0.544026)*/,
    -13, 7,   -11, 1 /*mean (0.0597391), correlation (0.550524)*/,
    -13, 12,  -11, -13 /*mean (0.0608974), correlation (0.55383)*/,
    6,   0,   11,  -13 /*mean (0.065126), correlation (0.552006)*/,
    0,   -1,  1,   4 /*mean (0.074224), correlation (0.546372)*/,
    -13, 3,   -9,  -2 /*mean (0.0808592), correlation (0.554875)*/,
    -9,  8,   -6,  -3 /*mean (0.0883378), correlation (0.551178)*/,
    -13, -6,  -8,  -2 /*mean (0.0901035), correlation (0.548446)*/,
    5,   -9,  8,   10 /*mean (0.0949843), correlation (0.554694)*/,
    2,   7,   3,  -9 /*mean (0.0994152), correlation (0.550979)*/,
    -1,  -6,  -1,  -1 /*mean (0.10045), correlation (0.552714)*/,
    9,   5,   11,  -2 /*mean (0.100686), correlation (0.552594)*/,
    11,  -3,  12,  -8 /*mean (0.101091), correlation (0.532394)*/,
    3,   0,   3,   5 /*mean (0.101147), correlation (0.525576)*/,
    -1,  4,   0,   10 /*mean (0.105263), correlation (0.531498)*/,
    3,   -6,  4,   5 /*mean (0.110785), correlation (0.540491)*/,
    -13, 0,   -10, 5 /*mean (0.112798), correlation (0.536582)*/,
    5,   8,   12,  11 /*mean (0.114181), correlation (0.55793)*/,
    8,   9,   9,   -6 /*mean (0.117431), correlation (0.553763)*/,
    7,   -4,  8,  -12 /*mean (0.118522), correlation (0.553452)*/,
    -10, 4,   -10, 9 /*mean (0.12094), correlation (0.554785)*/,
    7,   3,   12,  4 /*mean (0.122582), correlation (0.55825)*/,
    9,   -7,  10,  -2 /*mean (0.124978), correlation (0.549846)*/,
    7,   0,   12,  -2 /*mean (0.127002), correlation (0.537452)*/,
    -1,  -6,  0,  -11 /*mean (0.127148), correlation (0.547401)*/
};

/**
 * @brief ORB特征提取器构造函数
 * @param _nfeatures 总特征点数量
 * @param _scaleFactor 金字塔层间尺度因子
 * @param _nlevels 图像金字塔层数
 * @param _iniThFAST FAST角点初始阈值
 * @param _minThFAST FAST角点最低阈值
 */
ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                           int _iniThFAST, int _minThFAST)
    : nfeatures(_nfeatures),       // 总特征点数
      scaleFactor(_scaleFactor), // 金字塔层间缩放因子
      nlevels(_nlevels),       // 金字塔层数
      iniThFAST(_iniThFAST),   // FAST初始阈值
      minThFAST(_minThFAST)    // FAST最低阈值
{
    // 每层缩放因子数组
    mvScaleFactor.resize(nlevels);
    // 每层尺度sigma平方数组
    mvLevelSigma2.resize(nlevels);
    // 第0层缩放因子1.0
    mvScaleFactor[0] = 1.0f;
    mvLevelSigma2[0] = 1.0f;
    // 逐层计算缩放因子，等比数列
    for (int i = 1; i < nlevels; i++)
    {
        mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
        mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
    }

    // 逆缩放因子数组
    mvInvScaleFactor.resize(nlevels);
    // 逆sigma平方数组
    mvInvLevelSigma2.resize(nlevels);
    // 逐层计算倒数
    for (int i = 0; i < nlevels; i++)
    {
        mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
        mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
    }

    // 图像金字塔容器
    mvImagePyramid.resize(nlevels);
    // 每层特征点数量
    mnFeaturesPerLevel.resize(nlevels);

    // 按对数分配每层特征数，低层多高层少
    float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale =
        nfeatures * (1 - factor) /
        (1 - static_cast<float>(
            pow(static_cast<double>(factor), static_cast<double>(nlevels))));

    int sumFeatures = 0;
    // 前n-1层按比例分配
    for (int level = 0; level < nlevels - 1; level++)
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    // 最后一层分配剩余特征数
    mnFeaturesPerLevel[nlevels - 1] = std::max(nfeatures - sumFeatures, 0);

    // 采样点总数
    const int npoints = 512;
    // 整型数组转Point指针
    const Point* pattern0 = reinterpret_cast<const Point*>(bit_pattern_31_);
    // 复制pattern到成员变量尾部
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    // This is for orientation
    // 预计算圆形patch每行u_max数组
    umax.resize(HALF_PATCH_SIZE + 1);

    int v, v0;
    // 圆形patch最大v值
    int vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    // 半径平方
    const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
    // 计算每个v对应的最大u
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));

    // Make sure we are symmetric
    // 对称填充下半部分u_max
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1]) ++v0;
        umax[v] = v0;
        ++v0;
    }
}

/**
 * @brief 批量计算所有特征点主方向
 * @param image 输入图像
 * @param keypoints 特征点向量（角度被赋值）
 * @param umax 圆形邻域半宽数组
 */
static void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints,
                               const vector<int>& umax)
{
    // 遍历所有特征点
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                                   keypointEnd = keypoints.end();
        keypoint != keypointEnd; ++keypoint)
    {
        // 调用IC_Angle计算灰度质心角度
        keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
}

/**
 * @brief 八叉树节点四分割函数
 * @param n1 左上子节点
 * @param n2 右上子节点
 * @param n3 左下子节点
 * @param n4 右下子节点
 */
void ExtractorNode::DivideNode(ExtractorNode& n1, ExtractorNode& n2,
                            ExtractorNode& n3, ExtractorNode& n4)
{
    // 子节点半宽，向上取整
    const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
    // 子节点半高，向上取整
    const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

    // Define boundaries of childs
    // 左上子节点边界
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x + halfX, UL.y);
    n1.BL = cv::Point2i(UL.x, UL.y + halfY);
    n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
    n1.vKeys.reserve(vKeys.size());

    // 右上子节点边界
    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x, UL.y + halfY);
    n2.vKeys.reserve(vKeys.size());

    // 左下子节点边界
    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x, BL.y);
    n3.vKeys.reserve(vKeys.size());

    // 右下子节点边界
    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    // Associate points to childs
    // 按坐标分配特征点到四个子节点
    for (size_t i = 0; i < vKeys.size(); i++)
    {
        const cv::KeyPoint& kp = vKeys[i];
        // x小于中线
        if (kp.pt.x < n1.UR.x)
        {
            // y小于中线 → 左上
            if (kp.pt.y < n1.BR.y)
                n1.vKeys.push_back(kp);
            // y大于等于中线 → 左下
            else
                n3.vKeys.push_back(kp);
        }
        else if (kp.pt.y < n1.BR.y)
        {
            // y小于中线 → 右上
            n2.vKeys.push_back(kp);
        }
        else
        {
            // y大于等于中线 → 右下
            n4.vKeys.push_back(kp);
        }
    }

    // 子节点只有1个点时标记不再分割
    if (n1.vKeys.size() == 1) n1.bNoMore = true;
    if (n2.vKeys.size() == 1) n2.bNoMore = true;
    if (n3.vKeys.size() == 1) n3.bNoMore = true;
    if (n4.vKeys.size() == 1) n4.bNoMore = true;
}

/**
 * @brief 八叉树节点比较函数，用于排序
 * @param e1 节点对1
 * @param e2 节点对2
 * @return e1是否排在e2前
 */
static bool compareNodes(pair<int, ExtractorNode*>& e1,
                         pair<int, ExtractorNode*>& e2)
{
    // 先按特征点数升序
    if (e1.first < e2.first)
    {
        return true;
    }
    else if (e1.first > e2.first)
    {
        return false;
    }
    else
    {
        // 数量相同按x坐标升序
        if (e1.second->UL.x < e2.second->UL.x)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

/**
 * @brief 八叉树均匀化特征点分布
 * @param vToDistributeKeys 待分布特征点
 * @param minX 左边界
 * @param maxX 右边界
 * @param minY 上边界
 * @param maxY 下边界
 * @param N 期望保留特征点数
 * @param level 金字塔层级
 * @return 均匀分布后的特征点
 */
vector<cv::KeyPoint> ORBextractor::DistributeOctTree(
    const vector<cv::KeyPoint>& vToDistributeKeys, const int& minX,
    const int& maxX, const int& minY, const int& maxY, const int& N,
    const int& level)
{
    // Compute how many initial nodes
    // 初始节点数，按宽高比
    const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));
    const float hX = static_cast<float>(maxX - minX) / nIni;

    list<ExtractorNode> lNodes;       // 节点链表
    vector<ExtractorNode*> vpIniNodes; // 初始节点指针数组
    vpIniNodes.resize(nIni);

    // 初始化初始节点
    for (int i = 0; i < nIni; i++)
    {
        ExtractorNode ni;
        ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);
        ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0);
        ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
        ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    // Associate points to childs
    // 分配特征点到初始节点
    for (size_t i = 0; i < vToDistributeKeys.size(); i++)
    {
        const cv::KeyPoint& kp = vToDistributeKeys[i];
        vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
    }

    list<ExtractorNode>::iterator lit = lNodes.begin();
    // 第一轮标记单节点、删除空节点
    while (lit != lNodes.end())
    {
        if (lit->vKeys.size() == 1)
        {
            lit->bNoMore = true;
            lit++;
        }
        else if (lit->vKeys.empty())
        {
            lit = lNodes.erase(lit);
        }
        else
        {
            lit++;
        }
    }

    bool bFinish = false;
    int iteration = 0;
    vector<pair<int, ExtractorNode*> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size() * 4);

    // 迭代分割节点
    while (!bFinish)
    {
        iteration++;
        int prevSize = lNodes.size();
        lit = lNodes.begin();
        int nToExpand = 0;
        vSizeAndPointerToNode.clear();

        // 遍历所有节点，可分割则四分割
        while (lit != lNodes.end())
        {
            if (lit->bNoMore)
            {
                // If node only contains one point do not subdivide and continue
                lit++;
                continue;
            }
            else
            {
                // If more than one point, subdivide
                ExtractorNode n1, n2, n3, n4;
                lit->DivideNode(n1, n2, n3, n4);

                // Add childs if they contain points
                // 左上子节点入链表
                if (n1.vKeys.size() > 0)
                {
                    lNodes.push_front(n1);
                    if (n1.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(n1.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                // 右上子节点入链表
                if (n2.vKeys.size() > 0)
                {
                    lNodes.push_front(n2);
                    if (n2.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(n2.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                // 左下子节点入链表
                if (n3.vKeys.size() > 0)
                {
                    lNodes.push_front(n3);
                    if (n3.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(n3.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                // 右下子节点入链表
                if (n4.vKeys.size() > 0)
                {
                    lNodes.push_front(n4);
                    if (n4.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(n4.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit = lNodes.erase(lit);
                continue;
            }
        }

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        // 节点数达标或不再增加，结束
        if (static_cast<int>(lNodes.size()) >= N ||
            static_cast<int>(lNodes.size()) == prevSize)
        {
            bFinish = true;
        }
        else if ((static_cast<int>(lNodes.size()) + nToExpand * 3) > N)
        {
            // 预测超量则按大小排序，优先分割大节点
            while (!bFinish)
            {
                prevSize = lNodes.size();

                vector<pair<int, ExtractorNode*> > vPrevSizeAndPointerToNode =
                    vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end(),
                     compareNodes);

                // 从大到小分割
                for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--)
                {
                    ExtractorNode n1, n2, n3, n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

                    // Add childs if they contain points
                    if (n1.vKeys.size() > 0)
                    {
                        lNodes.push_front(n1);
                        if (n1.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                make_pair(n1.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n2.vKeys.size() > 0)
                    {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0)
                    {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0)
                    {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if (static_cast<int>(lNodes.size()) >= N) break;
                }

                if (static_cast<int>(lNodes.size()) >= N ||
                    static_cast<int>(lNodes.size()) == prevSize)
                    bFinish = true;
            }
        }
    }

    // Retain the best point in each node
    // 每个节点保留响应最大的特征点
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(nfeatures);
    for (list<ExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end();
         lit++)
    {
        vector<cv::KeyPoint>& vNodeKeys = lit->vKeys;
        cv::KeyPoint* pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        // 找节点内响应最大的点
        for (size_t k = 1; k < vNodeKeys.size(); k++)
        {
            if (vNodeKeys[k].response > maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}

/**
 * @brief 八叉树法提取每层特征点
 * @param allKeypoints 输出各层特征点
 */
void ORBextractor::ComputeKeyPointsOctTree(
    vector<vector<KeyPoint> >& allKeypoints)
{
    allKeypoints.resize(nlevels);

    const float W = 35; // 网格宽度

    // 逐层提取
    for (int level = 0; level < nlevels; ++level)
    {
        // 提取区域边界，留余量
        const int minBorderX = EDGE_THRESHOLD - 3;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
        const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;

        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nfeatures * 10);

        const float width = (maxBorderX - minBorderX);
        const float height = (maxBorderY - minBorderY);

        const int nCols = width / W;
        const int nRows = height / W;
        const int wCell = ceil(width / nCols);
        const int hCell = ceil(height / nRows);

        // 逐网格提取FAST角点
        for (int i = 0; i < nRows; i++)
        {
            const float iniY = minBorderY + i * hCell;
            float maxY = iniY + hCell + 6;

            if (iniY >= maxBorderY - 3) continue;
            if (maxY > maxBorderY) maxY = maxBorderY;

            for (int j = 0; j < nCols; j++)
            {
                const float iniX = minBorderX + j * wCell;
                float maxX = iniX + wCell + 6;
                if (iniX >= maxBorderX - 6) continue;
                if (maxX > maxBorderX) maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;

                // 初始阈值FAST提取
                FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                     vKeysCell, iniThFAST, true);

                /*if(bRight && j <= 13){
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                        vKeysCell,10,true);
                }
                else if(!bRight && j >= 16){
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                        vKeysCell,10,true);
                }
                else{
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                        vKeysCell,iniThFAST,true);
                }*/

                // 点太少则降低阈值再提取
                if (vKeysCell.empty())
                {
                    FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                         vKeysCell, minThFAST, true);
                    /*if(bRight && j <= 13){
                        FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                            vKeysCell,5,true);
                    }
                    else if(!bRight && j >= 16){
                        FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                            vKeysCell,5,true);
                    }
                    else{
                        FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                            vKeysCell,minThFAST,true);
                    }*/
                }

                // 非空则偏移坐标加入总列表
                if (!vKeysCell.empty())
                {
                    for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin();
                         vit != vKeysCell.end(); vit++)
                    {
                        (*vit).pt.x += j * wCell;
                        (*vit).pt.y += i * hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }
            }
        }

        vector<KeyPoint>& keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures);

        // 八叉树均匀分布
        keypoints =
            DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX, minBorderY,
                           maxBorderY, mnFeaturesPerLevel[level], level);

        const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

        // Add border to coordinates and scale information
        // 加边界偏移，设置层级和尺寸
        const int nkps = keypoints.size();
        for (int i = 0; i < nkps; i++)
        {
            keypoints[i].pt.x += minBorderX;
            keypoints[i].pt.y += minBorderY;
            keypoints[i].octave = level;
            keypoints[i].size = scaledPatchSize;
        }
    }

    // compute orientations
    // 逐层计算主方向
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}

/**
 * @brief 旧版网格法提取特征点
 * @param allKeypoints 输出各层特征点
 */
/**
 * @brief 旧版网格法均匀提取ORB特征点（保留实现，默认使用八叉树版本）
 * @param allKeypoints 输出各金字塔层的特征点向量
 */
void ORBextractor::ComputeKeyPointsOld(
    std::vector<std::vector<KeyPoint> >& allKeypoints)
{
    // 调整输出向量大小，与金字塔层数一致
    allKeypoints.resize(nlevels);

    // 计算第0层图像宽高比，用于按比例划分网格行列
    float imageRatio =
        static_cast<float>(mvImagePyramid[0].cols) / mvImagePyramid[0].rows;

    // 逐层提取特征点
    for (int level = 0; level < nlevels; ++level)
    {
        // 获取该层期望提取的特征点总数
        const int nDesiredFeatures = mnFeaturesPerLevel[level];

        // 计算网格列数，按特征点密度与宽高比推导，保证网格近似正方形
        const int levelCols =
            sqrt(static_cast<float>(nDesiredFeatures) / (5 * imageRatio));
        // 网格行数，与列数保持宽高比
        const int levelRows = imageRatio * levelCols;

        // 提取区域内边界，留出边缘阈值，避免特征点越界
        const int minBorderX = EDGE_THRESHOLD;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD;
        const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD;

        // 提取区域总宽度
        const int W = maxBorderX - minBorderX;
        // 提取区域总高度
        const int H = maxBorderY - minBorderY;

        // 单个网格单元宽度，向上取整
        const int cellW = ceil(static_cast<float>(W) / levelCols);
        // 单个网格单元高度，向上取整
        const int cellH = ceil(static_cast<float>(H) / levelRows);

        // 总网格单元数量
        const int nCells = levelRows * levelCols;
        // 每个网格单元期望特征点数，向上取整
        const int nfeaturesCell =
            ceil(static_cast<float>(nDesiredFeatures) / nCells);

        // 三维容器：行索引-列索引-该格特征点向量
        vector<vector<vector<KeyPoint> > > cellKeyPoints(
            levelRows, vector<vector<KeyPoint> >(levelCols));

        // 每个网格最终保留特征点数
        vector<vector<int> > nToRetain(levelRows, vector<int>(levelCols, 0));
        // 每个网格实际提取到的特征点总数
        vector<vector<int> > nTotal(levelRows, vector<int>(levelCols, 0));
        // 标记该网格是否已达配额上限，不再参与额外分配
        vector<vector<bool> > bNoMore(levelRows, vector<bool>(levelCols, false));

        // 每列起始x坐标缓存，避免重复计算
        vector<int> iniXCol(levelCols);
        // 每行起始y坐标缓存，避免重复计算
        vector<int> iniYRow(levelRows);

        // 已达配额上限的网格总数
        int nNoMore = 0;
        // 待二次分配的剩余特征点配额
        int nToDistribute = 0;

        // 网格竖直方向向外扩展3像素，避免边界处漏检特征
        float hY = cellH + 6;

        // 逐行遍历网格
        for (int i = 0; i < levelRows; i++)
        {
            // 当前行起始y坐标，向外扩3像素
            const float iniY = minBorderY + i * cellH - 3;
            // 缓存行起始y坐标
            iniYRow[i] = iniY;

            // 最后一行特殊处理，防止超出图像下边界
            if (i == levelRows - 1)
            {
                hY = maxBorderY + 3 - iniY;
                if (hY <= 0) continue;
            }

            // 网格水平方向向外扩展3像素
            float hX = cellW + 6;

            // 逐列遍历网格
            for (int j = 0; j < levelCols; j++)
            {
                float iniX;

                // 第一行计算并缓存每列起始x坐标
                if (i == 0)
                {
                    iniX = minBorderX + j * cellW - 3;
                    iniXCol[j] = iniX;
                }
                else
                {
                    // 后续行复用缓存的列起始x
                    iniX = iniXCol[j];
                }

                // 最后一列特殊处理，防止超出图像右边界
                if (j == levelCols - 1)
                {
                    hX = maxBorderX + 3 - iniX;
                    if (hX <= 0) continue;
                }

                // 截取当前网格单元的图像ROI
                Mat cellImage = mvImagePyramid[level]
                                    .rowRange(iniY, iniY + hY)
                                    .colRange(iniX, iniX + hX);

                // 预分配网格特征点容器容量
                cellKeyPoints[i][j].reserve(nfeaturesCell * 5);

                // 使用初始阈值提取FAST角点，开启非极大值抑制
                FAST(cellImage, cellKeyPoints[i][j], iniThFAST, true);

                // 提取点数过少（≤3），降低阈值重新提取，保证每个网格至少有特征点
                if (cellKeyPoints[i][j].size() <= 3)
                {
                    cellKeyPoints[i][j].clear();
                    // 使用最低阈值二次提取
                    FAST(cellImage, cellKeyPoints[i][j], minThFAST, true);
                }

                // 当前网格实际提取特征点数
                const int nKeys = cellKeyPoints[i][j].size();
                nTotal[i][j] = nKeys;

                // 实际点数超过配额，保留配额数，标记可继续分配
                if (nKeys > nfeaturesCell)
                {
                    nToRetain[i][j] = nfeaturesCell;
                    bNoMore[i][j] = false;
                }
                else
                {
                    // 实际点数不足配额，全部保留，剩余配额回收到总池
                    nToRetain[i][j] = nKeys;
                    nToDistribute += nfeaturesCell - nKeys;
                    // 标记该网格已达上限，不再参与分配
                    bNoMore[i][j] = true;
                    nNoMore++;
                }
            }
        }

        // Retain by score
        // 剩余配额迭代再分配：将剩余配额均匀分给未达上限的网格
        while (nToDistribute > 0 && nNoMore < nCells)
        {
            // 计算新一轮每个网格可分配的特征点数
            int nNewFeaturesCell =
                nfeaturesCell +
                ceil(static_cast<float>(nToDistribute) / (nCells - nNoMore));
            nToDistribute = 0;

            // 遍历所有网格更新配额
            for (int i = 0; i < levelRows; i++)
            {
                for (int j = 0; j < levelCols; j++)
                {
                    // 只处理未达上限的网格
                    if (!bNoMore[i][j])
                    {
                        // 实际点数仍高于新配额，保留新配额，可继续分配
                        if (nTotal[i][j] > nNewFeaturesCell)
                        {
                            nToRetain[i][j] = nNewFeaturesCell;
                            bNoMore[i][j] = false;
                        }
                        else
                        {
                            // 实际点数不足新配额，全部保留，剩余配额继续回收
                            nToRetain[i][j] = nTotal[i][j];
                            nToDistribute += nNewFeaturesCell - nTotal[i][j];
                            // 标记该网格已达上限
                            bNoMore[i][j] = true;
                            nNoMore++;
                        }
                    }
                }
            }
        }

        // 当前层输出特征点引用
        vector<KeyPoint>& keypoints = allKeypoints[level];
        keypoints.reserve(nDesiredFeatures * 2);

        // 当前层特征点patch尺寸，随金字塔缩放
        const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

        // Retain by score and transform coordinates
        // 逐网格保留高分特征点并还原全局坐标
        for (int i = 0; i < levelRows; i++)
        {
            for (int j = 0; j < levelCols; j++)
            {
                vector<KeyPoint>& keysCell = cellKeyPoints[i][j];
                // 按响应值保留前nToRetain个最佳特征点
                KeyPointsFilter::retainBest(keysCell, nToRetain[i][j]);
                // 裁剪到保留数量
                if (static_cast<int>(keysCell.size()) > nToRetain[i][j])
                    keysCell.resize(nToRetain[i][j]);

                // 每个特征点加回网格偏移，设置层级与尺寸信息
                for (size_t k = 0, kend = keysCell.size(); k < kend; k++)
                {
                    keysCell[k].pt.x += iniXCol[j];
                    keysCell[k].pt.y += iniYRow[i];
                    keysCell[k].octave = level;
                    keysCell[k].size = scaledPatchSize;
                    keypoints.push_back(keysCell[k]);
                }
            }
        }

        // 总特征点数超过期望，全局再做一次按响应值筛选
        if (static_cast<int>(keypoints.size()) > nDesiredFeatures)
        {
            KeyPointsFilter::retainBest(keypoints, nDesiredFeatures);
            keypoints.resize(nDesiredFeatures);
        }
    }

    // and compute orientations
    // 逐层计算所有特征点的主方向，实现旋转不变性
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}

/**
 * @brief 批量计算所有特征点的rBRIEF二进制描述子
 * @param image 输入灰度图像
 * @param keypoints 特征点向量
 * @param descriptors 输出描述子矩阵（N×32 uchar）
 * @param pattern 采样点模式数组首地址
 */
static void computeDescriptors(const Mat& image, vector<KeyPoint>& keypoints,
                               Mat& descriptors, const vector<Point>& pattern)
{
    // 创建描述子矩阵：N行32列，单字节无符号
    descriptors = Mat::zeros(static_cast<int>(keypoints.size()), 32, CV_8UC1);

    // 逐个特征点计算描述子，写入对应行
    for (size_t i = 0; i < keypoints.size(); i++)
        computeOrbDescriptor(keypoints[i], image, &pattern[0],
                          descriptors.ptr(static_cast<int>(i)));
}

/**
 * @brief ORB特征提取器仿函数入口，执行完整特征提取流程
 * @param _image 输入图像
 * @param _mask 掩码（未使用）
 * @param _keypoints 输出特征点向量
 * @param _descriptors 输出描述子矩阵
 * @param vLappingArea 立体视觉重叠区域x坐标范围
 * @return 单目区域特征点数量
 */
int ORBextractor::operator()(InputArray _image, InputArray _mask,
                              vector<KeyPoint>& _keypoints,
                              OutputArray _descriptors,
                              std::vector<int>& vLappingArea)
{
    // cout << "[ORBextractor]: Max Features: " << nfeatures << endl;
    // 输入图像为空直接返回错误
    if (_image.empty()) return -1;

    // 获取输入图像Mat引用
    Mat image = _image.getMat();
    // 断言必须是单通道8位灰度图
    assert(image.type() == CV_8UC1);

    // Pre-compute the scale pyramid
    // 构建高斯图像金字塔
    ComputePyramid(image);

    vector<vector<KeyPoint> > allKeypoints;
    // 使用八叉树法均匀提取特征点（默认启用，旧网格法注释）
    ComputeKeyPointsOctTree(allKeypoints);
    // ComputeKeyPointsOld(allKeypoints);

    Mat descriptors;
    int nkeypoints = 0;
    // 统计所有层特征点总数
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += static_cast<int>(allKeypoints[level].size());

    // 没有检测到特征点，释放输出描述符
    if (nkeypoints == 0)
    {
        _descriptors.release();
    }
    else
    {
        // 创建输出描述符矩阵
        _descriptors.create(nkeypoints, 32, CV_8U);
        descriptors = _descriptors.getMat();
    }

    // _keypoints.clear();
    // _keypoints.reserve(nkeypoints);
    // 构造输出特征点向量
    _keypoints = vector<cv::KeyPoint>(nkeypoints);

    int offset = 0;
    // Modified for speeding up stereo fisheye matching
    // 单目区域索引从前往后，双目重叠区域索引从后往前，加速立体匹配
    int monoIndex = 0, stereoIndex = nkeypoints - 1;

    // 逐层计算描述子并分配到单/双目区域
    for (int level = 0; level < nlevels; ++level)
    {
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = static_cast<int>(keypoints.size());

        if (nkeypointsLevel == 0) continue;

        // preprocess the resized image
        // 克隆当前层图像
        Mat workingMat = mvImagePyramid[level].clone();
        // 7x7高斯模糊，sigma=2，边界反射；rBRIEF对噪声敏感，先模糊提升鲁棒性
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2,
                     cv::BORDER_REFLECT_101);

        // Compute the descriptors
        // Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        Mat desc = cv::Mat(nkeypointsLevel, 32, CV_8U);
        // 计算该层所有特征点描述子
        computeDescriptors(workingMat, keypoints, desc, pattern);

        offset += nkeypointsLevel;

        float scale =
            mvScaleFactor[level];  // getScale(level, firstLevel, scaleFactor);

        int i = 0;
        // 遍历该层所有特征点，按坐标分配到单目/双目区域
        for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                                         keypointEnd = keypoints.end();
             keypoint != keypointEnd; ++keypoint)
        {
            // Scale keypoint coordinates
            // 非第0层将特征点坐标缩放回原始图像尺寸
            if (level != 0)
            {
                keypoint->pt *= scale;
            }

            // 特征点x坐标落在重叠区域内，放入双目索引区（从后往前）
            if (keypoint->pt.x >= vLappingArea[0] &&
                keypoint->pt.x <= vLappingArea[1])
            {
                _keypoints.at(stereoIndex) = (*keypoint);
                desc.row(i).copyTo(descriptors.row(stereoIndex));
                stereoIndex--;
            }
            else
            {
                // 非重叠区域放入单目索引区（从前往后）
                _keypoints.at(monoIndex) = (*keypoint);
                desc.row(i).copyTo(descriptors.row(monoIndex));
                monoIndex++;
            }
            i++;
        }
    }

    // cout << "[ORBextractor]: extracted " << _keypoints.size() << " KeyPoints"
    // << endl;
    // 返回单目区域特征点数量
    return monoIndex;
}

/**
 * @brief 构建高斯图像金字塔，每层缩放并做边界反射填充
 * @param image 输入原始图像
 */
void ORBextractor::ComputePyramid(cv::Mat image)
{
    // 逐层构建金字塔
    for (int level = 0; level < nlevels; ++level)
    {
        // 当前层逆缩放因子
        float scale = mvInvScaleFactor[level];

        // 缩放后图像尺寸，四舍五入取整
        Size sz(cvRound(static_cast<float>(image.cols) * scale),
                 cvRound(static_cast<float>(image.rows) * scale));

        // 加上两侧边缘阈值后的整体尺寸
        Size wholeSize(sz.width + EDGE_THRESHOLD * 2,
                       sz.height + EDGE_THRESHOLD * 2);

        // 创建带边界的临时大图
        Mat temp(wholeSize, image.type()), masktemp;

        // 金字塔第level层指向临时图的中间有效区域，不拷贝数据
        mvImagePyramid[level] =
            temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        // 非第0层，由上一层缩放得到
        if (level != 0)
        {
            // 双线性插值缩放到当前层尺寸
            resize(mvImagePyramid[level - 1], mvImagePyramid[level], sz, 0, 0,
                   cv::INTER_LINEAR);

            // 四周填充边缘阈值宽度的边界，采用反射模式，避免边缘引入强梯度
            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD,
                           EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);
        }
        else
        {
            // 第0层直接对原始图像做边界填充
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           EDGE_THRESHOLD, EDGE_THRESHOLD, cv::BORDER_REFLECT_101);
        }
    }
}

} // namespace ORB_SLAM3
