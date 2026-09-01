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
#include <algorithm>                   // STL通用算法库
#include <chrono>                      // C++时间工具库
#include <ctime>                       // C标准时间接口
#include <fstream>                     // 文件读写输入输出流
#include <iomanip>                     // IO输出格式控制
#include <iostream>                    // 标准输入输出cout/cerr
#include <opencv2/core/core.hpp>       // OpenCV核心模块，Mat基础数据结构
#include <sstream>                     // stringstream字符串流，用于解析csv文本

#include "ImuTypes.h"                  // ORB‑SLAM3 IMU数据点类型定义头文件
#include "Optimizer.h"                 // ORB‑SLAM3优化器头文件

using namespace std;

/**
 * @brief 双目图像帧集合结构体，保存一帧双目左右图路径与对应时间戳
 */
struct ImageSet {
    /**
     * @brief ImageSet构造函数
     * @param leftImg 左目图像完整文件路径
     * @param rightImg 右目图像完整文件路径
     * @param timestamp 图像帧时间戳，单位：秒
     */
    ImageSet(const string &leftImg, const string &rightImg, double timestamp)
        : mLeftImage(leftImg), mRightImage(rightImg), mTimestamp(timestamp) {}

    cv::Mat leftImage() const;         // 读取左目图像，返回cv::Mat
    cv::Mat rightImage() const;        // 读取右目图像，返回cv::Mat

    std::string mLeftImage, mRightImage; // 左、右图像文件完整路径
    double mTimestamp;                    // 当前双目帧时间戳，单位秒
};

/**
 * @brief EuRoC数据集单条序列类，存储一组双目图像序列与配套IMU测量数据
 */
struct EuRoCSequence {
    EuRoCSequence() = delete;          // 删除默认无参构造，禁止无参实例化对象
    /**
     * @brief EuRoCSequence带参构造函数
     * @param leftPath 左相机cam0图片文件夹路径
     * @param rightPath 右相机cam1图片文件夹路径
     * @param timestampPath 图像时间戳文本文件路径
     * @param imuPath IMU数据csv文件路径，默认为空字符串，为空则不加载IMU
     */
    EuRoCSequence(const string &leftPath, const string &rightPath,
                  const string &timestampPath, const string &imuPath = "");

    /**
     * @brief 加载IMU csv数据文件，过滤掉startTime之前的IMU采样点
     * @param imuPath IMU的data.csv文件路径
     * @param startTime 起始时间阈值，默认0.0，早于该时间的IMU数据丢弃
     */
    void loadImu(const string &imuPath, double startTime = 0.0);

    vector<ImageSet> mvImageSets;      // 容器，保存该序列全部双目图像帧集合
    vector<ORB_SLAM3::IMU::Point> vImu;// 容器，保存该序列全部IMU测量点

    /**
     * @brief 获取该序列双目图像总帧数
     * @return size_t 图像帧数量
     */
    size_t size() const { return mvImageSets.size(); }

    /**
     * @brief 计算图像帧之间时间间隔，简易实现，假设帧率恒定
     * @return double 帧间时间dt，单位秒；不足两帧返回0
     * @note Lazy solution, assume constant image rate 简易方案，强制假设图像帧率固定不变
     */
    double dt() const {
        if (size() > 1) {
            return mvImageSets.at(1).mTimestamp - mvImageSets.at(0).mTimestamp;
        } else {
            return 0;
        }
    }
};

/**
 * @brief EuRoCData类，管理多条EuRoC数据集序列，支持批量加载多段序列
 */
class EuRoCData {
public:
    /**
     * @brief 序列路径配置结构体，保存单条数据集序列的各个文件路径
     */
    struct SequencePaths {
        /**
         * @brief SequencePaths构造函数
         * @param imagePath EuRoC数据集根目录路径
         * @param timestampPath 图像时间戳文件路径
         * @param imuPath IMU csv文件路径，默认为空
         */
        SequencePaths(const string &imagePath, const string &timestampPath,
                      const string &imuPath = "")
            : mImagePath(imagePath),
              mTimestampPath(timestampPath),
              mImuPath(imuPath) {}

        std::string mImagePath, mTimestampPath, mImuPath; // 数据集根路径、时间戳文件路径、IMU文件路径
    };

    EuRoCData() = default;             // 默认构造函数，使用编译器默认生成

    /**
     * @brief 静态接口，批量加载多个EuRoC数据集序列
     * @param seqPaths 多条序列的路径配置数组
     * @param loadImu 是否加载IMU数据，默认false不加载
     * @return EuRoCData 返回装载全部序列的数据集对象
     */
    static EuRoCData LoadSequences(const std::vector<SequencePaths> &seqPaths,
                                   bool loadImu = false);

    /**
     * @brief 统计所有序列合计的双目图像总帧数
     * @return size_t 全部序列图像帧累加总数
     */
    size_t totalImages() const {
        size_t count = 0;
        for (auto const &seq : mvSequences) count += seq.size();
        return count;
    }

    std::vector<EuRoCSequence> mvSequences; // 容器，存放多条EuRoCSequence序列对象
};

void setupEurocSpdLogger(); // 初始化spdlog日志函数，配置Euroc模块日志输出格式与sink
