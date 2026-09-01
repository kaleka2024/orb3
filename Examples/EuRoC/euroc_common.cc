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
#include "euroc_common.h"                     // Euroc数据集相关结构体、类声明头文件
#include <spdlog/sinks/stdout_color_sinks.h>  // spdlog彩色标准输出日志sink
#include <spdlog/spdlog.h>                    // spdlog日志库主头文件

#include <algorithm>                          // STL算法工具
#include <chrono>                             // 时间工具库
#include <ctime>                              // C时间接口
#include <filesystem>                         // C++17文件系统，用于路径、文件存在性判断
#include <fstream>                            // 文件读写ifstream/ofstream
#include <iomanip>                            // 输出格式控制
#include <iostream>                           // cout、cerr标准IO
#include <opencv2/core/core.hpp>              // OpenCV核心Mat数据结构
#include <sstream>                            // stringstream字符串流解析csv文本

#include "ImuTypes.h"                         // ORB‑SLAM3 IMU数据类型定义头文件

using namespace std;

namespace fs = std::filesystem;               // 给std::filesystem起别名fs，简化书写

/**
 * @brief 批量加载多个EuRoC数据集序列
 * @param seqPaths 多个序列的路径配置数组
 * @param loadImu true同时加载IMU数据；false只加载双目图像时间戳，不加载IMU
 * @return EuRoCData 封装全部序列的数据集对象
 */
EuRoCData EuRoCData::LoadSequences(const std::vector<SequencePaths> &seqPaths,
                                   bool loadImu) {
  EuRoCData data;                              // 创建数据集总对象

  for (auto const &seq : seqPaths) {           // 遍历每一个待加载的数据集序列
    const string pathCam0 = seq.mImagePath + "/mav0/cam0/data";  // cam0左相机图片目录
    const string pathCam1 = seq.mImagePath + "/mav0/cam1/data";  // cam1右相机图片目录

    if (!fs::exists(pathCam0) || !fs::exists(pathCam1)) {       // 校验左右相机图片目录是否存在
      throw runtime_error("Canot find the image data");          // 目录缺失抛出运行时异常
    }

    if (loadImu) {                                              // 如果需要加载IMU数据
      // For now, load the IMU data directly from the EuRoC dataset
      const string pathImu = seq.mImagePath + "/mav0/imu0/data.csv"; // IMU csv文件路径

      if (!fs::exists(pathImu)) {                               // 校验IMU csv文件是否存在
        throw std::runtime_error("Cannot find IMU data");
      }
      // 构造EuRoCSequence对象，存入mvSequences，传入图像路径、时间戳文件、IMU文件
      data.mvSequences.emplace_back(pathCam0, pathCam1, seq.mTimestampPath,
                                    pathImu);
    } else {
      // 不加载IMU，只传入双目路径与时间戳文件
      data.mvSequences.emplace_back(pathCam0, pathCam1, seq.mTimestampPath);
    }
  }

  return data; // 返回组装完成的数据集对象
}

/**
 * @brief EuRoCSequence构造函数：加载双目图像时间戳，构建图像帧集合；可选加载IMU
 * @param leftPath cam0左相机图片文件夹路径
 * @param rightPath cam1右相机图片文件夹路径
 * @param timestampPath 时间戳文本文件路径
 * @param imuPath IMU csv文件路径，空字符串代表不加载IMU
 */
EuRoCSequence::EuRoCSequence(const string &leftPath, const string &rightPath,
                             const string &timestampPath,
                             const string &imuPath) {
  ifstream fTimes;                     // 打开时间戳文本文件输入流
  fTimes.open(timestampPath.c_str());

  while (!fTimes.eof()) {              // 循环读取直到文件末尾
    string s;
    getline(fTimes, s);                // 读取一整行文本
    if (!s.empty()) {                  // 跳过空行
      if (s[0] == '#') continue;       // 跳过#开头注释行

      stringstream ss;
      ss << s;                         // 将行字符串送入字符串流
      const string leftImg = leftPath + "/" + ss.str() + ".png";  // 拼接左图完整路径
      const string rightImg = rightPath + "/" + ss.str() + ".png";// 拼接右图完整路径

      double t;
      ss >> t;                         // 读取纳秒时间戳数值

      mvImageSets.emplace_back(leftImg, rightImg, t / 1e9); // 构造ImageSet，纳秒转秒存入容器
    }
  }

  // imuPath非空，且成功读到至少一组图像帧，则加载IMU，起始过滤掉早于第一帧图像的IMU样本
  if ((imuPath.size() > 0) && (mvImageSets.size() > 0)) {
    loadImu(imuPath, mvImageSets.begin()->mTimestamp);
  }
}

/**
 * @brief 加载EuRoC IMU的csv文件，过滤掉startTime之前的IMU采样
 * @param imuPath IMU data.csv文件路径
 * @param startTime 第一帧图像的时间戳(秒)，早于该时间的IMU直接丢弃
 */
void EuRoCSequence::loadImu(const string &imuPath, double startTime) {
  ifstream fImu;

  fImu.open(imuPath.c_str());         // 打开IMU csv文件

  while (!fImu.eof()) {               // 循环读取csv每一行
    string s;
    getline(fImu, s);

    if (s[0] == '#') continue;        // 跳过#开头注释行

    if (!s.empty()) {
      string item;
      size_t pos = 0;
      double data[7];                 // EuRoC IMU csv字段：timestamp,wx,wy,wz,ax,ay,az
      int count = 0;
      // 按逗号','分割csv字段，循环解析每一列
      while ((pos = s.find(',')) != string::npos) {
        item = s.substr(0, pos);       // 截取逗号前子串
        data[count++] = stod(item);    // 字符串转double存入数组
        s.erase(0, pos + 1);           // 删掉已经解析的部分，继续处理剩余字符串
      }
      item = s.substr(0, pos);         // 处理最后一个字段
      data[6] = stod(item);

      const double t = data[0] / 1e9; // 原始纳秒时间戳转为秒

      // Ignore data before startTime，丢弃早于第一帧图像的IMU数据
      if (t <= startTime) continue;

      // 构造IMU测量值对象存入vImu容器：ax,ay,az,gx,gy,gz,t
      vImu.emplace_back(data[4], data[5], data[6], data[1], data[2], data[3],
                        t);
    }
  }
}

/**
 * @brief 读取左目图像
 * @return cv::Mat 返回读取得到的左目灰度图像，保持原始位深
 */
cv::Mat ImageSet::leftImage() const {
  return cv::imread(mLeftImage, cv::IMREAD_UNCHANGED);
}

/**
 * @brief 读取右目图像
 * @return cv::Mat 返回读取得到的右目灰度图像，保持原始位深
 */
cv::Mat ImageSet::rightImage() const {
  return cv::imread(mRightImage, cv::IMREAD_UNCHANGED);
}

/**
 * @brief 初始化spdlog日志，配置彩色标准输出sink，设置日志格式，供Euroc数据集模块使用
 */
void setupEurocSpdLogger() {
  auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(); // 创建彩色控制台输出sink

  ORB_SLAM3::Logger::add_sink(stdout_sink);                  // 将sink注册进ORB‑SLAM3内部日志器
  spdlog::set_default_logger(
      std::make_shared<spdlog::logger>("euroc", stdout_sink)); // 设置全局默认logger实例，命名euroc
  spdlog::set_pattern("%E.%e [%-8n] [%6!l] %v");             // 设置日志打印格式：时间、logger名、日志级别、日志内容
}
