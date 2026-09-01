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
#include <signal.h>                 // Linux信号处理，捕获Ctrl‑C中断信号
#include <stdlib.h>

#include <algorithm>
#include <chrono>                   // 时间相关工具
#include <condition_variable>       // 条件变量，用于多线程等待‑通知机制
#include <ctime>
#include <fstream>                  // 文件输出流ofstream
#include <iomanip>                  // setprecision 设置输出浮点数精度
#include <iostream>                 // cout、cerr标准输入输出
#include <librealsense2/rs.hpp>     // RealSense SDK主头文件
#include <opencv2/core/core.hpp>    // OpenCV核心Mat、Size等基础结构
#include <opencv2/highgui.hpp>     // OpenCV窗口、imshow、imwrite
#include <sstream>

#include "librealsense2/rsutil.h"  // RealSense工具辅助函数
#include "opencv2/imgproc/imgproc.hpp" // OpenCV图像处理，resize缩放函数

using namespace std;

bool b_continue_session;            ///< 主录制循环运行标志，Ctrl‑C置false退出录制

const float reductionFactor = 0.5;  ///< 图像缩放系数，0.5代表宽高各缩小一半
const int colsRedIm = reductionFactor * 848; ///< 缩放后图像宽度，原始鱼眼分辨率848
const int rowsRedIm = reductionFactor * 800; ///< 缩放后图像高度，原始鱼眼分辨率800

/**
 * @brief SIGINT信号回调函数，捕获Ctrl‑C按键，结束录制会话
 * @param s 触发的信号编号
 */
void exit_loop_handler(int s) {
  cout << "Finishing session" << endl;
  b_continue_session = false;
}

/**
 * @brief 遍历打印传感器全部可配置选项，打印选项名称、是否支持、描述、当前值
 * @param sensor RealSense传感器对象，可以是相机或IMU传感器
 * @return 返回选中的rs2_option，本示例固定返回0
 */
static rs2_option get_sensor_option(const rs2::sensor& sensor) {
  // Sensors usually have several options to control their properties
  //  such as Exposure, Brightness etc.

  std::cout << "Sensor supports the following options:\n" << std::endl;

  // The following loop shows how to iterate over all available options
  // Starting from 0 until RS2_OPTION_COUNT (exclusive)
  for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
    rs2_option option_type = static_cast<rs2_option>(i);
    // SDK enum types can be streamed to get a string that represents them
    std::cout << "  " << i << ": " << option_type;

    // To control an option, use the following api:

    // First, verify that the sensor actually supports this option
    if (sensor.supports(option_type)) {
      std::cout << std::endl;

      // Get a human readable description of the option
      const char* description = sensor.get_option_description(option_type);
      std::cout << "       Description   : " << description << std::endl;

      // Get the current value of the option
      float current_value = sensor.get_option(option_type);
      std::cout << "       Current Value : " << current_value << std::endl;

      // To change the value of an option, please follow the
      // change_sensor_option() function
    } else {
      std::cout << " is not supported" << std::endl;
    }
  }

  uint32_t selected_sensor_option = 0;
  return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char** argv) {
  // 命令行参数校验，需要传入数据集保存文件夹路径
  if (argc != 2) {
    cerr << endl
         << "Usage: ./recorder_realsense_D435i path_to_saving_folder" << endl;
    return 1;
  }

  string directory = string(argv[argc - 1]); ///< 数据集输出根目录

  // 注册Ctrl‑C中断信号处理
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = exit_loop_handler; ///< 设置信号回调处理函数
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);
  b_continue_session = true;

  double offset = 0;  // ms 时间戳偏移量，本程序未实际使用

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe; ///< RealSense数据流管道，管理设备、开启流、接收帧
  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;    ///< 流配置对象，配置开启哪些流、分辨率、格式、帧率
  cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8, 30); ///< 开启左鱼眼相机1，8位灰度，30Hz
  cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8, 30); ///< 开启右鱼眼相机2，8位灰度，30Hz
  cfg.enable_stream(RS2_STREAM_ACCEL,
                    RS2_FORMAT_MOTION_XYZ32F);                   //, 250); // 63 ///< 开启加速度计，XYZ浮点格式
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);  //, 400); ///< 开启陀螺仪，XYZ浮点格式

  // IMU callback 多线程同步相关变量
  std::mutex imu_mutex;                     ///< 互斥锁，保护回调与主线程之间共享图像、IMU缓存
  std::condition_variable cond_image_rec;   ///< 条件变量，主线程阻塞等待图像帧就绪通知

  vector<double> v_gyro_timestamp;          ///< 陀螺仪时间戳容器，单位秒
  vector<rs2_vector> v_gyro_data;           ///< 陀螺仪三轴角速度数据容器
  vector<double> v_acc_timestamp;           ///< 加速度计时间戳容器，单位秒
  vector<rs2_vector> v_acc_data;            ///< 加速度计三轴加速度数据容器

  cv::Mat imCV_left, imCV_right;            ///< 回调线程内保存原始左右鱼眼图像Mat
  int width_img, height_img;                ///< 原始鱼眼图像宽高，从设备内参读取
  double timestamp_image;                   ///< 双目图像帧对应的时间戳，单位秒
  bool image_ready = false;                 ///< 标记回调线程是否收到新的双目图像帧

  /**
   * @brief RealSense帧回调lambda函数，运行在SDK后台线程
   * @param frame RealSense帧对象，可以是双目图像frameset，也可以是IMU motion_frame
   */
  auto imu_callback = [&](const rs2::frame& frame) {
    std::unique_lock<std::mutex> lock(imu_mutex);

    // 判断是否为图像帧集合frameset，包含左右鱼眼帧
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      rs2::video_frame color_frame_left = fs.get_fisheye_frame(1);
      rs2::video_frame color_frame_right = fs.get_fisheye_frame(2);
      // 将RealSense图像内存包装为OpenCV Mat，不拷贝原始数据
      imCV_left =
          cv::Mat(cv::Size(width_img, height_img), CV_8U,
                  (void*)(color_frame_left.get_data()), cv::Mat::AUTO_STEP);
      imCV_right =
          cv::Mat(cv::Size(width_img, height_img), CV_8U,
                  (void*)(color_frame_right.get_data()), cv::Mat::AUTO_STEP);

      timestamp_image = fs.get_timestamp() * 1e-3; // 时间戳ms转换为s
      image_ready = true;

      lock.unlock();
      cond_image_rec.notify_all(); // 唤醒主线程，通知双目图像已经就绪
    }
    // 判断是否为IMU运动帧，区分陀螺仪与加速度计
    else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()) {
      if (m_frame.get_profile().stream_name() == "Gyro") {
        // It runs at 200Hz，陀螺仪数据存入全局缓存
        v_gyro_data.push_back(m_frame.get_motion_data());
        v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
      } else if (m_frame.get_profile().stream_name() == "Accel") {
        // It runs at 60Hz，加速度计数据存入全局缓存
        v_acc_data.push_back(m_frame.get_motion_data());
        v_acc_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
      }
    }
  };

  // 启动pipeline，传入配置与帧回调函数
  rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);
  rs2::stream_profile cam_stream_left =
      pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1);
  rs2::stream_profile cam_stream_right =
      pipe_profile.get_stream(RS2_STREAM_FISHEYE, 2);
  rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);

  // 获取左鱼眼相机内参，提取原始图像宽高
  rs2_intrinsics intrinsics_cam =
      cam_stream_left.as<rs2::video_stream_profile>().get_intrinsics();
  width_img = intrinsics_cam.width;
  height_img = intrinsics_cam.height;

  cv::Mat imLeft, imRight;
  ofstream accFile, gyroFile, cam0TsFile, cam1TsFile;
  // 打开输出文件：加速度、陀螺仪、cam0左相机时间戳、cam1右相机时间戳
  accFile.open(directory + "/IMU/acc.txt");
  gyroFile.open(directory + "/IMU/gyro.txt");
  cam0TsFile.open(directory + "/cam0/times.txt");
  cam1TsFile.open(directory + "/cam1/times.txt");

  cout << directory + "/IMU/acc.txt" << endl;

  // 文件打开校验，失败直接退出程序
  if (!accFile.is_open() || !gyroFile.is_open() || !cam0TsFile.is_open()) {
    cerr << "FILES NOT OPENED" << endl;
    exit(-1);
  }

  // Clear IMU vectors，清空IMU全局缓存容器
  v_gyro_data.clear();
  v_gyro_timestamp.clear();
  v_acc_data.clear();
  v_acc_timestamp.clear();

  cv::namedWindow("cam0", cv::WINDOW_AUTOSIZE); ///< OpenCV窗口，显示左相机图像

  // 主录制循环，b_continue_session=false时退出，由Ctrl‑C信号修改该标志
  while (b_continue_session) {
    std::vector<rs2_vector> vGyro;
    std::vector<double> vGyro_times;
    std::vector<rs2_vector> vAccel, vAccel_Sync;
    std::vector<double> vAccel_times;
    double imTs;
    {
      {
        std::unique_lock<std::mutex> lk(imu_mutex);
        if (!image_ready)  // wait until image read from the other thread
          cond_image_rec.wait(lk); // 主线程阻塞，等待回调线程图像就绪通知
      }
      std::lock_guard<std::mutex> lk(imu_mutex);

      // Copy the IMU data to local single thread variables
      // 将回调线程的IMU、图像时间戳拷贝到主线程局部变量，隔离多线程竞争
      vGyro = v_gyro_data;
      vGyro_times = v_gyro_timestamp;
      vAccel = v_acc_data;
      vAccel_times = v_acc_timestamp;
      imTs = timestamp_image;

      // 根据缩放系数决定是直接克隆原图，还是resize缩放图像
      if (reductionFactor == 1.0) {
        imLeft = imCV_left.clone();
        imRight = imCV_right.clone();
      } else {
        cv::resize(imCV_left, imLeft, cv::Size(colsRedIm, rowsRedIm));
        cv::resize(imCV_right, imRight, cv::Size(colsRedIm, rowsRedIm));
      }

      // Clear IMU vectors，清空回调线程IMU缓存，准备接收下一批IMU采样
      v_gyro_data.clear();
      v_gyro_timestamp.clear();
      v_acc_data.clear();
      v_acc_timestamp.clear();

      image_ready = false;
    }

    cv::imshow("cam0", imLeft);
    cv::imshow("cam1", imRight);

    // save image and IMU data，时间戳由秒转为纳秒整数作为图片文件名
    long int imTsInt = (long int)(1e9 * imTs);
    string imgRepoLeft = directory + "/cam0/" + to_string(imTsInt) + ".png";
    if (!imLeft.empty()) {
      cv::imwrite(imgRepoLeft, imLeft);       // 保存左鱼眼png图像
      cam0TsFile << imTsInt << endl;          // 写入左相机时间戳文本
    } else {
      cout << " left image empty!! \n";
    }

    string imgRepoRight = directory + "/cam1/" + to_string(imTsInt) + ".png";
    if (!imRight.empty()) {
      cv::imwrite(imgRepoRight, imRight);     // 保存右鱼眼png图像
      cam1TsFile << imTsInt << endl;          // 写入右相机时间戳文本
    } else {
      cout << "right image empty!! \n";
    }

    // assert(vAccel.size() == vAccel_times.size());
    // assert(vGyro.size() == vGyro_times.size());

    // 循环写入加速度计数据文件，格式: timestamp,ax,ay,az
    for (int i = 0; i < vAccel.size(); ++i) {
      accFile << std::setprecision(15) << vAccel_times[i] << "," << vAccel[i].x
              << "," << vAccel[i].y << "," << vAccel[i].z << endl;
    }

    // 循环写入陀螺仪数据文件，格式: timestamp,gx,gy,gz
    for (int i = 0; i < vGyro.size(); ++i) {
      gyroFile << std::setprecision(15) << vGyro_times[i] << "," << vGyro[i].x
               << "," << vGyro[i].y << "," << vGyro[i].z << endl;
    }

    cv::waitKey(10); // OpenCV窗口事件轮询，延时10ms
  }

  // 录制循环结束，关闭全部文件输出流
  accFile.close();
  gyroFile.close();
  cam0TsFile.close();
  cam1TsFile.close();

  cout << "System shutdown!\n";
}
