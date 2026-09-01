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
#include <signal.h>                 // Linux信号处理，捕获Ctrl‑C中断
#include <stdlib.h>

#include <algorithm>
#include <chrono>                   // 时间时钟
#include <condition_variable>       // 条件变量，线程间等待通知
#include <ctime>
#include <fstream>                  // 文件读写ofstream/ifstream
#include <iomanip>                  // setprecision 输出精度控制
#include <iostream>                 // 标准输入输出cout cerr
#include <librealsense2/rs.hpp>     // RealSense设备SDK主头文件
#include <opencv2/core/core.hpp>    // OpenCV核心Mat数据结构
#include <opencv2/highgui.hpp>     // OpenCV窗口、imshow、imwrite
#include <sstream>

#include "librealsense2/rsutil.h"  // realsense工具函数

using namespace std;

bool b_continue_session; ///< 主循环运行标志位，Ctrl‑C置false退出录制

/**
 * @brief SIGINT信号回调函数，捕获Ctrl‑C，结束录制会话
 * @param s 信号编号
 */
void exit_loop_handler(int s) {
  cout << "Finishing session" << endl;
  b_continue_session = false;
}

/**
 * @brief 遍历打印传感器全部可配置选项，查看支持项、描述、当前值
 * @param sensor realsense传感器对象（相机/陀螺仪/加速度计）
 * @return 返回选中的sensor option，本示例固定返回0
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
  // 参数校验：需要传入保存数据集文件夹路径
  if (argc != 2) {
    cerr << endl
         << "Usage: ./recorder_realsense_D435i path_to_saving_folder" << endl;
    return 1;
  }

  string directory = string(argv[argc - 1]); ///< 数据集输出根目录

  // 注册Ctrl‑C信号处理
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = exit_loop_handler; ///< 设置中断回调函数
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);
  b_continue_session = true;

  double offset = 0;  // ms 时间戳偏移量，本示例未实际使用

  rs2::context ctx;                          ///< realsense上下文对象
  rs2::device_list devices = ctx.query_devices(); ///< 查询已连接设备列表
  rs2::device selected_device;
  if (devices.size() == 0) {
    std::cerr << "No device connected, please connect a RealSense device"
              << std::endl;
    return 0;
  } else
    selected_device = devices[0]; ///< 取第一个连接的RealSense设备(D435i)

  std::vector<rs2::sensor> sensors = selected_device.query_sensors(); ///< 获取设备全部传感器：红外相机、RGB、IMU
  int index = 0;
  // We can now iterate the sensors and print their names
  for (rs2::sensor sensor : sensors)
    if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
      ++index;
      if (index == 1) {
        // 深度红外传感器：开启自动曝光、设置曝光上限、关闭红外投射器
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 5000);
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
      }
      // std::cout << "  " << index << " : " <<
      // sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
      get_sensor_option(sensor);
      if (index == 2) {
        // RGB camera，设置RGB相机固定曝光值
        sensor.set_option(RS2_OPTION_EXPOSURE, 100.f);
      }

      if (index == 3) {
        // IMU传感器，关闭运动硬件校正
        sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
      }
    }

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe; ///< RealSense数据流管道，管理设备流
  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;    ///< 流配置对象，配置分辨率、格式、帧率
  // 开启左红外流：640×480，8位灰度，30Hz
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
  // 开启加速度计IMU流，XYZ32F浮点格式
  cfg.enable_stream(RS2_STREAM_ACCEL,
                    RS2_FORMAT_MOTION_XYZ32F);                   //, 250); // 63
  // 开启陀螺仪IMU流，XYZ32F浮点格式
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);  //, 400);

  // IMU callback 多线程同步变量
  std::mutex imu_mutex;                     ///< 保护图像、IMU数据缓冲区互斥锁
  std::condition_variable cond_image_rec;   ///< 条件变量：主线程等待图像帧就绪

  vector<double> v_gyro_timestamp;          ///< 陀螺仪时间戳，单位秒
  vector<rs2_vector> v_gyro_data;           ///< 陀螺仪三轴角速度数据
  vector<double> v_acc_timestamp;           ///< 加速度计时间戳，单位秒
  vector<rs2_vector> v_acc_data;            ///< 加速度计三轴加速度数据

  cv::Mat imCV;                             ///< 回调中保存红外图像Mat，外部线程访问
  int width_img, height_img;               ///< 图像宽高，从相机内参读取
  double timestamp_image;                   ///< 当前图像帧时间戳，单位秒
  bool image_ready = false;                 ///< 标记回调是否收到新图像帧

  /**
   * @brief RealSense帧回调lambda，SDK后台线程执行
   * @param frame realsense帧对象，可以是frameset图像组，也可以是motion_frame IMU帧
   */
  auto imu_callback = [&](const rs2::frame& frame) {
    std::unique_lock<std::mutex> lock(imu_mutex);

    // 如果是图像帧集合frameset
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      rs2::video_frame color_frame = fs.get_infrared_frame();
      // 把realsense图像内存包装成OpenCV Mat，不拷贝数据(AUTO_STEP)
      imCV = cv::Mat(cv::Size(width_img, height_img), CV_8U,
                     (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);

      timestamp_image = fs.get_timestamp() * 1e-3; // ms → s
      image_ready = true;

      lock.unlock();
      cond_image_rec.notify_all(); // 唤醒主线程等待，图像就绪
    }
    // 如果是IMU运动帧（陀螺仪/加速度计）
    else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()) {
      if (m_frame.get_profile().stream_name() == "Gyro") {
        // It runs at 200Hz，存入陀螺仪缓冲区
        v_gyro_data.push_back(m_frame.get_motion_data());
        v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
      } else if (m_frame.get_profile().stream_name() == "Accel") {
        // It runs at 60Hz，存入加速度计缓冲区
        v_acc_data.push_back(m_frame.get_motion_data());
        v_acc_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
      }
    }
  };

  // 启动pipeline，注册帧回调函数
  rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);
  rs2::stream_profile cam_stream =
      pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
  rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);

  // 获取相机内参，提取图像宽高
  rs2_intrinsics intrinsics_cam =
      cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
  width_img = intrinsics_cam.width;
  height_img = intrinsics_cam.height;

  cv::Mat im;
  ofstream accFile, gyroFile, cam0TsFile;
  // 打开输出文件：加速度、陀螺仪、相机时间戳
  accFile.open(directory + "/IMU/acc.txt");
  gyroFile.open(directory + "/IMU/gyro.txt");
  cam0TsFile.open(directory + "/cam0/times.txt");

  // Clear IMU vectors，清空全局IMU缓存
  v_gyro_data.clear();
  v_gyro_timestamp.clear();
  v_acc_data.clear();
  v_acc_timestamp.clear();

  cv::namedWindow("cam0", cv::WINDOW_AUTOSIZE); ///< OpenCV显示红外图像窗口

  // 主录制循环，Ctrl‑C修改b_continue_session退出
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
          cond_image_rec.wait(lk); // 阻塞等待回调线程通知图像就绪
      }
      std::lock_guard<std::mutex> lk(imu_mutex);

      // Copy the IMU data to local single thread variables
      // 将回调线程缓冲区拷贝到主线程局部变量，避免多线程竞争
      vGyro = v_gyro_data;
      vGyro_times = v_gyro_timestamp;
      vAccel = v_acc_data;
      vAccel_times = v_acc_timestamp;
      imTs = timestamp_image;
      im = imCV.clone(); // 深拷贝图像，防止回调覆盖内存

      // Clear IMU vectors，清空回调缓冲区，准备接收下一批IMU采样
      v_gyro_data.clear();
      v_gyro_timestamp.clear();
      v_acc_data.clear();
      v_acc_timestamp.clear();

      image_ready = false;
    }

    cv::imshow("cam0", im);

    // save image and IMU data，时间戳转为纳秒整数作为图片文件名
    long int imTsInt = (long int)(1e9 * imTs);
    string imgRepo = directory + "/cam0/" + to_string(imTsInt) + ".png";
    if (!im.empty()) {
      cv::imwrite(imgRepo, im);         // 保存红外png图片
      cam0TsFile << imTsInt << endl;    // 写入相机时间戳文本
    } else {
      cout << "image empty!! \n";
    }

    // assert(vAccel.size() == vAccel_times.size());
    // assert(vGyro.size() == vGyro_times.size());

    // 写入加速度计数据文件：timestamp,ax,ay,az
    for (int i = 0; i < vAccel.size(); ++i) {
      accFile << std::setprecision(15) << vAccel_times[i] << "," << vAccel[i].x
              << "," << vAccel[i].y << "," << vAccel[i].z << endl;
    }

    // 写入陀螺仪数据文件：timestamp,gx,gy,gz
    for (int i = 0; i < vGyro.size(); ++i) {
      gyroFile << std::setprecision(15) << vGyro_times[i] << "," << vGyro[i].x
               << "," << vGyro[i].y << "," << vGyro[i].z << endl;
    }

    cv::waitKey(10); // OpenCV窗口事件等待，10ms延时
  }

  // 循环退出，关闭所有文件流
  accFile.close();
  gyroFile.close();
  cam0TsFile.close();

  cout << "System shutdown!\n";
}
