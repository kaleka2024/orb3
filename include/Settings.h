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
#pragma once
// Flag to activate the measurement of time in each process (track,localmap,
// place recognition).
// #define REGISTER_TIMES  // 开启该宏可以打印各模块耗时统计，默认关闭

#include <unistd.h>                   // POSIX系统调用头文件
#include <cstdio>                     // C标准输入输出
#include <cstdlib>                    // C标准库，exit退出函数
#include <iostream>                   // C++标准输入输出流
#include <memory>                     // 智能指针 std::shared_ptr
#include <string>                     // std::string字符串
#include <vector>                     // std::vector动态数组

#include "CameraModels/GeometricCamera.h" // 相机模型基类，针孔/鱼眼相机都继承此类
#include "Expected.h"                 // tl::expected错误处理工具类
#include "Logging.h"                  // 日志打印模块 oslog
#include "Types.h"                    // 基础类型定义，SensorType传感器枚举

namespace ORB_SLAM3 {

// 前向声明
class System;
class Settings;

// TODO: change to double instead of float  // 待优化：把float全部替换为double提升精度

/**
 * @brief 配置文件加载器，负责解析yaml配置文件，生成Settings配置对象
 */
class SettingsLoader {
 public:
  /// 返回值类型：成功返回Settings智能指针对象；失败返回ExpectedError错误信息
  typedef tl::expected<std::shared_ptr<Settings>, ExpectedError> Expected;

  /**
   * @brief 静态入口函数，直接加载配置文件，生成Settings实例
   * @param configFile yaml配置文件路径
   * @param sensor 传感器类型：MONOCULAR/STEREO/RGBD/IMU等
   * @param vocabFile 词袋词典文件路径，可选参数
   * @return Expected 包含Settings对象或者错误码
   */
  static Expected Load(const std::string& configFile, const SensorType sensor,
                       const std::string& vocabFile = "");

  /**
   * @brief 构造函数，传入传感器类型
   * @param sensor 传感器类型枚举
   */
  explicit SettingsLoader(const SensorType sensor);

  /**
   * @brief 成员函数，执行配置文件解析加载
   * @param configFile yaml配置路径
   * @param vocabFile 词典文件路径，可选
   * @return Expected 成功返回Settings，失败返回错误
   */
  Expected load(const std::string& configFile,
                const std::string& vocabFile = "");

 private:
  /**
   * @brief 读取第一个相机(左目/单目)标定参数
   * @param fSettings OpenCV FileStorage，yaml文件句柄
   */
  void readCamera1(cv::FileStorage& fSettings);

  /**
   * @brief 读取第二个相机(双目右目)标定参数
   * @param fSettings yaml文件句柄
   */
  void readCamera2(cv::FileStorage& fSettings);

  /**
   * @brief 读取图像尺寸、RGB标志等图像相关参数
   * @param fSettings yaml文件句柄
   */
  void readImageInfo(cv::FileStorage& fSettings);

  /**
   * @brief 读取IMU噪声、随机游走、频率、Tbc外参等IMU参数
   * @param fSettings yaml文件句柄
   */
  void readIMU(cv::FileStorage& fSettings);

  /**
   * @brief 读取RGBD模式深度缩放因子参数
   * @param fSettings yaml文件句柄
   */
  void readRGBD(cv::FileStorage& fSettings);

  /**
   * @brief 读取ORB特征提取器参数：特征数、金字塔层数、缩放因子、FAST阈值
   * @param fSettings yaml文件句柄
   */
  void readORB(cv::FileStorage& fSettings);

  /**
   * @brief 读取可视化Viewer相关参数，相机大小、点云大小、视角等
   * @param fSettings yaml文件句柄
   */
  void readViewer(cv::FileStorage& fSettings);

  /**
   * @brief 读取地图保存加载文件路径
   * @param fSettings yaml文件句柄
   */
  void readLoadAndSave(cv::FileStorage& fSettings);

  /**
   * @brief 读取其余杂项参数，远点阈值等
   * @param fSettings yaml文件句柄
   */
  void readOtherParameters(cv::FileStorage& fSettings);

  std::shared_ptr<Settings> settings_; ///< 保存解析完成的配置对象

  /**
   * @brief 模板工具函数：读取yaml中的单个参数，区分必填/可选参数
   * @tparam T 参数数据类型 int/float/bool/std::string
   * @param fSettings yaml文件句柄
   * @param name yaml配置项key名称
   * @param found 输出标记，true代表参数存在；false不存在
   * @param required true为必填参数，缺失直接程序退出；false可选，缺失返回默认值
   * @return T 返回读取到的参数值
   */
  template <typename T>
  T readParameter(cv::FileStorage& fSettings, const std::string& name,
                  bool& found, const bool required = true) {
    cv::FileNode node = fSettings[name];
    if (node.empty()) {
      if (required) {
        oslog::error("Required parameter \"{}\" does not exist, aborting...",
                     name);
        exit(-1);
      } else {
        oslog::warn("Optional parameter \"{}\" does not exist.", name);
        found = false;
        return T();
      }

    } else {
      found = true;
      return (T)node;
    }
  }
};

/**
 * @brief 全局配置类，保存SLAM全部运行参数，由SettingsLoader从yaml解析填充
 * @details 包含相机标定、IMU、ORB特征、可视化、RGBD、地图读写全部配置
 */
class Settings {
 public:
  friend class SettingsLoader; ///< 友元，允许加载器直接访问私有成员

  /*
   * Enum for the different camera types implemented
   */
  /// 支持的相机模型枚举
  enum CameraType { PinHole = 0, Rectified = 1, KannalaBrandt = 2 };

  /*
   * Delete default constructor
   */
  Settings() = delete; ///< 删除默认无参构造，必须传入传感器类型构造

  /**
   * @brief 构造函数，传入传感器类型
   * @param sensor 传感器类型 MONO/STEREO/RGBD等
   */
  explicit Settings(const SensorType sensor);

  Settings(const Settings&) = default; ///< 默认拷贝构造函数

  ~Settings(); ///< 析构函数

  // Safety checks
  /**
   * @brief 参数合法性校验，检查标定参数是否合法
   * @return true参数合法；false存在错误
   */
  bool validate();

  /*
   * Ostream operator overloading to dump settings to the terminal
   */
  friend std::ostream& operator<<(std::ostream& output, const Settings& s);

  /*
   * Getter methods 一系列get接口，获取配置参数
   */
  CameraType cameraType() const { return cameraType_; }
  std::shared_ptr<GeometricCamera> camera1() const { return calibration1_; }
  std::shared_ptr<GeometricCamera> camera2() const { return calibration2_; }

  /**
   * @brief 获取第一相机畸变系数cv::Mat
   * @return cv::Mat 畸变系数矩阵
   */
  cv::Mat camera1DistortionCoef() {
    return cv::Mat(vPinHoleDistorsion1_.size(), 1, CV_32F,
                   vPinHoleDistorsion1_.data());
  }

  /**
   * @brief 获取第二相机畸变系数cv::Mat
   * @return cv::Mat 畸变系数矩阵
   */
  cv::Mat camera2DistortionCoef() {
    return cv::Mat(vPinHoleDistorsion2_.size(), 1, CV_32F,
                   vPinHoleDistorsion2_.data());
  }

  Sophus::SE3f Tlr() { return Tlr_; }          ///< 获取双目左右相机之间位姿变换T_lr
  float bf() { return bf_; }                   ///< 获取双目焦距×基线 bf = f*b
  float b() { return b_; }                    ///< 获取双目基线b
  float thDepth() { return thDepth_; }         ///< 深度阈值，区分近点远点

  bool needToUndistort() { return bNeedToUndistort_; } ///< 是否需要去畸变

  cv::Size newImSize() { return newImSize_; } ///< 处理后图像尺寸
  bool rgb() { return bRGB_; }                 ///< 输入图像是否RGB格式，false为BGR
  bool needToResize() { return bNeedToResize1_; } ///< 是否需要缩放图像
  bool needToRectify() { return bNeedToRectify_; } ///< 是否需要双目校正

  float noiseGyro() { return noiseGyro_; }     ///< 陀螺仪噪声标准差
  float noiseAcc() { return noiseAcc_; }       ///< 加速度计噪声标准差
  float gyroWalk() { return gyroWalk_; }        ///< 陀螺仪随机游走噪声
  float accWalk() { return accWalk_; }         ///< 加速度计随机游走噪声
  float imuFrequency() { return imuFrequency_; } ///< IMU采样频率
  Sophus::SE3f Tbc() { return Tbc_; }          ///< IMU到相机的外参Tbc
  bool insertKFsWhenLost() { return insertKFsWhenLost_; } ///< 跟踪丢失时是否插入关键帧

  float depthMapFactor() { return depthMapFactor_; } ///< RGBD深度缩放系数

  int nFeatures() { return nFeatures_; }        ///< ORB总特征点数量
  int nLevels() { return nLevels_; }           ///< ORB金字塔层数
  float initThFAST() { return initThFAST_; }   ///< FAST初始阈值
  float minThFAST() { return minThFAST_; }     ///< FAST最小降级阈值
  float scaleFactor() { return scaleFactor_; } ///< ORB金字塔缩放因子

  bool useViewer() const { return useViewer_; } ///< 是否启用可视化窗口
  float keyFrameSize() { return keyFrameSize_; }
  float keyFrameLineWidth() { return keyFrameLineWidth_; }
  float graphLineWidth() { return graphLineWidth_; }
  float pointSize() { return pointSize_; }
  float cameraSize() { return cameraSize_; }
  float cameraLineWidth() { return cameraLineWidth_; }
  float viewPointX() { return viewPointX_; }
  float viewPointY() { return viewPointY_; }
  float viewPointZ() { return viewPointZ_; }
  float viewPointF() { return viewPointF_; }
  float imageViewerScale() { return imageViewerScale_; }

  std::string atlasLoadFile() { return sLoadFrom_; } ///< 地图加载文件路径
  std::string atlasSaveFile() { return sSaveto_; }   ///< 地图保存文件路径

  float thFarPoints() { return thFarPoints_; }        ///< 远点距离阈值

  cv::Mat M1l() { return M1l_; }
  cv::Mat M2l() { return M2l_; }
  cv::Mat M1r() { return M1r_; }
  cv::Mat M2r() { return M2r_; }

  // For PinHole,       k = {fx, fy, cx, cy}, and dist can be 0, 4 or 5 params
  // For Rectified,     k = {fx, fy, cx, cy}  and dist is ignored
  // For KannalaBrandt, k = {fx, fy, cx, cy, k0, k1, k2, k3};
  /**
   * @brief 设置单目相机参数
   * @param type 相机模型类型
   * @param k 内参数组 fx,fy,cx,cy
   * @param dist 畸变系数数组，针孔/鱼眼各不相同，默认为空
   */
  void setMonoCamera(CameraType type, const std::vector<float>& k,
                     const std::vector<float>& dist = {});

  /**
   * @brief 设置双目右相机参数
   * @param k2 右目内参
   * @param dist2 右目畸变系数
   * @param T_c1_c2 左到右相机位姿变换
   * @param thDepth 深度阈值
   */
  void setRightCamera(const std::vector<float>& k2,
                      const std::vector<float>& dist2, const cv::Mat& T_c1_c2,
                      float thDepth);

  /**
   * @brief 设置已经校正完毕的双目相机参数
   * @param k 共用内参
   * @param baseline 基线长度
   * @param thDepth 深度阈值
   */
  void setStereoRectifiedCamera(const std::vector<float>& k, float baseline,
                                 float thDepth);

  /**
   * @brief 设置原始图像分辨率
   * @param width 宽
   * @param height 高
   */
  void setOriginalImageSize(int width, int height);

  /**
   * @brief 设置缩放后处理图像分辨率
   * @param width 宽
   * @param height 高
   */
  void setResizeImageSize(int width, int height);

  /**
   * @brief 预计算双目校正映射M1l M2l M1r M2r
   */
  void precomputeRectificationMaps();

  SensorType sensor_;                         ///< 传感器类型枚举
  CameraType cameraType_;                     ///< 相机模型类型

  /*
   * Visual stuff 视觉相机相关成员
   */
  std::shared_ptr<GeometricCamera> calibration1_, calibration2_; ///< 左右目相机模型对象
  std::shared_ptr<GeometricCamera> originalCalib1_, originalCalib2_; ///< 原始未修改相机标定
  std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_; ///< 针孔畸变系数

  cv::Size originalImSize_, newImSize_;       ///< 原始图像尺寸、处理后图像尺寸
  bool bRGB_;                                 ///< true输入图像RGB；false BGR

  bool bNeedToUndistort_;                     ///< 是否执行去畸变
  bool bNeedToRectify_;                       ///< 是否双目校正
  bool bNeedToResize1_, bNeedToResize2_;       ///< 左右目是否需要缩放

  Sophus::SE3f Tlr_;                          ///< 左相机到右相机位姿T_lr
  float thDepth_;                             ///< 远近点深度阈值
  float bf_, b_;                              ///< bf=f*b，b基线长度

  /*
   * Rectification stuff 双目校正映射表
   */
  cv::Mat M1l_, M2l_; ///< 左目校正映射，OpenCV remap参数
  cv::Mat M1r_, M2r_; ///< 右目校正映射

  /*
   * Inertial stuff IMU相关参数
   */
  float noiseGyro_, noiseAcc_;    ///< 陀螺仪、加速度计测量噪声
  float gyroWalk_, accWalk_;      ///< 陀螺仪、加速度计随机游走噪声
  float imuFrequency_;            ///< IMU采样频率
  Sophus::SE3f Tbc_;              ///< IMU坐标系到相机坐标系外参Tbc
  bool insertKFsWhenLost_;        ///< 跟踪丢失时是否插入关键帧

  /*
   * RGBD stuff RGBD深度相机参数
   */
  float depthMapFactor_;          ///< 深度图缩放因子，深度值 / depthMapFactor = 真实米

  /*
   * ORB stuff ORB特征提取参数
   */
  int nFeatures_;                 ///< 金字塔总特征点数目
  float scaleFactor_;             ///< 金字塔缩放系数
  int nLevels_;                   ///< 金字塔层数
  int initThFAST_, minThFAST_;    ///< FAST角点初始阈值、最小降级阈值

  /*
   * Viewer stuff 可视化Viewer参数
   */
  bool useViewer_;
  float keyFrameSize_;
  float keyFrameLineWidth_;
  float graphLineWidth_;
  float pointSize_;
  float cameraSize_;
  float cameraLineWidth_;
  float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
  float imageViewerScale_;

  /*
   * Save & load maps 地图保存加载路径
   */
  std::string sLoadFrom_, sSaveto_;

  /*
   * Other stuff 其他杂项参数
   */
  float thFarPoints_;             ///< 远点判定距离阈值

  bool loopClosing_;              ///< 是否开启回环检测
  std::string strVocFile_;        ///< ORB词袋词典文件路径
};

};  // namespace ORB_SLAM3
