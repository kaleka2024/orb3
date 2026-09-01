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
#include <stdio.h>                    // C标准IO
#include <stdlib.h>                   // C标准库
#include <unistd.h>                   // POSIX系统调用

#include <memory>                     // 智能指针 shared_ptr unique_ptr
#include <opencv2/core/core.hpp>      // OpenCV核心模块
#include <string>                     // std::string字符串
#include <thread>                     // C++多线程
#include <vector>                     // std::vector容器

#include "Atlas.h"                    // 地图集，管理多地图
#include "Expected.h"                 // tl::expected错误返回工具
#include "FrameDrawer.h"              // 帧画面绘制器
#include "ImuTypes.h"                 // IMU数据结构定义
#include "KeyFrameDatabase.h"         // 关键帧数据库，用于回环/重定位
#include "LocalMapping.h"             // 局部建图模块
#include "Logging.h"                  // 日志打印工具
#include "LoopClosing.h"              // 回环检测模块
#include "MapDrawer.h"                // 地图3D绘制器
#include "ORBVocabulary.h"            // ORB词袋词典
#include "Settings.h"                 // 系统配置参数类
#include "System/Factory.h"           // System工厂创建类
#include "Tracking.h"                 // 跟踪主线程模块
#include "Utils/FpsEstimator.h"       // FPS统计工具
#include "Viewer.h"                   // Pangolin可视化查看器

namespace ORB_SLAM3 {

// 前向声明
class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

// System should be created using  SystemFactory::create()
//
// It will validate settings and catch errors on startup
/**
 * @brief System类，ORB‑SLAM3对外顶层API接口，整个SLAM系统入口
 * @details 不允许直接new构造，必须通过SystemFactory::create工厂函数创建；
 * 管理所有子模块、线程，对外提供TrackMonocular/TrackStereo/TrackRGBD跟踪接口，
 * 提供模式切换、重置、关闭、轨迹保存等功能。
 */
class System : public std::enable_shared_from_this<System> {
 public:
  friend SystemFactory::Expected SystemFactory::create(
      const std::shared_ptr<Settings> &, bool, const string &);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐宏，类中包含Eigen成员必须添加

  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  /**
   * @brief 处理双目图像帧，双目图像必须时间同步且已经完成校正
   * @param imLeft 左目图像，支持RGB(CV_8UC3)或灰度(CV_8U)，内部会转灰度
   * @param imRight 右目图像
   * @param timestamp 图像时间戳
   * @param vImuMeas 当前帧之前累积的IMU测量值，VIO模式传入，纯视觉留空
   * @param filename 调试用文件名，可选
   * @return Sophus::SE3f Tcw 相机位姿(世界到相机)，跟踪失败返回无效位姿
   */
  Sophus::SE3f TrackStereo(
      cv::InputArray imLeft, cv::InputArray imRight, double timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Input depthmap: Float (CV_32F). Returns the camera pose (empty
  // if tracking fails).
  /**
   * @brief 处理RGBD深度相机帧，深度图需要和彩色图配准对齐
   * @param im 彩色/灰度图像
   * @param depthmap 深度图，CV_32F格式，单位米
   * @param timestamp 时间戳
   * @param vImuMeas IMU测量序列，VIO使用
   * @param filename 调试文件名
   * @return Sophus::SE3f Tcw相机位姿，跟踪失败返回无效位姿
   */
  Sophus::SE3f TrackRGBD(
      cv::InputArray im, cv::InputArray depthmap, double timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  // Proccess the given monocular frame and optionally imu data
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  /**
   * @brief 处理单目图像帧，VIO模式附带IMU数据
   * @param im 输入图像RGB/灰度
   * @param timestamp 图像时间戳
   * @param vImuMeas 该帧前IMU测量数据
   * @param filename 调试文件名
   * @return Sophus::SE3f Tcw相机位姿，跟踪失败返回无效位姿
   */
  Sophus::SE3f TrackMonocular(
      cv::InputArray im, double timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  // This stops local mapping thread (map building) and performs only camera
  // tracking.
  /**
   * @brief 激活定位模式：停止局部建图线程，只做跟踪+重定位，不新增地图点、关键帧
   */
  void ActivateLocalizationMode();

  // This resumes local mapping thread and performs SLAM again.
  /**
   * @brief 关闭定位模式，恢复局部建图线程，回到完整SLAM模式
   */
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  /**
   * @brief 查询自上次调用本函数后地图是否发生重大变更（回环、全局BA）
   * @return true地图发生大改动；false无重大改动
   */
  bool MapChanged();

  // Reset the system (clear Atlas or the active map)
  /**
   * @brief 系统完全重置，清空整个Atlas地图集，重置跟踪状态
   */
  void Reset();

  /**
   * @brief 仅重置当前激活的地图，保留其他子地图
   */
  void ResetActiveMap();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  /**
   * @brief 请求所有子线程退出，并阻塞等待线程结束；保存轨迹前必须调用Shutdown
   */
  void Shutdown();

  /**
   * @brief 查询系统是否已经执行Shutdown关闭
   * @return true已关闭；false运行中
   */
  bool isShutDown();

  // Save camera trajectory in the TUM RGB‑D dataset format.
  // Only for stereo and RGB‑D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd‑dataset
  /**
   * @brief 保存相机轨迹为TUM‑RGBD格式；仅双目/RGBD可用，单目不可用；调用前先Shutdown
   * @param filename 输出文件路径
   */
  void SaveTrajectoryTUM(const string &filename);

  // Save keyframe poses in the TUM RGB‑D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd‑dataset
  /**
   * @brief 保存关键帧位姿TUM格式；单目/双目/RGBD全部支持；调用前Shutdown
   * @param filename 输出文件路径
   */
  void SaveKeyFrameTrajectoryTUM(const string &filename);

  /**
   * @brief EuRoC数据集格式保存相机轨迹
   * @param filename 输出文件路径
   */
  void SaveTrajectoryEuRoC(const string &filename);

  /**
   * @brief EuRoC格式保存关键帧轨迹
   * @param filename 输出文件路径
   */
  void SaveKeyFrameTrajectoryEuRoC(const string &filename);

  /**
   * @brief EuRoC格式保存指定地图的相机轨迹，多地图场景使用
   * @param filename 输出路径
   * @param pMap 指定目标地图
   */
  void SaveTrajectoryEuRoC(const string &filename,
                           const std::shared_ptr<Map> &pMap);

  /**
   * @brief EuRoC格式保存指定地图关键帧轨迹
   * @param filename 输出路径
   * @param pMap 指定目标地图
   */
  void SaveKeyFrameTrajectoryEuRoC(const string &filename,
                                    const std::shared_ptr<Map> &pMap);

  // Save data used for initialization debug
  /**
   * @brief 保存初始化阶段调试数据
   * @param iniIdx 初始化序号索引
   */
  void SaveDebugData(const int &iniIdx);

  // Save camera trajectory in the KITTI dataset format.
  // Only for stereo and RGB‑D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at:
  // http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  /**
   * @brief KITTI里程计格式保存轨迹；双目/RGBD可用，单目不支持；调用前Shutdown
   * @param filename 输出文件路径
   */
  void SaveTrajectoryKITTI(const string &filename);

  // \todo{} Serialization is currently broken
  // SaveMap(const string &filename);
  // LoadMap(const string &filename);

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  /**
   * @brief 获取最近一帧的跟踪状态
   * @return 跟踪状态枚举：OK / LOST / NOT_INITIALIZED等
   */
  int GetTrackingState();

  /**
   * @brief 获取当前帧成功跟踪上的地图点
   * @return 地图点指针vector
   */
  std::vector<MapPoint *> GetTrackedMapPoints();

  /**
   * @brief 获取当前帧未去畸变的被跟踪关键点
   * @return cv::KeyPoint数组
   */
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

  std::shared_ptr<FrameDrawer> frameDrawer() { return mpFrameDrawer; }
  std::shared_ptr<MapDrawer> mapDrawer() { return mpMapDrawer; }

  const SensorType sensorType() const { return settings_->sensor_; }

  // For debugging
  /**
   * @brief 获取从IMU初始化完成到现在的时间，VIO调试用
   * @return 时间秒
   */
  double GetTimeFromIMUInit();

  /**
   * @brief 判断当前跟踪是否丢失
   * @return true丢失；false正常跟踪
   */
  bool isLost();

  /**
   * @brief 判断系统是否已经结束运行
   * @return true结束；false运行中
   */
  bool isFinished();

  float fps() const { return fps_estimator_.fps(); }

  /**
   * @brief 切换数据集，调试工具
   */
  void ChangeDataset();

  /**
   * @brief 获取图像缩放系数
   * @return 缩放比例float
   */
  float GetImageScale();

#ifdef REGISTER_TIMES
  void InsertRectTime(double &time);    // 记录去畸变校正耗时，REGISTER_TIMES宏开启才生效
  void InsertResizeTime(double &time);  // 记录图像缩放耗时
  void InsertTrackTime(double &time);   // 记录跟踪耗时
#endif

 protected:
  // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and
  // Viewer threads.
  //
  // All construction should go through the factory to ensure correct
  // initialization
  /**
   * @brief System构造函数，protected保护，外部不能直接调用，只能由SystemFactory工厂调用
   * @param settings 配置参数智能指针
   * @param initFr 是否初始化第一帧
   * @param strSequence 序列名称字符串
   */
  System(const std::shared_ptr<Settings> &settings, bool initFr = false,
         const string &strSequence = std::string());

  /**
   * @brief 打印程序启动banner版权信息
   */
  void printBanner();

  /**
   * @brief 内部初始化函数，启动各个子线程：局部建图、回环、可视化
   * @param initFr 是否初始化第一帧
   * @param strSequence 数据集序列名
   * @return true初始化成功；false失败
   */
  bool initialize(bool initFr = false,
                   const string &strSequence = std::string());

  /**
   * @brief 处理定位模式切换请求，线程间模式变更逻辑
   */
  void processLocalizationModeChange(void);

  /**
   * @brief 执行系统重置内部逻辑
   */
  void processReset(void);

  /**
   * @brief 更新缓存的跟踪状态，供外部API查询
   */
  void updateTrackingState();

  /**
   * @brief 将Atlas地图集保存到文件
   * @param type 文件类型枚举
   */
  void SaveAtlas(FileType type);

  /**
   * @brief 从磁盘加载Atlas地图集
   * @param type 文件类型枚举
   * @return true加载成功；false失败
   */
  bool LoadAtlas(FileType type);

  // ORB vocabulary used for place recognition and feature matching.
  std::shared_ptr<ORBVocabulary> mpVocabulary;           ///< ORB词袋词典，回环、重定位使用

  // KeyFrame database for place recognition (relocalization and loop
  // detection).
  std::shared_ptr<KeyFrameDatabase> mpKeyFrameDatabase;  ///< 关键帧数据库，词袋检索

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  // Map* mpMap;
  std::shared_ptr<Atlas> mpAtlas;                        ///< Atlas地图集，管理多个子地图

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints
  // and performs relocalization if tracking fails.
  std::shared_ptr<Tracking> mpTracker;                   ///< 跟踪模块，运行于主线程

  // Local Mapper. It manages the local map and performs local bundle
  // adjustment.
  std::shared_ptr<LocalMapping> mpLocalMapper;            ///< 局部建图模块，独立线程

  // Loop Closer. It searches loops with every new keyframe. If there is a loop
  // it performs a pose graph optimization and full bundle adjustment (in a new
  // thread) afterwards.
  std::shared_ptr<LoopClosing> mpLoopCloser;              ///< 回环检测模块，独立线程

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  std::shared_ptr<Viewer> mpViewer;                      ///< Pangolin可视化器，独立线程

  std::shared_ptr<FrameDrawer> mpFrameDrawer;             ///< 帧图像绘制，显示特征点跟踪结果
  std::shared_ptr<MapDrawer> mpMapDrawer;                 ///< 3D地图绘制，点云、关键帧、位姿

  // System threads: Local Mapping, Loop Closing, Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the
  // System object.
  std::unique_ptr<std::thread> mptLocalMapping;           ///< 局部建图线程句柄
  std::unique_ptr<std::thread> mptLoopClosing;            ///< 回环检测线程句柄
  std::unique_ptr<std::thread> mptViewer;                 ///< 可视化Viewer线程句柄

  // Reset flag
  std::mutex mMutexReset;                                 ///< 重置操作互斥锁
  bool mbReset;                                           ///< 全局系统重置标记
  bool mbResetActiveMap;                                  ///< 仅重置当前激活地图标记

  // Change mode flags
  std::mutex mMutexMode;                                  ///< 模式切换互斥锁
  bool mbActivateLocalizationMode;                        ///< 请求激活定位模式标记
  bool mbDeactivateLocalizationMode;                      ///< 请求退出定位模式标记

  // Shutdown flag
  bool mbShutDown;                                        ///< 系统关闭标记

  // Tracking state
  int mTrackingState;                                     ///< 缓存的跟踪状态，供外部查询
  std::vector<MapPoint *> mTrackedMapPoints;              ///< 缓存当前帧跟踪到的地图点
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;          ///< 缓存当前帧未畸变关键点
  std::mutex mMutexState;                                 ///< 跟踪状态读写互斥锁

  std::shared_ptr<Settings> settings_;                    ///< 系统全部配置参数

  FpsEstimator fps_estimator_;                            ///< FPS帧率统计工具
};

}  // namespace ORB_SLAM3
