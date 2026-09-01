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
#include <memory>        // std::shared_ptr 智能指针
#include <mutex>         // std::mutex 互斥锁，多线程同步

#include "FrameDrawer.h"  // 帧绘制器：负责图像、特征点可视化
#include "MapDrawer.h"    // 地图绘制器：负责地图点、关键帧可视化
#include "Settings.h"     // 配置参数管理类
#include "System.h"       // SLAM系统总入口
#include "Tracking.h"     // 跟踪线程，负责相机位姿求解

namespace ORB_SLAM3 {

// 前向声明，避免头文件循环依赖
class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Settings;

/**
 * @brief Viewer 可视化查看器类，基于Pangolin实现GUI窗口
 * @details 独立的UI线程；负责渲染图像窗口、3D地图窗口；
 * 控制可视化刷新帧率，提供停止、结束、单步调试等控制接口；
 * 和Tracking、FrameDrawer、MapDrawer配合，读取数据并绘制。
 */
class Viewer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen内存对齐宏，类中含有Eigen成员时启用

  /**
   * @brief Viewer构造函数
   * @param pSystem SLAM系统主对象指针
   * @param pFrameDrawer 帧绘制器智能指针，绘制图像与特征点
   * @param pMapDrawer 地图绘制器智能指针，绘制3D地图点、关键帧
   * @param pTracking 跟踪模块智能指针，读取跟踪状态
   * @param settings 配置参数智能指针，读取窗口、视角、帧率等参数
   */
  Viewer(System *pSystem, const std::shared_ptr<FrameDrawer> &pFrameDrawer,
         const std::shared_ptr<MapDrawer> &pMapDrawer,
         const std::shared_ptr<Tracking> &pTracking,
         const std::shared_ptr<Settings> &settings);

  // Main thread function. Draw points, keyframes, the current camera pose and
  // the last processed frame. Drawing is refreshed according to the camera fps.
  // We use Pangolin.
  /**
   * @brief 可视化线程主循环函数，在独立线程运行
   * @details 使用Pangolin创建GUI窗口；绘制图像窗口、3D地图窗口；
   * 绘制地图点、关键帧、相机位姿、当前帧；按照相机FPS控制刷新间隔。
   */
  void Run();

  /**
   * @brief 请求结束可视化线程，发出退出请求标记
   */
  void RequestFinish();

  /**
   * @brief 请求暂停可视化，发出停止请求标记
   */
  void RequestStop();

  /**
   * @brief 查询可视化线程是否已经完全结束
   * @return true 线程已结束；false 仍在运行
   */
  bool isFinished();

  /**
   * @brief 查询可视化是否处于暂停状态
   * @return true 已暂停；false 正常运行
   */
  bool isStopped();

  /**
   * @brief 查询是否处于单步调试模式
   * @return true 单步模式；false 连续刷新模式
   */
  bool isStepByStep();

  /**
   * @brief 释放暂停状态，继续可视化渲染
   */
  void Release();

  // void SetTrackingPause();

  bool both;  ///< 控制是否同时显示图像窗口与3D地图窗口

 private:
  /**
   * @brief 执行真正的暂停逻辑，内部私有函数
   * @return true 成功进入暂停
   */
  bool Stop();

  System *mpSystem;                          ///< SLAM系统主对象原始指针
  std::shared_ptr<FrameDrawer> mpFrameDrawer; ///< 帧绘制器智能指针
  std::shared_ptr<MapDrawer> mpMapDrawer;     ///< 地图绘制器智能指针
  std::shared_ptr<Tracking> mpTracker;        ///< 跟踪模块智能指针

  // 1/fps in ms
  double mT;               ///< 可视化刷新时间间隔，单位ms，等于1000.0/fps
  float mImageWidth, mImageHeight;  ///< 图像显示窗口宽高
  float mImageViewerScale; ///< 图像窗口缩放系数

  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF; ///< Pangolin 3D视角参数：相机位置与焦距

  /**
   * @brief 检查是否收到结束请求
   * @return true 需要结束；false 继续运行
   */
  bool CheckFinish();

  /**
   * @brief 设置结束标记，标记线程已经退出
   */
  void SetFinish();

  bool mbFinishRequested;  ///< 是否收到结束线程请求
  bool mbFinished;         ///< 标记可视化线程是否已经执行完毕退出
  std::mutex mMutexFinish; ///< 结束状态互斥锁，保护mbFinishRequested、mbFinished多线程访问

  bool mbStopped;          ///< 标记当前是否已经处于暂停状态
  bool mbStopRequested;    ///< 是否收到暂停请求
  std::mutex mMutexStop;   ///< 暂停状态互斥锁，保护mbStopped、mbStopRequested

  bool mbStopTrack;        ///< 跟踪暂停标记，用于单步调试模式
};

}  // namespace ORB_SLAM3
