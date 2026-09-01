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
#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <iostream>
#include <memory>
#include <mutex>

namespace ORB_SLAM3 {

// 查看器构造函数：初始化可视化模块，接收系统、帧绘制器、地图绘制器、跟踪器、配置参数
Viewer::Viewer(System *pSystem,
               const std::shared_ptr<FrameDrawer> &pFrameDrawer,
               const std::shared_ptr<MapDrawer> &pMapDrawer,
               const std::shared_ptr<Tracking> &pTracking,
               const std::shared_ptr<Settings> &settings)
    : both(false),                          // 是否双目模式，false仅显示左图
      mpSystem(pSystem),                   // SLAM系统主类指针
      mpFrameDrawer(pFrameDrawer),         // 帧绘制器智能指针，负责图像帧渲染
      mpMapDrawer(pMapDrawer),             // 地图绘制器智能指针，负责3D地图渲染
      mpTracker(pTracking),                // 跟踪模块智能指针
      mbFinishRequested(false),            // 请求结束查看器标志位
      mbFinished(true),                    // 查看器是否已经结束运行
      mbStopped(true),                     // 查看器是否处于停止状态
      mbStopRequested(false),              // 请求停止查看器标志位
      mbStopTrack(false)                   // 是否暂停跟踪（单步模式触发）
{
  mImageViewerScale = 1.f;                // 图像窗口缩放比例，初始化为1倍

  mT = 1e3 / 30;                          // 可视化刷新间隔，单位ms，30FPS，1000/30≈33ms

  cv::Size imSize = settings->newImSize();// 从配置读取处理后图像分辨率
  mImageHeight = imSize.height;           // 图像高度
  mImageWidth = imSize.width;             // 图像宽度

  mImageViewerScale = settings->imageViewerScale(); // 配置文件读取图像显示缩放系数
  mViewpointX = settings->viewPointX();   // Pangolin 3D视角X初始位置
  mViewpointY = settings->viewPointY();   // Pangolin 3D视角Y初始位置
  mViewpointZ = settings->viewPointZ();   // Pangolin 3D视角Z初始位置
  mViewpointF = settings->viewPointF();   // Pangolin相机焦距参数
}

// 查看器主运行循环，运行在独立Viewer线程，负责Pangolin3D窗口、OpenCV图像窗口、UI菜单交互
void Viewer::Run()
{
  mbFinished = false;                     // 标记查看器未结束
  mbStopped = false;                      // 标记查看器非停止状态

  // 创建Pangolin可视化窗口：窗口标题ORB‑SLAM3: Map Viewer，分辨率宽1024，高768
  pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer", 1024, 768);

  // 开启OpenGL深度测试，3D鼠标交互必须，处理物体遮挡渲染
  glEnable(GL_DEPTH_TEST);

  // 开启OpenGL混合，支持半透明绘制
  glEnable(GL_BLEND);
  // 设置混合因子：源颜色使用源alpha，目标颜色使用1‑源alpha，实现半透明效果
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // 创建左侧控制面板panel，Bounds参数：上下[0,1]，左右[0，像素175]，左侧占175像素宽度
  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(175));
  // 菜单控件：跟随相机视角，默认false，true代表可交互勾选框
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
  // 菜单控件：切换到相机视角，默认false，false代表按钮（非勾选）
  pangolin::Var<bool> menuCamView("menu.Camera View", false, false);
  // 菜单控件：切换到俯视全局视角，默认false，按钮
  pangolin::Var<bool> menuTopView("menu.Top View", false, false);
  // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
  // 菜单控件：显示地图点，默认勾选true
  pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
  // 菜单控件：显示关键帧，默认勾选true
  pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
  // 菜单控件：显示共视图边，默认不勾选false
  pangolin::Var<bool> menuShowGraph("menu.Show Graph", false, true);
  // 菜单控件：显示IMU惯性图，默认勾选true
  pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph", true,
                                            true);
  // 菜单控件：定位模式开关，仅定位不建图，默认false
  pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false,
                                           true);
  // 菜单控件：重置SLAM系统，按钮，默认false
  pangolin::Var<bool> menuReset("menu.Reset", false, false);
  // 菜单控件：停止系统，按钮，默认false
  pangolin::Var<bool> menuStop("menu.Stop", false, false);
  // 菜单控件：单步调试模式，一帧一帧运行，默认false可勾选
  pangolin::Var<bool> menuStepByStep("menu.Step By Step", false,
                                     true);  // false, true
  // 菜单控件：单步执行一次，按钮，默认false
  pangolin::Var<bool> menuStep("menu.Step", false, false);

  // 菜单控件：显示LBA局部光束调整优化标记，默认false
  pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
  // 定义Pangolin渲染状态对象s_cam，管理3D投影矩阵与视角矩阵
  // ProjectionMatrix：窗口宽高，焦距fx fy，主点cx cy，近裁剪面0.1，远裁剪面1000
  // ModelViewLookAt：相机位置(x,y,z)，看向目标点(0,0,0)，相机上方向向量(0,-1,0)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0,
                                0.0, -1.0, 0.0));

  // 创建3D显示视口d_cam；Bounds：上下0‑1，左侧从像素175到窗口最右；设置宽高比；挂载3D鼠标交互处理器Handler3D
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc, Twr;   // Twc：相机世界位姿矩阵；Twr备用矩阵
  Twc.SetIdentity();                 // 初始化为单位矩阵
  pangolin::OpenGlMatrix Ow;         // IMU重力对齐坐标系矩阵，z轴沿重力方向
  Ow.SetIdentity();                  // 初始化为单位矩阵
  // 创建OpenCV图像窗口，显示当前帧图像
  cv::namedWindow("ORB-SLAM3: Current Frame");

  bool bFollow = true;               // 内部状态：是否开启相机跟随
  bool bLocalizationMode = false;    // 内部状态：是否处于纯定位模式
  bool bStepByStep = false;          // 内部状态：是否开启单步调试模式
  bool bCameraView = true;           // 内部状态：true相机视角，falseIMU俯视全局视角

  // 如果传感器是单目/双目/RGBD，打开共视图显示开关
  if (mpTracker->mSensor == SensorType::MONOCULAR ||
      mpTracker->mSensor == SensorType::STEREO ||
      mpTracker->mSensor == SensorType::RGBD) {
    menuShowGraph = true;
  }

  // 获取跟踪模块内部图像缩放系数
  float trackedImageScale = mpTracker->GetImageScale();

  cout << "Starting the Viewer" << endl;
  // Viewer主线无限循环，直到收到退出请求
  while (1)
  {
    // OpenGL清除颜色缓冲区、深度缓冲区，准备新一帧绘制
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 从MapDrawer获取当前相机位姿Twc，IMU重力对齐姿态Ow
    mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc, Ow);

    // mbStopTrack置位，切换进入单步模式
    if (mbStopTrack)
    {
      menuStepByStep = true;
      mbStopTrack = false;
    }

    // 开启跟随相机并且内部bFollow为true，让Pangolin视角跟随当前相机位姿Twc/Ow
    if (menuFollowCamera && bFollow)
    {
      if (bCameraView)
        s_cam.Follow(Twc);   // 相机视角，跟随相机位姿Twc
      else
        s_cam.Follow(Ow);    // IMU全局俯视视角，跟随重力对齐Ow
    }
    // UI勾选跟随相机，但bFollow=false，做一次初始化视角切换后开启跟随
    else if (menuFollowCamera && !bFollow)
    {
      if (bCameraView)
      {
        // 设置投影矩阵
        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
            1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000));
        // 设置初始观察视角
        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
            mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
        s_cam.Follow(Twc); // 视角跟随相机
      }
      else
      {
        // IMU俯视模式，使用更大焦距，更远观测距离
        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
            1024, 768, 3000, 3000, 512, 389, 0.1, 1000));
        s_cam.SetModelViewMatrix(
            pangolin::ModelViewLookAt(0, 0.01, 10, 0, 0, 0, 0.0, 0.0, 1.0));
        s_cam.Follow(Ow);
      }
      bFollow = true; // 更新内部状态标记跟随已打开
    }
    // UI取消跟随相机，把内部bFollow置false，停止视角跟随
    else if (!menuFollowCamera && bFollow)
    {
      bFollow = false;
    }

    // 点击【Camera View】按钮，切换回相机第一人称视角
    if (menuCamView)
    {
      menuCamView = false; // 按钮复位
      bCameraView = true;  // 标记为相机视角模式
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
          1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
          mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
      s_cam.Follow(Twc); // 视角绑定当前相机位姿
    }

    // 点击【Top View】按钮，IMU完成初始化后，切换到全局俯视上帝视角
    if (menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
    {
      menuTopView = false; // 按钮复位
      bCameraView = false; // 标记非相机视角，使用Ow重力对齐坐标系
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
          1024, 768, 3000, 3000, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(
          pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
      s_cam.Follow(Ow); // 视角跟随重力对齐坐标系
    }

    // UI勾选Localization Mode，激活纯定位模式，不再新建地图
    if (menuLocalizationMode && !bLocalizationMode)
    {
      mpSystem->ActivateLocalizationMode();
      bLocalizationMode = true;
    }
    // UI取消勾选Localization Mode，退出纯定位模式，恢复建图
    else if (!menuLocalizationMode && bLocalizationMode)
    {
      mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
    }

    // UI勾选Step By Step，开启跟踪器单步模式，等待手动触发才处理下一帧
    if (menuStepByStep && !bStepByStep)
    {
      // cout << "Viewer: step by step" << endl;
      mpTracker->SetStepByStep(true);
      bStepByStep = true;
    }
    // UI取消勾选Step By Step，关闭单步模式，自动连续运行
    else if (!menuStepByStep && bStepByStep)
    {
      mpTracker->SetStepByStep(false);
      bStepByStep = false;
    }

    // 点击Step按钮，单步执行一帧，通知跟踪器执行一次
    if (menuStep)
    {
      mpTracker->mbStep = true;
      menuStep = false; // 按钮复位
    }

    // 激活3D视口，传入渲染状态s_cam，后续OpenGL绘制输出到此视口
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 设置背景颜色为纯白色
    mpMapDrawer->DrawCurrentCamera(Twc);  // 绘制当前相机可视化模型（彩色相机锥）
    // 根据菜单开关，绘制关键帧、共视图、IMU图、LBA优化标记
    if (menuShowKeyFrames || menuShowGraph || menuShowInertialGraph ||
        menuShowOptLba)
      mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph,
                                 menuShowInertialGraph, menuShowOptLba);
    if (menuShowPoints) mpMapDrawer->DrawMapPoints(); // 绘制地图点云

    pangolin::FinishFrame(); // Pangolin完成一帧渲染，交换缓冲区，刷新窗口

    cv::Mat toShow; // 最终要imshow输出的图像矩阵
    cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale); // 绘制左帧图像，画特征点、框

    // 双目模式：拼接左右图像到toShow
    if (both)
    {
      cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
      cv::hconcat(im, imRight, toShow);
    }
    else
    {
      toShow = im; // 单目/RGBD，仅左图
    }

    // 如果配置缩放系数不等于1，缩放显示图像窗口大小
    if (mImageViewerScale != 1.f)
    {
      int width = toShow.cols * mImageViewerScale;
      int height = toShow.rows * mImageViewerScale;
      cv::resize(toShow, toShow, cv::Size(width, height));
    }

    cv::imshow("ORB-SLAM3: Current Frame", toShow); // OpenCV弹出窗口显示图像
    cv::waitKey(mT); // 等待按键，延时mT毫秒，维持图像窗口刷新

    // 点击Reset按钮，重置SLAM活跃地图，恢复各项UI状态
    if (menuReset)
    {
      menuShowGraph = true;
      menuShowInertialGraph = true;
      menuShowKeyFrames = true;
      menuShowPoints = true;
      menuLocalizationMode = false;
      if (bLocalizationMode) mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
      bFollow = true;
      menuFollowCamera = true;
      mpSystem->ResetActiveMap();
      menuReset = false; // 按钮复位
    }

    // 点击Stop按钮：关闭系统，保存轨迹文件CameraTrajectory.txt、KeyFrameTrajectory.txt
    if (menuStop)
    {
      if (bLocalizationMode) mpSystem->DeactivateLocalizationMode();

      // Stop all threads
      mpSystem->Shutdown();

      // Save camera trajectory
      mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
      mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
      menuStop = false; // 按钮复位
    }

    // 如果收到停止请求，进入停止状态，循环等待Release释放停止
    if (Stop())
    {
      while (isStopped())
      {
        usleep(3000); // 休眠3000微秒=3ms
      }
    }

    // 检查是否收到结束Viewer线程请求，收到则跳出while(1)主循环
    if (CheckFinish()) break;
  }

  SetFinish(); // 标记Viewer线程已经执行完毕
}

// 请求结束Viewer线程，外部调用，加互斥锁修改mbFinishRequested标志
void Viewer::RequestFinish()
{
  unique_lock<mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

// 检查是否收到结束请求，返回标志位，线程安全
bool Viewer::CheckFinish()
{
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

// 设置Viewer已经完成退出，线程安全
void Viewer::SetFinish()
{
  unique_lock<mutex> lock(mMutexFinish);
  mbFinished = true;
}

// 查询Viewer是否已经结束运行，线程安全
bool Viewer::isFinished()
{
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinished;
}

// 请求停止Viewer（暂停，不是退出），线程安全
void Viewer::RequestStop()
{
  unique_lock<mutex> lock(mMutexStop);
  if (!mbStopped) mbStopRequested = true;
}

// 查询Viewer当前是否处于暂停停止状态，线程安全
bool Viewer::isStopped()
{
  unique_lock<mutex> lock(mMutexStop);
  return mbStopped;
}

// 处理停止请求：如果没有请求结束，把mbStopped置true，返回true代表成功进入停止；收到结束则返回false
bool Viewer::Stop()
{
  unique_lock<mutex> lock(mMutexStop);
  unique_lock<mutex> lock2(mMutexFinish);

  if (mbFinishRequested)
  {
    return false;
  }
  else if (mbStopRequested)
  {
    mbStopped = true;
    mbStopRequested = false;
    return true;
  }

  return false;
}

// 释放停止状态，恢复Viewer继续运行，线程安全
void Viewer::Release()
{
  unique_lock<mutex> lock(mMutexStop);
  mbStopped = false;
}

/*void Viewer::SetTrackingPause() {
    mbStopTrack = true;
}*/

} // namespace ORB_SLAM3
