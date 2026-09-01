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
#pragma once // 头文件保护，防止头文件重复包含引发编译重定义
#include <pangolin/pangolin.h> // pangolin可视化库，提供OpenGL绘图、视窗、矩阵类型
#include <memory>              // std::shared_ptr智能指针
#include <mutex>               // std::mutex互斥锁，多线程访问相机位姿同步

#include "Atlas.h"       // 多地图集Atlas，管理全部子地图
#include "KeyFrame.h"    // 关键帧类定义
#include "MapPoint.h"    // 地图点类定义
#include "Settings.h"    // 配置参数读取管理类

namespace ORB_SLAM3 {

class Settings; // 配置类前向声明

/**
 * @brief 地图绘制器类，负责pangolin窗口中渲染地图点、关键帧、位姿图、当前相机
 * @details 只做可视化渲染，不修改地图数据；从Settings读取绘图样式参数；多线程环境下相机位姿加锁保护
 */
class MapDrawer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类含有Sophus::SE3f成员必须添加

    /**
     * @brief MapDrawer构造函数
     * @param pAtlas 全局多地图集Atlas共享指针
     * @param settings 系统配置参数共享指针
     */
    MapDrawer(const std::shared_ptr<Atlas> &pAtlas,
              const std::shared_ptr<Settings> &settings);

    std::shared_ptr<Atlas> mpAtlas; // 全局Atlas指针，用于获取所有地图、关键帧、地图点

    /**
     * @brief 绘制所有地图点
     */
    void DrawMapPoints();

    /**
     * @brief 绘制关键帧、位姿图、惯性图、局部BA优化图
     * @param bDrawKF 是否绘制关键帧相机模型
     * @param bDrawGraph 是否绘制共视位姿图连线
     * @param bDrawInertialGraph 是否绘制惯性相关图连线
     * @param bDrawOptLba 是否绘制局部BA优化涉及的关键帧
     */
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph,
                       const bool bDrawInertialGraph, const bool bDrawOptLba);

    /**
     * @brief 获取当前相机OpenGL矩阵Twc，供pangolin绘制当前相机模型
     * @param Twc 输出pangolin的OpenGlMatrix矩阵
     */
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    /**
     * @brief 设置当前相机位姿Tcw(相机到世界)，Tracking线程调用更新
     * @param Tcw Sophus SE3f相机到世界位姿
     */
    void SetCurrentCameraPose(const Sophus::SE3f &Tcw);

    /**
     * @brief 设置参考关键帧，可视化标记参考帧
     * @param pKF 参考关键帧裸指针
     */
    void SetReferenceKeyFrame(KeyFrame *pKF);

    /**
     * @brief 将内部存储的Tcw转换为pangolin OpenGL矩阵M与MOw，供渲染管线使用
     * @param M 输出Tcw对应的OpenGL矩阵
     * @param MOw 输出世界原点矩阵
     */
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M,
                                      pangolin::OpenGlMatrix &MOw);
private:
    /**
     * @brief 从Settings加载绘图相关参数，初始化绘图尺寸、线宽
     * @param ssettings 配置参数共享指针
     */
    void newParameterLoader(const std::shared_ptr<Settings> &ssettings);

    float mKeyFrameSize;        // 关键帧相机模型绘制尺寸
    float mKeyFrameLineWidth;   // 关键帧相机框线条宽度
    float mGraphLineWidth;      // 位姿图连线线条宽度
    float mPointSize;           // 地图点绘制点大小
    float mCameraSize;          // 当前跟踪相机模型尺寸
    float mCameraLineWidth;     // 当前跟踪相机框线条宽度

    Sophus::SE3f mCameraPose;   // 保存当前相机位姿 Tcw(相机到世界)

    std::mutex mMutexCamera;    // 保护mCameraPose互斥锁，Tracking写、可视化读分属不同线程

    /// 多套绘制颜色，用于区分不同地图/不同关键帧，RGB数组
    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f}, {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f}, {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f}};
};

} // namespace ORB_SLAM3
