/**  * This file is part of ORB‑SLAM3  *
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
#include <memory>                     // C++智能指针std::shared_ptr
#include <mutex>                      // 互斥锁，多线程保护绘图数据
#include <opencv2/core/core.hpp>      // OpenCV基础核心模块，Mat、Point等
#include <opencv2/features2d/features2d.hpp> // OpenCV关键点KeyPoint
#include <unordered_set>              // 无序哈希集合（本文件未直接使用，头文件依赖）
#include <utility>                    // std::pair工具
#include <vector>                     // C++动态数组容器

#include "Atlas.h"                    // 地图集Atlas，管理所有子地图
#include "MapPoint.h"                 // 地图点MapPoint类定义
#include "Tracking.h"                 // 跟踪器Tracking类定义

namespace ORB_SLAM3 {

// 前向声明，减少头文件循环依赖
class Tracking;
class Viewer;

/**
 * @brief FrameDrawer帧绘制器类，负责可视化当前帧图像、关键点、匹配轨迹、系统状态文字
 * 被Viewer调用，从Tracking读取最新帧数据，生成带标注的可视化图像输出
 */
class FrameDrawer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，保证Eigen相关数据内存对齐

    /**
     * @brief FrameDrawer显式构造函数
     * @param pAtlas 全局地图集Atlas智能指针
     * @param draw_both 是否同时绘制左右双目图像，默认false
     */
    explicit FrameDrawer(const std::shared_ptr<Atlas> &pAtlas,
                        bool draw_both = false);

    // Update info from the last processed frame.
    /**
     * @brief 更新绘制器内部缓存，从Tracking读取最新处理完的帧所有数据
     * @param pTracker Tracking跟踪器的shared_ptr
     */
    void Update(const std::shared_ptr<Tracking> &pTracker);

    // Draw last processed frame.
    /**
     * @brief 绘制主（左目）可视化帧图像，绘制关键点、轨迹、状态文字
     * @param imageScale 图像缩放系数，默认1.0原始尺寸
     * @return cv::Mat 返回绘制完成的图像矩阵
     */
    cv::Mat DrawFrame(float imageScale = 1.f);
    /**
     * @brief 绘制右目可视化图像，双目模式生效
     * @param imageScale 图像缩放系数，默认1.0原始尺寸
     * @return cv::Mat 返回绘制完成的右目图像矩阵
     */
    cv::Mat DrawRightFrame(float imageScale = 1.f);

    bool both; ///< 标记是否开启双目双图绘制，draw_both入参赋值

protected:
    /**
     * @brief 在图像底部绘制系统状态文本信息（跟踪状态、丢失、初始化等提示）
     * @param im 原始输入图像
     * @param nState Tracking系统状态枚举值
     * @param imText 输出拼接了文字条的图像
     */
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;                  ///< 需要绘制的主(左目)原始图像
    cv::Mat mImRight;             ///< 右目原始图像，双目模式有效
    vector<cv::KeyPoint> mvCurrentKeys;    ///< 当前帧左目关键点集合
    vector<cv::KeyPoint> mvCurrentKeysRight; ///< 当前帧右目关键点集合
    vector<bool> mvbMap;          ///< bool数组，标记对应关键点是否关联地图点
    vector<bool> mvbVO;           ///< bool数组，标记该关键点是否属于纯VO无地图点观测
    bool mbOnlyTracking;          ///< true：系统处于仅跟踪模式，不做局部建图
    int mnTracked;                ///< 成功跟踪上的地图点关键点数量
    int mnTrackedVO;              ///< VO模式下跟踪成功的关键点数量
    vector<cv::KeyPoint> mvIniKeys;    ///< 初始化阶段的关键点
    vector<int> mvIniMatches;         ///< 初始化阶段关键点匹配索引
    int mState;                   ///< Tracking当前系统状态（OK / LOST / NOT_INITIALIZED等）
    std::vector<float> mvCurrentDepth; ///< 当前帧每个关键点对应的深度值
    float mThDepth;               ///< 远近点深度阈值，区分近点远点

    std::shared_ptr<Atlas> mpAtlas; ///< 全局地图集指针

    std::mutex mMutex; ///< 互斥锁，Update更新数据与Draw绘图之间多线程安全保护
    vector<pair<cv::Point2f, cv::Point2f> > mvTracks; ///< 跟踪轨迹线段集合，(上一帧像素点，当前帧像素点)

    std::shared_ptr<Frame> mCurrentFrame; ///< 缓存当前待绘制的Frame帧对象
    vector<MapPoint *> mvpLocalMap;      ///< 局部地图点集合
    vector<cv::KeyPoint> mvMatchedKeys;  ///< 成功匹配上地图点的关键点
    vector<MapPoint *> mvpMatchedMPs;    ///< 对应匹配成功的地图点裸指针
    vector<cv::KeyPoint> mvOutlierKeys;  ///< 判定为外点的关键点
    vector<MapPoint *> mvpOutlierMPs;    ///< 对应外点的地图点裸指针

    map<long unsigned int, cv::Point2f> mmProjectPoints; ///< map：地图点ID -> 当前帧投影像素坐标
    map<long unsigned int, cv::Point2f> mmMatchedInImage; ///< map：地图点ID -> 在图像中匹配到的像素坐标
};

}  // namespace ORB_SLAM3
