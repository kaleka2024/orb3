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

// 帧绘制器头文件：负责SLAM运行时的帧可视化渲染，叠加特征点、匹配、状态等信息
#include "FrameDrawer.h"
// STL标准库：智能指针管理
#include <memory>
// STL标准库：互斥锁，保证多线程数据访问安全
#include <mutex>
// OpenCV核心模块：矩阵、图像数据结构与基础运算
#include <opencv2/core/core.hpp>
// OpenCV高层GUI模块：图像绘制、显示相关函数
#include <opencv2/highgui/highgui.hpp>
// STL标准库：成对数据结构
#include <utility>
// STL标准库：动态数组容器
#include <vector>

// ORB-SLAM3内部：跟踪模块头文件，提供跟踪状态枚举与数据接口
#include "Tracking.h"

namespace ORB_SLAM3 {

// ==============================================
// 帧绘制器构造函数
// pAtlas：图集管理器指针，用于获取地图统计信息
// draw_both：是否绘制双目左右两帧
// ==============================================
FrameDrawer::FrameDrawer(const std::shared_ptr<Atlas> &pAtlas, bool draw_both)
    : both(false), mpAtlas(pAtlas) {
  // 初始化跟踪状态为系统未就绪
  mState = Tracking::SYSTEM_NOT_READY;
  // 初始化左目图像缓冲区：480行×640列，8位3通道彩色图，初始纯黑色
  mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  // 初始化右目图像缓冲区，参数同左目
  mImRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

// ==============================================
// 绘制左目可视化帧
// imageScale：图像缩放比例，用于调整显示大小
// 返回：叠加了特征点、匹配线、状态文字的可视化图像
// ==============================================
cv::Mat FrameDrawer::DrawFrame(float imageScale) {
  cv::Mat im;                          // 输出图像
  vector<cv::KeyPoint>       vIniKeys;  // 初始化阶段：参考帧的特征点
  vector<int>       vMatches;           // 初始化阶段：参考帧与当前帧的匹配索引
  vector<cv::KeyPoint> vCurrentKeys;    // 当前帧的特征点集合
  vector<bool> vbVO, vbMap;             // 标记：vbVO=仅视觉里程计匹配，vbMap=与地图点匹配
  vector<pair<cv::Point2f, cv::Point2f> > vTracks;  // 跟踪轨迹点对
  int state;                            // 当前跟踪状态
  vector<float> vCurrentDepth;          // 当前帧特征点对应的深度值
  float thDepth;                        // 近景深度阈值

  std::shared_ptr<Frame> currentFrame;  // 当前帧智能指针
  vector<MapPoint *> vpLocalMap;        // 局部地图点集合
  vector<cv::KeyPoint> vMatchesKeys;    // 匹配成功的特征点
  vector<MapPoint *> vpMatchedMPs;      // 匹配成功的地图点
  vector<cv::KeyPoint> vOutlierKeys;    // 外点特征点
  vector<MapPoint *> vpOutlierMPs;      // 外点对应的地图点
  map<long unsigned int, cv::Point2f> mProjectPoints;  // 地图点投影坐标缓存
  map<long unsigned int, cv::Point2f> mMatchedInImage;  // 图像中匹配点缓存

  // 绘制颜色定义（OpenCV为BGR顺序）
  cv::Scalar standardColor(0, 255, 0);  // 绿色：与地图匹配的有效特征点
  cv::Scalar odometryColor(255, 0, 0);  // 蓝色：仅视觉里程计匹配的特征点

  // 作用域锁：拷贝共享数据时加锁，防止与跟踪线程数据竞争
  {
    unique_lock<mutex> lock(mMutex);
    // 拷贝当前跟踪状态
    state = mState;
    // 系统未就绪状态降级为暂无图像状态
    if (mState == Tracking::SYSTEM_NOT_READY) mState = Tracking::NO_IMAGES_YET;

    // 拷贝左目原始图像
    mIm.copyTo(im);

    // 根据不同跟踪状态，拷贝对应的数据
    if (mState == Tracking::NOT_INITIALIZED) {
      // 初始化状态：拷贝初始化参考帧特征点、匹配索引、跟踪轨迹
      vCurrentKeys = mvCurrentKeys;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
      vTracks = mvTracks;
    } else if (mState == Tracking::OK) {
      // 跟踪正常状态：拷贝特征点、匹配标记、当前帧、局部地图等完整数据
      vCurrentKeys = mvCurrentKeys;
      vbVO = mvbVO;
      vbMap = mvbMap;

      currentFrame = mCurrentFrame;
      vpLocalMap = mvpLocalMap;
      vMatchesKeys = mvMatchedKeys;
      vpMatchedMPs = mvpMatchedMPs;
      vOutlierKeys = mvOutlierKeys;
      vpOutlierMPs = mvpOutlierMPs;
      mProjectPoints = mmProjectPoints;
      mMatchedInImage = mmMatchedInImage;

      vCurrentDepth = mvCurrentDepth;
      thDepth = mThDepth;
    } else if (mState == Tracking::LOST) {
      // 跟踪丢失状态：仅拷贝当前特征点
      vCurrentKeys = mvCurrentKeys;
    }
  }

  // 图像缩放：如果缩放比例不为1，则调整图像尺寸
  if (imageScale != 1.f) {
    int imWidth = im.cols / imageScale;
    int imHeight = im.rows / imageScale;
    cv::resize(im, im, cv::Size(imWidth, imHeight));
  }

  // 灰度图转彩色图：确保图像为3通道，便于绘制彩色标注
  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);

  // 根据跟踪状态绘制不同内容
  if (state == Tracking::NOT_INITIALIZED) {
    // ========== 初始化阶段：绘制参考帧与当前帧的匹配连线 ==========
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      // 匹配有效才绘制
      if (vMatches[i] >= 0) {
        cv::Point2f pt1, pt2;
        // 根据缩放比例调整点坐标
        if (imageScale != 1.f) {
          pt1 = vIniKeys[i].pt / imageScale;
          pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
        } else {
          pt1 = vIniKeys[i].pt;
          pt2 = vCurrentKeys[vMatches[i]].pt;
        }
        // 绘制匹配连线（绿色）
        cv::line(im, pt1, pt2, standardColor);
      }
    }
    // 绘制跟踪轨迹线（粗线）
    for (vector<pair<cv::Point2f, cv::Point2f> >::iterator it = vTracks.begin();
         it != vTracks.end(); it++) {
      cv::Point2f pt1, pt2;
      if (imageScale != 1.f) {
        pt1 = (*it).first / imageScale;
        pt2 = (*it).second / imageScale;
      } else {
        pt1 = (*it).first;
        pt2 = (*it).second;
      }
      cv::line(im, pt1, pt2, standardColor, 5);
    }

  } else if (state == Tracking::OK) {
    // ========== 跟踪正常阶段：绘制特征点标注 ==========
    mnTracked = 0;      // 统计地图匹配特征点数量
    mnTrackedVO = 0;    // 统计视觉里程计匹配特征点数量
    const float r = 5;  // 特征点方框半宽
    int n = vCurrentKeys.size();

    // 遍历所有特征点
    for (int i = 0; i < n; i++) {
      // 只要是VO匹配或地图匹配，就绘制标注
      if (vbVO[i] || vbMap[i]) {
        cv::Point2f pt1, pt2;  // 方框对角点
        cv::Point2f point;      // 特征点中心

        // 根据缩放比例调整坐标
        if (imageScale != 1.f) {
          point = vCurrentKeys[i].pt / imageScale;
          float px = vCurrentKeys[i].pt.x / imageScale;
          float py = vCurrentKeys[i].pt.y / imageScale;
          pt1.x = px - r;
          pt1.y = py - r;
          pt2.x = px + r;
          pt2.y = py + r;
        } else {
          point = vCurrentKeys[i].pt;
          pt1.x = vCurrentKeys[i].pt.x - r;
          pt1.y = vCurrentKeys[i].pt.y - r;
          pt2.x = vCurrentKeys[i].pt.x + r;
          pt2.y = vCurrentKeys[i].pt.y + r;
        }

        // 与地图点匹配：绿色方框+实心圆点
        // This is a match to a MapPoint in the map
        if (vbMap[i]) {
          cv::rectangle(im, pt1, pt2, standardColor);
          cv::circle(im, point, 2, standardColor, -1);
          mnTracked++;
        } else {
          // 仅视觉里程计匹配：蓝色方框+实心圆点
          // This is match to a "visual odometry" MapPoint created in the
          // last frame
          cv::rectangle(im, pt1, pt2, odometryColor);
          cv::circle(im, point, 2, odometryColor, -1);
          mnTrackedVO++;
        }
      }
    }
  }

  cv::Mat imWithInfo;
  // 在图像底部绘制状态文字信息栏
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

// ==============================================
// 绘制右目可视化帧（双目模式）
// imageScale：图像缩放比例
// 返回：右目叠加标注的可视化图像
// ==============================================
cv::Mat FrameDrawer::DrawRightFrame(float imageScale) {
  cv::Mat im;
  vector<cv::KeyPoint>       vIniKeys;  // 初始化参考帧特征点
  vector<int>       vMatches;           // 初始化匹配索引
  vector<cv::KeyPoint> vCurrentKeys;    // 当前右目特征点
  vector<bool> vbVO, vbMap;             // 匹配标记
  int state;                            // 跟踪状态

  // 作用域锁：拷贝共享数据
  // Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY) mState = Tracking::NO_IMAGES_YET;

    // 拷贝右目原始图像
    mImRight.copyTo(im);

    // 根据状态拷贝对应数据
    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeysRight;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
    } else if (mState == Tracking::OK) {
      vCurrentKeys = mvCurrentKeysRight;
      vbVO = mvbVO;
      vbMap = mvbMap;
    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeysRight;
    }
  }  // destroy scoped mutex -> release mutex

  // 图像缩放
  if (imageScale != 1.f) {
    int imWidth = im.cols / imageScale;
    int imHeight = im.rows / imageScale;
    cv::resize(im, im, cv::Size(imWidth, imHeight));
  }

  // 灰度转彩色
  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);

  // 根据状态绘制
  if (state == Tracking::NOT_INITIALIZED) {
    // INITIALIZING：绘制初始化匹配连线
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0) {
        cv::Point2f pt1, pt2;
        if (imageScale != 1.f) {
          pt1 = vIniKeys[i].pt / imageScale;
          pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
        } else {
          pt1 = vIniKeys[i].pt;
          pt2 = vCurrentKeys[vMatches[i]].pt;
        }

        cv::line(im, pt1, pt2, cv::Scalar(0, 255, 0));
      }
    }
  } else if (state == Tracking::OK) {
    // TRACKING：绘制右目特征点标注
    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 5;
    const int n = mvCurrentKeysRight.size();
    const int Nleft = mvCurrentKeys.size();  // 左目特征点总数，用于索引偏移

    // 遍历右目所有特征点
    for (int i = 0; i < n; i++) {
      // 右目特征点的匹配标记在数组中的索引 = 左目总数 + 当前右目索引
      if (vbVO[i + Nleft] || vbMap[i + Nleft]) {
        cv::Point2f pt1, pt2;
        cv::Point2f point;

        if (imageScale != 1.f) {
          point = mvCurrentKeysRight[i].pt / imageScale;
          float px = mvCurrentKeysRight[i].pt.x / imageScale;
          float py = mvCurrentKeysRight[i].pt.y / imageScale;
          pt1.x = px - r;
          pt1.y = py - r;
          pt2.x = px + r;
          pt2.y = py + r;
        } else {
          point = mvCurrentKeysRight[i].pt;
          pt1.x = mvCurrentKeysRight[i].pt.x - r;
          pt1.y = mvCurrentKeysRight[i].pt.y - r;
          pt2.x = mvCurrentKeysRight[i].pt.x + r;
          pt2.y = mvCurrentKeysRight[i].pt.y + r;
        }

        // 地图匹配：绿色
        // This is a match to a MapPoint in the map
        if (vbMap[i + Nleft]) {
          cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
          cv::circle(im, point, 2, cv::Scalar(0, 255, 0), -1);
          mnTracked++;
        } else {
          // 视觉里程计匹配：蓝色
          // This is match to a "visual odometry" MapPoint created in the
          // last frame
          cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
          cv::circle(im, point, 2, cv::Scalar(255, 0, 0), -1);
          mnTrackedVO++;
        }
      }
    }
  }

  cv::Mat imWithInfo;
  // 绘制底部状态文字
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

// ==============================================
// 在图像底部绘制状态信息文字栏
// im：输入原始图像
// nState：当前跟踪状态
// imText：输出带文字栏的图像
// ==============================================
void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText) {
  stringstream s;  // 文字流，拼接状态信息

  // 根据不同状态拼接对应文字
  if (nState == Tracking::NO_IMAGES_YET) {
    s << " WAITING FOR IMAGES";
  } else if (nState == Tracking::NOT_INITIALIZED) {
    s << " TRYING TO INITIALIZE ";
  } else if (nState == Tracking::OK) {
    // 跟踪正常：区分SLAM模式和纯定位模式
    if (!mbOnlyTracking)
      s << "SLAM MODE |  ";
    else
      s << "LOCALIZATION | ";

    // 获取地图、关键帧、地图点数量统计
    int nMaps = mpAtlas->CountMaps();
    int nKFs = mpAtlas->KeyFramesInMap();
    int nMPs = mpAtlas->MapPointsInMap();
    // 拼接统计信息
    s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs
      << ", Matches: " << mnTracked;
    // 有VO匹配则追加显示
    if (mnTrackedVO > 0) s << ", + VO matches: " << mnTrackedVO;
  } else if (nState == Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  } else if (nState == Tracking::SYSTEM_NOT_READY) {
    s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
  }

  int baseline = 0;
  // 计算文字尺寸，用于确定文字栏高度
  cv::Size textSize =
      cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  // 创建带文字栏的新图像：原图高度 + 文字高度 + 10像素边距
  imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
  // 将原图拷贝到新图像的上半部分
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  // 文字栏区域填充黑色背景
  imText.rowRange(im.rows, imText.rows) =
      cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
  // 在文字栏左下角绘制白色文字
  cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

// ==============================================
// 更新绘制器数据：从跟踪模块同步最新的帧数据与状态
// pTracker：跟踪模块智能指针
// 说明：由跟踪线程调用，更新绘制所需的全部数据，加锁保证线程安全
// ==============================================
void FrameDrawer::Update(const std::shared_ptr<Tracking> &pTracker) {
  unique_lock<mutex> lock(mMutex);

  // 拷贝左目灰度图像
  pTracker->mImGray.copyTo(mIm);
  // 拷贝当前帧左目特征点
  mvCurrentKeys = pTracker->mCurrentFrame->mvKeys;
  // 拷贝近景深度阈值
  mThDepth = pTracker->mCurrentFrame->mThDepth;
  // 拷贝特征点深度数组
  mvCurrentDepth = pTracker->mCurrentFrame->mvDepth;
  size_t N = mvCurrentKeys.size();

  // 双目模式：同步右目数据
  if (both) {
    mvCurrentKeysRight = pTracker->mCurrentFrame->mvKeysRight;
    pTracker->mImRight.copyTo(mImRight);
    N += mvCurrentKeysRight.size();  // 总特征点数累加右目数量
  }

  // 初始化匹配标记数组，默认全为false
  mvbVO = vector<bool>(N, false);
  mvbMap = vector<bool>(N, false);
  // 拷贝纯定位模式标记
  mbOnlyTracking = pTracker->mbOnlyTracking;

  // 可视化相关变量更新
  mCurrentFrame = pTracker->mCurrentFrame;
  mmProjectPoints = mCurrentFrame->mmProjectPoints;
  mmMatchedInImage.clear();

  vpLocalMap = pTracker->GetLocalMapMPS();
  // 预分配内存避免频繁扩容
  mvMatchedKeys.clear();
  mvMatchedKeys.reserve(N);
  mvpMatchedMPs.clear();
  mvpMatchedMPs.reserve(N);
  mvOutlierKeys.clear();
  mvOutlierKeys.reserve(N);
  mvpOutlierMPs.clear();
  mvpOutlierMPs.reserve(N);

  // 根据跟踪状态处理不同数据
  if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
    // 初始化阶段：同步初始帧和匹配数据
    if (!pTracker->mInitialFrame) return;

    mvIniKeys = pTracker->mInitialFrame->mvKeys;
    mvIniMatches = pTracker->mvIniMatches;

  } else if (pTracker->mLastProcessedState == Tracking::OK) {
    // 跟踪正常：逐个特征点判断匹配类型
    for (size_t i = 0; i < N; i++) {
      MapPoint *pMP = pTracker->mCurrentFrame->mvpMapPoints.at(i);
      if (pMP) {
        // 不是外点
        if (!pTracker->mCurrentFrame->mvbOutlier[i]) {
          // 有观测次数：属于地图点，标记为地图匹配
          if (pMP->Observations() > 0)
            mvbMap[i] = true;
          // 无观测次数：仅临时VO点，标记为VO匹配
          else
            mvbVO[i] = true;

          // 记录匹配点坐标
          mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
        } else {
          // 外点：加入外点列表
          mvpOutlierMPs.push_back(pMP);
          mvOutlierKeys.push_back(mvCurrentKeys[i]);
        }
      }
    }
  }

  // 更新跟踪状态
  mState = static_cast<int>(pTracker->mLastProcessedState);
}

}  // namespace ORB_SLAM3
