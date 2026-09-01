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

// Frame类自身头文件声明
#include "Frame.h"
// Kannala-Brandt8 鱼眼相机模型头文件
#include <include/CameraModels/KannalaBrandt8.h>
// 针孔相机模型头文件
#include <include/CameraModels/Pinhole.h>
// spdlog 高性能日志库头文件
#include <spdlog/spdlog.h>

// STL标准库：通用算法函数
#include <algorithm>
// STL标准库：智能指针管理
#include <memory>
// STL标准库：多线程支持
#include <thread>
// STL标准库：成对数据结构
#include <utility>
// STL标准库：动态数组容器
#include <vector>

// ORB-SLAM3内部工具：类型格式转换器
#include "Converter.h"
// ORB-SLAM3内部工具：g2o图优化相关类型定义
#include "G2oTypes.h"
// ORB-SLAM3内部：几何相机基类
#include "GeometricCamera.h"
// ORB-SLAM3内部：关键帧类
#include "KeyFrame.h"
// ORB-SLAM3内部：地图点类
#include "MapPoint.h"
// ORB-SLAM3内部：ORB特征提取器
#include "ORBextractor.h"
// ORB-SLAM3内部：ORB特征匹配器
#include "ORBmatcher.h"

// ORB-SLAM3核心命名空间
namespace ORB_SLAM3 {

// ==============================================
// 静态成员变量初始化（所有Frame实例共享）
// ==============================================
// 全局帧ID自增计数器，每创建一帧自动分配唯一ID
long unsigned int Frame::nNextId = 0;
// 首次计算标志位：仅在第一帧或标定参数变更时执行图像边界、网格等初始化
bool Frame::mbInitialComputations = true;
// 相机内参：主点坐标cx/cy、焦距fx/fy，以及焦距倒数invfx/invfy（避免重复除法）
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
// 去畸变后图像的有效边界：最小/最大X、Y像素坐标
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
// 网格单元宽度、高度的倒数，用于快速计算特征点所属网格索引
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

// 鱼眼双目匹配使用的暴力匹配器，采用汉明距离度量ORB二进制描述子相似度
// For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

// ==============================================
// 默认构造函数
// ==============================================
Frame::Frame()
    // 初始化各类智能指针成员为空
    : mpcpi(),
      mpImuPreintegrated(),
      mpPrevFrame(),
      mpImuPreintegratedFrame(),
      mpReferenceKF(),
      // 帧数据有效标志：未设置
      mbIsSet(false),
      // IMU预积分完成标志：未完成
      mbImuPreintegrated(false),
      // 相机位姿有效标志：无位姿
      mbHasPose(false),
      // 相机速度有效标志：无速度
      mbHasVelocity(false) {
// 若开启计时统计宏，初始化双目匹配、ORB提取的耗时变量
#ifdef REGISTER_TIMES
  mTimeStereoMatch = 0;
  mTimeORB_Ext = 0;
#endif
}

// ==============================================
// 拷贝构造函数
// ==============================================
Frame::Frame(const Frame &frame)
    // 启用shared_from_this机制，允许通过this获取shared_ptr
    : std::enable_shared_from_this<Frame>(),
      // 拷贝相机标定参数指针
      mpcpi(frame.mpcpi),
      // 拷贝ORB词袋字典指针
      mpORBvocabulary(frame.mpORBvocabulary),
      // 拷贝左目ORB特征提取器指针
      mpORBextractorLeft(frame.mpORBextractorLeft),
      // 拷贝右目ORB特征提取器指针
      mpORBextractorRight(frame.mpORBextractorRight),
      // 拷贝时间戳
      mTimeStamp(frame.mTimeStamp),
      // 拷贝相机内参矩阵（cv::Mat深拷贝）
      mK(frame.mK.clone()),
      // 拷贝转换为Eigen格式的内参矩阵
      mK_(Converter::toMatrix3f(frame.mK)),
      // 拷贝镜头畸变系数（深拷贝）
      mDistCoef(frame.mDistCoef.clone()),
      // 拷贝基线焦距乘积 bf = b * f（物理基线 × 焦距）
      mbf(frame.mbf),
      // 拷贝像素单位基线 mb = bf / fx
      mb(frame.mb),
      // 拷贝近景深度阈值
      mThDepth(frame.mThDepth),
      // 拷贝特征点总数量
      N(frame.N),
      // 拷贝左目原始特征点向量
      mvKeys(frame.mvKeys),
      // 拷贝右目原始特征点向量
      mvKeysRight(frame.mvKeysRight),
      // 拷贝去畸变后的特征点向量
      mvKeysUn(frame.mvKeysUn),
      // 拷贝每个左目特征点对应的右目u坐标
      mvuRight(frame.mvuRight),
      // 拷贝每个特征点对应的深度值
      mvDepth(frame.mvDepth),
      // 拷贝词袋向量
      mBowVec(frame.mBowVec),
      // 拷贝特征向量
      mFeatVec(frame.mFeatVec),
      // 拷贝左目ORB描述子矩阵（深拷贝）
      mDescriptors(frame.mDescriptors.clone()),
      // 拷贝右目ORB描述子矩阵（深拷贝）
      mDescriptorsRight(frame.mDescriptorsRight.clone()),
      // 拷贝特征点对应的地图点指针数组
      mvpMapPoints(frame.mvpMapPoints),
      // 拷贝外点标记布尔数组
      mvbOutlier(frame.mvbOutlier),
      // 拷贝IMU标定参数
      mImuCalib(frame.mImuCalib),
      // 拷贝近邻地图点数量
      mnCloseMPs(frame.mnCloseMPs),
      // 拷贝IMU预积分对象指针
      mpImuPreintegrated(frame.mpImuPreintegrated),
      // 拷贝预积分对应的参考帧指针
      mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame),
      // 拷贝IMU零偏
      mImuBias(frame.mImuBias),
      // 拷贝当前帧ID
      mnId(frame.mnId),
      // 拷贝参考关键帧指针
      mpReferenceKF(frame.mpReferenceKF),
      // 拷贝图像金字塔层数
      mnScaleLevels(frame.mnScaleLevels),
      // 拷贝金字塔尺度因子
      mfScaleFactor(frame.mfScaleFactor),
      // 拷贝尺度因子的自然对数值
      mfLogScaleFactor(frame.mfLogScaleFactor),
      // 拷贝每层尺度因子数组
      mvScaleFactors(frame.mvScaleFactors),
      // 拷贝每层尺度因子倒数数组
      mvInvScaleFactors(frame.mvInvScaleFactors),
      // 拷贝图像文件名
      mNameFile(frame.mNameFile),
      // 拷贝数据集编号
      mnDataset(frame.mnDataset),
      // 拷贝每层尺度对应的方差平方数组
      mvLevelSigma2(frame.mvLevelSigma2),
      // 拷贝每层尺度方差平方的倒数数组
      mvInvLevelSigma2(frame.mvInvLevelSigma2),
      // 拷贝上一普通帧指针
      mpPrevFrame(frame.mpPrevFrame),
      // 拷贝上一个关键帧指针
      mpLastKeyFrame(frame.mpLastKeyFrame),
      // 拷贝帧有效标志位
      mbIsSet(frame.mbIsSet),
      // 拷贝IMU预积分完成标志
      mbImuPreintegrated(frame.mbImuPreintegrated),
      // 拷贝IMU数据互斥锁指针
      mpMutexImu(frame.mpMutexImu),
      // 拷贝左相机对象指针
      mpCamera(frame.mpCamera),
      // 拷贝右相机对象指针
      mpCamera2(frame.mpCamera2),
      // 拷贝左目特征点数量
      Nleft(frame.Nleft),
      // 拷贝右目特征点数量
      Nright(frame.Nright),
      // 拷贝左目重叠区域起始特征点索引
      monoLeft(frame.monoLeft),
      // 拷贝右目重叠区域起始特征点索引
      monoRight(frame.monoRight),
      // 拷贝左目到右目的匹配索引数组
      mvLeftToRightMatch(frame.mvLeftToRightMatch),
      // 拷贝右目到左目的匹配索引数组
      mvRightToLeftMatch(frame.mvRightToLeftMatch),
      // 拷贝立体匹配得到的3D点数组（Eigen对齐分配器）
      mvStereo3Dpoints(frame.mvStereo3Dpoints),
      // 拷贝左目到右目的相对位姿 Tlr
      mTlr(frame.mTlr),
      // 拷贝右目到左目的相对旋转 Rlr
      mRlr(frame.mRlr),
      // 拷贝右目到左目的相对平移 tlr
      mtlr(frame.mtlr),
      // 拷贝右目到左目的相对位姿 Trl
      mTrl(frame.mTrl),
      // 拷贝世界到相机的位姿 Tcw
      mTcw(frame.mTcw),
      // 位姿有效标志初始化为假，后续通过SetPose正式设置
      mbHasPose(false),
      // 速度有效标志初始化为假，后续通过SetVelocity正式设置
      mbHasVelocity(false) {
  // 逐元素拷贝左目特征点二维网格
  for (int i = 0; i < FRAME_GRID_COLS; i++)
    for (int j = 0; j < FRAME_GRID_ROWS; j++) {
      mGrid[i][j] = frame.mGrid[i][j];
      // 若存在右目特征，同步拷贝右目网格
      if (frame.Nleft > 0) {
        mGridRight[i][j] = frame.mGridRight[i][j];
      }
    }

  // 源帧有有效位姿则拷贝并更新派生位姿矩阵
  if (frame.mbHasPose) SetPose(frame.GetPose());
  // 源帧有有效速度则拷贝速度
  if (frame.HasVelocity()) {
    SetVelocity(frame.GetVelocity());
  }

  // 拷贝投影点缓存与匹配点缓存map容器
  mmProjectPoints = frame.mmProjectPoints;
  mmMatchedInImage = frame.mmMatchedInImage;

// 若开启计时统计，拷贝耗时统计变量
#ifdef REGISTER_TIMES
  mTimeStereoMatch = frame.mTimeStereoMatch;
  mTimeORB_Ext = frame.mTimeORB_Ext;
#endif
}

// ==============================================
// 双目针孔相机构造函数
// 输入：左右目图像、时间戳、左右ORB提取器、词袋、内参、畸变系数、bf基线、深度阈值、相机对象、上一帧、IMU标定
// ==============================================
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,
             const double &timeStamp,
             const std::shared_ptr<ORBextractor> &extractorLeft,
             const std::shared_ptr<ORBextractor> &extractorRight,
             const std::shared_ptr<ORBVocabulary> &voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth,
             const std::shared_ptr<GeometricCamera> &pCamera,
             const std::shared_ptr<Frame> &pPrevF, const IMU::Calib &ImuCalib)
    // 初始化列表：各类指针与基础参数
    : mpcpi(nullptr),
      mpORBvocabulary(voc),
      mpORBextractorLeft(extractorLeft),
      mpORBextractorRight(extractorRight),
      mTimeStamp(timeStamp),
      mK(K.clone()),
      mK_(Converter::toMatrix3f(K)),
      mDistCoef(distCoef.clone()),
      mbf(bf),
      mThDepth(thDepth),
      mImuCalib(ImuCalib),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(pPrevF),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(),
      mbIsSet(false),
      mbImuPreintegrated(false),
      mpCamera(pCamera),
      mpCamera2(nullptr),
      mbHasPose(false),
      mbHasVelocity(false) {
  // 分配全局唯一帧ID，计数器自增
  // Frame ID
  mnId = nNextId++;

  // 初始化图像金字塔尺度相关参数，全部从ORB提取器获取
  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // 双线程并行提取左右目ORB特征
  // ORB extraction
#ifdef REGISTER_TIMES
  // 记录ORB提取起始时间点
  std::chrono::steady_clock::time_point time_StartExtORB =
      std::chrono::steady_clock::now();
#endif
  // 启动左目特征提取线程
  thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, 0, 0);
  // 启动右目特征提取线程
  thread threadRight(&Frame::ExtractORB, this, 1, imRight, 0, 0);
  // 等待左目线程执行完毕
  threadLeft.join();
  // 等待右目线程执行完毕
  threadRight.join();
#ifdef REGISTER_TIMES
  // 记录ORB提取结束时间点
  std::chrono::steady_clock::time_point time_EndExtORB =
      std::chrono::steady_clock::now();
  // 计算提取耗时并保存（单位：毫秒）
  mTimeORB_Ext =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndExtORB - time_StartExtORB)
          .count();
#endif

  // 统计左目特征点总数量
  N = mvKeys.size();
  // 无特征点则直接返回，不执行后续初始化
  if (mvKeys.empty()) return;

  // 初始化地图点指针数组：每个特征点对应一个地图点，初始为空
  mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));
  // 初始化外点标记数组：默认均为内点
  mvbOutlier = vector<bool>(N, false);
  // 清空投影点缓存与匹配点缓存
  mmProjectPoints.clear();
  mmMatchedInImage.clear();

  // 仅在第一帧或标定参数变化时执行一次性初始化计算
  // This is done only for the first Frame (or after a change in the
  // calibration)
  if (mbInitialComputations) {
    // 计算去畸变后的图像有效边界
    ComputeImageBounds(imLeft);

    // std::cout << "mbInitialComputations " << mnMaxX << "-" << mnMinX << "   "
    // << mnMaxY << "-" << mnMinY << endl;

    // 计算网格单元宽度的倒数 = 网格列数 / 图像有效宽度
    mfGridElementWidthInv =
        static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
    // 计算网格单元高度的倒数 = 网格行数 / 图像有效高度
    mfGridElementHeightInv =
        static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

    // 从内参矩阵提取焦距与主点坐标
    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    // 预计算焦距倒数，避免后续重复除法运算
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    // 标记首次计算完成，后续帧不再重复执行
    mbInitialComputations = false;
  }

  // 计算像素单位基线 mb = bf / fx
  mb = mbf / fx;
  // oslog::trace("Frame {}, mb {} = mbf {} / fx {}", mnId, mb, mbf, fx);

  // 继承上一帧的速度（恒速度运动模型）
  if (pPrevF) {
    if (pPrevF->HasVelocity()) SetVelocity(pPrevF->GetVelocity());
  } else {
    // 无上一帧则速度初始化为0
    mVw.setZero();
  }

  // 创建IMU数据互斥锁，保证多线程访问IMU数据的线程安全
  mpMutexImu = std::make_shared<std::mutex>();

  // 初始化鱼眼双目相关变量为无效值（针孔双目模式不使用）
  // Set no stereo fisheye information
  Nleft = -1;
  Nright = -1;
  mvLeftToRightMatch = vector<int>(0);
  mvRightToLeftMatch = vector<int>(0);
  mvStereo3Dpoints =
      vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>(0);
  monoLeft = -1;
  monoRight = -1;

  // 对所有特征点执行去畸变处理
  UndistortKeyPoints();

// 执行双目立体匹配计算深度
#ifdef REGISTER_TIMES
  // 记录立体匹配起始时间
  std::chrono::steady_clock::time_point time_StartStereoMatches =
      std::chrono::steady_clock::now();
#endif
  ComputeStereoMatches();
#ifdef REGISTER_TIMES
  // 记录立体匹配结束时间
  std::chrono::steady_clock::time_point time_EndStereoMatches =
      std::chrono::steady_clock::now();
  // 计算匹配耗时并保存
  mTimeStereoMatch =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndStereoMatches - time_StartStereoMatches)
          .count();
#endif

  // 将所有特征点分配到二维网格中，加速后续局部特征匹配
  AssignFeaturesToGrid();
}

// ==============================================
// RGBD相机构造函数
// 输入：灰度图、深度图、时间戳、ORB提取器、词袋、内参、畸变、bf、深度阈值、相机、上一帧、IMU标定
// ==============================================
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth,
             const double &timeStamp,
             const std::shared_ptr<ORBextractor> &extractor,
             const std::shared_ptr<ORBVocabulary> &voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth,
             const std::shared_ptr<GeometricCamera> &pCamera,
             const std::shared_ptr<Frame> &pPrevF, const IMU::Calib &ImuCalib)
    // 初始化列表：右目提取器为空
    : mpcpi(nullptr),
      mpORBvocabulary(voc),
      mpORBextractorLeft(extractor),
      mpORBextractorRight(nullptr),
      mTimeStamp(timeStamp),
      mK(K.clone()),
      mK_(Converter::toMatrix3f(K)),
      mDistCoef(distCoef.clone()),
      mbf(bf),
      mThDepth(thDepth),
      mImuCalib(ImuCalib),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(pPrevF),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(),
      mbIsSet(false),
      mbImuPreintegrated(false),
      mpCamera(pCamera),
      mpCamera2(nullptr),
      mbHasPose(false),
      mbHasVelocity(false) {
  // 分配全局唯一帧ID
  // Frame ID
  mnId = nNextId++;

  // 初始化图像金字塔尺度参数
  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // 单线程提取灰度图ORB特征（RGBD仅需左目灰度）
  // ORB extraction
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartExtORB =
      std::chrono::steady_clock::now();
#endif
  ExtractORB(0, imGray, 0, 0);
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndExtORB =
      std::chrono::steady_clock::now();
  mTimeORB_Ext =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndExtORB - time_StartExtORB)
          .count();
#endif

  // 统计特征点总数
  N = mvKeys.size();

  // 无特征点直接返回
  if (mvKeys.empty()) return;

  // 初始化地图点指针数组
  mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));

  // 清空投影与匹配缓存
  mmProjectPoints.clear();
  mmMatchedInImage.clear();

  // 初始化外点标记数组
  mvbOutlier = vector<bool>(N, false);

  // 首次计算：图像边界、网格、内参
  // This is done only for the first Frame (or after a change in the
  // calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imGray);

    // std::cout << "mbInitialComputations " << mnMaxX << "-" << mnMinX << "   "
    // << mnMaxY << "-" << mnMinY << endl;

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) /
                            static_cast<float>(mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) /
                             static_cast<float>(mnMaxY - mnMinY);

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }

  // 计算像素基线
  mb = mbf / fx;
  oslog::trace("Frame {}, mb {} = mbf {} / fx {}", mnId, mb, mbf, fx);

  // 继承上一帧速度
  if (pPrevF) {
    if (pPrevF->HasVelocity()) SetVelocity(pPrevF->GetVelocity());
  } else {
    mVw.setZero();
  }

  // 创建IMU互斥锁
  mpMutexImu = std::make_shared<std::mutex>();

  // 初始化鱼眼双目相关变量为无效
  // Set no stereo fisheye information
  Nleft = -1;
  Nright = -1;
  mvLeftToRightMatch = vector<int>(0);
  mvRightToLeftMatch = vector<int>(0);
  mvStereo3Dpoints =
      vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>(0);
  monoLeft = -1;
  monoRight = -1;

  // 特征点去畸变
  UndistortKeyPoints();

  // 从深度图读取每个特征点的深度，并计算对应右目u坐标
  ComputeStereoFromRGBD(imDepth);

  // 特征点分配到网格
  AssignFeaturesToGrid();
}

// ==============================================
// 单目相机构造函数
// 输入：灰度图、时间戳、ORB提取器、词袋、相机对象、畸变系数、bf、深度阈值、上一帧、IMU标定
// ==============================================
Frame::Frame(const cv::Mat &imGray, const double &timeStamp,
             const std::shared_ptr<ORBextractor> &extractor,
             const std::shared_ptr<ORBVocabulary> &voc,
             const std::shared_ptr<GeometricCamera> &pCamera, cv::Mat &distCoef,
             const float &bf, const float &thDepth,
             const std::shared_ptr<Frame> &pPrevF, const IMU::Calib &ImuCalib)
    // 初始化列表：右目相关全部为空
    : mpcpi(nullptr),
      mpORBvocabulary(voc),
      mpORBextractorLeft(extractor),
      mpORBextractorRight(nullptr),
      mTimeStamp(timeStamp),
      // 从针孔相机对象直接获取内参矩阵
      mK(dynamic_cast<Pinhole *>(pCamera.get())->toK()),
      mK_(dynamic_cast<Pinhole *>(pCamera.get())->toK_()),
      mDistCoef(distCoef.clone()),
      mbf(bf),
      mThDepth(thDepth),
      mImuCalib(ImuCalib),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(pPrevF),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(nullptr),
      mbIsSet(false),
      mbImuPreintegrated(false),
      mpCamera(pCamera),
      mpCamera2(nullptr),
      mbHasPose(false),
      mbHasVelocity(false) {
  // 分配帧ID
  // Frame ID
  mnId = nNextId++;

  // 初始化金字塔尺度参数
  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // 提取ORB特征，指定x坐标提取范围（0~1000为全图提取）
  // ORB extraction
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartExtORB =
      std::chrono::steady_clock::now();
#endif
  ExtractORB(0, imGray, 0, 1000);
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndExtORB =
      std::chrono::steady_clock::now();
  mTimeORB_Ext =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndExtORB - time_StartExtORB)
          .count();
#endif

  // 特征点总数
  N = mvKeys.size();
  if (mvKeys.empty()) return;

  // 特征点去畸变
  UndistortKeyPoints();

  // 初始化右目坐标与深度数组为-1（单目无立体信息）
  // Set no stereo information
  mvuRight = vector<float>(N, -1);
  mvDepth = vector<float>(N, -1);
  mnCloseMPs = 0;

  // 初始化地图点指针数组
  mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));

  // 清空投影与匹配缓存
  mmProjectPoints.clear();  // = map<long unsigned int, cv::Point2f>(N,
                             // static_cast<cv::Point2f>(nullptr));
  mmMatchedInImage.clear();

  // 初始化外点标记
  mvbOutlier = vector<bool>(N, false);

  // 首次计算图像边界、网格、内参
  // This is done only for the first Frame (or after a change in the
  // calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imGray);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) /
                            static_cast<float>(mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) /
                             static_cast<float>(mnMaxY - mnMinY);

    // 从针孔相机对象获取内参矩阵
    const auto camK = dynamic_cast<Pinhole *>(mpCamera.get())->toK();

    fx = camK.at<float>(0, 0);
    fy = camK.at<float>(1, 1);
    cx = camK.at<float>(0, 2);
    cy = camK.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }

  // 计算像素基线
  mb = mbf / fx;

  // 初始化鱼眼双目相关变量为无效
  // Set no stereo fisheye information
  Nleft = -1;
  Nright = -1;
  mvLeftToRightMatch = vector<int>(0);
  mvRightToLeftMatch = vector<int>(0);
  mvStereo3Dpoints =
      vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>(0);
  monoLeft = -1;
  monoRight = -1;

  // 特征点分配到网格
  AssignFeaturesToGrid();

  // 继承上一帧速度
  if (pPrevF) {
    if (pPrevF->HasVelocity()) {
      SetVelocity(pPrevF->GetVelocity());
    }
  } else {
    mVw.setZero();
  }

  // 创建IMU互斥锁
  mpMutexImu = std::make_shared<std::mutex>();
}

// ==============================================
// 析构函数
// ==============================================
Frame::~Frame() {}

// ==============================================
// 将特征点分配到二维网格
// 作用：将特征点按坐标分桶，后续局部匹配只需搜索相邻网格，大幅提升匹配速度
// ==============================================
void Frame::AssignFeaturesToGrid() {
  // 网格总单元数 = 列数 × 行数
  // Fill matrix with points
  const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

  // 每个网格预分配的特征点数量（总特征数/网格数的一半，预留空间避免频繁扩容）
  int nReserve = 0.5f * N / (nCells);

  // 遍历所有网格，提前预分配内存
  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
      mGrid[i][j].reserve(nReserve);
      // 若存在右目特征，同步为右目网格预分配
      if (Nleft != -1) {
        mGridRight[i][j].reserve(nReserve);
      }
    }

  // 遍历所有特征点，逐个分配到对应网格
  for (size_t i = 0; i < N; i++) {
    // 根据模式选择特征点：单目取去畸变点，双目取左目/右目原始点
    const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                             : (i < Nleft) ? mvKeys[i]
                                           : mvKeysRight[i - Nleft];

    // 计算特征点所属的网格坐标
    int nGridPosX, nGridPosY;
    if (PosInGrid(kp, nGridPosX, nGridPosY)) {
      // 左目特征或单目特征存入左网格
      if (Nleft == -1 || i < Nleft)
        mGrid[nGridPosX][nGridPosY].push_back(i);
      // 右目特征存入右网格
      else
        mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
    }
  }
}

// ==============================================
// 提取ORB特征
// flag: 0=左目，1=右目
// im: 输入图像
// x0, x1: 提取的x坐标范围（鱼眼相机用于仅提取重叠区域）
// ==============================================
void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0,
                       const int x1) {
  // 构造提取范围向量
  vector<int> vLapping = {x0, x1};
  if (flag == 0)
    // 调用左目提取器，输出特征点和描述子，返回重叠区域起始索引
    monoLeft =
        (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, vLapping);
  else
    // 调用右目提取器，输出右目特征点和描述子
    monoRight = (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight,
                                       mDescriptorsRight, vLapping);
}

// ==============================================
// 检查帧是否已设置有效位姿
// ==============================================
bool Frame::isSet() const { return mbIsSet; }

// ==============================================
// 设置相机位姿（世界→相机 SE3变换）
// ==============================================
void Frame::SetPose(const Sophus::SE3<float> &Tcw) {
  // 保存世界到相机的SE3位姿
  mTcw = Tcw;

  // 更新旋转矩阵、平移向量等派生位姿量
  UpdatePoseMatrices();
  // 标记位姿已设置且有效
  mbIsSet = true;
  mbHasPose = true;
}

// ==============================================
// 设置IMU零偏
// 同时更新当前帧保存的零偏和IMU预积分对象的零偏
// ==============================================
void Frame::SetNewBias(const IMU::Bias &b) {
  mImuBias = b;
  if (mpImuPreintegrated) mpImuPreintegrated->SetNewBias(b);
}

// ==============================================
// 设置世界坐标系下的相机速度
// ==============================================
void Frame::SetVelocity(const Eigen::Vector3f &Vwb) {
  mVw = Vwb;
  mbHasVelocity = true;
}

// ==============================================
// 获取世界坐标系下的相机速度
// ==============================================
Eigen::Vector3f Frame::GetVelocity() const { return mVw; }

// ==============================================
// 通过IMU位姿速度计算相机位姿
// 输入：IMU在世界系下的旋转Rwb、平移twb、速度Vwb
// 原理：利用IMU-相机外参Tcb，将IMU位姿转换为相机位姿
// ==============================================
void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb,
                               const Eigen::Vector3f &twb,
                               const Eigen::Vector3f &Vwb) {
  // 保存世界坐标系下的相机速度
  mVw = Vwb;
  mbHasVelocity = true;

  // 构造IMU到世界的SE3变换 Twb
  Sophus::SE3f Twb(Rwb, twb);
  // 求逆得到世界到IMU的变换 Tbw
  Sophus::SE3f Tbw = Twb.inverse();

  // 相机到世界的变换 Tcw = Tcb * Tbw （Tcb为IMU到相机的外参）
  mTcw = mImuCalib.mTcb * Tbw;

  // 更新位姿派生矩阵
  UpdatePoseMatrices();
  // 标记位姿有效
  mbIsSet = true;
  mbHasPose = true;
}

// ==============================================
// 更新位姿派生矩阵
// 从Tcw分解出旋转、平移，同时计算逆变换Twc的对应量
// ==============================================
void Frame::UpdatePoseMatrices() {
  // 求逆得到相机到世界的变换 Twc
  Sophus::SE3<float> Twc = mTcw.inverse();
  // 相机到世界的旋转矩阵
  mRwc = Twc.rotationMatrix();
  // 相机光心在世界坐标系下的位置
  mOw = Twc.translation();
  // 世界到相机的旋转矩阵
  mRcw = mTcw.rotationMatrix();
  // 世界到相机的平移向量
  mtcw = mTcw.translation();
}

// ==============================================
// 获取IMU在世界坐标系下的位置
// ==============================================
Eigen::Matrix<float, 3, 1> Frame::GetImuPosition() const {
  // 相机位置 + 旋转后的IMU外参平移
  return mRwc * mImuCalib.mTcb.translation() + mOw;
}

// ==============================================
// 获取IMU在世界坐标系下的旋转
// ==============================================
Eigen::Matrix<float, 3, 3> Frame::GetImuRotation() {
  // 相机旋转 × IMU外参旋转
  return mRwc * mImuCalib.mTcb.rotationMatrix();
}

// ==============================================
// 获取IMU在世界坐标系下的SE3位姿
// ==============================================
Sophus::SE3<float> Frame::GetImuPose() {
  // 相机位姿 × IMU外参
  return mTcw.inverse() * mImuCalib.mTcb;
}

// ==============================================
// 获取右目到左目的相对位姿 Trl
// ==============================================
Sophus::SE3f Frame::GetRelativePoseTrl() { return mTrl; }

// ==============================================
// 获取左目到右目的相对位姿 Tlr
// ==============================================
Sophus::SE3f Frame::GetRelativePoseTlr() { return mTlr; }

// ==============================================
// 获取左目到右目的相对旋转矩阵
// ==============================================
Eigen::Matrix3f Frame::GetRelativePoseTlr_rotation() {
  return mTlr.rotationMatrix();
}

// ==============================================
// 获取左目到右目的相对平移向量
// ==============================================
Eigen::Vector3f Frame::GetRelativePoseTlr_translation() {
  return mTlr.translation();
}

// ==============================================
// 判断地图点是否在相机视锥内
// pMP: 待检查地图点
// viewingCosLimit: 视角余弦阈值，过滤法线夹角过大的点
// ==============================================
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
  // 单目模式：仅检查左相机
  if (Nleft == -1) {
    // 初始化跟踪标记为不可见
    pMP->mbTrackInView = false;
    pMP->mTrackProjX = -1;
    pMP->mTrackProjY = -1;

    // 获取地图点的世界坐标
    // 3D in absolute coordinates
    Eigen::Matrix<float, 3, 1> P = pMP->GetWorldPos();

    // 转换到相机坐标系
    // 3D in camera coordinates
    const Eigen::Matrix<float, 3, 1> Pc = mRcw * P + mtcw;
    const float Pc_dist = Pc.norm();

    // 检查深度是否为正（点在相机前方）
    // Check positive depth
    const float &PcZ = Pc(2);
    const float invz = 1.0f / PcZ;
    if (PcZ < 0.0f) return false;

    // 投影到图像平面
    const Eigen::Vector2f uv = mpCamera->project(Pc);

    // 检查投影点是否在有效图像边界内
    if (uv(0) < mnMinX || uv(0) > mnMaxX) return false;
    if (uv(1) < mnMinY || uv(1) > mnMaxY) return false;

    // 保存投影坐标到地图点，供后续跟踪使用
    pMP->mTrackProjX = uv(0);
    pMP->mTrackProjY = uv(1);

    // 检查距离是否在地图点的尺度不变范围内
    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    // 相机光心指向地图点的向量
    const Eigen::Vector3f PO = P - mOw;
    const float dist = PO.norm();

    if (dist < minDistance || dist > maxDistance) return false;

    // 检查视角：地图点法线与视线的夹角
    // Check viewing angle
    Eigen::Vector3f Pn = pMP->GetNormal();

    // 计算视线与法线的夹角余弦值
    const float viewCos = PO.dot(Pn) / dist;

    if (viewCos < viewingCosLimit) return false;

    // 根据距离预测地图点在图像中的尺度层级
    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist, shared_from_this());

    // 更新地图点的跟踪相关数据
    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = uv(0);
    // 右目投影x坐标（通过基线偏移估算）
    pMP->mTrackProjXR = uv(0) - mbf * invz;

    pMP->mTrackDepth = Pc_dist;

    pMP->mTrackProjY = uv(1);
    pMP->mnTrackScaleLevel = nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
  } else {
    // 双目模式：检查左右两个相机，任意一个在视锥内即返回true
    pMP->mbTrackInView = false;
    pMP->mbTrackInViewR = false;
    pMP->mnTrackScaleLevel = -1;
    pMP->mnTrackScaleLevelR = -1;

    // 检查左相机
    pMP->mbTrackInView = isInFrustumChecks(pMP, viewingCosLimit);
    // 检查右相机
    pMP->mbTrackInViewR = isInFrustumChecks(pMP, viewingCosLimit, true);

    return pMP->mbTrackInView || pMP->mbTrackInViewR;
  }
}

// ==============================================
// 投影地图点并计算畸变后的像素坐标
// pMP: 输入地图点
// kp: 输出畸变后的图像点
// u, v: 输出畸变后的u、v像素坐标
// ==============================================
bool Frame::ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u,
                                float &v) {
  // 获取地图点世界坐标
  // 3D in absolute coordinates
  Eigen::Vector3f P = pMP->GetWorldPos();

  // 转换到相机坐标系
  // 3D in camera coordinates
  const Eigen::Vector3f Pc = mRcw * P + mtcw;
  const float &PcX = Pc(0);
  const float &PcY = Pc(1);
  const float &PcZ = Pc(2);

  // 检查正深度（点在相机前方）
  // Check positive depth
  if (PcZ < 0.0f) {
    oslog::warn("Negative depth: {PcZ}");
    return false;
  }

  // 针孔相机投影：归一化平面转像素坐标
  // Project in image and check it is not outside
  const float invz = 1.0f / PcZ;
  u = fx * PcX * invz + cx;
  v = fy * PcY * invz + cy;

  // 检查是否在图像边界内
  if (u < mnMinX || u > mnMaxX) return false;
  if (v < mnMinY || v > mnMaxY) return false;

  // 计算畸变：先转回归一化坐标
  float u_distort, v_distort;

  float x = (u - cx) * invfx;
  float y = (v - cy) * invfy;
  float r2 = x * x + y * y;
  // 读取畸变系数：k1,k2径向，p1,p2切向，k3径向（可选第五个系数）
  float k1 = mDistCoef.at<float>(0);
  float k2 = mDistCoef.at<float>(1);
  float p1 = mDistCoef.at<float>(2);
  float p2 = mDistCoef.at<float>(3);
  float k3 = 0;
  if (mDistCoef.total() == 5) {
    k3 = mDistCoef.at<float>(4);
  }

  // 径向畸变公式：x_distort = x * (1 + k1*r² + k2*r⁴ + k3*r⁶)
  // Radial distorsion
  float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
  float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

  // 切向畸变公式
  // Tangential distorsion
  x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
  y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

  // 畸变后的归一化坐标转回像素坐标
  u_distort = x_distort * fx + cx;
  v_distort = y_distort * fy + cy;

  // 赋值输出
  u = u_distort;
  v = v_distort;

  kp = cv::Point2f(u, v);

  return true;
}

// ==============================================
// 世界坐标点转换为相机坐标点
// ==============================================
Eigen::Vector3f Frame::inRefCoordinates(Eigen::Vector3f pCw) {
  return mRcw * pCw + mtcw;
}

// ==============================================
// 获取指定圆形区域内的特征点索引
// x,y: 区域中心坐标
// r: 区域半径
// minLevel, maxLevel: 特征点尺度层级范围
// bRight: 是否查询右目网格
// ==============================================
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y,
                                        const float &r, const int minLevel,
                                        const int maxLevel,
                                        const bool bRight) const {
  vector<size_t> vIndices;
  vIndices.reserve(N);

  float factorX = r;
  float factorY = r;

  // std::cout << x << "," << y << " " << r << " " << mnMinX << " " << mnMinY <<
  // endl; std::cout << mfGridElementWidthInv << " " << mfGridElementHeightInv
  // << endl;

  // 计算区域左边界对应的网格列号，向下取整，不小于0
  const int nMinCellX = max(0, static_cast<int>(floor((x - mnMinX - factorX) *
                                                     mfGridElementWidthInv)));
  if (nMinCellX >= FRAME_GRID_COLS) {
    return vIndices;
  }

  // 计算区域右边界对应的网格列号，向上取整，不超过最大列数
  const int nMaxCellX = min(
      static_cast<int>(FRAME_GRID_COLS - 1),
      static_cast<int>(ceil((x - mnMinX + factorX) * mfGridElementWidthInv)));
  if (nMaxCellX < 0) {
    return vIndices;
  }

  // 计算区域上下边界对应的网格行号
  const int nMinCellY = max(0, static_cast<int>(floor((y - mnMinY - factorY) *
                                                     mfGridElementHeightInv)));
  if (nMinCellY >= FRAME_GRID_ROWS) {
    return vIndices;
  }

  const int nMaxCellY = min(
      static_cast<int>(FRAME_GRID_ROWS - 1),
      static_cast<int>(ceil((y - mnMinY + factorY) * mfGridElementHeightInv)));
  if (nMaxCellY < 0) {
    return vIndices;
  }

  // 是否需要检查尺度层级范围
  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  // std::cout << nMinCellX << ", " << nMaxCellX << " -- " <<  nMinCellY << ", "
  // << nMaxCellY << endl;

  // 遍历范围内的所有网格
  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      // 获取对应网格的特征点索引列表
      const vector<size_t> vCell =
          (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
      if (vCell.empty()) continue;

      // 遍历网格内每个特征点
      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        // 获取对应的去畸变特征点
        const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]]
                                    : (!bRight)   ? mvKeys[vCell[j]]
                                                  : mvKeysRight[vCell[j]];
        // 检查尺度层级是否在指定范围内
        if (bCheckLevels) {
          if (kpUn.octave < minLevel) continue;
          if (maxLevel >= 0)
            if (kpUn.octave > maxLevel) continue;
        }

        // 计算特征点到区域中心的距离
        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        // 距离在半径范围内则加入结果列表
        if (fabs(distx) < factorX && fabs(disty) < factorY)
          vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

// ==============================================
// 计算特征点所属的网格坐标
// kp: 输入特征点
// posX, posY: 输出网格列、行索引
// ==============================================
bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
  // 网格x索引 = (特征点x - 图像最小x) × 网格宽度倒数
  posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
  // 网格y索引 = (特征点y - 图像最小y) × 网格高度倒数
  posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

  // 去畸变后的特征点可能超出图像边界，需要合法性检查
  // Keypoint's coordinates are undistorted, which could cause to go out of the
  // image
  if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 ||
      posY >= FRAME_GRID_ROWS)
    return false;

  return true;
}

// ==============================================
// 计算词袋向量BoW
// 将当前帧ORB描述子转换为词袋向量与特征向量，用于回环检测和重定位
// ==============================================
void Frame::ComputeBoW() {
  // 词袋向量为空时才计算，避免重复计算
  if (mBowVec.empty()) {
    oslog::info("[Frame::ComputeBoW] Computing BoW for {} x {} descriptors",
                mDescriptors.size().height, mDescriptors.size().width);

    // 将描述子矩阵转为向量形式
    vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // 调用词典进行转换，4为词典分支因子
    mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);

    oslog::info(
        "[Frame::ComputeBoW]  ... BoW vector size {}, feature vector size {}",
        mBowVec.size(), mFeatVec.size());
  }
}

// ==============================================
// 特征点去畸变
// 将原始畸变图像上的特征点转换为去畸变后的坐标，存入mvKeysUn
// ==============================================
void Frame::UndistortKeyPoints() {
  // 畸变系数k1为0表示无畸变，直接复制特征点
  if (mDistCoef.at<float>(0) == 0.0) {
    mvKeysUn = mvKeys;
    return;
  }

  // 构造N行2列的特征点坐标矩阵
  // Fill matrix with points
  cv::Mat mat(N, 2, CV_32F);

  for (size_t i = 0; i < N; i++) {
    mat.at<float>(i, 0) = mvKeys[i].pt.x;
    mat.at<float>(i, 1) = mvKeys[i].pt.y;
  }

  // 调用OpenCV去畸变函数
  // Undistort points
  mat = mat.reshape(2);
  cv::undistortPoints(mat, mat, dynamic_cast<Pinhole *>(mpCamera.get())->toK(),
                      mDistCoef, cv::Mat(), mK);
  mat = mat.reshape(1);

  // 填充去畸变后的特征点向量
  // Fill undistorted keypoint vector
  mvKeysUn.resize(N);
  for (size_t i = 0; i < N; i++) {
    cv::KeyPoint kp = mvKeys[i];
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    mvKeysUn[i] = kp;
  }
}

// ==============================================
// 计算去畸变后的图像有效边界
// 对图像四个角点去畸变，取最小/最大坐标作为有效图像范围
// ==============================================
void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
  // 有畸变时计算四个角点去畸变后的坐标
  if (mDistCoef.at<float>(0) != 0.0) {
    // 构造四个角点：左上、右上、左下、右下
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;
    mat.at<float>(1, 0) = imLeft.cols;
    mat.at<float>(1, 1) = 0.0;
    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imLeft.rows;
    mat.at<float>(3, 0) = imLeft.cols;
    mat.at<float>(3, 1) = imLeft.rows;

    // 对角点执行去畸变
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat,
                        dynamic_cast<Pinhole *>(mpCamera.get())->toK(),
                        mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);

    // 取最小最大坐标作为有效边界
    // Undistort corners
    mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
  } else {
    // 无畸变时直接使用图像原始尺寸
    mnMinX = 0.0f;
    mnMaxX = imLeft.cols;
    mnMinY = 0.0f;
    mnMaxY = imLeft.rows;
  }
}

// ==============================================
// 计算针孔双目立体匹配
// 基于行搜索+描述子匹配+亚像素拟合，得到每个左目特征点的右目坐标与深度
// ==============================================
void Frame::ComputeStereoMatches() {
  // 初始化右目u坐标和深度数组，-1表示无效
  mvuRight = vector<float>(N, -1.0f);
  mvDepth = vector<float>(N, -1.0f);

  // ORB描述子匹配阈值：高低阈值的平均值
  const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;
  // 第一层金字塔图像的高度（行数）
  const size_t nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

  // 构建行索引表：每个图像行对应右目特征点索引列表，大幅加速行搜索
  // Assign keypoints to row table
  vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

  for (size_t i = 0; i < nRows; i++) vRowIndices[i].reserve(200);

  // 右目特征点总数
  const size_t Nr = mvKeysRight.size();

  oslog::trace("Checking {} left and {} right keypoints", N, Nr);

  // 将右目特征点按行号存入索引表
  for (size_t iR = 0; iR < Nr; iR++) {
    const cv::KeyPoint &kp = mvKeysRight[iR];
    const float &kpY = kp.pt.y;
    // 搜索范围：上下2倍当前尺度的像素范围
    const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
    const int maxr = ceil(kpY + r);
    const int minr = floor(kpY - r);

    // 将特征点索引加入对应行的列表
    for (int yi = minr; yi <= maxr; yi++) vRowIndices[yi].push_back(iR);
  }

  // 设置视差搜索范围：最小视差mb，最大视差maxD = bf / minZ
  // Set limits for search
  const float minZ = mb;
  const float minD = 0;
  const float maxD = mbf / minZ;

  oslog::trace("minZ {} minD {} maxD {}", minZ, minD, maxD);

  // 遍历每个左目特征点，在右目中搜索匹配
  // For each left keypoint search a match in the right image
  vector<pair<int, int>> vDistIdx;
  vDistIdx.reserve(N);

  for (size_t iL = 0; iL < static_cast<size_t>(N); iL++) {
    const cv::KeyPoint &kpL = mvKeys[iL];
    const int &levelL = kpL.octave;
    const float &vL = kpL.pt.y;
    const float &uL = kpL.pt.x;

    // 获取当前行的右目候选特征点
    const vector<size_t> &vCandidates = vRowIndices[vL];

    if (iL < 10)
      oslog::trace("Pt at ({},{})  {} candidates", uL, vL, vCandidates.size());

    if (vCandidates.empty()) continue;

    // 计算视差范围对应的u坐标范围
    const float minU = uL - maxD;
    const float maxU = uL - minD;

    if (iL < 10)
      oslog::trace("uL {} maxD {} minD {} maxU {} minU {}", uL, maxD, minD,
                   maxU, minU);

    if (maxU < 0) continue;

    // 最佳匹配距离和对应右目索引
    int bestDist = ORBmatcher::TH_HIGH;
    size_t bestIdxR = 0;

    const cv::Mat &dL = mDescriptors.row(iL);
    // Compare descriptor to right keypoints
    // 遍历所有候选点，计算描述子汉明距离
    for (size_t iC = 0; iC < vCandidates.size(); iC++) {
      const size_t iR = vCandidates[iC];
      const cv::KeyPoint &kpR = mvKeysRight[iR];

      // 尺度差不超过1层
      if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1) continue;

      const float &uR = kpR.pt.x;

      // u坐标在视差范围内
      if (uR >= minU && uR <= maxU) {
        const cv::Mat &dR = mDescriptorsRight.row(iR);
        const int dist = ORBmatcher::DescriptorDistance(dL, dR);

        // 更新最佳匹配
        if (dist < bestDist) {
          bestDist = dist;
          bestIdxR = iR;
        }
      }
    }

    // 亚像素级匹配：通过图像块相关拟合亚像素精度
    // Subpixel match by correlation
    if (bestDist < thOrbDist) {
      // coordinates in image pyramid at keypoint scale
      // 右目匹配点的原始u坐标
      const float uR0 = mvKeysRight[bestIdxR].pt.x;
      // 当前层级的尺度因子倒数
      const float scaleFactor = mvInvScaleFactors[kpL.octave];
      // 左目点缩放到对应金字塔层级的坐标
      const float scaleduL = round(kpL.pt.x * scaleFactor);
      const float scaledvL = round(kpL.pt.y * scaleFactor);
      // 右目点缩放到对应金字塔层级的坐标
      const float scaleduR0 = round(uR0 * scaleFactor);

      // sliding window search
      // 滑动窗口半宽
      const int w = 5;
      // 提取左目对应位置的图像块
      cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave]
                       .rowRange(scaledvL - w, scaledvL + w + 1)
                       .colRange(scaleduL - w, scaleduL + w + 1);

      int bestDist = INT_MAX;
      int bestincR = 0;
      // 滑动范围：左右各5像素
      const int L = 5;
      vector<float> vDists;
      vDists.resize(2 * L + 1);

      // 滑动窗口的起止坐标
      const float iniu = scaleduR0 + L - w;
      const float endu = scaleduR0 + L + w + 1;
      if (iniu < 0 ||
          endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
        continue;

      // 沿u方向滑动窗口，计算L1距离
      for (int incR = -L; incR <= +L; incR++) {
        cv::Mat IR =
            mpORBextractorRight->mvImagePyramid[kpL.octave]
                .rowRange(scaledvL - w, scaledvL + w + 1)
                .colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);

        float dist = cv::norm(IL, IR, cv::NORM_L1);
        if (dist < bestDist) {
          bestDist = dist;
          bestincR = incR;
        }

        vDists[L + incR] = dist;
      }

      // 最佳匹配在边界上则跳过，结果不可靠
      if (bestincR == -L || bestincR == L) continue;

      // 抛物线拟合亚像素精度：利用相邻三个点的距离拟合抛物线，求最小值位置
      // Sub-pixel match (Parabola fitting)
      const float dist1 = vDists[L + bestincR - 1];
      const float dist2 = vDists[L + bestincR];
      const float dist3 = vDists[L + bestincR + 1];

      // 抛物线顶点偏移量
      const float deltaR =
          (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

      if (deltaR < -1 || deltaR > 1) continue;

      // 还原到原始图像分辨率的右目u坐标
      // Re-scaled coordinate
      float bestuR =
          mvScaleFactors[kpL.octave] * (static_cast<float>(scaleduR0) +
                                         static_cast<float>(bestincR) + deltaR);

      // 计算视差
      float disparity = (uL - bestuR);

      // 视差在有效范围内
      if (disparity >= minD && disparity < maxD) {
        // 视差为0或负则修正为极小值
        if (disparity <= 0) {
          disparity = 0.01;
          bestuR = uL - 0.01;
        }
        // 计算深度：depth = bf / disparity
        mvDepth[iL] = mbf / disparity;
        // 保存右目u坐标
        mvuRight[iL] = bestuR;

        // 保存匹配距离和索引，用于后续过滤
        // Save a list of the lowest correlation distance seen for each left
        // feature
        vDistIdx.push_back(pair<int, int>(bestDist, iL));
      }
    }
  }

  // 按匹配距离升序排序
  sort(vDistIdx.begin(), vDistIdx.end());
  // 取中位数作为参考
  const float median = vDistIdx[vDistIdx.size() / 2].first;
  // 阈值设为中位数的1.5*1.4倍
  const float thDist = 1.5f * 1.4f * median;  //?

  // 过滤掉匹配距离大于阈值的误匹配
  // Discard any depths where the correlation distance was greater than
  // some threshold from the median correlation distance.
  for (int i = static_cast<int>(vDistIdx.size() - 1); i >= 0; i--) {
    if (vDistIdx[i].first < thDist) {
      break;
    } else {
      mvuRight[vDistIdx[i].second] = -1;
      mvDepth[vDistIdx[i].second] = -1;
    }
  }

  // 统计有效匹配数量
  int keptCount = 0;
  for (auto d : mvDepth) {
    if (d > 0) keptCount++;
  }

  oslog::debug("Frame {}.  Kept {} stereo matches of {} features", mnId,
               keptCount, vDistIdx.size());
}

// ==============================================
// 从RGBD深度图计算深度和立体坐标
// ==============================================
void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth) {
  // 初始化右目u坐标和深度数组
  mvuRight = vector<float>(N, -1);
  mvDepth = vector<float>(N, -1);

  // 遍历每个特征点
  for (size_t i = 0; i < N; i++) {
    const cv::KeyPoint &kp = mvKeys[i];
    const cv::KeyPoint &kpU = mvKeysUn[i];

    const float &v = kp.pt.y;
    const float &u = kp.pt.x;

    // 从深度图读取对应像素的深度值
    const float d = imDepth.at<float>(v, u);

    // 深度有效则计算对应右目u坐标
    if (d > 0) {
      mvDepth[i] = d;
      // 视差 = bf / depth，右目u = 左目u - 视差
      mvuRight[i] = kpU.pt.x - mbf / d;
    }
  }
}

// ==============================================
// 立体特征点反投影到世界坐标系
// i: 特征点索引
// x3D: 输出世界坐标系3D点
// ==============================================
bool Frame::UnprojectStereo(const int &i, Eigen::Vector3f &x3D) {
  const float z = mvDepth[i];
  if (z > 0) {
    // 获取去畸变的特征点像素坐标
    const float u = mvKeysUn[i].pt.x;
    const float v = mvKeysUn[i].pt.y;
    // 像素坐标反投影为相机坐标系3D点
    const float x = (u - cx) * z * invfx;
    const float y = (v - cy) * z * invfy;
    Eigen::Vector3f x3Dc(x, y, z);
    // 相机坐标转换为世界坐标
    x3D = mRwc * x3Dc + mOw;
    return true;
  } else {
    return false;
  }
}

// ==============================================
// 检查IMU是否已完成预积分（线程安全）
// ==============================================
bool Frame::imuIsPreintegrated() {
  unique_lock<std::mutex> lock(*mpMutexImu);
  return mbImuPreintegrated;
}

// ==============================================
// 标记IMU预积分完成（线程安全）
// ==============================================
void Frame::setIntegrated() {
  unique_lock<std::mutex> lock(*mpMutexImu);
  mbImuPreintegrated = true;
}

// ==============================================
// 鱼眼双目相机构造函数
// 输入：左右目图像、时间戳、左右ORB提取器、词袋、内参、畸变、bf、深度阈值、左右相机、相对位姿Tlr、上一帧、IMU标定
// ==============================================
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,
             const double &timeStamp,
             const std::shared_ptr<ORBextractor> &extractorLeft,
             const std::shared_ptr<ORBextractor> &extractorRight,
             const std::shared_ptr<ORBVocabulary> &voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth,
             const std::shared_ptr<GeometricCamera> &pCamera,
             const std::shared_ptr<GeometricCamera> &pCamera2,
             Sophus::SE3f &Tlr, const std::shared_ptr<Frame> &pPrevF,
             const IMU::Calib &ImuCalib)
    // 初始化列表
    : mpcpi(nullptr),
      mpORBvocabulary(voc),
      mpORBextractorLeft(extractorLeft),
      mpORBextractorRight(extractorRight),
      mTimeStamp(timeStamp),
      mK(K.clone()),
      mK_(Converter::toMatrix3f(K)),
      mDistCoef(distCoef.clone()),
      mbf(bf),
      mThDepth(thDepth),
      mImuCalib(ImuCalib),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(pPrevF),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(),
      mbImuPreintegrated(false),
      mpCamera(pCamera),
      mpCamera2(pCamera2),
      mbHasPose(false),
      mbHasVelocity(false) {
  // 克隆左右目原始图像
  imgLeft = imLeft.clone();
  imgRight = imRight.clone();

  // 分配帧ID
  // Frame ID
  mnId = nNextId++;

  // 初始化金字塔尺度参数
  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // 并行提取左右目ORB特征，仅提取双目重叠区域
  // ORB extraction
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartExtORB =
      std::chrono::steady_clock::now();
#endif
  // 左目提取：指定重叠区域x坐标范围
  thread threadLeft(&Frame::ExtractORB, this, 0, imLeft,
                    dynamic_cast<KannalaBrandt8 &>(*mpCamera).mvLappingArea[0],
                    dynamic_cast<KannalaBrandt8 &>(*mpCamera).mvLappingArea[1]);
  // 右目提取：指定重叠区域x坐标范围
  thread threadRight(
      &Frame::ExtractORB, this, 1, imRight,
      dynamic_cast<KannalaBrandt8 &>(*mpCamera2).mvLappingArea[0],
      dynamic_cast<KannalaBrandt8 &>(*mpCamera2).mvLappingArea[1]);
  threadLeft.join();
  threadRight.join();
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndExtORB =
      std::chrono::steady_clock::now();
  mTimeORB_Ext =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndExtORB - time_StartExtORB)
          .count();
#endif

  // 统计左右目特征点数量和总数
  Nleft = mvKeys.size();
  Nright = mvKeysRight.size();
  N = Nleft + Nright;

  if (N == 0) return;

  // 首次计算图像边界、网格、内参
  // This is done only for the first Frame (or after a change in the
  // calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imLeft);

    mfGridElementWidthInv =
        static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
    mfGridElementHeightInv =
        static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }

  // 计算像素基线
  mb = mbf / fx;

  // 保存左右目相对位姿及派生量
  // Sophus/Eigen
  mTlr = Tlr;
  mTrl = mTlr.inverse();
  mRlr = mTlr.rotationMatrix();
  mtlr = mTlr.translation();

// 计算鱼眼立体匹配
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartStereoMatches =
      std::chrono::steady_clock::now();
#endif
  ComputeStereoFishEyeMatches();
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndStereoMatches =
      std::chrono::steady_clock::now();
  mTimeStereoMatch =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          time_EndStereoMatches - time_StartStereoMatches)
          .count();
#endif

  // 合并左右目描述子到同一个矩阵
  // Put all descriptors in the same matrix
  cv::vconcat(mDescriptors, mDescriptorsRight, mDescriptors);

  // 初始化地图点和外点数组
  mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));
  mvbOutlier = vector<bool>(N, false);

  // 特征点分配到网格
  AssignFeaturesToGrid();

  // 创建IMU互斥锁
  mpMutexImu = std::make_shared<std::mutex>();

  // 特征点去畸变
  UndistortKeyPoints();
}

// ==============================================
// 计算鱼眼双目立体匹配
// 基于重叠区域特征+knn匹配+Lowe比率检验+三角化，得到立体3D点
// ==============================================
void Frame::ComputeStereoFishEyeMatches() {
  // 提取重叠区域的特征点和描述子用于匹配，提升速度
  // Speed it up by matching keypoints in the lapping area
  vector<cv::KeyPoint> stereoLeft(mvKeys.begin() + monoLeft, mvKeys.end());
  vector<cv::KeyPoint> stereoRight(mvKeysRight.begin() + monoRight,
                                    mvKeysRight.end());

  cv::Mat stereoDescLeft = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
  cv::Mat stereoDescRight =
      mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);

  // 初始化匹配索引、深度、3D点数组
  mvLeftToRightMatch = vector<int>(Nleft, -1);
  mvRightToLeftMatch = vector<int>(Nright, -1);
  mvDepth = vector<float>(Nleft, -1.0f);
  mvuRight = vector<float>(Nleft, -1);
  mvStereo3Dpoints =
      vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>(Nleft);
  mnCloseMPs = 0;

  // 使用暴力匹配器进行knn匹配，每个点找2个最近邻
  // Perform a brute force between Keypoint in the left and right image
  vector<vector<cv::DMatch>> matches;

  BFmatcher.knnMatch(stereoDescLeft, stereoDescRight, matches, 2);

  int nMatches = 0;
  int descMatches = 0;

  // Lowe's ratio test：最近邻距离 / 次近邻距离 < 0.7 判定为有效匹配
  // Check matches using Lowe's ratio
  for (vector<vector<cv::DMatch>>::iterator it = matches.begin();
       it != matches.end(); ++it) {
    if ((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7) {
      // 对有效匹配进行三角化，计算3D点和深度，过滤误匹配
      // For every good match, check parallax and reprojection error to discard
      // spurious matches
      Eigen::Vector3f p3D;
      descMatches++;
      // 左右目特征点对应的尺度方差
      float
          sigma1 = mvLevelSigma2[mvKeys[(*it)[0].queryIdx + monoLeft].octave],
          sigma2 =
              mvLevelSigma2[mvKeysRight[(*it)[0].trainIdx + monoRight].octave];
      // 调用鱼眼相机的三角化函数
      float depth = dynamic_cast<KannalaBrandt8 *>(mpCamera.get())
                        ->TriangulateMatches(
                            mpCamera2, mvKeys[(*it)[0].queryIdx + monoLeft],
                            mvKeysRight[(*it)[0].trainIdx + monoRight], mRlr,
                            mtlr, sigma1, sigma2, p3D);
      // 深度有效则保存匹配结果
      if (depth > 0.001f) {
        mvLeftToRightMatch[(*it)[0].queryIdx + monoLeft] =
            (*it)[0].trainIdx + monoRight;
        mvRightToLeftMatch[(*it)[0].trainIdx + monoRight] =
            (*it)[0].queryIdx + monoLeft;
        mvStereo3Dpoints[(*it)[0].queryIdx + monoLeft] = p3D;
        mvDepth[(*it)[0].queryIdx + monoLeft] = depth;
        nMatches++;
      }
    }
  }
}

// ==============================================
// 视锥检查辅助函数（支持左右目相机）
// bRight: true=检查右相机，false=检查左相机
// ==============================================
bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit,
                              bool bRight) {
  // 获取地图点世界坐标
  // 3D in absolute coordinates
  Eigen::Vector3f P = pMP->GetWorldPos();

  Eigen::Matrix3f mR;
  Eigen::Vector3f mt, twc;
  if (bRight) {
    // 右相机：左→右相对变换 × 世界→左相机变换
    Eigen::Matrix3f Rrl = mTrl.rotationMatrix();
    Eigen::Vector3f trl = mTrl.translation();
    mR = Rrl * mRcw;
    mt = Rrl * mtcw + trl;
    twc = mRwc * mTlr.translation() + mOw;
  } else {
    // 左相机：直接使用世界到左相机的变换
    mR = mRcw;
    mt = mtcw;
    twc = mOw;
  }

  // 转换到相机坐标系
  // 3D in camera coordinates
  Eigen::Vector3f Pc = mR * P + mt;
  const float Pc_dist = Pc.norm();
  const float &PcZ = Pc(2);

  // 检查正深度
  // Check positive depth
  if (PcZ < 0.0f) return false;

  // 投影到图像平面
  // Project in image and check it is not outside
  Eigen::Vector2f uv;
  if (bRight)
    uv = mpCamera2->project(Pc);
  else
    uv = mpCamera->project(Pc);

  // 检查图像边界
  if (uv(0) < mnMinX || uv(0) > mnMaxX) return false;
  if (uv(1) < mnMinY || uv(1) > mnMaxY) return false;

  // 检查距离是否在尺度不变范围内
  // Check distance is in the scale invariance region of the MapPoint
  const float maxDistance = pMP->GetMaxDistanceInvariance();
  const float minDistance = pMP->GetMinDistanceInvariance();
  const Eigen::Vector3f PO = P - twc;
  const float dist = PO.norm();

  if (dist < minDistance || dist > maxDistance) return false;

  // 检查视角
  // Check viewing angle
  Eigen::Vector3f Pn = pMP->GetNormal();

  const float viewCos = PO.dot(Pn) / dist;

  if (viewCos < viewingCosLimit) return false;

  // 预测尺度层级
  // Predict scale in the image
  const int nPredictedLevel = pMP->PredictScale(dist, shared_from_this());

  // 更新地图点的跟踪数据
  if (bRight) {
    pMP->mTrackProjXR = uv(0);
    pMP->mTrackProjYR = uv(1);
    pMP->mnTrackScaleLevelR = nPredictedLevel;
    pMP->mTrackViewCosR = viewCos;
    pMP->mTrackDepthR = Pc_dist;
  } else {
    pMP->mTrackProjX = uv(0);
    pMP->mTrackProjY = uv(1);
    pMP->mnTrackScaleLevel = nPredictedLevel;
    pMP->mTrackViewCos = viewCos;
    pMP->mTrackDepth = Pc_dist;
  }

  return true;
}

// ==============================================
// 鱼眼立体点反投影到世界坐标系
// ==============================================
Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i) {
  // 相机坐标系3D点转换为世界坐标
  return mRwc * mvStereo3Dpoints[i] + mOw;
}

}  // namespace ORB_SLAM3
