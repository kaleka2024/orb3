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
#include <list>                         // std::list双向链表容器，IMU队列、轨迹缓存使用
#include <memory>                       // 智能指针 shared_ptr / weak_ptr
#include <mutex>                        // 互斥锁，多线程数据保护
#include <opencv2/core/core.hpp>        // OpenCV核心模块，Mat、基础数据结构
#include <opencv2/features2d/features2d.hpp> // OpenCV特征点相关
#include <string>                       // std::string字符串
#include <unordered_set>                // 无序哈希集合
#include <vector>                       // std::vector动态数组

#include "Atlas.h"                      // 地图集，管理多子地图
#include "Frame.h"                      // 帧类，保存图像、特征、位姿、地图点关联
#include "FrameDrawer.h"                // 帧画面绘制器，可视化特征跟踪
#include "GeometricCamera.h"            // 几何相机模型，支持针孔、鱼眼
#include "ImuTypes.h"                   // IMU数据、预积分、标定参数结构体
#include "KeyFrameDatabase.h"           // 关键帧词袋数据库，重定位、回环检索
#include "LocalMapping.h"               // 局部建图模块
#include "LoopClosing.h"                // 回环检测模块
#include "MapDrawer.h"                  // 3D地图绘制器
#include "ORBVocabulary.h"              // ORB词袋词典
#include "ORBextractor.h"               // ORB特征提取器
#include "Settings.h"                   // 配置参数加载管理
#include "System.h"                     // System顶层系统类
#include "Viewer.h"                     // Pangolin可视化查看器

namespace ORB_SLAM3 {

// 前向声明
class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;

/**
 * @brief Tracking类，SLAM跟踪主线程，运行在调用System::TrackXXX的用户主线程
 * @details 负责图像接收、ORB特征提取、初始化、位姿跟踪、局部地图匹配、重定位、判断是否生成新关键帧；
 * 接收IMU数据，执行IMU预积分；不单独开辟线程，由外部循环驱动。
 */
class Tracking : public std::enable_shared_from_this<Tracking> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Eigen内存对齐宏，类包含Eigen成员变量必须添加

  /**
   * @brief Tracking构造函数
   * @param pSys System顶层系统智能指针
   * @param pVoc ORB词袋词典
   * @param pFrameDrawer 帧绘制器
   * @param pMapDrawer 地图绘制器
   * @param pAtlas 地图集
   * @param pKFDB 关键帧数据库
   * @param settings 配置参数
   * @param _nameSeq 数据集序列名称，调试用
   */
  Tracking(const std::shared_ptr<System> &pSys,
           const std::shared_ptr<ORBVocabulary> &pVoc,
           const std::shared_ptr<FrameDrawer> &pFrameDrawer,
           const std::shared_ptr<MapDrawer> &pMapDrawer,
           const std::shared_ptr<Atlas> &pAtlas,
           const std::shared_ptr<KeyFrameDatabase> &pKFDB,
           const std::shared_ptr<Settings> &settings,
           const string &_nameSeq = std::string());

  ~Tracking();

  // Preprocess the input and call Track(). Extract features and performs stereo
  // matching.
  /**
   * @brief 双目图像入口，接收校正后的左右目图像，预处理、特征提取、双目匹配，调用Track主逻辑
   * @param imRectLeft 校正后左目图像
   * @param imRectRight 校正后右目图像
   * @param timestamp 图像时间戳
   * @param filename 调试文件名
   * @return Sophus::SE3f Tcw 世界到相机位姿
   */
  Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,
                                const cv::Mat &imRectRight,
                                const double &timestamp, string filename);

  /**
   * @brief RGBD图像入口，接收彩色图与深度图，预处理后执行跟踪
   * @param imRGB RGB/灰度图像
   * @param imD 深度图
   * @param timestamp 时间戳
   * @param filename 调试文件名
   * @return Sophus::SE3f Tcw世界到相机位姿
   */
  Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD,
                              const double &timestamp, string filename);

  /**
   * @brief 单目图像入口，接收单张图像，预处理后执行跟踪
   * @param im 输入图像
   * @param timestamp 时间戳
   * @param filename 调试文件名
   * @return Sophus::SE3f Tcw世界到相机位姿
   */
  Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp,
                                   string filename);

  /**
   * @brief 外部传入单条IMU测量值，压入IMU数据队列
   * @param imuMeasurement IMU加速度陀螺仪测量点
   */
  void GrabImuData(const IMU::Point &imuMeasurement);

  void SetLocalMapper(const std::shared_ptr<LocalMapping> &pLocalMapper) {
    mpLocalMapper = pLocalMapper;
  }

  void SetLoopClosing(const std::shared_ptr<LoopClosing> &pLoopClosing) {
    mpLoopClosing = pLoopClosing;
  }

  void SetViewer(const std::shared_ptr<Viewer> &pViewer) { mpViewer = pViewer; }

  /**
   * @brief 设置单步调试模式，一帧一帧暂停
   * @param bSet true开启单步；false关闭
   */
  void SetStepByStep(bool bSet) { bStepByStep = bSet; }
  bool GetStepByStep() const { return bStepByStep; }

  // Load new settings
  // The focal lenght should be similar or scale prediction will fail when
  // projecting points
  /**
   * @brief 动态加载新相机标定配置文件；焦距不能差异过大，否则点投影尺度预测失效
   * @param strSettingPath yaml配置文件路径
   */
  void ChangeCalibration(const string &strSettingPath);

  // Use this function if you have deactivated local mapping and you only want
  // to localize the camera.
  /**
   * @brief 设置仅定位模式标记；局部建图关闭时调用，只做定位不新建地图
   * @param flag true开启仅定位；false关闭
   */
  void InformOnlyTracking(const bool &flag);

  /**
   * @brief IMU初始化完成后更新当前关键帧尺度、IMU偏置
   * @param s 尺度因子
   * @param b IMU偏置bias
   * @param pCurrentKeyFrame 当前关键帧
   */
  void UpdateFrameIMU(const float s, const IMU::Bias &b,
                       const std::shared_ptr<KeyFrame> &pCurrentKeyFrame);

  std::shared_ptr<KeyFrame> GetLastKeyFrame() { return mpLastKeyFrame; }

  /**
   * @brief 在Atlas中创建新地图，多地图切换时使用
   */
  void CreateMapInAtlas();

  // std::mutex mMutexTracks;

  //--
  /**
   * @brief 切换新数据集，重置内部状态
   */
  void NewDataset();
  /**
   * @brief 获取数据集序号
   * @return int 数据集编号
   */
  int GetNumberDataset();
  /**
   * @brief 获取当前帧匹配内点数量
   * @return int 内点数目
   */
  int GetMatchesInliers();

  // DEBUG
  /**
   * @brief 保存子轨迹，帧轨迹与关键帧轨迹输出文件
   * @param strNameFile_frames 普通帧输出文件名
   * @param strNameFile_kf 关键帧输出文件名
   * @param strFolder 输出文件夹
   */
  void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf,
                          string strFolder = "");
  /**
   * @brief 保存指定地图的子轨迹
   * @param strNameFile_frames 普通帧文件名
   * @param strNameFile_kf 关键帧文件名
   * @param pMap 目标地图
   */
  void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf,
                          const std::shared_ptr<Map> &pMap);

  /**
   * @brief 获取图像缩放系数
   * @return float 缩放比例
   */
  float GetImageScale();

#ifdef REGISTER_LOOP
  void RequestStop();         // 请求Tracking停止，回环模块使用
  bool isStopped();           // 查询Tracking是否已停止
  void Release();             // 解除停止阻塞
  bool stopRequested();       // 查询是否收到停止请求
#endif

 public:
  // Tracking states
  /**
   * @brief 跟踪状态枚举
   */
  enum eTrackingState {
    SYSTEM_NOT_READY = -1,    // 系统未就绪
    NO_IMAGES_YET = 0,        // 尚未收到图像
    NOT_INITIALIZED = 1,      // 未完成地图初始化
    OK = 2,                   // 跟踪正常
    RECENTLY_LOST = 3,        // 刚刚丢失，短时间内尝试重定位
    LOST = 4,                 // 跟踪彻底丢失
    OK_KLT = 5                // KLT光流模式正常
  };

  eTrackingState mState;                ///< 当前跟踪状态
  eTrackingState mLastProcessedState;    ///< 上一帧处理完成后的跟踪状态

  // Input sensor
  SensorType mSensor;                   ///< 传感器类型：MONO/STEREO/RGBD

  // Current Frame
  std::shared_ptr<Frame> mCurrentFrame; ///< 当前处理帧
  std::shared_ptr<Frame> mLastFrame;    ///< 上一帧

  cv::Mat mImGray;                      ///< 当前帧灰度图像

  // Initialization Variables (Monocular)
  std::vector<int> mvIniLastMatches;    ///< 初始化阶段上一帧匹配索引
  std::vector<int> mvIniMatches;        ///< 初始化阶段匹配对
  std::vector<cv::Point2f> mvbPrevMatched; ///< 初始化保存上一帧关键点像素坐标
  std::vector<cv::Point3f> mvIniP3D;    ///< 初始化三角化得到的3D点
  std::shared_ptr<Frame> mInitialFrame; ///< 初始化参考帧

  // Lists used to recover the full camera trajectory at the end of the
  // execution. Basically we store the reference keyframe for each frame and its
  // relative transformation
  list<Sophus::SE3f> mlRelativeFramePoses;    ///< 每帧相对参考关键帧的位姿
  list<std::shared_ptr<KeyFrame>> mlpReferences; ///< 每一帧对应的参考关键帧
  list<double> mlFrameTimes;                   ///< 每一帧时间戳
  list<bool> mlbLost;                         ///< 标记每一帧是否跟踪丢失

  // frames with estimated pose
  int mTrackedFr;                     ///< 成功跟踪的帧计数
  bool mbStep;                        ///< 单步模式标记

  // True if local mapping is deactivated and we are performing only
  // localization
  bool mbOnlyTracking;                ///< true仅定位模式，局部建图关闭

  /**
   * @brief 重置Tracking状态
   * @param bLocMap 是否重置局部建图
   */
  void Reset(bool bLocMap = false);
  /**
   * @brief 重置当前激活地图对应的Tracking
   * @param bLocMap 是否重置局部建图
   */
  void ResetActiveMap(bool bLocMap = false);

  float mMeanTrack;                   ///< 平均跟踪匹配数统计
  bool mbInitWith3KFs;                ///< 是否使用3个关键帧做IMU初始化
  double t0;                          ///< 读取到第一帧图像时间戳
  double t0vis;                       ///< 第一个关键帧插入时间戳
  double t0IMU;                       ///< IMU初始化完成时间戳
  bool mFastInit = false;             ///< 是否开启快速初始化

  vector<MapPoint *> GetLocalMapMPS();///< 获取局部地图点集合

  bool mbWriteStats;                  ///< 是否输出统计日志开关

#ifdef REGISTER_TIMES
  void LocalMapStats2File();          // 局部地图统计信息写入文件
  void TrackStats2File();             // 跟踪统计写入文件
  void PrintTimeStats();              // 打印各步骤耗时统计

  vector<double> vdRectStereo_ms;    // 双目校正耗时ms
  vector<double> vdResizeImage_ms;   // 图像缩放耗时ms
  vector<double> vdORBExtract_ms;    // ORB特征提取耗时ms
  vector<double> vdStereoMatch_ms;   // 双目匹配耗时ms
  vector<double> vdIMUInteg_ms;      // IMU预积分耗时ms
  vector<double> vdPosePred_ms;      // IMU位姿预测耗时ms
  vector<double> vdLMTrack_ms;       // 局部地图跟踪耗时ms
  vector<double> vdNewKF_ms;         // 新关键帧决策耗时ms
  vector<double> vdTrackTotal_ms;    // Track总耗时ms
#endif

 protected:
  // Main tracking function. It is independent of the input sensor.
  /**
   * @brief 跟踪主逻辑函数，与传感器类型无关，GrabImageXXX内部调用
   */
  void Track();

  // Map initialization for stereo and RGB‑D
  /**
   * @brief 双目/RGBD地图初始化，第一帧直接生成地图点、关键帧
   */
  void StereoInitialization();

  // Map initialization for monocular
  /**
   * @brief 单目初始化，寻找两帧足够匹配点，求解基础矩阵/单应矩阵
   */
  void MonocularInitialization();
  // void CreateNewMapPoints();
  /**
   * @brief 单目初始化成功后，三角化生成初始地图，创建初始关键帧
   */
  void CreateInitialMapMonocular();

  /**
   * @brief 检查上一帧地图点是否被替换，做替换更新
   */
  void CheckReplacedInLastFrame();
  /**
   * @brief 跟踪参考关键帧：词袋匹配参考关键帧地图点，PnP求解位姿
   * @return true跟踪成功；false失败
   */
  bool TrackReferenceKeyFrame();
  /**
   * @brief 更新上一帧，复制地图点，用于运动模型跟踪
   */
  void UpdateLastFrame();
  /**
   * @brief 运动模型跟踪：用上一帧速度预测当前帧位姿，投影匹配上一帧地图点，PnP优化
   * @return true跟踪成功；false失败
   */
  bool TrackWithMotionModel();
  /**
   * @brief IMU预测当前帧状态（位姿速度），VIO模式
   * @return true预测有效；false失败
   */
  bool PredictStateIMU();

  /**
   * @brief 重定位：词袋检索关键帧，多候选PnP求解，恢复跟踪
   * @return true重定位成功；false失败
   */
  bool Relocalization();

  /**
   * @brief 更新局部地图：更新局部关键帧、局部地图点
   */
  void UpdateLocalMap();
  /**
   * @brief 更新局部地图点，清理失效点
   */
  void UpdateLocalPoints();
  /**
   * @brief 更新局部关键帧：参考关键帧的共视关键帧集合
   */
  void UpdateLocalKeyFrames();

  /**
   * @brief 局部地图跟踪，将当前帧与局部地图点匹配，优化位姿，核心跟踪步骤
   * @return true跟踪成功；false失败
   */
  bool TrackLocalMap();
  /**
   * @brief 搜索局部地图点，投影到当前帧寻找匹配
   */
  void SearchLocalPoints();

  /**
   * @brief 判断是否需要生成新关键帧
   * @return true需要新建；false不需要
   */
  bool NeedNewKeyFrame();
  /**
   * @brief 创建新关键帧，插入Atlas，通知局部建图处理
   */
  void CreateNewKeyFrame();

  // Perform preintegration from last frame
  /**
   * @brief 对上一关键帧到当前帧之间IMU数据做预积分
   */
  void PreintegrateIMU();

  // Reset IMU biases and compute frame velocity
  /**
   * @brief 重置帧层面IMU偏置，计算帧速度
   */
  void ResetFrameIMU();

  bool mbMapUpdated;                          ///< 标记地图是否刚被更新（回环、BA）

  // Imu preintegration from last frame
  std::shared_ptr<IMU::Preintegrated> mpImuPreintegratedFromLastKF; ///< 从上一关键帧开始的IMU预积分

  // Queue of IMU measurements between frames
  std::list<IMU::Point> mlQueueImuData;       ///< IMU测量数据队列，多线程接收

  // Vector of IMU measurements from previous to current frame (to be filled by
  // PreintegrateIMU)
  std::vector<IMU::Point> mvImuFromLastFrame; ///< 上帧到当前帧截取出来用于预积分的IMU数据
  std::mutex mMutexImuQueue;                  ///< IMU队列读写互斥锁

  // Imu calibration parameters
  IMU::Calib mImuCalib;                       ///< IMU标定参数：噪声、随机游走、外参Tbc

  // Last Bias Estimation (at keyframe creation)
  IMU::Bias mLastBias;                        ///< 上一关键帧估计出的IMU bias

  // In case of performing only localization, this flag is true when there are
  // no matches to points in the map. Still tracking will continue if there are
  // enough matches with temporal points. In that case we are doing visual
  // odometry. The system will try to do relocalization to recover "zero‑drift"
  // localization to the map.
  bool mbVO; ///< 仅定位模式下，和地图匹配点不足，依靠临时点做视觉里程计VO标记

  // Other Thread Pointers
  std::shared_ptr<LocalMapping> mpLocalMapper; ///< 局部建图模块指针
  std::shared_ptr<LoopClosing> mpLoopClosing;  ///< 回环检测模块指针

  // ORB
  std::shared_ptr<ORBextractor> mpORBextractorLeft, mpORBextractorRight; ///< 左右目ORB提取器
  std::shared_ptr<ORBextractor> mpIniORBextractor; ///< 初始化阶段ORB提取器，特征点数量更多

  // BoW
  std::shared_ptr<ORBVocabulary> mpORBVocabulary; ///< ORB词袋词典
  std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB; ///< 关键帧数据库

  // Initalization (only for monocular)
  bool mbReadyToInitializate;   ///< 单目是否满足初始化条件
  bool mbSetInit;               ///< 初始化标记

  // Local Map
  std::shared_ptr<KeyFrame> mpReferenceKF;                ///< 当前帧参考关键帧
  std::vector<std::shared_ptr<KeyFrame>> mvpLocalKeyFrames; ///< 局部关键帧集合
  std::vector<MapPoint *> mvpLocalMapPoints;              ///< 局部地图点集合

  // System
  std::shared_ptr<System> mpSystem; ///< System顶层系统指针

  // Drawers
  std::shared_ptr<Viewer> mpViewer;         ///< 可视化Viewer
  std::shared_ptr<FrameDrawer> mpFrameDrawer;///< 帧绘制器
  std::shared_ptr<MapDrawer> mpMapDrawer;   ///< 地图绘制器
  bool bStepByStep;                         ///< 单步调试模式开关

  // Atlas
  std::shared_ptr<Atlas> mpAtlas;           ///< 地图集

  // Calibration matrix
  cv::Mat mK;                               ///< OpenCV格式相机内参矩阵
  Eigen::Matrix3f mK_;                      ///< Eigen格式相机内参矩阵
  cv::Mat mDistCoef;                        ///< 畸变系数
  float mbf;                                ///< 双目基线×焦距 mb = f*b
  float mImageScale;                        ///< 图像缩放系数

  float mImuFreq;                           ///< IMU采样频率
  double mImuPer;                           ///< IMU采样周期
  bool mInsertKFsLost;                      ///< 丢失状态是否允许插入关键帧

  // New KeyFrame rules (according to fps)
  int mMinFrames;                           ///< 相邻关键帧最小间隔帧数
  int mMaxFrames;                           ///< 相邻关键帧最大间隔帧数

  int mnFirstImuFrameId;                    ///< 收到IMU后的第一帧ID
  int mnFramesToResetIMU;                   ///< IMU重置等待帧数

  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two
  // keyframes.
  float mThDepth; ///< 远近点深度阈值；近点单帧即可生成地图点，远点需要两关键帧确认

  // For RGB‑D inputs only. For some datasets (e.g. TUM) the depthmap values are
  // scaled.
  float mDepthMapFactor; ///< RGBD深度图缩放因子，TUM数据集深度需要除以该系数得到米

  // Current matches in frame
  int mnMatchesInliers;  ///< 当前帧跟踪匹配内点数量

  // Last Frame, KeyFrame and Relocalisation Info
  std::shared_ptr<KeyFrame> mpLastKeyFrame; ///< 上一个关键帧
  unsigned int mnLastKeyFrameId;           ///< 上一个关键帧帧ID
  unsigned int mnLastRelocFrameId;         ///< 上一次重定位发生的帧ID
  double mTimeStampLost;                  ///< 跟踪丢失时间戳
  double time_recently_lost;              ///< RECENTLY_LOST状态维持时间阈值

  unsigned int mnFirstFrameId;             ///< 整个SLAM第一帧ID
  unsigned int mnInitialFrameId;           ///< 初始化开始帧ID
  unsigned int mnLastInitFrameId;          ///< 初始化最后一帧ID

  bool mbCreatedMap;                       ///< 是否已经创建地图标记

  // Motion Model
  bool mbVelocity{false};                  ///< 速度是否有效标记
  Sophus::SE3f mVelocity;                  ///< 相机运动模型速度，上帧T = v * dt

  // Color order (true RGB, false BGR, ignored if grayscale)
  bool mbRGB;                              ///< 图像色彩顺序：true RGB；false BGR

  list<MapPoint *> mlpTemporalPoints;      ///< 仅定位模式临时地图点，VO使用

  // int nMapChangeIndex;

  int mnNumDataset;                        ///< 当前数据集序号

  ofstream f_track_stats;                  ///< 跟踪统计输出文件流
  ofstream f_track_times;                  ///< 耗时统计输出文件流

  double mTime_PreIntIMU;                  ///< IMU预积分耗时
  double mTime_PosePred;                   ///< 位姿预测耗时
  double mTime_LocalMapTrack;              ///< 局部地图跟踪耗时
  double mTime_NewKF_Dec;                  ///< 新关键帧决策耗时

  std::shared_ptr<GeometricCamera> mpCamera, mpCamera2; ///< 左右相机模型对象

  int initID, lastID;                      ///< 初始化帧ID，最后帧ID

  Sophus::SE3f mTlr;                       ///< 左右相机外参Tlr，双目模式

  /**
   * @brief 从Settings加载更新全部参数
   * @param settings 配置参数智能指针
   */
  void newParameterLoader(const std::shared_ptr<Settings> &settings);

#ifdef REGISTER_LOOP
  bool Stop();                 // 执行停止Tracking
  bool mbStopped;              // Tracking已经停止标记
  bool mbStopRequested;        // 请求停止标记
  bool mbNotStop;              // 不允许停止标记
  std::mutex mMutexStop;       // 停止操作互斥锁
#endif

 public:
  cv::Mat mImRight; ///< 双目模式保存右目灰度图像
};

}  // namespace ORB_SLAM3
