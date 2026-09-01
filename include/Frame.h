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
// 头文件保护，防止头文件重复包含
#pragma once

// Eigen核心矩阵向量定义
#include <Eigen/Core>
// Eigen对std::vector容器内存对齐支持
#include <Eigen/StdVector>
// C++标准输入输出流
#include <iostream>
// C++智能指针
#include <memory>
// 互斥锁，多线程保护IMU相关成员
#include <mutex>
// OpenCV完整接口，图像、关键点、矩阵等
#include <opencv2/opencv.hpp>
// C++标准字符串
#include <string>
// C++标准动态数组
#include <vector>

// 格式转换工具，OpenCV/Eigen/g2o/Sophus互相转换
#include "Converter.h"
// IMU预积分、标定、偏置等数据类型
#include "ImuTypes.h"
// 日志输出模块
#include "Logging.h"
// ORB词袋模型
#include "ORBVocabulary.h"
// 系统配置参数管理
#include "Settings.h"
// DBoW2词袋向量定义
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
// DBoW2特征向量定义
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
// Sophus SE3位姿几何
#include "sophus/geometry.hpp"
// Sophus se3头文件
#include "sophus/se3.hpp"

namespace ORB_SLAM3 {

/// 图像网格划分行数，用于快速特征区域查找
#define FRAME_GRID_ROWS 48
/// 图像网格划分列数，用于快速特征区域查找
#define FRAME_GRID_COLS 64

// 前向声明，减少头文件循环依赖
class MapPoint;
class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;

/**
 * @brief 普通帧Frame类，存储一帧图像全部信息：图像、关键点、描述子、位姿、IMU、立体匹配、网格、地图点关联
 * 继承std::enable_shared_from_this，可以从对象内部获取自身shared_ptr，供IMU预积分等场景使用
 */
class Frame : public std::enable_shared_from_this<Frame> {
public:
    // Eigen内存对齐宏，保证Eigen成员内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 默认构造函数
     */
    Frame();

    // Copy constructor.
    /**
     * @brief 拷贝构造函数，复制另一帧的全部数据
     * @param frame 源Frame对象
     */
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    /**
     * @brief 双目相机Frame构造函数
     * @param imLeft 左目图像
     * @param imRight 右目图像
     * @param timeStamp 帧时间戳
     * @param extractorLeft 左目ORB特征提取器
     * @param extractorRight 右目ORB特征提取器
     * @param voc ORB词袋
     * @param K 相机内参矩阵cv::Mat
     * @param distCoef 畸变系数
     * @param bf 基线*fx
     * @param thDepth 远近点深度阈值
     * @param pCamera 主相机模型
     * @param pPrevF 上一帧，用于IMU预积分，默认为nullptr
     * @param ImuCalib IMU标定参数
     */
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
          const std::shared_ptr<ORBextractor> &extractorLeft,
          const std::shared_ptr<ORBextractor> &extractorRight,
          const std::shared_ptr<ORBVocabulary> &voc, cv::Mat &K,
          cv::Mat &distCoef, const float &bf, const float &thDepth,
          const std::shared_ptr<GeometricCamera> &pCamera,
          const std::shared_ptr<Frame> &pPrevF = nullptr,
          const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for RGB‑D cameras.
    /**
     * @brief RGB‑D相机Frame构造函数
     * @param imGray 灰度彩色图
     * @param imDepth 深度图
     * @param timeStamp 帧时间戳
     * @param extractor ORB特征提取器
     * @param voc ORB词袋
     * @param K 相机内参
     * @param distCoef 畸变系数
     * @param bf 基线*fx
     * @param thDepth 远近点深度阈值
     * @param pCamera 相机模型
     * @param pPrevF 上一帧，IMU预积分用
     * @param ImuCalib IMU标定参数
     */
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp,
          const std::shared_ptr<ORBextractor> &extractor,
          const std::shared_ptr<ORBVocabulary> &voc, cv::Mat &K,
          cv::Mat &distCoef, const float &bf, const float &thDepth,
          const std::shared_ptr<GeometricCamera> &pCamera,
          const std::shared_ptr<Frame> &pPrevF = nullptr,
          const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for Monocular cameras.
    /**
     * @brief 单目相机Frame构造函数
     * @param imGray 灰度图像
     * @param timeStamp 帧时间戳
     * @param extractor ORB特征提取器
     * @param voc ORB词袋
     * @param pCamera 相机模型
     * @param distCoef 畸变系数
     * @param bf 基线*fx（单目一般无效）
     * @param thDepth 远近点深度阈值
     * @param pPrevF 上一帧，IMU预积分用
     * @param ImuCalib IMU标定参数
     */
    Frame(const cv::Mat &imGray, const double &timeStamp,
          const std::shared_ptr<ORBextractor> &extractor,
          const std::shared_ptr<ORBVocabulary> &voc,
          const std::shared_ptr<GeometricCamera> &pCamera, cv::Mat &distCoef,
          const float &bf, const float &thDepth,
          const std::shared_ptr<Frame> &pPrevF = nullptr,
          const IMU::Calib &ImuCalib = IMU::Calib());

    // Destructor
    /**
     * @brief Frame析构函数
     */
    ~Frame();

    // Extract ORB on the image. 0 for left image and 1 for right image.
    /**
     * @brief 提取ORB特征点与描述子
     * @param flag 0代表左图，1代表右图
     * @param im 待提取图像
     * @param x0 提取区域左边界
     * @param x1 提取区域右边界
     */
    void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

    // Compute Bag of Words representation.
    /**
     * @brief 计算当前帧的词袋BowVector、FeatureVector，用于重定位、回环
     */
    void ComputeBoW();

    // Set the camera pose. (Imu pose is not modified!)
    /**
     * @brief 设置相机位姿Tcw(世界到相机)，不会修改IMU位姿
     * @param Tcw Sophus SE3 float 世界到相机变换
     */
    void SetPose(const Sophus::SE3<float> &Tcw);

    // Set IMU velocity
    /**
     * @brief 设置IMU在世界坐标系下的速度Vw
     * @param Vw 世界坐标系速度向量
     */
    void SetVelocity(const Eigen::Vector3f &Vw);

    /**
     * @brief 获取IMU世界坐标系速度
     * @return Eigen::Vector3f 速度向量
     */
    Eigen::Vector3f GetVelocity() const;

    // Set IMU pose and velocity (implicitly changes camera pose)
    /**
     * @brief 设置IMU的位姿Rwb、twb与速度Vwb，会同步更新相机位姿
     * @param Rwb IMU到世界旋转矩阵
     * @param twb IMU到世界平移向量
     * @param Vwb IMU世界坐标系速度
     */
    void SetImuPoseVelocity(const Eigen::Matrix3f &Rwb,
                            const Eigen::Vector3f &twb,
                            const Eigen::Vector3f &Vwb);

    /**
     * @brief 获取IMU在世界坐标系下位置
     * @return Eigen::Matrix<float,3,1> IMU世界位置
     */
    Eigen::Matrix<float, 3, 1> GetImuPosition() const;
    /**
     * @brief 获取IMU到世界旋转矩阵
     * @return Eigen::Matrix<float,3,3> Rwb
     */
    Eigen::Matrix<float, 3, 3> GetImuRotation();
    /**
     * @brief 获取IMU的SE3位姿Twb
     * @return Sophus::SE3<float> IMU位姿
     */
    Sophus::SE3<float> GetImuPose();

    /**
     * @brief 获取从右相机到左相机相对位姿Trl
     * @return Sophus::SE3f Trl
     */
    Sophus::SE3f GetRelativePoseTrl();
    /**
     * @brief 获取从左相机到右相机相对位姿Tlr
     * @return Sophus::SE3f Tlr
     */
    Sophus::SE3f GetRelativePoseTlr();
    /**
     * @brief 获取Tlr的旋转部分
     * @return Eigen::Matrix3f 旋转矩阵
     */
    Eigen::Matrix3f GetRelativePoseTlr_rotation();
    /**
     * @brief 获取Tlr的平移部分
     * @return Eigen::Vector3f 平移向量
     */
    Eigen::Vector3f GetRelativePoseTlr_translation();

    /**
     * @brief 设置IMU新的偏置值
     * @param b IMU偏置对象
     */
    void SetNewBias(const IMU::Bias &b);

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    /**
     * @brief 判断地图点是否在当前帧视锥内，同时更新地图点观测角度
     * @param pMP 待检测地图点
     * @param viewingCosLimit 观测视角余弦阈值
     * @return true 在视锥内；false不在视锥内
     */
    bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

    /**
     * @brief 将地图点投影到图像，考虑畸变，输出图像坐标与归一化平面坐标
     * @param pMP 地图点
     * @param kp 输出像素坐标
     * @param u 输出归一化平面u
     * @param v 输出归一化平面v
     * @return true投影成功；false投影失败
     */
    bool ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v);

    /**
     * @brief 将世界坐标系点转换到相机坐标系
     * @param pCw 世界坐标系三维点
     * @return Eigen::Vector3f 相机坐标系三维点
     */
    Eigen::Vector3f inRefCoordinates(Eigen::Vector3f pCw);

    // Compute the cell of a keypoint (return false if outside the grid)
    /**
     * @brief 计算关键点落在网格的行列位置，超出图像返回false
     * @param kp 输入关键点
     * @param posX 输出网格列索引
     * @param posY 输出网格行索引
     * @return true在网格内；false超出图像边界
     */
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    /**
     * @brief 在指定图像圆形区域内获取满足金字塔层级的关键点索引，用于匹配加速
     * @param x 区域中心x像素
     * @param y 区域中心y像素
     * @param r 搜索半径
     * @param minLevel 最小金字塔层级，‑1不限制
     * @param maxLevel 最大金字塔层级，‑1不限制
     * @param bRight 是否右目图像关键点
     * @return vector<size_t> 关键点索引集合
     */
    vector<size_t> GetFeaturesInArea(const float &x, const float &y,
                                     const float &r, const int minLevel = -1,
                                     const int maxLevel = -1,
                                     const bool bRight = false) const;

    // Search a match for each keypoint in the left image to a keypoint in the
    // right image. If there is a match, depth is computed and the right
    // coordinate associated to the left keypoint is stored.
    /**
     * @brief 双目匹配：左图关键点在右图搜索匹配，计算深度，保存右图匹配坐标
     */
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the
    // depthmap.
    /**
     * @brief RGB‑D模式，根据深度图为每个关键点生成虚拟右目坐标与深度值
     * @param imDepth 输入深度图
     */
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world
    // coordinates.
    /**
     * @brief 反投影关键点，利用立体/RGBD深度得到世界坐标系三维点
     * @param i 关键点索引
     * @param x3D 输出世界三维点
     * @return true深度有效反投影成功；false深度无效
     */
    bool UnprojectStereo(const int &i, Eigen::Vector3f &x3D);

    /// IMU‑Pose约束指针
    ConstraintPoseImu *mpcpi;

    /**
     * @brief 判断IMU预积分是否已经完成
     * @return true已预积分；false未预积分
     */
    bool imuIsPreintegrated();
    /**
     * @brief 设置IMU预积分完成标记
     */
    void setIntegrated();

    /**
     * @brief 判断帧位姿是否已经设置
     * @return true位姿已设置；false未设置
     */
    bool isSet() const;

    // Computes rotation, translation and camera center matrices from the camera
    // pose.
    /**
     * @brief 根据mTcw更新mRcw、mtcw、mRwc、mOw等矩阵，SetPose之后调用
     */
    void UpdatePoseMatrices();

    // Returns the camera center.
    /**
     * @brief 获取相机光心在世界坐标系坐标Ow
     * @return Eigen::Vector3f 光心世界坐标
     */
    inline Eigen::Vector3f GetCameraCenter() { return mOw; }

    // Returns inverse of rotation
    /**
     * @brief 获取世界到相机旋转的逆Rwc(相机到世界旋转)
     * @return Eigen::Matrix3f Rwc
     */
    inline Eigen::Matrix3f GetRotationInverse() { return mRwc; }

    /**
     * @brief 获取相机位姿Tcw：世界→相机SE3变换
     * @return Sophus::SE3<float> Tcw位姿
     * @note TODO: can the Frame pose be accsessed from several threads? should this
     * be protected somehow?
     */
    inline Sophus::SE3<float> GetPose() const {
        return mTcw;
    }

    /**
     * @brief 获取Rwc，相机到世界旋转矩阵
     * @return Eigen::Matrix3f
     */
    inline Eigen::Matrix3f GetRwc() const { return mRwc; }
    /**
     * @brief 获取相机光心世界坐标Ow
     * @return Eigen::Vector3f
     */
    inline Eigen::Vector3f GetOw() const { return mOw; }
    /**
     * @brief 判断是否拥有有效相机位姿
     * @return true有位姿；false无位姿
     */
    inline bool HasPose() const { return mbHasPose; }
    /**
     * @brief 判断是否拥有IMU速度
     * @return true有速度；false无速度
     */
    inline bool HasVelocity() const { return mbHasVelocity; }

private:
    // Sophus/Eigen migration
    /// Tcw：世界坐标系到相机坐标系SE3变换
    Sophus::SE3<float> mTcw;
    /// Rwc：相机坐标系旋转到世界坐标系 Rwc = Rcw^T
    Eigen::Matrix<float, 3, 3> mRwc;
    /// mOw：相机光心在世界坐标系位置
    Eigen::Matrix<float, 3, 1> mOw;
    /// mRcw：世界旋转到相机坐标系
    Eigen::Matrix<float, 3, 3> mRcw;
    /// mtcw：世界到相机平移向量
    Eigen::Matrix<float, 3, 1> mtcw;
    /// 标记是否已经设置有效位姿
    bool mbHasPose;

    // Rcw_ not necessary as Sophus has a method for extracting the rotation
    // matrix: Tcw_.rotationMatrix() tcw_ not necessary as Sophus has a method for
    // extracting the translation vector: Tcw_.translation() Twc_ not necessary as
    // Sophus has a method for easily computing the inverse pose: Tcw_.inverse()

    /// Tlr：左相机到右相机相对位姿
    Sophus::SE3<float> mTlr, mTrl;
    /// Tlr旋转部分
    Eigen::Matrix<float, 3, 3> mRlr;
    /// Tlr平移部分
    Eigen::Vector3f mtlr;

    // IMU linear velocity
    /// mVw：IMU在世界坐标系下速度
    Eigen::Vector3f mVw;
    /// 是否拥有有效IMU速度
    bool mbHasVelocity;

public:
    // Vocabulary used for relocalization.
    /// ORB词袋，用于重定位、回环检测
    std::shared_ptr<ORBVocabulary> mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    /// ORB特征提取器，右目仅双目模式使用
    std::shared_ptr<ORBextractor> mpORBextractorLeft, mpORBextractorRight;

    // Frame timestamp.
    /// 帧时间戳
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    /// OpenCV格式相机内参矩阵
    cv::Mat mK;
    /// Eigen格式相机内参矩阵
    Eigen::Matrix3f mK_;
    /// 静态相机内参fx
    static float fx;
    /// 静态相机内参fy
    static float fy;
    /// 静态相机主点cx
    static float cx;
    /// 静态相机主点cy
    static float cy;
    /// 1/fx
    static float invfx;
    /// 1/fy
    static float invfy;
    /// OpenCV畸变系数
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    /// 双目基线 × fx
    float mbf;

    // Stereo baseline in meters.
    /// 双目基线，单位米
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    /// 远近点深度阈值，区分近点、远点地图点
    float mThDepth;

    // Number of KeyPoints.
    /// 本帧关键点总数量
    size_t N;

    // Vector of keypoints (original for visualization) and undistorted (actually
    // used by the system). In the stereo case, mvKeysUn is redundant as images
    // must be rectified. In the RGB‑D case, RGB images can be distorted.
    /// 原始关键点（带畸变）
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    /// 去畸变后的关键点，系统实际运算使用
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    /// 每个关键点关联的地图点，裸指针，nullptr代表无关联
    std::vector<MapPoint *> mvpMapPoints;
    // "Monocular" keypoints have a negative value.
    /// 每个关键点对应的右目u坐标，单目关键点存负数
    std::vector<float> mvuRight;
    /// 每个关键点对应的深度值
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    /// DBoW2词袋向量，BoW
    DBoW2::BowVector mBowVec;
    /// DBoW2特征向量，FeatureVector
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    /// 左目ORB描述子矩阵，一行对应一个关键点
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    // Flag to identify outlier associations.
    /// 标记关键点‑地图点关联是否为外点
    std::vector<bool> mvbOutlier;
    /// 本帧近点地图点计数
    int mnCloseMPs;

    // Keypoints are assigned to cells in a grid to reduce matching complexity
    // when projecting MapPoints.
    /// 网格每个像素宽度倒数
    static float mfGridElementWidthInv;
    /// 网格每个像素高度倒数
    static float mfGridElementHeightInv;
    /// 图像网格，每个格子存放落在该格子关键点索引
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    /// IMU预测阶段偏置
    IMU::Bias mPredBias;

    // IMU bias
    /// IMU实际偏置
    IMU::Bias mImuBias;

    // Imu calibration
    /// IMU标定参数
    IMU::Calib mImuCalib;

    // Imu preintegration from last keyframe
    /// IMU预积分对象，从上一关键帧到当前帧
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
    /// 预积分对应的上一个关键帧
    std::shared_ptr<KeyFrame> mpLastKeyFrame;

    // Pointer to previous frame
    /// 上一普通帧
    std::shared_ptr<Frame> mpPrevFrame;
    /// 帧间IMU预积分
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegratedFrame;

    // Current and Next Frame id.
    /// Frame全局ID计数器，静态变量，分配新帧ID
    static long unsigned int nNextId;
    /// 当前帧ID
    long unsigned int mnId;

    // Reference Keyframe.
    /// 参考关键帧
    std::shared_ptr<KeyFrame> mpReferenceKF;

    // Scale pyramid info.
    /// ORB金字塔总层数
    int mnScaleLevels;
    /// ORB金字塔缩放因子
    float mfScaleFactor;
    /// log(scaleFactor)
    float mfLogScaleFactor;
    /// 每层金字塔缩放系数
    vector<float> mvScaleFactors;
    /// 每层金字塔逆缩放系数
    vector<float> mvInvScaleFactors;
    /// 每层sigma平方
    vector<float> mvLevelSigma2;
    /// 每层逆sigma平方
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    /// 去畸变后图像边界最小X
    static float mnMinX;
    /// 去畸变后图像边界最大X
    static float mnMaxX;
    /// 去畸变后图像边界最小Y
    static float mnMinY;
    /// 去畸变后图像边界最大Y
    static float mnMaxY;

    /// 是否已经完成静态初始化（图像边界、网格参数）
    static bool mbInitialComputations;

    /// 地图点ID到投影像素点缓存
    map<long unsigned int, cv::Point2f> mmProjectPoints;
    /// 地图点ID到图像匹配点缓存
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

    /// 文件名，数据集读取时使用
    string mNameFile;
    /// 数据集编号
    int mnDataset;

#ifdef REGISTER_TIMES
    /// ORB特征提取耗时
    double mTimeORB_Ext;
    /// 双目匹配耗时
    double mTimeStereoMatch;
#endif

private:
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB‑D case. Stereo must be already rectified!
    // (called in the constructor).
    /**
     * @brief 对关键点做去畸变，生成mvKeysUn；双目已经校正不需要此步骤，RGB‑D会调用
     */
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the
    // constructor).
    /**
     * @brief 计算去畸变后图像边界mnMinX/mnMaxX/mnMinY/mnMaxY，构造函数调用
     * @param imLeft 输入左目图像
     */
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the
    // constructor).
    /**
     * @brief 将关键点分配到图像网格mGrid，加速局部特征搜索，构造函数调用
     */
    void AssignFeaturesToGrid();

    /// mbIsSet标记位姿是否被设置
    bool mbIsSet;
    /// IMU预积分完成标记
    bool mbImuPreintegrated;

    /// IMU相关成员互斥锁，多线程访问保护
    std::shared_ptr<std::mutex> mpMutexImu;

public:
    /// 主相机模型
    std::shared_ptr<GeometricCamera> mpCamera, mpCamera2;

    // Number of KeyPoints extracted in the left and right images
    /// 左目关键点数量
    int Nleft, Nright;
    // Number of Non Lapping Keypoints
    /// 非重叠区域左目关键点数目
    int monoLeft, monoRight;

    // For stereo matching
    /// 左关键点到右关键点匹配索引
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    // For stereo fisheye matching
    /// 鱼眼双目匹配暴力匹配器，静态
    static cv::BFMatcher BFmatcher;

    // Triangulated stereo observations using as reference the left camera. These
    // are computed during ComputeStereoFishEyeMatches
    /// 鱼眼双目三角化得到三维点，Eigen对齐分配器
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
        mvStereo3Dpoints;

    // Grid for the right image
    /// 右目图像网格，用于右目关键点快速区域查找
    std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    /**
     * @brief 鱼眼双目Frame构造函数，支持两个不同相机模型，输入左右相机外参Tlr
     * @param imLeft 左图
     * @param imRight 右图
     * @param timeStamp 时间戳
     * @param extractorLeft 左提取器
     * @param extractorRight 右提取器
     * @param voc ORB词袋
     * @param K 内参cv::Mat
     * @param distCoef 畸变系数
     * @param bf bf参数
     * @param thDepth 深度阈值
     * @param pCamera 左相机模型
     * @param pCamera2 右相机模型
     * @param Tlr 左到右相机外参
     * @param pPrevF 上一帧
     * @param ImuCalib IMU标定
     */
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
          const std::shared_ptr<ORBextractor> &extractorLeft,
          const std::shared_ptr<ORBextractor> &extractorRight,
          const std::shared_ptr<ORBVocabulary> &voc, cv::Mat &K,
          cv::Mat &distCoef, const float &bf, const float &thDepth,
          const std::shared_ptr<GeometricCamera> &pCamera,
          const std::shared_ptr<GeometricCamera> &pCamera2, Sophus::SE3f &Tlr,
          const std::shared_ptr<Frame> &pPrevF = nullptr,
          const IMU::Calib &ImuCalib = IMU::Calib());

    // Stereo fisheye
    /**
     * @brief 鱼眼双目模式执行双目匹配与三角化
     */
    void ComputeStereoFishEyeMatches();

    /**
     * @brief 鱼眼版本视锥判断，可选择右相机
     * @param pMP 地图点
     * @param viewingCosLimit 视角余弦阈值
     * @param bRight true使用右相机；false使用主相机
     * @return true点在视锥内
     */
    bool isInFrustumChecks(MapPoint *pMP, float viewingCosLimit,
                           bool bRight = false);

    /**
     * @brief 鱼眼双目关键点反投影，得到世界三维点
     * @param i 关键点索引
     * @return Eigen::Vector3f 世界坐标系三维点
     */
    Eigen::Vector3f UnprojectStereoFishEye(const int &i);

    /// 左目图像
    cv::Mat imgLeft, imgRight;

    /**
     * @brief 调试打印帧内左右图像有效地图点分布，输出debug日志
     */
    void PrintPointDistribution() {
        int left = 0, right = 0;
        const size_t Nlim = (Nleft != -1) ? Nleft : N;
        for (size_t i = 0; i < N; i++) {
            if (mvpMapPoints[i] && !mvbOutlier[i]) {
                if (i < Nlim)
                    left++;
                else
                    right++;
            }
        }
        oslog::debug("Point distribution in Frame: left-> {} --- right-> {}", left,
                     right);
    }

    /// 测试用SE3位姿变量，调试使用
    Sophus::SE3<double> T_test;
};

}  // namespace ORB_SLAM3
