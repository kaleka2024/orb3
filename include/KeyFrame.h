/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
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
#pragma once // 头文件保护，防止多次include引发重定义
#include <boost/serialization/base_object.hpp>    // boost序列化基类支持
#include <boost/serialization/map.hpp>             // boost对std::map容器序列化支持
#include <boost/serialization/shared_ptr.hpp>      // boost对std::shared_ptr智能指针序列化
#include <boost/serialization/vector.hpp>          // boost对std::vector容器序列化
#include <cstdint>                                 // 固定宽度整数类型头文件
#include <iostream>                                 // C++标准输入输出流，调试打印
#include <map>                                      // std::map有序映射容器
#include <memory>                                   // std::shared_ptr、std::enable_shared_from_this智能指针
#include <mutex>                                    // std::mutex互斥锁，多线程同步保护
#include <set>                                      // std::set有序集合容器
#include <string>                                   // std::string字符串
#include <vector>                                   // std::vector动态数组容器

#include "Frame.h"                                  // 普通帧Frame类定义
#include "GeometricCamera.h"                        // 相机模型基类，针孔/鱼眼相机
#include "ImuTypes.h"                               // IMU相关数据结构、预积分定义
#include "KeyFrameDatabase.h"                       // 关键帧数据库，用于回环检测、重定位
#include "MapPoint.h"                               // 地图点MapPoint类
#include "ORBVocabulary.h"                          // ORB词袋字典
#include "ORBextractor.h"                           // ORB特征提取器
#include "SerializationUtils.h"                     // ORB‑SLAM3自定义序列化工具函数
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"       // DBoW2词袋向量BowVector
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"   // DBoW2特征向量FeatureVector

namespace ORB_SLAM3 {

class Map;               // 地图类前向声明
class MapPoint;          // 地图点前向声明
class Frame;             // 普通帧前向声明
class KeyFrameDatabase;  // 关键帧数据库前向声明

class GeometricCamera;   // 几何相机模型前向声明

// 关键帧类：继承enable_shared_from_this，允许类内部获取自身shared_ptr，SLAM地图核心单元
class KeyFrame : public std::enable_shared_from_this<KeyFrame> {
    friend class boost::serialization::access; // boost序列化友元，允许访问私有成员做序列化

    // boost序列化模板函数，实现KeyFrame对象磁盘保存/加载
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & mnId;                                      // 序列化关键帧唯一ID
        ar& const_cast<unsigned long int&>(mnFrameId);   // 序列化对应的普通帧ID，const成员强转供boost序列化
        ar& const_cast<double&>(mTimeStamp);             // 序列化时间戳
        // Grid 网格加速匹配相关常量
        ar& const_cast<int&>(mnGridCols);
        ar& const_cast<int&>(mnGridRows);
        ar& const_cast<float&>(mfGridElementWidthInv);
        ar& const_cast<float&>(mfGridElementHeightInv);

        // Variables of tracking 跟踪模块标记变量，代码注释掉不序列化
        // ar & mnTrackReferenceForFrame;
        // ar & mnFuseTargetForKF;
        // Variables of local mapping 局部建图模块标记变量，不序列化
        // ar & mnBALocalForKF;
        // ar & mnBAFixedForKF;
        // ar & mnNumberOfOpt;
        // Variables used by KeyFrameDatabase 关键帧数据库标记变量，不序列化
        // ar & mnLoopQuery;
        // ar & mnLoopWords;
        // ar & mLoopScore;
        // ar & mnRelocQuery;
        // ar & mnRelocWords;
        // ar & mRelocScore;
        // ar & mnMergeQuery;
        // ar & mnMergeWords;
        // ar & mMergeScore;
        // ar & mnPlaceRecognitionQuery;
        // ar & mnPlaceRecognitionWords;
        // ar & mPlaceRecognitionScore;
        // ar & mbCurrentPlaceRecognition;
        // Variables of loop closing 回环闭合GBA相关变量，不序列化
        // serializeMatrix(ar,mTcwGBA,version);
        // serializeMatrix(ar,mTcwBefGBA,version);
        // serializeMatrix(ar,mVwbGBA,version);
        // serializeMatrix(ar,mVwbBefGBA,version);
        // ar & mBiasGBA;
        // ar & mnBAGlobalForKF;
        // Variables of Merging 地图融合相关变量，不序列化
        // serializeMatrix(ar,mTcwMerge,version);
        // serializeMatrix(ar,mTcwBefMerge,version);
        // serializeMatrix(ar,mTwcBefMerge,version);
        // serializeMatrix(ar,mVwbMerge,version);
        // serializeMatrix(ar,mVwbBefMerge,version);
        // ar & mBiasMerge;
        // ar & mnMergeCorrectedForKF;
        // ar & mnMergeForKF;
        // ar & mfScaleMerge;
        // ar & mnBALocalForMerge;

        // Scale 尺度因子
        ar & mfScale;
        // Calibration parameters 相机内参，const成员强转序列化
        ar& const_cast<float&>(fx);
        ar& const_cast<float&>(fy);
        ar& const_cast<float&>(invfx);
        ar& const_cast<float&>(invfy);
        ar& const_cast<float&>(cx);
        ar& const_cast<float&>(cy);
        ar& const_cast<float&>(mbf);
        ar& const_cast<float&>(mb);
        ar& const_cast<float&>(mThDepth);
        serializeMatrix(ar, mDistCoef, version); // 畸变系数cv::Mat序列化
        // Number of Keypoints 特征点数量
        ar& const_cast<int&>(N);
        // KeyPoints 关键点、去畸变关键点
        serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
        serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
        ar& const_cast<vector<float>&>(mvuRight); // 右目视差，单目为负数
        ar& const_cast<vector<float>&>(mvDepth);  // 深度，单目为负数
        serializeMatrix<Archive>(ar, mDescriptors, version); // ORB描述子矩阵序列化
        // BOW 词袋向量与特征向量
        ar & mBowVec;
        ar & mFeatVec;
        // Pose relative to parent 相对父关键帧位姿
        serializeSophusSE3<Archive>(ar, mTcp, version);
        // Scale 图像金字塔参数
        ar& const_cast<int&>(mnScaleLevels);
        ar& const_cast<float&>(mfScaleFactor);
        ar& const_cast<float&>(mfLogScaleFactor);
        ar& const_cast<vector<float>&>(mvScaleFactors);
        ar& const_cast<vector<float>&>(mvLevelSigma2);
        ar& const_cast<vector<float>&>(mvInvLevelSigma2);
        // Image bounds and calibration 图像边界
        ar& const_cast<int&>(mnMinX);
        ar& const_cast<int&>(mnMinY);
        ar& const_cast<int&>(mnMaxX);
        ar& const_cast<int&>(mnMaxY);
        ar& boost::serialization::make_array(mK_.data(), mK_.size()); // 相机内参Eigen矩阵数组序列化
        // Pose 相机世界位姿Tcw
        serializeSophusSE3<Archive>(ar, mTcw, version);
        // MapPointsId associated to keypoints 备份地图点ID，序列化不用裸指针
        ar & mvBackupMapPointsId;
        // Grid 特征匹配网格
        ar & mGrid;
        // Connected KeyFrameWeight 共视关键帧ID‑权重备份
        ar & mBackupConnectedKeyFrameIdWeights;
        // Spanning Tree and Loop Edges 生成树、回环边、融合边备份ID
        ar & mbFirstConnection;
        ar & mBackupParentId;
        ar & mvBackupChildrensId;
        ar & mvBackupLoopEdgesId;
        ar & mvBackupMergeEdgesId;
        // Bad flags 坏帧标记
        ar & mbNotErase;
        ar & mbToBeErased;
        ar & mbBad;

        ar & mHalfBaseline; // 半基线，可视化使用

        ar & mnOriginMapId; // 所属地图ID

        // Camera variables 双目/鱼眼双相机备份ID
        ar & mnBackupIdCamera;
        ar & mnBackupIdCamera2;

        // Fisheye variables 鱼眼双目匹配索引、右目关键点、右目网格
        ar & mvLeftToRightMatch;
        ar & mvRightToLeftMatch;
        ar& const_cast<int&>(NLeft);
        ar& const_cast<int&>(NRight);
        serializeSophusSE3<Archive>(ar, mTlr, version);
        serializeVectorKeyPoints<Archive>(ar, mvKeysRight, version);
        ar & mGridRight;

        // Inertial variables IMU相关：bias、预积分、标定、前后关键帧备份ID、速度、IMU启用标记
        ar & mImuBias;
        ar & mpBackupImuPreintegrated;
        ar & mImuCalib;
        ar & mBackupPrevKFId;
        ar & mBackupNextKFId;
        ar & bImu;
        ar& boost::serialization::make_array(mVw.data(), mVw.size());
        ar& boost::serialization::make_array(mOwb.data(), mOwb.size());
        ar & mbHasVelocity;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类含Sophus/Eigen固定尺寸成员必须添加

    KeyFrame() = default; // 默认构造函数，序列化加载时使用
    // 由普通Frame构造关键帧，传入地图、关键帧数据库
    KeyFrame(const std::shared_ptr<Frame>& F, const std::shared_ptr<Map>& pMap,
             const std::shared_ptr<KeyFrameDatabase>& pKFDB);

    // Pose functions 位姿相关接口
    void SetPose(const Sophus::SE3f& Tcw);          // 设置相机到世界变换Tcw
    void SetVelocity(const Eigen::Vector3f& Vw_);   // 设置IMU世界坐标系下速度Vw

    Sophus::SE3f GetPose();                         // 获取Tcw：相机到世界位姿
    Sophus::SE3f GetPoseInverse();                  // 获取Twc：世界到相机位姿
    Eigen::Vector3f GetCameraCenter();              // 获取相机光心在世界坐标系坐标

    Eigen::Vector3f GetImuPosition();               // 获取IMU在世界坐标系位置
    Eigen::Matrix3f GetImuRotation();               // 获取IMU在世界坐标系旋转矩阵
    Sophus::SE3f GetImuPose();                      // 获取IMU的SE3世界位姿
    Eigen::Matrix3f GetRotation();                  // 获取相机旋转Rcw
    Eigen::Vector3f GetTranslation();               // 获取相机平移tcw
    Eigen::Vector3f GetVelocity();                  // 获取IMU世界速度
    bool isVelocitySet();                           // 判断是否已经设置速度

    // Bag of Words Representation 词袋计算
    void ComputeBoW();                              // 根据关键点描述子计算BowVec与FeatVec，用于回环/重定位

    // Covisibility graph functions 共视图相关接口
    void AddConnection(const std::shared_ptr<KeyFrame>& pKF, const int& weight); // 添加共视连接，weight为共视地图点数量
    void EraseConnection(const std::shared_ptr<KeyFrame>& pKF);                  // 删除一条共视连接

    void UpdateConnections(bool upParent = true);                                // 更新全部共视关系，可选更新生成树父节点
    void UpdateBestCovisibles();                                                // 按权重排序共视关键帧
    std::set<std::shared_ptr<KeyFrame>> GetConnectedKeyFrames();                // 获取全部共视关键帧集合
    std::vector<std::shared_ptr<KeyFrame>> GetVectorCovisibleKeyFrames();        // 获取共视关键帧vector
    std::vector<std::shared_ptr<KeyFrame>> GetBestCovisibilityKeyFrames(int N);  // 获取权重最高的N个共视关键帧
    std::vector<std::shared_ptr<KeyFrame>> GetCovisiblesByWeight(int w);         // 获取权重大于等于w的共视关键帧
    int GetWeight(const std::shared_ptr<KeyFrame>& pKF);                         // 获取与指定关键帧之间的共视权重

    // Spanning tree functions 生成树接口
    void AddChild(const std::shared_ptr<KeyFrame>& pKF);                         // 添加子关键帧
    void EraseChild(const std::shared_ptr<KeyFrame>& pKF);                       // 删除子关键帧
    void ChangeParent(const std::shared_ptr<KeyFrame>& pKF);                      // 修改父关键帧
    std::set<std::shared_ptr<KeyFrame>> GetChilds();                             // 获取所有子关键帧集合
    std::shared_ptr<KeyFrame> GetParent();                                        // 获取父关键帧
    bool hasChild(const std::shared_ptr<KeyFrame>& pKF);                         // 判断pKF是否为本帧子节点
    void SetFirstConnection(bool bFirst);                                         // 设置mbFirstConnection标记，用于生成树初始化

    // Loop Edges 回环边接口
    void AddLoopEdge(const std::shared_ptr<KeyFrame>& pKF);                       // 添加回环边
    std::set<std::shared_ptr<KeyFrame>> GetLoopEdges();                           // 获取所有回环边关键帧集合

    // Merge Edges 地图融合边接口
    void AddMergeEdge(const std::shared_ptr<KeyFrame>& pKF);                      // 添加融合边
    set<std::shared_ptr<KeyFrame>> GetMergeEdges();                               // 获取全部融合边关键帧集合

    // MapPoint observation functions 地图点观测相关接口
    int GetNumberMPs();                                                           // 获取关联的有效地图点数量
    void AddMapPoint(MapPoint* pMP, const size_t& idx);                           // 将地图点pMP关联到第idx号关键点
    void EraseMapPointMatch(const int& idx);                                      // 清除第idx关键点关联的地图点
    void EraseMapPointMatch(MapPoint* pMP);                                       // 清除指定地图点在此关键帧上的关联
    void ReplaceMapPointMatch(const int& idx, MapPoint* pMP);                     // 替换idx关键点对应的地图点
    std::set<MapPoint*> GetMapPoints();                                           // 获取本关键帧观测的全部地图点集合
    std::vector<MapPoint*> GetMapPointMatches();                                   // 获取关键点索引对应的地图点数组
    int TrackedMapPoints(const int& minObs);                                       // 返回观测数>=minObs的有效地图点数目
    MapPoint* GetMapPoint(const size_t& idx);                                      // 获取第idx关键点关联的地图点

    // KeyPoint functions 关键点相关接口
    std::vector<size_t> GetFeaturesInArea(const float& x, const float& y,
                                          const float& r,
                                          const bool bRight = false) const;       // 在图像(x,y)半径r范围内查找关键点索引，bRight控制取右目图像
    bool UnprojectStereo(int i, Eigen::Vector3f& x3D);                             // 立体反投影，用第i个关键点恢复3D世界点

    // Image 图像边界判断
    bool IsInImage(const float& x, const float& y) const;                          // 判断像素坐标(x,y)是否落在图像内

    // Enable/Disable bad flag changes 坏帧保护标记
    void SetNotErase();                                                            // 设置mbNotErase，禁止本关键帧被删除
    void SetErase();                                                               // 取消mbNotErase，允许被删除

    // Set/check bad flag 坏帧标记
    void SetBadFlag();                                                             // 设置本关键帧为坏帧，不再参与优化
    bool isBad();                                                                  // 判断是否是坏关键帧

    // Compute Scene Depth (q=2 median). Used in monocular. 计算场景中值深度，单目初始化使用，q为取第几分位数
    float ComputeSceneMedianDepth(const int q);

    // 静态比较函数：共视权重降序排序用，a>b返回true
    static bool weightComp(int a, int b) { return a > b; }

    // 静态比较函数：按关键帧mnId升序排序
    static bool lId(const std::shared_ptr<KeyFrame>& pKF1,
                   const std::shared_ptr<KeyFrame>& pKF2) {
        return pKF1->mnId < pKF2->mnId;
    }

    std::shared_ptr<Map> GetMap();                          // 获取所属地图
    void UpdateMap(const std::shared_ptr<Map>& pMap);        // 更新所属地图指针

    void SetNewBias(const IMU::Bias& b);                    // 设置IMU bias
    Eigen::Vector3f GetGyroBias();                          // 获取陀螺仪bias
    Eigen::Vector3f GetAccBias();                           // 获取加速度计bias
    IMU::Bias GetImuBias();                                 // 获取完整IMU bias结构体

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v);    // 投影地图点到图像，输出带畸变像素坐标
    bool ProjectPointUnDistort(MapPoint* pMP, cv::Point2f& kp, float& u,
                               float& v);                                             // 投影地图点到图像，输出去畸变像素坐标

    // 序列化保存前预处理：收集需要序列化的关键帧、地图点、相机，把裸指针转为ID备份
    void PreSave(set<std::shared_ptr<KeyFrame>>& spKF, set<MapPoint*>& spMP,
                set<std::shared_ptr<GeometricCamera>>& spCam);
    // 序列化加载后后处理：通过备份ID还原shared_ptr裸指针关系
    void PostLoad(map<unsigned long int, std::shared_ptr<KeyFrame>>& mpKFid,
                 map<unsigned long int, MapPoint*>& mpMPid,
                 map<unsigned int, std::shared_ptr<GeometricCamera>>& mpCamId);

    void SetORBVocabulary(const std::shared_ptr<ORBVocabulary>& pORBVoc);    // 设置ORB词袋字典
    void SetKeyFrameDatabase(const std::shared_ptr<KeyFrameDatabase>& pKFDB); // 设置关键帧数据库

    bool bImu; // 是否启用IMU，true代表VIO模式

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
    // 下面成员仅单线程访问或初始化后不再修改，不需要互斥锁
public:
    static unsigned long int nNextId;       // 静态全局，下一个关键帧分配ID
    unsigned long int mnId;                 // 当前关键帧唯一ID
    unsigned long int mnFrameId;            // 由哪个普通Frame生成，对应Frame的ID

    double mTimeStamp;                      // 关键帧时间戳

    // Grid (to speed up feature matching) 图像网格，加速特征匹配
    int mnGridCols;                         // 网格列数
    int mnGridRows;                         // 网格行数
    float mfGridElementWidthInv;            // 每个网格宽度倒数
    float mfGridElementHeightInv;           // 每个网格高度倒数

    // Variables used by the tracking 跟踪模块标记变量
    unsigned long int mnTrackReferenceForFrame; // 作为哪一帧的跟踪参考关键帧
    unsigned long int mnFuseTargetForKF;        // 地图点融合目标标记

    // Variables used by the local mapping 局部建图标记变量
    unsigned long int mnBALocalForKF;           // 局部BA归属标记
    unsigned long int mnBAFixedForKF;           // 局部BA中固定位姿标记

    // Number of optimizations by BA(amount of iterations in BA) BA优化迭代计数
    unsigned long int mnNumberOfOpt;

    // Variables used by the keyframe database 关键帧数据库查询标记
    unsigned long int mnLoopQuery;              // 回环查询标记
    int mnLoopWords;                            // 回环匹配共同单词数
    float mLoopScore;                           // 回环词袋得分
    unsigned long int mnRelocQuery;             // 重定位查询标记
    int mnRelocWords;                           // 重定位共同单词数
    float mRelocScore;                          // 重定位词袋得分
    unsigned long int mnMergeQuery;             // 地图融合查询标记
    int mnMergeWords;                           // 融合共同单词数
    float mMergeScore;                          // 融合词袋得分
    unsigned long int mnPlaceRecognitionQuery;  // 地点识别查询标记
    int mnPlaceRecognitionWords;                // 地点识别共同单词数
    float mPlaceRecognitionScore;               // 地点识别得分

    bool mbCurrentPlaceRecognition;             // 当前是否正在执行地点识别

    // Variables used by loop closing 回环闭合GBA全局BA变量
    Sophus::SE3f mTcwGBA;                       // GBA优化后的Tcw
    Sophus::SE3f mTcwBefGBA;                    // GBA优化前Tcw
    Eigen::Vector3f mVwbGBA;                    // GBA优化后IMU世界速度
    Eigen::Vector3f mVwbBefGBA;                 // GBA优化前IMU世界速度
    IMU::Bias mBiasGBA;                         // GBA优化后的IMU bias
    unsigned long int mnBAGlobalForKF;          // GBA归属标记

    // Variables used by merging 多地图融合相关变量
    Sophus::SE3f mTcwMerge;                     // 融合校正后Tcw
    Sophus::SE3f mTcwBefMerge;                  // 融合校正前Tcw
    Sophus::SE3f mTwcBefMerge;                  // 融合校正前Twc
    Eigen::Vector3f mVwbMerge;                  // 融合校正后速度
    Eigen::Vector3f mVwbBefMerge;               // 融合校正前速度
    IMU::Bias mBiasMerge;                       // 融合校正后bias
    unsigned long int mnMergeCorrectedForKF;    // 融合校正标记
    unsigned long int mnMergeForKF;             // 融合任务标记
    float mfScaleMerge;                         // 融合尺度因子
    unsigned long int mnBALocalForMerge;        // 融合触发局部BA标记

    float mfScale; // 尺度，单目/IMU模式使用

    // Calibration parameters 相机内参
    float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    cv::Mat mDistCoef; // OpenCV畸变系数

    // Number of KeyPoints 提取的ORB关键点总数量
    int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    // 关键点、立体观测、描述子，数组下标一一对应
    std::vector<cv::KeyPoint> mvKeys;      // 原始关键点（带畸变）
    std::vector<cv::KeyPoint> mvKeysUn;    // 去畸变之后关键点
    std::vector<float> mvuRight;           // 右目视差，单目存储负数
    std::vector<float> mvDepth;            // 深度值，单目存储负数
    cv::Mat mDescriptors;                  // ORB描述子矩阵 N×32

    // BoW DBoW2词袋输出
    DBoW2::BowVector mBowVec;     // 词袋向量，单词‑权重
    DBoW2::FeatureVector mFeatVec;// 特征向量，单词‑特征索引

    // Pose relative to parent (this is computed when bad flag is activated)
    // 相对生成树父关键帧的位姿，标记坏帧时用来保留拓扑
    Sophus::SE3f mTcp;

    // Scale 图像金字塔参数
    int mnScaleLevels;                  // 金字塔层数
    float mfScaleFactor;                // 金字塔缩放因子
    float mfLogScaleFactor;             // log(scaleFactor)
    std::vector<float> mvScaleFactors;  // 每层金字塔缩放系数
    std::vector<float> mvLevelSigma2;   // 每层sigma平方
    std::vector<float> mvInvLevelSigma2;// 每层sigma平方倒数

    // Image bounds and calibration 图像边界，去畸变后图像有效范围
    int mnMinX;
    int mnMinY;
    int mnMaxX;
    int mnMaxY;

    // Preintegrated IMU measurements from previous keyframe
    // IMU预积分：和前后相邻关键帧关系
    std::shared_ptr<KeyFrame> mPrevKF;              // 上一个关键帧
    std::shared_ptr<KeyFrame> mNextKF;              // 下一个关键帧

    std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated; // 从上一关键帧到本帧IMU预积分
    IMU::Calib mImuCalib;                                    // IMU‑相机标定参数

    unsigned int mnOriginMapId; // 该关键帧最初所属地图ID

    string mNameFile;           // 数据集文件名，调试用
    int mnDataset;              // 数据集编号

    std::vector<std::shared_ptr<KeyFrame>> mvpLoopCandKFs;  // 回环候选关键帧
    std::vector<std::shared_ptr<KeyFrame>> mvpMergeCandKFs;// 地图融合候选关键帧

    // bool mbHasHessian;
    // cv::Mat mHessianPose;

    // The following variables need to be accessed trough a mutex to be thread safe.
    // 下面成员多线程并发读写，访问时必须加互斥锁
protected:
    // sophus poses SE3位姿
    Sophus::SE3<float> mTcw;    // Tcw：相机到世界变换
    Eigen::Matrix3f mRcw;       // Rcw旋转矩阵缓存
    Sophus::SE3<float> mTwc;    // Twc：世界到相机变换
    Eigen::Matrix3f mRwc;       // Rwc旋转矩阵缓存

    // IMU position IMU在世界坐标系下位置
    Eigen::Vector3f mOwb;
    // Velocity (Only used for inertial SLAM) IMU世界坐标系速度，VIO模式有效
    Eigen::Vector3f mVw;
    bool mbHasVelocity;         // 速度是否有效标记

    // Transformation matrix between cameras in stereo fisheye 鱼眼双目左右相机外参
    Sophus::SE3<float> mTlr;    // T_lr 左相机到右相机
    Sophus::SE3<float> mTrl;    // T_rl 右相机到左相机

    // Imu bias 当前关键帧IMU bias状态
    IMU::Bias mImuBias;

    // MapPoints associated to keypoints 关键点对应的地图点指针数组，下标和关键点一一对应
    std::vector<MapPoint*> mvpMapPoints;
    // For save relation without pointer, this is necessary for save/load function
    // 序列化备份：不存裸指针，存储地图点ID，加载后恢复指针
    std::vector<long long int> mvBackupMapPointsId;

    // BoW 词袋相关指针
    std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
    std::shared_ptr<ORBVocabulary> mpORBvocabulary;

    // Grid over the image to speed up feature matching 图像网格，存储每个网格内关键点索引，加速匹配
    std::vector<std::vector<std::vector<size_t>>> mGrid;

    std::map<std::shared_ptr<KeyFrame>, int> mConnectedKeyFrameWeights; // <共视关键帧，共视权重>
    std::vector<std::shared_ptr<KeyFrame>> mvpOrderedConnectedKeyFrames; // 按权重降序排序共视关键帧
    std::vector<int> mvOrderedWeights;                                   // 对应排序后的权重
    // For save relation without pointer, this is necessary for save/load function
    // 序列化备份：ID‑权重，加载重建mConnectedKeyFrameWeights
    std::map<unsigned long int, int> mBackupConnectedKeyFrameIdWeights;

    // Spanning Tree and Loop Edges 生成树、回环边、融合边
    bool mbFirstConnection;                                  // 是否第一次建立生成树连接
    std::shared_ptr<KeyFrame> mpParent;                      // 生成树父关键帧
    std::set<std::shared_ptr<KeyFrame>> mspChildrens;        // 生成树子关键帧集合
    std::set<std::shared_ptr<KeyFrame>> mspLoopEdges;        // 回环边关键帧集合
    std::set<std::shared_ptr<KeyFrame>> mspMergeEdges;       // 地图融合边关键帧集合
    // For save relation without pointer, this is necessary for save/load function
    // 序列化备份ID，加载恢复智能指针
    long long int mBackupParentId;
    std::vector<unsigned long int> mvBackupChildrensId;
    std::vector<unsigned long int> mvBackupLoopEdgesId;
    std::vector<unsigned long int> mvBackupMergeEdgesId;

    // Bad flags 坏帧标记
    bool mbNotErase;       // 禁止删除标记
    bool mbToBeErased;     // 待删除标记
    bool mbBad;            // 坏帧标记，true代表该关键帧失效

    float mHalfBaseline;   // 半基线，可视化使用

    std::shared_ptr<Map> mpMap; // 所属地图智能指针

    // Backup variables for inertial IMU序列化备份ID与预积分对象
    long long int mBackupPrevKFId;
    long long int mBackupNextKFId;
    std::shared_ptr<IMU::Preintegrated> mpBackupImuPreintegrated;

    // Backup for Cameras 相机序列化备份ID
    int mnBackupIdCamera, mnBackupIdCamera2;

    // Calibration Eigen格式相机内参矩阵K
    Eigen::Matrix3f mK_;

    // Mutex 多线程互斥锁
    std::mutex mMutexPose;      // 保护位姿、速度、IMU bias读写
    std::mutex mMutexConnections;// 保护共视图、生成树、回环边连接关系
    std::mutex mMutexFeatures;  // 保护关键点、地图点观测关联
    std::mutex mMutexMap;       // 保护地图指针访问

public:
    std::shared_ptr<GeometricCamera> mpCamera, mpCamera2; // 主相机、第二相机(双目/鱼眼)

    // Indexes of stereo observations correspondences 鱼眼双目左右关键点匹配索引
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    Sophus::SE3f GetRelativePoseTrl(); // 获取T_rl：右相机到左相机位姿
    Sophus::SE3f GetRelativePoseTlr(); // 获取T_lr：左相机到右相机位姿

    // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    // 鱼眼双目右图像关键点
    std::vector<cv::KeyPoint> mvKeysRight;

    int NLeft, NRight; // 左目关键点数量，右目关键点数量

    std::vector<std::vector<std::vector<size_t>>> mGridRight; // 右目图像匹配网格

    Sophus::SE3<float> GetRightPose();              // 获取右相机Tcw位姿
    Sophus::SE3<float> GetRightPoseInverse();       // 获取右相机Twc位姿
    Eigen::Vector3f GetRightCameraCenter();         // 获取右相机光心世界坐标
    Eigen::Matrix<float, 3, 3> GetRightRotation();  // 获取右相机Rcw
    Eigen::Vector3f GetRightTranslation();          // 获取右相机tcw

    // 调试打印：统计本关键帧左右目有效地图点分布数量
    void PrintPointDistribution() {
        int left = 0, right = 0;
        int Nlim = (NLeft != -1) ? NLeft : N;
        for (int i = 0; i < N; i++) {
            if (mvpMapPoints[i]) {
                if (i < Nlim)
                    left++;
                else
                    right++;
            }
        }
        oslog::debug("Point distribution in KeyFrame: left-> {} --- right-> {}",
                     left, right);
    }
};

}  // namespace ORB_SLAM3
