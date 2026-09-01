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
#pragma once // 头文件保护，防止重复包含引发编译重定义
#include <boost/serialization/array.hpp>        // boost序列化数组支持
#include <boost/serialization/map.hpp>          // boost对std::map容器序列化支持
#include <boost/serialization/serialization.hpp>// boost序列化基础头
#include <boost/serialization/shared_ptr.hpp>   // boost对shared_ptr智能指针序列化支持
#include <map>                                  // std::map关联容器，存储观测关系
#include <memory>                               // std::shared_ptr智能指针
#include <mutex>                                // std::mutex互斥锁，多线程数据保护
#include <opencv2/core/core.hpp>                // OpenCV核心模块，cv::Mat描述子
#include <set>                                  // std::set集合容器
#include <tuple>                                // std::tuple元组，保存特征点索引

#include "Converter.h"          // 类型转换工具，Eigen与OpenCV互相转换
#include "Frame.h"              // 普通帧Frame类定义
#include "KeyFrame.h"           // 关键帧KeyFrame类定义
#include "Map.h"                // 子地图Map类定义
#include "SerializationUtils.h"// 序列化工具函数，矩阵序列化

namespace ORB_SLAM3 {

class KeyFrame; // 关键帧前向声明
class Map;      // 子地图前向声明
class Frame;    // 普通帧前向声明

/**
 * @brief 3D地图点类，代表空间中一个路标点
 * @details 保存世界坐标、观测关系、ORB描述子、法向、尺度距离；
 * 被Tracking、LocalMapping、LoopClosing、Merge模块读写；支持boost序列化保存加载；
 * 多线程访问内部成员需要加对应互斥锁
 */
class MapPoint {
    friend class boost::serialization::access; // 友元，允许boost序列化访问私有成员
    /**
     * @brief boost序列化模板函数，完成MapPoint对象保存与磁盘加载
     * @param ar 序列化归档对象
     * @param version 序列化版本号
     */
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & mnId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;

        // Variables used by the tracking
        ar & mTrackProjX;
        ar & mTrackProjY;
        ar & mTrackDepth;
        ar & mTrackDepthR;
        ar & mTrackProjXR;
        ar & mTrackProjYR;
        ar & mbTrackInView;
        ar & mbTrackInViewR;
        ar & mnTrackScaleLevel;
        ar & mnTrackScaleLevelR;
        ar & mTrackViewCos;
        ar & mTrackViewCosR;
        ar & mnTrackReferenceForFrame;
        ar & mnLastFrameSeen;

        // Variables used by local mapping
        ar & mnBALocalForKF;
        ar & mnFuseCandidateForKF;

        // Variables used by loop closing and merging
        ar & mnLoopPointForKF;
        ar & mnCorrectedByKF;
        ar & mnCorrectedReference;
        ar& boost::serialization::make_array(mPosGBA.data(), mPosGBA.size());
        // serializeMatrix(ar,mPosGBA,version);
        ar & mnBAGlobalForKF;
        ar & mnBALocalForMerge;
        ar& boost::serialization::make_array(mPosMerge.data(), mPosMerge.size());
        ar& boost::serialization::make_array(mNormalVectorMerge.data(),
                                             mNormalVectorMerge.size());
        // serializeMatrix(ar,mPosMerge,version);
        // serializeMatrix(ar,mNormalVectorMerge,version);

        // Protected variables
        ar& boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
        ar& boost::serialization::make_array(mNormalVector.data(),
                                             mNormalVector.size());
        // ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);

        // ar & mObservations;

        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar, mDescriptor, version);
        ar & mBackupRefKFId;
        ar & mnVisible;
        ar & mnFound;

        ar & mbBad;
        ar & mBackupReplacedId;

        ar & mfMinDistance;
        ar & mfMaxDistance;
    }
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类含有Eigen向量成员必须添加

    MapPoint(); // 默认构造函数

    /**
     * @brief 由世界坐标、参考关键帧构造地图点
     * @param Pos 地图点世界三维坐标
     * @param pRefKF 参考关键帧
     * @param pMap 所属子地图
     */
    MapPoint(const Eigen::Vector3f& Pos, const std::shared_ptr<KeyFrame>& pRefKF,
             const std::shared_ptr<Map>& pMap);

    /**
     * @brief 逆深度模式构造地图点，用于单目初始化、逆深度优化
     * @param invDepth 逆深度
     * @param uv_init 图像上2D像素坐标
     * @param pRefKF 参考关键帧
     * @param pHostKF 宿主关键帧
     * @param pMap 所属子地图
     */
    MapPoint(const double invDepth, cv::Point2f uv_init,
             const std::shared_ptr<KeyFrame>& pRefKF,
             const std::shared_ptr<KeyFrame>& pHostKF,
             const std::shared_ptr<Map>& pMap);

    /**
     * @brief 由普通Frame生成地图点
     * @param Pos 世界三维坐标
     * @param pMap 所属子地图
     * @param pFrame 普通帧对象
     * @param idxF Frame内特征点索引
     */
    MapPoint(const Eigen::Vector3f& Pos, const std::shared_ptr<Map>& pMap,
             const std::shared_ptr<Frame>& pFrame, const int& idxF);

    /**
     * @brief 设置地图点世界坐标，加锁保护
     * @param Pos 三维世界坐标
     */
    void SetWorldPos(const Eigen::Vector3f& Pos);

    /**
     * @brief 获取地图点世界坐标，加锁读取
     * @return Eigen::Vector3f 世界坐标
     */
    Eigen::Vector3f GetWorldPos();

    /**
     * @brief 获取该地图点平均观测方向(法向量)
     * @return 观测方向单位向量
     */
    Eigen::Vector3f GetNormal();

    /**
     * @brief 设置观测法向向量
     * @param normal 观测方向向量
     */
    void SetNormalVector(const Eigen::Vector3f& normal);

    /**
     * @brief 获取该地图点的参考关键帧
     * @return 参考关键帧shared_ptr
     */
    std::shared_ptr<KeyFrame> GetReferenceKeyFrame();

    /**
     * @brief 获取全部观测关系map：key=关键帧，tuple<左图特征索引,右图特征索引>
     * @return mObservations完整拷贝
     */
    std::map<std::shared_ptr<KeyFrame>, std::tuple<int, int>> GetObservations();

    /**
     * @brief 返回观测数量nObs，即能看到该点的关键帧总数
     * @return 观测计数
     */
    int Observations();

    /**
     * @brief 添加一条观测：某关键帧看到该地图点，记录特征点索引
     * @param pKF 观测该点的关键帧
     * @param idx 该关键帧里特征点索引
     */
    void AddObservation(const std::shared_ptr<KeyFrame>& pKF, int idx);

    /**
     * @brief 删除一条观测关系，该关键帧不再观测此地图点
     * @param pKF 待移除的关键帧
     */
    void EraseObservation(const std::shared_ptr<KeyFrame>& pKF);

    /**
     * @brief 获取该地图点在指定关键帧中的特征索引tuple(左索引,右索引)
     * @param pKF 查询的关键帧
     * @return std::tuple<int,int> 左右特征索引
     */
    std::tuple<int, int> GetIndexInKeyFrame(const std::shared_ptr<KeyFrame>& pKF);

    /**
     * @brief 判断该关键帧是否观测到当前地图点
     * @param pKF 目标关键帧
     * @return true存在观测
     */
    bool IsInKeyFrame(const std::shared_ptr<KeyFrame>& pKF);

    /**
     * @brief 将地图点标记为坏点，不再参与匹配与BA；不会直接释放内存
     */
    void SetBadFlag();

    /**
     * @brief 判断地图点是否为坏点
     * @return true为坏点
     */
    bool isBad();

    /**
     * @brief 用另一个地图点替换当前点，用于地图点融合合并
     * @param pMP 用来替换的目标地图点
     */
    void Replace(MapPoint* pMP);

    /**
     * @brief 获取替换当前点的那个地图点
     * @return 被替换指向的MapPoint裸指针
     */
    MapPoint* GetReplaced();

    /**
     * @brief 增加可视计数：该点投影到帧内、在视野范围内
     * @param n 增加数量，默认+1
     */
    void IncreaseVisible(int n = 1);

    /**
     * @brief 增加找到计数：该点成功完成特征匹配
     * @param n 增加数量，默认+1
     */
    void IncreaseFound(int n = 1);

    /**
     * @brief 获取匹配成功率 found / visible，用于筛选可靠地图点
     * @return 比例浮点数
     */
    float GetFoundRatio();

    /**
     * @brief 获取成功匹配计数mnFound
     * @return int mnFound
     */
    inline int GetFound() { return mnFound; }

    /**
     * @brief 从所有观测的描述子中选出最具有区分度的描述子，作为该地图点代表描述子
     */
    void ComputeDistinctiveDescriptors();

    /**
     * @brief 获取地图点代表ORB描述子
     * @return cv::Mat 描述子
     */
    cv::Mat GetDescriptor();

    /**
     * @brief 更新平均观测法向量NormalVector，同时更新最小最大观测距离mfMinDistance、mfMaxDistance
     */
    void UpdateNormalAndDepth();

    /**
     * @brief 获取该地图点不变尺度的最小观测距离
     * @return float 最小距离
     */
    float GetMinDistanceInvariance();

    /**
     * @brief 获取该地图点不变尺度的最大观测距离
     * @return float 最大距离
     */
    float GetMaxDistanceInvariance();

    /**
     * @brief 根据当前观测距离预测该地图点应当落在图像金字塔哪一层，输入关键帧
     * @param currentDist 当前相机到地图点距离
     * @param pKF 参考关键帧
     * @return 金字塔层级
     */
    int PredictScale(const float& currentDist,
                     const std::shared_ptr<KeyFrame>& pKF);

    /**
     * @brief 根据当前观测距离预测该地图点应当落在图像金字塔哪一层，输入普通帧Frame
     * @param currentDist 当前相机到地图点距离
     * @param pF 普通帧
     * @return 金字塔层级
     */
    int PredictScale(const float& currentDist, const std::shared_ptr<Frame>& pF);

    /**
     * @brief 获取该地图点所属子地图
     * @return 所属Map共享指针
     */
    std::shared_ptr<Map> GetMap();

    /**
     * @brief 更新地图点所属子地图，多地图合并时调用
     * @param pMap 新的子地图
     */
    void UpdateMap(const std::shared_ptr<Map>& pMap);

    /**
     * @brief 打印该地图点全部观测关系，调试接口
     */
    void PrintObservations();

    /**
     * @brief 序列化保存前预处理，把智能指针转为ID备份，解决指针序列化问题
     * @param spKF 全局关键帧集合
     * @param spMP 全局地图点集合
     */
    void PreSave(set<std::shared_ptr<KeyFrame>>& spKF, set<MapPoint*>& spMP);

    /**
     * @brief 从磁盘加载后后置处理，通过ID恢复智能指针观测关系
     * @param mpKFid 关键帧ID到对象映射
     * @param mpMPid 地图点ID到对象映射
     */
    void PostLoad(map<long unsigned int, std::shared_ptr<KeyFrame>>& mpKFid,
                  map<long unsigned int, MapPoint*>& mpMPid);

public:
    long unsigned int mnId;                     // 地图点全局唯一ID
    static long unsigned int nNextId;           // 静态全局自增ID，新建MapPoint分配mnId
    long int mnFirstKFid;                       // 生成该地图点的第一个关键帧ID
    long int mnFirstFrame;                      // 生成该地图点的第一个普通帧ID
    int nObs;                                   // 观测计数，能看到该点的关键帧数量

    // Variables used by the tracking
    float mTrackProjX;                          // Tracking阶段预测投影图像x坐标(左目)
    float mTrackProjY;                          // Tracking阶段预测投影图像y坐标(左目)
    float mTrackDepth;                          // Tracking阶段左目深度
    float mTrackDepthR;                         // Tracking阶段右目深度
    float mTrackProjXR;                         // Tracking阶段预测投影图像x坐标(右目)
    float mTrackProjYR;                         // Tracking阶段预测投影图像y坐标(右目)
    bool mbTrackInView, mbTrackInViewR;         // 左/右目：地图点投影是否落在图像范围内
    int mnTrackScaleLevel, mnTrackScaleLevelR;  // 左/右目预测金字塔层级
    float mTrackViewCos, mTrackViewCosR;        // 左/右目观测方向夹角余弦值
    long unsigned int mnTrackReferenceForFrame;  // 标记该地图点作为哪一帧的跟踪参考点
    long unsigned int mnLastFrameSeen;           // 上一次看到该地图点的帧ID

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;           // 标记该点被哪一个关键帧触发局部BA
    long unsigned int mnFuseCandidateForKF;     // 标记该点作为哪一帧的候选融合点

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;         // 回环检测：标记该点属于哪个回环候选关键帧
    long unsigned int mnCorrectedByKF;          // 回环：被哪个关键帧校正过
    long unsigned int mnCorrectedReference;      // 回环：校正时参考的关键帧ID
    Eigen::Vector3f mPosGBA;                    // 全局BA过程中临时保存的地图点位置
    long unsigned int mnBAGlobalForKF;          // 触发全局BA的关键帧ID
    long unsigned int mnBALocalForMerge;        // 地图合并时局部BA标记

    // Variable used by merging
    Eigen::Vector3f mPosMerge;                  // 地图合并过程临时坐标
    Eigen::Vector3f mNormalVectorMerge;         // 地图合并过程临时法向量

    // Fopr inverse depth optimization
    double mInvDepth;                           // 逆深度，逆深度优化模式使用
    double mInitU;                               // 初始化像素u
    double mInitV;                               // 初始化像素v
    std::shared_ptr<KeyFrame> mpHostKF;         // 逆深度模式宿主关键帧

    static std::mutex mGlobalMutex;              // 全局静态互斥锁，地图点全局操作使用

    unsigned int mnOriginMapId;                 // 该地图点原始所属地图ID

protected:
    // Position in absolute coordinates
    Eigen::Vector3f mWorldPos;                   // 地图点世界三维坐标

    // Keyframes observing the point and associated index in keyframe
    std::map<std::shared_ptr<KeyFrame>, std::tuple<int, int>> mObservations; // 观测关系：关键帧->(左特征索引,右特征索引)
    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupObservationsId1; // 序列化备份观测ID1
    std::map<long unsigned int, int> mBackupObservationsId2; // 序列化备份观测ID2

    // Mean viewing direction
    Eigen::Vector3f mNormalVector;               // 平均观测方向单位法向量

    // Best descriptor to fast matching
    cv::Mat mDescriptor;                        // 该地图点最优代表性ORB描述子

    // Reference KeyFrame
    std::shared_ptr<KeyFrame> mpRefKF;          // 参考关键帧，该点主要生成来源
    long unsigned int mBackupRefKFId;           // 序列化备份参考关键帧ID

    // Tracking counters
    int mnVisible;                               // 可视计数：投影到帧视野内
    int mnFound;                                 // 匹配成功计数：实际匹配上特征

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;                                  // 坏点标记，true代表该点失效
    MapPoint* mpReplaced;                       // 指向替换自己的地图点，被融合时使用
    // For save relation without pointer, this is necessary for save/load function
    long long int mBackupReplacedId;            // 序列化备份被替换地图点ID

    // Scale invariance distances
    float mfMinDistance;                         // 该地图点观测最小距离(对应金字塔最高层)
    float mfMaxDistance;                         // 该地图点观测最大距离(对应金字塔最底层)

    std::shared_ptr<Map> mpMap;                  // 该地图点所属子地图

    // Mutex
    std::mutex mMutexPos;                        // 保护世界坐标mWorldPos互斥锁
    std::mutex mMutexFeatures;                   // 保护观测、描述子、法向量互斥锁
    std::mutex mMutexMap;                       // 保护所属地图mpMap互斥锁
};

} // namespace ORB_SLAM3
