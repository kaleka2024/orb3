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
#include <pangolin/pangolin.h>                         // pangolin可视化库，用于地图界面渲染
#include <boost/serialization/base_object.hpp>         // boost序列化基类支持
#include <boost/serialization/set.hpp>                 // boost对std::set容器序列化支持
#include <boost/serialization/shared_ptr.hpp>          // boost对shared_ptr智能指针序列化支持
#include <cstdint>                                     // 固定宽度整数类型头文件
#include <list>                                        // std::list双向链表容器
#include <memory>                                      // std::shared_ptr智能指针
#include <mutex>                                       // std::mutex互斥锁，多线程访问地图数据同步
#include <set>                                         // std::set有序集合，存储地图点、关键帧
#include <string>                                      // std::string字符串，日志、保存路径
#include <vector>                                      // std::vector动态数组容器

#include "KeyFrame.h"   // 关键帧类定义
#include "MapPoint.h"   // 地图点类定义

namespace ORB_SLAM3 {

class MapPoint;       // 地图点类前向声明
class KeyFrame;       // 关键帧类前向声明
class Atlas;          // 多地图集Atlas前向声明，管理多个Map子地图
class KeyFrameDatabase; // 关键帧词袋数据库前向声明

/**
 * @brief 单个子地图类，Atlas可以持有多个Map对象；继承enable_shared_from_this获取自身shared_ptr
 * @details 保存本地图内全部关键帧、地图点；维护地图状态、IMU初始化状态、地图变更索引；支持boost序列化保存加载地图
 */
class Map : public std::enable_shared_from_this<Map>
{
    friend class boost::serialization::access; // 友元，允许boost序列化访问私有成员

    /**
     * @brief boost序列化模板函数，完成地图对象保存与加载
     * @param ar 序列化归档对象
     * @param version 序列化版本号
     * @note ubuntu16.04 boost1.58版本set序列化存在bug，改用vector做备份存储
     */
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        // Save/load a set structure, the set structure is broken in libboost 1.58
        // for ubuntu 16.04, a vector is serializated
        ar & mspKeyFrames;
        ar & mspMapPoints;
        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;
    }
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类中包含Eigen类型成员必须添加

    Map();                           // 默认构造函数
    explicit Map(int initKFid);      // 带初始关键帧ID的构造函数，explicit禁止隐式转换

    ~Map();                          // 析构函数

    /**
     * @brief 向地图中添加一个关键帧
     * @param pKF 待加入的关键帧共享指针
     */
    void AddKeyFrame(const std::shared_ptr<KeyFrame>& pKF);

    /**
     * @brief 向地图添加地图点
     * @param pMP 待加入地图点裸指针
     */
    void AddMapPoint(MapPoint* pMP);

    /**
     * @brief 从地图擦除一个地图点
     * @param pMP 需要删除的地图点裸指针
     */
    void EraseMapPoint(MapPoint* pMP);

    /**
     * @brief 从地图擦除一个关键帧
     * @param pKF 需要删除的关键帧共享指针
     */
    void EraseKeyFrame(const std::shared_ptr<KeyFrame>& pKF);

    /**
     * @brief 设置参考地图点，用于可视化显示
     * @param vpMPs 参考地图点数组
     */
    void SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);

    /**
     * @brief 通知地图发生重大变更(回环、全局BA)，更新大变更索引
     */
    void InformNewBigChange();

    /**
     * @brief 获取上一次重大变更索引
     * @return 大变更索引整数值
     */
    int GetLastBigChangeIdx();

    /**
     * @brief 获取地图内全部关键帧，set转vector返回副本
     * @return 地图所有关键帧vector容器
     */
    std::vector<std::shared_ptr<KeyFrame> > GetAllKeyFrames();

    /**
     * @brief 获取地图内全部地图点，set转vector返回副本
     * @return 地图所有地图点vector容器
     */
    std::vector<MapPoint*> GetAllMapPoints();

    /**
     * @brief 获取参考地图点集合，用于可视化渲染
     * @return 参考地图点vector
     */
    std::vector<MapPoint*> GetReferenceMapPoints();

    /**
     * @brief 获取当前地图内地图点总数量
     * @return 地图点数目
     */
    unsigned long int MapPointsInMap();

    /**
     * @brief 获取当前地图内关键帧总数量
     * @return 关键帧数目
     */
    long unsigned KeyFramesInMap();

    /**
     * @brief 获取当前Map子地图唯一ID
     * @return 地图ID
     */
    unsigned long int GetId();

    /**
     * @brief 获取本地图初始关键帧ID
     * @return 初始关键帧ID
     */
    unsigned long int GetInitKFid();

    /**
     * @brief 设置本地图初始关键帧ID
     * @param initKFif 初始关键帧ID
     */
    void SetInitKFid(unsigned long int initKFif);

    /**
     * @brief 获取本地图最大关键帧ID
     * @return 地图内最大关键帧ID
     */
    unsigned long int GetMaxKFid();

    /**
     * @brief 获取地图原点起始关键帧
     * @return 原点关键帧共享指针
     */
    std::shared_ptr<KeyFrame> GetOriginKF();

    /**
     * @brief 将本地图设置为Atlas当前激活正在使用的地图
     */
    void SetCurrentMap();

    /**
     * @brief 将本地图设置为已存储的休眠地图(多地图模式)
     */
    void SetStoredMap();

    /**
     * @brief 判断地图是否存在缩略图(可视化截图)
     * @return true存在缩略图
     */
    bool HasThumbnail();

    /**
     * @brief 判断该地图是否正在被系统使用
     * @return true正在使用
     */
    bool IsInUse();

    /**
     * @brief 将地图标记为坏地图，该地图将被丢弃
     */
    void SetBad();

    /**
     * @brief 查询地图是否为坏地图
     * @return true为坏地图
     */
    bool IsBad();

    /**
     * @brief 清空地图，清除所有关键帧、地图点数据
     */
    void clear();

    /**
     * @brief 获取地图普通变更索引，局部BA、增删地图点会递增
     * @return 地图变更索引
     */
    int GetMapChangeIndex();

    /**
     * @brief 增加地图普通变更索引
     */
    void IncreaseChangeIndex();

    /**
     * @brief 获取上一次通知外部的地图变更ID
     * @return 上一次通知变更id
     */
    int GetLastMapChange();

    /**
     * @brief 设置上一次通知外部的地图变更ID
     * @param currentChangeId 变更id
     */
    void SetLastMapChange(int currentChangeId);

    /**
     * @brief 设置IMU已经完成初始化
     */
    void SetImuInitialized();

    /**
     * @brief 查询IMU是否已经初始化完成
     * @return true IMU初始化完毕
     */
    bool isImuInitialized();

    /**
     * @brief 对整个地图应用缩放+旋转变换，单目IMU地图重置尺度时调用
     * @param T SE3位姿变换
     * @param s 缩放尺度因子
     * @param bScaledVel 是否同步缩放IMU速度
     */
    void ApplyScaledRotation(const Sophus::SE3f& T, const float s,
                            const bool bScaledVel = false);

    /**
     * @brief 设置该地图为惯性模式(带IMU)
     */
    void SetInertialSensor();

    /**
     * @brief 查询地图是否为惯性IMU模式
     * @return true使用IMU
     */
    bool IsInertial();

    /**
     * @brief 标记完成第一阶段IMU‑BA
     */
    void SetInertialBA1();

    /**
     * @brief 标记完成第二阶段IMU‑BA
     */
    void SetInertialBA2();

    /**
     * @brief 获取IMU第一阶段BA标记
     * @return true已执行IMU‑BA1
     */
    bool GetInertialBA1();

    /**
     * @brief 获取IMU第二阶段BA标记
     * @return true已执行IMU‑BA2
     */
    bool GetInertialBA2();

    /**
     * @brief 打印本质图连接信息，调试接口
     */
    void PrintEssentialGraph();

    /**
     * @brief 检查本质图的合法性，调试校验接口
     * @return true本质图合法
     */
    bool CheckEssentialGraph();

    /**
     * @brief 修改地图ID，加载地图备份时使用
     * @param nId 新地图ID
     */
    void ChangeId(unsigned long int nId);

    /**
     * @brief 获取地图内ID最小的关键帧ID
     * @return 最小关键帧ID
     */
    unsigned int GetLowerKFID();

    /**
     * @brief 地图保存前预处理，收集相机对象，序列化前调用
     * @param spCams 输出集合，收集本地图用到的相机
     */
    void PreSave(std::set<std::shared_ptr<GeometricCamera> >& spCams);

    /**
     * @brief 地图从磁盘加载完成后后置处理，重建关键帧数据库、词袋关联、相机映射
     * @param pKFDB 全局关键帧数据库指针
     * @param pORBVoc ORB词袋字典
     * @param mpCams 加载得到的相机ID‑相机对象映射
     */
    void PostLoad(
        const std::shared_ptr<KeyFrameDatabase>& pKFDB,
        const std::shared_ptr<ORBVocabulary>&           pORBVoc /*, map<unsigned long int, KeyFrame*>& mpKeyFrameId*/,
        map<unsigned int, std::shared_ptr<GeometricCamera> >& mpCams);

    /**
     * @brief 打印重投影误差，调试工具函数，输出局部窗口关键帧误差
     * @param lpLocalWindowKFs 局部窗口关键帧列表
     * @param mpCurrentKF 当前关键帧
     * @param name 输出文件名
     * @param name_folder 输出文件夹路径
     */
    void printReprojectionError(
        list<std::shared_ptr<KeyFrame> >& lpLocalWindowKFs,
        const std::shared_ptr<KeyFrame>& mpCurrentKF, string& name,
        string& name_folder);

    vector<std::shared_ptr<KeyFrame> > mvpKeyFrameOrigins;   // 地图起源关键帧集合，多地图模式
    vector<unsigned long int> mvBackupKeyFrameOriginsId;     // 序列化备份起源关键帧ID
    std::shared_ptr<KeyFrame> mpFirstRegionKF;               // 该地图第一个区域关键帧
    std::mutex mMutexMapUpdate;                              // 地图更新互斥锁，保护地图修改操作

    // This avoid that two points are created simultaneously in separate threads
    // (id conflict)
    std::mutex mMutexPointCreation;                          // 地图点创建互斥锁，防止多线程生成地图点ID冲突

    bool mbFail;                                             // 地图失败标记

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;                      // 可视化缩略图宽度，2的幂
    static const int THUMB_HEIGHT = 512;                     // 可视化缩略图高度，2的幂

    static unsigned long int nNextId;                        // 静态全局自增地图ID，新建Map分配id

    // DEBUG: show KFs which are used in LBA
    std::set<unsigned long int> msOptKFs;                   // 调试：局部BA优化的关键帧ID集合
    std::set<unsigned long int> msFixedKFs;                  // 调试：局部BA固定不动的关键帧ID集合

protected:
    unsigned long int mnId;                                  // 当前子地图实例ID

    std::set<MapPoint*> mspMapPoints;                        // 本地图全部地图点集合
    std::set<std::shared_ptr<KeyFrame> > mspKeyFrames;       // 本地图全部关键帧集合

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a
    // vector is serializated
    std::vector<MapPoint*> mvpBackupMapPoints;               // 序列化备份地图点vector，规避boost set bug
    std::vector<std::shared_ptr<KeyFrame> > mvpBackupKeyFrames; // 序列化备份关键帧vector

    std::shared_ptr<KeyFrame> mpKFinitial;                   // 地图初始关键帧
    std::shared_ptr<KeyFrame> mpKFlowerID;                   // 地图ID最小的关键帧

    long int mnBackupKFinitialID;                            // 序列化备份初始关键帧ID
    long int mnBackupKFlowerID;                              // 序列化备份最小ID关键帧ID

    std::vector<MapPoint*> mvpReferenceMapPoints;            // 参考地图点，可视化渲染使用

    bool mbImuInitialized;                                   // IMU是否初始化完成标记

    int mnMapChange;                                         // 地图变更计数器，每次增删点/帧+1
    int mnMapChangeNotified;                                 // 已经通知外部的地图变更计数

    unsigned long int mnInitKFid;                            // 本地图初始关键帧ID
    unsigned long int mnMaxKFid;                             // 本地图最大关键帧ID
    // unsigned long int mnLastLoopKFid;                      // 上一次回环关键帧ID，注释未使用

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;                                      // 地图重大变更索引(回环、全局BA)

    bool mIsInUse;                                           // 地图是否正在被系统使用
    bool mHasTumbnail;                                       // 是否存在可视化缩略图
    bool mbBad = false;                                      // 地图是否标记为坏地图，需要丢弃

    bool mbIsInertial;                                       // 该地图是否启用IMU惯性模式
    bool mbIMU_BA1;                                          // IMU第一阶段BA完成标记
    bool mbIMU_BA2;                                          // IMU第二阶段BA完成标记

    // Mutex
    std::mutex mMutexMap;                                    // 保护地图核心数据成员的互斥锁
};

} // namespace ORB_SLAM3
