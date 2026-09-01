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

// boost序列化导出支持
#include <boost/serialization/export.hpp>
// boost对std::shared_ptr序列化支持
#include <boost/serialization/shared_ptr.hpp>
// boost对std::vector容器序列化支持
#include <boost/serialization/vector.hpp>
// C++智能指针
#include <memory>
// 互斥锁，Atlas多线程访问保护
#include <mutex>
// std::set集合容器
#include <set>
// std::vector动态数组容器
#include <vector>

// 相机几何基类
#include "GeometricCamera.h"
// Kannala‑Brandt8鱼眼相机模型
#include "KannalaBrandt8.h"
// 关键帧类
#include "KeyFrame.h"
// 单个地图类
#include "Map.h"
// 地图点类
#include "MapPoint.h"
// Pinhole针孔相机模型
#include "Pinhole.h"

namespace ORB_SLAM3 {

// 前向声明，减少头文件循环依赖
class Viewer;
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;

// BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")
// BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

/**
 * @brief 地图集Atlas，管理多个Map子地图，支持多地图切换、地图序列化、多相机管理
 * ORB‑SLAM3支持多地图模式，Atlas保存全部子地图集合，维护当前激活地图，负责地图保存加载
 */
class Atlas {
    // boost序列化友元，允许序列化私有成员
    friend class boost::serialization::access;

    /**
     * @brief boost序列化函数，完成Atlas存档与读档
     * @tparam Archive boost序列化归档类型
     * @param ar 归档对象引用
     * @param version 序列化版本号
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        // 注册派生相机类型，序列化识别Pinhole、KannalaBrandt8多态对象
        ar.template register_type<Pinhole>();
        ar.template register_type<KannalaBrandt8>();

        // Save/load a set structure, the set structure is broken in libboost 1.58
        // for ubuntu 16.04, a vector is serializated
        // ar & mspMaps;
        // 使用备份vector序列化地图集合，规避boost1.58 set序列化bug
        ar & mvpBackupMaps;
        // 序列化全部相机对象
        ar & mvpCameras;
        // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
        // 序列化各类全局静态ID计数器，保证加载后ID连续不冲突
        ar &Map::nNextId;
        ar &Frame::nNextId;
        ar &KeyFrame::nNextId;
        ar &MapPoint::nNextId;
        ar &GeometricCamera::nNextId;
        // 序列化上一次初始化地图的关键帧ID
        ar & mnLastInitKFidMap;
    }

public:
    // Eigen内存对齐宏，用于Eigen成员内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 默认构造函数
     */
    Atlas();

    /**
     * @brief 带初始关键帧ID的构造函数，系统初始化时创建第一个地图
     * @param initKFid 初始化参考关键帧ID
     */
    explicit Atlas(int initKFid);  // When its initialization the first map is created

    /**
     * @brief Atlas析构函数，释放地图、相机等资源
     */
    ~Atlas();

    /**
     * @brief 创建新的子地图，多地图模式下回环失败时新建地图使用
     */
    void CreateNewMap();

    /**
     * @brief 切换当前激活工作地图
     * @param pMap 需要切换成为当前地图的Map智能指针
     */
    void ChangeMap(const std::shared_ptr<Map> &pMap);

    /**
     * @brief 获取上一次地图初始化对应的关键帧ID
     * @return unsigned long int 关键帧ID
     */
    unsigned long int GetLastInitKFid();

    /**
     * @brief 设置可视化Viewer对象
     * @param pViewer 可视化模块智能指针
     */
    void SetViewer(const std::shared_ptr<Viewer> &pViewer);

    // Method for change components in the current map
    /**
     * @brief 向当前激活地图添加关键帧
     * @param pKF 待添加关键帧智能指针
     */
    void AddKeyFrame(const std::shared_ptr<KeyFrame> &pKF);

    /**
     * @brief 向当前激活地图添加地图点
     * @param pMP 待添加地图点裸指针
     */
    void AddMapPoint(MapPoint *pMP);

    // void EraseMapPoint(MapPoint* pMP);
    // void EraseKeyFrame(KeyFrame* pKF);

    /**
     * @brief 添加相机对象到Atlas，管理多相机，返回该相机智能指针
     * @param pCam 传入GeometricCamera相机智能指针
     * @return std::shared_ptr<GeometricCamera> 存入的相机智能指针
     */
    std::shared_ptr<GeometricCamera> AddCamera(const std::shared_ptr<GeometricCamera> &pCam);

    /**
     * @brief 获取Atlas管理的全部相机集合
     * @return std::vector<std::shared_ptr<GeometricCamera>> 全部相机vector
     */
    std::vector<std::shared_ptr<GeometricCamera>> GetAllCameras();

    /* All methods without Map pointer work on current map */
    /**
     * @brief 设置当前地图参考地图点，用于局部地图跟踪
     * @param vpMPs 地图点裸指针vector
     */
    void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

    /**
     * @brief 通知当前地图发生大规模变更（地图合并、回环），更新大变更索引
     */
    void InformNewBigChange();

    /**
     * @brief 获取当前地图最后一次大规模变更索引
     * @return int 变更序号
     */
    int GetLastBigChangeIdx();

    /**
     * @brief 获取当前地图内地图点总数量
     * @return long unsigned int 地图点数目
     */
    long unsigned int MapPointsInMap();

    /**
     * @brief 获取当前地图内关键帧总数量
     * @return long unsigned 关键帧数目
     */
    long unsigned KeyFramesInMap();

    // Method for get data in current map
    /**
     * @brief 获取当前地图全部关键帧
     * @return std::vector<std::shared_ptr<KeyFrame>> 关键帧智能指针数组
     */
    std::vector<std::shared_ptr<KeyFrame>> GetAllKeyFrames();

    /**
     * @brief 获取当前地图全部地图点
     * @return std::vector<MapPoint *> 地图点裸指针数组
     */
    std::vector<MapPoint *> GetAllMapPoints();

    /**
     * @brief 获取当前地图参考地图点集合
     * @return std::vector<MapPoint *> 参考地图点裸指针数组
     */
    std::vector<MapPoint *> GetReferenceMapPoints();

    /**
     * @brief 获取Atlas中保存的所有子地图
     * @return vector<std::shared_ptr<Map>> 全部地图智能指针vector
     */
    vector<std::shared_ptr<Map>> GetAllMaps();

    /**
     * @brief 统计Atlas内部子地图总个数
     * @return int 地图数量
     */
    int CountMaps();

    /**
     * @brief 清空当前激活地图的数据
     */
    void clearMap();

    /**
     * @brief 清空整个Atlas，清除全部地图、相机等数据
     */
    void clearAtlas();

    /**
     * @brief 获取当前正在工作的激活地图
     * @return std::shared_ptr<Map> 当前地图智能指针
     */
    std::shared_ptr<Map> GetCurrentMap();

    /**
     * @brief 将指定地图标记为坏地图，放入坏地图集合等待清理
     * @param pMap 需要标记为坏的地图
     */
    void SetMapBad(const std::shared_ptr<Map> &pMap);

    /**
     * @brief 删除所有标记为bad的子地图
     */
    void RemoveBadMaps();

    /**
     * @brief 判断Atlas是否存在惯性IMU传感器数据
     * @return true 启用IMU；false 纯视觉模式
     */
    bool isInertial();

    /**
     * @brief 设置Atlas开启惯性传感器模式
     */
    void SetInertialSensor();

    /**
     * @brief 设置IMU完成初始化
     */
    void SetImuInitialized();

    /**
     * @brief 判断IMU是否已经完成初始化
     * @return true IMU初始化完成
     */
    bool isImuInitialized();

    // Function for garantee the correction of serialization of this object
    /**
     * @brief 序列化保存前预处理函数，保存地图前调用，处理容器转换等
     */
    void PreSave();

    /**
     * @brief 从文件加载地图之后的后处理，修复容器、重建指针关联
     */
    void PostLoad();

    /**
     * @brief 获取Atlas下全部地图的所有关键帧，key为关键帧ID
     * @return map<long unsigned int, std::shared_ptr<KeyFrame>> ID到关键帧的映射
     */
    map<long unsigned int, std::shared_ptr<KeyFrame>> GetAtlasKeyframes();

    /**
     * @brief 设置关键帧数据库对象
     * @param pKFDB 关键帧数据库智能指针
     */
    void SetKeyFrameDababase(const std::shared_ptr<KeyFrameDatabase> &pKFDB);

    /**
     * @brief 获取关键帧数据库对象
     * @return std::shared_ptr<KeyFrameDatabase> 关键帧数据库智能指针
     */
    std::shared_ptr<KeyFrameDatabase> GetKeyFrameDatabase();

    /**
     * @brief 设置ORB词袋对象
     * @param pORBVoc ORB词袋智能指针
     */
    void SetORBVocabulary(const std::shared_ptr<ORBVocabulary> &pORBVoc);

    /**
     * @brief 获取ORB词袋对象
     * @return std::shared_ptr<ORBVocabulary> ORB词袋智能指针
     */
    std::shared_ptr<ORBVocabulary> GetORBVocabulary();

    /**
     * @brief 获取Atlas所有存活有效关键帧总数量
     * @return long unsigned int 存活关键帧数目
     */
    long unsigned int GetNumLivedKF();

    /**
     * @brief 获取Atlas所有存活有效地图点总数量
     * @return long unsigned int 存活地图点数目
     */
    long unsigned int GetNumLivedMP();

protected:
    /// 全部有效子地图集合
    std::set<std::shared_ptr<Map>> mspMaps;
    /// 标记为坏的子地图集合，等待清理
    std::set<std::shared_ptr<Map>> mspBadMaps;
    // Its necessary change the container from set to vector because libboost 1.58
    // and Ubuntu 16.04 have an error with this cointainer
    /// 序列化备份vector，boost1.58 set序列化存在bug，使用vector做存档载体
    std::vector<std::shared_ptr<Map>> mvpBackupMaps;

    /// 当前激活正在使用的地图
    std::shared_ptr<Map> mpCurrentMap;

    /// Atlas管理的全部相机对象集合，支持多相机
    std::vector<std::shared_ptr<GeometricCamera>> mvpCameras;

    /// 上一次地图初始化对应的关键帧ID
    unsigned long int mnLastInitKFidMap;

    /// 可视化Viewer模块智能指针
    std::shared_ptr<Viewer> mpViewer;
    // Class references for the map reconstruction from the save file
    /// 关键帧数据库，用于回环检测、重定位
    std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
    /// ORB词袋模型，用于词袋匹配
    std::shared_ptr<ORBVocabulary> mpORBVocabulary;

    // Mutex
    /// Atlas全局互斥锁，保护多线程并发读写Atlas成员
    std::mutex mMutexAtlas;
};  // class Atlas

}  // namespace ORB_SLAM3
