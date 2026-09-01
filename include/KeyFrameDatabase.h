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
#pragma once // 头文件保护，防止重复包含造成重定义编译错误
#include <boost/serialization/base_object.hpp> // boost序列化基类支持头文件
#include <boost/serialization/list.hpp>         // boost对std::list容器的序列化支持
#include <boost/serialization/vector.hpp>       // boost对std::vector容器的序列化支持
#include <list>                                 // std::list双向链表容器，倒排索引存储关键帧
#include <memory>                               // std::shared_ptr智能指针
#include <mutex>                                // std::mutex互斥锁，多线程访问数据库同步
#include <set>                                  // std::set有序集合容器
#include <vector>                               // std::vector动态数组容器

#include "Frame.h"                              // 普通帧Frame类定义
#include "KeyFrame.h"                           // 关键帧KeyFrame类定义
#include "Map.h"                                // 地图Map类定义
#include "ORBVocabulary.h"                      // ORB词袋字典类定义

namespace ORB_SLAM3 {

class KeyFrame; // 关键帧类前向声明
class Frame;    // 普通帧类前向声明
class Map;      // 地图类前向声明

/**
 * @brief 关键帧数据库类
 * @details 基于DBoW2词袋倒排文件，用于回环检测、地图融合、重定位，根据词袋向量快速检索候选关键帧
 */
class KeyFrameDatabase {
    friend class boost::serialization::access; // boost序列化友元，允许访问私有成员完成序列化

    /**
     * @brief boost序列化函数，对象磁盘保存/加载
     * @param ar 序列化归档对象
     * @param version 序列化版本号
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & mvBackupInvertedFileId; // 序列化倒排文件ID备份容器，不直接序列化智能指针
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Eigen内存对齐宏，类内存在Eigen相关类型必须添加

    KeyFrameDatabase() = default; // 默认构造函数，序列化加载对象时使用
    /**
     * @brief 带参构造，绑定ORB词袋字典
     * @param voc 共享指针形式的ORB词袋字典
     */
    explicit KeyFrameDatabase(const std::shared_ptr<ORBVocabulary> &voc);

    /**
     * @brief 将关键帧加入数据库，更新倒排索引
     * @param pKF 待加入的关键帧智能指针
     */
    void add(const std::shared_ptr<KeyFrame> &pKF);

    /**
     * @brief 从数据库中删除指定关键帧，清理倒排索引内该关键帧记录
     * @param pKF 需要移除的关键帧智能指针
     */
    void erase(const std::shared_ptr<KeyFrame> &pKF);

    /**
     * @brief 清空整个数据库，清空全部倒排索引
     */
    void clear();

    /**
     * @brief 清空指定地图对应的所有关键帧记录，多地图模式使用
     * @param pMap 指定地图的智能指针
     */
    void clearMap(const std::shared_ptr<Map> &pMap);

    // Loop Detection(DEPRECATED) 回环检测（已废弃，旧版本接口保留）
    /**
     * @brief 【废弃接口】检测回环候选关键帧
     * @param pKF 当前查询关键帧
     * @param minScore 词袋相似度最低阈值
     * @return 回环候选关键帧vector
     */
    std::vector<std::shared_ptr<KeyFrame>> DetectLoopCandidates(
        const std::shared_ptr<KeyFrame> &pKF, float minScore);

    // Loop and Merge Detection 回环与地图融合检测
    /**
     * @brief 同时检测回环候选与地图融合候选关键帧，按最小得分筛选
     * @param pKF 查询用的当前关键帧
     * @param minScore 词袋相似度最低得分阈值
     * @param vpLoopCand 输出参数，回环候选关键帧数组
     * @param vpMergeCand 输出参数，地图融合候选关键帧数组
     */
    void DetectCandidates(const std::shared_ptr<KeyFrame> &pKF, float minScore,
                          vector<std::shared_ptr<KeyFrame>> &vpLoopCand,
                          vector<std::shared_ptr<KeyFrame>> &vpMergeCand);

    /**
     * @brief 检测最优回环、融合候选，设置共同单词最小数量约束
     * @param pKF 查询关键帧
     * @param vpLoopCand 输出回环候选
     * @param vpMergeCand 输出融合候选
     * @param nMinWords 两个关键帧之间必须匹配的最少共同单词数量
     */
    void DetectBestCandidates(const std::shared_ptr<KeyFrame> &pKF,
                              vector<std::shared_ptr<KeyFrame>> &vpLoopCand,
                              vector<std::shared_ptr<KeyFrame>> &vpMergeCand,
                              int nMinWords);

    /**
     * @brief 检索得分最高的N个回环候选、N个融合候选
     * @param pKF 查询关键帧
     * @param vpLoopCand 输出回环候选
     * @param vpMergeCand 输出融合候选
     * @param nNumCandidates 需要获取候选帧的最大数量
     */
    void DetectNBestCandidates(const std::shared_ptr<KeyFrame> &pKF,
                               vector<std::shared_ptr<KeyFrame>> &vpLoopCand,
                               vector<std::shared_ptr<KeyFrame>> &vpMergeCand,
                               size_t nNumCandidates);

    // Relocalization 重定位
    /**
     * @brief 根据普通帧Frame检测重定位候选关键帧
     * @param F 当前普通帧，用于词袋匹配检索
     * @param pMap 在该指定地图范围内检索候选
     * @return 重定位候选关键帧数组
     */
    std::vector<std::shared_ptr<KeyFrame>> DetectRelocalizationCandidates(
        const std::shared_ptr<Frame> &F, const std::shared_ptr<Map> &pMap);

    /**
     * @brief 序列化保存前预处理，把智能指针转换为ID备份，避免直接序列化指针
     */
    void PreSave();

    /**
     * @brief 序列化加载完成后后处理，使用备份ID还原关键帧智能指针倒排索引
     * @param mpKFid map<关键帧ID，关键帧共享指针>，全局ID到关键帧映射表
     */
    void PostLoad(map<long unsigned int, std::shared_ptr<KeyFrame>> mpKFid);

    /**
     * @brief 设置ORB词袋字典
     * @param pORBVoc ORB词袋字典共享指针
     */
    void SetORBVocabulary(const std::shared_ptr<ORBVocabulary> &pORBVoc);

protected:
    // Associated vocabulary 关联的ORB词袋字典
    std::shared_ptr<ORBVocabulary> mpVoc;

    // Inverted file DBoW2倒排文件：数组下标对应单词ID，每个单词保存出现过该单词的关键帧链表
    std::vector<list<std::shared_ptr<KeyFrame>>> mvInvertedFile;

    // For save relation without pointer, this is necessary for save/load function
    // 序列化备份倒排文件：存储关键帧ID，磁盘保存不能直接存shared_ptr指针，加载后PostLoad恢复
    std::vector<list<long unsigned int>> mvBackupInvertedFileId;

    // Mutex 互斥锁，add/erase/查询候选多线程并发访问数据库时做同步保护
    std::mutex mMutex;
};

} // namespace ORB_SLAM3
