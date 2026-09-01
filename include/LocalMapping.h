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
#pragma once // 头文件保护，防止头文件重复包含引发编译重定义错误
#include <list>        // std::list双向链表，用于存储待处理新关键帧、新增地图点
#include <memory>      // std::shared_ptr智能指针，管理各类对象生命周期
#include <mutex>       // std::mutex互斥锁，多线程之间同步访问共享数据
#include <string>      // std::string字符串，序列名称、日志输出文件名
#include <vector>      // std::vector动态数组，计时统计数据存储

#include "Atlas.h"         // 多地图集Atlas头文件，管理全部子地图
#include "KeyFrame.h"      // 关键帧KeyFrame类定义
#include "KeyFrameDatabase.h" // 关键帧数据库，词袋检索
#include "LoopClosing.h"   // 回环检测与闭环修正模块
#include "Settings.h"      // 配置参数读取管理类
#include "Tracking.h"     // 跟踪线程模块头文件

namespace ORB_SLAM3 {

class System;    // 系统总入口类前向声明
class Tracking;  // 跟踪线程类前向声明
class LoopClosing; // 回环闭合线程前向声明
class Atlas;     // 地图集前向声明

/**
 * @brief 局部建图线程类
 * @details ORB‑SLAM3三大核心线程之一；接收Tracking输出的新关键帧，执行地图点生成、筛选、局部BA、关键帧剔除；IMU模式下完成IMU初始化与尺度优化
 */
class LocalMapping {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类内存在Eigen矩阵成员必须添加，避免内存崩溃

    /**
     * @brief 局部建图构造函数
     * @param pSys 系统总入口智能指针
     * @param pAtlas 全局地图集Atlas智能指针
     * @param bMonocular 是否单目模式，true=单目，false=双目/RGBD
     * @param bInertial 是否开启IMU惯性模式
     * @param _strSeqName 数据集序列名称，用于调试日志输出
     */
    LocalMapping(const std::shared_ptr<System> &pSys,
                 const std::shared_ptr<Atlas> &pAtlas, const float bMonocular,
                 bool bInertial, const string &_strSeqName = std::string());

    /**
     * @brief 设置回环闭合模块指针，局部建图处理完关键帧后交给回环线程
     * @param pLoopCloser 回环闭合对象共享指针
     */
    void SetLoopCloser(const std::shared_ptr<LoopClosing> &pLoopCloser);

    /**
     * @brief 设置跟踪模块指针，局部建图需要和跟踪线程交互同步状态
     * @param pTracker 跟踪线程对象共享指针
     */
    void SetTracker(const std::shared_ptr<Tracking> &pTracker);

    // Main function 线程主函数，局部建图线程启动后循环运行该函数
    void Run();

    /**
     * @brief 插入新关键帧到局部建图待处理队列，由Tracking线程调用
     * @param pKF 待加入队列的关键帧智能指针
     */
    void InsertKeyFrame(const std::shared_ptr<KeyFrame> &pKF);

    /**
     * @brief 清空新关键帧队列，重置、中断场景调用
     */
    void EmptyQueue();

    // Thread Synch 线程同步接口
    /**
     * @brief 请求停止局部建图线程，发出停止标记，非立刻终止
     */
    void RequestStop();

    /**
     * @brief 请求全局重置，清空全部地图与缓存数据
     */
    void RequestReset();

    /**
     * @brief 请求仅重置当前激活地图，多地图Atlas模式使用
     * @param pMap 需要重置的目标地图指针
     */
    void RequestResetActiveMap(const std::shared_ptr<Map> &pMap);

    /**
     * @brief 阻塞等待局部建图线程真正停止，返回停止状态
     * @return true代表线程已经成功停止
     */
    bool Stop();

    /**
     * @brief 释放线程相关资源
     */
    void Release();

    /**
     * @brief 查询线程是否已经处于停止状态
     * @return true已停止，false运行中
     */
    bool isStopped();

    /**
     * @brief 查询是否收到停止请求标记
     * @return true收到停止请求
     */
    bool stopRequested();

    /**
     * @brief 查询局部建图是否接收新关键帧
     * @return true允许接收，false拒绝接收Tracking发来的关键帧
     */
    bool AcceptKeyFrames();

    /**
     * @brief 设置是否接收新关键帧的开关
     * @param flag true接收；false拒绝
     */
    void SetAcceptKeyFrames(bool flag);

    /**
     * @brief 设置不允许线程停止标记，局部BA运算时防止被Stop打断
     * @param flag true禁止停止；false允许停止
     * @return 返回设置后的状态
     */
    bool SetNotStop(bool flag);

    /**
     * @brief 中断正在执行的局部BA优化，回环触发时调用
     */
    void InterruptBA();

    /**
     * @brief 请求线程正常结束，系统退出时调用
     */
    void RequestFinish();

    /**
     * @brief 查询局部建图线程是否已经执行完毕退出
     * @return true线程已结束
     */
    bool isFinished();

    /**
     * @brief 获取待处理队列里面关键帧的数量，inline内联函数
     * @return 队列中关键帧数目
     */
    int KeyframesInQueue() {
        unique_lock<std::mutex> lock(mMutexNewKFs); // 加锁保护共享队列mlNewKeyFrames
        return mlNewKeyFrames.size();
    }

    /**
     * @brief 判断当前是否正在做IMU初始化
     * @return true正在IMU初始化阶段
     */
    bool IsInitializing();

    /**
     * @brief 获取当前正在处理关键帧的时间戳
     * @return 时间戳double
     */
    double GetCurrKFTime();

    /**
     * @brief 获取局部建图当前处理的关键帧智能指针
     * @return 当前关键帧共享指针
     */
    std::shared_ptr<KeyFrame> GetCurrKF();

    std::mutex mMutexImuInit; // IMU初始化相关成员变量访问互斥锁

    Eigen::MatrixXd mcovInertial; // IMU初始化协方差矩阵
    Eigen::Matrix3d mRwg;         // IMU陀螺仪到世界坐标系旋转矩阵
    Eigen::Vector3d mbg;          // 陀螺仪bias
    Eigen::Vector3d mba;          // 加速度计bias
    double mScale;                // IMU初始化求解得到尺度因子
    double mInitTime;             // IMU初始化消耗时间
    double mCostTime;             // 单次优化耗时

    unsigned int mInitSect; // IMU初始化分段标记
    unsigned int mIdxInit;  // IMU初始化迭代轮次索引
    unsigned int mnKFs;     // IMU初始化使用关键帧数量
    double mFirstTs;        // IMU初始化第一帧时间戳
    int mnMatchesInliers;   // IMU初始化有效匹配内点数量

    // For debugging (erase in normal mode) 调试用变量，正式发布版本可删除
    int mInitFr;        // IMU初始化起始帧编号
    int mIdxIteration;  // 迭代次数索引
    string strSequence; // 当前数据集序列名字

    bool mbNotBA1; // 标记跳过第一阶段BA
    bool mbNotBA2; // 标记跳过第二阶段BA
    bool mbBadImu; // IMU初始化失败标记，IMU数据质量差

    bool mbWriteStats; // 是否输出统计日志文件开关

    // not consider far points (clouds) 不考虑远距离地图点（点云远景噪声点）
    bool mbFarPoints;  // 是否开启远距离点过滤
    float mThFarPoints;// 远距离点距离阈值

#ifdef REGISTER_TIMES // 编译宏，开启各步骤耗时统计，调试性能使用
    vector<double> vdKFInsert_ms;     // 关键帧插入耗时(ms)
    vector<double> vdMPCulling_ms;    // 地图点剔除耗时(ms)
    vector<double> vdMPCreation_ms;   // 新建地图点耗时(ms)
    vector<double> vdLBA_ms;          // 局部BA总耗时(ms)
    vector<double> vdKFCulling_ms;    // 关键帧剔除耗时(ms)
    vector<double> vdLMTotal_ms;      // 局部建图单次循环总耗时(ms)

    vector<double> vdLBASync_ms;      // 同步模式局部BA耗时
    vector<double> vdKFCullingSync_ms;// 同步模式关键帧剔除耗时
    vector<int> vnLBA_edges;          // 局部BA边数量
    vector<int> vnLBA_KFopt;          // 局部BA参与优化关键帧数量
    vector<int> vnLBA_KFfixed;        // 局部BA固定不动关键帧数量
    vector<int> vnLBA_MPs;            // 局部BA参与优化地图点数量
    int nLBA_exec;                    // 局部BA执行总次数
    int nLBA_abort;                   // 局部BA被中断放弃次数
#endif
protected:
    /**
     * @brief 检查待处理队列是否存在新关键帧
     * @return true队列有新关键帧；false队列为空
     */
    bool CheckNewKeyFrames();

    /**
     * @brief 处理队列取出的当前关键帧：词袋计算、地图点观测更新、剔除坏地图点
     */
    void ProcessNewKeyFrame();

    /**
     * @brief 三角化生成新地图点，对当前关键帧和共视关键帧未匹配特征做三角化
     */
    void CreateNewMapPoints();

    /**
     * @brief 地图点剔除，对刚生成的mlpRecentAddedMapPoints做质量筛选，删除质量差地图点
     */
    void MapPointCulling();

    /**
     * @brief 在共视邻居关键帧之间做特征搜索匹配，补充地图点观测关系
     */
    void SearchInNeighbors();

    /**
     * @brief 关键帧剔除，冗余关键帧判定删除，减少地图规模
     */
    void KeyFrameCulling();

    std::shared_ptr<System> mpSystem; // 系统总入口智能指针

    bool mbMonocular; // 是否单目模式 true=单目；false双目/RGBD
    bool mbInertial;  // 是否开启IMU惯性模式

    /**
     * @brief 如果收到重置请求，执行重置逻辑，清空队列、地图、缓存
     */
    void ResetIfRequested();
    bool mbResetRequested;             // 全局重置请求标记
    bool mbResetRequestedActiveMap;    // 仅重置当前激活地图请求标记
    std::shared_ptr<Map> mpMapToReset; // 需要重置的目标地图
    std::mutex mMutexReset;            // 重置相关成员互斥锁

    /**
     * @brief 检查是否收到线程结束请求
     * @return true收到结束请求
     */
    bool CheckFinish();

    /**
     * @brief 设置线程已经完成结束标记
     */
    void SetFinish();
    bool mbFinishRequested; // 请求线程结束标记
    bool mbFinished;        // 线程已经结束标记
    std::mutex mMutexFinish;// 结束状态变量互斥锁

    std::shared_ptr<Atlas> mpAtlas; // 全局地图集Atlas指针

    std::shared_ptr<LoopClosing> mpLoopCloser; // 回环闭合模块指针
    std::shared_ptr<Tracking> mpTracker;        // 跟踪线程模块指针

    std::list<std::shared_ptr<KeyFrame>> mlNewKeyFrames; // 待处理新关键帧队列，Tracking推送过来

    std::shared_ptr<KeyFrame> mpCurrentKeyFrame; // 当前局部建图正在处理的关键帧

    std::list<MapPoint *> mlpRecentAddedMapPoints; // 刚刚新生成的地图点原始指针列表，用于后续筛选剔除

    std::mutex mMutexNewKFs; // 保护mlNewKeyFrames队列互斥锁

    bool mbAbortBA; // 中断BA标记；true代表需要立刻终止正在运行的局部BA

    bool mbStopped;         // 线程已经停止标记
    bool mbStopRequested;   // 收到停止请求标记
    bool mbNotStop;         // 禁止线程停止标记，BA运算期间置true防止被打断
    std::mutex mMutexStop;  // 停止状态相关变量互斥锁

    bool mbAcceptKeyFrames; // 是否接收Tracking发来新关键帧开关
    std::mutex mMutexAccept;// 接收关键帧标记的互斥锁

    /**
     * @brief IMU初始化函数，求解陀螺仪、加速度计bias、尺度、重力方向
     * @param priorG 陀螺仪先验噪声权重
     * @param priorA 加速度计先验噪声权重
     * @param bFirst 是否第一次IMU初始化
     */
    void InitializeIMU(float priorG = 1e2, float priorA = 1e6,
                       bool bFirst = false);

    /**
     * @brief IMU初始化完成后尺度精细化优化
     */
    void ScaleRefinement();

    bool bInitializing; // IMU正在初始化标志位

    Eigen::MatrixXd infoInertial; // IMU信息矩阵
    int mNumLM;                   // 局部BA最大迭代次数
    int mNumKFCulling;            // 关键帧剔除计数

    float mTinit;       // IMU初始化时间阈值
    int countRefinement;// 尺度优化迭代计数

    // DEBUG 调试输出文件流
    ofstream f_lm;
};

} // namespace ORB_SLAM3
