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
#pragma once // 头文件保护宏，防止头文件被多次重复包含引发重定义编译错误
#include <Eigen/Core>        // Eigen基础头文件，矩阵向量基础数据结构
#include <Eigen/Dense>       // Eigen稠密矩阵求解、矩阵运算模块
#include <Eigen/Geometry>    // Eigen几何模块，旋转矩阵、四元数相关
#include <boost/serialization/serialization.hpp>    // boost序列化核心，用于对象磁盘持久化保存加载
#include <boost/serialization/vector.hpp>           // boost对std::vector容器的序列化支持
#include <iostream>          // C++标准输入输出流，打印调试日志
#include <memory>            // C++智能指针 std::shared_ptr
#include <mutex>             // C++互斥锁，多线程访问保护共享数据
#include <opencv2/core/core.hpp> // OpenCV核心模块，cv::Point3f等数据结构
#include <sophus/se3.hpp>    // Sophus库，SE(3)/SO(3)李群李代数实现
#include <utility>           // C++通用工具库
#include <vector>            // C++动态数组容器std::vector

#include "SerializationUtils.h" // ORB‑SLAM3自定义序列化工具，Sophus SE3序列化辅助函数

namespace ORB_SLAM3 {

namespace IMU {
const float GRAVITY_VALUE = 9.81; // 重力加速度常量，单位m/s²

// IMU measurement (gyro, accelerometer and timestamp)
// IMU原始测量点类：保存单条IMU数据，包含加速度、角速度、时间戳
class Point {
public:
    // 构造函数1：直接传入xyz三轴加速度、三轴角速度、时间戳
    Point(const float &acc_x, const float &acc_y, const float &acc_z,
          const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
          const double &timestamp)
        : a(acc_x, acc_y, acc_z),
          w(ang_vel_x, ang_vel_y, ang_vel_z),
          t(timestamp) {}

    // 构造函数2：接收OpenCV的cv::Point3f格式加速度、陀螺仪数据，加上时间戳
    Point(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp)
        : a(Acc.x, Acc.y, Acc.z), w(Gyro.x, Gyro.y, Gyro.z), t(timestamp) {}

public:
    Eigen::Vector3f a; // IMU加速度计测量值，载体坐标系
    Eigen::Vector3f w; // IMU陀螺仪角速度测量值，载体坐标系
    double t;          // 该条IMU测量对应的时间戳
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏，类内含有Eigen固定大小向量必须添加，防止内存未对齐崩溃
};

// IMU biases (gyro and accelerometer)
// IMU偏置类：保存加速度计bias与陀螺仪bias，VIO状态变量之一
class Bias {
    friend class boost::serialization::access; // boost序列化友元声明，允许序列化私有成员
    // boost序列化模板函数，实现Bias对象序列化/反序列化
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & bax; // 序列化加速度计x轴bias
        ar & bay; // 序列化加速度计y轴bias
        ar & baz; // 序列化加速度计z轴bias

        ar & bwx; // 序列化陀螺仪x轴bias
        ar & bwy; // 序列化陀螺仪y轴bias
        ar & bwz; // 序列化陀螺仪z轴bias
    }

public:
    // 默认构造函数，所有bias初始化为0
    Bias() : bax(0), bay(0), baz(0), bwx(0), bwy(0), bwz(0) {}

    // 带参构造函数，传入三轴加速度bias、三轴陀螺仪bias完成初始化
    Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
         const float &b_ang_vel_x, const float &b_ang_vel_y,
         const float &b_ang_vel_z)
        : bax(b_acc_x),
          bay(b_acc_y),
          baz(b_acc_z),
          bwx(b_ang_vel_x),
          bwy(b_ang_vel_y),
          bwz(b_ang_vel_z) {}

    // 将入参Bias对象完整拷贝至当前对象
    void CopyFrom(Bias &b);
    // 重载输出流运算符，方便直接打印Bias全部bias数值用于调试
    friend std::ostream &operator<<(std::ostream &out, const Bias &b);

public:
    float bax, bay, baz; // 加速度计三轴bias
    float bwx, bwy, bwz; // 陀螺仪三轴bias
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏
};

// IMU calibration (Tbc, Tcb, noise)
// IMU标定参数类：存储相机‑IMU外参Tbc/Tcb、IMU噪声、随机游走噪声
class Calib {
    friend class boost::serialization::access; // boost序列化友元
    // boost序列化函数，序列化Calib全部标定参数
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        serializeSophusSE3(ar, mTcb, version); // 序列化IMU到相机外参Tcb
        serializeSophusSE3(ar, mTbc, version); // 序列化相机到IMU外参Tbc

        // 序列化IMU测量噪声对角矩阵对角线元素
        ar &boost::serialization::make_array(Cov.diagonal().data(),
                                             Cov.diagonal().size());
        // 序列化bias随机游走噪声对角矩阵对角线元素
        ar &boost::serialization::make_array(CovWalk.diagonal().data(),
                                             CovWalk.diagonal().size());

        ar & mbIsSet; // 序列化标定参数是否已经设置标志位
    }

public:
    // 构造函数：传入相机‑IMU外参Tbc，陀螺仪噪声ng，加速度噪声na，gyro随机游走ngw，acc随机游走naw
    Calib(const Sophus::SE3<float> &Tbc, const float &ng, const float &na,
          const float &ngw, const float &naw) {
        Set(Tbc, ng, na, ngw, naw);
    }

    // 拷贝构造函数，从另一个Calib对象复制全部标定参数
    Calib(const Calib &calib);
    // 默认构造，标定参数置为未设置状态
    Calib() { mbIsSet = false; }

    // void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const
    // float &ngw, const float &naw);
    // 设置全部IMU标定参数：外参、测量噪声、bias随机游走噪声
    void Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na,
             const float &ngw, const float &naw);

public:
    // Sophus/Eigen implementation
    Sophus::SE3<float> mTcb;    // IMU到相机坐标系变换 T_cam_imu
    Sophus::SE3<float> mTbc;    // 相机到IMU坐标系变换 T_imu_cam
    Eigen::DiagonalMatrix<float, 6> Cov, CovWalk; // Cov：IMU测量噪声；CovWalk：bias随机游走噪声，6维分别对应3gyro+3acc
    bool mbIsSet;               // 标记标定参数是否已经合法设置完成
};

// Integration of 1 gyro measurement
// 单步陀螺仪积分类：对单段角速度做SO3旋转积分，保存deltaR与右雅可比
class IntegratedRotation {
public:
    IntegratedRotation() {} // 默认空构造
    // 构造函数：输入角速度、IMU bias、积分时间，执行旋转积分
    IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias,
                       const float &time);

public:
    float deltaT;                // 积分时间间隔
    Eigen::Matrix3f deltaR;      // 积分得到的SO3旋转增量
    Eigen::Matrix3f rightJ;      // SO3右雅可比矩阵
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏
};

// Preintegration of Imu Measurements
// IMU预积分主类，ORB‑SLAM3 VIO核心，存储预积分结果、误差传递雅可比、协方差矩阵
class Preintegrated {
    friend class boost::serialization::access; // boost序列化友元
    // boost序列化函数，序列化预积分对象全部成员
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & dT; // 总预积分时间
        ar &boost::serialization::make_array(C.data(), C.size()); // 15×15误差传递矩阵C
        ar &boost::serialization::make_array(Info.data(), Info.size()); // 15×15信息矩阵
        ar &boost::serialization::make_array(Nga.diagonal().data(),
                                             Nga.diagonal().size()); // IMU原始噪声
        ar &boost::serialization::make_array(NgaWalk.diagonal().data(),
                                             NgaWalk.diagonal().size()); // bias随机游走噪声
        ar & b; // 预积分计算时使用的原始bias
        ar &boost::serialization::make_array(dR.data(), dR.size()); // 旋转预积分增量
        ar &boost::serialization::make_array(dV.data(), dV.size()); // 速度预积分增量
        ar &boost::serialization::make_array(dP.data(), dP.size()); // 位置预积分增量
        ar &boost::serialization::make_array(JRg.data(), JRg.size()); // R对gyro bias雅可比
        ar &boost::serialization::make_array(JVg.data(), JVg.size()); // V对gyro bias雅可比
        ar &boost::serialization::make_array(JVa.data(), JVa.size()); // V对acc bias雅可比
        ar &boost::serialization::make_array(JPg.data(), JPg.size()); // P对gyro bias雅可比
        ar &boost::serialization::make_array(JPa.data(), JPa.size()); // P对acc bias雅可比
        ar &boost::serialization::make_array(avgA.data(), avgA.size()); // 预积分段平均加速度
        ar &boost::serialization::make_array(avgW.data(), avgW.size()); // 预积分段平均角速度

        ar & bu; // 更新后的bias
        ar &boost::serialization::make_array(db.data(), db.size()); // bias偏差量 db = bu‑b
        ar & mvMeasurements; // 保存该段预积分全部原始IMU测量
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏
    // 构造函数：使用给定初始bias与IMU标定参数，初始化预积分
    Preintegrated(const Bias &b_, const Calib &calib);

    // \todo{}  Why not the default copy constructor?
    // 从另一个预积分智能指针对象拷贝构造新预积分对象
    explicit Preintegrated(const std::shared_ptr<Preintegrated> &pImuPre);

    Preintegrated() = default; // 默认构造，编译器生成默认实现
    ~Preintegrated() {} // 空析构函数

    // 从另一个预积分对象完整复制所有数据
    void CopyFrom(const std::shared_ptr<Preintegrated> &pImuPre);
    // 使用指定bias重置初始化预积分状态
    void Initialize(const Bias &b_);
    // 传入单条IMU测量值与时间间隔dt，执行增量预积分
    void IntegrateNewMeasurement(const Eigen::Vector3f &acceleration,
                                 const Eigen::Vector3f &angVel, const float &dt);
    // 使用原始存储的IMU观测，重新执行一遍完整预积分（bias发生变化时调用）
    void Reintegrate();
    // 将上一段预积分与当前预积分进行合并，得到拼接后的预积分
    void MergePrevious(const std::shared_ptr<Preintegrated> &pPrev);
    // 设置新的bias，计算bias差值db，用于修正预积分结果，不重新积分
    void SetNewBias(const Bias &bu_);
    // 输入bias，返回与原始bias之间的bias偏差deltaBias
    IMU::Bias GetDeltaBias(const Bias &b_);

    // 根据传入bias，修正并获取旋转增量deltaR
    Eigen::Matrix3f GetDeltaRotation(const Bias &b_);
    // 根据传入bias，修正并获取速度增量deltaV
    Eigen::Vector3f GetDeltaVelocity(const Bias &b_);
    // 根据传入bias，修正并获取位置增量deltaP
    Eigen::Vector3f GetDeltaPosition(const Bias &b_);

    // 获取经过SetNewBias更新bias之后的旋转增量
    Eigen::Matrix3f GetUpdatedDeltaRotation();
    // 获取经过SetNewBias更新bias之后的速度增量
    Eigen::Vector3f GetUpdatedDeltaVelocity();
    // 获取经过SetNewBias更新bias之后的位置增量
    Eigen::Vector3f GetUpdatedDeltaPosition();

    // 获取原始bias计算得到的旋转增量，不做bias修正
    Eigen::Matrix3f GetOriginalDeltaRotation();
    // 获取原始bias计算得到的速度增量，不做bias修正
    Eigen::Vector3f GetOriginalDeltaVelocity();
    // 获取原始bias计算得到的位置增量，不做bias修正
    Eigen::Vector3f GetOriginalDeltaPosition();

    // 获取6维bias偏差向量，前3维acc bias，后3维gyro bias
    Eigen::Matrix<float, 6, 1> GetDeltaBias();

    // 获取预积分计算时使用的原始bias
    Bias GetOriginalBias();
    // 获取更新之后的bias bu
    Bias GetUpdatedBias();

    // 调试打印：输出本预积分段保存的全部IMU测量时间戳
    void printMeasurements() const {
        std::cout << "pint meas:\n";
        for (size_t i = 0; i < mvMeasurements.size(); i++) {
            std::cout << "meas " << mvMeasurements[i].t << std::endl;
        }
        std::cout << "end pint meas:\n";
    }

public:
    float dT;                                  // 该段预积分总时间
    Eigen::Matrix<float, 15, 15> C;            // 15维状态误差传递矩阵，状态：θ,v,p,ba,bg
    Eigen::Matrix<float, 15, 15> Info;         // 15×15信息矩阵，协方差矩阵的逆
    Eigen::DiagonalMatrix<float, 6> Nga, NgaWalk; // Nga IMU测量噪声；NgaWalk bias随机游走噪声

    // Values for the original bias (when integration was computed)
    Bias b;                                    // 计算预积分时使用的原始IMU bias
    Eigen::Matrix3f dR;                        // 原始bias下预积分旋转增量ΔR
    Eigen::Vector3f dV, dP;                    // 原始bias下预积分速度增量ΔV、位置增量ΔP
    Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;   // 各状态对bias的雅可比矩阵
    Eigen::Vector3f avgA, avgW;                // 该段预积分的平均加速度、平均角速度

private:
    // Updated bias
    Bias bu;                                   // 更新后的IMU bias，优化求解后得到的bias
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    Eigen::Matrix<float, 6, 1> db;             // bias偏差 db = bu − b，用于一阶修正预积分结果

    // 内部结构体：保存用于重积分的单条IMU观测数据
    struct integrable {
        template <class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar &boost::serialization::make_array(a.data(), a.size()); // 序列化加速度
            ar &boost::serialization::make_array(w.data(), w.size()); // 序列化角速度
            ar & t; // 序列化时间间隔
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐宏
        integrable() {} // 默认构造
        // 构造函数，赋值加速度、角速度、时间间隔
        integrable(const Eigen::Vector3f &a_, const Eigen::Vector3f &w_,
                   const float &t_)
            : a(a_), w(w_), t(t_) {}
        Eigen::Vector3f a, w; // 加速度、角速度观测
        float t;              // 时间间隔dt
    };

    std::vector<integrable> mvMeasurements; // 保存该预积分区间内全部原始IMU观测，用于Reintegrate重积分

    std::mutex mMutex; // 互斥锁，多线程访问预积分对象时做线程安全保护
};

// Lie Algebra Functions
// SO3右雅可比，输入旋转向量三轴分量x y z，返回右雅可比矩阵Jr
Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y,
                                 const float &z);
// SO3右雅可比重载，直接输入旋转向量v
Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v);

// SO3右雅可比的逆，输入旋转向量三轴分量x y z
Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y,
                                        const float &z);
// SO3右雅可比的逆重载，直接输入旋转向量v
Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v);

// 旋转矩阵归一化，消除数值漂移，保证输出严格正交SO(3)矩阵
Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R);

}  // namespace IMU

}  // namespace ORB_SLAM3
