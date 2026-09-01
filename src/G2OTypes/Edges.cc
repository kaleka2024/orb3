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
 * ORB‑SLAM3. If not, see http://www.gnu.org/licenses/.
 */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Converter.h"
#include "G2oTypes.h"
#include "ImuTypes.h"

namespace ORB_SLAM3
{

//-------------------------------------------------------------------
/**
 * @brief 单目重投影误差边：线性化计算雅可比矩阵，g2o边基类接口
 * 边连接：_vertices[0] 地图点VertexPointXYZ，_vertices[1] 位姿顶点VertexPose
 * 误差：图像平面重投影误差(2维)
 */
void EdgeMono::linearizeOplus()
{
    // 将g2o顶点指针强转为自定义位姿顶点，_vertices[1]为位姿顶点
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    // 将_vertices[0]强转为g2o三维地图点顶点
    const g2o::VertexPointXYZ* VPoint = static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);

    // 获取第cam_idx相机：世界到相机坐标系旋转矩阵Rcw
    const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
    // 获取第cam_idx相机：世界到相机坐标系平移向量tcw
    const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];

    // 世界坐标系下地图点 -> 相机坐标系下三维点 Xc = Rcw*Pw + tcw
    const Eigen::Vector3d Xc = Rcw * VPoint->estimate() + tcw;

    // 相机坐标系点Xc转换到IMU机体坐标系Xb：Xb = Rbc*Xc + tbc，外参cam‑imu
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];

    // 获取IMU机体到相机的旋转矩阵Rcb，Rcb = Rbc^T
    const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

    // 获取相机投影函数对相机坐标系三维点Xc的雅可比矩阵(2×3)，d(u,v)/d(Xc,Yc,Zc)
    const Eigen::Matrix<double, 2, 3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);

    // _jacobianOplusXi：误差对顶点0(地图点Pw)的雅可比，de/dPw = ‑proj_jac * Rcw
    _jacobianOplusXi = -proj_jac * Rcw;

    // SE(3)右扰动雅可比辅助矩阵，机体坐标系点Xb，用于计算d(Xb)/d(ξ)，ξ是SE3微小扰动[ω ρ]^T
    Eigen::Matrix<double, 3, 6> SE3deriv;
    // 取出机体坐标系三维点Xb的x,y,z分量
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    // SE3右乘扰动下，三维点对扰动ξ的导数矩阵 [⌊Xb⌋ I]，3×6
    SE3deriv << 0.0,  z, -y, 1.0, 0.0, 0.0,
                -z, 0.0,  x, 0.0, 1.0, 0.0,
                 y, -x, 0.0, 0.0, 0.0, 1.0;

    // _jacobianOplusXj：误差对顶点1(相机位姿)的雅可比，de/dξ = proj_jac * Rcb * SE3deriv
    _jacobianOplusXj = proj_jac * Rcb * SE3deriv; // TODO optimize this product
}

//-------------------------------------------------------------------
/**
 * @brief 单目仅位姿边：固定地图点，只优化位姿，计算雅可比
 * 边连接：_vertices[0]位姿顶点VertexPose；地图点Xw为类成员变量固定值
 * 误差：图像平面重投影误差(2维)
 */
void EdgeMonoOnlyPose::linearizeOplus()
{
    // 获取位姿顶点，本边只有位姿一个顶点
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
    // 获取该相机世界‑相机旋转Rcw
    const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
    // 获取该相机世界‑相机平移tcw
    const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];

    // 固定世界点Xw投影到相机坐标系 Xc = Rcw*Xw + tcw
    const Eigen::Vector3d Xc = Rcw * Xw + tcw;
    // 相机坐标系点转换IMU机体坐标系Xb
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
    // 获取机体到相机旋转Rcb
    const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

    // 获取投影雅可比 d(u,v)/dXc (2×3)
    Eigen::Matrix<double, 2, 3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);

    // SE(3)右扰动点导数矩阵，机体坐标系Xb
    Eigen::Matrix<double, 3, 6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0,  z, -y, 1.0, 0.0, 0.0,
                -z, 0.0,  x, 0.0, 1.0, 0.0,
                 y, -x, 0.0, 0.0, 0.0, 1.0;

    // 误差对位姿顶点的雅可比矩阵，本边只有一个顶点，存入_jacobianOplusXi
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv; // symbol different becasue of update mode
}

/**
 * @brief 双目重投影误差边线性化，误差维度3：u,v, 视差d
 * 顶点0：地图点VertexPointXYZ；顶点1：位姿VertexPose
 */
void EdgeStereo::linearizeOplus()
{
    // 获取位姿顶点 _vertices[1]
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    // 获取地图点顶点 _vertices[0]
    const g2o::VertexPointXYZ* VPoint = static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);

    // 获取相机外参Rcw、tcw，世界到相机
    const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];

    // 世界地图点投影到相机坐标系 Xc
    const Eigen::Vector3d Xc = Rcw * VPoint->estimate() + tcw;
    // 相机坐标系点转到IMU机体坐标系Xb
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
    // 机体到相机旋转Rcb
    const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

    // 获取双目基线乘焦距 bf
    const double bf = VPose->estimate().bf;
    // 1/(Zc)^2，视差求导需要
    const double inv_z2 = 1.0 / (Xc(2) * Xc(2));

    // 双目投影雅可比矩阵3×3，三行分别对应u、v、视差d
    Eigen::Matrix<double, 3, 3> proj_jac;
    // 前两行：普通针孔相机u,v对Xc雅可比
    proj_jac.block<2, 3>(0, 0) = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    // 视差d初始拷贝u行雅可比
    proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
    // 视差对Zc求导附加项 bf / Zc²
    proj_jac(2, 2) += bf * inv_z2;

    // 误差对地图点(顶点0)雅可比 de/dPw = ‑proj_jac * Rcw
    _jacobianOplusXi = -proj_jac * Rcw;

    // SE3右扰动三维点导数矩阵，机体坐标系Xb
    Eigen::Matrix<double, 3, 6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0,  z, -y, 1.0, 0.0, 0.0,
                -z, 0.0,  x, 0.0, 1.0, 0.0,
                 y, -x, 0.0, 0.0, 0.0, 1.0;

    // 误差对位姿顶点(顶点1)雅可比 de/dξ = proj_jac * Rcb * SE3deriv
    _jacobianOplusXj = proj_jac * Rcb * SE3deriv;
}

/**
 * @brief 双目仅位姿边：固定地图点，仅优化位姿，计算雅可比
 * 顶点0：位姿VertexPose；地图点Xw为类成员固定值
 */
void EdgeStereoOnlyPose::linearizeOplus()
{
    // 获取唯一顶点：位姿顶点
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
    // 获取相机Rcw、tcw
    const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];

    // 固定世界点Xw投影相机坐标系
    const Eigen::Vector3d Xc = Rcw * Xw + tcw;
    // 转换到IMU机体坐标系
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
    // 获取Rcb机体‑相机旋转
    const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

    // 获取bf基线焦距乘积
    const double bf = VPose->estimate().bf;
    // 1/Zc²
    const double inv_z2 = 1.0 / (Xc(2) * Xc(2));

    // 双目3×3投影雅可比矩阵 u,v,视差d
    Eigen::Matrix<double, 3, 3> proj_jac;
    proj_jac.block<2, 3>(0, 0) = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
    proj_jac(2, 2) += bf * inv_z2;

    // SE3右扰动三维点导数矩阵
    Eigen::Matrix<double, 3, 6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0,  z, -y, 1.0, 0.0, 0.0,
                -z, 0.0,  x, 0.0, 1.0, 0.0,
                 y, -x, 0.0, 0.0, 0.0, 1.0;

    // 误差对位姿顶点雅可比，仅一个顶点存入Xi
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv;
}

//-------------------------------------------------------------------
/**
 * @brief 速度顶点构造函数，由关键帧初始化速度顶点
 * @param pKF 关键帧智能指针，读取关键帧速度赋值顶点估计值
 */
VertexVelocity::VertexVelocity(const std::shared_ptr<KeyFrame>& pKF)
{
    // 将关键帧速度Eigen::Vector3f转为double，设置为顶点的待优化变量
    setEstimate(pKF->GetVelocity().cast<double>());
}

/**
 * @brief 速度顶点构造函数，由普通帧初始化速度顶点
 * @param pF 普通帧智能指针，读取普通帧速度赋值顶点估计值
 */
VertexVelocity::VertexVelocity(const std::shared_ptr<Frame>& pF)
{
    setEstimate(pF->GetVelocity().cast<double>());
}

/**
 * @brief 陀螺仪bias顶点构造函数，由关键帧初始化陀螺零偏
 * @param pKF 关键帧智能指针，读取关键帧陀螺bias
 */
VertexGyroBias::VertexGyroBias(const std::shared_ptr<KeyFrame>& pKF)
{
    setEstimate(pKF->GetGyroBias().cast<double>());
}

/**
 * @brief 陀螺仪bias顶点构造函数，由普通帧初始化陀螺零偏
 * @param pF 普通帧智能指针，从帧IMU数据结构体取出三轴陀螺bias
 */
VertexGyroBias::VertexGyroBias(const std::shared_ptr<Frame>& pF)
{
    Eigen::Vector3d bg;
    // 提取普通帧IMU bias结构体中wx wy wz三轴陀螺零偏
    bg << pF->mImuBias.bwx, pF->mImuBias.bwy, pF->mImuBias.bwz;
    // 设置为顶点待优化值
    setEstimate(bg);
}

/**
 * @brief 加速度计bias顶点构造函数，关键帧初始化加速度零偏
 * @param pKF 关键帧智能指针，读取关键帧加速度bias
 */
VertexAccBias::VertexAccBias(const std::shared_ptr<KeyFrame>& pKF)
{
    setEstimate(pKF->GetAccBias().cast<double>());
}

/**
 * @brief 加速度计bias顶点构造函数，普通帧初始化加速度零偏
 * @param pF 普通帧智能指针，提取帧IMU bias结构体三轴加速度bias
 */
VertexAccBias::VertexAccBias(const std::shared_ptr<Frame>& pF)
{
    Eigen::Vector3d ba;
    // 提取bax bay baz三轴加速度计零偏
    ba << pF->mImuBias.bax, pF->mImuBias.bay, pF->mImuBias.baz;
    setEstimate(ba);
}

//-------------------------------------------------------------------
/**
 * @brief IMU预积分边构造函数，连接两个时刻IMU状态，共6个顶点
 * @param pInt IMU预积分结果智能指针，保存预积分ΔR ΔV ΔP，雅可比，协方差矩阵
 */
EdgeInertial::EdgeInertial(const std::shared_ptr<IMU::Preintegrated>& pInt)
    : JRg(pInt->JRg.cast<double>()),
      JVg(pInt->JVg.cast<double>()),
      JPg(pInt->JPg.cast<double>()),
      JVa(pInt->JVa.cast<double>()),
      JPa(pInt->JPa.cast<double>()),
      mpInt(pInt),
      dt(pInt->dT)
{
    // 该边绑定6个顶点：P1,V1,Bg1,Ba1,P2,V2
    resize(6);
    // 世界坐标系重力向量 g=[0,0,-g0]
    g << 0, 0, -IMU::GRAVITY_VALUE;

    // 取出预积分协方差矩阵左上角9×9(旋转、速度、位置)，转为double再求逆得到信息矩阵
    Matrix9d Info = pInt->C.block<9, 9>(0, 0).cast<double>().inverse();
    // 强制信息矩阵对称，消除数值误差带来的不对称
    Info = (Info + Info.transpose()) / 2;

    // 特征值分解，做半正定修复，把极小负特征值置0，保证信息矩阵半正定
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9> > es(Info);
    Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
    for (int i = 0; i < 9; i++)
        if (eigs[i] < 1e-12)
            eigs[i] = 0;
    // 重构修复后的信息矩阵
    Info = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    // 设置g2o边的信息矩阵
    setInformation(Info);
}

/**
 * @brief IMU预积分边计算残差，误差9维：旋转残差er(3)、速度残差ev(3)、位置残差ep(3)
 */
void EdgeInertial::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too big
    // 顶点0：前一时刻位姿P1
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    // 顶点1：前一时刻速度V1
    const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
    // 顶点2：前一时刻陀螺bias Bg1
    const VertexGyroBias* VG1 = static_cast<const VertexGyroBias*>(_vertices[2]);
    // 顶点3：前一时刻加速度bias Ba1
    const VertexAccBias* VA1 = static_cast<const VertexAccBias*>(_vertices[3]);
    // 顶点4：后一时刻位姿P2
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    // 顶点5：后一时刻速度V2
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);

    // 组装当前迭代的IMU bias，由bias顶点当前估计值
    const IMU::Bias b1(VA1->estimate()[0], VA1->estimate()[1], VA1->estimate()[2],
                       VG1->estimate()[0], VG1->estimate()[1], VG1->estimate()[2]);

    // 使用当前bias重新修正预积分得到ΔR、ΔV、ΔP
    const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b1).cast<double>();
    const Eigen::Vector3d dV = mpInt->GetDeltaVelocity(b1).cast<double>();
    const Eigen::Vector3d dP = mpInt->GetDeltaPosition(b1).cast<double>();

    // 旋转残差 er = LogSO3( ΔR^T * Rwb1^T * Rwb2 )，SO3对数映射得到旋转误差向量
    const Eigen::Vector3d er = LogSO3( dR.transpose() * VP1->estimate().Rwb.transpose() * VP2->estimate().Rwb );

    // 速度残差 ev = Rwb1^T*(V2‑V1‑g*dt) − ΔV，转到P1机体坐标系下
    const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose() * (VV2->estimate() - VV1->estimate() - g * dt) - dV;

    // 位置残差 ep = Rwb1^T*(P2‑P1‑V1*dt‑0.5*g*dt²) − ΔP
    const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose() * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt - g * dt * dt / 2) - dP;

    // 拼接9维残差 [er; ev; ep]
    _error << er, ev, ep;
}

/**
 * @brief IMU预积分边线性化，计算对全部6个顶点的雅可比矩阵
 */
void EdgeInertial::linearizeOplus()
{
    // 取出全部6个顶点
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG1 = static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA1 = static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);

    // 获取当前bias顶点估计，组装bias结构体
    const IMU::Bias b1(VA1->estimate()[0], VA1->estimate()[1], VA1->estimate()[2],
                       VG1->estimate()[0], VG1->estimate()[1], VG1->estimate()[2]);
    // 获取bias相对于线性化点的偏差db
    const IMU::Bias db = mpInt->GetDeltaBias(b1);
    // 提取陀螺bias偏差dbg(三维)
    Eigen::Vector3d dbg;
    dbg << db.bwx, db.bwy, db.bwz;

    // Rwb1：时刻1机体‑世界旋转；Rbw1 = Rwb1^T 世界‑机体旋转
    const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    // Rwb2：时刻2机体‑世界旋转
    const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;

    // 使用当前bias修正后的预积分旋转ΔR
    const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b1).cast<double>();
    // eR = ΔR^T * Rbw1 * Rwb2，用于计算旋转残差er
    const Eigen::Matrix3d eR = dR.transpose() * Rbw1 * Rwb2;
    // er = LogSO3(eR) 旋转残差向量
    const Eigen::Vector3d er = LogSO3(eR);
    // SO3右雅可比逆矩阵，用于旋转残差求导
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // ========== 对顶点0：P1(位姿1，6维：旋转3，平移3) 的雅可比 ==========
    _jacobianOplus[0].setZero();
    // 旋转残差er对P1旋转的雅可比 3×3
    _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1; // OK
    // 速度残差ev对P1旋转的雅可比 3×3
    _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat( Rbw1 * (VV2->estimate() - VV1->estimate() - g * dt)); // OK
    // 位置残差ep对P1旋转的雅可比 3×3
    _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat( Rbw1 * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt - 0.5 * g * dt * dt)); // OK
    // 位置残差ep对P1平移的雅可比 3×3
    _jacobianOplus[0].block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity(); // OK

    // ========== 对顶点1：V1(速度1，3维) 的雅可比 ==========
    _jacobianOplus[1].setZero();
    // 速度残差ev对V1的雅可比 3×3
    _jacobianOplus[1].block<3, 3>(3, 0) = -Rbw1; // OK
    // 位置残差ep对V1的雅可比 3×3
    _jacobianOplus[1].block<3, 3>(6, 0) = -Rbw1 * dt; // OK

    // ========== 对顶点2：Bg1(陀螺bias1，3维) 的雅可比 ==========
    _jacobianOplus[2].setZero();
    // 旋转残差er对陀螺bias的雅可比 3×3
    _jacobianOplus[2].block<3, 3>(0, 0) = -invJr * eR.transpose() * RightJacobianSO3(JRg * dbg) * JRg; // OK
    // 速度残差ev对陀螺bias雅可比 3×3
    _jacobianOplus[2].block<3, 3>(3, 0) = -JVg; // OK
    // 位置残差ep对陀螺bias雅可比 3×3
    _jacobianOplus[2].block<3, 3>(6, 0) = -JPg; // OK

    // ========== 对顶点3：Ba1(加速度bias1，3维) 的雅可比 ==========
    _jacobianOplus[3].setZero();
    // 速度残差ev对加速度bias雅可比3×3
    _jacobianOplus[3].block<3, 3>(3, 0) = -JVa; // OK
    // 位置残差ep对加速度bias雅可比3×3
    _jacobianOplus[3].block<3, 3>(6, 0) = -JPa; // OK

    // ========== 对顶点4：P2(位姿2，6维) 的雅可比 ==========
    _jacobianOplus[4].setZero();
    // 旋转残差er对P2旋转雅可比3×3
    _jacobianOplus[4].block<3, 3>(0, 0) = invJr; // OK
    // 位置残差ep对P2平移雅可比3×3
    _jacobianOplus[4].block<3, 3>(6, 3) = Rbw1 * Rwb2; // OK

    // ========== 对顶点5：V2(速度2，3维) 的雅可比 ==========
    _jacobianOplus[5].setZero();
    // 速度残差ev对V2雅可比3×3
    _jacobianOplus[5].block<3, 3>(3, 0) = Rbw1; // OK
}

//-------------------------------------------------------------------
/**
 * @brief 重力方向+尺度的IMU预积分边，用于单目IMU初始化，共8个顶点
 * @param pInt IMU预积分数据，继承EdgeInertial大部分逻辑，新增重力方向、尺度顶点
 */
EdgeInertialGS::EdgeInertialGS(const std::shared_ptr<IMU::Preintegrated>& pInt)
    : JRg(pInt->JRg.cast<double>()),
      JVg(pInt->JVg.cast<double>()),
      JPg(pInt->JPg.cast<double>()),
      JVa(pInt->JVa.cast<double>()),
      JPa(pInt->JPa.cast<double>()),
      mpInt(pInt),
      dt(pInt->dT)
{
    // 该边绑定8个顶点：P1,V1,Bg,Ba,P2,V2,Gdir,Scale
    resize(8);
    // 标准重力向量，未旋转，[0,0,-g0]
    gI << 0, 0, -IMU::GRAVITY_VALUE;

    // 取预积分协方差9×9，求逆得到信息矩阵
    Matrix9d Info = pInt->C.block<9, 9>(0, 0).cast<double>().inverse();
    // 强制对称消除数值噪声
    Info = (Info + Info.transpose()) / 2;
    // 特征值分解修复半正定，极小特征值置0
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9> > es(Info);
    Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
    for (int i = 0; i < 9; i++)
        if (eigs[i] < 1e-12)
            eigs[i] = 0;
    Info = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    setInformation(Info);
}

/**
 * @brief 带重力方向、尺度的IMU边计算残差，9维残差er ev ep
 */
void EdgeInertialGS::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too big
    // 顶点0：P1位姿1
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    // 顶点1：V1速度1
    const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
    // 顶点2：Bg陀螺bias
    const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
    // 顶点3：Ba加速度bias
    const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);
    // 顶点4：P2位姿2
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    // 顶点5：V2速度2
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    // 顶点6：重力方向顶点，2维参数化重力方向旋转
    const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
    // 顶点7：尺度s顶点，单目尺度因子
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);

    // 组装当前bias
    const IMU::Bias b(VA->estimate()[0], VA->estimate()[1], VA->estimate()[2],
                      VG->estimate()[0], VG->estimate()[1], VG->estimate()[2]);
    // 旋转标准重力得到世界坐标系下实际重力向量g
    g = VGDir->estimate().Rwg * gI;
    // 获取单目尺度因子s
    const double s = VS->estimate();

    // bias修正预积分得到ΔR ΔV ΔP
    const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b).cast<double>();
    const Eigen::Vector3d dV = mpInt->GetDeltaVelocity(b).cast<double>();
    const Eigen::Vector3d dP = mpInt->GetDeltaPosition(b).cast<double>();

    // 旋转残差er，和普通IMU边一致
    const Eigen::Vector3d er = LogSO3( dR.transpose() * VP1->estimate().Rwb.transpose() * VP2->estimate().Rwb );
    // 速度残差ev，引入尺度s，s*(V2‑V1)
    const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose() * (s * (VV2->estimate() - VV1->estimate()) - g * dt) - dV;
    // 位置残差ep，引入尺度s，s作用于位置差、速度项
    const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose() * (s * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt) - g * dt * dt / 2) - dP;

    // 拼接9维残差
    _error << er, ev, ep;
}

/**
 * @brief 带重力方向、尺度IMU边的雅可比计算，8个顶点全部求导
 */
void EdgeInertialGS::linearizeOplus()
{
    // 取出全部8个顶点
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);

    // 当前bias估计
    const IMU::Bias b(VA->estimate()[0], VA->estimate()[1], VA->estimate()[2],
                      VG->estimate()[0], VG->estimate()[1], VG->estimate()[2]);
    // bias相对于线性点偏差db
    const IMU::Bias db = mpInt->GetDeltaBias(b);
    Eigen::Vector3d dbg;
    dbg << db.bwx, db.bwy, db.bwz;

    // Rwb1时刻1机体‑世界；Rbw1世界‑机体；Rwb2时刻2机体‑世界
    const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;
    // Rwg：重力方向顶点的旋转矩阵，把标准重力转到世界系
    const Eigen::Matrix3d Rwg = VGDir->estimate().Rwg;

    // Gm矩阵，重力向量对重力方向2维扰动的导数中间矩阵
    Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(3, 2);
    Gm(0, 1) = -IMU::GRAVITY_VALUE;
    Gm(1, 0) = IMU::GRAVITY_VALUE;
    // dg/dθ：世界重力向量对重力方向2维扰动的雅可比3×2
    const Eigen::MatrixXd dGdTheta = Rwg * Gm;

    // 获取尺度因子s
    const double s = VS->estimate();

    // bias修正预积分旋转ΔR
    const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b).cast<double>();
    const Eigen::Matrix3d eR = dR.transpose() * Rbw1 * Rwb2;
    const Eigen::Vector3d er = LogSO3(eR);
    // SO3右雅可比逆
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // ========== 顶点0：P1位姿1(6维) ==========
    _jacobianOplus[0].setZero();
    // er对P1旋转雅可比
    _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1;
    // ev对P1旋转雅可比，引入尺度s
    _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat( Rbw1 * (s * (VV2->estimate() - VV1->estimate()) - g * dt));
    // ep对P1旋转雅可比，引入尺度s
    _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat( Rbw1 * (s * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt) - 0.5 * g * dt * dt));
    // ep对P1平移雅可比，尺度s乘单位矩阵
    _jacobianOplus[0].block<3, 3>(6, 3) = Eigen::DiagonalMatrix<double, 3>(-s, -s, -s);

    // ========== 顶点1：V1速度1(3维) ==========
    _jacobianOplus[1].setZero();
    // ev对V1雅可比带s
    _jacobianOplus[1].block<3, 3>(3, 0) = -s * Rbw1;
    // ep对V1雅可比带s
    _jacobianOplus[1].block<3, 3>(6, 0) = -s * Rbw1 * dt;

    // ========== 顶点2：Bg陀螺bias(3维) ==========
    _jacobianOplus[2].setZero();
    // er对陀螺bias雅可比
    _jacobianOplus[2].block<3, 3>(0, 0) = -invJr * eR.transpose() * RightJacobianSO3(JRg * dbg) * JRg;
    // ev对陀螺bias雅可比
    _jacobianOplus[2].block<3, 3>(3, 0) = -JVg;
    // ep对陀螺bias雅可比
    _jacobianOplus[2].block<3, 3>(6, 0) = -JPg;

    // ========== 顶点3：Ba加速度bias(3维) ==========
    _jacobianOplus[3].setZero();
    // ev对加速度bias雅可比
    _jacobianOplus[3].block<3, 3>(3, 0) = -JVa;
    // ep对加速度bias雅可比
    _jacobianOplus[3].block<3, 3>(6, 0) = -JPa;

    // ========== 顶点4：P2位姿2(6维) ==========
    _jacobianOplus[4].setZero();
    // er对P2旋转雅可比
    _jacobianOplus[4].block<3, 3>(0, 0) = invJr;
    // ep对P2平移雅可比，带尺度s
    _jacobianOplus[4].block<3, 3>(6, 3) = s * Rbw1 * Rwb2;

    // ========== 顶点5：V2速度2(3维) ==========
    _jacobianOplus[5].setZero();
    // ev对V2雅可比，带尺度s
    _jacobianOplus[5].block<3, 3>(3, 0) = s * Rbw1;

    // ========== 顶点6：重力方向GDir(2维参数) ==========
    _jacobianOplus[6].setZero();
    // ev对重力方向扰动雅可比3×2
    _jacobianOplus[6].block<3, 2>(3, 0) = -Rbw1 * dGdTheta * dt;
    // ep对重力方向扰动雅可比3×2
    _jacobianOplus[6].block<3, 2>(6, 0) = -0.5 * Rbw1 * dGdTheta * dt * dt;

    // ========== 顶点7：尺度因子Scale(1维) ==========
    _jacobianOplus[7].setZero();
    // ev对尺度s的雅可比3×1
    _jacobianOplus[7].block<3, 1>(3, 0) = Rbw1 * (VV2->estimate() - VV1->estimate());
    // ep对尺度s的雅可比3×1
    _jacobianOplus[7].block<3, 1>(6, 0) = Rbw1 * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt);
}

//-------------------------------------------------------------------
/**
 * @brief IMU位姿速度bias先验边，给定约束，对P V Bg Ba施加先验约束
 * @param c 先验约束结构体，包含参考P,V,Bg,Ba以及先验信息矩阵H
 */
EdgePriorPoseImu::EdgePriorPoseImu(ConstraintPoseImu* c)
{
    // 绑定4个顶点：Pose, Velocity, GyroBias, AccBias
    resize(4);
    // 拷贝先验参考值
    Rwb = c->Rwb;
    twb = c->twb;
    vwb = c->vwb;
    bg = c->bg;
    ba = c->ba;
    // 设置先验信息矩阵
    setInformation(c->H);
}

/**
 * @brief IMU先验边计算残差：旋转、平移、速度、陀螺bias、加速度bias残差
 */
void EdgePriorPoseImu::computeError()
{
    // 顶点0：位姿顶点
    const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
    // 顶点1：速度顶点
    const VertexVelocity* VV = static_cast<const VertexVelocity*>(_vertices[1]);
    // 顶点2：陀螺bias顶点
    const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
    // 顶点3：加速度bias顶点
    const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);

    // 旋转残差 er = LogSO3(Rwb_ref^T * Rwb_current)
    const Eigen::Vector3d er = LogSO3(Rwb.transpose() * VP->estimate().Rwb);
    // 平移残差 et = Rwb_ref^T*(twb_current‑twb_ref)
    const Eigen::Vector3d et = Rwb.transpose() * (VP->estimate().twb - twb);
    // 速度残差 ev = V_current‑V_ref
    const Eigen::Vector3d ev = VV->estimate() - vwb;
    // 陀螺bias残差 ebg = bg_current‑bg_ref
    const Eigen::Vector3d ebg = VG->estimate() - bg;
    // 加速度bias残差 eba = ba_current‑ba_ref
    const Eigen::Vector3d eba = VA->estimate() - ba;

    // 拼接残差 [er;et;ev;ebg;eba]，总15维
    _error << er, et, ev, ebg, eba;
}

/**
 * @brief IMU先验边线性化，计算各个顶点雅可比矩阵
 */
void EdgePriorPoseImu::linearizeOplus()
{
    // 获取位姿顶点
    const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
    // 计算旋转残差er
    const Eigen::Vector3d er = LogSO3(Rwb.transpose() * VP->estimate().Rwb);

    // ========== 顶点0 Pose(6维) ==========
    _jacobianOplus[0].setZero();
    // er对位姿旋转的雅可比：InverseRightJacobianSO3(er)
    _jacobianOplus[0].block<3, 3>(0, 0) = InverseRightJacobianSO3(er);
    // et对位姿平移的雅可比 Rwb_ref^T * Rwb_current
    _jacobianOplus[0].block<3, 3>(3, 3) = Rwb.transpose() * VP->estimate().Rwb;

    // ========== 顶点1 Velocity(3维) ==========
    _jacobianOplus[1].setZero();
    // ev对速度顶点雅可比为单位矩阵
    _jacobianOplus[1].block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();

    // ========== 顶点2 GyroBias(3维) ==========
    _jacobianOplus[2].setZero();
    // ebg对陀螺bias雅可比单位矩阵
    _jacobianOplus[2].block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();

    // ========== 顶点3 AccBias(3维) ==========
    _jacobianOplus[3].setZero();
    // eba对加速度bias雅可比单位矩阵
    _jacobianOplus[3].block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();
}

/**
 * @brief 加速度计bias先验边，雅可比，误差对bias导数为单位阵
 */
void EdgePriorAcc::linearizeOplus()
{
    // Jacobian wrt bias
    _jacobianOplusXi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
}

/**
 * @brief 陀螺仪bias先验边，雅可比，误差对bias导数为单位阵
 */
void EdgePriorGyro::linearizeOplus()
{
    // Jacobian wrt bias
    _jacobianOplusXi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
}

//-------------------------------------------------------------------
/**
 * @brief 4DoF约束边：仅优化yaw+平移，roll pitch固定，计算相对位姿残差
 */
void Edge4DoF::computeError()
{
    // 顶点i：4DoF位姿顶点
    const VertexPose4DoF* VPi = static_cast<const VertexPose4DoF*>(_vertices[0]);
    // 顶点j：4DoF位姿顶点
    const VertexPose4DoF* VPj = static_cast<const VertexPose4DoF*>(_vertices[1]);

    // 残差：[旋转残差LogSO3(Ri*Rj^T*dRij^T); 平移残差]
    _error << LogSO3(VPi->estimate().Rcw[0] * VPj->estimate().Rcw[0].transpose() * dRij.transpose()),
             VPi->estimate().Rcw[0] * (-VPj->estimate().Rcw[0].transpose() * VPj->estimate().tcw[0]) + VPi->estimate().tcw[0] - dtij;
}

//-------------------------------------------------------------------
// SO3 FUNCTIONS
/**
 * @brief SO3指数映射，向量w∈R3转为旋转矩阵R=ExpSO3(w)，外层重载接口
 * @param w 旋转向量(轴角)
 * @return 旋转矩阵3×3
 */
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w)
{
    return ExpSO3(w[0], w[1], w[2]);
}

/**
 * @brief SO3指数映射实现，输入x,y,z轴角三分量，罗德里格斯公式
 * @param x 旋转向量x分量
 * @param y 旋转向量y分量
 * @param z 旋转向量z分量
 * @return 旋转矩阵R
 */
Eigen::Matrix3d ExpSO3(const double x, const double y, const double z)
{
    // 旋转向量模长平方 d² = x²+y²+z²
    const double d2 = x * x + y * y + z * z;
    // 旋转向量模长 d = ||w||，旋转角度
    const double d = sqrt(d2);
    // w的反对称矩阵W=⌊w⌋
    Eigen::Matrix3d W;
    W << 0.0, -z, y,
         z, 0.0, -x,
        -y, x, 0.0;

    // 小角度近似，d趋近0，泰勒展开截断二阶
    if (d < 1e-5)
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
        return NormalizeRotation(res);
    }
    else
    {
        // 罗德里格斯公式 R = I + sin(d)/d * W + (1‑cos(d))/d² * W²
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W * sin(d) / d + W * W * (1.0 - cos(d)) / d2;
        return NormalizeRotation(res);
    }
}

/**
 * @brief SO3对数映射，旋转矩阵R转为旋转向量w=LogSO3(R)
 * @param R 输入旋转矩阵
 * @return 旋转向量w∈R3
 */
Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R)
{
    // 旋转矩阵迹 tr(R)
    const double tr = R(0, 0) + R(1, 1) + R(2, 2);
    // 提取反对称部分，得到未缩放的旋转向量
    Eigen::Vector3d w;
    w << (R(2, 1) - R(1, 2)) / 2,
         (R(0, 2) - R(2, 0)) / 2,
         (R(1, 0) - R(0, 1)) / 2;

    // cosθ = (tr‑1)/2
    const double costheta = (tr - 1.0) * 0.5f;
    // 数值异常保护，cosθ超出[-1,1]直接返回原始w
    if (costheta > 1 || costheta < -1)
        return w;
    // 旋转角度θ
    const double theta = acos(costheta);
    const double s = sin(theta);
    // sinθ接近0，小角度直接返回w
    if (fabs(s) < 1e-5)
        return w;
    else
        // w = θ / sinθ * w_raw
        return theta * w / s;
}

/**
 * @brief SO3右雅可比逆矩阵重载接口，输入Eigen向量v
 * @param v 旋转向量
 * @return Jr⁻¹(v) 3×3矩阵
 */
Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v)
{
    return InverseRightJacobianSO3(v[0], v[1], v[2]);
}

/**
 * @brief SO3右雅可比逆矩阵实现 Jr⁻¹(v)
 * @param x,y,z 旋转向量三分量
 * @return 右雅可比逆矩阵
 */
// 计算SO(3)流形上的右逆雅可比矩阵 J_r^{-1}(φ)
// 输入：旋转向量φ的三个分量 x,y,z，对应角轴表示的旋转向量
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y,
                                         const double z) {
  // d2: 旋转向量φ的模长的平方，θ² = x²+y²+z²
  const double d2 = x * x + y * y + z * z;
  // d: 旋转向量φ的模长，也就是旋转角θ
  const double d = sqrt(d2);

  // W: 旋转向量对应的反对称矩阵(斜对称矩阵) [φ]_×
  Eigen::Matrix3d W;
  W << 0.0, -z, y,
       z, 0.0, -x,
      -y, x, 0.0;

  // 当旋转角θ趋近于0时，小角度近似，右逆雅可比退化为单位矩阵
  if (d < 1e-5)
    return Eigen::Matrix3d::Identity();
  else
    // 右逆雅可比闭式公式：J_r^{-1}(φ) = I + 1/2[φ]_× + [φ]_×² (1/θ² − (1+cosθ)/(2θ sinθ))
      return Eigen::Matrix3d::Identity() + W / 2 +
             W * W * (1.0 / d2 - (1.0 + cos(d)) / (2.0 * d * sin(d)));
}

// 重载版本：接收Eigen三维向量形式的旋转向量v，转发给底层三double参数实现
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v) {
  return RightJacobianSO3(v[0], v[1], v[2]);
}

// 计算SO(3)流形上的右雅可比矩阵 J_r(φ)
// 输入：旋转向量φ的三个分量 x,y,z，角轴形式旋转向量
Eigen::Matrix3d RightJacobianSO3(const double x, const double y,
                                  const double z) {
  // d2: 旋转向量模长平方 θ² = x²+y²+z²
  const double d2 = x * x + y * y + z * z;
  // d: 旋转向量模长，旋转角度θ
  const double d = sqrt(d2);

  // W: 旋转向量对应的反对称斜对称矩阵 [φ]_×
  Eigen::Matrix3d W;
  W << 0.0, -z, y,
       z, 0.0, -x,
      -y, x, 0.0;

  // 旋转角θ趋近于0，小角度近似，右雅可比退化为单位矩阵
  if (d < 1e-5) {
    return Eigen::Matrix3d::Identity();
  } else {
    // SO3右雅可比闭式公式：J_r(φ)=I − [φ]_×*(1−cosθ)/θ² + [φ]_×²*(θ−sinθ)/(θ²θ)
    return Eigen::Matrix3d::Identity() - W * (1.0 - cos(d)) / d2 +
           W * W * (d - sin(d)) / (d2 * d);
  }
}

// 将三维向量w转换为对应的3×3反对称斜对称矩阵 [w]_×
// 用于实现叉乘等价矩阵运算：[w]_× * u = w × u
Eigen::Matrix3d Skew(const Eigen::Vector3d& w) {
  Eigen::Matrix3d W;
  W << 0.0, -w[2], w[1],
       w[2], 0.0, -w[0],
      -w[1], w[0], 0.0;
  return W;
}

}  // namespace ORB_SLAM3
