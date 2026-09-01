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
// C++智能指针头文件，用于std::shared_ptr
#include <memory>
// C++动态数组容器头文件
#include <vector>

// ORB‑SLAM3类型转换工具头文件
#include "Converter.h"
// g2o图优化自定义顶点边类型头文件
#include "G2oTypes.h"
// IMU相关数据结构、标定参数定义头文件
#include "ImuTypes.h"

// ORB‑SLAM3顶层命名空间
namespace ORB_SLAM3 {

/**
 * @brief ImuCamPose构造函数，由关键帧KeyFrame初始化IMU‑相机位姿结构体
 * @param pKF 输入关键帧智能指针
 * @note its 迭代计数，用于控制旋转矩阵正交归一化频率，初始化为0
 */
ImuCamPose::ImuCamPose(const std::shared_ptr<KeyFrame>& pKF) : its(0) {
  // Load IMU pose
  // twb: IMU本体坐标系原点在世界坐标系下的平移向量，从关键帧读取IMU位置，转换为double精度
  twb = pKF->GetImuPosition().cast<double>();
  // Rwb: IMU本体到世界坐标系的旋转矩阵 R_wb，世界=Rwb*本体，读取关键帧IMU旋转并转double
  Rwb = pKF->GetImuRotation().cast<double>();

  // Load camera poses
  // 判断相机数量，单目=1，双目/双相机=2
  int num_cams;
  if (pKF->mpCamera2)
    num_cams = 2;
  else
    num_cams = 1;

  // tcw[i]: 第i号相机坐标系原点在世界坐标系下平移 t_cw
  tcw.resize(num_cams);
  // Rcw[i]: 第i号相机到世界旋转矩阵 R_cw，世界点转到相机坐标系：Xc = Rcw*Xw + tcw
  Rcw.resize(num_cams);
  // tcb[i]: IMU本体坐标系到第i号相机坐标系外参平移 t_cb，相机 = Tcb * IMU
  tcb.resize(num_cams);
  // Rcb[i]: IMU本体到第i号相机外参旋转 R_cb
  Rcb.resize(num_cams);
  // Rbc[i]: 第i号相机到IMU本体旋转 R_bc = Rcb^T
  Rbc.resize(num_cams);
  // tbc[i]: 第i号相机到IMU本体平移 t_bc
  tbc.resize(num_cams);
  // pCamera[i]: 第i号相机模型智能指针
  pCamera.resize(num_cams);

  // Left camera
  // 读取左主相机世界‑相机平移 t_cw
  tcw[0] = pKF->GetTranslation().cast<double>();
  // 读取左主相机世界‑相机旋转 R_cw
  Rcw[0] = pKF->GetRotation().cast<double>();
  // 读取IMU标定外参Tcb的平移部分，IMU→相机平移
  tcb[0] = pKF->mImuCalib.mTcb.translation().cast<double>();
  // 读取IMU标定外参Tcb的旋转矩阵，IMU→相机旋转
  Rcb[0] = pKF->mImuCalib.mTcb.rotationMatrix().cast<double>();
  // Rbc = Rcb转置，相机→IMU旋转矩阵
  Rbc[0] = Rcb[0].transpose();
  // 读取预存的Tbc外参平移，相机→IMU平移
  tbc[0] = pKF->mImuCalib.mTbc.translation().cast<double>();
  // 绑定左相机模型指针
  pCamera[0] = pKF->mpCamera;
  // 读取双目基线bf = baseline * fx
  bf = pKF->mbf;

  // 双目/双相机分支，处理右相机
  if (num_cams > 1) {
    // Trl：右相机相对于左相机的位姿变换矩阵T_rl，左→右，转为double矩阵
    Eigen::Matrix4d Trl = pKF->GetRelativePoseTrl().matrix().cast<double>();
    // 右相机Rcw[1] = T_rl旋转 * 左相机Rcw[0]
    Rcw[1] = Trl.block<3, 3>(0, 0) * Rcw[0];
    // 右相机tcw[1] = T_rl旋转*左tcw + T_rl平移
    tcw[1] = Trl.block<3, 3>(0, 0) * tcw[0] + Trl.block<3, 1>(0, 3);
    // 右相机tcb[1] = T_rl旋转*左tcb + T_rl平移，IMU到右相机平移
    tcb[1] = Trl.block<3, 3>(0, 0) * tcb[0] + Trl.block<3, 1>(0, 3);
    // 右相机Rcb[1] = T_rl旋转 * 左Rcb[0]，IMU到右相机旋转
    Rcb[1] = Trl.block<3, 3>(0, 0) * Rcb[0];
    // Rbc[1]取转置，右相机到IMU旋转
    Rbc[1] = Rcb[1].transpose();
    // tbc[1] = -Rbc[1] * tcb[1]，右相机到IMU平移，由外参变换推导
    tbc[1] = -Rbc[1] * tcb[1];
    // 绑定右相机模型指针
    pCamera[1] = pKF->mpCamera2;
  }

  // For posegraph 4DoF
  // Rwb0保存初始IMU世界旋转，用于4自由度位姿图优化
  Rwb0 = Rwb;
  // DR增量旋转矩阵初始化为单位阵，4DoF优化累积旋转增量
  DR.setIdentity();
}

/**
 * @brief ImuCamPose构造函数，由普通帧Frame初始化IMU‑相机位姿结构体
 * @param pF 输入普通帧智能指针
 * @note its迭代计数初始0，用于控制旋转归一化
 */
ImuCamPose::ImuCamPose(const std::shared_ptr<Frame>& pF) : its(0) {
  // Load IMU pose
  // 获取Frame中IMU在世界坐标系平移twb，转double
  twb = pF->GetImuPosition().cast<double>();
  // 获取Frame中IMU到世界旋转Rwb，转double
  Rwb = pF->GetImuRotation().cast<double>();

  // Load camera poses
  // 判断相机数量，单目1，双目2
  int num_cams;
  if (pF->mpCamera2)
    num_cams = 2;
  else
    num_cams = 1;

  // 调整各相机位姿、外参容器大小
  tcw.resize(num_cams);
  Rcw.resize(num_cams);
  tcb.resize(num_cams);
  Rcb.resize(num_cams);
  Rbc.resize(num_cams);
  tbc.resize(num_cams);
  pCamera.resize(num_cams);

  // Left camera
  // 从Frame位姿提取左相机tcw平移
  tcw[0] = pF->GetPose().translation().cast<double>();
  // 从Frame位姿提取左相机Rcw旋转矩阵
  Rcw[0] = pF->GetPose().rotationMatrix().cast<double>();
  // IMU标定外参Tcb平移，IMU→相机
  tcb[0] = pF->mImuCalib.mTcb.translation().cast<double>();
  // IMU标定外参Tcb旋转矩阵，IMU→相机
  Rcb[0] = pF->mImuCalib.mTcb.rotationMatrix().cast<double>();
  // Rbc = Rcb转置，相机→IMU旋转
  Rbc[0] = Rcb[0].transpose();
  // 读取预存Tbc平移，相机→IMU
  tbc[0] = pF->mImuCalib.mTbc.translation().cast<double>();
  // 绑定左相机模型
  pCamera[0] = pF->mpCamera;
  // 读取帧的bf基线参数
  bf = pF->mbf;

  // 双目双相机，计算右相机全套位姿与外参
  if (num_cams > 1) {
    // Trl 左到右相机相对位姿矩阵，转为double
    Eigen::Matrix4d Trl = pF->GetRelativePoseTrl().matrix().cast<double>();
    // 右相机Rcw = Trl旋转 * 左Rcw
    Rcw[1] = Trl.block<3, 3>(0, 0) * Rcw[0];
    // 右相机tcw = Trl旋转*左tcw + Trl平移
    tcw[1] = Trl.block<3, 3>(0, 0) * tcw[0] + Trl.block<3, 1>(0, 3);
    // IMU到右相机平移tcb
    tcb[1] = Trl.block<3, 3>(0, 0) * tcb[0] + Trl.block<3, 1>(0, 3);
    // IMU到右相机旋转Rcb
    Rcb[1] = Trl.block<3, 3>(0, 0) * Rcb[0];
    // 右相机到IMU旋转Rbc取转置
    Rbc[1] = Rcb[1].transpose();
    // 右相机到IMU平移tbc，坐标变换推导
    tbc[1] = -Rbc[1] * tcb[1];
    // 绑定右相机模型指针
    pCamera[1] = pF->mpCamera2;
  }

  // For posegraph 4DoF
  // 保存IMU初始旋转用于4DoF位姿图优化
  Rwb0 = Rwb;
  // 累积旋转增量DR初始化为单位矩阵
  DR.setIdentity();
}

/**
 * @brief ImuCamPose构造函数，仅用于位姿图posegraph，由相机世界位姿反推IMU位姿
 * @param _Rwc 相机到世界旋转矩阵 R_wc
 * @param _twc 相机原点在世界坐标系平移 t_wc
 * @param pKF 参考关键帧，读取IMU‑相机外参
 * @note 该构造不处理多相机，只单相机
 */
ImuCamPose::ImuCamPose(Eigen::Matrix3d& _Rwc, Eigen::Vector3d& _twc,
                       const std::shared_ptr<KeyFrame>& pKF)
    : its(0) {
  // This is only for posegrpah, we do not care about multicamera
  // 仅单相机，容器resize为1
  tcw.resize(1);
  Rcw.resize(1);
  tcb.resize(1);
  Rcb.resize(1);
  Rbc.resize(1);
  tbc.resize(1);
  pCamera.resize(1);

  // 从关键帧读取IMU→相机外参平移tcb
  tcb[0] = pKF->mImuCalib.mTcb.translation().cast<double>();
  // 从关键帧读取IMU→相机外参旋转Rcb
  Rcb[0] = pKF->mImuCalib.mTcb.rotationMatrix().cast<double>();
  // Rbc = Rcb转置，相机→IMU旋转
  Rbc[0] = Rcb[0].transpose();
  // 读取预存Tbc相机→IMU平移
  tbc[0] = pKF->mImuCalib.mTbc.translation().cast<double>();
  // 反算IMU世界平移 twb = Rwc * tcb + twc
  twb = _Rwc * tcb[0] + _twc;
  // 反算IMU世界旋转 Rwb = Rwc * Rcb
  Rwb = _Rwc * Rcb[0];
  // Rcw = Rwc转置，世界点转到相机坐标系旋转
  Rcw[0] = _Rwc.transpose();
  // tcw = -Rcw * _twc，世界到相机平移
  tcw[0] = -Rcw[0] * _twc;
  // 绑定相机模型指针
  pCamera[0] = pKF->mpCamera;
  // 读取bf基线参数
  bf = pKF->mbf;

  // For posegraph 4DoF
  // 保存IMU初始旋转，用于4DoF位姿图
  Rwb0 = Rwb;
  // 累积旋转增量DR置单位阵
  DR.setIdentity();
}

/**
 * @brief SetParam 根据输入相机位姿、相机‑IMU外参，设置ImuCamPose全部成员变量
 * @param _Rcw 各相机 Rcw：世界到相机旋转
 * @param _tcw 各相机 tcw：世界到相机平移
 * @param _Rbc 各相机 Rbc：相机到IMU本体旋转
 * @param _tbc 各相机 tbc：相机到IMU本体平移
 * @param _bf 双目基线参数 bf
 */
void ImuCamPose::SetParam(const std::vector<Eigen::Matrix3d>& _Rcw,
                          const std::vector<Eigen::Vector3d>& _tcw,
                          const std::vector<Eigen::Matrix3d>& _Rbc,
                          const std::vector<Eigen::Vector3d>& _tbc,
                          const double& _bf) {
  // 赋值相机→IMU旋转Rbc
  Rbc = _Rbc;
  // 赋值相机→IMU平移tbc
  tbc = _tbc;
  // 赋值世界到相机旋转Rcw
  Rcw = _Rcw;
  // 赋值世界到相机平移tcw
  tcw = _tcw;
  // 获取相机总数量
  const int num_cams = Rbc.size();
  // resize IMU→相机外参容器
  Rcb.resize(num_cams);
  tcb.resize(num_cams);

  // 循环：由Rbc、tbc反求Rcb、tcb，Rcb=Rbc^T，tcb = -Rcb*tbc
  for (size_t i = 0; i < tcb.size(); i++) {
    Rcb[i] = Rbc[i].transpose();
    tcb[i] = -Rcb[i] * tbc[i];
  }
  // 计算IMU本体到世界旋转 Rwb = Rcw[0]^T * Rcb[0]，主相机推导IMU位姿
  Rwb = Rcw[0].transpose() * Rcb[0];
  // 计算IMU本体世界平移 twb = Rcw[0]^T * (tcb[0] - tcw[0])
  twb = Rcw[0].transpose() * (tcb[0] - tcw[0]);

  // 赋值双目bf参数
  bf = _bf;
}

/**
 * @brief Project 将世界坐标系3D点投影到指定相机像素平面，返回二维像素坐标
 * @param Xw 输入世界坐标系三维点
 * @param cam_idx 指定使用第几个相机
 * @return Eigen::Vector2d 像素坐标(u,v)
 */
Eigen::Vector2d ImuCamPose::Project(const Eigen::Vector3d& Xw,
                                     int cam_idx) const {
  // 世界点转换到相机坐标系 Xc = Rcw*Xw + tcw
  Eigen::Vector3d Xc = Rcw[cam_idx] * Xw + tcw[cam_idx];

  // 调用相机模型project函数得到像素坐标返回
  return pCamera[cam_idx]->project(Xc);
}

/**
 * @brief ProjectStereo 世界点投影到双目相机，返回三维输出 [u, v, ur]，ur为右目u坐标
 * @param Xw 世界坐标系三维点
 * @param cam_idx 相机索引
 * @return Eigen::Vector3d (u, v, ur)，ur = u − bf/Z
 */
Eigen::Vector3d ImuCamPose::ProjectStereo(const Eigen::Vector3d& Xw,
                                           int cam_idx) const {
  // 世界点变换到相机坐标系Pc
  Eigen::Vector3d Pc = Rcw[cam_idx] * Xw + tcw[cam_idx];
  Eigen::Vector3d pc;
  // 1/Z，相机深度倒数
  double invZ = 1 / Pc(2);
  // 调用相机投影得到左目像素u、v
  pc.head(2) = pCamera[cam_idx]->project(Pc);
  // 计算右目u坐标 ur = u − bf / Z
  pc(2) = pc(0) - bf * invZ;
  return pc;
}

/**
 * @brief isDepthPositive 判断世界点在指定相机中深度是否大于0，即点是否在相机前方
 * @param Xw 世界三维点
 * @param cam_idx 相机索引
 * @return true 深度>0；false深度<=0，点在相机后方
 */
bool ImuCamPose::isDepthPositive(const Eigen::Vector3d& Xw, int cam_idx) const {
  // Rcw第三行*Xw + tcw[2] 等价于Xc.z，相机坐标系Z深度值
  return (Rcw[cam_idx].row(2) * Xw + tcw[cam_idx](2)) > 0.0;
}

/**
 * @brief Update IMU本体位姿右乘更新，流形增量：旋转增量ur，平移增量ut，右扰动更新
 * @param pu 数组指针，pu[0‑2]旋转扰动ur，pu[3‑5]平移扰动ut
 * @note 每迭代3次做一次旋转矩阵正交归一化，消除数值漂移
 */
void ImuCamPose::Update(const double* pu) {
  Eigen::Vector3d ur, ut;
  // ur接收旋转扰动向量(SO3流形右扰动)
  ur << pu[0], pu[1], pu[2];
  // ut接收平移扰动向量
  ut << pu[3], pu[4], pu[5];

  // Update body pose
  // twb更新：右扰动平移，twb ← twb + Rwb * ut
  twb += Rwb * ut;
  // Rwb右乘SO3指数映射，施加旋转增量 Rwb = Rwb * Exp(ur)
  Rwb = Rwb * ExpSO3(ur);

  // Normalize rotation after 5 updates
  // 更新迭代计数
  its++;
  // 累计迭代>=3，执行旋转矩阵归一化，抑制数值误差
  if (its >= 3) {
    NormalizeRotation(Rwb);
    its = 0;
  }

  // Update camera poses
  // Rbw = Rwb转置：世界到IMU本体旋转
  const Eigen::Matrix3d Rbw = Rwb.transpose();
  // tbw = −Rbw * twb：世界到IMU本体平移
  const Eigen::Vector3d tbw = -Rbw * twb;

  // 根据IMU更新后的位姿，循环更新每一个相机的Rcw、tcw
  for (size_t i = 0; i < pCamera.size(); i++) {
    // Rcw[i] = Rcb[i] * Rbw，IMU到相机旋转 × 世界到IMU旋转，得到世界到相机旋转
    Rcw[i] = Rcb[i] * Rbw;
    // tcw[i] = Rcb[i] * tbw + tcb[i]，更新世界到相机平移
    tcw[i] = Rcb[i] * tbw + tcb[i];
  }
}

/**
 * @brief UpdateW 4DoF位姿图模式更新，世界坐标系下施加扰动，DR累积旋转增量
 * @param pu pu[0‑2]旋转扰动ur；pu[3‑5]平移扰动ut
 * @note 每5次迭代修正DR矩阵，强制部分元素置零，再做旋转归一化
 */
void ImuCamPose::UpdateW(const double* pu) {
  Eigen::Vector3d ur, ut;
  // 读取旋转扰动ur
  ur << pu[0], pu[1], pu[2];
  // 读取平移扰动ut
  ut << pu[3], pu[4], pu[5];

  // SO3指数映射得到旋转增量dR
  const Eigen::Matrix3d dR = ExpSO3(ur);
  // DR累积旋转增量 DR = dR * DR
  DR = dR * DR;
  // Rwb = DR * Rwb0；基于初始Rwb0，乘累积DR得到当前IMU世界旋转
  Rwb = DR * Rwb0;
  // Update body pose
  // twb直接加上世界坐标系平移增量ut
  twb += ut;

  // Normalize rotation after 5 updates
  // 迭代计数自增
  its++;
  // 累计迭代>=5，修正DR矩阵，消除漂移
  if (its >= 5) {
    // 强制DR矩阵部分元素置零，4DoF约束处理
    DR(0, 2) = 0.0;
    DR(1, 2) = 0.0;
    DR(2, 0) = 0.0;
    DR(2, 1) = 0.0;
    // DR矩阵做正交归一化，保证为合法旋转矩阵
    NormalizeRotation(DR);
    its = 0;
  }

  // Update camera pose
  // Rbw = Rwb转置，世界到IMU本体旋转
  const Eigen::Matrix3d Rbw = Rwb.transpose();
  // tbw = −Rbw * twb，世界到IMU本体平移
  const Eigen::Vector3d tbw = -Rbw * twb;

  // 根据IMU更新结果，循环更新各个相机Rcw与tcw
  for (size_t i = 0; i < pCamera.size(); i++) {
    Rcw[i] = Rcb[i] * Rbw;
    tcw[i] = Rcb[i] * tbw + tcb[i];
  }
}

/**
 * @brief InvDepthPoint逆深度地图点构造函数，存储逆深度参数化的路标点
 * @param _rho 逆深度 ρ = 1/Z
 * @param _u 宿主关键帧中特征点u像素坐标
 * @param _v 宿主关键帧中特征点v像素坐标
 * @param pHostKF 宿主关键帧，读取相机内参fx,fy,cx,cy,bf
 */
InvDepthPoint::InvDepthPoint(double _rho, double _u, double _v,
                              const std::shared_ptr<KeyFrame>& pHostKF)
    : u(_u),
      v(_v),
      rho(_rho),
      fx(pHostKF->fx),
      fy(pHostKF->fy),
      cx(pHostKF->cx),
      cy(pHostKF->cy),
      bf(pHostKF->mbf) {}

/**
 * @brief Update 逆深度点更新，g2o顶点增量更新，pu指向rho的增量值
 * @param pu 增量指针，*pu是逆深度扰动delta_rho
 */
void InvDepthPoint::Update(const double* pu) { rho += *pu; }

/**
 * @brief VertexPose g2o自定义位姿顶点：从输入流读取顶点状态数据
 * @param is 输入istream流
 * @return bool true读取成功；false失败
 */
bool VertexPose::read(std::istream& is) {
  std::vector<Eigen::Matrix<double, 3, 3> > Rcw;
  std::vector<Eigen::Matrix<double, 3, 1> > tcw;
  std::vector<Eigen::Matrix<double, 3, 3> > Rbc;
  std::vector<Eigen::Matrix<double, 3, 1> > tbc;

  // 获取相机数量，来自_estimate内部Rbc容器大小
  const size_t num_cams = _estimate.Rbc.size();
  // 循环逐个相机读取Rcw、tcw、Rbc、tbc、相机内参
  for (size_t idx = 0; idx < num_cams; idx++) {
    // 读取Rcw 3×3矩阵每一个元素
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) is >> Rcw[idx](i, j);
    }
    // 读取tcw三维向量
    for (int i = 0; i < 3; i++) {
      is >> tcw[idx](i);
    }

    // 读取Rbc 3×3矩阵
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) is >> Rbc[idx](i, j);
    }
    // 读取tbc三维向量
    for (int i = 0; i < 3; i++) {
      is >> tbc[idx](i);
    }

    // 读取相机模型参数，逐个赋值给相机对象
    float nextParam;
    for (size_t i = 0; i < _estimate.pCamera[idx]->size(); i++) {
      is >> nextParam;
      _estimate.pCamera[idx]->setParameter(nextParam, i);
    }
  }

  // 读取bf基线参数
  double bf;
  is >> bf;
  // 将读取全部参数设置到ImuCamPose对象_estimate
  _estimate.SetParam(Rcw, tcw, Rbc, tbc, bf);
  // 更新g2o顶点内部缓存
  updateCache();

  return true;
}

/**
 * @brief VertexPose g2o自定义位姿顶点：把顶点状态序列化写入输出流
 * @param os 输出ostream流
 * @return bool true写入成功；false失败
 */
bool VertexPose::write(std::ostream& os) const {
  // 拷贝当前_estimate内部各相机位姿、外参
  std::vector<Eigen::Matrix<double, 3, 3> > Rcw = _estimate.Rcw;
  std::vector<Eigen::Matrix<double, 3, 1> > tcw = _estimate.tcw;

  std::vector<Eigen::Matrix<double, 3, 3> > Rbc = _estimate.Rbc;
  std::vector<Eigen::Matrix<double, 3, 1> > tbc = _estimate.tbc;

  // 获取相机数目
  const int num_cams = tcw.size();

  // 循环每个相机，序列化Rcw、tcw、Rbc、tbc、相机参数
  for (int idx = 0; idx < num_cams; idx++) {
    // 输出Rcw 3×3全部元素
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) os << Rcw[idx](i, j) << " ";
    }
    // 输出tcw三维向量
    for (int i = 0; i < 3; i++) {
      os << tcw[idx](i) << " ";
    }

    // 输出Rbc 3×3矩阵
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) os << Rbc[idx](i, j) << " ";
    }
    // 输出tbc三维向量
    for (int i = 0; i < 3; i++) {
      os << tbc[idx](i) << " ";
    }

    // 输出相机模型全部参数
    for (size_t i = 0; i < _estimate.pCamera[idx]->size(); i++) {
      os << _estimate.pCamera[idx]->getParameter(i) << " ";
    }
  }

  // 输出bf基线参数
  os << _estimate.bf << " ";

  // 返回流状态是否正常
  return os.good();
}

}  // namespace ORB_SLAM3
