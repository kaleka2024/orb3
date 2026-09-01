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

#include <memory>
#include <vector>

#include "Converter.h"
#include "G2oTypes.h"
#include "ImuTypes.h"

namespace ORB_SLAM3 {

//-------------------------------------------------------------------

void EdgeMono::linearizeOplus() {
  const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
  const g2o::VertexPointXYZ* VPoint =
      static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * VPoint->estimate() + tcw;
  const Eigen::Vector3d Xb =
      VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

  const Eigen::Matrix<double, 2, 3> proj_jac =
      VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
  _jacobianOplusXi = -proj_jac * Rcw;

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);

  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0,
      0.0, 0.0, 1.0;

  _jacobianOplusXj = proj_jac * Rcb * SE3deriv;  // TODO optimize this product
}

//-------------------------------------------------------------------

void EdgeMonoOnlyPose::linearizeOplus() {
  const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * Xw + tcw;
  const Eigen::Vector3d Xb =
      VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

  Eigen::Matrix<double, 2, 3> proj_jac =
      VPose->estimate().pCamera[cam_idx]->projectJac(Xc);

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0,
      0.0, 0.0, 1.0;
  _jacobianOplusXi =
      proj_jac * Rcb * SE3deriv;  // symbol different becasue of update mode
}

void EdgeStereo::linearizeOplus() {
  const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
  const g2o::VertexPointXYZ* VPoint =
      static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * VPoint->estimate() + tcw;
  const Eigen::Vector3d Xb =
      VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];
  const double bf = VPose->estimate().bf;
  const double inv_z2 = 1.0 / (Xc(2) * Xc(2));

  Eigen::Matrix<double, 3, 3> proj_jac;
  proj_jac.block<2, 3>(0, 0) =
      VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
  proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
  proj_jac(2, 2) += bf * inv_z2;

  _jacobianOplusXi = -proj_jac * Rcw;

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);

  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0,
      0.0, 0.0, 1.0;

  _jacobianOplusXj = proj_jac * Rcb * SE3deriv;
}

void EdgeStereoOnlyPose::linearizeOplus() {
  const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * Xw + tcw;
  const Eigen::Vector3d Xb =
      VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];
  const double bf = VPose->estimate().bf;
  const double inv_z2 = 1.0 / (Xc(2) * Xc(2));

  Eigen::Matrix<double, 3, 3> proj_jac;
  proj_jac.block<2, 3>(0, 0) =
      VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
  proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
  proj_jac(2, 2) += bf * inv_z2;

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0,
      0.0, 0.0, 1.0;
  _jacobianOplusXi = proj_jac * Rcb * SE3deriv;
}

//-------------------------------------------------------------------

VertexVelocity::VertexVelocity(const std::shared_ptr<KeyFrame>& pKF) {
  setEstimate(pKF->GetVelocity().cast<double>());
}

VertexVelocity::VertexVelocity(const std::shared_ptr<Frame>& pF) {
  setEstimate(pF->GetVelocity().cast<double>());
}

VertexGyroBias::VertexGyroBias(const std::shared_ptr<KeyFrame>& pKF) {
  setEstimate(pKF->GetGyroBias().cast<double>());
}

VertexGyroBias::VertexGyroBias(const std::shared_ptr<Frame>& pF) {
  Eigen::Vector3d bg;
  bg << pF->mImuBias.bwx, pF->mImuBias.bwy, pF->mImuBias.bwz;
  setEstimate(bg);
}

VertexAccBias::VertexAccBias(const std::shared_ptr<KeyFrame>& pKF) {
  setEstimate(pKF->GetAccBias().cast<double>());
}

VertexAccBias::VertexAccBias(const std::shared_ptr<Frame>& pF) {
  Eigen::Vector3d ba;
  ba << pF->mImuBias.bax, pF->mImuBias.bay, pF->mImuBias.baz;
  setEstimate(ba);
}

//-------------------------------------------------------------------

EdgeInertial::EdgeInertial(const std::shared_ptr<IMU::Preintegrated>& pInt)
    : JRg(pInt->JRg.cast<double>()),
      JVg(pInt->JVg.cast<double>()),
      JPg(pInt->JPg.cast<double>()),
      JVa(pInt->JVa.cast<double>()),
      JPa(pInt->JPa.cast<double>()),
      mpInt(pInt),
      dt(pInt->dT) {
  // This edge links 6 vertices
  resize(6);
  g << 0, 0, -IMU::GRAVITY_VALUE;

  Matrix9d Info = pInt->C.block<9, 9>(0, 0).cast<double>().inverse();
  Info = (Info + Info.transpose()) / 2;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9> > es(Info);
  Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
  for (int i = 0; i < 9; i++)
    if (eigs[i] < 1e-12) eigs[i] = 0;
  Info = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
  setInformation(Info);
}

void EdgeInertial::computeError() {
  // TODO Maybe Reintegrate inertial measurments when difference between
  // linearization point and current estimate is too big
  const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyroBias* VG1 = static_cast<const VertexGyroBias*>(_vertices[2]);
  const VertexAccBias* VA1 = static_cast<const VertexAccBias*>(_vertices[3]);
  const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
  const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
  const IMU::Bias b1(VA1->estimate()[0], VA1->estimate()[1], VA1->estimate()[2],
                     VG1->estimate()[0], VG1->estimate()[1],
                     VG1->estimate()[2]);
  const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b1).cast<double>();
  const Eigen::Vector3d dV = mpInt->GetDeltaVelocity(b1).cast<double>();
  const Eigen::Vector3d dP = mpInt->GetDeltaPosition(b1).cast<double>();

  const Eigen::Vector3d er = LogSO3(
      dR.transpose() * VP1->estimate().Rwb.transpose() * VP2->estimate().Rwb);
  const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose() *
                                 (VV2->estimate() - VV1->estimate() - g * dt) -
                             dV;
  const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose() *
                                 (VP2->estimate().twb - VP1->estimate().twb -
                                  VV1->estimate() * dt - g * dt * dt / 2) -
                             dP;

  _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus() {
  const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyroBias* VG1 = static_cast<const VertexGyroBias*>(_vertices[2]);
  const VertexAccBias* VA1 = static_cast<const VertexAccBias*>(_vertices[3]);
  const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
  const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
  const IMU::Bias b1(VA1->estimate()[0], VA1->estimate()[1], VA1->estimate()[2],
                     VG1->estimate()[0], VG1->estimate()[1],
                     VG1->estimate()[2]);
  const IMU::Bias db = mpInt->GetDeltaBias(b1);
  Eigen::Vector3d dbg;
  dbg << db.bwx, db.bwy, db.bwz;

  const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
  const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
  const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;

  const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b1).cast<double>();
  const Eigen::Matrix3d eR = dR.transpose() * Rbw1 * Rwb2;
  const Eigen::Vector3d er = LogSO3(eR);
  const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

  // Jacobians wrt Pose 1
  _jacobianOplus[0].setZero();
  // rotation
  _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1;  // OK
  _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(
      Rbw1 * (VV2->estimate() - VV1->estimate() - g * dt));  // OK
  _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat(
      Rbw1 * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt -
              0.5 * g * dt * dt));  // OK
  // translation
  _jacobianOplus[0].block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();  // OK

  // Jacobians wrt Velocity 1
  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(3, 0) = -Rbw1;       // OK
  _jacobianOplus[1].block<3, 3>(6, 0) = -Rbw1 * dt;  // OK

  // Jacobians wrt Gyro 1
  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(0, 0) =
      -invJr * eR.transpose() * RightJacobianSO3(JRg * dbg) * JRg;  // OK
  _jacobianOplus[2].block<3, 3>(3, 0) = -JVg;                       // OK
  _jacobianOplus[2].block<3, 3>(6, 0) = -JPg;                       // OK

  // Jacobians wrt Accelerometer 1
  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(3, 0) = -JVa;  // OK
  _jacobianOplus[3].block<3, 3>(6, 0) = -JPa;  // OK

  // Jacobians wrt Pose 2
  _jacobianOplus[4].setZero();
  // rotation
  _jacobianOplus[4].block<3, 3>(0, 0) = invJr;  // OK
  // translation
  _jacobianOplus[4].block<3, 3>(6, 3) = Rbw1 * Rwb2;  // OK

  // Jacobians wrt Velocity 2
  _jacobianOplus[5].setZero();
  _jacobianOplus[5].block<3, 3>(3, 0) = Rbw1;  // OK
}

//-------------------------------------------------------------------

EdgeInertialGS::EdgeInertialGS(const std::shared_ptr<IMU::Preintegrated>& pInt)
    : JRg(pInt->JRg.cast<double>()),
      JVg(pInt->JVg.cast<double>()),
      JPg(pInt->JPg.cast<double>()),
      JVa(pInt->JVa.cast<double>()),
      JPa(pInt->JPa.cast<double>()),
      mpInt(pInt),
      dt(pInt->dT) {
  // This edge links 8 vertices
  resize(8);
  gI << 0, 0, -IMU::GRAVITY_VALUE;

  Matrix9d Info = pInt->C.block<9, 9>(0, 0).cast<double>().inverse();
  Info = (Info + Info.transpose()) / 2;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9> > es(Info);
  Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
  for (int i = 0; i < 9; i++)
    if (eigs[i] < 1e-12) eigs[i] = 0;
  Info = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
  setInformation(Info);
}

void EdgeInertialGS::computeError() {
  // TODO Maybe Reintegrate inertial measurments when difference between
  // linearization point and current estimate is too big
  const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
  const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);
  const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
  const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
  const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
  const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);
  const IMU::Bias b(VA->estimate()[0], VA->estimate()[1], VA->estimate()[2],
                    VG->estimate()[0], VG->estimate()[1], VG->estimate()[2]);
  g = VGDir->estimate().Rwg * gI;
  const double s = VS->estimate();
  const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b).cast<double>();
  const Eigen::Vector3d dV = mpInt->GetDeltaVelocity(b).cast<double>();
  const Eigen::Vector3d dP = mpInt->GetDeltaPosition(b).cast<double>();

  const Eigen::Vector3d er = LogSO3(
      dR.transpose() * VP1->estimate().Rwb.transpose() * VP2->estimate().Rwb);
  const Eigen::Vector3d ev =
      VP1->estimate().Rwb.transpose() *
          (s * (VV2->estimate() - VV1->estimate()) - g * dt) -
      dV;
  const Eigen::Vector3d ep =
      VP1->estimate().Rwb.transpose() *
          (s * (VP2->estimate().twb - VP1->estimate().twb -
                VV1->estimate() * dt) -
           g * dt * dt / 2) -
      dP;

  _error << er, ev, ep;
}

void EdgeInertialGS::linearizeOplus() {
  const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
  const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);
  const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
  const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
  const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
  const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);
  const IMU::Bias b(VA->estimate()[0], VA->estimate()[1], VA->estimate()[2],
                    VG->estimate()[0], VG->estimate()[1], VG->estimate()[2]);
  const IMU::Bias db = mpInt->GetDeltaBias(b);

  Eigen::Vector3d dbg;
  dbg << db.bwx, db.bwy, db.bwz;

  const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
  const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
  const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;
  const Eigen::Matrix3d Rwg = VGDir->estimate().Rwg;
  Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(3, 2);
  Gm(0, 1) = -IMU::GRAVITY_VALUE;
  Gm(1, 0) = IMU::GRAVITY_VALUE;
  const double s = VS->estimate();
  const Eigen::MatrixXd dGdTheta = Rwg * Gm;
  const Eigen::Matrix3d dR = mpInt->GetDeltaRotation(b).cast<double>();
  const Eigen::Matrix3d eR = dR.transpose() * Rbw1 * Rwb2;
  const Eigen::Vector3d er = LogSO3(eR);
  const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

  // Jacobians wrt Pose 1
  _jacobianOplus[0].setZero();
  // rotation
  _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1;
  _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(
      Rbw1 * (s * (VV2->estimate() - VV1->estimate()) - g * dt));
  _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat(
      Rbw1 *
      (s * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt) -
       0.5 * g * dt * dt));
  // translation
  _jacobianOplus[0].block<3, 3>(6, 3) =
      Eigen::DiagonalMatrix<double, 3>(-s, -s, -s);

  // Jacobians wrt Velocity 1
  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(3, 0) = -s * Rbw1;
  _jacobianOplus[1].block<3, 3>(6, 0) = -s * Rbw1 * dt;

  // Jacobians wrt Gyro bias
  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(0, 0) =
      -invJr * eR.transpose() * RightJacobianSO3(JRg * dbg) * JRg;
  _jacobianOplus[2].block<3, 3>(3, 0) = -JVg;
  _jacobianOplus[2].block<3, 3>(6, 0) = -JPg;

  // Jacobians wrt Accelerometer bias
  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(3, 0) = -JVa;
  _jacobianOplus[3].block<3, 3>(6, 0) = -JPa;

  // Jacobians wrt Pose 2
  _jacobianOplus[4].setZero();
  // rotation
  _jacobianOplus[4].block<3, 3>(0, 0) = invJr;
  // translation
  _jacobianOplus[4].block<3, 3>(6, 3) = s * Rbw1 * Rwb2;

  // Jacobians wrt Velocity 2
  _jacobianOplus[5].setZero();
  _jacobianOplus[5].block<3, 3>(3, 0) = s * Rbw1;

  // Jacobians wrt Gravity direction
  _jacobianOplus[6].setZero();
  _jacobianOplus[6].block<3, 2>(3, 0) = -Rbw1 * dGdTheta * dt;
  _jacobianOplus[6].block<3, 2>(6, 0) = -0.5 * Rbw1 * dGdTheta * dt * dt;

  // Jacobians wrt scale factor
  _jacobianOplus[7].setZero();
  _jacobianOplus[7].block<3, 1>(3, 0) =
      Rbw1 * (VV2->estimate() - VV1->estimate());
  _jacobianOplus[7].block<3, 1>(6, 0) =
      Rbw1 * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt);
}

//-------------------------------------------------------------------

EdgePriorPoseImu::EdgePriorPoseImu(ConstraintPoseImu* c) {
  resize(4);
  Rwb = c->Rwb;
  twb = c->twb;
  vwb = c->vwb;
  bg = c->bg;
  ba = c->ba;
  setInformation(c->H);
}

void EdgePriorPoseImu::computeError() {
  const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
  const VertexVelocity* VV = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
  const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);

  const Eigen::Vector3d er = LogSO3(Rwb.transpose() * VP->estimate().Rwb);
  const Eigen::Vector3d et = Rwb.transpose() * (VP->estimate().twb - twb);
  const Eigen::Vector3d ev = VV->estimate() - vwb;
  const Eigen::Vector3d ebg = VG->estimate() - bg;
  const Eigen::Vector3d eba = VA->estimate() - ba;

  _error << er, et, ev, ebg, eba;
}

void EdgePriorPoseImu::linearizeOplus() {
  const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
  const Eigen::Vector3d er = LogSO3(Rwb.transpose() * VP->estimate().Rwb);
  _jacobianOplus[0].setZero();
  _jacobianOplus[0].block<3, 3>(0, 0) = InverseRightJacobianSO3(er);
  _jacobianOplus[0].block<3, 3>(3, 3) = Rwb.transpose() * VP->estimate().Rwb;
  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();
}

void EdgePriorAcc::linearizeOplus() {
  // Jacobian wrt bias
  _jacobianOplusXi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
}

void EdgePriorGyro::linearizeOplus() {
  // Jacobian wrt bias
  _jacobianOplusXi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
}

//-------------------------------------------------------------------

void Edge4DoF::computeError() {
  const VertexPose4DoF* VPi = static_cast<const VertexPose4DoF*>(_vertices[0]);
  const VertexPose4DoF* VPj = static_cast<const VertexPose4DoF*>(_vertices[1]);
  _error << LogSO3(VPi->estimate().Rcw[0] * VPj->estimate().Rcw[0].transpose() *
                   dRij.transpose()),
      VPi->estimate().Rcw[0] *
              (-VPj->estimate().Rcw[0].transpose() * VPj->estimate().tcw[0]) +
          VPi->estimate().tcw[0] - dtij;
}

//-------------------------------------------------------------------

// SO3 FUNCTIONS
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w) {
  return ExpSO3(w[0], w[1], w[2]);
}

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z) {
  const double d2 = x * x + y * y + z * z;
  const double d = sqrt(d2);
  Eigen::Matrix3d W;
  W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
  if (d < 1e-5) {
    Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
    return NormalizeRotation(res);
  } else {
    Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W * sin(d) / d +
                          W * W * (1.0 - cos(d)) / d2;
    return NormalizeRotation(res);
  }
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R) {
  const double tr = R(0, 0) + R(1, 1) + R(2, 2);
  Eigen::Vector3d w;
  w << (R(2, 1) - R(1, 2)) / 2, (R(0, 2) - R(2, 0)) / 2,
      (R(1, 0) - R(0, 1)) / 2;
  const double costheta = (tr - 1.0) * 0.5f;
  if (costheta > 1 || costheta < -1) return w;
  const double theta = acos(costheta);
  const double s = sin(theta);
  if (fabs(s) < 1e-5)
    return w;
  else
    return theta * w / s;
}

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v) {
  return InverseRightJacobianSO3(v[0], v[1], v[2]);
}

Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y,
                                        const double z) {
  const double d2 = x * x + y * y + z * z;
  const double d = sqrt(d2);

  Eigen::Matrix3d W;
  W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
  if (d < 1e-5)
    return Eigen::Matrix3d::Identity();
  else
    return Eigen::Matrix3d::Identity() + W / 2 +
           W * W * (1.0 / d2 - (1.0 + cos(d)) / (2.0 * d * sin(d)));
}

Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v) {
  return RightJacobianSO3(v[0], v[1], v[2]);
}

Eigen::Matrix3d RightJacobianSO3(const double x, const double y,
                                 const double z) {
  const double d2 = x * x + y * y + z * z;
  const double d = sqrt(d2);

  Eigen::Matrix3d W;
  W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
  if (d < 1e-5) {
    return Eigen::Matrix3d::Identity();
  } else {
    return Eigen::Matrix3d::Identity() - W * (1.0 - cos(d)) / d2 +
           W * W * (d - sin(d)) / (d2 * d);
  }
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& w) {
  Eigen::Matrix3d W;
  W << 0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0;
  return W;
}

}  // namespace ORB_SLAM3
