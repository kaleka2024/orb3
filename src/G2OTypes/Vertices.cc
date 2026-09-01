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

ImuCamPose::ImuCamPose(const std::shared_ptr<KeyFrame>& pKF) : its(0) {
  // Load IMU pose
  twb = pKF->GetImuPosition().cast<double>();
  Rwb = pKF->GetImuRotation().cast<double>();

  // Load camera poses
  int num_cams;
  if (pKF->mpCamera2)
    num_cams = 2;
  else
    num_cams = 1;

  tcw.resize(num_cams);
  Rcw.resize(num_cams);
  tcb.resize(num_cams);
  Rcb.resize(num_cams);
  Rbc.resize(num_cams);
  tbc.resize(num_cams);
  pCamera.resize(num_cams);

  // Left camera
  tcw[0] = pKF->GetTranslation().cast<double>();
  Rcw[0] = pKF->GetRotation().cast<double>();
  tcb[0] = pKF->mImuCalib.mTcb.translation().cast<double>();
  Rcb[0] = pKF->mImuCalib.mTcb.rotationMatrix().cast<double>();
  Rbc[0] = Rcb[0].transpose();
  tbc[0] = pKF->mImuCalib.mTbc.translation().cast<double>();
  pCamera[0] = pKF->mpCamera;
  bf = pKF->mbf;

  if (num_cams > 1) {
    Eigen::Matrix4d Trl = pKF->GetRelativePoseTrl().matrix().cast<double>();
    Rcw[1] = Trl.block<3, 3>(0, 0) * Rcw[0];
    tcw[1] = Trl.block<3, 3>(0, 0) * tcw[0] + Trl.block<3, 1>(0, 3);
    tcb[1] = Trl.block<3, 3>(0, 0) * tcb[0] + Trl.block<3, 1>(0, 3);
    Rcb[1] = Trl.block<3, 3>(0, 0) * Rcb[0];
    Rbc[1] = Rcb[1].transpose();
    tbc[1] = -Rbc[1] * tcb[1];
    pCamera[1] = pKF->mpCamera2;
  }

  // For posegraph 4DoF
  Rwb0 = Rwb;
  DR.setIdentity();
}

ImuCamPose::ImuCamPose(const std::shared_ptr<Frame>& pF) : its(0) {
  // Load IMU pose
  twb = pF->GetImuPosition().cast<double>();
  Rwb = pF->GetImuRotation().cast<double>();

  // Load camera poses
  int num_cams;
  if (pF->mpCamera2)
    num_cams = 2;
  else
    num_cams = 1;

  tcw.resize(num_cams);
  Rcw.resize(num_cams);
  tcb.resize(num_cams);
  Rcb.resize(num_cams);
  Rbc.resize(num_cams);
  tbc.resize(num_cams);
  pCamera.resize(num_cams);

  // Left camera
  tcw[0] = pF->GetPose().translation().cast<double>();
  Rcw[0] = pF->GetPose().rotationMatrix().cast<double>();
  tcb[0] = pF->mImuCalib.mTcb.translation().cast<double>();
  Rcb[0] = pF->mImuCalib.mTcb.rotationMatrix().cast<double>();
  Rbc[0] = Rcb[0].transpose();
  tbc[0] = pF->mImuCalib.mTbc.translation().cast<double>();
  pCamera[0] = pF->mpCamera;
  bf = pF->mbf;

  if (num_cams > 1) {
    Eigen::Matrix4d Trl = pF->GetRelativePoseTrl().matrix().cast<double>();
    Rcw[1] = Trl.block<3, 3>(0, 0) * Rcw[0];
    tcw[1] = Trl.block<3, 3>(0, 0) * tcw[0] + Trl.block<3, 1>(0, 3);
    tcb[1] = Trl.block<3, 3>(0, 0) * tcb[0] + Trl.block<3, 1>(0, 3);
    Rcb[1] = Trl.block<3, 3>(0, 0) * Rcb[0];
    Rbc[1] = Rcb[1].transpose();
    tbc[1] = -Rbc[1] * tcb[1];
    pCamera[1] = pF->mpCamera2;
  }

  // For posegraph 4DoF
  Rwb0 = Rwb;
  DR.setIdentity();
}

ImuCamPose::ImuCamPose(Eigen::Matrix3d& _Rwc, Eigen::Vector3d& _twc,
                       const std::shared_ptr<KeyFrame>& pKF)
    : its(0) {
  // This is only for posegrpah, we do not care about multicamera
  tcw.resize(1);
  Rcw.resize(1);
  tcb.resize(1);
  Rcb.resize(1);
  Rbc.resize(1);
  tbc.resize(1);
  pCamera.resize(1);

  tcb[0] = pKF->mImuCalib.mTcb.translation().cast<double>();
  Rcb[0] = pKF->mImuCalib.mTcb.rotationMatrix().cast<double>();
  Rbc[0] = Rcb[0].transpose();
  tbc[0] = pKF->mImuCalib.mTbc.translation().cast<double>();
  twb = _Rwc * tcb[0] + _twc;
  Rwb = _Rwc * Rcb[0];
  Rcw[0] = _Rwc.transpose();
  tcw[0] = -Rcw[0] * _twc;
  pCamera[0] = pKF->mpCamera;
  bf = pKF->mbf;

  // For posegraph 4DoF
  Rwb0 = Rwb;
  DR.setIdentity();
}

void ImuCamPose::SetParam(const std::vector<Eigen::Matrix3d>& _Rcw,
                          const std::vector<Eigen::Vector3d>& _tcw,
                          const std::vector<Eigen::Matrix3d>& _Rbc,
                          const std::vector<Eigen::Vector3d>& _tbc,
                          const double& _bf) {
  Rbc = _Rbc;
  tbc = _tbc;
  Rcw = _Rcw;
  tcw = _tcw;
  const int num_cams = Rbc.size();
  Rcb.resize(num_cams);
  tcb.resize(num_cams);

  for (size_t i = 0; i < tcb.size(); i++) {
    Rcb[i] = Rbc[i].transpose();
    tcb[i] = -Rcb[i] * tbc[i];
  }
  Rwb = Rcw[0].transpose() * Rcb[0];
  twb = Rcw[0].transpose() * (tcb[0] - tcw[0]);

  bf = _bf;
}

Eigen::Vector2d ImuCamPose::Project(const Eigen::Vector3d& Xw,
                                    int cam_idx) const {
  Eigen::Vector3d Xc = Rcw[cam_idx] * Xw + tcw[cam_idx];

  return pCamera[cam_idx]->project(Xc);
}

Eigen::Vector3d ImuCamPose::ProjectStereo(const Eigen::Vector3d& Xw,
                                          int cam_idx) const {
  Eigen::Vector3d Pc = Rcw[cam_idx] * Xw + tcw[cam_idx];
  Eigen::Vector3d pc;
  double invZ = 1 / Pc(2);
  pc.head(2) = pCamera[cam_idx]->project(Pc);
  pc(2) = pc(0) - bf * invZ;
  return pc;
}

bool ImuCamPose::isDepthPositive(const Eigen::Vector3d& Xw, int cam_idx) const {
  return (Rcw[cam_idx].row(2) * Xw + tcw[cam_idx](2)) > 0.0;
}

void ImuCamPose::Update(const double* pu) {
  Eigen::Vector3d ur, ut;
  ur << pu[0], pu[1], pu[2];
  ut << pu[3], pu[4], pu[5];

  // Update body pose
  twb += Rwb * ut;
  Rwb = Rwb * ExpSO3(ur);

  // Normalize rotation after 5 updates
  its++;
  if (its >= 3) {
    NormalizeRotation(Rwb);
    its = 0;
  }

  // Update camera poses
  const Eigen::Matrix3d Rbw = Rwb.transpose();
  const Eigen::Vector3d tbw = -Rbw * twb;

  for (size_t i = 0; i < pCamera.size(); i++) {
    Rcw[i] = Rcb[i] * Rbw;
    tcw[i] = Rcb[i] * tbw + tcb[i];
  }
}

void ImuCamPose::UpdateW(const double* pu) {
  Eigen::Vector3d ur, ut;
  ur << pu[0], pu[1], pu[2];
  ut << pu[3], pu[4], pu[5];

  const Eigen::Matrix3d dR = ExpSO3(ur);
  DR = dR * DR;
  Rwb = DR * Rwb0;
  // Update body pose
  twb += ut;

  // Normalize rotation after 5 updates
  its++;
  if (its >= 5) {
    DR(0, 2) = 0.0;
    DR(1, 2) = 0.0;
    DR(2, 0) = 0.0;
    DR(2, 1) = 0.0;
    NormalizeRotation(DR);
    its = 0;
  }

  // Update camera pose
  const Eigen::Matrix3d Rbw = Rwb.transpose();
  const Eigen::Vector3d tbw = -Rbw * twb;

  for (size_t i = 0; i < pCamera.size(); i++) {
    Rcw[i] = Rcb[i] * Rbw;
    tcw[i] = Rcb[i] * tbw + tcb[i];
  }
}

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

void InvDepthPoint::Update(const double* pu) { rho += *pu; }

bool VertexPose::read(std::istream& is) {
  std::vector<Eigen::Matrix<double, 3, 3> > Rcw;
  std::vector<Eigen::Matrix<double, 3, 1> > tcw;
  std::vector<Eigen::Matrix<double, 3, 3> > Rbc;
  std::vector<Eigen::Matrix<double, 3, 1> > tbc;

  const size_t num_cams = _estimate.Rbc.size();
  for (size_t idx = 0; idx < num_cams; idx++) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) is >> Rcw[idx](i, j);
    }
    for (int i = 0; i < 3; i++) {
      is >> tcw[idx](i);
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) is >> Rbc[idx](i, j);
    }
    for (int i = 0; i < 3; i++) {
      is >> tbc[idx](i);
    }

    float nextParam;
    for (size_t i = 0; i < _estimate.pCamera[idx]->size(); i++) {
      is >> nextParam;
      _estimate.pCamera[idx]->setParameter(nextParam, i);
    }
  }

  double bf;
  is >> bf;
  _estimate.SetParam(Rcw, tcw, Rbc, tbc, bf);
  updateCache();

  return true;
}

bool VertexPose::write(std::ostream& os) const {
  std::vector<Eigen::Matrix<double, 3, 3> > Rcw = _estimate.Rcw;
  std::vector<Eigen::Matrix<double, 3, 1> > tcw = _estimate.tcw;

  std::vector<Eigen::Matrix<double, 3, 3> > Rbc = _estimate.Rbc;
  std::vector<Eigen::Matrix<double, 3, 1> > tbc = _estimate.tbc;

  const int num_cams = tcw.size();

  for (int idx = 0; idx < num_cams; idx++) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) os << Rcw[idx](i, j) << " ";
    }
    for (int i = 0; i < 3; i++) {
      os << tcw[idx](i) << " ";
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) os << Rbc[idx](i, j) << " ";
    }
    for (int i = 0; i < 3; i++) {
      os << tbc[idx](i) << " ";
    }

    for (size_t i = 0; i < _estimate.pCamera[idx]->size(); i++) {
      os << _estimate.pCamera[idx]->getParameter(i) << " ";
    }
  }

  os << _estimate.bf << " ";

  return os.good();
}

}  // namespace ORB_SLAM3
