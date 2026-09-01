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

#include "Logging.h"
#include "OptimizableTypes.h"

namespace ORB_SLAM3 {

bool EdgeSE3ProjectXYZOnlyPoseToBody::read(std::istream& is) {
  for (int i = 0; i < 2; i++) {
    is >> _measurement[i];
  }
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}

bool EdgeSE3ProjectXYZOnlyPoseToBody::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

void EdgeSE3ProjectXYZOnlyPoseToBody::computeError() {
  const g2o::VertexSE3Expmap* v1 =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  Eigen::Vector2d obs(_measurement);
  _error = obs - pCamera->project((mTrl * v1->estimate()).map(Xw));
}

bool EdgeSE3ProjectXYZOnlyPoseToBody::isDepthPositive() {
  const g2o::VertexSE3Expmap* v1 =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
}

void EdgeSE3ProjectXYZOnlyPoseToBody::linearizeOplus() {
  const g2o::VertexSE3Expmap* vi =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  g2o::SE3Quat T_lw(vi->estimate());
  Eigen::Vector3d X_l = T_lw.map(Xw);
  Eigen::Vector3d X_r = mTrl.map(T_lw.map(Xw));

  double x_w = X_l[0];
  double y_w = X_l[1];
  double z_w = X_l[2];

  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.f, z_w, -y_w, 1.f, 0.f, 0.f, -z_w, 0.f, x_w, 0.f, 1.f, 0.f, y_w,
      -x_w, 0.f, 0.f, 0.f, 1.f;

  _jacobianOplusXi =
      -pCamera->projectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
}

}  // namespace ORB_SLAM3
