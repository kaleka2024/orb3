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
bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is) {
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

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

void EdgeSE3ProjectXYZOnlyPose::computeError() {
  const g2o::VertexSE3Expmap* v1 =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  Eigen::Vector2d obs(_measurement);
  _error = obs - pCamera->project(v1->estimate().map(Xw));
}

bool EdgeSE3ProjectXYZOnlyPose::isDepthPositive() {
  const g2o::VertexSE3Expmap* v1 =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  return (v1->estimate().map(Xw))(2) > 0.0;
}

void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
  //  const g2o::VertexSE3Expmap* vi = static_cast<const
  //  g2o::VertexSE3Expmap*>(_vertices[0]); const Eigen::Vector3d xyz_trans =
  //  vi->estimate().map(Xw);

  const g2o::VertexSE3Expmap* vi =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
  const g2o::SE3Quat T_lw = vi->estimate();
  const Eigen::Vector3d X_l = T_lw.map(Xw);

  const double x = X_l[0];
  const double y = X_l[1];
  const double z = X_l[2];

  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f, -z, 0.f, x, 0.f, 1.f, 0.f, y, -x, 0.f,
      0.f, 0.f, 1.f;

  Eigen::Matrix<double, 2, 6> p;
  p.noalias() = (-pCamera->projectJac(X_l).eval() * SE3deriv);
  _jacobianOplusXi = p;
}

}  // namespace ORB_SLAM3
