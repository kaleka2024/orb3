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

EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ()
    : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                     g2o::VertexSE3Expmap>() {}

bool EdgeSE3ProjectXYZ::read(std::istream& is) {
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

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

void EdgeSE3ProjectXYZ::linearizeOplus() {
  const g2o::VertexSE3Expmap* vj =
      static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
  const g2o::SE3Quat T(vj->estimate());
  const g2o::VertexPointXYZ* vi =
      static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
  const Eigen::Vector3d xyz = vi->estimate();
  const Eigen::Vector3d xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];

  Eigen::Matrix<double, 2, 3> projectJac = -pCamera->projectJac(xyz_trans);

  _jacobianOplusXi = projectJac * T.rotation().toRotationMatrix();

  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f, -z, 0.f, x, 0.f, 1.f, 0.f, y, -x, 0.f,
      0.f, 0.f, 1.f;

  _jacobianOplusXj = projectJac * SE3deriv;
}

}  // namespace ORB_SLAM3
