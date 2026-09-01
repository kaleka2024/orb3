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

EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ()
    : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                          VertexSim3Expmap>() {}

bool EdgeSim3ProjectXYZ::read(std::istream& is) {
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

bool EdgeSim3ProjectXYZ::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << _measurement[i] << " ";
  }

  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ()
    : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                          VertexSim3Expmap>() {}

bool EdgeInverseSim3ProjectXYZ::read(std::istream& is) {
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

bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const {
  for (int i = 0; i < 2; i++) {
    os << _measurement[i] << " ";
  }

  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++) {
      os << " " << information()(i, j);
    }
  return os.good();
}

}  // namespace ORB_SLAM3
