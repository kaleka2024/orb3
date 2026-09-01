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

VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, g2o::Sim3>() {
  _marginalized = false;
  _fix_scale = false;
}

bool VertexSim3Expmap::read(std::istream& is) {
  g2o::Vector7 cam2world;
  for (int i = 0; i < 6; i++) {
    is >> cam2world[i];
  }
  is >> cam2world[6];

  float nextParam;
  for (size_t i = 0; i < pCamera1->size(); i++) {
    is >> nextParam;
    pCamera1->setParameter(nextParam, i);
  }

  for (size_t i = 0; i < pCamera2->size(); i++) {
    is >> nextParam;
    pCamera2->setParameter(nextParam, i);
  }

  setEstimate(g2o::Sim3(cam2world).inverse());
  return true;
}

bool VertexSim3Expmap::write(std::ostream& os) const {
  g2o::Sim3 cam2world(estimate().inverse());
  g2o::Vector7 lv = cam2world.log();
  for (int i = 0; i < 7; i++) {
    os << lv[i] << " ";
  }

  for (size_t i = 0; i < pCamera1->size(); i++) {
    os << pCamera1->getParameter(i) << " ";
  }

  for (size_t i = 0; i < pCamera2->size(); i++) {
    os << pCamera2->getParameter(i) << " ";
  }

  return os.good();
}

}  // namespace ORB_SLAM3
