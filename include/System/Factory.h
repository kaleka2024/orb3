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

#pragma once

#include <memory>
#include <string>

#include "Settings.h"

namespace ORB_SLAM3 {

class System;

class SystemFactory {
 public:
  typedef tl::expected<std::shared_ptr<System>, ExpectedError> Expected;

  static Expected create(const std::shared_ptr<Settings> &settings,
                         bool initFr = false,
                         const std::string &strSequence = std::string());

  static Expected create(const std::string &configFile, const SensorType sensor,
                         bool initFr = false,
                         const std::string &strSequence = std::string());

  // Provided for compatibility with old API
  static Expected create(const std::string &configFile,
                         const std::string &vocabFile, const SensorType sensor,
                         bool initFr = false,
                         const std::string &strSequence = std::string());
};
}  // namespace ORB_SLAM3
