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

#include <cstdint>
#include <string>

namespace ORB_SLAM3 {

// Rather than a conventional enum class, use this pattern which lets us
// add member functions
class SensorType {
 public:
  enum Value : uint8_t {
    MONOCULAR = 0,
    STEREO = 1,
    RGBD = 2,
    IMU_MONOCULAR = 3,
    IMU_STEREO = 4,
    IMU_RGBD = 5,
  };

  SensorType() = delete;
  SensorType(Value stype) : value(stype) {}  // NOLINT {runtime/explicit}

  constexpr bool operator==(SensorType a) const { return value == a.value; }
  constexpr bool operator!=(SensorType a) const { return value != a.value; }

  constexpr bool operator==(Value a) const { return value == a; }
  constexpr bool operator!=(Value a) const { return value != a; }

  constexpr bool isImu() const {
    return (value == IMU_MONOCULAR || value == IMU_RGBD || value == IMU_STEREO);
  }
  constexpr bool isRGBD() const { return (value == RGBD || value == IMU_RGBD); }
  constexpr bool isStereo() const {
    return (value == STEREO || value == IMU_STEREO);
  }
  constexpr bool isMonocular() const {
    return (value == MONOCULAR || value == IMU_MONOCULAR);
  }

  std::string toString() const {
    if (value == SensorType::MONOCULAR)
      return "Monocular";
    else if (value == SensorType::STEREO)
      return "Stereo";
    else if (value == SensorType::RGBD)
      return "RGB-D";
    else if (value == SensorType::IMU_MONOCULAR)
      return "Monocular-Inertial";
    else if (value == SensorType::IMU_STEREO)
      return "Stereo-Inertial";
    else if (value == SensorType::IMU_RGBD)
      return "RGB-D-Inertial";

    return "(unknown)";
  }

 private:
  Value value;
};

enum class FileType { TEXT_FILE, BINARY_FILE };

}  // namespace ORB_SLAM3
