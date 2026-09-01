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

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>

#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

struct ImageSet {
  ImageSet(const string &leftImg, const string &rightImg, double timestamp)
      : mLeftImage(leftImg), mRightImage(rightImg), mTimestamp(timestamp) {}

  cv::Mat leftImage() const;
  cv::Mat rightImage() const;

  std::string mLeftImage, mRightImage;
  double mTimestamp;
};

struct EuRoCSequence {
  EuRoCSequence() = delete;
  EuRoCSequence(const string &leftPath, const string &rightPath,
                const string &timestampPath, const string &imuPath = "");

  void loadImu(const string &imuPath, double startTime = 0.0);

  vector<ImageSet> mvImageSets;
  vector<ORB_SLAM3::IMU::Point> vImu;

  size_t size() const { return mvImageSets.size(); }

  // Lazy solution, assume constant image rate
  double dt() const {
    if (size() > 1) {
      return mvImageSets.at(1).mTimestamp - mvImageSets.at(0).mTimestamp;
    } else {
      return 0;
    }
  }
};

class EuRoCData {
 public:
  struct SequencePaths {
    SequencePaths(const string &imagePath, const string &timestampPath,
                  const string &imuPath = "")
        : mImagePath(imagePath),
          mTimestampPath(timestampPath),
          mImuPath(imuPath) {}

    std::string mImagePath, mTimestampPath, mImuPath;
  };

  EuRoCData() = default;

  static EuRoCData LoadSequences(const std::vector<SequencePaths> &seqPaths,
                                 bool loadImu = false);

  size_t totalImages() const {
    size_t count = 0;
    for (auto const &seq : mvSequences) count += seq.size();
    return count;
  }

  std::vector<EuRoCSequence> mvSequences;
};

void setupEurocSpdLogger();
