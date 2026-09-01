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

#include "euroc_common.h"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>

#include "ImuTypes.h"

using namespace std;

namespace fs = std::filesystem;

EuRoCData EuRoCData::LoadSequences(const std::vector<SequencePaths> &seqPaths,
                                   bool loadImu) {
  EuRoCData data;

  for (auto const &seq : seqPaths) {
    const string pathCam0 = seq.mImagePath + "/mav0/cam0/data";
    const string pathCam1 = seq.mImagePath + "/mav0/cam1/data";

    if (!fs::exists(pathCam0) || !fs::exists(pathCam1)) {
      throw runtime_error("Canot find the image data");
    }

    if (loadImu) {
      // For now, load the IMU data directly from the EuRoC dataset
      const string pathImu = seq.mImagePath + "/mav0/imu0/data.csv";

      if (!fs::exists(pathImu)) {
        throw std::runtime_error("Cannot find IMU data");
      }

      data.mvSequences.emplace_back(pathCam0, pathCam1, seq.mTimestampPath,
                                    pathImu);
    } else {
      data.mvSequences.emplace_back(pathCam0, pathCam1, seq.mTimestampPath);
    }
  }

  return data;
}

EuRoCSequence::EuRoCSequence(const string &leftPath, const string &rightPath,
                             const string &timestampPath,
                             const string &imuPath) {
  ifstream fTimes;
  fTimes.open(timestampPath.c_str());

  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      if (s[0] == '#') continue;

      stringstream ss;
      ss << s;
      const string leftImg = leftPath + "/" + ss.str() + ".png";
      const string rightImg = rightPath + "/" + ss.str() + ".png";

      double t;
      ss >> t;

      mvImageSets.emplace_back(leftImg, rightImg, t / 1e9);
    }
  }

  if ((imuPath.size() > 0) && (mvImageSets.size() > 0)) {
    loadImu(imuPath, mvImageSets.begin()->mTimestamp);
  }
}

void EuRoCSequence::loadImu(const string &imuPath, double startTime) {
  ifstream fImu;

  fImu.open(imuPath.c_str());

  while (!fImu.eof()) {
    string s;
    getline(fImu, s);

    if (s[0] == '#') continue;

    if (!s.empty()) {
      string item;
      size_t pos = 0;
      double data[7];
      int count = 0;
      while ((pos = s.find(',')) != string::npos) {
        item = s.substr(0, pos);
        data[count++] = stod(item);
        s.erase(0, pos + 1);
      }
      item = s.substr(0, pos);
      data[6] = stod(item);

      const double t = data[0] / 1e9;

      // Ignore data before startTime
      if (t <= startTime) continue;

      vImu.emplace_back(data[4], data[5], data[6], data[1], data[2], data[3],
                        t);
    }
  }
}

cv::Mat ImageSet::leftImage() const {
  return cv::imread(mLeftImage, cv::IMREAD_UNCHANGED);
}

cv::Mat ImageSet::rightImage() const {
  return cv::imread(mRightImage, cv::IMREAD_UNCHANGED);
}

void setupEurocSpdLogger() {
  auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

  ORB_SLAM3::Logger::add_sink(stdout_sink);
  spdlog::set_default_logger(
      std::make_shared<spdlog::logger>("euroc", stdout_sink));
  spdlog::set_pattern("%E.%e [%-8n] [%6!l] %v");
}
