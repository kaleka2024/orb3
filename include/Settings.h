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

// Flag to activate the measurement of time in each process (track,localmap,
// place recognition).
// #define REGISTER_TIMES

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "CameraModels/GeometricCamera.h"
#include "Expected.h"
#include "Logging.h"
#include "Types.h"

namespace ORB_SLAM3 {

class System;
class Settings;

// TODO: change to double instead of float

class SettingsLoader {
 public:
  typedef tl::expected<std::shared_ptr<Settings>, ExpectedError> Expected;
  static Expected Load(const std::string& configFile, const SensorType sensor,
                       const std::string& vocabFile = "");

  explicit SettingsLoader(const SensorType sensor);
  Expected load(const std::string& configFile,
                const std::string& vocabFile = "");

 private:
  void readCamera1(cv::FileStorage& fSettings);
  void readCamera2(cv::FileStorage& fSettings);
  void readImageInfo(cv::FileStorage& fSettings);
  void readIMU(cv::FileStorage& fSettings);
  void readRGBD(cv::FileStorage& fSettings);
  void readORB(cv::FileStorage& fSettings);
  void readViewer(cv::FileStorage& fSettings);
  void readLoadAndSave(cv::FileStorage& fSettings);
  void readOtherParameters(cv::FileStorage& fSettings);

  std::shared_ptr<Settings> settings_;

  template <typename T>
  T readParameter(cv::FileStorage& fSettings, const std::string& name,
                  bool& found, const bool required = true) {
    cv::FileNode node = fSettings[name];
    if (node.empty()) {
      if (required) {
        oslog::error("Required parameter \"{}\" does not exist, aborting...",
                     name);
        exit(-1);
      } else {
        oslog::warn("Optional parameter \"{}\" does not exist.", name);
        found = false;
        return T();
      }

    } else {
      found = true;
      return (T)node;
    }
  }
};

class Settings {
 public:
  friend class SettingsLoader;

  /*
   * Enum for the different camera types implemented
   */
  enum CameraType { PinHole = 0, Rectified = 1, KannalaBrandt = 2 };

  /*
   * Delete default constructor
   */
  Settings() = delete;

  explicit Settings(const SensorType sensor);
  Settings(const Settings&) = default;

  ~Settings();

  // Safety checks
  bool validate();

  /*
   * Ostream operator overloading to dump settings to the terminal
   */
  friend std::ostream& operator<<(std::ostream& output, const Settings& s);

  /*
   * Getter methods
   */
  CameraType cameraType() const { return cameraType_; }
  std::shared_ptr<GeometricCamera> camera1() const { return calibration1_; }
  std::shared_ptr<GeometricCamera> camera2() const { return calibration2_; }
  cv::Mat camera1DistortionCoef() {
    return cv::Mat(vPinHoleDistorsion1_.size(), 1, CV_32F,
                   vPinHoleDistorsion1_.data());
  }
  cv::Mat camera2DistortionCoef() {
    return cv::Mat(vPinHoleDistorsion2_.size(), 1, CV_32F,
                   vPinHoleDistorsion2_.data());
  }

  Sophus::SE3f Tlr() { return Tlr_; }
  float bf() { return bf_; }
  float b() { return b_; }
  float thDepth() { return thDepth_; }

  bool needToUndistort() { return bNeedToUndistort_; }

  cv::Size newImSize() { return newImSize_; }
  bool rgb() { return bRGB_; }
  bool needToResize() { return bNeedToResize1_; }
  bool needToRectify() { return bNeedToRectify_; }

  float noiseGyro() { return noiseGyro_; }
  float noiseAcc() { return noiseAcc_; }
  float gyroWalk() { return gyroWalk_; }
  float accWalk() { return accWalk_; }
  float imuFrequency() { return imuFrequency_; }
  Sophus::SE3f Tbc() { return Tbc_; }
  bool insertKFsWhenLost() { return insertKFsWhenLost_; }

  float depthMapFactor() { return depthMapFactor_; }

  int nFeatures() { return nFeatures_; }
  int nLevels() { return nLevels_; }
  float initThFAST() { return initThFAST_; }
  float minThFAST() { return minThFAST_; }
  float scaleFactor() { return scaleFactor_; }

  bool useViewer() const { return useViewer_; }
  float keyFrameSize() { return keyFrameSize_; }
  float keyFrameLineWidth() { return keyFrameLineWidth_; }
  float graphLineWidth() { return graphLineWidth_; }
  float pointSize() { return pointSize_; }
  float cameraSize() { return cameraSize_; }
  float cameraLineWidth() { return cameraLineWidth_; }
  float viewPointX() { return viewPointX_; }
  float viewPointY() { return viewPointY_; }
  float viewPointZ() { return viewPointZ_; }
  float viewPointF() { return viewPointF_; }
  float imageViewerScale() { return imageViewerScale_; }

  std::string atlasLoadFile() { return sLoadFrom_; }
  std::string atlasSaveFile() { return sSaveto_; }

  float thFarPoints() { return thFarPoints_; }

  cv::Mat M1l() { return M1l_; }
  cv::Mat M2l() { return M2l_; }
  cv::Mat M1r() { return M1r_; }
  cv::Mat M2r() { return M2r_; }

  // For PinHole,       k = {fx, fy, cx, cy}, and dist can be 0, 4 or 5 params
  // For Rectified,     k = {fx, fy, cx, cy}  and dist is ignored
  // For KannalaBrandt, k = {fx, fy, cx, cy, k0, k1, k2, k3};
  void setMonoCamera(CameraType type, const std::vector<float>& k,
                     const std::vector<float>& dist = {});
  void setRightCamera(const std::vector<float>& k2,
                      const std::vector<float>& dist2, const cv::Mat& T_c1_c2,
                      float thDepth);

  void setStereoRectifiedCamera(const std::vector<float>& k, float baseline,
                                float thDepth);

  void setOriginalImageSize(int width, int height);
  void setResizeImageSize(int width, int height);

  void precomputeRectificationMaps();

  SensorType sensor_;
  CameraType cameraType_;  // Camera type

  /*
   * Visual stuff
   */
  std::shared_ptr<GeometricCamera> calibration1_,
      calibration2_;  // Camera calibration
  std::shared_ptr<GeometricCamera> originalCalib1_, originalCalib2_;
  std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

  cv::Size originalImSize_, newImSize_;
  bool bRGB_;

  bool bNeedToUndistort_;
  bool bNeedToRectify_;
  bool bNeedToResize1_, bNeedToResize2_;

  Sophus::SE3f Tlr_;
  float thDepth_;
  float bf_, b_;

  /*
   * Rectification stuff
   */
  cv::Mat M1l_, M2l_;
  cv::Mat M1r_, M2r_;

  /*
   * Inertial stuff
   */
  float noiseGyro_, noiseAcc_;
  float gyroWalk_, accWalk_;
  float imuFrequency_;
  Sophus::SE3f Tbc_;
  bool insertKFsWhenLost_;

  /*
   * RGBD stuff
   */
  float depthMapFactor_;

  /*
   * ORB stuff
   */
  int nFeatures_;
  float scaleFactor_;
  int nLevels_;
  int initThFAST_, minThFAST_;

  /*
   * Viewer stuff
   */
  bool useViewer_;
  float keyFrameSize_;
  float keyFrameLineWidth_;
  float graphLineWidth_;
  float pointSize_;
  float cameraSize_;
  float cameraLineWidth_;
  float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
  float imageViewerScale_;

  /*
   * Save & load maps
   */
  std::string sLoadFrom_, sSaveto_;

  /*
   * Other stuff
   */
  float thFarPoints_;

  bool loopClosing_;
  std::string strVocFile_;
};
};  // namespace ORB_SLAM3
