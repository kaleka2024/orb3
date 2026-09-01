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

#include <Eigen/Eigen>
#include <filesystem>  // NOLINT {build/c++17}
#include <iostream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>
#include <string>
#include <vector>

#include "CameraModels/KannalaBrandt8.h"
#include "CameraModels/Pinhole.h"
#include "Settings.h"
#include "System.h"

namespace ORB_SLAM3 {

template <>
float SettingsLoader::readParameter<float>(cv::FileStorage& fSettings,
                                           const std::string& name, bool& found,
                                           const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      oslog::error("Required parameter \"{}\" does not exist, aborting...",
                   name);
      exit(-1);
    } else {
      oslog::warn("Optional parameter \"{}\" does not exist.", name);
      found = false;
      return 0.0f;
    }
  } else if (!node.isReal()) {
    oslog::error("{} parameter must be a real number, aborting...", name);
    exit(-1);
  } else {
    found = true;
    return node.real();
  }
}

template <>
int SettingsLoader::readParameter<int>(cv::FileStorage& fSettings,
                                       const std::string& name, bool& found,
                                       const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      oslog::error("Required parameter \"{}\" does not exist, aborting...",
                   name);

      exit(-1);
      oslog::warn("Optional parameter \"{}\" does not exist.", name);
      found = false;
      return 0;
    }
  } else if (!node.isInt()) {
    oslog::error("{} parameter must be an integer number, aborting...", name);

    exit(-1);
  } else {
    found = true;
    return node.operator int();
  }
}

template <>
string SettingsLoader::readParameter<string>(cv::FileStorage& fSettings,
                                             const std::string& name,
                                             bool& found, const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      oslog::error("Required parameter \"{}\" does not exist, aborting...",
                   name);

      exit(-1);
    } else {
      oslog::warn("Optional parameter \"{}\" does not exist.", name);
      found = false;
      return string();
    }
  } else if (!node.isString()) {
    oslog::error("{} parameter must be a string, aborting...", name);
    exit(-1);
  } else {
    found = true;
    return node.string();
  }
}

template <>
cv::Mat SettingsLoader::readParameter<cv::Mat>(cv::FileStorage& fSettings,
                                               const std::string& name,
                                               bool& found,
                                               const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      oslog::error("Required parameter \"{}\" does not exist, aborting...",
                   name);
      exit(-1);
    } else {
      oslog::warn("Optional parameter \"{}\" does not exist.", name);
      found = false;
      return cv::Mat();
    }
  } else {
    found = true;
    return node.mat();
  }
}

SettingsLoader::Expected SettingsLoader::Load(const std::string& configFile,
                                              const SensorType sensor,
                                              const std::string& vocabFile) {
  SettingsLoader loader(sensor);
  return loader.load(configFile, vocabFile);
}

SettingsLoader::SettingsLoader(const SensorType sensor)
    : settings_(std::make_shared<Settings>(sensor)) {}

SettingsLoader::Expected SettingsLoader::load(const std::string& configFile,
                                              const std::string& vocabFile) {
  // Open settings file
  cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    oslog::error("[ERROR]: could not open configuration file \"{}\"",
                 configFile);
    oslog::error("Aborting...");

    return tl::make_unexpected(
        ExpectedError::fmt("Unable to open configuration file {}", configFile));
  } else {
    oslog::info("Loading settings from {}", configFile);
  }

  // Read first camera
  readCamera1(fSettings);
  oslog::info("\t-Loaded camera 1");

  // Read second camera if stereo (not rectified)
  if (settings_->sensor_.isStereo()) {
    readCamera2(fSettings);
    oslog::info("\t-Loaded camera 2");
  }

  // Read image info
  readImageInfo(fSettings);
  oslog::info("\t-Loaded image info");

  if (settings_->sensor_.isImu()) {
    readIMU(fSettings);
    oslog::info("\t-Loaded IMU calibration");
  }

  if (settings_->sensor_.isRGBD()) {
    readRGBD(fSettings);
    oslog::info("\t-Loaded RGB-D calibration");
  }

  readORB(fSettings);
  oslog::info("\t-Loaded ORB settings");
  readViewer(fSettings);
  oslog::info("\t-Loaded viewer settings");
  readLoadAndSave(fSettings);
  oslog::info("\t-Loaded Atlas settings");
  readOtherParameters(fSettings);
  oslog::info("\t-Loaded misc parameters");

  oslog::info("----------------------------------");

  if (vocabFile.size() > 0) {
    settings_->strVocFile_ = vocabFile;
  }

  if (settings_->bNeedToRectify_) {
    settings_->precomputeRectificationMaps();
    oslog::info("\t-Computed rectification maps");
  }

  if (settings_->validate()) {
    return settings_;
  } else {
    return tl::unexpected<ExpectedError>("Setting did not validate");
  }
}

void SettingsLoader::readCamera1(cv::FileStorage& fSettings) {
  bool found;

  // Read camera model
  string cameraModel = readParameter<string>(fSettings, "Camera.type", found);

  vector<float> vCalibration, vDistortion;
  if (cameraModel == "PinHole") {
    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    settings_->setMonoCamera(Settings::PinHole, vCalibration);

    // Check if it is a distorted PinHole
    readParameter<float>(fSettings, "Camera1.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera1.k3", found, false);
      if (found) {
        vDistortion.resize(5);
        vDistortion[4] = readParameter<float>(fSettings, "Camera1.k3", found);
      } else {
        vDistortion.resize(4);
      }
      vDistortion[0] = readParameter<float>(fSettings, "Camera1.k1", found);
      vDistortion[1] = readParameter<float>(fSettings, "Camera1.k2", found);
      vDistortion[2] = readParameter<float>(fSettings, "Camera1.p1", found);
      vDistortion[3] = readParameter<float>(fSettings, "Camera1.p2", found);
    }

    settings_->setMonoCamera(Settings::PinHole, vCalibration, vDistortion);

  } else if (cameraModel == "Rectified") {
    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    settings_->setMonoCamera(Settings::Rectified, vCalibration, {});

    // Rectified images are assumed to be ideal PinHole images (no distortion)
  } else if (cameraModel == "KannalaBrandt8") {
    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
    float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
    float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
    float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

    vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

    settings_->setMonoCamera(Settings::KannalaBrandt, vCalibration, {});

    // if (sensor_ == SensorType::STEREO || sensor_ == SensorType::IMU_STEREO) {
    //   int colBegin =
    //       readParameter<int>(fSettings, "Camera1.overlappingBegin", found);
    //   int colEnd =
    //       readParameter<int>(fSettings, "Camera1.overlappingEnd", found);
    //   vector<int> vOverlapping = {colBegin, colEnd};

    //   dynamic_cast<KannalaBrandt8&>(*calibration1_).mvLappingArea =
    //       vOverlapping;
    //}
  } else {
    oslog::error("Error: {} not known", cameraModel);
    exit(-1);
  }
}

void SettingsLoader::readCamera2(cv::FileStorage& fSettings) {
  bool found;
  vector<float> vCalibration, vDistortion;
  if (settings_->cameraType_ == Settings::PinHole) {
    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera2.fx", found);
    float fy = readParameter<float>(fSettings, "Camera2.fy", found);
    float cx = readParameter<float>(fSettings, "Camera2.cx", found);
    float cy = readParameter<float>(fSettings, "Camera2.cy", found);

    vCalibration = {fx, fy, cx, cy};

    // Check if it is a distorted PinHole
    readParameter<float>(fSettings, "Camera2.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera2.k3", found, false);
      if (found) {
        vDistortion.resize(5);
        vDistortion[4] = readParameter<float>(fSettings, "Camera2.k3", found);
      } else {
        vDistortion.resize(4);
      }
      vDistortion[0] = readParameter<float>(fSettings, "Camera2.k1", found);
      vDistortion[1] = readParameter<float>(fSettings, "Camera2.k2", found);
      vDistortion[2] = readParameter<float>(fSettings, "Camera2.p1", found);
      vDistortion[3] = readParameter<float>(fSettings, "Camera2.p2", found);
    }

  } else if (settings_->cameraType_ == Settings::KannalaBrandt) {
    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera2.fx", found);
    float fy = readParameter<float>(fSettings, "Camera2.fy", found);
    float cx = readParameter<float>(fSettings, "Camera2.cx", found);
    float cy = readParameter<float>(fSettings, "Camera2.cy", found);

    float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
    float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
    float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
    float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

    vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

    // int colBegin =
    //     readParameter<int>(fSettings, "Camera2.overlappingBegin", found);
    // int colEnd = readParameter<int>(fSettings, "Camera2.overlappingEnd",
    // found); vector<int> vOverlapping = {colBegin, colEnd};

    // dynamic_cast<KannalaBrandt8&>(*calibration2_).mvLappingArea =
    // vOverlapping;
  }

  float thDepth = readParameter<float>(fSettings, "Stereo.ThDepth", found);

  // Load stereo extrinsic calibration
  if (settings_->cameraType_ == Settings::Rectified) {
    const float baseline = readParameter<float>(fSettings, "Stereo.b", found);
    // setRightCamera( vCalibration, vDistortion, baseline, thDepth );

    settings_->b_ = baseline;
    settings_->bf_ = baseline * settings_->calibration1_->getParameter(0);

  } else {
    cv::Mat cvTlr = readParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
    settings_->setRightCamera(vCalibration, vDistortion, cvTlr, thDepth);
  }
}

//===

void SettingsLoader::readImageInfo(cv::FileStorage& fSettings) {
  bool found;
  // Read original and desired image dimensions
  int originalRows = readParameter<int>(fSettings, "Camera.height", found);
  int originalCols = readParameter<int>(fSettings, "Camera.width", found);

  settings_->setOriginalImageSize(originalCols, originalRows);

  bool resizeHeightFound = false, resizeWidthFound = false;
  int resizeHeight = originalRows, resizeWidth = originalCols;

  auto h = readParameter<int>(fSettings, "Camera.newHeight", resizeHeightFound,
                              false);
  if (resizeHeightFound) resizeHeight = h;

  auto w =
      readParameter<int>(fSettings, "Camera.newWidth", resizeWidthFound, false);
  if (resizeWidthFound) resizeWidth = w;

  if (resizeHeightFound || resizeWidthFound)
    settings_->setResizeImageSize(resizeWidth, resizeHeight);

  settings_->bRGB_ =
      static_cast<bool>(readParameter<int>(fSettings, "Camera.RGB", found));
}

void SettingsLoader::readIMU(cv::FileStorage& fSettings) {
  bool found;
  settings_->noiseGyro_ =
      readParameter<float>(fSettings, "IMU.NoiseGyro", found);
  settings_->noiseAcc_ = readParameter<float>(fSettings, "IMU.NoiseAcc", found);
  settings_->gyroWalk_ = readParameter<float>(fSettings, "IMU.GyroWalk", found);
  settings_->accWalk_ = readParameter<float>(fSettings, "IMU.AccWalk", found);
  settings_->imuFrequency_ =
      readParameter<float>(fSettings, "IMU.Frequency", found);

  cv::Mat cvTbc = readParameter<cv::Mat>(fSettings, "IMU.T_b_c1", found);
  settings_->Tbc_ = Converter::toSophus(cvTbc);

  readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
  if (found) {
    settings_->insertKFsWhenLost_ = static_cast<bool>(
        readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false));
  } else {
    settings_->insertKFsWhenLost_ = true;
  }
}

void SettingsLoader::readRGBD(cv::FileStorage& fSettings) {
  bool found;

  settings_->depthMapFactor_ =
      readParameter<float>(fSettings, "RGBD.DepthMapFactor", found);
  settings_->thDepth_ =
      readParameter<float>(fSettings, "Stereo.ThDepth", found);
  settings_->b_ = readParameter<float>(fSettings, "Stereo.b", found);
  settings_->bf_ = settings_->b_ * settings_->calibration1_->getParameter(0);
}

void SettingsLoader::readORB(cv::FileStorage& fSettings) {
  bool found;

  settings_->nFeatures_ =
      readParameter<int>(fSettings, "ORBextractor.nFeatures", found);
  settings_->scaleFactor_ =
      readParameter<float>(fSettings, "ORBextractor.scaleFactor", found);
  settings_->nLevels_ =
      readParameter<int>(fSettings, "ORBextractor.nLevels", found);
  settings_->initThFAST_ =
      readParameter<int>(fSettings, "ORBextractor.iniThFAST", found);
  settings_->minThFAST_ =
      readParameter<int>(fSettings, "ORBextractor.minThFAST", found);
}

void SettingsLoader::readViewer(cv::FileStorage& fSettings) {
  bool found;

  settings_->useViewer_ = readParameter<int>(fSettings, "Viewer.Enable", found);
  settings_->keyFrameSize_ =
      readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
  settings_->keyFrameLineWidth_ =
      readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
  settings_->graphLineWidth_ =
      readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
  settings_->pointSize_ =
      readParameter<float>(fSettings, "Viewer.PointSize", found);
  settings_->cameraSize_ =
      readParameter<float>(fSettings, "Viewer.CameraSize", found);
  settings_->cameraLineWidth_ =
      readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
  settings_->viewPointX_ =
      readParameter<float>(fSettings, "Viewer.ViewpointX", found);
  settings_->viewPointY_ =
      readParameter<float>(fSettings, "Viewer.ViewpointY", found);
  settings_->viewPointZ_ =
      readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
  settings_->viewPointF_ =
      readParameter<float>(fSettings, "Viewer.ViewpointF", found);
  settings_->imageViewerScale_ =
      readParameter<float>(fSettings, "Viewer.imageViewScale", found, false);

  if (!found) settings_->imageViewerScale_ = 1.0f;
}

void SettingsLoader::readLoadAndSave(cv::FileStorage& fSettings) {
  bool found;

  settings_->sLoadFrom_ = readParameter<string>(
      fSettings, "System.LoadAtlasFromFile", found, false);
  settings_->sSaveto_ =
      readParameter<string>(fSettings, "System.SaveAtlasToFile", found, false);
}

void SettingsLoader::readOtherParameters(cv::FileStorage& fSettings) {
  bool found;

  settings_->thFarPoints_ =
      readParameter<float>(fSettings, "System.thFarPoints", found, false);

  settings_->loopClosing_ = static_cast<bool>(
      readParameter<int>(fSettings, "System.loopClosing", found, true));
}

};  // namespace ORB_SLAM3
