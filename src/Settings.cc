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

#include "Settings.h"

#include <filesystem>  // NOLINT {build/c++17}
#include <iostream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>
#include <string>
#include <vector>

#include "CameraModels/KannalaBrandt8.h"
#include "CameraModels/Pinhole.h"
#include "System.h"

namespace ORB_SLAM3 {

Settings::Settings(const SensorType sensor)
    : bNeedToUndistort_(false),
      bNeedToRectify_(false),
      bNeedToResize1_(false),
      bNeedToResize2_(false),
      loopClosing_(true),
      sensor_(sensor),
      imageViewerScale_(1.0f),
      imuFrequency_(1.0f),
      thDepth_(5) {
  // if(bNeedToRectify_){
  //     precomputeRectificationMaps();
  //     cout << "\t-Computed rectification maps" << endl;
  // }
}

Settings::~Settings() { ; }

bool Settings::validate(void) {
  // Check all of the variables that are assumed to be set
  if (!calibration1_) return false;
  if (!originalCalib1_) return false;

  if (originalImSize_.width == 0) return false;
  if (originalImSize_.height == 0) return false;

  if (strVocFile_.size() == 0) {
    oslog::warn("Vocab file not specified");
    return false;
  } else if (!std::filesystem::exists(strVocFile_)) {
    oslog::warn("Vocab file {} does not exist.", strVocFile_);
    return false;
  }

  return true;
}

//===

void Settings::setMonoCamera(CameraType type, const std::vector<float>& k,
                             const std::vector<float>& dist) {
  cameraType_ = type;

  if (cameraType_ == PinHole) {
    calibration1_ = std::make_shared<Pinhole>(k);
    originalCalib1_ = std::make_shared<Pinhole>(k);

    vPinHoleDistorsion1_ = dist;

    // Check if we need to correct distortion from the images
    if (sensor_.isMonocular() && vPinHoleDistorsion1_.size() != 0) {
      bNeedToUndistort_ = true;
    }
  } else if (cameraType_ == Rectified) {
    calibration1_ = std::make_shared<Pinhole>(k);
    originalCalib1_ = std::make_shared<Pinhole>(k);

    // Rectified images are assumed to be ideal PinHole images (no distortion)
  } else if (cameraType_ == KannalaBrandt) {
    if (k.size() != 8) {
      oslog::error("Incorrect number of params for KannalaBrandt");
      return;
    }

    calibration1_ = std::make_shared<KannalaBrandt8>(k);
    originalCalib1_ = std::make_shared<KannalaBrandt8>(k);

    // \todo{AMM}   Commented this out while making SettingsLoader, did not feel
    //              like converting it at the time
    // if (sensor_.isStereo()) {
    //   int colBegin =
    //       readParameter<int>(fSettings, "Camera1.overlappingBegin", found);
    //   int colEnd =
    //       readParameter<int>(fSettings, "Camera1.overlappingEnd", found);
    //   vector<int> vOverlapping = {colBegin, colEnd};

    //   dynamic_cast<KannalaBrandt8&>(*calibration1_).mvLappingArea =
    //       vOverlapping;
    // }
  } else {
    oslog::error("Error: {} not known", static_cast<int>(type));
    exit(-1);
  }
}

void Settings::setRightCamera(const std::vector<float>& k2,
                              const std::vector<float>& dist2,
                              const cv::Mat& T_c1_c2, float thDepth) {
  if (cameraType_ == PinHole) {
    bNeedToRectify_ = true;

    calibration2_ = std::make_shared<Pinhole>(k2);
    originalCalib2_ = std::make_shared<Pinhole>(k2);

    vPinHoleDistorsion2_ = dist2;

    // Note to self, for Rectified cameras, the calibration is the same for both
    // L and R
  } else if (cameraType_ == KannalaBrandt) {
    calibration2_ = std::make_shared<KannalaBrandt8>(k2);
    originalCalib2_ = std::make_shared<KannalaBrandt8>(k2);

    // \todo{AMM}   Commented this out while making SettingsLoader, did not feel
    //              like converting it at the time
    // int colBegin =
    //     readParameter<int>(fSettings, "Camera2.overlappingBegin", found);
    // int colEnd = readParameter<int>(fSettings, "Camera2.overlappingEnd",
    // found); vector<int> vOverlapping = {colBegin, colEnd};

    // dynamic_cast<KannalaBrandt8&>(*calibration2_).mvLappingArea =
    // vOverlapping;
  }

  cv::Mat cvTlr = T_c1_c2;
  Tlr_ = Converter::toSophus(cvTlr);

  // TODO: also search for Trl and invert if necessary

  b_ = Tlr_.translation().norm();
  bf_ = b_ * calibration1_->getParameter(0);

  thDepth_ = thDepth;
}

void Settings::setStereoRectifiedCamera(const std::vector<float>& k,
                                        float baseline, float thDepth) {
  cameraType_ = Rectified;

  calibration1_ = std::make_shared<Pinhole>(k);
  originalCalib1_ = std::make_shared<Pinhole>(k);

  b_ = baseline;
  bf_ = b_ * calibration1_->getParameter(0);
}

//===

void Settings::setOriginalImageSize(int width, int height) {
  // Read original and desired image dimensions
  int originalRows = height;
  int originalCols = width;

  originalImSize_.width = originalCols;
  originalImSize_.height = originalRows;

  newImSize_ = originalImSize_;
}

void Settings::setResizeImageSize(int width, int height) {
  if (!calibration1_) {
    throw std::runtime_error(
        "setResizeImageSize() must be called after loading camera parameters");
  }

  if (height != originalImSize_.height) {
    bNeedToResize1_ = true;
    newImSize_.height = height;

    if (!bNeedToRectify_) {
      // Update calibration
      const float scaleRowFactor = static_cast<float>(newImSize_.height) /
                                   static_cast<float>(originalImSize_.height);
      calibration1_->setParameter(
          calibration1_->getParameter(1) * scaleRowFactor, 1);
      calibration1_->setParameter(
          calibration1_->getParameter(3) * scaleRowFactor, 3);

      if ((sensor_.isStereo()) && cameraType_ != Rectified) {
        calibration2_->setParameter(
            calibration2_->getParameter(1) * scaleRowFactor, 1);
        calibration2_->setParameter(
            calibration2_->getParameter(3) * scaleRowFactor, 3);
      }
    }
  }

  if (width != originalImSize_.width) {
    bNeedToResize1_ = true;
    newImSize_.width = width;

    if (!bNeedToRectify_) {
      // Update calibration
      const float scaleColFactor = static_cast<float>(newImSize_.width) /
                                   static_cast<float>(originalImSize_.width);
      calibration1_->setParameter(
          calibration1_->getParameter(0) * scaleColFactor, 0);
      calibration1_->setParameter(
          calibration1_->getParameter(2) * scaleColFactor, 2);

      if ((sensor_.isStereo()) && cameraType_ != Rectified) {
        calibration2_->setParameter(
            calibration2_->getParameter(0) * scaleColFactor, 0);
        calibration2_->setParameter(
            calibration2_->getParameter(2) * scaleColFactor, 2);

        if (cameraType_ == KannalaBrandt) {
          dynamic_cast<KannalaBrandt8*>(calibration1_.get())
              ->mvLappingArea[0] *= scaleColFactor;
          dynamic_cast<KannalaBrandt8*>(calibration1_.get())
              ->mvLappingArea[1] *= scaleColFactor;

          dynamic_cast<KannalaBrandt8*>(calibration2_.get())
              ->mvLappingArea[0] *= scaleColFactor;
          dynamic_cast<KannalaBrandt8*>(calibration2_.get())
              ->mvLappingArea[1] *= scaleColFactor;
        }
      }
    }
  }
}

void Settings::precomputeRectificationMaps() {
  oslog::trace(
      "[Settings::precomputeRectificationMaps] Precomputing rectification "
      "maps");

  // Precompute rectification maps, new calibrations, ...
  cv::Mat K1 = dynamic_cast<Pinhole&>(*calibration1_).toK();
  K1.convertTo(K1, CV_64F);
  cv::Mat K2 = dynamic_cast<Pinhole&>(*calibration2_).toK();
  K2.convertTo(K2, CV_64F);

  cv::Mat cvTlr;
  cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);
  cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
  R12.convertTo(R12, CV_64F);
  cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
  t12.convertTo(t12, CV_64F);

  cv::Mat R_r1_u1, R_r2_u2;
  cv::Mat P1, P2, Q;

  cv::stereoRectify(K1, camera1DistortionCoef(), K2, camera2DistortionCoef(),
                    originalImSize_, R12, t12, R_r1_u1, R_r2_u2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY, -1, newImSize_);
  cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1,
                              P1.rowRange(0, 3).colRange(0, 3), newImSize_,
                              CV_32F, M1l_, M2l_);
  cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2,
                              P2.rowRange(0, 3).colRange(0, 3), newImSize_,
                              CV_32F, M1r_, M2r_);

  // Update calibration
  // (updating calibrations in place can lead to problems.  Ask me how I
  // know...)
  calibration1_->setParameter(P1.at<double>(0, 0), 0);
  calibration1_->setParameter(P1.at<double>(1, 1), 1);
  calibration1_->setParameter(P1.at<double>(0, 2), 2);
  calibration1_->setParameter(P1.at<double>(1, 2), 3);

  // Update bf
  bf_ = b_ * P1.at<double>(0, 0);

  // Update relative pose between camera 1 and IMU if necessary
  if (sensor_ == SensorType::IMU_STEREO) {
    Eigen::Matrix3f eigenR_r1_u1;
    cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
    Sophus::SE3f T_r1_u1(eigenR_r1_u1, Eigen::Vector3f::Zero());
    Tbc_ = Tbc_ * T_r1_u1.inverse();
  }
}

ostream& operator<<(std::ostream& output, const Settings& settings) {
  output << "SLAM settings: " << endl;

  output << "\t-Camera 1 parameters (";
  if (settings.cameraType_ == Settings::PinHole) {
    output << "Pinhole";

  } else if (settings.cameraType_ == Settings::Rectified) {
    output << "Rectified";
  } else {
    output << "Kannala-Brandt";
  }
  output << ")" << ": [";
  for (size_t i = 0; i < settings.originalCalib1_->size(); i++) {
    output << " " << settings.originalCalib1_->getParameter(i);
  }
  output << " ]" << endl;

  if (!settings.vPinHoleDistorsion1_.empty()) {
    output << "\t-Camera 1 distortion parameters: [ ";
    for (float d : settings.vPinHoleDistorsion1_) {
      output << " " << d;
    }
    output << " ]" << endl;
  }

  if ((settings.sensor_.isStereo()) &&
      (settings.cameraType_ != Settings::Rectified)) {
    output << "\t-Camera 2 parameters (";
    if (settings.cameraType_ == Settings::PinHole ||
        settings.cameraType_ == Settings::Rectified) {
      output << "Pinhole";
    } else {
      output << "Kannala-Brandt";
    }
    output << "" << ": [";
    for (size_t i = 0; i < settings.originalCalib2_->size(); i++) {
      output << " " << settings.originalCalib2_->getParameter(i);
    }
    output << " ]" << endl;

    if (!settings.vPinHoleDistorsion2_.empty()) {
      output << "\t-Camera 2 distortion parameters: [ ";
      for (float d : settings.vPinHoleDistorsion2_) {
        output << " " << d;
      }
      output << " ]" << endl;
    }
  }

  output << "\t-Original image size: [ " << settings.originalImSize_.width
         << " , " << settings.originalImSize_.height << " ]" << endl;
  output << "\t-Current image size: [ " << settings.newImSize_.width << " , "
         << settings.newImSize_.height << " ]" << endl;

  if (settings.bNeedToRectify_) {
    output << "\t-Camera 1 parameters after rectification: [ ";
    for (size_t i = 0; i < settings.calibration1_->size(); i++) {
      output << " " << settings.calibration1_->getParameter(i);
    }
    output << " ]" << endl;
  } else if (settings.bNeedToResize1_) {
    output << "\t-Camera 1 parameters after resize: [ ";
    for (size_t i = 0; i < settings.calibration1_->size(); i++) {
      output << " " << settings.calibration1_->getParameter(i);
    }
    output << " ]" << endl;

    if ((settings.sensor_ == SensorType::STEREO ||
         settings.sensor_ == SensorType::IMU_STEREO) &&
        settings.cameraType_ == Settings::KannalaBrandt) {
      output << "\t-Camera 2 parameters after resize: [ ";
      for (size_t i = 0; i < settings.calibration2_->size(); i++) {
        output << " " << settings.calibration2_->getParameter(i);
      }
      output << " ]" << endl;
    }
  }

  // Stereo stuff
  if (settings.sensor_.isStereo()) {
    output << "\t-Stereo baseline: " << settings.b_ << endl;
    output << "\t-Stereo depth threshold : " << settings.thDepth_ << endl;

    if (settings.cameraType_ == Settings::KannalaBrandt) {
      auto vOverlapping1 =
          dynamic_cast<KannalaBrandt8*>(settings.calibration1_.get())
              ->mvLappingArea;
      auto vOverlapping2 =
          dynamic_cast<KannalaBrandt8*>(settings.calibration2_.get())
              ->mvLappingArea;
      output << "\t-Camera 1 overlapping area: [ " << vOverlapping1[0] << " , "
             << vOverlapping1[1] << " ]" << endl;
      output << "\t-Camera 2 overlapping area: [ " << vOverlapping2[0] << " , "
             << vOverlapping2[1] << " ]" << endl;
    }
  }

  if (settings.sensor_.isImu()) {
    output << "\t-Gyro noise: " << settings.noiseGyro_ << endl;
    output << "\t-Accelerometer noise: " << settings.noiseAcc_ << endl;
    output << "\t-Gyro walk: " << settings.gyroWalk_ << endl;
    output << "\t-Accelerometer walk: " << settings.accWalk_ << endl;
    output << "\t-IMU frequency: " << settings.imuFrequency_ << endl;
  }

  if (settings.sensor_.isRGBD()) {
    output << "\t-RGB-D depth map factor: " << settings.depthMapFactor_ << endl;
  }

  output << "\t-Features per image: " << settings.nFeatures_ << endl;
  output << "\t-ORB scale factor: " << settings.scaleFactor_ << endl;
  output << "\t-ORB number of scales: " << settings.nLevels_ << endl;
  output << "\t-Initial FAST threshold: " << settings.initThFAST_ << endl;
  output << "\t-Min FAST threshold: " << settings.minThFAST_ << endl;

  output << "\t-Loop closing: " << (settings.loopClosing_ ? "YES" : "NO")
         << endl;

  return output;
}
};  // namespace ORB_SLAM3
