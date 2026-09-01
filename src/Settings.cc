/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB‑SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB‑SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB‑SLAM3. If not, see http://www.gnu.org/licenses/.
 */
#include "Settings.h"
#include <filesystem> // NOLINT {build/c++17}
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

// Settings构造函数：根据传感器类型初始化全部配置标志与默认参数
Settings::Settings(const SensorType sensor)
    : bNeedToUndistort_(false),        // 是否需要对图像执行去畸变
      bNeedToRectify_(false),          // 是否需要双目立体校正
      bNeedToResize1_(false),          // 是否需要对cam1图像做缩放
      bNeedToResize2_(false),          // 是否需要对cam2图像做缩放
      loopClosing_(true),              // 是否开启回环检测功能
      sensor_(sensor),                 // 传感器类型：单目/双目/IMU‑双目/RGBD等
      imageViewerScale_(1.0f),         // 可视化窗口图像缩放系数
      imuFrequency_(1.0f),             // IMU采样频率，默认1Hz
      thDepth_(5)                      // 双目深度阈值，区分近/远点
{
    // if(bNeedToRectify_){
    //     precomputeRectificationMaps();
    //     cout << "\t‑Computed rectification maps" << endl;
    // }
}

// Settings析构函数，无自定义资源释放，空实现
Settings::~Settings() { ; }

// validate：校验关键配置项是否完整合法，SLAM启动前调用
bool Settings::validate(void) {
    // Check all of the variables that are assumed to be set
    if (!calibration1_) return false;        // cam1相机模型指针不能为空
    if (!originalCalib1_) return false;     // cam1原始标定参数备份不能为空

    if (originalImSize_.width == 0) return false;  // 原始图像宽度不能为0
    if (originalImSize_.height == 0) return false; // 原始图像高度不能为0

    if (strVocFile_.size() == 0) {          // ORB词袋文件路径未配置
        oslog::warn("Vocab file not specified");
        return false;
    } else if (!std::filesystem::exists(strVocFile_)) { // 词袋文件物理不存在
        oslog::warn("Vocab file {} does not exist.", strVocFile_);
        return false;
    }

    return true; // 全部校验通过
}

//===
// setMonoCamera：配置单目相机参数，支持针孔、已校正针孔、Kannala‑Brandt鱼眼
void Settings::setMonoCamera(CameraType type, const std::vector<float>& k,
                             const std::vector<float>& dist) {
    cameraType_ = type; // 设置当前相机模型类型

    if (cameraType_ == PinHole) { // 普通针孔相机，带畸变
        calibration1_ = std::make_shared<Pinhole>(k);      // 工作使用的相机模型
        originalCalib1_ = std::make_shared<Pinhole>(k);    // 保存原始标定参数备份

        vPinHoleDistorsion1_ = dist; // 保存cam1畸变系数

        // Check if we need to correct distortion from the images
        // 单目模式且畸变系数非空，标记需要做图像去畸变
        if (sensor_.isMonocular() && vPinHoleDistorsion1_.size() != 0) {
            bNeedToUndistort_ = true;
        }
    } else if (cameraType_ == Rectified) { // 输入图像已经完成立体校正，理想针孔无畸变
        calibration1_ = std::make_shared<Pinhole>(k);
        originalCalib1_ = std::make_shared<Pinhole>(k);
        // Rectified images are assumed to be ideal PinHole images (no distortion)
    } else if (cameraType_ == KannalaBrandt) { // Kannala‑Brandt8鱼眼相机模型
        if (k.size() != 8) { // KB模型固定需要8个参数
            oslog::error("Incorrect number of params for KannalaBrandt");
            return;
        }
        calibration1_ = std::make_shared<KannalaBrandt8>(k);
        originalCalib1_ = std::make_shared<KannalaBrandt8>(k);

        // \todo{AMM} Commented this out while making SettingsLoader, did not feel
        // like converting it at the time
        // if (sensor_.isStereo()) {
        //   int colBegin =
        //       readParameter<int>(fSettings, "Camera1.overlappingBegin", found);
        //   int colEnd =
        //       readParameter<int>(fSettings, "Camera1.overlappingEnd", found);
        //   vector<int> vOverlapping = {colBegin, colEnd};
        //   dynamic_cast<KannalaBrandt8&>(*calibration1_).mvLappingArea =
        //       vOverlapping;
        // }
    } else { // 未知相机模型，直接退出程序
        oslog::error("Error: {} not known", static_cast<int>(type));
        exit(-1);
    }
}

// setRightCamera：配置双目系统的右相机，输入内外参、左右相机外参T_c1_c2、深度阈值
void Settings::setRightCamera(const std::vector<float>& k2,
                              const std::vector<float>& dist2,
                              const cv::Mat& T_c1_c2, float thDepth) {
    if (cameraType_ == PinHole) { // 针孔双目，标记需要执行立体校正
        bNeedToRectify_ = true;
        calibration2_ = std::make_shared<Pinhole>(k2);     // 右相机工作模型
        originalCalib2_ = std::make_shared<Pinhole>(k2);   // 右相机原始参数备份
        vPinHoleDistorsion2_ = dist2;                      // 右相机畸变系数
        // Note to self, for Rectified cameras, the calibration is the same for both
        // L and R
    } else if (cameraType_ == KannalaBrandt) { // KB鱼眼双目配置右相机
        calibration2_ = std::make_shared<KannalaBrandt8>(k2);
        originalCalib2_ = std::make_shared<KannalaBrandt8>(k2);

        // \todo{AMM} Commented this out while making SettingsLoader, did not feel
        // like converting it at the time
        // int colBegin =
        //     readParameter<int>(fSettings, "Camera2.overlappingBegin", found);
        // int colEnd = readParameter<int>(fSettings, "Camera2.overlappingEnd",
        // found); vector<int> vOverlapping = {colBegin, colEnd};
        // dynamic_cast<KannalaBrandt8&>(*calibration2_).mvLappingArea =
        // vOverlapping;
    }

    cv::Mat cvTlr = T_c1_c2;                  // cv::Mat格式：左相机到右相机变换T_lr
    Tlr_ = Converter::toSophus(cvTlr);        // 转换为Sophus::SE3f格式存储

    // TODO: also search for Trl and invert if necessary
    b_ = Tlr_.translation().norm();           // 求取双目基线长度，平移向量模长
    bf_ = b_ * calibration1_->getParameter(0);// bf = baseline * fx，双目恢复深度核心参数

    thDepth_ = thDepth;                       // 更新双目远近点区分阈值
}

// setStereoRectifiedCamera：配置已经预先校正好的双目，不需要再做cv::stereoRectify
void Settings::setStereoRectifiedCamera(const std::vector<float>& k,
                                        float baseline, float thDepth) {
    cameraType_ = Rectified; // 设置相机类型为已校正模式

    calibration1_ = std::make_shared<Pinhole>(k);
    originalCalib1_ = std::make_shared<Pinhole>(k);

    b_ = baseline;                             // 直接赋值基线
    bf_ = b_ * calibration1_->getParameter(0); // 计算bf值
}

//===
// setOriginalImageSize：保存原始图像分辨率，未做缩放、校正前的尺寸
void Settings::setOriginalImageSize(int width, int height) {
    // Read original and desired image dimensions
    int originalRows = height;
    int originalCols = width;

    originalImSize_.width = originalCols;  // 原始图像宽
    originalImSize_.height = originalRows; // 原始图像高

    newImSize_ = originalImSize_;          // 目标输出尺寸先初始化为原始尺寸
}

// setResizeImageSize：设置运行时图像缩放后的分辨率，同步更新相机内参
void Settings::setResizeImageSize(int width, int height) {
    if (!calibration1_) { // 必须先加载相机标定参数，才能做缩放
        throw std::runtime_error(
            "setResizeImageSize() must be called after loading camera parameters");
    }

    // 高度发生变化
    if (height != originalImSize_.height) {
        bNeedToResize1_ = true;               // 标记需要缩放图像
        newImSize_.height = height;           // 更新目标高度

        if (!bNeedToRectify_) { // 如果不需要立体校正，手动缩放内参；校正模式下内参由stereoRectify输出P矩阵覆盖
            // Update calibration
            const float scaleRowFactor = static_cast<float>(newImSize_.height) /
                                         static_cast<float>(originalImSize_.height);
            // cy 乘以行缩放因子
            calibration1_->setParameter(
                calibration1_->getParameter(1) * scaleRowFactor, 1);
            // fy 乘以行缩放因子
            calibration1_->setParameter(
                calibration1_->getParameter(3) * scaleRowFactor, 3);

            // 双目且非Rectified模式，同步更新右相机内参fy、cy
            if ((sensor_.isStereo()) && cameraType_ != Rectified) {
                calibration2_->setParameter(
                    calibration2_->getParameter(1) * scaleRowFactor, 1);
                calibration2_->setParameter(
                    calibration2_->getParameter(3) * scaleRowFactor, 3);
            }
        }
    }

    // 宽度发生变化
    if (width != originalImSize_.width) {
        bNeedToResize1_ = true;                // 标记需要缩放图像
        newImSize_.width = width;              // 更新目标宽度

        if (!bNeedToRectify_) { // 非校正模式手动更新内参
            // Update calibration
            const float scaleColFactor = static_cast<float>(newImSize_.width) /
                                         static_cast<float>(originalImSize_.width);
            // fx 乘以列缩放因子
            calibration1_->setParameter(
                calibration1_->getParameter(0) * scaleColFactor, 0);
            // cx 乘以列缩放因子
            calibration1_->setParameter(
                calibration1_->getParameter(2) * scaleColFactor, 2);

            // 双目非Rectified，同步更新右相机fx、cx
            if ((sensor_.isStereo()) && cameraType_ != Rectified) {
                calibration2_->setParameter(
                    calibration2_->getParameter(0) * scaleColFactor, 0);
                calibration2_->setParameter(
                    calibration2_->getParameter(2) * scaleColFactor, 2);

                // KB鱼眼相机，同步缩放左右相机重叠区域的像素边界
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

// precomputeRectificationMaps：预计算双目立体校正映射表，OpenCV stereoRectify+initUndistortRectifyMap
void Settings::precomputeRectificationMaps() {
    oslog::trace(
        "[Settings::precomputeRectificationMaps] Precomputing rectification "
        "maps");

    // Precompute rectification maps, new calibrations, ...
    // 获取左相机原始内参K，转64位浮点
    cv::Mat K1 = dynamic_cast<Pinhole&>(*calibration1_).toK();
    K1.convertTo(K1, CV_64F);
    // 获取右相机原始内参K，转64位浮点
    cv::Mat K2 = dynamic_cast<Pinhole&>(*calibration2_).toK();
    K2.convertTo(K2, CV_64F);

    cv::Mat cvTlr;
    // Tlr是左到右，stereoRectify需要右到左T_rl，取inverse，转cv::Mat 3x4
    cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);
    cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3); // 右到左旋转矩阵R_rl
    R12.convertTo(R12, CV_64F);
    cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);         // 右到左平移向量t_rl
    t12.convertTo(t12, CV_64F);

    cv::Mat R_r1_u1, R_r2_u2; // 左右相机校正旋转矩阵
    cv::Mat P1, P2, Q;        // 校正后投影矩阵P1/P2，重投影Q矩阵

    // OpenCV双目立体校正主函数，CALIB_ZERO_DISPARITY令主点水平对齐
    cv::stereoRectify(K1, camera1DistortionCoef(), K2, camera2DistortionCoef(),
                      originalImSize_, R12, t12, R_r1_u1, R_r2_u2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, -1, newImSize_);

    // 计算左相机去畸变+校正映射 M1l_ M2l_
    cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1,
                                P1.rowRange(0, 3).colRange(0, 3), newImSize_,
                                CV_32F, M1l_, M2l_);
    // 计算右相机去畸变+校正映射 M1r_ M2r_
    cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2,
                                P2.rowRange(0, 3).colRange(0, 3), newImSize_,
                                CV_32F, M1r_, M2r_);

    // Update calibration
    // (updating calibrations in place can lead to problems. Ask me how I
    // know...)
    // 将校正后的投影矩阵P1赋值给calibration1_，作为运行时内参
    calibration1_->setParameter(P1.at<double>(0, 0), 0);
    calibration1_->setParameter(P1.at<double>(1, 1), 1);
    calibration1_->setParameter(P1.at<double>(0, 2), 2);
    calibration1_->setParameter(P1.at<double>(1, 2), 3);

    // Update bf：基线乘校正后的fx，更新bf
    bf_ = b_ * P1.at<double>(0, 0);

    // Update relative pose between camera 1 and IMU if necessary
    // IMU‑双目模式，需要修正Tbc：相机做了校正旋转，IMU‑相机外参要同步补偿
    if (sensor_ == SensorType::IMU_STEREO) {
        Eigen::Matrix3f eigenR_r1_u1;
        cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
        Sophus::SE3f T_r1_u1(eigenR_r1_u1, Eigen::Vector3f::Zero());
        Tbc_ = Tbc_ * T_r1_u1.inverse();
    }
}

// 重载输出流运算符，打印全部Settings配置信息，便于调试日志输出
ostream& operator<<(std::ostream& output, const Settings& settings) {
    output << "SLAM settings: " << endl;

    output << "\t‑Camera 1 parameters (";
    if (settings.cameraType_ == Settings::PinHole) {
        output << "Pinhole";
    } else if (settings.cameraType_ == Settings::Rectified) {
        output << "Rectified";
    } else {
        output << "Kannala‑Brandt";
    }
    output << ")" << ": [";
    for (size_t i = 0; i < settings.originalCalib1_->size(); i++) {
        output << " " << settings.originalCalib1_->getParameter(i);
    }
    output << " ]" << endl;

    // 打印cam1畸变系数
    if (!settings.vPinHoleDistorsion1_.empty()) {
        output << "\t‑Camera 1 distortion parameters: [ ";
        for (float d : settings.vPinHoleDistorsion1_) {
            output << " " << d;
        }
        output << " ]" << endl;
    }

    // 双目并且不是Rectified模式，打印cam2参数
    if ((settings.sensor_.isStereo()) &&
        (settings.cameraType_ != Settings::Rectified)) {
        output << "\t‑Camera 2 parameters (";
        if (settings.cameraType_ == Settings::PinHole ||
            settings.cameraType_ == Settings::Rectified) {
            output << "Pinhole";
        } else {
            output << "Kannala‑Brandt";
        }
        output << "" << ": [";
        for (size_t i = 0; i < settings.originalCalib2_->size(); i++) {
            output << " " << settings.originalCalib2_->getParameter(i);
        }
        output << " ]" << endl;

        // 打印cam2畸变系数
        if (!settings.vPinHoleDistorsion2_.empty()) {
            output << "\t‑Camera 2 distortion parameters: [ ";
            for (float d : settings.vPinHoleDistorsion2_) {
                output << " " << d;
            }
            output << " ]" << endl;
        }
    }

    // 打印原始图像尺寸、当前工作图像尺寸
    output << "\t‑Original image size: [ " << settings.originalImSize_.width
           << " , " << settings.originalImSize_.height << " ]" << endl;
    output << "\t‑Current image size: [ " << settings.newImSize_.width << " , "
           << settings.newImSize_.height << " ]" << endl;

    // 打印校正之后的cam1内参
    if (settings.bNeedToRectify_) {
        output << "\t‑Camera 1 parameters after rectification: [ ";
        for (size_t i = 0; i < settings.calibration1_->size(); i++) {
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << endl;
    } else if (settings.bNeedToResize1_) { // 打印缩放之后cam1内参
        output << "\t‑Camera 1 parameters after resize: [ ";
        for (size_t i = 0; i < settings.calibration1_->size(); i++) {
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << endl;

        // 双目KB鱼眼打印缩放后cam2参数
        if ((settings.sensor_ == SensorType::STEREO ||
             settings.sensor_ == SensorType::IMU_STEREO) &&
            settings.cameraType_ == Settings::KannalaBrandt) {
            output << "\t‑Camera 2 parameters after resize: [ ";
            for (size_t i = 0; i < settings.calibration2_->size(); i++) {
                output << " " << settings.calibration2_->getParameter(i);
            }
            output << " ]" << endl;
        }
    }

    // Stereo stuff 双目相关参数输出
    if (settings.sensor_.isStereo()) {
        output << "\t‑Stereo baseline: " << settings.b_ << endl;
        output << "\t‑Stereo depth threshold : " << settings.thDepth_ << endl;

        // KB鱼眼输出左右相机重叠区域
        if (settings.cameraType_ == Settings::KannalaBrandt) {
            auto vOverlapping1 =
                dynamic_cast<KannalaBrandt8*>(settings.calibration1_.get())
                    ->mvLappingArea;
            auto vOverlapping2 =
                dynamic_cast<KannalaBrandt8*>(settings.calibration2_.get())
                    ->mvLappingArea;
            output << "\t‑Camera 1 overlapping area: [ " << vOverlapping1[0] << " , "
                   << vOverlapping1[1] << " ]" << endl;
            output << "\t‑Camera 2 overlapping area: [ " << vOverlapping2[0] << " , "
                   << vOverlapping2[1] << " ]" << endl;
        }
    }

    // IMU传感器，输出IMU噪声、随机游走、采样频率
    if (settings.sensor_.isImu()) {
        output << "\t‑Gyro noise: " << settings.noiseGyro_ << endl;
        output << "\t‑Accelerometer noise: " << settings.noiseAcc_ << endl;
        output << "\t‑Gyro walk: " << settings.gyroWalk_ << endl;
        output << "\t‑Accelerometer walk: " << settings.accWalk_ << endl;
        output << "\t‑IMU frequency: " << settings.imuFrequency_ << endl;
    }

    // RGBD传感器输出深度缩放因子
    if (settings.sensor_.isRGBD()) {
        output << "\t‑RGB‑D depth map factor: " << settings.depthMapFactor_ << endl;
    }

    // ORB特征提取相关参数
    output << "\t‑Features per image: " << settings.nFeatures_ << endl;
    output << "\t‑ORB scale factor: " << settings.scaleFactor_ << endl;
    output << "\t‑ORB number of scales: " << settings.nLevels_ << endl;
    output << "\t‑Initial FAST threshold: " << settings.initThFAST_ << endl;
    output << "\t‑Min FAST threshold: " << settings.minThFAST_ << endl;

    // 回环开关状态
    output << "\t‑Loop closing: " << (settings.loopClosing_ ? "YES" : "NO")
           << endl;

    return output;
}

}; // namespace ORB_SLAM3
