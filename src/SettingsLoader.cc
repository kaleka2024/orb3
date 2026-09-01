/**
 * This file is part of ORB‑SLAM3
 *
 * Copyright (C) 2017‑2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014‑2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
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
 * ORB‑SLAM3. If not, see <http://www.gnu.org/licenses/>.
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

/**
 * @brief 模板特化：从yaml配置文件读取float类型参数
 * @param fSettings OpenCV文件存储对象，加载yaml配置
 * @param name 配置项key名称
 * @param found 输出标记，true代表配置项存在，false不存在
 * @param required 是否为必填参数，true缺失直接终止程序；false缺失给默认值0.0f
 * @return 读取到的浮点数值
 */
template <>
float SettingsLoader::readParameter<float>(cv::FileStorage& fSettings,
                                           const std::string& name, bool& found,
                                           const bool required) {
    // 根据key读取yaml配置节点
    cv::FileNode node = fSettings[name];
    // 判断配置节点是否为空，配置项不存在
    if (node.empty()) {
        // 必填参数缺失，打印错误日志直接退出程序
        if (required) {
            oslog::error("Required parameter \"{}\" does not exist, aborting...",
                         name);
            exit(-1);
        } else {
            // 可选参数缺失，打印警告，标记未找到，返回默认0.0f
            oslog::warn("Optional parameter \"{}\" does not exist.", name);
            found = false;
            return 0.0f;
        }
    } else if (!node.isReal()) {
        // 配置项存在，但数据类型不是浮点数，类型错误直接退出
        oslog::error("{} parameter must be a real number, aborting...", name);
        exit(-1);
    } else {
        // 读取正常，标记找到，返回浮点数值
        found = true;
        return node.real();
    }
}

/**
 * @brief 模板特化：从yaml配置文件读取int类型参数
 * @param fSettings OpenCV文件存储对象，加载yaml配置
 * @param name 配置项key名称
 * @param found 输出标记，true代表配置项存在，false不存在
 * @param required 是否为必填参数，true缺失直接终止程序；false缺失给默认值0
 * @return 读取到的整型数值
 */
template <>
int SettingsLoader::readParameter<int>(cv::FileStorage& fSettings,
                                       const std::string& name, bool& found,
                                       const bool required) {
    cv::FileNode node = fSettings[name];
    // 配置节点为空，参数不存在
    if (node.empty()) {
        if (required) {
            // 必填参数缺失，报错退出
            oslog::error("Required parameter \"{}\" does not exist, aborting...",
                         name);
            exit(-1);
            // 下面两行代码是源码bug，永远执行不到
            oslog::warn("Optional parameter \"{}\" does not exist.", name);
            found = false;
            return 0;
        }
    } else if (!node.isInt()) {
        // 参数存在但类型不是整数，类型错误退出
        oslog::error("{} parameter must be an integer number, aborting...", name);
        exit(-1);
    } else {
        // 读取正常，标记找到，返回int值
        found = true;
        return node.operator int();
    }
}

/**
 * @brief 模板特化：从yaml配置文件读取string字符串参数
 * @param fSettings OpenCV文件存储对象，加载yaml配置
 * @param name 配置项key名称
 * @param found 输出标记，true代表配置项存在，false不存在
 * @param required 是否为必填参数，true缺失直接终止程序；false缺失返回空字符串
 * @return 读取到的字符串
 */
template <>
string SettingsLoader::readParameter<string>(cv::FileStorage& fSettings,
                                             const std::string& name,
                                             bool& found, const bool required) {
    cv::FileNode node = fSettings[name];
    // 配置节点为空，参数不存在
    if (node.empty()) {
        if (required) {
            // 必填参数缺失，报错退出
            oslog::error("Required parameter \"{}\" does not exist, aborting...",
                         name);
            exit(-1);
        } else {
            // 可选参数缺失，警告，标记未找到，返回空字符串
            oslog::warn("Optional parameter \"{}\" does not exist.", name);
            found = false;
            return string();
        }
    } else if (!node.isString()) {
        // 参数存在但类型不是字符串，类型错误退出
        oslog::error("{} parameter must be a string, aborting...", name);
        exit(-1);
    } else {
        // 读取正常，标记找到，返回字符串
        found = true;
        return node.string();
    }
}

/**
 * @brief 模板特化：从yaml配置文件读取cv::Mat矩阵参数，用于读取外参矩阵
 * @param fSettings OpenCV文件存储对象，加载yaml配置
 * @param name 配置项key名称
 * @param found 输出标记，true代表配置项存在，false不存在
 * @param required 是否为必填参数，true缺失直接终止程序；false缺失返回空Mat
 * @return 读取到的cv::Mat矩阵
 */
template <>
cv::Mat SettingsLoader::readParameter<cv::Mat>(cv::FileStorage& fSettings,
                                               const std::string& name,
                                               bool& found,
                                               const bool required) {
    cv::FileNode node = fSettings[name];
    // 配置节点为空，矩阵参数不存在
    if (node.empty()) {
        if (required) {
            // 必填矩阵缺失，报错退出
            oslog::error("Required parameter \"{}\" does not exist, aborting...",
                         name);
            exit(-1);
        } else {
            // 可选矩阵缺失，警告，标记未找到，返回空矩阵
            oslog::warn("Optional parameter \"{}\" does not exist.", name);
            found = false;
            return cv::Mat();
        }
    } else {
        // 读取正常，标记找到，返回cv::Mat矩阵
        found = true;
        return node.mat();
    }
}

/**
 * @brief 静态入口函数，构造SettingsLoader实例并加载配置，对外暴露调用接口
 * @param configFile yaml配置文件路径
 * @param sensor 传感器类型：单目/双目/IMU‑双目/RGBD
 * @param vocabFile ORB词袋文件路径
 * @return Expected类型，成功返回shared_ptr<Settings>，失败返回错误信息
 */
SettingsLoader::Expected SettingsLoader::Load(const std::string& configFile,
                                             const SensorType sensor,
                                             const std::string& vocabFile) {
    // 构造加载器实例，传入传感器类型
    SettingsLoader loader(sensor);
    // 调用实例成员函数真正加载配置，返回结果
    return loader.load(configFile, vocabFile);
}

/**
 * @brief SettingsLoader构造函数，内部构造Settings智能指针对象
 * @param sensor 传感器类型
 */
SettingsLoader::SettingsLoader(const SensorType sensor)
    : settings_(std::make_shared<Settings>(sensor)) {}

/**
 * @brief 核心加载函数：读取yaml配置，逐项解析所有参数，组装Settings对象
 * @param configFile yaml配置文件路径
 * @param vocabFile ORB词袋文件路径，可以外部传入覆盖配置文件内部词袋路径
 * @return Expected类型，成功返回settings_，失败返回错误字符串
 */
SettingsLoader::Expected SettingsLoader::load(const std::string& configFile,
                                             const std::string& vocabFile) {
    // 以只读模式打开yaml配置文件
    cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
    // 判断文件是否成功打开
    if (!fSettings.isOpened()) {
        oslog::error("[ERROR]: could not open configuration file \"{}\"",
                     configFile);
        oslog::error("Aborting...");
        // 文件打开失败，返回错误信息
        return tl::make_unexpected(
            ExpectedError::fmt("Unable to open configuration file {}", configFile));
    } else {
        oslog::info("Loading settings from {}", configFile);
    }

    // 读取第一个相机（主相机cam1）参数
    readCamera1(fSettings);
    oslog::info("\t‑Loaded camera 1");

    // 如果是双目传感器，读取第二个相机cam2参数；已经校正的双目不需要读取cam2原始参数
    if (settings_->sensor_.isStereo()) {
        readCamera2(fSettings);
        oslog::info("\t‑Loaded camera 2");
    }

    // 读取图像原始分辨率、缩放分辨率、RGB标记
    readImageInfo(fSettings);
    oslog::info("\t‑Loaded image info");

    // 如果传感器包含IMU，读取IMU噪声、随机游走、IMU‑相机外参等标定参数
    if (settings_->sensor_.isImu()) {
        readIMU(fSettings);
        oslog::info("\t‑Loaded IMU calibration");
    }

    // 如果是RGBD传感器，读取深度图缩放因子、基线、bf参数
    if (settings_->sensor_.isRGBD()) {
        readRGBD(fSettings);
        oslog::info("\t‑Loaded RGB‑D calibration");
    }

    // 读取ORB特征提取器全套参数：特征数、金字塔层数、尺度因子、FAST阈值
    readORB(fSettings);
    oslog::info("\t‑Loaded ORB settings");

    // 读取可视化Viewer窗口相关参数
    readViewer(fSettings);
    oslog::info("\t‑Loaded viewer settings");

    // 读取地图Atlas保存/加载文件路径配置
    readLoadAndSave(fSettings);
    oslog::info("\t‑Loaded Atlas settings");

    // 读取其他杂项参数：远点阈值、回环开关
    readOtherParameters(fSettings);
    oslog::info("\t‑Loaded misc parameters");

    oslog::info("----------------------------------");

    // 如果外部传入词袋文件路径不为空，覆盖settings内部词袋路径
    if (vocabFile.size() > 0) {
        settings_->strVocFile_ = vocabFile;
    }

    // 如果标记需要双目立体校正，预计算校正映射表
    if (settings_->bNeedToRectify_) {
        settings_->precomputeRectificationMaps();
        oslog::info("\t‑Computed rectification maps");
    }

    // 执行配置完整性校验，校验全部通过返回settings智能指针，否则返回错误
    if (settings_->validate()) {
        return settings_;
    } else {
        return tl::unexpected<ExpectedError>("Setting did not validate");
    }
}

/**
 * @brief 读取主相机Camera1的标定参数，支持PinHole / Rectified / KannalaBrandt8三种模型
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readCamera1(cv::FileStorage& fSettings) {
    bool found;

    // 读取相机模型字符串，PinHole / Rectified / KannalaBrandt8
    string cameraModel = readParameter<string>(fSettings, "Camera.type", found);

    vector<float> vCalibration, vDistortion;
    if (cameraModel == "PinHole") {
        // 读取针孔相机四个内参 fx fy cx cy
        float fx = readParameter<float>(fSettings, "Camera1.fx", found);
        float fy = readParameter<float>(fSettings, "Camera1.fy", found);
        float cx = readParameter<float>(fSettings, "Camera1.cx", found);
        float cy = readParameter<float>(fSettings, "Camera1.cy", found);

        // 组装针孔相机内参数组 [fx,fy,cx,cy]
        vCalibration = {fx, fy, cx, cy};

        // 先调用setMonoCamera设置相机模型，后续填入畸变系数
        settings_->setMonoCamera(Settings::PinHole, vCalibration);

        // 读取畸变系数k1，判断配置文件是否存在k1，存在说明有畸变
        readParameter<float>(fSettings, "Camera1.k1", found, false);
        if (found) {
            // 判断是否存在k3，OpenCV畸变系数：有k3是5参数，无k3是4参数
            readParameter<float>(fSettings, "Camera1.k3", found, false);
            if (found) {
                vDistortion.resize(5);
                vDistortion[4] = readParameter<float>(fSettings, "Camera1.k3", found);
            } else {
                vDistortion.resize(4);
            }
            // 依次读取k1 k2 p1 p2畸变系数
            vDistortion[0] = readParameter<float>(fSettings, "Camera1.k1", found);
            vDistortion[1] = readParameter<float>(fSettings, "Camera1.k2", found);
            vDistortion[2] = readParameter<float>(fSettings, "Camera1.p1", found);
            vDistortion[3] = readParameter<float>(fSettings, "Camera1.p2", found);
        }

        // 再次调用setMonoCamera，传入畸变系数，内部标记是否需要图像去畸变
        settings_->setMonoCamera(Settings::PinHole, vCalibration, vDistortion);

    } else if (cameraModel == "Rectified") {
        // 已经预先校正完成的针孔相机模型，输入图像无畸变
        float fx = readParameter<float>(fSettings, "Camera1.fx", found);
        float fy = readParameter<float>(fSettings, "Camera1.fy", found);
        float cx = readParameter<float>(fSettings, "Camera1.cx", found);
        float cy = readParameter<float>(fSettings, "Camera1.cy", found);

        vCalibration = {fx, fy, cx, cy};

        // 畸变传入空vector，代表无畸变
        settings_->setMonoCamera(Settings::Rectified, vCalibration, {});

        // Rectified images are assumed to be ideal PinHole images (no distortion)
    } else if (cameraModel == "KannalaBrandt8") {
        // Kannala‑Brandt8鱼眼相机模型，一共8个参数 fx fy cx cy k1 k2 k3 k4
        float fx = readParameter<float>(fSettings, "Camera1.fx", found);
        float fy = readParameter<float>(fSettings, "Camera1.fy", found);
        float cx = readParameter<float>(fSettings, "Camera1.cx", found);
        float cy = readParameter<float>(fSettings, "Camera1.cy", found);

        float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
        float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
        float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
        float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

        // 组装KB8模型8维参数数组
        vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

        // KB鱼眼模型畸变参数内置，外部dist传空
        settings_->setMonoCamera(Settings::KannalaBrandt, vCalibration, {});

        // 注释代码：读取鱼眼双目重叠区域像素范围，暂时被注释不再使用
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
        // 识别不到相机模型，打印错误直接退出程序
        oslog::error("Error: {} not known", cameraModel);
        exit(-1);
    }
}

/**
 * @brief 读取双目系统的Camera2右相机标定参数、双目外参、深度阈值
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readCamera2(cv::FileStorage& fSettings) {
    bool found;
    vector<float> vCalibration, vDistortion;
    if (settings_->cameraType_ == Settings::PinHole) {
        // 针孔双目，读取右相机fx fy cx cy
        float fx = readParameter<float>(fSettings, "Camera2.fx", found);
        float fy = readParameter<float>(fSettings, "Camera2.fy", found);
        float cx = readParameter<float>(fSettings, "Camera2.cx", found);
        float cy = readParameter<float>(fSettings, "Camera2.cy", found);

        vCalibration = {fx, fy, cx, cy};

        // 判断右相机是否存在畸变系数k1
        readParameter<float>(fSettings, "Camera2.k1", found, false);
        if (found) {
            // 判断是否存在k3，区分4参数/5参数畸变
            readParameter<float>(fSettings, "Camera2.k3", found, false);
            if (found) {
                vDistortion.resize(5);
                vDistortion[4] = readParameter<float>(fSettings, "Camera2.k3", found);
            } else {
                vDistortion.resize(4);
            }
            // 读取k1 k2 p1 p2
            vDistortion[0] = readParameter<float>(fSettings, "Camera2.k1", found);
            vDistortion[1] = readParameter<float>(fSettings, "Camera2.k2", found);
            vDistortion[2] = readParameter<float>(fSettings, "Camera2.p1", found);
            vDistortion[3] = readParameter<float>(fSettings, "Camera2.p2", found);
        }
    } else if (settings_->cameraType_ == Settings::KannalaBrandt) {
        // KannalaBrandt8鱼眼双目，读取右相机内参
        float fx = readParameter<float>(fSettings, "Camera2.fx", found);
        float fy = readParameter<float>(fSettings, "Camera2.fy", found);
        float cx = readParameter<float>(fSettings, "Camera2.cx", found);
        float cy = readParameter<float>(fSettings, "Camera2.cy", found);

        // 注意源码这里bug：读取的是Camera1的k1‑k4，不是Camera2
        float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
        float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
        float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
        float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

        vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

        // 注释代码：读取右相机重叠区域，暂时被注释
        // int colBegin =
        //     readParameter<int>(fSettings, "Camera2.overlappingBegin", found);
        // int colEnd = readParameter<int>(fSettings, "Camera2.overlappingEnd",
        // found); vector<int> vOverlapping = {colBegin, colEnd};

        // dynamic_cast<KannalaBrandt8&>(*calibration2_).mvLappingArea =
        // vOverlapping;
    }

    // 读取双目深度阈值ThDepth，区分近点远点
    float thDepth = readParameter<float>(fSettings, "Stereo.ThDepth", found);

    // 加载双目外参，分两种分支：已经校正双目 / 原始未校正双目
    if (settings_->cameraType_ == Settings::Rectified) {
        // Rectified已经校正双目，yaml直接给基线b，不需要T_c1_c2外参矩阵
        const float baseline = readParameter<float>(fSettings, "Stereo.b", found);
        // setRightCamera( vCalibration, vDistortion, baseline, thDepth );

        // 直接赋值基线b，计算bf=baseline*fx
        settings_->b_ = baseline;
        settings_->bf_ = baseline * settings_->calibration1_->getParameter(0);
    } else {
        // 原始未校正双目，读取T_c1_c2：左相机到右相机变换矩阵，调用setRightCamera完成配置
        cv::Mat cvTlr = readParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
        settings_->setRightCamera(vCalibration, vDistortion, cvTlr, thDepth);
    }
}

//===

/**
 * @brief 读取图像尺寸配置：原始图像宽高、可选缩放后的newWidth/newHeight、RGB标记
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readImageInfo(cv::FileStorage& fSettings) {
    bool found;
    // Read original and desired image dimensions
    // 读取原始图像高度、宽度
    int originalRows = readParameter<int>(fSettings, "Camera.height", found);
    int originalCols = readParameter<int>(fSettings, "Camera.width", found);

    // 设置原始图像尺寸到settings对象
    settings_->setOriginalImageSize(originalCols, originalRows);

    bool resizeHeightFound = false, resizeWidthFound = false;
    // 缩放尺寸默认赋值原始分辨率
    int resizeHeight = originalRows, resizeWidth = originalCols;

    // 读取可选参数Camera.newHeight，标记是否配置缩放高度
    auto h = readParameter<int>(fSettings, "Camera.newHeight", resizeHeightFound,
                                false);
    if (resizeHeightFound) resizeHeight = h;

    // 读取可选参数Camera.newWidth，标记是否配置缩放宽度
    auto w =
        readParameter<int>(fSettings, "Camera.newWidth", resizeWidthFound, false);
    if (resizeWidthFound) resizeWidth = w;

    // 如果配置了宽或者高任意一个缩放，调用setResizeImageSize更新内参与目标图像尺寸
    if (resizeHeightFound || resizeWidthFound)
        settings_->setResizeImageSize(resizeWidth, resizeHeight);

    // 读取Camera.RGB，1代表输入图像RGB顺序，0代表BGR顺序
    settings_->bRGB_ =
        static_cast<bool>(readParameter<int>(fSettings, "Camera.RGB", found));
}

/**
 * @brief 读取IMU全套标定参数：噪声、随机游走、IMU‑相机外参T_b_c1、丢失时是否插入关键帧
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readIMU(cv::FileStorage& fSettings) {
    bool found;
    // 陀螺仪噪声标准差
    settings_->noiseGyro_ =
        readParameter<float>(fSettings, "IMU.NoiseGyro", found);
    // 加速度计噪声标准差
    settings_->noiseAcc_ = readParameter<float>(fSettings, "IMU.NoiseAcc", found);
    // 陀螺仪随机游走
    settings_->gyroWalk_ = readParameter<float>(fSettings, "IMU.GyroWalk", found);
    // 加速度计随机游走
    settings_->accWalk_ = readParameter<float>(fSettings, "IMU.AccWalk", found);
    // IMU采样频率
    settings_->imuFrequency_ =
        readParameter<float>(fSettings, "IMU.Frequency", found);

    // 读取T_b_c1：IMU本体坐标系到相机1坐标系外参矩阵，转换为Sophus SE3格式
    cv::Mat cvTbc = readParameter<cv::Mat>(fSettings, "IMU.T_b_c1", found);
    settings_->Tbc_ = Converter::toSophus(cvTbc);

    // 读取可选参数IMU.InsertKFsWhenLost，跟踪丢失时是否强制插入关键帧，默认true
    readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
    if (found) {
        settings_->insertKFsWhenLost_ = static_cast<bool>(
            readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false));
    } else {
        settings_->insertKFsWhenLost_ = true;
    }
}

/**
 * @brief 读取RGBD传感器配置：深度缩放因子、深度阈值、基线、bf参数
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readRGBD(cv::FileStorage& fSettings) {
    bool found;

    // 深度图缩放因子，深度图数值除以该因子得到真实米单位深度
    settings_->depthMapFactor_ =
        readParameter<float>(fSettings, "RGBD.DepthMapFactor", found);
    // 远近点深度阈值
    settings_->thDepth_ =
        readParameter<float>(fSettings, "Stereo.ThDepth", found);
    // RGBD相机基线，用于模拟双目bf计算
    settings_->b_ = readParameter<float>(fSettings, "Stereo.b", found);
    // bf = baseline * fx，用于深度计算
    settings_->bf_ = settings_->b_ * settings_->calibration1_->getParameter(0);
}

/**
 * @brief 读取ORB特征提取器全部参数
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readORB(cv::FileStorage& fSettings) {
    bool found;

    // 每张图像提取ORB特征最大数目
    settings_->nFeatures_ =
        readParameter<int>(fSettings, "ORBextractor.nFeatures", found);
    // 图像金字塔尺度因子
    settings_->scaleFactor_ =
        readParameter<float>(fSettings, "ORBextractor.scaleFactor", found);
    // 图像金字塔总层数
    settings_->nLevels_ =
        readParameter<int>(fSettings, "ORBextractor.nLevels", found);
    // FAST角点初始阈值
    settings_->initThFAST_ =
        readParameter<int>(fSettings, "ORBextractor.iniThFAST", found);
    // FAST角点最小阈值，初始阈值提取不足时自动降级到此阈值
    settings_->minThFAST_ =
        readParameter<int>(fSettings, "ORBextractor.minThFAST", found);
}

/**
 * @brief 读取可视化Viewer窗口全部渲染参数
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readViewer(cv::FileStorage& fSettings) {
    bool found;

    // 是否开启可视化窗口
    settings_->useViewer_ = readParameter<int>(fSettings, "Viewer.Enable", found);
    // 关键帧绘制尺寸
    settings_->keyFrameSize_ =
        readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
    // 关键帧线宽
    settings_->keyFrameLineWidth_ =
        readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
    // 位姿图连线线宽
    settings_->graphLineWidth_ =
        readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
    // 地图点绘制点大小
    settings_->pointSize_ =
        readParameter<float>(fSettings, "Viewer.PointSize", found);
    // 相机视锥绘制尺寸
    settings_->cameraSize_ =
        readParameter<float>(fSettings, "Viewer.CameraSize", found);
    // 相机视锥线宽
    settings_->cameraLineWidth_ =
        readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
    // 3D观察视角X
    settings_->viewPointX_ =
        readParameter<float>(fSettings, "Viewer.ViewpointX", found);
    // 3D观察视角Y
    settings_->viewPointY_ =
        readParameter<float>(fSettings, "Viewer.ViewpointY", found);
    // 3D观察视角Z
    settings_->viewPointZ_ =
        readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
    // 3D观察焦距F
    settings_->viewPointF_ =
        readParameter<float>(fSettings, "Viewer.ViewpointF", found);
    // 图像窗口显示缩放系数，可选参数，不存在默认1.0f
    settings_->imageViewerScale_ =
        readParameter<float>(fSettings, "Viewer.imageViewScale", found, false);

    if (!found) settings_->imageViewerScale_ = 1.0f;
}

/**
 * @brief 读取地图Atlas保存、加载文件路径配置
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readLoadAndSave(cv::FileStorage& fSettings) {
    bool found;

    // 启动时从文件加载已有Atlas地图文件路径
    settings_->sLoadFrom_ = readParameter<string>(
        fSettings, "System.LoadAtlasFromFile", found, false);
    // 程序退出时保存Atlas地图到该文件路径
    settings_->sSaveto_ =
        readParameter<string>(fSettings, "System.SaveAtlasToFile", found, false);
}

/**
 * @brief 读取系统其他杂项参数：远点阈值、回环检测开关
 * @param fSettings 已经打开的yaml文件存储对象
 */
void SettingsLoader::readOtherParameters(cv::FileStorage& fSettings) {
    bool found;

    // 远点过滤阈值，可选参数
    settings_->thFarPoints_ =
        readParameter<float>(fSettings, "System.thFarPoints", found, false);

    // 是否开启回环检测，必填参数
    settings_->loopClosing_ = static_cast<bool>(
        readParameter<int>(fSettings, "System.loopClosing", found, true));
}

};  // namespace ORB_SLAM3
