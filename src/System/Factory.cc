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
#include "System/Factory.h"   // SystemFactory工厂类头文件，本文件实现SystemFactory

#include <memory>            // C++智能指针 std::shared_ptr
#include <string>            // C++字符串 std::string

#include "System.h"          // ORB‑SLAM3核心System系统类

namespace ORB_SLAM3 {       // ORB‑SLAM3命名空间

using std::string;          // 简化写法，直接使用string代替std::string

/**
 * @brief 工厂创建System实例，传入已经构造校验完成的Settings配置对象
 * @param settings 配置参数智能指针对象，包含相机、IMU、传感器等全部配置
 * @param initFr 是否启用第一帧初始化标志
 * @param strSequence 序列数据集名称/路径字符串
 * @return SystemFactory::Expected 返回expected类型：成功返回std::shared_ptr<System>；失败携带错误信息
 * @note 不能使用std::make_shared，因为System构造函数为友元私有构造；分开调用initialize，内部依赖shared_from_this()
 */
SystemFactory::Expected SystemFactory::create(
    const std::shared_ptr<Settings> &settings, bool initFr,
    const string &strSequence)
{
    // 校验Settings配置对象参数合法性，检查参数范围、相机参数有效性等
    if (!settings->validate())
    {
        // 配置校验失败，返回unexpected错误对象，携带错误描述字符串
        return tl::make_unexpected(ExpectedError::fmt("Settings do not validate"));
    }

    // Cannot use make_shared with friend constructors?
    // 直接new System再包装为shared_ptr；System构造函数是私有友元，make_shared无法访问私有构造函数
    auto sys = std::shared_ptr<System>(new System(settings, initFr, strSequence));

    // Initialization must occur separately because we use shared_from_this
    // 必须单独调用initialize初始化；System内部initialize会调用shared_from_this()，要求对象已经被shared_ptr托管
    if (!sys->initialize())
    {
        // SLAM系统初始化失败，返回携带错误信息的unexpected
        return tl::make_unexpected(
            ExpectedError::fmt("Unable to initialize SLAM system"));
    }

    // 全部流程正常，返回构造完成并初始化完毕的System智能指针
    return sys;
}

/**
 * @brief 重载create接口：传入配置文件路径+传感器类型，内部加载Settings再调用上面的create
 * @param configFile yaml配置文件路径
 * @param sensor 传感器类型枚举：单目/双目/RGBD/IMU组合等
 * @param initFr 是否开启第一帧初始化
 * @param strSequence 数据集序列名称/路径
 * @return SystemFactory::Expected expected变体，成功返回System，失败返回错误信息
 */
SystemFactory::Expected SystemFactory::create(const std::string &configFile,
                                              const SensorType sensor,
                                              bool initFr,
                                              const string &strSequence)
{
    // 使用SettingsLoader工具从yaml配置文件加载配置，返回expected<shared_ptr<Settings>>
    auto exSettings = SettingsLoader::Load(configFile, sensor);

    // 判断加载配置是否失败，exSettings包含unexpected则进入分支
    if (!exSettings)
    {
        // 返回错误，提示无法加载配置文件
        return tl::make_unexpected(ExpectedError::fmt("Unable to load settings"));
    }

    // exSettings.value()取出成功的Settings对象，转发调用上面的create工厂函数
    return SystemFactory::create(exSettings.value(), initFr, strSequence);
}

/**
 * @brief 重载create接口：传入配置文件、词袋文件路径、传感器类型，加载Settings后创建System
 * @param configFile yaml配置文件路径
 * @param vocabFile ORB词袋vocabulary文件路径
 * @param sensor 传感器类型枚举
 * @param initFr 是否开启第一帧初始化
 * @param strSequence 数据集序列名称/路径
 * @return SystemFactory::Expected expected变体，成功返回System智能指针，失败返回错误
 */
SystemFactory::Expected SystemFactory::create(const std::string &configFile,
                                              const std::string &vocabFile,
                                              const SensorType sensor,
                                              bool initFr,
                                              const string &strSequence)
{
    // 加载配置，同时传入词袋文件路径给SettingsLoader
    auto exSettings = SettingsLoader::Load(configFile, sensor, vocabFile);

    // 配置加载失败，返回错误信息
    if (!exSettings)
    {
        return tl::make_unexpected(ExpectedError::fmt("Unable to load settings"));
    }

    // 取出成功加载的Settings智能指针
    auto settings = exSettings.value();
    // 转发调用第一个create重载函数完成System对象创建与初始化
    return SystemFactory::create(settings, initFr, strSequence);
}

}  // namespace ORB_SLAM3
