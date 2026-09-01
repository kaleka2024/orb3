/**  * This file is part of ORB‑SLAM3  *
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
// 头文件保护，防止头文件重复包含
#pragma once

// C++标准智能指针
#include <memory>
// C++标准字符串类，用于文件路径、序列名称
#include <string>

// SLAM配置参数管理头文件，Settings封装全部配置加载逻辑
#include "Settings.h"

namespace ORB_SLAM3 {

// 前向声明System主系统类，避免循环头文件依赖
class System;

/**
 * @brief System工厂类，负责创建ORB‑SLAM3 System实例
 * 提供多套静态create接口，支持新Settings对象、配置文件、兼容旧版API三种创建方式
 * 使用tl::expected做错误返回，成功返回System智能指针，失败携带错误信息ExpectedError
 */
class SystemFactory {
public:
  /**
   * @brief 别名定义，工厂函数返回值类型
   * tl::expected：C++期望类型，成功保存std::shared_ptr<System>；失败保存ExpectedError错误结构体
   */
  typedef tl::expected<std::shared_ptr<System>, ExpectedError> Expected;

  /**
   * @brief 工厂创建接口：传入已构造完成的Settings配置对象，生成System实例
   * @param settings 已经加载解析完毕的Settings配置智能指针
   * @param initFr 是否初始化帧记录模块（frame recorder）
   * @param strSequence 序列名称字符串，用于日志、保存文件命名，默认为空字符串
   * @return Expected tl::expected对象，成功持有System共享指针，失败携带错误码与错误信息
   */
  static Expected create(const std::shared_ptr<Settings> &settings,
                          bool initFr = false,
                          const std::string &strSequence = std::string());

  /**
   * @brief 工厂创建接口：传入配置文件路径与传感器类型，内部自动构建Settings对象再创建System
   * @param configFile yaml配置文件路径
   * @param sensor 传感器类型：MONOCULAR / STEREO / RGBD / IMU‑MONO / IMU‑STEREO
   * @param initFr 是否初始化帧记录模块
   * @param strSequence 序列名称，默认为空字符串
   * @return Expected tl::expected对象，成功持有System共享指针，失败携带错误信息
   */
  static Expected create(const std::string &configFile, const SensorType sensor,
                          bool initFr = false,
                          const std::string &strSequence = std::string());

  /**
   * @brief 兼容旧版API的工厂创建接口，传入配置文件、词袋文件路径、传感器类型
   * @note 保留该接口用于兼容老项目，新版本优先使用Settings对象版本接口
   * @param configFile yaml配置文件路径
   * @param vocabFile ORB词袋vocabulary文件路径
   * @param sensor 传感器类型
   * @param initFr 是否初始化帧记录模块
   * @param strSequence 序列名称，默认为空字符串
   * @return Expected tl::expected对象，成功持有System共享指针，失败携带错误信息
   */
  static Expected create(const std::string &configFile,
                          const std::string &vocabFile, const SensorType sensor,
                          bool initFr = false,
                          const std::string &strSequence = std::string());
};

}  // namespace ORB_SLAM3
