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
#pragma once
#include <cstdint>     // 固定宽度整数类型 uint8_t
#include <string>      // std::string字符串

namespace ORB_SLAM3 {

// Rather than a conventional enum class, use this pattern which lets us
// add member functions
/**
 * @brief SensorType 传感器类型包装类
 * @details 没有直接使用原生enum class，采用包装结构体模式，在枚举基础上增加成员判断函数；
 * 区分纯视觉模式与视觉+IMU惯性模式，提供isImu/isRGBD/isStereo/isMonocular判断接口，以及字符串打印接口。
 */
class SensorType {
 public:
  /**
   * @brief 底层枚举原始值，uint8_t节省内存
   */
  enum Value : uint8_t {
    MONOCULAR = 0,        ///< 纯单目视觉，无IMU
    STEREO = 1,           ///< 纯双目视觉，无IMU
    RGBD = 2,             ///< 纯RGBD深度相机，无IMU
    IMU_MONOCULAR = 3,    ///< 单目+IMU，单目惯性VIO
    IMU_STEREO = 4,       ///< 双目+IMU，双目惯性VIO
    IMU_RGBD = 5,         ///< RGBD+IMU，RGBD惯性VIO
  };

  SensorType() = delete;                                 ///< 删除默认无参构造，禁止无参实例化
  SensorType(Value stype) : value(stype) {}  // NOLINT {runtime/explicit}  ///< 枚举值构造，允许隐式转换，NOLINT关闭clang‑tidy explicit警告

  /**
   * @brief 相等运算符重载，SensorType对象之间判等
   * @param a 对比的SensorType对象
   * @return true类型相等；false不相等
   */
  constexpr bool operator==(SensorType a) const { return value == a.value; }
  /**
   * @brief 不等运算符重载，SensorType对象之间判不等
   * @param a 对比的SensorType对象
   * @return true类型不相等；false相等
   */
  constexpr bool operator!=(SensorType a) const { return value != a.value; }

  /**
   * @brief 对象与底层枚举Value判等
   * @param a 原始枚举Value值
   * @return true相等；false不相等
   */
  constexpr bool operator==(Value a) const { return value == a; }
  /**
   * @brief 对象与底层枚举Value判不等
   * @param a 原始枚举Value值
   * @return true不相等；false相等
   */
  constexpr bool operator!=(Value a) const { return value != a; }

  /**
   * @brief constexpr编译期判断：当前传感器是否搭载IMU惯性单元
   * @return true 是IMU惯性模式；false纯视觉
   */
  constexpr bool isImu() const {
    return (value == IMU_MONOCULAR || value == IMU_RGBD || value == IMU_STEREO);
  }

  /**
   * @brief constexpr编译期判断：是否RGBD传感器（含RGBD‑IMU）
   * @return true RGBD系；false非RGBD
   */
  constexpr bool isRGBD() const { return (value == RGBD || value == IMU_RGBD); }

  /**
   * @brief constexpr编译期判断：是否双目传感器（含双目‑IMU）
   * @return true 双目系；false非双目
   */
  constexpr bool isStereo() const {
    return (value == STEREO || value == IMU_STEREO);
  }

  /**
   * @brief constexpr编译期判断：是否单目传感器（含单目‑IMU）
   * @return true 单目系；false非单目
   */
  constexpr bool isMonocular() const {
    return (value == MONOCULAR || value == IMU_MONOCULAR);
  }

  /**
   * @brief 将传感器类型转为可读英文字符串，用于日志打印输出
   * @return std::string 传感器名字字符串，未知返回"(unknown)"
   */
  std::string toString() const {
    if (value == SensorType::MONOCULAR)
      return "Monocular";
    else if (value == SensorType::STEREO)
      return "Stereo";
    else if (value == SensorType::RGBD)
      return "RGB‑D";
    else if (value == SensorType::IMU_MONOCULAR)
      return "Monocular‑Inertial";
    else if (value == SensorType::IMU_STEREO)
      return "Stereo‑Inertial";
    else if (value == SensorType::IMU_RGBD)
      return "RGB‑D‑Inertial";

    return "(unknown)";
  }

 private:
  Value value;  ///< 保存实际传感器枚举数值
};

/**
 * @brief FileType 文件存储格式枚举
 */
enum class FileType { TEXT_FILE, BINARY_FILE };

}  // namespace ORB_SLAM3
