/**  * This file was added to ORB‑SLAM3  *
* Copyright (C) 2026b Aaron Marburg
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

// C++标准字符串，用于文件路径、校验和结果字符串
#include <string>

// ORB‑SLAM3自定义类型定义头文件，包含FileType枚举类型
#include "Types.h"

namespace ORB_SLAM3 {

/**
 * @brief 校验和子命名空间，提供文件校验码计算接口，用于地图文件完整性校验
 */
namespace Checksum {

/**
 * @brief 计算指定文件的校验和，用于SLAM地图存档文件完整性校验
 * @param filename 待计算校验和的文件路径字符串
 * @param type 文件类型枚举FileType，区分不同类型的文件（地图、配置等）
 * @return std::string 返回生成的校验和字符串
 */
std::string Calculate(std::string filename, FileType type);

} // namespace Checksum

}  // namespace ORB_SLAM3
