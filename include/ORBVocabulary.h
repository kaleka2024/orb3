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
#pragma once                          // 头文件保护，避免头文件被重复包含引发编译错误
#include "Thirdparty/DBoW2/DBoW2/FORB.h"        // DBoW2库中ORB描述子专用模板，定义FORB的TDescriptor数据类型、汉明距离计算接口
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h" // DBoW2通用词袋模板类，可针对不同描述子实例化生成视觉词典

namespace ORB_SLAM3 {

/**
 * @brief ORB视觉词典类型别名，基于DBoW2模板实例化
 * @details TemplatedVocabulary<描述子类型, 描述子处理类>
 * 第一个模板参数：DBoW2::FORB::TDescriptor，ORB描述子底层数据类型，对应cv::Mat的32字节二进制描述子
 * 第二个模板参数：DBoW2::FORB，提供ORB描述子距离计算、转换、序列化等底层操作
 * ORBVocabulary用于加载orb词典文件，计算图像BoW词袋向量、特征节点索引，服务重定位与回环检测
 */
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;

}  // namespace ORB_SLAM3
