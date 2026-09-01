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
 * ORB‑SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */
#include <pangolin/pangolin.h>                     // Pangolin可视化库头文件，此处本文件未直接使用，System模块其他逻辑依赖
#include <algorithm>                                // C++算法标准库
#include <boost/archive/binary_iarchive.hpp>        // Boost序列化：二进制输入归档，读取二进制存档
#include <boost/archive/binary_oarchive.hpp>        // Boost序列化：二进制输出归档，写入二进制存档
#include <boost/archive/text_iarchive.hpp>          // Boost序列化：文本输入归档，读取文本格式存档
#include <boost/archive/text_oarchive.hpp>          // Boost序列化：文本输出归档，写入文本格式存档
#include <boost/archive/xml_iarchive.hpp>           // Boost序列化：XML输入归档（本代码未使用）
#include <boost/archive/xml_oarchive.hpp>           // Boost序列化：XML输出归档（本代码未使用）
#include <boost/serialization/base_object.hpp>      // Boost序列化：支持基类对象序列化
#include <boost/serialization/shared_ptr.hpp>       // Boost序列化：std::shared_ptr智能指针序列化支持
#include <boost/serialization/string.hpp>           // Boost序列化：std::string字符串序列化支持
#include <cstdio>                                   // C标准库，std::remove删除文件函数依赖
#include <exception>                                // C++标准异常库，runtime_error异常依赖
#include <iomanip>                                  // C++输入输出格式控制库（本代码未直接使用）
#include <iostream>                                 // C++标准输入输出流
#include <list>                                     // C++list容器头文件（本代码未直接使用）
#include <memory>                                   // C++智能指针std::shared_ptr等定义
#include <string>                                   // C++std::string字符串
#include <thread>                                   // C++线程库（本代码未直接使用）
#include <vector>                                   // C++vector动态数组容器（本代码未直接使用）

#include "Converter.h"                              // ORB‑SLAM3格式转换工具，Eigen、cv::Mat数据互转（本函数未直接调用）
#include "System.h"                                 // ORB‑SLAM3系统主类头文件，本函数属于System类成员
#include "Utils/Checksum.h"                         // ORB‑SLAM3校验和工具，计算文件MD5哈希，用于词袋文件一致性校验

namespace ORB_SLAM3 {

/**
 * @brief 将Atlas地图集保存到磁盘文件，支持文本/二进制两种Boost序列化存档格式，后缀为.osa
 * @param type 文件类型枚举：FileType::TEXT_FILE文本格式 / FileType::BINARY_FILE二进制格式
 * @note 保存内容：词袋文件名、词袋MD5校验值、完整Atlas地图集对象；需要配置文件指定atlasSaveFile保存文件名
 */
void System::SaveAtlas(FileType type) {
  // 从配置参数读取Atlas保存的文件名，不含后缀
  const string mStrSaveAtlasToFile = settings_->atlasSaveFile();

  // 判断配置中是否设置了保存文件名，非空才执行保存逻辑
  if (!mStrSaveAtlasToFile.empty()) {
    // clock_t start = clock();                   // 注释掉的计时代码，可用于统计保存耗时

    // 调用Atlas预保存回调函数，保存前做预处理：清理临时状态、准备序列化数据
    mpAtlas->PreSave();

    string pathSaveFileName = "./";               // 设置保存路径前缀为当前工作目录
    pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile); // 拼接用户配置的保存文件名
    pathSaveFileName = pathSaveFileName.append(".osa");              // 追加ORB‑SLAM3地图集后缀.osa

    // 获取词袋vocabulary文件完整路径
    const string vocabularyFilePath = settings_->strVocFile_;
    // 计算词袋文件的MD5校验和，文本文件模式读取
    string strVocabularyChecksum =
        Checksum::Calculate(vocabularyFilePath, FileType::TEXT_FILE);
    // 查找路径中最后一个斜杠/反斜杠位置，用于分离目录与文件名，兼容linux/windows路径分隔符
    std::size_t found = vocabularyFilePath.find_last_of("/\\");
    // 截取得到词袋文件名（剥离前面目录路径，只保留文件名）
    string strVocabularyName = vocabularyFilePath.substr(found + 1);

    // 分支：文本格式存档保存
    if (type == FileType::TEXT_FILE) {
      // File text
      oslog::debug("Starting to write the save text file ");  // 打印调试日志，开始写文本格式地图文件
      std::remove(pathSaveFileName.c_str());                   // 删除旧的.osa文件，避免旧文件残留干扰
      // 创建输出文件流，使用binary模式防止系统对文本做换行转换破坏boost归档格式
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      // 构造boost文本输出归档对象，绑定文件输出流ofs
      boost::archive::text_oarchive oa(ofs);

      oa << strVocabularyName;       // 序列化写入：词袋文件名
      oa << strVocabularyChecksum;   // 序列化写入：词袋MD5校验字符串
      oa << mpAtlas;                 // 序列化写入：完整Atlas地图集智能指针对象
      oslog::debug("End to write the save text file");        // 打印调试日志，文本格式地图文件写入完成
    } else if (type == FileType::BINARY_FILE) {
      // File binary
      oslog::debug("Starting to write the save binary file"); // 打印调试日志，开始写二进制格式地图文件
      std::remove(pathSaveFileName.c_str());                   // 删除旧的.osa文件
      // 创建输出文件流，binary二进制模式打开
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      // 构造boost二进制输出归档对象，绑定文件输出流ofs
      boost::archive::binary_oarchive oa(ofs);
      oa << strVocabularyName;       // 序列化写入：词袋文件名
      oa << strVocabularyChecksum;   // 序列化写入：词袋MD5校验字符串
      oa << mpAtlas;                 // 序列化写入：完整Atlas地图集智能指针对象
      oslog::debug("End to write save binary file");          // 打印调试日志，二进制格式地图文件写入完成
    }
  }
}

/**
 * @brief 从磁盘的.osa存档文件加载Atlas地图集，支持文本、二进制两种序列化格式；加载时校验词袋文件MD5一致性
 * @param type 文件类型枚举：FileType::TEXT_FILE / FileType::BINARY_FILE
 * @return bool true加载成功；false加载失败（文件不存在、词袋MD5不匹配等）
 * @exception std::runtime_error 反序列化后mpAtlas为空抛出运行时异常
 */
bool System::LoadAtlas(FileType type) {
  string strFileVoc, strVocChecksum;    // strFileVoc存档内保存的词袋文件名；strVocChecksum存档内保存的词袋MD5校验和

  // 从配置读取Atlas待加载的文件名（不含后缀）
  const string mStrLoadAtlasFromFile = settings_->atlasLoadFile();
  // 获取当前程序正在使用的词袋文件完整路径
  const string vocabularyFilePath = settings_->strVocFile_;
  bool isRead = false;                   // 标记文件是否成功读取解析完成，初始false

  string pathLoadFileName = "./";        // 加载路径前缀：当前工作目录
  pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile); // 拼接配置的加载文件名
  pathLoadFileName = pathLoadFileName.append(".osa");                 // 拼接地图集后缀.osa

  // 分支：加载文本格式存档
  if (type == FileType::TEXT_FILE) {
    // File text
    oslog::debug("Starting to read the save text file ");    // 调试日志：开始读取文本地图存档
    // 二进制模式打开文本存档，避免换行符转换破坏boost归档
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    // 判断文件流状态，文件打不开、损坏直接返回false
    if (!ifs.good()) {
      oslog::error("Cannot find Atlas file {}", pathLoadFileName);
      return false;
    }
    // 构造boost文本输入归档对象，绑定输入文件流ifs
    boost::archive::text_iarchive ia(ifs);
    ia >> strFileVoc;               // 反序列化读出：存档中保存的词袋文件名
    ia >> strVocChecksum;           // 反序列化读出：存档中保存的词袋MD5校验串
    ia >> mpAtlas;                  // 反序列化读出：完整Atlas地图集对象，赋值给系统mpAtlas成员

    oslog::debug("Finished loading the saved text file ");    // 调试日志：文本存档读取解析完毕
    isRead = true;                 // 标记读取成功
  } else if (type == FileType::BINARY_FILE) {
    // File binary
    oslog::debug("Starting to read the save binary file");    // 调试日志：开始读取二进制地图存档
    // 二进制模式打开二进制存档文件
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    // 判断文件流状态，文件打不开直接返回false
    if (!ifs.good()) {
      oslog::error("Cannot find Atlas file {}", pathLoadFileName);
      return false;
    }
    // 构造boost二进制输入归档对象，绑定输入文件流ifs
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;               // 反序列化读出：存档中保存的词袋文件名
    ia >> strVocChecksum;           // 反序列化读出：存档中保存的词袋MD5校验串
    ia >> mpAtlas;                  // 反序列化读出：完整Atlas地图集对象，赋值系统mpAtlas成员

    oslog::debug("Finished loading the saved binary file");   // 调试日志：二进制存档读取解析完毕
    isRead = true;                 // 标记读取成功
  }

  // 反序列化完成后校验mpAtlas指针有效性，如果为空抛出运行时异常
  if (!mpAtlas) {
    throw std::runtime_error("mpAtlas not initialized when it should be");
  }

  // 文件读取解析成功，执行后续校验与Atlas后处理
  if (isRead) {
    // 计算当前程序实际加载的词袋文件MD5校验和
    string strInputVocabularyChecksum =
        Checksum::Calculate(vocabularyFilePath, FileType::TEXT_FILE);

    // 比较：当前词袋MD5 和存档内部保存的MD5是否一致；compare返回非0代表两者不相等
    if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
      oslog::warn(
          "The vocabulary load isn't the same which the load session was "
          "created.  Loading vocab file {}",
          strFileVoc);
      return false;  // Both are differents  // 词袋文件不一致，加载失败返回false
    }

    // 将系统的关键帧数据库指针设置给加载出来的Atlas
    mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    // 将系统当前ORB词袋模型指针设置给加载出来的Atlas
    mpAtlas->SetORBVocabulary(mpVocabulary);
    // Atlas加载完成后回调后处理函数：重建内部关联、恢复各类指针、初始化加载后状态
    mpAtlas->PostLoad();

    return true;  // Atlas全部加载校验完成，返回true成功
  }
  return false;    // isRead为false，加载失败返回false
}

}  // namespace ORB_SLAM3
