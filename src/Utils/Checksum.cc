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
#include "Utils/Checksum.h"
#include <openssl/md5.h>   // OpenSSL MD5哈希算法头文件，用于计算文件MD5校验和
#include <cstdio>          // C标准输入输出库，snprintf函数依赖
#include <fstream>         // C++文件流库，std::ifstream文件读取
#include <ios>             // C++IO流状态与打开模式定义
#include <string>          // C++字符串std::string

#include "Logging.h"       // ORB‑SLAM3内部日志模块，oslog日志输出
#include "Types.h"         // ORB‑SLAM3自定义类型，包含FileType枚举定义

namespace ORB_SLAM3 {

namespace Checksum {     // 校验和工具命名空间，封装MD5文件校验相关功能

using std::string;       // 导入std::string，后续代码直接使用string

/**
 * @brief 计算指定文件的MD5校验和字符串
 * @param filename 待计算哈希的文件路径
 * @param type 文件类型：文本文件 / BINARY_FILE二进制文件，控制文件打开模式
 * @return std::string 返回32位小写十六进制MD5字符串；文件打开失败返回空字符串
 */
string Calculate(string filename, FileType type) {
  string checksum = "";                          // 存储最终MD5结果字符串，初始为空

  unsigned char c[MD5_DIGEST_LENGTH];           // MD5_DIGEST_LENGTH=16，MD5原始二进制输出缓冲区，存放16字节原始哈希结果

  std::ios_base::openmode flags = std::ios::in;  // 文件打开模式，默认只读文本模式
  if (type == FileType::BINARY_FILE)  // Binary file
    flags = std::ios::in | std::ios::binary;     // 如果是二进制文件，追加binary标记，关闭系统换行符自动转换，避免二进制数据损坏

  std::ifstream f(filename.c_str(), flags);      // 构造文件输入流，传入c风格文件路径与打开模式
  if (!f.is_open()) {                            // 判断文件是否成功打开
    oslog::error("[E] Unable to open the in file {} for Md5 hash.", filename); // 日志输出错误信息，打印无法打开的文件名
    return checksum;                             // 打开失败直接返回空字符串
  }

  MD5_CTX md5Context;                            // OpenSSL MD5上下文结构体，保存MD5计算过程中间状态
  char buffer[1024];                             // 文件读取缓冲区，每次读取1024字节块做流式MD5更新

  MD5_Init(&md5Context);                         // 初始化MD5上下文，重置内部状态，准备开始哈希计算
  // f.readsome：尽可能读取buffer大小字节，返回本次实际读到的字节数；循环分块读取整个文件
  while (int count = f.readsome(buffer, sizeof(buffer))) {
    MD5_Update(&md5Context, buffer, count);      // 将本次读到的文件块送入MD5上下文，迭代更新哈希状态
  }

  f.close();                                     // 文件读取完毕，关闭文件流

  MD5_Final(c, &md5Context);                     // MD5计算收尾，输出16字节原始MD5结果存入数组c

  // 遍历16字节MD5原始二进制，逐个字节转为2位十六进制字符拼接成32位MD5字符串
  for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    char aux[10];                                // 临时字符缓冲区，存放单字节转出来的2位十六进制字符串
    snprintf(aux, sizeof(aux), "%02x", c[i]);    // %02x：将unsigned char字节格式化为2位小写十六进制，不足两位前面补0
    checksum = checksum + aux;                   // 把单字节十六进制字符串追加到最终结果
  }

  return checksum;                               // 返回组装完成的32字符MD5校验和字符串
}

}  // namespace Checksum

}  // namespace ORB_SLAM3
