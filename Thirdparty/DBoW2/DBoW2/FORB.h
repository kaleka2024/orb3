/**
 * File: FORB.h
 * Date: June 2012
 * Author: Dorian Galvez‑Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 */
// 头文件保护宏，防止头文件被重复包含
#ifndef __D_T_F_ORB__
#define __D_T_F_ORB__

// OpenCV核心模块头文件，cv::Mat定义
#include <opencv2/core/core.hpp>
// STL动态数组容器
#include <vector>
// STL字符串类
#include <string>

// 引入描述子操作基类FClass
#include "FClass.h"

namespace DBoW2 {

/**
 * @brief ORB描述子操作类，继承FClass，作为DBoW2模板参数F
 * @note 保护继承FClass，实现二进制ORB描述子全套操作接口；
 * ORB描述子为1×32 CV_8U，一共256bit二进制描述子
 */
class FORB: protected FClass {
public:

  /// Descriptor type 描述子实际类型，OpenCV Mat，CV_8U单通道
  typedef cv::Mat TDescriptor; // CV_8U
  /// Pointer to a single descriptor 常量描述子指针类型
  typedef const TDescriptor *pDescriptor;
  /// Descriptor length (in bytes) ORB描述子字节长度，标准ORB为32字节
  static const int L;

  /**
   * Calculates the mean value of a set of descriptors
   * @param descriptors 输入一批描述子的指针集合
   * @param mean mean descriptor [out]输出计算得到的均值描述子
   * @brief 计算一组ORB二进制描述子的均值描述子；二进制按bit投票过半置1
   */
  static void meanValue(const std::vector<pDescriptor> &descriptors,
    TDescriptor &mean);

  /**
   * Calculates the distance between two descriptors
   * @param a 第一个ORB描述子
   * @param b 第二个ORB描述子
   * @return distance 返回汉明距离，即不相同bit的数量，取值0~256
   * @brief 计算两个ORB描述子之间汉明距离，使用高效位运算实现
   */
  static int distance(const TDescriptor &a, const TDescriptor &b);

  /**
   * Returns a string version of the descriptor
   * @param a descriptor 输入ORB描述子
   * @return string version 返回序列化字符串，每个字节数字空格分隔
   * @brief 将ORB描述子序列化为字符串，用于磁盘保存
   */
  static std::string toString(const TDescriptor &a);

  /**
   * Returns a descriptor from a string
   * @param a descriptor [out]输出还原后的ORB描述子
   * @param s string version 输入序列化字符串
   * @brief 从字符串反序列化恢复ORB描述子cv::Mat
   */
  static void fromString(TDescriptor &a, const std::string &s);

  /**
   * Returns a mat with the descriptors in float format
   * @param descriptors 输入ORB描述子数组
   * @param mat (out) NxL 32F matrix 输出N×256 CV_32F浮点矩阵，每个bit展开为0.0/1.0
   * @brief 将一批ORB二进制描述子展开为浮点矩阵，每一个bit转为float数值
   */
  static void toMat32F(const std::vector<TDescriptor> &descriptors,
    cv::Mat &mat);

  /**
   * @brief 将一批ORB描述子输出为CV_8U矩阵，每行直接存放原始32字节描述子
   * @param descriptors 输入ORB描述子数组
   * @param mat [out]输出 N × 32 CV_8U矩阵
   */
  static void toMat8U(const std::vector<TDescriptor> &descriptors,
    cv::Mat &mat);

};

} // namespace DBoW2

#endif
