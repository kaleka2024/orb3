/**
 * File: FClass.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: generic FClass to instantiate templated classes
 * License: see the LICENSE.txt file
 *
 */
// 头文件保护宏，避免头文件重复包含
#ifndef __D_T_FCLASS__
#define __D_T_FCLASS__

// OpenCV核心模块头文件，cv::Mat相关
#include <opencv2/core/core.hpp>
// STL动态数组容器
#include <vector>
// STL字符串类
#include <string>

namespace DBoW2 {

/// Generic class to encapsulate functions to manage descriptors.
/**
 * This class must be inherited. Derived classes can be used as the
 * parameter F when creating Templated structures
 * (TemplatedVocabulary, TemplatedDatabase, ...)
 * @brief 描述子操作的通用基类，封装描述子的各类操作接口
 * @note 本类为抽象模板基类，必须被继承实现；派生类作为模板参数F，
 * 用于实例化TemplatedVocabulary、TemplatedDatabase等模板类
 */
class FClass {
  // 内部前置声明描述子类型TDescriptor，实际由派生类给出真实定义
  class TDescriptor;
  // 定义常量描述子指针别名pDescriptor，指向TDescriptor常量对象
  typedef const TDescriptor *pDescriptor;

  /**
   * Calculates the mean value of a set of descriptors
   * @param descriptors 输入一批描述子指针集合
   * @param mean mean descriptor 输出计算得到的均值描述子
   */
  // virtual void meanValue(const std::vector<pDescriptor> &descriptors,
  //   TDescriptor &mean) = 0

  /**
   * Calculates the distance between two descriptors
   * @param a 第一个待比较描述子
   * @param b 第二个待比较描述子
   * @return distance 返回两个描述子之间的距离
   */
  static double distance(const TDescriptor &a, const TDescriptor &b);

  /**
   * Returns a string version of the descriptor
   * @param a descriptor 输入待转换的描述子
   * @return string version 返回描述子序列化后的字符串
   */
  static std::string toString(const TDescriptor &a);

  /**
   * Returns a descriptor from a string
   * @param a descriptor 输出还原得到的描述子
   * @param s string version 输入保存描述子信息的字符串
   */
  static void fromString(TDescriptor &a, const std::string &s);

  /**
   * Returns a mat with the descriptors in float format
   * @param descriptors 输入描述子数组
   * @param mat (out) NxL 32F matrix 输出OpenCV浮点矩阵，N个描述子，每个L维，CV_32F
   */
  static void toMat32F(const std::vector<TDescriptor> &descriptors,
     cv::Mat &mat);
};

} // namespace DBoW2

#endif
