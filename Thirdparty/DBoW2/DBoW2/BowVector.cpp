/**
 * File: BowVector.cpp
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */
// 标准输入输出流
#include <iostream>
// 文件读写流
#include <fstream>
// STL动态数组容器
#include <vector>
// STL算法库
#include <algorithm>
// 数学函数库 fabs sqrt
#include <cmath>

// BowVector词袋向量头文件
#include "BowVector.h"

namespace DBoW2 {

// --------------------------------------------------------------------------
/**
 * @brief BowVector默认构造函数，BowVector继承自std::map<WordId,WordValue>
 */
BowVector::BowVector(void) {
}

// --------------------------------------------------------------------------
/**
 * @brief BowVector析构函数
 */
BowVector::~BowVector(void) {
}

// --------------------------------------------------------------------------
/**
 * @brief 添加单词权重，若单词id已存在则权重累加，不存在则插入新键值对
 * @param id 单词ID WordId
 * @param v 单词权重 WordValue
 */
void BowVector::addWeight(WordId id, WordValue v) {
  // lower_bound: 返回第一个不小于id的迭代器，map有序，快速查找key
  BowVector::iterator vit = this->lower_bound(id);

  // vit不是末尾，并且key_comp比较：id不小于vit->first，代表key已经存在
  if(vit != this->end() && !(this->key_comp()(id, vit->first)))
  {
    // key已存在，权重累加
    vit->second += v;
  }
  else
  {
    // key不存在，在vit位置插入新的(id, v)键值对
    this->insert(vit, BowVector::value_type(id, v));
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 仅当单词id不存在时才插入，已存在直接跳过，不会累加权重
 * @param id 单词ID WordId
 * @param v 单词权重 WordValue
 */
void BowVector::addIfNotExist(WordId id, WordValue v) {
  // lower_bound查找第一个>=id的元素迭代器
  BowVector::iterator vit = this->lower_bound(id);

  // 到达末尾 或者 id小于vit->first，说明id这个key不存在
  if(vit == this->end() || (this->key_comp()(id, vit->first)))
  {
    // 在vit位置插入新单词键值对
    this->insert(vit, BowVector::value_type(id, v));
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 对词袋向量做归一化，支持L1范数 / L2范数
 * @param norm_type 归一化类型 DBoW2::L1 / DBoW2::L2
 */
void BowVector::normalize(LNorm norm_type) {
  double norm = 0.0;
  BowVector::iterator it;

  if(norm_type == DBoW2::L1)
  {
    // L1归一化：计算所有权重绝对值之和
    for(it = begin(); it != end(); ++it)
      norm += fabs(it->second);
  }
  else
  {
    // L2归一化：先计算权重平方累加，再开根号得到L2范数
    for(it = begin(); it != end(); ++it)
      norm += it->second * it->second;

    norm = sqrt(norm);
  }

  // 范数大于0才做归一化，避免除0
  if(norm > 0.0)
  {
    // 每个权重除以范数完成归一化
    for(it = begin(); it != end(); ++it)
      it->second /= norm;
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 重载输出流运算符，打印BowVector内容，格式 <id, weight>, <id, weight>
 * @param out 输出流对象
 * @param v 待打印的词袋向量const引用
 * @return std::ostream& 返回输出流，支持链式输出
 */
std::ostream& operator<< (std::ostream &out, const BowVector &v) {
  BowVector::const_iterator vit;
  std::vector<unsigned int>::const_iterator iit;
  unsigned int i = 0;
  const unsigned int N = v.size();
  for(vit = v.begin(); vit != v.end(); ++vit, ++i)
  {
    out << "<" << vit->first << ", " << vit->second << ">";

    // 不是最后一个元素，输出逗号分隔
    if(i < N-1) out << ", ";
  }
  return out;
}

// --------------------------------------------------------------------------
/**
 * @brief 将稀疏的BowVector保存为完整稠密向量文件，未出现的单词权重填0
 * @param filename 输出文件路径
 * @param W 词典总单词总数，向量总长度为W
 */
void BowVector::saveM(const std::string &filename, size_t W) const {
  // 打开文件，输出模式
  std::fstream f(filename.c_str(), std::ios::out);

  WordId last = 0;
  BowVector::const_iterator bit;
  // 遍历BowVector内部map，只存储出现过的单词
  for(bit = this->begin(); bit != this->end(); ++bit)
  {
    // 当前last到bit->first之间缺失的单词，全部输出0
    for(; last < bit->first; ++last)
    {
      f << "0 ";
    }
    // 输出当前单词实际权重
    f << bit->second << " ";

    // 更新last到下一个待处理单词id
    last = bit->first + 1;
  }
  // 遍历结束，剩余id从last到W‑1全部填充0
  for(; last < (WordId)W; ++last)
    f << "0 ";

  // 关闭文件流
  f.close();
}

// --------------------------------------------------------------------------
} // namespace DBoW2
