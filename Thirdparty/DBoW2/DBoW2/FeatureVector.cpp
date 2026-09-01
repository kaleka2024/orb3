/**
 * File: FeatureVector.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */
// FeatureVector类头文件
#include "FeatureVector.h"
// std::map容器头文件，FeatureVector继承map
#include <map>
// STL动态数组vector
#include <vector>
// 标准输入输出流
#include <iostream>

namespace DBoW2 {

// ---------------------------------------------------------------------------
/**
 * @brief FeatureVector默认构造函数
 * @note FeatureVector继承 std::map<NodeId, std::vector<unsigned int>>
 * key为词汇树节点ID，value为属于该节点的图像特征下标集合
 */
FeatureVector::FeatureVector(void) {
}

// ---------------------------------------------------------------------------
/**
 * @brief FeatureVector析构函数
 */
FeatureVector::~FeatureVector(void) {
}

// ---------------------------------------------------------------------------
/**
 * @brief 添加一个特征，记录该特征归属于词汇树哪个节点
 * @param id 词汇树节点NodeId
 * @param i_feature 图像里特征点的索引下标
 */
void FeatureVector::addFeature(NodeId id, unsigned int i_feature) {
  // lower_bound查找map中第一个key >= id的迭代器
  FeatureVector::iterator vit = this->lower_bound(id);

  // 迭代器未到末尾，并且key等于目标id，该节点已经存在
  if(vit != this->end() && vit->first == id)
  {
    // 直接把特征下标追加到该节点对应的vector中
    vit->second.push_back(i_feature);
  }
  else
  {
    // 该节点不存在，插入新的键值对，value是空vector
    vit = this->insert(vit, FeatureVector::value_type(id,
       std::vector<unsigned int>() ));
    // 将特征下标加入新创建的vector
    vit->second.push_back(i_feature);
  }
}

// ---------------------------------------------------------------------------
/**
 * @brief 重载输出流运算符，打印FeatureVector内容
 * @param out 输出流对象
 * @param v 待打印的FeatureVector常量引用
 * @return std::ostream& 返回输出流，支持链式输出
 * 输出格式示例：<nodeId: [featIdx0, featIdx1]>, <nodeId: [featIdx2]>
 */
std::ostream& operator<<(std::ostream &out,
   const FeatureVector &v) {
  // 判断map不为空才执行打印
  if(!v.empty())
  {
    // 获取map首元素迭代器
    FeatureVector::const_iterator vit = v.begin();

    // 指向当前节点对应的特征下标vector
    const std::vector<unsigned int>* f = &vit->second;

    out << "<" << vit->first << ": [";
    // vector非空，打印第一个特征下标
    if(!f->empty()) out << (*f)[0];
    // 循环打印剩余特征下标，逗号分隔
    for(unsigned int i = 1; i < f->size(); ++i)
    {
      out << ", " << (*f)[i];
    }
    out << "]>";

    // 处理map剩下的所有节点
    for(++vit; vit != v.end(); ++vit)
    {
      f = &vit->second;

      out << ", <" << vit->first << ": [";
      if(!f->empty()) out << (*f)[0];
      for(unsigned int i = 1; i < f->size(); ++i)
      {
        out << ", " << (*f)[i];
      }
      out << "]>";
    }
  }

  return out;
  }

// ---------------------------------------------------------------------------
} // namespace DBoW2
