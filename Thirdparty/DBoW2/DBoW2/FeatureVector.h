/**
 * File: FeatureVector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */
// 头文件保护宏，防止头文件重复包含
#ifndef __D_T_FEATURE_VECTOR__
#define __D_T_FEATURE_VECTOR__

// 引入BowVector.h，使用NodeId类型定义
#include "BowVector.h"
// std::map容器，FeatureVector继承map
#include <map>
// STL动态数组容器
#include <vector>
// C++标准输出流
#include <iostream>

// boost序列化基础头文件，用于对象序列化保存加载
#include <boost/serialization/serialization.hpp>
// boost对std::map容器的序列化支持
#include <boost/serialization/map.hpp>

namespace DBoW2 {

/// Vector of nodes with indexes of local features
/**
 * @brief 特征向量类，记录图像每个特征归属词汇树哪个节点
 * @note 继承std::map，key为词汇树NodeId，value是该节点下对应的图像特征下标集合；
 * 一张图像的每个特征向下遍历词汇树最终落到某个叶子节点，本类保存节点与原始特征索引映射
 */
class FeatureVector:
   public std::map<NodeId, std::vector<unsigned int> >
{
    // boost序列化友元，允许序列化库访问类私有成员
    friend class boost::serialization::access;
    /**
     * @brief boost序列化模板函数，序列化父类std::map
     * @tparam Archive 序列化归档类型
     * @param ar 归档对象
     * @param version 序列化版本号
     */
    template<class Archive>
    void serialize(Archive& ar, const int version)
    {
        // 将当前对象当做基类std::map进行序列化读写
        ar & boost::serialization::base_object<std::map<NodeId, std::vector<unsigned int> > >(*this);
    }

public:

    /**
     * Constructor
     * @brief FeatureVector默认构造函数
     */
   FeatureVector(void);

    /**
     * Destructor
     * @brief FeatureVector析构函数
     */
   ~FeatureVector(void);

    /**
     * Adds a feature to an existing node, or adds a new node with an initial
     * feature
     * @param id node id to add or to modify
     * @param i_feature index of feature to add to the given node
     * @brief 向节点添加一个图像特征下标；节点存在则追加，不存在则新建节点再存入特征下标
     * @param id 词汇树节点ID
     * @param i_feature 图像中原始特征点的索引编号
     */
   void addFeature(NodeId id, unsigned int i_feature);

    /**
     * Sends a string versions of the feature vector through the stream
     * @param out stream
     * @param v feature vector
     * @brief 输出流重载，打印FeatureVector文本信息，友元函数
     */
   friend std::ostream& operator<<(std::ostream &out, const FeatureVector &v);

};

} // namespace DBoW2

#endif
