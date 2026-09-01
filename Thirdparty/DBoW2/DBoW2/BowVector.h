/**
 * File: BowVector.h
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */
// 头文件保护宏，防止头文件被重复包含
#ifndef __D_T_BOW_VECTOR__
#define __D_T_BOW_VECTOR__

// C++标准输出流
#include <iostream>
// std::map容器，BowVector继承map
#include <map>
// std::vector动态数组容器
#include <vector>

// boost序列化基础头文件，用于对象磁盘序列化保存加载
#include <boost/serialization/serialization.hpp>
// boost对std::map容器的序列化支持
#include <boost/serialization/map.hpp>

namespace DBoW2 {

/// Id of words 词典里单词的ID类型
typedef unsigned int WordId;

/// Value of a word 单词对应的权重数值类型
typedef double WordValue;

/// Id of nodes in the vocabulary treee 词汇树内部节点ID类型
typedef unsigned int NodeId;

/// L‑norms for normalization 归一化使用的L范数枚举
enum LNorm {
  L1,    // L1范数归一化
  L2     // L2范数归一化
};

/// Weighting type 词袋权重计算方式枚举
enum WeightingType {
  TF_IDF,   // 词频‑逆文档频率，DBoW2最常用
  TF,       // 仅使用词频TF
  IDF,      // 仅使用逆文档频率IDF
  BINARY    // 二值权重，单词出现为1，不出现为0
};

/// Scoring type 两幅图像词袋向量之间相似度打分算法枚举
enum ScoringType {
  L1_NORM,          // L1范数得分
  L2_NORM,          // L2范数得分
  CHI_SQUARE,       // 卡方距离
  KL,               // KL散度
  BHATTACHARYYA,    // 巴氏距离
  DOT_PRODUCT,      // 点积相似度
};

/**
 * @brief 词袋向量类，用来表示一张图像，继承std::map<WordId, WordValue>
 * @note map的key：WordId单词编号；value：WordValue单词权重；
 * 采用稀疏存储，只保存图像中出现过的单词，未出现单词不在map内
 */
class BowVector:
         public std::map<WordId, WordValue>
{
    // boost序列化友元，允许序列化库访问类私有成员
    friend class boost::serialization::access;
    /**
     * @brief boost序列化模板函数，序列化基类std::map
     * @tparam Archive 序列化归档类型
     * @param ar 归档对象
     * @param version 版本号
     */
    template<class Archive>
    void serialize(Archive& ar, const int version)
    {
        // 将本对象当做基类std::map进行序列化读写
        ar & boost::serialization::base_object<std::map<WordId, WordValue> >(*this);
    }

public:

    /**
     * Constructor
     * @brief BowVector默认构造函数
     */
    BowVector(void);

    /**
     * Destructor
     * @brief BowVector析构函数
     */
    ~BowVector(void);

    /**
     * Adds a value to a word value existing in the vector, or creates a new
     * word with the given value
     * @param id word id to look for
     * @param v value to create the word with, or to add to existing word
     * @brief 添加单词权重，id已存在则权重累加；不存在则插入新单词
     */
    void addWeight(WordId id, WordValue v);

    /**
     * Adds a word with a value to the vector only if this does not exist yet
     * @param id word id to look for
     * @param v value to give to the word if this does not exist
     * @brief 仅单词id不存在时才插入，已存在直接跳过，不累加权重
     */
    void addIfNotExist(WordId id, WordValue v);

    /**
     * L1‑Normalizes the values in the vector
     * @param norm_type norm used
     * @brief 对词袋向量做L1/L2归一化
     */
    void normalize(LNorm norm_type);

    /**
     * Prints the content of the bow vector
     * @param out stream
     * @param v
     * @brief 输出流重载，打印BowVector内容，友元函数
     */
    friend std::ostream& operator<<(std::ostream &out, const BowVector &v);

    /**
     * Saves the bow vector as a vector in a matlab file
     * @param filename
     * @param W number of words in the vocabulary
     * @brief 将稀疏BowVector输出为稠密向量文本文件，可供matlab读取
     * @param W 词典总单词数量，稠密向量总长度等于W
     */
    void saveM(const std::string &filename, size_t W) const;
};

} // namespace DBoW2

#endif
