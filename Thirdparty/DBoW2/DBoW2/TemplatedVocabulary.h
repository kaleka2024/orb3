/**
 * This is a modified version of TemplatedVocabulary.h from DBoW2 (see below).
 * Added functions: Save and Load from text files without using cv::FileStorage.
 * Date: August 2015
 * Raúl Mur‑Artal
 */
/**
 * File: TemplatedVocabulary.h
 * Date: February 2011
 * Author: Dorian Galvez‑Lopez
 * Description: templated vocabulary
 * License: see the LICENSE.txt file
 *
 */
// 头文件保护宏，防止头文件重复包含
#ifndef __D_T_TEMPLATED_VOCABULARY__
#define __D_T_TEMPLATED_VOCABULARY__

// assert断言，调试期参数合法性校验
#include <cassert>

// STL容器
#include <vector>
// accumulate累加算法
#include <numeric>
// 文件流读写
#include <fstream>
// std::string字符串
#include <string>
// std::algorithm算法库
#include <algorithm>
// OpenCV核心模块，Mat等
#include <opencv2/core/core.hpp>
// 数值极限，获取浮点数最大最小值
#include <limits>

// 特征向量FeatureVector头文件，保存特征与树节点映射
#include "FeatureVector.h"
// BowVector词袋向量头文件
#include "BowVector.h"
// 打分策略对象头文件
#include "ScoringObject.h"

// DUtils随机数工具，kmeans++初始化需要
#include "../DUtils/Random.h"

using namespace std;

namespace DBoW2 {

/// @param TDescriptor class of descriptor 描述子类型，如cv::Mat(ORB)
/// @param F class of descriptor functions 描述子操作函数类，如FORB
template<class TDescriptor, class F>
/// Generic Vocabulary class 通用词袋词典模板类，构建k叉L层树结构词典
class TemplatedVocabulary {
              public:
    /**
    * Initiates an empty vocabulary
    * @param k branching factor 树的分支因子k，每个节点最多k个子节点
    * @param L depth levels 树深度L，叶子层为单词
    * @param weighting weighting type 权重计算方式 TF/IDF/TF_IDF/BINARY
    * @param scoring scoring type 相似度打分策略 L1_NORM/L2_NORM等
    */
  TemplatedVocabulary(int k = 10, int L = 5,
     WeightingType weighting = TF_IDF, ScoringType scoring = L1_NORM);

  /**
    * Creates the vocabulary by loading a file
    * @param filename 词典yaml文件路径，string版本
    */
  TemplatedVocabulary(const std::string &filename);

  /**
    * Creates the vocabulary by loading a file
    * @param filename 词典yaml文件路径，char*版本
    */
  TemplatedVocabulary(const char *filename);

   /**
     * Copy constructor 拷贝构造函数
    * @param voc 待拷贝的词典对象
    */
  TemplatedVocabulary(const TemplatedVocabulary<TDescriptor, F> &voc);

  /**
    * Destructor 虚析构函数
    */
  virtual ~TemplatedVocabulary();

   /**
     * Assigns the given vocabulary to this by copying its data and removing
    * all the data contained by this vocabulary before
    * @param voc 源词典
    * @return reference to this vocabulary 返回本对象引用，支持链式赋值
    */
  TemplatedVocabulary<TDescriptor, F>& operator=(
    const TemplatedVocabulary<TDescriptor, F> &voc);

   /**
     * Creates a vocabulary from the training features with the already
    * defined parameters 使用已设置好的k、L、weighting、scoring训练词典
    * @param training_features 训练特征集合，外层vector是图像，内层是单张图像所有描述子
    */
  virtual void create
    (const std::vector<std::vector<TDescriptor> > &training_features);

  /**
    * Creates a vocabulary from the training features, setting the branching
    * factor and the depth levels of the tree
    * @param training_features 训练特征集合
    * @param k branching factor 分支因子
    * @param L depth levels 树深度
    */
  virtual void create
    (const std::vector<std::vector<TDescriptor> > &training_features,
     int k, int L);

  /**
    * Creates a vocabulary from the training features, setting the branching
    * factor nad the depth levels of the tree, and the weighting and scoring
    * schemes
    * @brief 全部参数传入，训练生成词典
    */
  virtual void create
    (const std::vector<std::vector<TDescriptor> > &training_features,
    int k, int L, WeightingType weighting, ScoringType scoring);

  /**
    * Returns the number of words in the vocabulary
    * @return number of words 返回词典单词数量，即叶子节点总数
    */
  virtual inline unsigned int size() const;

  /**
    * Returns whether the vocabulary is empty (i.e. it has not been trained)
    * @return true iff the vocabulary is empty 返回true代表词典为空，未训练/未加载
    */
  virtual inline bool empty() const;

  /**
    * Transforms a set of descriptores into a bow vector
    * @param features 单张图像全部描述子
    * @param v (out) bow vector of weighted words 输出加权词袋向量BowVector
    */
  virtual void transform(const std::vector<TDescriptor>& features, BowVector &v)
     const;

  /**
    * Transform a set of descriptors into a bow vector and a feature vector
    * @param features 图像描述子集合
    * @param v (out) bow vector 输出词袋向量
    * @param fv (out) feature vector of nodes and feature indexes 输出FeatureVector，节点与原始特征下标映射
    * @param levelsup levels to go up the vocabulary tree to get the node index 向上回溯几层取节点id
    */
  virtual void transform(const std::vector<TDescriptor>& features,
    BowVector &v, FeatureVector &fv, int levelsup) const;

  /**
    * Transforms a single feature into a word (without weight)
    * @param feature 单个描述子
    * @return word id 返回该描述子落到叶子单词的word id
    */
  virtual WordId transform(const TDescriptor& feature) const;

  /**
    * Returns the score of two vectors
    * @param a vector 词袋向量a
    * @param b vector 词袋向量b
    * @return score between vectors 返回相似度分数
    * @note the vectors must be already sorted and normalized if necessary 向量必须有序，部分打分策略需要预先归一化
    */
  inline double score(const BowVector &a, const BowVector &b) const;

  /**
    * Returns the id of the node that is "levelsup" levels from the word given
    * @param wid word id 单词id（叶子节点）
    * @param levelsup 0..L 向上回溯层数
    * @return node id. if levelsup is 0, returns the node id associated to the
    *   word id 返回向上回溯levelsup层后的节点id；0返回叶子节点本身，到根节点返回0
    */
  virtual NodeId getParentNode(WordId wid, int levelsup) const;

  /**
    * Returns the ids of all the words that are under the given node id,
    * by traversing any of the branches that goes down from the node
    * @param nid starting node id 起始节点id
    * @param words ids of words [out]输出该节点下所有叶子单词id集合
    */
  void getWordsFromNode(NodeId nid, std::vector<WordId> &words) const;

  /**
    * Returns the branching factor of the tree (k)
    * @return k 获取分支因子k
    */
  inline int getBranchingFactor() const { return m_k; }

   /**
     * Returns the depth levels of the tree (L)
    * @return L 获取设置的树深度L
    */
  inline int getDepthLevels() const { return m_L; }

  /**
    * Returns the real depth levels of the tree on average
    * @return average of depth levels of leaves 获取叶子节点实际平均深度
    */
  float getEffectiveLevels() const;

  /**
    * Returns the descriptor of a word
    * @param wid word id 单词id
    * @return descriptor 返回单词对应的描述子
    */
  virtual inline TDescriptor getWord(WordId wid) const;

  /**
    * Returns the weight of a word
    * @param wid word id 单词id
    * @return weight 返回单词权重idf值
    */
  virtual inline WordValue getWordWeight(WordId wid) const;

   /**
     * Returns the weighting method
    * @return weighting method 获取权重计算类型
    */
  inline WeightingType getWeightingType() const { return m_weighting; }

   /**
     * Returns the scoring method
    * @return scoring method 获取打分策略类型
    */
  inline ScoringType getScoringType() const { return m_scoring; }

  /**
    * Changes the weighting method
    * @param type new weighting type 修改权重计算方式
    */
  inline void setWeightingType(WeightingType type);

  /**
    * Changes the scoring method
    * @param type new scoring type 修改打分策略，内部会重建m_scoring_object对象
    */
  void setScoringType(ScoringType type);

  /**
    * Loads the vocabulary from a text file
    * @param filename 加载文本格式词典，ORB‑SLAM2新增接口，不依赖cv::FileStorage
    */
  bool loadFromTextFile(const std::string &filename);

  /**
    * Saves the vocabulary into a text file
    * @param filename 保存词典到文本文件，ORB‑SLAM2新增接口
    */
  void saveToTextFile(const std::string &filename) const;

  /**
    * Saves the vocabulary into a file
    * @param filename 保存词典为OpenCV YAML格式
    */
  void save(const std::string &filename) const;

  /**
    * Loads the vocabulary from a file
    * @param filename 从YAML文件加载词典
    */
  void load(const std::string &filename);

   /**
     * Saves the vocabulary to a file storage structure
    * @param fn node in file storage cv::FileStorage引用，写入节点
    */
  virtual void save(cv::FileStorage &fs,
     const std::string &name = "vocabulary") const;

  /**
    * Loads the vocabulary from a file storage node
    * @param fn first node FileStorage根节点
    * @param subname name of the child node of fn where the tree is stored.
    *   If not given, the fn node is used instead 词典子节点名，默认vocabulary
    */
    virtual void load(const cv::FileStorage &fs,
     const std::string &name = "vocabulary");

   /**
     * Stops those words whose weight is below minWeight.
    * Words are stopped by setting their weight to 0. There are not returned
    * later when transforming image features into vectors.
    * Note that when using IDF or TF_IDF, the weight is the idf part, which
    * is equivalent to -log(f), where f is the frequency of the word
    * (f = Ni/N, Ni: number of training images where the word is present,
    * N: number of training images).
    * Note that the old weight is forgotten, and subsequent calls to this
    * function with a lower minWeight have no effect.
    * @return number of words stopped now 停用权重低于minWeight的单词，权重置0，transform时忽略；返回停用单词数量
    */
  virtual int stopWords(double minWeight);

protected:
   /// Pointer to descriptor 描述子常量指针别名
  typedef const TDescriptor *pDescriptor;

  /// Tree node 词典树节点结构体
  struct Node
  {
    /// Node id 节点全局id
    NodeId id;
    /// Weight if the node is a word 仅叶子节点有效，单词idf权重
    WordValue weight;
    /// Children 子节点id列表
    vector<NodeId> children;
    /// Parent node (undefined in case of root) 父节点id；根节点parent无意义
    NodeId parent;
    /// Node descriptor 该节点聚类中心描述子
    TDescriptor descriptor;

    /// Word id if the node is a word 叶子节点对应的word id；非叶子无效
    WordId word_id;

    /**
      * Empty constructor 默认构造
      */
    Node(): id(0), weight(0), parent(0), word_id(0){}

    /**
      * Constructor
      * @param _id node id 指定节点id构造
      */
    Node(NodeId _id): id(_id), weight(0), parent(0), word_id(0){}

    /**
      * Returns whether the node is a leaf node
      * @return true iff the node is a leaf 判断是否叶子节点：children为空即为叶子
      */
    inline bool isLeaf() const { return children.empty(); }
  };

protected:

  /**
    * Creates an instance of the scoring object accoring to m_scoring
    * @brief 根据m_scoring类型，new对应的打分对象，赋值给m_scoring_object
    */
  void createScoringObject();

   /**
     * Returns a set of pointers to descriptores
    * @param training_features all the features 输入训练特征集合
    * @param features (out) pointers to the training features [out]输出全部描述子指针集合，不拷贝数据
    */
  void getFeatures(
    const vector<vector<TDescriptor> > &training_features,
    vector<pDescriptor> &features) const;

  /**
    * Returns the word id associated to a feature
    * @param feature 输入单个描述子
    * @param id (out) word id [out]输出匹配到的单词id
    * @param weight (out) word weight [out]输出单词权重
    * @param nid (out) if given, id of the node "levelsup" levels up 可选输出向上回溯levelsup层的节点id
    * @param levelsup 向上回溯层数
    */
  virtual void transform(const TDescriptor &feature,
    WordId &id, WordValue &weight, NodeId* nid = NULL, int levelsup = 0) const;

  /**
    * Returns the word id associated to a feature
    * @param feature 输入描述子
    * @param id (out) word id [out]输出单词id
    */
  virtual void transform(const TDescriptor &feature, WordId &id) const;

  /**
    * Creates a level in the tree, under the parent, by running kmeans with
    * a descriptor set, and recursively creates the subsequent levels too
    * @param parent_id id of parent node 父节点id
    * @param descriptors descriptors to run the kmeans on 当前层待聚类的描述子指针集合
    * @param current_level current level in the tree 当前处理树层级，根是0
    * @brief Hierarchical Kmeans，HKmeans，层次kmeans递归构建词典树
    */
  void HKmeansStep(NodeId parent_id, const vector<pDescriptor> &descriptors,
     int current_level);

  /**
    * Creates k clusters from the given descriptors with some seeding algorithm.
    * @note In this class, kmeans++ is used, but this function should be
    *   overriden by inherited classes.
    * @brief 初始化k个聚类中心，内部调用kmeans++实现，可被子类重写替换初始化策略
    */
  virtual void initiateClusters(const vector<pDescriptor> &descriptors,
    vector<TDescriptor> &clusters) const;

  /**
    * Creates k clusters from the given descriptor sets by running the
    * initial step of kmeans++
    * @param descriptors 待聚类描述子指针集合
    * @param clusters resulting clusters [out]输出k个聚类中心描述子
    * @brief kmeans++种子初始化算法，选出k个初始聚类中心
    */
  void initiateClustersKMpp(const vector<pDescriptor> &descriptors,
     vector<TDescriptor> &clusters) const;

  /**
    * Create the words of the vocabulary once the tree has been built
    * @brief 树节点全部构建完成后，遍历所有叶子节点，生成m_words数组，建立word_id与Node映射
    */
  void createWords();

  /**
    * Sets the weights of the nodes of tree according to the given features.
    * Before calling this function, the nodes and the words must be already
    * created (by calling HKmeansStep and createWords)
    * @param features 全部训练图像特征集合，用来统计idf权重
    * @brief 计算每个叶子单词的权重（IDF），根据训练图像统计单词出现的图像数
    */
  void setNodeWeights(const vector<vector<TDescriptor> > &features);

protected:
   /// Branching factor 分支因子k
  int m_k;

   /// Depth levels 树深度L
  int m_L;

   /// Weighting method 权重计算策略 TF / IDF / TF_IDF / BINARY
  WeightingType m_weighting;

   /// Scoring method 打分策略枚举
  ScoringType m_scoring;

   /// Object for computing scores 打分策略对象指针，多态调用score接口
  GeneralScoring* m_scoring_object;

   /// Tree nodes 词典树全部节点数组，下标等于NodeId，m_nodes[0]为根节点
  std::vector<Node> m_nodes;

   /// Words of the vocabulary (tree leaves)
   /// this condition holds: m_words[wid]->word_id == wid
   /// 单词数组，每个元素指向叶子Node；数组下标就是WordId
  std::vector<Node*> m_words;

};

// --------------------------------------------------------------------------
/**
 * @brief 构造函数，初始化空词典，设置k L weighting scoring，创建打分对象
 */
template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary
  (int k, int L, WeightingType weighting, ScoringType scoring)
  : m_k(k), m_L(L), m_weighting(weighting), m_scoring(scoring),
  m_scoring_object(NULL)
{
  createScoringObject();
}

// --------------------------------------------------------------------------
/**
 * @brief 从yaml文件路径构造词典，string版本
 */
template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary
  (const std::string &filename): m_scoring_object(NULL)
{
  load(filename);
}

// --------------------------------------------------------------------------
/**
 * @brief 从yaml文件路径构造词典，char*版本
 */
template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary
  (const char *filename): m_scoring_object(NULL)
{
  load(filename);
}

// --------------------------------------------------------------------------
/**
 * @brief 根据m_scoring枚举值new对应打分对象，释放旧对象
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::createScoringObject()
{
  delete m_scoring_object;
  m_scoring_object = NULL;

  switch(m_scoring)
  {
    case L1_NORM:
       m_scoring_object = new L1Scoring;
      break;

    case L2_NORM:
      m_scoring_object = new L2Scoring;
      break;

    case CHI_SQUARE:
      m_scoring_object = new ChiSquareScoring;
      break;

    case KL:
      m_scoring_object = new KLScoring;
      break;

    case BHATTACHARYYA:
      m_scoring_object = new BhattacharyyaScoring;
      break;

    case DOT_PRODUCT:
      m_scoring_object = new DotProductScoring;
      break;
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 修改打分策略，更新m_scoring，重建打分对象
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::setScoringType(ScoringType type)
{
  m_scoring = type;
  createScoringObject();
}

// --------------------------------------------------------------------------
/**
 * @brief 修改权重计算策略，仅赋值，不重建对象
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::setWeightingType(WeightingType type)
{
  this->m_weighting = type;
}

// --------------------------------------------------------------------------
/**
 * @brief 拷贝构造，调用赋值运算符重载
 */
template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary(
  const TemplatedVocabulary<TDescriptor, F> &voc)
  : m_scoring_object(NULL)
{
  *this = voc;
}

// --------------------------------------------------------------------------
/**
 * @brief 析构，释放打分对象堆内存
 */
template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::~TemplatedVocabulary()
{
  delete m_scoring_object;
}

// --------------------------------------------------------------------------
/**
 * @brief 赋值运算符重载，深拷贝词典树、参数，重建打分对象
 */
template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor, F>&
TemplatedVocabulary<TDescriptor,F>::operator=
  (const TemplatedVocabulary<TDescriptor, F> &voc)
{
    this->m_k = voc.m_k;
  this->m_L = voc.m_L;
  this->m_scoring = voc.m_scoring;
  this->m_weighting = voc.m_weighting;

   this->createScoringObject();

   this->m_nodes.clear();
  this->m_words.clear();

   this->m_nodes = voc.m_nodes;
  this->createWords();

   return *this;
}

// --------------------------------------------------------------------------
/**
 * @brief 训练词典主入口：清空旧数据，收集全部训练特征，创建根节点，递归HKmeansStep构建树，生成words，计算单词IDF权重
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::create(
  const std::vector<std::vector<TDescriptor> > &training_features)
{
  m_nodes.clear();
  m_words.clear();

  // expected_nodes = Sum_{i=0..L} ( k^i )
        int expected_nodes =
                (int)((pow((double)m_k, (double)m_L + 1) - 1)/(m_k - 1));

   m_nodes.reserve(expected_nodes); // avoid allocations when creating the tree

        vector<pDescriptor> features;
  getFeatures(training_features, features);

    // create root
    m_nodes.push_back(Node(0)); // root

    // create the tree
  HKmeansStep(0, features, 1);

   // create the words
  createWords();

   // and set the weight of each node of the tree
  setNodeWeights(training_features);

}

// --------------------------------------------------------------------------
/**
 * @brief 指定k、L参数，调用无参create开始训练
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::create(
  const std::vector<std::vector<TDescriptor> > &training_features,
  int k, int L)
{
  m_k = k;
  m_L = L;

  create(training_features);
}

// --------------------------------------------------------------------------
/**
 * @brief 指定全部参数k L weighting scoring，先更新成员，创建打分对象，再调用create训练词典
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::create(
  const std::vector<std::vector<TDescriptor> > &training_features,
  int k, int L, WeightingType weighting, ScoringType scoring)
{
  m_k = k;
  m_L = L;
  m_weighting = weighting;
  m_scoring = scoring;
  createScoringObject();

  create(training_features);
}

// --------------------------------------------------------------------------
/**
 * @brief 遍历training_features，收集所有描述子指针到features，不拷贝原始描述子数据
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::getFeatures(
  const vector<vector<TDescriptor> > &training_features,
   vector<pDescriptor> &features) const
{
  features.resize(0);

  typename vector<vector<TDescriptor> >::const_iterator vvit;
  typename vector<TDescriptor>::const_iterator vit;
  for(vvit = training_features.begin(); vvit != training_features.end(); ++vvit)
  {
    features.reserve(features.size() + vvit->size());
    for(vit = vvit->begin(); vit != vvit->end(); ++vit)
    {
      features.push_back(&(*vit));
    }
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 层次Kmeans，递归构建词典树一层；对descriptors做kmeans聚类，生成子节点，递归下一层
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::HKmeansStep(NodeId parent_id,
   const vector<pDescriptor> &descriptors, int current_level)
{
  if(descriptors.empty()) return;

           // features associated to each cluster
  vector<TDescriptor> clusters;

        vector<vector<unsigned int> > groups; // groups[i] = [j1, j2, ...]
        // j1, j2, ... indices of descriptors associated to cluster i
  clusters.reserve(m_k);

        groups.reserve(m_k);

  //const int msizes[] = { m_k, descriptors.size() };
  //cv::SparseMat assoc(2, msizes, CV_8U);
  //cv::SparseMat last_assoc(2, msizes, CV_8U);
    //// assoc.row(cluster_idx).col(descriptor_idx) = 1 iif associated

  if((int)descriptors.size() <= m_k)
  {
    // trivial case: one cluster per feature
    groups.resize(descriptors.size());

    for(unsigned int i = 0; i < descriptors.size(); i++)
    {
      groups[i].push_back(i);
      clusters.push_back(*descriptors[i]);
    }
  }
  else
  {
    // select clusters and groups with kmeans

       bool first_time = true;
    bool goon = true;

       // to check if clusters move after iterations
    vector<int> last_association, current_association;

    while(goon)
    {
      // 1. Calculate clusters

                       if(first_time)
                       {
        // random sample
         initiateClusters(descriptors, clusters);
      }
      else
      {
        // calculate cluster centres

         for(unsigned int c = 0; c < clusters.size(); ++c)
        {
          vector<pDescriptor> cluster_descriptors;
          cluster_descriptors.reserve(groups[c].size());

                     /*
                     for(unsigned int d = 0; d < descriptors.size(); ++d)
                     {
                       if( assoc.find<unsigned char>(c, d) )
                       {
                         cluster_descriptors.push_back(descriptors[d]);
                       }
                     }
                     */

                     vector<unsigned int>::const_iterator vit;
          for(vit = groups[c].begin(); vit != groups[c].end(); ++vit)
          {
            cluster_descriptors.push_back(descriptors[*vit]);
          }

                                 F::meanValue(cluster_descriptors, clusters[c]);
        }

       } // if(!first_time)

      // 2. Associate features with clusters
      // calculate distances to cluster centers
      groups.clear();
      groups.resize(clusters.size(), vector<unsigned int>());
      current_association.resize(descriptors.size());

       //assoc.clear();

      typename vector<pDescriptor>::const_iterator fit;
      //unsigned int d = 0;
      for(fit = descriptors.begin(); fit != descriptors.end(); ++fit)//, ++d)
      {
        double best_dist = F::distance(*(*fit), clusters[0]);
        unsigned int icluster = 0;

         for(unsigned int c = 1; c < clusters.size(); ++c)
        {
          double dist = F::distance(*(*fit), clusters[c]);
          if(dist < best_dist)
          {
            best_dist = dist;
            icluster = c;
          }
        }

         //assoc.ref<unsigned char>(icluster, d) = 1;

         groups[icluster].push_back(fit - descriptors.begin());
        current_association[ fit - descriptors.begin() ] = icluster;
      }

             // kmeans++ ensures all the clusters has any feature associated with them

      // 3. check convergence
      if(first_time)
      {
        first_time = false;
      }
      else
      {
        //goon = !eqUChar(last_assoc, assoc);

                 goon = false;
        for(unsigned int i = 0; i < current_association.size(); i++)
        {
          if(current_association[i] != last_association[i]){
            goon = true;
            break;
          }
        }
      }

                        if(goon)
                        {
                               // copy last feature‑cluster association
                               last_association = current_association;
                               //last_assoc = assoc.clone();
                        }

                    } // while(goon)

   } // if must run kmeans

   // create nodes
  for(unsigned int i = 0; i < clusters.size(); ++i)
  {
    NodeId id = m_nodes.size();
    m_nodes.push_back(Node(id));
    m_nodes.back().descriptor = clusters[i];
    m_nodes.back().parent = parent_id;
    m_nodes[parent_id].children.push_back(id);
  }

   // go on with the next level
  if(current_level < m_L)
  {
    // iterate again with the resulting clusters
    const vector<NodeId> &children_ids = m_nodes[parent_id].children;
    for(unsigned int i = 0; i < clusters.size(); ++i)
    {
      NodeId id = children_ids[i];

       vector<pDescriptor> child_features;
      child_features.reserve(groups[i].size());

       vector<unsigned int>::const_iterator vit;
      for(vit = groups[i].begin(); vit != groups[i].end(); ++vit)
      {
        child_features.push_back(descriptors[*vit]);
      }

       if(child_features.size() > 1)
      {
        HKmeansStep(id, child_features, current_level + 1);
      }
    }
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 初始化聚类中心，调用kmeans++实现，可重写
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor, F>::initiateClusters
  (const vector<pDescriptor> &descriptors, vector<TDescriptor> &clusters) const
{
  initiateClustersKMpp(descriptors, clusters);
 }

// --------------------------------------------------------------------------
/**
 * @brief kmeans++种子初始化算法，选出m_k个初始聚类中心
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::initiateClustersKMpp(
  const vector<pDescriptor> &pfeatures, vector<TDescriptor> &clusters) const
{
  // Implements kmeans++ seeding algorithm
  // Algorithm:
  // 1. Choose one center uniformly at random from among the data points.
  // 2. For each data point x, compute D(x), the distance between x and the nearest
   //    center that has already been chosen.
  // 3. Add one new data point as a center. Each point x is chosen with probability
   //    proportional to D(x)^2.
  // 4. Repeat Steps 2 and 3 until k centers have been chosen.
  // 5. Now that the initial centers have been chosen, proceed using standard k‑means
   //    clustering.

   DUtils::Random::SeedRandOnce();

   clusters.resize(0);
  clusters.reserve(m_k);
  vector<double> min_dists(pfeatures.size(), std::numeric_limits<double>::max());

   // 1.
      int ifeature = DUtils::Random::RandomInt(0, pfeatures.size()-1);

      // create first cluster
   clusters.push_back(*pfeatures[ifeature]);

    // compute the initial distances
   typename vector<pDescriptor>::const_iterator fit;
   vector<double>::iterator dit;
   dit = min_dists.begin();
   for(fit = pfeatures.begin(); fit != pfeatures.end(); ++fit, ++dit)
   {
     *dit = F::distance(*(*fit), clusters.back());
   }

   while((int)clusters.size() < m_k)
   {
     // 2.
     dit = min_dists.begin();
     for(fit = pfeatures.begin(); fit != pfeatures.end(); ++fit, ++dit)
     {
       if(*dit > 0)
       {
         double dist = F::distance(*(*fit), clusters.back());
         if(dist < *dit) *dit = dist;
       }
     }

          // 3.
     double dist_sum = std::accumulate(min_dists.begin(), min_dists.end(), 0.0);

      if(dist_sum > 0)
     {
       double cut_d;
       do
       {
         cut_d = DUtils::Random::RandomValue<double>(0, dist_sum);
       } while(cut_d == 0.0);

        double d_up_now = 0;
       for(dit = min_dists.begin(); dit != min_dists.end(); ++dit)
       {
         d_up_now += *dit;
         if(d_up_now >= cut_d) break;
       }

              if(dit == min_dists.end())
          ifeature = pfeatures.size()-1;
       else
         ifeature = dit - min_dists.begin();

              clusters.push_back(*pfeatures[ifeature]);

     } // if dist_sum > 0
     else
       break;

          } // while(used_clusters < m_k)
}

// --------------------------------------------------------------------------
/**
 * @brief 遍历全部节点，叶子节点生成word_id，填充m_words数组
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::createWords()
{
  m_words.resize(0);

   if(!m_nodes.empty())
  {
    m_words.reserve( (int)pow((double)m_k, (double)m_L) );

     typename vector<Node>::iterator nit;

         nit = m_nodes.begin(); // ignore root
    for(++nit; nit != m_nodes.end(); ++nit)
    {
      if(nit->isLeaf())
      {
        nit->word_id = m_words.size();
        m_words.push_back( &(*nit) );
      }
    }
  }
}

// --------------------------------------------------------------------------
/**
 * @brief 根据训练图像统计每个单词出现的图像数，计算IDF权重 log(NDocs/Ni)
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::setNodeWeights
  (const vector<vector<TDescriptor> > &training_features)
{
  const unsigned int NWords = m_words.size();
  const unsigned int NDocs = training_features.size();

   if(m_weighting == TF || m_weighting == BINARY)
  {
    // idf part must be 1 always
    for(unsigned int i = 0; i < NWords; i++)
      m_words[i]->weight = 1;
  }
  else if(m_weighting == IDF || m_weighting == TF_IDF)
  {
    // IDF and TF‑IDF: we calculte the idf path now

      // Note: this actually calculates the idf part of the tf‑idf score.
     // The complete tf‑idf score is calculated in ::transform

      vector<unsigned int> Ni(NWords, 0);
    vector<bool> counted(NWords, false);

     typename vector<vector<TDescriptor> >::const_iterator mit;
    typename vector<TDescriptor>::const_iterator fit;

     for(mit = training_features.begin(); mit != training_features.end(); ++mit)
    {
      fill(counted.begin(), counted.end(), false);

       for(fit = mit->begin(); fit < mit->end(); ++fit)
      {
        WordId word_id;
        transform(*fit, word_id);

         if(!counted[word_id])
        {
          Ni[word_id]++;
          counted[word_id] = true;
        }
      }
    }

      // set ln(N/Ni)
    for(unsigned int i = 0; i < NWords; i++)
    {
      if(Ni[i] > 0)
      {
        m_words[i]->weight = log((double)NDocs / (double)Ni[i]);
      }// else // This cannot occur if using kmeans++
    }

     }
}

// --------------------------------------------------------------------------
/**
 * @brief 返回词典单词数量，叶子节点总数
 */
template<class TDescriptor, class F>
inline unsigned int TemplatedVocabulary<TDescriptor,F>::size() const
{
  return m_words.size();
}

// --------------------------------------------------------------------------
/**
 * @brief 判断词典是否为空
 */
template<class TDescriptor, class F>
inline bool TemplatedVocabulary<TDescriptor,F>::empty() const
{
  return m_words.empty();
}

// --------------------------------------------------------------------------
/**
 * @brief 计算所有叶子节点的实际平均深度
 */
template<class TDescriptor, class F>
float TemplatedVocabulary<TDescriptor,F>::getEffectiveLevels() const
{
  long sum = 0;
  typename std::vector<Node*>::const_iterator wit;
  for(wit = m_words.begin(); wit != m_words.end(); ++wit)
  {
    const Node *p = *wit;

         for(; p->id != 0; sum++) p = &m_nodes[p->parent];
  }

      return (float)((double)sum / (double)m_words.size());
}

// --------------------------------------------------------------------------
/**
 * @brief 获取指定word id对应的描述子
 */
template<class TDescriptor, class F>
TDescriptor TemplatedVocabulary<TDescriptor,F>::getWord(WordId wid) const
{
  return m_words[wid]->descriptor;
}

// --------------------------------------------------------------------------
/**
 * @brief 获取指定word id单词权重idf
 */
template<class TDescriptor, class F>
WordValue TemplatedVocabulary<TDescriptor, F>::getWordWeight(WordId wid) const
{
  return m_words[wid]->weight;
}

// --------------------------------------------------------------------------
/**
 * @brief 输入单个描述子，返回对应的word id
 */
template<class TDescriptor, class F>
WordId TemplatedVocabulary<TDescriptor, F>::transform
  (const TDescriptor& feature) const
{
  if(empty())
  {
    return 0;
  }

      WordId wid;
  transform(feature, wid);
  return wid;
}

// --------------------------------------------------------------------------
/**
 * @brief 输入一整组描述子，输出BowVector词袋向量；内部根据weighting策略做TF/BINARY/TF‑IDF，按需归一化
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform(
  const std::vector<TDescriptor>& features, BowVector &v) const
{
  v.clear();

      if(empty())
  {
    return;
  }

   // normalize
   LNorm norm;
  bool must = m_scoring_object->mustNormalize(norm);

          typename vector<TDescriptor>::const_iterator fit;

   if(m_weighting == TF || m_weighting == TF_IDF)
  {
    for(fit = features.begin(); fit < features.end(); ++fit)
    {
      WordId id;
      WordValue w;

       // w is the idf value if TF_IDF, 1 if TF

              transform(*fit, id, w);

              // not stopped
      if(w > 0) v.addWeight(id, w);
    }

          if(!v.empty() && !must)
    {
      // unnecessary when normalizing
      const double nd = v.size();
      for(BowVector::iterator vit = v.begin(); vit != v.end(); vit++)
         vit->second /= nd;
    }

     }
  else // IDF || BINARY
  {
    for(fit = features.begin(); fit < features.end(); ++fit)
    {
      WordId id;
      WordValue w;
      // w is idf if IDF, or 1 if BINARY

              transform(*fit, id, w);

              // not stopped
      if(w > 0) v.addIfNotExist(id, w);

           } // if add_features
  } // if m_weighting == ...

      if(must) v.normalize(norm);
}

// --------------------------------------------------------------------------
/**
 * @brief transform重载，同时输出BowVector与FeatureVector，记录每个原始特征对应向上回溯levelsup层的节点id
 */
template<class TDescriptor, class F>
 void TemplatedVocabulary<TDescriptor,F>::transform(
  const std::vector<TDescriptor>& features,
  BowVector &v, FeatureVector &fv, int levelsup) const
{
  v.clear();
  fv.clear();

      if(empty()) // safe for subclasses
  {
    return;
  }

      // normalize
   LNorm norm;
  bool must = m_scoring_object->mustNormalize(norm);

      typename vector<TDescriptor>::const_iterator fit;

      if(m_weighting == TF || m_weighting == TF_IDF)
  {
    unsigned int i_feature = 0;
    for(fit = features.begin(); fit < features.end(); ++fit, ++i_feature)
    {
      WordId id;
      NodeId nid;
      WordValue w;

       // w is the idf value if TF_IDF, 1 if TF

              transform(*fit, id, w, &nid, levelsup);

              if(w > 0) // not stopped
      {
         v.addWeight(id, w);
        fv.addFeature(nid, i_feature);
      }
    }

          if(!v.empty() && !must)
    {
      // unnecessary when normalizing
      const double nd = v.size();
      for(BowVector::iterator vit = v.begin(); vit != v.end(); vit++)
         vit->second /= nd;
    }

   }
  else // IDF || BINARY
  {
    unsigned int i_feature = 0;
    for(fit = features.begin(); fit < features.end(); ++fit, ++i_feature)
    {
      WordId id;
      NodeId nid;
      WordValue w;
      // w is idf if IDF, or 1 if BINARY

              transform(*fit, id, w, &nid, levelsup);

              if(w > 0) // not stopped
      {
        v.addIfNotExist(id, w);
        fv.addFeature(nid, i_feature);
      }
    }
  } // if m_weighting == ...

      if(must) v.normalize(norm);
}

// --------------------------------------------------------------------------
/**
 * @brief 调用打分对象计算两个BowVector相似度分数
 */
template<class TDescriptor, class F>
 inline double TemplatedVocabulary<TDescriptor,F>::score
  (const BowVector &v1, const BowVector &v2) const
{
  return m_scoring_object->score(v1, v2);
}

// --------------------------------------------------------------------------
/**
 * @brief 单个描述子transform，只输出word id，调用完整transform
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform
  (const TDescriptor &feature, WordId &id) const
{
  WordValue weight;
  transform(feature, id, weight);
}

// --------------------------------------------------------------------------
/**
 * @brief 核心匹配函数：描述子从根向下遍历树，每层找距离最近子节点，直到叶子；输出word_id、weight；可选输出向上回溯levelsup层节点nid
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform(const TDescriptor &feature,
   WordId &word_id, WordValue &weight, NodeId *nid, int levelsup) const
{
   // propagate the feature down the tree
  vector<NodeId> nodes;
  typename vector<NodeId>::const_iterator nit;

   // level at which the node must be stored in nid, if given
  const int nid_level = m_L - levelsup;
  if(nid_level <= 0 && nid != NULL) *nid = 0; // root

   NodeId final_id = 0; // root
  int current_level = 0;

   do
  {
    ++current_level;
    nodes = m_nodes[final_id].children;
    final_id = nodes[0];

     double best_d = F::distance(feature, m_nodes[final_id].descriptor);

     for(nit = nodes.begin() + 1; nit != nodes.end(); ++nit)
    {
      NodeId id = *nit;
      double d = F::distance(feature, m_nodes[id].descriptor);
      if(d < best_d)
      {
        best_d = d;
        final_id = id;
      }
    }

         if(nid != NULL && current_level == nid_level)
      *nid = final_id;

   } while( !m_nodes[final_id].isLeaf() );

   // turn node id into word id
  word_id = m_nodes[final_id].word_id;
  weight = m_nodes[final_id].weight;
}

// --------------------------------------------------------------------------
/**
 * @brief 根据word id，向上回溯levelsup层，返回对应node id
 */
template<class TDescriptor, class F>
NodeId TemplatedVocabulary<TDescriptor,F>::getParentNode
  (WordId wid, int levelsup) const
{
  NodeId ret = m_words[wid]->id; // node id
  while(levelsup > 0 && ret != 0) // ret == 0 --> root
  {
    --levelsup;
    ret = m_nodes[ret].parent;
  }
  return ret;
}

// --------------------------------------------------------------------------
/**
 * @brief 给定节点nid，遍历子树收集全部叶子word id存入words输出
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::getWordsFromNode
  (NodeId nid, std::vector<WordId> &words) const
{
  words.clear();

      if(m_nodes[nid].isLeaf())
  {
    words.push_back(m_nodes[nid].word_id);
  }
  else
  {
    words.reserve(m_k); // ^1, ^2, ...

         vector<NodeId> parents;
    parents.push_back(nid);

         while(!parents.empty())
    {
      NodeId parentid = parents.back();
      parents.pop_back();

              const vector<NodeId> &child_ids = m_nodes[parentid].children;
      vector<NodeId>::const_iterator cit;

              for(cit = child_ids.begin(); cit != child_ids.end(); ++cit)
      {
        const Node &child_node = m_nodes[*cit];

                 if(child_node.isLeaf())
          words.push_back(child_node.word_id);
        else
          parents.push_back(*cit);

               } // for each child
    } // while !parents.empty
  }
}

// --------------------------------------------------------------------------
/**
 * @brief stopWords，weight小于minWeight的单词weight置0，transform时忽略；返回被停用单词数量
 */
template<class TDescriptor, class F>
int TemplatedVocabulary<TDescriptor,F>::stopWords(double minWeight)
{
  int c = 0;
  typename vector<Node*>::iterator wit;
  for(wit = m_words.begin(); wit != m_words.end(); ++wit)
  {
    if((*wit)->weight < minWeight)
    {
      ++c;
      (*wit)->weight = 0;
    }
  }
  return c;
}

// --------------------------------------------------------------------------
/**
 * @brief ORB‑SLAM2新增：从文本文件加载词典，不依赖cv::FileStorage
 */
template<class TDescriptor, class F>
bool TemplatedVocabulary<TDescriptor,F>::loadFromTextFile(const std::string &filename)
{
     ifstream f;
     f.open(filename.c_str());

              if(f.eof())
         return false;

      m_words.clear();
     m_nodes.clear();

      string s;
     getline(f,s);
     stringstream ss;
     ss << s;
     ss >> m_k;
     ss >> m_L;
     int n1, n2;
     ss >> n1;
     ss >> n2;

      if(m_k<0 || m_k>20 || m_L<1 || m_L>10 || n1<0 || n1>5 || n2<0 || n2>3)
     {
         std::cerr << "Vocabulary loading failure: This is not a correct text file!" << endl;
         return false;
     }

          m_scoring = (ScoringType)n1;
     m_weighting = (WeightingType)n2;
     createScoringObject();

      // nodes
     int expected_nodes =
     (int)((pow((double)m_k, (double)m_L + 1) - 1)/(m_k - 1));
     m_nodes.reserve(expected_nodes);

      m_words.reserve(pow((double)m_k, (double)m_L + 1));

      m_nodes.resize(1);
     m_nodes[0].id = 0;
     while(!f.eof())
     {
         string snode;
         getline(f,snode);
         stringstream ssnode;
         ssnode << snode;

          int nid = m_nodes.size();
         m_nodes.resize(m_nodes.size()+1);
         m_nodes[nid].id = nid;

                  int pid ;
         ssnode >> pid;
         m_nodes[nid].parent = pid;
         m_nodes[pid].children.push_back(nid);

          int nIsLeaf;
         ssnode >> nIsLeaf;

          stringstream ssd;
         for(int iD=0;iD<F::L;iD++)
         {
             string sElement;
             ssnode >> sElement;
             ssd << sElement << " ";
         }
         F::fromString(m_nodes[nid].descriptor, ssd.str());

          ssnode >> m_nodes[nid].weight;

          if(nIsLeaf>0)
         {
             int wid = m_words.size();
             m_words.resize(wid+1);

              m_nodes[nid].word_id = wid;
             m_words[wid] = &m_nodes[nid];
         }
         else
         {
             m_nodes[nid].children.reserve(m_k);
         }
     }

      return true;
}

// --------------------------------------------------------------------------
/**
 * @brief ORB‑SLAM2新增：把词典保存为纯文本文件
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::saveToTextFile(const std::string &filename) const
{
     fstream f;
     f.open(filename.c_str(),ios_base::out);
     f << m_k << " " << m_L << " " << " " << m_scoring << " " << m_weighting << endl;

      for(size_t i=1; i<m_nodes.size();i++)
     {
         const Node& node = m_nodes[i];

          f << node.parent << " ";
         if(node.isLeaf())
             f << 1 << " ";
         else
             f << 0 << " ";

          f << F::toString(node.descriptor) << " " << (double)node.weight << endl;
     }

      f.close();
}

// --------------------------------------------------------------------------
/**
 * @brief 保存词典到yaml文件，string文件名入口
 */
template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::save(const std::string &filename) const
{
  cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
  if(!fs.isOpened()) throw string("Could not open file ") + filename;

     save(fs);
}

// --------------------------------------------------------------------------
/**
 * @brief 从yaml文件加载词典，string文件名入口
 */
template<class TDescriptor, class F> void TemplatedVocabulary<TDescriptor,F>::load(const std::string &filename) {
    // 使用OpenCV的FileStorage以只读模式打开YAML/XML词典文件
    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    // 判断文件是否成功打开，打开失败则抛出异常，携带出错文件名
    if(!fs.isOpened()) throw string("Could not open file ") + filename;

    // 调用内部重载load函数，从已打开的FileStorage对象读取词典数据
    this->load(fs);
}

// --------------------------------------------------------------------------
template<class TDescriptor, class F> void TemplatedVocabulary<TDescriptor,F>::save(cv::FileStorage &f,
    const std::string &name) const {
    // YAML输出格式说明：
    // vocabulary
    // {
    //   k:                // 树的分支因子，每个非叶子节点最大子节点数量
    //   L:                // 树的层级深度
    //   scoringType:      // 打分算法类型枚举值
    //   weightingType:    // 权重计算方式枚举值
    //   nodes             // 词典树节点数组
    //   [
    //     {
    //       nodeId:       // 节点唯一ID
    //       parentId:     // 父节点ID
    //       weight:       // 节点IDF权重
    //       descriptor:   // 节点对应的特征描述子字符串
    //     }
    //   ]
    //   words             // 叶子单词数组，每个单词映射到树的叶子节点
    //   [
    //     {
    //       wordId:      // 单词的索引ID
    //       nodeId:      // 该单词对应的树叶子节点ID
    //     }
    //   ]
    // }
    //
    // 根节点索引为0，根节点不存入nodes序列化数组中

    // 写入外层词典大括号起始标记，name为外层key名称
    f << name << "{";

    // 保存分支因子k
    f << "k" << m_k;
    // 保存树深度L
    f << "L" << m_L;
    // 保存打分类型枚举
    f << "scoringType" << m_scoring;
    // 保存权重加权类型枚举
    f << "weightingType" << m_weighting;

    // ----------------序列化树节点nodes数组----------------
    f << "nodes" << "[";
    vector<NodeId> parents, children;
    vector<NodeId>::const_iterator pit;

    // 根节点ID=0，压入栈，开始深度优先遍历整棵词典树
    parents.push_back(0); // root

    // 深度优先循环，栈不为空持续处理节点
    while(!parents.empty())
    {
        // 取出栈末尾元素作为当前待处理父节点ID，弹出栈
        NodeId pid = parents.back();
        parents.pop_back();

        // 获取父节点的Node结构体引用
        const Node& parent = m_nodes[pid];
        // 获取父节点全部子节点ID列表
        children = parent.children;

        // 遍历当前父节点所有子节点
        for(pit = children.begin(); pit != children.end(); pit++)
        {
            // *pit是子节点ID，拿到子节点结构体引用
            const Node& child = m_nodes[*pit];

            // 写入单个节点yaml块，{:代表流输出紧凑格式
            f << "{:";
            // 写入节点ID，强转int输出
            f << "nodeId" << (int)child.id;
            // 写入该节点的父节点ID
            f << "parentId" << (int)pid;
            // 写入节点idf权重，强转double存储
            f << "weight" << (double)child.weight;
            // 使用F模板类的toString把描述子转为字符串存入yaml
            f << "descriptor" << F::toString(child.descriptor);
            // 当前节点字段结束
            f << "}";

            // 如果该子节点不是叶子节点，把它压入parents栈，后续遍历它的子孩子
            if(!child.isLeaf())
            {
                parents.push_back(*pit);
            }
        }
    }

    f << "]"; // nodes数组闭合

    // ----------------序列化words单词数组----------------
    f << "words" << "[";

    // m_words存储所有单词，元素是Node*指针，指向树的叶子节点
    typename vector<Node*>::const_iterator wit;
    // 遍历全部单词
    for(wit = m_words.begin(); wit != m_words.end(); wit++)
    {
        // wordId = 当前迭代器相对于容器首元素的偏移下标
        WordId id = wit - m_words.begin();
        f << "{:";
        f << "wordId" << (int)id;
        // 获取该单词对应的树叶子节点id
        f << "nodeId" << (int)(*wit)->id;
        f << "}";
    }

    f << "]"; // words数组闭合

    f << "}"; // 外层vocabulary大括号闭合
}

// --------------------------------------------------------------------------
template<class TDescriptor, class F> void TemplatedVocabulary<TDescriptor,F>::load(const cv::FileStorage &fs,
    const std::string &name) {
    // 清空原有单词列表、树节点容器，准备加载新词典
    m_words.clear();
    m_nodes.clear();

    // 根据name拿到yaml词典根节点FileNode
    cv::FileNode fvoc = fs[name];

    // 读取分支因子k
    m_k = (int)fvoc["k"];
    // 读取树深度L
    m_L = (int)fvoc["L"];
    // 读取打分类型，强转为ScoringType枚举
    m_scoring = (ScoringType)((int)fvoc["scoringType"]);
    // 读取加权类型，强转为WeightingType枚举
    m_weighting = (WeightingType)((int)fvoc["weightingType"]);

    // 根据读取到的scoringType创建对应的打分计算对象
    createScoringObject();

    // ----------------读取nodes树节点数组----------------
    cv::FileNode fn = fvoc["nodes"];

    // 节点容器resize，+1留出下标0给根节点（根节点不在yaml节点数组内）
    m_nodes.resize(fn.size() + 1); // +1 to include root
    // 设置根节点id=0
    m_nodes[0].id = 0;

    // 遍历yaml中每一个节点记录
    for(unsigned int i = 0; i < fn.size(); ++i)
    {
        // 读取yaml保存的节点ID
        NodeId nid = (int)fn[i]["nodeId"];
        // 读取该节点的父节点ID
        NodeId pid = (int)fn[i]["parentId"];
        // 读取节点idf权重
        WordValue weight = (WordValue)fn[i]["weight"];
        // 读取序列化后的描述子字符串
        string d = (string)fn[i]["descriptor"];

        // 赋值节点ID
        m_nodes[nid].id = nid;
        // 设置该节点的父节点编号
        m_nodes[nid].parent = pid;
        // 设置节点权重
        m_nodes[nid].weight = weight;
        // 将当前子节点nid加入父节点pid的children子节点列表
        m_nodes[pid].children.push_back(nid);

        // 调用F模板的fromString，字符串还原为TDescriptor描述子对象
        F::fromString(m_nodes[nid].descriptor, d);
    }

    // ----------------读取words单词数组----------------
    fn = fvoc["words"];

    // 根据yaml单词数量resize m_words容器
    m_words.resize(fn.size());

    // 遍历yaml每一条单词记录
    for(unsigned int i = 0; i < fn.size(); ++i)
    {
        // yaml存储的wordId
        NodeId wid = (int)fn[i]["wordId"];
        // yaml存储的该单词对应的树叶子节点nodeId
        NodeId nid = (int)fn[i]["nodeId"];

        // 在树节点上标记归属的word id
        m_nodes[nid].word_id = wid;
        // m_words[wid]保存指针，指向树中对应的叶子Node节点
        m_words[wid] = &m_nodes[nid];
    }
}

// --------------------------------------------------------------------------
/**
 * Writes printable information of the vocabulary
 * @param os stream to write to
 * @param voc
 */
template<class TDescriptor, class F> std::ostream& operator<<(std::ostream &os,
    const TemplatedVocabulary<TDescriptor,F> &voc) {
    // 输出词典基础信息：分支因子k，树深度L
    os << "Vocabulary: k = " << voc.getBranchingFactor()
       << ", L = " << voc.getDepthLevels()
       << ", Weighting = ";

    // 根据加权类型枚举输出可读字符串
    switch(voc.getWeightingType())
    {
        case TF_IDF: os << "tf-idf"; break;
        case TF: os << "tf"; break;
        case IDF: os << "idf"; break;
        case BINARY: os << "binary"; break;
    }

    os << ", Scoring = ";
    // 根据打分类型枚举输出可读字符串
    switch(voc.getScoringType())
    {
        case L1_NORM: os << "L1-norm"; break;
        case L2_NORM: os << "L2-norm"; break;
        case CHI_SQUARE: os << "Chi square distance"; break;
        case KL: os << "KL-divergence"; break;
        case BHATTACHARYYA: os << "Bhattacharyya coefficient"; break;
        case DOT_PRODUCT: os << "Dot product"; break;
    }

    // 输出词典单词总数量
    os << ", Number of words = " << voc.size();

    return os;
}

} // namespace DBoW2
#endif

