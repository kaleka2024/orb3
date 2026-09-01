/**
 * File: ScoringObject.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions to compute bow scores
 * License: see the LICENSE.txt file
 *
 */
// 头文件保护宏，防止头文件重复包含
#ifndef __D_T_SCORING_OBJECT__
#define __D_T_SCORING_OBJECT__

// 引入BowVector，词袋向量数据结构
#include "BowVector.h"

namespace DBoW2 {

/**
 * @brief 打分函数抽象基类，定义词袋向量相似度计算统一接口
 */
class GeneralScoring {
public:
  /**
   * Computes the score between two vectors. Vectors must be sorted and
   * normalized if necessary
   * @param v (in/out) 输入第一个词袋向量
   * @param w (in/out) 输入第二个词袋向量
   * @return score 返回两个向量的相似度分数
   * @brief 纯虚函数，计算两个BowVector之间相似度；子类必须实现；向量需有序，部分算法要求预先归一化
   */
  virtual double score(const BowVector &v, const BowVector &w) const = 0;

  /**
   * Returns whether a vector must be normalized before scoring according
   * to the scoring scheme
   * @param norm norm to use [out]输出需要使用的归一化范数类型
   * @return true iff must normalize 返回true代表该打分算法要求向量预先归一化
   * @brief 纯虚函数，查询该打分算法是否需要对BowVector做归一化
   */
  virtual bool mustNormalize(LNorm &norm) const = 0;

  /// Log of epsilon KL散度计算使用，log机器极小值，避免log(0)得到负无穷
        static const double LOG_EPS;
    // If you change the type of WordValue, make sure you change also the
        // epsilon value (this is needed by the KL method)

  virtual ~GeneralScoring() {} //!< Required for virtual base classes 虚析构函数，多态继承基类必备，保证子类析构正确调用

};

/**
 * Macro for defining Scoring classes
 * @param NAME name of class 打分子类类名
 * @param MUSTNORMALIZE if vectors must be normalized to compute the score bool常量，是否需要归一化
 * @param NORM type of norm to use when MUSTNORMALIZE 使用哪种范数L1/L2
 * @brief 宏，批量生成各个打分派生类；减少重复模板代码；继承GeneralScoring，实现mustNormalize接口；score函数声明留给cpp实现
 */
#define __SCORING_CLASS(NAME, MUSTNORMALIZE, NORM) \
  NAME: public GeneralScoring \
  { public: \
    /** \
     * Computes score between two vectors \
     * @param v \
     * @param w \
     * @return score between v and w \
     */ \
    virtual double score(const BowVector &v, const BowVector &w) const; \
    \
    /** \
     * Says if a vector must be normalized according to the scoring function \
     * @param norm (out) if true, norm to use      \
     * @return true iff vectors must be normalized \
     */ \
    virtual inline bool mustNormalize(LNorm &norm) const  \
      { norm = NORM; return MUSTNORMALIZE; } \
  }

/// L1 Scoring object class L1范数相似度打分类，需要归一化，使用L1范数
__SCORING_CLASS(L1Scoring, true, L1);

/// L2 Scoring object class L2范数相似度打分类，需要归一化，使用L2范数
__SCORING_CLASS(L2Scoring, true, L2);

/// Chi square Scoring object class 卡方相似度打分类，需要归一化，使用L1范数
__SCORING_CLASS(ChiSquareScoring, true, L1);

/// KL divergence Scoring object class KL散度打分类，需要归一化，使用L1范数
__SCORING_CLASS(KLScoring, true, L1);

/// Bhattacharyya Scoring object class 巴氏系数相似度打分类，需要归一化，使用L1范数
__SCORING_CLASS(BhattacharyyaScoring, true, L1);

/// Dot product Scoring object class 原始点积打分类，不需要归一化，范数参数仅占位
__SCORING_CLASS(DotProductScoring, false, L1);

// 取消宏定义，避免后续代码污染
#undef __SCORING_CLASS

} // namespace DBoW2

#endif
