/**
 * File: ScoringObject.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions to compute bow scores
 * License: see the LICENSE.txt file
 *
 */
// 浮点极限常量头文件，DBL_EPSILON双精度机器epsilon
#include <cfloat>
// 模板词典头文件
#include "TemplatedVocabulary.h"
// 词袋向量BowVector头文件
#include "BowVector.h"

// 使用DBoW2命名空间
using namespace DBoW2;

// If you change the type of WordValue, make sure you change also the
// epsilon value (this is needed by the KL method)
// KL散度计算用到log极小值，防止log(0)负无穷；取双精度浮点数最小有效值的对数
const double GeneralScoring::LOG_EPS = log(DBL_EPSILON); // FLT_EPSILON

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
/**
 * @brief L1范数打分，计算两个归一化BowVector相似度
 * @param v1 图像1词袋向量
 * @param v2 图像2词袋向量
 * @return double 相似度分数，范围 [0,1]，越大代表两张图像越相似
 * @note 算法参考Nister 2006论文，利用有序map双指针遍历，只遍历两边都出现的单词，提升稀疏向量计算效率
 */
double L1Scoring::score(const BowVector &v1, const BowVector &v2) const {
  BowVector::const_iterator v1_it, v2_it;
  const BowVector::const_iterator v1_end = v1.end();
  const BowVector::const_iterator v2_end = v2.end();

  v1_it = v1.begin();
  v2_it = v2.begin();

  double score = 0;

  // 双指针同时遍历两个有序map BowVector
  while(v1_it != v1_end && v2_it != v2_end)
  {
    const WordValue& vi = v1_it->second;
    const WordValue& wi = v2_it->second;

    // 当前单词id相等，两边都存在该单词
    if(v1_it->first == v2_it->first)
    {
      score += fabs(vi - wi) - fabs(vi) - fabs(wi);

      // move v1 and v2 forward
      ++v1_it;
      ++v2_it;
    }
    else if(v1_it->first < v2_it->first)
    {
      // move v1 forward
      // v1的单词id更小，跳到大于等于v2当前id的位置，跳过v2不存在的单词
      v1_it = v1.lower_bound(v2_it->first);
      // v1_it = (first element >= v2_it.id)
    }
    else
    {
      // move v2 forward
      // v2的单词id更小，跳到大于等于v1当前id的位置，跳过v1不存在的单词
      v2_it = v2.lower_bound(v1_it->first);
      // v2_it = (first element >= v1_it.id)
    }
  }

  // ||v - w||_{L1} = 2 + Sum(|v_i - w_i| - |v_i| - |w_i|)
  //           for all i | v_i != 0 and w_i != 0
  // (Nister, 2006)
  // scaled_||v - w||_{L1} = 1 - 0.5 * ||v - w||_{L1}
  score = -score/2.0;

  return score; // [0..1]
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
/**
 * @brief L2范数打分，基于L2归一化词袋向量计算相似度
 * @param v1 图像1词袋向量，必须L2归一化
 * @param v2 图像2词袋向量，必须L2归一化
 * @return double 相似度分数 [0,1]，数值越大图像越相似
 */
double L2Scoring::score(const BowVector &v1, const BowVector &v2) const {
  BowVector::const_iterator v1_it, v2_it;
  const BowVector::const_iterator v1_end = v1.end();
  const BowVector::const_iterator v2_end = v2.end();

  v1_it = v1.begin();
  v2_it = v2.begin();

  double score = 0;

  // 双指针遍历两个有序稀疏BowVector
  while(v1_it != v1_end && v2_it != v2_end)
  {
    const WordValue& vi = v1_it->second;
    const WordValue& wi = v2_it->second;

    // 两边都存在该单词，累加vi*wi点积项
    if(v1_it->first == v2_it->first)
    {
      score += vi * wi;

      // move v1 and v2 forward
      ++v1_it;
      ++v2_it;
    }
    else if(v1_it->first < v2_it->first)
    {
      // move v1 forward
      v1_it = v1.lower_bound(v2_it->first);
      // v1_it = (first element >= v2_it.id)
    }
    else
    {
      // move v2 forward
      v2_it = v2.lower_bound(v1_it->first);
      // v2_it = (first element >= v1_it.id)
    }
  }

  // ||v - w||_{L2} = sqrt( 2 - 2 * Sum(v_i * w_i) )
  //            for all i | v_i != 0 and w_i != 0 )
  // (Nister, 2006)
  // 浮点舍入误差保护，点积理论最大值为1，防止略大于1
        if(score >= 1) // rounding errors
          score = 1.0;
        else
     score = 1.0 - sqrt(1.0 - score); // [0..1]

    return score;
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
/**
 * @brief 卡方距离打分Chi‑Square，计算两张图像词袋相似度
 * @param v1 图像1词袋向量
 * @param v2 图像2词袋向量
 * @return double 相似度分数 [0,1]，越大越相似
 */
double ChiSquareScoring::score(const BowVector &v1, const BowVector &v2)
   const {
  BowVector::const_iterator v1_it, v2_it;
  const BowVector::const_iterator v1_end = v1.end();
  const BowVector::const_iterator v2_end = v2.end();

  v1_it = v1.begin();
  v2_it = v2.begin();

  double score = 0;

  // all the items are taken into account

  while(v1_it != v1_end && v2_it != v2_end)
  {
    const WordValue& vi = v1_it->second;
    const WordValue& wi = v2_it->second;

    if(v1_it->first == v2_it->first)
    {
      // (v‑w)^2/(v+w) - v - w = -4 vw/(v+w)
      // we move the -4 out
      // 分母不为0时累加项
      if(vi + wi != 0.0) score += vi * wi / (vi + wi);

      // move v1 and v2 forward
      ++v1_it;
      ++v2_it;
    }
    else if(v1_it->first < v2_it->first)
    {
      // move v1 forward
      v1_it = v1.lower_bound(v2_it->first);
    }
    else
    {
      // move v2 forward
      v2_it = v2.lower_bound(v1_it->first);
    }
  }

       // this takes the -4 into account
   score = 2. * score; // [0..1]

   return score;
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
/**
 * @brief KL散度打分 Kullback‑Leibler，衡量两个概率分布差异；值越小分布越接近
 * @param v1 作为参考分布的词袋向量
 * @param v2 待比较词袋向量
 * @return double KL散度值，无固定归一区间，越小越相似
 * @note 只以v1全部单词为基准，v2独有的单词不参与计算；LOG_EPS避免log(0)
 */
double KLScoring::score(const BowVector &v1, const BowVector &v2) const {
   BowVector::const_iterator v1_it, v2_it;
  const BowVector::const_iterator v1_end = v1.end();
  const BowVector::const_iterator v2_end = v2.end();

  v1_it = v1.begin();
  v2_it = v2.begin();

  double score = 0;

  // all the items or v are taken into account

  while(v1_it != v1_end && v2_it != v2_end)
  {
    const WordValue& vi = v1_it->second;
    const WordValue& wi = v2_it->second;

    if(v1_it->first == v2_it->first)
    {
      // vi wi都不为0，计算vi*log(vi/wi)
      if(vi != 0 && wi != 0) score += vi * log(vi/wi);

      // move v1 and v2 forward
      ++v1_it;
      ++v2_it;
    }
    else if(v1_it->first < v2_it->first)
    {
      // move v1 forward
      // v1有该单词，v2没有，wi视为接近0，使用LOG_EPS
      score += vi * (log(vi) - LOG_EPS);
      ++v1_it;
    }
    else
    {
      // move v2_it forward, do not add any score
      // v2独有单词，不参与KL计算
      v2_it = v2.lower_bound(v1_it->first);
      // v2_it = (first element >= v1_it.id)
    }
  }

  // sum rest of items of v
  // 处理v1剩下的所有独有单词
  for(; v1_it != v1_end; ++v1_it)
     if(v1_it->second != 0)
      score += v1_it->second * (log(v1_it->second) - LOG_EPS);

  return score; // cannot be scaled
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
/**
 * @brief 巴氏距离 Bhattacharyya，计算两个词袋分布相似度
 * @param v1 词袋向量1
 * @param v2 词袋向量2
 * @return double 相似度分数，[0,1]，越大代表分布越接近
 */
double BhattacharyyaScoring::score(const BowVector &v1,
   const BowVector &v2) const {
  BowVector::const_iterator v1_it, v2_it;
  const BowVector::const_iterator v1_end = v1.end();
  const BowVector::const_iterator v2_end = v2.end();

  v1_it = v1.begin();
  v2_it = v2.begin();

  double score = 0;

  while(v1_it != v1_end && v2_it != v2_end)
  {
    const WordValue& vi = v1_it->second;
    const WordValue& wi = v2_it->second;

    if(v1_it->first == v2_it->first)
    {
      // 累加sqrt(vi * wi)巴氏系数项
      score += sqrt(vi * wi);

      // move v1 and v2 forward
      ++v1_it;
      ++v2_it;
    }
    else if(v1_it->first < v2_it->first)
    {
      // move v1 forward
      v1_it = v1.lower_bound(v2_it->first);
      // v1_it = (first element >= v2_it.id)
    }
    else
    {
      // move v2 forward
      v2_it = v2.lower_bound(v1_it->first);
      // v2_it = (first element >= v1_it.id)
    }
  }

  return score; // already scaled
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
/**
 * @brief 原始点积打分DotProduct，直接计算两个BowVector点积
 * @param v1 词袋向量1
 * @param v2 词袋向量2
 * @return double 点积结果，没有归一化到固定区间
 */
double DotProductScoring::score(const BowVector &v1,
   const BowVector &v2) const {
  BowVector::const_iterator v1_it, v2_it;
  const BowVector::const_iterator v1_end = v1.end();
  const BowVector::const_iterator v2_end = v2.end();

  v1_it = v1.begin();
  v2_it = v2.begin();

  double score = 0;

  while(v1_it != v1_end && v2_it != v2_end)
  {
    const WordValue& vi = v1_it->second;
    const WordValue& wi = v2_it->second;

    if(v1_it->first == v2_it->first)
    {
      // 共同单词，累加vi*wi
      score += vi * wi;

      // move v1 and v2 forward
      ++v1_it;
      ++v2_it;
    }
    else if(v1_it->first < v2_it->first)
    {
      // move v1 forward
      v1_it = v1.lower_bound(v2_it->first);
      // v1_it = (first element >= v2_it.id)
    }
    else
    {
      // move v2 forward
      v2_it = v2.lower_bound(v1_it->first);
      // v2_it = (first element >= v1_it.id)
    }
  }

  return score; // cannot scale
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
