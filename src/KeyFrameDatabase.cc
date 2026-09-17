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

// 关键帧数据库头文件：基于词袋倒排索引的关键帧检索，用于回环检测、重定位、地图合并
#include "KeyFrameDatabase.h"
// STL标准库：双向链表
#include <list>
// STL标准库：智能指针
#include <memory>
// STL标准库：互斥锁，保证数据库多线程访问安全
#include <mutex>
// STL标准库：集合容器
#include <set>
// STL标准库：成对结构
#include <utility>
// STL标准库：动态数组
#include <vector>

// ORB-SLAM3内部：关键帧类
#include "KeyFrame.h"
// 第三方DBoW2库：词袋向量定义
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

namespace ORB_SLAM3 {

// ==============================================
// 关键帧数据库构造函数
// 输入：voc ORB词典智能指针
// 功能：初始化倒排文件，大小与词典单词数一致
// 倒排文件结构：每个单词ID对应一个链表，存储所有包含该单词的关键帧
// ==============================================
KeyFrameDatabase::KeyFrameDatabase(const std::shared_ptr<ORBVocabulary>& voc)
    : mpVoc(voc) {
  // 倒排文件大小初始化为词典总单词数
  mvInvertedFile.resize(voc->size());
}

// ==============================================
// 向数据库中添加一个关键帧
// 原理：遍历关键帧的词袋向量，将关键帧指针加入每个对应单词的倒排列表
// 线程安全：加互斥锁
// ==============================================
void KeyFrameDatabase::add(const std::shared_ptr<KeyFrame>& pKF) {
  unique_lock<mutex> lock(mMutex);

  // 遍历关键帧的每个词袋元素（单词ID，权重）
  for (auto const& vit : pKF->mBowVec) mvInvertedFile[vit.first].push_back(pKF);
}

// ==============================================
// 从数据库中删除一个关键帧
// 原理：遍历关键帧的所有单词，在对应倒排列表中查找并删除该关键帧
// ==============================================
void KeyFrameDatabase::erase(const std::shared_ptr<KeyFrame>& pKF) {
  unique_lock<mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  // 遍历关键帧的每个词袋单词
  for (auto vit : pKF->mBowVec) {
    // List of keyframes that share the word
    // 获取该单词对应的关键帧链表
    auto& lKFs = mvInvertedFile[vit.first];

    // 在链表中查找目标关键帧并删除
    for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
      if (pKF == *lit) {
        lKFs.erase(lit);
        break;
      }
    }
  }
}

// ==============================================
// 清空整个数据库
// ==============================================
void KeyFrameDatabase::clear() {
  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

// ==============================================
// 清空指定地图的所有关键帧
// 原理：遍历所有倒排列表，删除属于目标地图的关键帧条目
// ==============================================
void KeyFrameDatabase::clearMap(const std::shared_ptr<Map>& pMap) {
  unique_lock<mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  // 遍历倒排文件的每个单词对应的链表
  for (auto lKFs : mvInvertedFile) {
    // List of keyframes that share the word
    // 遍历链表，注意删除元素时用erase返回值更新迭代器，避免迭代器失效
    for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend;) {
      // 关键帧属于目标地图则删除
      if (pMap == (*lit)->GetMap()) {
        lit = lKFs.erase(lit);
        // Dont delete the KF because the class Map clean all the KF when it is
        // destroyed
      } else {
        ++lit;
      }
    }
  }
}

// ==============================================
// 检测回环候选关键帧（同地图内）
// 输入：pKF 当前查询关键帧，minScore 最小相似度阈值
// 返回：按置信度排序的回环候选关键帧向量
// 算法：词袋初筛 + 共视邻域累积得分增强，典型的DBoW回环检测流程
// ==============================================
vector<std::shared_ptr<KeyFrame>> KeyFrameDatabase::DetectLoopCandidates(
    const std::shared_ptr<KeyFrame>& pKF, float minScore) {
  // 获取当前关键帧的共视关键帧集合，这些是相邻帧，直接排除，不可能是回环
  set<std::shared_ptr<KeyFrame>> spConnectedKeyFrames =
      pKF->GetConnectedKeyFrames();
  list<std::shared_ptr<KeyFrame>> lKFsSharingWords;

  // Search all keyframes that share a word with current keyframes
  // Discard keyframes connected to the query keyframe
  // 第一步：倒排索引初筛，找出所有与当前帧有公共单词的关键帧
  {
    unique_lock<mutex> lock(mMutex);

    // 遍历当前帧的每个词袋单词
    for (auto const vit : pKF->mBowVec) {
      auto const& lKFs = mvInvertedFile[vit.first];

      // 遍历包含该单词的所有关键帧
      for (auto pKFi : lKFs) {
        // For consider a loop candidate it a candidate it must be in the same
        // map
        // 仅考虑同地图的关键帧
        if (pKFi->GetMap() == pKF->GetMap()) {
          // 该帧还未被本次查询标记过
          if (pKFi->mnLoopQuery != pKF->mnId) {
            // 重置单词计数
            pKFi->mnLoopWords = 0;
            // 不是共视关键帧才加入候选列表
            if (!spConnectedKeyFrames.count(pKFi)) {
              pKFi->mnLoopQuery = pKF->mnId;
              lKFsSharingWords.push_back(pKFi);
            }
          }
          // 公共单词数计数+1
          pKFi->mnLoopWords++;
        }
      }
    }
  }

  // 无候选直接返回空
  if (lKFsSharingWords.empty()) return vector<std::shared_ptr<KeyFrame>>();

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lScoreAndMatch;

  // Only compare against those keyframes that share enough words
  // 第二步：按公共单词数过滤，保留单词数足够的候选
  int maxCommonWords = 0;
  // 找出最大公共单词数
  for (std::list<std::shared_ptr<KeyFrame>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    if ((*lit)->mnLoopWords > maxCommonWords)
      maxCommonWords = (*lit)->mnLoopWords;
  }

  // 阈值设为最大值的80%，过滤掉单词太少的噪声候选
  int minCommonWords = maxCommonWords * 0.8f;

  int nscores = 0;

  // Compute similarity score. Retain the matches whose score is higher than
  // minScore
  // 第三步：计算词袋相似度得分，超过最小阈值的保留
  for (std::list<std::shared_ptr<KeyFrame>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    std::shared_ptr<KeyFrame> pKFi = *lit;

    // 公共单词数超过阈值才计算得分
    if (pKFi->mnLoopWords > minCommonWords) {
      nscores++;

      // 计算两个词袋向量的相似度得分
      float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

      pKFi->mLoopScore = si;
      // 得分高于最小阈值则加入候选列表
      if (si >= minScore) lScoreAndMatch.push_back(std::make_pair(si, pKFi));
    }
  }

  if (lScoreAndMatch.empty()) return vector<std::shared_ptr<KeyFrame>>();

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lAccScoreAndMatch;
  float bestAccScore = minScore;

  // Lets now accumulate score by covisibility
  // 第四步：共视邻域累积得分增强——利用空间连续性，每个候选加上其共视帧的得分
  for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
           it = lScoreAndMatch.begin(),
           itend = lScoreAndMatch.end();
       it != itend; it++) {
    std::shared_ptr<KeyFrame> pKFi = it->second;
    // 获取该候选帧的前10个最佳共视关键帧
    vector<std::shared_ptr<KeyFrame>> vpNeighs =
        pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = it->first;
    std::shared_ptr<KeyFrame> pBestKF = pKFi;

    // 遍历共视邻域帧
    for (vector<std::shared_ptr<KeyFrame>>::iterator vit = vpNeighs.begin(),
                                                     vend = vpNeighs.end();
         vit != vend; vit++) {
      std::shared_ptr<KeyFrame> pKF2 = *vit;

      // 共视帧也属于本次回环查询候选，且单词数达标
      if (pKF2->mnLoopQuery == pKF->mnId &&
          pKF2->mnLoopWords > minCommonWords) {
        // 累积得分
        accScore += pKF2->mLoopScore;
        // 更新邻域内最佳得分帧
        if (pKF2->mLoopScore > bestScore) {
          pBestKF = pKF2;
          bestScore = pKF2->mLoopScore;
        }
      }
    }

    // 保存累积得分和对应最佳帧
    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) bestAccScore = accScore;
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  // 第五步：取最高累积得分的75%作为阈值，筛选最终候选
  float minScoreToRetain = 0.75f * bestAccScore;

  set<std::shared_ptr<KeyFrame>> spAlreadyAddedKF;
  vector<std::shared_ptr<KeyFrame>> vpLoopCandidates;
  vpLoopCandidates.reserve(lAccScoreAndMatch.size());

  // 遍历累积得分列表，超过阈值且不重复的加入结果
  for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
           it = lAccScoreAndMatch.begin(),
           itend = lAccScoreAndMatch.end();
       it != itend; it++) {
    if (it->first > minScoreToRetain) {
      std::shared_ptr<KeyFrame> pKFi = it->second;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpLoopCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpLoopCandidates;
}

// ==============================================
// 同时检测回环候选（同地图）与合并候选（跨地图）
// 输入：pKF 查询关键帧，minScore 最小得分阈值
// 输出：vpLoopCand 回环候选，vpMergeCand 合并候选
// ==============================================
void KeyFrameDatabase::DetectCandidates(
    const std::shared_ptr<KeyFrame>& pKF, float minScore,
    vector<std::shared_ptr<KeyFrame>>& vpLoopCand,
    vector<std::shared_ptr<KeyFrame>>& vpMergeCand) {
  // 获取当前帧的共视关键帧，全部排除
  auto spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
  list<std::shared_ptr<KeyFrame>> lKFsSharingWordsLoop, lKFsSharingWordsMerge;

  // Search all keyframes that share a word with current keyframes
  // Discard keyframes connected to the query keyframe
  // 第一步：倒排索引初筛，分同地图（回环）和跨地图（合并）两类
  {
    unique_lock<mutex> lock(mMutex);

    // 遍历查询帧的每个词袋单词
    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(),
                                          vend = pKF->mBowVec.end();
         vit != vend; vit++) {
      list<std::shared_ptr<KeyFrame>>& lKFs = mvInvertedFile[vit->first];

      // 遍历包含该单词的所有关键帧
      for (auto pKFi : lKFs) {
        // For consider a loop candidate it a candidate it must be in the same
        // map
        // 同地图：作为回环候选
        if (pKFi->GetMap() == pKF->GetMap()) {
          if (pKFi->mnLoopQuery != pKF->mnId) {
            pKFi->mnLoopWords = 0;
            if (!spConnectedKeyFrames.count(pKFi)) {
              pKFi->mnLoopQuery = pKF->mnId;
              lKFsSharingWordsLoop.push_back(pKFi);
            }
          }
          pKFi->mnLoopWords++;
        } else if (!pKFi->GetMap()->IsBad()) {
          // 不同地图且地图有效：作为合并候选
          if (pKFi->mnMergeQuery != pKF->mnId) {
            pKFi->mnMergeWords = 0;
            if (!spConnectedKeyFrames.count(pKFi)) {
              pKFi->mnMergeQuery = pKF->mnId;
              lKFsSharingWordsMerge.push_back(pKFi);
            }
          }
          pKFi->mnMergeWords++;
        }
      }
    }
  }

  // 两类都为空直接返回
  if (lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty()) return;

  // ========== 处理回环候选（同地图） ==========
  if (!lKFsSharingWordsLoop.empty()) {
    std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // 计算最大公共单词数
    int maxCommonWords = 0;
    for (auto lit = lKFsSharingWordsLoop.begin(),
              lend = lKFsSharingWordsLoop.end();
         lit != lend; lit++) {
      if ((*lit)->mnLoopWords > maxCommonWords)
        maxCommonWords = (*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    int nscores = 0;

    // Compute similarity score. Retain the matches whose score is higher than
    // minScore
    // 计算词袋相似度
    for (auto lit = lKFsSharingWordsLoop.begin(),
              lend = lKFsSharingWordsLoop.end();
         lit != lend; lit++) {
      std::shared_ptr<KeyFrame> pKFi = *lit;

      if (pKFi->mnLoopWords > minCommonWords) {
        nscores++;

        float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

        pKFi->mLoopScore = si;
        if (si >= minScore) lScoreAndMatch.push_back(std::make_pair(si, pKFi));
      }
    }

    if (!lScoreAndMatch.empty()) {
      std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lAccScoreAndMatch;
      float bestAccScore = minScore;

      // Lets now accumulate score by covisibility
      // 共视邻域累积得分
      for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
               it = lScoreAndMatch.begin(),
               itend = lScoreAndMatch.end();
           it != itend; it++) {
        std::shared_ptr<KeyFrame> pKFi = it->second;
        vector<std::shared_ptr<KeyFrame>> vpNeighs =
            pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        std::shared_ptr<KeyFrame> pBestKF = pKFi;
        for (vector<std::shared_ptr<KeyFrame>>::iterator vit = vpNeighs.begin(),
                                                         vend = vpNeighs.end();
             vit != vend; vit++) {
          std::shared_ptr<KeyFrame> pKF2 = *vit;
          if (pKF2->mnLoopQuery == pKF->mnId &&
              pKF2->mnLoopWords > minCommonWords) {
            accScore += pKF2->mLoopScore;
            if (pKF2->mLoopScore > bestScore) {
              pBestKF = pKF2;
              bestScore = pKF2->mLoopScore;
            }
          }
        }

        lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
        if (accScore > bestAccScore) bestAccScore = accScore;
      }

      // Return all those keyframes with a score higher than 0.75*bestScore
      // 阈值筛选
      float minScoreToRetain = 0.75f * bestAccScore;

      set<std::shared_ptr<KeyFrame>> spAlreadyAddedKF;
      vpLoopCand.reserve(lAccScoreAndMatch.size());

      for (auto it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end();
           it != itend; it++) {
        if (it->first > minScoreToRetain) {
          std::shared_ptr<KeyFrame> pKFi = it->second;
          if (!spAlreadyAddedKF.count(pKFi)) {
            vpLoopCand.push_back(pKFi);
            spAlreadyAddedKF.insert(pKFi);
          }
        }
      }
    }
  }

  // ========== 处理合并候选（跨地图） ==========
  if (!lKFsSharingWordsMerge.empty()) {
    std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // 最大公共单词数
    int maxCommonWords = 0;
    for (std::list<std::shared_ptr<KeyFrame>>::iterator
              lit = lKFsSharingWordsMerge.begin(),
              lend = lKFsSharingWordsMerge.end();
         lit != lend; lit++) {
      if ((*lit)->mnMergeWords > maxCommonWords)
        maxCommonWords = (*lit)->mnMergeWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    int nscores = 0;

    // Compute similarity score. Retain the matches whose score is higher than
    // minScore
    // 计算词袋相似度
    for (std::list<std::shared_ptr<KeyFrame>>::iterator
              lit = lKFsSharingWordsMerge.begin(),
              lend = lKFsSharingWordsMerge.end();
         lit != lend; lit++) {
      std::shared_ptr<KeyFrame> pKFi = *lit;

      if (pKFi->mnMergeWords > minCommonWords) {
        nscores++;

        float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

        pKFi->mMergeScore = si;
        if (si >= minScore) lScoreAndMatch.push_back(std::make_pair(si, pKFi));
      }
    }

    if (!lScoreAndMatch.empty()) {
      std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lAccScoreAndMatch;
      float bestAccScore = minScore;

      // Lets now accumulate score by covisibility
      // 共视邻域累积得分
      for (auto it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end();
           it != itend; it++) {
        std::shared_ptr<KeyFrame> pKFi = it->second;
        vector<std::shared_ptr<KeyFrame>> vpNeighs =
            pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        std::shared_ptr<KeyFrame> pBestKF = pKFi;
        for (vector<std::shared_ptr<KeyFrame>>::iterator vit = vpNeighs.begin(),
                                                         vend = vpNeighs.end();
             vit != vend; vit++) {
          std::shared_ptr<KeyFrame> pKF2 = *vit;
          if (pKF2->mnMergeQuery == pKF->mnId &&
              pKF2->mnMergeWords > minCommonWords) {
            accScore += pKF2->mMergeScore;
            if (pKF2->mMergeScore > bestScore) {
              pBestKF = pKF2;
              bestScore = pKF2->mMergeScore;
            }
          }
        }

        lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
        if (accScore > bestAccScore) bestAccScore = accScore;
      }

      // Return all those keyframes with a score higher than 0.75*bestScore
      // 阈值筛选
      float minScoreToRetain = 0.75f * bestAccScore;

      set<std::shared_ptr<KeyFrame>> spAlreadyAddedKF;
      vpMergeCand.reserve(lAccScoreAndMatch.size());

      for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
               it = lAccScoreAndMatch.begin(),
               itend = lAccScoreAndMatch.end();
           it != itend; it++) {
        if (it->first > minScoreToRetain) {
          std::shared_ptr<KeyFrame> pKFi = it->second;
          if (!spAlreadyAddedKF.count(pKFi)) {
            vpMergeCand.push_back(pKFi);
            spAlreadyAddedKF.insert(pKFi);
          }
        }
      }
    }
  }

  // 重置所有关键帧的查询标记，避免影响下次查询
  for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(),
                                        vend = pKF->mBowVec.end();
       vit != vend; vit++) {
    list<std::shared_ptr<KeyFrame>>& lKFs = mvInvertedFile[vit->first];

    for (auto pKFi : lKFs) {
      pKFi->mnLoopQuery = -1;
      pKFi->mnMergeQuery = -1;
    }
  }
}

// ==============================================
// 检测最佳候选（位置识别用），区分回环/合并候选
// ==============================================
void KeyFrameDatabase::DetectBestCandidates(
    const std::shared_ptr<KeyFrame>& pKF,
    vector<std::shared_ptr<KeyFrame>>& vpLoopCand,
    vector<std::shared_ptr<KeyFrame>>& vpMergeCand, int nMinWords) {
  list<std::shared_ptr<KeyFrame>> lKFsSharingWords;
  set<std::shared_ptr<KeyFrame>> spConnectedKF;

  // Search all keyframes that share a word with current frame
  // 倒排索引初筛
  {
    unique_lock<mutex> lock(mMutex);

    spConnectedKF = pKF->GetConnectedKeyFrames();

    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(),
                                          vend = pKF->mBowVec.end();
         vit != vend; vit++) {
      list<std::shared_ptr<KeyFrame>>& lKFs = mvInvertedFile[vit->first];

      for (std::list<std::shared_ptr<KeyFrame>>::iterator
               lit = lKFs.begin(),
               lend = lKFs.end();
           lit != lend; lit++) {
        std::shared_ptr<KeyFrame> pKFi = *lit;
        // 跳过共视关键帧
        if (spConnectedKF.find(pKFi) != spConnectedKF.end()) {
          continue;
        }
        // 新查询则重置计数
        if (pKFi->mnPlaceRecognitionQuery != pKF->mnId) {
          pKFi->mnPlaceRecognitionWords = 0;
          pKFi->mnPlaceRecognitionQuery = pKF->mnId;
          lKFsSharingWords.push_back(pKFi);
        }
        pKFi->mnPlaceRecognitionWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) return;

  // Only compare against those keyframes that share enough words
  // 最大公共单词数
  int maxCommonWords = 0;
  for (std::list<std::shared_ptr<KeyFrame>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    if ((*lit)->mnPlaceRecognitionWords > maxCommonWords)
      maxCommonWords = (*lit)->mnPlaceRecognitionWords;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  // 不小于指定最小单词数
  if (minCommonWords < nMinWords) {
    minCommonWords = nMinWords;
  }

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  // 计算词袋相似度
  for (std::list<std::shared_ptr<KeyFrame>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    std::shared_ptr<KeyFrame> pKFi = *lit;

    if (pKFi->mnPlaceRecognitionWords > minCommonWords) {
      nscores++;
      float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
      pKFi->mPlaceRecognitionScore = si;
      lScoreAndMatch.push_back(std::make_pair(si, pKFi));
    }
  }

  if (lScoreAndMatch.empty()) return;

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lAccScoreAndMatch;
  float bestAccScore = 0;

  // Lets now accumulate score by covisibility
  // 共视邻域累积得分
  for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
           it = lScoreAndMatch.begin(),
           itend = lScoreAndMatch.end();
       it != itend; it++) {
    std::shared_ptr<KeyFrame> pKFi = it->second;
    auto vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = bestScore;
    auto pBestKF = pKFi;
    for (vector<std::shared_ptr<KeyFrame>>::iterator vit = vpNeighs.begin(),
                                                     vend = vpNeighs.end();
         vit != vend; vit++) {
      auto pKF2 = *vit;
      if (pKF2->mnPlaceRecognitionQuery != pKF->mnId) continue;

      accScore += pKF2->mPlaceRecognitionScore;
      if (pKF2->mPlaceRecognitionScore > bestScore) {
        pBestKF = pKF2;
        bestScore = pKF2->mPlaceRecognitionScore;
      }
    }
    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) bestAccScore = accScore;
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  // 75%最高得分阈值筛选
  float minScoreToRetain = 0.75f * bestAccScore;
  set<std::shared_ptr<KeyFrame>> spAlreadyAddedKF;
  vpLoopCand.reserve(lAccScoreAndMatch.size());
  vpMergeCand.reserve(lAccScoreAndMatch.size());
  for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
           it = lAccScoreAndMatch.begin(),
           itend = lAccScoreAndMatch.end();
       it != itend; it++) {
    const float& si = it->first;
    if (si > minScoreToRetain) {
      std::shared_ptr<KeyFrame> pKFi = it->second;
      if (!spAlreadyAddedKF.count(pKFi)) {
        // 同地图加入回环候选
        if (pKF->GetMap() == pKFi->GetMap()) {
          vpLoopCand.push_back(pKFi);
        } else {
          // 不同地图加入合并候选
          vpMergeCand.push_back(pKFi);
        }
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }
}

// ==============================================
// 比较函数：按得分降序排列，用于排序
// ==============================================
bool compFirst(const std::pair<float, std::shared_ptr<KeyFrame>>& a,
               const std::pair<float, std::shared_ptr<KeyFrame>>& b) {
  return a.first > b.first;
}

// ==============================================
// 检测前N个最佳候选，分别输出回环和合并候选
// 输入：nNumCandidates 需要返回的候选数量
// ==============================================
void KeyFrameDatabase::DetectNBestCandidates(
    const std::shared_ptr<KeyFrame>& pKF,
    vector<std::shared_ptr<KeyFrame>>& vpLoopCand,
    vector<std::shared_ptr<KeyFrame>>& vpMergeCand, size_t nNumCandidates) {
  list<std::shared_ptr<KeyFrame>> lKFsSharingWords;
  set<std::shared_ptr<KeyFrame>> spConnectedKF;

  // Search all keyframes that share a word with current frame
  // 倒排索引初筛
  {
    unique_lock<mutex> lock(mMutex);

    spConnectedKF = pKF->GetConnectedKeyFrames();

    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(),
                                          vend = pKF->mBowVec.end();
         vit != vend; vit++) {
      auto& lKFs = mvInvertedFile[vit->first];

      for (std::list<std::shared_ptr<KeyFrame>>::iterator
               lit = lKFs.begin(),
               lend = lKFs.end();
           lit != lend; lit++) {
        std::shared_ptr<KeyFrame> pKFi = *lit;

        if (pKFi->mnPlaceRecognitionQuery != pKF->mnId) {
          pKFi->mnPlaceRecognitionWords = 0;
          if (!spConnectedKF.count(pKFi)) {
            pKFi->mnPlaceRecognitionQuery = pKF->mnId;
            lKFsSharingWords.push_back(pKFi);
          }
        }
        pKFi->mnPlaceRecognitionWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) return;

  // Only compare against those keyframes that share enough words
  // 最大公共单词数
  int maxCommonWords = 0;
  for (auto pKFi : lKFsSharingWords) {
    if (pKFi->mnPlaceRecognitionWords > maxCommonWords)
      maxCommonWords = pKFi->mnPlaceRecognitionWords;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  // 计算词袋相似度
  for (std::list<std::shared_ptr<KeyFrame>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    std::shared_ptr<KeyFrame> pKFi = *lit;

    if (pKFi->mnPlaceRecognitionWords > minCommonWords) {
      nscores++;
      float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
      pKFi->mPlaceRecognitionScore = si;
      lScoreAndMatch.push_back(std::make_pair(si, pKFi));
    }
  }

  if (lScoreAndMatch.empty()) return;

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lAccScoreAndMatch;
  float bestAccScore = 0;

  // Lets now accumulate score by covisibility
  // 共视邻域累积得分
  for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
           it = lScoreAndMatch.begin(),
           itend = lScoreAndMatch.end();
       it != itend; it++) {
    std::shared_ptr<KeyFrame> pKFi = it->second;
    auto vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = bestScore;
    std::shared_ptr<KeyFrame> pBestKF = pKFi;
    for (auto pKF2 : vpNeighs) {
      if (pKF2->mnPlaceRecognitionQuery != pKF->mnId) continue;

      accScore += pKF2->mPlaceRecognitionScore;
      if (pKF2->mPlaceRecognitionScore > bestScore) {
        pBestKF = pKF2;
        bestScore = pKF2->mPlaceRecognitionScore;
      }
    }
    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) bestAccScore = accScore;
  }

  // 按累积得分降序排序
  lAccScoreAndMatch.sort(compFirst);

  vpLoopCand.reserve(nNumCandidates);
  vpMergeCand.reserve(nNumCandidates);
  set<std::shared_ptr<KeyFrame>> spAlreadyAddedKF;
  size_t i = 0;
  std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator it =
      lAccScoreAndMatch.begin();

  // 取前N个候选，分别放入回环和合并列表
  while (i < lAccScoreAndMatch.size() &&
         (vpLoopCand.size() < nNumCandidates ||
          vpMergeCand.size() < nNumCandidates)) {
    std::shared_ptr<KeyFrame> pKFi = it->second;
    if (pKFi->isBad()) continue;

    if (!spAlreadyAddedKF.count(pKFi)) {
      // 同地图且回环候选未满，加入回环
      if (pKF->GetMap() == pKFi->GetMap() &&
          vpLoopCand.size() < nNumCandidates) {
        vpLoopCand.push_back(pKFi);
      } else if (pKF->GetMap() != pKFi->GetMap() &&
                 vpMergeCand.size() < nNumCandidates &&
                 !pKFi->GetMap()->IsBad()) {
        // 不同地图且合并候选未满，加入合并
        vpMergeCand.push_back(pKFi);
      }
      spAlreadyAddedKF.insert(pKFi);
    }
    i++;
    it++;
  }
}

// ==============================================
// 检测重定位候选关键帧
// 输入：F 查询普通帧，pMap 目标地图
// 返回：目标地图内的重定位候选关键帧向量
// ==============================================
vector<std::shared_ptr<KeyFrame>> KeyFrameDatabase::DetectRelocalizationCandidates(
    const std::shared_ptr<Frame>& F, const std::shared_ptr<Map>& pMap) {
  list<std::shared_ptr<KeyFrame>> lKFsSharingWords;

  // Search all keyframes that share a word with current frame
  // 倒排索引初筛
  {
    unique_lock<mutex> lock(mMutex);

    // 遍历当前帧的词袋单词
    for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(),
                                          vend = F->mBowVec.end();
         vit != vend; vit++) {
      list<std::shared_ptr<KeyFrame>>& lKFs = mvInvertedFile[vit->first];

      for (std::list<std::shared_ptr<KeyFrame>>::iterator
               lit = lKFs.begin(),
               lend = lKFs.end();
           lit != lend; lit++) {
        std::shared_ptr<KeyFrame> pKFi = *lit;
        // 新查询则重置计数
        if (pKFi->mnRelocQuery != F->mnId) {
          pKFi->mnRelocWords = 0;
          pKFi->mnRelocQuery = F->mnId;
          lKFsSharingWords.push_back(pKFi);
        }
        pKFi->mnRelocWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) return vector<std::shared_ptr<KeyFrame>>();

  // Only compare against those keyframes that share enough words
  // 最大公共单词数
  int maxCommonWords = 0;
  for (std::list<std::shared_ptr<KeyFrame>>::iterator
           lit = lKFsSharingWords.begin(),
           lend = lKFsSharingWords.end();
       lit != lend; lit++) {
    if ((*lit)->mnRelocWords > maxCommonWords)
      maxCommonWords = (*lit)->mnRelocWords;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  // 计算词袋相似度
  for (auto pKFi : lKFsSharingWords) {
    if (pKFi->mnRelocWords > minCommonWords) {
      nscores++;
      float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
      pKFi->mRelocScore = si;
      lScoreAndMatch.push_back(std::make_pair(si, pKFi));
    }
  }

  if (lScoreAndMatch.empty()) return vector<std::shared_ptr<KeyFrame>>();

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lAccScoreAndMatch;
  float bestAccScore = 0;

  // Lets now accumulate score by covisibility
  // 共视邻域累积得分
  for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
           it = lScoreAndMatch.begin(),
           itend = lScoreAndMatch.end();
       it != itend; it++) {
    std::shared_ptr<KeyFrame> pKFi = it->second;
    vector<std::shared_ptr<KeyFrame>> vpNeighs =
        pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = bestScore;
    std::shared_ptr<KeyFrame> pBestKF = pKFi;
    for (auto pKF2 : vpNeighs) {
      if (pKF2->mnRelocQuery != F->mnId) continue;

      accScore += pKF2->mRelocScore;
      if (pKF2->mRelocScore > bestScore) {
        pBestKF = pKF2;
        bestScore = pKF2->mRelocScore;
      }
    }
    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) bestAccScore = accScore;
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  // 75%阈值筛选
  float minScoreToRetain = 0.75f * bestAccScore;
  set<std::shared_ptr<KeyFrame>> spAlreadyAddedKF;
  vector<std::shared_ptr<KeyFrame>> vpRelocCandidates;
  vpRelocCandidates.reserve(lAccScoreAndMatch.size());
  for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator
           it = lAccScoreAndMatch.begin(),
           itend = lAccScoreAndMatch.end();
       it != itend; it++) {
    const float& si = it->first;
    if (si > minScoreToRetain) {
      std::shared_ptr<KeyFrame> pKFi = it->second;
      // 仅保留目标地图内的关键帧
      if (pKFi->GetMap() != pMap) continue;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpRelocCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpRelocCandidates;
}

// ==============================================
// 设置ORB词典，重置倒排文件
// ==============================================
void KeyFrameDatabase::SetORBVocabulary(
    const std::shared_ptr<ORBVocabulary>& pORBVoc) {
  // WTF?
  // ORBVocabulary** ptr;
  // ptr = (ORBVocabulary**)( &mpVoc );
  // *ptr = pORBVoc;
  mpVoc = pORBVoc;

  // 清空并重新初始化倒排文件大小
  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

}  // namespace ORB_SLAM3