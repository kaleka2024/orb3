/**
 * @file ORBmatcher.cc
 * @brief ORB特征匹配器实现，包含投影匹配、词袋匹配、三角化匹配、帧间跟踪、地图点融合等全套匹配功能
 * 属于ORB-SLAM3视觉SLAM库
 */

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

#include "ORBmatcher.h"           // ORB匹配器类头文件声明
#include <spdlog/spdlog.h>        // 高性能日志库，用于运行日志输出
#include <stdint.h>               // 标准整数类型定义（uint32_t、int32_t等）
#include <limits>                  // 数值极限常量（INT_MAX等）
#include <memory>                 // 智能指针（shared_ptr）
#include <opencv2/core/core.hpp>     // OpenCV核心模块，Mat、KeyPoint等基础类型
#include <set>                    // STL集合容器，用于去重、存在性判断
#include <utility>                // STL pair、swap等通用工具
#include <vector>                  // STL动态数组容器
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"  // DBoW2词袋特征向量，用于快速词袋匹配

namespace ORB_SLAM3 {

// ==================== 类常量定义 ====================
const int ORBmatcher::TH_HIGH = 100;       // 严格匹配汉明距离上限（高阈值，更严格）
const int ORBmatcher::TH_LOW = 50;         // 宽松匹配汉明距离上限（低阈值，更宽松）
const int ORBmatcher::HISTO_LENGTH = 30;    // 旋转直方图分箱数，用于方向一致性校验

// ==================== 构造函数 ====================
/**
 * @brief ORB匹配器构造函数
 * @param nnratio 最佳与次佳匹配距离比阈值，用于剔除模糊匹配
 * @param checkOri 是否启用旋转方向一致性校验
 */
ORBmatcher::ORBmatcher(float nnratio, bool checkOri)
    : mfNNratio(nnratio), mbCheckOrientation(checkOri) {}

// ==================== 投影匹配（Frame-地图点） ====================
/**
 * @brief 通过投影法将地图点匹配到普通帧特征点
 * @param F 普通帧
 * @param vpMapPoints 待匹配地图点集合
 * @param th 搜索半径系数
 * @param bFarPoints 是否启用远景点过滤
 * @param thFarPoints 远景点深度阈值
 * @return 成功匹配数量
 */
int ORBmatcher::SearchByProjection(const std::shared_ptr<Frame> &F,
                                   const vector<MapPoint *> &vpMapPoints,
                                   const float th, const bool bFarPoints,
                                   const float thFarPoints) {
    int nmatches = 0, left = 0, right = 0;  // 总匹配数；左目匹配计数；右目匹配计数

    const bool bFactor = th != 1.0;  // 搜索半径是否需要缩放（th≠1时乘以特征点尺度）

    // 遍历所有待匹配地图点
    for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++) {
        MapPoint *pMP = vpMapPoints[iMP];  // 当前处理的地图点

        // 地图点不在左目也不在右目视野内，直接跳过
        if (!pMP->mbTrackInView && !pMP->mbTrackInViewR) continue;

        // 远景点过滤模式：深度超过远景点阈值的跳过
        if (bFarPoints && pMP->mTrackDepth > thFarPoints) continue;

        // 标记为坏点的地图点跳过
        if (pMP->isBad()) continue;

        // ==================== 左目投影匹配 ====================
        if (pMP->mbTrackInView) {
            const int &nPredictedLevel = pMP->mnTrackScaleLevel;  // 地图点预测的金字塔层级

            // 根据视角余弦计算基础搜索半径（视角越大半径越大）
            float r = RadiusByViewingCos(pMP->mTrackViewCos);

            if (bFactor) r *= th;  // 乘以阈值系数，得到最终搜索半径

            // 在预测投影点周围搜索指定半径内的特征点索引，跨上下一层金字塔
            const vector<size_t> vIndices =
                F->GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY,
                                        r * F->mvScaleFactors.at(nPredictedLevel),
                                        nPredictedLevel - 1, nPredictedLevel);

            if (!vIndices.empty()) {  // 找到候选特征点才继续
                const cv::Mat MPdescriptor = pMP->GetDescriptor();  // 地图点描述子

                int bestDist = 256;      // 最佳匹配汉明距离，初始为最大值（32字节最大256）
                int bestLevel = -1;     // 最佳匹配所在金字塔层级
                int bestDist2 = 256;     // 次佳匹配汉明距离
                int bestLevel2 = -1;    // 次佳匹配所在金字塔层级
                int bestIdx = -1;       // 最佳匹配特征点索引

                // 遍历所有候选特征点，找最佳和次佳匹配
                for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                                     vend = vIndices.end();
                     vit != vend; vit++) {
                    const size_t idx = *vit;

                    // 该特征点已经绑定地图点且观测数大于0，跳过避免重复匹配
                    if (F->mvpMapPoints[idx])
                        if (F->mvpMapPoints[idx]->Observations() > 0) continue;

                    // 单目模式且存在极线校正时，校验极线误差
                    if (F->Nleft == -1 && F->mvuRight[idx] > 0) {
                        const float er = fabs(pMP->mTrackProjXR - F->mvuRight[idx]);
                        // 极线误差超过半径阈值则跳过
                        if (er > r * F->mvScaleFactors[nPredictedLevel]) continue;
                    }

                    const cv::Mat &d = F->mDescriptors.row(idx);  // 候选特征点描述子

                    const int dist = DescriptorDistance(MPdescriptor, d);  // 计算汉明距离

                    if (dist < bestDist) {
                        // 优于当前最佳，更新次佳、最佳
                        bestDist2 = bestDist;
                        bestDist = dist;
                        bestLevel2 = bestLevel;
                        // 获取最佳匹配的金字塔层级，区分单目/左目/右目
                        bestLevel = (F->Nleft == -1) ? F->mvKeysUn[idx].octave
                                         : (idx < F->Nleft)
                                             ? F->mvKeys[idx].octave
                                             : F->mvKeysRight[idx - F->Nleft].octave;
                        bestIdx = idx;
                    } else if (dist < bestDist2) {
                        // 优于次佳，更新次佳
                        bestLevel2 = (F->Nleft == -1) ? F->mvKeysUn[idx].octave
                                          : (idx < F->Nleft)
                                               ? F->mvKeys[idx].octave
                                               : F->mvKeysRight[idx - F->Nleft].octave;
                        bestDist2 = dist;
                    }
                }

                // 最佳匹配小于高阈值，进行比例校验
                if (bestDist <= TH_HIGH) {
                    // 同层级且最佳距离大于次佳的nnratio倍，匹配模糊，跳过
                    if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                        continue;

                    // 不同层级 或 距离比例满足阈值，接受匹配
                    if (bestLevel != bestLevel2 || bestDist <= mfNNratio * bestDist2) {
                        F->mvpMapPoints[bestIdx] = pMP;  // 绑定地图点到特征点

                        // 双目模式且存在左右目预匹配，同时绑定右目对应点
                        if (F->Nleft != -1 && F->mvLeftToRightMatch[bestIdx] !=
                                                   -1) {
                            F->mvpMapPoints[F->mvLeftToRightMatch[bestIdx] + F->Nleft] = pMP;
                            nmatches++;
                            right++;
                        }

                        nmatches++;
                        left++;
                    }
                }
            }
        }

        // ==================== 右目投影匹配（双目模式） ====================
        if (F->Nleft != -1 && pMP->mbTrackInViewR) {
            const int &nPredictedLevel = pMP->mnTrackScaleLevelR;  // 右目预测金字塔层级
            if (nPredictedLevel != -1) {
                float r = RadiusByViewingCos(pMP->mTrackViewCosR);  // 右目视角计算搜索半径

                // 右目区域搜索特征点，最后参数true表示右目
                const vector<size_t> vIndices =
                    F->GetFeaturesInArea(pMP->mTrackProjXR, pMP->mTrackProjYR,
                                            r * F->mvScaleFactors[nPredictedLevel],
                                            nPredictedLevel - 1, nPredictedLevel, true);

                if (vIndices.empty()) continue;

                const cv::Mat MPdescriptor = pMP->GetDescriptor();

                int bestDist = 256;
                int bestLevel = -1;
                int bestDist2 = 256;
                int bestLevel2 = -1;
                int bestIdx = -1;

                // 遍历候选特征点找最佳次佳
                for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                                     vend = vIndices.end();
                     vit != vend; vit++) {
                    const size_t idx = *vit;

                    // 右目特征点已绑定地图点则跳过
                    if (F->mvpMapPoints[idx + F->Nleft])
                        if (F->mvpMapPoints[idx + F->Nleft]->Observations() > 0) continue;

                    const cv::Mat &d = F->mDescriptors.row(idx + F->Nleft);  // 右目描述子

                    const int dist = DescriptorDistance(MPdescriptor, d);

                    if (dist < bestDist) {
                        bestDist2 = bestDist;
                        bestDist = dist;
                        bestLevel2 = bestLevel;
                        bestLevel = F->mvKeysRight[idx].octave;
                        bestIdx = idx;
                    } else if (dist < bestDist2) {
                        bestLevel2 = F->mvKeysRight[idx].octave;
                        bestDist2 = dist;
                    }
                }

                // 最佳匹配小于高阈值，比例校验
                if (bestDist <= TH_HIGH) {
                    if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                        continue;

                    // 存在右到左预匹配，同时绑定左目对应点
                    if (F->Nleft != -1 && F->mvRightToLeftMatch[bestIdx] !=
                                                   -1) {
                        F->mvpMapPoints[F->mvRightToLeftMatch[bestIdx]] = pMP;
                        nmatches++;
                        left++;
                    }

                    F->mvpMapPoints[bestIdx + F->Nleft] = pMP;
                    nmatches++;
                    right++;
                }
            }
        }
    }

    return nmatches;
}

// ==================== 搜索半径计算 ====================
/**
 * @brief 根据视角余弦值计算特征点搜索半径
 * @param viewCos 视角方向余弦
 * @return 搜索半径像素值
 */
float ORBmatcher::RadiusByViewingCos(float viewCos) const {
    if (viewCos > 0.998)
        return 2.5;  // 接近正视，小半径2.5像素
    else
        return 4.0;  // 大视角，大半径4.0像素
}

// ==================== 词袋匹配（关键帧-Frame） ====================
/**
 * @brief 通过DBoW2词袋向量快速匹配关键帧与普通帧的特征点
 * @param pKF 关键帧
 * @param F 普通帧
 * @param vpMapPointMatches 输出匹配的地图点数组
 * @return 成功匹配数量
 */
int ORBmatcher::SearchByBoW(const std::shared_ptr<KeyFrame> &pKF,
                             const std::shared_ptr<Frame> &F,
                             vector<MapPoint *> &vpMapPointMatches) {
    const vector<MapPoint *> vpMapPointsKF = pKF->GetMapPointMatches();  // 关键帧对应的地图点
    vpMapPointMatches = vector<MapPoint *>(F->N, nullptr);  // 初始化输出数组，全空
    int nmatches = 0;  // 匹配计数

    vector<int> rotHist[HISTO_LENGTH];          // 旋转方向直方图
    for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);  // 每个bin预分配容量
    const float factor = 1.0f / HISTO_LENGTH;   // 角度转直方图bin系数

    // DBoW2特征向量迭代器，双指针同步遍历加速匹配
    DBoW2::FeatureVector::const_iterator KFit = pKF->mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator Fit = F->mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = pKF->mFeatVec.end();
    DBoW2::FeatureVector::const_iterator Fend = F->mFeatVec.end();

    // 日志输出两个特征向量大小
    oslog::info(
        "[ORBmatcher::SearchByBoW] {} feature vector in Keyframe, {} in Frame",
        pKF->mFeatVec.size(), F->mFeatVec.size());

    // 双指针遍历两个有序特征向量
    while (KFit != KFend && Fit != Fend) {
        if (KFit->first == Fit->first) {
            // 两个向量当前word id相同，处理该word下所有特征点
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;

            // 遍历关键帧该word的所有特征点
            for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++) {
                const unsigned int realIdxKF = vIndicesKF[iKF];
                MapPoint *pMP = vpMapPointsKF[realIdxKF];  // 关键帧特征点对应地图点

                if (!pMP) {
                    // oslog::warn("[ORBmatcher::SearchByBoW] Null map point, skipping");
                    continue;
                }

                if (pMP->isBad()) {
                    // oslog::warn("[ORBmatcher::SearchByBoW] Bad map point, skipping");
                    continue;
                }

                const cv::Mat &dKF = pKF->mDescriptors.row(realIdxKF);  // 关键帧描述子

                int bestDist1 = 256;    // 左目/单目最佳距离
                int bestIdxF = -1;     // 左目/单目最佳索引
                int bestDist2 = 256;    // 左目/单目次佳距离

                int bestDist1R = 256;   // 右目最佳距离
                int bestIdxFR = -1;    // 右目最佳索引
                int bestDist2R = 256;   // 右目次佳距离

                // 遍历当前帧该word的所有特征点
                for (size_t iF = 0; iF < vIndicesF.size(); iF++) {
                    if (F->Nleft == -1) {
                        // ==================== 单目模式 ====================
                        const unsigned int realIdxF = vIndicesF[iF];

                        if (vpMapPointMatches[realIdxF]) continue;  // 已匹配跳过

                        const cv::Mat &dF = F->mDescriptors.row(realIdxF);
                        const int dist = DescriptorDistance(dKF, dF);

                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = realIdxF;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    } else {
                        // ==================== 双目模式 ====================
                        const unsigned int realIdxF = vIndicesF[iF];

                        if (vpMapPointMatches[realIdxF]) continue;

                        const cv::Mat &dF = F->mDescriptors.row(realIdxF);
                        const int dist = DescriptorDistance(dKF, dF);

                        auto const FNleft_sizet = static_cast<size_t>(F->Nleft);

                        if (realIdxF < FNleft_sizet && dist < bestDist1) {
                            // 左目特征点
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = realIdxF;
                        } else if (realIdxF < FNleft_sizet && dist < bestDist2) {
                            bestDist2 = dist;
                        }

                        if (realIdxF >= FNleft_sizet && dist < bestDist1R) {
                            // 右目特征点
                            bestDist2R = bestDist1R;
                            bestDist1R = dist;
                            bestIdxFR = realIdxF;
                        } else if (realIdxF >= FNleft_sizet && dist < bestDist2R) {
                            bestDist2R = dist;
                        }
                    }
                }

                // ==================== 左目/单目匹配校验 ====================
                if (bestDist1 <= TH_LOW) {
                    // 最佳次佳比例满足阈值，接受匹配
                    if (static_cast<float>(bestDist1) <
                        mfNNratio * static_cast<float>(bestDist2)) {
                        vpMapPointMatches[bestIdxF] = pMP;  // 绑定地图点

                        // 获取关键帧特征点，用于方向校验
                        const cv::KeyPoint &kp =
                            (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF]
                            : (realIdxKF >= static_cast<size_t>(pKF->NLeft))
                                  ? pKF->mvKeysRight[realIdxKF -
                                                          static_cast<size_t>(pKF->NLeft)]
                                  : pKF->mvKeys[realIdxKF];

                        if (mbCheckOrientation) {
                            // 获取当前帧特征点
                            cv::KeyPoint &Fkp =
                                (!pKF->mpCamera2 || F->Nleft == -1) ? F->mvKeys[bestIdxF]
                                : (bestIdxF >= static_cast<size_t>(F->Nleft))
                                      ? F->mvKeysRight[bestIdxF - static_cast<size_t>(F->Nleft)]
                                      : F->mvKeys[bestIdxF];

                            // 计算角度差并入直方图
                            float rot = kp.angle - Fkp.angle;
                            if (rot < 0.0) rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH) bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }

                        nmatches++;
                    }
                }

                // ==================== 右目匹配校验（双目） ====================
                if (bestDist1R <= TH_LOW) {
                    if (static_cast<float>(bestDist1R) <
                            mfNNratio * static_cast<float>(bestDist2R) ||
                        true) {
                        vpMapPointMatches[bestIdxFR] = pMP;

                        const cv::KeyPoint &kp =
                            (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF]
                                : (realIdxKF >= static_cast<size_t>(pKF->NLeft))
                                      ? pKF->mvKeysRight[realIdxKF -
                                                          static_cast<size_t>(pKF->NLeft)]
                                      : pKF->mvKeys[realIdxKF];

                        if (mbCheckOrientation) {
                            cv::KeyPoint &Fkp =
                                (!F->mpCamera2) ? F->mvKeys[bestIdxFR]
                                    : (bestIdxFR >= static_cast<size_t>(F->Nleft))
                                          ? F->mvKeysRight[bestIdxFR - static_cast<size_t>(F->Nleft)]
                                          : F->mvKeys[bestIdxFR];

                            float rot = kp.angle - Fkp.angle;
                            if (rot < 0.0) rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH) bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxFR);
                        }

                        nmatches++;
                    }
                }
            }

            KFit++;  // 关键帧迭代器后移
            Fit++;   // 当前帧迭代器后移
        } else if (KFit->first < Fit->first) {
            // 关键帧word id更小，跳转到当前帧word位置
            KFit = pKF->mFeatVec.lower_bound(Fit->first);
        } else {
            // 当前帧word id更小，跳转到关键帧word位置
            Fit = F->mFeatVec.lower_bound(KFit->first);
        }
    }

    // ==================== 方向一致性校验 ====================
    const int nmatches_before_orientation_check = nmatches;
    if (mbCheckOrientation) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);  // 找直方图前三大峰值

        // 清除非三大峰值的所有匹配
        for (int i = 0; i < HISTO_LENGTH; i++) {
            if (i == ind1 || i == ind2 || i == ind3) continue;
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                nmatches--;
            }
        }
    }

    oslog::info("  Due to orientation check, {} matches -> {}",
               nmatches_before_orientation_check, nmatches);
    return nmatches;
}

// ==================== 投影匹配（关键帧-地图点，Sim3变换） ====================
/**
 * @brief 通过Sim3位姿投影匹配关键帧与地图点
 * @param pKF 关键帧
 * @param Scw 世界到相机的Sim3相似变换
 * @param vpPoints 待匹配地图点
 * @param vpMatched 输出匹配地图点数组
 * @param th 搜索半径系数
 * @param ratioHamming 汉明距离比例系数
 * @return 成功匹配数量
 */
int ORBmatcher::SearchByProjection(const std::shared_ptr<KeyFrame> &pKF,
                                    Sophus::Sim3f &Scw,
                                    const vector<MapPoint *> &vpPoints,
                                    vector<MapPoint *> &vpMatched, int th,
                                    float ratioHamming) {
    // 分解Sim3变换得到旋转和平移
    Sophus::SE3f Tcw =
        Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();  // 相机光心世界坐标

    // 已匹配地图点集合，去重
    set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint *>(NULL));

    int nmatches = 0;

    // 遍历所有候选地图点
    for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
        MapPoint *pMP = vpPoints[iMP];

        // 坏点或已匹配，跳过
        if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

        Eigen::Vector3f p3Dw = pMP->GetWorldPos();  // 地图点世界坐标
        Eigen::Vector3f p3Dc = Tcw * p3Dw;         // 变换到相机坐标系

        if (p3Dc(2) < 0.0) continue;  // 深度为负（相机后方），跳过

        // 投影到像素平面
        const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

        if (!pKF->IsInImage(uv(0), uv(1))) continue;  // 投影在图像外，跳过

        // 深度在尺度不变性范围内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3f PO = p3Dw - Ow;
        const float dist = PO.norm();

        if (dist < minDistance || dist > maxDistance) continue;

        // 视角小于60度（法线夹角）
        Eigen::Vector3f Pn = pMP->GetNormal();
        if (PO.dot(Pn) < 0.5 * dist) continue;

        int nPredictedLevel = pMP->PredictScale(dist, pKF);  // 预测金字塔层级

        const float radius = th * pKF->mvScaleFactors[nPredictedLevel];  // 搜索半径
        const vector<size_t> vIndices =
            pKF->GetFeaturesInArea(uv(0), uv(1), radius);  // 区域搜索特征点

        if (vIndices.empty()) continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;

        // 找最佳匹配
        for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                             vend = vIndices.end();
             vit != vend; vit++) {
            const size_t idx = *vit;
            if (vpMatched[idx]) continue;  // 已匹配跳过

            const int &kpLevel = pKF->mvKeysUn[idx].octave;
            // 层级差超过1层跳过
            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);
            const int dist = DescriptorDistance(dMP, dKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // 小于低阈值乘以比例系数，接受匹配
        if (bestDist <= TH_LOW * ratioHamming) {
            vpMatched[bestIdx] = pMP;
            nmatches++;
        }
    }

    return nmatches;
}

// ==================== 投影匹配（多关键帧版本） ====================
/**
 * @brief 多关键帧版本投影匹配，同时记录匹配的关键帧指针对
 * @param pKF 参考关键帧
 * @param Scw Sim3变换
 * @param vpPoints 待匹配地图点
 * @param vpPointsKFs 对应地图点的关键帧
 * @param vpMatched 输出匹配地图点
 * @param vpMatchedKF 输出匹配的关键帧
 * @param th 搜索半径
 * @param ratioHamming 汉明比例
 * @return 匹配数量
 */
int ORBmatcher::SearchByProjection(
    const std::shared_ptr<KeyFrame> &pKF, Sophus::Sim3<float> &Scw,
    const std::vector<MapPoint *> &vpPoints,
    const std::vector<std::shared_ptr<KeyFrame>> &vpPointsKFs,
    std::vector<MapPoint *> &vpMatched,
    std::vector<std::shared_ptr<KeyFrame>> &vpMatchedKF, int th,
    float ratioHamming) {
    // 相机内参
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    Sophus::SE3f Tcw =
        Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint *>(NULL));

    int nmatches = 0;

    for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
        MapPoint *pMP = vpPoints[iMP];
        std::shared_ptr<KeyFrame> pKFi = vpPointsKFs[iMP];

        if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

        Eigen::Vector3f p3Dw = pMP->GetWorldPos();
        Eigen::Vector3f p3Dc = Tcw * p3Dw;

        if (p3Dc(2) < 0.0) continue;

        // 反投影到像素
        const float invz = 1 / p3Dc(2);
        const float x = p3Dc(0) * invz;
        const float y = p3Dc(1) * invz;
        const float u = fx * x + cx;
        const float v = fy * y + cy;

        if (!pKF->IsInImage(u, v)) continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3f PO = p3Dw - Ow;
        const float dist = PO.norm();

        if (dist < minDistance || dist > maxDistance) continue;

        Eigen::Vector3f Pn = pMP->GetNormal();
        if (PO.dot(Pn) < 0.5 * dist) continue;

        int nPredictedLevel = pMP->PredictScale(dist, pKF);

        const float radius = th * pKF->mvScaleFactors[nPredictedLevel];
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

        if (vIndices.empty()) continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;

        for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                             vend = vIndices.end();
             vit != vend; vit++) {
            const size_t idx = *vit;
            if (vpMatched[idx]) continue;

            const int &kpLevel = pKF->mvKeysUn[idx].octave;
            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);
            const int dist = DescriptorDistance(dMP, dKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if (bestDist <= TH_LOW * ratioHamming) {
            vpMatched[bestIdx] = pMP;
            vpMatchedKF[bestIdx] = pKFi;
            nmatches++;
        }
    }

    return nmatches;
}

// ==================== 初始化匹配（前一帧-当前帧） ====================
/**
 * @brief 两帧之间特征点匹配，用于初始化跟踪
 * @param F1 前一帧
 * @param F2 当前帧
 * @param vbPrevMatched 上一帧匹配的像素坐标
 * @param vnMatches12 输出F1到F2的匹配索引
 * @param windowSize 搜索窗口大小
 * @return 匹配数量
 */
int ORBmatcher::SearchForInitialization(const std::shared_ptr<Frame> &F1,
                                         const std::shared_ptr<Frame> &F2,
                                         vector<cv::Point2f> &vbPrevMatched,
                                         vector<int> &vnMatches12,
                                         int windowSize) {
    int nmatches = 0;
    vnMatches12 = vector<int>(F1->mvKeysUn.size(), -1);  // 初始化匹配索引数组，-1表示未匹配

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    vector<int> vMatchedDistance(F2->mvKeysUn.size(), INT_MAX);  // 匹配距离数组
    vector<int> vnMatches21(F2->mvKeysUn.size(), -1);       // F2到F1反向匹配索引

    // 遍历F1所有特征点
    for (size_t i1 = 0, iend1 = F1->mvKeysUn.size(); i1 < iend1; i1++) {
        cv::KeyPoint kp1 = F1->mvKeysUn[i1];
        int level1 = kp1.octave;
        if (level1 > 0) continue;  // 只匹配第0层

        // 在上一帧位置周围搜索
        vector<size_t> vIndices2 = F2->GetFeaturesInArea(
            vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize, level1, level1);

        if (vIndices2.empty()) continue;

        cv::Mat d1 = F1->mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        // 找最佳次佳
        for (vector<size_t>::iterator vit = vIndices2.begin();
             vit != vIndices2.end(); vit++) {
            size_t i2 = *vit;
            cv::Mat d2 = F2->mDescriptors.row(i2);
            int dist = DescriptorDistance(d1, d2);

            if (vMatchedDistance[i2] <= dist) continue;

            if (dist < bestDist) {
                bestDist2 = bestDist;
                bestDist = dist;
                bestIdx2 = i2;
            } else if (dist < bestDist2) {
                bestDist2 = dist;
            }
        }

        if (bestDist <= TH_LOW) {
            if (bestDist < static_cast<float>(bestDist2 * mfNNratio)) {
                // 双向一致性校验
                if (vnMatches21[bestIdx2] >= 0) {
                    vnMatches12[vnMatches21[bestIdx2]] = -1;
                    nmatches--;
                }

                vnMatches12[i1] = bestIdx2;
                vnMatches21[bestIdx2] = i1;
                vMatchedDistance[bestIdx2] = bestDist;
                nmatches++;

                if (mbCheckOrientation) {
                    float rot = F1->mvKeysUn[i1].angle - F2->mvKeysUn[bestIdx2].angle;
                    if (rot < 0.0) rot += 360.0f;
                    int bin = round(rot * factor);
                    if (bin == HISTO_LENGTH) bin = 0;
                    assert(bin >= 0 && bin < HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }
    }

    // 方向一致性校验
    if (mbCheckOrientation) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++) {
            if (i == ind1 || i == ind2 || i == ind3) continue;
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                int idx1 = rotHist[i][j];
                if (vnMatches12[idx1] >= 0) {
                    vnMatches12[idx1] = -1;
                    nmatches--;
                }
            }
        }
    }

    // 更新上一帧匹配坐标为当前帧坐标
    for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
        if (vnMatches12[i1] >= 0)
            vbPrevMatched[i1] = F2->mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

// ==================== 词袋匹配（关键帧-关键帧） ====================
/**
 * @brief 两个关键帧之间通过词袋进行特征匹配
 * @param pKF1 关键帧1
 * @param pKF2 关键帧2
 * @param vpMatches12 输出匹配地图点数组
 * @return 匹配数量
 */
int ORBmatcher::SearchByBoW(const std::shared_ptr<KeyFrame> &pKF1,
                             const std::shared_ptr<KeyFrame> &pKF2,
                             vector<MapPoint *> &vpMatches12) {
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
    const cv::Mat &Descriptors1 = pKF1->mDescriptors;

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
    const cv::Mat &Descriptors2 = pKF2->mDescriptors;

    vpMatches12 =
        vector<MapPoint *>(vpMapPoints1.size(), static_cast<MapPoint *>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(), false);

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    int nmatches = 0;

    // 双指针遍历两个特征向量
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while (f1it != f1end && f2it != f2end) {
        if (f1it->first == f2it->first) {
            // 同word，遍历所有特征点对
            for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
                const size_t idx1 = f1it->second[i1];

                // 双目模式且索引超过左目范围，跳过
                if (pKF1->NLeft != -1 && idx1 >= pKF1->mvKeysUn.size()) {
                    continue;
                }

                MapPoint *pMP1 = vpMapPoints1[idx1];
                if (!pMP1) continue;
                if (pMP1->isBad()) continue;

                const cv::Mat &d1 = Descriptors1.row(idx1);

                int bestDist1 = 256;
                int bestIdx2 = -1;
                int bestDist2 = 256;

                for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
                    const size_t idx2 = f2it->second[i2];

                    if (pKF2->NLeft != -1 && idx2 >= pKF2->mvKeysUn.size()) {
                        continue;
                    }

                    MapPoint *pMP2 = vpMapPoints2[idx2];
                    if (vbMatched2[idx2] || !pMP2) continue;
                    if (pMP2->isBad()) continue;

                    const cv::Mat &d2 = Descriptors2.row(idx2);
                    int dist = DescriptorDistance(d1, d2);

                    if (dist < bestDist1) {
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                        bestIdx2 = idx2;
                    } else if (dist < bestDist2) {
                        bestDist2 = dist;
                    }
                }

                if (bestDist1 < TH_LOW) {
                    if (static_cast<float>(bestDist1) <
                        mfNNratio * static_cast<float>(bestDist2)) {
                        vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2] = true;

                        if (mbCheckOrientation) {
                            float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
                            if (rot < 0.0) rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH) bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }

                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        } else if (f1it->first < f2it->first) {
            f1it = vFeatVec1.lower_bound(f2it->first);
        } else {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    // 方向一致性校验
    if (mbCheckOrientation) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++) {
            if (i == ind1 || i == ind2 || i == ind3) continue;
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                vpMatches12[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

// ==================== 三角化匹配 ====================
/**
 * @brief 用于三角化的特征匹配，带极线约束
 * @param pKF1 关键帧1
 * @param pKF2 关键帧2
 * @param vMatchedPairs 输出匹配点对
 * @param bOnlyStereo 是否只匹配双目点
 * @param bCoarse 是否粗匹配
 * @return 匹配数量
 */
int ORBmatcher::SearchForTriangulation(
    const std::shared_ptr<KeyFrame> &pKF1,
    const std::shared_ptr<KeyFrame> &pKF2,
    vector<pair<size_t, size_t>> &vMatchedPairs, const bool bOnlyStereo,
    const bool bCoarse) {
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

    // 计算F2在F1中的极点
    Sophus::SE3f T1w = pKF1->GetPose();
    Sophus::SE3f T2w = pKF2->GetPose();
    Sophus::SE3f Tw2 = pKF2->GetPoseInverse();  // F2到F1变换

    Eigen::Vector3f Cw = pKF1->GetCameraCenter();
    Eigen::Vector3f C2 = T2w * Cw;
    Eigen::Vector2f ep = pKF2->mpCamera->project(C2);

    Sophus::SE3f T12;
    Sophus::SE3f Tll, Tlr, Trl, Trr;
    Eigen::Matrix3f R12;
    Eigen::Vector3f t12;

    std::shared_ptr<GeometricCamera> pCamera1 = pKF1->mpCamera,
                                pCamera2 = pKF2->mpCamera;

    // 单双目情况计算不同的变换
    if (!pKF1->mpCamera2 && !pKF2->mpCamera2) {
        T12 = T1w * Tw2;
        R12 = T12.rotationMatrix();
        t12 = T12.translation();
    } else {
        Sophus::SE3f Tr1w = pKF1->GetRightPose();
        Sophus::SE3f Twr2 = pKF2->GetRightPoseInverse();
        Tll = T1w * Tw2;
        Tlr = T1w * Twr2;
        Trl = Tr1w * Tw2;
        Trr = Tr1w * Twr2;
    }

    Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr = Tlr.rotationMatrix(),
                       Rrl = Trl.rotationMatrix(), Rrr = Trr.rotationMatrix();
    Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(),
                       trl = Trl.translation(), trr = Trr.translation();

    vector<bool> vbMatched2(pKF2->N, false);
    vector<int> vMatches12(pKF1->N, -1);

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    int nmatches = 0;

    // 双指针词袋匹配
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while (f1it != f1end && f2it != f2end) {
        if (f1it->first == f2it->first) {
            for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
                const size_t idx1 = f1it->second[i1];

                MapPoint *pMP1 = pKF1->GetMapPoint(idx1);

                if (pMP1) {
                    continue;
                }

                const bool bStereo1 = (!pKF1->mpCamera2 && pKF1->mvuRight[idx1] >= 0);

                if (bOnlyStereo)
                    if (!bStereo1) continue;

                const cv::KeyPoint &kp1 = (pKF1->NLeft == -1) ? pKF1->mvKeysUn[idx1]
                                                   : (idx1 < pKF1->NLeft)
                                                          ? pKF1->mvKeys[idx1]
                                                          : pKF1->mvKeysRight[idx1 - pKF1->NLeft];
                const bool bRight1 =
                    (pKF1->NLeft == -1 || idx1 < pKF1->NLeft) ? false : true;
                const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

                int bestDist = TH_LOW;
                int bestIdx2 = -1;

                for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
                    size_t idx2 = f2it->second[i2];

                    MapPoint *pMP2 = pKF2->GetMapPoint(idx2);
                    if (vbMatched2[idx2] || pMP2) continue;

                    const bool bStereo2 = (!pKF2->mpCamera2 && pKF2->mvuRight[idx2] >= 0);
                    if (bOnlyStereo)
                        if (!bStereo2) continue;

                    const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);
                    const int dist = DescriptorDistance(d1, d2);

                    if (dist > TH_LOW || dist > bestDist) continue;

                    const cv::KeyPoint &kp2 = (pKF2->NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                       : (idx2 < pKF2->NLeft)
                                                                ? pKF2->mvKeys[idx2]
                                                                : pKF2->mvKeysRight[idx2 - pKF2->NLeft];
                    const bool bRight2 =
                        (pKF2->NLeft == -1 || idx2 < pKF2->NLeft) ? false : true;

                    // 极线距离粗筛
                    if (!bStereo1 && !bStereo2 && !pKF1->mpCamera2) {
                        const float distex = ep(0) - kp2.pt.x;
                        const float distey = ep(1) - kp2.pt.y;
                        if (distex * distex + distey * distey <
                            100 * pKF2->mvScaleFactors[kp2.octave]) {
                            continue;
                        }
                    }

                    // 双目模式选择对应左右目变换
                    if (pKF1->mpCamera2 && pKF2->mpCamera2) {
                        if (bRight1 && bRight2) {
                            R12 = Rrr;
                            t12 = trr;
                            T12 = Trr;

                            pCamera1 = pKF1->mpCamera2;
                            pCamera2 = pKF2->mpCamera2;
                        } else if (bRight1 && !bRight2) {
                            R12 = Rrl;
                            t12 = trl;
                            T12 = Trl;

                            pCamera1 = pKF1->mpCamera2;
                            pCamera2 = pKF2->mpCamera;
                        } else if (!bRight1 && bRight2) {
                            R12 = Rlr;
                            t12 = tlr;
                            T12 = Tlr;

                            pCamera1 = pKF1->mpCamera;
                            pCamera2 = pKF2->mpCamera2;
                        } else {
                            R12 = Rll;
                            t12 = tll;
                            T12 = Tll;

                            pCamera1 = pKF1->mpCamera;
                            pCamera2 = pKF2->mpCamera;
                        }
                    }

                    // 对极几何约束校验
                    if (bCoarse ||
                        pCamera1->epipolarConstrain(pCamera2, kp1, kp2, R12, t12,
                                                   pKF1->mvLevelSigma2[kp1.octave],
                                                   pKF2->mvLevelSigma2[kp2.octave])) {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }

                if (bestIdx2 >= 0) {
                    const cv::KeyPoint &kp2 =
                        (pKF2->NLeft == -1) ? pKF2->mvKeysUn[bestIdx2]
                        : (bestIdx2 < pKF2->NLeft)
                                              ? pKF2->mvKeys[bestIdx2]
                                                                      : pKF2->mvKeysRight[bestIdx2 - pKF2->NLeft];

                    vMatches12[idx1] = bestIdx2;
                    nmatches++;

                    if (mbCheckOrientation) {
                        float rot = kp1.angle - kp2.angle;
                        if (rot < 0.0) rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH) bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }

            f1it++;
            f2it++;
        } else if (f1it->first < f2it->first) {
            f1it = vFeatVec1.lower_bound(f2it->first);
        } else {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    // 方向一致性校验
    if (mbCheckOrientation) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++) {
            if (i == ind1 || i == ind2 || i == ind3) continue;
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                vMatches12[rotHist[i][j]] = -1;
                nmatches--;
            }
        }
    }

    // 整理输出匹配对
    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);
    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
        if (vMatches12[i] < 0) continue;
        vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
    }

    return nmatches;
}

// ==================== 地图点融合（左/右目） ====================
/**
 * @brief 将地图点融合到关键帧中（数据关联）
 * @param pKF 关键帧
 * @param vpMapPoints 待融合地图点
 * @param th 匹配阈值
 * @param bRight 是否右目
 * @return 成功融合数量
 */
int ORBmatcher::Fuse(const std::shared_ptr<KeyFrame> &pKF,
                      const vector<MapPoint *> &vpMapPoints, const float th,
                      const bool bRight) {
    std::shared_ptr<GeometricCamera> pCamera;
    Sophus::SE3f Tcw;
    Eigen::Vector3f Ow;

    // 选择左目或右目位姿
    if (bRight) {
        Tcw = pKF->GetRightPose();
        Ow = pKF->GetRightCameraCenter();
        pCamera = pKF->mpCamera2;
    } else {
        Tcw = pKF->GetPose();
        Ow = pKF->GetCameraCenter();
        pCamera = pKF->mpCamera;
    }

    const float &bf = pKF->mbf;  // 基线

    int nFused = 0;
    const int nMPs = vpMapPoints.size();

    // 调试计数
    int count_notMP = 0, count_bad = 0, count_isinKF = 0, count_negdepth = 0,
        count_notinim = 0, count_dist = 0, count_normal = 0, count_notidx = 0,
        count_thcheck = 0;

    for (int i = 0; i < nMPs; i++) {
        MapPoint *pMP = vpMapPoints[i];

        if (!pMP) {
            count_notMP++;
            continue;
        }

        if (pMP->isBad()) {
            count_bad++;
            continue;
        } else if (pMP->IsInKeyFrame(pKF)) {
            count_isinKF++;
            continue;
        }

        Eigen::Vector3f p3Dw = pMP->GetWorldPos();
        Eigen::Vector3f p3Dc = Tcw * p3Dw;

        if (p3Dc(2) < 0.0f) {
            count_negdepth++;
            continue;
        }

        const float invz = 1 / p3Dc(2);
        const Eigen::Vector2f uv = pCamera->project(p3Dc);

        if (!pKF->IsInImage(uv(0), uv(1))) {
            count_notinim++;
            continue;
        }

        const float ur = uv(0) - bf * invz;  // 右目基线校正

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3f PO = p3Dw - Ow;
        const float dist3D = PO.norm();

        if (dist3D < minDistance || dist3D > maxDistance) {
            count_dist++;
            continue;
        }

        // 视角法线夹角小于60度
        Eigen::Vector3f Pn = pMP->GetNormal();
        if (PO.dot(Pn) < 0.5 * dist3D) {
            count_normal++;
            continue;
        }

        int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

        const float radius = th * pKF->mvScaleFactors[nPredictedLevel];
        const vector<size_t> vIndices =
            pKF->GetFeaturesInArea(uv(0), uv(1), radius, bRight);

        if (vIndices.empty()) {
            count_notidx++;
            continue;
        }

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;

        // 找最佳匹配
        for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                             vend = vIndices.end();
             vit != vend; vit++) {
            size_t idx = *vit;

            const cv::KeyPoint &kp = (pKF->NLeft == -1) ? pKF->mvKeysUn[idx]
                                            : (!bRight)
                                   ? pKF->mvKeys[idx]
                                                           : pKF->mvKeysRight[idx];
            const int &kpLevel = kp.octave;

            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

            // 双目极线误差校验
            if (pKF->mvuRight[idx] >= 0) {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float &kpr = pKF->mvuRight[idx];
                const float ex = uv(0) - kpx;
                const float ey = uv(1) - kpy;
                const float er = ur - kpr;
                const float e2 = ex * ex + ey * ey + er * er;

                if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 7.8) continue;
            } else {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = uv(0) - kpx;
                const float ey = uv(1) - kpy;
                const float e2 = ex * ex + ey * ey;

                if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 5.99) continue;
            }

            if (bRight) idx += pKF->NLeft;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);
            const int dist = DescriptorDistance(dMP, dKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // 匹配成功，融合地图点
        if (bestDist <= TH_LOW) {
            MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);

            if (pMPinKF) {
                if (!pMPinKF->isBad()) {
                    if (pMPinKF->Observations() > pMP->Observations())
                        pMP->Replace(pMPinKF);
                    else
                        pMPinKF->Replace(pMP);
                }
            } else {
                pMP->AddObservation(pKF, bestIdx);
                pKF->AddMapPoint(pMP, bestIdx);
            }

            nFused++;
        } else {
            count_thcheck++;
        }
    }

    return nFused;
}

// ==================== 地图点融合（Sim3变换版本） ====================
/**
 * @brief Sim3变换下的地图点融合
 * @param pKF 关键帧
 * @param Scw Sim3变换
 * @param vpPoints 待融合地图点
 * @param th 阈值
 * @param vpReplacePoint 输出被替换的地图点
 * @return 融合数量
 */
int ORBmatcher::Fuse(const std::shared_ptr<KeyFrame> &pKF, Sophus::Sim3f &Scw,
                      const vector<MapPoint *> &vpPoints, float th,
                      vector<MapPoint *> &vpReplacePoint) {
    // 分解Sim3
    Sophus::SE3f Tcw =
        Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();

    int nFused = 0;
    const int nPoints = vpPoints.size();

    for (int iMP = 0; iMP < nPoints; iMP++) {
        MapPoint *pMP = vpPoints[iMP];

        if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

        Eigen::Vector3f p3Dw = pMP->GetWorldPos();
        Eigen::Vector3f p3Dc = Tcw * p3Dw;

        if (p3Dc(2) < 0.0f) continue;

        const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

        if (!pKF->IsInImage(uv(0), uv(1))) continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3f PO = p3Dw - Ow;
        const float dist3D = PO.norm();

        if (dist3D < minDistance || dist3D > maxDistance) continue;

        Eigen::Vector3f Pn = pMP->GetNormal();
        if (PO.dot(Pn) < 0.5 * dist3D) continue;

        int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

        const float radius = th * pKF->mvScaleFactors[nPredictedLevel];
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0), uv(1), radius);

        if (vIndices.empty()) continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;

        for (auto const &idx : vIndices) {
            const int &kpLevel = pKF->mvKeysUn[idx].octave;
            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);
            int dist = DescriptorDistance(dMP, dKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if (bestDist <= TH_LOW) {
            MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);

            if (pMPinKF) {
                if (!pMPinKF->isBad()) vpReplacePoint[iMP] = pMPinKF;
            } else {
                pMP->AddObservation(pKF, bestIdx);
                pKF->AddMapPoint(pMP, bestIdx);
            }

            nFused++;
        }
    }

    return nFused;
}

// ==================== Sim3运动下搜索匹配 ====================
/**
 * @brief 通过Sim3相似变换搜索两帧匹配，用于回环检测
 * @param pKF1 关键帧1
 * @param pKF2 关键帧2
 * @param vpMatches12 输出匹配
 * @param S12 两帧间Sim3变换
 * @param th 搜索阈值
 * @return 匹配数量
 */
int ORBmatcher::SearchBySim3(const std::shared_ptr<KeyFrame> &pKF1,
                              const std::shared_ptr<KeyFrame> &pKF2,
                              std::vector<MapPoint *> &vpMatches12,
                              const Sophus::Sim3f &S12, const float th) {
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    Sophus::SE3f T1w = pKF1->GetPose();
    Sophus::SE3f T2w = pKF2->GetPose();

    Sophus::Sim3f S21 = S12.inverse();

    const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();
    const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1, false);
    vector<bool> vbAlreadyMatched2(N2, false);

    // 标记已匹配的点
    for (int i = 0; i < N1; i++) {
        MapPoint *pMP = vpMatches12[i];
        if (pMP) {
            vbAlreadyMatched1[i] = true;
            int idx2 = get<0>(pMP->GetIndexInKeyFrame(pKF2));
            if (idx2 >= 0 && idx2 < N2) vbAlreadyMatched2[idx2] = true;
        }
    }

    vector<int> vnMatch1(N1, -1);
    vector<int> vnMatch2(N2, -1);

    // KF1地图点投影到KF2搜索匹配
    for (int i1 = 0; i1 < N1; i1++) {
        MapPoint *pMP = vpMapPoints1[i1];

        if (!pMP || vbAlreadyMatched1[i1]) continue;
        if (pMP->isBad()) continue;

        Eigen::Vector3f p3Dw = pMP->GetWorldPos();
        Eigen::Vector3f p3Dc1 = T1w * p3Dw;
        Eigen::Vector3f p3Dc2 = S21 * p3Dc1;

        if (p3Dc2(2) < 0.0) continue;

        const float invz = 1.0 / p3Dc2(2);
        const float x = p3Dc2(0) * invz;
        const float y = p3Dc2(1) * invz;
        const float u = fx * x + cx;
        const float v = fy * y + cy;

        if (!pKF2->IsInImage(u, v)) continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = p3Dc2.norm();

        if (dist3D < minDistance || dist3D > maxDistance) continue;

        int nPredictedLevel = pMP->PredictScale(dist3D, pKF2);

        const float radius = th * pKF2->mvScaleFactors[nPredictedLevel];
        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u, v, radius);

        if (vIndices.empty()) continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;

        for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                             vend = vIndices.end();
             vit != vend; vit++) {
            const size_t idx = *vit;
            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];
            if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF2->mDescriptors.row(idx);
            const int dist = DescriptorDistance(dMP, dKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if (bestDist <= TH_HIGH) {
            vnMatch1[i1] = bestIdx;
        }
    }

    // KF2地图点投影到KF1搜索匹配（双向一致性）
    for (int i2 = 0; i2 < N2; i2++) {
        MapPoint *pMP = vpMapPoints2[i2];

        if (!pMP || vbAlreadyMatched2[i2]) continue;
        if (pMP->isBad()) continue;

        Eigen::Vector3f p3Dw = pMP->GetWorldPos();
        Eigen::Vector3f p3Dc2 = T2w * p3Dw;
        Eigen::Vector3f p3Dc1 = S12 * p3Dc2;

        if (p3Dc1(2) < 0.0) continue;

        const float invz = 1.0 / p3Dc1(2);
        const float x = p3Dc1(0) * invz;
        const float y = p3Dc1(1) * invz;
        const float u = fx * x + cx;
        const float v = fy * y + cy;

        if (!pKF1->IsInImage(u, v)) continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = p3Dc1.norm();

        if (dist3D < minDistance || dist3D > maxDistance) continue;

        int nPredictedLevel = pMP->PredictScale(dist3D, pKF1);

        const float radius = th * pKF1->mvScaleFactors[nPredictedLevel];
        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u, v, radius);

        if (vIndices.empty()) continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;

        for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                             vend = vIndices.end();
             vit != vend; vit++) {
            const size_t idx = *vit;
            const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];
            if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF1->mDescriptors.row(idx);
            const int dist = DescriptorDistance(dMP, dKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if (bestDist <= TH_HIGH) {
            vnMatch2[i2] = bestIdx;
        }
    }

    // 双向一致性校验，统计成功匹配
    int nFound = 0;
    for (int i1 = 0; i1 < N1; i1++) {
        int idx2 = vnMatch1[i1];

        if (idx2 >= 0) {
            int idx1 = vnMatch2[idx2];
            if (idx1 == i1) {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

// ==================== 帧间投影跟踪 ====================
/**
 * @brief 相邻帧之间通过投影进行特征点跟踪匹配
 * @param CurrentFrame 当前帧
 * @param LastFrame 上一帧
 * @param th 搜索阈值
 * @param bMono 是否单目模式
 * @return 匹配数量
 */
int ORBmatcher::SearchByProjection(const std::shared_ptr<Frame> &CurrentFrame,
                                    const std::shared_ptr<Frame> &LastFrame,
                                    const float th, const bool bMono) {
    int nmatches = 0;

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    const Sophus::SE3f Tcw = CurrentFrame->GetPose();
    const Eigen::Vector3f twc = Tcw.inverse().translation();
    const Sophus::SE3f Tlw = LastFrame->GetPose();
    const Eigen::Vector3f tlc = Tlw * twc;

    const bool bForward = tlc(2) > CurrentFrame->mb && !bMono;   // 向前运动
    const bool bBackward = -tlc(2) > CurrentFrame->mb && !bMono;  // 向后运动

    // 遍历上一帧所有地图点
    for (size_t i = 0; i < LastFrame->N; i++) {
        MapPoint *pMP = LastFrame->mvpMapPoints[i];

        if (pMP) {
            if (!LastFrame->mvbOutlier[i]) {
                Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                Eigen::Vector3f x3Dc = Tcw * x3Dw;

                const float invzc = 1.0 / x3Dc(2);
                if (invzc < 0) continue;

                Eigen::Vector2f uv = CurrentFrame->mpCamera->project(x3Dc);

                if (uv(0) < CurrentFrame->mnMinX || uv(0) > CurrentFrame->mnMaxX)
                    continue;
                if (uv(1) < CurrentFrame->mnMinY || uv(1) > CurrentFrame->mnMaxY)
                    continue;

                int nLastOctave =
                    (LastFrame->Nleft == -1 || i < LastFrame->Nleft)
                        ? LastFrame->mvKeys[i].octave
                        : LastFrame->mvKeysRight[i - LastFrame->Nleft].octave;

                float radius = th * CurrentFrame->mvScaleFactors[nLastOctave];

                vector<size_t> vIndices2;

                if (bForward)
                    vIndices2 = CurrentFrame->GetFeaturesInArea(uv(0), uv(1), radius,
                                                           nLastOctave);
                else if (bBackward)
                    vIndices2 = CurrentFrame->GetFeaturesInArea(uv(0), uv(1), radius, 0,
                                                           nLastOctave);
                else
                    vIndices2 = CurrentFrame->GetFeaturesInArea(
                        uv(0), uv(1), radius, nLastOctave - 1, nLastOctave + 1);

                if (vIndices2.empty()) continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for (vector<size_t>::const_iterator vit = vIndices2.begin(),
                                                     vend = vIndices2.end();
                     vit != vend; vit++) {
                    const size_t i2 = *vit;

                    if (CurrentFrame->mvpMapPoints[i2])
                        if (CurrentFrame->mvpMapPoints[i2]->Observations() > 0) continue;

                    // 极线校正
                    if (CurrentFrame->Nleft == -1 && CurrentFrame->mvuRight[i2] > 0) {
                        const float ur = uv(0) - CurrentFrame->mbf * invzc;
                        const float er = fabs(ur - CurrentFrame->mvuRight[i2]);
                        if (er > radius) continue;
                    }

                    const cv::Mat &d = CurrentFrame->mDescriptors.row(i2);
                    const int dist = DescriptorDistance(dMP, d);

                    if (dist < bestDist) {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                if (bestDist <= TH_HIGH) {
                    CurrentFrame->mvpMapPoints[bestIdx2] = pMP;
                    nmatches++;

                    if (mbCheckOrientation) {
                        cv::KeyPoint kpLF =
                            (LastFrame->Nleft == -1) ? LastFrame->mvKeysUn[i]
                                : (i < LastFrame->Nleft)
                                      ? LastFrame->mvKeys[i]
                                      : LastFrame->mvKeysRight[i - LastFrame->Nleft];

                        cv::KeyPoint kpCF =
                            (CurrentFrame->Nleft == -1) ? CurrentFrame->mvKeysUn[bestIdx2]
                                : (bestIdx2 < CurrentFrame->Nleft)
                                      ? CurrentFrame->mvKeys[bestIdx2]
                                      : CurrentFrame->mvKeysRight[bestIdx2 - CurrentFrame->Nleft];

                        float rot = kpLF.angle - kpCF.angle;
                        if (rot < 0.0) rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH) bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

                // 右目匹配（双目）
                if (CurrentFrame->Nleft != -1) {
                    Eigen::Vector3f x3Dr = CurrentFrame->GetRelativePoseTrl() * x3Dc;
                    Eigen::Vector2f uv = CurrentFrame->mpCamera->project(x3Dr);

                    int nLastOctave =
                        (LastFrame->Nleft == -1 || i < LastFrame->Nleft)
                            ? LastFrame->mvKeys[i].octave
                            : LastFrame->mvKeysRight[i - LastFrame->Nleft].octave;

                    float radius = th * CurrentFrame->mvScaleFactors[nLastOctave];

                    vector<size_t> vIndices2;
                    if (bForward)
                        vIndices2 = CurrentFrame->GetFeaturesInArea(uv(0), uv(1), radius,
                                                               nLastOctave, -1, true);
                    else if (bBackward)
                        vIndices2 = CurrentFrame->GetFeaturesInArea(uv(0), uv(1), radius, 0,
                                                               nLastOctave, true);
                    else
                        vIndices2 = CurrentFrame->GetFeaturesInArea(
                            uv(0), uv(1), radius, nLastOctave - 1, nLastOctave + 1, true);

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    for (vector<size_t>::const_iterator vit = vIndices2.begin(),
                                                         vend = vIndices2.end();
                         vit != vend; vit++) {
                        const size_t i2 = *vit;

                        if (CurrentFrame->mvpMapPoints[i2 + CurrentFrame->Nleft])
                            if (CurrentFrame->mvpMapPoints[i2 + CurrentFrame->Nleft]
                                    ->Observations() > 0)
                                continue;

                        const cv::Mat &d =
                            CurrentFrame->mDescriptors.row(i2 + CurrentFrame->Nleft);

                        const int dist = DescriptorDistance(dMP, d);

                        if (dist < bestDist) {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    if (bestDist <= TH_HIGH) {
                        CurrentFrame->mvpMapPoints[bestIdx2 + CurrentFrame->Nleft] = pMP;
                        nmatches++;

                        if (mbCheckOrientation) {
                            cv::KeyPoint kpLF =
                                (LastFrame->Nleft == -1) ? LastFrame->mvKeysUn[i]
                                    : (i < LastFrame->Nleft)
                                          ? LastFrame->mvKeys[i]
                                          : LastFrame->mvKeysRight[i - LastFrame->Nleft];

                            cv::KeyPoint kpCF = CurrentFrame->mvKeysRight[bestIdx2];

                            float rot = kpLF.angle - kpCF.angle;
                            if (rot < 0.0) rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH) bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2 + CurrentFrame->Nleft);
                        }
                    }
                }
            }
        }
    }

    // 方向一致性校验
    if (mbCheckOrientation) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++) {
            if (i != ind1 && i != ind2 && i != ind3) {
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    CurrentFrame->mvpMapPoints[rotHist[i][j]] =
                        static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

// ==================== 重定位投影匹配 ====================
/**
 * @brief 重定位模式下投影匹配，将地图点匹配到当前帧
 * @param CurrentFrame 当前帧
 * @param pKF 参考关键帧
 * @param sAlreadyFound 已找到地图点集合
 * @param th 搜索阈值
 * @param ORBdist ORB距离阈值
 * @return 匹配数量
 */
int ORBmatcher::SearchByProjection(const std::shared_ptr<Frame> &CurrentFrame,
                                   const std::shared_ptr<KeyFrame> &pKF,
                                   const set<MapPoint *> &sAlreadyFound,
                                   const float th, const int ORBdist) {
    int nmatches = 0; // 统计成功匹配的地图点数量

    // 获取当前普通帧的位姿 Tcw：世界坐标系 -> 当前帧相机坐标系
    const Sophus::SE3f Tcw = CurrentFrame->GetPose();
    // Tcw.inverse() 是相机到世界变换，translation()拿到相机光心在世界坐标系下的坐标Ow
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // 旋转角度直方图，用于做旋转一致性校验，过滤外点匹配
    vector<int> rotHist[HISTO_LENGTH];
    // 对每一个直方图桶预分配500容量，避免频繁vector扩容
    for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
    // 系数：将0~360度角度映射到直方图下标[0,HISTO_LENGTH)
    const float factor = 1.0f / HISTO_LENGTH;

    // 获取关键帧关联的所有地图点，部分元素可为nullptr
    const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

    // 遍历关键帧每一个特征点对应的地图点
    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        MapPoint *pMP = vpMPs[i];

        // 如果该地图点指针有效
        if (pMP) {
            // 地图点不是坏点，并且该地图点还没有被匹配过
            if (!pMP->isBad() && !sAlreadyFound.count(pMP)) {
                // 获取地图点世界坐标
                Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                // 世界点变换到当前帧相机坐标系：Xc = Tcw * Xw
                Eigen::Vector3f x3Dc = Tcw * x3Dw;

                // 将相机坐标系3D点投影到当前帧图像平面，得到像素uv坐标
                const Eigen::Vector2f uv = CurrentFrame->mpCamera->project(x3Dc);

                // 投影像素超出图像有效边界，跳过该地图点
                if (uv(0) < CurrentFrame->mnMinX || uv(0) > CurrentFrame->mnMaxX)
                    continue;
                if (uv(1) < CurrentFrame->mnMinY || uv(1) > CurrentFrame->mnMaxY)
                    continue;

                // 计算地图点到当前相机光心的向量 PO = Pw - Ow
                Eigen::Vector3f PO = x3Dw - Ow;
                // 3D空间距离：地图点到相机光心距离
                float dist3D = PO.norm();

                // 获取该地图点观测的最大、最小有效观测距离（不变量，由尺度金字塔推导）
                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // 3D点距离不在地图点可观测深度范围内，跳过，避免尺度层级不匹配
                if (dist3D < minDistance || dist3D > maxDistance) continue;

                // 根据3D距离预测该地图点应当落在图像金字塔哪一层
                int nPredictedLevel = pMP->PredictScale(dist3D, CurrentFrame);

                // 搜索窗口半径：阈值th乘上预测金字塔层级对应的尺度因子
                const float radius = th * CurrentFrame->mvScaleFactors[nPredictedLevel];

                // 在当前帧上，以uv为中心，radius为半径，在[nPredictedLevel‑1, nPredictedLevel+1]金字塔层级内取出所有特征点索引
                const vector<size_t> vIndices2 = CurrentFrame->GetFeaturesInArea(
                    uv(0), uv(1), radius, nPredictedLevel - 1, nPredictedLevel + 1);

                // 该区域没有任何特征点，直接跳过
                if (vIndices2.empty()) continue;

                // 获取该地图点的ORB描述子
                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256; // ORB描述子最大汉明距离255，初始设为256代表无匹配
                int bestIdx2 = -1;  // 保存当前帧最优匹配特征点索引

                // 遍历搜索区域内所有候选特征点
                for (vector<size_t>::const_iterator vit = vIndices2.begin();
                     vit != vIndices2.end(); vit++) {
                    const size_t i2 = *vit;
                    // 如果该特征点已经关联了别的地图点，跳过，不做重复匹配
                    if (CurrentFrame->mvpMapPoints[i2]) continue;

                    // 当前帧该候选点的ORB描述子
                    const cv::Mat &d = CurrentFrame->mDescriptors.row(i2);
                    // 计算两个ORB描述子之间汉明距离
                    const int dist = DescriptorDistance(dMP, d);
                    // 更新最优匹配：距离更小则更新
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                // 最优描述子距离小于等于设定阈值，判定匹配成功
                if (bestDist <= ORBdist) {
                    // 将当前帧特征点bestIdx2和地图点pMP做关联
                    CurrentFrame->mvpMapPoints[bestIdx2] = pMP;
                    nmatches++; // 匹配计数+1

                    // 如果开启旋转一致性检查逻辑
                    if (mbCheckOrientation) {
                        // 计算角度差：关键帧特征点角度 - 当前帧匹配特征点角度
                        float rot =
                            pKF->mvKeysUn[i].angle - CurrentFrame->mvKeysUn[bestIdx2].angle;
                        // 角度为负，加360°归一化到0~360范围
                        if (rot < 0.0) rot += 360.0f;
                        // 映射到直方图桶索引
                        int bin = round(rot * factor);
                        // 防止等于HISTO_LENGTH越界，回卷到0号桶
                        if (bin == HISTO_LENGTH) bin = 0;
                        // 断言保证桶下标合法
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        // 将匹配点索引放入对应角度直方图桶
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }

    // 开启旋转一致性校验，根据角度直方图剔除角度不一致的错误匹配
    if (mbCheckOrientation) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        // 计算直方图中数量最多的前三个桶下标 ind1/ind2/ind3
        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        // 遍历全部直方图桶
        for (int i = 0; i < HISTO_LENGTH; i++) {
            // 不属于前三最大计数桶，认为是角度异常，剔除这些匹配
            if (i != ind1 && i != ind2 && i != ind3) {
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    // 清空当前帧该特征点的地图点关联
                    CurrentFrame->mvpMapPoints[rotHist[i][j]] = NULL;
                    nmatches--; // 有效匹配计数减一
                }
            }
        }
    }

    return nmatches; // 返回经过旋转过滤之后最终有效匹配对数
}

// 从直方图中找出计数最大的前3个桶索引，支持阈值过滤次要峰值
void ORBmatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1,
                                    int &ind2, int &ind3) {
    int max1 = 0; // 第一大桶样本数量
    int max2 = 0; // 第二大桶样本数量
    int max3 = 0; // 第三大桶样本数量

    // 遍历全部直方图桶
    for (int i = 0; i < L; i++) {
        const int s = histo[i].size(); // 当前桶内样本个数
        // 当前数量大于第一大，三个最大值整体向后滚动更新
        if (s > max1) {
            max3 = max2;
            max2 = max1;
            max1 = s;
            ind3 = ind2;
            ind2 = ind1;
            ind1 = i;
        } else if (s > max2) { // 大于第二大，更新第二、第三
            max3 = max2;
            max2 = s;
            ind3 = ind2;
            ind2 = i;
        } else if (s > max3) { // 大于第三大，更新第三
            max3 = s;
            ind3 = i;
        }
    }

    // 如果第二大峰值不足第一大的10%，认为没有第二有效峰值，置-1无效
    if (max2 < 0.1f * static_cast<float>(max1)) {
        ind2 = -1;
        ind3 = -1;
    } else if (max3 < 0.1f * static_cast<float>(max1)) {
        // 第三大峰值不足第一大10%，置第三索引无效
        ind3 = -1;
    }
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
// 计算两个ORB二进制描述子之间汉明距离，统计异或后为1的bit个数
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
    // 把描述子内存视作32位int指针，ORB描述子256bit，一共8个int32_t
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist = 0; // 汉明距离累加

    // 循环8次，每次处理32bit，总共256bit完整描述子
    for (int i = 0; i < 8; i++, pa++, pb++) {
        unsigned int v = *pa ^ *pb; // 两个32bit字异或，bit=1代表该位不同
        // 并行计算每个2bit分组内1的个数
        v = v - ((v >> 1) & 0x55555555);
        // 合并相邻2bit结果，得到4bit分组内1的计数
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        // 继续归并，把4bit计数累加，取出高8bit，加到总汉明距离dist
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

}  // namespace ORB_SLAM3
