/**
 * This file is part of ORB‑SLAM3
 *
 * Copyright (C) 2017‑2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014‑2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB‑SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB‑SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB‑SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once                          // 头文件保护，避免重复包含
#include <list>                       // std::list双向链表，用于八叉树节点管理
#include <opencv2/opencv.hpp>         // OpenCV主头文件，图像、关键点、Mat等
#include <vector>                     // std::vector动态数组容器

namespace ORB_SLAM3 {

/**
 * @brief ORB特征提取八叉树节点，用于特征点均匀化分布
 * @details 每个节点代表图像上一块矩形区域，保存该区域内关键点，可分裂为4个子节点
 */
class ExtractorNode {
 public:
  /**
   * @brief 默认构造函数，bNoMore标记初始置false，表示还可以继续分裂
   */
  ExtractorNode() : bNoMore(false) {}

  /**
   * @brief 将当前节点四等分分裂成4个子节点n1 n2 n3 n4
   * @param n1 左上子节点
   * @param n2 右上子节点
   * @param n3 左下子节点
   * @param n4 右下子节点
   */
  void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3,
                   ExtractorNode &n4);

  std::vector<cv::KeyPoint> vKeys; ///< 当前节点区域内存储的关键点集合
  cv::Point2i UL, UR, BL, BR;      ///< 矩形区域四个角点坐标：左上、右上、左下、右下
  std::list<ExtractorNode>::iterator lit; ///< 当前节点在list链表中的迭代器
  bool bNoMore;                    ///< true表示该节点不能再继续分裂；false可继续分裂
};

/**
 * @brief ORB特征提取器类，负责构建图像金字塔、FAST角点检测、八叉树均匀化、BRIEF描述子计算
 */
class ORBextractor {
 public:
  /// 角点评分模式枚举
  enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

  /**
   * @brief ORB提取器构造函数
   * @param nfeatures 希望提取的总特征点数量
   * @param scaleFactor 金字塔缩放因子，相邻层尺度比例
   * @param nlevels 金字塔总层数
   * @param iniThFAST FAST角点初始阈值
   * @param minThFAST FAST角点最小阈值，初始阈值检测点过少时降级使用
   */
  ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST,
                int minThFAST);

  /**
   * @brief 析构函数，使用默认
   */
  ~ORBextractor() {}

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  /**
   * @brief 重载()运算符，对外核心接口，输入图像输出关键点与描述子
   * @param _image 输入灰度图像
   * @param _mask 掩码，当前版本未实际生效
   * @param _keypoints 输出检测到的ORB关键点
   * @param _descriptors 输出BRIEF描述子，cv::Mat
   * @param vLappingArea 输出重叠区域信息
   * @return int 返回提取成功的特征点数量
   */
  int operator()(cv::InputArray _image, cv::InputArray _mask,
                  std::vector<cv::KeyPoint> &_keypoints,
                  cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

  /**
   * @brief 获取金字塔层数
   * @return 金字塔层数nlevels
   */
  int inline GetLevels() { return nlevels; }

  /**
   * @brief 获取金字塔缩放系数scaleFactor
   * @return scaleFactor
   */
  float inline GetScaleFactor() { return scaleFactor; }

  /**
   * @brief 获取每层相对于第0层的尺度因子数组
   * @return mvScaleFactor向量
   */
  std::vector<float> inline GetScaleFactors() { return mvScaleFactor; }

  /**
   * @brief 获取每层尺度因子的倒数数组
   * @return mvInvScaleFactor向量
   */
  std::vector<float> inline GetInverseScaleFactors() {
    return mvInvScaleFactor;
  }

  /**
   * @brief 获取每层尺度因子的平方数组
   * @return mvLevelSigma2向量
   */
  std::vector<float> inline GetScaleSigmaSquares() { return mvLevelSigma2; }

  /**
   * @brief 获取每层尺度因子平方的倒数数组
   * @return mvInvLevelSigma2向量
   */
  std::vector<float> inline GetInverseScaleSigmaSquares() {
    return mvInvLevelSigma2;
  }

  std::vector<cv::Mat> mvImagePyramid; ///< 图像金字塔，存储每一层缩放后的图像

 protected:
  /**
   * @brief 根据输入图像构建完整图像金字塔
   * @param image 原始输入灰度图
   */
  void ComputePyramid(cv::Mat image);

  /**
   * @brief 使用八叉树算法在金字塔每一层提取均匀分布的FAST关键点
   * @param allKeypoints 输出，外层是金字塔层级，内层为该层关键点集合
   */
  void ComputeKeyPointsOctTree(
      std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

  /**
   * @brief 八叉树分布函数，对传入关键点做空间均匀采样，返回该层筛选后的关键点
   * @param vToDistributeKeys 待分配的原始关键点
   * @param minX 图像区域x最小边界
   * @param maxX 图像区域x最大边界
   * @param minY 图像区域y最小边界
   * @param maxY 图像区域y最大边界
   * @param nFeatures 该层希望保留的特征点数目
   * @param level 当前金字塔层级
   * @return 均匀化筛选之后的关键点vector
   */
  std::vector<cv::KeyPoint> DistributeOctTree(
      const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
      const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
      const int &level);

  /**
   * @brief 旧版关键点提取方式，不使用八叉树，特征分布容易聚集，作为备用
   * @param allKeypoints 输出各层关键点集合
   */
  void ComputeKeyPointsOld(
      std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

  std::vector<cv::Point> pattern; ///< BRIEF描述子采样点对模板，预定义好的256对点坐标

  int nfeatures;                  ///< 整个金字塔希望提取的总特征点数量
  double scaleFactor;             ///< 金字塔相邻层缩放比例
  int nlevels;                    ///< 金字塔总层数
  int iniThFAST;                  ///< FAST角点检测初始阈值
  int minThFAST;                  ///< FAST角点检测最小降级阈值

  std::vector<int> mnFeaturesPerLevel; ///< 金字塔每一层期望分配的特征点数量

  std::vector<int> umax;          ///< 用于FAST角点计算旋转方向的查表数组

  std::vector<float> mvScaleFactor;    ///< 每一层相对于第0层的尺度因子
  std::vector<float> mvInvScaleFactor; ///< 每一层尺度因子的倒数
  std::vector<float> mvLevelSigma2;    ///< 每一层尺度因子的平方
  std::vector<float> mvInvLevelSigma2; ///< 每一层尺度因子平方的倒数
};

}  // namespace ORB_SLAM3
