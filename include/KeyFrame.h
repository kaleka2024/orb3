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

#pragma once

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "Frame.h"
#include "GeometricCamera.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "SerializationUtils.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

namespace ORB_SLAM3 {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class GeometricCamera;

class KeyFrame : public std::enable_shared_from_this<KeyFrame> {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & mnId;
    ar& const_cast<unsigned long int&>(mnFrameId);
    ar& const_cast<double&>(mTimeStamp);
    // Grid
    ar& const_cast<int&>(mnGridCols);
    ar& const_cast<int&>(mnGridRows);
    ar& const_cast<float&>(mfGridElementWidthInv);
    ar& const_cast<float&>(mfGridElementHeightInv);

    // Variables of tracking
    // ar & mnTrackReferenceForFrame;
    // ar & mnFuseTargetForKF;
    // Variables of local mapping
    // ar & mnBALocalForKF;
    // ar & mnBAFixedForKF;
    // ar & mnNumberOfOpt;
    // Variables used by KeyFrameDatabase
    // ar & mnLoopQuery;
    // ar & mnLoopWords;
    // ar & mLoopScore;
    // ar & mnRelocQuery;
    // ar & mnRelocWords;
    // ar & mRelocScore;
    // ar & mnMergeQuery;
    // ar & mnMergeWords;
    // ar & mMergeScore;
    // ar & mnPlaceRecognitionQuery;
    // ar & mnPlaceRecognitionWords;
    // ar & mPlaceRecognitionScore;
    // ar & mbCurrentPlaceRecognition;
    // Variables of loop closing
    // serializeMatrix(ar,mTcwGBA,version);
    // serializeMatrix(ar,mTcwBefGBA,version);
    // serializeMatrix(ar,mVwbGBA,version);
    // serializeMatrix(ar,mVwbBefGBA,version);
    // ar & mBiasGBA;
    // ar & mnBAGlobalForKF;
    // Variables of Merging
    // serializeMatrix(ar,mTcwMerge,version);
    // serializeMatrix(ar,mTcwBefMerge,version);
    // serializeMatrix(ar,mTwcBefMerge,version);
    // serializeMatrix(ar,mVwbMerge,version);
    // serializeMatrix(ar,mVwbBefMerge,version);
    // ar & mBiasMerge;
    // ar & mnMergeCorrectedForKF;
    // ar & mnMergeForKF;
    // ar & mfScaleMerge;
    // ar & mnBALocalForMerge;

    // Scale
    ar & mfScale;
    // Calibration parameters
    ar& const_cast<float&>(fx);
    ar& const_cast<float&>(fy);
    ar& const_cast<float&>(invfx);
    ar& const_cast<float&>(invfy);
    ar& const_cast<float&>(cx);
    ar& const_cast<float&>(cy);
    ar& const_cast<float&>(mbf);
    ar& const_cast<float&>(mb);
    ar& const_cast<float&>(mThDepth);
    serializeMatrix(ar, mDistCoef, version);
    // Number of Keypoints
    ar& const_cast<int&>(N);
    // KeyPoints
    serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
    serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
    ar& const_cast<vector<float>&>(mvuRight);
    ar& const_cast<vector<float>&>(mvDepth);
    serializeMatrix<Archive>(ar, mDescriptors, version);
    // BOW
    ar & mBowVec;
    ar & mFeatVec;
    // Pose relative to parent
    serializeSophusSE3<Archive>(ar, mTcp, version);
    // Scale
    ar& const_cast<int&>(mnScaleLevels);
    ar& const_cast<float&>(mfScaleFactor);
    ar& const_cast<float&>(mfLogScaleFactor);
    ar& const_cast<vector<float>&>(mvScaleFactors);
    ar& const_cast<vector<float>&>(mvLevelSigma2);
    ar& const_cast<vector<float>&>(mvInvLevelSigma2);
    // Image bounds and calibration
    ar& const_cast<int&>(mnMinX);
    ar& const_cast<int&>(mnMinY);
    ar& const_cast<int&>(mnMaxX);
    ar& const_cast<int&>(mnMaxY);
    ar& boost::serialization::make_array(mK_.data(), mK_.size());
    // Pose
    serializeSophusSE3<Archive>(ar, mTcw, version);
    // MapPointsId associated to keypoints
    ar & mvBackupMapPointsId;
    // Grid
    ar & mGrid;
    // Connected KeyFrameWeight
    ar & mBackupConnectedKeyFrameIdWeights;
    // Spanning Tree and Loop Edges
    ar & mbFirstConnection;
    ar & mBackupParentId;
    ar & mvBackupChildrensId;
    ar & mvBackupLoopEdgesId;
    ar & mvBackupMergeEdgesId;
    // Bad flags
    ar & mbNotErase;
    ar & mbToBeErased;
    ar & mbBad;

    ar & mHalfBaseline;

    ar & mnOriginMapId;

    // Camera variables
    ar & mnBackupIdCamera;
    ar & mnBackupIdCamera2;

    // Fisheye variables
    ar & mvLeftToRightMatch;
    ar & mvRightToLeftMatch;
    ar& const_cast<int&>(NLeft);
    ar& const_cast<int&>(NRight);
    serializeSophusSE3<Archive>(ar, mTlr, version);
    serializeVectorKeyPoints<Archive>(ar, mvKeysRight, version);
    ar & mGridRight;

    // Inertial variables
    ar & mImuBias;
    ar & mpBackupImuPreintegrated;
    ar & mImuCalib;
    ar & mBackupPrevKFId;
    ar & mBackupNextKFId;
    ar & bImu;
    ar& boost::serialization::make_array(mVw.data(), mVw.size());
    ar& boost::serialization::make_array(mOwb.data(), mOwb.size());
    ar & mbHasVelocity;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KeyFrame() = default;
  KeyFrame(const std::shared_ptr<Frame>& F, const std::shared_ptr<Map>& pMap,
           const std::shared_ptr<KeyFrameDatabase>& pKFDB);

  // Pose functions
  void SetPose(const Sophus::SE3f& Tcw);
  void SetVelocity(const Eigen::Vector3f& Vw_);

  Sophus::SE3f GetPose();

  Sophus::SE3f GetPoseInverse();
  Eigen::Vector3f GetCameraCenter();

  Eigen::Vector3f GetImuPosition();
  Eigen::Matrix3f GetImuRotation();
  Sophus::SE3f GetImuPose();
  Eigen::Matrix3f GetRotation();
  Eigen::Vector3f GetTranslation();
  Eigen::Vector3f GetVelocity();
  bool isVelocitySet();

  // Bag of Words Representation
  void ComputeBoW();

  // Covisibility graph functions
  void AddConnection(const std::shared_ptr<KeyFrame>& pKF, const int& weight);
  void EraseConnection(const std::shared_ptr<KeyFrame>& pKF);

  void UpdateConnections(bool upParent = true);
  void UpdateBestCovisibles();
  std::set<std::shared_ptr<KeyFrame>> GetConnectedKeyFrames();
  std::vector<std::shared_ptr<KeyFrame>> GetVectorCovisibleKeyFrames();
  std::vector<std::shared_ptr<KeyFrame>> GetBestCovisibilityKeyFrames(int N);
  std::vector<std::shared_ptr<KeyFrame>> GetCovisiblesByWeight(int w);
  int GetWeight(const std::shared_ptr<KeyFrame>& pKF);

  // Spanning tree functions
  void AddChild(const std::shared_ptr<KeyFrame>& pKF);
  void EraseChild(const std::shared_ptr<KeyFrame>& pKF);
  void ChangeParent(const std::shared_ptr<KeyFrame>& pKF);
  std::set<std::shared_ptr<KeyFrame>> GetChilds();
  std::shared_ptr<KeyFrame> GetParent();
  bool hasChild(const std::shared_ptr<KeyFrame>& pKF);
  void SetFirstConnection(bool bFirst);

  // Loop Edges
  void AddLoopEdge(const std::shared_ptr<KeyFrame>& pKF);
  std::set<std::shared_ptr<KeyFrame>> GetLoopEdges();

  // Merge Edges
  void AddMergeEdge(const std::shared_ptr<KeyFrame>& pKF);
  set<std::shared_ptr<KeyFrame>> GetMergeEdges();

  // MapPoint observation functions
  int GetNumberMPs();
  void AddMapPoint(MapPoint* pMP, const size_t& idx);
  void EraseMapPointMatch(const int& idx);
  void EraseMapPointMatch(MapPoint* pMP);
  void ReplaceMapPointMatch(const int& idx, MapPoint* pMP);
  std::set<MapPoint*> GetMapPoints();
  std::vector<MapPoint*> GetMapPointMatches();
  int TrackedMapPoints(const int& minObs);
  MapPoint* GetMapPoint(const size_t& idx);

  // KeyPoint functions
  std::vector<size_t> GetFeaturesInArea(const float& x, const float& y,
                                        const float& r,
                                        const bool bRight = false) const;
  bool UnprojectStereo(int i, Eigen::Vector3f& x3D);

  // Image
  bool IsInImage(const float& x, const float& y) const;

  // Enable/Disable bad flag changes
  void SetNotErase();
  void SetErase();

  // Set/check bad flag
  void SetBadFlag();
  bool isBad();

  // Compute Scene Depth (q=2 median). Used in monocular.
  float ComputeSceneMedianDepth(const int q);

  static bool weightComp(int a, int b) { return a > b; }

  static bool lId(const std::shared_ptr<KeyFrame>& pKF1,
                  const std::shared_ptr<KeyFrame>& pKF2) {
    return pKF1->mnId < pKF2->mnId;
  }

  std::shared_ptr<Map> GetMap();
  void UpdateMap(const std::shared_ptr<Map>& pMap);

  void SetNewBias(const IMU::Bias& b);
  Eigen::Vector3f GetGyroBias();

  Eigen::Vector3f GetAccBias();

  IMU::Bias GetImuBias();

  bool ProjectPointDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v);
  bool ProjectPointUnDistort(MapPoint* pMP, cv::Point2f& kp, float& u,
                             float& v);

  void PreSave(set<std::shared_ptr<KeyFrame>>& spKF, set<MapPoint*>& spMP,
               set<std::shared_ptr<GeometricCamera>>& spCam);
  void PostLoad(map<unsigned long int, std::shared_ptr<KeyFrame>>& mpKFid,
                map<unsigned long int, MapPoint*>& mpMPid,
                map<unsigned int, std::shared_ptr<GeometricCamera>>& mpCamId);

  void SetORBVocabulary(const std::shared_ptr<ORBVocabulary>& pORBVoc);
  void SetKeyFrameDatabase(const std::shared_ptr<KeyFrameDatabase>& pKFDB);

  bool bImu;

  // The following variables are accesed from only 1 thread or never change (no
  // mutex needed).
 public:
  static unsigned long int nNextId;
  unsigned long int mnId;
  unsigned long int mnFrameId;

  double mTimeStamp;

  // Grid (to speed up feature matching)
  int mnGridCols;
  int mnGridRows;
  float mfGridElementWidthInv;
  float mfGridElementHeightInv;

  // Variables used by the tracking
  unsigned long int mnTrackReferenceForFrame;
  unsigned long int mnFuseTargetForKF;

  // Variables used by the local mapping
  unsigned long int mnBALocalForKF;
  unsigned long int mnBAFixedForKF;

  // Number of optimizations by BA(amount of iterations in BA)
  unsigned long int mnNumberOfOpt;

  // Variables used by the keyframe database
  unsigned long int mnLoopQuery;
  int mnLoopWords;
  float mLoopScore;
  unsigned long int mnRelocQuery;
  int mnRelocWords;
  float mRelocScore;
  unsigned long int mnMergeQuery;
  int mnMergeWords;
  float mMergeScore;
  unsigned long int mnPlaceRecognitionQuery;
  int mnPlaceRecognitionWords;
  float mPlaceRecognitionScore;

  bool mbCurrentPlaceRecognition;

  // Variables used by loop closing
  Sophus::SE3f mTcwGBA;
  Sophus::SE3f mTcwBefGBA;
  Eigen::Vector3f mVwbGBA;
  Eigen::Vector3f mVwbBefGBA;
  IMU::Bias mBiasGBA;
  unsigned long int mnBAGlobalForKF;

  // Variables used by merging
  Sophus::SE3f mTcwMerge;
  Sophus::SE3f mTcwBefMerge;
  Sophus::SE3f mTwcBefMerge;
  Eigen::Vector3f mVwbMerge;
  Eigen::Vector3f mVwbBefMerge;
  IMU::Bias mBiasMerge;
  unsigned long int mnMergeCorrectedForKF;
  unsigned long int mnMergeForKF;
  float mfScaleMerge;
  unsigned long int mnBALocalForMerge;

  float mfScale;

  // Calibration parameters
  float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
  cv::Mat mDistCoef;

  // Number of KeyPoints
  int N;

  // KeyPoints, stereo coordinate and descriptors (all associated by an index)
  std::vector<cv::KeyPoint> mvKeys;
  std::vector<cv::KeyPoint> mvKeysUn;
  std::vector<float> mvuRight;  // negative value for monocular points
  std::vector<float> mvDepth;   // negative value for monocular points
  cv::Mat mDescriptors;

  // BoW
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // Pose relative to parent (this is computed when bad flag is activated)
  Sophus::SE3f mTcp;

  // Scale
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  std::vector<float> mvScaleFactors;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;

  // Image bounds and calibration
  int mnMinX;
  int mnMinY;
  int mnMaxX;
  int mnMaxY;

  // Preintegrated IMU measurements from previous keyframe
  std::shared_ptr<KeyFrame> mPrevKF;
  std::shared_ptr<KeyFrame> mNextKF;

  std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
  IMU::Calib mImuCalib;

  unsigned int mnOriginMapId;

  string mNameFile;

  int mnDataset;

  std::vector<std::shared_ptr<KeyFrame>> mvpLoopCandKFs;
  std::vector<std::shared_ptr<KeyFrame>> mvpMergeCandKFs;

  // bool mbHasHessian;
  // cv::Mat mHessianPose;

  // The following variables need to be accessed trough a mutex to be thread
  // safe.
 protected:
  // sophus poses
  Sophus::SE3<float> mTcw;
  Eigen::Matrix3f mRcw;
  Sophus::SE3<float> mTwc;
  Eigen::Matrix3f mRwc;

  // IMU position
  Eigen::Vector3f mOwb;
  // Velocity (Only used for inertial SLAM)
  Eigen::Vector3f mVw;
  bool mbHasVelocity;

  // Transformation matrix between cameras in stereo fisheye
  Sophus::SE3<float> mTlr;
  Sophus::SE3<float> mTrl;

  // Imu bias
  IMU::Bias mImuBias;

  // MapPoints associated to keypoints
  std::vector<MapPoint*> mvpMapPoints;
  // For save relation without pointer, this is necessary for save/load function
  std::vector<long long int> mvBackupMapPointsId;

  // BoW
  std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
  std::shared_ptr<ORBVocabulary> mpORBvocabulary;

  // Grid over the image to speed up feature matching
  std::vector<std::vector<std::vector<size_t>>> mGrid;

  std::map<std::shared_ptr<KeyFrame>, int> mConnectedKeyFrameWeights;
  std::vector<std::shared_ptr<KeyFrame>> mvpOrderedConnectedKeyFrames;
  std::vector<int> mvOrderedWeights;
  // For save relation without pointer, this is necessary for save/load function
  std::map<unsigned long int, int> mBackupConnectedKeyFrameIdWeights;

  // Spanning Tree and Loop Edges
  bool mbFirstConnection;
  std::shared_ptr<KeyFrame> mpParent;
  std::set<std::shared_ptr<KeyFrame>> mspChildrens;
  std::set<std::shared_ptr<KeyFrame>> mspLoopEdges;
  std::set<std::shared_ptr<KeyFrame>> mspMergeEdges;
  // For save relation without pointer, this is necessary for save/load function
  long long int mBackupParentId;
  std::vector<unsigned long int> mvBackupChildrensId;
  std::vector<unsigned long int> mvBackupLoopEdgesId;
  std::vector<unsigned long int> mvBackupMergeEdgesId;

  // Bad flags
  bool mbNotErase;
  bool mbToBeErased;
  bool mbBad;

  float mHalfBaseline;  // Only for visualization

  std::shared_ptr<Map> mpMap;

  // Backup variables for inertial
  long long int mBackupPrevKFId;
  long long int mBackupNextKFId;
  std::shared_ptr<IMU::Preintegrated> mpBackupImuPreintegrated;

  // Backup for Cameras
  int mnBackupIdCamera, mnBackupIdCamera2;

  // Calibration
  Eigen::Matrix3f mK_;

  // Mutex
  std::mutex mMutexPose;  // for pose, velocity and biases
  std::mutex mMutexConnections;
  std::mutex mMutexFeatures;
  std::mutex mMutexMap;

 public:
  std::shared_ptr<GeometricCamera> mpCamera, mpCamera2;

  // Indexes of stereo observations correspondences
  std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

  Sophus::SE3f GetRelativePoseTrl();
  Sophus::SE3f GetRelativePoseTlr();

  // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
  std::vector<cv::KeyPoint> mvKeysRight;

  int NLeft, NRight;

  std::vector<std::vector<std::vector<size_t>>> mGridRight;

  Sophus::SE3<float> GetRightPose();
  Sophus::SE3<float> GetRightPoseInverse();

  Eigen::Vector3f GetRightCameraCenter();
  Eigen::Matrix<float, 3, 3> GetRightRotation();
  Eigen::Vector3f GetRightTranslation();

  void PrintPointDistribution() {
    int left = 0, right = 0;
    int Nlim = (NLeft != -1) ? NLeft : N;
    for (int i = 0; i < N; i++) {
      if (mvpMapPoints[i]) {
        if (i < Nlim)
          left++;
        else
          right++;
      }
    }
    oslog::debug("Point distribution in KeyFrame: left-> {} --- right-> {}",
                 left, right);
  }
};

}  // namespace ORB_SLAM3
