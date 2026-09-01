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

#include <pangolin/pangolin.h>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM3 {

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map : public std::enable_shared_from_this<Map> {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & mnId;
    ar & mnInitKFid;
    ar & mnMaxKFid;
    ar & mnBigChangeIdx;

    // Save/load a set structure, the set structure is broken in libboost 1.58
    // for ubuntu 16.04, a vector is serializated
    ar & mspKeyFrames;
    ar & mspMapPoints;
    ar & mvpBackupKeyFrames;
    ar & mvpBackupMapPoints;

    ar & mvBackupKeyFrameOriginsId;

    ar & mnBackupKFinitialID;
    ar & mnBackupKFlowerID;

    ar & mbImuInitialized;
    ar & mbIsInertial;
    ar & mbIMU_BA1;
    ar & mbIMU_BA2;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Map();
  explicit Map(int initKFid);

  ~Map();

  void AddKeyFrame(const std::shared_ptr<KeyFrame>& pKF);
  void AddMapPoint(MapPoint* pMP);
  void EraseMapPoint(MapPoint* pMP);
  void EraseKeyFrame(const std::shared_ptr<KeyFrame>& pKF);
  void SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);
  void InformNewBigChange();
  int GetLastBigChangeIdx();

  std::vector<std::shared_ptr<KeyFrame> > GetAllKeyFrames();
  std::vector<MapPoint*> GetAllMapPoints();
  std::vector<MapPoint*> GetReferenceMapPoints();

  unsigned long int MapPointsInMap();
  long unsigned KeyFramesInMap();

  unsigned long int GetId();

  unsigned long int GetInitKFid();
  void SetInitKFid(unsigned long int initKFif);
  unsigned long int GetMaxKFid();

  std::shared_ptr<KeyFrame> GetOriginKF();

  void SetCurrentMap();
  void SetStoredMap();

  bool HasThumbnail();
  bool IsInUse();

  void SetBad();
  bool IsBad();

  void clear();

  int GetMapChangeIndex();
  void IncreaseChangeIndex();
  int GetLastMapChange();
  void SetLastMapChange(int currentChangeId);

  void SetImuInitialized();
  bool isImuInitialized();

  void ApplyScaledRotation(const Sophus::SE3f& T, const float s,
                           const bool bScaledVel = false);

  void SetInertialSensor();
  bool IsInertial();
  void SetInertialBA1();
  void SetInertialBA2();
  bool GetInertialBA1();
  bool GetInertialBA2();

  void PrintEssentialGraph();
  bool CheckEssentialGraph();
  void ChangeId(unsigned long int nId);

  unsigned int GetLowerKFID();

  void PreSave(std::set<std::shared_ptr<GeometricCamera> >& spCams);
  void PostLoad(
      const std::shared_ptr<KeyFrameDatabase>& pKFDB,
      const std::shared_ptr<ORBVocabulary>&
          pORBVoc /*, map<unsigned long int, KeyFrame*>& mpKeyFrameId*/,
      map<unsigned int, std::shared_ptr<GeometricCamera> >& mpCams);

  void printReprojectionError(
      list<std::shared_ptr<KeyFrame> >& lpLocalWindowKFs,
      const std::shared_ptr<KeyFrame>& mpCurrentKF, string& name,
      string& name_folder);

  vector<std::shared_ptr<KeyFrame> > mvpKeyFrameOrigins;
  vector<unsigned long int> mvBackupKeyFrameOriginsId;
  std::shared_ptr<KeyFrame> mpFirstRegionKF;
  std::mutex mMutexMapUpdate;

  // This avoid that two points are created simultaneously in separate threads
  // (id conflict)
  std::mutex mMutexPointCreation;

  bool mbFail;

  // Size of the thumbnail (always in power of 2)
  static const int THUMB_WIDTH = 512;
  static const int THUMB_HEIGHT = 512;

  static unsigned long int nNextId;

  // DEBUG: show KFs which are used in LBA
  std::set<unsigned long int> msOptKFs;
  std::set<unsigned long int> msFixedKFs;

 protected:
  unsigned long int mnId;

  std::set<MapPoint*> mspMapPoints;
  std::set<std::shared_ptr<KeyFrame> > mspKeyFrames;

  // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a
  // vector is serializated
  std::vector<MapPoint*> mvpBackupMapPoints;
  std::vector<std::shared_ptr<KeyFrame> > mvpBackupKeyFrames;

  std::shared_ptr<KeyFrame> mpKFinitial;
  std::shared_ptr<KeyFrame> mpKFlowerID;

  long int mnBackupKFinitialID;
  long int mnBackupKFlowerID;

  std::vector<MapPoint*> mvpReferenceMapPoints;

  bool mbImuInitialized;

  int mnMapChange;
  int mnMapChangeNotified;

  unsigned long int mnInitKFid;
  unsigned long int mnMaxKFid;
  // unsigned long int mnLastLoopKFid;

  // Index related to a big change in the map (loop closure, global BA)
  int mnBigChangeIdx;

  bool mIsInUse;
  bool mHasTumbnail;
  bool mbBad = false;

  bool mbIsInertial;
  bool mbIMU_BA1;
  bool mbIMU_BA2;

  // Mutex
  std::mutex mMutexMap;
};

}  // namespace ORB_SLAM3
