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

#include "Atlas.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

#include "GeometricCamera.h"
#include "KannalaBrandt8.h"
#include "Pinhole.h"
#include "Viewer.h"

namespace ORB_SLAM3 {

Atlas::Atlas() : mpCurrentMap(nullptr), mpViewer(nullptr) {}

Atlas::Atlas(int initKFid)
    : mnLastInitKFidMap(initKFid), mpCurrentMap(nullptr), mpViewer(nullptr) {
  CreateNewMap();
}

Atlas::~Atlas() { mspMaps.clear(); }

void Atlas::CreateNewMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  oslog::info("Creation of new map with id: {}", Map::nNextId);
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                          1;  // The init KF is the next of current maximum

    mpCurrentMap->SetStoredMap();
    oslog::info("Stored map with ID: {}", mpCurrentMap->GetId());

    // if(mpViewer)
    //     mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
  }
  oslog::info("Creation of new map with last KF id: {}", mnLastInitKFidMap);

  mpCurrentMap = std::make_shared<Map>(mnLastInitKFidMap);
  mpCurrentMap->SetCurrentMap();
  mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(const std::shared_ptr<Map> &pMap) {
  unique_lock<mutex> lock(mMutexAtlas);
  oslog::info("Change to map with id: {}", pMap->GetId());
  if (mpCurrentMap) {
    mpCurrentMap->SetStoredMap();
  }

  mpCurrentMap = pMap;
  mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mnLastInitKFidMap;
}

void Atlas::SetViewer(const std::shared_ptr<Viewer> &pViewer) {
  mpViewer = pViewer;
}

void Atlas::AddKeyFrame(const std::shared_ptr<KeyFrame> &pKF) {
  std::shared_ptr<Map> pMapKF = pKF->GetMap();
  pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint *pMP) {
  std::shared_ptr<Map> pMapMP = pMP->GetMap();
  pMapMP->AddMapPoint(pMP);
}

std::shared_ptr<GeometricCamera> Atlas::AddCamera(
    const std::shared_ptr<GeometricCamera> &pCam) {
  auto const it = std::find(mvpCameras.begin(), mvpCameras.end(), pCam);
  if (it != mvpCameras.end()) {
    return *it;
  }

  mvpCameras.push_back(pCam);
  return pCam;
}

std::vector<std::shared_ptr<GeometricCamera>> Atlas::GetAllCameras() {
  return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs) {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->KeyFramesInMap();
}

std::vector<std::shared_ptr<KeyFrame>> Atlas::GetAllKeyFrames() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint *> Atlas::GetAllMapPoints() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint *> Atlas::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetReferenceMapPoints();
}

vector<std::shared_ptr<Map>> Atlas::GetAllMaps() {
  unique_lock<mutex> lock(mMutexAtlas);
  struct compFunctor {
    inline bool operator()(const std::shared_ptr<Map> &elem1,
                           const std::shared_ptr<Map> &elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  vector<std::shared_ptr<Map>> vMaps(mspMaps.begin(), mspMaps.end());
  sort(vMaps.begin(), vMaps.end(), compFunctor());
  return vMaps;
}

int Atlas::CountMaps() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mspMaps.size();
}

void Atlas::clearMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->clear();
}

void Atlas::clearAtlas() {
  unique_lock<mutex> lock(mMutexAtlas);
  /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end();
  it!=send; it++)
  {
      (*it)->clear();
      delete *it;
  }*/
  mspMaps.clear();
  mpCurrentMap.reset();
  mnLastInitKFidMap = 0;
}

std::shared_ptr<Map> Atlas::GetCurrentMap() {
  unique_lock<mutex> lock(mMutexAtlas);
  if (!mpCurrentMap) CreateNewMap();
  while (mpCurrentMap->IsBad()) usleep(3000);

  return mpCurrentMap;
}

void Atlas::SetMapBad(const std::shared_ptr<Map> &pMap) {
  mspMaps.erase(pMap);
  pMap->SetBad();

  mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps() { mspBadMaps.clear(); }

bool Atlas::isInertial() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized() {
  unique_lock<mutex> lock(mMutexAtlas);
  mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized() {
  unique_lock<mutex> lock(mMutexAtlas);
  return mpCurrentMap->isImuInitialized();
}

void Atlas::PreSave() {
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                          1;  // The init KF is the next of current maximum
  }

  struct compFunctor {
    inline bool operator()(const std::shared_ptr<Map> &elem1,
                           const std::shared_ptr<Map> &elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
  sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

  std::set<std::shared_ptr<GeometricCamera>> spCams(mvpCameras.begin(),
                                                    mvpCameras.end());
  for (auto pMi : mvpBackupMaps) {
    if (!pMi || pMi->IsBad()) continue;

    if (pMi->GetAllKeyFrames().size() == 0) {
      // Empty map, erase before of save it.
      SetMapBad(pMi);
      continue;
    }
    pMi->PreSave(spCams);
  }
  RemoveBadMaps();
}

void Atlas::PostLoad() {
  map<unsigned int, std::shared_ptr<GeometricCamera>> mpCams;
  for (auto pCam : mvpCameras) {
    mpCams[pCam->GetId()] = pCam;
  }

  mspMaps.clear();
  unsigned long int numKF = 0, numMP = 0;
  for (auto pMi : mvpBackupMaps) {
    mspMaps.insert(pMi);
    pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
    numKF += pMi->GetAllKeyFrames().size();
    numMP += pMi->GetAllMapPoints().size();
  }
  mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(
    const std::shared_ptr<KeyFrameDatabase> &pKFDB) {
  mpKeyFrameDB = pKFDB;
}

std::shared_ptr<KeyFrameDatabase> Atlas::GetKeyFrameDatabase() {
  return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(const std::shared_ptr<ORBVocabulary> &pORBVoc) {
  mpORBVocabulary = pORBVoc;
}

std::shared_ptr<ORBVocabulary> Atlas::GetORBVocabulary() {
  return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF() {
  unique_lock<mutex> lock(mMutexAtlas);
  long unsigned int num = 0;
  for (auto const &pMap_i : mspMaps) {
    num += pMap_i->GetAllKeyFrames().size();
  }

  return num;
}

long unsigned int Atlas::GetNumLivedMP() {
  unique_lock<mutex> lock(mMutexAtlas);
  long unsigned int num = 0;
  for (auto const &pMap_i : mspMaps) {
    num += pMap_i->GetAllMapPoints().size();
  }

  return num;
}

map<long unsigned int, std::shared_ptr<KeyFrame>> Atlas::GetAtlasKeyframes() {
  map<long unsigned int, std::shared_ptr<KeyFrame>> mpIdKFs;
  for (auto const &pMap_i : mvpBackupMaps) {
    vector<std::shared_ptr<KeyFrame>> vpKFs_Mi = pMap_i->GetAllKeyFrames();

    for (auto const &pKF_j_Mi : vpKFs_Mi) {
      mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
    }
  }

  return mpIdKFs;
}

}  // namespace ORB_SLAM3
