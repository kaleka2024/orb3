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

#include <openssl/md5.h>
#include <pangolin/pangolin.h>

#include <algorithm>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Converter.h"
#include "System.h"

namespace ORB_SLAM3 {

void System::SaveTrajectoryTUM(const string &filename) {
  vector<std::shared_ptr<KeyFrame>> vpKFs = mpAtlas->GetAllKeyFrames();

  if (vpKFs.size() == 0) {
    oslog::error("Cannot save TUM trajectory, there are no Keyframes");
    return;
  }

  oslog::info("Saving camera trajectory to {} ...", filename);
  if (sensorType() == SensorType::MONOCULAR) {
    oslog::error("ERROR: SaveTrajectoryTUM cannot be used for monocular.");
    return;
  }

  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  list<std::shared_ptr<KeyFrame>>::iterator lRit =
      mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();
  for (list<Sophus::SE3f>::iterator
           lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    if (*lbL) continue;

    std::shared_ptr<KeyFrame> pKF = *lRit;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    Sophus::SE3f Tcw = (*lit) * Trw;
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " "
      << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z()
      << " " << q.w() << endl;
  }
  f.close();
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
  oslog::info("Saving keyframe trajectory to {} ...", filename);

  vector<std::shared_ptr<KeyFrame>> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (auto pKF : vpKFs) {
    if (pKF->isBad()) continue;

    Sophus::SE3f Twc = pKF->GetPoseInverse();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    Eigen::Vector3f t = Twc.translation();
    f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0)
      << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " "
      << q.z() << " " << q.w() << endl;
  }

  f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename) {
  oslog::info("Saving trajectory to {} ...", filename);

  /*if(sensorType()==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  endl; return;
  }*/

  vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();
  size_t numMaxKFs = 0;
  std::shared_ptr<Map> pBiggerMap;
  oslog::debug("There are {} maps in the atlas", std::to_string(vpMaps.size()));
  for (auto pMap : vpMaps) {
    oslog::debug("  Map {} has {} KFs", std::to_string(pMap->GetId()),
                 std::to_string(pMap->GetAllKeyFrames().size()));
    if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  auto vpKFs = pBiggerMap->GetAllKeyFrames();
  if (vpKFs.size() == 0) {
    oslog::error("Cannot save EUROC trajectory, there are no Keyframes");
    return;
  }

  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb;  // Can be word to cam0 or world to b depending on IMU or not.
  if (sensorType().isImu()) {
    Twb = vpKFs[0]->GetImuPose();
  } else {
    Twb = vpKFs[0]->GetPoseInverse();
  }

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  auto lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();

  // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
  // cout << "size mlRelativeFramePoses: " <<
  // mpTracker->mlRelativeFramePoses.size() << endl; cout << "size
  // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl; cout
  // << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

  for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
            lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    if (*lbL) continue;

    std::shared_ptr<KeyFrame> pKF = *lRit;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF) continue;

    while (pKF->isBad()) {
      // cout << " 2.bad" << endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // cout << "--Parent KF: " << pKF->mnId << endl;
    }

    if (!pKF || pKF->GetMap() != pBiggerMap) {
      // cout << "--Parent KF is from another map" << endl;
      continue;
    }

    Trw = Trw * pKF->GetPose() *
          Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    if (sensorType().isImu()) {
      Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0)
        << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = ((*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f twc = Twc.translation();
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0)
        << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << endl;
    }
  }
  f.close();
  oslog::info("End of saving trajectory to {} ...", filename);
}

void System::SaveTrajectoryEuRoC(const string &filename,
                                 const std::shared_ptr<Map> &pMap) {
  oslog::info("Saving trajectory of map {} to {} ...", pMap->GetId(), filename);

  /*if(sensorType()==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  endl; return;
  }*/

  auto vpKFs = pMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb;  // Can be word to cam0 or world to b dependingo on IMU or not.
  if (sensorType().isImu()) {
    Twb = vpKFs[0]->GetImuPose();
  } else {
    Twb = vpKFs[0]->GetPoseInverse();
  }
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  auto lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();

  // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
  // cout << "size mlRelativeFramePoses: " <<
  // mpTracker->mlRelativeFramePoses.size() << endl; cout << "size
  // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl; cout
  // << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

  for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
            lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    // cout << "1" << endl;
    if (*lbL) continue;

    std::shared_ptr<KeyFrame> pKF = *lRit;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF) continue;

    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // cout << "--Parent KF: " << pKF->mnId << endl;
    }

    if (!pKF || pKF->GetMap() != pMap) {
      // cout << "--Parent KF is from another map" << endl;
      continue;
    }

    Trw = Trw * pKF->GetPose() *
          Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    if (sensorType().isImu()) {
      Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0)
        << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = ((*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f twc = Twc.translation();
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0)
        << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << endl;
    }
  }
  f.close();
  oslog::info("End of saving trajectory to {} ...", filename);
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename) {
  oslog::info("Saving keyframe trajectory to {} ...", filename);

  vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();
  std::shared_ptr<Map> pBiggerMap;
  size_t numMaxKFs = 0;
  for (auto pMap : vpMaps) {
    if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  if (!pBiggerMap) {
    oslog::error("There is not a map!!");
    return;
  }

  auto vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (auto pKF : vpKFs) {
    // pKF->SetPose(pKF->GetPose()*Two);

    if (!pKF || pKF->isBad()) continue;
    if (sensorType().isImu()) {
      Sophus::SE3f Twb = pKF->GetImuPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;

    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << endl;
    }
  }
  f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename,
                                         const std::shared_ptr<Map> &pMap) {
  oslog::info("Saving keyframe trajectory of map {} to {} ...", pMap->GetId(),
              filename);

  auto vpKFs = pMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (auto pKF : vpKFs) {
    if (!pKF || pKF->isBad()) continue;

    if (sensorType().isImu()) {
      Sophus::SE3f Twb = pKF->GetImuPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << endl;
    }
  }

  f.close();
}

void System::SaveTrajectoryKITTI(const string &filename) {
  oslog::info("Saving camera trajectory to {} ...", filename);
  if (sensorType() == SensorType::MONOCULAR) {
    oslog::error("ERROR: SaveTrajectoryKITTI cannot be used for monocular.");
    return;
  }

  auto vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  auto lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  for (list<Sophus::SE3f>::iterator
           lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++) {
    auto pKF = *lRit;

    Sophus::SE3f Trw;

    if (!pKF) continue;

    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Tow;

    Sophus::SE3f Tcw = (*lit) * Trw;
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix3f Rwc = Twc.rotationMatrix();
    Eigen::Vector3f twc = Twc.translation();

    f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2)
      << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " "
      << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1)
      << " " << Rwc(2, 2) << " " << twc(2) << endl;
  }
  f.close();
}

void System::SaveDebugData(const int &initIdx) {
  // 0. Save initialization trajectory
  SaveTrajectoryEuRoC("init_FrameTrajectoy_" +
                      to_string(mpLocalMapper->mInitSect) + "_" +
                      to_string(initIdx) + ".txt");

  // 1. Save scale
  ofstream f;
  f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mScale << endl;
  f.close();

  // 2. Save gravity direction
  f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << ","
    << mpLocalMapper->mRwg(0, 2) << endl;
  f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << ","
    << mpLocalMapper->mRwg(1, 2) << endl;
  f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << ","
    << mpLocalMapper->mRwg(2, 2) << endl;
  f.close();

  // 3. Save computational cost
  f.open("init_CompCost_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mCostTime << endl;
  f.close();

  // 4. Save biases
  f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << ","
    << mpLocalMapper->mbg(2) << endl;
  f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << ","
    << mpLocalMapper->mba(2) << endl;
  f.close();

  // 5. Save covariance matrix
  f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" +
             to_string(initIdx) + ".txt",
         ios_base::app);
  f << fixed;
  for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++) {
    for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++) {
      if (j != 0) f << ",";
      f << setprecision(15) << mpLocalMapper->mcovInertial(i, j);
    }
    f << endl;
  }
  f.close();

  // 6. Save initialization time
  f.open("init_Time_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mInitTime << endl;
  f.close();
}

}  // namespace ORB_SLAM3
