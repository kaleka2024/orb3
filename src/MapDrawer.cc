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

#include "MapDrawer.h"

#include <pangolin/pangolin.h>

#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM3 {

MapDrawer::MapDrawer(const std::shared_ptr<Atlas> &pAtlas,
                     const std::shared_ptr<Settings> &settings)
    : mpAtlas(pAtlas) {
  newParameterLoader(settings);
}

void MapDrawer::newParameterLoader(const std::shared_ptr<Settings> &settings) {
  mKeyFrameSize = settings->keyFrameSize();
  mKeyFrameLineWidth = settings->keyFrameLineWidth();
  mGraphLineWidth = settings->graphLineWidth();
  mPointSize = settings->pointSize();
  mCameraSize = settings->cameraSize();
  mCameraLineWidth = settings->cameraLineWidth();
}

void MapDrawer::DrawMapPoints() {
  std::shared_ptr<Map> pActiveMap = mpAtlas->GetCurrentMap();
  if (!pActiveMap) return;

  const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
  const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

  set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

  if (vpMPs.empty()) return;

  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(0.0, 0.0, 0.0);

  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i])) continue;
    Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
    glVertex3f(pos(0), pos(1), pos(2));
  }
  glEnd();

  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(1.0, 0.0, 0.0);

  for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end();
       sit != send; sit++) {
    if ((*sit)->isBad()) continue;
    Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
    glVertex3f(pos(0), pos(1), pos(2));
  }

  glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph,
                              const bool bDrawInertialGraph,
                              const bool bDrawOptLba) {
  const float &w = mKeyFrameSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  std::shared_ptr<Map> pActiveMap = mpAtlas->GetCurrentMap();
  // DEBUG LBA
  std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
  std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

  if (!pActiveMap) return;

  auto const vpKFs = pActiveMap->GetAllKeyFrames();

  if (bDrawKF) {
    for (auto const &pKF : vpKFs) {
      Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
      // unsigned int index_color = pKF->mnOriginMapId;

      glPushMatrix();

      glMultMatrixf(static_cast<GLfloat *>(Twc.data()));

      if (!pKF->GetParent()) {
        // It is the first KF in the map
        glLineWidth(mKeyFrameLineWidth * 5);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
      } else {
        // cout << "Child KF: " << vpKFs[i]->mnId << endl;
        glLineWidth(mKeyFrameLineWidth);
        if (bDrawOptLba) {
          if (sOptKFs.find(pKF->mnId) != sOptKFs.end()) {
            glColor3f(0.0f, 1.0f, 0.0f);  // Green -> Opt KFs
          } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
            glColor3f(1.0f, 0.0f, 0.0f);  // Red -> Fixed KFs
          } else {
            glColor3f(0.0f, 0.0f, 1.0f);  // Basic color
          }
        } else {
          glColor3f(0.0f, 0.0f, 1.0f);  // Basic color
        }
        glBegin(GL_LINES);
      }

      glVertex3f(0, 0, 0);
      glVertex3f(w, h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, h, z);

      glVertex3f(w, h, z);
      glVertex3f(w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(-w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(w, h, z);

      glVertex3f(-w, -h, z);
      glVertex3f(w, -h, z);
      glEnd();

      glPopMatrix();

      glEnd();
    }
  }

  if (bDrawGraph) {
    glLineWidth(mGraphLineWidth);
    glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
    glBegin(GL_LINES);

    // cout << "-----------------Draw graph-----------------" << endl;

    for (auto const &pKFi : vpKFs) {
      // Covisibility Graph
      auto const vCovKFs = pKFi->GetCovisiblesByWeight(100);
      Eigen::Vector3f Ow = pKFi->GetCameraCenter();
      if (!vCovKFs.empty()) {
        for (auto const &pKF2 : vCovKFs) {
          if (pKF2->mnId < pKFi->mnId) continue;
          Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
          glVertex3f(Ow(0), Ow(1), Ow(2));
          glVertex3f(Ow2(0), Ow2(1), Ow2(2));
        }
      }

      // Spanning tree
      auto pParent = pKFi->GetParent();
      if (pParent) {
        Eigen::Vector3f Owp = pParent->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owp(0), Owp(1), Owp(2));
      }

      // Loops
      auto const sLoopKFs = pKFi->GetLoopEdges();
      for (auto pKF2 : sLoopKFs) {
        if (pKF2->mnId < pKFi->mnId) continue;
        Eigen::Vector3f Owl = pKF2->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owl(0), Owl(1), Owl(2));
      }
    }

    glEnd();
  }

  if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
    glLineWidth(mGraphLineWidth);
    glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
    glBegin(GL_LINES);

    // Draw inertial links
    for (auto const &pKFi : vpKFs) {
      Eigen::Vector3f Ow = pKFi->GetCameraCenter();
      auto const &pNext = pKFi->mNextKF;
      if (pNext) {
        Eigen::Vector3f Owp = pNext->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owp(0), Owp(1), Owp(2));
      }
    }

    glEnd();
  }

  vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();

  if (bDrawKF) {
    for (auto pMap : vpMaps) {
      if (pMap == pActiveMap) continue;

      auto const vpKFs = pMap->GetAllKeyFrames();

      for (auto const &pKF : vpKFs) {
        Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
        unsigned int index_color = pKF->mnOriginMapId;

        glPushMatrix();

        glMultMatrixf(static_cast<GLfloat *>(Twc.data()));

        if (!pKF->GetParent()) {
          // It is the first KF in the map
          glLineWidth(mKeyFrameLineWidth * 5);
          glColor3f(1.0f, 0.0f, 0.0f);
          glBegin(GL_LINES);
        } else {
          glLineWidth(mKeyFrameLineWidth);
          glColor3f(mfFrameColors[index_color][0],
                    mfFrameColors[index_color][1],
                    mfFrameColors[index_color][2]);
          glBegin(GL_LINES);
        }

        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
      }
    }
  }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
  const float &w = mCameraSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();

#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif

  glLineWidth(mCameraLineWidth);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();

  glPopMatrix();
}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw) {
  unique_lock<mutex> lock(mMutexCamera);
  mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M,
                                             pangolin::OpenGlMatrix &MOw) {
  Eigen::Matrix4f Twc;
  {
    unique_lock<mutex> lock(mMutexCamera);
    Twc = mCameraPose.matrix();
  }

  for (int i = 0; i < 4; i++) {
    M.m[4 * i] = Twc(0, i);
    M.m[4 * i + 1] = Twc(1, i);
    M.m[4 * i + 2] = Twc(2, i);
    M.m[4 * i + 3] = Twc(3, i);
  }

  MOw.SetIdentity();
  MOw.m[12] = Twc(0, 3);
  MOw.m[13] = Twc(1, 3);
  MOw.m[14] = Twc(2, 3);
}
}  // namespace ORB_SLAM3
