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

// 地图绘制器头文件声明
#include "MapDrawer.h"
// Pangolin轻量级OpenGL可视化库：用于3D点云、相机、连线的渲染
#include <pangolin/pangolin.h>
// STL标准库：智能指针管理
#include <memory>
// STL标准库：互斥锁，保证多线程下绘制数据的线程安全
#include <mutex>
// STL标准库：集合容器，用于去重、快速查找
#include <set>
// STL标准库：动态数组容器
#include <vector>

// ORB-SLAM3内部：关键帧类
#include "KeyFrame.h"
// ORB-SLAM3内部：地图点类
#include "MapPoint.h"

namespace ORB_SLAM3 {

// ==============================================
// 地图绘制器构造函数
// 输入：pAtlas 图集管理器指针，settings 可视化配置参数对象
// 功能：绑定图集管理器，加载所有绘制参数
// ==============================================
MapDrawer::MapDrawer(const std::shared_ptr<Atlas> &pAtlas,
                     const std::shared_ptr<Settings> &settings)
    : mpAtlas(pAtlas) {
  // 从配置对象加载可视化参数
  newParameterLoader(settings);
}

// ==============================================
// 从配置加载可视化参数
// 输入：settings 配置参数智能指针
// 功能：读取关键帧、连线、地图点、相机等元素的尺寸与线宽
// ==============================================
void MapDrawer::newParameterLoader(const std::shared_ptr<Settings> &settings) {
  mKeyFrameSize = settings->keyFrameSize();          // 关键帧相机模型大小
  mKeyFrameLineWidth = settings->keyFrameLineWidth();  // 关键帧线框线宽
  mGraphLineWidth = settings->graphLineWidth();          // 共视图/生成树连线宽度
  mPointSize = settings->pointSize();                // 地图点像素大小
  mCameraSize = settings->cameraSize();              // 当前相机模型大小
  mCameraLineWidth = settings->cameraLineWidth();      // 当前相机线框线宽
}

// ==============================================
// 绘制所有地图点
// 颜色规则：普通地图点黑色，参考地图点红色；坏点跳过不绘制
// ==============================================
void MapDrawer::DrawMapPoints() {
  // 获取当前活动地图
  std::shared_ptr<Map> pActiveMap = mpAtlas->GetCurrentMap();
  if (!pActiveMap) return;

  // 获取活动地图的全部地图点、参考地图点
  const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
  const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

  // 参考点转集合，用于快速去重查找
  set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

  if (vpMPs.empty()) return;

  // 设置点大小，开始绘制普通地图点（黑色）
  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(0.0, 0.0, 0.0);

  // 遍历所有地图点，跳过坏点和参考点
  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i])) continue;
    Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
    glVertex3f(pos(0), pos(1), pos(2));
  }
  glEnd();

  // 绘制参考地图点（红色，用于重定位、初始化的参考标记）
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

// ==============================================
// 绘制关键帧与各类连接图
// 输入：
//   bDrawKF：是否绘制关键帧相机线框模型
//   bDrawGraph：是否绘制共视图+生成树+回环边
//   bDrawInertialGraph：是否绘制IMU时间序列连线
//   bDrawOptLba：是否用颜色区分局部BA的优化/固定关键帧（DEBUG调试用）
// ==============================================
void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph,
                              const bool bDrawInertialGraph,
                              const bool bDrawOptLba) {
  // 关键帧相机模型尺寸：宽w，高h，深度z（金字塔视锥比例）
  const float &w = mKeyFrameSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  std::shared_ptr<Map> pActiveMap = mpAtlas->GetCurrentMap();

  // DEBUG LBA：获取局部BA中优化、固定的关键帧ID集合，用于着色调试
  std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
  std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

  if (!pActiveMap) return;

  // 获取活动地图的所有关键帧
  auto const vpKFs = pActiveMap->GetAllKeyFrames();

  // ========== 绘制关键帧相机模型 ==========
  if (bDrawKF) {
    for (auto const &pKF : vpKFs) {
      // 获取关键帧的逆位姿（相机→世界）矩阵，用于OpenGL模型变换
      Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
      // unsigned int index_color = pKF->mnOriginMapId;

      // 压入矩阵栈，保存当前变换状态
      glPushMatrix();

      // 应用位姿变换：将原点处的相机模型变换到世界坐标系对应位置
      glMultMatrixf(static_cast<GLfloat *>(Twc.data()));

      // 根节点（地图第一个关键帧）：线宽5倍，红色高亮
      if (!pKF->GetParent()) {
        // It is the first KF in the map
        glLineWidth(mKeyFrameLineWidth * 5);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
      } else {
        // cout << "Child KF: " << vpKFs[i]->mnId << endl;
        glLineWidth(mKeyFrameLineWidth);

        // DEBUG模式：按局部BA角色着色
        if (bDrawOptLba) {
          if (sOptKFs.find(pKF->mnId) != sOptKFs.end()) {
            glColor3f(0.0f, 1.0f, 0.0f);  // Green -> Opt KFs 优化帧绿色
          } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
            glColor3f(1.0f, 0.0f, 0.0f);  // Red -> Fixed KFs 固定帧红色
          } else {
            glColor3f(0.0f, 0.0f, 1.0f);  // Basic color 普通帧蓝色
          }
        } else {
          glColor3f(0.0f, 0.0f, 1.0f);  // Basic color 默认蓝色
        }
        glBegin(GL_LINES);
      }

      // 绘制相机视锥线框（金字塔形，顶点在相机光心，底面朝向场景）
      // 顶点 → 右上角
      glVertex3f(0, 0, 0);
      glVertex3f(w, h, z);
      // 顶点 → 右下角
      glVertex3f(0, 0, 0);
      glVertex3f(w, -h, z);
      // 顶点 → 左下角
      glVertex3f(0, 0, 0);
      glVertex3f(-w, -h, z);
      // 顶点 → 左上角
      glVertex3f(0, 0, 0);
      glVertex3f(-w, h, z);

      // 绘制视锥底面四条边
      glVertex3f(w, h, z);
      glVertex3f(w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(-w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(w, h, z);

      glVertex3f(-w, -h, z);
      glVertex3f(w, -h, z);
      glEnd();

      // 弹出矩阵栈，恢复上级变换状态
      glPopMatrix();

      glEnd();
    }
  }

  // ========== 绘制共视图、生成树、回环边 ==========
  if (bDrawGraph) {
    glLineWidth(mGraphLineWidth);
    glColor4f(0.0f, 1.0f, 0.0f, 0.6f);  // 绿色半透明
    glBegin(GL_LINES);

    // cout << "-----------------Draw graph-----------------" << endl;

    for (auto const &pKFi : vpKFs) {
      // Covisibility Graph
      // 绘制共视连接：取权重前100的共视关键帧对
      auto const vCovKFs = pKFi->GetCovisiblesByWeight(100);
      Eigen::Vector3f Ow = pKFi->GetCameraCenter();

      if (!vCovKFs.empty()) {
        for (auto const &pKF2 : vCovKFs) {
          // 只绘制ID更大的一侧，避免同一条边重复绘制两次
          if (pKF2->mnId < pKFi->mnId) continue;
          Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
          glVertex3f(Ow(0), Ow(1), Ow(2));
          glVertex3f(Ow2(0), Ow2(1), Ow2(2));
        }
      }

      // Spanning tree
      // 绘制生成树的父子连接
      auto pParent = pKFi->GetParent();
      if (pParent) {
        Eigen::Vector3f Owp = pParent->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owp(0), Owp(1), Owp(2));
      }

      // Loops
      // 绘制回环边连接
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

  // ========== 绘制IMU惯性时序连线 ==========
  if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
    glLineWidth(mGraphLineWidth);
    glColor4f(1.0f, 0.0f, 0.0f, 0.6f);  // 红色半透明
    glBegin(GL_LINES);

    // Draw inertial links
    // 按时间顺序连接相邻关键帧，体现IMU时间序列
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

  // ========== 绘制其他非活动地图的关键帧 ==========
  vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();

  if (bDrawKF) {
    for (auto pMap : vpMaps) {
      // 跳过活动地图（已在上文绘制）
      if (pMap == pActiveMap) continue;

      auto const vpKFs = pMap->GetAllKeyFrames();

      for (auto const &pKF : vpKFs) {
        Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
        unsigned int index_color = pKF->mnOriginMapId;

        glPushMatrix();

        glMultMatrixf(static_cast<GLfloat *>(Twc.data()));

        // 其他地图的根节点同样加粗红色
        if (!pKF->GetParent()) {
          // It is the first KF in the map
          glLineWidth(mKeyFrameLineWidth * 5);
          glColor3f(1.0f, 0.0f, 0.0f);
          glBegin(GL_LINES);
        } else {
          glLineWidth(mKeyFrameLineWidth);
          // 按起源地图ID取预设颜色，区分不同子地图
          glColor3f(mfFrameColors[index_color][0],
                      mfFrameColors[index_color][1],
                      mfFrameColors[index_color][2]);
          glBegin(GL_LINES);
        }

        // 同样绘制相机视锥线框
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

// ==============================================
// 绘制当前实时相机的线框模型
// 输入：Twc 相机→世界的OpenGL变换矩阵
// 说明：当前相机用绿色绘制，代表跟踪的实时位姿
// ==============================================
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
  // 当前相机模型尺寸
  const float &w = mCameraSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  // 压入矩阵栈
  glPushMatrix();

#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif

  glLineWidth(mCameraLineWidth);
  glColor3f(0.0f, 1.0f, 0.0f);  // 绿色：当前实时相机
  glBegin(GL_LINES);

  // 绘制相机视锥线框，与关键帧金字塔结构一致
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

  // 弹出矩阵栈
  glPopMatrix();
}

// ==============================================
// 设置当前相机的位姿（由跟踪线程更新）
// 输入：Tcw 世界→相机的SE3位姿
// 线程安全：加相机互斥锁保护位姿数据
// ==============================================
void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw) {
  unique_lock<mutex> lock(mMutexCamera);
  // 存储逆位姿（相机→世界），用于绘制时的模型变换
  mCameraPose = Tcw.inverse();
}

// ==============================================
// 获取当前相机的OpenGL格式矩阵与世界平移矩阵
// 输出：M 相机位姿的OpenGL矩阵，MOw 仅含平移的世界坐标矩阵
// 说明：适配OpenGL列主序存储格式，用于设置pangolin观察相机视角
// ==============================================
void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M,
                                             pangolin::OpenGlMatrix &MOw) {
  Eigen::Matrix4f Twc;
  {
    unique_lock<mutex> lock(mMutexCamera);
    // 获取相机→世界的位姿矩阵
    Twc = mCameraPose.matrix();
  }

  // Eigen行主序转OpenGL列主序：逐列填充矩阵元素
  for (int i = 0; i < 4; i++) {
    M.m[4 * i] = Twc(0, i);
    M.m[4 * i + 1] = Twc(1, i);
    M.m[4 * i + 2] = Twc(2, i);
    M.m[4 * i + 3] = Twc(3, i);
  }

  // MOw：单位矩阵 + 相机世界坐标平移，用于世界坐标系下的相机定位
  MOw.SetIdentity();
  MOw.m[12] = Twc(0, 3);
  MOw.m[13] = Twc(1, 3);
  MOw.m[14] = Twc(2, 3);
}

}  // namespace ORB_SLAM3
