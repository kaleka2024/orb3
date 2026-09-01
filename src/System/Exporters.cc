/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
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
#include <openssl/md5.h>                          // OpenSSL MD5哈希库，本文件未直接使用
#include <pangolin/pangolin.h>                    // Pangolin可视化库，本文件未直接使用

#include <algorithm>                              // C++标准算法库，sort排序依赖
#include <boost/archive/binary_iarchive.hpp>      // Boost序列化二进制读，本文件未直接使用
#include <boost/archive/binary_oarchive.hpp>      // Boost序列化二进制写，本文件未直接使用
#include <boost/archive/text_iarchive.hpp>        // Boost序列化文本读，本文件未直接使用
#include <boost/archive/text_oarchive.hpp>        // Boost序列化文本写，本文件未直接使用
#include <boost/archive/xml_iarchive.hpp>         // Boost序列化XML读，本文件未直接使用
#include <boost/archive/xml_oarchive.hpp>         // Boost序列化XML写，本文件未直接使用
#include <boost/serialization/base_object.hpp>    // Boost序列化基类支持，本文件未直接使用
#include <boost/serialization/shared_ptr.hpp>     // Boost序列化智能指针支持，本文件未直接使用
#include <boost/serialization/string.hpp>         // Boost序列化字符串支持，本文件未直接使用
#include <cstdio>                                 // C标准IO库
#include <exception>                              // C++异常库
#include <iomanip>                                // C++输出格式控制，setprecision精度设置依赖
#include <iostream>                               // C++标准输入输出
#include <list>                                   // C++list双向链表容器，跟踪器历史数据存储容器
#include <memory>                                 // C++智能指针std::shared_ptr
#include <string>                                 // C++std::string字符串
#include <thread>                                 // C++线程库，本文件未直接使用
#include <vector>                                 // C++vector动态数组容器

#include "Converter.h"                            // ORB‑SLAM3格式转换工具，Sophus/Eigen/cv::Mat互转
#include "System.h"                               // ORB‑SLAM3系统主类，本文件全部为System类成员函数

namespace ORB_SLAM3 {

/**
 * @brief 保存普通帧轨迹，TUM数据集格式，时间戳 tx ty tz qx qy qz qw
 * @param filename 输出文件路径名称
 * @note 单目不可调用；输出世界到相机Twc；丢失跟踪的帧直接跳过；以第一帧关键帧作为世界原点
 */
void System::SaveTrajectoryTUM(const string &filename) {
  // 获取Atlas中全部关键帧
  vector<std::shared_ptr<KeyFrame>> vpKFs = mpAtlas->GetAllKeyFrames();

  // 如果没有关键帧，打印错误直接返回，无法输出轨迹
  if (vpKFs.size() == 0) {
    oslog::error("Cannot save TUM trajectory, there are no Keyframes");
    return;
  }

  oslog::info("Saving camera trajectory to {} ...", filename);
  // TUM格式普通帧轨迹不支持单目，单目尺度不确定
  if (sensorType() == SensorType::MONOCULAR) {
    oslog::error("ERROR: SaveTrajectoryTUM cannot be used for monocular.");
    return;
  }

  // 根据关键帧ID从小到大排序关键帧
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  // Two：第一帧关键帧位姿的逆，把第一帧关键帧置于世界坐标系原点
  Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

  ofstream f;                 // 文件输出流对象
  f.open(filename.c_str());   // 打开输出轨迹文件
  f << fixed;                 // 设置输出浮点数固定小数格式

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  list<std::shared_ptr<KeyFrame>>::iterator lRit =
      mpTracker->mlpReferences.begin();       // 每帧对应的参考关键帧迭代器
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin(); // 每帧时间戳迭代器
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();       // 跟踪是否丢失标记迭代器

  // lit：相对参考关键帧的帧位姿；lend为list末尾迭代器
  for (list<Sophus::SE3f>::iterator
           lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    if (*lbL) continue;                     // 如果本帧跟踪丢失，跳过该帧不保存

    std::shared_ptr<KeyFrame> pKF = *lRit;  // 获取当前帧对应的参考关键帧

    Sophus::SE3f Trw;                       // 累积变换，处理被剔除的关键帧，沿着生成树回溯

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    // 如果参考关键帧被剔除(isBad)，沿着生成树向上回溯父关键帧，累积相对变换mTcp
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    // Trw = 回溯累积变换 * 参考关键帧世界位姿 * Two(第一帧置原点变换)
    Trw = Trw * pKF->GetPose() * Two;

    Sophus::SE3f Tcw = (*lit) * Trw;        // Tcw：相机到世界变换
    Sophus::SE3f Twc = Tcw.inverse();       // Twc：世界到相机变换，TUM格式输出Twc

    Eigen::Vector3f twc = Twc.translation();// 取出平移向量
    Eigen::Quaternionf q = Twc.unit_quaternion(); // 取出单位四元数旋转

    // 输出：时间戳  x y z qx qy qz qw
    f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " "
      << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z()
      << " " << q.w() << endl;
  }
  f.close(); // 关闭文件流
}

/**
 * @brief 保存关键帧轨迹 TUM格式，仅输出关键帧位姿
 * @param filename 输出文件路径
 */
void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
  oslog::info("Saving keyframe trajectory to {} ...", filename);

  vector<std::shared_ptr<KeyFrame>> vpKFs = mpAtlas->GetAllKeyFrames(); // 获取全部关键帧
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId); // 按关键帧ID升序排序

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // 遍历所有关键帧
  for (auto pKF : vpKFs) {
    if (pKF->isBad()) continue; // 被剔除的坏关键帧跳过不输出

    Sophus::SE3f Twc = pKF->GetPoseInverse(); // Twc世界到相机位姿
    Eigen::Quaternionf q = Twc.unit_quaternion(); // 旋转四元数
    Eigen::Vector3f t = Twc.translation();        // 平移向量
    // TUM格式输出：时间戳 x y z qx qy qz qw
    f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0)
      << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " "
      << q.z() << " " << q.w() << endl;
  }

  f.close();
}

/**
 * @brief 保存普通帧轨迹 EuRoC数据集格式，时间单位纳秒，IMU模式输出世界到机体Twb，非IMU输出Twc
 * @param filename 输出文件路径
 * @note 自动选择Atlas内关键帧数量最多的子地图作为参考地图；跟踪丢失帧跳过；时间戳*1e9转为纳秒
 */
void System::SaveTrajectoryEuRoC(const string &filename) {
  oslog::info("Saving trajectory to {} ...", filename);

  /*if(sensorType()==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  endl; return;
  }*/

  vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps(); // 获取Atlas全部子地图
  size_t numMaxKFs = 0;                                        // 记录最多关键帧数目
  std::shared_ptr<Map> pBiggerMap;                             // 保存关键帧最多的地图
  oslog::debug("There are {} maps in the atlas", std::to_string(vpMaps.size()));
  // 遍历所有子地图，挑选关键帧数量最大的地图作为输出参考地图
  for (auto pMap : vpMaps) {
    oslog::debug("  Map {} has {} KFs", std::to_string(pMap->GetId()),
                 std::to_string(pMap->GetAllKeyFrames().size()));
    if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  auto vpKFs = pBiggerMap->GetAllKeyFrames(); // 获取最大地图下全部关键帧
  if (vpKFs.size() == 0) {
    oslog::error("Cannot save EUROC trajectory, there are no Keyframes");
    return;
  }

  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId); // 关键帧按ID升序排序

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb;  // Can be word to cam0 or world to b depending on IMU or not.
  // IMU传感器模式：Twb取第一关键帧世界到机体b位姿；无IMU取世界到相机Twc
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
  auto lRit = mpTracker->mlpReferences.begin();                // 参考关键帧迭代器
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin(); // 帧时间戳迭代器
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();       // 跟踪丢失标记迭代器

  // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
  // cout << "size mlRelativeFramePoses: " <<
  // mpTracker->mlRelativeFramePoses.size() << endl; cout << "size
  // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl; cout
  // << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

  // 遍历每一帧相对位姿
  for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    if (*lbL) continue;                     // 跟踪丢失，跳过该帧

    std::shared_ptr<KeyFrame> pKF = *lRit;  // 当前帧对应的参考关键帧

    Sophus::SE3f Trw;                       // 回溯生成树累积变换

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF) continue;                     // 参考关键帧为空直接跳过

    // 参考关键帧被剔除，沿着生成树向上回溯父关键帧累积mTcp
    while (pKF->isBad()) {
      // cout << " 2.bad" << endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // cout << "--Parent KF: " << pKF->mnId << endl;
    }

    // 回溯得到的有效关键帧不属于我们选定的最大地图，则跳过本帧
    if (!pKF || pKF->GetMap() != pBiggerMap) {
      // cout << "--Parent KF is from another map" << endl;
      continue;
    }

    // Trw = 回溯累积变换 * 参考关键帧位姿 * Twb(第一帧置原点变换)
    Trw = Trw * pKF->GetPose() *
          Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    // IMU传感器模式，输出世界到机体Twb；无IMU输出世界到相机Twc
    if (sensorType().isImu()) {
      Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      // EuRoC格式：纳秒时间戳 x y z qx qy qz qw
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

/**
 * @brief 指定某一个子地图，保存该地图对应的普通帧EuRoC轨迹
 * @param filename 输出文件路径
 * @param pMap 指定需要输出轨迹的子地图指针
 */
void System::SaveTrajectoryEuRoC(const string &filename,
                                 const std::shared_ptr<Map> &pMap) {
  oslog::info("Saving trajectory of map {} to {} ...", pMap->GetId(), filename);

  /*if(sensorType()==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  endl; return;
  }*/

  auto vpKFs = pMap->GetAllKeyFrames();         // 获取指定地图的全部关键帧
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId); // 按ID升序排序

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb;  // Can be word to cam0 or world to b dependingo on IMU or not.
  // IMU模式取世界到机体位姿；非IMU取世界到相机位姿
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
  auto lRit = mpTracker->mlpReferences.begin();                // 参考关键帧迭代器
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin(); // 帧时间戳迭代器
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();       // 跟踪丢失标记迭代器

  // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
  // cout << "size mlRelativeFramePoses: " <<
  // mpTracker->mlRelativeFramePoses.size() << endl; cout << "size
  // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl; cout
  // << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

  // 遍历全部帧相对位姿
  for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    // cout << "1" << endl;
    if (*lbL) continue;                     // 跟踪丢失跳过

    std::shared_ptr<KeyFrame> pKF = *lRit;  // 获取参考关键帧

    Sophus::SE3f Trw;                       // 生成树回溯累积变换

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF) continue;                     // 参考关键帧为空直接跳过

    // 参考关键帧被剔除，沿着生成树向上回溯父关键帧累积mTcp
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // cout << "--Parent KF: " << pKF->mnId << endl;
    }

    // 回溯得到的关键帧不属于传入的pMap地图，跳过本帧
    if (!pKF || pKF->GetMap() != pMap) {
      // cout << "--Parent KF is from another map" << endl;
      continue;
    }

    // 累积变换，把第一关键帧置为世界原点
    Trw = Trw * pKF->GetPose() *
          Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    // IMU模式输出机体位姿Twb；非IMU输出相机位姿Twc
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

/**
 * @brief EuRoC格式保存关键帧轨迹，自动选取Atlas内关键帧最多的子地图
 * @param filename 输出文件路径；时间戳转为纳秒
 */
void System::SaveKeyFrameTrajectoryEuRoC(const string &filename) {
  oslog::info("Saving keyframe trajectory to {} ...", filename);

  vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps(); // 获取Atlas全部子地图
  std::shared_ptr<Map> pBiggerMap;
  size_t numMaxKFs = 0;
  // 挑选关键帧数量最多的子地图
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

  auto vpKFs = pBiggerMap->GetAllKeyFrames(); // 获取该地图全部关键帧
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId); // 按ID升序排序

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // 遍历所有关键帧
  for (auto pKF : vpKFs) {
    // pKF->SetPose(pKF->GetPose()*Two);

    if (!pKF || pKF->isBad()) continue; // 空指针或者被剔除的坏关键帧跳过
    if (sensorType().isImu()) {
      Sophus::SE3f Twb = pKF->GetImuPose();          // IMU模式获取世界到机体位姿
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();      // 非IMU获取世界到相机位姿
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << endl;
    }
  }
  f.close();
}

/**
 * @brief EuRoC格式，输出指定pMap子地图的关键帧轨迹
 * @param filename 输出文件路径
 * @param pMap 指定输出轨迹的子地图
 */
void System::SaveKeyFrameTrajectoryEuRoC(const string &filename,
                                         const std::shared_ptr<Map> &pMap) {
  oslog::info("Saving keyframe trajectory of map {} to {} ...", pMap->GetId(),
              filename);

  auto vpKFs = pMap->GetAllKeyFrames(); // 获取指定地图全部关键帧
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId); // 按ID升序排序

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // 遍历关键帧
  for (auto pKF : vpKFs) {
    if (!pKF || pKF->isBad()) continue; // 坏关键帧跳过

    if (sensorType().isImu()) {
      Sophus::SE3f Twb = pKF->GetImuPose();          // IMU模式：世界到机体b
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();      // 非IMU模式：世界到相机
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << endl;
    }
  }

  f.close();
}

/**
 * @brief 保存普通帧轨迹 KITTI数据集格式；输出3×4位姿矩阵；没有时间戳列；单目不可用
 * @param filename 输出文件路径
 */
void System::SaveTrajectoryKITTI(const string &filename) {
  oslog::info("Saving camera trajectory to {} ...", filename);
  if (sensorType() == SensorType::MONOCULAR) {
    oslog::error("ERROR: SaveTrajectoryKITTI cannot be used for monocular.");
    return;
  }

  auto vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId); // 关键帧按ID升序

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse(); // 第一帧置原点变换

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  auto lRit = mpTracker->mlpReferences.begin();                // 参考关键帧迭代器
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin(); // 帧时间戳迭代器
  // 遍历每一帧相对位姿
  for (list<Sophus::SE3f>::iterator
           lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++) {
    auto pKF = *lRit;

    Sophus::SE3f Trw; // 生成树回溯累积变换

    if (!pKF) continue; // 参考关键帧为空跳过

    // 参考关键帧被剔除，向上回溯父关键帧累积mTcp
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Tow;

    Sophus::SE3f Tcw = (*lit) * Trw;
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix3f Rwc = Twc.rotationMatrix(); // 旋转矩阵3×3
    Eigen::Vector3f twc = Twc.translation();     // 平移向量

    // KITTI格式输出3行4列矩阵，一行输出：r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz
    f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2)
      << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " "
      << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1)
      << " " << Rwc(2, 2) << " " << twc(2) << endl;
  }
  f.close();
}

/**
 * @brief IMU初始化阶段调试数据保存函数，输出各类初始化中间结果到本地txt文件
 * @param initIdx 当前初始化轮次索引，用于区分多次初始化输出文件
 */
void System::SaveDebugData(const int &initIdx) {
  // 0. Save initialization trajectory
  // 保存初始化阶段帧轨迹EuRoC格式，文件名带上初始化段编号与initIdx
  SaveTrajectoryEuRoC("init_FrameTrajectoy_" +
                      to_string(mpLocalMapper->mInitSect) + "_" +
                      to_string(initIdx) + ".txt");

  // 1. Save scale
  ofstream f;
  // ios_base::app 追加模式打开文件，不覆盖原有内容
  f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mScale << endl; // 输出IMU初始化求解得到尺度
  f.close();

  // 2. Save gravity direction
  f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  // 输出重力旋转矩阵mRwg，按行输出，逗号分隔
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
  f << mpLocalMapper->mCostTime << endl; // 输出初始化耗时
  f.close();

  // 4. Save biases
  f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  // 输出陀螺仪bias mbg，逗号分隔
  f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << ","
    << mpLocalMapper->mbg(2) << endl;
  // 输出加速度计bias mba，逗号分隔
  f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << ","
    << mpLocalMapper->mba(2) << endl;
  f.close();

  // 5. Save covariance matrix
  f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" +
              to_string(initIdx) + ".txt",
          ios_base::app);
  f << fixed;
  // 遍历惯性初始化协方差矩阵，逗号分隔每个元素
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
  f << mpLocalMapper->mInitTime << endl; // 输出IMU初始化总时间
  f.close();
}

}  // namespace ORB_SLAM3
