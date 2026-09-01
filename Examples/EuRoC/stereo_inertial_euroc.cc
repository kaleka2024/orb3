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

#include <System.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "euroc_common.h"

using namespace std;

int main(int argc, char **argv) {
  setupEurocSpdLogger();
  spdlog::set_level(spdlog::level::info);

  if (argc < 5) {
    cerr << endl
         << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings "
         << std::endl
         << "path_to_image_folder_1 path_to_times_file_1 " << std::endl
         << "[path_to_image_folder_2 path_to_times_file_2] ... " << std::endl
         << "[path_to_image_folder_N path_to_times_file_N] "
            "[trajectory_file_name]"
         << endl;
    exit(-1);
  }

  const int num_seq = (argc - 3) / 2;
  cout << "num_seq = " << num_seq << endl;
  bool bFileName = (((argc - 3) % 2) == 1);
  string trajFileName;
  if (bFileName) {
    trajFileName = string(argv[argc - 1]);
    cout << "file name: " << trajFileName << endl;
  }

  // Load all sequences:
  int seq;
  vector<EuRoCData::SequencePaths> imagePaths;
  for (seq = 0; seq < num_seq; seq++) {
    cout << "Loading images for sequence " << seq << "..." << endl;

    const string pathSeq(argv[(2 * seq) + 3]);
    const string pathTimeStamps(argv[(2 * seq) + 4]);

    imagePaths.emplace_back(pathSeq, pathTimeStamps);
  }

  auto eurocData = EuRoCData::LoadSequences(imagePaths);

  std::setprecision(17);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  auto exSLAM = ORB_SLAM3::SystemFactory::create(
      argv[1], argv[2], ORB_SLAM3::SensorType::IMU_STEREO, false);

  if (!exSLAM) {
    cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
    exit(-1);
  }

  auto SLAM = exSLAM.value();

  cv::Mat imLeft, imRight;
  size_t nseq = 0;
  for (auto const &seq : eurocData.mvSequences) {
    if (nseq > 0) {
      spdlog::info("Changing dataset");
      SLAM->ChangeDataset();
    }

    // Seq loop
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double t_rect = 0.f;
    double t_resize = 0.f;
    double t_track = 0.f;
    int ni = 0;

    auto imuIt = seq.vImuMeas.begin();

    for (auto const &imgSet : seq.mvImageSets) {
      cout << "=== Processing image " << ni << " of " << seq.size() << " at "
           << imgSet.mTimestamp << " ===" << endl;

      // Read left and right images from file
      imLeft = imgSet.leftImage();
      imRight = imgSet.rightImage();

      if (imLeft.empty()) {
        cerr << endl
             << "Failed to load image at: " << imgSet.mTimestamp << endl;
        return 1;
      }

      if (imRight.empty()) {
        cerr << endl
             << "Failed to load image at: " << imgSet.mTimestamp << endl;
        return 1;
      }

      // Load imu measurements from previous frame
      vImuMeas.clear();

      if (ni > 0) {
        // Retain the deep copy for now...

        cout << "IMU data starts at " << imuIt->mTimestamp << endl;

        auto imuEnd =
            std::find_if(imuIt, seq.vImuMeas.end(),
                         [&](const ORB_SLAM3::IMU::Point &pt) -> bool {
                           return pt.t > imgSet.mTimestamp;
                         });

        std::copy(imuIt, imuEnd, std::back_inserter(vImuMeas));
        imutIt = imuEnd;

        cout << "Pushed " << vImuMeas.size() << " IMU measurements into queue"
             << endl;
      }

      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();

      // Pass the images to the SLAM system
      SLAM->TrackStereo(imLeft, imRight, imgSet.mTimestamp, vImuMeas);

      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
      t_track = t_rect + t_resize +
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli> >(t2 - t1)
                    .count();
      SLAM->InsertTrackTime(t_track);
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
              .count();

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages[seq] - 1)
        T = vTimestampsCam[seq][ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestampsCam[seq][ni - 1];

      if (ttrack < T) usleep((T - ttrack) * 1e6);  // 1e6

      ni++;
    }

    if (seq < num_seq - 1) {
      cout << "Changing the dataset" << endl;

      SLAM->ChangeDataset();
    }

    nseq++;
  }
  // Stop all threads
  SLAM->Shutdown();

  // Save camera trajectory
  if (bFileName) {
    const string kf_file = "kf_" + trajFileName + ".txt";
    const string f_file = "f_" + trajFileName + ".txt";
    SLAM->SaveTrajectoryTUM(f_file);
    SLAM->SaveKeyFrameTrajectoryTUM(kf_file);
  } else {
    SLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }

  return 0;
}
