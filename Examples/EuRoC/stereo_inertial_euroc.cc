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
#include <System.h>                     // ORB‑SLAM3系统主头文件，SystemFactory工厂接口
#include <algorithm>                    // STL算法库，find_if、copy、back_inserter
#include <chrono>                       // C++高精度时钟，统计各阶段耗时
#include <ctime>                        // C标准时间库
#include <fstream>                      // 文件读写流
#include <iomanip>                      // IO输出格式控制
#include <iostream>                     // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>        // OpenCV核心模块，Mat图像结构

#include "euroc_common.h"               // EuRoC数据集加载工具头文件

using namespace std;

/**
 * @brief IMU‑STEREO 双目+IMU模式运行EuRoC数据集主程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char **argv) {
    setupEurocSpdLogger();              // 初始化spdlog日志器，配置控制台彩色输出
    spdlog::set_level(spdlog::level::info); // 设置日志输出等级为info

    // 命令行参数校验，参数不足打印使用帮助并退出
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

    // 计算数据集序列数量，argv[1]词袋，argv[2]配置文件，后续每两个参数对应一组序列
    const int num_seq = (argc - 3) / 2;
    cout << "num_seq = " << num_seq << endl;
    // 判断末尾参数是否为轨迹输出文件名；剩余参数总数奇数代表附带输出文件名
    bool bFileName = (((argc - 3) % 2) == 1);
    string trajFileName;
    if (bFileName) {
        trajFileName = string(argv[argc - 1]); // 获取用户指定轨迹输出文件名
        cout << "file name: " << trajFileName << endl;
    }

    // Load all sequences: 组装全部序列路径配置
    int seq;
    vector<EuRoCData::SequencePaths> imagePaths; // 存放每条数据集序列路径信息
    for (seq = 0; seq < num_seq; seq++) {
        cout << "Loading images for sequence " << seq << "..." << endl;

        // 解析一组序列：图像文件夹路径、图像时间戳文件路径
        const string pathSeq(argv[(2 * seq) + 3]);
        const string pathTimeStamps(argv[(2 * seq) + 4]);

        imagePaths.emplace_back(pathSeq, pathTimeStamps);
    }

    // 加载EuRoC数据集图像序列，此处未传入true，默认不加载IMU
    auto eurocData = EuRoCData::LoadSequences(imagePaths);

    std::setprecision(17);              // 设置浮点数输出精度17位，保证时间戳完整打印

    // Vector for tracking time statistics 跟踪耗时统计容器
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);     // 根据总图像数调整容器大小，tot_images为外部变量

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM系统实例，传感器类型IMU_STEREO双目+IMU，false关闭多数据集标记
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::IMU_STEREO, false);

    // 校验SLAM系统初始化是否失败，打印错误信息后退出
    if (!exSLAM) {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value();         // 获取SLAM系统有效指针

    cv::Mat imLeft, imRight;            // 存放左目、右目图像
    size_t nseq = 0;                    // 当前处理的序列编号
    for (auto const &seq : eurocData.mvSequences) { // 遍历每一条数据集序列
        if (nseq > 0) {
            spdlog::info("Changing dataset");
            SLAM->ChangeDataset();     // 切换数据集接口，用于多序列连续运行
        }

        // Seq loop 单条序列内部循环处理每一帧双目图像
        vector<ORB_SLAM3::IMU::Point> vImuMeas; // 存放当前帧需要使用的IMU测量值
        double t_rect = 0.f;            // 双目校正耗时，本示例未实际使用
        double t_resize = 0.f;          // 图像缩放耗时，本示例未实际使用
        double t_track = 0.f;           // 单帧总处理耗时，单位毫秒
        int ni = 0;                     // 当前序列内部帧索引

        auto imuIt = seq.vImuMeas.begin(); // IMU迭代器，标记已经消费到哪个IMU样本

        for (auto const &imgSet : seq.mvImageSets) { // 遍历序列内每一组双目帧
            cout << "=== Processing image " << ni << " of " << seq.size() << " at "
                 << imgSet.mTimestamp << " ===" << endl;

            // Read left and right images from file 读取左、右目图像
            imLeft = imgSet.leftImage();
            imRight = imgSet.rightImage();

            // 左图读取失败，打印错误返回
            if (imLeft.empty()) {
                cerr << endl
                     << "Failed to load image at: " << imgSet.mTimestamp << endl;
                return 1;
            }

            // 右图读取失败，打印错误返回
            if (imRight.empty()) {
                cerr << endl
                     << "Failed to load image at: " << imgSet.mTimestamp << endl;
                return 1;
            }

            // Load imu measurements from previous frame 清空IMU容器，准备收集当前帧所需IMU数据
            vImuMeas.clear();

            if (ni > 0) { // 不是第0帧才去取IMU数据
                // Retain the deep copy for now...
                cout << "IMU data starts at " << imuIt->mTimestamp << endl;

                // 查找第一个时间戳大于当前图像帧时间的IMU点，作为IMU区间结束位置
                auto imuEnd =
                    std::find_if(imuIt, seq.vImuMeas.end(),
                                 [&](const ORB_SLAM3::IMU::Point &pt) -> bool {
                                     return pt.t > imgSet.mTimestamp;
                                 });
                // 将[imuIt, imuEnd)区间IMU样本拷贝到vImuMeas，即上一帧到当前帧之间IMU数据
                std::copy(imuIt, imuEnd, std::back_inserter(vImuMeas));
                imutIt = imuEnd; // 更新IMU迭代器，下一帧从此位置继续读取（源码变量名疑似笔误）

                cout << "Pushed " << vImuMeas.size() << " IMU measurements into queue"
                     << endl;
            }

            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now(); // 记录跟踪开始时刻

            // Pass the images to the SLAM system
            // 传入双目图像、时间戳、对应区间IMU测量，执行双目+IMU跟踪
            SLAM->TrackStereo(imLeft, imRight, imgSet.mTimestamp, vImuMeas);

            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now(); // 记录跟踪结束时刻

#ifdef REGISTER_TIMES
            // 开启REGISTER_TIMES宏时，统计校正+resize+跟踪总耗时，送入SLAM耗时统计模块
            t_track = t_rect + t_resize +
                      std::chrono::duration_cast<
                          std::chrono::duration<double, std::milli> >(t2 - t1)
                          .count();
            SLAM->InsertTrackTime(t_track);
#endif

            // 计算本帧跟踪耗时，单位秒
            double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                    .count();

            vTimesTrack[ni] = ttrack; // 保存本帧跟踪耗时到统计数组

            // Wait to load the next frame 计算需要休眠的时间，模拟真实相机帧率
            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe;
            else if (ni > 0)
                T = tframe - vTimestampsCam[seq][ni - 1];

            if (ttrack < T) usleep((T - ttrack) * 1e6);  // 1e6，处理速度快于帧间隔则休眠补齐
            ni++;
        }

        if (seq < num_seq - 1) {
            cout << "Changing the dataset" << endl;

            SLAM->ChangeDataset(); // 切换数据集
        }

        nseq++;
    }
    // Stop all threads 全部序列处理完毕，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Save camera trajectory 输出TUM格式轨迹文件
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
