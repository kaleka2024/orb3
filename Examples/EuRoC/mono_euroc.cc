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
#include <algorithm>                    // STL算法库
#include <chrono>                       // C++高精度时钟，统计耗时
#include <fstream>                      // 文件读写
#include <iostream>                     // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>        // OpenCV核心Mat、Size等基础结构

#include "euroc_common.h"               // EuRoC数据集加载工具头文件

using namespace std;

/**
 * @brief monocular模式运行EuRoC数据集的主程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char **argv) {
    setupEurocSpdLogger();              // 初始化spdlog日志器，配置控制台彩色输出
    spdlog::set_level(spdlog::level::info); // 设置日志输出等级为info，info及以上才打印

    // 命令行参数合法性校验，参数不足打印使用帮助
    if (argc < 5) {
        cerr << endl
             << "Usage: ./mono_euroc path_to_vocabulary path_to_settings "
             << std::endl
             << "path_to_image_folder_1 path_to_times_file_1 " << std::endl
             << "[path_to_image_folder_2 path_to_times_file_2] ... " << std::endl
             << "[path_to_image_folder_N path_to_times_file_N] "
                "[trajectory_file_name]"
             << endl;
        return 1;
    }

    // 计算输入的数据集序列数量，argv[1]词袋，argv[2]配置文件，后面每两个参数对应一组序列
    const int num_seq = (argc - 3) / 2;
    cout << "num_seq = " << num_seq << endl;

    // 判断最后一个参数是否为轨迹输出文件名：(argc‑3)为剩余参数总数，奇数代表末尾多一个文件名
    bool bFileName = (((argc - 3) % 2) == 1);
    string trajFileName;
    if (bFileName) {
        trajFileName = string(argv[argc - 1]); // 取出轨迹输出文件名
        cout << "file name: " << trajFileName << endl;
    }

    // Load all sequences: 准备所有序列路径配置
    int seq;
    vector<EuRoCData::SequencePaths> imagePaths; // 存放每一条数据集序列路径信息
    for (seq = 0; seq < num_seq; seq++) {
        cout << "Loading images for sequence " << seq << "..." << endl;

        // 每一组：图像文件夹路径、时间戳文件路径
        const string pathSeq(argv[(2 * seq) + 3]);
        const string pathTimeStamps(argv[(2 * seq) + 4]);

        imagePaths.emplace_back(pathSeq, pathTimeStamps);
    }

    // 调用静态接口加载全部EuRoC图像序列，此处不加载IMU数据
    auto eurocData = EuRoCData::LoadSequences(imagePaths);

    // Vector for tracking time statistics 跟踪耗时统计容器，保存每帧跟踪耗时
    vector<float> vTimesTrack(eurocData.totalImages());

    cout << endl << "-------" << endl;
    cout.precision(17); // 设置浮点数输出精度17位，保证时间戳完整打印

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM系统实例，启动所有后台线程
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::MONOCULAR, false);

    // 判断SLAM系统是否初始化失败
    if (!exSLAM) {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM系统有效指针

    float imageScale = SLAM->GetImageScale(); // 获取配置文件设定的图像缩放系数
    double t_resize = 0.f;                   // 图像resize耗时(毫秒)
    double t_track = 0.f;                    // 单帧总处理耗时(毫秒)

    int nseq = 0; // 当前处理的序列编号
    for (auto const &seq : eurocData.mvSequences) { // 遍历每一条数据集序列
        if (nseq > 0) {
            cout << "Changing the dataset" << endl;
            SLAM->ChangeDataset(); // 切换数据集，用于多序列连续运行
        }

        // Main loop 单条序列内部遍历每一帧图像
        cv::Mat im;
        size_t ni = 0; // 当前序列内帧索引
        for (auto const &imgSet : seq.mvImageSets) {
            cout << "=== Processing image " << ni << " of " << seq.size() << " at "
                 << imgSet.mTimestamp << " ===" << endl;

            // Read image from file 读取左目图像
            im = imgSet.leftImage();

            // 图像读取失败直接退出程序
            if (im.empty()) {
                cerr << endl
                     << "Failed to load image at: " << imgSet.mTimestamp << endl;
                exit(-1);
            }

            // 如果图像缩放系数不等于1，执行图像缩放
            if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
                // 开启计时宏时，记录resize开始时间点
                std::chrono::steady_clock::time_point t_Start_Resize =
                    std::chrono::steady_clock::now();
#endif
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height)); // 执行图像缩放

#ifdef REGISTER_TIMES
                // 记录resize结束时间点，计算resize耗时并送入SLAM统计模块
                std::chrono::steady_clock::time_point t_End_Resize =
                    std::chrono::steady_clock::now();

                t_resize = std::chrono::duration_cast<
                               std::chrono::duration<double, std::milli> >(
                               t_End_Resize - t_Start_Resize)
                               .count();
                SLAM->InsertResizeTime(t_resize);
#endif
            }

            // 记录跟踪开始时刻
            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now();

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            SLAM->TrackMonocular(
                im, imgSet.mTimestamp);  // TODO change to monocular_inertial 输入单目图像与时间戳执行跟踪

            // 记录跟踪结束时刻
            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
            // 开启计时宏，统计resize+跟踪总耗时，送入SLAM统计模块
            t_track =
                t_resize + std::chrono::duration_cast<
                               std::chrono::duration<double, std::milli> >(t2 - t1)
                               .count();
            SLAM->InsertTrackTime(t_track);
#endif

            // 计算本帧跟踪耗时，单位秒
            double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                    .count();

            vTimesTrack[ni] = ttrack; // 保存本帧跟踪耗时

            // 模拟真实相机帧率，如果处理耗时小于帧间隔，则usleep休眠补齐时间
            const double dt = seq.dt();
            if (ttrack < dt) usleep((dt - ttrack) * 1e6);

            ni++;
        }

        // 当前序列跑完，保存该子地图对应的完整轨迹与关键帧轨迹到SubMaps目录
        string kf_file_submap =
            "./SubMaps/kf_SubMap_" + std::to_string(nseq) + ".txt";
        string f_file_submap =
            "./SubMaps/f_SubMap_" + std::to_string(nseq) + ".txt";
        SLAM->SaveTrajectoryEuRoC(f_file_submap);        // 保存完整轨迹(EuRoC格式)
        SLAM->SaveKeyFrameTrajectoryEuRoC(kf_file_submap);// 保存关键帧轨迹(EuRoC格式)

        nseq++;
    }
    // Stop all threads 全部序列处理完毕，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Save camera trajectory 保存全局输出轨迹，TUM格式
    if (bFileName) {
        // 用户指定输出文件名
        const string kf_file = "kf_" + trajFileName + ".txt";
        const string f_file = "f_" + trajFileName + ".txt";
        SLAM->SaveTrajectoryTUM(f_file);
        SLAM->SaveKeyFrameTrajectoryTUM(kf_file);
    } else {
        // 未指定文件名，使用默认文件名
        SLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
        SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    return 0;
}
