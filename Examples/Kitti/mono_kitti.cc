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
#include <algorithm>                    // STL算法库，sort排序函数
#include <chrono>                       // C++高精度时钟，用于统计各阶段耗时
#include <fstream>                      // 文件读写流，读取KITTI的times.txt时间戳文件
#include <iomanip>                      // IO格式控制，setfill、setw用于图片文件名补零
#include <iostream>                     // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>        // OpenCV核心模块，Mat图像结构

#include "System.h"                     // ORB‑SLAM3系统主头文件，SystemFactory工厂接口

using namespace std;

/**
 * @brief 加载KITTI数据集图像路径与对应时间戳
 * @param strSequence KITTI数据集序列根目录路径
 * @param vstrImageFilenames 输出，图像完整文件路径数组
 * @param vTimestamps 输出，图像对应的时间戳数组
 */
void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

/**
 * @brief MONOCULAR单目模式运行KITTI数据集主程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char **argv) {
    // 校验命令行参数，必须传入3个参数：词袋路径、配置文件路径、数据集序列路径
    if (argc != 4) {
        cerr << endl
             << "Usage: ./mono_kitti path_to_vocabulary path_to_settings "
                "path_to_sequence"
             << endl;
        return 1;
    }

    // Retrieve paths to images 读取图像路径和时间戳
    vector<string> vstrImageFilenames; // 保存每张图像完整路径
    vector<double> vTimestamps;        // 保存每张图像对应的时间戳
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size(); // 获取序列图像总数量

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM系统实例；传感器类型MONOCULAR单目，true开启多数据集模式标记
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::MONOCULAR, true);

    // 判断SLAM系统是否初始化失败，打印错误信息后退出
    if (!exSLAM) {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM系统有效智能指针

    float imageScale = SLAM->GetImageScale(); // 从配置文件读取图像缩放比例系数

    // Vector for tracking time statistics 跟踪耗时统计容器，存储每帧跟踪耗时
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop 主循环，逐帧处理序列中所有图像
    double t_resize = 0.f; // 图像缩放耗时，单位毫秒，REGISTER_TIMES宏生效时统计
    double t_track = 0.f; // 单帧总处理耗时，单位毫秒，REGISTER_TIMES宏生效时统计

    cv::Mat im; // 存储读取到的单目图像
    for (int ni = 0; ni < nImages; ni++) {
        // Read image from file 从磁盘读取图像
        im = cv::imread(vstrImageFilenames[ni],
                        cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni]; // 当前帧对应的时间戳

        // 图像读取失败，打印错误信息并退出
        if (im.empty()) {
            cerr << endl
                 << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        // 如果配置的缩放系数不等于1，执行图像缩放
        if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
            // 记录缩放操作开始时刻
            std::chrono::steady_clock::time_point t_Start_Resize =
                std::chrono::steady_clock::now();
#endif
            int width = im.cols * imageScale;  // 缩放后图像宽度
            int height = im.rows * imageScale; // 缩放后图像高度
            cv::resize(im, im, cv::Size(width, height)); // 原地缩放图像

#ifdef REGISTER_TIMES
            // 记录缩放操作结束时刻
            std::chrono::steady_clock::time_point t_End_Resize =
                std::chrono::steady_clock::now();
            // 计算缩放耗时，单位毫秒
            t_resize = std::chrono::duration_cast<
                           std::chrono::duration<double, std::milli> >(t_End_Resize -
                                                                       t_Start_Resize)
                           .count();
            SLAM->InsertResizeTime(t_resize); // 将缩放耗时送入SLAM内部统计模块
#endif
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now(); // 跟踪开始时刻

        // Pass the image to the SLAM system
        // 向SLAM传入图像、时间戳、空IMU数组、图像路径，执行单目跟踪
        SLAM->TrackMonocular(im, tframe, vector<ORB_SLAM3::IMU::Point>(),
                             vstrImageFilenames[ni]);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now(); // 跟踪结束时刻

#ifdef REGISTER_TIMES
        // 开启计时宏时，统计缩放+跟踪总耗时，送入SLAM统计模块
        t_track =
            t_resize +
            std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(
                t2 - t1)
                .count();
        SLAM->InsertTrackTime(t_track);
#endif

        // 计算本帧跟踪耗时，单位秒
        double ttrack =
            std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                .count();

        vTimesTrack[ni] = ttrack; // 保存本帧跟踪耗时到统计数组

        // Wait to load the next frame 计算需要休眠时长，模拟真实相机帧率
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe; // 取当前帧与下一帧时间差
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1]; // 最后一帧取和上一帧的时间差

        if (ttrack < T) usleep((T - ttrack) * 1e6); // 处理速度快于帧间隔则休眠补齐时间，usleep入参为微秒
    }

    // Stop all threads 序列全部处理完成，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Tracking time statistics 跟踪耗时统计计算
    sort(vTimesTrack.begin(), vTimesTrack.end()); // 对所有帧耗时做升序排序
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni]; // 累加全部帧跟踪总耗时
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl; // 输出跟踪耗时中位数
    cout << "mean tracking time: " << totaltime / nImages << endl;       // 输出跟踪耗时平均值

    // Save camera trajectory 保存关键帧轨迹，输出TUM格式文件
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

/**
 * @brief 加载KITTI数据集的时间戳与图像文件名
 * @param strPathToSequence KITTI序列根目录
 * @param vstrImageFilenames [out]图像完整路径列表
 * @param vTimestamps [out]时间戳列表
 */
void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps) {
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt"; // 拼接时间戳文件完整路径
    fTimes.open(strPathTimeFile.c_str()); // 打开时间戳文件
    while (!fTimes.eof()) { // 循环读取直到文件末尾
        string s;
        getline(fTimes, s); // 读取一行文本
        if (!s.empty()) { // 非空行才解析
            stringstream ss;
            ss << s;
            double t;
            ss >> t; // 解析得到时间戳
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/"; // 左目图像文件夹路径前缀

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    // 拼接6位补零png图像文件名，KITTI命名格式000000.png
    for (int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i; // 设置宽度6位，不足位置补0
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
