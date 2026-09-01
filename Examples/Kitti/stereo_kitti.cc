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
#include <algorithm>                    // STL算法库，sort排序函数
#include <chrono>                       // C++高精度时钟，用于统计各阶段耗时
#include <fstream>                      // 文件读写流，读取KITTI times.txt时间戳
#include <iomanip>                      // IO格式控制，setfill、setw实现文件名6位补零
#include <iostream>                     // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>        // OpenCV核心模块，Mat图像容器

using namespace std;

/**
 * @brief 加载KITTI双目数据集：左右图像路径、时间戳
 * @param strPathToSequence KITTI数据集序列根目录
 * @param vstrImageLeft 输出，左目图像完整路径数组
 * @param vstrImageRight 输出，右目图像完整路径数组
 * @param vTimestamps 输出，每帧对应的时间戳数组
 */
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

/**
 * @brief STEREO双目模式运行KITTI数据集主函数入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char **argv) {
    // 校验命令行参数，需要3个入参：词袋路径、yaml配置、数据集序列路径
    if (argc != 4) {
        cerr << endl
             << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings "
                "path_to_sequence"
             << endl;
        return 1;
    }

    // Retrieve paths to images 读取图像路径与时间戳
    vector<string> vstrImageLeft;      // 存放左目图像完整路径
    vector<string> vstrImageRight;     // 存放右目图像完整路径
    vector<double> vTimestamps;        // 存放每帧时间戳
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size(); // 获取序列总帧数

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM实例；传感器STEREO双目，true开启多数据集标记
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::STEREO, true);

    // 判断SLAM初始化是否失败，打印错误信息退出
    if (!exSLAM) {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM系统有效智能指针

    float imageScale = SLAM->GetImageScale(); // 从配置读取图像缩放系数

    // Vector for tracking time statistics 跟踪耗时统计容器
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_track = 0.f;  // 单帧总跟踪耗时，单位ms，REGISTER_TIMES宏启用才统计
    double t_resize = 0.f; // 图像缩放耗时，单位ms，REGISTER_TIMES宏启用才统计

    // Main loop 主循环，逐帧处理双目图像
    cv::Mat imLeft, imRight; // 存放读取的左、右目图像
    for (int ni = 0; ni < nImages; ni++) {
        // Read left and right images from file 从磁盘读取双目图像
        imLeft = cv::imread(vstrImageLeft[ni],
                            cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],
                             cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni]; // 当前帧时间戳

        // 左目图像读取失败，打印错误返回
        if (imLeft.empty()) {
            cerr << endl
                 << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        // 如果缩放系数不等于1，对左右图像做缩放
        if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
            // 记录缩放操作开始时间点
            std::chrono::steady_clock::time_point t_Start_Resize =
                std::chrono::steady_clock::now();
#endif
            int width = imLeft.cols * imageScale;  // 缩放后图像宽度
            int height = imLeft.rows * imageScale; // 缩放后图像高度
            cv::resize(imLeft, imLeft, cv::Size(width, height));  // 缩放左图
            cv::resize(imRight, imRight, cv::Size(width, height)); // 缩放右图

#ifdef REGISTER_TIMES
            // 记录缩放操作结束时间点
            std::chrono::steady_clock::time_point t_End_Resize =
                std::chrono::steady_clock::now();
            // 计算缩放耗时，单位毫秒
            t_resize = std::chrono::duration_cast<
                           std::chrono::duration<double, std::milli> >(t_End_Resize -
                                                                       t_Start_Resize)
                           .count();
            SLAM->InsertResizeTime(t_resize); // 将缩放耗时送入SLAM内部统计
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif

        // Pass the images to the SLAM system 向SLAM送入双目图像，执行双目跟踪
        SLAM->TrackStereo(imLeft, imRight, tframe);

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

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

        vTimesTrack[ni] = ttrack; // 保存本帧耗时到统计数组

        // Wait to load the next frame 计算休眠时间，模拟真实相机帧率
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe; // 当前帧与下一帧时间差
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1]; // 最后一帧取和上一帧时间差

        if (ttrack < T) usleep((T - ttrack) * 1e6); // 处理速度快于帧间隔则休眠，usleep单位微秒
    }

    // Stop all threads 序列处理完毕，关闭SLAM全部后台线程
    SLAM->Shutdown();

    // Tracking time statistics 跟踪耗时统计
    sort(vTimesTrack.begin(), vTimesTrack.end()); // 耗时数组升序排序
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni]; // 累加所有帧总耗时
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl; // 输出耗时中位数
    cout << "mean tracking time: " << totaltime / nImages << endl;       // 输出耗时平均值

    // Save camera trajectory 保存KITTI格式完整相机轨迹
    SLAM->SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

/**
 * @brief 读取KITTI数据集时间戳文件、拼接左右图像完整路径
 * @param strPathToSequence KITTI序列根目录
 * @param vstrImageLeft [out]左目图像路径列表
 * @param vstrImageRight [out]右目图像路径列表
 * @param vTimestamps [out]时间戳列表
 */
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt"; // 拼接时间戳文件路径
    fTimes.open(strPathTimeFile.c_str()); // 打开时间戳文本文件
    while (!fTimes.eof()) { // 循环读取直到文件末尾
        string s;
        getline(fTimes, s); // 读取一行字符串
        if (!s.empty()) { // 跳过空行
            stringstream ss;
            ss << s;
            double t;
            ss >> t; // 解析浮点数时间戳
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/"; // 左目图像文件夹前缀
    string strPrefixRight = strPathToSequence + "/image_1/";// 右目图像文件夹前缀

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    // 拼接6位补零png文件名，KITTI命名格式000000.png
    for (int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
}
