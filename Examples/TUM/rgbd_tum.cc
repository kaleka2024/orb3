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
#include <System.h>                    // ORB‑SLAM3系统主头文件，SystemFactory工厂接口
#include <algorithm>                   // STL算法库，sort用于耗时统计数组排序
#include <chrono>                      // C++高精度时钟库，统计每一帧SLAM处理耗时
#include <fstream>                     // 文件输入流，读取TUM数据集association关联文件
#include <iostream>                    // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>       // OpenCV核心模块，Mat图像容器、图像读写缩放接口

using namespace std;

/**
 * @brief 解析TUM数据集association关联文件，读取RGB图像路径、深度图路径、时间戳
 * @param strAssociationFilename association.txt关联文件路径
 * @param vstrImageFilenamesRGB 输出，RGB图像相对路径数组
 * @param vstrImageFilenamesD 输出，深度图相对路径数组
 * @param vTimestamps 输出，帧时间戳数组，单位秒
 */
void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps);

/**
 * @brief RGBD模式运行TUM数据集主函数，读取RGB+深度图，执行RGBD‑SLAM
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常退出，非0异常退出
 */
int main(int argc, char **argv)
{
    // 校验命令行参数，共5个参数：程序、词袋路径、yaml配置、数据集目录、association文件
    if (argc != 5)
    {
        cerr << endl
             << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings "
                "path_to_sequence path_to_association"
             << endl;
        return 1;
    }

    // Retrieve paths to images 读取RGB、深度图路径与对应时间戳
    vector<string> vstrImageFilenamesRGB;   // RGB图像相对路径
    vector<string> vstrImageFilenamesD;     // 深度图相对路径
    vector<double> vTimestamps;             // 每一帧对应的时间戳
    string strAssociationFilename = string(argv[4]); // 获取association.txt文件路径
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD,
               vTimestamps);

    // Check consistency in the number of images and depthmaps 校验RGB与深度图数量一致性
    int nImages = vstrImageFilenamesRGB.size(); // 获取RGB图像总帧数
    if (vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 构造SLAM系统实例，传感器类型RGBD，true开启多数据集模式
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::RGBD, true);

    // 判断SLAM系统初始化是否失败，打印错误信息并退出程序
    if (!exSLAM)
    {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM系统有效智能指针

    float imageScale = SLAM->GetImageScale(); // 从yaml配置读取图像缩放系数

    // Vector for tracking time statistics 跟踪耗时统计数组，大小等于图像总帧数
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop 数据集主循环，逐帧读取RGB图像与深度图送入SLAM
    cv::Mat imRGB, imD; // imRGB存放彩色图像，imD存放深度图像
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read image and depthmap from file 读取RGB图像，IMREAD_UNCHANGED保持原始通道与位深
        imRGB = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesRGB[ni],
                           cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
        // 读取深度图，TUM深度图为16位uint16
        imD = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesD[ni],
                         cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni]; // 当前帧的时间戳

        // RGB图像读取失败，打印路径并返回异常
        if (imRGB.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << string(argv[3]) << "/"
                 << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        // 如果配置缩放系数不等于1，同步缩放RGB图像与深度图
        if (imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height)); // 缩放彩色图
            cv::resize(imD, imD, cv::Size(width, height));     // 同步缩放深度图
        }

        // 记录帧处理开始时间点
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system 将RGB图像、深度图、时间戳送入SLAM执行RGBD跟踪
        SLAM->TrackRGBD(imRGB, imD, tframe);

        // 记录帧处理结束时间点
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        // 计算本帧SLAM处理耗时，单位秒
        double ttrack =
            std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                .count();

        vTimesTrack[ni] = ttrack; // 保存本帧耗时到统计数组

        // Wait to load the next frame 模拟真实相机帧率，处理速度快则休眠补齐时间间隔
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe; // 非最后一帧，取当前帧与下一帧时间差
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1]; // 最后一帧，取和上一帧时间差

        if (ttrack < T) usleep((T - ttrack) * 1e6); // usleep入参单位微秒，秒转微秒乘1e6
    }

    // Stop all threads 全部帧处理完毕，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Tracking time statistics 跟踪耗时统计
    sort(vTimesTrack.begin(), vTimesTrack.end()); // 耗时数组升序排序，用于求取中位数
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni]; // 累加所有帧跟踪耗时
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl; // 输出跟踪耗时中位数
    cout << "mean tracking time: " << totaltime / nImages << endl;       // 输出跟踪耗时平均值

    // Save camera trajectory 保存相机完整轨迹与关键帧轨迹，TUM文本格式
    SLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

/**
 * @brief 读取TUM数据集association.txt关联文件，解析时间戳、RGB路径、深度图路径
 * @param strAssociationFilename association.txt文件路径
 * @param vstrImageFilenamesRGB [out]RGB图像相对路径输出数组
 * @param vstrImageFilenamesD [out]深度图相对路径输出数组
 * @param vTimestamps [out]时间戳输出数组，单位秒
 */
void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str()); // 打开association关联文件
    while (!fAssociation.eof()) // 循环读取每一行直到文件末尾
    {
        string s;
        getline(fAssociation, s);
        if (!s.empty()) // 跳过空行
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;                // 解析RGB帧时间戳
            vTimestamps.push_back(t);
            ss >> sRGB;             // 解析RGB图像相对路径
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;                // 解析深度图时间戳（不使用，仅占位读取）
            ss >> sD;               // 解析深度图相对路径
            vstrImageFilenamesD.push_back(sD);
        }
    }
}
