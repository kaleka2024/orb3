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
#include <unistd.h>                    // Linux系统调用，usleep休眠函数
#include <algorithm>                   // STL算法库，sort用于耗时数组排序
#include <chrono>                      // C++高精度时钟，统计帧处理耗时
#include <fstream>                     // 文件输入流，读取TUM‑VI时间戳文本
#include <iomanip>                     // IO格式化输出，控制浮点数精度
#include <iostream>                    // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>       // OpenCV核心模块，Mat、CLAHE图像增强

#include "Converter.h"                 // ORB‑SLAM3格式转换工具头文件
#include "System.h"                    // ORB‑SLAM3系统主头文件，SystemFactory工厂接口

using namespace std;

/**
 * @brief 加载TUM‑VI数据集图像路径与时间戳
 * @param strImagePath 图像文件夹目录
 * @param strPathTimes 时间戳txt文件路径
 * @param vstrImages 输出，图像完整路径数组
 * @param vTimeStamps 输出，转换为秒的图像时间戳数组
 */
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

double ttrack_tot = 0;                 // 全局变量，累计所有帧跟踪总耗时，单位秒

/**
 * @brief 单目模式运行TUM‑VI数据集，支持多序列连续处理
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char **argv)
{
    // 计算数据集序列数量，每组序列占用2个命令行参数(图像文件夹、时间戳文件)
    const int num_seq = (argc - 3) / 2;
    cout << "num_seq = " << num_seq << endl;
    // 判断末尾是否传入可选自定义轨迹文件名
    bool bFileName = (((argc - 3) % 2) == 1);

    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]); // 获取末尾传入的轨迹文件名
        cout << "file name: " << file_name << endl;
    }

    // 参数合法性校验，至少需要一组完整序列
    if (argc < 4)
    {
        cerr
            << endl
            << "Usage: ./mono_tum_vi path_to_vocabulary path_to_settings "
               "path_to_image_folder_1 path_to_times_file_1 "
               "(path_to_image_folder_2 path_to_times_file_2 ... "
               "path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)"
            << endl;
        return 1;
    }

    // Load all sequences: 定义多序列存储容器，二维vector第一维代表序列序号
    int seq;
    vector<vector<string> > vstrImageFilenames;  // 每个序列内部全部图像路径
    vector<vector<double> > vTimestampsCam;       // 每个序列相机图像时间戳
    vector<int> nImages;                          // 每个序列图像总帧数

    // 根据序列数量调整容器大小
    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0; // 全部序列累加的图像总数量
    for (seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        // 读取当前序列图像路径与时间戳，计算argv偏移下标
        LoadImages(string(argv[(2 * seq) + 3]), string(argv[(2 * seq) + 4]),
                   vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size(); // 当前序列图像帧数
        tot_images += nImages[seq];                    // 累加全局总图像数

        // 当前序列图像加载为空，报错退出
        if ((nImages[seq] <= 0))
        {
            cerr << "ERROR: Failed to load images for sequence" << seq << endl;
            return 1;
        }
    }

    // Vector for tracking time statistics 跟踪耗时统计数组，总图像数大小
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17); // 设置浮点数输出精度17位，保证纳秒转秒后时间戳完整

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM实例，传感器MONOCULAR单目，false关闭多数据集标记，传入轨迹文件名
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::MONOCULAR, false, file_name);

    // SLAM系统初始化失败，打印错误信息退出程序
    if (!exSLAM)
    {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM有效智能指针

    float imageScale = SLAM->GetImageScale(); // 从配置yaml读取图像缩放系数

    double t_resize = 0.f; // 图像缩放耗时，单位ms，REGISTER_TIMES宏开启才统计
    double t_track = 0.f; // 单帧总处理耗时，单位ms，REGISTER_TIMES宏开启才统计

    int proccIm = 0; // 全局已处理图像计数
    for (seq = 0; seq < num_seq; seq++)
    {
        // Main loop 当前序列主循环
        cv::Mat im;                          // 存储读取、缩放、增强后的灰度图像
        proccIm = 0;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8)); // 创建CLAHE对比度受限直方图均衡对象

        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++)
        {
            // Read image from file 以灰度模式读取图像文件
            im = cv::imread(vstrImageFilenames[seq][ni],
                            cv::IMREAD_GRAYSCALE);  //,cv::IMREAD_GRAYSCALE);

            // 如果配置缩放系数不等于1，执行图像缩放
            if (imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
                // 记录缩放操作开始时间点
                std::chrono::steady_clock::time_point t_Start_Resize =
                    std::chrono::steady_clock::now();
#endif
                int width = im.cols * imageScale;   // 缩放后图像宽度
                int height = im.rows * imageScale;  // 缩放后图像高度
                cv::resize(im, im, cv::Size(width, height)); // 原地缩放图像

#ifdef REGISTER_TIMES
                // 记录缩放操作结束时间点
                std::chrono::steady_clock::time_point t_End_Resize =
                    std::chrono::steady_clock::now();

                // 计算缩放耗时，单位毫秒
                t_resize = std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli> >(
                                t_End_Resize - t_Start_Resize)
                                .count();
                SLAM->InsertResizeTime(t_resize); // 将缩放耗时送入SLAM做统计
#endif
            }

            // clahe 执行CLAHE对比度增强，提升图像弱纹理区域对比度
            clahe->apply(im, im);

            // cout << "mat type: " << im.type() << endl;
            double tframe = vTimestampsCam[seq][ni]; // 当前相机帧时间戳，单位秒

            // 图像读取失败，打印路径并返回异常
            if (im.empty())
            {
                cerr << endl
                     << "Failed to load image at: " << vstrImageFilenames[seq][ni]
                     << endl;
                return 1;
            }

            // 记录跟踪处理开始时间点
            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now();

            // Pass the image to the SLAM system 将图像与时间戳送入SLAM执行单目跟踪
            SLAM->TrackMonocular(im, tframe);  // TODO change to monocular_inertial

            // 记录跟踪处理结束时间点
            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
            // 计算缩放+跟踪总耗时，单位毫秒
            t_track =
                t_resize + std::chrono::duration_cast<
                               std::chrono::duration<double, std::milli> >(t2 - t1)
                               .count();
            SLAM->InsertTrackTime(t_track); // 将总耗时送入SLAM做统计
#endif

            // 计算本帧跟踪耗时，单位秒
            double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                    .count();
            ttrack_tot += ttrack; // 累加到全局总跟踪耗时

            vTimesTrack[ni] = ttrack; // 保存本帧耗时到统计数组

            // Wait to load the next frame 模拟真实相机帧率，处理速度快则休眠补齐时间间隔
            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe; // 非最后一帧，取当前帧与下一帧时间差
            else if (ni > 0)
                T = tframe - vTimestampsCam[seq][ni - 1]; // 最后一帧，取和上一帧时间差

            if (ttrack < T) usleep((T - ttrack) * 1e6);  // 1e6，usleep入参单位微秒，秒转微秒乘1e6
        }

        // 如果不是最后一个数据集序列，调用接口切换数据集
        if (seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM->ChangeDataset();
        }
    }

    // cout << "ttrack_tot = " << ttrack_tot << std::endl;
    // Stop all threads 全部序列处理完成，关闭SLAM全部后台线程
    SLAM->Shutdown();

    // Tracking time statistics

    // Save camera trajectory 根据是否传入自定义文件名选择输出轨迹文件名
    if (bFileName)
    {
        const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
        const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
        SLAM->SaveTrajectoryEuRoC(f_file);         // 保存完整相机轨迹，EuRoC格式
        SLAM->SaveKeyFrameTrajectoryEuRoC(kf_file); // 保存关键帧轨迹，EuRoC格式
    }
    else
    {
        SLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    sort(vTimesTrack.begin(), vTimesTrack.end()); // 耗时数组升序排序，用于求中位数
    float totaltime = 0;
    for (int ni = 0; ni < nImages[0]; ni++)
    {
        totaltime += vTimesTrack[ni]; // 累加第一个序列所有帧跟踪耗时
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages[0] / 2] << endl; // 输出跟踪耗时中位数
    cout << "mean tracking time: " << totaltime / proccIm << endl;           // 输出跟踪耗时平均值

    return 0;
}

/**
 * @brief 读取TUM‑VI时间戳文件，解析时间戳，拼接png图像完整路径，纳秒时间戳转为秒
 * @param strImagePath 图像文件夹路径
 * @param strPathTimes 时间戳txt文件路径
 * @param vstrImages [out]输出图像完整路径列表
 * @param vTimeStamps [out]输出转换为秒的时间戳列表
 */
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);  // 预分配容器空间，减少内存重新分配开销
    vstrImages.reserve(5000);
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);

        if (!s.empty())
        {
            if (s[0] == '#') continue; // 跳过#开头的注释行

            int pos = s.find(' ');     // 查找空格分隔符
            string item = s.substr(0, pos); // 截取纳秒时间戳字符串

            vstrImages.push_back(strImagePath + "/" + item + ".png"); // 拼接图像完整路径
            double t = stod(item);
            vTimeStamps.push_back(t / 1e9); // 纳秒转换为秒存入时间戳数组
        }
    }
}
