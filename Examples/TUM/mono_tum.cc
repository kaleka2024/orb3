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
#include <System.h>                     // ORB‑SLAM3核心系统头文件，SystemFactory工厂接口
#include <algorithm>                    // STL算法库，用于sort排序耗时数组
#include <chrono>                       // C++高精度时钟库，统计图像处理耗时
#include <fstream>                      // 文件输入流，读取TUM数据集rgb.txt时间戳文件
#include <iostream>                     // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>        // OpenCV核心模块，Mat图像容器、图像读写接口

using namespace std;

/**
 * @brief 解析TUM数据集rgb.txt，读取图像相对路径与对应时间戳
 * @param strFile rgb.txt文件完整路径
 * @param vstrImageFilenames 输出，图像相对路径数组
 * @param vTimestamps 输出，图像时间戳数组，单位秒
 */
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

/**
 * @brief MONOCULAR单目模式运行TUM数据集主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常退出，非0异常退出
 */
int main(int argc, char **argv)
{
    // 校验命令行参数，需要4个参数：可执行程序、词袋路径、配置yaml、数据集路径
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
                "path_to_sequence"
             << endl;
        return 1;
    }

    // Retrieve paths to images 读取图像路径和时间戳
    vector<string> vstrImageFilenames;   // 存储rgb.txt中图像相对路径
    vector<double> vTimestamps;          // 存储每一帧图像的时间戳
    string strFile = string(argv[3]) + "/rgb.txt"; // 拼接rgb.txt完整文件路径
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size(); // 获取数据集图像总帧数

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 构造SLAM系统实例，传感器类型MONOCULAR单目，true开启多数据集模式
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::MONOCULAR, true);

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

    double t_resize = 0.f; // 图像缩放耗时，单位ms，仅REGISTER_TIMES宏定义开启时统计生效
    double t_track = 0.f; // 单帧完整处理耗时，单位ms，仅REGISTER_TIMES宏定义开启时统计生效

    // Main loop 数据集主循环，逐帧处理图像
    cv::Mat im; // 存放读取到的图像
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read image from file 读取图像，IMREAD_UNCHANGED保持原始图像通道与位深
        im = cv::imread(string(argv[3]) + "/" + vstrImageFilenames[ni],
                        cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni]; // 当前帧对应的时间戳

        // 图像读取失败，打印路径并返回异常
        if (im.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << string(argv[3]) << "/"
                 << vstrImageFilenames[ni] << endl;
            return 1;
        }

        // 如果配置文件缩放系数不等于1，执行图像缩放
        if (imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
            // 记录缩放操作开始时间点
            std::chrono::steady_clock::time_point t_Start_Resize =
                std::chrono::steady_clock::now();
#endif
            int width = im.cols * imageScale;  // 缩放后图像宽度
            int height = im.rows * imageScale; // 缩放后图像高度
            cv::resize(im, im, cv::Size(width, height)); // 原地缩放图像

#ifdef REGISTER_TIMES
            // 记录缩放操作结束时间点
            std::chrono::steady_clock::time_point t_End_Resize =
                std::chrono::steady_clock::now();

            // 计算缩放耗时，单位毫秒
            t_resize = std::chrono::duration_cast<
                            std::chrono::duration<double, std::milli> >(t_End_Resize -
                                                                        t_Start_Resize)
                            .count();
            SLAM->InsertResizeTime(t_resize); // 将缩放耗时送入SLAM做统计
#endif
        }

        // 记录跟踪处理开始时间点
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system 将图像与时间戳送入SLAM，执行单目跟踪
        SLAM->TrackMonocular(im, tframe);

        // 记录跟踪处理结束时间点
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
        // 计算图像缩放+跟踪总耗时，单位毫秒
        t_track =
            t_resize +
            std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(
                t2 - t1)
                .count();
        SLAM->InsertTrackTime(t_track); // 将总耗时送入SLAM做统计
#endif

        // 计算本帧跟踪耗时，单位秒
        double ttrack =
            std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                .count();

        vTimesTrack[ni] = ttrack; // 保存本帧耗时到统计数组

        // Wait to load the next frame 模拟真实相机帧率，处理过快则休眠补齐时间间隔
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe; // 非最后一帧：取当前帧与下一帧时间差
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1]; // 最后一帧：取与上一帧时间差

        if (ttrack < T) usleep((T - ttrack) * 1e6); // usleep单位为微秒，乘以1e6做单位转换
    }

    // Stop all threads 全部帧处理完毕，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Tracking time statistics 跟踪耗时统计
    sort(vTimesTrack.begin(), vTimesTrack.end()); // 耗时数组升序排序，方便求中位数
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni]; // 累加所有帧跟踪耗时
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl; // 输出跟踪耗时中位数
    cout << "mean tracking time: " << totaltime / nImages << endl;       // 输出跟踪耗时平均值

    // Save camera trajectory 保存关键帧轨迹，TUM格式输出文本文件
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

/**
 * @brief 读取TUM数据集rgb.txt，跳过前3行注释，解析时间戳和图像相对路径
 * @param strFile rgb.txt文件路径
 * @param vstrImageFilenames [out]图像相对路径输出数组
 * @param vTimestamps [out]时间戳输出数组，单位秒
 */
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str()); // 打开rgb.txt文本文件

    // skip first three lines 跳过rgb.txt文件开头三行注释说明
    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    // 循环读取文件每一行，直到文件末尾
    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty()) // 跳过空行
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;                // 解析时间戳浮点数
            vTimestamps.push_back(t);
            ss >> sRGB;             // 解析图像相对路径字符串
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
