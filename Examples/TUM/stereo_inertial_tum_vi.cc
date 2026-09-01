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
#include <algorithm>                   // STL算法库，sort用于耗时数组排序
#include <chrono>                      // C++高精度时钟，统计帧处理耗时、系统时间
#include <ctime>                       // C时间库，时间戳转日历时间
#include <fstream>                     // 文件输入流，读取图像时间戳、IMU csv数据
#include <iostream>                    // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>       // OpenCV核心模块，Mat、Point3f、CLAHE、图像读写缩放
#include <sstream>                     // stringstream字符串流，字符串与数值转换
#include <string>                      // std::string字符串
#include <vector>                      // std::vector动态数组容器

#include "ImuTypes.h"                  // ORB‑SLAM3 IMU数据结构定义头文件
#include "System.h"                    // ORB‑SLAM3系统主头文件，SystemFactory工厂接口

using std::string;
using std::vector;

/**
 * @brief 加载TUM‑VI双目相机图像路径与时间戳
 * @param strPathLeft 左目图像文件夹路径
 * @param strPathRight 右目图像文件夹路径
 * @param strPathTimes 相机时间戳txt文件路径
 * @param vstrImageLeft 输出，左目图像完整路径数组
 * @param vstrImageRight 输出，右目图像完整路径数组
 * @param vTimeStamps 输出，转换为秒的相机时间戳数组
 */
void LoadImagesTUMVI(const string &strPathLeft, const string &strPathRight,
                     const string &strPathTimes, vector<string> &vstrImageLeft,
                     vector<string> &vstrImageRight,
                     vector<double> &vTimeStamps);

/**
 * @brief 解析TUM‑VI IMU csv文件，读取IMU时间戳、加速度计、陀螺仪测量值
 * @param strImuPath IMU csv文件路径
 * @param vTimeStamps 输出，IMU时间戳数组(单位秒)
 * @param vAcc 输出，加速度计三轴测量值
 * @param vGyro 输出，陀螺仪三轴测量值
 */
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;                 // 全局变量，累计所有帧跟踪总耗时，单位秒

/**
 * @brief 双目+IMU模式运行TUM‑VI数据集，支持多序列连续处理
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char **argv)
{
    // 计算数据集序列数量，每组序列占用4个参数：左图目录、右图目录、相机时间戳、IMU数据文件
    const int num_seq = (argc - 3) / 4;
    cout << "num_seq = " << num_seq << endl;
    // 判断末尾是否携带可选自定义轨迹文件名
    bool bFileName = (((argc - 3) % 4) == 1);
    string file_name;
    if (bFileName)
        file_name = string(argv[argc - 1]);

    // 参数校验，至少需要一组完整双目IMU序列
    if (argc < 7)
    {
        cerr << endl
             << "Usage: ./stereo_inertial_tum_vi path_to_vocabulary "
                "path_to_settings path_to_image_folder_1 path_to_image_folder_2 "
                "path_to_times_file path_to_imu_data (trajectory_file_name)"
             << endl;
        return 1;
    }

    // Load all sequences: 多序列容器，二维vector第一维代表序列序号
    int seq;
    vector<vector<string> > vstrImageLeftFilenames;   // 每个序列左目图像路径集合
    vector<vector<string> > vstrImageRightFilenames;  // 每个序列右目图像路径集合
    vector<vector<double> > vTimestampsCam;            // 每个序列相机帧时间戳
    vector<vector<cv::Point3f> > vAcc, vGyro;         // 每个序列IMU加速度、陀螺仪数据
    vector<vector<double> > vTimestampsImu;           // 每个序列IMU时间戳
    vector<int> nImages;                               // 每个序列相机图像帧数
    vector<int> nImu;                                  // 每个序列IMU测量总条数
    vector<int> first_imu(num_seq, 0);                // 每个序列与首帧相机匹配的IMU起始下标

    // 根据序列数量调整容器大小
    vstrImageLeftFilenames.resize(num_seq);
    vstrImageRightFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0; // 全部序列累加图像总数量
    for (seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        // 读取当前序列双目图像路径与相机时间戳，计算argv偏移下标
        LoadImagesTUMVI(
            string(argv[4 * (seq + 1) - 1]), string(argv[4 * (seq + 1)]),
            string(argv[4 * (seq + 1) + 1]), vstrImageLeftFilenames[seq],
            vstrImageRightFilenames[seq], vTimestampsCam[seq]);
        cout << "Total images: " << vstrImageLeftFilenames[seq].size() << endl;
        cout << "Total cam ts: " << vTimestampsCam[seq].size() << endl;
        cout << "first cam ts: " << vTimestampsCam[seq][0] << endl;

        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        // 读取当前序列IMU数据
        LoadIMU(string(argv[4 * (seq + 1) + 2]), vTimestampsImu[seq], vAcc[seq],
                vGyro[seq]);
        cout << "Total IMU meas: " << vTimestampsImu[seq].size() << endl;
        cout << "first IMU ts: " << vTimestampsImu[seq][0] << endl;
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeftFilenames[seq].size(); // 当前序列图像帧数
        tot_images += nImages[seq];                        // 累加全局总图像数
        nImu[seq] = vTimestampsImu[seq].size();            // 当前序列IMU测量条数

        // 图像或者IMU数据为空，报错退出
        if ((nImages[seq] <= 0) || (nImu[seq] <= 0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        // 定位第一个时间戳大于等于首帧相机时间戳的IMU下标，IMU数据一般早于相机启动
        while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--;  // first imu measurement to be considered，回退，取该相机帧之前最后一条IMU
    }

    // Vector for tracking time statistics 跟踪耗时统计数组，总图像数大小
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17); // 设置浮点数输出精度17位，保证纳秒转秒时间戳完整保留

    /*cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "IMU data in the sequence: " << nImu << endl << endl;*/

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM实例，传感器IMU_STEREO双目惯性，true开启多数据集模式，传入自定义轨迹文件名
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::IMU_STEREO, true, file_name);

    // SLAM系统初始化失败，打印错误信息退出程序
    if (!exSLAM)
    {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM有效智能指针

    float imageScale = SLAM->GetImageScale(); // 从yaml配置读取图像缩放系数

    double t_resize = 0.f; // 图像缩放耗时，单位ms，REGISTER_TIMES宏开启才统计
    double t_track = 0.f; // 单帧总处理耗时，单位ms，REGISTER_TIMES宏开启才统计

    int proccIm = 0; // 全局已处理图像计数
    for (seq = 0; seq < num_seq; seq++)
    {
        // Main loop 当前序列主循环
        cv::Mat imLeft, imRight;                          // 存放左右目灰度图像
        vector<ORB_SLAM3::IMU::Point> vImuMeas;           // 存储当前帧之前累积的IMU测量值
        proccIm = 0;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8)); // 创建CLAHE对比度受限直方图均衡对象

        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++)
        {
            // Read image from file 灰度模式读取左、右目图像
            imLeft =
                cv::imread(vstrImageLeftFilenames[seq][ni], cv::IMREAD_GRAYSCALE);
            imRight =
                cv::imread(vstrImageRightFilenames[seq][ni], cv::IMREAD_GRAYSCALE);

            // 如果配置缩放系数不等于1，同步缩放左右目图像
            if (imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
                // 记录缩放操作开始时间点
                std::chrono::steady_clock::time_point t_Start_Resize =
                    std::chrono::steady_clock::now();
#endif
                int width = imLeft.cols * imageScale;
                int height = imLeft.rows * imageScale;
                cv::resize(imLeft, imLeft, cv::Size(width, height));
                cv::resize(imRight, imRight, cv::Size(width, height));
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

            // clahe 对左右目图像执行CLAHE对比度增强，改善弱纹理
            clahe->apply(imLeft, imLeft);
            clahe->apply(imRight, imRight);

            double tframe = vTimestampsCam[seq][ni]; // 当前相机帧时间戳，单位秒

            // 左目或者右目图像读取失败，打印路径返回异常
            if (imLeft.empty() || imRight.empty())
            {
                cerr << endl
                     << "Failed to load image at: " << vstrImageLeftFilenames[seq][ni]
                     << endl;
                return 1;
            }

            // Load imu measurements from previous frame 清空IMU测量容器，准备收集当前帧需要的IMU数据
            vImuMeas.clear();

            if (ni > 0)
            {
                // cout << "t_cam " << tframe << endl;
                // 把时间戳小于等于当前相机帧时间的所有IMU测量装入容器
                while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni])
                {
                    // vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu],vGyro[first_imu],vTimestampsImu[first_imu]));
                    // 构造IMU测量点，加速度三轴、陀螺仪三轴、时间戳
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                        vAcc[seq][first_imu[seq]].x, vAcc[seq][first_imu[seq]].y,
                        vAcc[seq][first_imu[seq]].z, vGyro[seq][first_imu[seq]].x,
                        vGyro[seq][first_imu[seq]].y, vGyro[seq][first_imu[seq]].z,
                        vTimestampsImu[seq][first_imu[seq]]));
                    // cout << "t_imu = " << fixed << vImuMeas.back().t << endl;
                    first_imu[seq]++; // IMU下标向后移动
                }
            }

            /*cout << "first imu: " << first_imu[seq] << endl;
            cout << "first imu time: " << fixed << vTimestampsImu[seq][0] << endl;
            cout << "size vImu: " << vImuMeas.size() << endl;*/

            // 记录帧跟踪处理开始时间点
            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now();

            // Pass the image to the SLAM system 将双目图像、时间戳、中间IMU测量送入SLAM执行双目惯性跟踪
            SLAM->TrackStereo(imLeft, imRight, tframe, vImuMeas);

            // 记录帧跟踪处理结束时间点
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

            // 计算本帧SLAM处理耗时，单位秒
            double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                    .count();
            ttrack_tot += ttrack; // 累加到全局总跟踪耗时
            // std::cout << "ttrack: " << ttrack << std::endl;

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

    // Stop all threads 全部序列处理完成，关闭SLAM全部后台线程
    SLAM->Shutdown();

    // Tracking time statistics

    // Save camera trajectory 获取系统当前时间，用于轨迹文件命名备用
    std::chrono::system_clock::time_point scNow =
        std::chrono::system_clock::now();
    std::time_t now = std::chrono::system_clock::to_time_t(scNow);
    std::stringstream ss;
    ss << now;

    // 根据是否传入自定义文件名选择输出EuRoC格式轨迹文件
    if (bFileName)
    {
        const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
        const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
        SLAM->SaveTrajectoryEuRoC(f_file);         // 保存完整相机轨迹 EuRoC格式
        SLAM->SaveKeyFrameTrajectoryEuRoC(kf_file); // 保存关键帧轨迹 EuRoC格式
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
 * @brief 读取TUM‑VI相机时间戳txt，解析纳秒时间戳，拼接左右目png图像完整路径，时间戳转为秒
 * @param strPathLeft 左目图像文件夹路径
 * @param strPathRight 右目图像文件夹路径
 * @param strPathTimes 相机时间戳txt文件路径
 * @param vstrImageLeft [out]输出左目图像完整路径列表
 * @param vstrImageRight [out]输出右目图像完整路径列表
 * @param vTimeStamps [out]输出转换为秒的相机时间戳列表
 */
void LoadImagesTUMVI(const string &strPathLeft, const string &strPathRight,
                     const string &strPathTimes, vector<string> &vstrImageLeft,
                     vector<string> &vstrImageRight,
                     vector<double> &vTimeStamps)
{
    ifstream fTimes;
    cout << strPathLeft << endl;
    cout << strPathRight << endl;
    cout << strPathTimes << endl;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);    // 预分配容器空间，减少内存重新分配开销
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);

        if (!s.empty())
        {
            if (s[0] == '#') continue; // 跳过#开头注释行

            int pos = s.find(' ');              // 查找空格分隔符
            string item = s.substr(0, pos);     // 截取纳秒时间戳字符串

            vstrImageLeft.push_back(strPathLeft + "/" + item + ".png");  // 拼接左图完整路径
            vstrImageRight.push_back(strPathRight + "/" + item + ".png");// 拼接右图完整路径

            double t = stod(item);
            vTimeStamps.push_back(t / 1e9); // 纳秒转换为秒存入时间戳数组
        }
    }
}

/**
 * @brief 解析TUM‑VI IMU csv逗号分隔文件，读取时间戳、陀螺仪、加速度计
 * @param strImuPath IMU csv文件路径
 * @param vTimeStamps [out]IMU时间戳数组，单位秒
 * @param vAcc [out]加速度计三轴测量值
 * @param vGyro [out]陀螺仪三轴测量值
 */
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000); // 预分配容器空间
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while (!fImu.eof())
    {
        string s;
        getline(fImu, s);
        if (s[0] == '#') continue; // 跳过#开头注释行

        if (!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            // 按逗号','分割csv一行，提取7个数值
            while ((pos = s.find(',')) != string::npos)
            {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0] / 1e9);                // data[0]纳秒时间戳，转为秒
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6])); // 加速度 x y z
            vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));// 陀螺仪 x y z
        }
    }
}
