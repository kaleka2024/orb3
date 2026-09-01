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
#include <chrono>                       // C++高精度时钟，统计各阶段耗时
#include <ctime>                        // C标准时间库
#include <fstream>                      // 文件流，读取TUM‑VI图像时间戳、IMU csv文件
#include <iostream>                     // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>        // OpenCV核心模块，Mat、Point3f、CLAHE
#include <sstream>                      // 字符串流处理

#include "ImuTypes.h"                   // ORB‑SLAM3 IMU数据结构定义头文件

using namespace std;

/**
 * @brief 加载TUM‑VI数据集图像路径与对应时间戳
 * @param strImagePath 图像文件夹路径
 * @param strPathTimes 图像时间戳文件路径
 * @param vstrImages 输出，图像完整路径数组
 * @param vTimeStamps 输出，图像时间戳数组（转换为秒）
 */
void LoadImagesTUMVI(const string &strImagePath, const string &strPathTimes,
                     vector<string> &vstrImages, vector<double> &vTimeStamps);

/**
 * @brief 加载TUM‑VI数据集IMU csv数据
 * @param strImuPath IMU csv文件路径
 * @param vTimeStamps 输出，IMU时间戳数组（转换为秒）
 * @param vAcc 输出，加速度计三维数据数组
 * @param vGyro 输出，陀螺仪三维数据数组
 */
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;                  // 全局变量，累计所有帧跟踪总耗时，单位秒

/**
 * @brief IMU_MONOCULAR单目+IMU模式运行TUM‑VI数据集主程序，支持多序列连续运行
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char **argv) {
    // 计算数据集序列数量，每一组序列占3个命令行参数
    const int num_seq = (argc - 3) / 3;
    cout << "num_seq = " << num_seq << endl;
    // 判断末尾是否传入可选轨迹保存文件名，argc%3==1代表末尾多出一个文件名参数
    bool bFileName = ((argc % 3) == 1);

    string file_name;
    if (bFileName) file_name = string(argv[argc - 1]); // 获取自定义轨迹文件名

    cout << "file name: " << file_name << endl;

    // 参数合法性校验，至少需要一组完整序列，总参数不能小于6
    if (argc < 6) {
        cerr << endl
             << "Usage: ./mono_inertial_tum_vi path_to_vocabulary path_to_settings "
                "path_to_image_folder_1 path_to_times_file_1 path_to_imu_data_1 "
                "(path_to_image_folder_2 path_to_times_file_2 path_to_imu_data_2 "
                "... path_to_image_folder_N path_to_times_file_N "
                "path_to_imu_data_N) (trajectory_file_name)"
             << endl;
        return 1;
    }

    // Load all sequences: 定义多序列存储容器，二维vector，第一维为序列序号
    int seq;
    vector<vector<string> > vstrImageFilenames;    // 每个序列内所有图像路径
    vector<vector<double> > vTimestampsCam;         // 每个序列相机图像时间戳
    vector<vector<cv::Point3f> > vAcc, vGyro;      // 每个序列加速度、陀螺仪数据
    vector<vector<double> > vTimestampsImu;         // 每个序列IMU时间戳
    vector<int> nImages;                            // 每个序列图像总帧数
    vector<int> nImu;                               // 每个序列IMU测量点总数
    vector<int> first_imu(num_seq, 0);              // 每个序列起始使用的IMU下标

    // 根据序列数量调整容器大小
    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0; // 全部序列图像总数量
    for (seq = 0; seq < num_seq; seq++) {
        cout << "Loading images for sequence " << seq << "...";
        // 读取当前序列图像与相机时间戳，计算argv偏移位置
        LoadImagesTUMVI(string(argv[3 * (seq + 1)]),
                        string(argv[3 * (seq + 1) + 1]), vstrImageFilenames[seq],
                        vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        // 读取当前序列IMU数据
        LoadIMU(string(argv[3 * (seq + 1) + 2]), vTimestampsImu[seq], vAcc[seq],
                vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size(); // 当前序列图像数目
        tot_images += nImages[seq];                    // 累加全局总图像数
        nImu[seq] = vTimestampsImu[seq].size();        // 当前序列IMU样本数目

        // 图像或IMU数据为空，直接报错退出
        if ((nImages[seq] <= 0) || (nImu[seq] <= 0)) {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        // 找到第一个时间戳大于等于第一帧相机图像时间的IMU下标，IMU数据早于图像
        while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--;  // first imu measurement to be considered，回退，取该图像之前最后一条IMU
    }

    // Vector for tracking time statistics 跟踪耗时统计数组，总图像数大小
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17); // 设置浮点数输出精度17位，保证时间戳完整

    /*cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "IMU data in the sequence: " << nImu << endl << endl;*/

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM实例，传感器IMU_MONOCULAR单目+IMU，true多数据集模式，传入轨迹文件名
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::IMU_MONOCULAR, true, file_name);

    // SLAM系统初始化失败，打印错误退出
    if (!exSLAM) {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM有效智能指针

    float imageScale = SLAM->GetImageScale(); // 从配置文件读取图像缩放系数

    double t_resize = 0.f; // 图像缩放耗时，单位ms，REGISTER_TIMES宏开启生效
    double t_track = 0.f; // 单帧总处理耗时，单位ms，REGISTER_TIMES宏开启生效

    int proccIm = 0; // 全局已处理图像计数
    for (seq = 0; seq < num_seq; seq++) {
        // Main loop 当前序列主循环
        cv::Mat im;                          // 存储读取并处理后的灰度图像
        vector<ORB_SLAM3::IMU::Point> vImuMeas; // 当前帧需要传入SLAM的IMU测量值
        proccIm = 0;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8)); // 创建CLAHE对比度均衡对象

        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
            // Read image from file 以灰度模式读取图像
            im = cv::imread(vstrImageFilenames[seq][ni],
                            cv::IMREAD_GRAYSCALE);  //,cv::IMREAD_GRAYSCALE);

            // clahe 执行对比度受限直方图均衡，增强图像对比度
            clahe->apply(im, im);

            // cout << "mat type: " << im.type() << endl;
            double tframe = vTimestampsCam[seq][ni]; // 当前相机帧时间戳

            // 图像读取失败报错返回
            if (im.empty()) {
                cerr << endl
                     << "Failed to load image at: " << vstrImageFilenames[seq][ni]
                     << endl;
                return 1;
            }

            // Load imu measurements from previous frame 清空IMU容器，准备装填本帧所需IMU数据
            vImuMeas.clear();

            if (ni > 0) {
                // cout << "t_cam " << tframe << endl;
                // 把时间戳小于等于当前相机帧时间的IMU数据全部取出，存入vImuMeas
                while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni]) {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                        vAcc[seq][first_imu[seq]].x, vAcc[seq][first_imu[seq]].y,
                        vAcc[seq][first_imu[seq]].z, vGyro[seq][first_imu[seq]].x,
                        vGyro[seq][first_imu[seq]].y, vGyro[seq][first_imu[seq]].z,
                        vTimestampsImu[seq][first_imu[seq]]));
                    // cout << "t_imu = " << fixed << vImuMeas.back().t << endl;
                    first_imu[seq]++; // IMU下标向后移动
                }
            }

            // 如果缩放系数不等于1，执行图像缩放
            if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point t_Start_Resize =
                    std::chrono::steady_clock::now();
#endif
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point t_End_Resize =
                    std::chrono::steady_clock::now();

                t_resize = std::chrono::duration_cast<
                               std::chrono::duration<double, std::milli> >(
                               t_End_Resize - t_Start_Resize)
                               .count();
                SLAM->InsertResizeTime(t_resize);
#endif
            }

            // cout << "first imu: " << first_imu[seq] << endl;
            /*cout << "first imu time: " << fixed << vTimestampsImu[first_imu] << endl;
            cout << "size vImu: " << vImuMeas.size() << endl;*/
            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now(); // 跟踪开始时间点

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            SLAM->TrackMonocular(im, tframe,
                                 vImuMeas);  // TODO change to monocular_inertial，执行单目惯性跟踪

            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now(); // 跟踪结束时间点

#ifdef REGISTER_TIMES
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
            ttrack_tot += ttrack; // 累加到全局总耗时
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni] = ttrack; // 保存本帧耗时到统计数组

            // Wait to load the next frame 计算休眠时间，模拟真实相机帧率
            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe; // 当前帧与下一帧时间差
            else if (ni > 0)
                T = tframe - vTimestampsCam[seq][ni - 1]; // 最后一帧取和上一帧时间差

            if (ttrack < T) usleep((T - ttrack) * 1e6);  // 1e6，usleep单位微秒，处理快则休眠补齐时间
        }
        // 如果不是最后一个序列，切换数据集
        if (seq < num_seq - 1) {
            cout << "Changing the dataset" << endl;

            SLAM->ChangeDataset();
        }
    }

    // cout << "ttrack_tot = " << ttrack_tot << std::endl;
    // Stop all threads，全部序列处理完成，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Tracking time statistics

    // Save camera trajectory 根据是否传入自定义文件名选择输出轨迹文件名
    if (bFileName) {
        const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
        const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
        SLAM->SaveTrajectoryEuRoC(f_file);         // 保存完整相机轨迹，EuRoC格式
        SLAM->SaveKeyFrameTrajectoryEuRoC(kf_file);// 保存关键帧轨迹，EuRoC格式
    } else {
        SLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    sort(vTimesTrack.begin(), vTimesTrack.end()); // 耗时数组升序排序
    float totaltime = 0;
    for (int ni = 0; ni < nImages[0]; ni++) {
        totaltime += vTimesTrack[ni]; // 累加第一序列所有帧耗时
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages[0] / 2] << endl; // 输出跟踪耗时中位数
    cout << "mean tracking time: " << totaltime / proccIm << endl;           // 输出跟踪耗时平均值

    /*const string kf_file =  "kf_" + ss.str() + ".txt";
    const string f_file =  "f_" + ss.str() + ".txt";

    SLAM->SaveTrajectoryEuRoC(f_file);
    SLAM->SaveKeyFrameTrajectoryEuRoC(kf_file);*/

    return 0;
}

/**
 * @brief 读取TUM‑VI图像时间戳文件，拼接图像路径，时间单位由纳秒转为秒
 * @param strImagePath 图像文件夹路径
 * @param strPathTimes times.txt时间戳文件路径
 * @param vstrImages [out]图像完整路径列表
 * @param vTimeStamps [out]转换为秒的时间戳列表
 */
void LoadImagesTUMVI(const string &strImagePath, const string &strPathTimes,
                     vector<string> &vstrImages, vector<double> &vTimeStamps) {
    ifstream fTimes;
    cout << strImagePath << endl;
    cout << strPathTimes << endl;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);  // 预分配容器空间，提升性能
    vstrImages.reserve(5000);
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);

        if (!s.empty()) {
            if (s[0] == '#') continue; // 跳过注释行

            int pos = s.find(' ');     // 查找空格分隔符
            string item = s.substr(0, pos); // 截取时间戳字符串（纳秒）

            vstrImages.push_back(strImagePath + "/" + item + ".png"); // 拼接png图像完整路径
            double t = stod(item);
            vTimeStamps.push_back(t / 1e9); // 纳秒转秒存入时间戳数组
        }
    }
}

/**
 * @brief 读取TUM‑VI IMU的csv文件，解析时间戳、加速度、陀螺仪
 * @param strImuPath IMU csv文件路径
 * @param vTimeStamps [out]IMU时间戳，单位秒
 * @param vAcc [out]加速度计三轴数据
 * @param vGyro [out]陀螺仪三轴数据
 */
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro) {
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while (!fImu.eof()) {
        string s;
        getline(fImu, s);
        if (s[0] == '#') continue; // 跳过注释行

        if (!s.empty()) {
            string item;
            size_t pos = 0;
            double data[7]; // csv一行7列：timestamp,wx,wy,wz,ax,ay,az
            int count = 0;
            // 按逗号分割csv字段
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0] / 1e9); // 时间戳纳秒转秒
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6])); // 加速度 x y z
            vGyro.push_back(cv::Point3f(data[1], data[2], data[3])); // 陀螺仪 x y z
        }
    }
}
