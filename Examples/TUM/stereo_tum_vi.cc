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
#include <unistd.h>                    // POSIX系统调用，提供usleep函数实现线程休眠

#include <algorithm>                   // STL算法库，sort用于跟踪耗时数组排序
#include <chrono>                      // C++11高精度时钟，统计每帧SLAM处理耗时
#include <fstream>                     // 文件输入流，读取相机时间戳txt文件
#include <iomanip>                     // IO格式化输出，设置输出精度
#include <iostream>                    // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>       // OpenCV核心模块，Mat、CLAHE、图像读写、图像缩放

using namespace std;

/**
 * @brief 加载TUM‑VI数据集双目图像路径与相机时间戳
 * @param strPathLeft 左目图像文件夹路径
 * @param strPathRight 右目图像文件夹路径
 * @param strPathTimes 相机时间戳txt文件路径
 * @param vstrImageLeft 输出参数，左目图像完整路径容器
 * @param vstrImageRight 输出参数，右目图像完整路径容器
 * @param vTimeStamps 输出参数，转换为秒单位的相机时间戳容器
 */
void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

double ttrack_tot = 0;                 // 全局变量，累计所有帧SLAM跟踪总耗时，单位：秒

/**
 * @brief 双目模式运行TUM‑VI数据集，支持多序列连续处理，无IMU
 * @param argc 命令行参数总个数
 * @param argv 命令行参数字符串数组
 * @return int 程序退出码，0代表正常结束，非0代表异常退出
 */
int main(int argc, char **argv)
{
    // 计算待处理数据集序列数目，每组双目序列占用3个参数：左图目录、右图目录、时间戳文件
    const int num_seq = (argc - 3) / 3;
    cout << "num_seq = " << num_seq << endl;
    // 判断命令行末尾是否携带可选自定义轨迹输出文件名
    bool bFileName = (((argc - 3) % 3) == 1);
    string file_name;
    if (bFileName)
        file_name = string(argv[argc - 1]);

    // 参数合法性校验，至少需要1组完整双目数据集参数
    if (argc < 6)
    {
        cerr << endl
             << "Usage: ./stereo_tum_vi path_to_vocabulary path_to_settings "
                "path_to_image_folder1_1 path_to_image_folder2_1 "
                "path_to_times_file_1 (path_to_image_folder1_2 "
                "path_to_image_folder2_2 path_to_times_file_2 ... "
                "path_to_image_folder1_N path_to_image_folder2_N "
                "path_to_times_file_N) (trajectory_file_name)"
             << endl;
        return 1;
    }

    // Load all sequences: 多序列存储容器，二维vector第一维对应序列序号
    int seq;
    vector<vector<string> > vstrImageLeftFilenames;   // 每个序列左目图像路径集合
    vector<vector<string> > vstrImageRightFilenames;  // 每个序列右目图像路径集合
    vector<vector<double> > vTimestampsCam;            // 每个序列相机帧时间戳集合
    vector<int> nImages;                               // 每个序列的图像帧数

    // 根据序列数量调整各容器大小
    vstrImageLeftFilenames.resize(num_seq);
    vstrImageRightFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0; // 全部序列累加得到的图像总数量
    for (seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        // 读取当前序列双目图像路径与相机时间戳，计算argv数组偏移下标
        LoadImages(string(argv[(3 * seq) + 3]), string(argv[(2 * seq) + 4]),
                   string(argv[(2 * seq) + 5]), vstrImageLeftFilenames[seq],
                   vstrImageRightFilenames[seq], vTimestampsCam[seq]);
        cout << "Total images: " << vstrImageLeftFilenames[seq].size() << endl;
        cout << "Total cam ts: " << vTimestampsCam[seq].size() << endl;
        cout << "first cam ts: " << vTimestampsCam[seq][0] << endl;

        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeftFilenames[seq].size(); // 记录当前序列图像总帧数
        tot_images += nImages[seq];                        // 累加到全局总图像计数

        // 当前序列图像加载失败，帧数小于等于0，报错退出程序
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
    cout.precision(17); // 设置浮点数输出精度17位，完整保留纳秒转换后的时间戳

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 构造SLAM系统实例，传感器类型STEREO双目，true开启多数据集模式
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::STEREO, true);

    // SLAM系统初始化失败，打印错误信息，退出程序
    if (!exSLAM)
    {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM系统有效智能指针

    float imageScale = SLAM->GetImageScale(); // 从yaml配置文件读取图像缩放系数

    cout << endl << "-------" << endl;
    cout.precision(17);

    cv::Mat imLeft, imRight;                                      // 存放左目、右目灰度图像
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8)); // 创建CLAHE对比度受限直方图均衡器，增强图像弱纹理

    double t_resize = 0.f; // 图像缩放耗时，单位ms，仅REGISTER_TIMES宏开启才统计
    double t_track = 0.f;  // 单帧总处理耗时，单位ms，仅REGISTER_TIMES宏开启才统计

    int proccIm = 0; // 全局已经处理完成的图像计数
    for (seq = 0; seq < num_seq; seq++)
    {
        // Main loop 当前数据集序列主循环
        proccIm = 0;
        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++)
        {
            // Read image from file 灰度模式读取左目图像
            imLeft =
                cv::imread(vstrImageLeftFilenames[seq][ni], cv::IMREAD_GRAYSCALE);
            // 灰度模式读取右目图像
            imRight =
                cv::imread(vstrImageRightFilenames[seq][ni], cv::IMREAD_GRAYSCALE);

            // 如果配置图像缩放系数不等于1，同步缩放左右目图像
            if (imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
                // 记录图像缩放操作开始时间点
                std::chrono::steady_clock::time_point t_Start_Resize =
                    std::chrono::steady_clock::now();
#endif
                int width = imLeft.cols * imageScale;
                int height = imLeft.rows * imageScale;
                cv::resize(imLeft, imLeft, cv::Size(width, height));  // 缩放左目图像
                cv::resize(imRight, imRight, cv::Size(width, height)); // 缩放右目图像
#ifdef REGISTER_TIMES
                // 记录图像缩放操作结束时间点
                std::chrono::steady_clock::time_point t_End_Resize =
                    std::chrono::steady_clock::now();

                // 计算缩放耗时，转换为毫秒
                t_resize = std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli> >(
                                t_End_Resize - t_Start_Resize)
                                .count();
                SLAM->InsertResizeTime(t_resize); // 将缩放耗时送入SLAM内部做统计
#endif
            }

            // clahe 对左右目图像做CLAHE对比度增强，改善暗光、弱纹理场景
            clahe->apply(imLeft, imLeft);
            clahe->apply(imRight, imRight);

            double tframe = vTimestampsCam[seq][ni]; // 当前相机帧时间戳，单位秒

            // 左目或者右目图像读取失败为空，打印路径，返回异常退出
            if (imLeft.empty() || imRight.empty())
            {
                cerr << endl
                     << "Failed to load image at: " << vstrImageLeftFilenames[seq][ni]
                     << endl;
                return 1;
            }

            // 记录SLAM跟踪处理本帧的起始时间点
            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now();

            // Pass the image to the SLAM system 将双目图像、时间戳送入SLAM执行双目跟踪
            SLAM->TrackStereo(imLeft, imRight, tframe);

            // 记录SLAM跟踪处理本帧的结束时间点
            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
            // 图像缩放耗时 + 跟踪计算耗时，总耗时单位毫秒
            t_track =
                t_resize + std::chrono::duration_cast<
                               std::chrono::duration<double, std::milli> >(t2 - t1)
                               .count();
            SLAM->InsertTrackTime(t_track); // 将总耗时送入SLAM内部统计
#endif

            // 计算本帧SLAM处理耗时，单位秒
            double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                    .count();
            ttrack_tot += ttrack; // 累加到全局总跟踪耗时
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni] = ttrack; // 保存本帧耗时到统计数组

            // Wait to load the next frame 模拟真实相机帧率，处理速度快时休眠补齐帧间隔
            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe; // 非最后一帧，取当前帧与下一帧时间差
            else if (ni > 0)
                T = tframe - vTimestampsCam[seq][ni - 1]; // 最后一帧，取和上一帧时间差

            if (ttrack < T)
                usleep((T - ttrack) * 1e6);  // 1e6，usleep入参单位微秒，秒转微秒乘以1e6
        }
        // 如果不是最后一个数据集序列，调用接口切换数据集
        if (seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM->ChangeDataset();
        }
    }

    // Stop all threads 全部序列处理完毕，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Tracking time statistics

    // Save camera trajectory 获取系统当前时间，用于轨迹文件备用
    std::chrono::system_clock::time_point scNow =
        std::chrono::system_clock::now();
    std::time_t now = std::chrono::system_clock::to_time_t(scNow);
    std::stringstream ss;
    ss << now;

    // 根据是否传入自定义文件名，输出EuRoC格式轨迹文件
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

    sort(vTimesTrack.begin(), vTimesTrack.end()); // 耗时数组升序排序，用于求取中位数
    float totaltime = 0;
    for (int ni = 0; ni < nImages[0]; ni++)
    {
        totaltime += vTimesTrack[ni]; // 累加第一个序列全部帧跟踪耗时
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages[0] / 2] << endl; // 输出跟踪耗时中位数
    cout << "mean tracking time: " << totaltime / proccIm << endl;           // 输出跟踪耗时平均值

    return 0;
}

/*void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps) {
    ifstream fTimes;
    cout << strPathLeft << endl;
    cout << strPathRight << endl;
    cout << strPathTimes << endl;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);
        }
    }
}*/

/**
 * @brief 读取TUM‑VI相机时间戳txt文件，解析纳秒时间戳，拼接双目png图像完整路径，时间戳转为秒
 * @param strPathLeft 左目图像文件夹路径
 * @param strPathRight 右目图像文件夹路径
 * @param strPathTimes 相机时间戳txt文件路径
 * @param vstrImageLeft [out]输出左目图像完整路径列表
 * @param vstrImageRight [out]输出右目图像完整路径列表
 * @param vTimeStamps [out]输出转换为秒的相机时间戳列表
 */
void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    cout << strPathLeft << endl;
    cout << strPathRight << endl;
    cout << strPathTimes << endl;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);    // 预分配容器内存，减少动态扩容开销
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);

        if (!s.empty())
        {
            if (s[0] == '#')
                continue; // 跳过#开头注释行

            int pos = s.find(' ');             // 查找空格分隔符
            string item = s.substr(0, pos);    // 截取纳秒时间戳字符串

            vstrImageLeft.push_back(strPathLeft + "/" + item + ".png");  // 拼接左目图像完整路径
            vstrImageRight.push_back(strPathRight + "/" + item + ".png");// 拼接右目图像完整路径

            double t = stod(item);
            vTimeStamps.push_back(t / 1e9); // 纳秒时间戳转换为秒存入容器
        }
    }
}
