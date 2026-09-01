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
#include <System.h>                     // ORB‑SLAM3系统主头文件，包含SystemFactory工厂接口
#include <algorithm>                    // STL算法库，find_if、copy、back_inserter等
#include <chrono>                       // C++高精度时钟，用于统计各阶段耗时
#include <ctime>                        // C标准时间接口
#include <fstream>                      // 文件读写流
#include <iostream>                     // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>        // OpenCV核心模块，Mat、Size基础数据结构
#include <sstream>                      // stringstream字符串流解析文本

#include "euroc_common.h"               // EuRoC数据集加载工具头文件

using namespace std;

double ttrack_tot = 0;                  // 全局变量，累加所有帧跟踪总耗时，单位秒
/**
 * @brief IMU‑MONOCULAR模式运行EuRoC数据集主程序入口，单目+IMU
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char *argv[]) {
    setupEurocSpdLogger();              // 初始化spdlog日志器，配置控制台彩色输出
    spdlog::set_level(spdlog::level::info); // 设置日志输出等级为info，info及以上级别打印

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
    // 判断末尾参数是否为轨迹输出文件名；剩余参数总数为奇数代表末尾附带输出文件名
    bool bFileName = (((argc - 3) % 2) == 1);
    string trajFileName;
    if (bFileName) {
        trajFileName = string(argv[argc - 1]); // 取出用户指定的轨迹输出文件名
        cout << "file name: " << trajFileName << endl;
    }

    // Load all sequences: 组装全部序列路径配置
    int seq;
    vector<EuRoCData::SequencePaths> imagePaths; // 存放每条数据集序列的路径信息
    for (seq = 0; seq < num_seq; seq++) {
        cout << "Loading images for sequence " << seq << "..." << endl;

        // 解析一组序列：图像文件夹路径、图像时间戳文件路径
        const string pathSeq(argv[(2 * seq) + 3]);
        const string pathTimeStamps(argv[(2 * seq) + 4]);

        imagePaths.emplace_back(pathSeq, pathTimeStamps);
    }

    // 加载EuRoC数据集，第二个参数true表示同时加载IMU测量数据
    auto eurocData = EuRoCData::LoadSequences(imagePaths, true);

    // Vector for tracking time statistics 跟踪耗时统计容器，保存每一帧跟踪耗时
    vector<float> vTimesTrack(eurocData.totalImages());

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM系统实例，启动全部后台线程；传感器类型IMU_MONOCULAR(单目+IMU)，开启IMU
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::IMU_MONOCULAR, true);

    // 校验SLAM系统初始化是否失败
    if (!exSLAM) {
        cerr << "Failure to initialize ORBSLAM3: " << exSLAM.error().msg() << endl;
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM系统有效指针

    float imageScale = SLAM->GetImageScale(); // 获取配置文件设定的图像缩放系数
    cv::Mat imLeft, imRight;                  // 双目图像变量，本demo只用左图
    size_t nseq = 0;                          // 当前正在处理的序列编号
    for (auto const &seq : eurocData.mvSequences) { // 遍历每一条数据集序列
        if (nseq > 0) {
            cout << "Changing the dataset" << endl;
            SLAM->ChangeDataset(); // 切换数据集接口，用于多序列连续运行
        }

        // Seq loop 单条序列内部循环处理每一帧
        vector<ORB_SLAM3::IMU::Point> vImuMeas; // 存放当前帧之前、上一帧之后的IMU测量值
        double t_rect = 0.f;                    // 校正耗时（本demo未使用）
        double t_resize = 0.f;                  // 图像resize耗时，被注释的代码使用
        double t_track = 0.f;                   // 单帧总处理耗时(毫秒)
        int ni = 0;                             // 当前序列内部帧索引

        auto imuIt = seq.vImu.begin();          // IMU迭代器，记录已经消费到哪个IMU样本

        cout << "Loaded " << seq.vImu.size() << " imu points and " << seq.size()
             << " images" << endl;

        for (auto const &imgSet : seq.mvImageSets) { // 遍历序列内每一组双目图像帧
            cout << "=== Processing image " << ni << " of " << seq.size() << " at "
                 << std::setprecision(17) << imgSet.mTimestamp << " ===" << endl;

            cv::Mat im = imgSet.leftImage(); // 读取左目图像

            // 图像读取失败直接返回退出
            if (im.empty()) {
                cerr << "Failed to load image at: " << imgSet.mTimestamp << endl;
                return 1;
            }

            //       if (imageScale != 1.f) {
            // #ifdef REGISTER_TIMES
            //         std::chrono::steady_clock::time_point t_Start_Resize =
            //             std::chrono::steady_clock::now();
            // #endif
            //         int width = im.cols * imageScale;
            //         int height = im.rows * imageScale;
            //         cv::resize(im, im, cv::Size(width, height));
            // #ifdef REGISTER_TIMES
            //         std::chrono::steady_clock::time_point t_End_Resize =
            //             std::chrono::steady_clock::now();

            //         t_resize = std::chrono::duration_cast<
            //                        std::chrono::duration<double, std::milli> >(
            //                        t_End_Resize - t_Start_Resize)
            //                        .count();
            //         SLAM->InsertResizeTime(t_resize);
            // #endif
            //       }

            // Load imu measurements from previous frame 清空IMU测量容器，准备收集本帧需要的IMU数据
            vImuMeas.clear();

            // 不是第0帧，并且IMU迭代器没有走到末尾
            if ((ni > 0) && (imuIt != seq.vImu.end())) {
                // Retain the deep copy for now...
                // 查找第一个时间戳大于当前图像帧时间的IMU点，作为IMU区间结束位置
                auto imuEnd =
                    std::find_if(imuIt, seq.vImu.end(),
                                 [&](const ORB_SLAM3::IMU::Point &pt) -> bool {
                                     return pt.t > imgSet.mTimestamp;
                                 });
                // 将[imuIt, imuEnd)之间所有IMU样本拷贝到vImuMeas，即上一帧到当前帧之间IMU数据
                std::copy(imuIt, imuEnd, std::back_inserter(vImuMeas));
                imuIt = imuEnd; // 更新迭代器，标记已经消费到该位置，下一帧从此处继续

                cout << "Using " << vImuMeas.size() << " IMU measurements" << endl;
            }

            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now(); // 记录跟踪开始时刻

            // 传入单目图像、图像时间戳、对应区间IMU测量，执行跟踪；TODO注释提示后续替换为monocular_inertial接口
            SLAM->TrackMonocular(im, imgSet.mTimestamp,
                                vImuMeas);  // TODO change to monocular_inertial

            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now(); // 记录跟踪结束时刻

#ifdef REGISTER_TIMES
            // 开启计时宏，统计resize+跟踪总耗时，送入SLAM内部耗时统计模块
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

            vTimesTrack[ni] = ttrack; // 保存本帧跟踪耗时到统计数组
            const double dt = seq.dt(); // 获取序列帧间时间间隔
            if (ttrack < dt) usleep((dt - ttrack) * 1e6); // 如果处理比帧间隔快，休眠补齐时间，模拟真实相机帧率

            ni++;
        }

        nseq++;
    }

    // Stop all threads 全部序列处理完成，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Save camera trajectory 保存全局轨迹，输出TUM格式轨迹文件
    if (bFileName) {
        // 用户指定输出文件名
        const string kf_file = "kf_" + trajFileName + ".txt";
        const string f_file = "f_" + trajFileName + ".txt";
        SLAM->SaveTrajectoryTUM(f_file);
        SLAM->SaveKeyFrameTrajectoryTUM(kf_file);
    } else {
        // 未指定文件名，使用默认文件名输出
        SLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
        SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    return 0;
}
