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
#include <System.h>                     // ORB‑SLAM3系统主头文件，SystemFactory工厂类接口
#include <algorithm>                    // STL通用算法库
#include <chrono>                       // C++高精度时钟，用于统计各阶段耗时
#include <fstream>                      // 文件读写流
#include <iomanip>                      // IO输出格式控制
#include <iostream>                     // cout/cerr标准输入输出
#include <opencv2/core/core.hpp>        // OpenCV核心模块，Mat图像数据结构

#include "euroc_common.h"               // EuRoC数据集加载工具头文件

using namespace std;

/**
 * @brief 纯双目STEREO模式运行EuRoC数据集主程序入口，不使用IMU
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0正常，非0异常
 */
int main(int argc, char **argv) {
    setupEurocSpdLogger();              // 初始化spdlog日志器，配置控制台彩色输出
    spdlog::set_level(spdlog::level::debug); // 设置日志等级为debug，debug及以上日志全部打印

    // 命令行参数合法性校验，参数不足打印使用帮助并退出程序
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

    // 计算待加载数据集序列数量，argv[1]词袋文件，argv[2]配置文件，后续每两个参数对应一组序列
    const int num_seq = (argc - 3) / 2;
    spdlog::info("Loading {} sequences", num_seq);

    // 判断末尾参数是否为轨迹输出文件名；剩余参数总数为奇数代表末尾附带输出文件名
    bool bFileName = (((argc - 3) % 2) == 1);
    string trajFileName;
    if (bFileName) {
        trajFileName = string(argv[argc - 1]); // 获取用户指定的轨迹输出文件名
        spdlog::info("Saving outputs to: {}", trajFileName);
    }

    // Load all sequences: 组装全部序列路径配置
    int seq;
    vector<EuRoCData::SequencePaths> imagePaths; // 存放每条数据集序列路径信息
    for (seq = 0; seq < num_seq; seq++) {
        spdlog::info("Loading images for sequence {} ....", seq);

        // 解析一组序列参数：图像根目录、图像时间戳文件路径
        const string pathSeq(argv[(2 * seq) + 3]);
        const string pathTimeStamps(argv[(2 * seq) + 4]);

        imagePaths.emplace_back(pathSeq, pathTimeStamps);
    }

    // 加载EuRoC图像序列，第二个参数默认false，不加载IMU数据
    auto eurocData = EuRoCData::LoadSequences(imagePaths);

    // Vector for tracking time statistics 跟踪耗时统计容器，保存每帧跟踪耗时
    vector<float> vTimesTrack(eurocData.totalImages());

    spdlog::info("------");

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames. 创建SLAM系统实例，启动全部后台线程；传感器类型STEREO纯双目，true开启多数据集模式
    auto exSLAM = ORB_SLAM3::SystemFactory::create(
        argv[1], argv[2], ORB_SLAM3::SensorType::STEREO, true);

    // 校验SLAM系统初始化是否失败，打印错误信息后退出
    if (!exSLAM) {
        spdlog::error("Failed to initialize ORBSLAM3: {}", exSLAM.error().msg());
        exit(-1);
    }

    auto SLAM = exSLAM.value(); // 获取SLAM系统有效智能指针

    cv::Mat imLeft, imRight;    // 存储左目、右目读取出来的图像
    size_t nseq = 0;            // 当前正在处理的序列编号
    for (auto const &seq : eurocData.mvSequences) { // 遍历每一条数据集序列
        if (nseq > 0) {
            spdlog::info("Changing dataset");
            SLAM->ChangeDataset(); // 切换数据集接口，用于多序列连续运行
        }

        // Seq loop 单条序列内部循环处理每一帧双目图像
        double t_resize = 0;     // 图像缩放耗时，本示例未实际使用
        double t_rect = 0;      // 双目校正耗时，本示例未实际使用
        double t_track = 0;     // 单帧总处理耗时，单位毫秒
        int ni = 0;             // 当前序列内部帧索引

        for (auto const &imgSet : seq.mvImageSets) { // 遍历序列内每一组双目帧
            spdlog::info("=== Processing image {} of {} at {:.9}", ni, seq.size(),
                         imgSet.mTimestamp);

            // Read left and right images from file 读取左、右目图像
            imLeft = imgSet.leftImage();
            imRight = imgSet.rightImage();

            // 左图读取失败，打印错误退出
            if (imLeft.empty()) {
                spdlog::error("Failed to load left image at: {}", imgSet.mTimestamp);
                exit(-1);
            }

            // 右图读取失败，打印错误退出
            if (imRight.empty()) {
                spdlog::error("Failed to load righ timage at: {}", imgSet.mTimestamp);
                exit(-1);
            }

            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now(); // 记录跟踪开始时刻

            // Pass the images to the SLAM system
            // 传入双目图像、时间戳、空IMU容器、左图路径，执行双目跟踪
            SLAM->TrackStereo(imLeft, imRight, imgSet.mTimestamp,
                              vector<ORB_SLAM3::IMU::Point>(), imgSet.mLeftImage);

            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now(); // 记录跟踪结束时刻

#ifdef REGISTER_TIMES
            // 开启REGISTER_TIMES宏时，统计resize+校正+跟踪总耗时，送入SLAM耗时统计模块
            t_track = t_resize + t_rect +
                      std::chrono::duration_cast<
                          std::chrono::duration<double, std::milli> >(t2 - t1)
                          .count();
            SLAM->InsertTrackTime(t_track);
#endif

            // 计算本帧跟踪耗时，单位秒
            double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                    .count();

            vTimesTrack[ni] = ttrack; // 保存本帧跟踪耗时到统计数组

            const double dt = seq.dt(); // 获取序列帧间时间间隔
            if (ttrack < dt) usleep((dt - ttrack) * 1e6); // 处理速度快于帧间隔则休眠补齐，模拟真实相机帧率
            ni++;
        }

        nseq++;
    }
    // Stop all threads 全部序列处理完毕，关闭SLAM所有后台线程
    SLAM->Shutdown();

    // Save camera trajectory 输出TUM格式轨迹文件
    if (trajFileName.size() > 0) {
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
