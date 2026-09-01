> 说明：这是一份第三方fork分支的README译文，前面是开发者的修改说明，后面是ORB‑SLAM3官方原版README。链接、文件路径保留原样，代码片段、命令行保持原始格式，专业术语沿用SLAM领域中文习惯。

---

# 说明
这是我个人用于开发调试的 ORB‑SLAM3 分支，是将 ORB‑SLAM3 集成到 ROS2 项目的一部分。本仓库与原作者上游仓库存在间接关联，**仓库内部不包含任何ROS2专属代码**。真正的ROS2集成实现位于：https://gitlab.com/apl-ocean-engineering/orbslam3_ros2，该仓库将本仓库作为子模块引入。

## 编译构建
我仅在 Ubuntu 24.04 下做过测试。

如上所述，我尽可能优先使用系统软件包；对于没有预编译二进制包的依赖（ROS以外），使用 vcpkg 进行管理。如果你想改变该行为，让 vcpkg 从源码编译更多依赖包，请删除 `file:///D:/ORB_SLAM3-main/vcpkg_overlays/` 目录下对应的目录项。

我完全转向使用 Ninja 作为构建工具。

在Ubuntu下编译，直接使用提供的便捷脚本：
```bash
./install_apt_dependencies.sh
./build.sh
```
脚本会编译 Release 版本的 ORB_SLAM3 库以及全部示例程序。可执行文件生成在各个 `Example/...` 目录下的 Release 子目录中。

## 代码变更
随着对源码理解加深，我做了较多代码风格层面的改动。我的目标只做正向改进：提升代码可读性、可移植性与性能，但实际效果因人而异（YMMV）：
- 启动C++现代化改造，当前目标为 C++17
- 绝大多数场景下，将裸指针替换为智能托管指针
- 对 `System` 与配置类 `Setting` 的初始化流程做小幅更新：主要将配置对象（从文件或其他来源生成）的创建逻辑与System系统初始化逻辑解耦；在初始化阶段提供更多捕获与上报错误的途径。详见 `file:///D:/ORB_SLAM3-main/Examples/`。
- 新增配置文件 `file:///D:/ORB_SLAM3-main/.pre‑commit‑config.yaml`，由此带来大量文本格式改动。

### 依赖清理
- 移除内置的 g2o 与 Sophus 源码，改为通过依赖管理器获取：非ROS环境使用vcpkg；ROS环境使用rosdep。
- 引入第三方库：https://github.com/TartanLlama/expected，遵循 CC0 公有领域协议 http://creativecommons.org/publicdomain/zero/1.0/；未来切换到C++20标准后可能移除该库。

当前仅支持 Ubuntu 24.04，构建策略如下：
1. **ROS2编译场景**：使用 https://gitlab.com/apl-ocean‑engineering/orbslam3_ros2，该仓库把本仓库作为子模块。g2o、Sophus、Pangolin 等依赖通过rosdep从ROS的apt软件源获取。
2. **非ROS编译场景**：使用 vcpkg 作为依赖管理器，用于编译apt源无法安装的依赖（如Pangolin）。同时使用overlay机制，只要条件允许优先使用系统apt版本的包（例如ffmpeg）。
3. 部分库（g2o及其依赖）强制使用vcpkg版本，保证依赖版本同步一致。

其他小改动：
- 删除内置Realsense相机支持；Realsense相关可执行程序应当独立放在单独软件包中。

---

> 下面是原作者的README文档

# ORB‑SLAM3
V1.0，2021‑12‑22

作者：Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, http://webdiis.unizar.es/~josemari/ , http://webdiis.unizar.es/~jdtardos/

版本更新日志见：https://github.com/UZ‑SLAMLab/ORB_SLAM3/blob/master/Changelog.md，里面记录各版本新增特性。

ORB‑SLAM3 是一套实时SLAM开源库，支持**视觉SLAM、视觉‑惯性SLAM、多地图SLAM**；兼容单目、双目、RGB‑D相机，支持针孔相机与鱼眼镜头模型。在全部传感器配置下，ORB‑SLAM3 具备与文献中顶尖系统相当的鲁棒性，同时精度显著更高。

提供示例程序，可在 EuRoC 数据集上运行双目/单目，可开启/关闭IMU；也可在 TUM‑VI 数据集运行鱼眼双目/单目，可开启/关闭IMU。部分运行效果视频：https://www.youtube.com/channel/UCXVt‑kXG6T95Z4tVaYlU80Q。

本软件基于 ORB‑SLAM2：https://github.com/raulmur/ORB_SLAM2，由 Raúl Mur‑Artal、Juan D. Tardós、José M. M. Montiel、Dorian Gálvez‑López（DBoW2作者）开发。

## 相关参考文献
[ORB‑SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós. ORB‑SLAM3: An Accurate Open‑Source Library for Visual, Visual‑Inertial and Multi‑Map SLAM. *IEEE Transactions on Robotics*, 37(6):1874‑1890, Dec. 2021. https://arxiv.org/abs/2007.11898

[IMU‑Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós. Inertial‑Only Optimization for Visual‑Inertial Initialization. ICRA 2020. https://arxiv.org/pdf/2003.05766.pdf

[ORBSLAM‑Atlas] Richard Elvira, J. M. M. Montiel and Juan D. Tardós. ORBSLAM‑Atlas: a robust and accurate multi‑map system. IROS 2019. https://arxiv.org/pdf/1908.11585.pdf

[ORBSLAM‑VI] Raúl Mur‑Artal, and Juan D. Tardós. Visual‑inertial monocular SLAM with map reuse. *IEEE Robotics and Automation Letters*, vol. 2 no. 2, pp. 796‑803, 2017. https://arxiv.org/pdf/1610.05949.pdf

[Stereo and RGB‑D] Raúl Mur‑Artal and Juan D. Tardós. ORB‑SLAM2: an Open‑Source SLAM System for Monocular, Stereo and RGB‑D Cameras. *IEEE Transactions on Robotics*, vol. 33, no. 5, pp. 1255‑1262, 2017. https://arxiv.org/pdf/1610.06475.pdf

[Monocular] Raúl Mur‑Artal, José M. M. Montiel and Juan D. Tardós. ORB‑SLAM: A Versatile and Accurate Monocular SLAM System. *IEEE Transactions on Robotics*, vol. 31, no. 5, pp. 1147‑1163, 2015.（2015 IEEE TRO 最佳论文奖）https://arxiv.org/pdf/1502.00956.pdf

[DBoW2 回环检测] Dorian Gálvez‑López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. *IEEE Transactions on Robotics*, vol. 28, no. 5, pp. 1188‑1197, 2012. http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf

## 1. 许可证
ORB‑SLAM3 许可证：https://github.com/UZ‑SLAMLab/ORB_SLAM3/LICENSE。
全部依赖库与对应许可证清单见：https://github.com/UZ‑SLAMLab/ORB_SLAM3/blob/master/Dependencies.md。

商业闭源版本请联系原作者邮箱：orbslam@unizar.es。

学术引用请使用如下BibTeX：
```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB‑SLAM3}: An Accurate Open‑Source Library for Visual, Visual‑Inertial and Multi‑Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\'omez, Juan J. AND Montiel, Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={37},
  number={6},
  pages={1874‑1890},
  year={2021}
}
```

## 2. 依赖环境
官方在 Ubuntu16.04、18.04 完成测试，其他平台也可编译。建议使用性能较强机器（例如i7）保证实时运行、结果更稳定。

- **C++11 / C++0x 编译器**：用到C++11的线程、时间库。
- **Pangolin**：可视化与UI界面。安装：https://github.com/stevenlovegrove/Pangolin
- **OpenCV**：图像处理、特征提取。最低版本3.0；测试版本3.2.0、4.4.0。官网：http://opencv.org/
- **Eigen3**：g2o依赖。最低版本3.1.0。官网：http://eigen.tuxfamily.org/
- **DBoW2、g2o**：原版放在Thirdparty目录；DBoW2用于回环检测，g2o用于非线性优化。
> 本fork分支说明：已经移除内置修改版g2o，改为使用系统版本。
- **Python**：用于计算轨迹与真值对齐，需要numpy模块。
  - Windows：http://www.python.org/downloads/windows
  - Debian/Ubuntu：`sudo apt install libpython2.7‑dev`
  - MacOS：系统自带
- **ROS（可选）**：原版支持ROS1，可处理单目、单目‑惯性、双目、双目‑惯性、RGB‑D输入。仅在Ubuntu18.04 + ROS Melodic测试。
> 本fork分支说明：ROS1支持已移除，ROS2版本参考：https://gitlab.com/apl‑ocean‑engineering/orbslam3_ros2

## 3. 编译 ORB‑SLAM3 库与示例
克隆仓库：
```bash
git clone https://github.com/UZ‑SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

提供脚本 `build.sh` 编译第三方库与ORB‑SLAM3。确认依赖全部装好后执行：
```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
编译完成，`libORB_SLAM3.so` 输出在lib目录；可执行程序输出在Examples目录。

## 4. 使用自己的相机运行ORB‑SLAM3
Examples文件夹包含多个demo与标定配置文件，适配Intel Realsense T265、D435i。
使用自定义相机步骤：
1. 根据 `Calibration_Tutorial.pdf` 完成相机标定，生成你的相机配置文件 `your_camera.yaml`。
2. 修改现有demo适配你的相机模型，重新编译。
3. USB3或对应接口连接相机。
4. 运行程序。以D435i双目惯性为例：
```bash
./Examples/Stereo‑Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo‑Inertial/RealSense_D435i.yaml
```

## 5. EuRoC数据集示例
EuRoC数据集采用两台针孔相机+IMU采集。提供脚本可以运行全部传感器模式。
1. 下载ASL格式数据集序列。
2. 打开项目根目录脚本 `euroc_examples.sh`，修改`pathDatasetEuroc`变量指向数据集解压目录。
3. 运行脚本处理全部序列：
```bash
./euroc_examples
```

### 评估
EuRoC真值基于IMU机体坐标系；纯视觉模式输出轨迹以左相机为坐标系。evaluation文件夹提供真值转换到左相机坐标系的转换文件；视觉‑惯性模式直接使用数据集原始真值。

运行脚本计算RMS‑ATE误差：
```bash
./euroc_eval_examples
```

## 6. TUM‑VI数据集示例
TUM‑VI数据集使用两台鱼眼相机+IMU采集。
1. 下载数据集并解压。
2. 打开脚本 `tum_vi_examples.sh`，修改`pathDatasetTUM_VI`指向解压目录。
3. 运行：
```bash
./tum_vi_examples
```

### 评估
TUM‑VI仅在场景起始与结束位置提供真值，误差主要反映序列末端漂移。
计算RMS‑ATE：
```bash
./tum_vi_eval_examples
```

## 7. ROS示例（原版ROS1）
编译单目、单目‑惯性、双目、双目‑惯性、RGB‑D节点，测试环境 Ubuntu18.04 + ROS Melodic。

1. 将 `Examples/ROS/ORB_SLAM3` 添加到 `ROS_PACKAGE_PATH`，修改`.bashrc`：
```bash
gedit ~/.bashrc
```
末尾添加，把PATH替换为你的仓库路径：
```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
```
2. 执行编译脚本：
```bash
chmod +x build_ros.sh
./build_ros.sh
```

**运行单目节点**
话题输入 `/camera/image_raw`，需要词袋文件与配置文件：
```bash
rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```

**运行单目‑惯性节点**
图像话题 `/camera/image_raw`，IMU话题 `/imu`；第三个可选参数传true开启CLAHE图像均衡（多用于TUM‑VI数据集）：
```bash
rosrun ORB_SLAM3 Mono_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION]
```

**运行双目节点**
左图 `/camera/left/image_raw`，右图 `/camera/right/image_raw`。针孔相机：配置文件给出校正矩阵，程序在线校正图像；否则图像必须预先校正。鱼眼相机不需要校正，直接使用原图。
```bash
rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
```

**运行双目‑惯性节点**
双目图像话题 + IMU话题；校正逻辑同双目节点：
```bash
rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION [EQUALIZATION]
```

**运行RGB‑D节点**
RGB话题 `/camera/rgb/image_raw`，深度话题 `/camera/depth_registered/image_raw`：
```bash
rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```

**ROS bag完整示例（双目‑惯性）**
下载EuRoC的bag包，打开三个终端分别执行：
```bash
roscore
rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo‑Inertial/EuRoC.yaml true
rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
```
等待ORB‑SLAM3加载词袋，在rosbag终端按空格开始回放。

> 备注：TUM‑VI数据集bag文件会因块大小产生回放异常，可以重新打包使用默认块大小：
```bash
rosrun rosbag fastrebag.py dataset‑room1_512_16.bag dataset‑room1_512_16_small_chunks.bag
```

## 8. 运行耗时统计
修改头文件 `include/Config.h`，取消注释宏 `#define REGISTER_TIMES`，开启计时统计。运行结束终端输出耗时统计，同时保存到文本文件 `ExecTimeMean.txt`。

## 9. 相机标定
视觉‑惯性标定教程、配置文件字段说明见文档 `Calibration_Tutorial.pdf`。