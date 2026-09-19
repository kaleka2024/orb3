---
### 说明
这是一个面向 ORB-SLAM3 的个人开发调试分支，属于 ORB-SLAM3 向 ROS2 项目集成工作的一部分。本仓库与原作者的上游仓库为间接关联，**仓库本身不包含任何 ROS2 专属代码**。完整的 ROS2 集成实现位于：[https://gitlab.com/apl-ocean-engineering/orbslam3_ros2](https://gitlab.com/apl-ocean-engineering/orbslam3_ros2)，该仓库将本仓库作为子模块引入使用。

#### 编译构建
本分支仅在 Ubuntu 24.04 环境下完成过测试。
如前文所述，依赖项优先使用系统软件包；对于没有预编译二进制包的依赖（ROS 相关除外），通过 vcpkg 进行管理。如果希望调整该策略，让 vcpkg 从源码编译更多依赖包，删除 `file:///D:/ORB_SLAM3-main/vcpkg_overlays/` 目录下对应的目录项即可。

本分支已全面切换为 Ninja 构建工具。
在 Ubuntu 系统下编译，可直接使用提供的便捷脚本：
```bash
./install_apt_dependencies.sh
./build.sh
```
脚本会编译 Release 版本的 ORB_SLAM3 库以及全部示例程序。可执行文件生成在各 `Example/...` 目录下的 Release 子目录中。

#### 代码变更
随着对源码理解的深入，作者做了较多代码风格层面的改动。改动目标仅为正向改进：提升代码可读性、可移植性与运行性能，但实际效果因人而异（YMMV）：
- 启动 C++ 现代化改造，当前目标标准为 C++17
- 绝大多数场景下，将原生裸指针替换为智能托管指针
- 对 `System` 核心系统类与配置类 `Setting` 的初始化流程做了小幅更新：核心是将配置对象（从文件或其他来源生成）的创建逻辑与 System 系统初始化逻辑解耦；在初始化阶段提供更多错误捕获与上报的途径。详见 `file:///D:/ORB_SLAM3-main/Examples/`。
- 新增配置文件 `file:///D:/ORB_SLAM3-main/.pre-commit-config.yaml`，由此带来大量文本格式类改动。

##### 依赖清理
- 移除了仓库内置的 g2o 与 Sophus 源码，改为通过依赖管理器获取：非 ROS 环境使用 vcpkg；ROS 环境使用 rosdep。
- 引入第三方工具库：[https://github.com/TartanLlama/expected](https://github.com/TartanLlama/expected)，遵循 CC0 公有领域协议 [http://creativecommons.org/publicdomain/zero/1.0/](http://creativecommons.org/publicdomain/zero/1.0/)；未来切换到 C++20 标准后可能移除该库。

当前仅支持 Ubuntu 24.04，构建策略分为三类：
1.  **ROS2 编译场景**：使用 [https://gitlab.com/apl-ocean-engineering/orbslam3_ros2](https://gitlab.com/apl-ocean-engineering/orbslam3_ros2) 仓库，该仓库将本仓库作为子模块引入。g2o、Sophus、Pangolin 等依赖通过 rosdep 从 ROS 的 apt 软件源获取。
2.  **非 ROS 编译场景**：使用 vcpkg 作为依赖管理器，用于编译 apt 源无法安装的依赖（如 Pangolin）。同时使用 overlay 机制，只要条件允许就优先使用系统 apt 版本的包（例如 ffmpeg）。
3.  部分库（g2o 及其依赖）强制使用 vcpkg 版本，保证依赖版本同步一致。

其他细节改动：
- 删除了内置的 Realsense 相机支持；Realsense 相关可执行程序应当独立放在单独的软件包中。

---

> 以下为原作者官方 README 文档

# ORB-SLAM3
版本 V1.0，发布于 2021 年 12 月 22 日
作者：Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez，主页：[http://webdiis.unizar.es/~josemari/](http://webdiis.unizar.es/~josemari/) 、[http://webdiis.unizar.es/~jdtardos/](http://webdiis.unizar.es/~jdtardos/)
版本更新日志见：[https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md)，记录了各版本新增特性。

ORB-SLAM3 是一套实时 SLAM（同步定位与地图构建）开源库，支持**纯视觉 SLAM、视觉-惯性 SLAM、多地图 SLAM** 三种模式；兼容单目、双目、RGB-D 三类相机，同时支持针孔相机与鱼眼镜头模型。在全部传感器配置下，ORB-SLAM3 都具备与学界顶尖系统相当的鲁棒性，同时定位精度显著更高。

项目提供了示例程序，可在 EuRoC 数据集上运行双目/单目模式，支持开启或关闭 IMU 融合；也可在 TUM-VI 数据集上运行鱼眼双目/单目模式，支持开启或关闭 IMU 融合。
部分运行效果演示视频：[https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q)。

本软件基于 ORB-SLAM2 开发：[https://github.com/raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)，原作者为 Raúl Mur-Artal、Juan D. Tardós、José M. M. Montiel、Dorian Gálvez-López（DBoW2 词袋库作者）。

## 相关参考文献
[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós. ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM. *IEEE Transactions on Robotics*, 37(6):1874-1890, Dec. 2021. [https://arxiv.org/abs/2007.11898](https://arxiv.org/abs/2007.11898)
[IMU 初始化] Carlos Campos, J. M. M. Montiel and Juan D. Tardós. Inertial-Only Optimization for Visual-Inertial Initialization. ICRA 2020. [https://arxiv.org/pdf/2003.05766.pdf](https://arxiv.org/pdf/2003.05766.pdf)
[ORBSLAM-Atlas 多地图] Richard Elvira, J. M. M. Montiel and Juan D. Tardós. ORBSLAM-Atlas: a robust and accurate multi-map system. IROS 2019. [https://arxiv.org/pdf/1908.11585.pdf](https://arxiv.org/pdf/1908.11585.pdf)
[ORBSLAM-VI 视觉惯性] Raúl Mur-Artal, and Juan D. Tardós. Visual-inertial monocular SLAM with map reuse. *IEEE Robotics and Automation Letters*, vol. 2 no. 2, pp. 796-803, 2017. [https://arxiv.org/pdf/1610.05949.pdf](https://arxiv.org/pdf/1610.05949.pdf)
[双目与 RGB-D] Raúl Mur-Artal and Juan D. Tardós. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras. *IEEE Transactions on Robotics*, vol. 33, no. 5, pp. 1255-1262, 2017. [https://arxiv.org/pdf/1610.06475.pdf](https://arxiv.org/pdf/1610.06475.pdf)
[单目 SLAM] Raúl Mur-Artal, José M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. *IEEE Transactions on Robotics*, vol. 31, no. 5, pp. 1147-1163, 2015.（2015 IEEE TRO 最佳论文奖）[https://arxiv.org/pdf/1502.00956.pdf](https://arxiv.org/pdf/1502.00956.pdf)
[DBoW2 回环检测] Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. *IEEE Transactions on Robotics*, vol. 28, no. 5, pp. 1188-1197, 2012. [http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)

## 1. 许可证
ORB-SLAM3 开源许可证：[https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE)。
全部依赖库及对应许可证清单见：[https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md)。
商业闭源版本授权请联系原作者邮箱：orbslam@unizar.es。

学术引用请使用如下 BibTeX 格式：
```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\'omez, Juan J. AND Montiel, Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
}
```

## 2. 依赖环境
官方测试环境为 Ubuntu 16.04、18.04，其他平台也可编译。建议使用性能较强的机器（如 i7 处理器）以保证实时运行效果与结果稳定性。
- **C++11 / C++0x 编译器**：用到 C++11 标准的线程、时间库。
- **Pangolin**：用于可视化与 UI 交互。安装地址：[https://github.com/stevenlovegrove/Pangolin](https://github.com/stevenlovegrove/Pangolin)
- **OpenCV**：用于图像处理、特征提取。最低版本 3.0；测试兼容版本 3.2.0、4.4.0。官网：[http://opencv.org/](http://opencv.org/)
- **Eigen3**：g2o 优化库的依赖。最低版本 3.1.0。官网：[http://eigen.tuxfamily.org/](http://eigen.tuxfamily.org/)
- **DBoW2、g2o**：原版仓库内置在 Thirdparty 目录中；DBoW2 用于回环检测，g2o 用于非线性束调整优化。
> 本 fork 分支说明：已移除内置修改版 g2o，改为使用系统版本。
- **Python**：用于计算轨迹与真值对齐，需要 numpy 模块。
  - Windows：[http://www.python.org/downloads/windows](http://www.python.org/downloads/windows)
  - Debian/Ubuntu：`sudo apt install libpython2.7-dev`
  - MacOS：系统自带
- **ROS（可选）**：官方原版支持 ROS1，可处理单目、单目-惯性、双目、双目-惯性、RGB-D 输入流。仅在 Ubuntu 18.04 + ROS Melodic 环境下完成测试。
> 本 fork 分支说明：已移除 ROS1 支持，ROS2 适配版本参考：[https://gitlab.com/apl-ocean-engineering/orbslam3_ros2](https://gitlab.com/apl-ocean-engineering/orbslam3_ros2)

## 3. 编译 ORB-SLAM3 库与示例
克隆仓库：
```bash
git clone [https://github.com/UZ-SLAMLab/ORB_SLAM3.git](https://github.com/UZ-SLAMLab/ORB_SLAM3.git) ORB_SLAM3
```
项目提供 `build.sh` 脚本用于编译第三方库与 ORB-SLAM3 本体。确认全部依赖安装完成后执行：
```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
编译完成后，`libORB_SLAM3.so` 库文件输出在 lib 目录；可执行程序输出在 Examples 目录下。

## 4. 使用自定义相机运行 ORB-SLAM3
Examples 文件夹包含多个演示程序与标定配置文件，适配 Intel Realsense T265、D435i 等相机。

使用自定义相机的步骤：
1.  参照 `Calibration_Tutorial.pdf` 完成相机标定，生成你的相机配置文件 `your_camera.yaml`。
2.  修改现有演示程序适配你的相机模型，重新编译。
3.  通过 USB3 或对应接口连接相机。
4.  运行程序。以 D435i 双目惯性模式为例：
```bash
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```

## 5. EuRoC 数据集示例
EuRoC 数据集由两台针孔相机+IMU 采集而成。项目提供脚本可运行全部传感器模式。
1.  下载 ASL 格式的数据集序列。
2.  打开项目根目录下的 `euroc_examples.sh` 脚本，修改 `pathDatasetEuroc` 变量指向数据集解压目录。
3.  运行脚本处理全部序列：
```bash
./euroc_examples
```

### 精度评估
EuRoC 数据集的真值基于 IMU 机体坐标系；纯视觉模式输出的轨迹以左相机为坐标系。evaluation 文件夹提供了真值到左相机坐标系的转换文件；视觉-惯性模式可直接使用数据集原始真值。

运行脚本计算 RMS-ATE（绝对轨迹误差均方根）：
```bash
./euroc_eval_examples
```

## 6. TUM-VI 数据集示例
TUM-VI 数据集由两台鱼眼相机+IMU 采集而成。
1.  下载数据集并解压。
2.  打开脚本 `tum_vi_examples.sh`，修改 `pathDatasetTUM_VI` 指向解压目录。
3.  运行：
```bash
./tum_vi_examples
```

### 精度评估
TUM-VI 仅在场景起始与结束位置提供真值，误差主要反映序列末端的漂移量。
计算 RMS-ATE：
```bash
./tum_vi_eval_examples
```

## 7. ROS 示例（官方原版 ROS1）
可编译单目、单目-惯性、双目、双目-惯性、RGB-D 节点，测试环境为 Ubuntu 18.04 + ROS Melodic。
1.  将 `Examples/ROS/ORB_SLAM3` 添加到 `ROS_PACKAGE_PATH`，修改 `.bashrc`：
```bash
gedit ~/.bashrc
```
在末尾添加以下内容，将 PATH 替换为你的仓库路径：
```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
```
2.  执行编译脚本：
```bash
chmod +x build_ros.sh
./build_ros.sh
```

**运行单目节点**
图像输入话题 `/camera/image_raw`，需要传入词袋文件路径与配置文件路径：
```bash
rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```

**运行单目-惯性节点**
图像话题 `/camera/image_raw`，IMU 话题 `/imu`；第三个可选参数传 true 可开启 CLAHE 图像均衡（多用于 TUM-VI 数据集）：
```bash
rosrun ORB_SLAM3 Mono_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION]
```

**运行双目节点**
左图话题 `/camera/left/image_raw`，右图话题 `/camera/right/image_raw`。针孔相机模式：配置文件给出校正矩阵，程序在线校正图像；否则图像必须预先校正。鱼眼相机不需要校正，直接使用原图。
```bash
rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
```

**运行双目-惯性节点**
双目图像话题 + IMU 话题；校正逻辑同双目节点：
```bash
rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION [EQUALIZATION]
```

**运行 RGB-D 节点**
RGB 图像话题 `/camera/rgb/image_raw`，深度图像话题 `/camera/depth_registered/image_raw`：
```bash
rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```

**ROS bag 完整示例（双目-惯性模式）**
下载 EuRoC 的 bag 包，打开三个终端分别执行：
```bash
roscore
rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
```
等待 ORB-SLAM3 加载完词袋，在 rosbag 终端按空格开始回放。

> 备注：TUM-VI 数据集的 bag 文件会因块大小问题产生回放异常，可以重新打包为默认块大小使用：
```bash
rosrun rosbag fastrebag.py dataset-room1_512_16.bag dataset-room1_512_16_small_chunks.bag
```

## 8. 运行耗时统计
修改头文件 `include/Config.h`，取消注释宏 `#define REGISTER_TIMES`，即可开启计时统计。运行结束后终端会输出耗时统计，同时保存到文本文件 `ExecTimeMean.txt`。

## 9. 相机标定
视觉-惯性标定教程、配置文件字段说明见文档 `Calibration_Tutorial.pdf`。

---

# 深度讲解：ORB-SLAM 生态与本分支的来龙去脉
## 第一步：ORB-SLAM 系列的技术演进脉络
ORB-SLAM 系列出自西班牙萨拉戈萨大学（Universidad de Zaragoza）的机器人与实时系统研究组（RRTG），核心主导者为 José M. M. Montiel 教授与 Juan D. Tardós 教授，是视觉 SLAM 领域最具影响力的开源项目之一，整个演进路径清晰且层层递进：

### 阶段1：奠基——ORB-SLAM（2015）
2015 年，Raúl Mur-Artal 在两位教授指导下发布 ORB-SLAM（后世称 ORB-SLAM1），首次将 ORB 特征、DBoW2 词袋回环检测、关键帧束调整整合为一套完整的单目 SLAM 系统，拿下当年 IEEE TRO 最佳论文奖。它解决了当时单目 SLAM 鲁棒性差、无法有效回环的痛点，成为首个真正工程化可用的纯单目开源 SLAM 方案。

### 阶段2：扩展——ORB-SLAM2（2017）
在单目基础上扩展支持双目、RGB-D 相机，新增了重定位、地图复用、轻量级定位模式，支持室内外多种场景。这一版成为了 SLAM 领域的事实标准，被广泛用于机器人、AR、无人机等领域，也是绝大多数开发者接触最多的版本。

### 阶段3：探索——视觉惯性与多地图（2017-2020）
团队没有止步于纯视觉，向两个方向突破：
- **视觉惯性融合**：2017 年发布 ORB-SLAM-VI，首次实现紧耦合的视觉惯性 SLAM 与地图复用；2020 年进一步提出纯惯性优化初始化方法，解决了视觉惯性 SLAM 初始化慢、精度差的行业难题。
- **多地图系统**：2019 年发布 ORBSLAM-Atlas，支持跟踪丢失后自动新建子地图、重回已建区域时无缝合并，解决了长时序、大场景下跟踪丢失后无法恢复的痛点。

### 阶段4：集大成——ORB-SLAM3（2021）
将前面所有技术整合，正式推出 ORB-SLAM3，核心突破有三点：
1.  **全模式紧耦合视觉惯性 SLAM**：基于最大后验估计（MAP）的紧耦合融合，即使在 IMU 初始化阶段也保持鲁棒，精度比前代方案提升 2-5 倍，EuRoC 数据集上双目惯性模式平均精度可达 3.6 cm。
2.  **多地图 SLAM**：内置 Atlas 多地图机制，长时间视觉缺失时自动创建新地图，重访旧区域时自动合并，支持多会话建图。
3.  **全相机模型支持**：同时支持针孔、鱼眼相机模型，覆盖单目、双目、RGB-D 全品类传感器。

至此，ORB-SLAM3 成为了开源特征点 SLAM 的精度天花板，也是目前工业界落地应用最广泛的方案之一。

## 第二步：为什么会出现这个第三方 Fork 分支？
ORB-SLAM3 官方版本在 2021 年发布 V1.0 后，核心更新逐渐放缓，且存在几个社区广为诟病的痛点：
1.  **ROS 版本停留在 ROS1**：官方仅支持 ROS1 Melodic，而 ROS2 早已成为行业主流，社区急需 ROS2 适配版本。
2.  **依赖管理混乱**：原版内置修改版的 g2o、DBoW2、Sophus 源码，嵌套在 Thirdparty 目录里，版本老旧，编译麻烦，和系统其他依赖容易冲突。
3.  **代码风格老旧**：基于 C++11 标准，大量使用裸指针，错误处理机制简陋，不符合现代 C++ 工程规范。
4.  **构建工具链原始**：纯手写 CMake 脚本，构建慢，跨平台一致性差。

在这样的背景下，社区开发者推出了这个第三方 fork 分支，核心目标是做**现代化工程改造 + ROS2 集成底座**。它本身不写 ROS2 业务代码，只负责把 ORB-SLAM3 本体改造成适合作为子模块被 ROS2 项目调用的形态，真正的 ROS2 节点封装在另一个 GitLab 仓库中。

## 第三步：本分支的技术改造逻辑（一步一步拆解）
这个分支的改动不是随意的代码重构，每一项都针对性解决原版的工程痛点：

### 1. 依赖体系重构：从内置源码到包管理
原版把 g2o、Sophus 等依赖的源码直接放进仓库，好处是开箱即编译，坏处是版本锁死、编译慢、和上层项目依赖冲突。
本分支的策略是：
- 移除全部内置第三方源码，统一用包管理器管理；
- ROS2 场景下用 rosdep 拉取 ROS 生态内的依赖，和整个 ROS 工作空间版本一致；
- 非 ROS 场景下用 vcpkg 管理，同时用 overlay 机制优先调用系统 apt 包，兼顾版本一致性与编译速度；
- g2o 这类核心依赖强制统一用 vcpkg 版本，避免版本不一致导致的 ABI 不兼容。

### 2. C++ 现代化升级：从 C++11 到 C++17
- 裸指针替换为智能指针（`std::unique_ptr`、`std::shared_ptr`），大幅降低内存泄漏、野指针风险，提升代码稳定性；
- 引入 `tl::expected` 库（CC0 协议）替代传统的错误码+返回值模式，让错误处理更清晰、更符合现代 C++ 风格；等未来升级到 C++20 可以直接换成标准库 `std::expected`，平滑过渡。
- 配置与系统初始化解耦：把配置加载和系统启动拆成两步，错误可以逐层上报，而不是原版直接内部断言退出，更适合作为库被上层项目调用。

### 3. 工程化完善
- 切换 Ninja 构建工具，比原版 Make 构建速度更快，增量编译体验更好；
- 加入 pre-commit 代码规范配置，统一代码格式，适合多人协作开发；
- 移除内置 Realsense 相机驱动代码，遵循“核心库和外设驱动分离”的工程原则，让相机支持以独立包的形式存在，核心库更轻量化。

## 第四步：两种编译场景的适用场景
### 场景1：ROS2 开发场景
如果你是做 ROS2 机器人开发，直接用上游的 `orbslam3_ros2` 仓库即可，它会把本仓库作为子模块自动拉取，所有依赖通过 rosdep 安装，和你的 ROS 工作空间无缝集成。适合机器人定位、AR 导航、无人机等 ROS 生态项目。

### 场景2：原生 C++ 开发场景
如果你不使用 ROS，直接用本仓库+vcpkg 编译，得到纯净的 ORB-SLAM3 库，可以集成到你自己的 C++ 应用里。适合嵌入式设备、桌面端应用、定制化硬件平台的 SLAM 部署。

## 第五步：本分支的定位与价值
它不是 ORB-SLAM3 的功能增强版，没有新增算法层面的特性，而是**工程化改良版**。它的价值在于：
- 让 ORB-SLAM3 能更好地融入现代 C++ 工程与 ROS2 生态；
- 降低了二次开发的门槛，错误处理更清晰，接口更规范；
- 统一了依赖管理，解决了原版编译难、依赖冲突的老问题。

对于算法研究，原版 ORB-SLAM3 依然是首选；对于工程落地、产品开发，这个 fork 分支的工程化改进能显著提升开发效率与稳定性。

# 第六步：ORB-SLAM3 原版核心算法模块完整拆解（一步一步运行流程）

> 
> 这是原版ORB-SLAM3的内核执行链路，fork分支只改工程代码，**算法逻辑完全继承原版**。
> ORB-SLAM3整体分为四大核心线程 + Atlas多地图管理模块，多线程并行是它实时性的根本保障：

1. **跟踪线程 Tracking（主线程）**：最高优先级，负责实时读取图像+IMU、特征提取、帧位姿预测、跟踪当前地图；跟踪丢失时触发重定位。
2. **局部建图线程 LocalMapping**：接收Tracking发来的新关键帧，做IMU预积分、视觉惯性优化、新增地图点、剔除冗余关键帧/坏点。
3. **回环与地图合并线程 LoopClosing**：DBoW2词袋检索，识别回环；回环检测成功后执行位姿图优化；跨地图匹配成功则触发Atlas地图合并。
4. **可视化线程 Pangolin Viewer**：渲染相机轨迹、地图点、关键帧，非核心，可关闭提升性能。
5. **Atlas 多地图管理器**：管理多个独立子地图；跟踪丢失一段时间后自动新建子地图；重访旧场景时地图合并。

## 完整运行时序（一步一步）

### 阶段1：系统初始化（分两种模式：纯视觉 / 视觉惯性VI）

#### 分支A：纯视觉初始化（单目最麻烦，双目/RGBD更快）

1. 读取连续图像帧，提取ORB角点特征（FAST角点 + BRIEF描述子）。
2. 等待足够视差：单目必须移动相机产生平移，不能原地旋转；通过两帧匹配特征求解基础矩阵F/单应矩阵H。
3. 恢复两帧相对位姿，三角化生成第一批3D地图点，构建初始地图。> 
> 单目初始化尺度不确定，地图尺度是模糊的；双目/RGBD可以直接得到真实尺度。

#### 分支B：视觉惯性VI初始化（ORB-SLAM3核心创新点）

原版ORB-SLAM2没有这个能力。ORB-SLAM3的VI初始化分两步：

1. **纯惯性优化阶段（ICRA2020论文）**：收集多帧图像+IMU预积分数据，只使用IMU预积分约束，估计陀螺仪bias、加速度计bias、初始速度、重力方向。
2. **视觉惯性联合优化**：把视觉特征约束和IMU预积分约束放到一起优化，求解初始相机位姿、地图点、IMU偏置；一次性恢复全局尺度。> 
> 重点：传统VIO需要缓慢摇动设备做初始化；ORB-SLAM3的纯惯性优化，允许更快完成VI初始化，鲁棒性更强。

初始化成功 → 系统进入正常跟踪模式；初始化失败 → 丢弃当前帧，继续收集数据重试。

### 阶段2：Tracking 实时跟踪（每一帧都执行）

1. 读取图像，如果是VI模式同步读取IMU数据，做IMU预积分。
2. 提取ORB特征；图像做CLAHE直方图均衡（可选，TUM-VI鱼眼场景常用，提升暗光特征）。
3. **位姿预测**
   - 正常跟踪：用上一帧位姿+IMU预积分预测当前帧相机位姿；
   - 刚重定位成功：用重定位得到的位姿作为初始猜测。
4. 特征匹配：把当前帧ORB特征和**局部地图中的地图点**做匹配。
5. 运动仅BA优化（位姿优化）：固定所有3D地图点，只优化当前帧相机位姿，最小化重投影误差+IMU残差（VI模式）。
6. 跟踪状态判断：
   - 匹配点足够：跟踪成功；判断是否满足条件，把当前帧作为**新关键帧**送入LocalMapping线程；
   - 匹配点不足：标记为跟踪丢失。
7. 跟踪丢失后的处理：
   - 启动重定位：用当前帧ORB描述子，DBoW2词袋搜索Atlas所有地图里的候选关键帧；
   - 找到足够匹配候选帧，PnP求解相机位姿，完成重定位，回到正常跟踪；
   - 如果长时间无法重定位：Atlas新建一个子地图，后续跟踪在新地图里继续。

### 阶段3：LocalMapping 局部建图（异步线程，处理关键帧）

> 
> 注意：不是每帧图像都进这里，只有Tracking选出的**关键帧**才会送入。

1. 插入新关键帧，存储IMU预积分数据。
2. 对关键帧特征做三角化，生成新的3D地图点。
3. 剔除坏地图点：观测数太少、重投影误差过大、视差异常的点直接删除。
4. **局部BA优化**：优化当前关键帧+相邻局部关键帧，以及它们关联的地图点；VI模式同时优化IMU bias。
5. 冗余关键帧剔除：如果一个关键帧90%以上地图点可以被其他关键帧观测到，就删除该关键帧，控制地图规模，防止优化越来越慢。

### 阶段4：LoopClosing 回环检测 + Atlas地图合并（异步线程）

1. DBoW2词袋：把关键帧ORB描述子转为词袋向量，检索数据库，寻找外观相似的候选关键帧。
2. 几何校验：词袋相似只是外观相似，必须做特征匹配+RANSAC几何验证，排除假回环。
3. 回环校正：计算回环帧之间的位姿偏差，执行**位姿图优化**，消除长时间累积漂移。
4. Atlas地图合并：如果检测到当前帧匹配的关键帧属于**另一个子地图**，不再只是回环校正，而是触发地图合并，把两个子地图融合成一张大地图。> 
> 这就是ORB-SLAM3“多地图Atlas”的核心：长时间运行，跟踪反复丢失重建多个子地图，回到旧区域时自动合并地图。

### 阶段5：输出轨迹与可视化

Pangolin窗口实时显示相机位姿、彩色3D地图点、关键帧；运行结束，可以导出相机轨迹txt文件，用evo评估ATE/RPE误差。

---

# 第七步：原版ORB-SLAM3 VS 这个现代化fork分支，差异对照表

| 维度 | ORB-SLAM3 官方原版（2021 V1.0） | 本第三方fork分支（Ubuntu24.04，面向ROS2） |
| --- | --- | --- |
| C++标准 | C++11 | C++17（目标未来C++20） |
| 指针管理 | 大量裸指针，内存风险高 | 大规模替换为unique_ptr/shared_ptr智能指针 |
| 依赖管理 | g2o、Sophus内置Thirdparty目录，源码内嵌 | 删除内置第三方库，使用vcpkg/rosdep外部包管理 |
| 错误处理 | 大量assert断言，出错直接程序崩溃 | 解耦配置加载与System初始化，增加错误上报；引入tl::expected |
| 构建系统 | CMake + make | CMake + Ninja，构建速度更快 |
| ROS支持 | ROS1 Melodic，原生ROS节点 | 删除ROS1；本体无ROS代码，ROS2封装放在独立仓库作为子模块 |
| 相机硬件 | 内置Realsense D435i/T265驱动Demo | 删除内置Realsense代码，相机驱动独立分包 |
| 算法内核 | ORB-SLAM3全套算法（VI、Atlas多地图） | **算法完全不变，只改工程层代码** |
| 目标系统 | Ubuntu16.04 / 18.04 | Ubuntu24.04 |
| 代码规范 | 无统一pre-commit格式化 | 增加pre-commit配置，自动格式化代码 |

> 
> 一句话总结：**fork分支不改SLAM算法，只做软件工程现代化改造，解决原版“编译难、依赖打架、裸指针不安全、不支持ROS2”的工程痛点。**

# 第八步：ORB-SLAM3 VS XRSLAM 横向对比（承接上一份XRSLAM文档）

> 
> 你前面看了XRSLAM，这里做对照，方便选型：

## 1）底层路线差异

- **ORB-SLAM3**：**特征点法SLAM**。提取ORB稀疏角点；基于词袋回环；Atlas多地图；紧耦合VI。优势：成熟、精度高、回环可靠；劣势：依赖稳定角点，弱纹理场景容易跟踪丢失；计算开销中等。
- **XRSLAM（RD-VIO）**：**优化型VIO，主打移动端AR**。重点面向动态环境鲁棒性，没有完整的全局回环+多地图Atlas机制（XRSLAM有独立visual localization模块做场景定位，不是ORB式的内置回环）；轻量化，iOS移动端原生AR Demo。

## 2）定位侧重点

ORB-SLAM3：机器人、无人机，大场景长时间建图，**重视全局一致性、回环、多地图合并**。适合静态场景，学术数据集基准首选。
XRSLAM：手机/AR眼镜AR场景，**重点解决人走动的动态场景干扰**；轻量化，优先保证跟踪不丢失，移动端部署友好。

## 3）传感器支持

ORB-SLAM3：单目 / 双目 / RGB-D + IMU；针孔+鱼眼；传感器模式齐全。
XRSLAM：视觉惯性VIO为主。

## 4）开源协议

ORB-SLAM3：原版代码**不是Apache**，商用需要联系作者购买商业授权；fork分支依赖库各自协议。
XRSLAM：Apache 2.0，可以自由商用。

## 5）适合场景选型建议

- 做机器人导航、离线建图、需要回环消除漂移、学术论文基准测试：优先 ORB-SLAM3。
- 手机AR、AR眼镜、动态人流场景、嵌入式轻量实时定位、商用项目：优先 XRSLAM。

# 第九步：这个fork分支编译踩坑点（一步一步避坑）

> 
> 文档里写的脚本看起来简单，但Ubuntu24.04环境下容易踩坑：

1. **vcpkg overlay机制坑**
分支优先使用系统apt包（如ffmpeg）；g2o强制vcpkg版本。如果你手动删除overlay目录，所有依赖全部源码编译，编译时间会非常长。
2. **tl::expected库**
CC0协议，C++17下用来做错误返回；升级C++20之后可以删掉，直接使用std::expected。如果编译报expected找不到，就是vcpkg没拉取这个包。
3. **Ninja构建**
必须安装ninja-build；如果仍然调用make，说明CMake生成构建器失败。
4. **ROS2子模块关系**
本仓库本身**没有任何rclcpp/ROS2相关代码**。很多人误解，以为这个fork带ROS2节点。ROS2节点代码放在独立gitlab仓库，把当前仓库作为git submodule引入。

> 
> 工作结构：`orbslam3_ros2（ROS节点，子模块） --> fork-orbslam3（纯C++ SLAM库）`
5. **Realsense相机**
fork分支移除了内置Realsense Demo。想用D435i/T265，需要自己写独立的采集程序，读取图像+IMU送入ORB-SLAM3 System接口。原版Demo不能直接用。

# 第十步：代码调用接口极简流程（C++ API，fork分支和原版API大体兼容）

```
// 1. 创建配置对象（fork改动点：配置创建和System初始化解耦）
std::unique_ptr<Settings> settings = Settings::LoadFromYaml("camera.yaml");

// 2. 创建SLAM系统，传入词袋路径、配置、传感器模式
std::unique_ptr<System> slam = std::make_unique<System>("ORBvoc.txt", settings, System::STEREO_INERTIAL);

// 3. 循环读取图像+IMU数据送入Track函数
while(has_data)
{
    cv::Mat im_left, im_right;
    std::vector<IMUData> imus;
    double timestamp;
    // Track返回相机位姿Tcw
    Sophus::SE3f pose = slam->TrackStereoInertial(im_left, im_right, imus, timestamp);
}

// 4. 系统关闭，保存轨迹
slam->Shutdown();
slam->SaveTrajectoryEuRoC("trajectory.txt");
```

> 
> fork分支改动：`Settings`配置对象独立出来，可以单独捕获yaml解析错误；原版ORB-SLAM3是在System构造函数内部直接加载yaml，出错直接assert崩溃。

---

# 第十一步：扩展：如果要基于这个fork做二次开发，开发路线（一步一步）

1. 环境准备：Ubuntu24.04，安装apt依赖，安装vcpkg、ninja。
2. 拉取本fork仓库，执行`./install_apt_dependencies.sh`，再`./build.sh`编译库，跑EuRoC数据集验证基础功能正常。
3. 基于C++ API写图像/IMU数据输入模块：
   - 方案A：数据集回放（EuRoC / TUM-VI），调试算法；
   - 方案B：接入相机SDK（Realsense等），自己写采集代码，把图像+IMU喂给`TrackXXX`接口。
4. 相机标定：用Kalibr做双目+IMU联合标定，生成yaml配置文件，替换示例yaml。
5. 评估：导出轨迹，evo计算ATE、RPE，对比真值，评估漂移。
6. 可选：接入ROS2：拉取`orbslam3_ros2`仓库，它会自动拉取本仓库作为子模块，编译ROS2节点，用ROS2话题收发图像IMU。
7. 性能调优：打开`REGISTER_TIMES`宏，输出各模块耗时；调整关键帧策略、ORB特征点数量，平衡精度和帧率。

# 第十二步：局限性总结

1. 算法层面，fork分支没有增强ORB-SLAM3本身，原版所有固有问题依然存在：弱纹理、全黑场景、剧烈快速运动依旧会跟踪丢失。
2. ORB词袋文件体积大，加载慢，嵌入式设备部署需要做词袋压缩。
3. 特征点法，在低纹理墙面、天空这类场景，特征稀少，漂移会增大。
4. 多地图Atlas合并会带来较大的优化计算峰值，会出现瞬时卡顿。
5. 商用授权：底层ORB-SLAM3论文代码版权归萨拉戈萨大学，商用需要联系原作者，**这个fork分支的工程修改不代表你获得商用授权**。

---

# 方向A：ORB-SLAM3源码模块逐段拆解 + IMU预积分细节
> 依然区分：**算法内核是原版ORB-SLAM3，fork分支只改工程层，这部分逻辑完全不变**
## 1. Tracking.cc 主线程逐模块解析
Tracking是系统入口，每一帧图像/IMU数据都会进入这个模块，是整个SLAM的“感知与预测大脑”。
### 1.1 构造函数
```cpp
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSetting\(\mathbf{P}\)ath, const int sensor):
    mSensor(sensor), mpSystem(pSys), mpVocabulary(pVoc), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mpKeyFrameDB(pKFDB)
{
    // 读取yaml配置：相机内参、畸变系数、ORB特征数量、IMU参数、深度阈值等
    // 初始化ORB提取器，左右相机（双目）两个提取器
    // 初始化IMU预积分器（VI模式才启用）
}
```
fork分支改动：原版在Tracking构造里直接读取yaml；fork把配置抽成独立`Settings`类，yaml加载提前做，失败直接返回错误，不会直接assert崩溃。

### 1.2 入口函数 `GrabImageStereo()` / `GrabImageStereoInertial()`
业务流程：
1. 图像去畸变（针孔相机）；鱼眼模型跳过校正。
2. 构造当前帧`Frame`对象，调用ORB提取器提取特征点、计算描述子。
3. VI模式：将当前帧对应的IMU测量值送入预积分器，更新预积分项`IMU\(\mathbf{P}\)reintegrator`。
4. 调用`Track()`核心函数，执行跟踪逻辑。
5. 返回当前帧位姿。

### 1.3 Track() 核心状态机
```
Track()
├─ 状态分支：
│  ├─ 系统未初始化 → 调用MonocularInitialization / StereoInertialInitialization
│  ├─ 跟踪正常 → 位姿预测 + 局部地图匹配 + 运动BA
│  ├─ 跟踪丢失 → 进入重定位Relocalization()
├─ 判断是否生成新关键帧 NeedNewKeyFrame()
│  └─ 满足条件 → 生成KeyFrame，推入LocalMapping队列
└─ 更新帧状态，给可视化模块送数据
```
#### 位姿预测三种策略（优先级从上到下）
1. **IMU预积分预测（VI模式首选）**：用上一帧位姿+IMU预积分得到的速度、bias，预测当前帧位姿。运动快的时候，IMU预测远优于匀速模型。
2. **匀速运动模型**：纯视觉模式，假设相机运动速度不变，用上一帧的运动推算当前位姿。相机急停、转向时容易失效。
3. **重定位成功后**：重定位得到的位姿直接作为初始值，不使用运动模型。

#### 局部地图匹配
不是和上一帧匹配，而是把当前帧特征，和**局部窗口内的关键帧对应的地图点**匹配。这是ORB-SLAM3抗漂移的关键，只做帧间匹配会快速漂移。
> 帧间匹配：短时间稳定，长时间漂移大；局部地图匹配：利用历史三维点约束，抑制漂移。

#### 运动BA（仅优化当前帧位姿）
固定所有地图点，只优化当前相机位姿；VI模式会同时最小化**重投影残差 + IMU预积分残差**。属于轻量优化，保证实时性。

### 1.4 跟踪丢失与重定位 Relocalization()
1. 当前帧提取ORB，生成词袋向量，查询`KeyFrameDatabase`。
2. 在Atlas所有子地图中检索外观相似候选关键帧。
3. 候选帧和当前帧做ORB匹配，RANSAC+\(\mathbf{P}\)n\(\mathbf{P}\)求解相机位姿。
4. 位姿求解成功后，做一次BA优化，恢复跟踪；失败则持续尝试。
5. 长时间重定位失败：Atlas新建子地图，后续建图在新地图。

## 2. IMU预积分模块 IMU\(\mathbf{P}\)reintegrator.h/cc（ORB-SLAM3核心创新）
### 为什么需要IMU预积分？
IMU频率远高于图像（IMU 200Hz~1000Hz，相机20~30Hz）。两帧图像之间会收到几十条IMU数据。
如果每次优化都积分一遍所有IMU，计算量爆炸。
> 预积分：把**两个图像帧之间的所有IMU测量，预先积分成一个相对增量（相对旋转、相对平移、相对速度）+协方差矩阵**。优化时只需要这个增量，不用重复遍历所有IMU样本。

### 预积分公式要点
IMU测量模型：
$$
\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega} + \mathbf{b}_g + \boldsymbol{\eta}_g \\
\tilde{\mathbf{a}} = \mathbf{R}_{wb}(\mathbf{g}+\dot{\mathbf{v}}) + \mathbf{b}_a + \boldsymbol{\eta}_a
$$
$\boldsymbol{b}_g$：陀螺仪bias；$\boldsymbol{b}_a$：加速度计bias。bias会缓慢漂移，是优化变量。

预积分输出：
- $\Delta \mathbf{R}_{ij}$：i帧到j帧的旋转增量
- $\Delta \mathbf{v}_{ij}$：i帧到j帧速度增量
- $\Delta \mathbf{p}_{ij}$：i帧到j帧位置增量
- 协方差矩阵 $\mathbf{\(\mathbf{P}\)}$：记录噪声传播，用于优化权重

> 重点：预积分增量**依赖bias**。当优化中bias更新，预积分结果需要重新传播。ORB-SLAM3采用一阶近似更新预积分，不用全部重积分，节省算力。

### VI初始化（ICRA2020纯惯性优化）
传统VIO初始化：必须缓慢移动，同时看足够多视觉点，才能估计重力、bias、尺度。
ORB-SLAM3创新：**先纯惯性优化**，只使用IMU预积分序列，求解：重力方向、gyro bias、加速度bias、各帧速度。
纯惯性阶段不需要良好的视觉三角化点。纯惯性求解完成后，再联合视觉做全局优化，一次性恢复尺度。
> 代价：纯惯性对IMU噪声敏感；剧烈抖动场景会失败。

## 3. LocalMapping.cc 局部建图线程（异步）
只处理Tracking送来的关键帧，不处理普通图像帧。
```
LocalMapping::Run()
├─ 等待队列拿到新关键帧
├─ 插入地图，关联IMU预积分
├─ 三角化生成新地图点
├─ 剔除坏地图点（观测少、重投影误差大）
├─ 局部BA：优化当前KF + 相邻KF + 关联地图点；VI模式优化bias
└─ 剔除冗余关键帧
```
冗余关键帧判定：90%地图点能被其他关键帧观测到，则删除。目的控制地图规模，防止BA越来越慢。

## 4. LoopClosing.cc 回环与Atlas地图合并线程
```
LoopClosing::Run()
├─ 收到新关键帧，词袋检索候选回环帧
├─ RANSAC几何校验，过滤假回环
├─ 若候选帧属于**同一个子地图**：回环校正，位姿图优化
└─ 若候选帧属于**另一个子地图**：触发Atlas地图合并
```
位姿图优化：只优化关键帧位姿，不优化地图点，快速消除全局漂移。
Atlas多地图：多个独立子地图，各自保存关键帧、地图点。跟踪丢失新建地图；重访旧区域自动合并。

## 5. ORBVocabulary / DBoW2 词袋模块
ORB特征描述子聚类成单词。图像转为词袋向量，快速图像检索。
缺点：词袋文件巨大（ORBvoc.txt几十MB），加载耗时；嵌入式需要二进制压缩版本。

---
# 方向B：完整实操部署流程（Ubuntu24.04，fork分支 + EuRoC数据集 + evo评估）
## 环境准备
```bash
# 基础依赖
sudo apt update
sudo apt install git cmake ninja-build libopencv-dev libeigen3-dev python3 python3-numpy
# vcpkg安装
git clone [https://github.com/microsoft/vcpkg](https://github.com/microsoft/vcpkg)
./vcpkg/bootstrap-vcpkg.sh
```
## 拉取fork分支源码
```bash
git clone 【fork仓库地址】 ORB_SLAM3_fork
cd ORB_SLAM3_fork
# 安装apt依赖
./install_apt_dependencies.sh
# 编译，自动调用Ninja
./build.sh
```
编译产物：`lib/libORB_SLAM3.so`；可执行文件在`Examples/Stereo-Inertial/Release/`。

## EuRoC数据集准备
EuRoC数据集官网下载序列，例如 `V1_02_medium`，解压到 `~/dataset/EuRoC/V1_02_medium`。
修改脚本 `euroc_examples.sh`
```bash
pathDatasetEuroc="~/dataset/EuRoC"
```
运行全部EuRoC序列：
```bash
./euroc_examples.sh
```
运行完成，轨迹文件输出在 `Examples/Stereo-Inertial/`，后缀 `traj.txt`。

## 精度评估 evo
```bash
# 安装evo
pip install evo --upgrade
# 评估ATE绝对轨迹误差
evo_ape euroc groundtruth.txt traj.txt -a -p
# 评估R\(\mathbf{P}\)E相对位姿误差（漂移）
evo_rpe euroc groundtruth.txt traj.txt -a -p
```
- ATE：全局漂移大小；
- R\(\mathbf{P}\)E：每段距离内的局部漂移。

## 相机标定（双目+IMU联合标定，Kalibr）
1. 准备棋盘格标定板，录制3~5段rosbag：平移、旋转、倾斜，充分激励IMU。
2. Kalibr命令：
```bash
kalibr_calibrate_imu_camera --target april_6x6.yaml --cam camchain.yaml --imu imu.yaml --bag record.bag
```
3. 输出标定结果yaml，内参、畸变、相机IMU外参 $T_{ci}$。
4. 将标定参数填入ORB-SLAM3的相机配置yaml。

## ROS2集成流程（orbslam3_ros2）
```bash
# 创建ROS2工作空间
mkdir -p ws_orbslam2/src && cd ws_orbslam2/src
git clone [https://gitlab.com/apl-ocean-engineering/orbslam3_ros2.git](https://gitlab.com/apl-ocean-engineering/orbslam3_ros2.git)
cd orbslam3_ros2
git submodule update --init --recursive
cd ../..
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TY\(\mathbf{P}\)E=Release
source install/setup.bash
```
> 原理：`orbslam3_ros2`拉取fork分支作为子模块；ROS2节点封装图像、IMU话题接收，调用fork分支的C++ A\(\mathbf{P}\)I。

## 实操常见调试手段
1. 开启计时：`include/Config.h` 取消注释 `#define REGISTER_TIMES`，运行输出各模块耗时。
2. 关闭\(\mathbf{P}\)angolin可视化：修改配置文件`Viewer: false`，提升算力资源。
3. 特征点数量调整：yaml文件 `ORBextractor.nFeatures`，减少特征点提升帧率，代价精度下降。
4. 轨迹保存：`SaveTrajectoryEuRoC()` 输出EuRoC格式，适配evo。

---
# 补充：ORB-SLAM3 / XRSLAM / 本fork分支综合选型清单
|项目|算法类型|核心优势|短板|适用场景|商用授权|
|---|---|---|---|---|---|
|ORB-SLAM3原版|稀疏特征点VI-SLAM|全局回环、Atlas多地图、学术基准精度|编译麻烦，裸指针，仅ROS1，弱纹理易丢|机器人、无人机、离线建图|需联系原作者购买商业授权|
|本fork分支|稀疏特征点VI-SLAM（算法不变，工程重构）|C++17、智能指针、vcpkg依赖管理、Ubuntu24.04，可作为ROS2子模块|无算法增强，原版缺陷全部继承|ROS2机器人项目、现代C++工程落地|底层ORB-SLAM3商用授权不变|
|XRSLAM|稠密/半稠密VIO|移动端轻量化，动态人鲁棒，鱼眼友好，Apache2.0|全局回环能力弱，无Atlas多地图|AR眼镜、手机AR、穿戴设备|Apache2.0，可商用|

# 进阶开发可选方向
1. 替换ORB特征为Super\(\mathbf{P}\)oint深度学习特征，提升暗光/弱纹理跟踪；
2. 替换DBoW2词袋，用轻量深度学习全局描述子做回环；
3. 多线程优化LocalMapping，增加G\(\mathbf{P}\)U加速特征提取；
4. 增加地图持久化，支持保存/加载Atlas多地图。

# 方向2：逐行拆解 `Tracking.cc` Track() + IMU预积分代码 + g2o顶点边定义

## 一、Tracking.cc Track() 主函数源码逐段解析（原版ORB-SLAM3逻辑，fork仅工程改造）

```
void Tracking::Track()
{
    // Step1：判断系统状态，分支路由
    if(mState==SYSTEM_NOT_READY)
    {
        return;
    }

    if(mState==NO_IMAGES_YET)
    {
        // 第一帧图像，直接构造初始帧，不做跟踪
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;
    Frame &mCurrentFrame = mpSystem->mCurrentFrame;

    // VI模式：更新IMU预积分，把当前帧与上一帧之间所有IMU测量值累积
    if(mpInertial)
    {
        if(!mpImuPreintegratedFromLastKF)
        {
            // 新建预积分器，参数来自yaml：IMU噪声、重力大小
            mpImuPreintegratedFromLastKF = new IMUPreintegrator(mImuCalib);
        }
        // 把两帧之间IMU数据推入预积分器，递推ΔR Δv Δp和协方差
        for(IMUData imu : mvImuMeas)
        {
            mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(imu.acc, imu.gyr, imu.t);
        }
        mvImuMeas.clear();
    }

    // Step2：系统未初始化分支（单目/VI初始化）
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::MONOCULAR)
        {
            // 单目纯视觉初始化，找两帧足够视差，H/F矩阵分解
            MonocularInitialization();
        }
        else if(mSensor==System::STEREO_INERTIAL || mSensor==System::RGBD_INERTIAL)
        {
            // VI初始化：ORB-SLAM3核心，先纯惯性优化，再视觉惯性联合优化
            if(!mpInertial->Initialize(mCurrentFrame))
            {
                // VI初始化失败，等待下一帧继续收集数据
                return;
            }
            // VI初始化成功，状态切换到OK
            mState=OK;
        }
        if(mState!=OK)
            return;
    }
    else
    {
        // ========== 系统已经初始化完成，正常跟踪逻辑 ==========
        bool bOK;
        // Step3：位姿预测（3种策略）
        if(mState==OK)
        {
            // 优先：VI模式使用IMU预积分预测当前帧Tcw
            if(mpInertial)
            {
                bOK = PredictStateIMU();
            }
            else
            {
                // 纯视觉：匀速运动模型预测位姿
                bOK = PredictState();
            }
            if(!bOK)
            {
                // 预测失败，标记跟踪丢失
                mState=LOST;
            }
        }

        // Step4：跟踪正常 → 局部地图匹配 + 运动仅BA优化
        if(mState==OK)
        {
            // 当前帧特征 和 局部地图点做匹配
            bOK = TrackLocalMap();
            if(bOK)
            {
                // Motion-only BA：固定所有地图点，只优化当前帧位姿；VI加入IMU残差
                Optimizer::PoseOptimization(&mCurrentFrame);
            }

            // 匹配点过少，判定跟踪丢失
            if(mCurrentFrame.mnMatchesInliers<10)
            {
                mState=LOST;
            }
        }

        // Step5：跟踪丢失分支 → 执行重定位 Relocalization()
        if(mState==LOST)
        {
            bOK = Relocalization();
            if(bOK)
            {
                // 重定位成功，恢复跟踪状态
                mState=OK;
            }
            else
            {
                // 长时间无法重定位，Atlas新建子地图，等待重新初始化
                if(mpAtlas->GetCurrentMap()->KeyFramesInMap()>100)
                {
                    mpAtlas->CreateNewMap();
                }
                return;
            }
        }
    }

    // Step6：判断是否生成新关键帧，送入LocalMapping线程队列
    if(NeedNewKeyFrame())
    {
        CreateNewKeyFrame();
    }

    // Step7：更新上一帧信息，供下一帧预测使用
    mLastFrame = Frame(mCurrentFrame);
}
```

### 关键函数简要说明

1. `PredictStateIMU()`：利用上一关键帧位姿、速度、IMU预积分增量，预测当前帧相机位姿、速度。
2. `TrackLocalMap()`：核心匹配函数，搜索局部窗口关键帧对应的地图点，和当前帧ORB特征匹配，建立重投影约束。
3. `Optimizer::PoseOptimization()`：g2o构建图，仅对当前帧位姿做优化，**不改动地图点**，属于轻量优化保证实时。
4. `Relocalization()`：词袋检索候选关键帧，RANSAC PnP求解位姿，重定位恢复跟踪。
5. `NeedNewKeyFrame()`：关键帧判定条件：跟踪质量、时间间隔、视差变化、局部建图线程负载。

## 二、IMUPreintegrator 预积分核心代码拆解

### 头文件核心变量

```
class IMUPreintegrator
{
public:
    // IMU噪声参数，来自标定
    Eigen::Matrix<double,6,6> covNoise;
    // 预积分输出增量：i到j
    Eigen::Matrix3d deltaR; // ΔR_ij
    Eigen::Vector3d deltaV; // Δv_ij
    Eigen::Vector3d deltaP; // Δp_ij
    // 协方差矩阵，误差传播
    Eigen::Matrix<double,15,15> P;
    // bias
    Eigen::Vector3d bg, ba;

    // 构造函数，接收IMU标定参数
    IMUPreintegrator(const IMUCalib& imucalib);
    // 核心：积分一条IMU测量，递推状态与协方差
    void IntegrateNewMeasurement(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, double dt);
    // 当bias被优化更新，一阶近似更新预积分增量，避免重积分所有IMU
    void UpdateDeltaRVP(const Eigen::Vector3d &bg_new, const Eigen::Vector3d &ba_new);
};
```

### IntegrateNewMeasurement 积分递推核心逻辑

```
void IMUPreintegrator::IntegrateNewMeasurement(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, double dt)
{
    // 1. 减去bias，得到真实角速度、加速度
    Eigen::Vector3d w = gyr - bg;
    Eigen::Vector3d a = acc - ba;

    // 2. 旋转增量离散积分（四元数更新）
    Eigen::Quaterniond q(Eigen::AngleAxisd(w.norm()*dt, w.normalized()));
    deltaR = q.toRotationMatrix() * deltaR;

    // 3. 速度、位置递推
    deltaV += deltaR * a * dt;
    deltaP += deltaV * dt + 0.5 * deltaR * a * dt * dt;

    // 4. 误差状态协方差传播，15维误差状态：[δθ,δp,δv,δbg,δba]
    PropagateCovariance(dt, w, a);
}
```

> 
> 重点：预积分增量是**相对量**，不是全局位姿；当g2o优化修改bg/ba时，调用`UpdateDeltaRVP`一阶修正ΔR/Δv/Δp，不需要重新遍历全部IMU数据，大幅节省计算。

## 三、g2o图优化：顶点与边定义（ORB-SLAM3 VI优化）

### 1. 顶点（待优化变量）

1. **VertexPose**：相机位姿 `SE3`，旋转+平移；关键帧/当前帧位姿顶点。
2. **VertexPointXYZ**：3D地图点坐标 `Eigen::Vector3d`。
3. **VertexVelocity**：IMU速度变量。
4. **VertexBias**：IMU bias（陀螺仪+加速度计）。

### 2. 边（残差约束）

1. **EdgeProjectXYZ2UV**：视觉重投影边。残差 = 观测像素坐标 - 投影像素坐标；用于纯视觉BA。
2. **EdgeInertial**：**VI核心边**，IMU预积分残差。残差由三部分组成：旋转残差、速度残差、位置残差，同时关联两个关键帧顶点+bias顶点。
$$
\begin{cases}
r_R = Log(\Delta R_{ij}^\top R_i^\top R_j) \
r_v = R_i^\top(v_j - v_i - g\Delta t) - \Delta v_{ij} \
r_p = R_i^\top(p_j - p_i -v_i\Delta t -0.5g\Delta t^2) - \Delta p_{ij}
\end{cases}
$$
3. **EdgeSE3**：位姿图边，回环优化使用，只约束关键帧之间相对位姿，不包含地图点，用于LoopClosing位姿图优化。

### 两种优化场景区分

1. **Motion-only BA（PoseOptimization）**：顶点只有当前帧相机位姿；边只有视觉重投影边（VI增加IMU边）；地图点全部固定。轻量，Tracking线程实时调用。
2. **Local BA（LocalMapping）**：优化局部窗口关键帧位姿、速度、bias、地图点；视觉+IMU残差联合；计算量大，异步LocalMapping线程。

## 四、源码阅读踩坑点

1. Frame/KeyFrame大量使用**指针**（原版），fork改成智能指针，需要关注所有权。
2. 坐标系：ORB-SLAM3使用**相机坐标系Z向前**；IMU在机体坐标系，外参Tbc把IMU转到相机系。
3. g2o版本：ORB-SLAM3使用老版本g2o，现代fork一般升级g2o，顶点边接口会有少量改动。
4. Atlas多地图：每个子地图`Map`独立管理关键帧、地图点；不同Map的KeyFrame不能直接参与BA，只有地图合并之后才会融合。

## 五、可选的源码断点调试方案

1. 在VSCode配置C++调试，在`Track()`入口、`PredictStateIMU()`、`PoseOptimization()`打断点。
2. 跑EuRoC V1_02_medium，查看每帧：匹配内点数量、IMU预积分增量、优化前后位姿变化。
3. 开启`REGISTER_TIMES`宏，打印各模块耗时，定位性能瓶颈。

---

下一部分可以继续深入：
方案A：`EdgeInertial` 残差+雅可比矩阵完整推导和源码
方案B：LocalMapping线程完整源码逐段解析
方案C：LoopClosing回环检测、位姿图优化、Atlas地图合并源码

# A. EdgeInertial 残差 + 雅可比完整推导与源码

## A.1 残差定义

IMU预积分边连接两个关键帧顶点（i, j），以及 bias 顶点（$b_g, b_a$）。预积分器存储的是从 i 到 j 的**相对增量** $\Delta R_{ij}, \Delta v_{ij}, \Delta p_{ij}$。

残差定义在 IMU 机体坐标系下：

# $$
\mathbf{r}*{I} =
\begin{bmatrix}
\mathbf{r}*{\Delta R} \
\mathbf{r}*{\Delta v} \
\mathbf{r}*{\Delta p}
\end{bmatrix}

\begin{bmatrix}
\log\big((\Delta R_{ij}^\top) \cdot (\mathbf{R}_i^\top (\mathbf{R}_j))\big) \
\mathbf{R}_i^\top (\mathbf{v}*j - \mathbf{v}*i - \mathbf{g}\Delta t*{ij}) - \Delta v*{ij} \
\mathbf{R}_i^\top (\mathbf{p}*j - \mathbf{p}*i - \mathbf{v}*i\Delta t*{ij} - \tfrac{1}{2}\mathbf{g}\Delta t*{ij}^2) - \Delta p*{ij}
\end{bmatrix}
$$

三项含义：

- **旋转残差**：j 帧相对 i 帧的旋转，与预积分旋转增量的偏差（李群对数映射到切空间）
- **速度残差**：两帧间速度变化，扣除重力项后，与预积分速度增量的偏差
- **位置残差**：两帧间位移，扣除重力/初速度项后，与预积分位移增量的偏差

当 bias 被优化更新时，预积分增量需要一阶修正：

$$
\Delta R_{ij}(b_g^{new}) \approx \Delta R_{ij}(b_g^{old}) \cdot \text{Exp}(J_{g}^{R}\delta b_g)
$$

这就是 `IMUPreintegrator::UpdateDeltaRVP()` 的作用。

## A.2 雅可比矩阵

EdgeInertial 是一个 15 维残差（旋转 3 + 速度 3 + 位置 3 + bias 6 不对，实际残差是 9 维），对优化变量的雅可比：

| 优化变量 | 维度 | 雅可比块 |
| --- | --- | --- |
| $T_i$（i帧位姿） | 6 | 影响 $\mathbf{r}_R, \mathbf{r}_v, \mathbf{r}_p$ |
| $T_j$（j帧位姿） | 6 | 影响 $\mathbf{r}_R, \mathbf{r}_v, \mathbf{r}_p$ |
| $v_i$（i帧速度） | 3 | 影响 $\mathbf{r}_v, \mathbf{r}_p$ |
| $v_j$（j帧速度） | 3 | 影响 $\mathbf{r}_v$ |
| $b_g$（陀螺bias） | 3 | 影响 $\Delta R, \Delta v, \Delta p$ 的一阶修正 |
| $b_a$（加表bias） | 3 | 影响 $\Delta v, \Delta p$ 的一阶修正 |

## A.3 g2o 边类源码骨架

```
// g2o 边：IMU 预积分残差
class EdgeInertial : public g2o::BaseBinaryEdge<9, Eigen::Matrix<double,9,1>,
    VertexPoseAndBiases, VertexVelocity>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeInertial(IMUPreintegrator* pInt) : pInt_(pInt) {}

    void computeError() override {
        // 取两个端点顶点：i帧位姿+bias，j帧速度
        auto* vi = static_cast<VertexPoseAndBiases*>(_vertices[0]);
        auto* vj = static_cast<VertexVelocity*>(_vertices[1]);

        Sophus::SE3f TWi = vi->estimate().Twi;
        Eigen::Vector3f vi_v = vi->estimate().v;
        Eigen::Vector3f vj_v = vj->estimate();
        Eigen::Vector3f bg = vi->estimate().bg;
        Eigen::Vector3f ba = vi->estimate().ba;

        // 1. bias 更新后一阶修正预积分增量
        pInt_->UpdateDeltaRVP(big, ba);

        // 2. 计算旋转残差
        Eigen::Matrix3f dR = pInt_->deltaR.cast<float>();
        Eigen::Matrix3f Rij = TWi.so3().matrix().inverse() * Tj.so3().matrix();
        Eigen::Matrix3f errR = dR.transpose() * Rij;
        Eigen::Vector3f rR = Sophus::SO3f::log(errR);

        // 3. 速度残差
        Eigen::Vector3f rv = TWi.so3().matrix().inverse() *
                             (vj_v - vi_v - g*dt) - pInt_->deltaV.cast<float>();

        // 4. 位置残差
        Eigen::Vector3f rp = TWi.so3().matrix().inverse() *
                             (Tj.translation() - Ti.translation() - vi_v*dt - 0.5f*g*dt*dt)
                             - pInt_->deltaP.cast<float>();

        _error << rR, rv, rp;
    }

    void linearizeOplus() override {
        // 对 T_i, T_j, v_i, v_j, b_g, b_a 的雅可比
        // 预计算雅可比 J_R, J_v, J_p 存储在 pInt_ 中
        // 这里填充到 _jacobianOplusXi / _jacobianOplusXj
    }
};
```

> 
> 关键点：
> 
> 
> - `UpdateDeltaRVP()` 在 computeError 开头调用，保证 bias 变化后预积分增量是修正过的
> - 雅可比主要由预积分器内部存储的 **雅可比矩阵 $J_{g}^{R}, J_{g}^{v}, J_{a}^{v}$** 构成，不需要每次重新推导
> - 边的信息矩阵 = 预积分协方差矩阵的逆，自动加权

---

# B. LocalMapping 线程完整源码逐段解析

## B.1 主循环 Run()

```
void LocalMapping::Run()
{
    while (true)
    {
        // 等待新关键帧进入队列
        if (CheckNewKeyFrames())
        {
            // Step1：处理新关键帧
            ProcessNewKeyFrame();

            // Step2：三角化生成新地图点
            if (CheckNewKeyFrames())
            {
                CreateNewMapPoints();
            }

            // Step3：剔除坏地图点
            MapPointCulling();

            // Step4：邻接关键帧搜索补充匹配
            SearchInNeighbors();

            // Step5：局部 BA 优化
            Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap);

            // Step6：剔除冗余关键帧
            KeyFrameCulling();
        }
    }
}
```

## B.2 ProcessNewKeyFrame() — 插入新关键帧

```
void LocalMapping::ProcessNewKeyFrame()
{
    // 从队列取出新关键帧
    KeyFrame* pKF = mlNewKeyFrames.front();
    mlNewKeyFrames.pop_front();

    // 计算词袋向量，用于后续回环检测
    pKF->ComputeBoW();

    // VI模式：关联IMU预积分器
    if (pKF->mpImuPreintegrated)
    {
        // 将上一个KF到当前KF的预积分数据存入KF
        pKF->mpImuPreintegrated->SetNewBias(pKF->GetImuBias());
    }

    // 加入地图
    mpMap->AddKeyFrame(pKF);

    // 为关键帧每个特征点创建/关联地图点
    for (auto& obs : pKF->mMapPointMatches)
    {
        MapPoint* pMP = obs;
        if (pMP)
        {
            pMP->AddObservation(pKF, idx);
            pMP->ComputeDistinctiveDescriptors();
        }
    }

    mpCurrentKeyFrame = pKF;
}
```

## B.3 CreateNewMapPoints() — 三角化

```
void LocalMapping::CreateNewMapPoints()
{
    // 1. 找到当前KF的共视图邻接KF（共享地图点的其他KF）
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    // 2. 对每对 (当前KF, 邻接KF)：
    for (auto pKF2 : vpNeighKFs)
    {
        // 词袋加速匹配两个KF之间未关联的特征
        vector<cv::DMatch> vMatches;
        ORBmatcher::SearchForTriangulation(mpCurrentKeyFrame, pKF2, vMatches);

        // 3. 对每对匹配点三角化
        for (auto& m : vMatches)
        {
            // 用两个KF的投影矩阵三角化得到3D点
            cv::xfeatures2d::SIFT::triangulatePoints(...);

            // 4. 校验：
            // - 视差角足够大（>1度）
            // - 重投影误差小
            // - 深度在合理范围内
            if (checkParallax && checkReprojErr && checkDepth)
            {
                MapPoint* pMP = new MapPoint(pt3D, mpCurrentKeyFrame);
                pMP->AddObservation(mpCurrentKeyFrame, m.queryIdx);
                pMP->AddObservation(pKF2, m.trainIdx);
                mpMap->AddMapPoint(pMP);
            }
        }
    }
}
```

## B.4 MapPointCulling() — 剔除坏点

```
void LocalMapping::MapPointCulling()
{
    // 遍历当前KF创建的新地图点
    for (auto pMP : vpRecentMapPoints)
    {
        int nObs = pMP->Observations(); // 被多少个KF观测到
        float fFoundRatio = pMP->GetFoundRatio(); // 被观测率

        // 剔除条件（满足任一就删）：
        // 1. 最近帧被观测率 < 25%
        // 2. 该点被创建超过2个KF，但观测它的KF数 < 3
        // 3. 平均重投影误差过大
        if (fFoundRatio < 0.25 || (nKFsSinceCreated > 2 && nObs < 3) || meanErr > threshold)
        {
            pMP->SetBadFlag();
        }
    }
}
```

## B.5 SearchInNeighbors() — 邻接补充匹配

```
void LocalMapping::SearchInNeighbors()
{
    // 找共视图上隔一跳的邻接KF
    vector<KeyFrame*> vpNeigh1 = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
    vector<KeyFrame*> vpNeigh2 = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(10);

    // 对这些邻接KF，把当前KF还没关联的地图点做额外匹配
    // 目的：让同一场景的地图点被更多KF观测到，增强BA约束
    for (auto pKF : vpNeigh1)
    {
        ORBmatcher::SearchByProjection(pKF, mpCurrentKeyFrame, 0.9);
    }
}
```

## B.6 LocalBundleAdjustment() — 局部BA

```
void Optimizer::LocalBundleAdjustment(KeyFrame* pCurrKF, bool* pbStopFlag, Map* pMap)
{
    // g2o 优化器初始化
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver =
        new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // 1. 收集局部关键帧：当前KF + 共视图邻接KF
    // 2. 收集局部地图点：这些KF观测到的所有MapPoint
    // 3. 收集外部KF：观测到局部地图点，但不在局部窗口内的KF → 作为固定顶点

    // 添加顶点：
    // - 局部KF位姿（优化）+ 速度 + bias（VI模式）
    // - 局部地图点（优化）
    // - 外部KF位姿（固定，不优化）

    // 添加边：
    // - EdgeProjectXYZ2UV：视觉重投影边，每个MapPoint × 每个观测它的KF
    // - EdgeInertial：IMU预积分边，连接相邻KF（VI模式）

    // 4. 鲁棒核函数（Huber），剔除外点
    // 5. 迭代优化：通常5次Levenberg-Marquardt

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // 6. 检查大误差边，标记为外点剔除，再优化一轮
}
```

## B.7 KeyFrameCulling() — 冗余关键帧剔除

```
void LocalMapping::KeyFrameCulling()
{
    // 遍历当前KF的共视图邻接KF
    vector<KeyFrame*> vpLocalKFs = mpMap->GetAllKeyFrames();

    for (auto pKF : vpLocalKFs)
    {
        int nMPs = 0;
        int nRedundant = 0;

        // 对pKF观测的每个地图点：
        for (auto& obs : pKF->mMapPointMatches)
        {
            nMPs++;
            // 如果同一个MapPoint在其他至少3个KF中也被观测到
            // 且其他KF观测该点的视角不偏差太大
            if (pMP->Observations() >= 3 && goodViewCount >= 3)
            {
                nRedundant++;
            }
        }

        // 冗余率 > 90% → 删除该KF
        if (nRedundant > 0.9 * nMPs)
        {
            pKF->SetBadFlag();
            mpMap->EraseKeyFrame(pKF);
        }
    }
}
```

> 
> 为什么要删冗余KF？BA复杂度和KF数量呈立方关系，保留过多KF会让优化越来越慢。90%阈值是ORB-SLAM3权衡精度与速度的经验值。

---

# C. LoopClosing 回环检测 + 位姿图优化 + Atlas 地图合并

## C.1 主循环 Run()

```
void LoopClosing::Run()
{
    while (true)
    {
        if (CheckNewKeyFrames())
        {
            KeyFrame* pKF = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();

            // Step1：检测回环候选
            if (DetectLoopCandidates(pKF))
            {
                // Step2：计算Sim3相似变换（尺度对齐）
                if (ComputeSim3())
                {
                    // Step3：执行回环校正 + 地图融合
                    CorrectLoop();
                }
            }
        }
    }
}
```

## C.2 DetectLoopCandidates() — 词袋检索

```
bool LoopClosing::DetectLoopCandidates(KeyFrame* pKF)
{
    // 1. 计算当前KF的词袋BoW向量
    BowVector currentBow = pKF->mBowVec;

    // 2. 在KeyFrameDatabase中检索相似KF
    vector<KeyFrame*> vpCandidateKFs =
        mpKeyFrameDB->DetectLoopCandidates(pKF, nCovisibilityConsistencyTh);

    // 3. 几何一致性检查：
    // - 候选KF不能是当前KF的直接邻接KF（排除相邻帧自然相似）
    // - 至少3个连续候选KF形成时间窗口
    // - DBoW相似度分数高于平均分数的0.8倍

    // 4. 跨地图检查：如果候选KF属于另一个子地图 → 触发Atlas合并而非普通回环
    bool bMerge = (pKF->GetMap() != pLoopKF->GetMap());

    return !vpCandidateKFs.empty();
}
```

## C.3 ComputeSim3() — 相似变换求解

```
bool LoopClosing::ComputeSim3()
{
    // 为什么用Sim3而不是SE3？
    // 单目SLAM没有真实尺度，两个KF之间的相对变换可能包含尺度漂移
    // Sim3 = SE3 + 尺度因子s

    // 1. RANSAC 框架
    for (int iteration = 0; iteration < nMaxIterations; iteration++)
    {
        // 2. 随机抽样3对匹配点
        // 3. 用 Horn 算法求解 Sim3 变换 (R, t, s)
        Sophus::Sim3f Scw = Sophus::Sim3f::rotxyz(R, t, s);

        // 4. 投影匹配点，统计内点数量
        int nInliers = 0;
        for (auto& m : vMatches)
        {
            // 将当前KF的MapPoint用Scw变换到回环KF坐标系
            // 投影到回环KF图像上，检查重投影误差
            if (reprojError < threshold)
                nInliers++;
        }

        // 5. 内点数量足够 → 退出RANSAC
        if (nInliers > nMinInliers)
            break;
    }

    // 6. 用所有内点优化Sim3
    Optimizer::OptimizeSim3(pKF1, pKF2, vpMatches, Scw);

    return bSuccess;
}
```

## C.4 CorrectLoop() — 回环校正

```
void LoopClosing::CorrectLoop()
{
    // 1. 停止LocalMapping线程，避免地图被修改
    SetLocalMapperStopped(true);

    // 2. 计算当前地图所有KF相对于回环KF的Sim3变换
    //    构建位姿图：当前地图KF节点 + 回环地图KF节点 + 回环边
    Optimizer::OptimizeEssentialGraph(
        mpMap, pLoopKF, pMergeKF,
        g2o::Sim3Scw, g2o::Sim3ScwInv,
        mg2oLoopEdges, mbFixScale);

    // 3. 地图融合：
    //    - 把当前地图的MapPoint投影到回环地图
    //    - 重叠区域的MapPoint做融合（取均值），避免重复
    Fuse(pLoopKF, pMergeKF, Scw);

    // 4. 全局BA（可选，异步执行）
    if (mbStartGlobalBA)
        mpOptimizer->GlobalBundleAdjustment();

    // 5. 重置词袋数据库，把融合后的KF重新加入
    mpKeyFrameDB->clear();
    for (auto pKF : mpMap->GetAllKeyFrames())
        mpKeyFrameDB->add(pKF);

    // 6. 恢复LocalMapping线程
    SetLocalMapperStopped(false);
}
```

## C.5 Atlas 多地图合并（跨地图回环）

```
// 当 DetectLoopCandidates 发现回环候选KF属于另一个子地图时：
void LoopClosing::MergeMaps(KeyFrame* pLoopKF, KeyFrame* pMergeKF)
{
    // 1. 计算两个地图之间的Sim3变换
    Sophus::Sim3f Sws = ComputeSim3BetweenMaps(pLoopKF, pMergeKF);

    // 2. 把 pMergeMap 的所有KF和MapPoint，用Sim3变换到 pLoopMap 坐标系
    for (auto pKF : pMergeMap->GetAllKeyFrames())
    {
        // 变换KF位姿、速度、bias
        pKF->Transform(Sws);
    }
    for (auto pMP : pMergeMap->GetAllMapPoints())
    {
        // 变换MapPoint坐标
        pMP->Transform(Sws);
    }

    // 3. 地图点融合：
    //    遍历两个地图重叠区域的MapPoint
    //    距离 < 1cm 的点融合为一个，取观测好的点保留
    FuseMaps(pLoopMap, pMergeMap, overlapTolerance);

    // 4. 合并共视图：建立两个地图KF之间的共视关系
    MergeSpanningTree(pLoopMap, pMergeMap);

    // 5. 从Atlas中删除被合并的子地图
    mpAtlas->EraseMap(pMergeMap);

    // 6. 位姿图优化：融合后的新地图做一次全局图优化
    Optimizer::OptimizeEssentialGraph(mergedMap, ...);
}
```

> 
> Atlas 合并的关键区别：
> 
> 
> - **普通回环**：在同一个Map内部，用位姿图优化校正漂移，不合并地图点
> - **Atlas合并**：两个不同Map的KF和MapPoint真正融合成一个Map，后续BA在统一坐标系下进行

## C.6 关键数据结构

```
Atlas
 ├── Map_1
 │    ├── KeyFrames: [KF1, KF2, ...]
 │    ├── MapPoints: [MP1, MP2, ...]
 │    └── KeyFrameDatabase 关联
 ├── Map_2
 │    ├── KeyFrames: [KF101, KF102, ...]
 │    └── ...
 └── 当前活跃Map指针

LoopClosing 检测到 pKF(当前Map) ↔ pLoopKF(另一个Map)
  → 计算Sim3
  → 变换Map_2所有KF/MP到Map_1坐标系
  → 融合重叠MP
  → Atlas.EraseMap(Map_2)
  → 位姿图优化
```

---

# 三条主线的源码阅读路径总结

| 阶段 | 线程 | 核心文件 | 核心函数 |
| --- | --- | --- | --- |
| 实时跟踪 | Tracking | Tracking.cc | Track() → PredictStateIMU → TrackLocalMap → PoseOptimization |
| 局部建图 | LocalMapping | LocalMapping.cc | Run() → ProcessNewKeyFrame → CreateNewMapPoints → LocalBA |
| 回环/合并 | LoopClosing | LoopClosing.cc | Run() → DetectLoopCandidates → ComputeSim3 → CorrectLoop / MergeMaps |

阅读建议：先跑通 EuRoC 数据集，在 `Track()` 入口打断点，观察每帧状态切换；再逐步深入 `Optimizer::PoseOptimization()` 的 g2o 构建过程，最后看回环线程的 Sim3 求解。

