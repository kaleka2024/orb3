# 已知依赖项列表
## ORB‑SLAM3 v1.0
本文档列举 ORB‑SLAM3 所引入的全部代码片段，以及不属于 ORB‑SLAM3 作者所有的链接库。

### src 和 include 文件夹内的代码
- **ORBextractor.cc**：该文件是 OpenCV 库中 `orb.cpp` 的修改版本，原始代码采用 BSD 许可证。
- **MLPnPsolver.h、MLPnPsolver.cc**：该文件是 Steffen Urban 所实现 MLPnP 算法的修改版本，源码来源于 [opengv](https://github.com/urbste/opengv)。原始代码采用 BSD 许可证。
- ORBmatcher.cc 文件中的 **ORBmatcher::DescriptorDistance** 函数：代码取自 http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel，该代码属于公有领域。

##### Thirdparty 文件夹内的代码
- **DBoW2** 文件夹下全部代码：为 [DBoW2](https://github.com/dorian3d/DBoW2) 与 [DLib](https://github.com/dorian3d/DLib) 库的修改版本，其中所有文件均使用 BSD 许可证。