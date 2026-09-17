#!/bin/bash

# 彻底清掉所有可能污染编译的环境变量
unset CONDA_SHLVL CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PYTHON_EXE
unset CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH CXX_INCLUDE_PATH
unset LIBRARY_PATH LD_LIBRARY_PATH
unset CMAKE_PREFIX_PATH CMAKE_INCLUDE_PATH CMAKE_LIBRARY_PATH
unset PKG_CONFIG_PATH

# 删除 CMake user package registry 中残留的旧 spdlog/fmt 注册表
# 这是之前 /data/xr/xrslam 项目构建时留下的，会让 find_package 找到旧版 spdlog
rm -rf ~/.cmake/packages/spdlog
rm -rf ~/.cmake/packages/fmt

# 打印确认
echo "==== ENV CHECK ===="
echo "CONDA_PREFIX: $CONDA_PREFIX"
echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
echo "CPATH: $CPATH"
echo "==================="

BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}
BUILD_DIR="build-${BUILD_TYPE}"

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# 注意：显式禁用 conda 和 /data/xr/xrslam 的 find 路径，并强制指定 /usr/local 下的新版 spdlog/fmt
cmake -G Ninja \
      -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
      -DCMAKE_CXX_STANDARD=17 \
      -DCMAKE_IGNORE_PATH="/data/miniconda3;/data/miniconda3/include;/data/miniconda3/lib;/data/miniconda3/lib/cmake;/data/xr/xrslam/build/_deps" \
      -DCMAKE_IGNORE_PREFIX_PATH="/data/miniconda3;/data/xr/xrslam/build/_deps" \
      -Dspdlog_DIR=/usr/local/lib/cmake/spdlog \
      -Dfmt_DIR=/usr/local/lib/cmake/fmt \
      -B "${BUILD_DIR}" \
      -S .

cmake --build "${BUILD_DIR}" -v