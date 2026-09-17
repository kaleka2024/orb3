cmake_minimum_required(VERSION 3.28)
project(ORB_SLAM3)

# ============================================================================
#  构建配置
# ============================================================================
IF(NOT CONFIG)
    SET(CONFIG Release)
ENDIF()
MESSAGE("Build type: " ${CONFIG})

add_compile_options(
        "$<$<CONFIG:Debug>:-g;-Wall;-Wno-reorder>"
        "$<$<CONFIG:RelWithDebInfo>:-O3;-g;-UNDEBUG>"
        "$<$<CONFIG:Release>:-march=native;-O3>"
)

if(DO_SANITIZE)
    add_compile_options(
            "$<$<CONFIG:Debug>:-fsanitize=address;-fno-omit-frame-pointer;-fsanitize=undefined>"
            "$<$<CONFIG:RelWithDebInfo>:-fsanitize=address;-fno-omit-frame-pointer;-fsanitize=undefined>"
            "$<$<CONFIG:Release>:-fsanitize=address;-fno-omit-frame-pointer;-fsanitize=undefined>"
    )
    add_link_options(
            -fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined
    )
endif()

# ============================================================================
#  C++ 标准
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# ============================================================================
#  全局 spdlog/fmt 宏定义
#  ---------------------------------------------------------------------------
#  Examples 的 target 编译时若没带 -DSPDLOG_FMT_EXTERNAL，
#  会导致 spdlog 头文件尝试 include <spdlog/fmt/bundled/core.h>，
#  而系统安装的 spdlog 没有 bundled 目录，编译失败。
#  这里在顶层统一给所有 target 加宏。
# ============================================================================
add_definitions(-DSPDLOG_FMT_EXTERNAL)
add_definitions(-DSPDLOG_COMPILED_LIB)

# ============================================================================
#  抑制无用警告
#  ---------------------------------------------------------------------------
#  -Wno-deprecated-declarations：抑制 OpenSSL 3.0 的 MD5 弃用警告
# ============================================================================
add_compile_options(-Wno-deprecated-declarations)

# ============================================================================
#  依赖查找
# ============================================================================
find_package(OpenCV 4.4)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 CONFIG REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(g2o CONFIG REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

add_subdirectory(Thirdparty/DBoW2)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ============================================================================
#  ORB_SLAM3 主库
#  ---------------------------------------------------------------------------
#  【方法 1 核心改动】
#    把 SHARED 改成 STATIC，避免链接 libspdlog.a（无 -fPIC）时
#    报错：
#       relocation R_X86_64_TPOFF32 against ... can not be used when
#       making a shared object; recompile with -fPIC
#
#  原因：
#    静态库不要求所有 .o 都带 -fPIC，所以可以直接链接 libspdlog.a。
#    Examples 链接这个 .a 也能正常编译运行。
#
#  影响：
#    生成 libORB_SLAM3.a 而不是 .so。
#    功能完全不受影响，Examples 照常编译运行。
# ============================================================================
add_library(${PROJECT_NAME} STATIC
        src/Atlas.cc
        src/CameraModels/KannalaBrandt8.cpp
        src/CameraModels/Pinhole.cpp
        src/Converter.cc
        src/Frame.cc
        src/FrameDrawer.cc
        src/G2OTypes/Edges.cc
        src/G2OTypes/Vertices.cc
        src/GeometricTools.cc
        src/ImuTypes.cc
        src/KeyFrame.cc
        src/KeyFrameDatabase.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/Map.cc
        src/MapDrawer.cc
        src/MapPoint.cc
        src/MLPnPsolver.cpp
        src/OptimizableTypes/edge_se3_project_xyz_only_pose.cpp
        src/OptimizableTypes/edge_se3_project_xyz_only_pose_to_body.cpp
        src/OptimizableTypes/edge_se3_project_xyz.cpp
        src/OptimizableTypes/edge_se3_project_xyz_to_body.cpp
        src/OptimizableTypes/edge_sim3_project_xyz.cpp
        src/OptimizableTypes/vertex_sim3_expmap.cpp
        src/Optimizer.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/Settings.cc
        src/SettingsLoader.cc
        src/Sim3Solver.cc
        src/System.cc
        src/System/Atlas.cc
        src/System/Exporters.cc
        src/System/Factory.cc
        src/Tracking.cc
        src/TwoViewReconstruction.cc
        src/Utils/Checksum.cc
        src/Viewer.cc
)

# ============================================================================
#  spdlog + fmt 配置
#  ---------------------------------------------------------------------------
#  因为主库现在是 STATIC，直接链接静态库 libspdlog.a / libfmt.a 即可，
#  不再需要 -fPIC，链接 .a 完全没问题。
#
#  这里仍然保留“显式指定共享库优先”的逻辑，如果系统里有 .so 就用 .so，
#  没有就用 .a。对 STATIC 库来说两者都能链接成功。
# ============================================================================
set(spdlog_DIR "/usr/local/lib/cmake/spdlog" CACHE PATH "spdlog cmake dir" FORCE)
set(fmt_DIR    "/usr/local/lib/cmake/fmt"    CACHE PATH "fmt cmake dir"    FORCE)

find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# 给所有链接 spdlog::spdlog 的目标也加上宏（双保险）
target_compile_definitions(spdlog::spdlog INTERFACE
        SPDLOG_FMT_EXTERNAL
        SPDLOG_COMPILED_LIB
)

# ---- 优先用共享库，没有就用 ::target（静态库也能链接 STATIC 主库）----
set(SPDLOG_SHARED_LIB "/usr/local/lib/libspdlog.so")
set(FMT_SHARED_LIB    "/usr/local/lib/libfmt.so")

if (EXISTS ${SPDLOG_SHARED_LIB} AND EXISTS ${FMT_SHARED_LIB})
    message(STATUS "=================================================")
    message(STATUS "Using SHARED spdlog/fmt:")
    message(STATUS "  SPDLOG: ${SPDLOG_SHARED_LIB}")
    message(STATUS "  FMT:    ${FMT_SHARED_LIB}")
    message(STATUS "=================================================")

    target_link_libraries(${PROJECT_NAME} PRIVATE
            ${SPDLOG_SHARED_LIB}
            ${FMT_SHARED_LIB}
    )
else()
    message(STATUS "Using spdlog::spdlog / fmt::fmt target (may be static)")
    target_link_libraries(${PROJECT_NAME} PRIVATE
            spdlog::spdlog
            fmt::fmt
    )
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC REGISTER_TIMES)

# ============================================================================
#  主库依赖
# ============================================================================
target_link_libraries(${PROJECT_NAME}
        PUBLIC
        opencv_core
        opencv_imgproc
        ${Pangolin_LIBRARIES}
        DBoW2
        Boost::boost
        Boost::serialization
        PRIVATE
        OpenSSL::Crypto
        Eigen3::Eigen
        Sophus::Sophus
        g2o::core
        g2o::types_sim3
        g2o::types_slam3d
)

# ============================================================================
#  Examples
#  ---------------------------------------------------------------------------
#  顶层 add_definitions(-DSPDLOG_FMT_EXTERNAL) 已让所有 example 自动继承宏。
#  ${PROJECT_NAME} 现在是 STATIC 库，example 链接后即可拿到 spdlog/fmt。
# ============================================================================

# ===============================================
# EuRoC examples
# ===============================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/EuRoC)

add_executable(stereo_euroc
        Examples/EuRoC/stereo_euroc.cc
        Examples/EuRoC/euroc_common.cc)
target_link_libraries(stereo_euroc PRIVATE ${PROJECT_NAME})

# Not ported yet...
# add_executable(stereo_inertial_euroc
#         Examples/EuRoC/stereo_inertial_euroc.cc
#         Examples/EuRoC/euroc_common.cc)
# target_link_libraries(stereo_inertial_euroc PRIVATE ${PROJECT_NAME})

add_executable(mono_euroc
        Examples/EuRoC/mono_euroc.cc
        Examples/EuRoC/euroc_common.cc)
target_link_libraries(mono_euroc PRIVATE ${PROJECT_NAME})

add_executable(mono_inertial_euroc
        Examples/EuRoC/mono_inertial_euroc.cc
        Examples/EuRoC/euroc_common.cc)
target_link_libraries(mono_inertial_euroc PRIVATE ${PROJECT_NAME})

# ===============================================
# KITTI examples
# ===============================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Kitti)

add_executable(stereo_kitti
        Examples/Kitti/stereo_kitti.cc)
target_link_libraries(stereo_kitti PRIVATE ${PROJECT_NAME})

add_executable(mono_kitti
        Examples/Kitti/mono_kitti.cc)
target_link_libraries(mono_kitti PRIVATE ${PROJECT_NAME})

# ===============================================
# TUM examples
# ===============================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/TUM)

add_executable(rgbd_tum
        Examples/TUM/rgbd_tum.cc)
target_link_libraries(rgbd_tum PRIVATE ${PROJECT_NAME})

add_executable(mono_tum
        Examples/TUM/mono_tum.cc)
target_link_libraries(mono_tum PRIVATE ${PROJECT_NAME})

add_executable(stereo_tum_vi
        Examples/TUM/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi PRIVATE ${PROJECT_NAME})

add_executable(mono_tum_vi
        Examples/TUM/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi PRIVATE ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi
        Examples/TUM/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi PRIVATE ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi
        Examples/TUM/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi PRIVATE ${PROJECT_NAME})