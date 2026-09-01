cmake_minimum_required(VERSION 3.28)
project(ORB_SLAM3)

IF(NOT CONFIG)
  SET(CONFIG Release)
ENDIF()

MESSAGE("Build type: " ${CONFIG})

add_compile_options(
        "$<$<CONFIG:Debug>:-g;-Wall;-Wno-reorder>"
        "$<$<CONFIG:RelWithDebInfo>:-O3;-g;-UNDEBUG>"
        "$<$<CONFIG:Release>:-march=native;-O3>"
)

add_link_options(
        -fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined
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

#add_compile_definitions(EIGEN_DONT_VECTORIZE EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

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
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
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

add_library(${PROJECT_NAME} SHARED
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
target_compile_definitions(${PROJECT_NAME} PUBLIC REGISTER_TIMES)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        opencv_core
        opencv_imgproc
        ${Pangolin_LIBRARIES}
        DBoW2
        fmt::fmt
        spdlog::spdlog
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

# Build examples

# ===============================================
# EuRoC examples
# ===============================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/EuRoC)

add_executable(stereo_euroc
        Examples/EuRoC/stereo_euroc.cc
        Examples/EuRoC/euroc_common.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

# Not ported yet...
# add_executable(stereo_inertial_euroc
#         Examples/EuRoC/stereo_inertial_euroc.cc
#         Examples/EuRoC/euroc_common.cc)
# target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

add_executable(mono_euroc
        Examples/EuRoC/mono_euroc.cc
        Examples/EuRoC/euroc_common.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_inertial_euroc
        Examples/EuRoC/mono_inertial_euroc.cc
        Examples/EuRoC/euroc_common.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

# ===============================================
# KITTI examples
# ===============================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Kitti)

add_executable(stereo_kitti
        Examples/Kitti/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(mono_kitti
        Examples/Kitti/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

# ===============================================
# TUM examples
# ===============================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/TUM)

add_executable(rgbd_tum
        Examples/TUM/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})


add_executable(mono_tum
        Examples/TUM/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(stereo_tum_vi
        Examples/TUM/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

add_executable(mono_tum_vi
        Examples/TUM/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi
        Examples/TUM/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi
        Examples/TUM/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})
