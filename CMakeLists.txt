cmake_minimum_required(VERSION 3.13)
project(cse481wi19)

set(CMAKE_CXX_STANDARD 14)

include_directories(fetch_ikfast_plugin/include)
include_directories(kuri_gazebo/include)
include_directories(kuri_gazebo/include/kuri_gazebo)
include_directories(madmux)
include_directories(madmux/include)
include_directories(perception/include)
include_directories(perception/include/perception)

add_executable(cse481wi19
        fetch_ikfast_plugin/include/ikfast.h
        fetch_ikfast_plugin/src/fetch_arm_ikfast_moveit_plugin.cpp
        fetch_ikfast_plugin/src/fetch_arm_ikfast_solver.cpp
        kuri_camera/src/ros_publisher.cpp
        kuri_camera/src/save_image.cpp
        kuri_gazebo/include/kuri_gazebo/kuri_gazebo_hardware.h
        kuri_gazebo/src/kuri_hardware_gazebo.cpp
        madmux/include/madmux.h
        perception/include/perception/crop.h
        perception/include/perception/downsample.h
        perception/include/perception/segmentation.h
        perception/src/crop.cpp
        perception/src/downsample.cpp
        perception/src/point_cloud_demo.cpp
        perception/src/save_cloud_main.cpp
        perception/src/segmentation.cpp)
