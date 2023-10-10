cmake_minimum_required (VERSION 3.1.0)

project(KalmanFilterExample)
add_executable(main main.cpp)

# Kalman
add_library(KalmanFilter kalman.cpp)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# RealSense2
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})

# Link against the libraries
target_link_libraries(main Eigen3::Eigen ${realsense2_LIBRARY} KalmanFilter)
