cmake_minimum_required (VERSION 3.1.0)

project(KalmanFilterVehicleLocation)
add_executable(main main.cpp)

# Kalman
add_library(KalmanFilter kalman.cpp)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Link against the libraries
target_link_libraries(main Eigen3::Eigen KalmanFilter)
