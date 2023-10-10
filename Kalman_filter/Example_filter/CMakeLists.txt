cmake_minimum_required (VERSION 3.1.0)

project(KalmanFilterExample)

find_package(Eigen3 REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main Eigen3::Eigen)

# Find RealSense2
#find_package(realsense2 REQUIRED)
#include_directories(${realsense2_INCLUDE_DIRS})

# Find OpenCV
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

#add_executable(rs-imu-realsense main.cpp)
#set_property(TARGET rs-imu-realsense PROPERTY CXX_STANDART 11)

# Link against the libraries
#target_link_libraries(rs-imu-realsense ${OpenCV_LIBS} ${realsense2_LIBRARY}) 

#set_target_properties(rs-imu-realsense PROPERTIES FOLDER "Examples")

#install(TARGETS rs-imu-realsense
#        RUNTIME DESTINATION ./build)
