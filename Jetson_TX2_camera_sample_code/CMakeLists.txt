cmake_minimum_required (VERSION 3.10)

project(opencv_1)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

#add_executable(opencv_1 opencv_1.cpp)
add_executable(camera_calibration camera_calibration.cpp)

#target_link_libraries(opencv_1 ${OpenCV_LIBS})
target_link_libraries(camera_calibration ${OpenCV_LIBS})
