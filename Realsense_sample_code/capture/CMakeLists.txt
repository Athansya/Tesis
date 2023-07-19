cmake_minimum_required (VERSION 3.1.0)

project(RealsenseExamplesCaptureRealSense)

# Find RealSense2
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(rs-capture-realsense rs-capture-realsense.cpp)
set_property(TARGET rs-capture-realsense PROPERTY CXX_STANDART 11)

# Link against the libraries
target_link_libraries(rs-capture-realsense ${OpenCV_LIBS} ${realsense2_LIBRARY}) 

set_target_properties(rs-capture-realsense PROPERTIES FOLDER "Examples")

install(TARGETS rs-capture-realsense
        RUNTIME DESTINATION ./build)
