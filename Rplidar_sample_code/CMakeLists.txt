cmake_minimum_required (VERSION 3.1.0)  # Should change to newer one

project(Rplidar_simple)

add_executable(Rplidar_simple main.cpp)

# RPLIDAR
set(RPLIDAR_SDK_PATH "/home/nvidia/Documentos/Tesis/Rplidar_sample_code/sdk")
add_library(rplidar STATIC IMPORTED)
set_property(TARGET rplidar
    PROPERTY IMPORTED_LOCATION ${RPLIDAR_SDK_PATH}/output/Linux/Release/librplidar_sdk.a)
target_include_directories(rplidar INTERFACE
    ${RPLIDAR_SDK_PATH}/sdk/include/
    ${RPLIDAR_SDK_PATH}/sdk/src/)

#target_link_libraries(${PROJECT_NAME} rplidar)

# THREADS
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads PUBLIC rplidar)



