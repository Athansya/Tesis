cmake_minimum_required(VERSION 3.20)
project(imgui_simple)

set(SOURCES
    main.cpp
    )

# OpenGL, it links with GL
find_package(OpenGL REQUIRED)

# GLFW
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

# Set compiler and linker flags using GLFW pkg-config variables
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GLFW_CFLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_CX_FLAGS} ${GLFW_CFLAGS}")


# IMGUI
set(IMGUI_LIB "./imgui_api")
include_directories(${IMGUI_LIB})

# Add IMGUI source files
file(GLOB IMGUI_SOURCES "${IMGUI_LIB}/*.cpp")
file(GLOB IMGUI_HEADERS "${IMGUI_LIB}/*.h")

# Add executable and link it
add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCES} ${IMGUI_HEADERS})

# Set the libraries to link with
target_link_libraries(${PROJECT_NAME} PRIVATE ${GLFW_STATIC_LIBRARIES} OpenGL)
