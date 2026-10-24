cmake_minimum_required(VERSION 3.10)
project(open_vins)


# 自动生成 compile_commands.json，供 clangd 等语言服务器使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable optimizations
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
if(APPLE)
    # Homebrew Boost 1.90+ is header-only for most components; no separate component libs
    find_package(Boost REQUIRED)
else()
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
endif()
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

# Common definitions
add_definitions(-DROS_AVAILABLE=0)
add_definitions(-DENABLE_ARUCO_TAGS=0)

# Platform-specific IPC definitions
if(APPLE)
    # macOS: use socket-based IPC (POSIX mqueue not available)
    add_definitions(-DUSE_SOCKET_IPC=1)
    add_definitions(-DUSE_MQUEUE=0)
else()
    # Linux: use POSIX mqueue
    add_definitions(-DUSE_SOCKET_IPC=0)
    add_definitions(-DUSE_MQUEUE=1)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/core/ov_core/src
    ${CMAKE_CURRENT_SOURCE_DIR}/core/ov_init/src
    ${CMAKE_CURRENT_SOURCE_DIR}/core/ov_msckf/src
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# Apple Clang needs explicit cassert (GCC includes it transitively)
if(APPLE)
    add_compile_options(-include cassert)
endif()

# Subdirectories for libraries
add_subdirectory(core/ov_core)
add_subdirectory(core/ov_init)
add_subdirectory(core/ov_msckf)

# Platform-specific system libraries
if(APPLE)
    # macOS: no librt, link boost_filesystem explicitly
    set(PLATFORM_LIBS
        /opt/homebrew/lib/libboost_filesystem.dylib
    )
else()
    # Linux: link rt for mqueue/clock support
    set(PLATFORM_LIBS rt)
endif()

# --- VINS Main Process ---
add_executable(vins apps/vins/main.cpp)
target_link_libraries(vins
    ov_msckf_lib
    ov_init_lib
    ov_core_lib
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp::yaml-cpp
    pthread
    ${PLATFORM_LIBS}
)

# --- IMU Process ---
add_executable(imu_process apps/imu/main.cpp)
target_link_libraries(imu_process
    ${OpenCV_LIBRARIES}
    pthread
    ${PLATFORM_LIBS}
)

# --- Camera Process ---
add_executable(camera_process apps/camera/main.cpp)
target_link_libraries(camera_process
    ${OpenCV_LIBRARIES}
    pthread
    ${PLATFORM_LIBS}
)
