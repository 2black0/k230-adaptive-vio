cmake_minimum_required(VERSION 3.12)
project(eqvio VERSION 1.1)
set(CMAKE_CXX_STANDARD 20)

include(CheckCXXCompilerFlag)


# Options and settings
option( EQVIO_BUILD_TESTS "Build Tests" ON)
option( EQVIO_BUILD_VISUALISATION "Build Visualisation Tool" ON)
option( EQVIO_BUILD_ROSBAG "Build EqVIO with rosbag reading enabled" OFF)
option( EQVIO_SINGLE_THREADED_DATASERVER "Use SimpleDataServer instead of ThreadedDataServer (single-core targets)" OFF)
option( EQVIO_SUPPORT_CONCEPTS "Build EqVIO using c++20 concepts" ON)
option( EXTRA_WARNINGS "Enable extra warnings from gcc" ON)

if(EQVIO_SUPPORT_CONCEPTS)
    message("Adding the flag -fconcepts")
    add_compile_options("-fconcepts")
else()
    message("C++20 concepts disabled (-DEQVIO_SUPPORT_CONCEPTS=0)")
endif()

add_compile_definitions(EQVIO_BUILD_VISUALISATION=$<BOOL:${EQVIO_BUILD_VISUALISATION}>)
add_compile_definitions(EQVIO_BUILD_ROSBAG=$<BOOL:${EQVIO_BUILD_ROSBAG}>)
add_compile_definitions(EQVIO_SINGLE_THREADED_DATASERVER=$<BOOL:${EQVIO_SINGLE_THREADED_DATASERVER}>)
add_compile_definitions(EQVIO_SUPPORT_CONCEPTS=$<BOOL:${EQVIO_SUPPORT_CONCEPTS}>)

option( EQVIO_USE_MARCH_NATIVE "Use the flag -march=native" OFF)
if (EQVIO_USE_MARCH_NATIVE AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
    if(COMPILER_SUPPORTS_MARCH_NATIVE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
    else()
        message("-march=native was requested but is not supported.")
    endif()
endif()

if(NOT EQVIO_USE_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

option( EQVIO_USE_LTO "Use the flag -flto" OFF)
if (EQVIO_USE_LTO AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-flto" COMPILER_SUPPORTS_LTO)
    if(COMPILER_SUPPORTS_LTO)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto")
    else()
        message("-flto was requested but is not supported.")
    endif()
endif()

option( EQVIO_USE_THREAD_SANTIZER "Use the gcc thread sanitizer option" OFF)
if (EQVIO_USE_THREAD_SANTIZER)
    CHECK_CXX_COMPILER_FLAG("-fsanitize=thread" COMPILER_SUPPORTS_THREAD_SANITIZER)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
    if(NOT COMPILER_SUPPORTS_THREAD_SANITIZER)
        message("-fsanitize=thread may not be supported. Trying anyway.")
    endif()
endif()

option( EQVIO_USE_UB_SANTIZER "Use the gcc undefined behaviour sanitizer option" OFF)
if (EQVIO_USE_UB_SANTIZER)
    CHECK_CXX_COMPILER_FLAG("-fsanitize=undefined" COMPILER_SUPPORTS_UB_SANITIZER)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
    if(NOT COMPILER_SUPPORTS_UB_SANITIZER)
        message("-fsanitize=undefined may not be supported. Trying anyway.")
    endif()
endif()

option( EQVIO_USE_MEM_SANTIZER "Use the gcc address sanitizer option" OFF)
if (EQVIO_USE_MEM_SANTIZER)
    CHECK_CXX_COMPILER_FLAG("-fsanitize=address" COMPILER_SUPPORTS_MEM_SANITIZER)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
    if(NOT COMPILER_SUPPORTS_MEM_SANITIZER)
        message("-fsanitize=address may not be supported. Trying anyway.")
    endif()
endif()


# Import external dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()


# Add GIFT
find_package(GIFT QUIET)
if (${GIFT_FOUND})
    message("GIFT is being used as an installed package.")
    set(GIFT_INCLUDE_DIRS "GIFT::GIFT")
else()
    message("GIFT is being used as a git submodule.")
    if (NOT EXISTS "${PROJECT_SOURCE_DIR}/external/GIFT/CMakeLists.txt")
        message(FATAL_ERROR "GIFT was not found as a package and could not be included as a submodule.")
    endif()
    add_subdirectory(external/GIFT)
    set(GIFT_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/external/GIFT/GIFT/include")
    # So the tracker can report its own stage timings through the application's
    # global LoopTimer. LoopTimer.h is header-only, so this adds no link edge.
    target_include_directories(GIFT PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
endif()

# Add argparse
find_package(argparse QUIET)
if (${argparse_FOUND})
    message("argparse is being used as an installed package.")
    set(argparse_INCLUDE_DIRS "argparse::argparse")
else()
    message("argparse is being used as a git submodule.")
    if (NOT EXISTS "${PROJECT_SOURCE_DIR}/external/argparse/include")
        message(FATAL_ERROR "argparse was not found as a package and could not be included as a submodule.")
    endif()
    set(argparse_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/external/argparse/include")
endif()

# Add LiePP
message("LiePP is being used as a git submodule.")
if (NOT EXISTS "${PROJECT_SOURCE_DIR}/external/LiePP/include")
    message(FATAL_ERROR "LiePP could not be included as a submodule.")
endif()
set(LiePP_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/external/LiePP/include")

# Add libs
add_subdirectory(libs)

# Add the include files
set(EQVIO_HEADER_FILES
    include/eqvio/common/safeConfig.h
    include/eqvio/common/LieYaml.h
    include/eqvio/common/aofstream.h

    include/eqvio/mathematical/EqFMatrices.h
    include/eqvio/mathematical/Geometry.h
    include/eqvio/mathematical/IMUVelocity.h
    include/eqvio/mathematical/VIO_eqf.h
    include/eqvio/mathematical/VIOGroup.h
    include/eqvio/mathematical/VIOState.h
    include/eqvio/mathematical/VisionMeasurement.h

    include/eqvio/csv/CSVLine.h
    include/eqvio/csv/CSVReader.h
    include/eqvio/csv/CSVFile.h

    include/eqvio/dataserver/dataservers.h
    include/eqvio/dataserver/DatasetReaderBase.h
    include/eqvio/dataserver/ASLDatasetReader.h
    include/eqvio/dataserver/APDatasetReader.h
    include/eqvio/dataserver/UZHFPVDatasetReader.h
    include/eqvio/dataserver/DataServerBase.h
    include/eqvio/dataserver/SimpleDataServer.h
    include/eqvio/dataserver/ThreadedDataServer.h
    include/eqvio/dataserver/SimulationDataServer.h

    include/eqvio/VIOFilter.h
    include/eqvio/VIOFilterSettings.h
    include/eqvio/VIOSimulator.h
    include/eqvio/LoopTimer.h
    include/eqvio/VIOWriter.h
    )
set(EQVIO_SOURCE_FILES
    src/mathematical/VIOState.cpp
    src/mathematical/VIOGroup.cpp
    src/mathematical/VIO_eqf.cpp
    src/mathematical/VisionMeasurement.cpp
    src/mathematical/IMUVelocity.cpp
    src/mathematical/EqFMatrices.cpp
    src/mathematical/Geometry.cpp
    src/mathematical/coordinateSuite/euclid.cpp
    src/mathematical/coordinateSuite/invdepth.cpp
    src/mathematical/coordinateSuite/normal.cpp

    src/dataserver/ASLDatasetReader.cpp
    src/dataserver/APDatasetReader.cpp
    src/dataserver/UZHFPVDatasetReader.cpp
    src/dataserver/DataServerBase.cpp
    src/dataserver/SimpleDataServer.cpp
    src/dataserver/ThreadedDataServer.cpp
    src/dataserver/SimulationDataServer.cpp

    src/VIOFilter.cpp
    src/VIOSimulator.cpp
    src/LoopTimer.cpp
    src/VIOWriter.cpp
)

# Try to add rosbag
if (EQVIO_BUILD_ROSBAG)
    find_package(rosbag REQUIRED)
    find_package(cv_bridge REQUIRED)
    set(CMAKE_CXX_STANDARD 17)

    list(APPEND EQVIO_HEADER_FILES
        include/eqvio/dataserver/RosbagDatasetReader.h
        include/eqvio/dataserver/HiltiDatasetReader.h
    )
    list(APPEND EQVIO_SOURCE_FILES
        src/dataserver/RosbagDatasetReader.cpp
        src/dataserver/HiltiDatasetReader.cpp
    )
    message("Building with rosbag support.")
endif()

# Add visualisation if requested
if (EQVIO_BUILD_VISUALISATION)
    message("Building with visualisation support.")
else()
    message("Building without visualisation support (stubs only).")
endif()

# VIOVisualiser is always compiled (contains stubs when EQVIO_BUILD_VISUALISATION=0)
list(APPEND EQVIO_HEADER_FILES
    include/eqvio/VIOVisualiser.h
)
list(APPEND EQVIO_SOURCE_FILES
    src/VIOVisualiser.cpp
)

# Add the EqVIO library

add_library(eqvio_lib
    ${EQVIO_SOURCE_FILES}
    ${EQVIO_HEADER_FILES}
)

target_include_directories(eqvio_lib
    PUBLIC include
    PUBLIC ${EQVIO_MODULE_INCLUDE_DIRS}
    PUBLIC ${OPENCV_INCLUDE_DIRS}
    PUBLIC ${GIFT_INCLUDE_DIRS}
    PUBLIC ${LiePP_INCLUDE_DIRS}
    PUBLIC ${EIGEN_INCLUDE_DIRS}
)

if (EQVIO_BUILD_ROSBAG)
    target_include_directories(eqvio_lib
        PUBLIC ${rosbag_INCLUDE_DIRS}
        PUBLIC ${cv_bridge_INCLUDE_DIRS}
    )
endif()

target_link_libraries(eqvio_lib
    ${EQVIO_MODULE_LIBS}
    ${OpenCV_LIBS}
    GIFT
)

if (EQVIO_BUILD_ROSBAG)
    target_link_libraries(eqvio_lib
        ${rosbag_LIBRARIES}
        ${cv_bridge_LIBRARIES}
    )
endif()

if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    message("GCC version is less than 9. Manually linking std::filesystem.")
    target_link_libraries(eqvio_lib stdc++fs)
endif()

if (${EXTRA_WARNINGS})
    message("Extra warnings are enabled.")
    target_compile_options(eqvio_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Add the tests

if(EQVIO_BUILD_TESTS)
    enable_testing()
    message("Building the tests.")
    add_subdirectory(test)
endif()

# Add the main executable
add_executable(eqvio_opt src/main_opt.cpp)
add_executable(eqvio_sim src/main_sim.cpp)
set(EXECUTABLES_LIST
    eqvio_opt
    eqvio_sim
)

foreach(EXECUTABLE_NAME ${EXECUTABLES_LIST})
    target_include_directories(${EXECUTABLE_NAME}
        PRIVATE include
        PRIVATE ${EIGEN_INCLUDE_DIRS}
        PRIVATE ${OpenCV_INCLUDE_DIRS}
        PRIVATE ${YAML_CPP_INCLUDE_DIR}
        PRIVATE ${LiePP_INCLUDE_DIRS}
        PRIVATE ${GIFT_INCLUDE_DIRS}
        PRIVATE ${argparse_INCLUDE_DIRS}
    )
    target_link_libraries(${EXECUTABLE_NAME}
        eqvio_lib
        ${EQVIO_MODULE_LIBS}
        ${OpenCV_LIBS}
        yaml-cpp::yaml-cpp
        GIFT
    )
endforeach()


# Add documentation
option( EQVIO_BUILD_DOCS "Use doxygen to build documentation" OFF)
if (EQVIO_BUILD_DOCS)
    find_package(Doxygen REQUIRED)
    message("Documentation will be built using doxygen.")
    set(DOXYGEN_USE_MATHJAX true)
    set(DOXYGEN_MATHJAX_EXTENSIONS TeX/AMSmath TeX/AMSsymbols)
    doxygen_add_docs(eqvio_docs
        ${EQVIO_HEADER_FILES}
        ${EQVIO_SOURCE_FILES}
        README.md
        docs/main.md
        ALL
    )
endif()
