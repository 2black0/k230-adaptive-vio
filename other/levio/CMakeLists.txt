# k230-levio — build for host (x86) and cross (K230 RISC-V Linux)
#
# Phase 1 develops on the host: full debugger, fast iteration, no cross-compile
# variables in play. Phase 3 step 3 switches to the cross toolchain by pointing
# CMAKE_C_COMPILER at riscv64-unknown-linux-gnu-gcc (see build_k230.sh).
#
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(k230_vio C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------
option(LEVIO_PARALLEL  "Use OpenMP for pi_cl_team_fork"       OFF)
option(LEVIO_RVV       "Enable RISC-V vector flags (K230)"    OFF)
option(LEVIO_WERROR    "Treat warnings as errors"             OFF)
option(LEVIO_STRICT_FP "Disable FMA contraction for cross-architecture reproducibility" OFF)

# Which dataset/resolution the binary is built for. Image size, patch grid, map
# sizes and camera/IMU calibration are compile-time constants in LEVIO, so each
# combination gets its own binary rather than a runtime switch.
#   euroc         : cropped 160x120 baseline
#   euroc_half    : full-frame 376x240 (half of 752x480)
#   euroc_quarter : full-frame 188x120 (quarter of 752x480)
#   tumvi         : 256x256 (half of 512x512; legacy name retained)
#   tumvi_quarter : 128x128 (quarter of 512x512)
#   viode         : VIODE, 160x120 / 320x240 / 640x480 via -DVIODE_320 / -DVIODE_640
set(LEVIO_DATASET_PROFILE "euroc" CACHE STRING
    "Dataset profile: euroc | euroc_half | euroc_quarter | tumvi | tumvi_quarter | viode")

# Overrides for config.h. Empty means "keep the upstream LEVIO default".
set(LEVIO_NB_CORES  "" CACHE STRING "Override NB_CORES (GAP9 default 8, K230 has 2)")
set(LEVIO_LOG_LEVEL "" CACHE STRING "Override LOG_LEVEL (0=ERROR 1=WARN 2=INFO 3=DEBUG)")

if(LEVIO_NB_CORES)
    add_compile_definitions(NB_CORES=${LEVIO_NB_CORES})
endif()
if(NOT LEVIO_LOG_LEVEL STREQUAL "")
    add_compile_definitions(LOG_LEVEL=${LEVIO_LOG_LEVEL})
endif()

# ---------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------
set(LEVIO_SRCS
    src/levio/definitions/prng.c
    src/levio/definitions/trace.c
    src/levio/definitions/sorting.c
    src/levio/definitions/work_memory.c
    src/levio/definitions/matrix.c
    src/levio/definitions/vio_definitions.c
    src/levio/definitions/transformation.c
    src/levio/definitions/pose_graph.c
    src/levio/definitions/matrix3.c
    src/levio/feature_handling/orb.c
    src/levio/feature_handling/orb_gap.c
    src/levio/feature_handling/bf_matcher.c
    src/levio/visual_odometry/vo.c
    src/levio/visual_odometry/triangulation.c
    src/levio/visual_odometry/essential_matrix.c
    src/levio/visual_odometry/epnp.c
    src/levio/optimizer/imu_optimization.c
    src/levio/optimizer/imu_bias_optimization.c
    src/levio/optimizer/vi_alignment.c
    src/levio/optimizer/imu_calibration.c
    src/levio/optimizer/pose_graph_optimizer.c
    src/levio/optimizer/imu_preintegration.c
)

set(HARNESS_SRCS
    src/compat/pmsis.c
    src/harness/pgm_io.c
    src/harness/source_pgm.c
    src/harness/source_paced.c
    src/harness/traj_writer.c
    src/harness/metrics.c
    src/harness/main.c
)

add_executable(levio_replay ${LEVIO_SRCS} ${HARNESS_SRCS})

# ---------------------------------------------------------------------
# Include paths
#
# src/compat comes first so our pmsis.h shadows any SDK header of the same
# name. The LEVIO sources use both "definitions/foo.h" and bare "foo.h", so
# every subdirectory is on the path.
# ---------------------------------------------------------------------
target_include_directories(levio_replay PRIVATE
    src/compat
    src/levio
    src/levio/definitions
    src/levio/feature_handling
    src/levio/visual_odometry
    src/levio/optimizer
    src/harness
)

target_link_libraries(levio_replay PRIVATE m)

find_package(Threads REQUIRED)
target_link_libraries(levio_replay PRIVATE Threads::Threads)

# ---------------------------------------------------------------------
# Warnings
#
# LEVIO is third-party code that predates our warning settings; we keep the
# warnings visible but do not fail the build on them by default.
# ---------------------------------------------------------------------
target_compile_options(levio_replay PRIVATE
    -Wall -Wextra
    -Wno-unused-parameter
    -Wno-sign-compare
)
if(LEVIO_WERROR)
    target_compile_options(levio_replay PRIVATE -Werror)
endif()

# ---------------------------------------------------------------------
# Dataset profile -> compile definitions
# ---------------------------------------------------------------------
if(LEVIO_DATASET_PROFILE STREQUAL "tumvi")
    target_compile_definitions(levio_replay PRIVATE DATASET_PROFILE_TUMVI=1)
    message(STATUS "k230-levio: dataset profile TUM-VI (256x256)")
elseif(LEVIO_DATASET_PROFILE STREQUAL "tumvi_quarter")
    target_compile_definitions(levio_replay PRIVATE DATASET_PROFILE_TUMVI_QUARTER=1)
    message(STATUS "k230-levio: dataset profile TUM-VI quarter (128x128)")
elseif(LEVIO_DATASET_PROFILE STREQUAL "viode")
    target_compile_definitions(levio_replay PRIVATE DATASET_PROFILE_VIODE=1)
    message(STATUS "k230-levio: dataset profile VIODE")
elseif(LEVIO_DATASET_PROFILE STREQUAL "euroc_half")
    target_compile_definitions(levio_replay PRIVATE DATASET_PROFILE_EUROC_HALF=1)
    message(STATUS "k230-levio: dataset profile EuRoC half (376x240)")
elseif(LEVIO_DATASET_PROFILE STREQUAL "euroc_quarter")
    target_compile_definitions(levio_replay PRIVATE DATASET_PROFILE_EUROC_QUARTER=1)
    message(STATUS "k230-levio: dataset profile EuRoC quarter (188x120)")
elseif(LEVIO_DATASET_PROFILE STREQUAL "euroc")
    message(STATUS "k230-levio: dataset profile EuRoC (160x120)")
else()
    message(FATAL_ERROR
        "unknown LEVIO_DATASET_PROFILE '${LEVIO_DATASET_PROFILE}' "
        "(euroc | euroc_half | euroc_quarter | tumvi | tumvi_quarter | viode)")
endif()

# ---------------------------------------------------------------------
# Parallelism
# ---------------------------------------------------------------------
# LEVIO_PARALLEL controls how pi_cl_team_fork dispatches. SINGLE_CORE selects
# which LEVIO code path is taken in the first place, so the two must agree:
# a multi-core code path with a serial dispatcher still produces correct
# results (the shim iterates the core indices), but not the other way round.
if(LEVIO_PARALLEL)
    find_package(OpenMP REQUIRED)
    target_link_libraries(levio_replay PRIVATE OpenMP::OpenMP_C)
    target_compile_definitions(levio_replay PRIVATE LEVIO_PARALLEL=1 SINGLE_CORE=0)
    message(STATUS "k230-levio: OpenMP parallel path ENABLED (SINGLE_CORE=0)")
else()
    target_compile_definitions(levio_replay PRIVATE LEVIO_PARALLEL=0)
    message(STATUS "k230-levio: serial reference path (LEVIO_PARALLEL=OFF)")
endif()

# ---------------------------------------------------------------------
# Floating-point reproducibility
#
# RISC-V GCC contracts a*b+c into a fused multiply-add; x86 without -mfma does
# not. The 1-ULP difference that follows is amplified chaotically by RANSAC,
# whose inlier decisions are discrete thresholds on continuous values: on the
# synthetic sequence the trajectories diverged by 0.94 m with contraction on,
# and by 5.5e-9 m with it off (docs/14).
#
# Leave this OFF for production builds -- FMA is faster and usually more
# accurate. Turn it ON when comparing results across architectures or across
# optimisation levels.
# ---------------------------------------------------------------------
if(LEVIO_STRICT_FP)
    target_compile_options(levio_replay PRIVATE -ffp-contract=off)
    message(STATUS "k230-levio: strict FP (FMA contraction disabled)")
endif()

# ---------------------------------------------------------------------
# RISC-V vector extension (K230 only)
# ---------------------------------------------------------------------
if(LEVIO_RVV)
    target_compile_options(levio_replay PRIVATE
        -mcpu=c908v -mabi=lp64d -mtune=c908
        -mrvv-v0p10-compatible -mrvv-auto-vectorize
    )
    message(STATUS "k230-levio: RVV flags ENABLED (C908V)")
endif()

install(TARGETS levio_replay DESTINATION /root/app/k230-levio)
