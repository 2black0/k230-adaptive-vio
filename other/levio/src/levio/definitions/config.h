// Copyright (c) 2026 ETH Zurich. All rights reserved.
// SPDX-FileCopyrightText: 2026 ETH Zurich
// SPDX-License-Identifier: MIT
// Author: Jonas Kühne

#ifndef __CONFIG__H__
#define __CONFIG__H__

/* ===================================================================
 * LEVIO VIO Configuration
 *
 * Central configuration file for all tunable parameters.
 * Modify these values to adapt the system to different cameras,
 * IMU sensors, or operating conditions.
 * =================================================================== */

/* -------------------------------------------------------------------
 * Camera Intrinsics
 * ------------------------------------------------------------------- */
/* Dataset profiles. The camera geometry is compile-time because the original
 * LEVIO code stores the calibration in static matrices. The EuRoC baseline is
 * intentionally kept byte-compatible with the historical cropped 160x120
 * conversion; the new half/quarter profiles are full-frame reductions. */
#if defined(DATASET_PROFILE_VIODE) && defined(VIODE_640)
/* VIODE at 640x480 (no subsample, 56 px cropped each side). */
#define CAMERA_FX  376.00f
#define CAMERA_FY  376.00f
#define CAMERA_CX  320.00f
#define CAMERA_CY  240.00f
#elif defined(DATASET_PROFILE_VIODE) && defined(VIODE_320)
/* VIODE at 320x240 (subsample 2). The city scene sits much further away than
 * EuRoC's machine hall, so inter-frame parallax at 160x120 is only 0.24-0.60 px
 * for depths of 20-50 m -- below what the essential matrix can resolve.
 * Halving the subsample doubles it. */
#define CAMERA_FX  188.00f
#define CAMERA_FY  188.00f
#define CAMERA_CX  160.00f
#define CAMERA_CY  119.50f
#elif defined(DATASET_PROFILE_VIODE)
/* VIODE at 160x120. Ideal pinhole, no distortion (cam0_pinhole.yaml).
 *   fx = 376/4 = 94.00      cx = (376-57)/4 = 79.75
 *   fy = 376/4 = 94.00      cy = (240-1)/4  = 59.75 */
#define CAMERA_FX   94.00f
#define CAMERA_FY   94.00f
#define CAMERA_CX   79.75f
#define CAMERA_CY   59.75f
#elif defined(DATASET_PROFILE_TUMVI_QUARTER)
/* TUM-VI quarter resolution: 512x512 -> 128x128. */
#define CAMERA_FX   47.74461928782179f
#define CAMERA_FY   47.74332676303065f
#define CAMERA_CX   63.73292651483869f
#define CAMERA_CY   64.22436072491260f
#elif defined(DATASET_PROFILE_TUMVI)
/* TUM-VI room sequences. dso/camchain.yaml gives [fu, fv, cu, cv] at 512x512;
 * these are those values scaled by 0.5 to the 256x256 working resolution. The
 * converter removes the (equidistant) distortion, so they are pinhole. */
#define CAMERA_FX   95.48923857564358f
#define CAMERA_FY   95.4866535260613f
#define CAMERA_CX   127.46585302967738f
#define CAMERA_CY   128.4487214498252f
#elif defined(DATASET_PROFILE_EUROC_HALF)
/* EuRoC full-frame half resolution: 752x480 -> 376x240. */
#define CAMERA_FX  229.327000f
#define CAMERA_FY  228.648000f
#define CAMERA_CX  183.607500f
#define CAMERA_CY  124.187500f
#elif defined(DATASET_PROFILE_EUROC_QUARTER)
/* EuRoC full-frame quarter resolution: 752x480 -> 188x120. */
#define CAMERA_FX  114.663500f
#define CAMERA_FY  114.324000f
#define CAMERA_CX   91.803750f
#define CAMERA_CY   62.093750f
#else
/* EuRoC MAV cam0, after cv2.undistort + [1::4, 57:-57:4]. */
#define CAMERA_FX  114.66f
#define CAMERA_FY  114.32f
#define CAMERA_CX   77.8f
#define CAMERA_CY   62.09f
#endif

/* -------------------------------------------------------------------
 * Image Dimensions
 * ------------------------------------------------------------------- */
#if defined(VIODE_640)
#define IMG_WIDTH  640
#define IMG_HEIGHT 480
#elif defined(VIODE_320)
#define IMG_WIDTH  320
#define IMG_HEIGHT 240
#elif defined(DATASET_PROFILE_TUMVI_QUARTER)
#define IMG_WIDTH  128
#define IMG_HEIGHT 128
#elif defined(DATASET_PROFILE_TUMVI)
/* TUM-VI at 256x256: the source is 512x512, halved once. No crop -- the whole
 * field of view is kept, so the intrinsics stay at the centre of the frame. */
#define IMG_WIDTH  256
#define IMG_HEIGHT 256
#elif defined(DATASET_PROFILE_EUROC_HALF)
#define IMG_WIDTH  376
#define IMG_HEIGHT 240
#elif defined(DATASET_PROFILE_EUROC_QUARTER)
#define IMG_WIDTH  188
#define IMG_HEIGHT 120
#else
#define IMG_WIDTH  160
#define IMG_HEIGHT 120
#endif
#define IMG_SIZE   (IMG_WIDTH * IMG_HEIGHT)

/* -------------------------------------------------------------------
 * Camera / Frame Rate
 * ------------------------------------------------------------------- */
#define CAMERA_FPS 20.0f

/* -------------------------------------------------------------------
 * IMU Configuration
 * ------------------------------------------------------------------- */
#define IMU_SAMPLING_PERIOD  0.005f  /* 200 Hz IMU rate (seconds) */

/* IMU-to-camera extrinsic rotation (row-major 3x3), i.e. transpose of the
 * camera-to-body rotation the datasets publish. */
#ifdef DATASET_PROFILE_VIODE
/* calibration.yaml body_T_cam0 is a pure permutation, transposed here. */
#define IMU_TO_CAM_R00  0.0f
#define IMU_TO_CAM_R01  1.0f
#define IMU_TO_CAM_R02  0.0f
#define IMU_TO_CAM_R10  0.0f
#define IMU_TO_CAM_R11  0.0f
#define IMU_TO_CAM_R12  1.0f
#define IMU_TO_CAM_R20  1.0f
#define IMU_TO_CAM_R21  0.0f
#define IMU_TO_CAM_R22  0.0f
#elif defined(DATASET_PROFILE_TUMVI_QUARTER) || defined(DATASET_PROFILE_TUMVI)
/* dso/camchain.yaml cam0 T_cam_imu, transposed: LEVIO's imu_to_cam_tf maps
 * IMU-frame vectors into the camera frame (optimizer/imu_preintegration.c:103),
 * while T_cam_imu is the camera-to-IMU transform. Same relation as EuRoC. */
#define IMU_TO_CAM_R00 -0.9995250378696743f
#define IMU_TO_CAM_R01  0.0075019185074052044f
#define IMU_TO_CAM_R02 -0.02989013031643309f
#define IMU_TO_CAM_R10  0.029615343885863205f
#define IMU_TO_CAM_R11 -0.03439736061393144f
#define IMU_TO_CAM_R12 -0.998969345370175f
#define IMU_TO_CAM_R20 -0.008522328211654736f
#define IMU_TO_CAM_R21 -0.9993800792498829f
#define IMU_TO_CAM_R22  0.03415885127385616f
#else
#define IMU_TO_CAM_R00  0.01486554f
#define IMU_TO_CAM_R01  0.99955725f
#define IMU_TO_CAM_R02 -0.02577444f
#define IMU_TO_CAM_R10 -0.99988093f
#define IMU_TO_CAM_R11  0.01496721f
#define IMU_TO_CAM_R12  0.00375619f
#define IMU_TO_CAM_R20  0.0041403f
#define IMU_TO_CAM_R21  0.02571553f
#define IMU_TO_CAM_R22  0.99966073f
#endif

/* -------------------------------------------------------------------
 * Processing Configuration
 * ------------------------------------------------------------------- */
/* K230-VIO: made overridable from the build system so the core count can be
 * matched to the target (GAP9 has 8 cluster cores, K230 has 2) and the serial
 * reference path can be selected without editing this file. Defaults are
 * unchanged from upstream LEVIO. */
#ifndef NB_CORES
#define NB_CORES    8
#endif
#ifndef SINGLE_CORE
#define SINGLE_CORE 1
#endif

/* L1 scratch-pad memory budget (bytes) */
/* Scratch arena. 110 KB is the GAP9 L1 budget; on K230 there is 1 GiB of DDR,
 * so the only reason to keep it small is fidelity to upstream. At 320x240 a
 * single image is 76.8 KB and the arena no longer fits the working set -- the
 * failure is a segfault before the first frame, with no log output. */
#if defined(VIODE_640)
#define L1_WORK_MEMORY_SIZE 3520000
#elif defined(VIODE_320)
#define L1_WORK_MEMORY_SIZE 880000
#elif defined(DATASET_PROFILE_TUMVI_QUARTER)
/* 128x128 has 138 bins at PATCH_SIZE 8, i.e. 552 candidate features. */
#define L1_WORK_MEMORY_SIZE 220000
#elif defined(DATASET_PROFILE_TUMVI)
/* Both VIODE profiles work out at ~11.5 bytes per pixel of the arena, and that
 * ratio holds at 256x256 too: 65536 px -> ~750 KB. Rounded up for headroom.
 * Undersizing here fails as a segfault before the first frame, with no log. */
#define L1_WORK_MEMORY_SIZE 800000
#elif defined(DATASET_PROFILE_EUROC_HALF)
/* Full-frame half EuRoC needs a larger arena and a coarser grid. */
#define L1_WORK_MEMORY_SIZE 2000000
#elif defined(DATASET_PROFILE_EUROC_QUARTER)
#define L1_WORK_MEMORY_SIZE 180000
#else
#define L1_WORK_MEMORY_SIZE 110000
#endif

/* Cluster and fabric controller frequency (Hz) */
#define CLUSTER_FREQUENCY_HZ (370 * 1000 * 1000)

/* -------------------------------------------------------------------
 * ORB Feature Detection
 * ------------------------------------------------------------------- */
#ifndef FAST_THRESHOLD
#define FAST_THRESHOLD     20
#endif
#ifndef HARRIS_THRESHOLD
#define HARRIS_THRESHOLD 5000
#endif
/* Patch grid. The detector emits FEATURES_PER_PATCH per patch, so the patch
 * count times FEATURES_PER_PATCH must stay under MAX_KEYPOINTS -- at 320x240
 * with PATCH_SIZE 8 that would be 40*30*4 = 4800 against a budget of 704, and
 * the overflow is a segfault, not a graceful cap. Doubling the patch size with
 * the resolution keeps the grid, the budget and the matching cost identical. */
#if defined(VIODE_640)
#define PATCH_SIZE         32
#elif defined(VIODE_320)
#define PATCH_SIZE         16
#elif defined(DATASET_PROFILE_TUMVI_QUARTER)
#define PATCH_SIZE          8
#elif defined(DATASET_PROFILE_TUMVI)
/* bins = (256-32)^2/16^2 = 196, i.e. 784 features at FEATURES_PER_PATCH 4 --
 * within 12 % of EuRoC's 176 bins / 704 features, so the matching cost stays in
 * the same class. At PATCH_SIZE 8 the grid would be 784 bins / 3136 features,
 * several times the MAX_KEYPOINTS budget. */
#define PATCH_SIZE         16
#elif defined(DATASET_PROFILE_EUROC_HALF)
#define PATCH_SIZE         16
#elif defined(DATASET_PROFILE_EUROC_QUARTER)
#define PATCH_SIZE          8
#else
#define PATCH_SIZE          8
#endif
#define FEATURES_PER_PATCH  4

/* -------------------------------------------------------------------
 * Feature Matching
 * ------------------------------------------------------------------- */
#ifndef HAMMING_THRESHOLD
#if defined(DATASET_PROFILE_VIODE)
/* VIODE moves 5.8x faster than EuRoC (2.56 vs 0.44 m/s, 0.128 vs 0.022 m per
 * frame), so BRIEF descriptors change far more between frames and the EuRoC
 * value of 35 starves the matcher. But loosening it too far buys quantity at
 * the cost of correctness -- measured 8-point inlier ratio against threshold:
 *
 *   35 -> 71 matches, ratio 1.07, Sim(3) 15.99
 *   45 -> 144,        ratio 0.83, Sim(3) 11.89
 *   50 -> 190,        ratio 0.75, Sim(3) 10.48   <- best
 *   64 -> 326,        ratio 0.57, Sim(3) 14.48
 *
 * EuRoC's ratio is 0.99 for comparison; VIODE's front-end is simply weaker. */
#define HAMMING_THRESHOLD    50
#else
#define HAMMING_THRESHOLD    35
#endif
#endif
#ifndef MAX_FLOW
#define MAX_FLOW            200
#endif
#define DISABLE_MAX_FLOW      0

/* -------------------------------------------------------------------
 * Pose Graph Dimensions
 * ------------------------------------------------------------------- */
/* At 320x240 the frontend returns roughly 3x the matches, so the map and the
 * observation buffer have to grow with it. Overflow here shows up as a segfault
 * inside EPnP a few hundred frames in, not as a bounds error. */
#if defined(VIODE_640) || defined(VIODE_320)
#define MAX_KEYPOINTS      1408
#define MAX_WORLD_FEATURES 3000
#define MAX_KEYFRAMES         8
#define MAX_OBSERVATIONS   (600 * 12)
#elif defined(DATASET_PROFILE_TUMVI_QUARTER)
#define MAX_KEYPOINTS       704
#define MAX_WORLD_FEATURES 1000
#define MAX_KEYFRAMES         8
#define MAX_OBSERVATIONS   (200 * 12)
#elif defined(DATASET_PROFILE_TUMVI)
/* The detector emits bins*FEATURES_PER_PATCH = 196*4 = 784 features per frame,
 * so MAX_KEYPOINTS only needs headroom above that. The map and observation
 * buffers grow with the feature count (784 vs EuRoC's 704). */
#define MAX_KEYPOINTS      1024
#define MAX_WORLD_FEATURES 1500
#define MAX_KEYFRAMES         8
#define MAX_OBSERVATIONS   (300 * 12)
#elif defined(DATASET_PROFILE_EUROC_HALF)
#define MAX_KEYPOINTS      1408
#define MAX_WORLD_FEATURES 2000
#define MAX_KEYFRAMES         8
#define MAX_OBSERVATIONS   (400 * 12)
#elif defined(DATASET_PROFILE_EUROC_QUARTER)
#define MAX_KEYPOINTS      1024
#define MAX_WORLD_FEATURES 1500
#define MAX_KEYFRAMES         8
#define MAX_OBSERVATIONS   (300 * 12)
#else
#define MAX_KEYPOINTS       704
#define MAX_WORLD_FEATURES 1000
#define MAX_KEYFRAMES         8
#define MAX_OBSERVATIONS   (200 * 12)
#endif

/* -------------------------------------------------------------------
 * RANSAC Iterations
 * ------------------------------------------------------------------- */
#define EPNP_ITERS       64
#define ESSENTIAL_ITERS 640

/* Adaptive RANSAC stopping for the 8-point essential-matrix solver.
 *
 * ESSENTIAL_ITERS is an unconditional iteration count upstream, and on the
 * K230 that made the 8-point stage the single largest cost in the pipeline:
 * 30.17 ms whenever it runs, 35.1 % of all per-frame time (docs/17 4.4).
 *
 * Measured on MH01 over 981 invocations, the inlier ratio reaching this stage
 * has median 0.991 and p10 0.892 -- the two-way matcher plus the Hamming
 * threshold have already removed nearly every outlier. The standard stopping
 * rule N = log(1-p) / log(1-w^s) then asks for a median of 2 iterations, never
 * more than 640. Roughly 99 % of the fixed budget is wasted work.
 *
 * ESSENTIAL_ITERS stays as the hard cap for the low-inlier case.
 * Set RANSAC_ADAPTIVE to 0 to restore the upstream fixed-count behaviour. */
#ifndef RANSAC_ADAPTIVE
#define RANSAC_ADAPTIVE 1
#endif
#ifndef RANSAC_CONFIDENCE
#define RANSAC_CONFIDENCE 0.99f  /* probability of drawing one outlier-free sample */
#endif
#ifndef RANSAC_MIN_ITERS
#define RANSAC_MIN_ITERS  16     /* floor, so a lucky early sample cannot stop us */
#endif

/* -------------------------------------------------------------------
 * Visual Odometry Thresholds
 * ------------------------------------------------------------------- */
#ifndef MIN_MATCHES_EPNP
#if defined(DATASET_PROFILE_VIODE)
/* EPnP disabled on VIODE. Measured frame by frame, its translation direction is
 * 78 deg off ground truth over 96 % of frames -- indistinguishable from random,
 * because the map it solves against is triangulated from poses that are
 * themselves wrong. Forcing every frame through the essential matrix plus IMU
 * scale is worse in theory and better in practice here. */
#define MIN_MATCHES_EPNP       9999
#else
#define MIN_MATCHES_EPNP         25
#endif
#endif
#define MIN_MATCHES_ESSENTIAL    10    /* 8 for algorithm + 2 margin */

/* Minimum RANSAC support before an essential matrix may be used.
 *
 * Upstream checks only that there are enough *matches* to run the algorithm,
 * never that the winning model is actually supported by them. On EuRoC V2_03
 * frame 1 the 8-point RANSAC returned a model with 1 inlier out of 141
 * matches; decomposing it gave a rotation whose middle row was all zeros.
 * normalize() leaves a zero column alone (matrix.c:164) so Gram-Schmidt
 * propagated the rank deficiency silently, and the resulting garbage pose
 * seeded the world map. The run then survived 985 more frames before
 * exploding to 6.6e40 m.
 *
 * Worse, whether it exploded at all depended on the *compiler*: two builds of
 * identical source differing only by one printf gave max|p| of 6.6e40 m and
 * 1.5 m respectively, because normalising a zero vector is 0/0. It was not
 * uninitialised memory (arena poisoning and MALLOC_PERTURB_ are both clean,
 * ASan is clean) -- it was an unchecked degenerate solve. See docs/20.
 *
 * Swept over inliers x ratio on MH01/MH04/MH05/V103/V202/V203 (see
 * results/euroc-full/sweep-inliers). The absolute floor turned out to be a
 * bad idea: it monotonically *worsened* mean SE(3) ATE (3.728 -> 3.774 at 8,
 * -> 3.862 at 12) because it also discards weak-but-usable frames, and once
 * it bites it dominates the ratio entirely. The ratio alone, anywhere in
 * 0.10-0.20, fixed V2_03 at exactly zero cost on the other five sequences.
 * 0.15 is the centre of that flat optimum rather than either edge of it.
 *
 * The floor is therefore off by default and kept only as an ablation knob.
 * Safety does not rest on it: ROTATION_VALIDITY_TOL below is the hard net. */
#ifndef MIN_ESSENTIAL_INLIERS
#define MIN_ESSENTIAL_INLIERS     0
#endif
#ifndef MIN_ESSENTIAL_INLIER_RATIO
#define MIN_ESSENTIAL_INLIER_RATIO 0.15f
#endif

/* Tolerance for "is this actually a rotation" on the 8-point result.
 *
 * Not a tuning parameter: SO(3) membership is not a matter of degree. It is
 * set loose enough to absorb float32 round-off through an SVD and two 3x3
 * products, and tight enough that a zero column (error 1.0) cannot pass. */
#ifndef ROTATION_VALIDITY_TOL
#define ROTATION_VALIDITY_TOL   1e-2f
#endif
/* Ablation switch only. Turning this off is what reproduces the V2_03
 * divergence; leave it on. */
#ifndef ROTATION_VALIDITY_CHECK
#define ROTATION_VALIDITY_CHECK 1
#endif
#define MAX_WORLD_POINT_AGE      15
#define REPROJECTION_ERROR_SQ   2.0f   /* squared pixel threshold */
#ifndef KEYFRAME_PARALLAX_THRESHOLD
#define KEYFRAME_PARALLAX_THRESHOLD 0.05f
#endif
#define SAMPSON_PIXEL_THRESHOLD 0.7f   /* pixel error for essential matrix inlier test */
#define EPNP_INLIER_THRESHOLD_SQ 2.0f /* sqrtf(2.0) pixels, squared = 2.0 */
#define MIN_TRANSLATION_SCALE   0.01f  /* minimum scale for relative translation */

/* Zero-velocity handling (K230-VIO addition).
 *
 * The 8-point essential matrix recovers translation *direction* only; the
 * magnitude comes from the IMU as |prev_velocity| * dt. When the platform is
 * stationary that magnitude goes to zero, but upstream clamps it with
 * Max(scale, MIN_TRANSLATION_SCALE) -- so every frame is pushed 1 cm along a
 * direction that is pure noise, because a zero-baseline essential matrix is
 * geometrically degenerate.
 *
 * On MH01 that costs a lot. The sequence contains a ~30 s stationary period;
 * at 20 Hz that is ~600 frames x 0.01 m of random walk. Measured on the board,
 * ATE over the moving-only first 400 frames is 0.153 m versus 1.001 m over the
 * 1578-frame window that includes the standstill (docs/17 4.2).
 *
 *   0 = upstream behaviour, always clamp
 *   1 = drop the clamp, let the scale go to zero
 *   2 = clamp only when parallax AND scale both look stationary  (never fires)
 *   3 = zero the translation on low parallax alone
 *
 * Modes 1 and 2 were rejected by measurement. The reason mode 2 never fired is
 * the interesting part: it gated on scale < MIN_TRANSLATION_SCALE, but during
 * MH01's standstill the scale is 1.288 m per frame, not near zero. The velocity
 * estimate freezes -- no parallax means no keyframe, no keyframe means the
 * optimizer never runs, so prev_velocity is never updated, while dt grows until
 * Min(dt, 1.0) clamps it. 400 stationary frames x 1.288 m is ~515 m of invented
 * motion, which is the 469 m docs/12 section 4 measured.
 *
 * Mode 3 therefore gates on parallax alone -- the one signal that cannot lie
 * about a static scene.
 */
/* Default 3. Modes 1 and 2 were measured across MH01-MH05 and rejected; mode 3
 * combined with SCALE_DT_MODE=1 cuts metric ATE from 8.67 m to 5.19 m mean.
 * See docs/18 for the full experiment record. */
#ifndef ZUPT_MODE
#define ZUPT_MODE 3
#endif
#ifndef ZUPT_PARALLAX_THRESHOLD
#define ZUPT_PARALLAX_THRESHOLD 0.02f
#endif

/* Which dt multiplies |prev_velocity| to turn the unit-length essential-matrix
 * translation into metres.
 *
 * Upstream uses Min(preintegrator->delta_t, 1.0). delta_t accumulates since the
 * last preintegration reset, not since the previous frame, so the product is
 * dimensionally inconsistent with a frame-to-frame translation. Measured on
 * MH01 the resulting scale has median 0.822 m per frame while ground truth
 * moves 0.022 m per frame -- 37x too large. A constant factor is invisible to
 * evo's Sim(3) alignment, which is why this survived the EuRoC gate.
 *
 *   0 = upstream
 *   1 = camera frame period, 1/CAMERA_FPS
 *   2 = drop prev_velocity entirely and take the speed straight from the IMU:
 *       v = |delta_p - 0.5*g*delta_t^2| / delta_t.
 *
 * Mode 2 exists because on VIODE the optimiser's velocity state never leaves
 * zero (0.003 m/s measured against a ground truth of 2.56 m/s), which collapses
 * the 8-point scale, which collapses the triangulated map, which collapses the
 * EPnP poses that are built on it. The preintegrated displacement does not
 * depend on the optimiser, and integrating it independently reproduces the
 * VIODE ground truth to 2.2 m over 157.7 m -- so the IMU itself is sound.
 * delta_p carries gravity (LEVIO removes it later, in the optimiser), hence the
 * explicit 0.5*g*dt^2 term here.
 */
#ifndef SCALE_DT_MODE
#define SCALE_DT_MODE 1
#endif

/* Motion gate (K230-VIO addition).
 *
 * Per-frame camera displacement is bounded by what the IMU says is physically
 * possible. Measured on MH01 the visual estimate violates that badly: the EPnP
 * path produces a median step of 0.066 m against a ground-truth median of
 * 0.022 m, with p99 at 1.58 m and a maximum of 4.92 m -- 98 m/s over a 50 ms
 * frame. Those spikes are bad pose solutions, and they dominate the trajectory
 * length (EPnP contributes 248 m of the 391 m total; ground truth is 80.6 m).
 *
 * The bound is derived from the IMU, not from a dataset-specific constant:
 * v_imu = |delta_p| / delta_t is the preintegrated average speed, and a frame
 * cannot cover more than MOTION_GATE_K times that in one camera period. When
 * the visual step exceeds it, the camera centre is pulled back along the same
 * direction; rotation is left untouched, since the gate has no evidence about
 * it. */
#ifndef MOTION_GATE_ENABLED
#define MOTION_GATE_ENABLED 1
#endif
#ifndef MOTION_GATE_K
#define MOTION_GATE_K 0.3f     /* how far past the IMU estimate we still trust vision */
#endif
#ifndef MOTION_GATE_MARGIN
#define MOTION_GATE_MARGIN 0.02f  /* metres; keeps the gate open when v_imu ~ 0 */
#endif
/* K and MARGIN were swept on MH01-MH05 (docs/18 section 4.3). Re-tuned after
 * the gravity correction below made v_imu larger and the gate correspondingly
 * looser: k=0.3 now gives mean Sim(3) 4.375 against the upstream baseline's
 * 4.65. Tuned on the same sequences it is reported on -- VIODE is the held-out
 * set, and there the gate is better left off entirely (docs/19). */
#define BEHIND_CAMERA_PENALTY   1e3f   /* penalty weight for points with negative depth */

/* -------------------------------------------------------------------
 * Optimizer Parameters
 * ------------------------------------------------------------------- */
#define UPDATE_WEIGHT  1.0f
#define HUBER_DELTA    1.0f
#define LM_DAMPING     1e-4f  /* Levenberg-Marquardt damping factor */

/* -------------------------------------------------------------------
 * IMU Noise Parameters
 * ------------------------------------------------------------------- */
#ifndef GYRO_NOISE_DENSITY
#define GYRO_NOISE_DENSITY          0.02f
#endif    /* rad/s/sqrt(Hz)   */
#ifndef ACCEL_NOISE_DENSITY
#define ACCEL_NOISE_DENSITY         0.1f
#endif     /* m/s^2/sqrt(Hz)   */
#ifndef GRAVITY_UNCERTAINTY
#if defined(DATASET_PROFILE_VIODE)
/* Drives the velocity-initialisation blend in initilaize_newest_velocity():
 *   alpha = 1 / (1 + (3*GRAVITY_UNCERTAINTY)^2 * dt / ACCEL_NOISE_DENSITY^2)
 * At 1.0 that gives alpha ~= 0.01 -- only 1 % of each IMU velocity increment is
 * taken. EuRoC survives that because its jerky motion produces large
 * increments; VIODE flies smoothly (0.3 m/s^2 against EuRoC's 3 m/s^2), so the
 * velocity state never leaves zero and the map collapses with it. 0.2 gives
 * alpha ~= 0.16, the best of a sweep over 0.02 .. 1.0. */
#define GRAVITY_UNCERTAINTY         0.2f     /* m/s^2 */
#else
#define GRAVITY_UNCERTAINTY         1.0f     /* m/s^2 */
#endif
#endif
#ifndef IMU_WEIGHT_SCALE
#define IMU_WEIGHT_SCALE            1.0f
#endif
/* Per-keyframe leak applied to the propagated velocity. 0 = upstream (unbounded
 * open-loop integration). The preprocessor cannot compare floats, so the on/off
 * switch is a separate integer from the leak itself. See
 * initilaize_newest_velocity() for the measurements that motivate it. */
/* Take the frame-to-frame rotation from the gyro instead of from the essential
 * matrix decomposition, keeping only the translation direction from vision.
 * See the measurements in vo.c and docs/23. */
/* Feed the pose graph's estimated IMU biases back into the preintegrator.
 * Upstream computes them and discards them. IMU_BIAS_GAIN damps the per-keyframe
 * update; IMU_BIAS_CLAMP rejects a diverged solve. */
#ifndef FIXED_STEP_SCALE
#define FIXED_STEP_SCALE 0.128f   /* metres; only used by SCALE_DT_MODE 3 */
#endif

/* -------------------------------------------------------------------
 * Visual-inertial alignment (optimizer/vi_alignment.c)
 * ------------------------------------------------------------------- */
/* Solves scale, gravity and per-keyframe velocities jointly over the keyframe
 * window, which is the mechanism LEVIO lacks entirely. See vi_alignment.h. */
#ifndef VI_ALIGN_ENABLE
#define VI_ALIGN_ENABLE 0
#endif
#ifndef VI_ALIGN_MIN_KEYFRAMES
#define VI_ALIGN_MIN_KEYFRAMES 5      /* below this the window is underdetermined */
#endif
#ifndef VI_ALIGN_MAX_DT
#define VI_ALIGN_MAX_DT 2.0f          /* s; a longer gap means the window is stale */
#endif
#ifndef VI_ALIGN_DAMPING
#define VI_ALIGN_DAMPING 1e-8f        /* ridge, applied AFTER Jacobi preconditioning */
#endif
#ifndef VI_ALIGN_MIN_SCALE
#define VI_ALIGN_MIN_SCALE 1e-3f
#endif
#ifndef VI_ALIGN_MAX_SCALE
#define VI_ALIGN_MAX_SCALE 1e3f
#endif
#ifndef VI_ALIGN_SCALE_GAIN
#define VI_ALIGN_SCALE_GAIN 1.0f    /* how much of the solved scale to apply to the map */
#endif
#ifndef VI_ALIGN_VELOCITY_GAIN
#define VI_ALIGN_VELOCITY_GAIN 1.0f
#endif
#ifndef VI_ALIGN_GRAVITY_GAIN
#define VI_ALIGN_GRAVITY_GAIN 0.1f
#endif

#ifndef APPLY_ESTIMATED_IMU_BIAS
#define APPLY_ESTIMATED_IMU_BIAS 0
#endif
#ifndef IMU_BIAS_GAIN
#define IMU_BIAS_GAIN  0.2f
#endif
#ifndef IMU_BIAS_CLAMP
#define IMU_BIAS_CLAMP 2.0f
#endif

/* Sliding-window average of the essential-matrix translation direction,
 * weighted by squared inlier ratio. 1 = off. See vo.c and docs/24. */
#ifndef DIRECTION_SMOOTHING_WINDOW
#define DIRECTION_SMOOTHING_WINDOW 1
#endif

#ifndef ROTATION_FROM_GYRO
#define ROTATION_FROM_GYRO 0
#endif

/* Use the exact kinematic velocity recursion instead of upstream's damped one,
 * and rotate gravity into the body frame with R rather than R_inv. Both are
 * required together; see initilaize_newest_velocity() and docs/27. */
/* Bundle-adjustment iterations per keyframe. 0 disables it entirely, which is
 * a diagnostic, not a configuration: see docs/27. */
/* Hold the rotation block fixed during bundle adjustment, optimising only the
 * translations and landmarks. Only sensible together with ROTATION_FROM_GYRO,
 * which supplies an attitude BA cannot beat. See docs/27. */
/* Bound the motion gate by the actual preintegration interval instead of one
 * camera period. Required once the other keyframe-interval fixes are on. */
#ifndef MOTION_GATE_EXACT_DT
#define MOTION_GATE_EXACT_DT 0
#endif

#ifndef BA_FREEZE_ROTATION
#define BA_FREEZE_ROTATION 0
#endif

#ifndef BA_ITERATIONS
#define BA_ITERATIONS 10
#endif

#ifndef VELOCITY_UPDATE_EXACT
#define VELOCITY_UPDATE_EXACT 0
#endif

#ifndef VELOCITY_LEAK_ENABLE
#define VELOCITY_LEAK_ENABLE 0
#endif
#ifndef VELOCITY_LEAK
#define VELOCITY_LEAK 0.0f
#endif

#define GYRO_BIAS_RANDOM_WALK_NOISE  2.0e-5f /* rad/s            */
#define ACCEL_BIAS_RANDOM_WALK_NOISE 3.0e-4f /* m/s^2            */

/* -------------------------------------------------------------------
 * IMU Data Dimensions
 * ------------------------------------------------------------------- */
#define IMU_WIDTH  28
#define IMU_HEIGHT 10

/* -------------------------------------------------------------------
 * Gravity
 * ------------------------------------------------------------------- */
#define GRAVITY_MAGNITUDE 9.81f

/* Initial gravity estimate (body frame, m/s^2).
 * Updated at runtime by the IMU calibration. */
#ifdef DATASET_PROFILE_VIODE
/* Gravity in the CAMERA frame, magnitude normalised to 9.81.
 *
 * This is not the raw accelerometer reading. pose_graph_optimizer.c:17 seeds
 * g[3] with these values and update_gravity_gyro_bias() later refreshes it via
 * g_world = R_cw^T * (-gravity_b), so the initial value has to be in the same
 * frame -- the camera frame, which is also the world frame at bootstrap.
 *
 *   g_cam = normalise( IMU_TO_CAM_R * (-mean_accel_at_rest) ) * 9.81
 *
 * Getting this wrong is not subtle: seeding it with the body-frame reading
 * instead put gravity on the wrong axis with the wrong sign, and the velocity
 * estimate came out 23x too small, which collapsed the whole trajectory. */
#define GRAVITY_INIT_X  0.0000f
#define GRAVITY_INIT_Y  9.8100f
#define GRAVITY_INIT_Z  0.0000f
#elif defined(DATASET_PROFILE_TUMVI_QUARTER) || defined(DATASET_PROFILE_TUMVI)
/* Same derivation as the VIODE block above, on the first 3 s of room1 (the
 * device is at rest there: accel std 0.17-0.20 m/s^2, |gyro| mean 0.11 rad/s):
 *
 *   g_cam = normalise( IMU_TO_CAM_R * (-mean_accel_at_rest) ) * 9.81
 *   mean_accel_at_rest = (0.102954, 0.346758, 9.762541)
 *
 * The result sits 2.3 deg off the camera's +Y axis, which is where the MoCap
 * rig holds it. |g| = 9.81 by construction. */
#define GRAVITY_INIT_X  0.393743f
#define GRAVITY_INIT_Y  9.802085f
#define GRAVITY_INIT_Z  0.014002f
#else
#define GRAVITY_INIT_X  1.6978f
#define GRAVITY_INIT_Y  9.0580f
#define GRAVITY_INIT_Z  3.3624f
#endif

/* -------------------------------------------------------------------
 * Numerical Tolerances (matrix / SVD internals)
 * ------------------------------------------------------------------- */
#define SVD_MAX_ITER_JACOBI        10
#define SVD_TOL_JACOBI          1e-12f
#define MAX_ITER_INV_POWER         15
#define TOL_INV_POWER           1e-10f
#define NUMERICAL_ZERO_THRESHOLD 1e-12f
#define VECTOR_ZERO_NORM_THRESHOLD 1e-26f

/* -------------------------------------------------------------------
 * Logging
 * ------------------------------------------------------------------- */
/* K230-VIO: made overridable from the build system (-DLOG_LEVEL=...) so the
 * verbosity can be raised for debugging without editing this file. Defaults
 * are unchanged from upstream LEVIO. */
#ifndef LOG_LEVEL
#define LOG_LEVEL          0x02  /* 0x00=ERROR, 0x01=WARNING, 0x02=INFO, 0x03=DEBUG */
#endif
#ifndef LOG_TIMING_ENABLED
#define LOG_TIMING_ENABLED 0x01
#endif

#endif /* __CONFIG__H__ */
