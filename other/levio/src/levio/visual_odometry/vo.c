// Copyright (c) 2026 ETH Zurich. All rights reserved.
// SPDX-FileCopyrightText: 2026 ETH Zurich
// SPDX-License-Identifier: MIT
// Author: Jonas Kühne

#include "vo.h"
#include "trace.h"

#include "feature_handling/orb_gap.h"
#include "feature_handling/bf_matcher.h"
#include "triangulation.h"
#include "essential_matrix.h"
#include "epnp.h"
#include "optimizer/imu_preintegration.h"

#include "levio_hooks.h"   /* K230-VIO HOOK: structured pose output */

#include "math.h"

#define Abs(a)          (((int)(a)<0)?(-(a)):(a))
#define Min(a, b)       (((a)<(b))?(a):(b))
#define Max(a, b)       (((a)>(b))?(a):(b))


#define IS_KEYFRAME 1
#define IS_REGULAR_FRAME 0

const unsigned char* levio_dynamic_mask = NULL;

/* Drops every keypoint whose pixel is marked dynamic, compacting the keypoint
 * and descriptor arrays in place.
 *
 * Applied straight after detection so that nothing downstream -- matching,
 * RANSAC, triangulation, the map -- ever sees a feature on a moving object.
 * Returns the number of keypoints removed. */
static uint16_t levio_drop_dynamic_keypoints(orb_features_t* features)
{
    if (levio_dynamic_mask == NULL || features == NULL) { return 0; }
    uint16_t kept = 0;
    for (uint16_t i = 0; i < features->kpt_counter; ++i) {
        const uint16_t x = features->kpts[i].x;
        const uint16_t y = features->kpts[i].y;
        if (x < IMG_WIDTH && y < IMG_HEIGHT &&
            levio_dynamic_mask[(size_t) y * IMG_WIDTH + x] != 0) {
            continue;                       /* on a moving object: drop */
        }
        if (kept != i) {
            features->kpts[kept] = features->kpts[i];
            /* orb_descriptor_t is an array type, so it needs a copy, not an
             * assignment. */
            memcpy(&features->descs[kept], &features->descs[i],
                   sizeof(orb_descriptor_t));
        }
        kept++;
    }
    uint16_t dropped = features->kpt_counter - kept;
    features->kpt_counter = kept;
    return dropped;
}

float K_data[9] = {CAMERA_FX,     0.0, CAMERA_CX,
                        0.0, CAMERA_FY, CAMERA_CY,
                        0.0,       0.0,       1.0};


/**
 * @brief DMA-transfers one image frame from L2 to L1 and runs ORB detection + descriptor computation.
 *
 * Copies the raw image from img_l2_buffer into L1 work memory, then runs either the
 * single-core or multi-core ORB pipeline depending on the SINGLE_CORE compile flag.
 * Results are written into orb_features (keypoints and descriptors).
 */
void process_frame(orb_features_t* orb_features,
                   uint8_t* img_l2_buffer,
                   work_memory_t work_memory)
{
    pi_cl_dma_cmd_t cmd;
    /* DMA Copy Frame */
    uint8_t* image_buffer = allocate_work_memory(&work_memory,(size_t) (IMG_SIZE));
    pi_cl_dma_cmd((uintptr_t) img_l2_buffer, (uintptr_t) image_buffer, IMG_SIZE, PI_CL_DMA_DIR_EXT2LOC, &cmd);
    /* Wait for DMA transfer to finish. */
    pi_cl_dma_wait(&cmd);
    image_data_t img = {IMG_WIDTH, IMG_HEIGHT, image_buffer};
    if(SINGLE_CORE)
    {
        orb_detect_and_compute_single_core(&img, orb_features,work_memory);
    }
    else
    {
        orb_detect_and_compute_multi_core(&img, orb_features,work_memory,NB_CORES);
    }
}

/**
 * @brief Matches current features against the world model and estimates the camera pose via EPnP RANSAC.
 *
 * Loads world descriptors from L2, runs brute-force two-way matching, then filters matches
 * by world-point age (MAX_WORLD_POINT_AGE). If enough inliers remain, calls EPnP RANSAC
 * to compute T (camera-to-world). Returns 0 on success, 1 if there are insufficient matches.
 *
 * @return 0 on success, 1 if EPnP failed (insufficient matches before or after age filter).
 */
uint8_t process_epnp_ransac(orb_features_t* curr_features,
                            pose_graph_stats_t* graph_stats,
                            matrix_2D_t* K,
                            matrix_2D_t* T,
                            feature_match_t* matches,
                            uint16_t* world_match_counter,
                            work_memory_t work_memory)
{
    orb_features_t* world_features = allocate_work_memory(&work_memory, sizeof(orb_features_t));
    world_features->kpts = NULL;
    world_features->descs = allocate_work_memory(&work_memory, (MAX_WORLD_FEATURES*sizeof(orb_descriptor_t)));
    world_features->kpt_counter = Min(graph_stats->total_features,MAX_WORLD_FEATURES);
    world_features->kpt_capacity = MAX_WORLD_FEATURES;
    storage_move_in(world_features->descs, WORLD_DESCS);

    uint16_t match_counter;
    if(SINGLE_CORE)
    {
        match_counter = bf_match_two_way_max_flow(matches,world_features,curr_features,DISABLE_MAX_FLOW,HAMMING_THRESHOLD);
    }
    else
    {
        match_counter = bf_match_two_way_max_flow_multicore(matches,world_features,curr_features,DISABLE_MAX_FLOW,HAMMING_THRESHOLD,NB_CORES);
    }
    if (match_counter < MIN_MATCHES_EPNP)
    {
        *world_match_counter = match_counter;
        mateye(T);
        pi_perf_stop(); 
        LOG_TIMING("[Feature Match World] Number of cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
        /* K230-VIO HOOK */
        levio_hook_stage("match_world", pi_perf_read(PI_PERF_CYCLES));
        pi_perf_reset();
        pi_perf_start();
        LOG_INFO("EPnP failed: insufficient matches after bf matcher (got %u, need %u)\n", match_counter, MIN_MATCHES_EPNP);
        return 1;  // EPNP_ERR_INSUFFICIENT_MATCHES
    }

    point3D_float_t* all_world_points = allocate_work_memory(&work_memory, MAX_WORLD_FEATURES*sizeof(point3D_float_t));
    point3D_float_t* matched_world_points = allocate_work_memory(&work_memory, match_counter*sizeof(point3D_float_t));
    point2D_u16_t* image_points = allocate_work_memory(&work_memory, match_counter*sizeof(point2D_u16_t));
    uint8_t* world_point_ages = allocate_work_memory(&work_memory, MAX_WORLD_FEATURES*sizeof(uint8_t));
    storage_move_in(all_world_points,WORLD_PTS);
    storage_move_in(world_point_ages, WORLD_PT_AGE);

    uint16_t filtered_match_counter = 0;
    for (uint16_t i = 0; i < match_counter; ++i){
        feature_match_t cur_match = matches[i];
        uint8_t age = world_point_ages[cur_match.feat_idx0];
        if (age < MAX_WORLD_POINT_AGE)
        {
            matched_world_points[filtered_match_counter] = all_world_points[cur_match.feat_idx0];
            image_points[filtered_match_counter] = curr_features->kpts[cur_match.feat_idx1];
            filtered_match_counter++;
        }
    }
    pi_perf_stop(); 
    LOG_TIMING("[Feature Match World] Number of cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
    /* K230-VIO HOOK */
    levio_hook_stage("match_world", pi_perf_read(PI_PERF_CYCLES));
    pi_perf_reset();
    pi_perf_start();
    if (filtered_match_counter < MIN_MATCHES_EPNP)
    {
        *world_match_counter = match_counter;
        mateye(T);
        LOG_INFO("EPnP failed: insufficient matches after age filter (got %u from %u, need %u)\n", filtered_match_counter, match_counter, MIN_MATCHES_EPNP);
        return 1;  // EPNP_ERR_AGE_FILTER
    }
    if(SINGLE_CORE)
    {
        ransac_epnp_compute_pose(matched_world_points, image_points, K, T, work_memory, filtered_match_counter, EPNP_ITERS);
    }
    else
    {
        ransac_epnp_compute_pose_multicore(matched_world_points, image_points, K, T, work_memory, filtered_match_counter, EPNP_ITERS, NB_CORES);
    }
    *world_match_counter = match_counter;
    LOG_INFO("EPnP successful\n");
    return 0;
}

/**
 * @brief Estimates relative pose via 8-point RANSAC and recovers metric scale from IMU preintegration.
 *
 * Runs RANSAC on the essential matrix from curr↔prev feature matches, recovers the relative
 * rotation and (unit-scale) translation via recoverPose, then scales the translation by
 * ||prev_velocity|| * min(dt, 1s) from IMU preintegration. The resulting T_rel is
 * composed with T_prev to produce the global pose T.
 * Falls back to T = I * T_prev if match_counter < 10.
 */
#if MOTION_GATE_ENABLED
/* Camera centre of a row-major world-to-camera 4x4: c = -R^T t. */
static void levio_cam_centre(const matrix_2D_t* T, float* c)
{
    const float* d = T->data;
    for (int i = 0; i < 3; ++i) {
        c[i] = -(d[0*4+i]*d[3] + d[1*4+i]*d[7] + d[2*4+i]*d[11]);
    }
}

/* Write a new camera centre back into the transform: t = -R c. */
static void levio_set_cam_centre(matrix_2D_t* T, const float* c)
{
    float* d = T->data;
    for (int i = 0; i < 3; ++i) {
        d[i*4+3] = -(d[i*4+0]*c[0] + d[i*4+1]*c[1] + d[i*4+2]*c[2]);
    }
}
#endif

void process_eight_point_ransac(orb_features_t* curr_features,
                                orb_features_t* prev_features,
                                feature_match_t* matches,
                                matrix_2D_t* T_prev,
                                pose_graph_stats_t* graph_stats,
                                matrix_2D_t* K,
                                matrix_2D_t* T,
                                uint16_t match_counter,
                                work_memory_t work_memory)
{
    float* t_rel = allocate_work_memory(&work_memory,16*sizeof(float));
    matrix_2D_t T_rel = {t_rel,{4,4,0,0}};
    if (match_counter < MIN_MATCHES_ESSENTIAL)
    {
        mateye(&T_rel);
        matmul(&T_rel,T_prev,T);
        LOG_INFO("8-point RANSAC failed: insufficient matches (got %u, need 10, 8 for algorithm and 2 as threshold)\n", match_counter);
        matprint(T, INFO_LEVEL);
        return;
    }
    float E_data[9] = {0};
    matrix_2D_t E = {E_data,{3,3,0,0}};

    uint16_t essential_inliers;
    if(SINGLE_CORE)
    {
        essential_inliers = ransacEssentialMatrix(prev_features->kpts,curr_features->kpts,matches, K, &E, work_memory, match_counter, ESSENTIAL_ITERS);
    }
    else
    {
        essential_inliers = ransacEssentialMatrixMulticore(prev_features->kpts,curr_features->kpts,matches, K, &E, work_memory, match_counter, ESSENTIAL_ITERS, NB_CORES);
    }

    /* Refuse a model the data does not support. Without this the decomposition
     * can return a rank-deficient rotation, which is silently accepted and
     * seeds the world map with garbage. See MIN_ESSENTIAL_INLIERS in config.h
     * for the EuRoC V2_03 case that motivated it. Falling back to identity
     * leaves the pose on IMU propagation for this frame, which is exactly what
     * the insufficient-matches path above already does. */
    TRACE_SET(ess_inliers, essential_inliers);
    TRACE_SET(n_match_frame, match_counter);
    if (essential_inliers < MIN_ESSENTIAL_INLIERS ||
        (float) essential_inliers < MIN_ESSENTIAL_INLIER_RATIO * (float) match_counter)
    {
        TRACE_SET(ess_rejected, 1);
        mateye(&T_rel);
        matmul(&T_rel,T_prev,T);
        LOG_INFO("8-point RANSAC rejected: %u inliers of %u matches (need >= %u and >= %.0f%%)\n",
                 essential_inliers, match_counter, (unsigned) MIN_ESSENTIAL_INLIERS,
                 (double) (100.0f * MIN_ESSENTIAL_INLIER_RATIO));
        matprint(T, INFO_LEVEL);
        return;
    }

    recoverPose(prev_features->kpts,curr_features->kpts,matches, K, &E, &T_rel, work_memory, match_counter, SINGLE_CORE, NB_CORES);

#if ROTATION_FROM_GYRO
    /* Replace the essential-matrix rotation with the gyro-integrated one,
     * keeping only the translation *direction* from vision.
     *
     * Measured on VIODE city_day 1_low, 1327 frames, against ground truth:
     *
     *   essential-matrix rotation  0.466 deg/frame median, but a systematic
     *                              0.0668 deg/frame bias -> 127.65 deg after 66 s
     *   gyro integration                                  ->   0.212 deg after 66 s
     *
     * Open-loop trajectory ATE with ground-truth scale, same vision directions:
     *   essential-matrix rotation  11.80 m
     *   gyro rotation               2.97 m   (identical to using GT rotation)
     *
     * The gyro is 600x better here because VIODE's simulated gyro noise density
     * is 0.00024 rad/s/sqrt(Hz), and because a bias that small cannot compete
     * with the essential matrix's structural bias at this baseline. See docs/23.
     *
     * The interval is keyframe-to-current-frame, matching the 8-point path's
     * reference. delta_R composes j-from-i while T_rel is i-from-j, hence the
     * transpose. */
    {
        float dR[9];
        if (get_keyframe_delta_R(dR)) {
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    t_rel[i*4+j] = dR[j*3+i];      /* transpose */
        }
    }
#endif

#if DIRECTION_SMOOTHING_WINDOW > 1
    /* Average the recovered translation direction over a short sliding window,
     * weighted by RANSAC support.
     *
     * A two-view essential matrix resolves the translation direction only as
     * well as its baseline allows, and at 20 fps the per-frame baseline is
     * short. Measured on VIODE 1_low against ground truth, split by the size of
     * the true step: 47.9 deg for the smallest quartile against 8.7 deg for the
     * largest. The signal is there over a longer interval -- direction error
     * falls from 74.8 deg at one frame to 20.8 deg at two seconds -- so
     * averaging recovers part of it without any new geometry.
     *
     * Weighted by the squared inlier ratio because nothing else predicts the
     * error: inlier count, match count, parallax and keypoint count all
     * correlate below 0.13 with direction error (docs/24). The inlier ratio is
     * the least bad of them.
     *
     * Stored directions live in the camera frame of the frame that produced
     * them, so each is rotated forward by the current frame-to-frame rotation
     * before being averaged into the present frame. */
    {
        static float dir_buf[DIRECTION_SMOOTHING_WINDOW][3];
        static float dir_w[DIRECTION_SMOOTHING_WINDOW];
        static uint8_t dir_n = 0;

        float tn = sqrtf(t_rel[3]*t_rel[3] + t_rel[7]*t_rel[7] + t_rel[11]*t_rel[11]);
        if (tn > 1e-9f) {
            /* Bring every stored direction into the current camera frame. */
            for (uint8_t b = 0; b < dir_n; ++b) {
                float v[3] = { dir_buf[b][0], dir_buf[b][1], dir_buf[b][2] };
                for (int r0 = 0; r0 < 3; ++r0) {
                    dir_buf[b][r0] = t_rel[r0*4+0]*v[0] + t_rel[r0*4+1]*v[1] + t_rel[r0*4+2]*v[2];
                }
            }
            if (dir_n == DIRECTION_SMOOTHING_WINDOW) {
                for (uint8_t b = 1; b < DIRECTION_SMOOTHING_WINDOW; ++b) {
                    dir_buf[b-1][0] = dir_buf[b][0];
                    dir_buf[b-1][1] = dir_buf[b][1];
                    dir_buf[b-1][2] = dir_buf[b][2];
                    dir_w[b-1] = dir_w[b];
                }
                dir_n--;
            }
            float w = (match_counter > 0)
                    ? ((float) essential_inliers / (float) match_counter) : 0.0f;
            dir_buf[dir_n][0] = t_rel[3]  / tn;
            dir_buf[dir_n][1] = t_rel[7]  / tn;
            dir_buf[dir_n][2] = t_rel[11] / tn;
            dir_w[dir_n] = w * w;
            dir_n++;

            float acc[3] = {0.0f, 0.0f, 0.0f};
            for (uint8_t b = 0; b < dir_n; ++b) {
                acc[0] += dir_w[b] * dir_buf[b][0];
                acc[1] += dir_w[b] * dir_buf[b][1];
                acc[2] += dir_w[b] * dir_buf[b][2];
            }
            float an = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
            if (an > 1e-9f) {
                t_rel[3]  = acc[0] / an;
                t_rel[7]  = acc[1] / an;
                t_rel[11] = acc[2] / an;
            }
        }
    }
#endif

#ifdef LEVIO_TRACE_CSV
    /* recoverPose leaves t as a unit vector; capture it before scaling. */
    for (int _i = 0; _i < 3; _i++)
        for (int _j = 0; _j < 3; _j++)
            levio_trace.R_rel[_i*3+_j] = t_rel[_i*4+_j];
    TRACE_VEC(t_dir, t_rel[3], t_rel[7], t_rel[11]);
#endif

    /* Last line of defence. The inlier-ratio test above is statistical and can
     * be tuned wrong; this one is not tunable -- either the decomposition
     * produced an element of SO(3) or it did not. Cheap enough to keep even
     * when the ratio test is disabled for an ablation. */
    if (ROTATION_VALIDITY_CHECK && !is_valid_rotation_4x4(&T_rel, ROTATION_VALIDITY_TOL))
    {
        TRACE_SET(ess_rejected, 2);
        mateye(&T_rel);
        matmul(&T_rel,T_prev,T);
        LOG_INFO("8-point RANSAC rejected: recovered rotation is not in SO(3)\n");
        matprint(T, INFO_LEVEL);
        return;
    }

    LOG_DEBUG("Prev Velocity [%f, %f, %f]\n",graph_stats->prev_velocity.x,graph_stats->prev_velocity.y,graph_stats->prev_velocity.z);
    point3D_float_t dp;
    float dt;
    get_current_dp_and_dt(&dp, &dt);
    LOG_DEBUG("IMU Preintegration dp [%f, %f, %f], dt %f\n",dp.x, dp.y, dp.z, dt);
#if SCALE_DT_MODE == 3
    /* Diagnostic: a fixed step length, bypassing the velocity state entirely.
     * Not a usable configuration -- it needs the true mean speed -- but it
     * isolates how much of the remaining error is scale and how much is
     * rotation and direction. See docs/23. */
    (void) dt;
    float scale = FIXED_STEP_SCALE;
#elif SCALE_DT_MODE == 0
    float scale = vec3_norm(&graph_stats->prev_velocity)*(Min(dt, 1.0f));
#elif SCALE_DT_MODE == 1
    (void) dt;
    float scale = vec3_norm(&graph_stats->prev_velocity)*(1.0f/CAMERA_FPS);
#else
    /* Speed straight from the IMU; see config.h. */
    float scale;
    {
        extern float g[3];
        float half_dt2 = 0.5f * dt * dt;
        point3D_float_t dp_free = {
            dp.x - g[0] * half_dt2,
            dp.y - g[1] * half_dt2,
            dp.z - g[2] * half_dt2
        };
        float v_imu = (dt > 1e-3f) ? (vec3_norm(&dp_free) / dt) : 0.0f;
        float v_opt = vec3_norm(&graph_stats->prev_velocity);
        scale = Max(v_imu, v_opt) * (1.0f / CAMERA_FPS);
        LOG_INFO("Scale src: v_imu %f v_opt %f\n", v_imu, v_opt);
    }
#endif
    LOG_INFO("Scale: %f\n", scale);
    TRACE_SET(scale, scale);

#if ZUPT_MODE == 0
    scaleTranslationOfTransformation(&T_rel, Max(scale, MIN_TRANSLATION_SCALE));
#elif ZUPT_MODE == 1
    /* No floor: a stationary platform gets a zero-length translation instead of
     * a 1 cm step in a direction the degenerate essential matrix cannot know. */
    scaleTranslationOfTransformation(&T_rel, scale);
    LOG_INFO("ZUPT mode 1, applied scale: %f\n", scale);
#elif ZUPT_MODE == 2
    /* Apply the floor only when the image also says we moved. Parallax is an
     * independent witness: the IMU can read near-zero velocity while drifting,
     * but it cannot make static pixels move. */
    {
        float zupt_parallax = averageParallax(prev_features->kpts, curr_features->kpts,
                                              matches, K, match_counter);
        float applied = (scale < MIN_TRANSLATION_SCALE &&
                         zupt_parallax < ZUPT_PARALLAX_THRESHOLD)
                        ? 0.0f
                        : Max(scale, MIN_TRANSLATION_SCALE);
        LOG_INFO("ZUPT parallax %f, applied scale: %f\n", zupt_parallax, applied);
        scaleTranslationOfTransformation(&T_rel, applied);
    }
#elif ZUPT_MODE == 3
    /* Parallax alone. During a standstill the velocity estimate freezes at a
     * stale non-zero value (no keyframe -> no optimizer -> no update), so any
     * gate that tests the scale is useless. Static pixels are the one witness
     * that cannot be fooled. */
    {
        float zupt_parallax = averageParallax(prev_features->kpts, curr_features->kpts,
                                              matches, K, match_counter);
        if (zupt_parallax < ZUPT_PARALLAX_THRESHOLD) {
            TRACE_SET(zupt_fired, 1);
            scaleTranslationOfTransformation(&T_rel, 0.0f);
            LOG_INFO("ZUPT: standstill, parallax %f, translation zeroed\n", zupt_parallax);
        } else {
            scaleTranslationOfTransformation(&T_rel, Max(scale, MIN_TRANSLATION_SCALE));
        }
    }
#else
#error "unknown ZUPT_MODE"
#endif
    matprint(&T_rel, INFO_LEVEL);

    /* Apply new transformation to previous pose */
    matmul(&T_rel,T_prev,T);
    LOG_INFO("8-point result\n");
    matprint(T, INFO_LEVEL);
}

/**
 * @brief Updates the world map by re-observing existing landmarks and triangulating new ones.
 *
 * For each world match, updates the descriptor and resets the age of landmarks that
 * re-project within a 2-pixel threshold. For frame matches where neither keypoint has a
 * known world-point ID, triangulates a new 3D point and inserts it into the first available
 * slot (determined by age threshold, adaptively lowered if the buffer is full).
 * curr_wp_ids and prev_wp_ids are updated in-place and written back to L2 storage.
 */
void update_world_points(orb_features_t* prev_features,
                         orb_features_t* curr_features,
                         feature_match_t* frame_matches,
                         feature_match_t* world_matches,
                         int16_t* prev_wp_ids,
                         int16_t* curr_wp_ids,
                         matrix_2D_t* prev_T,
                         matrix_2D_t* curr_T,
                         matrix_2D_t* K,
                         pose_graph_stats_t* graph_stats,
                         work_memory_t work_memory,
                         uint16_t frame_match_counter,
                         uint16_t world_match_counter)
{
    /* Init Data */
    point3D_float_t* world_points = allocate_work_memory(&work_memory, (MAX_WORLD_FEATURES*sizeof(point3D_float_t)));
    orb_descriptor_t* world_descs = allocate_work_memory(&work_memory, (MAX_WORLD_FEATURES*sizeof(orb_descriptor_t)));
    uint8_t* world_point_ages = allocate_work_memory(&work_memory, MAX_WORLD_FEATURES*sizeof(uint8_t));
    for(uint16_t i = 0; i < MAX_KEYPOINTS; ++i)
    {
        curr_wp_ids[i] = -1;
    }
    storage_move_in(world_points,WORLD_PTS);
    storage_move_in(world_descs, WORLD_DESCS);
    storage_move_in(world_point_ages, WORLD_PT_AGE);
    for(uint16_t i = 0; i < Min(graph_stats->total_features, MAX_WORLD_FEATURES); ++i)
    {
        world_point_ages[i] +=1;
    }

    uint16_t known_counter = 0;
    for(uint16_t i = 0; i < world_match_counter; ++i)
    {
        feature_match_t match = world_matches[i];
        uint16_t idx0 = match.feat_idx0;
        uint16_t idx1 = match.feat_idx1;
        point3D_float_t wp = world_points[idx0];
        float error = reprojection_error_squared(&wp, &curr_features->kpts[idx1], curr_T, K);
        if(error < REPROJECTION_ERROR_SQ)
        {
            /* Add new observation / Update descriptor */
            curr_wp_ids[idx1] = idx0;
            memcpy(&world_descs[idx0],&curr_features->descs[idx1],sizeof(orb_descriptor_t));
            world_point_ages[idx0] = 0;
            known_counter += 1;
        }
    }

    uint16_t new_counter = 0;
    uint16_t next_wp_index = 0;
    int16_t age_threshold = MAX_WORLD_POINT_AGE;
    for(uint16_t i = 0; i < frame_match_counter; ++i)
    {
        feature_match_t match = frame_matches[i];
        uint16_t idx0 = match.feat_idx0;
        uint16_t idx1 = match.feat_idx1;
        if ((prev_wp_ids[idx0] == -1) && (curr_wp_ids[idx1] == -1))
        {
            point3D_float_t wp = triangulatePoint(&prev_features->kpts[idx0], &curr_features->kpts[idx1], K, prev_T, curr_T, work_memory);
            if (fabsf(wp.x) < 1e-6f && fabsf(wp.y) < 1e-6f && fabsf(wp.z) < 1e-6f){
                continue;
            }
            float error0 = reprojection_error_squared(&wp, &prev_features->kpts[idx0], prev_T, K);
            float error1 = reprojection_error_squared(&wp, &curr_features->kpts[idx1], curr_T, K);
            if(error0 < REPROJECTION_ERROR_SQ && error1 < REPROJECTION_ERROR_SQ)
            {
                while (world_point_ages[next_wp_index] < age_threshold)
                {
                    next_wp_index++;
                    if(next_wp_index >= MAX_WORLD_FEATURES)
                    {
                        age_threshold -= 1;
                        next_wp_index = 0;
                        LOG_INFO("Adapting age threshold to %d\n", age_threshold);
                        if (age_threshold <= 0)
                        {
                            LOG_WARNING("\nWARNING: LANDMARK BUFFER FULL, NO FREE SLOT\n\n");
                            break;
                        }
                        if (age_threshold < MAX_KEYFRAMES)
                        {
                            LOG_WARNING("\nWARNING: LANDMARK BUFFER OVERFILLING\n\n");
                        }
                    }
                }
                if (age_threshold <= 0) break;
                int16_t offset = next_wp_index;
                memcpy(world_points+offset, &wp, sizeof(point3D_float_t));
                memcpy(world_descs+offset,curr_features->descs+idx1, sizeof(orb_descriptor_t));
                curr_wp_ids[idx1] = offset;
                /* Requires Update of pose graph of prev keyframe */
                prev_wp_ids[idx0] = offset;
                world_point_ages[offset] = 0;
                graph_stats->total_features += 1;
                new_counter += 1;
            }
        }
    }
    LOG_INFO("Newly triangulated points: %d\nRe-observed points: %d\n", new_counter, known_counter);
    storage_move_out(curr_wp_ids, PREV_WP_IDS);
    storage_move_out(world_points,WORLD_PTS);
    storage_move_out(world_descs, WORLD_DESCS);
    storage_move_out(world_point_ages, WORLD_PT_AGE);
    return;
}

uint8_t process_vo_pipeline(pose_graph_stats_t* graph_stats,
                            uint8_t* img_l2_buffer,
                            work_memory_t work_memory)
{
    LOG_WARNING("*** VO Pipeline Results ***\n");
    LOG_WARNING("Total Frames: \t%d\nKeyframes: \t%d\nFeatures: \t%d\n",graph_stats->total_frames,graph_stats->total_keyframes,graph_stats->total_features);
    LOG_WARNING("***\n\n");

    pi_perf_reset();
    pi_perf_start();
    /* Extract new features */
    orb_features_t* curr_features = allocate_work_memory(&work_memory, sizeof(orb_features_t));
    curr_features->kpts = allocate_work_memory(&work_memory, (MAX_KEYPOINTS*sizeof(point2D_u16_t)));
    curr_features->descs = allocate_work_memory(&work_memory, (MAX_KEYPOINTS*sizeof(orb_descriptor_t)));
    curr_features->kpt_counter = 0;
    curr_features->kpt_capacity = MAX_KEYPOINTS;
    process_frame(curr_features, img_l2_buffer, work_memory);
    {
        uint16_t dropped = levio_drop_dynamic_keypoints(curr_features);
        if (dropped) {
            LOG_INFO("Dynamic mask: dropped %u keypoints, %u remain\n",
                     dropped, curr_features->kpt_counter);
        }
        TRACE_SET(n_dyn_dropped, dropped);
    }
    graph_stats->total_frames += 1;
    if (graph_stats->total_keyframes == 0)
    {
        pose_graph_add_first_keyframe(graph_stats, curr_features, work_memory);
        /* K230-VIO HOOK */
        levio_hook_pose(NULL, LEVIO_POSE_FIRST_KEYFRAME, graph_stats->total_frames);
        TRACE_SET(is_keyframe, 1);
        return IS_KEYFRAME;
    }
    pi_perf_stop(); 
    LOG_TIMING("[Extract Features] Number of cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
    /* K230-VIO HOOK: same number, structured */
    levio_hook_stage("extract", pi_perf_read(PI_PERF_CYCLES));


    pi_perf_reset();
    pi_perf_start();
    /* Run EPnP RANSAC */
    matrix_2D_t K = {K_data,{3,3,0,0}};
    float* t = allocate_work_memory(&work_memory,16*sizeof(float));
    matrix_2D_t T = {t,{4,4,0,0}};
    feature_match_t* world_matches = allocate_work_memory(&work_memory,MAX_WORLD_FEATURES*sizeof(feature_match_t));
    uint16_t world_match_counter = 0;
    uint8_t epnp_failed = process_epnp_ransac(curr_features,graph_stats,&K,&T,world_matches,&world_match_counter,work_memory);
    pi_perf_stop(); 
    LOG_TIMING("[EPnP] Number of cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
    /* K230-VIO HOOK */
    levio_hook_stage("epnp", pi_perf_read(PI_PERF_CYCLES));
    TRACE_SET(epnp_used, (uint8_t)(epnp_failed ? 0 : 1));
    TRACE_SET(n_match_world, world_match_counter);
    TRACE_SET(n_kpts, curr_features->kpt_counter);
    matprint(&T, INFO_LEVEL);

    /* Match with previous keyframe features */
    pi_perf_reset();
    pi_perf_start();
    orb_features_t* prev_features = pose_graph_get_prev_features(graph_stats, &work_memory);
    feature_match_t* frame_matches = allocate_work_memory(&work_memory, (sizeof(feature_match_t)*MAX_KEYPOINTS));
    uint16_t frame_match_counter;
    if(SINGLE_CORE)
    {
        frame_match_counter = bf_match_two_way_max_flow(frame_matches,prev_features,curr_features,DISABLE_MAX_FLOW,HAMMING_THRESHOLD);
    }
    else
    {
        frame_match_counter = bf_match_two_way_max_flow_multicore(frame_matches,prev_features,curr_features,DISABLE_MAX_FLOW,HAMMING_THRESHOLD, NB_CORES);
    }
    pi_perf_stop(); 
    LOG_TIMING("[Feature Match Frames] Number of cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
    /* K230-VIO HOOK */
    levio_hook_stage("match_frames", pi_perf_read(PI_PERF_CYCLES));

    /* Solve 8-point problem using RANAC (if epnp failed) */
    pi_perf_reset();
    pi_perf_start();
    float* t_prev = (float*)graph_stats->prev_pose;
    matrix_2D_t T_prev = {t_prev,{4,4,0,0}};
    if(epnp_failed)
    {
        process_eight_point_ransac(curr_features,prev_features,frame_matches,&T_prev,graph_stats,&K,&T,frame_match_counter,work_memory);
    }
    pi_perf_stop(); 
    LOG_TIMING("[8-point] Number of cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
    /* K230-VIO HOOK */
    levio_hook_stage("ess", pi_perf_read(PI_PERF_CYCLES));

#if MOTION_GATE_ENABLED
    /* Bound the frame-to-frame displacement by what the IMU says is possible.
     * Applies to both the EPnP and the 8-point result, since both can produce
     * physically impossible jumps. See config.h. */
    {
        point3D_float_t dp_g; float dt_g;
        get_current_dp_and_dt(&dp_g, &dt_g);
        /* delta_p still carries gravity -- LEVIO removes it later, in the
         * optimiser. Using it raw makes v_imu roughly 0.5*|g|*dt, which on a
         * fast sequence is *smaller* than the true speed and the gate then
         * throttles real motion. On VIODE at 320x240 that collapsed the whole
         * trajectory to a 1.5 m blob. */
        extern float g[3];
        float half_dt2_g = 0.5f * dt_g * dt_g;
        point3D_float_t dp_free = { dp_g.x - g[0]*half_dt2_g,
                                    dp_g.y - g[1]*half_dt2_g,
                                    dp_g.z - g[2]*half_dt2_g };
        /* Right after a keyframe the preintegrator has just been reset, so dt is
         * a fraction of a frame and dp carries no usable speed. Gating on that
         * throttles real motion -- on VIODE, where 43 % of frames follow a
         * keyframe, it collapsed the trajectory to a 2.6 m blob. Skip the gate
         * until the IMU has integrated at least one frame period. */
        float min_dt = 0.5f / CAMERA_FPS;
        float v_imu  = (dt_g > min_dt) ? (vec3_norm(&dp_free) / dt_g) : -1.0f;
        float v_ref  = Max(v_imu, vec3_norm(&graph_stats->prev_velocity));
#if MOTION_GATE_EXACT_DT
        /* T_rel spans the interval since the last KEYFRAME, not one camera
         * period: the 8-point reference is the previous keyframe. Budgeting
         * v_ref/CAMERA_FPS therefore under-bounds every frame that is not the
         * first after a keyframe, and at 2.6 frames per keyframe -- 8.7 at
         * KEYFRAME_PARALLAX_THRESHOLD 0.15 -- that is nearly all of them.
         *
         * Third instance of the same mistake in this pipeline: the rotation
         * (get_frame_delta_R vs get_keyframe_delta_R) and the translation scale
         * (SCALE_DT_MODE) both assumed one frame period as well. See docs/27. */
        float gate_dt  = (dt_g > 1e-4f) ? dt_g : (1.0f / CAMERA_FPS);
        float max_step = (v_imu < 0.0f && v_ref <= 0.0f)
                         ? 1e30f                    /* no evidence: do not gate */
                         : MOTION_GATE_K * (v_ref * gate_dt) + MOTION_GATE_MARGIN;
#else
        float max_step = (v_imu < 0.0f && v_ref <= 0.0f)
                         ? 1e30f                    /* no evidence: do not gate */
                         : MOTION_GATE_K * (v_ref / CAMERA_FPS) + MOTION_GATE_MARGIN;
#endif

        float c_prev[3], c_curr[3];
        levio_cam_centre(&T_prev, c_prev);
        levio_cam_centre(&T,      c_curr);
        float dx = c_curr[0]-c_prev[0], dy = c_curr[1]-c_prev[1], dz = c_curr[2]-c_prev[2];
        float step = sqrtf(dx*dx + dy*dy + dz*dz);

#ifdef LEVIO_GATE_TRACE
        LOG_INFO("Motion gate state: dt %f v_imu %f v_prev %f max_step %f step %f\n",
                 dt_g, v_imu, vec3_norm(&graph_stats->prev_velocity), max_step, step);
#endif

        TRACE_SET(gate_step, step);
        TRACE_SET(gate_max,  max_step);
        TRACE_SET(imu_dt,    dt_g);
        TRACE_VEC(dp, dp_g.x, dp_g.y, dp_g.z);
        TRACE_VEC(v_opt, graph_stats->prev_velocity.x,
                         graph_stats->prev_velocity.y,
                         graph_stats->prev_velocity.z);
#ifdef LEVIO_TRACE_CSV
        { extern float g[3]; TRACE_VEC(g_world, g[0], g[1], g[2]); }
#endif
        if (step > max_step && step > 1e-6f) {
            TRACE_SET(gate_fired, 1);
            float k = max_step / step;
            float c_new[3] = { c_prev[0] + dx*k, c_prev[1] + dy*k, c_prev[2] + dz*k };
            levio_set_cam_centre(&T, c_new);
            LOG_INFO("Motion gate: step %f > %f m, clamped\n", step, max_step);
        }
    }
#endif

    float parallax = averageParallax(prev_features->kpts,curr_features->kpts,frame_matches, &K, frame_match_counter);
    LOG_INFO("Parallax normalized %f\n", parallax);
    TRACE_SET(parallax, parallax);
    TRACE_SET(n_match_frame, frame_match_counter);
    if(parallax < KEYFRAME_PARALLAX_THRESHOLD)
    {
        /* No keyframe, just report transformation */
        LOG_WARNING("Regular Frame with Transformation\n");
        matprint(&T, WARNING_LEVEL);
        /* K230-VIO HOOK */
        levio_hook_pose(T.data, LEVIO_POSE_REGULAR, graph_stats->total_frames);
        TRACE_SET(is_keyframe, 0);
        return IS_REGULAR_FRAME;
    }

    LOG_INFO("Triangulating new world points\n");
    pi_perf_reset();
    pi_perf_start();
    int16_t* curr_wp_ids = allocate_work_memory(&work_memory, (MAX_KEYPOINTS*sizeof(int16_t)));
    int16_t* prev_wp_ids = allocate_work_memory(&work_memory, (MAX_KEYPOINTS*sizeof(int16_t)));
    storage_move_in(prev_wp_ids, PREV_WP_IDS);
    update_world_points(prev_features, curr_features, frame_matches, world_matches, prev_wp_ids, curr_wp_ids, &T_prev, &T, &K, graph_stats, work_memory, frame_match_counter, world_match_counter);
    
    pose_graph_add_keyframe(graph_stats, T.data, curr_features, prev_features, curr_wp_ids, prev_wp_ids, work_memory);

    pi_perf_stop(); 
    LOG_TIMING("[Triangulation] Number of cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
    /* K230-VIO HOOK */
    levio_hook_stage("triang", pi_perf_read(PI_PERF_CYCLES));

    LOG_WARNING("Adding keyframe\n");
    matprint(&T, WARNING_LEVEL);

    /* K230-VIO HOOK */
    levio_hook_pose(T.data, LEVIO_POSE_KEYFRAME, graph_stats->total_frames);

    TRACE_SET(is_keyframe, 1);
    return IS_KEYFRAME;
}