// K230-VIO — visual-inertial alignment over the keyframe window.
//
// The piece LEVIO is missing. Upstream obtains the monocular translation scale
// from a velocity state that is pure open-loop dead reckoning: every keyframe
// adds alpha*(dv + g dt) and nothing ever subtracts, so on VIODE city_day
// 1_low it ramps 0 -> 47 m/s against a true 2.4 m/s and every frame step comes
// out ~9x too long (docs/22 section 6).
//
// Preintegration cannot fix that on its own: dp is defined as the residual
// after v_i*dt and gravity are removed, so for near-constant velocity it is
// essentially zero -- measured |dp| 0.0494 m against a gravity term of
// 0.0491 m (docs/23 section 6).
//
// What *is* observable is the joint system over a window of keyframes:
//
//     s*(c_j - c_i) = v_i*dt + 0.5*g*dt^2 + R_i*dp_ij
//     v_j           = v_i    +     g*dt   + R_i*dv_ij
//
// with c the camera centres the pose graph already holds, s a single scale
// correction for the window, g the gravity vector and v the per-keyframe
// velocities. Linear in all of them, and overdetermined: 6*(n-1) equations
// against 4+3n unknowns, i.e. 42 against 28 at MAX_KEYFRAMES = 8. This is the
// same structure VINS-Mono solves in visualInitialAlign().
//
// The single s per window is an approximation -- the per-frame scales differ
// because they come from a drifting |v| -- but over the ~1 s a window spans
// the drift is a few percent, and s is anchored by the accelerometer content
// in dp, which is real information rather than a recursion of itself.
//
// SPDX-License-Identifier: MIT

#ifndef __VI_ALIGNMENT__H__
#define __VI_ALIGNMENT__H__

#include "pmsis.h"
#include "definitions/type_definitions.h"
#include "definitions/pose_graph.h"
#include "definitions/work_memory.h"

/**
 * @brief Solves for scale, gravity and per-keyframe velocities over the window.
 *
 * Poses are the T_cw the pose graph holds; camera centres are derived as
 * c = -R^T t. Velocities and gravity are in the world frame, matching the
 * convention in initilaize_newest_velocity().
 *
 * @param poses       Ring buffer of MAX_KEYFRAMES poses.
 * @param imu_factors Ring buffer of factors; imu_factors[j] connects j-1 to j.
 * @param velocities  In/out: overwritten with the solved velocities on success.
 * @param g_world     In/out: gravity, refined and renormalised to |g|.
 * @param num_poses   Number of valid poses in the window.
 * @param total_poses Running keyframe count, for ring-buffer indexing.
 * @param scale_out   Optional: receives the solved scale correction.
 *
 * @return 1 if the solve succeeded and the result passed the sanity checks,
 *         0 if it was rejected (in which case nothing is modified).
 */
uint8_t vi_align_window(const pose_t* poses,
                        const imu_factor_with_bias_t* imu_factors,
                        point3D_float_t* velocities,
                        float* g_world,
                        uint16_t num_poses,
                        uint16_t total_poses,
                        float* scale_out,
                        work_memory_t work_memory);

#endif
