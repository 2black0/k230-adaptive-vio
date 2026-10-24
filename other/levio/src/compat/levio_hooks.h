/*
 * K230-VIO — observation hooks into the LEVIO pipeline.
 *
 * LEVIO reports poses only via printf, which the upstream visualize_log.py
 * scrapes back out of the console. That is too fragile and too lossy (%f gives
 * 6 decimals) to serve as the phase 2 validation gate, so we expose the pose
 * through a callback instead.
 *
 * This is the ONLY intentional modification to the LEVIO sources in phase 1.
 * It is additive: the original printf/matprint calls are left untouched, so
 * visualize_log.py keeps working. Call sites are marked "K230-VIO HOOK" in
 * src/levio/visual_odometry/vo.c.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __LEVIO_HOOKS_H__
#define __LEVIO_HOOKS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LEVIO_POSE_FIRST_KEYFRAME 2
#define LEVIO_POSE_KEYFRAME       1
#define LEVIO_POSE_REGULAR        0

/**
 * @brief Called once per processed frame with the estimated pose.
 *
 * @param T_cw_16 Row-major 4x4 world-to-camera transform, or NULL for the
 *                bootstrap keyframe where the pose is the identity by
 *                construction.
 * @param kind    One of LEVIO_POSE_*.
 * @param frame_number Value of graph_stats->total_frames at emission time.
 *
 * The harness provides the implementation. A no-op default lives in
 * traj_writer.c so the pipeline links even when no writer is attached.
 */
void levio_hook_pose(const float* T_cw_16, int kind, uint32_t frame_number);

/**
 * @brief Called once per pipeline stage, with that stage's measured duration.
 *
 * LEVIO already times every stage with pi_perf_* and prints the result through
 * LOG_TIMING, but only as prose on stdout. This hook carries the same number
 * out in structured form so the harness can put it in framelog.csv.
 *
 * Emitted next to the existing LOG_TIMING call, so the text log is unchanged.
 *
 * @param name  Stage label: "extract", "match_world", "match_frames", "epnp",
 *              "ess", "triang", "optimizer", "imu". Unknown names are ignored.
 * @param ns    Stage duration, in nanoseconds (the unit of pi_perf_read on
 *              every architecture here; see compat/pmsis.c).
 *
 * The harness provides the implementation. A no-op default lives in
 * harness/metrics.c so the pipeline links even with no writer attached.
 */
void levio_hook_stage(const char* name, uint32_t ns);

#ifdef __cplusplus
}
#endif

/* ------------------------------------------------------------------ */
/* Ground-truth dynamic-object mask, one byte per pixel, non-zero where a
 * moving object is. Set by the replay harness before each frame and consumed
 * in process_vo_pipeline() to drop keypoints that land on moving objects.
 *
 * This is the ORACLE arm of the dynamic-rejection study: it answers "how much
 * would perfect detection be worth" without a detector in the loop. NULL when
 * masking is off, which is the default. See docs/26.
 */
extern const unsigned char* levio_dynamic_mask;

#endif /* __LEVIO_HOOKS_H__ */
