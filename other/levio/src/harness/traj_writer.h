/*
 * K230-VIO — trajectory writer (TUM format).
 *
 * Emits "timestamp tx ty tz qx qy qz qw" so the result can be evaluated
 * directly with evo:
 *     evo_ape tum groundtruth.txt estimated.txt -a -s --plot
 *
 * The -s (Umeyama scale alignment) flag is mandatory for monocular runs.
 * See docs/09-evaluation-protocol.md section 2.1.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __TRAJ_WRITER_H__
#define __TRAJ_WRITER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opens the trajectory file. Passing NULL disables writing.
 * @return 0 on success, negative on error.
 */
int traj_writer_open(const char* path);

/**
 * @brief Sets the timestamp applied to subsequently emitted poses.
 *
 * The harness calls this before handing a frame to the pipeline, because
 * the LEVIO hook has no notion of dataset time.
 */
void traj_writer_set_time(double t);

/** @brief Number of poses written so far. */
uint32_t traj_writer_count(void);

/** @brief Number of keyframes written so far. */
uint32_t traj_writer_keyframe_count(void);

/** @brief Flushes and closes the file. Safe to call when never opened. */
void traj_writer_close(void);

/**
 * @brief Returns the pose most recently emitted by levio_hook_pose().
 *
 * In world frame, same convention as the TUM rows. Used by the metrics module
 * so the pose in framelog.csv is identical to the row in trajectory.tum
 * instead of being re-derived from the raw world-to-camera matrix.
 *
 * @param t_out Receives tx, ty, tz.
 * @param q_out Receives qx, qy, qz, qw.
 * @return 1 if a pose has been emitted since open, 0 otherwise.
 */
int traj_writer_last_pose(double t_out[3], double q_out[4]);

#ifdef __cplusplus
}
#endif

#endif /* __TRAJ_WRITER_H__ */
