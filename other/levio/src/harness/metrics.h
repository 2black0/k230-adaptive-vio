/*
 * k230-levio — structured run artifacts.
 *
 * Emits, into a directory given by --log-dir:
 *
 *   trajectory.tum   estimated trajectory, TUM format
 *   framelog.csv     per-frame metrics: fps, latency, features, pose, stage times
 *   timing.csv       per-frame timing subset (same columns as the EqVIO runner)
 *   resource.csv     CPU%, memory%, RSS MB, thread count
 *   realtime.csv     paced scheduled arrival, processing start, and finish
 *   summary.csv      run-level aggregates (fps, latency percentiles, RSS peak)
 *   power.csv        header only; filled in by hand from a USB-C meter
 *
 * resource.csv and the first four columns of timing.csv deliberately reuse the
 * column names k230-eqvio already writes (scripts/board_run.sh, run-euroc.sh)
 * so one parser reads both projects.
 *
 * Everything here is a no-op unless metrics_open() was called with a writable
 * directory, so runs without --log-dir behave exactly as before.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __METRICS_H__
#define __METRICS_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Creates @p log_dir if needed and opens every artifact, writing headers.
 * @return 0 on success, negative if the directory could not be created.
 */
int metrics_open(const char* log_dir);

/** @brief Non-zero once metrics_open() has succeeded. */
int metrics_enabled(void);

/**
 * @brief Marks the start of one pipeline iteration.
 *
 * Called before the source is read, so the measured latency covers the whole
 * iteration including frame I/O.
 *
 * @param frame_idx Index of the frame within this run (0-based).
 */
void metrics_frame_begin(uint32_t frame_idx);

/**
 * @brief Marks input availability for a paced replay frame.
 *
 * Establishes the common replay epoch on the first frame and records its
 * dataset timestamp as the zero point for subsequent scheduled arrivals.
 */
void metrics_frame_source_ready(double t_dataset, double replay_epoch_s);

/**
 * @brief Marks the end of one pipeline iteration and writes its CSV rows.
 *
 * Readings are taken from the per-frame trace struct and the trajectory
 * writer, so the pose in framelog.csv is byte-for-byte the pose in
 * trajectory.tum. Must be called before levio_trace_flush() clears the trace.
 *
 * @param t_dataset Frame timestamp on the dataset clock, in seconds.
 * @param n_kfs     Keyframes in the pose graph right now (graph_stats).
 * @param n_lms     World landmarks in the pose graph right now (graph_stats).
 */
void metrics_frame_end(double t_dataset, uint32_t n_kfs, uint32_t n_lms);

/**
 * @brief Samples /proc/self; cheap enough to call per frame, but the caller
 *        should throttle to a few Hz. Emits one resource.csv row per call.
 */
void metrics_sample_resources(void);

/**
 * @brief Records the duration of one pipeline stage for the current frame.
 *        Called by levio_hook_stage(); names are matched against the fixed
 *        stage list, unknown names are ignored.
 */
void metrics_add_stage_ns(const char* name, uint32_t ns);

/**
 * @brief Writes summary.csv and prints a short run summary to stdout.
 *
 * @param sequence        Sequence name, recorded verbatim.
 * @param mode            "fast" or "paced".
 * @param frames          Frames processed.
 * @param frames_dropped  Frames dropped by the source.
 * @param keyframes       Keyframes accepted by the pipeline.
 * @param wall_time_s     Total wall-clock run time, in seconds.
 * @param alloc_peak_bytes Peak LEVIO scratch allocation (not process RSS).
 */
void metrics_summary(const char* sequence, const char* mode,
                     uint32_t frames, uint32_t frames_dropped,
                     uint32_t keyframes, double wall_time_s,
                     size_t alloc_peak_bytes);

/** @brief Flushes and closes every open artifact. */
void metrics_close(void);

#ifdef __cplusplus
}
#endif

#endif /* __METRICS_H__ */
