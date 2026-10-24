/*
 * K230-VIO — sensor source abstraction.
 *
 * Everything downstream of this interface is unaware of where frames and IMU
 * samples come from. Phase 1-2 use the PGM replay sources; phase 4-5 will add
 * a live camera/IMU implementation without touching the pipeline.
 * See docs/05-dataset-replay-harness.md section 6.1.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __SOURCE_H__
#define __SOURCE_H__

#include <stdint.h>
#include <stddef.h>

#include "pmsis.h"
#include "definitions/config.h"
#include "definitions/vio_definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Return codes shared by all sources. */
#define SOURCE_OK        0
#define SOURCE_END       1   /* clean end of stream            */
#define SOURCE_ERROR   (-1)  /* unrecoverable                  */
#define SOURCE_DROPPED   2   /* frame skipped (fault injection)*/

typedef struct sensor_source sensor_source_t;

struct sensor_source {
    /** Monotonic scheduled-arrival epoch for paced replay; zero otherwise. */
    double replay_epoch_s;

    /**
     * Fetches the next camera frame.
     * @param img_out Buffer of at least IMG_SIZE bytes.
     * @param t_out   Frame timestamp in seconds (dataset clock).
     */
    int (*next_frame)(sensor_source_t* self, uint8_t* img_out, double* t_out);

    /**
     * Fetches the IMU batch belonging to the most recent frame.
     * @param out   Array of n imu_measurement_t.
     * @param n     Number of samples requested (IMU_HEIGHT).
     * @param n_out Number of samples actually written.
     */
    int (*next_imu_batch)(sensor_source_t* self, imu_measurement_t* out,
                          int n, int* n_out);

    void (*close)(sensor_source_t* self);

    /* Statistics, filled by the source, reported by the harness. */
    uint32_t frames_delivered;
    uint32_t frames_dropped;
    uint32_t imu_batches_delivered;
    uint32_t imu_short_batches;   /* fewer than IMU_HEIGHT samples */

    void* ctx;
};

/* ------------------------------------------------------------------ */
/* Configuration common to the replay sources                          */
/* ------------------------------------------------------------------ */
typedef struct {
    const char* data_dir;     /* directory holding images/ and imu/      */
    int         start_index;  /* first frame index                       */
    int         stop_index;   /* one past the last frame (0 = autodetect)*/

    /* Fault injection (Mode B), see docs/05 section 2. */
    double      jitter_ms;    /* uniform timestamp jitter, +/- ms        */
    double      drop_rate;    /* probability of dropping a frame [0..1]  */
    double      imu_drop_rate;/* probability of dropping an IMU sample   */
    uint32_t    seed;         /* seed for the fault-injection RNG        */
    double      speed;        /* Mode B playback rate multiplier         */
} replay_config_t;

/**
 * @brief Opens the deterministic replay source (Mode A).
 *
 * Reads as fast as possible, no pacing, fully reproducible. This is the
 * source used for the phase 2 numerical validation gate.
 */
int source_pgm_open(sensor_source_t* src, const replay_config_t* cfg);

/**
 * @brief Opens the real-time paced replay source (Mode B).
 *
 * Delivers frames at their dataset timestamp intervals (or CAMERA_FPS when
 * timestamps are unavailable) and IMU at IMU_SAMPLING_PERIOD, with optional
 * jitter and drop injection. Exercises the timing and buffering paths without
 * any sensor hardware attached.
 */
int source_paced_open(sensor_source_t* src, const replay_config_t* cfg);

/**
 * @brief Counts frame%06d.pgm files present in <data_dir>/images.
 * @return highest index + 1, or negative on error.
 */
int source_autodetect_stop_index(const char* data_dir);

/**
 * @brief Loads absolute per-frame timestamps from <data_dir>/timestamps.txt.
 *
 * Real datasets carry their own clock, and the estimated trajectory has to be
 * expressed on that clock for evo to associate it with the ground truth.
 * The EuRoC converter writes this file; the synthetic generator does not, in
 * which case the sources fall back to the nominal index/CAMERA_FPS timeline.
 *
 * @param out       Receives a malloc'd array of timestamps (caller frees).
 * @param out_count Receives the number of timestamps read.
 * @return SOURCE_OK if the file was read, SOURCE_END if it does not exist.
 */
int source_load_timestamps(const char* data_dir, double** out, int* out_count);

/**
 * @brief Absolute dataset time for a frame, or index/CAMERA_FPS if unavailable.
 */
double source_frame_time(const double* timestamps, int count, int index);

#ifdef __cplusplus
}
#endif

#endif /* __SOURCE_H__ */
