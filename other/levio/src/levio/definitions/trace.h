// K230-VIO — per-frame numerical trace.
//
// LEVIO's own logging is prose interleaved with matrix dumps: fine for reading
// one frame, useless for asking "how does the recovered translation direction
// compare with ground truth over 1300 frames". This writes one CSV row per
// frame with every intermediate the visual and inertial paths produce, so the
// comparison against ground truth happens offline in a few lines of numpy
// instead of by regexing a 400 MB log.
//
// The struct and the TRACE_SET/TRACE_VEC macros are always present: the
// harness metrics module reads them to fill framelog.csv. Only the writing of
// <out>.trace.csv is gated behind LEVIO_TRACE_CSV, which costs memory and I/O
// nobody needs for a benchmark run.
//
// SPDX-License-Identifier: MIT

#ifndef __LEVIO_TRACE_H__
#define __LEVIO_TRACE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t frame;
    double   t_stamp;

    /* ---- visual front end ---- */
    uint16_t n_kpts;
    uint16_t n_match_world;      /* two-way matches against the world map   */
    uint16_t n_match_frame;      /* two-way matches against the prev frame  */
    uint16_t n_dyn_dropped;      /* keypoints removed by the dynamic mask    */

    /* ---- EPnP branch ---- */
    uint8_t  epnp_used;
    float    epnp_err;

    /* ---- essential-matrix branch ---- */
    uint16_t ess_inliers;
    uint16_t ess_iters;
    uint8_t  ess_rejected;       /* 1 = ratio test, 2 = not in SO(3)        */
    float    R_rel[9];           /* rotation from the recovered relative pose */
    float    t_dir[3];           /* UNIT translation direction, pre-scaling  */

    /* ---- scale and gating ---- */
    float    scale;              /* metres applied to t_dir                  */
    float    parallax;
    float    gate_step;
    float    gate_max;
    uint8_t  gate_fired;
    uint8_t  zupt_fired;

    /* ---- inertial ---- */
    float    kf_dt;               /* preintegration interval at a keyframe    */
    float    kf_dp[3];            /* preintegrated position increment, frame i */
    float    kf_dv[3];            /* preintegrated velocity increment, frame i */
    float    imu_dt;
    float    dv[3];              /* preintegrated velocity increment, body   */
    float    dp[3];              /* preintegrated position increment, body   */
    float    v_opt[3];           /* optimiser velocity state, world          */
    float    g_world[3];

    /* ---- output ---- */
    uint8_t  is_keyframe;
    float    pose[12];           /* T_cw rows 0..2                           */
} levio_trace_row_t;

extern levio_trace_row_t levio_trace;

void levio_trace_open(const char* path);
void levio_trace_flush(void);      /* writes the current row (if enabled), then
                                      always clears it and advances the counter */
void levio_trace_close(void);

#define TRACE_SET(field, value)  do { levio_trace.field = (value); } while (0)
#define TRACE_VEC(field, a, b, c) do { levio_trace.field[0] = (a); \
                                       levio_trace.field[1] = (b); \
                                       levio_trace.field[2] = (c); } while (0)

#ifdef __cplusplus
}
#endif

#endif
