// K230-VIO — per-frame numerical trace. See trace.h.
//
// SPDX-License-Identifier: MIT

#include "trace.h"

#include <stdio.h>
#include <string.h>

levio_trace_row_t levio_trace;

#ifdef LEVIO_TRACE_CSV
static FILE* trace_fp = NULL;
#endif

void levio_trace_open(const char* path)
{
#ifdef LEVIO_TRACE_CSV
    trace_fp = fopen(path, "w");
    if (!trace_fp) { return; }
    fprintf(trace_fp,
        "frame,t_stamp,n_kpts,n_match_world,n_match_frame,n_dyn_dropped,"
        "epnp_used,epnp_err,ess_inliers,ess_iters,ess_rejected,"
        "R00,R01,R02,R10,R11,R12,R20,R21,R22,"
        "tdir_x,tdir_y,tdir_z,scale,parallax,"
        "gate_step,gate_max,gate_fired,zupt_fired,"
        "kf_dt,kfdp_x,kfdp_y,kfdp_z,kfdv_x,kfdv_y,kfdv_z,"
        "imu_dt,dv_x,dv_y,dv_z,dp_x,dp_y,dp_z,"
        "vopt_x,vopt_y,vopt_z,gw_x,gw_y,gw_z,is_keyframe,"
        "p00,p01,p02,p03,p10,p11,p12,p13,p20,p21,p22,p23\n");
#else
    (void) path;
#endif
    memset(&levio_trace, 0, sizeof(levio_trace));
}

#ifdef LEVIO_TRACE_CSV
static void trace_write_row(void)
{
    const levio_trace_row_t* r = &levio_trace;
    fprintf(trace_fp, "%u,%.6f,%u,%u,%u,%u,%u,%.6g,%u,%u,%u",
            r->frame, r->t_stamp, r->n_kpts, r->n_match_world, r->n_match_frame,
            r->n_dyn_dropped,
            r->epnp_used, r->epnp_err, r->ess_inliers, r->ess_iters, r->ess_rejected);
    for (int i = 0; i < 9;  i++) fprintf(trace_fp, ",%.7g", r->R_rel[i]);
    for (int i = 0; i < 3;  i++) fprintf(trace_fp, ",%.7g", r->t_dir[i]);
    fprintf(trace_fp, ",%.7g,%.7g,%.7g,%.7g,%u,%u",
            r->scale, r->parallax, r->gate_step, r->gate_max,
            r->gate_fired, r->zupt_fired);
    fprintf(trace_fp, ",%.7g", r->kf_dt);
    for (int i = 0; i < 3;  i++) fprintf(trace_fp, ",%.7g", r->kf_dp[i]);
    for (int i = 0; i < 3;  i++) fprintf(trace_fp, ",%.7g", r->kf_dv[i]);
    fprintf(trace_fp, ",%.7g", r->imu_dt);
    for (int i = 0; i < 3;  i++) fprintf(trace_fp, ",%.7g", r->dv[i]);
    for (int i = 0; i < 3;  i++) fprintf(trace_fp, ",%.7g", r->dp[i]);
    for (int i = 0; i < 3;  i++) fprintf(trace_fp, ",%.7g", r->v_opt[i]);
    for (int i = 0; i < 3;  i++) fprintf(trace_fp, ",%.7g", r->g_world[i]);
    fprintf(trace_fp, ",%u", r->is_keyframe);
    for (int i = 0; i < 12; i++) fprintf(trace_fp, ",%.7g", r->pose[i]);
    fputc('\n', trace_fp);
}
#endif

void levio_trace_flush(void)
{
#ifdef LEVIO_TRACE_CSV
    if (trace_fp) { trace_write_row(); }
#endif
    /* Always clear and advance. The harness metrics module reads this struct
       to fill framelog.csv, so a field the current frame never set would
       otherwise be reported with the previous frame's value. */
    uint32_t next = levio_trace.frame + 1;
    memset(&levio_trace, 0, sizeof(levio_trace));
    levio_trace.frame = next;
}

void levio_trace_close(void)
{
#ifdef LEVIO_TRACE_CSV
    if (trace_fp) { fclose(trace_fp); trace_fp = NULL; }
#endif
}
