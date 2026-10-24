/*
 * K230-VIO — trajectory writer, and the implementation of levio_hook_pose.
 * SPDX-License-Identifier: MIT
 */

#include "traj_writer.h"
#include "levio_hooks.h"

#include <stdio.h>
#include <math.h>

static FILE*    g_file      = NULL;
static double   g_time      = 0.0;
static uint32_t g_count     = 0;
static uint32_t g_keyframes = 0;

/* Last pose emitted for the frame currently being processed, in world frame.
   Cleared by traj_writer_set_time() at the start of every frame, so the
   metrics module can tell "no pose this frame" from "same pose as before". */
static double   g_last_t[3];
static double   g_last_q[4];
static int      g_has_last  = 0;

int traj_writer_open(const char* path)
{
    if (path == NULL) {
        return 0;                       /* writing disabled */
    }
    g_file = fopen(path, "w");
    if (g_file == NULL) {
        fprintf(stderr, "[traj] cannot create '%s'\n", path);
        return -1;
    }
    fprintf(g_file, "# K230-VIO trajectory (TUM format)\n");
    fprintf(g_file, "# timestamp tx ty tz qx qy qz qw\n");
    g_count     = 0;
    g_keyframes = 0;
    return 0;
}

void traj_writer_set_time(double t)
{
    g_time     = t;
    g_has_last = 0;
}

int traj_writer_last_pose(double t_out[3], double q_out[4])
{
    if (!g_has_last) {
        return 0;
    }
    for (int i = 0; i < 3; ++i) { t_out[i] = g_last_t[i]; }
    for (int i = 0; i < 4; ++i) { q_out[i] = g_last_q[i]; }
    return 1;
}

uint32_t traj_writer_count(void)           { return g_count; }
uint32_t traj_writer_keyframe_count(void)  { return g_keyframes; }

void traj_writer_close(void)
{
    if (g_file != NULL) {
        fclose(g_file);
        g_file = NULL;
    }
}

/*
 * Converts a rotation matrix to a unit quaternion (x, y, z, w).
 * Shepperd's method: pick the branch with the largest denominator so the
 * division stays numerically well conditioned for every rotation.
 */
static void rot_to_quat(const float R[9], double q[4])
{
    const double m00 = R[0], m01 = R[1], m02 = R[2];
    const double m10 = R[3], m11 = R[4], m12 = R[5];
    const double m20 = R[6], m21 = R[7], m22 = R[8];
    const double trace = m00 + m11 + m22;
    double s;

    if (trace > 0.0) {
        s = sqrt(trace + 1.0) * 2.0;
        q[3] = 0.25 * s;
        q[0] = (m21 - m12) / s;
        q[1] = (m02 - m20) / s;
        q[2] = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        s = sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q[3] = (m21 - m12) / s;
        q[0] = 0.25 * s;
        q[1] = (m01 + m10) / s;
        q[2] = (m02 + m20) / s;
    } else if (m11 > m22) {
        s = sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q[3] = (m02 - m20) / s;
        q[0] = (m01 + m10) / s;
        q[1] = 0.25 * s;
        q[2] = (m12 + m21) / s;
    } else {
        s = sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q[3] = (m10 - m01) / s;
        q[0] = (m02 + m20) / s;
        q[1] = (m12 + m21) / s;
        q[2] = 0.25 * s;
    }

    /* Renormalise: the input R comes from float32 arithmetic and is only
       approximately orthonormal after many pose-graph updates. */
    {
        double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (n > 1e-12) {
            q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
        } else {
            q[0] = q[1] = q[2] = 0.0; q[3] = 1.0;
        }
    }
}

void levio_hook_pose(const float* T_cw_16, int kind, uint32_t frame_number)
{
    float  R_wc[9];
    double t_wc[3];
    double q[4];

    (void) frame_number;

    ++g_count;
    if (kind != LEVIO_POSE_REGULAR) {
        ++g_keyframes;
    }

    if (T_cw_16 == NULL) {
        /* Bootstrap keyframe: the world frame is defined to coincide with
           the first camera frame, so the camera pose is the identity. */
        g_last_t[0] = g_last_t[1] = g_last_t[2] = 0.0;
        g_last_q[0] = g_last_q[1] = g_last_q[2] = 0.0;
        g_last_q[3] = 1.0;
        g_has_last  = 1;
        if (g_file != NULL) {
            fprintf(g_file, "%.9f 0.000000000 0.000000000 0.000000000 0 0 0 1\n", g_time);
        }
        return;
    }

    /*
     * LEVIO stores a row-major world-to-camera transform:
     *     T_cw = [ R  t ]
     *            [ 0  1 ]
     * TUM expects the camera pose in the world frame, so invert:
     *     R_wc = R^T,  t_wc = -R^T * t
     * (This matches visualize_log.py, which inverts the logged matrix.)
     */
    R_wc[0] = T_cw_16[0]; R_wc[1] = T_cw_16[4]; R_wc[2] = T_cw_16[8];
    R_wc[3] = T_cw_16[1]; R_wc[4] = T_cw_16[5]; R_wc[5] = T_cw_16[9];
    R_wc[6] = T_cw_16[2]; R_wc[7] = T_cw_16[6]; R_wc[8] = T_cw_16[10];

    {
        const double tx = T_cw_16[3];
        const double ty = T_cw_16[7];
        const double tz = T_cw_16[11];
        t_wc[0] = -((double) R_wc[0] * tx + (double) R_wc[1] * ty + (double) R_wc[2] * tz);
        t_wc[1] = -((double) R_wc[3] * tx + (double) R_wc[4] * ty + (double) R_wc[5] * tz);
        t_wc[2] = -((double) R_wc[6] * tx + (double) R_wc[7] * ty + (double) R_wc[8] * tz);
    }

    /* A diverged optimizer can produce NaN; writing it would silently poison
       the evo evaluation, so drop the sample and say so. */
    if (!isfinite(t_wc[0]) || !isfinite(t_wc[1]) || !isfinite(t_wc[2])) {
        fprintf(stderr, "[traj] non-finite pose at t=%.6f, sample skipped\n", g_time);
        return;
    }

    rot_to_quat(R_wc, q);

    g_last_t[0] = t_wc[0];
    g_last_t[1] = t_wc[1];
    g_last_t[2] = t_wc[2];
    for (int i = 0; i < 4; ++i) { g_last_q[i] = q[i]; }
    g_has_last  = 1;

    if (g_file == NULL) {
        return;
    }

    fprintf(g_file, "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n",
            g_time, t_wc[0], t_wc[1], t_wc[2], q[0], q[1], q[2], q[3]);
}
