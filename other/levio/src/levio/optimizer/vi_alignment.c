// K230-VIO — visual-inertial alignment over the keyframe window. See the
// header for the derivation and for why LEVIO needs this at all.
//
// SPDX-License-Identifier: MIT

#include "vi_alignment.h"
#include "pose_graph_optimizer.h"
#include "definitions/matrix.h"
#include "definitions/logging.h"
#include "definitions/config.h"

#include <math.h>
#include <string.h>

/* Unknown vector layout, after eliminating the per-keyframe velocities:
 *
 *   x[0]      scale correction s
 *   x[1..3]   velocity of the OLDEST keyframe in the window, world frame
 *   x[4..6]   gravity, world frame
 *
 * Keeping all n velocities as free unknowns -- the obvious formulation, and the
 * one tried first -- makes the scale nearly unobservable: the position block
 *
 *     s*(cj - ci) = v_i*dt + 0.5*g*dt^2 + R_i*dp
 *
 * can be satisfied for ANY s by choosing v_i to absorb it, so s is pinned only
 * indirectly, through the velocity block. Measured consequence on VIODE 1_low:
 * the solve returned s = 0.010..0.051 where the true correction is 0.72..0.86.
 *
 * The velocity block is exact, so it can be substituted instead of merely
 * constraining:
 *
 *     v_k = v_0 + g*T_k + S_k ,   T_k = sum of dt up to k
 *                                 S_k = sum of R_m*dv_m up to k
 *
 * That leaves 7 unknowns against 3(n-1) equations -- 21 at MAX_KEYFRAMES = 8 --
 * and the scale is now carried by the accelerometer content of dp, which is
 * what actually observes it.
 */
#define VIA_NX  7

/* Camera centre of a row-major world-to-camera 4x4: c = -R^T t. */
static void via_centre(const float* T, float* c)
{
    for (int i = 0; i < 3; ++i) {
        c[i] = -(T[0*4+i]*T[3] + T[1*4+i]*T[7] + T[2*4+i]*T[11]);
    }
}

/* out = R_wc * v, where T is T_cw so R_wc is the transpose of its rotation. */
static void via_rot_to_world(const float* T, const float* v, float* out)
{
    for (int i = 0; i < 3; ++i) {
        out[i] = T[0*4+i]*v[0] + T[1*4+i]*v[1] + T[2*4+i]*v[2];
    }
}

uint8_t vi_align_window(const pose_t* poses,
                        const imu_factor_with_bias_t* imu_factors,
                        point3D_float_t* velocities,
                        float* g_world,
                        uint16_t num_poses,
                        uint16_t total_poses,
                        float* scale_out,
                        work_memory_t work_memory)
{
    const int n = (int) num_poses;
    if (n < VI_ALIGN_MIN_KEYFRAMES) { return 0; }

    const int nx = VIA_NX;
    float* Hb = allocate_work_memory(&work_memory, (size_t) nx * (nx + 1) * sizeof(float));
    float* x  = allocate_work_memory(&work_memory, (size_t) nx * sizeof(float));
    if (Hb == NULL || x == NULL) { return 0; }
    memset(Hb, 0, (size_t) nx * (nx + 1) * sizeof(float));

    const int oldest = (int) ((total_poses - num_poses) % MAX_KEYFRAMES);

    /* Running T_k and S_k for the velocity substitution. */
    float T_acc = 0.0f;
    float S_acc[3] = {0.0f, 0.0f, 0.0f};

    /* Cumulative form, against the oldest keyframe rather than the previous one.
     *
     * Summing the per-interval position equation from 0 to k gives
     *
     *     s*(c_k - c_0) = v_0*T_k + g*G_k + Q_k
     *     G_k = sum(T_m*dt_m + dt_m^2/2),  Q_k = sum(S_m*dt_m + dp_m)
     *
     * which uses the same information but over a long baseline. That matters
     * here: consecutive-keyframe displacements are dominated by the trajectory
     * jitter (60 % of steps reverse by more than 90 deg, docs/22), and noise in
     * a regressor biases its coefficient towards zero -- classic attenuation.
     * With consecutive differences the solve returned s ~ 0.10 against a true
     * 0.72..0.86. Direction error falls from 74.8 deg at one frame to 20.8 deg
     * at two seconds (docs/23), so the long baseline is where the signal is. */
    int rows = 0;
    float G_acc = 0.0f;
    float Q_acc[3] = {0.0f, 0.0f, 0.0f};
    float c0[3];
    via_centre((const float*) &poses[oldest], c0);

    for (int k = 0; k + 1 < n; ++k) {
        const int i  = (oldest + k)     % MAX_KEYFRAMES;
        const int jj = (oldest + k + 1) % MAX_KEYFRAMES;

        const imu_factor_t* fac = &imu_factors[jj].base;
        const float dt = fac->dt;
        if (!(dt > 1e-4f) || !(dt < VI_ALIGN_MAX_DT)) { return 0; }

        float dp_b[3] = { fac->dp.x, fac->dp.y, fac->dp.z };
        float dv_b[3] = { fac->dv.x, fac->dv.y, fac->dv.z };
        float dp_w[3], dv_w[3];
        via_rot_to_world((const float*) &poses[i], dp_b, dp_w);
        via_rot_to_world((const float*) &poses[i], dv_b, dv_w);

        G_acc += T_acc * dt + 0.5f * dt * dt;
        for (int d = 0; d < 3; ++d) { Q_acc[d] += S_acc[d] * dt + dp_w[d]; }
        T_acc += dt;
        for (int d = 0; d < 3; ++d) { S_acc[d] += dv_w[d]; }

        float cj[3];
        via_centre((const float*) &poses[jj], cj);

        /*  s*(c_k - c_0) - T_k*v_0 - G_k*g = Q_k  */
        for (int d = 0; d < 3; ++d) {
            float a[VIA_NX] = {0};
            a[0]     = cj[d] - c0[d];
            a[1 + d] = -T_acc;
            a[4 + d] = -G_acc;
            const float rhs = Q_acc[d];
            for (int q1 = 0; q1 < nx; ++q1) {
                if (a[q1] == 0.0f) { continue; }
                for (int q2 = 0; q2 < nx; ++q2) { Hb[q1*(nx+1)+q2] += a[q1] * a[q2]; }
                Hb[q1*(nx+1)+nx] += a[q1] * rhs;
            }
            rows++;
        }
    }

    if (rows < nx) { return 0; }

    /* Jacobi preconditioning, then damping -- in that order, and the order is
     * the whole point.
     *
     * The columns differ by three orders of magnitude: the scale column carries
     * |dc| ~ 2 m, the velocity columns dt ~ 0.13 s, the gravity columns
     * (T*dt + dt^2/2) ~ 0.008..0.13 s^2. On a synthetic problem with a known
     * answer, trace(H) came to 227.6 while the smallest eigenvalue was 9.9e-7 --
     * and that smallest direction is precisely the one carrying the scale.
     * A flat 1e-4 ridge, the value used at first, is a hundred times larger
     * than the eigenvalue it is supposed to regularise: it returned s = 0.00036
     * against a true 0.116. Only lambda <= 1e-10 recovered the answer, which is
     * indistinguishable from zero in float32 and would leave the solve
     * unprotected.
     *
     * Normalising each column to unit diagonal first makes the damping
     * dimensionless, so a small relative ridge regularises without swamping. */
    {
        float* d = allocate_work_memory(&work_memory, (size_t) nx * sizeof(float));
        if (d == NULL) { return 0; }
        for (int p = 0; p < nx; ++p) {
            float hp = Hb[p*(nx+1)+p];
            d[p] = (hp > 1e-20f) ? (1.0f / sqrtf(hp)) : 0.0f;
            if (d[p] == 0.0f) { return 0; }   /* an unobservable direction */
        }
        for (int p = 0; p < nx; ++p) {
            for (int q = 0; q < nx; ++q) { Hb[p*(nx+1)+q] *= d[p] * d[q]; }
            Hb[p*(nx+1)+nx] *= d[p];
            Hb[p*(nx+1)+p]  += VI_ALIGN_DAMPING;
        }
        if (!solve_dense_system(Hb, x, nx, work_memory)) { return 0; }
        for (int p = 0; p < nx; ++p) { x[p] *= d[p]; }
    }

    const float s = x[0];
    float gx = x[4], gy = x[5], gz = x[6];
    const float gn = sqrtf(gx*gx + gy*gy + gz*gz);

    /* Sanity. A solve that returns a negative or absurd scale, or a gravity
     * pointing nowhere near 9.81 m/s^2, has failed regardless of what the
     * residual says -- and silently accepting it would poison every later
     * frame, exactly the failure mode docs/20 section 6 documents for the
     * unchecked essential matrix. */
    if (!(s > VI_ALIGN_MIN_SCALE && s < VI_ALIGN_MAX_SCALE)) {
        LOG_INFO("VI align rejected: scale %f out of range\n", (double) s);
        return 0;
    }
    if (!(gn > 0.5f * GRAVITY_MAGNITUDE && gn < 2.0f * GRAVITY_MAGNITUDE)) {
        LOG_INFO("VI align rejected: |g| %f\n", (double) gn);
        return 0;
    }
    for (int p = 0; p < nx; ++p) {
        if (!(x[p] > -1e6f && x[p] < 1e6f)) {
            LOG_INFO("VI align rejected: non-finite solution\n");
            return 0;
        }
    }

    /* Gravity magnitude is known exactly; only its direction was in question. */
    const float gs = GRAVITY_MAGNITUDE / gn;
    gx *= gs; gy *= gs; gz *= gs;

    const float bg = VI_ALIGN_GRAVITY_GAIN;
    g_world[0] = (1.0f - bg) * g_world[0] + bg * gx;
    g_world[1] = (1.0f - bg) * g_world[1] + bg * gy;
    g_world[2] = (1.0f - bg) * g_world[2] + bg * gz;
    const float gr = sqrtf(g_world[0]*g_world[0] + g_world[1]*g_world[1] + g_world[2]*g_world[2]);
    if (gr > 1e-6f) {
        const float kk = GRAVITY_MAGNITUDE / gr;
        g_world[0] *= kk; g_world[1] *= kk; g_world[2] *= kk;
    }

    /* Rebuild every velocity in the window from the solved v0 and gravity,
     * using the same exact substitution the solve was built on. Blended rather
     * than replaced: the window solve is noisy per keyframe, and the velocity
     * sets the translation scale of every subsequent frame, so a jump there is
     * visible immediately. */
    const float bv = VI_ALIGN_VELOCITY_GAIN;
    T_acc = 0.0f;
    S_acc[0] = S_acc[1] = S_acc[2] = 0.0f;
    for (int k = 0; k < n; ++k) {
        const int idx = (oldest + k) % MAX_KEYFRAMES;
        const float gvec[3] = { gx, gy, gz };
        float vk[3];
        for (int d = 0; d < 3; ++d) {
            vk[d] = x[1 + d] + gvec[d] * T_acc + S_acc[d];
        }
        velocities[idx].x = (1.0f - bv) * velocities[idx].x + bv * vk[0];
        velocities[idx].y = (1.0f - bv) * velocities[idx].y + bv * vk[1];
        velocities[idx].z = (1.0f - bv) * velocities[idx].z + bv * vk[2];

        if (k + 1 < n) {
            const int jj = (oldest + k + 1) % MAX_KEYFRAMES;
            const int i  = (oldest + k)     % MAX_KEYFRAMES;
            const imu_factor_t* fac = &imu_factors[jj].base;
            float dv_b[3] = { fac->dv.x, fac->dv.y, fac->dv.z };
            float dv_w[3];
            via_rot_to_world((const float*) &poses[i], dv_b, dv_w);
            T_acc += fac->dt;
            for (int d = 0; d < 3; ++d) { S_acc[d] += dv_w[d]; }
        }
    }

    if (scale_out) { *scale_out = s; }
    LOG_INFO("VI align: s=%f |g|=%f rows=%d\n", (double) s, (double) gn, rows);
    return 1;
}
