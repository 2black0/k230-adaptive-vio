// Copyright (c) 2026 ETH Zurich. All rights reserved.
// SPDX-FileCopyrightText: 2026 ETH Zurich
// SPDX-License-Identifier: MIT
// Author: Jonas Kühne

#include "pose_graph_optimizer.h"
#include "imu_optimization.h"
#include "imu_bias_optimization.h"
#include "vi_alignment.h"
#include "imu_preintegration.h"
#include "imu_calibration.h"
#include "levio_hooks.h"   /* K230-VIO HOOK: structured stage timing */

#include "math.h"

float K_data2[9] = {CAMERA_FX,     0.0, CAMERA_CX,
                        0.0, CAMERA_FY, CAMERA_CY,
                        0.0,       0.0,       1.0};

float g[3] = {GRAVITY_INIT_X, GRAVITY_INIT_Y, GRAVITY_INIT_Z};
float g_magnitude = GRAVITY_MAGNITUDE;
float g_timer = 1.0;

#define Min(a, b)       (((a)<(b))?(a):(b))
#define Max(a, b)       (((a)>(b))?(a):(b))

#define POSE_PARAMS_SIZE 9
#define LANDMARK_PARAMS_SIZE 3

typedef struct pose_graph_data
{
    pose_t* poses;
    observation_t* observations;
    point3D_float_t* landmarks;
    uint8_t* landmark_ages;
    point3D_float_t* velocities;
    imu_factor_with_bias_t* imu_factors;
    uint16_t num_landmarks;
    uint16_t num_observations;
    uint16_t num_poses;
    uint16_t total_poses;
    uint16_t newest_pose_offset;
    uint16_t anchor_pose_offset;
} pose_graph_data_t;

/**
 * @brief Applies a Gauss-Newton pose update delta to a T_cw transformation matrix.
 *
 * Extracts the rotation (as a Rodrigues vector) and translation from pose, subtracts
 * delta_p[0:3] from the rotation vector and delta_p[3:6] from the translation, then
 * re-orthonormalizes the rotation and recomposes into pose_new.
 *
 * @param delta_p  6-element update vector [delta_r(3), delta_t(3)].
 */
void apply_delta_to_pose(pose_t* pose, float* delta_p, pose_t* pose_new)
{
    float rot_data[9] = {0};
    matrix_2D_t T = {(float*) pose, {4,4,0,0}};
    matrix_2D_t R = {rot_data, {3,3,0,0}};
    float rot_vec[3];
    float t_vec[3];
    rotationMatrixOfTransformation(&T,&R);
    translationVectorOfTransformation(&T,t_vec);
    matrixToRodrigues(R.data,rot_vec);
#if BA_FREEZE_ROTATION
    /* Leave the rotation alone: it came from the gyro and bundle adjustment
     * cannot improve on it.
     *
     * Measured on VIODE 1_low, attitude error against ground truth after the
     * full run, with the gyro supplying every relative rotation:
     *
     *   relative rotations, accumulated by hand   1.59 deg
     *   pose graph with BA                       41.77 deg median
     *   pose graph with BA disabled               1.79 deg median
     *
     * So BA was contributing 40 degrees of attitude error on its own, on a
     * quantity already good to two. It still earns its place for the
     * translations and landmarks -- with BA off the path came out 88 m against
     * a true 157.7 m, with BA on 152.8 m -- so the fix is to freeze the
     * rotation block rather than to switch BA off. See docs/27. */
    (void) 0;
#else
    for(int j=0; j<3; ++j) 
    {
        rot_vec[j] -= (UPDATE_WEIGHT*delta_p[j]);
    }
#endif
    rodriguesToMatrix(rot_vec,R.data);
    float rot_data_norm[9] = {0};
    matrix_2D_t R_norm = {rot_data_norm, {3,3,0,0}};
    orthonormalize(&R,&R_norm);
    for(int j=0; j<3; ++j) 
    {
        t_vec[j] -= (UPDATE_WEIGHT*delta_p[3+j]);
    }
    matrix_2D_t T_new = {(float*) pose_new, {4,4,0,0}};
    composeTransformation(&R_norm, t_vec, &T_new);
}

/**
 * @brief Solves an n×n linear system in-place from an augmented n×(n+1) matrix [A|b].
 *
 * Applies Gaussian elimination with partial pivoting directly on Ab, then
 * back-substitutes to compute x. Ab is modified in place.
 *
 * @return 1 on success, 0 if the system is singular.
 */
int solve_dense_system(float* Ab, float* x, int n, work_memory_t work_memory) {
    // Forward elimination
    for (int i = 0; i < n; i++) {
        int pivot = i;
        for (int j = i + 1; j < n; j++) {
            if (fabsf(Ab[j * (n + 1) + i]) > fabsf(Ab[pivot * (n + 1) + i])) {
                pivot = j;
            }
        }
        if (pivot != i) { // Swap rows
            for (int k = 0; k < n + 1; k++) {
                float temp = Ab[i * (n + 1) + k];
                Ab[i * (n + 1) + k] = Ab[pivot * (n + 1) + k];
                Ab[pivot * (n + 1) + k] = temp;
            }
        }
        if (fabsf(Ab[i * (n + 1) + i]) < 1e-12f) { return 0; } // No unique solution
        for (int j = i + 1; j < n; j++) {
            float factor = Ab[j * (n + 1) + i] / Ab[i * (n + 1) + i];
            for (int k = i; k < n + 1; k++) {
                Ab[j * (n + 1) + k] -= factor * Ab[i * (n + 1) + k];
            }
        }
    }

    // Back substitution
    for (int i = n - 1; i >= 0; i--) {
        x[i] = Ab[i * (n + 1) + n];
        for (int j = i + 1; j < n; j++) {
            x[i] -= Ab[i * (n + 1) + j] * x[j];
        }
        x[i] /= Ab[i * (n + 1) + i];
    }

    return 1;
}

/**
 * @brief Computes the total reprojection error over all observations in the current optimization window.
 *
 * Observations outside the window (older than MAX_KEYFRAMES) are skipped.
 * Uses the squared reprojection error per observation.
 *
 * @return Sum of squared reprojection errors across all valid observations.
 */
float reprojection_error_obs(pose_t* poses, point3D_float_t* landmarks, observation_t* observations, matrix_2D_t* K, uint16_t num_observations, uint16_t total_poses)
{
    float error = 0;
    for (uint16_t i = 0; i < num_observations; ++i) {
        observation_t* obs = &observations[i];
        if(total_poses - obs->pose_id > MAX_KEYFRAMES)
        {
            /* Not in optimization window */
            continue;
        }
        int p_idx = (obs->pose_id % MAX_KEYFRAMES);
        int l_idx = obs->wp_id;
        matrix_2D_t pose = {(float*) &poses[(obs->pose_id % MAX_KEYFRAMES)],{4,4,0,0}};

        float err_sq = reprojection_error_squared(&landmarks[l_idx], &obs->kpt, &pose, K);
        float err_mag = sqrtf(err_sq);
        if (err_mag <= HUBER_DELTA) {
            error += err_sq;
        } else {
            error += 2.0f * HUBER_DELTA * err_mag - HUBER_DELTA * HUBER_DELTA;
        }
    }
    return error;
}

void update_gravity_gyro_bias(imu_calib_result_t*calib_result, const pose_t* pose_at_calib)
{
    // gravity_b is in camera/body frame; rotate to world frame via R_cw^T
    float R_data[9];
    matrix_2D_t T = {(float*)pose_at_calib, {4,4,0,0}};
    matrix_2D_t R = {R_data, {3,3,0,0}};
    rotationMatrixOfTransformation(&T, &R);
    // g_world = R_cw^T * (-gravity_b)
    float gb[3] = {-calib_result->gravity_b.x, -calib_result->gravity_b.y, -calib_result->gravity_b.z};
    g[0] = R_data[0]*gb[0] + R_data[3]*gb[1] + R_data[6]*gb[2];
    g[1] = R_data[1]*gb[0] + R_data[4]*gb[1] + R_data[7]*gb[2];
    g[2] = R_data[2]*gb[0] + R_data[5]*gb[1] + R_data[8]*gb[2];
    // Enforce known gravity magnitude
    float norm = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    if (norm > 1e-6f) {
        float scale = g_magnitude / norm;
        g[0] *= scale; g[1] *= scale; g[2] *= scale;
    }
    LOG_INFO("Gravity updated (world frame): [%f, %f, %f]\n", g[0], g[1], g[2]);
}

/**
 * @brief Computes the 2D reprojection error and Jacobians for one observation.
 *
 * Transforms the landmark into camera coordinates via pose, projects it with K,
 * and computes:
 *   J_pose     (2×6): Jacobian w.r.t. [r_cw(3), t_cw(3)] using the skew-symmetric
 *                     of the camera-frame point for the rotation block.
 *   J_landmark (2×3): Jacobian w.r.t. the 3D landmark position (= -J_proj * R).
 *   error      (2×1): Observed pixel - projected pixel.
 */
void compute_jacobian_and_error(matrix_2D_t* pose, point3D_float_t* landmark, matrix_2D_t* K, observation_t* obs,
                                float* J_pose, float* J_landmark, float* error) {
    // Transform landmark to camera coordinates
    float p_world[4];
    p_world[0] = landmark->x;
    p_world[1] = landmark->y;
    p_world[2] = landmark->z;
    p_world[3] = 1.0;
    float p_cam[4];
    matvec(pose,p_world,p_cam,4);

    // Project point
    float fu = K->data[0];
    float fv = K->data[4];
    float cu = K->data[2];
    float cv = K->data[5];
    float inv_z = 1.0f / p_cam[2];
    float inv_z2 = inv_z * inv_z;
    float proj[2] = { fu * p_cam[0] * inv_z + cu,
                      fv * p_cam[1] * inv_z + cv };

    // Compute error
    reprojection_error_vector(landmark, &obs->kpt, pose, K, error);

    // Jacobian of projection wrt camera coordinates (2x3 matrix)
    float J_proj[6] = {
        fu * inv_z, 0, -fu * p_cam[0] * inv_z2,
        0, fv * inv_z, -fv * p_cam[1] * inv_z2
    };
    float R[9];
    matrix_2D_t R_mat = {R, {3,3,0,0}};
    rotationMatrixOfTransformation(pose, &R_mat);

    // a) Jacobian w.r.t. Landmark Position (d(P_cam)/d(P_world) = R)
    // J_landmark = -J_proj * R
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j) {
            J_landmark[i*3 + j] = -(J_proj[i*3 + 0] * R[0*3 + j] +
                                    J_proj[i*3 + 1] * R[1*3 + j] +
                                    J_proj[i*3 + 2] * R[2*3 + j]);
        }
    }

    // b) Jacobian w.r.t. Camera Translation (d(P_cam)/d(t) = I)
    // J_pose_t = -J_proj * I = -J_proj
    J_pose[3] =  -J_proj[0]; // du/dtx
    J_pose[4] =  -J_proj[1]; // du/dty
    J_pose[5] =  -J_proj[2]; // du/dtz
    J_pose[9] =  -J_proj[3]; // dv/dtx
    J_pose[10] = -J_proj[4]; // dv/dty
    J_pose[11] = -J_proj[5]; // dv/dtz

    // c) Jacobian w.r.t. Camera Rotation (d(P_cam)/d(r) = -[P_rot]_x)
    // Create the skew-symmetric matrix of the rotated point P_rot
    mat3x3_t P_cam_skew;
    point3D_float_t p_cam_3d = {p_cam[0], p_cam[1], p_cam[2]};
    mat3x3_skew_symmetric(&p_cam_3d, &P_cam_skew);
    
    // J_pose_r = -J_proj * (-P_rot_skew) = J_proj * P_rot_skew
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j) {
            J_pose[i*6 + j] = (J_proj[i*3 + 0] * P_cam_skew.m[0*3 + j] +
                               J_proj[i*3 + 1] * P_cam_skew.m[1*3 + j] +
                               J_proj[i*3 + 2] * P_cam_skew.m[2*3 + j]);
        }
    }
}

typedef struct jacobi_data_landmark
{
    float H_ll_block[9];
    float J_p_i[MAX_KEYFRAMES*12];
    float J_l_i[MAX_KEYFRAMES*6];
    float e_i[MAX_KEYFRAMES*2];

}jacobi_data_landmark_t;

jacobi_data_landmark_t* l2_cache;

void optimizer_init_l2_cache()
{
    l2_cache = pi_l2_malloc(MAX_WORLD_FEATURES*sizeof(jacobi_data_landmark_t));
}

void l2_cache_move_in(jacobi_data_landmark_t* local, int l_idx, pi_cl_dma_cmd_t* cmd)
{
    jacobi_data_landmark_t* external = &l2_cache[l_idx];
    /* DMA Copy Data */
    pi_cl_dma_cmd((uintptr_t) external, (uintptr_t) local, sizeof(jacobi_data_landmark_t), PI_CL_DMA_DIR_EXT2LOC, cmd);
    /* Wait for DMA transfer to finish. */
}

void l2_cache_move_out(jacobi_data_landmark_t* local, int l_idx, pi_cl_dma_cmd_t* cmd)
{
    jacobi_data_landmark_t* external = &l2_cache[l_idx];
    /* DMA Copy Data */
    pi_cl_dma_cmd((uintptr_t) external, (uintptr_t) local, sizeof(jacobi_data_landmark_t), PI_CL_DMA_DIR_LOC2EXT, cmd);
    /* Wait for DMA transfer to finish. */
}

/**
 * @brief Builds the Schur-complement reduced camera system and solves for pose and landmark updates.
 *
 * Implements the landmark marginalization step of bundle adjustment:
 *   1. For each landmark, accumulates H_ll (3×3), H_pl (6×3), and J_l_e using all
 *      observations within the window. Applies Huber weighting.
 *   2. Marginalizes landmarks out of the system via the Schur complement:
 *        S -= H_pl * H_ll^-1 * H_lp
 *   3. Adds IMU constraints and kinematic velocity prior into the reduced system.
 *   4. Solves the damped (Levenberg-Marquardt) reduced system via Cholesky for delta_p.
 *   5. Back-substitutes to compute landmark updates delta_l via H_ll^-1 * (J_l_e - H_lp * delta_p).
 *
 * @param delta_p  Output pose update vector, length num_poses * POSE_PARAMS_SIZE.
 * @param delta_l  Output landmark update vector, length num_landmarks * 3.
 * @param lambda   Levenberg-Marquardt damping factor.
 * @return Total reprojection error before the update.
 */
float calculate_update_steps_schur(pose_graph_data_t* pg_data, matrix_2D_t* K,
                                   float* delta_p, float* delta_l,
                                   float lambda, work_memory_t work_memory)
{
    int pose_system_size = pg_data->num_poses * POSE_PARAMS_SIZE;
    int landmark_system_size = pg_data->num_landmarks * LANDMARK_PARAMS_SIZE;

    // Allocate memory for system components
    float* Sb = allocate_work_memory(&work_memory, pose_system_size * (pose_system_size + 1) * sizeof(float)); // Reduced camera system matrix & vector
    memset(Sb, 0, pose_system_size * (pose_system_size + 1) * sizeof(float));

    float* J_l_e = delta_l;    // Landmark error vectors (re-use memory)
    memset(J_l_e, 0, landmark_system_size * sizeof(float));

    jacobi_data_landmark_t* jacobi_data = allocate_work_memory(&work_memory, sizeof(jacobi_data_landmark_t));

    float* H_ll_block = jacobi_data->H_ll_block;
    float* J_p_i = jacobi_data->J_p_i;
    float* J_l_i = jacobi_data->J_l_i;
    float* e_i = jacobi_data->e_i;
    float* H_ll_inv = allocate_work_memory(&work_memory, 9 * sizeof(float));
    float* Hpl_Hll_inv = allocate_work_memory(&work_memory, 18 * sizeof(float)); // 6x3 matrix
    float* S_update = allocate_work_memory(&work_memory, 36 * sizeof(float)); // 6x6

    float current_error = 0.0;
    int current_obs_idx = 0;
    for (int j = 0; j < pg_data->num_landmarks; ++j) {
        memset(H_ll_block, 0, 9 * sizeof(float));
        memset(J_p_i, 0, MAX_KEYFRAMES * 12 * sizeof(float));
        memset(J_l_i, 0, MAX_KEYFRAMES * 6 * sizeof(float));
        memset(e_i, 0, MAX_KEYFRAMES * 2 * sizeof(float));
        int obs_start_idx = current_obs_idx;
        while (current_obs_idx < pg_data->num_observations && pg_data->observations[current_obs_idx].wp_id == j) {
            current_obs_idx++;
        }
        int obs_end_idx = current_obs_idx;
        int num_obs_for_j = obs_end_idx - obs_start_idx;
        if (num_obs_for_j == 0) continue;

        int l_idx = j;

        for (int i = obs_start_idx; i < obs_start_idx+num_obs_for_j; ++i) {
            observation_t* obs = &pg_data->observations[i];
            if(pg_data->total_poses - obs->pose_id > MAX_KEYFRAMES)
            {
                /* Not in optimization window */
                continue;
            }
            int p_idx = (obs->pose_id % MAX_KEYFRAMES);

            float* J_p = &J_p_i[p_idx*12];
            float* J_l = &J_l_i[p_idx*6];
            float* e = &e_i[p_idx*2];

            matrix_2D_t pose = {(float*) &pg_data->poses[(obs->pose_id % MAX_KEYFRAMES)],{4,4,0,0}};
            compute_jacobian_and_error(&pose, &pg_data->landmarks[obs->wp_id], K, obs, J_p, J_l, e);

            float error_sq = e[0]*e[0] + e[1]*e[1];
            float error_magnitude = sqrtf(error_sq);

            // Huber cost: rho(s) = s if s <= delta^2, else 2*delta*sqrt(s) - delta^2
            if (error_magnitude <= HUBER_DELTA) {
                current_error += error_sq;
            } else {
                current_error += 2.0f * HUBER_DELTA * error_magnitude - HUBER_DELTA * HUBER_DELTA;
            }

            float w = 1.0f;
            if (error_magnitude > HUBER_DELTA) {
                w = HUBER_DELTA / error_magnitude;
            }
            float sqrt_w = sqrtf(w);
            // Scale both residuals and Jacobians by sqrt_w so that the normal equations
            // H = (√w·J)^T(√w·J) and b = (√w·J)^T(√w·e) are consistent (weighted least squares).
            e[0] *= sqrt_w;
            e[1] *= sqrt_w;
            for (int k = 0; k < 6; ++k) {
                J_p[k]   *= sqrt_w;
                J_p[6+k] *= sqrt_w;
            }
            for (int k = 0; k < 6; ++k) J_l[k] *= sqrt_w;

            // H_ll_j += J_l^T * J_l
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    H_ll_block[r*3 + c] += J_l[r]*J_l[c] + J_l[3+r]*J_l[3+c];
                }
            }
            // J_l_e_j += J_l^T * e
            for (int r = 0; r < 3; ++r) {
                J_l_e[l_idx*3 + r] += J_l[r]*e[0] + J_l[3+r]*e[1];
            }
        }

        // Invert H_ll_j block
        pi_cl_dma_cmd_t cmd;
        l2_cache_move_out(jacobi_data, l_idx, &cmd);
        // Wait before modifying the buffer: the DMA is still reading H_ll_block (and
        // the rest of jacobi_data) asynchronously. Modifying it before the transfer
        // completes is a race condition that can corrupt the L2 cache copy, which the
        // back-substitution pass later reloads to add lambda and invert a second time.
        pi_cl_dma_wait(&cmd);
        matrix_2D_t H_ll_mat = {H_ll_block, {3,3,0,0}};
        matrix_2D_t H_ll_inv_mat = {H_ll_inv, {3,3,0,0}};
        for(int k=0; k<3; ++k) H_ll_block[k*3+k] += lambda; // Add damping to H_ll
        if (!matinv3x3(&H_ll_mat, &H_ll_inv_mat)){
            continue;
        }

        // Collect valid observation indices and their pose indices for this landmark
        int valid_obs[MAX_KEYFRAMES];
        int valid_pidx[MAX_KEYFRAMES];
        int n_valid = 0;
        for (int i = obs_start_idx; i < obs_start_idx+num_obs_for_j; ++i) {
            observation_t* obs = &pg_data->observations[i];
            if(pg_data->total_poses - obs->pose_id > MAX_KEYFRAMES) continue;
            valid_obs[n_valid] = i;
            valid_pidx[n_valid] = obs->pose_id % MAX_KEYFRAMES;
            n_valid++;
        }

        // For each pair of observations (a, b) of this landmark, accumulate
        // the full Schur complement including cross-pose terms.
        for (int a = 0; a < n_valid; ++a) {
            int p_a = valid_pidx[a];
            float* J_p_a = &J_p_i[p_a*12];
            float* J_l_a = &J_l_i[p_a*6];
            float* e_a = &e_i[p_a*2];
            int S_a = p_a * POSE_PARAMS_SIZE;

            // Compute H_pl_a * H_ll^-1 for observation a
            for(int r=0; r<6; ++r){
                for(int c=0; c<3; ++c){
                    Hpl_Hll_inv[r*3+c] = 0;
                    for(int k=0; k<3; ++k) {
                        float H_pl_rk = J_p_a[r]*J_l_a[k] + J_p_a[6+r]*J_l_a[3+k];
                        Hpl_Hll_inv[r*3+c] += H_pl_rk * H_ll_inv[k*3+c];
                    }
                }
            }

            for (int b = 0; b < n_valid; ++b) {
                int p_b = valid_pidx[b];
                float* J_p_b = &J_p_i[p_b*12];
                float* J_l_b = &J_l_i[p_b*6];
                int S_b = p_b * POSE_PARAMS_SIZE;

                // S[a,b] -= H_pl_a * H_ll^-1 * H_lp_b
                for(int r=0; r<6; ++r){
                    for(int c=0; c<6; ++c){
                        float val = 0;
                        for(int k=0; k<3; ++k){
                            float H_lp_kc = J_l_b[k]*J_p_b[c] + J_l_b[3+k]*J_p_b[6+c];
                            val += Hpl_Hll_inv[r*3+k] * H_lp_kc;
                        }
                        Sb[(S_a+r)*(pose_system_size+1) + S_b+c] -= val;
                    }
                }
            }

            // H_pp_aa += J_p_a^T * J_p_a (diagonal contribution from this observation)
            for(int r=0; r<6; ++r){
                for(int c=0; c<6; ++c){
                    Sb[(S_a+r)*(pose_system_size+1) + S_a+c] += J_p_a[r]*J_p_a[c] + J_p_a[6+r]*J_p_a[6+c];
                }
            }

            // b_a += J_p_a^T * e_a - H_pl_a * H_ll^-1 * J_l_e
            for(int r=0; r<6; ++r){
                float b_update = 0;
                for(int k=0; k<3; ++k) b_update += Hpl_Hll_inv[r*3+k] * J_l_e[l_idx*3+k];
                Sb[(S_a+r)*(pose_system_size+1)+pose_system_size] += J_p_a[r]*e_a[0] + J_p_a[6+r]*e_a[1] - b_update;
            }
        }
    }

    // Add IMU contraints
    current_error += add_imu_factors(pg_data->poses, pg_data->num_poses,
                                     pg_data->total_poses,
                                     pg_data->velocities,
                                     pg_data->imu_factors,
                                     Sb,
                                     g,
                                     work_memory);
    add_kinematic_velocity_prior(Sb, 
                                 pg_data->poses,
                                 pg_data->velocities,
                                 pg_data->imu_factors,
                                 pg_data->num_poses,
                                 pg_data->total_poses, (1.0f/ACCEL_NOISE_DENSITY), work_memory);

    // Add damping to the pose part
    for (int i = 0; i < pose_system_size; ++i) Sb[i*(pose_system_size+1) + i] += lambda;

    uint8_t offset_of_anchor = pg_data->anchor_pose_offset;
    int anchor_base = offset_of_anchor * POSE_PARAMS_SIZE;
    float anchor_weight = 1e6f;  // Constraint weight
    // Anchor only R (3 DOF) and p (3 DOF), NOT v (3 DOF)
    for (int i = 0; i < 6; ++i) {
        Sb[(anchor_base + i)*(pose_system_size+1) + (anchor_base + i)] += anchor_weight;
    }


    // Solve for pose updates
    if (!solve_cholesky_system(Sb, delta_p, pose_system_size, work_memory)) {
        return current_error;
    }


    // Back-substitute for landmark updates
    int obs_counter = 0;
    for (int j = 0; j < pg_data->num_landmarks; ++j) {
        pi_cl_dma_cmd_t cmd;
        l2_cache_move_in(jacobi_data, j, &cmd);
        pi_cl_dma_wait(&cmd);

        matrix_2D_t H_ll_mat = {H_ll_block, {3,3,0,0}};
        matrix_2D_t H_ll_inv_mat = {H_ll_inv, {3,3,0,0}};
        for(int k=0; k<3; ++k) H_ll_block[k*3+k] += lambda;
        if (!matinv3x3(&H_ll_mat, &H_ll_inv_mat)) continue;

        float H_lp_delta_p[3] = {0,0,0};
        while(obs_counter < pg_data->num_observations){
            if (pg_data->observations[obs_counter].wp_id != j)
            {
                break;
            }
            if(pg_data->total_poses - pg_data->observations[obs_counter].pose_id > MAX_KEYFRAMES)
            {
                /* Not in optimization window */
                ++obs_counter;
                continue;
            }
            int p_idx = (pg_data->observations[obs_counter].pose_id % MAX_KEYFRAMES);
            float* J_p = &J_p_i[p_idx*12];
            float* J_l = &J_l_i[p_idx*6];
            float* e = &e_i[p_idx*2];
            for(int r=0; r<3; ++r){
                for(int c=0; c<6; ++c){
                    float H_lp_rc = J_l[r]*J_p[c] + J_l[3+r]*J_p[6+c];
                    H_lp_delta_p[r] += H_lp_rc * delta_p[p_idx*POSE_PARAMS_SIZE+c];
                }
            }
            ++obs_counter;
        }

        float rhs[3] = {J_l_e[j*3+0] - H_lp_delta_p[0],
                        J_l_e[j*3+1] - H_lp_delta_p[1],
                        J_l_e[j*3+2] - H_lp_delta_p[2]};

        for(int r=0; r<3; ++r){
            delta_l[j*3+r] = 0;
            for(int c=0; c<3; ++c){
                delta_l[j*3+r] += H_ll_inv[r*3+c] * rhs[c];
            }
        }
    }
    return current_error;
}

uint evaluate_step_and_update_parameters(pose_graph_data_t* pg_data, matrix_2D_t* K,
                                         float* delta_p, float* delta_l,
                                         float current_error, work_memory_t work_memory)
{
    pose_t* poses_new = allocate_work_memory(&work_memory, MAX_KEYFRAMES*sizeof(pose_t));
    point3D_float_t* landmarks_new = allocate_work_memory(&work_memory, (MAX_WORLD_FEATURES*sizeof(point3D_float_t)));
    point3D_float_t* velocities_new = allocate_work_memory(&work_memory, MAX_KEYFRAMES*sizeof(point3D_float_t));

    for(int i=0; i<pg_data->num_poses; ++i) {
        apply_delta_to_pose(&pg_data->poses[i], &delta_p[i*STATE_SIZE],&poses_new[i]);
        LOG_DEBUG("Old velocity: [%.4f, %.4f, %.4f], delta: [%.4f, %.4f, %.4f]\n", 
                 pg_data->velocities[i].x, pg_data->velocities[i].y, pg_data->velocities[i].z,
                 delta_p[i*STATE_SIZE+6], delta_p[i*STATE_SIZE+7], delta_p[i*STATE_SIZE+8]);
        velocities_new[i].x = pg_data->velocities[i].x - (UPDATE_WEIGHT*delta_p[i*STATE_SIZE+6]);
        velocities_new[i].y = pg_data->velocities[i].y - (UPDATE_WEIGHT*delta_p[i*STATE_SIZE+7]);
        velocities_new[i].z = pg_data->velocities[i].z - (UPDATE_WEIGHT*delta_p[i*STATE_SIZE+8]);
    }
    for(int i=0; i<pg_data->num_landmarks; ++i) {
        if(pg_data->landmark_ages[i] < MAX_KEYFRAMES)
        {
            landmarks_new[i].x = pg_data->landmarks[i].x - (UPDATE_WEIGHT*delta_l[i*3+0]);
            landmarks_new[i].y = pg_data->landmarks[i].y - (UPDATE_WEIGHT*delta_l[i*3+1]);
            landmarks_new[i].z = pg_data->landmarks[i].z - (UPDATE_WEIGHT*delta_l[i*3+2]);
        }
        else
        {
            landmarks_new[i].x = pg_data->landmarks[i].x;
            landmarks_new[i].y = pg_data->landmarks[i].y;
            landmarks_new[i].z = pg_data->landmarks[i].z;
        }
    }
    float new_error = reprojection_error_obs(poses_new,landmarks_new,pg_data->observations,K,pg_data->num_observations,pg_data->total_poses);
    new_error += add_imu_factors(poses_new, pg_data->num_poses, pg_data->total_poses, velocities_new, pg_data->imu_factors, NULL, g, work_memory);
    if (new_error < current_error)
    {
        LOG_INFO("Optimizer Success: New error %f (was %f)!\n", new_error, current_error);
        memcpy(pg_data->poses,poses_new,pg_data->num_poses*sizeof(pose_t));
        memcpy(pg_data->velocities,velocities_new,(pg_data->num_poses*sizeof(point3D_float_t)));
        memcpy(pg_data->landmarks,landmarks_new,(pg_data->num_landmarks*sizeof(point3D_float_t)));
        return 1;
    }
    return 0;
}

void print_vec3n(const char* name, const point3D_float_t* v) {
    printf("%s: [%.4f, %.4f, %.4f]\n", name, v->x, v->y, v->z);
}

// Bundle Adjustment with Schur Complement
void bundle_adjustment_schur(pose_graph_data_t* pg_data,
                             matrix_2D_t* K,
                             int max_iter, float lambda_init,
                             work_memory_t work_memory)
{
    int pose_system_size = pg_data->num_poses * POSE_PARAMS_SIZE;
    int landmark_system_size = pg_data->num_landmarks * LANDMARK_PARAMS_SIZE;

    float* delta_p = allocate_work_memory(&work_memory, pose_system_size * sizeof(float));
    float* delta_l = allocate_work_memory(&work_memory, landmark_system_size * sizeof(float));
    memset(delta_p, 0, pose_system_size * sizeof(float));
    memset(delta_l, 0, landmark_system_size * sizeof(float));

    float biases[MAX_KEYFRAMES][BIAS_SIZE];

    float lambda = lambda_init;
    float current_error = 0;

    imu_factor_with_bias_t* factor_new = &pg_data->imu_factors[pg_data->newest_pose_offset];
    pose_t* prev_pose = &pg_data->poses[(MAX_KEYFRAMES + pg_data->newest_pose_offset - 1) % MAX_KEYFRAMES];

    if (factor_new->base.dt > 5.0f){
        LOG_INFO("Newest IMU factor is old (dt=%f), assuming standstill\n", factor_new->base.dt);
        point3D_float_t zero_vel = {0.0,0.0,0.0};
        imu_calib_result_t calib_result;
        imu_calib_stationary(&factor_new->base, 0.5f, 0.5f, &calib_result);
        print_vec3n("Calibrated gravity", &calib_result.gravity_b);
        print_vec3n("Calibrated accel bias", &calib_result.gyro_bias);
        update_gravity_gyro_bias(&calib_result, prev_pose);
        imu_make_zero_motion_factor(factor_new->base.dt, calib_result.gyro_bias, calib_result.gravity_b, factor_new);
        memcpy(&pg_data->poses[pg_data->newest_pose_offset], prev_pose, sizeof(pose_t));
    }

    for (int iter = 0; iter < max_iter; ++iter) {
        current_error = calculate_update_steps_schur(pg_data, K,
                                                     delta_p, delta_l,
                                                     lambda, work_memory);
        
        // Update parameters (with simple LM strategy)
        // Check if the error decreases and reject the step if not.
        uint success = evaluate_step_and_update_parameters(pg_data, K, 
                                                           delta_p, delta_l,
                                                           current_error, work_memory);


        memset(biases, 0, MAX_KEYFRAMES * BIAS_SIZE * sizeof(float));
        optimize_biases_sequentially(
            biases,
            pg_data->poses,
            pg_data->velocities,
            pg_data->imu_factors, 
            pg_data->num_poses,
            pg_data->total_poses,
            g,
            work_memory
        );
        LOG_DEBUG("Estimated biases\n");
        for (int i = 0; i < pg_data->num_poses; ++i) {
            LOG_DEBUG("Pose %d ",i);
            vecprint(biases[i], BIAS_SIZE, DEBUG_LEVEL);
        }

#if APPLY_ESTIMATED_IMU_BIAS
        /* Upstream estimates the biases here and then only *prints* them: the
         * result never reaches the preintegrator, so the accelerometer bias is
         * re-integrated in full on every subsequent frame. That is the second
         * half of the unbounded velocity ramp documented in
         * initilaize_newest_velocity() -- the first half being attitude error,
         * which ROTATION_FROM_GYRO removes.
         *
         * Applied with a gain rather than outright, because the estimate is
         * noisy per keyframe and the preintegrator is the one thing that must
         * not oscillate, and clamped because a diverged solve here would poison
         * every later frame. Bias layout is [0..2] gyro, [3..5] accel, both in
         * the camera frame -- the same frame preintegration_integrate()
         * subtracts them in. */
        {
            uint8_t nb = pg_data->newest_pose_offset;
            ImuBias cur = get_bias();
            float upd[BIAS_SIZE];
            uint8_t sane = 1;
            for (int i = 0; i < BIAS_SIZE; ++i) {
                upd[i] = biases[nb][i];
                if (!(upd[i] > -IMU_BIAS_CLAMP && upd[i] < IMU_BIAS_CLAMP)) { sane = 0; }
            }
            if (sane) {
                const float k = IMU_BIAS_GAIN;
                cur.gyro.x += k * upd[0]; cur.gyro.y += k * upd[1]; cur.gyro.z += k * upd[2];
                cur.acc.x  += k * upd[3]; cur.acc.y  += k * upd[4]; cur.acc.z  += k * upd[5];
                set_bias(cur);
                LOG_INFO("Bias applied: gyro [%f %f %f] acc [%f %f %f]\n",
                         cur.gyro.x, cur.gyro.y, cur.gyro.z,
                         cur.acc.x,  cur.acc.y,  cur.acc.z);
            }
        }
#endif
        
        if (success)
        {
            lambda = Max(0.1f*lambda, 1e-8f);
        }
        else
        {
            lambda *= 10.0f;
        }
    }
}

int compare(const void* a, const void* b) {
    observation_t* a_ptr = (observation_t*)a;
    observation_t* b_ptr = (observation_t*)b;
    return (a_ptr->wp_id) - (b_ptr->wp_id);
}

void reorder_obs_by_landmarks(observation_t* reordered_observations,
                              uint16_t num_observations)
{
    combsort(reordered_observations, num_observations, sizeof(observation_t), compare);
}

pose_graph_data_t allocate_load_pose_graph_data(pose_graph_stats_t* graph_stats, work_memory_t* work_memory)
{
    pose_graph_data_t pg_data;
    pg_data.poses = allocate_work_memory(work_memory, MAX_KEYFRAMES*sizeof(pose_t));
    pg_data.observations = allocate_work_memory(work_memory, MAX_OBSERVATIONS*sizeof(observation_t));
    pg_data.landmarks = allocate_work_memory(work_memory, (MAX_WORLD_FEATURES*sizeof(point3D_float_t)));
    pg_data.landmark_ages = allocate_work_memory(work_memory, MAX_WORLD_FEATURES*sizeof(uint8_t));
    pg_data.velocities = allocate_work_memory(work_memory, MAX_KEYFRAMES*sizeof(point3D_float_t));
    pg_data.imu_factors = allocate_work_memory(work_memory, MAX_KEYFRAMES*sizeof(imu_factor_with_bias_t));
    storage_move_in(pg_data.poses, KEYFRAME_POSES);
    storage_move_in(pg_data.observations, OBSERVATIONS);
    storage_move_in(pg_data.landmarks, WORLD_PTS);
    storage_move_in(pg_data.landmark_ages, WORLD_PT_AGE);
    storage_move_in(pg_data.velocities, KEYFRAME_VELOCITIES);
    storage_move_in(pg_data.imu_factors, IMU_FACTORS);

    pg_data.num_landmarks = MAX_WORLD_FEATURES;
    pg_data.num_observations = Min(graph_stats->total_observations, MAX_OBSERVATIONS);
    pg_data.num_poses = Min(graph_stats->total_keyframes, MAX_KEYFRAMES);
    pg_data.total_poses = graph_stats->total_keyframes;
    pg_data.newest_pose_offset = (graph_stats->total_keyframes-1) % MAX_KEYFRAMES;
    pg_data.anchor_pose_offset = (Max(graph_stats->total_keyframes-MAX_KEYFRAMES, 0)) % MAX_KEYFRAMES;

    return pg_data;
}

void store_pose_graph_data(pose_graph_data_t* pg_data)
{
    storage_move_out(pg_data->poses, KEYFRAME_POSES);
    storage_move_out(pg_data->velocities, KEYFRAME_VELOCITIES);
    storage_move_out(pg_data->landmarks, WORLD_PTS);
}

void initilaize_newest_velocity(pose_graph_data_t* pg_data,
                                work_memory_t work_memory)
{
    uint8_t current_pose_idx = (pg_data->total_poses-1) % MAX_KEYFRAMES;
    uint8_t prev_pose_idx = (pg_data->total_poses-2) % MAX_KEYFRAMES;
    float* r = allocate_work_memory(&work_memory, 9*sizeof(float));
    float* r_inv = allocate_work_memory(&work_memory, 9*sizeof(float));
    float g_body[3];
    matrix_2D_t T = {(float*)pg_data->poses[prev_pose_idx],{4,4,0,0}};
    matrix_2D_t R = {r,{3,3,0,0}};
    matrix_2D_t R_inv = {r_inv,{3,3,0,0}};
    rotationMatrixOfTransformation(&T, &R);
    matinv3x3(&R,&R_inv);
#if VELOCITY_UPDATE_EXACT
    /* Gravity is stored in the WORLD frame (see update_gravity_gyro_bias, which
     * builds it as R_cw^T * -gravity_b). Getting it into the body frame needs
     * R_cw, i.e. R -- not R_inv, which maps the other way. Upstream uses R_inv
     * here, so the compensation term is only correct while the camera has not
     * rotated. */
    matvec(&R,g,g_body,3);
#else
    matvec(&R_inv,g,g_body,3);
#endif
    float dt = pg_data->imu_factors[current_pose_idx].base.dt;

    // v_new = v_prev + alpha * R_prev^T * (dv + g_body * dt)
    // Blend factor: alpha = sensor_var / (sensor_var + gravity_var)
    //   = σ_a²·dt / (σ_a²·dt + σ_g_init²·dt²) = 1 / (1 + σ_g_init²·dt / σ_a²)
    // The initialization uses a larger effective gravity uncertainty than the
    // information matrix because the full systematic gravity misalignment biases
    // the predicted dv, whereas the information matrix only models local perturbations.
#if VELOCITY_UPDATE_EXACT
    /* No blending. v_j = v_i + g*dt + R_wc*dv is not an estimate to be damped,
     * it is the exact kinematic relation the preintegration was defined to
     * satisfy, and the preintegration is accurate: measured against ground
     * truth on VIODE 1_low, |dv| ratio 1.001 and |dp| ratio 0.969.
     *
     * Damping it does not reduce noise, it introduces a systematic deficit.
     * Integrated offline with ground-truth rotation over the full 66 s:
     *
     *   alpha = 1      |v| median 3.181 m/s, error vs GT 0.080 m/s, no drift
     *   alpha = 0.217  |v| median 0.689 m/s, error vs GT 2.465 m/s
     *   ground truth   |v| median 3.141 m/s
     *
     * The runaway to 47 m/s seen in the shipped pipeline is this deficit
     * combined with the 127 deg of accumulated attitude error from the
     * essential-matrix rotation; with ROTATION_FROM_GYRO the attitude is good
     * to 0.212 deg and the exact recursion tracks. See docs/27. */
    float alpha = 1.0f;
#else
    float sigma_g_init = 3.0f * GRAVITY_UNCERTAINTY;
    float alpha = 1.0f / (1.0f + sigma_g_init * sigma_g_init * dt
                                / (ACCEL_NOISE_DENSITY * ACCEL_NOISE_DENSITY));
#endif

    float dv_body[3] = {
        pg_data->imu_factors[current_pose_idx].base.dv.x + g_body[0]*dt,
        pg_data->imu_factors[current_pose_idx].base.dv.y + g_body[1]*dt,
        pg_data->imu_factors[current_pose_idx].base.dv.z + g_body[2]*dt
    };
    // R_inv = R_cw^T maps body->world
    float dv_world[3];
    matvec(&R_inv, dv_body, dv_world, 3);
    float vx = pg_data->velocities[prev_pose_idx].x + alpha * dv_world[0];
    float vy = pg_data->velocities[prev_pose_idx].y + alpha * dv_world[1];
    float vz = pg_data->velocities[prev_pose_idx].z + alpha * dv_world[2];

#if VELOCITY_LEAK_ENABLE
    /* Leaky integrator, because the recursion above has no correction at all.
     *
     * v_new = v_prev + alpha*(dv + g dt) is open-loop dead reckoning: every
     * keyframe adds, nothing ever subtracts, and LEVIO's optimiser leaves the
     * velocity state essentially untouched (deltas ~1e-4). Any residual
     * accelerometer bias or gravity misalignment therefore integrates without
     * bound. Measured monotone ramps, all still rising at the last keyframe:
     *
     *   EuRoC V101   0 -> 1.00 m/s over 144 s   (GT median 0.40)  0.007 m/s^2
     *   EuRoC MH01   0 -> 5.48 m/s over 182 s   (GT median 0.45)  0.030 m/s^2
     *   VIODE 1_low  0 -> 47.1 m/s over  66 s   (GT median 2.56)  0.71  m/s^2
     *
     * The 24x spread tracks alpha exactly: VIODE runs GRAVITY_UNCERTAINTY 0.2
     * (alpha 0.217) against EuRoC's 1.0 (alpha 0.011). Tuning alpha cannot fix
     * this -- small alpha pins the velocity at zero and collapses the map,
     * large alpha lets it run away. The integrator needs a leak, not a gain.
     *
     * Blending towards the velocity implied by the keyframe positions was tried
     * first and is *circular*: SCALE_DT_MODE takes the translation scale from
     * |v|, so deriving |v| back from those translations is positive feedback.
     * Measured: the trajectory collapsed from 290 m to 9.7 m of path.
     *
     * A leak is not circular. It bounds |v| at alpha*|dv|/lambda using no
     * measurement at all, which is the standard treatment for an inertial
     * velocity with no absolute reference. */
    {
        float k = 1.0f - VELOCITY_LEAK;
        vx *= k; vy *= k; vz *= k;
    }
#endif

    pg_data->velocities[current_pose_idx].x = vx;
    pg_data->velocities[current_pose_idx].y = vy;
    pg_data->velocities[current_pose_idx].z = vz;
    LOG_INFO("Velocity init: alpha=%.3f, dv_body [%f, %f, %f]\n",
        alpha, dv_body[0], dv_body[1], dv_body[2]);
}


void process_pose_graph_optimizer(pose_graph_stats_t* graph_stats,
                                  work_memory_t work_memory)
{
    LOG_WARNING("*** Optimizer Results ***\n");
    LOG_WARNING("Total Frames: \t%d\n",graph_stats->total_frames);
    LOG_WARNING("Keyframes: \t%d\n",graph_stats->total_keyframes);
    LOG_WARNING("Features: \t%d\n",graph_stats->total_features);
    LOG_WARNING("Observations: \t%d\n",graph_stats->total_observations);
    LOG_WARNING("***\n\n");
    if(graph_stats->total_keyframes < 2)
    {
        return;
    }

    pi_perf_reset();
    pi_perf_start();

    pose_graph_data_t pg_data = allocate_load_pose_graph_data(graph_stats, &work_memory);
    reorder_obs_by_landmarks(pg_data.observations, pg_data.num_observations);
    initilaize_newest_velocity(&pg_data, work_memory);
    matrix_2D_t K = {K_data2,{3,3,0,0}};

    matrix_2D_t T = {(float*)&pg_data.poses[pg_data.newest_pose_offset], {4,4,0,0}};
    LOG_INFO("Un-Opt. keyframe\n");
    matprint(&T, INFO_LEVEL);

#if BA_ITERATIONS > 0
    bundle_adjustment_schur(&pg_data,
                            &K, BA_ITERATIONS,
                            0.001, work_memory);
#endif

#if VI_ALIGN_ENABLE
    /* After bundle adjustment, so the camera centres the alignment reads are
     * the optimised ones. This overwrites the velocities that
     * initilaize_newest_velocity() dead-reckoned, which is the point: those
     * ramp without bound, these are anchored to the accelerometer content of
     * the preintegration over the whole window. See vi_alignment.h. */
    {
        float s_align = 1.0f;
        if (vi_align_window(pg_data.poses, pg_data.imu_factors, pg_data.velocities,
                            g, pg_data.num_poses, pg_data.total_poses,
                            &s_align, work_memory)) {
            /* The solved velocities are metric; the map is not. Applying only
             * the velocities leaves the two in different units, and the next
             * window then sees a world whose displacements no longer match its
             * own velocity -- measured, that feedback collapsed the trajectory
             * from 157.7 m of path to 17.6 m. So the map is rescaled with the
             * same factor, which is the step VINS-Mono performs once after
             * initialisation. Once the two agree, s comes out at 1 and this is
             * a no-op. */
            float sa = 1.0f + VI_ALIGN_SCALE_GAIN * (s_align - 1.0f);
            if (sa > VI_ALIGN_MIN_SCALE && sa < VI_ALIGN_MAX_SCALE) {
                for (int i = 0; i < MAX_KEYFRAMES; ++i) {
                    float* T = (float*) &pg_data.poses[i];
                    T[3] *= sa; T[7] *= sa; T[11] *= sa;   /* c = -R^T t */
                }
                for (uint16_t i = 0; i < pg_data.num_landmarks; ++i) {
                    pg_data.landmarks[i].x *= sa;
                    pg_data.landmarks[i].y *= sa;
                    pg_data.landmarks[i].z *= sa;
                }
                LOG_INFO("VI align applied: solved s=%f, world rescaled by %f\n",
                         (double) s_align, (double) sa);
            }
        }
    }
#endif

    store_pose_graph_data(&pg_data);
    memcpy(&graph_stats->prev_velocity, &pg_data.velocities[pg_data.newest_pose_offset], sizeof(point3D_float_t));
    memcpy(graph_stats->prev_pose, &pg_data.poses[pg_data.newest_pose_offset], sizeof(pose_t));
    pi_perf_stop(); 
    
    LOG_TIMING("[Optimizer] Number of cycles %d\n",pi_perf_read(PI_PERF_CYCLES));
    /* K230-VIO HOOK */
    levio_hook_stage("optimizer", pi_perf_read(PI_PERF_CYCLES));
    LOG_WARNING("Oldest Optimized keyframe\n");
    matrix_2D_t T_anchor = {(float*)&pg_data.poses[pg_data.anchor_pose_offset], {4,4,0,0}};
    matprint(&T_anchor, WARNING_LEVEL);
    LOG_WARNING("Newest Optimized keyframe\n");
    matprint(&T, WARNING_LEVEL);
    LOG_INFO("Velocity [%f, %f, %f]\n",pg_data.velocities[pg_data.newest_pose_offset].x,pg_data.velocities[pg_data.newest_pose_offset].y,pg_data.velocities[pg_data.newest_pose_offset].z);
}