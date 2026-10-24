// Copyright (c) 2026 ETH Zurich. All rights reserved.
// SPDX-FileCopyrightText: 2026 ETH Zurich
// SPDX-License-Identifier: MIT
// Author: Jonas Kühne

#ifndef __POSE_GRAPH_OPTIMIZER__H__
#define __POSE_GRAPH_OPTIMIZER__H__

#include "pmsis.h"
#include "definitions/type_definitions.h"

/**
 * @brief Initialize L2 cache for the pose graph optimizer.
 * 
 * Sets up and prepares the L2 cache to optimize memory access patterns
 * during pose graph optimization computations.
 */
void optimizer_init_l2_cache();

/**
 * @brief Process and optimize a pose graph using the configured optimizer.
 * 
 * Performs pose graph optimization by adjusting node poses to minimize
 * errors in edge constraints. Updates statistics about the optimization process.
 * 
 * @param graph_stats Pointer to pose_graph_stats_t structure containing
 *                    statistics about the pose graph and optimization results.
 * @param work_memory work_memory_t structure providing allocated memory buffers
 *                    for intermediate computations during optimization.
 */
void process_pose_graph_optimizer(pose_graph_stats_t* graph_stats,
                                  work_memory_t work_memory);

/**
 * @brief Solves an n×n linear system in place from an augmented matrix [A|b].
 *
 * Upstream declares this only in the .c file, where it was the sole caller.
 * vi_alignment.c calls it too, so the prototype has to be visible here: GCC 14
 * rejects an implicit declaration, which is what the K230 cross build does.
 *
 * @param Ab Augmented n×(n+1) matrix, modified in place.
 * @param x  Output vector of length n.
 * @param n  System size.
 * @param work_memory Scratch arena for the elimination workspace.
 * @return 1 on success, 0 if the system is singular.
 */
int solve_dense_system(float* Ab, float* x, int n, work_memory_t work_memory);

#endif /* __POSE_GRAPH_OPTIMIZER__H__ */