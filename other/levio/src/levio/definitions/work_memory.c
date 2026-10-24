// Copyright (c) 2026 ETH Zurich. All rights reserved.
// SPDX-FileCopyrightText: 2026 ETH Zurich
// SPDX-License-Identifier: MIT
// Author: Jonas Kühne

#include "work_memory.h"
#include "logging.h"

/* Every hand-out is rounded up so the *next* one stays aligned.
 *
 * Upstream bumps the pointer by the raw byte count. That is fine as long as
 * nothing wider than a byte follows an odd-sized allocation, which held on
 * GAP9 but not here: in ransacEssentialMatrixMulticore an `uint8_t` mask of
 * match_counter bytes is allocated immediately before two point2D_float_t
 * arrays, so an odd match_counter leaves the float arrays on an odd address.
 *
 * GCC's auto-vectoriser then emits `vsseg2e32.v` for the interleaved float
 * store, the C908 requires element alignment for it, and the process dies
 * with SIGBUS (scause 6, store address misaligned). The scalar build survives
 * only because it never widens the access.
 *
 * 8 bytes covers double and pointer on lp64d. The root arena comes from
 * malloc (via the pi_cl_l1_malloc shim), which is already 16-byte aligned. */
#define WORK_MEMORY_ALIGN ((size_t) 8)

static inline size_t work_memory_round_up(size_t n)
{
    return (n + (WORK_MEMORY_ALIGN - 1)) & ~(WORK_MEMORY_ALIGN - 1);
}

void* allocate_work_memory(work_memory_t* work_memory, size_t size)
{
    size_t padded = work_memory_round_up(size);

    if(work_memory->size_left < padded)
    {
        LOG_ERROR("\nOUT OF MEMORY!\n\n");
        return NULL;
    }
    void* cur_work_ptr = work_memory->memory_ptr;
    work_memory->memory_ptr += padded;
    work_memory->size_left -= (uint32_t) padded;
    return cur_work_ptr;
}

void print_work_memory(work_memory_t* work_memory)
{
    printf("Memory Available %d, ptr %d \n",work_memory->size_left,work_memory->memory_ptr);
}

work_memory_t split_work_memory(work_memory_t* work_memory, uint8_t total_chunks, uint8_t current_id)
{
    /* Round the stride *down* so every chunk starts aligned and the chunks
     * still fit. Same reason as allocate_work_memory above: an unaligned
     * per-core sub-arena reintroduces the misaligned vector store. */
    uint32_t subset_size = work_memory->size_left/total_chunks;
    subset_size &= ~((uint32_t) WORK_MEMORY_ALIGN - 1u);
    work_memory_t subset_work_memory = {work_memory->memory_ptr+current_id*subset_size,subset_size};
    return subset_work_memory;
}