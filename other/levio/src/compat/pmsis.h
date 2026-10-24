/*
 * K230-VIO — compatibility shim replacing the GAP9 PMSIS runtime.
 *
 * LEVIO includes "pmsis.h" from 25 translation units. By providing a header with
 * the same name on the include path, the LEVIO sources compile unmodified on a
 * POSIX host (x86 or RISC-V Linux).
 *
 * Mapping (see docs/03-porting-gap9-to-k230.md for the full inventory):
 *   pi_l2_malloc / pi_cl_l1_malloc  -> malloc
 *   pi_cl_dma_cmd / pi_cl_dma_wait  -> memcpy
 *   pi_cl_team_fork                 -> OpenMP parallel region (or serial call)
 *   pi_core_id                      -> thread index
 *   pi_perf_*                       -> monotonic clock (or rdcycle on RISC-V)
 *   pi_cluster_* / pi_open          -> direct function call
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __PMSIS_SHIM_H__
#define __PMSIS_SHIM_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Memory placement attributes                                         */
/* On GAP9 these pin symbols to a specific memory bank. Linux has a    */
/* single address space, so they expand to nothing.                    */
/* ------------------------------------------------------------------ */
#define PI_L2
#define PI_L1
#define PI_FC_L1
#define PI_CL_L1
#define PI_L2_DATA
#define PI_CL_L1_DATA

/* ------------------------------------------------------------------ */
/* Name collision with the C standard library                          */
/*                                                                     */
/* LEVIO's definitions/prng.h declares its own uint32_t rand() and      */
/* rand_r(uint32_t*), which clash with the <stdlib.h> declarations we   */
/* have just pulled in. On GAP9 that header was never in scope.         */
/*                                                                     */
/* Renaming here (after <stdlib.h>, so the standard prototypes stay     */
/* intact) keeps the LEVIO sources byte-identical to upstream: their    */
/* declaration, definition and every call site all rewrite together.    */
/* Harness code must therefore not call the standard rand()/rand_r();   */
/* it uses its own generator instead.                                   */
/* ------------------------------------------------------------------ */
#define rand   levio_prng_rand
#define rand_r levio_prng_rand_r

/* ------------------------------------------------------------------ */
/* Device handle                                                       */
/* LEVIO only ever passes this through to allocation calls, which      */
/* ignore it here. Kept as a distinct type so signatures still match.  */
/* ------------------------------------------------------------------ */
typedef void pi_device_t;

/* ------------------------------------------------------------------ */
/* Allocation                                                          */
/* ------------------------------------------------------------------ */
void*    levio_compat_malloc(size_t size);
void     levio_compat_free(void* ptr);
size_t   levio_compat_alloc_total(void);   /* bytes handed out so far   */
void     levio_compat_free_all(void);      /* release tracked blocks    */

#define pi_l2_malloc(size)             levio_compat_malloc((size_t)(size))
#define pi_cl_l1_malloc(dev, size)     (((void)(dev)), levio_compat_malloc((size_t)(size)))
#define pi_l2_free(ptr, size)          (((void)(size)), levio_compat_free(ptr))
#define pi_cl_l1_free(dev, ptr, size)  (((void)(dev)), ((void)(size)), levio_compat_free(ptr))

/* ------------------------------------------------------------------ */
/* DMA -> memcpy                                                       */
/* ------------------------------------------------------------------ */
typedef struct { int unused; } pi_cl_dma_cmd_t;

#define PI_CL_DMA_DIR_EXT2LOC 0
#define PI_CL_DMA_DIR_LOC2EXT 1

static inline void pi_cl_dma_cmd(uintptr_t ext, uintptr_t loc, uint32_t size,
                                 int dir, pi_cl_dma_cmd_t* cmd)
{
    (void) cmd;
    if (dir == PI_CL_DMA_DIR_EXT2LOC) {
        memcpy((void*)(uintptr_t) loc, (const void*)(uintptr_t) ext, (size_t) size);
    } else {
        memcpy((void*)(uintptr_t) ext, (const void*)(uintptr_t) loc, (size_t) size);
    }
}

static inline void pi_cl_dma_wait(void* cmd) { (void) cmd; }

/* ------------------------------------------------------------------ */
/* Parallelism                                                         */
/*                                                                     */
/* GAP9 semantics: fork nb_cores workers that all run the same function */
/* with the same argument, distinguish themselves via pi_core_id(), and */
/* rejoin at an implicit barrier. OpenMP maps onto this one-to-one.     */
/* ------------------------------------------------------------------ */
uint16_t pi_core_id(void);
uint32_t pi_cluster_id(void);
void     pi_cl_team_fork(int nb_cores, void (*fn)(void*), void* arg);
void     pi_cl_team_barrier(void);

/* ------------------------------------------------------------------ */
/* Performance counters                                                */
/*                                                                     */
/* LEVIO prints these as "Number of cycles". On RISC-V we read the      */
/* cycle CSR so the unit stays faithful to the original instrumentation */
/* and stays comparable with the GAP9 numbers in the LEVIO paper.       */
/* Elsewhere (x86 host) the value is elapsed microseconds — the log     */
/* line states which unit is in use at startup.                        */
/* ------------------------------------------------------------------ */
#define PI_PERF_CYCLES 0

void     pi_perf_reset(void);
void     pi_perf_start(void);
void     pi_perf_stop(void);
uint32_t pi_perf_read(int event);
const char* levio_compat_perf_unit(void);

/* ------------------------------------------------------------------ */
/* Cluster lifecycle — collapses to direct calls                       */
/* ------------------------------------------------------------------ */
#define PI_CORE_CLUSTER 0

struct pi_cluster_task {
    void  (*entry)(void*);
    void*   arg;
};

static inline int pi_open(int type, pi_device_t** device)
{
    (void) type;
    *device = NULL;            /* no real device; allocations ignore it */
    return 0;                  /* success */
}

static inline struct pi_cluster_task* pi_cluster_task(struct pi_cluster_task* task,
                                                      void (*entry)(void*), void* arg)
{
    task->entry = entry;
    task->arg   = arg;
    return task;
}

static inline int pi_cluster_send_task_to_cl(pi_device_t* device, struct pi_cluster_task* task)
{
    (void) device;
    task->entry(task->arg);    /* synchronous, same as blocking on GAP9 */
    return 0;
}

static inline void pi_cluster_close(pi_device_t* device) { (void) device; }

/* ------------------------------------------------------------------ */
/* Frequency / exit                                                    */
/* ------------------------------------------------------------------ */
#define PI_FREQ_DOMAIN_FC 0
#define PI_FREQ_DOMAIN_CL 1

static inline int pi_freq_set(int domain, uint32_t freq) { (void) domain; (void) freq; return 0; }

#define pmsis_exit(code) exit(code)

#ifdef __cplusplus
}
#endif

#endif /* __PMSIS_SHIM_H__ */
