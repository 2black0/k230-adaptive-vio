/*
 * K230-VIO — implementation of the PMSIS compatibility shim.
 * SPDX-License-Identifier: MIT
 */

#include "pmsis.h"

#include <time.h>

#if defined(LEVIO_PARALLEL) && LEVIO_PARALLEL
#include <omp.h>
#endif

/* ================================================================== */
/* Allocation                                                          */
/*                                                                     */
/* LEVIO allocates once at startup and never frees (it targets a bare  */
/* metal scratchpad). We keep a small registry so the harness can       */
/* report the real footprint and release everything cleanly on exit —   */
/* useful for comparing against the 110 KB L1 budget of the original.   */
/* ================================================================== */

#define COMPAT_MAX_BLOCKS 64

static void*  g_blocks[COMPAT_MAX_BLOCKS];
static size_t g_block_count = 0;
static size_t g_alloc_total = 0;

void* levio_compat_malloc(size_t size)
{
    /* 8-byte alignment matches the GAP9 allocator and keeps float/double
       accesses aligned on RISC-V, where misaligned access can trap. */
    void* p = NULL;
    if (size == 0) {
        size = 8;
    }
    size = (size + 7u) & ~((size_t) 7u);

    p = malloc(size);
    if (p == NULL) {
        fprintf(stderr, "[compat] allocation of %zu bytes failed\n", size);
        return NULL;
    }

    if (g_block_count < COMPAT_MAX_BLOCKS) {
        g_blocks[g_block_count++] = p;
    }
    g_alloc_total += size;
    return p;
}

void levio_compat_free(void* ptr)
{
    if (ptr == NULL) {
        return;
    }
    for (size_t i = 0; i < g_block_count; ++i) {
        if (g_blocks[i] == ptr) {
            g_blocks[i] = g_blocks[--g_block_count];
            break;
        }
    }
    free(ptr);
}

size_t levio_compat_alloc_total(void) { return g_alloc_total; }

void levio_compat_free_all(void)
{
    for (size_t i = 0; i < g_block_count; ++i) {
        free(g_blocks[i]);
    }
    g_block_count = 0;
}

/* ================================================================== */
/* Parallelism                                                         */
/* ================================================================== */

/* Thread-local so each OpenMP worker reports its own index, matching the
   GAP9 cluster where every core has a distinct hardware ID. */
#if defined(_MSC_VER)
static __declspec(thread) uint16_t g_core_id = 0;
#else
static __thread uint16_t g_core_id = 0;
#endif

uint16_t pi_core_id(void)    { return g_core_id; }
uint32_t pi_cluster_id(void) { return 0; }

void pi_cl_team_fork(int nb_cores, void (*fn)(void*), void* arg)
{
#if defined(LEVIO_PARALLEL) && LEVIO_PARALLEL
    if (nb_cores <= 1) {
        g_core_id = 0;
        fn(arg);
        return;
    }
    #pragma omp parallel num_threads(nb_cores)
    {
        g_core_id = (uint16_t) omp_get_thread_num();
        fn(arg);
    }
    /* Implicit barrier at the end of the parallel region matches the
       GAP9 fork/join semantics. Restore the master identity so any code
       running outside the region still sees core 0. */
    g_core_id = 0;
#else
    /* Serial reference path. Running the kernel once with core_id 0 is NOT
       equivalent to running it on nb_cores cores: the LEVIO kernels derive
       their work range from core_id, so a single call would only cover the
       first slice. We therefore iterate over every core index in turn,
       which produces exactly the same result as a real fork, just serially. */
    for (int core = 0; core < nb_cores; ++core) {
        g_core_id = (uint16_t) core;
        fn(arg);
    }
    g_core_id = 0;
#endif
}

void pi_cl_team_barrier(void)
{
    /* pi_cl_team_fork already provides join semantics; nothing to do. */
}

/* ================================================================== */
/* Performance counters                                                */
/* ================================================================== */

/* CLOCK_MONOTONIC is the default everywhere, including RISC-V.
 *
 * The obvious thing on RISC-V is `rdcycle`, and that is what this used to do.
 * It works on x86 hosts (never taken) and under QEMU, but it dies with SIGILL
 * on the K230 board: Linux 6.6 gates the cycle counter behind the CY bit of
 * scounteren, which it only sets for a task that has mmap'ed a perf event.
 * Setting /proc/sys/kernel/perf_user_access = 1 is not enough -- the board
 * already has it and still traps. The faulting word is 0xc0002773, i.e.
 * `csrrs a4, cycle, x0`.
 *
 * Using the monotonic clock instead is also better methodology: cycle counts
 * drift with DVFS, and reporting nanoseconds makes host and board numbers
 * directly comparable. plan.md section 6.5 asks for exactly this mapping.
 *
 * Build with -DCOMPAT_USE_RDCYCLE to opt back in where the counter is known
 * to be readable from user mode. */
#if defined(COMPAT_USE_RDCYCLE) && defined(__riscv) && (__riscv_xlen == 64)
#define COMPAT_HAVE_RDCYCLE 1
static inline uint64_t compat_now(void)
{
    uint64_t c;
    __asm__ volatile ("rdcycle %0" : "=r" (c));
    return c;
}
static const char* const COMPAT_PERF_UNIT = "cycles";
#else
#define COMPAT_HAVE_RDCYCLE 0
static inline uint64_t compat_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t) ts.tv_sec * 1000000000ull + (uint64_t) ts.tv_nsec;
}
static const char* const COMPAT_PERF_UNIT = "ns";
#endif

static uint64_t g_perf_start   = 0;
static uint64_t g_perf_elapsed = 0;

void pi_perf_reset(void) { g_perf_elapsed = 0; }
void pi_perf_start(void) { g_perf_start = compat_now(); }
void pi_perf_stop(void)  { g_perf_elapsed += compat_now() - g_perf_start; }

uint32_t pi_perf_read(int event)
{
    (void) event;
    return (uint32_t) g_perf_elapsed;
}

const char* levio_compat_perf_unit(void) { return COMPAT_PERF_UNIT; }
