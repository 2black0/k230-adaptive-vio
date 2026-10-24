/*
 * K230-VIO — replay harness entry point.
 *
 * Replaces the GAP9 main.c: same pipeline orchestration, but driven by a
 * pluggable sensor source instead of hardcoded file paths, and emitting a
 * TUM-format trajectory for evaluation with evo.
 *
 * See docs/05-dataset-replay-harness.md.
 *
 * SPDX-License-Identifier: MIT
 */

#include "trace.h"
#include "pmsis.h"

#include "definitions/type_definitions.h"
#include "feature_handling/orb_gap.h"
#include "visual_odometry/vo.h"
#include "optimizer/pose_graph_optimizer.h"
#include "optimizer/imu_preintegration.h"

#include "source.h"
#include "traj_writer.h"
#include "pgm_io.h"
#include "levio_hooks.h"
#include "metrics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/*
 * The PGM-encoded IMU layout depends on imu_measurement_t being exactly
 * IMU_WIDTH bytes with no padding. Catch a layout change at compile time
 * rather than as mysterious drift at runtime.
 */
typedef char levio_imu_layout_check[(sizeof(imu_measurement_t) == IMU_WIDTH) ? 1 : -1];

/* Globals that lived in the GAP9 main.c. */
work_memory_t       total_l1_work_memory;
pose_graph_stats_t* graph_stats;

typedef struct {
    uint8_t*           img_buffer;
    imu_measurement_t* imu_batch;
    int                imu_count;
} cluster_arguments_t;

/* ------------------------------------------------------------------ */
/* Pipeline setup and per-frame step (mirrors the GAP9 cluster tasks)   */
/* ------------------------------------------------------------------ */

static void init_data_structures(void* arg)
{
    pi_device_t* cluster_device = (pi_device_t*) arg;

    graph_stats = init_pose_graph_stats(cluster_device);
    initialize_orb_storage_gap(cluster_device, IMG_WIDTH, IMG_HEIGHT);
    initialize_imu_preintegrator(cluster_device);

    total_l1_work_memory.memory_ptr = pi_cl_l1_malloc(cluster_device, L1_WORK_MEMORY_SIZE);
    total_l1_work_memory.size_left  = L1_WORK_MEMORY_SIZE;

    if (total_l1_work_memory.memory_ptr == NULL) {
        fprintf(stderr, "[harness] failed to reserve %d bytes of work memory\n",
                L1_WORK_MEMORY_SIZE);
        exit(1);
    }
}

static void cluster_delegate(void* arg)
{
    work_memory_t        work_memory = total_l1_work_memory;
    cluster_arguments_t* arguments   = (cluster_arguments_t*) arg;
    uint8_t              is_keyframe;

#ifdef LEVIO_POISON_ARENA
    /* Diagnostic build only. The arena is a bump allocator that is rewound,
     * never cleared, so a read-before-write silently picks up the previous
     * frame's bytes: deterministic, plausible-looking, and invisible. Filling
     * it with signalling-NaN bit patterns turns any such read into a NaN that
     * propagates to the pose, which is trivial to spot. Used to establish that
     * the V203 divergence is NOT an uninitialised read (docs/20 section 6). */
    {
        uint32_t* p = (uint32_t*) total_l1_work_memory.memory_ptr;
        for (uint32_t i = 0; i < total_l1_work_memory.size_left / 4u; i++) {
            p[i] = 0x7FA00000u;   /* signalling NaN */
        }
    }
#endif

    if (graph_stats->total_keyframes > 0 && arguments->imu_batch != NULL) {
        pi_perf_reset();
        pi_perf_start();
        process_imu_preintegration(arguments->imu_batch, work_memory);
        pi_perf_stop();
        LOG_TIMING("[IMU Pre-Integration] Number of cycles %d\n",
                   pi_perf_read(PI_PERF_CYCLES));
        /* K230-VIO HOOK */
        levio_hook_stage("imu", pi_perf_read(PI_PERF_CYCLES));
    }

    is_keyframe = process_vo_pipeline(graph_stats, arguments->img_buffer, work_memory);
    LOG_WARNING("Optimization required? %d\n", is_keyframe);

    if (is_keyframe) {
        extract_and_restart_imu_preintegration(graph_stats, work_memory);
        process_pose_graph_optimizer(graph_stats, work_memory);
    }
}

/* ------------------------------------------------------------------ */
/* Command line                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    const char* data_dir;
    const char* out_path;
    int         paced;
    int         start_index;
    int         stop_index;
    double      jitter_ms;
    double      drop_rate;
    double      imu_drop_rate;
    double      speed;
    uint32_t    seed;
    int         quiet;
    int         dynmask;
    const char* log_dir;
} options_t;

static void usage(const char* prog)
{
    printf(
"K230-VIO replay harness\n"
"\n"
"Usage: %s --data <dir> [options]\n"
"\n"
"Required:\n"
"  --data <dir>          dataset directory containing images/ and imu/\n"
"\n"
"Options:\n"
"  --mode fast|paced     fast   = Mode A, deterministic, no pacing (default)\n"
"                        paced  = Mode B, real-time paced with a producer thread\n"
"  --out <file>          write TUM trajectory to <file>\n"
"  --log-dir <dir>       write <dir>/trajectory.tum plus framelog.csv,\n"
"                        timing.csv, resource.csv, summary.csv, power.csv.\n"
"                        Implies --out <dir>/trajectory.tum unless --out is given.\n"
"  --start <n>           first frame index (default 0)\n"
"  --stop <n>            one past the last frame (default: autodetect)\n"
"  --speed <x>           Mode B playback rate multiplier (default 1.0)\n"
"  --jitter-ms <x>       Mode B: inject +/- x ms timestamp jitter\n"
"  --drop-rate <x>       Mode B: drop frames with probability x [0..1]\n"
"  --imu-drop-rate <x>   Mode B: drop IMU samples with probability x [0..1]\n"
"  --seed <n>            seed for fault injection (default 0x12345678)\n"
"  --dynmask             drop keypoints on ground-truth dynamic objects\n"
"                        (oracle arm of the dynamic-rejection study)\n"
"  --quiet               suppress the LEVIO per-frame log\n"
"  --help                show this message\n"
"\n"
"Evaluate the result with:\n"
"  evo_ape tum groundtruth.txt trajectory.txt -a -s --plot\n",
        prog);
}

static int parse_args(int argc, char** argv, options_t* opt)
{
    memset(opt, 0, sizeof(*opt));
    opt->speed = 1.0;
    opt->seed  = 0x12345678u;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];

        #define NEED_VALUE()                                                   \
            do {                                                               \
                if (i + 1 >= argc) {                                           \
                    fprintf(stderr, "[harness] %s requires a value\n", a);      \
                    return -1;                                                 \
                }                                                              \
            } while (0)

        if (strcmp(a, "--help") == 0 || strcmp(a, "-h") == 0) {
            usage(argv[0]);
            return 1;
        } else if (strcmp(a, "--data") == 0) {
            NEED_VALUE(); opt->data_dir = argv[++i];
        } else if (strcmp(a, "--out") == 0) {
            NEED_VALUE(); opt->out_path = argv[++i];
        } else if (strcmp(a, "--log-dir") == 0) {
            NEED_VALUE(); opt->log_dir = argv[++i];
        } else if (strcmp(a, "--mode") == 0) {
            NEED_VALUE();
            ++i;
            if (strcmp(argv[i], "paced") == 0) {
                opt->paced = 1;
            } else if (strcmp(argv[i], "fast") == 0) {
                opt->paced = 0;
            } else {
                fprintf(stderr, "[harness] unknown mode '%s'\n", argv[i]);
                return -1;
            }
        } else if (strcmp(a, "--start") == 0) {
            NEED_VALUE(); opt->start_index = atoi(argv[++i]);
        } else if (strcmp(a, "--stop") == 0) {
            NEED_VALUE(); opt->stop_index = atoi(argv[++i]);
        } else if (strcmp(a, "--speed") == 0) {
            NEED_VALUE(); opt->speed = atof(argv[++i]);
        } else if (strcmp(a, "--jitter-ms") == 0) {
            NEED_VALUE(); opt->jitter_ms = atof(argv[++i]);
        } else if (strcmp(a, "--drop-rate") == 0) {
            NEED_VALUE(); opt->drop_rate = atof(argv[++i]);
        } else if (strcmp(a, "--imu-drop-rate") == 0) {
            NEED_VALUE(); opt->imu_drop_rate = atof(argv[++i]);
        } else if (strcmp(a, "--seed") == 0) {
            NEED_VALUE(); opt->seed = (uint32_t) strtoul(argv[++i], NULL, 0);
        } else if (strcmp(a, "--dynmask") == 0) {
            opt->dynmask = 1;
        } else if (strcmp(a, "--quiet") == 0) {
            opt->quiet = 1;
        } else {
            fprintf(stderr, "[harness] unknown option '%s'\n", a);
            return -1;
        }

        #undef NEED_VALUE
    }

    if (opt->data_dir == NULL) {
        fprintf(stderr, "[harness] --data is required (try --help)\n");
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */

static double wall_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

int main(int argc, char** argv)
{
    options_t          opt;
    replay_config_t    cfg;
    sensor_source_t    source;
    pi_device_t*       cluster_dev = NULL;
    struct pi_cluster_task cl_task;

    uint8_t*           img_buffer;
    uint8_t*           dyn_mask_buf = NULL;
    imu_measurement_t  imu_batch[IMU_HEIGHT];
    int                imu_count = 0;
    double             t_frame   = 0.0;
    uint32_t           processed = 0;
    double             t_begin;
    double             t_last_resource_sample = 0.0;
    char               traj_path[1200];
    int                rc;

    rc = parse_args(argc, argv, &opt);
    if (rc != 0) {
        return rc > 0 ? 0 : 1;
    }

    /* ---- resolve frame range ---- */
    if (opt.stop_index <= 0) {
        opt.stop_index = source_autodetect_stop_index(opt.data_dir);
        if (opt.stop_index <= 0) {
            fprintf(stderr, "[harness] no frames found under '%s/images'\n", opt.data_dir);
            return 1;
        }
    }
    if (opt.start_index >= opt.stop_index) {
        fprintf(stderr, "[harness] empty frame range [%d, %d)\n",
                opt.start_index, opt.stop_index);
        return 1;
    }

    printf("=== K230-VIO replay harness ===\n");
    printf("dataset      : %s\n", opt.data_dir);
    printf("frames       : [%d, %d)  (%d frames)\n",
           opt.start_index, opt.stop_index, opt.stop_index - opt.start_index);
    printf("resolution   : %dx%d\n", IMG_WIDTH, IMG_HEIGHT);
    printf("mode         : %s\n", opt.paced ? "paced (Mode B)" : "fast (Mode A)");
    printf("parallel     : %s (NB_CORES=%d, SINGLE_CORE=%d)\n",
#if defined(LEVIO_PARALLEL) && LEVIO_PARALLEL
           "on",
#else
           "off",
#endif
           NB_CORES, SINGLE_CORE);
    printf("perf unit    : %s\n", levio_compat_perf_unit());
    printf("trajectory   : %s\n", opt.out_path ? opt.out_path : "(not written)");
    printf("================================\n");

    if (opt.quiet) {
        /* LEVIO logs through printf; redirecting stdout would also swallow our
           own report, so instead we simply note that the log stays on. */
        printf("[harness] --quiet: LEVIO logging is compile-time (LOG_LEVEL in config.h);\n"
               "[harness]          rebuild with a lower LOG_LEVEL to silence it.\n");
    }

    /* ---- open source ---- */
    memset(&cfg, 0, sizeof(cfg));
    cfg.data_dir      = opt.data_dir;
    cfg.start_index   = opt.start_index;
    cfg.stop_index    = opt.stop_index;
    cfg.jitter_ms     = opt.jitter_ms;
    cfg.drop_rate     = opt.drop_rate;
    cfg.imu_drop_rate = opt.imu_drop_rate;
    cfg.seed          = opt.seed;
    cfg.speed         = opt.speed;

    rc = opt.paced ? source_paced_open(&source, &cfg)
                   : source_pgm_open(&source, &cfg);
    if (rc != SOURCE_OK) {
        fprintf(stderr, "[harness] cannot open sensor source\n");
        return 1;
    }

    /* A --log-dir run directory should be self-contained, so it gets the
       trajectory too. An explicit --out still wins. */
    if (opt.log_dir != NULL && opt.out_path == NULL) {
        snprintf(traj_path, sizeof(traj_path), "%s/trajectory.tum", opt.log_dir);
        opt.out_path = traj_path;
    }

    if (metrics_open(opt.log_dir) != 0) {
        source.close(&source);
        return 1;
    }

    if (traj_writer_open(opt.out_path) != 0) {
        source.close(&source);
        metrics_close();
        return 1;
    }

    /* Per-frame numerical trace, alongside the trajectory. Compiled out unless
     * LEVIO_TRACE_CSV is defined; see definitions/trace.h. */
#ifdef LEVIO_TRACE_CSV
    {
        static char trace_path[1024];
        snprintf(trace_path, sizeof(trace_path), "%s.trace.csv",
                 opt.out_path ? opt.out_path : "levio");
        levio_trace_open(trace_path);
        printf("[harness] per-frame trace: %s\n", trace_path);
    }
#endif

    /* ---- pipeline init (mirrors GAP9 main.c) ---- */
    if (opt.dynmask) {
        dyn_mask_buf = (uint8_t*) pi_l2_malloc(IMG_SIZE);
        if (dyn_mask_buf == NULL) {
            fprintf(stderr, "[harness] cannot allocate dynamic mask buffer\n");
            return 1;
        }
        printf("[harness] dynamic-object masking: ON (ground-truth oracle)\n");
    }

    img_buffer = (uint8_t*) pi_l2_malloc(IMG_SIZE);
    if (img_buffer == NULL) {
        fprintf(stderr, "[harness] cannot allocate frame buffer\n");
        source.close(&source);
        return 1;
    }

    initialize_data_storage();
    optimizer_init_l2_cache();

    if (pi_open(PI_CORE_CLUSTER, &cluster_dev)) {
        fprintf(stderr, "[harness] cluster open failed\n");
        return 1;
    }

    pi_cluster_send_task_to_cl(cluster_dev,
        pi_cluster_task(&cl_task, init_data_structures, cluster_dev));

    printf("[harness] work memory reserved: %d bytes; total allocated: %zu bytes\n",
           L1_WORK_MEMORY_SIZE, levio_compat_alloc_total());

    /* ---- main loop ---- */
    t_begin = wall_seconds();

    for (;;) {
        cluster_arguments_t arguments;

        /* Opened before the source read so the measured latency covers the
           whole iteration, frame I/O included. */
        metrics_frame_begin(processed);

        rc = source.next_frame(&source, img_buffer, &t_frame);
        if (rc == SOURCE_END) {
            break;
        }
        if (rc == SOURCE_ERROR) {
            fprintf(stderr, "[harness] source error after %u frames\n", processed);
            break;
        }
        if (opt.paced) {
            metrics_frame_source_ready(t_frame, source.replay_epoch_s);
        }

        traj_writer_set_time(t_frame);
        TRACE_SET(t_stamp, t_frame);

        /* Ground-truth dynamic mask for this frame, if the oracle arm is on.
         * Loaded here rather than inside the source so that both the paced and
         * the fast source get it with no duplication. A missing mask is not an
         * error: it means no dynamic pixels in that frame. */
        if (opt.dynmask) {
            char mpath[1024];
            snprintf(mpath, sizeof(mpath), "%s/dynmask/mask%06d.pgm",
                     opt.data_dir, opt.start_index + (int) processed);
            if (pgm_read(mpath, IMG_WIDTH, IMG_HEIGHT, dyn_mask_buf, IMG_SIZE) == 0) {
                levio_dynamic_mask = dyn_mask_buf;
            } else {
                levio_dynamic_mask = NULL;
            }
        }

        /* The bootstrap frame has no preceding IMU interval, matching the way
           the GAP9 main.c primes the pipeline with a NULL IMU pointer. */
        imu_count = 0;
        if (processed > 0) {
            int imu_rc = source.next_imu_batch(&source, imu_batch, IMU_HEIGHT, &imu_count);
            if (imu_rc == SOURCE_ERROR) {
                fprintf(stderr, "[harness] IMU source error at frame %u\n", processed);
                break;
            }
        }

        LOG_WARNING("\n***** Frame %u (t=%.3f s) *****\n", processed, t_frame);

        arguments.img_buffer = img_buffer;
        arguments.imu_batch  = (imu_count > 0) ? imu_batch : NULL;
        arguments.imu_count  = imu_count;

        pi_cluster_send_task_to_cl(cluster_dev,
            pi_cluster_task(&cl_task, cluster_delegate, &arguments));

        /* Reads the per-frame trace, so it has to run before the flush below
           clears it. */
        metrics_frame_end(t_frame, graph_stats->total_keyframes,
                          graph_stats->total_features);
        levio_trace_flush();

        if (metrics_enabled() &&
            (wall_seconds() - t_last_resource_sample) >= 0.25) {
            metrics_sample_resources();
            t_last_resource_sample = wall_seconds();
        }

        ++processed;
    }

    /* ---- report ---- */
    {
        const double elapsed = wall_seconds() - t_begin;

        printf("\n=== run summary ===\n");
        printf("frames processed   : %u\n", processed);
        printf("frames delivered   : %u\n", source.frames_delivered);
        printf("frames dropped     : %u\n", source.frames_dropped);
        printf("IMU batches        : %u\n", source.imu_batches_delivered);
        printf("IMU short batches  : %u\n", source.imu_short_batches);
        printf("keyframes          : %u\n", traj_writer_keyframe_count());
        printf("poses written      : %u\n", traj_writer_count());
        printf("wall time          : %.2f s\n", elapsed);
        if (elapsed > 0.0 && processed > 0) {
            printf("throughput         : %.2f fps\n", (double) processed / elapsed);
        }
        printf("peak allocation    : %zu bytes\n", levio_compat_alloc_total());
        printf("===================\n");

        if (metrics_enabled()) {
            const char* seq   = opt.data_dir;
            const char* slash = strrchr(seq, '/');
            if (slash != NULL) {
                seq = slash + 1;
            }
            metrics_summary(seq, opt.paced ? "paced" : "fast",
                            processed, source.frames_dropped,
                            traj_writer_keyframe_count(), elapsed,
                            levio_compat_alloc_total());
        }
    }

    pi_cluster_close(cluster_dev);
    metrics_close();
    levio_trace_close();
    traj_writer_close();
    source.close(&source);
    levio_compat_free_all();

    return 0;
}
