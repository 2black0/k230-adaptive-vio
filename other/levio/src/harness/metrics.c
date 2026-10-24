/*
 * k230-levio — structured run artifacts. See metrics.h.
 *
 * SPDX-License-Identifier: MIT
 */

#include "metrics.h"

#include "trace.h"
#include "traj_writer.h"
#include "levio_hooks.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

/* ------------------------------------------------------------------ */
/* Stage timings                                                       */
/* ------------------------------------------------------------------ */

enum {
    STAGE_EXTRACT,
    STAGE_MATCH_WORLD,
    STAGE_MATCH_FRAMES,
    STAGE_EPNP,
    STAGE_ESS,
    STAGE_TRIANG,
    STAGE_OPTIMIZER,
    STAGE_IMU,
    STAGE_COUNT
};

static const char* const STAGE_NAMES[STAGE_COUNT] = {
    "extract", "match_world", "match_frames", "epnp",
    "ess", "triang", "optimizer", "imu"
};

static const char* const STAGE_COLS[STAGE_COUNT] = {
    "t_extract_us", "t_match_world_us", "t_match_frames_us", "t_epnp_us",
    "t_ess_us", "t_triang_us", "t_optimizer_us", "t_imu_us"
};

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

static FILE* g_framelog = NULL;
static FILE* g_timing   = NULL;
static FILE* g_resource = NULL;
static FILE* g_realtime = NULL;
static int   g_enabled  = 0;
static char  g_log_dir[512];

static uint32_t g_frame_idx   = 0;
static double   g_t_dataset   = 0.0;
static double   g_frame_start = 0.0;   /* wall clock, start of this frame   */
static double   g_prev_start  = 0.0;   /* wall clock, start of previous frame */
static double   g_interval_us = 0.0;   /* wall delta between the two above  */
static double   g_stage_us[STAGE_COUNT];
static double   g_realtime_epoch = 0.0;
static double   g_realtime_first_timestamp = 0.0;
static double   g_realtime_arrival = 0.0;
static double   g_realtime_start = 0.0;
static int      g_realtime_frame_active = 0;

/* Run-level accumulators, filled one entry per frame. Grown geometrically;
   a full EuRoC sequence is a few thousand entries, i.e. tens of KB. */
static double*  g_latency_us  = NULL;
static double*  g_fps         = NULL;
static uint32_t g_sample_count = 0;
static uint32_t g_sample_cap   = 0;

static double   g_cpu_pct_sum  = 0.0;
static uint32_t g_cpu_samples  = 0;
static double   g_rss_peak_mb  = 0.0;
static double   g_run_start    = 0.0;

/* /proc sampling bookkeeping. */
static long                g_clk_tck       = 100;
static double              g_mem_total_kb   = 0.0;
static double              g_res_last_wall = 0.0;
static unsigned long long  g_res_last_ticks = 0;
static int                 g_res_have_last  = 0;

static double wall_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

/* ------------------------------------------------------------------ */
/* /proc helpers                                                       */
/* ------------------------------------------------------------------ */

/*
 * /proc/self/stat: fields 14 and 15 are utime/stime, field 20 is num_threads.
 * Field 2 (comm) can contain spaces and parentheses, so parsing starts after
 * the LAST ')' rather than by splitting on whitespace.
 */
static int read_cpu_ticks(unsigned long long* ticks, long* threads)
{
    char  buf[1024];
    char  state = 0;
    char* p;
    unsigned long long utime = 0, stime = 0;
    long  num_threads = 0;
    FILE* f;
    int   n;

    f = fopen("/proc/self/stat", "r");
    if (f == NULL) {
        return -1;
    }
    if (fgets(buf, sizeof(buf), f) == NULL) {
        fclose(f);
        return -1;
    }
    fclose(f);

    p = strrchr(buf, ')');
    if (p == NULL || p[1] == '\0') {
        return -1;
    }
    p += 2;                             /* step over ") " */

    /* Fields are whitespace separated, so the ten between the state and utime
       (ppid .. cmajflt) and the four between stime and num_threads
       (cutime .. nice) are skipped as bare tokens. Assignment suppression with
       a length modifier (%*lu) is a GNU extension GCC warns about. */
    n = sscanf(p,
               "%c %*s %*s %*s %*s %*s %*s %*s %*s %*s %*s "
               "%llu %llu %*s %*s %*s %*s %ld",
               &state, &utime, &stime, &num_threads);
    if (n != 4) {
        return -1;
    }
    (void) state;

    *ticks   = utime + stime;
    *threads = num_threads;
    return 0;
}

/* field 2 of /proc/self/statm is the resident set size in pages. */
static long read_rss_pages(void)
{
    long  size = 0, resident = 0;
    FILE* f = fopen("/proc/self/statm", "r");

    if (f == NULL) {
        return -1;
    }
    if (fscanf(f, "%ld %ld", &size, &resident) != 2) {
        resident = -1;
    }
    fclose(f);
    return resident;
}

static double read_mem_total_kb(void)
{
    char   key[64];
    double value = 0.0;
    char   unit[32];
    FILE*  f = fopen("/proc/meminfo", "r");

    if (f == NULL) {
        return 0.0;
    }
    while (fscanf(f, "%63s %lf %31s", key, &value, unit) == 3) {
        if (strcmp(key, "MemTotal:") == 0) {
            fclose(f);
            return value;
        }
    }
    fclose(f);
    return 0.0;
}

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
/* ------------------------------------------------------------------ */

static int cmp_double(const void* a, const void* b)
{
    const double da = *(const double*) a;
    const double db = *(const double*) b;
    return (da > db) - (da < db);
}

/* Nearest-rank percentile over an already sorted array. p in [0,1]. */
static double percentile(const double* sorted, uint32_t n, double p)
{
    uint32_t idx;

    if (n == 0) {
        return 0.0;
    }
    idx = (uint32_t) (p * (double) (n - 1) + 0.5);
    if (idx >= n) {
        idx = n - 1;
    }
    return sorted[idx];
}

static double mean_of(const double* v, uint32_t n)
{
    double sum = 0.0;

    if (n == 0) {
        return 0.0;
    }
    for (uint32_t i = 0; i < n; ++i) {
        sum += v[i];
    }
    return sum / (double) n;
}

static void push_sample(double latency_us, double fps)
{
    if (g_sample_count == g_sample_cap) {
        uint32_t cap = g_sample_cap ? g_sample_cap * 2 : 1024;
        double*  lat = (double*) realloc(g_latency_us, (size_t) cap * sizeof(double));
        double*  fps_new;

        if (lat == NULL) {
            return;                      /* keep going; just stop recording */
        }
        g_latency_us = lat;
        fps_new = (double*) realloc(g_fps, (size_t) cap * sizeof(double));
        if (fps_new == NULL) {
            return;
        }
        g_fps = fps_new;
        g_sample_cap = cap;
    }
    g_latency_us[g_sample_count] = latency_us;
    g_fps[g_sample_count]        = fps;
    ++g_sample_count;
}

/* ------------------------------------------------------------------ */
/* Open / close                                                        */
/* ------------------------------------------------------------------ */

int metrics_open(const char* log_dir)
{
    char path[600];

    if (log_dir == NULL || log_dir[0] == '\0') {
        return 0;                        /* metrics stay disabled */
    }

    if (mkdir(log_dir, 0777) != 0 && errno != EEXIST) {
        fprintf(stderr, "[metrics] cannot create '%s': %s\n",
                log_dir, strerror(errno));
        return -1;
    }

    snprintf(g_log_dir, sizeof(g_log_dir), "%s", log_dir);

    snprintf(path, sizeof(path), "%s/framelog.csv", log_dir);
    g_framelog = fopen(path, "w");
    snprintf(path, sizeof(path), "%s/timing.csv", log_dir);
    g_timing = fopen(path, "w");
    snprintf(path, sizeof(path), "%s/resource.csv", log_dir);
    g_resource = fopen(path, "w");
    snprintf(path, sizeof(path), "%s/realtime.csv", log_dir);
    g_realtime = fopen(path, "w");

    if (g_framelog == NULL || g_timing == NULL || g_resource == NULL ||
        g_realtime == NULL) {
        fprintf(stderr, "[metrics] cannot open the CSV files under '%s'\n", log_dir);
        metrics_close();
        return -1;
    }

    /* Full buffering: the estimator writes one row per frame, and an
       unbuffered write would put the board's storage latency inside the
       measurement it is supposed to be recording. */
    {
        static char framelog_buf[64 * 1024];
        static char timing_buf[32 * 1024];
        static char resource_buf[16 * 1024];
        static char realtime_buf[16 * 1024];
        setvbuf(g_framelog, framelog_buf, _IOFBF, sizeof(framelog_buf));
        setvbuf(g_timing,   timing_buf,   _IOFBF, sizeof(timing_buf));
        setvbuf(g_resource, resource_buf, _IOFBF, sizeof(resource_buf));
        setvbuf(g_realtime, realtime_buf, _IOFBF, sizeof(realtime_buf));
    }

    /* Header comments record the definitions that are easy to get wrong when
       comparing across projects. */
    fprintf(g_framelog,
        "# frame_interval_us   = wall-clock delta between consecutive frame starts"
        " (fps = 1e6 / this)\n"
        "# pipeline_latency_us = wall-clock for one frame, from source read to pose emitted\n"
        "# timestamp_ns        = frame timestamp on the dataset clock\n"
        "# cum_keyframes/cum_landmarks = running totals LEVIO keeps in graph_stats;\n"
        "#                       LEVIO holds a fixed 8-keyframe window, so these are\n"
        "#                       cumulative counts, not current map occupancy\n"
        "# tracking_state      = 1 when a pose was emitted for this frame, 0 otherwise\n");

    fprintf(g_timing, "frame_idx,timestamp_ns,frame_interval_us,pipeline_latency_us\n");
    fprintf(g_realtime, "frame_idx,timestamp_s,arrival_s,start_s,finish_s\n");

    fprintf(g_framelog,
        "frame_idx,timestamp_ns,frame_interval_us,pipeline_latency_us,"
        "tx,ty,tz,qx,qy,qz,qw,"
        "is_keyframe,num_features,cum_keyframes,cum_landmarks,tracking_state,"
        "epnp_used,ess_inliers,scale,parallax,zupt_fired");
    for (int i = 0; i < STAGE_COUNT; ++i) {
        fprintf(g_framelog, ",%s", STAGE_COLS[i]);
    }
    fputc('\n', g_framelog);

    /* Same columns as k230-eqvio/scripts/board_run.sh, so one parser reads
       both projects. */
    fprintf(g_resource, "elapsed_sec,cpu_pct,mem_pct,rss_mb,threads\n");

    /* Power cannot be measured in software on the K230: /sys/class/power_supply
       is empty and there is no powercap or cpufreq. The file exists so a run
       directory has the same shape as the EqVIO campaign, and is filled in by
       hand from a USB-C inline meter (docs/21). */
    snprintf(path, sizeof(path), "%s/power.csv", log_dir);
    {
        FILE* f = fopen(path, "w");
        if (f != NULL) {
            fprintf(f, "phase,t_iso,voltage_v,current_a,power_w,run_ref,note\n");
            fclose(f);
        }
    }

    g_clk_tck      = sysconf(_SC_CLK_TCK);
    if (g_clk_tck <= 0) {
        g_clk_tck = 100;
    }
    g_mem_total_kb = read_mem_total_kb();
    g_run_start    = wall_seconds();
    g_enabled      = 1;

    return 0;
}

int metrics_enabled(void) { return g_enabled; }

void metrics_close(void)
{
    if (g_framelog != NULL) { fclose(g_framelog); g_framelog = NULL; }
    if (g_timing   != NULL) { fclose(g_timing);   g_timing   = NULL; }
    if (g_resource != NULL) { fclose(g_resource); g_resource = NULL; }
    if (g_realtime != NULL) { fclose(g_realtime); g_realtime = NULL; }

    free(g_latency_us); g_latency_us = NULL;
    free(g_fps);        g_fps        = NULL;
    g_sample_count = 0;
    g_sample_cap   = 0;
    g_realtime_epoch = 0.0;
    g_realtime_first_timestamp = 0.0;
    g_realtime_frame_active = 0;
    g_enabled      = 0;
}

/* ------------------------------------------------------------------ */
/* Per-frame                                                           */
/* ------------------------------------------------------------------ */

void metrics_add_stage_ns(const char* name, uint32_t ns)
{
    if (!g_enabled || name == NULL) {
        return;
    }
    for (int i = 0; i < STAGE_COUNT; ++i) {
        if (strcmp(name, STAGE_NAMES[i]) == 0) {
            /* Accumulate: a stage that runs twice in one frame (the two
               feature-match-world sites) should report its total cost. */
            g_stage_us[i] += (double) ns * 1e-3;
            return;
        }
    }
}

void levio_hook_stage(const char* name, uint32_t ns)
{
    metrics_add_stage_ns(name, ns);
}

void metrics_frame_begin(uint32_t frame_idx)
{
    if (!g_enabled) {
        return;
    }

    g_frame_idx   = frame_idx;
    g_frame_start = wall_seconds();

    g_interval_us = (frame_idx == 0 || g_prev_start <= 0.0)
                        ? 0.0
                        : (g_frame_start - g_prev_start) * 1e6;
    g_prev_start  = g_frame_start;

    memset(g_stage_us, 0, sizeof(g_stage_us));
}

void metrics_frame_source_ready(double t_dataset, double replay_epoch_s)
{
    double now;

    if (!g_enabled || g_realtime == NULL) {
        return;
    }

    now = wall_seconds();
    if (g_realtime_epoch == 0.0) {
        g_realtime_epoch = replay_epoch_s > 0.0 ? replay_epoch_s : now;
        g_realtime_first_timestamp = t_dataset;
    }
    g_realtime_arrival = t_dataset - g_realtime_first_timestamp;
    g_realtime_start = now - g_realtime_epoch;
    g_realtime_frame_active = 1;
}

void metrics_frame_end(double t_dataset, uint32_t n_kfs, uint32_t n_lms)
{
    double   t_pose[3];
    double   q_pose[4];
    int      have_pose;
    double   latency_us;
    uint64_t ts_ns;

    if (!g_enabled) {
        return;
    }

    g_t_dataset = t_dataset;
    latency_us  = (wall_seconds() - g_frame_start) * 1e6;
    ts_ns       = (uint64_t) (t_dataset * 1e9);

    if (g_realtime_frame_active) {
        const double finish = wall_seconds() - g_realtime_epoch;
        fprintf(g_realtime, "%u,%.9f,%.9f,%.9f,%.9f\n",
                g_frame_idx, t_dataset, g_realtime_arrival,
                g_realtime_start, finish);
        g_realtime_frame_active = 0;
    }

    if (g_timing != NULL) {
        fprintf(g_timing, "%u,%llu,%.1f,%.1f\n",
                g_frame_idx, (unsigned long long) ts_ns,
                g_interval_us, latency_us);
    }

    if (g_framelog == NULL) {
        return;
    }

    have_pose = traj_writer_last_pose(t_pose, q_pose);

    fprintf(g_framelog, "%u,%llu,%.1f,%.1f",
            g_frame_idx, (unsigned long long) ts_ns, g_interval_us, latency_us);

    if (have_pose) {
        fprintf(g_framelog, ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                t_pose[0], t_pose[1], t_pose[2],
                q_pose[0], q_pose[1], q_pose[2], q_pose[3]);
    } else {
        /* Seven empty fields: this frame produced no usable pose. */
        fprintf(g_framelog, ",,,,,,");
    }

    fprintf(g_framelog, ",%u,%u,%u,%u,%u,%u,%u,%.6g,%.6g,%u",
            (unsigned) levio_trace.is_keyframe,
            (unsigned) levio_trace.n_kpts,
            (unsigned) n_kfs,
            (unsigned) n_lms,
            (unsigned) (have_pose ? 1 : 0),     /* tracking_state */
            (unsigned) levio_trace.epnp_used,
            (unsigned) levio_trace.ess_inliers,
            (double) levio_trace.scale,
            (double) levio_trace.parallax,
            (unsigned) levio_trace.zupt_fired);

    for (int i = 0; i < STAGE_COUNT; ++i) {
        fprintf(g_framelog, ",%.1f", g_stage_us[i]);
    }
    fputc('\n', g_framelog);

    if (latency_us > 0.0 && latency_us < 1e9) {
        const double fps = (g_interval_us > 0.0) ? 1e6 / g_interval_us : 0.0;
        push_sample(latency_us, fps);
    }
}

void metrics_sample_resources(void)
{
    unsigned long long ticks   = 0;
    long               threads = 0;
    long               rss_pages;
    double             now, elapsed, wall_delta;
    double             cpu_pct = 0.0, mem_pct = 0.0, rss_mb = 0.0;

    if (!g_enabled || g_resource == NULL) {
        return;
    }
    if (read_cpu_ticks(&ticks, &threads) != 0) {
        return;
    }
    now       = wall_seconds();
    elapsed   = now - g_run_start;
    wall_delta = now - g_res_last_wall;

    if (g_res_have_last && wall_delta > 1e-6) {
        const double dticks = (double) (ticks - g_res_last_ticks);
        cpu_pct = dticks / (double) g_clk_tck / wall_delta * 100.0;
        if (cpu_pct > 100.0 * (double) sysconf(_SC_NPROCESSORS_ONLN)) {
            cpu_pct = 100.0 * (double) sysconf(_SC_NPROCESSORS_ONLN);
        }
    }
    g_res_last_wall  = now;
    g_res_last_ticks = ticks;
    g_res_have_last  = 1;

    rss_pages = read_rss_pages();
    if (rss_pages > 0) {
        const double page_kb = (double) sysconf(_SC_PAGESIZE) / 1024.0;
        rss_mb = (double) rss_pages * page_kb / 1024.0;
        if (g_mem_total_kb > 0.0) {
            mem_pct = (double) rss_pages * page_kb / g_mem_total_kb * 100.0;
        }
    }
    if (rss_mb > g_rss_peak_mb) {
        g_rss_peak_mb = rss_mb;
    }

    fprintf(g_resource, "%.3f,%.2f,%.3f,%.3f,%ld\n",
            elapsed, cpu_pct, mem_pct, rss_mb, threads);

    g_cpu_pct_sum += cpu_pct;
    ++g_cpu_samples;
}

/* ------------------------------------------------------------------ */
/* Summary                                                             */
/* ------------------------------------------------------------------ */

void metrics_summary(const char* sequence, const char* mode,
                     uint32_t frames, uint32_t frames_dropped,
                     uint32_t keyframes, double wall_time_s,
                     size_t alloc_peak_bytes)
{
    char   path[600];
    FILE*  f;
    double fps_mean = 0.0, fps_p95 = 0.0;
    double lat_mean = 0.0, lat_p95 = 0.0;
    double cpu_mean = 0.0;

    if (!g_enabled) {
        return;
    }

    if (g_sample_count > 0) {
        qsort(g_latency_us, g_sample_count, sizeof(double), cmp_double);
        qsort(g_fps,        g_sample_count, sizeof(double), cmp_double);
        lat_mean = mean_of(g_latency_us, g_sample_count);
        lat_p95  = percentile(g_latency_us, g_sample_count, 0.95);
        fps_mean = mean_of(g_fps, g_sample_count);
        fps_p95  = percentile(g_fps, g_sample_count, 0.95);
    }
    if (g_cpu_samples > 0) {
        cpu_mean = g_cpu_pct_sum / (double) g_cpu_samples;
    }

    snprintf(path, sizeof(path), "%s/summary.csv", g_log_dir);
    f = fopen(path, "w");
    if (f != NULL) {
        fprintf(f, "metric,value\n");
        fprintf(f, "sequence,%s\n",        sequence ? sequence : "");
        fprintf(f, "mode,%s\n",            mode ? mode : "");
        fprintf(f, "frames,%u\n",          frames);
        fprintf(f, "frames_dropped,%u\n",  frames_dropped);
        fprintf(f, "keyframes,%u\n",       keyframes);
        fprintf(f, "wall_time_s,%.3f\n",   wall_time_s);
        /* Two fps figures on purpose: throughput_fps is frames/wall-clock,
           while fps_mean averages 1/frame_interval over the frames. The
           second is higher because it excludes start-up and gives a slow
           frame no more weight than a fast one. Compare like with like. */
        if (wall_time_s > 0.0 && frames > 0) {
            fprintf(f, "throughput_fps,%.3f\n", (double) frames / wall_time_s);
        }
        fprintf(f, "fps_mean,%.3f\n",      fps_mean);
        fprintf(f, "fps_p95,%.3f\n",       fps_p95);
        fprintf(f, "latency_mean_us,%.1f\n", lat_mean);
        fprintf(f, "latency_p95_us,%.1f\n",  lat_p95);
        fprintf(f, "cpu_pct_mean,%.2f\n",  cpu_mean);
        fprintf(f, "rss_peak_mb,%.3f\n",   g_rss_peak_mb);
        fprintf(f, "alloc_peak_bytes,%zu\n", alloc_peak_bytes);
        fclose(f);
    }

    /* Kept short: the runner prints one line per sequence, and the full
       estimator log already goes to run.log. */
    printf("[metrics] fps mean/p95      : %.2f / %.2f\n", fps_mean, fps_p95);
    printf("[metrics] latency mean/p95  : %.2f / %.2f ms\n", lat_mean / 1e3, lat_p95 / 1e3);
    printf("[metrics] cpu mean, rss peak: %.1f %%, %.1f MB\n", cpu_mean, g_rss_peak_mb);
    printf("[metrics] artifacts         : %s\n", g_log_dir);
}
