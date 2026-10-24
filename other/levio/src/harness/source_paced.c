/*
 * K230-VIO — Mode B: real-time paced PGM replay.
 *
 * A producer thread reads the dataset and publishes frames on its timestamp
 * timeline into a bounded queue; the pipeline consumes from the other end.
 * This exercises the threading, buffering and deadline paths with no sensor
 * hardware attached — "hardware-in-the-loop minus the hardware".
 *
 * Optional fault injection (timestamp jitter, frame drops, IMU sample drops)
 * stresses the assumptions LEVIO makes about its input, in particular the
 * "exactly IMU_HEIGHT samples per frame" invariant.
 *
 * See docs/05-dataset-replay-harness.md section 2.
 *
 * SPDX-License-Identifier: MIT
 */

#include "source.h"
#include "pgm_io.h"
#include "paced_schedule.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

#define QUEUE_CAPACITY 8

typedef struct {
    uint8_t           img[IMG_SIZE];
    imu_measurement_t imu[IMU_HEIGHT];
    int               imu_count;
    double            t_frame;
    int               status;        /* SOURCE_OK / SOURCE_END / SOURCE_ERROR */
} slot_t;

typedef struct {
    replay_config_t cfg;

    double*         timestamps;      /* absolute dataset clock, may be NULL */
    int             ts_count;

    slot_t          queue[QUEUE_CAPACITY];
    int             head;            /* next slot to read  */
    int             tail;            /* next slot to write */
    int             count;

    pthread_mutex_t lock;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;

    pthread_t       producer;
    int             producer_running;
    int             stop_requested;
    double          producer_start_s;

    /* deadline accounting, reported at the end of the run */
    uint32_t        queue_full_events;   /* producer had to wait: consumer too slow */
    uint32_t        queue_empty_events;  /* consumer had to wait: producer too slow */
    double          max_starve_s;

    uint32_t        rng_state;
} paced_ctx_t;

/* ------------------------------------------------------------------ */
/* Deterministic RNG for fault injection — independent of LEVIO's PRNG  */
/* so that enabling faults never perturbs RANSAC reproducibility.       */
/* ------------------------------------------------------------------ */
static uint32_t rng_next(uint32_t* state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static double rng_uniform(uint32_t* state)
{
    return (double) (rng_next(state) >> 8) / (double) (1u << 24);
}

/* ------------------------------------------------------------------ */
/* Clock helpers                                                       */
/* ------------------------------------------------------------------ */
static double now_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

static void sleep_seconds(double s)
{
    struct timespec ts;
    if (s <= 0.0) {
        return;
    }
    ts.tv_sec  = (time_t) s;
    ts.tv_nsec = (long) ((s - (double) ts.tv_sec) * 1e9);
    nanosleep(&ts, NULL);
}

/* ------------------------------------------------------------------ */
/* Producer                                                            */
/* ------------------------------------------------------------------ */
static void* producer_main(void* arg)
{
    sensor_source_t* self = (sensor_source_t*) arg;
    paced_ctx_t*     ctx  = (paced_ctx_t*) self->ctx;

    const double speed = ctx->cfg.speed > 0.0 ? ctx->cfg.speed : 1.0;
    const double period = 1.0 / ((double) CAMERA_FPS * speed);
    double  t_start = ctx->producer_start_s;
    double  first_timestamp = source_frame_time(
        ctx->timestamps, ctx->ts_count, ctx->cfg.start_index);
    int     index      = ctx->cfg.start_index;
    int     imu_index  = ctx->cfg.start_index > 0 ? ctx->cfg.start_index : 1;
    uint8_t imu_raw[IMU_WIDTH * IMU_HEIGHT];
    char    path[1024];

    for (;;) {
        slot_t  staging;
        double  target;
        int     rc;
        int     dropped = 0;

        memset(&staging, 0, sizeof(staging));

        if (index >= ctx->cfg.stop_index) {
            staging.status = SOURCE_END;
        } else {
            /* ---- frame drop injection ---- */
            if (ctx->cfg.drop_rate > 0.0 &&
                rng_uniform(&ctx->rng_state) < ctx->cfg.drop_rate) {
                dropped = 1;
            }

            if (!dropped) {
                snprintf(path, sizeof(path), "%s/images/frame%06d.pgm",
                         ctx->cfg.data_dir, index);
                rc = pgm_read(path, IMG_WIDTH, IMG_HEIGHT, staging.img, IMG_SIZE);
                if (rc != 0) {
                    fprintf(stderr, "[source] %s\n", pgm_last_error());
                    staging.status = SOURCE_ERROR;
                } else {
                    staging.status = SOURCE_OK;
                }
            }

            /* ---- IMU batch ---- */
            if (staging.status == SOURCE_OK) {
                snprintf(path, sizeof(path), "%s/imu/imu%06d.pgm",
                         ctx->cfg.data_dir, imu_index);
                if (pgm_read(path, IMU_WIDTH, IMU_HEIGHT,
                             imu_raw, sizeof(imu_raw)) == 0) {
                    int kept = 0;
                    for (int s = 0; s < IMU_HEIGHT; ++s) {
                        if (ctx->cfg.imu_drop_rate > 0.0 &&
                            rng_uniform(&ctx->rng_state) < ctx->cfg.imu_drop_rate) {
                            continue;          /* drop this sample */
                        }
                        memcpy(&staging.imu[kept],
                               imu_raw + (size_t) s * IMU_WIDTH,
                               IMU_WIDTH);
                        ++kept;
                    }
                    staging.imu_count = kept;
                } else {
                    staging.imu_count = 0;
                }
            }

            staging.t_frame = source_frame_time(ctx->timestamps, ctx->ts_count, index);
            ++index;
            ++imu_index;
        }

        /* ---- pace to the dataset timeline ---- */
        if (staging.status == SOURCE_END) {
            target = now_seconds();
        } else {
            target = source_paced_target(t_start, first_timestamp,
                                         staging.t_frame, speed, period);
        }
        if (ctx->cfg.jitter_ms > 0.0) {
            double j = (rng_uniform(&ctx->rng_state) * 2.0 - 1.0) *
                       ctx->cfg.jitter_ms * 1e-3;
            target += j;
        }
        sleep_seconds(target - now_seconds());

        if (dropped) {
            ++self->frames_dropped;
            continue;                    /* nothing published this period */
        }

        /* ---- publish ---- */
        pthread_mutex_lock(&ctx->lock);
        while (ctx->count == QUEUE_CAPACITY && !ctx->stop_requested) {
            ++ctx->queue_full_events;
            pthread_cond_wait(&ctx->not_full, &ctx->lock);
        }
        if (ctx->stop_requested) {
            pthread_mutex_unlock(&ctx->lock);
            break;
        }
        ctx->queue[ctx->tail] = staging;
        ctx->tail = (ctx->tail + 1) % QUEUE_CAPACITY;
        ++ctx->count;
        pthread_cond_signal(&ctx->not_empty);
        pthread_mutex_unlock(&ctx->lock);

        if (staging.status != SOURCE_OK) {
            break;                       /* END or ERROR published; done */
        }
    }

    return NULL;
}

/* ------------------------------------------------------------------ */
/* Consumer side                                                       */
/* ------------------------------------------------------------------ */

/* The paced source hands over frame and IMU together, so next_frame stages
   the slot and next_imu_batch simply copies out of it. */
static slot_t g_current;                 /* single consumer, no contention */
static int    g_current_valid = 0;

static int paced_next_frame(sensor_source_t* self, uint8_t* img_out, double* t_out)
{
    paced_ctx_t* ctx = (paced_ctx_t*) self->ctx;
    double       wait_start;
    double       waited;

    pthread_mutex_lock(&ctx->lock);
    wait_start = now_seconds();
    while (ctx->count == 0) {
        pthread_cond_wait(&ctx->not_empty, &ctx->lock);
    }
    waited = now_seconds() - wait_start;
    if (waited > 1e-4) {
        ++ctx->queue_empty_events;
        if (waited > ctx->max_starve_s) {
            ctx->max_starve_s = waited;
        }
    }

    g_current = ctx->queue[ctx->head];
    ctx->head = (ctx->head + 1) % QUEUE_CAPACITY;
    --ctx->count;
    pthread_cond_signal(&ctx->not_full);
    pthread_mutex_unlock(&ctx->lock);

    g_current_valid = 1;

    if (g_current.status != SOURCE_OK) {
        return g_current.status;
    }

    memcpy(img_out, g_current.img, IMG_SIZE);
    *t_out = g_current.t_frame;
    ++self->frames_delivered;
    return SOURCE_OK;
}

static int paced_next_imu_batch(sensor_source_t* self, imu_measurement_t* out,
                                int n, int* n_out)
{
    int copy;

    *n_out = 0;
    if (!g_current_valid || g_current.imu_count == 0) {
        return SOURCE_END;
    }

    copy = g_current.imu_count < n ? g_current.imu_count : n;
    memcpy(out, g_current.imu, (size_t) copy * sizeof(imu_measurement_t));
    *n_out = copy;

    if (copy < IMU_HEIGHT) {
        ++self->imu_short_batches;
    }
    ++self->imu_batches_delivered;
    return SOURCE_OK;
}

static void paced_close(sensor_source_t* self)
{
    paced_ctx_t* ctx = (paced_ctx_t*) self->ctx;

    pthread_mutex_lock(&ctx->lock);
    ctx->stop_requested = 1;
    pthread_cond_broadcast(&ctx->not_full);
    pthread_cond_broadcast(&ctx->not_empty);
    pthread_mutex_unlock(&ctx->lock);

    if (ctx->producer_running) {
        pthread_join(ctx->producer, NULL);
    }

    printf("[source] paced stats: queue-full %u, queue-empty %u, max starve %.1f ms\n",
           ctx->queue_full_events, ctx->queue_empty_events, ctx->max_starve_s * 1e3);

    free(ctx->timestamps);
    pthread_mutex_destroy(&ctx->lock);
    pthread_cond_destroy(&ctx->not_empty);
    pthread_cond_destroy(&ctx->not_full);

    free(ctx);
    self->ctx = NULL;
    g_current_valid = 0;
}

int source_paced_open(sensor_source_t* src, const replay_config_t* cfg)
{
    paced_ctx_t* ctx;

    memset(src, 0, sizeof(*src));
    g_current_valid = 0;

    ctx = (paced_ctx_t*) calloc(1, sizeof(paced_ctx_t));
    if (ctx == NULL) {
        return SOURCE_ERROR;
    }

    ctx->cfg       = *cfg;
    ctx->rng_state = cfg->seed ? cfg->seed : 0x12345678u;
    source_load_timestamps(cfg->data_dir, &ctx->timestamps, &ctx->ts_count);

    pthread_mutex_init(&ctx->lock, NULL);
    pthread_cond_init(&ctx->not_empty, NULL);
    pthread_cond_init(&ctx->not_full, NULL);

    src->ctx            = ctx;
    src->next_frame     = paced_next_frame;
    src->next_imu_batch = paced_next_imu_batch;
    src->close          = paced_close;

    ctx->producer_start_s = now_seconds();
    src->replay_epoch_s = ctx->producer_start_s +
        1.0 / (CAMERA_FPS * (cfg->speed > 0.0 ? cfg->speed : 1.0));
    if (pthread_create(&ctx->producer, NULL, producer_main, src) != 0) {
        fprintf(stderr, "[source] cannot start producer thread\n");
        free(ctx);
        src->ctx = NULL;
        return SOURCE_ERROR;
    }
    ctx->producer_running = 1;

    return SOURCE_OK;
}
