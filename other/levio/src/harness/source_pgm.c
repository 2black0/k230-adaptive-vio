/*
 * K230-VIO — Mode A: deterministic PGM replay.
 *
 * Reads frames and IMU batches as fast as possible. No threads, no pacing,
 * no randomness beyond what the caller injects. This is the source used for
 * the phase 2 validation gate, so reproducibility is the priority.
 *
 * SPDX-License-Identifier: MIT
 */

#include "source.h"
#include "pgm_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    replay_config_t cfg;
    int             index;       /* next frame index to read      */
    int             imu_index;   /* next IMU batch index to read  */
    char            img_path[1024];
    char            imu_path[1024];
    uint8_t         imu_raw[IMU_WIDTH * IMU_HEIGHT];
    double*         timestamps;  /* NULL if the dataset has none  */
    int             ts_count;
} pgm_ctx_t;

int source_load_timestamps(const char* data_dir, double** out, int* out_count)
{
    char   path[1024];
    FILE*  f;
    double value;
    int    capacity = 1024;
    int    count = 0;
    double* buf;

    *out = NULL;
    *out_count = 0;

    snprintf(path, sizeof(path), "%s/timestamps.txt", data_dir);
    f = fopen(path, "r");
    if (f == NULL) {
        return SOURCE_END;               /* absent is not an error */
    }

    buf = (double*) malloc((size_t) capacity * sizeof(double));
    if (buf == NULL) {
        fclose(f);
        return SOURCE_ERROR;
    }

    while (fscanf(f, "%lf", &value) == 1) {
        if (count == capacity) {
            double* grown;
            capacity *= 2;
            grown = (double*) realloc(buf, (size_t) capacity * sizeof(double));
            if (grown == NULL) {
                free(buf);
                fclose(f);
                return SOURCE_ERROR;
            }
            buf = grown;
        }
        buf[count++] = value;
    }
    fclose(f);

    *out = buf;
    *out_count = count;
    return SOURCE_OK;
}

/* Absolute dataset time for a frame, or the nominal timeline as a fallback. */
double source_frame_time(const double* timestamps, int count, int index)
{
    if (timestamps != NULL && index >= 0 && index < count) {
        return timestamps[index];
    }
    return (double) index / (double) CAMERA_FPS;
}

int source_autodetect_stop_index(const char* data_dir)
{
    char path[1024];
    int  index = 0;
    FILE* f;

    for (;;) {
        snprintf(path, sizeof(path), "%s/images/frame%06d.pgm", data_dir, index);
        f = fopen(path, "rb");
        if (f == NULL) {
            break;
        }
        fclose(f);
        ++index;
        if (index > 1000000) {           /* guard against runaway loops */
            break;
        }
    }
    return index;
}

static int pgm_next_frame(sensor_source_t* self, uint8_t* img_out, double* t_out)
{
    pgm_ctx_t* ctx = (pgm_ctx_t*) self->ctx;
    int rc;

    if (ctx->index >= ctx->cfg.stop_index) {
        return SOURCE_END;
    }

    snprintf(ctx->img_path, sizeof(ctx->img_path),
             "%s/images/frame%06d.pgm", ctx->cfg.data_dir, ctx->index);

    rc = pgm_read(ctx->img_path, IMG_WIDTH, IMG_HEIGHT, img_out, IMG_SIZE);
    if (rc != 0) {
        fprintf(stderr, "[source] %s\n", pgm_last_error());
        return SOURCE_ERROR;
    }

    *t_out = source_frame_time(ctx->timestamps, ctx->ts_count, ctx->index);

    ++ctx->index;
    ++self->frames_delivered;
    return SOURCE_OK;
}

static int pgm_next_imu_batch(sensor_source_t* self, imu_measurement_t* out,
                              int n, int* n_out)
{
    pgm_ctx_t* ctx = (pgm_ctx_t*) self->ctx;
    int rc;
    size_t bytes;

    *n_out = 0;

    if (n > IMU_HEIGHT) {
        n = IMU_HEIGHT;
    }

    snprintf(ctx->imu_path, sizeof(ctx->imu_path),
             "%s/imu/imu%06d.pgm", ctx->cfg.data_dir, ctx->imu_index);

    rc = pgm_read(ctx->imu_path, IMU_WIDTH, IMU_HEIGHT,
                  ctx->imu_raw, sizeof(ctx->imu_raw));
    if (rc != 0) {
        /* Running out of IMU batches before frames is normal at the tail of a
           dataset; the caller decides whether that ends the run. */
        return SOURCE_END;
    }

    /* Each PGM row is one imu_measurement_t: 7 native-endian floats.
       IMU_WIDTH (28) must equal sizeof(imu_measurement_t); the static
       assertion in main.c enforces that at compile time. */
    bytes = (size_t) n * (size_t) IMU_WIDTH;
    memcpy(out, ctx->imu_raw, bytes);

    ++ctx->imu_index;
    *n_out = n;
    ++self->imu_batches_delivered;
    return SOURCE_OK;
}

static void pgm_close(sensor_source_t* self)
{
    pgm_ctx_t* ctx = (pgm_ctx_t*) self->ctx;
    free(ctx->timestamps);
    free(ctx);
    self->ctx = NULL;
}

int source_pgm_open(sensor_source_t* src, const replay_config_t* cfg)
{
    pgm_ctx_t* ctx;

    memset(src, 0, sizeof(*src));

    ctx = (pgm_ctx_t*) calloc(1, sizeof(pgm_ctx_t));
    if (ctx == NULL) {
        return SOURCE_ERROR;
    }

    ctx->cfg       = *cfg;
    ctx->index     = cfg->start_index;
    /* LEVIO numbers IMU batches from 1 while frames start at 0: batch k holds
       the samples between frame k-1 and frame k (see the GAP9 README). */
    ctx->imu_index = cfg->start_index > 0 ? cfg->start_index : 1;

    if (source_load_timestamps(cfg->data_dir, &ctx->timestamps, &ctx->ts_count) == SOURCE_OK) {
        printf("[source] using %d absolute timestamps from the dataset\n", ctx->ts_count);
    }

    src->ctx            = ctx;
    src->next_frame     = pgm_next_frame;
    src->next_imu_batch = pgm_next_imu_batch;
    src->close          = pgm_close;

    return SOURCE_OK;
}
