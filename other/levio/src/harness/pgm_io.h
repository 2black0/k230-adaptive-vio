/*
 * K230-VIO — PGM reader/writer replacing gaplib/ImgIO.h.
 *
 * LEVIO stores both camera frames and IMU batches as binary PGM (P5):
 *   - frames: IMG_WIDTH x IMG_HEIGHT, 8-bit grayscale
 *   - IMU:    28 x 10 raw bytes, i.e. 10 imu_measurement_t records of
 *             7 native-endian floats each (see docs/05, section 3.3)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __PGM_IO_H__
#define __PGM_IO_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reads a binary PGM (P5) file into a caller-supplied buffer.
 *
 * Verifies that the file dimensions match the expected ones, so a mismatched
 * dataset fails loudly instead of silently corrupting the pipeline.
 *
 * @param path       File to read.
 * @param exp_width  Expected width in pixels.
 * @param exp_height Expected height in pixels.
 * @param out        Destination buffer, at least exp_width*exp_height bytes.
 * @param out_size   Size of the destination buffer, for bounds checking.
 * @return 0 on success, negative on error.
 */
int pgm_read(const char* path, int exp_width, int exp_height,
             uint8_t* out, size_t out_size);

/**
 * @brief Writes a binary PGM (P5) file. Used to dump frames captured live.
 * @return 0 on success, negative on error.
 */
int pgm_write(const char* path, int width, int height, const uint8_t* data);

/**
 * @brief Returns a human-readable message for the last pgm_* failure.
 */
const char* pgm_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* __PGM_IO_H__ */
