/*
 * K230-VIO — PGM reader/writer.
 * SPDX-License-Identifier: MIT
 */

#include "pgm_io.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

static char g_err[256] = "";

const char* pgm_last_error(void) { return g_err; }

static void set_err(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_err, sizeof(g_err), fmt, ap);
    va_end(ap);
}

/*
 * Reads the next integer from a PGM header, skipping whitespace and
 * '#' comment lines (cv2.imwrite does not emit them, but hand-made or
 * third-party datasets often do).
 */
static int read_header_int(FILE* f, int* out)
{
    int c;
    int value = 0;
    int digits = 0;

    for (;;) {
        c = fgetc(f);
        if (c == EOF) {
            return -1;
        }
        if (c == '#') {
            while (c != '\n' && c != EOF) {
                c = fgetc(f);
            }
            continue;
        }
        if (isspace(c)) {
            continue;
        }
        break;
    }

    while (isdigit(c)) {
        value = value * 10 + (c - '0');
        ++digits;
        c = fgetc(f);
    }
    if (digits == 0) {
        return -1;
    }
    /* One whitespace character terminates the header field; the binary
       payload starts immediately after the maxval field's separator. */
    *out = value;
    return 0;
}

int pgm_read(const char* path, int exp_width, int exp_height,
             uint8_t* out, size_t out_size)
{
    FILE* f = NULL;
    char magic[3] = {0};
    int width = 0, height = 0, maxval = 0;
    size_t expected;
    size_t got;

    f = fopen(path, "rb");
    if (f == NULL) {
        set_err("cannot open '%s'", path);
        return -1;
    }

    if (fread(magic, 1, 2, f) != 2 || magic[0] != 'P' || magic[1] != '5') {
        set_err("'%s' is not a binary PGM (P5)", path);
        fclose(f);
        return -2;
    }

    if (read_header_int(f, &width)  != 0 ||
        read_header_int(f, &height) != 0 ||
        read_header_int(f, &maxval) != 0) {
        set_err("malformed PGM header in '%s'", path);
        fclose(f);
        return -3;
    }

    if (maxval > 255) {
        set_err("'%s' has maxval %d; only 8-bit PGM is supported", path, maxval);
        fclose(f);
        return -4;
    }

    if (width != exp_width || height != exp_height) {
        set_err("'%s' is %dx%d, expected %dx%d", path, width, height, exp_width, exp_height);
        fclose(f);
        return -5;
    }

    expected = (size_t) width * (size_t) height;
    if (expected > out_size) {
        set_err("buffer too small for '%s' (%zu > %zu)", path, expected, out_size);
        fclose(f);
        return -6;
    }

    got = fread(out, 1, expected, f);
    fclose(f);

    if (got != expected) {
        set_err("'%s' truncated: read %zu of %zu bytes", path, got, expected);
        return -7;
    }

    return 0;
}

int pgm_write(const char* path, int width, int height, const uint8_t* data)
{
    FILE* f = fopen(path, "wb");
    if (f == NULL) {
        set_err("cannot create '%s'", path);
        return -1;
    }
    fprintf(f, "P5\n%d %d\n255\n", width, height);
    if (fwrite(data, 1, (size_t) width * (size_t) height, f) !=
        (size_t) width * (size_t) height) {
        set_err("short write to '%s'", path);
        fclose(f);
        return -2;
    }
    fclose(f);
    return 0;
}
