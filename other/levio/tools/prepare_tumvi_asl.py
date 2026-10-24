#!/usr/bin/env python3
# k230-levio — TUM-VI room sequences (ASL layout) -> LEVIO PGM format.
#
# The TUM-VI source under ~/dataset/TUM-VI/dataset-room{1..6}_512_16 has:
#   mav0/cam0/data/*.png + mav0/cam0/data.csv   512x512 16-bit, 20 Hz
#   mav0/imu0/data.csv                          ~200 Hz
#   mav0/mocap0/data.csv                        motion capture ground truth
#   dso/camchain.yaml                           the ONLY calibration
# and no mav0/cam0/sensor.yaml, unlike EuRoC.
#
# Three conventions that are easy to get wrong, all verified against the data:
#
#   1. TUM-VI ships an equidistant (fisheye) distortion model, not EuRoC's
#      radial-tangential one, so undistortion goes through cv2.fisheye. The four
#      coefficients are tiny (k1 = 0.0035, i.e. ~0.12 % at the frame edge), so
#      this is nearly a no-op -- but it is the correct model and it costs one
#      remap per frame.
#
#   2. imu0/data.csv is ordered timestamp, GYRO(3), ACCEL(3). LEVIO's
#      imu_measurement_t is {t_stamp, acc(3), gyro(3)} -- accelerometer first.
#      The columns are reordered on write.
#
#   3. dso/camchain.yaml cam0 T_cam_imu is the camera-to-IMU transform, and its
#      rotation is the transpose of IMU_TO_CAM_R in config.h. The ground truth
#      is the body pose, so it is composed with T_BS to land in the camera
#      frame LEVIO estimates: T_wc = T_wb @ T_BS.
#
# Geometry: 512x512 -> 256x256, no crop. The 16-bit frames are reduced to 8-bit
# with >> 8 AFTER the resize, matching the EqVIO conversion of the same dataset,
# so the two roots stay directly comparable.
#
# Usage:
#   python3 tools/prepare_tumvi_asl.py <tumvi_root>/dataset-room1_512_16 \
#       --out "$HOME/dataset/LEVIO-PGM-TUMVI/room1"
#
# SPDX-License-Identifier: MIT

import argparse
import os
import re
import struct
import sys

import cv2
import numpy as np

IMU_SAMPLES_PER_BATCH = 10        # 200 Hz IMU / 20 Hz camera
OUT_W, OUT_H = 256, 256

FLOAT_RE = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")
KEY_RE = re.compile(r"^  ([A-Za-z_][A-Za-z0-9_]*):")


def load_camchain_cam0(path):
    """
    Pulls the cam0 entry out of a TUM-VI dso/camchain.yaml.

    Hand-parsed rather than with PyYAML, so the script needs nothing beyond
    numpy and OpenCV (same choice as prepare_euroc_asl.py). The file is a
    two-level mapping whose only awkward part is T_cam_imu, a 4x4 matrix
    written as a YAML sequence of rows -- stripped to bare numbers here.
    """
    with open(path) as f:
        text = f.read()

    lines = text.splitlines()
    start = None
    for i, line in enumerate(lines):
        if line.startswith("cam0:"):
            start = i + 1
            break
    if start is None:
        raise ValueError("no cam0 entry in %s" % path)

    # The cam0 block ends at the next top-level key (cam1:, ...).
    end = len(lines)
    for i in range(start, len(lines)):
        if lines[i] and not lines[i][0].isspace() and lines[i].rstrip().endswith(":"):
            end = i
            break

    # Split the block into key -> value text, keeping multi-line values together.
    fields = {}
    key = None
    buf = []
    for line in lines[start:end]:
        m = KEY_RE.match(line)
        if m:
            if key is not None:
                fields[key] = "\n".join(buf)
            key = m.group(1)
            buf = [line[m.end():]]
        elif key is not None:
            buf.append(line)
    if key is not None:
        fields[key] = "\n".join(buf)

    def numbers(name, expected):
        if name not in fields:
            raise ValueError("cam0 has no '%s'" % name)
        vals = [float(v) for v in FLOAT_RE.findall(fields[name])]
        if len(vals) != expected:
            raise ValueError("cam0 '%s': expected %d numbers, got %d"
                             % (name, expected, len(vals)))
        return vals

    return {
        "intrinsics": numbers("intrinsics", 4),
        "distortion": numbers("distortion_coeffs", 4),
        "T_cam_imu": np.array(numbers("T_cam_imu", 16)).reshape(4, 4),
        "resolution": numbers("resolution", 2),
    }


def quat_from_R(R):
    """Rotation matrix to (x, y, z, w), Shepperd's method."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def R_from_quat_wxyz(qw, qx, qy, qz):
    n = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ])


def write_pgm(path, arr):
    h, w = arr.shape
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(np.ascontiguousarray(arr, dtype=np.uint8).tobytes())


def main():
    ap = argparse.ArgumentParser(
        description="Convert a TUM-VI room sequence to the LEVIO PGM format")
    ap.add_argument("sequence", help="sequence directory, e.g. .../dataset-room1_512_16")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--width", type=int, default=OUT_W, help="working width (default 256)")
    ap.add_argument("--height", type=int, default=OUT_H, help="working height (default 256)")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0 = all)")
    ap.add_argument("--skip-images", action="store_true",
                    help="only regenerate IMU and ground truth")
    args = ap.parse_args()

    mav = os.path.join(args.sequence, "mav0")
    if not os.path.isdir(mav):
        sys.exit("error: %s does not look like a TUM-VI sequence (no mav0/)" % args.sequence)

    camchain_path = os.path.join(args.sequence, "dso", "camchain.yaml")
    if not os.path.isfile(camchain_path):
        sys.exit("error: no dso/camchain.yaml under %s" % args.sequence)
    calib = load_camchain_cam0(camchain_path)

    src_w, src_h = int(calib["resolution"][0]), int(calib["resolution"][1])
    fu, fv, cu, cv_ = calib["intrinsics"]
    K = np.array([[fu, 0.0, cu], [0.0, fv, cv_], [0.0, 0.0, 1.0]])
    D = np.array(calib["distortion"], dtype=np.float64).reshape(4, 1)

    # Intrinsics for the working resolution: the undistorted image is a pinhole
    # projection with the same K, and resizing scales f and c by the axis ratio.
    sx, sy = args.width / src_w, args.height / src_h
    print("calibration   : %dx%d -> %dx%d (sx=%.4f sy=%.4f)" % (src_w, src_h, args.width, args.height, sx, sy))
    print("working K     : fx=%.6f fy=%.6f cx=%.6f cy=%.6f"
          % (fu * sx, fv * sy, cu * sx, cv_ * sy))

    img_dir = os.path.join(args.out, "images")
    imu_dir = os.path.join(args.out, "imu")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)

    # ---- camera index ----
    cam_rows = []
    with open(os.path.join(mav, "cam0", "data.csv")) as f:
        for line in f:
            if line.startswith("#"):
                continue
            ts, name = line.strip().split(",")
            cam_rows.append((int(ts), name))
    if args.max_frames > 0:
        cam_rows = cam_rows[:args.max_frames]
    cam_ts = np.array([r[0] for r in cam_rows], dtype=np.int64)
    print("camera frames : %d" % len(cam_rows))

    # ---- IMU ----
    imu = np.loadtxt(os.path.join(mav, "imu0", "data.csv"), delimiter=",", comments="#")
    imu_ts = imu[:, 0].astype(np.int64)
    gyro = imu[:, 1:4]
    acc = imu[:, 4:7]
    print("IMU samples   : %d" % len(imu_ts))

    # ---- images ----
    if not args.skip_images:
        # Rectification map, built once with the Knew = K convention: the output
        # is a plain pinhole image with the SAME intrinsics, sampling only the
        # source radius that a pinhole of that focal length can represent. For
        # this camera that is ~208 px of the 256 px half-width, so the corner of
        # the output still lands inside the source and there is no black border.
        #
        # Built explicitly and applied with remap rather than through
        # cv2.fisheye.undistortImage, which on this OpenCV returns a much darker
        # image (mean 2594 against the source's 8522, max clipped to 10894):
        # the explicit map is both correct and inspectable.
        map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), K, (src_w, src_h), cv2.CV_32FC1)

        for i, (_, name) in enumerate(cam_rows):
            raw = cv2.imread(os.path.join(mav, "cam0", "data", name), cv2.IMREAD_UNCHANGED)
            if raw is None:
                sys.exit("error: cannot read %s" % name)
            undistorted = cv2.remap(raw.astype(np.float32), map_x, map_y,
                                    interpolation=cv2.INTER_LINEAR)
            resized = cv2.resize(undistorted, (args.width, args.height),
                                 interpolation=cv2.INTER_AREA)
            # 16-bit -> 8-bit with >>8, matching the EqVIO conversion of the same
            # dataset so the two roots stay comparable.
            frame8 = np.clip(resized, 0, 65535) / 256.0
            write_pgm(os.path.join(img_dir, "frame%06d.pgm" % i), frame8)
            if (i + 1) % 500 == 0:
                print("  images %d/%d" % (i + 1, len(cam_rows)))
        print("wrote %d images" % len(cam_rows))

    # ---- IMU batches ----
    # Batch k covers the interval (t_cam[k-1], t_cam[k]], matching the indexing
    # in src/harness/source_pgm.c where batch numbering starts at 1.
    short = 0
    long_ = 0
    for k in range(1, len(cam_rows)):
        lo, hi = cam_ts[k - 1], cam_ts[k]
        sel = np.where((imu_ts > lo) & (imu_ts <= hi))[0]

        if len(sel) < IMU_SAMPLES_PER_BATCH:
            short += 1
            # Pad by repeating the last available sample. LEVIO expects a fixed
            # batch size; a short batch would leave stale values in the buffer.
            if len(sel) == 0:
                sel = np.array([np.searchsorted(imu_ts, hi) - 1])
            sel = np.concatenate([sel,
                                  np.repeat(sel[-1], IMU_SAMPLES_PER_BATCH - len(sel))])
        elif len(sel) > IMU_SAMPLES_PER_BATCH:
            long_ += 1
            sel = sel[:IMU_SAMPLES_PER_BATCH]

        rows = []
        for s in sel:
            # (t, ax, ay, az, wx, wy, wz) -- accelerometer before gyroscope,
            # matching imu_measurement_t. TUM-VI stores gyroscope first.
            rec = (imu_ts[s] * 1e-9,
                   acc[s, 0], acc[s, 1], acc[s, 2],
                   gyro[s, 0], gyro[s, 1], gyro[s, 2])
            packed = b"".join(struct.pack("f", float(v)) for v in rec)
            rows.append(np.frombuffer(packed, dtype=np.uint8))
        write_pgm(os.path.join(imu_dir, "imu%06d.pgm" % k), np.array(rows))

    print("wrote %d IMU batches (%d short, %d long)"
          % (len(cam_rows) - 1, short, long_))

    # ---- ground truth, converted to the camera frame ----
    gt_path = os.path.join(mav, "mocap0", "data.csv")
    out_gt = os.path.join(args.out, "groundtruth.txt")
    if os.path.exists(gt_path):
        gt = np.loadtxt(gt_path, delimiter=",", comments="#")
        T_BS = calib["T_cam_imu"]
        n_written = 0
        with open(out_gt, "w") as f:
            f.write("# TUM-VI ground truth (mocap0), camera (cam0) frame, TUM format\n")
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for row in gt:
                t = row[0] * 1e-9
                p_wb = row[1:4]
                # TUM-VI stores the quaternion as w, x, y, z.
                R_wb = R_from_quat_wxyz(row[4], row[5], row[6], row[7])

                T_wb = np.eye(4)
                T_wb[:3, :3] = R_wb
                T_wb[:3, 3] = p_wb

                # Ground truth is the body (mocap) pose; LEVIO estimates the
                # camera pose. Composing with T_BS removes the ~5 cm lever arm,
                # which trajectory alignment cannot absorb because it applies on
                # the opposite side of the transform.
                T_wc = T_wb @ T_BS

                q = quat_from_R(T_wc[:3, :3])
                f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n"
                        % (t, T_wc[0, 3], T_wc[1, 3], T_wc[2, 3],
                           q[0], q[1], q[2], q[3]))
                n_written += 1
        print("wrote ground truth: %s (%d poses, camera frame)" % (out_gt, n_written))
    else:
        print("warning: no ground truth found at %s" % gt_path)

    # Absolute per-frame timestamps. Without these the harness would synthesise
    # index/CAMERA_FPS, which cannot be associated with the mocap timeline.
    ts_path = os.path.join(args.out, "timestamps.txt")
    with open(ts_path, "w") as f:
        for ts in cam_ts:
            f.write("%.9f\n" % (ts * 1e-9))
    print("wrote %s (%d absolute timestamps)" % (ts_path, len(cam_ts)))

    dt = np.diff(cam_ts) * 1e-9
    print("frame interval: mean %.4f s, min %.4f, max %.4f (nominal %.4f)"
          % (dt.mean(), dt.min(), dt.max(), 1.0 / 20.0))
    print("IMU per frame : %.2f (batch size %d)"
          % (len(imu_ts) / max(1, len(cam_rows)), IMU_SAMPLES_PER_BATCH))


if __name__ == "__main__":
    main()
