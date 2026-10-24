#!/usr/bin/env python3
# K230-VIO — EuRoC MAV (ASL folder format) -> LEVIO PGM format.
#
# The converter shipped with LEVIO (scripts/prepare_euroc_data.py) reads
# rosbags. The EuRoC dataset is also distributed in the ASL folder layout
# (mav0/cam0/data/*.png + CSVs), which needs no ROS at all. This script does the
# same conversion from that layout, and additionally writes a TUM ground-truth
# file expressed in the *camera* frame so evo can be pointed straight at it.
#
# Image processing is kept bit-identical to the upstream script for the
# baseline profile:
#   undistorted = cv2.undistort(raw, K, D)
#   subsampled  = undistorted[1::4, 57:-57:4]     -> 160x120
# The half/quarter profiles instead use the complete undistorted frame resized
# to 376x240 or 188x120. Their intrinsics are read from sensor.yaml and are
# compiled into config.h for the corresponding LEVIO profile.
#
# Two conventions that are easy to get wrong, both verified against the data:
#
#   1. EuRoC imu0/data.csv is ordered timestamp, GYRO(3), ACCEL(3).
#      LEVIO's imu_measurement_t is {t_stamp, acc(3), gyro(3)} - accelerometer
#      first. The columns are reordered on write.
#
#   2. cam0/sensor.yaml T_BS holds the camera-to-body transform. Its rotation
#      is exactly the transpose of IMU_TO_CAM_R in config.h, confirming that
#      LEVIO's imu_to_cam_tf maps IMU-frame vectors into the camera frame
#      (optimizer/imu_preintegration.c:103).
#
# Usage:
#   python3 tools/prepare_euroc_asl.py <euroc_root>/MH_01_easy --out data/MH01
#
# SPDX-License-Identifier: MIT

import argparse
import os
import struct
import sys

import cv2
import numpy as np

# EuRoC cam0 intrinsics, matching prepare_euroc_data.py and cam0/sensor.yaml.
K = np.array([[458.654, 0, 367.215], [0, 457.296, 248.375], [0, 0, 1]])
D = np.array([-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05])

IMU_SAMPLES_PER_BATCH = 10        # 200 Hz IMU / 20 Hz camera
OUT_W, OUT_H = 160, 120


def load_csv(path):
    """Loads an ASL data.csv, skipping the '#' header line."""
    return np.loadtxt(path, delimiter=",", comments="#")


def load_T_BS(sensor_yaml):
    """
    Extracts the 4x4 T_BS matrix from an ASL sensor.yaml.

    Parsed by hand rather than with PyYAML so the script has no dependency
    beyond numpy and OpenCV, which the LEVIO conda environment already has.
    """
    with open(sensor_yaml) as f:
        text = f.read()
    start = text.index("T_BS:")
    start = text.index("data:", start)
    open_b = text.index("[", start)
    close_b = text.index("]", open_b)
    values = [float(v) for v in text[open_b + 1:close_b].replace("\n", " ").split(",")]
    if len(values) != 16:
        raise ValueError("expected 16 values in T_BS, got %d" % len(values))
    return np.array(values).reshape(4, 4)


def load_camera_calibration(sensor_yaml):
    """Read EuRoC pinhole calibration from sensor.yaml, with legacy fallback."""
    if not os.path.isfile(sensor_yaml):
        return K.copy(), D.copy(), (752, 480)
    with open(sensor_yaml) as f:
        text = f.read()

    def values_after(name, count):
        marker = name + ":"
        if marker not in text:
            return None
        tail = text[text.index(marker) + len(marker):]
        start = tail.find("[")
        end = tail.find("]", start)
        if start < 0 or end < 0:
            return None
        vals = [float(v.strip()) for v in tail[start + 1:end].split(",")]
        return vals if len(vals) == count else None

    resolution = values_after("resolution", 2) or [752, 480]
    intrinsics = values_after("intrinsics", 4)
    distortion = values_after("distortion_coefficients", 4)
    if intrinsics is None or distortion is None:
        return K.copy(), D.copy(), (int(resolution[0]), int(resolution[1]))
    fx, fy, cx, cy = intrinsics
    return (np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]),
            np.array(distortion),
            (int(resolution[0]), int(resolution[1])))


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
        f.write(arr.astype(np.uint8).tobytes())


def main():
    ap = argparse.ArgumentParser(
        description="Convert a EuRoC ASL sequence to the LEVIO PGM format")
    ap.add_argument("sequence", help="sequence directory, e.g. .../MH_01_easy")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop after N frames (0 = all)")
    ap.add_argument("--width", type=int, default=OUT_W,
                    help="output width (default 160)")
    ap.add_argument("--height", type=int, default=OUT_H,
                    help="output height (default 120)")
    ap.add_argument("--full-frame", action="store_true",
                    help="resize the complete undistorted frame; use for half/quarter")
    ap.add_argument("--skip-images", action="store_true",
                    help="only regenerate IMU and ground truth")
    args = ap.parse_args()

    mav = os.path.join(args.sequence, "mav0")
    if not os.path.isdir(mav):
        sys.exit("error: %s does not look like an ASL sequence (no mav0/)" % args.sequence)

    img_dir = os.path.join(args.out, "images")
    imu_dir = os.path.join(args.out, "imu")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)

    # ---- camera index ----
    cam_csv = os.path.join(mav, "cam0", "data.csv")
    cam_rows = []
    with open(cam_csv) as f:
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
    imu = load_csv(os.path.join(mav, "imu0", "data.csv"))
    imu_ts = imu[:, 0].astype(np.int64)
    gyro = imu[:, 1:4]
    acc = imu[:, 4:7]
    print("IMU samples   : %d" % len(imu_ts))

    # ---- camera calibration and images ----
    sensor_yaml = os.path.join(mav, "cam0", "sensor.yaml")
    camera_K, camera_D, source_resolution = load_camera_calibration(sensor_yaml)
    print("calibration   : %dx%d -> %dx%d%s"
          % (source_resolution[0], source_resolution[1], args.width, args.height,
             " (full frame)" if args.full_frame else " (legacy crop)"))
    if not args.skip_images:
        for i, (_, name) in enumerate(cam_rows):
            raw = cv2.imread(os.path.join(mav, "cam0", "data", name), cv2.IMREAD_GRAYSCALE)
            if raw is None:
                sys.exit("error: cannot read %s" % name)
            undistorted = cv2.undistort(raw, camera_K, camera_D)
            if args.full_frame:
                image = cv2.resize(undistorted, (args.width, args.height),
                                   interpolation=cv2.INTER_AREA)
            else:
                image = undistorted[1::4, 57:-57:4]
                if (args.width, args.height) != (OUT_W, OUT_H):
                    sys.exit("legacy crop is fixed at %dx%d; use --full-frame for another resolution"
                             % (OUT_W, OUT_H))
            if image.shape != (args.height, args.width):
                sys.exit("converted to %s, expected (%d, %d)"
                         % (image.shape, args.height, args.width))
            write_pgm(os.path.join(img_dir, "frame%06d.pgm" % i), image)
            if (i + 1) % 250 == 0:
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
            # matching imu_measurement_t.
            rec = (imu_ts[s] * 1e-9,
                   acc[s, 0], acc[s, 1], acc[s, 2],
                   gyro[s, 0], gyro[s, 1], gyro[s, 2])
            packed = b"".join(struct.pack("f", float(v)) for v in rec)
            rows.append(np.frombuffer(packed, dtype=np.uint8))
        write_pgm(os.path.join(imu_dir, "imu%06d.pgm" % k), np.array(rows))

    print("wrote %d IMU batches (%d short, %d long)"
          % (len(cam_rows) - 1, short, long_))

    # ---- ground truth, converted to the camera frame ----
    gt_path = os.path.join(mav, "state_groundtruth_estimate0", "data.csv")
    out_gt = os.path.join(args.out, "groundtruth.txt")
    if os.path.exists(gt_path):
        gt = load_csv(gt_path)
        T_BS = load_T_BS(os.path.join(mav, "cam0", "sensor.yaml"))
        n_written = 0
        with open(out_gt, "w") as f:
            f.write("# EuRoC ground truth, camera (cam0) frame, TUM format\n")
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for row in gt:
                t = row[0] * 1e-9
                p_wb = row[1:4]
                # EuRoC stores the quaternion as w, x, y, z.
                R_wb = R_from_quat_wxyz(row[4], row[5], row[6], row[7])

                T_wb = np.eye(4)
                T_wb[:3, :3] = R_wb
                T_wb[:3, 3] = p_wb

                # Ground truth is the body (IMU) pose; LEVIO estimates the
                # camera pose. Composing with T_BS removes the ~7 cm lever arm,
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
    # index/CAMERA_FPS, which cannot be associated with the EuRoC ground-truth
    # timeline. src/harness/source_pgm.c picks this file up automatically.
    ts_path = os.path.join(args.out, "timestamps.txt")
    with open(ts_path, "w") as f:
        for ts in cam_ts:
            f.write("%.9f\n" % (ts * 1e-9))
    print("wrote %s (%d absolute timestamps)" % (ts_path, len(cam_ts)))

    dt = np.diff(cam_ts) * 1e-9
    print("frame interval: mean %.4f s, min %.4f, max %.4f (nominal %.4f)"
          % (dt.mean(), dt.min(), dt.max(), 1.0 / 20.0))


if __name__ == "__main__":
    main()
