#!/usr/bin/env python3
# K230-VIO — synthetic dataset generator.
#
# Produces a LEVIO-format sequence (images/*.pgm, imu/*.pgm) together with an
# exact TUM ground-truth file. Its purpose is to exercise the whole pipeline
# end to end without needing the EuRoC rosbags, so the port can be smoke-tested
# and regression-checked offline.
#
# WHAT THIS VALIDATES
#   - the PGM I/O path, both images and the 28x10 IMU encoding
#   - ORB detection / matching / RANSAC / triangulation on real image data
#   - the optimizer running to completion without NaN or divergence
#   - trajectory output and the evo evaluation workflow
#
# WHAT IT DOES NOT VALIDATE
#   It is not a substitute for EuRoC. The renderer draws idealised corner
#   patches with no motion blur, no photometric variation and no lens
#   distortion, so absolute accuracy numbers from this generator say nothing
#   about real-world performance. The phase 2 gate in docs/09 remains EuRoC.
#
# CONVENTIONS (matched to the LEVIO sources)
#   - camera looks along +Z, X right, Y down (standard pinhole)
#   - LEVIO applies acc_cam = imu_to_cam_tf * acc_imu
#     (optimizer/imu_preintegration.c:103), so measurements are emitted in the
#     IMU frame as acc_imu = R_ic^T * acc_cam
#   - the accelerometer reports specific force: at rest it reads +1 g opposing
#     gravity, giving |acc| = 9.81
#
# SPDX-License-Identifier: MIT

import argparse
import os
import struct

import numpy as np

# ---------------------------------------------------------------------------
# Must match src/levio/definitions/config.h
# ---------------------------------------------------------------------------
IMG_W, IMG_H = 160, 120
FX, FY = 114.66, 114.32
CX, CY = 77.80, 62.09
CAMERA_FPS = 20.0
IMU_RATE = 200.0
IMU_PER_FRAME = 10
GRAVITY = 9.81

# IMU-to-camera rotation from config.h lines 43-51 (row-major).
R_IC = np.array([
    [ 0.01486554,  0.99955725, -0.02577444],
    [-0.99988093,  0.01496721,  0.00375619],
    [ 0.0041403,   0.02571553,  0.99966073],
])

K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]])


def write_pgm(path, arr):
    """Writes an 8-bit binary PGM (P5), the format LEVIO's reader expects."""
    h, w = arr.shape
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(arr.astype(np.uint8).tobytes())


def rodrigues(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    axis = axis / n
    kx, ky, kz = axis
    Kx = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
    return np.eye(3) + np.sin(angle) * Kx + (1 - np.cos(angle)) * (Kx @ Kx)


def make_trajectory(n_frames, standstill_frames, amplitude, yaw_amplitude):
    """
    Builds a smooth camera trajectory: an initial standstill (so LEVIO's
    gravity calibration and standstill detector have something to latch onto),
    followed by a gentle lateral sweep with a little yaw.

    Returns per-frame (R_wc, t_wc): camera-to-world rotation and position.
    """
    Rs, ts = [], []
    for i in range(n_frames):
        if i < standstill_frames:
            u = 0.0
        else:
            # normalised progress through the moving section
            u = (i - standstill_frames) / max(1, (n_frames - standstill_frames - 1))

        # Lateral sweep in X, slight rise in Y, slow forward creep in Z.
        x = amplitude * np.sin(2.0 * np.pi * u)
        y = 0.15 * amplitude * np.sin(4.0 * np.pi * u)
        z = 0.6 * amplitude * u
        t = np.array([x, y, z])

        yaw = yaw_amplitude * np.sin(2.0 * np.pi * u)
        R = rodrigues([0, 1, 0], yaw)

        Rs.append(R)
        ts.append(t)
    return np.array(Rs), np.array(ts)


def make_texture(rng, size=1024, block=8, blur=1.0):
    """
    Builds a blocky random texture.

    The block structure is deliberate. FAST needs a contiguous ring of pixels
    that differ from the centre, and Harris needs gradient energy in two
    directions: the corners of the blocks provide exactly that. A light blur
    softens the edges so the response is stable under sub-pixel motion instead
    of flickering as edges cross pixel boundaries.
    """
    coarse = rng.integers(25, 231, size=(size // block, size // block))
    tex = np.kron(coarse, np.ones((block, block))).astype(np.float64)
    # Add a little fine detail so large flat blocks still carry some signal.
    tex += rng.normal(0, 8, tex.shape)
    if blur > 0:
        tex = gaussian_blur(tex, blur)
    return np.clip(tex, 0, 255)


def gaussian_blur(img, sigma):
    """Separable Gaussian blur; avoids a hard dependency on scipy/cv2."""
    radius = max(1, int(3 * sigma))
    x = np.arange(-radius, radius + 1)
    k = np.exp(-0.5 * (x / sigma) ** 2)
    k /= k.sum()
    out = np.apply_along_axis(lambda m: np.convolve(m, k, mode="same"), 0, img)
    out = np.apply_along_axis(lambda m: np.convolve(m, k, mode="same"), 1, out)
    return out


def bilinear_sample(tex, u, v):
    """Bilinear texture lookup with wrap-around addressing."""
    h, w = tex.shape
    u0 = np.floor(u).astype(np.int64)
    v0 = np.floor(v).astype(np.int64)
    fu = u - u0
    fv = v - v0
    u0 %= w; v0 %= h
    u1 = (u0 + 1) % w
    v1 = (v0 + 1) % h
    return (tex[v0, u0] * (1 - fu) * (1 - fv) +
            tex[v0, u1] * fu * (1 - fv) +
            tex[v1, u0] * (1 - fu) * fv +
            tex[v1, u1] * fu * fv)


class Plane:
    """A textured plane: point p0, normal n, and in-plane axes for texturing."""

    def __init__(self, p0, n, axis_u, tex, scale):
        self.p0 = np.asarray(p0, float)
        self.n = np.asarray(n, float) / np.linalg.norm(n)
        au = np.asarray(axis_u, float)
        au = au - self.n * (au @ self.n)
        self.u = au / np.linalg.norm(au)
        self.v = np.cross(self.n, self.u)
        self.tex = tex
        self.scale = scale        # texels per metre


def render(planes, R_wc, t_wc, rays_cam):
    """
    Renders one frame by casting a ray through every pixel and shading with the
    nearest plane hit. Inverse (per-pixel) rendering rather than forward
    splatting is what makes the imagery sub-pixel consistent between frames,
    which is what BRIEF needs in order to produce repeatable descriptors.
    """
    d_w = rays_cam @ R_wc.T                       # (N,3) ray directions
    o = t_wc

    best_t = np.full(d_w.shape[0], np.inf)
    colour = np.full(d_w.shape[0], 40.0)

    for pl in planes:
        denom = d_w @ pl.n
        ok = np.abs(denom) > 1e-9
        t = np.full(d_w.shape[0], np.inf)
        t[ok] = ((pl.p0 - o) @ pl.n) / denom[ok]
        hit = ok & (t > 0.2) & (t < best_t)
        if not np.any(hit):
            continue
        p = o + d_w[hit] * t[hit][:, None]
        rel = p - pl.p0
        tu = (rel @ pl.u) * pl.scale
        tv = (rel @ pl.v) * pl.scale
        colour[hit] = bilinear_sample(pl.tex, tu, tv)
        best_t[hit] = t[hit]

    return colour


def main():
    ap = argparse.ArgumentParser(
        description="Generate a synthetic LEVIO-format sequence with ground truth")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--block", type=int, default=8,
                    help="texture block size in texels")
    ap.add_argument("--tex-scale", type=float, default=60.0,
                    help="texels per metre on the scene planes")
    ap.add_argument("--standstill", type=int, default=20,
                    help="leading frames held still, for gravity calibration")
    ap.add_argument("--amplitude", type=float, default=0.6,
                    help="lateral sweep amplitude in metres")
    ap.add_argument("--yaw", type=float, default=0.08,
                    help="yaw amplitude in radians")
    ap.add_argument("--noise", type=float, default=3.0,
                    help="image noise sigma in grey levels")
    ap.add_argument("--imu-noise", type=float, default=0.0,
                    help="IMU noise sigma (0 = perfect measurements)")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)

    img_dir = os.path.join(args.out, "images")
    imu_dir = os.path.join(args.out, "imu")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)

    # ---- scene: three textured planes forming a corner ----
    # Three planes at different depths and orientations keep the structure
    # genuinely 3D. A single plane would make the scene degenerate for the
    # essential-matrix estimator, which assumes non-planar structure.
    planes = [
        # back wall
        Plane([0, 0, 8.0],  [0, 0, -1], [1, 0, 0],
              make_texture(rng, block=args.block), args.tex_scale),
        # left wall
        Plane([-2.5, 0, 0], [1, 0, 0],  [0, 0, 1],
              make_texture(rng, block=args.block), args.tex_scale),
        # right wall
        Plane([2.5, 0, 0],  [-1, 0, 0], [0, 0, 1],
              make_texture(rng, block=args.block), args.tex_scale),
        # floor (camera Y points down, so +Y is below the optical axis)
        Plane([0, 2.0, 0],  [0, -1, 0], [1, 0, 0],
              make_texture(rng, block=args.block), args.tex_scale),
    ]

    # Precompute normalised camera-frame ray directions, one per pixel.
    uu, vv = np.meshgrid(np.arange(IMG_W) + 0.5, np.arange(IMG_H) + 0.5)
    rays_cam = np.stack([(uu - CX) / FX, (vv - CY) / FY, np.ones_like(uu)], axis=-1)
    rays_cam = rays_cam.reshape(-1, 3)

    # ---- trajectory ----
    Rs, ts = make_trajectory(args.frames, args.standstill, args.amplitude, args.yaw)

    # ---- IMU: sample the trajectory at IMU_RATE and differentiate ----
    # Position and orientation are interpolated on the IMU timeline so that
    # angular velocity and acceleration come from the same analytic curve the
    # images were rendered from.
    n_imu = args.frames * IMU_PER_FRAME
    t_imu = np.arange(n_imu) / IMU_RATE
    t_cam = np.arange(args.frames) / CAMERA_FPS

    pos_imu = np.column_stack([
        np.interp(t_imu, t_cam, ts[:, d]) for d in range(3)
    ])

    # Yaw angle recovered from the rotation matrices, then interpolated.
    yaws = np.array([np.arctan2(R[0, 2], R[0, 0]) for R in Rs])
    yaw_imu = np.interp(t_imu, t_cam, yaws)

    dt = 1.0 / IMU_RATE
    # Central differences; edges fall back to one-sided.
    vel = np.gradient(pos_imu, dt, axis=0)
    acc_w = np.gradient(vel, dt, axis=0)
    yaw_rate = np.gradient(yaw_imu, dt)

    g_w = np.array([0.0, 0.0, -GRAVITY])   # world Z up, gravity points down

    imu_records = []
    for i in range(n_imu):
        R_wc = rodrigues([0, 1, 0], yaw_imu[i])
        R_cw = R_wc.T

        # Specific force in the camera frame, then into the IMU frame.
        f_cam = R_cw @ (acc_w[i] - g_w)
        w_cam = R_cw @ np.array([0.0, yaw_rate[i], 0.0])

        acc_imu = R_IC.T @ f_cam
        gyr_imu = R_IC.T @ w_cam

        if args.imu_noise > 0:
            acc_imu = acc_imu + rng.normal(0, args.imu_noise, 3)
            gyr_imu = gyr_imu + rng.normal(0, args.imu_noise * 0.1, 3)

        imu_records.append((t_imu[i], *acc_imu, *gyr_imu))

    # ---- render frames + write ground truth ----
    gt_path = os.path.join(args.out, "groundtruth.txt")
    visible_counts = []

    with open(gt_path, "w") as gt:
        gt.write("# synthetic ground truth (TUM format)\n")
        gt.write("# timestamp tx ty tz qx qy qz qw\n")

        for i in range(args.frames):
            R_wc, t_wc = Rs[i], ts[i]
            R_cw = R_wc.T

            img = render(planes, R_wc, t_wc, rays_cam).reshape(IMG_H, IMG_W)
            visible_counts.append(int(np.count_nonzero(img > 0)))
            if args.noise > 0:
                img = img + rng.normal(0, args.noise, img.shape)
            write_pgm(os.path.join(img_dir, "frame%06d.pgm" % i),
                      np.clip(img, 0, 255))

            # ground truth: camera pose in the world frame
            q = quat_from_R(R_wc)
            gt.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" %
                     (t_cam[i], t_wc[0], t_wc[1], t_wc[2], q[0], q[1], q[2], q[3]))

    # ---- write IMU batches ----
    # LEVIO numbers batches from 1: batch k covers the interval ending at
    # frame k (see the GAP9 README and source_pgm.c).
    n_batches = 0
    for b in range(1, args.frames):
        rows = []
        for s in range(IMU_PER_FRAME):
            rec = imu_records[b * IMU_PER_FRAME + s]
            row = b"".join(struct.pack("f", float(v)) for v in rec)
            rows.append(np.frombuffer(row, dtype=np.uint8))
        write_pgm(os.path.join(imu_dir, "imu%06d.pgm" % b), np.array(rows))
        n_batches += 1

    # ---- report ----
    R_wc0 = Rs[0]
    g_cam0 = R_wc0.T @ (-g_w)          # gravity direction sensed at rest
    print("wrote %d frames to %s" % (args.frames, img_dir))
    print("wrote %d IMU batches to %s" % (n_batches, imu_dir))
    print("ground truth: %s" % gt_path)
    print("trajectory length: %.3f m" %
          float(np.sum(np.linalg.norm(np.diff(ts, axis=0), axis=1))))
    print("gravity in camera frame at t=0: [%.4f, %.4f, %.4f]  (|g| = %.3f)"
          % (g_cam0[0], g_cam0[1], g_cam0[2], np.linalg.norm(g_cam0)))
    print("  -> config.h GRAVITY_INIT_{X,Y,Z} for this sequence")


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


if __name__ == "__main__":
    main()
