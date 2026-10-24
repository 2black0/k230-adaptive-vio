# LARVIO (K230 Port)

Lightweight, Accurate and Robust Visual-Inertial Odometry (LARVIO) based on MSCKF (monocular camera + IMU). This port runs headless for embedded RISC-V evaluation on the Canaan Kendryte K230.

---

## 1. Cross-Compile for K230

Prerequisites: XuanTie RISC-V toolchain (`riscv64-unknown-linux-gnu-`) and target dependencies in `k230-deps`.

```bash
# Build for K230 with RVV vector acceleration (recommended for C908)
./scripts/build.sh --target k230 --rvv 1

# Or build for standard rv64gc
./scripts/build.sh --target k230 --rvv 0
```

Compiled binaries are saved to `build-k230-rvv/larvio` and `build-k230-rvv/larvio_eval`.

---

## 2. Run on EuRoC-MAV

Expected dataset format: PGM half-resolution (`376x240`) or raw EuRoC folder (`mav0/`).

```bash
# Run a specific sequence
./scripts/run_euroc_mav.sh --sequence MH_01_easy

# Run all 11 EuRoC sequences
./scripts/run_euroc_mav.sh --sequence all

# Quick test with first 200 frames
./scripts/run_euroc_mav.sh --sequence MH_01_easy --short-run 200
```

Direct binary execution on K230 board:
```bash
./build-k230-rvv/larvio \
  /path/to/mav0/imu0/data.csv \
  /path/to/mav0/cam0/data.csv \
  /path/to/mav0/cam0/data \
  config/euroc_mav_376_pgm.yaml \
  --dataset-name MH_01_easy
```

---

## 3. Run on TUM-VI

Expected dataset format: PGM 256x256 or TUM-VI folder (`mav0/`).

```bash
# Run a specific room sequence
./scripts/run_tum_vi.sh --sequence room1

# Run all 6 room sequences
./scripts/run_tum_vi.sh --sequence all
```

Direct binary execution on K230 board:
```bash
./build-k230-rvv/larvio \
  /path/to/mav0/imu0/data.csv \
  /path/to/mav0/cam0/data.csv \
  /path/to/mav0/cam0/data \
  config/tum_vi.yaml \
  --dataset-name room1
```
