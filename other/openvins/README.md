# OpenVINS (K230 Port)

OpenVINS is an on-manifold Multi-State Constraint Kalman Filter (MSCKF) state estimator (stereo camera + IMU). This port runs headless on the Canaan Kendryte K230 using a multi-process IPC architecture (`vins`, `imu_process`, `camera_process`).

---

## 1. Cross-Compile for K230

Prerequisites: XuanTie RISC-V toolchain (`riscv64-unknown-linux-gnu-`) and target Ceres/Eigen dependencies in `k230-deps`.

```bash
# Build for K230 with RVV vector acceleration (recommended for C908)
./scripts/build.sh --target k230 --rvv 1

# Or build for standard rv64gc
./scripts/build.sh --target k230 --rvv 0
```

Compiled binaries are generated in `build-k230-rvv/`:
- `build-k230-rvv/vins` (estimator)
- `build-k230-rvv/imu_process` (IMU feeder)
- `build-k230-rvv/camera_process` (camera feeder)

---

## 2. Run on EuRoC-MAV

Runs stereo half-resolution configuration (`config/euroc_mav_376_stereo/estimator_config.yaml`).

```bash
# Run a specific sequence
./scripts/run_euroc_mav.sh --sequence MH_01_easy

# Run all 11 EuRoC sequences
./scripts/run_euroc_mav.sh --sequence all

# Quick test with first 200 frames
./scripts/run_euroc_mav.sh --sequence MH_01_easy --short-run 200
```

---

## 3. Run on TUM-VI

Runs TUM-VI room configuration (`config/tum_vi/estimator_config.yaml`).

```bash
# Run a specific room sequence
./scripts/run_tum_vi.sh --sequence room1_512_16

# Run all 6 room sequences
./scripts/run_tum_vi.sh --sequence all
```
