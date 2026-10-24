# LEVIO (K230 Port)

Lightweight Embedded Visual-Inertial Odometry (LEVIO) ported to the 64-bit RISC-V Canaan Kendryte K230 SoC. This port runs headless using an out-of-tree POSIX harness.

---

## 1. Cross-Compile for K230

Prerequisite: Prepared `k230_linux_sdk` with cross-toolchain and sysroot.

```bash
# Build for EuRoC-MAV (half resolution: 376x240)
K230_SDK=/path/to/k230_linux_sdk ./scripts/build.sh --target k230 --profile euroc_half

# Build for TUM-VI (half resolution: 256x256)
K230_SDK=/path/to/k230_linux_sdk ./scripts/build.sh --target k230 --profile tumvi
```

Binaries are generated at `build-k230-euroc_half/levio_replay` and `build-k230-tumvi/levio_replay`.

---

## 2. Run on EuRoC-MAV

LEVIO uses preprocessed PGM frames and IMU batches.

```bash
# Run a specific sequence on K230 board
./scripts/run_euroc_mav.sh --board -s MH01

# Run all 11 EuRoC sequences on board
./scripts/run_euroc_mav.sh --board
```

Direct binary execution on K230 board:
```bash
./build-k230-euroc_half/levio_replay \
  --data /path/to/levio-data/MH_01_easy \
  --log-dir /path/to/output_dir
```

---

## 3. Run on TUM-VI

```bash
# Run a specific room on K230 board
./scripts/run_tum_vi.sh --board -s room1

# Run all 6 room sequences on board
./scripts/run_tum_vi.sh --board
```

Direct binary execution on K230 board:
```bash
./build-k230-tumvi/levio_replay \
  --data /path/to/levio-data/room1 \
  --log-dir /path/to/output_dir
```
