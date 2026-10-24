#!/usr/bin/env bash
# Build script for LARVIO: Host (x86_64) or Canaan Kendryte K230 (RISC-V C908)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${TARGET:-host}"
JOBS="${JOBS:-4}"
RVV="${RVV:-1}"

usage() {
  cat <<'EOF'
Usage: ./scripts/build.sh [--target host|k230] [--jobs N] [--rvv 0|1]

  --target host|k230  Target architecture (default: host)
  --jobs N            Parallel compilation jobs (default: 4)
  --rvv 0|1           Enable RISC-V Vector acceleration for K230 (default: 1)
  -h, --help          Show this help
EOF
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)
      TARGET="${2:?--target requires host or k230}"
      shift 2 ;;
    --jobs)
      JOBS="${2:?--jobs requires a number}"
      shift 2 ;;
    --rvv)
      RVV="${2:?--rvv requires 0 or 1}"
      shift 2 ;;
    -h|--help)
      usage 0 ;;
    *)
      echo "Unknown argument: $1" >&2
      usage 1 ;;
  esac
done

cd "$SCRIPT_DIR"

if [[ "$TARGET" == "k230" ]]; then
  echo "[larvio] Building for Canaan K230 (RISC-V)..."
  DEPS_DIR="${K230_DEPS:-${SCRIPT_DIR}/../../../k230-deps}"
  TOOLCHAIN="${DEPS_DIR}/toolchain-k230.cmake"
  TOOL_BIN=/opt/toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2/bin
  READELF="${TOOL_BIN}/riscv64-unknown-linux-gnu-readelf"

  case "${RVV}" in
    1)
      ARCH_FLAGS="-mcpu=c908v -mabi=lp64d -mtune=c908 -mrvv-v0p10-compatible -mrvv-auto-vectorize"
      VARIANT=rvv
      ;;
    0)
      ARCH_FLAGS="-march=rv64gc -mabi=lp64d"
      VARIANT=rv64gc
      ;;
    *)
      echo "RVV must be 0 or 1" >&2
      exit 2
      ;;
  esac

  if [[ ! -f "$DEPS_DIR/riscv64/lib/libceres.a" || ! -f "$DEPS_DIR/riscv64/lib/libspqr.a" ]]; then
    if [[ -x "$DEPS_DIR/scripts/build_larvio_deps_k230.sh" ]]; then
      "$DEPS_DIR/scripts/build_larvio_deps_k230.sh"
    fi
  fi

  BUILD_DIR="build-k230-${VARIANT}"
  mkdir -p "$BUILD_DIR"
  GENERATOR_ARGS=()
  command -v ninja >/dev/null 2>&1 && GENERATOR_ARGS=(-G Ninja)

  cmake -S . -B "$BUILD_DIR" "${GENERATOR_ARGS[@]}" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG" \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
    -DCMAKE_C_FLAGS="$ARCH_FLAGS" \
    -DCMAKE_CXX_FLAGS="$ARCH_FLAGS -pthread" \
    -DCMAKE_EXE_LINKER_FLAGS="-pthread -lgfortran -lm -fopenmp" \
    -DCeres_DIR="$DEPS_DIR/riscv64/lib/cmake/Ceres" \
    -DSUITESPARSE_INCLUDE_DIR_HINTS="$DEPS_DIR/riscv64/include;$DEPS_DIR/riscv64/include/suitesparse" \
    -DSUITESPARSE_LIBRARY_DIR_HINTS="$DEPS_DIR/riscv64/lib" \
    -DBoost_NO_SYSTEM_PATHS=ON \
    -DMAKE_SCENE=OFF

  cmake --build "$BUILD_DIR" --parallel "$JOBS"
  echo "[larvio] K230 build complete in $BUILD_DIR"
else
  echo "[larvio] Building for Host (x86_64)..."
  mkdir -p build
  GENERATOR_ARGS=()
  command -v ninja >/dev/null 2>&1 && GENERATOR_ARGS=(-G Ninja)

  cmake -S . -B build "${GENERATOR_ARGS[@]}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
    -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG" \
    -DMAKE_SCENE=OFF

  cmake --build build --parallel "$JOBS"
  echo "[larvio] Host build complete in build/"
fi
