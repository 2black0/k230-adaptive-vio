#!/usr/bin/env bash
# Build script for LEVIO: Host (x86_64) or Canaan Kendryte K230 (RISC-V C908)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TARGET="${TARGET:-host}"
PROFILE="${PROFILE:-euroc_half}"
MODE="${MODE:-serial}"
JOBS="${JOBS:-4}"

usage() {
  cat <<'EOF'
Usage: ./scripts/build.sh [--target host|k230] [--profile PROFILE] [--jobs N]

  --target host|k230   Target platform (default: host)
  --profile PROFILE    Dataset profile: euroc, euroc_half, euroc_quarter, tumvi, tumvi_quarter (default: euroc_half)
  --mode MODE          serial or parallel (default: serial)
  --jobs N             Parallel build jobs (default: 4)
  -h, --help           Show this help
EOF
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)
      TARGET="${2:?--target requires host or k230}"
      shift 2 ;;
    --profile)
      PROFILE="${2:?--profile requires a value}"
      shift 2 ;;
    --mode)
      MODE="${2:?--mode requires serial or parallel}"
      shift 2 ;;
    --jobs)
      JOBS="${2:?--jobs requires a number}"
      shift 2 ;;
    -h|--help)
      usage 0 ;;
    *)
      echo "Unknown argument: $1" >&2
      usage 1 ;;
  esac
done

if [[ "$TARGET" == "k230" ]]; then
  echo "[levio] Building for Canaan K230 (RISC-V)..."
  SDK_ROOT="${K230_SDK:-${ROOT}/../../../k230_linux_sdk}"

  if [ ! -f "${SDK_ROOT}/.last_conf" ]; then
    echo "error: '${SDK_ROOT}/.last_conf' not found. Please set K230_SDK=/path/to/k230_linux_sdk." >&2
    exit 1
  fi

  CONF="$(cut -d= -f2 < "${SDK_ROOT}/.last_conf")"
  OUTPUT="${SDK_ROOT}/output/${CONF}"
  GCC_PATH="$(grep BR2_TOOLCHAIN_EXTERNAL_PATH "${OUTPUT}/.config" | cut -d'"' -f2)/bin"
  SYSROOT="$(realpath "${OUTPUT}/staging")"

  CC="${GCC_PATH}/riscv64-unknown-linux-gnu-gcc"
  if [ ! -x "${CC}" ]; then
    echo "error: cross compiler not found at ${CC}" >&2
    exit 1
  fi

  ARGS=(
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_C_COMPILER="${CC}"
    -DCMAKE_SYSROOT="${SYSROOT}"
    -DCMAKE_SYSTEM_NAME=Linux
    -DCMAKE_SYSTEM_PROCESSOR=riscv64
    -DLEVIO_RVV=ON
  )

  if [ "$MODE" = "parallel" ]; then
    ARGS+=(-DLEVIO_PARALLEL=ON -DLEVIO_NB_CORES=2)
  fi

  ARGS+=("-DLEVIO_DATASET_PROFILE=${PROFILE}")
  BUILD_DIR="build-k230-${PROFILE}"

  cmake -B "${BUILD_DIR}" "${ARGS[@]}"
  cmake --build "${BUILD_DIR}" -j"${JOBS}"
  echo "[levio] Built: ${BUILD_DIR}/levio_replay"
else
  echo "[levio] Building for Host (x86_64)..."
  DIR="build"
  ARGS=(-DCMAKE_BUILD_TYPE=Release)
  if [ "$PROFILE" != "euroc" ]; then
    DIR="build-${PROFILE}"
    ARGS+=("-DLEVIO_DATASET_PROFILE=${PROFILE}")
  fi

  cmake -B "${DIR}" "${ARGS[@]}"
  cmake --build "${DIR}" -j"${JOBS}"
  echo "[levio] Built: ${DIR}/levio_replay"
fi
