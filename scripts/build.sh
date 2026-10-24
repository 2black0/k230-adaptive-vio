#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET=host
JOBS=4
USE_RVV=OFF
CLEAN=false

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [--target host|k230] [--jobs N] [--clean] [--rvv]

  --target host|k230  Build the native or K230 RISC-V binary (default: host)
  --jobs N            Parallel build jobs (default: 4)
  --clean             Remove the selected build directory before building
  --rvv               Enable RVV for the K230 build
  -h, --help          Show this help

For a K230 build, set K230_SDK to the prepared K230 Linux SDK directory.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)
      TARGET="${2:?--target needs host or k230}"
      shift 2
      ;;
    --jobs)
      JOBS="${2:?--jobs needs a number}"
      shift 2
      ;;
    --rvv)
      USE_RVV=ON
      shift
      ;;
    --clean)
      CLEAN=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "$TARGET" == host || "$TARGET" == k230 ]] || {
  echo "Error: --target must be 'host' or 'k230'" >&2
  exit 2
}
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || {
  echo "Error: --jobs must be a positive integer" >&2
  exit 2
}
[[ "$TARGET" == k230 || "$USE_RVV" == OFF ]] || {
  echo "Error: --rvv is only supported with --target k230" >&2
  exit 2
}

build_host() {
  local path_entry
  local -a path_entries=() clean_path=()
  IFS=: read -r -a path_entries <<< "$PATH"
  for path_entry in "${path_entries[@]}"; do
    [[ "$path_entry" == *linuxbrew* ]] || clean_path+=("$path_entry")
  done
  PATH="$(IFS=:; printf '%s' "${clean_path[*]}")"
  export PATH

  local build_dir="$PROJECT_DIR/build"
  local log_file="$build_dir/build.log"
  local start end elapsed
  local generator_args=()

  if [[ "$CLEAN" == true ]]; then
    rm -rf "$build_dir"
  fi
  mkdir -p "$build_dir"
  command -v ninja >/dev/null 2>&1 && generator_args=(-G Ninja)

  start=$(date +%s)
  cmake -S "$PROJECT_DIR" -B "$build_dir" "${generator_args[@]}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
    -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG" \
    -DCMAKE_PREFIX_PATH="/usr;/usr/local" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DUSE_MARCH_NATIVE=OFF \
    -DEQVIO_BUILD_TESTS=OFF \
    -DEQVIO_BUILD_VISUALISATION=OFF \
    -DEQVIO_BUILD_ROSBAG=OFF \
    -DEQVIO_SUPPORT_CONCEPTS=OFF \
    -DEQVIO_USE_LTO=OFF \
    2>&1 | tee "$log_file"
  cmake --build "$build_dir" --parallel "$JOBS" 2>&1 | tee -a "$log_file"

  local binary="$build_dir/eqvio_opt"
  [[ -x "$binary" ]] || {
    echo "Error: host binary was not produced: $binary" >&2
    exit 1
  }
  elapsed=$(( $(date +%s) - start ))
  echo "Host build completed in ${elapsed}s: $binary"
  echo "Build log: $log_file"
}

build_k230() {
  local sdk_root="${K230_SDK:-$PROJECT_DIR/../k230_linux_sdk}"
  local conf output sysroot toolchain_dir cc cxx build_dir log_file
  local arch_flags="-march=rv64gc -mabi=lp64d"
  local generator_args=()
  local start elapsed binary file_out lib

  [[ -f "$sdk_root/.last_conf" ]] || {
    echo "Error: K230 SDK config not found at $sdk_root/.last_conf" >&2
    echo "Set K230_SDK to the prepared K230 Linux SDK directory." >&2
    exit 1
  }
  conf="$(cut -d= -f2 < "$sdk_root/.last_conf")"
  output="$sdk_root/output/$conf"
  [[ -f "$output/.config" ]] || {
    echo "Error: SDK build config not found: $output/.config" >&2
    exit 1
  }
  sysroot="$(realpath "$output/staging")"
  toolchain_dir="$(grep BR2_TOOLCHAIN_EXTERNAL_PATH "$output/.config" |
    cut -d'"' -f2)/bin"
  cc="$toolchain_dir/riscv64-unknown-linux-gnu-gcc"
  cxx="$toolchain_dir/riscv64-unknown-linux-gnu-g++"
  [[ -x "$cc" ]] || { echo "Error: cross C compiler not found: $cc" >&2; exit 1; }
  [[ -x "$cxx" ]] || { echo "Error: cross C++ compiler not found: $cxx" >&2; exit 1; }
  [[ -d "$sysroot/usr/include/eigen3" ]] || {
    echo "Error: Eigen headers not found in $sysroot/usr/include/eigen3" >&2
    exit 1
  }
  for lib in libopencv_core libopencv_features2d libopencv_calib3d libyaml-cpp; do
    ls "$sysroot"/usr/lib/"$lib".so* >/dev/null 2>&1 || {
      echo "Error: $lib not found in $sysroot/usr/lib" >&2
      exit 1
    }
  done

  [[ "$USE_RVV" == ON ]] && arch_flags="-march=rv64gcv -mabi=lp64d"
  build_dir="$PROJECT_DIR/build-k230"
  log_file="$build_dir/build.log"
  [[ "$CLEAN" == false ]] || rm -rf "$build_dir"
  mkdir -p "$build_dir"
  command -v ninja >/dev/null 2>&1 && generator_args=(-G Ninja)

  echo "K230 SDK : $sdk_root"
  echo "defconfig: $conf"
  echo "sysroot  : $sysroot"
  echo "arch     : $arch_flags"

  start=$(date +%s)
  cmake -S "$PROJECT_DIR" -B "$build_dir" "${generator_args[@]}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=riscv64 \
    -DCMAKE_C_COMPILER="$cc" \
    -DCMAKE_CXX_COMPILER="$cxx" \
    -DCMAKE_SYSROOT="$sysroot" \
    -DCMAKE_FIND_ROOT_PATH="$sysroot;$sysroot/usr" \
    -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER \
    -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY \
    -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY \
    -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY \
    -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG $arch_flags" \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG $arch_flags" \
    -DCMAKE_EXE_LINKER_FLAGS="-pthread" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DEIGEN3_INCLUDE_DIR="$sysroot/usr/include/eigen3" \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DUSE_MARCH_NATIVE=OFF \
    -DEQVIO_USE_MARCH_NATIVE=OFF \
    -DEQVIO_BUILD_TESTS=OFF \
    -DEQVIO_BUILD_VISUALISATION=OFF \
    -DEQVIO_BUILD_ROSBAG=OFF \
    -DEQVIO_SINGLE_THREADED_DATASERVER=ON \
    -DEQVIO_SUPPORT_CONCEPTS=OFF \
    -DEQVIO_USE_LTO=OFF \
    2>&1 | tee "$log_file"
  cmake --build "$build_dir" --parallel "$JOBS" 2>&1 | tee -a "$log_file"

  binary="$build_dir/eqvio_opt"
  [[ -f "$binary" ]] || {
    echo "Error: K230 binary was not produced: $binary" >&2
    exit 1
  }
  file_out="$(file -b "$binary")"
  [[ "$file_out" == *RISC-V* ]] || {
    echo "Error: expected a RISC-V binary, got: $file_out" >&2
    exit 1
  }
  elapsed=$(( $(date +%s) - start ))
  echo "K230 build completed in ${elapsed}s: $binary"
  echo "$file_out"
  if [[ -x "$toolchain_dir/riscv64-unknown-linux-gnu-readelf" ]]; then
    "$toolchain_dir/riscv64-unknown-linux-gnu-readelf" -d "$binary" 2>/dev/null |
      grep NEEDED | sed 's/^/  needs /' || true
  fi
  echo "Build log: $log_file"
}

cd "$PROJECT_DIR"
if [[ "$TARGET" == host ]]; then
  build_host
else
  build_k230
fi
