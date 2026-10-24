#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LEGACY_RUNNER="${ROOT}/scripts-old/run_euroc_mav.sh"
CONFIG="${ROOT}/results/SC-FBC/_campaign/EuRoC-MAV/frozen/euroc.yaml"
DATASET_ROOT="${EUROC_DATASET_PATH_376:-$HOME/dataset/EuRoC-MAV-376}"
BOARD_DATA_ROOT="${K230_EUROC_HALF_ROOT:-/root/dataset/PGM-half}"
TARGET=host
SEQUENCE=MH_04_difficult
ARM_SELECTION=all
SERIAL="${K230_ADB_SERIAL:-k230-02}"
DRY_RUN=false
MONITOR=true

usage() {
  cat <<'HELP'
Run the five EuRoC-MAV half-resolution manuscript arms.

Usage: bash scripts/run_euroc_mav.sh [OPTIONS]
  --target host|k230       Execution target (default: host)
  --sequence NAME          Canonical EuRoC sequence (default: MH_04_difficult)
  --resolution half        Half-resolution input only (default: half)
  --arm NAME|all           eqvio, base-policy, floor-only, hold-only, sc-fbc
                           (default: all; folders: EqVIO, Fixed-Threshold,
                           SC-FBC, Auto-Hold, Dual-Loop)
  --dataset-root PATH      Host mirror containing *_376x240 sequences
  --serial SERIAL          K230 ADB serial (default: k230-02)
  --no-monitor             Disable host resource monitoring
  --dry-run                Print planned runs without executing them
  -h, --help               Show this help

K230 half-resolution data must be under /root/dataset/PGM-half (or
K230_EUROC_HALF_ROOT). Results are saved under
result/<target>/<arm>/EuRoC-MAV/<sequence>/.
HELP
}

die() {
  echo "Error: $*" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)
      [[ "${2:-}" == host || "${2:-}" == k230 ]] || die "--target must be host or k230"
      TARGET="$2"; shift 2 ;;
    --sequence)
      [[ -n "${2:-}" && "$2" != -* ]] || die "--sequence requires a canonical sequence name"
      SEQUENCE="$2"; shift 2 ;;
    --resolution)
      [[ "${2:-}" == half ]] || die "only --resolution half is supported"
      shift 2 ;;
    --arm)
      [[ -n "${2:-}" && "$2" != -* ]] || die "--arm requires a name or all"
      ARM_SELECTION="$2"; shift 2 ;;
    --dataset-root)
      [[ -n "${2:-}" && "$2" != -* ]] || die "--dataset-root requires a path"
      DATASET_ROOT="$2"; shift 2 ;;
    --serial)
      [[ -n "${2:-}" && "$2" != -* ]] || die "--serial requires an ADB serial"
      SERIAL="$2"; shift 2 ;;
    --no-monitor)
      MONITOR=false; shift ;;
    --dry-run)
      DRY_RUN=true; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      die "unknown option '$1' (see --help)" ;;
  esac
done

[[ -f "$LEGACY_RUNNER" ]] || die "archived runner not found: $LEGACY_RUNNER"
[[ -f "$CONFIG" ]] || die "frozen manuscript config not found: $CONFIG"
[[ "$SEQUENCE" =~ ^(MH_[0-9]{2}_(easy|medium|difficult)|V[12]_[0-9]{2}_(easy|medium|difficult))$ ]] \
  || die "unsupported EuRoC sequence name: $SEQUENCE"

SEQUENCE_DIR="${SEQUENCE}_376x240"
DATASET="${DATASET_ROOT}/${SEQUENCE_DIR}"
CALIB="${DATASET}/intrinsics.yaml"
[[ -s "${DATASET}/mav0/cam0/data.csv" ]] || die "camera manifest not found: ${DATASET}/mav0/cam0/data.csv"
[[ -s "${DATASET}/mav0/imu0/data.csv" ]] || die "IMU manifest not found: ${DATASET}/mav0/imu0/data.csv"
[[ -s "${DATASET}/mav0/state_groundtruth_estimate0/data.csv" ]] || die "ground truth not found under $DATASET"
[[ -s "$CALIB" ]] || die "half-resolution calibration not found: $CALIB"

EXPECTED_FRAMES="$(python3 - "${DATASET}/mav0/cam0/data.csv" <<'PY'
import sys
with open(sys.argv[1]) as stream:
    print(sum(1 for line in stream if line.strip() and not line.startswith("#")))
PY
)"
HOST_IMAGES="$(find "${DATASET}/mav0/cam0/data" -maxdepth 1 -type f -name '*.png' | wc -l | tr -d '[:space:]')"
[[ "$EXPECTED_FRAMES" =~ ^[0-9]+$ && "$EXPECTED_FRAMES" -gt 0 ]] || die "invalid camera frame count"
[[ "$HOST_IMAGES" == "$EXPECTED_FRAMES" ]] || die "camera manifest lists $EXPECTED_FRAMES frames but dataset has $HOST_IMAGES PNGs"

case "${ARM_SELECTION,,}" in
  all) ARMS=(eqvio base-policy floor-only hold-only sc-fbc) ;;
  *)
    IFS=',' read -ra REQUESTED_ARMS <<< "$ARM_SELECTION"
    ARMS=()
    for arm in "${REQUESTED_ARMS[@]}"; do
      arm="${arm,,}"
      case "$arm" in
        eqvio|base-policy|floor-only|hold-only|sc-fbc) ;;
        *) die "unknown arm '$arm'" ;;
      esac
      if [[ ! " ${ARMS[*]} " == *" ${arm} "* ]]; then
        ARMS+=("$arm")
      fi
    done
    ;;
esac

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_$$"
BOARD_ROOT="/tmp/k230-adaptive-vio-euroc-${RUN_ID}"
BOARD_SEQUENCE="${BOARD_ROOT}/${SEQUENCE_DIR}"
BOARD_SOURCE="${BOARD_DATA_ROOT}/euroc_${SEQUENCE}"
BOARD_OUT="${BOARD_ROOT}/out/run"

if [[ "$TARGET" == k230 ]]; then
  [[ "$BOARD_ROOT" =~ ^/[[:alnum:]_./-]+$ && "$BOARD_SOURCE" =~ ^/[[:alnum:]_./-]+$ ]] \
    || die "K230 paths may contain only letters, digits, '/', '.', '_', and '-'"
  if [[ "$DRY_RUN" == false ]]; then
    state="$(adb -s "$SERIAL" get-state 2>/dev/null || true)"
    [[ "$state" == device ]] || die "ADB device '$SERIAL' is not online"
    source_ready="$(adb -s "$SERIAL" shell "if test -d '${BOARD_SOURCE}/mav0/cam0/data' && test -s '${BOARD_SOURCE}/mav0/cam0/data.csv' && test -s '${BOARD_SOURCE}/mav0/imu0/data.csv' && test -s '${BOARD_SOURCE}/mav0/state_groundtruth_estimate0/data.csv'; then echo ready; else echo missing; fi" | tr -d '\r\n')"
    [[ "$source_ready" == ready ]] || die "half-resolution board dataset is incomplete: $BOARD_SOURCE"
    board_frames="$(adb -s "$SERIAL" shell "find '${BOARD_SOURCE}/mav0/cam0/data' -maxdepth 1 -type f | wc -l" | tr -d '\r[:space:]')"
    [[ "$board_frames" == "$EXPECTED_FRAMES" ]] \
      || die "board dataset has $board_frames images; host manifest expects $EXPECTED_FRAMES"
    existing="$(adb -s "$SERIAL" shell "if [ -e '${BOARD_ROOT}' ] || [ -L '${BOARD_ROOT}' ]; then echo yes; else echo no; fi" | tr -d '\r\n')"
    [[ "$existing" == no ]] || die "temporary board path already exists or could not be checked: $BOARD_ROOT"
    prepared="$(adb -s "$SERIAL" shell "if mkdir -p '${BOARD_ROOT}/cfg' '${BOARD_ROOT}/out' && ln -s '${BOARD_SOURCE}' '${BOARD_SEQUENCE}'; then echo prepared; else echo failed; fi" | tr -d '\r\n')"
    [[ "$prepared" == prepared ]] || die "could not prepare temporary K230 run directory"
  else
    echo "K230 dataset: $BOARD_SOURCE -> $BOARD_SEQUENCE"
  fi
  export ANDROID_SERIAL="$SERIAL"
fi

run_arm() {
  local arm="$1" label="$1" result_label="$1" extra_args prefix result_dir summary framelog
  local common="--pacedReplay --seed 0 --featureDistOverride 80"
  local policy="--adaptiveBudget starve --adaptiveBudgetMaxLow 5 --adaptiveBudgetMaxHigh 20 --adaptiveBudgetMeasFloor 8 --adaptiveBudgetHold 5 --adaptiveBudgetAutoFloorWindow 50 --adaptiveBudgetAutoFloorPercentile 40 --adaptiveBudgetAutoHoldWindow 50 --adaptiveBudgetAutoHoldGain 40"

  case "$arm" in
    eqvio)
      label="EqVIO"; result_label="EqVIO"; extra_args="$common" ;;
    base-policy)
      label="Fixed-Threshold"; result_label="Fixed-Threshold"; extra_args="$common $policy" ;;
    floor-only)
      label="SC-FBC"; result_label="SC-FBC"; extra_args="$common $policy --adaptiveBudgetAutoFloor" ;;
    hold-only)
      label="Auto-Hold"; result_label="Auto-Hold"; extra_args="$common $policy --adaptiveBudgetAutoHold" ;;
    sc-fbc)
      label="Dual-Loop"; result_label="Dual-Loop"; extra_args="$common $policy --adaptiveBudgetAutoFloor --adaptiveBudgetAutoHold" ;;
  esac

  prefix="${RUN_ID}_${TARGET}_${arm}"
  echo "== ${label}: ${SEQUENCE} (half, ${TARGET}) =="
  echo "   frames=${EXPECTED_FRAMES}; output=result/${TARGET}/${result_label}/EuRoC-MAV/${SEQUENCE}/${prefix}_*"
  echo "   EqVIO args: ${extra_args}"

  local args=(
    --target "$TARGET"
    --sequence "$SEQUENCE_DIR"
    --dataset-path "$DATASET_ROOT"
    --config "$CONFIG"
    --cam-calib "$CALIB"
    --output-name "$prefix"
    --eqvio-args "$extra_args"
    --filter-log
  )
  if [[ "$TARGET" == k230 ]]; then
    args+=(--board-root "$BOARD_ROOT" --board-out "$BOARD_OUT")
  fi
  if [[ "$MONITOR" == false ]]; then
    args+=(--no-monitor)
  fi
  if [[ "$DRY_RUN" == true ]]; then
    args+=(--dry-run)
  fi
  RESULT_ARM_LABEL="$result_label" bash "$LEGACY_RUNNER" "${args[@]}"

  if [[ "$DRY_RUN" == false ]]; then
    result_dir="${ROOT}/result/${TARGET}/${result_label}/EuRoC-MAV/${SEQUENCE}"
    summary="${result_dir}/${prefix}_summary.json"
    framelog="${result_dir}/${prefix}_framelog.csv"
    [[ -s "${result_dir}/${prefix}_trajectory.tum" && -s "$summary" && -s "$framelog" ]] \
      || die "${label} did not produce trajectory, summary, and per-frame log in $result_dir"
    python3 - "$summary" "$framelog" "$EXPECTED_FRAMES" <<'PY'
import csv, json, sys
summary_path, log_path, expected = sys.argv[1], sys.argv[2], int(sys.argv[3])
with open(summary_path) as stream:
    summary = json.load(stream)
tracking = summary.get("tracking", {})
total = tracking.get("total_frames")
processed = tracking.get("processed_frames")
with open(log_path, newline="") as stream:
    rows = sum(1 for _ in csv.DictReader(stream))
if total != expected or processed not in (expected, expected - 1) or rows != processed:
    raise SystemExit(
        f"expected {expected} input frames and matching per-frame rows "
        f"(allowing one initialization frame); got {total}/{processed}/{rows}"
    )
startup = " (one initialization frame omitted)" if processed == expected - 1 else ""
print(f"   verified {total} input frames, {processed} states, and {rows} per-frame rows{startup}")
PY
  fi
}

for arm in "${ARMS[@]}"; do
  run_arm "$arm"
done

if [[ "$TARGET" == k230 && "$DRY_RUN" == false ]]; then
  cleanup="$(adb -s "$SERIAL" shell "if rm -rf '${BOARD_ROOT}' && [ ! -e '${BOARD_ROOT}' ] && [ ! -L '${BOARD_ROOT}' ]; then echo removed; else echo failed; fi" | tr -d '\r\n')"
  [[ "$cleanup" == removed ]] || die "runs completed but temporary board directory remains at $BOARD_ROOT"
fi

echo "All selected arms completed. Results: result/${TARGET}/<arm>/EuRoC-MAV/${SEQUENCE}/"
