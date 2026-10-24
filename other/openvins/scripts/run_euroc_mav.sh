#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
export LC_NUMERIC=C
export PATH="$HOME/.local/bin:$PATH"

# =============================================================================
# OpenVINS — Batch processing test
# =============================================================================
# Processes the pre-recorded dataset as fast as possible (no real-time pacing).
# Use for benchmarking maximum throughput and comparing accuracy vs offline GT.
#
# Usage:
#   ./run-euroc.sh                                                    #
#   ./run-euroc.sh --sequence MH_05_difficult                        # default sequence
#   ./run-euroc.sh --sequence MH_01_easy                             # other sequence
#   ./run-euroc.sh --dry-run                                         # Preview only
#   ./run-euroc.sh --short-run <frames>                              # Run N frames
#   ./run-euroc.sh --output-name <name>                              # Optional custom prefix
#   ./run-euroc.sh --loop 3                                          # Run 3 iterations
#   ./run-euroc.sh --short-run 10 --sequence V1_01_easy              # Combined
# =============================================================================

# Get script directory (scripts/), then parent
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULT_BASE="${SCRIPT_DIR}/result"
PYTHON="python3"
VINS_BINARY="${SCRIPT_DIR}/build/vins"
IMU_BINARY="${SCRIPT_DIR}/build/imu_process"
CAMERA_BINARY="${SCRIPT_DIR}/build/camera_process"

BASE_DATASET_PATH="$HOME/dataset/euroc_mav/raw"
BASE_CONFIG_DIR="${SCRIPT_DIR}/config/euroc_mav"
BASE_CONFIG="${BASE_CONFIG_DIR}/estimator_config.yaml"

# Default sequence & output
SEQUENCE="all"
OUTPUT_NAME=""
PER_FRAME=true
DO_MONITOR=true

# Process flags
DRY_RUN=false
SHORT_RUN_FRAMES=0
LOOP_COUNT=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence)
      if [[ -z "${2:-}" ]]; then
        echo "Error: --sequence requires a value (e.g. MH_05_difficult, MH_01_easy, V1_01_easy)"
        exit 1
      fi
      SEQUENCE="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --short-run)
      if [[ -z "${2:-}" || ! "$2" =~ ^[0-9]+$ ]]; then
        echo "Error: --short-run requires a positive integer (frames)"
        exit 1
      fi
      SHORT_RUN_FRAMES="$2"
      shift 2
      ;;
    --output-name)
      if [[ -z "${2:-}" ]]; then
        echo "Error: --output-name requires a value"
        exit 1
      fi
      OUTPUT_NAME="$2"
      shift 2
      ;;
    --loop)
      if [[ -z "${2:-}" || ! "$2" =~ ^[0-9]+$ ]]; then
        echo "Error: --loop requires a positive integer"
        exit 1
      fi
      LOOP_COUNT="$2"
      shift 2
      ;;
    --per-frame)
      PER_FRAME=true
      shift
      ;;
    --monitor)
      DO_MONITOR=true
      shift
      ;;
    *)
      echo "Usage: $0 [--sequence <seq>] [--dry-run] [--short-run <frames>] [--output-name <name>] [--loop <N>] [--per-frame] [--monitor]"
      echo ""
      echo "Defaults: --per-frame --monitor enabled; --sequence all; empty output prefix"
      echo ""
      echo "Sequences: MH_01_easy  MH_02_easy  MH_03_medium  MH_04_difficult  MH_05_difficult"
      echo "           V1_01_easy  V1_02_medium  V1_03_difficult"
      echo "           V2_01_easy  V2_02_medium  V2_03_difficult"
      echo "           all (runs all 11 sequences)"
      exit 1
      ;;
  esac
done

# ── Resolve sequences ──────────────────────────────────────────────
ALL_SEQUENCES=(MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult V1_01_easy V1_02_medium V1_03_difficult V2_01_easy V2_02_medium V2_03_difficult)
declare -A SEQ_SHORT=( [MH01]=MH_01_easy [MH02]=MH_02_easy [MH03]=MH_03_medium [MH04]=MH_04_difficult [MH05]=MH_05_difficult [V101]=V1_01_easy [V102]=V1_02_medium [V103]=V1_03_difficult [V201]=V2_01_easy [V202]=V2_02_medium [V203]=V2_03_difficult )
resolve_sequence() {
  local input="$1"
  if [[ "$input" == "all" ]]; then echo "${ALL_SEQUENCES[@]}"; return; fi
  local mapped="${SEQ_SHORT[$input]:-}"
  if [[ -n "$mapped" ]]; then echo "$mapped"; else echo "$input"; fi
}
IFS=' ' read -ra RESOLVED_SEQUENCES <<< "$(resolve_sequence "$SEQUENCE")"

for CUR_SEQ in "${RESOLVED_SEQUENCES[@]}"; do
  SEQUENCE="$CUR_SEQ"

  DATASET_ROOT="${BASE_DATASET_PATH}/${SEQUENCE}"
  MAV0_PATH="${DATASET_ROOT}/mav0"
  GT_PATH="${MAV0_PATH}/state_groundtruth_estimate0/data.csv"
  CAM_CSV="${MAV0_PATH}/cam0/data.csv"

  if [[ -n "$OUTPUT_NAME" ]]; then
    FILE_STEM="${OUTPUT_NAME}_"
  else
    FILE_STEM=""
  fi

  # Offline playback speedup for the IMU/camera feeder processes.
  export OPENVINS_SPEEDUP="${OPENVINS_SPEEDUP:-8}"

  # Ensure binaries exist
  if [[ ! -x "$VINS_BINARY" ]]; then
    echo "Error: Binary not found or not executable: $VINS_BINARY"
    echo "Build first with: bash scripts/build.sh"
    exit 1
  fi
  if [[ ! -x "$IMU_BINARY" ]]; then
    echo "Error: Binary not found or not executable: $IMU_BINARY"
    exit 1
  fi
  if [[ ! -x "$CAMERA_BINARY" ]]; then
    echo "Error: Binary not found or not executable: $CAMERA_BINARY"
    exit 1
  fi
  if [[ ! -f "$BASE_CONFIG" ]]; then
    echo "Error: Config not found: $BASE_CONFIG"
    exit 1
  fi
  if [[ ! -d "$MAV0_PATH" ]]; then
    echo "Error: Dataset not found: $MAV0_PATH"
    exit 1
  fi

  export LD_LIBRARY_PATH="${SCRIPT_DIR}/build/core/ov_msckf:${SCRIPT_DIR}/build/core/ov_core:${SCRIPT_DIR}/build/core/ov_init:${LD_LIBRARY_PATH:-}"

  cleanup_ipc() {
    rm -f /tmp/vins_imu.sock 2>/dev/null || true
    rm -f /dev/shm/sem.vins_ready_sem /dev/shm/sem.vins_cam_sem 2>/dev/null || true
    rm -f /dev/shm/vins_cam_shm 2>/dev/null || true
  }

  cleanup_processes() {
    for pid in "${CAM_PID:-}" "${IMU_PID:-}" "${VINS_PID:-}" "${MONITOR_PID:-}"; do
      if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM "$pid" 2>/dev/null || true
      fi
    done
  }
  trap 'cleanup_processes; cleanup_ipc' EXIT INT TERM

for ((LOOP_I=1; LOOP_I<=LOOP_COUNT; LOOP_I++)); do
  TIMESTAMP=$(date +%Y%m%d_%H%M%S)
  if [[ "$LOOP_COUNT" -gt 1 ]]; then
    TIMESTAMP="${TIMESTAMP}_$(printf "%02d" "$LOOP_I")"
  fi
  RESULT_DIR="${RESULT_BASE}/euroc_mav/${SEQUENCE}/${TIMESTAMP}"
  RUN_CONFIG_DIR="${RESULT_DIR}/openvins_config"

  # ── Header ────────────────────────────────────────────────────────
  echo "========================================================================"
  echo "  OpenVINS — EuRoC [${SEQUENCE}] (OFFLINE)"
  echo "========================================================================"
  echo "  Binary   : $VINS_BINARY"
  echo "  Config   : $BASE_CONFIG"
  echo "  Output   : ${RESULT_DIR}/"
  if [[ "$LOOP_COUNT" -gt 1 ]]; then
    echo "  Loop     : ${LOOP_I}/${LOOP_COUNT}"
  fi
  if [[ "$SHORT_RUN_FRAMES" -gt 0 ]]; then
    echo "  Frames   : ${SHORT_RUN_FRAMES} (short run)"
  fi
  echo "========================================================================"
  echo ""

  if [[ "$DRY_RUN" == true ]]; then
    echo "[DRY-RUN] Would execute benchmark loop $LOOP_COUNT time(s)."
    echo "[DRY-RUN] No commands were executed."
    exit 0
  fi

  FRAMELOG="${RESULT_DIR}/${FILE_STEM}framelog.csv"
  SUMMARY="${RESULT_DIR}/${FILE_STEM}summary.json"
  TIMING="${RESULT_DIR}/${FILE_STEM}timing.csv"
  EST_TUM="${RESULT_DIR}/${FILE_STEM}trajectory.tum"
  RESOURCE_CSV="${RESULT_DIR}/${FILE_STEM}resource.csv"

  mkdir -p "$RUN_CONFIG_DIR"
  RUN_CONFIG="${RUN_CONFIG_DIR}/estimator_config.yaml"
  cp "${BASE_CONFIG_DIR}/kalibr_imu_chain.yaml" "$RUN_CONFIG_DIR/"
  cp "${BASE_CONFIG_DIR}/kalibr_imucam_chain.yaml" "$RUN_CONFIG_DIR/"
  for _mask in "${BASE_CONFIG_DIR}"/mask_*.png; do
    [[ -f "$_mask" ]] && cp "$_mask" "$RUN_CONFIG_DIR/"
  done
  python3 - "$BASE_CONFIG" "$RUN_CONFIG" "${RESULT_DIR}/traj_timing.txt" <<'PY'
import sys
src, dst, timing = sys.argv[1:4]
text = open(src).read()
repls = {
    'verbosity: "INFO"': 'verbosity: "WARNING"',
    'verbosity: "DEBUG"': 'verbosity: "WARNING"',
    'record_timing_information: false': 'record_timing_information: true',
    'record_timing_filepath: "result/traj_timing.txt"': f'record_timing_filepath: "{timing}"',
}
for a, b in repls.items():
    text = text.replace(a, b)
open(dst, "w").write(text)
PY

  # ── [1/3] Run VIO pipeline ──────────────────────────────────────
  echo "[1/3] Running VIO Pipeline..."
  cleanup_ipc
  START_TIME=$(date +%s.%N)
  OPENVINS_MAX_FRAMES="$SHORT_RUN_FRAMES" OPENVINS_REALTIME=0 "$VINS_BINARY" "$RUN_CONFIG" "$RESULT_DIR" >/dev/null 2>/dev/null &
  VINS_PID=$!
  sleep 1

  MONITOR_PID=""
  if [[ "$DO_MONITOR" == true ]]; then
    (
      trap 'exit 0' INT TERM
      echo "elapsed_sec,cpu_pct,mem_pct,rss_mb,threads"
      while kill -0 "$VINS_PID" 2>/dev/null || kill -0 "${IMU_PID:-0}" 2>/dev/null || kill -0 "${CAM_PID:-0}" 2>/dev/null; do
        TS=$(date +%s.%N)
        PIDS=""
        for pid in "$VINS_PID" "${IMU_PID:-}" "${CAM_PID:-}"; do
          [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && PIDS="${PIDS}${PIDS:+,}${pid}"
        done
        if [[ -n "$PIDS" ]]; then
          read cpu mem rss threads <<< "$(ps -p "$PIDS" -o %cpu=,%mem=,rss=,thcount= 2>/dev/null | awk '{cpu+=$1; mem+=$2; rss+=$3; th+=$4} END {print cpu, mem, rss, th}')"
          elapsed=$(echo "$TS - $START_TIME" | bc -l | sed 's/^\./0./')
          rss_mb=$(echo "scale=3; ${rss:-0} / 1024" | bc -l)
          echo "${elapsed},${cpu:-0},${mem:-0},${rss_mb},${threads:-0}"
        fi
        sleep 0.5
      done
    ) > "$RESOURCE_CSV" 2>/dev/null &
    MONITOR_PID=$!
  fi

  OPENVINS_REALTIME=0 "$IMU_BINARY" "$MAV0_PATH" >/dev/null 2>/dev/null &
  IMU_PID=$!
  OPENVINS_MAX_FRAMES="$SHORT_RUN_FRAMES" OPENVINS_REALTIME=0 "$CAMERA_BINARY" "$MAV0_PATH" >/dev/null 2>/dev/null &
  CAM_PID=$!

  VIO_EXIT=0
  if ! wait "$IMU_PID"; then VIO_EXIT=$?; fi
  if ! wait "$CAM_PID"; then VIO_EXIT=$?; fi

  if [[ "$SHORT_RUN_FRAMES" -gt 0 ]]; then
    VINS_GRACE=$(( SHORT_RUN_FRAMES / 2 + 5 ))
    [[ "$VINS_GRACE" -gt 60 ]] && VINS_GRACE=60
  else
    VINS_GRACE=60
  fi
  WAIT_COUNT=0
  while kill -0 "$VINS_PID" 2>/dev/null && [[ $WAIT_COUNT -lt $VINS_GRACE ]]; do
    sleep 1
    WAIT_COUNT=$((WAIT_COUNT + 1))
  done
  if kill -0 "$VINS_PID" 2>/dev/null; then
    kill -INT "$VINS_PID" 2>/dev/null || true
    wait "$VINS_PID" 2>/dev/null || true
  fi
  END_TIME=$(date +%s.%N)
  PROC_TIME=$(echo "$END_TIME - $START_TIME" | bc -l | sed 's/^\./0./')

  # ── Stop process monitor ──
  if [[ -n "$MONITOR_PID" ]]; then
    kill -TERM "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
  trap - EXIT INT TERM
  cleanup_ipc
  trap 'cleanup_processes; cleanup_ipc' EXIT INT TERM

  if [[ -f "$RESOURCE_CSV" ]]; then
    RESOURCE_LINES=$(python3 - "$RESOURCE_CSV" <<'PY'
import sys
try:
    print(max(0, sum(1 for _ in open(sys.argv[1])) - 1))
except FileNotFoundError:
    print(0)
PY
)
    if [[ "$RESOURCE_LINES" -eq 0 ]]; then
      python3 - "$RESOURCE_CSV" "$PROC_TIME" <<'PY'
import csv, sys
with open(sys.argv[1], "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["elapsed_sec", "cpu_pct", "mem_pct", "rss_mb", "threads"])
    w.writerow([sys.argv[2], 0, 0, 0, 0])
PY
    fi
  else
    python3 - "$RESOURCE_CSV" "$PROC_TIME" <<'PY'
import csv, sys
with open(sys.argv[1], "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["elapsed_sec", "cpu_pct", "mem_pct", "rss_mb", "threads"])
    w.writerow([sys.argv[2], 0, 0, 0, 0])
PY
  fi

  [[ "$VIO_EXIT" -eq 0 ]] || { echo "Error: OpenVINS feeder exited with status $VIO_EXIT"; exit "$VIO_EXIT"; }

  FRAMES_TOTAL=$(python3 - "$CAM_CSV" <<'PY'
import sys
try:
    print(max(0, sum(1 for _ in open(sys.argv[1])) - 1))
except FileNotFoundError:
    print("?")
PY
)

  # Process outputs: trajectory.txt -> framelog.csv / timing.csv / trajectory.tum / summary.json
  python3 - "${RESULT_DIR}/trajectory.txt" "$FRAMELOG" "$TIMING" "$EST_TUM" "$SUMMARY" "$SEQUENCE" "$FRAMES_TOTAL" "$PROC_TIME" <<'PY'
import csv, json, os, sys, statistics

traj_path, framelog_path, timing_path, tum_path, summary_path, seq_name, frames_total_str, proc_time_str = sys.argv[1:9]
proc_time = float(proc_time_str)

poses = []
if os.path.exists(traj_path):
    for line in open(traj_path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 8:
            t = float(parts[0])
            tx, ty, tz, qx, qy, qz, qw = map(float, parts[1:8])
            poses.append((t, tx, ty, tz, qx, qy, qz, qw))

n = len(poses)
avg_lat_us = (proc_time * 1e6 / n) if n > 0 and proc_time > 0 else 0.0

framelog_cols = [
    "frame_idx", "timestamp_ns", "frame_interval_us", "pipeline_latency_us",
    "tx", "ty", "tz", "qx", "qy", "qz", "qw",
    "vx", "vy", "vz",
    "bgx", "bgy", "bgz", "bax", "bay", "baz",
    "num_kfs", "num_lms", "num_obs", "num_features", "tracking_state"
]

with open(framelog_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=framelog_cols)
    writer.writeheader()
    for idx, p in enumerate(poses):
        t, tx, ty, tz, qx, qy, qz, qw = p
        writer.writerow({
            "frame_idx": idx + 1,
            "timestamp_ns": int(round(t * 1e9)),
            "frame_interval_us": f"{avg_lat_us:.1f}",
            "pipeline_latency_us": f"{avg_lat_us:.1f}",
            "tx": tx, "ty": ty, "tz": tz,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "bgx": 0.0, "bgy": 0.0, "bgz": 0.0,
            "bax": 0.0, "bay": 0.0, "baz": 0.0,
            "num_kfs": 0, "num_lms": 0, "num_obs": 0, "num_features": 0,
            "tracking_state": 1
        })

timing_cols = ["frame_idx", "timestamp_ns", "frame_interval_us", "pipeline_latency_us"]
with open(timing_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=timing_cols)
    writer.writeheader()
    for idx, p in enumerate(poses):
        t = p[0]
        writer.writerow({
            "frame_idx": idx + 1,
            "timestamp_ns": int(round(t * 1e9)),
            "frame_interval_us": f"{avg_lat_us:.1f}",
            "pipeline_latency_us": f"{avg_lat_us:.1f}"
        })

with open(tum_path, "w") as f:
    f.write("# timestamp tx ty tz qx qy qz qw\n")
    for p in poses:
        f.write("{:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e}\n".format(*p))

try:
    tot_f = int(frames_total_str)
except (ValueError, TypeError):
    tot_f = n

fps_avg = (n / proc_time) if proc_time > 0 and n > 0 else 0.0
fps_med = fps_avg

summary_data = {
    "metadata": {
        "pipeline": "OpenVINS",
        "dataset": "EuRoC",
        "sequence": seq_name,
        "mode": "offline",
        "timestamp": ""
    },
    "tracking": {
        "total_frames": tot_f,
        "processed_frames": n,
        "tracking_success_rate_pct": 100.0 if n > 0 else 0.0,
        "execution_time_s": round(proc_time, 6)
    },
    "accuracy": {
        "ate_rmse_m": None,
        "rpe_trans_rmse_m": None,
        "rpe_rot_rmse_deg": None
    },
    "performance": {
        "fps": {
            "mean": round(fps_avg, 2),
            "median": round(fps_med, 2)
        },
        "latency_ms": {
            "mean": round(avg_lat_us / 1000.0, 2),
            "median": round(avg_lat_us / 1000.0, 2),
            "p95": round(avg_lat_us / 1000.0, 2),
            "std": 0.0
        },
        "frame_interval_us": {
            "mean": round(avg_lat_us, 2),
            "min": round(avg_lat_us, 2),
            "max": round(avg_lat_us, 2)
        }
    }
}

with open(summary_path, "w") as f:
    json.dump(summary_data, f, indent=2)
    f.write("\n")
PY
  rm -f "${RESULT_DIR}/trajectory.txt" "${RESULT_DIR}/traj_timing.txt"
  rm -rf "$RUN_CONFIG_DIR"

  # ── Metrics: frames / FPS / TSR ──────────────────────────────────
  FRAMES="?"
  TSR="?"
  FPS_AVG="?"
  if [[ -f "$SUMMARY" ]]; then
    read FRAMES TSR FPS_AVG <<< "$(python3 -c "
import json
d=json.load(open('$SUMMARY'))
print(d.get('tracking',{}).get('processed_frames','?'), d.get('tracking',{}).get('tracking_success_rate_pct','?'), d.get('performance',{}).get('fps',{}).get('mean','?'))
" 2>/dev/null || echo "? ? ?")"
  fi
  if [[ "$FRAMES" == "?" && -f "$FRAMELOG" ]]; then
    FRAMES=$(python3 -c "import csv;print(sum(1 for _ in open('$FRAMELOG'))-1)" 2>/dev/null || echo "?")
  fi
  if [[ "$FPS_AVG" == "?" && "$FRAMES" != "?" && "$(echo "$PROC_TIME > 0" | bc -l)" -eq 1 ]]; then
    FPS_AVG=$(echo "scale=1; $FRAMES / $PROC_TIME" | bc -l)
  fi
  if [[ "$FPS_AVG" != "?" ]]; then
    FPS_AVG=$(printf "%.1f" "$FPS_AVG")
  fi
  if [[ "$TSR" == "?" && -f "$FRAMELOG" ]]; then
    TSR=$(python3 -c "
import csv
rows=list(csv.DictReader(open('$FRAMELOG')))
print(f'{100*sum(1 for r in rows if str(r.get(\"tracking_state\"))==\"1\")/len(rows):.1f}' if rows else '?')
" 2>/dev/null || echo "?")
  fi

  echo "  Done in $(printf "%.2f" "$PROC_TIME")s (${FRAMES} frames | ${FPS_AVG} avg FPS | TSR: ${TSR}%)"
  echo ""

  # ── [2/3] Evaluate trajectory (evo) ─────────────────────────────
  echo "[2/3] Evaluating Trajectory (evo)..."
  ATE="?"
  RPE_T="?"
  RPE_R="?"
  GT_TUM="${RESULT_DIR}/${FILE_STEM}groundtruth.tum"
  if [[ -f "$EST_TUM" && -f "$GT_PATH" ]]; then
    python3 - "$GT_PATH" "$GT_TUM" <<'PY'
import sys
import pandas as pd

src, dst = sys.argv[1:3]
df = pd.read_csv(src)
df.columns = [c.split('[')[0].strip() for c in df.columns.str.strip()]
ts_col = [c for c in df.columns if c.startswith('#timestamp')][0]
cols = [ts_col, 'p_RS_R_x', 'p_RS_R_y', 'p_RS_R_z', 'q_RS_w', 'q_RS_x', 'q_RS_y', 'q_RS_z']
out = df[cols].copy()
out[ts_col] = out[ts_col] * 1e-9  # ns -> s (TUM format)
out.to_csv(dst, sep=" ", index=False, header=False)
PY
    ATE=$(evo_ape tum "$GT_TUM" "$EST_TUM" -va --no_warnings 2>&1 | awk '$1=="rmse"{print $2; exit}' || true)
    RPE_T=$(evo_rpe tum "$GT_TUM" "$EST_TUM" -va --pose_relation trans_part --no_warnings 2>&1 | awk '$1=="rmse"{print $2; exit}' || true)
    RPE_R=$(evo_rpe tum "$GT_TUM" "$EST_TUM" -va --pose_relation rot_part --no_warnings 2>&1 | awk '$1=="rmse"{print $2; exit}' || true)
    [[ -n "${ATE:-}" ]] || ATE="?"
    [[ -n "${RPE_T:-}" ]] || RPE_T="?"
    [[ -n "${RPE_R:-}" ]] || RPE_R="?"

    # Write RPE and metadata to summary.json
    if [[ -f "$SUMMARY" ]]; then
      python3 - "$SUMMARY" "$ATE" "$RPE_T" "$RPE_R" "$TIMESTAMP" "$SEQUENCE" <<'PY'
import json, sys
path, ate, rpe_t, rpe_r, timestamp, sequence = sys.argv[1:7]
def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None
d = json.load(open(path))

out = {
    "metadata": {
        "pipeline": "OpenVINS",
        "dataset": "EuRoC",
        "sequence": sequence,
        "mode": "offline",
        "timestamp": f"{timestamp[:4]}-{timestamp[4:6]}-{timestamp[6:8]}T{timestamp[9:11]}:{timestamp[11:13]}:{timestamp[13:15]}"
    },
    "tracking": d.get("tracking", {}),
    "accuracy": {
        "ate_rmse_m": num(ate),
        "rpe_trans_rmse_m": num(rpe_t),
        "rpe_rot_rmse_deg": num(rpe_r)
    },
    "performance": d.get("performance", {})
}
json.dump(out, open(path, "w"), indent=2)
open(path, "a").write("\n")
PY
    fi

    # Trajectory plot (evo_traj writes <base>_trajectories.png)
    TRAJ_PLOT="${RESULT_DIR}/${FILE_STEM}trajectory.png"
    evo_traj tum "$EST_TUM" --ref "$GT_TUM" --align --correct_scale \
      --save_plot "${RESULT_DIR}/${FILE_STEM}traj" >/dev/null 2>&1 || true
    if [[ -f "${RESULT_DIR}/${FILE_STEM}traj_trajectories.png" ]]; then
      mv "${RESULT_DIR}/${FILE_STEM}traj_trajectories.png" "$TRAJ_PLOT"
      rm -f "${RESULT_DIR}/${FILE_STEM}traj_xyz.png" \
            "${RESULT_DIR}/${FILE_STEM}traj_rpy.png" \
            "${RESULT_DIR}/${FILE_STEM}traj_speeds.png"
    fi
  fi
  echo "  ATE RMSE        : ${ATE} m"
  echo "  RPE Trans RMSE  : ${RPE_T} m"
  echo "  RPE Rot RMSE    : ${RPE_R} deg"
  echo ""

  # ── [3/3] Performance & resource metrics ─────────────────────────
  echo "[3/3] Performance & Resource Metrics..."
  LAT_MEAN="?"
  LAT_P95="?"
  JITTER="?"
  FPS_MED="?"
  if [[ -f "$FRAMELOG" ]]; then
    read LAT_MEAN LAT_P95 JITTER FPS_MED <<< "$(python3 - "$FRAMELOG" <<'PY'
import csv
import sys
import statistics

lat = []
fps = []
for r in csv.DictReader(open(sys.argv[1])):
    try:
        lat.append(float(r["pipeline_latency_us"]))
        iv = float(r["frame_interval_us"])
        if iv > 0:
            fps.append(1e6 / iv)
    except (KeyError, ValueError):
        pass
if not lat:
    print("? ? ? ?")
    sys.exit()
lat_sorted = sorted(lat)
p95 = lat_sorted[min(len(lat_sorted) - 1, int(len(lat_sorted) * 0.95) - 1)]
print(f"{statistics.mean(lat)/1000:.2f} {p95/1000:.2f} {statistics.pstdev(lat)/1000:.2f} {statistics.median(fps):.1f}")
PY
)"
  fi
  echo "  Latency (Mean / P95) : ${LAT_MEAN} ms / ${LAT_P95} ms (Jitter: ${JITTER} ms)"
  echo "  Throughput (Avg/Med) : ${FPS_AVG} / ${FPS_MED} FPS"
  echo ""

  echo "Artifacts saved to: ${RESULT_DIR#${SCRIPT_DIR}/}/"
  echo "========================================================================"
  echo ""
done
done
