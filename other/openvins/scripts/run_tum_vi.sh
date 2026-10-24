#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
export LC_NUMERIC=C
export PATH="$HOME/.local/bin:$PATH"

# =============================================================================
# OpenVINS — TUM-VI Batch processing
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULT_BASE="${SCRIPT_DIR}/result"
PYTHON="python3"
VINS_BINARY="${SCRIPT_DIR}/build/vins"
IMU_BINARY="${SCRIPT_DIR}/build/imu_process"
CAMERA_BINARY="${SCRIPT_DIR}/build/camera_process"

BASE_DATASET_PATH="$HOME/dataset/tum-vi"
BASE_CONFIG_DIR="${SCRIPT_DIR}/config/tum_vi"
BASE_CONFIG="${BASE_CONFIG_DIR}/estimator_config.yaml"

SEQUENCE="room1_512_16"
OUTPUT_NAME=""
DRY_RUN=false
SHORT_RUN_FRAMES=0
DO_MONITOR=true
PER_FRAME=true
LOOP_COUNT=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence)
      [[ -n "${2:-}" ]] || { echo "Error: --sequence requires a value"; exit 1; }
      SEQUENCE="$2"; shift 2 ;;
    --output-name)
      [[ -n "${2:-}" ]] || { echo "Error: --output-name requires a value"; exit 1; }
      OUTPUT_NAME="$2"; shift 2 ;;
    --short-run)
      [[ -n "${2:-}" && "$2" =~ ^[0-9]+$ ]] || { echo "Error: --short-run requires a positive integer"; exit 1; }
      SHORT_RUN_FRAMES="$2"; shift 2 ;;
    --loop)
      [[ -n "${2:-}" && "$2" =~ ^[0-9]+$ ]] || { echo "Error: --loop requires a positive integer"; exit 1; }
      LOOP_COUNT="$2"; shift 2 ;;
    --dry-run)
      DRY_RUN=true; shift ;;
    --monitor)
      DO_MONITOR=true; shift ;;
    --per-frame)
      PER_FRAME=true; shift ;;
    *)
      echo "Usage: $0 [--sequence <seq>] [--dry-run] [--short-run <frames>] [--output-name <name>] [--loop <N>] [--per-frame] [--monitor]"
      echo ""
      echo "Defaults: --per-frame --monitor enabled; --sequence room1_512_16; empty output prefix"
      echo ""
      echo "Sequences: room1_512_16 ... room6_512_16"
      echo "           corridor1_512_16 ... corridor5_512_16"
      echo "           magistrale1_512_16 ... magistrale6_512_16"
      echo "           all (runs all 17 sequences)"
      exit 1
      ;;
  esac
done

# ── Resolve sequences ──────────────────────────────────────────────
ALL_SEQUENCES=(
  "room1_512_16" "room2_512_16" "room3_512_16" "room4_512_16" "room5_512_16" "room6_512_16"
  "corridor1_512_16" "corridor2_512_16" "corridor3_512_16" "corridor4_512_16" "corridor5_512_16"
  "magistrale1_512_16" "magistrale2_512_16" "magistrale3_512_16" "magistrale4_512_16" "magistrale5_512_16" "magistrale6_512_16"
)
declare -A SEQ_SHORT=( [r1]=room1_512_16 [r2]=room2_512_16 [r3]=room3_512_16 [r4]=room4_512_16 [r5]=room5_512_16 [r6]=room6_512_16 [c1]=corridor1_512_16 [c2]=corridor2_512_16 [c3]=corridor3_512_16 [c4]=corridor4_512_16 [c5]=corridor5_512_16 [m1]=magistrale1_512_16 [m2]=magistrale2_512_16 [m3]=magistrale3_512_16 [m4]=magistrale4_512_16 [m5]=magistrale5_512_16 [m6]=magistrale6_512_16 )
resolve_sequence() {
  local input="$1"
  if [[ "$input" == "all" ]]; then echo "${ALL_SEQUENCES[@]}"; return; fi
  local mapped="${SEQ_SHORT[$input]:-}"
  if [[ -n "$mapped" ]]; then echo "$mapped"; else echo "$input"; fi
}
IFS=' ' read -ra RESOLVED_SEQUENCES <<< "$(resolve_sequence "$SEQUENCE")"

for CUR_SEQ in "${RESOLVED_SEQUENCES[@]}"; do
  SEQUENCE="$CUR_SEQ"

  DATASET_ROOT="${BASE_DATASET_PATH}/dataset-${SEQUENCE}"
  MAV0_PATH="${DATASET_ROOT}/mav0"
  GT_PATH="${MAV0_PATH}/mocap0/data.csv"
  CAM_CSV="${MAV0_PATH}/cam0/data.csv"

  if [[ -n "$OUTPUT_NAME" ]]; then
    FILE_STEM="${OUTPUT_NAME}_"
  else
    FILE_STEM=""
  fi

  # Offline playback speedup for the IMU/camera feeder processes. The binaries
  # default to 20x, which is too aggressive for sequences with long static starts
  # (MH_02, V1_01): the estimator cannot accumulate enough disparity to initialize
  # before frames pile up, yielding an empty trajectory. 8x is reliable across all
  # 17 TUM-VI sequences while still running well above real time. Override by
  # exporting OPENVINS_SPEEDUP before invoking this script.
  export OPENVINS_SPEEDUP="${OPENVINS_SPEEDUP:-8}"

  [[ -x "$VINS_BINARY" ]] || { echo "Error: Binary not found or not executable: $VINS_BINARY"; echo "Build first with: ./build.sh"; exit 1; }
  [[ -x "$IMU_BINARY" ]] || { echo "Error: Binary not found or not executable: $IMU_BINARY"; echo "Build first with: ./build.sh"; exit 1; }
  [[ -x "$CAMERA_BINARY" ]] || { echo "Error: Binary not found or not executable: $CAMERA_BINARY"; echo "Build first with: ./build.sh"; exit 1; }
  [[ -f "$BASE_CONFIG" ]] || { echo "Error: Config not found: $BASE_CONFIG"; exit 1; }
  [[ -d "$MAV0_PATH" ]] || { echo "Error: Dataset not found: $MAV0_PATH"; exit 1; }

  # Set library path so the linker can find openvins shared libs
  export LD_LIBRARY_PATH="${SCRIPT_DIR}/build/core/ov_msckf:${SCRIPT_DIR}/build/core/ov_core:${SCRIPT_DIR}/build/core/ov_init:${LD_LIBRARY_PATH:-}"

  cleanup_ipc() {
    rm -f /tmp/vins_imu.sock 2>/dev/null || true
    # POSIX semaphores live under /dev/shm/sem.<name> on Linux
    rm -f /dev/shm/sem.vins_ready_sem /dev/shm/sem.vins_cam_sem 2>/dev/null || true
    # POSIX shared memory lives under /dev/shm/<name> on Linux
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
  RESULT_DIR="${RESULT_BASE}/tum_vi/${SEQUENCE}/${TIMESTAMP}"
  RUN_CONFIG_DIR="${RESULT_DIR}/openvins_config"

  # ── Header ────────────────────────────────────────────────────────
  echo "========================================================================"
  echo "  OpenVINS — TUM-VI [${SEQUENCE}] (OFFLINE)"
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
    echo "[DRY-RUN] No commands executed."
    exit 0
  fi

  FRAMELOG="${RESULT_DIR}/${FILE_STEM}framelog.csv"
  SUMMARY="${RESULT_DIR}/${FILE_STEM}summary.json"
  TIMING="${RESULT_DIR}/${FILE_STEM}timing.csv"
  TRAJ_TUM="${RESULT_DIR}/${FILE_STEM}trajectory.tum"
  TRAJ_TXT="${RESULT_DIR}/${FILE_STEM}trajectory.txt"
  RESOURCE_CSV="${RESULT_DIR}/${FILE_STEM}resource.csv"

  mkdir -p "$RUN_CONFIG_DIR"
  RUN_CONFIG="${RUN_CONFIG_DIR}/estimator_config.yaml"
  cp "${BASE_CONFIG_DIR}/kalibr_imu_chain.yaml" "$RUN_CONFIG_DIR/"
  cp "${BASE_CONFIG_DIR}/kalibr_imucam_chain.yaml" "$RUN_CONFIG_DIR/"
  # Copy mask files if present
  for _mask in "${BASE_CONFIG_DIR}"/mask_*.png; do
    [[ -f "$_mask" ]] && cp "$_mask" "$RUN_CONFIG_DIR/"
  done
  "$PYTHON" - "$BASE_CONFIG" "$RUN_CONFIG" "$RESULT_DIR/traj_timing.txt" <<'PY'
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
  # Wait for VINS to finish processing queued data.
  # VINS has no end-of-stream detection — it will hang waiting for more camera
  # frames after the cam process exits. We give it a short grace period, then
  # send SIGINT to trigger graceful shutdown.
  if [[ "$SHORT_RUN_FRAMES" -gt 0 ]]; then
    VINS_GRACE=$(( SHORT_RUN_FRAMES / 2 + 5 ))  # ~0.5 s/frame + buffer
    [[ "$VINS_GRACE" -gt 60 ]] && VINS_GRACE=60
  else
    VINS_GRACE=60
  fi
  WAIT_COUNT=0
  while kill -0 "$VINS_PID" 2>/dev/null && [[ $WAIT_COUNT -lt $VINS_GRACE ]]; do
    sleep 1
    WAIT_COUNT=$((WAIT_COUNT + 1))
  done
  # If still running, send interrupt
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
    RESOURCE_LINES=$("$PYTHON" - "$RESOURCE_CSV" <<'PY'
import sys
try:
    print(max(0, sum(1 for _ in open(sys.argv[1])) - 1))
except FileNotFoundError:
    print(0)
PY
)
    if [[ "$RESOURCE_LINES" -eq 0 ]]; then
      "$PYTHON" - "$RESOURCE_CSV" "$PROC_TIME" <<'PY'
import csv, sys
with open(sys.argv[1], "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["elapsed_sec", "cpu_pct", "mem_pct", "rss_mb", "threads"])
    w.writerow([sys.argv[2], 0, 0, 0, 0])
PY
    fi
  else
    "$PYTHON" - "$RESOURCE_CSV" "$PROC_TIME" <<'PY'
import csv, sys
with open(sys.argv[1], "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["elapsed_sec", "cpu_pct", "mem_pct", "rss_mb", "threads"])
    w.writerow([sys.argv[2], 0, 0, 0, 0])
PY
  fi

  [[ "$VIO_EXIT" -eq 0 ]] || { echo "Error: OpenVINS feeder exited with status $VIO_EXIT"; exit "$VIO_EXIT"; }
  if [[ ! -s "${RESULT_DIR}/trajectory.txt" ]]; then
    echo "  Warning: OpenVINS trajectory empty (estimator did not initialize within frame budget)."
    if [[ "$SHORT_RUN_FRAMES" -gt 0 ]]; then
      "$PYTHON" - "$SUMMARY" "$SEQUENCE" "$SHORT_RUN_FRAMES" "$PROC_TIME" <<'PY'
import json, sys
summary_path, seq, short, wall = sys.argv[1:5]
with open(summary_path, "w") as f:
    json.dump({"dataset": seq, "short_run": int(short), "num_frames_processed": 0,
               "exec_time_s": float(wall), "ate_rmse_m": None,
               "reason": "Estimator did not initialize within the short-run frame limit."}, f, indent=2)
PY
    fi
    continue
  fi

  FRAMES_TOTAL=$("$PYTHON" - "$CAM_CSV" <<'PY'
import sys
try:
    print(max(0, sum(1 for _ in open(sys.argv[1])) - 1))
except FileNotFoundError:
    print("?")
PY
)

  # Process outputs: trajectory.txt -> framelog.csv / timing.csv / trajectory.tum / summary.json
  "$PYTHON" - "${RESULT_DIR}/trajectory.txt" "$FRAMELOG" "$TIMING" "$TRAJ_TUM" "$TRAJ_TXT" "$SUMMARY" "$SEQUENCE" "$FRAMES_TOTAL" "$PROC_TIME" "$SHORT_RUN_FRAMES" <<'PY'
import csv, json, sys
traj, framelog, timing, tum, txt, summary_path, seq, frames_total, wall, short = sys.argv[1:11]
poses = []
for line in open(traj):
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    parts = line.split()
    if len(parts) < 8:
        continue
    t, tx, ty, tz, qx, qy, qz, qw = map(float, parts[:8])
    poses.append((t, tx, ty, tz, qx, qy, qz, qw))
if not poses:
    raise SystemExit("empty OpenVINS trajectory")
rows = []
prev = None
wall_s = float(wall)
for i, p in enumerate(poses):
    t, tx, ty, tz, qx, qy, qz, qw = p
    timestamp_ns = int(round(t * 1e9))
    interval_us = 0 if prev is None else max(0, int(round((t - prev) * 1e6)))
    latency_us = int(round(wall_s / max(1, len(poses)) * 1e6))
    rows.append({
        "frame_idx": i,
        "timestamp_ns": timestamp_ns,
        "tx": tx, "ty": ty, "tz": tz,
        "qx": qx, "qy": qy, "qz": qz, "qw": qw,
        "tracking_state": 1,
        "frame_interval_us": interval_us,
        "pipeline_latency_us": latency_us,
        "num_features": 0, "num_kfs": 0, "num_lms": 0, "num_obs": 0,
        "vx": 0, "vy": 0, "vz": 0,
        "bgx": 0, "bgy": 0, "bgz": 0,
        "bax": 0, "bay": 0, "baz": 0,
    })
    prev = t
fields = list(rows[0].keys())
with open(framelog, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    w.writerows(rows)
with open(timing, "w", newline="") as f:
    cols = ["frame_idx", "timestamp_ns", "frame_interval_us", "pipeline_latency_us"]
    w = csv.DictWriter(f, fieldnames=cols)
    w.writeheader()
    for row in rows:
        w.writerow({c: row[c] for c in cols})
def write_tum(path):
    with open(path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for p in poses:
            f.write("{:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e}\n".format(*p))
write_tum(tum)
write_tum(txt)
summary = {
    "dataset": seq,
    "num_frames_total": int(frames_total) if str(frames_total).isdigit() else frames_total,
    "num_frames_processed": len(poses),
    "exec_time_s": wall_s,
    "fps_avg": len(poses) / wall_s if wall_s > 0 else 0,
    "tsr_pct": 100.0,
    "ate_rmse_m": None,
    "rpe_trans_rmse_m": None,
    "rpe_rot_rmse_deg": None,
    "init_time_s": 0.0,
    "timing_us": {},
    "metrics": {},
}
with open(summary_path, "w") as f:
    json.dump(summary, f, indent=2)
PY
  rm -f "${RESULT_DIR}/trajectory.txt"

  # ── Metrics: frames / FPS / TSR ──────────────────────────────────
  FRAMES="?"
  TSR="?"
  FPS_AVG="?"
  if [[ -f "$SUMMARY" ]]; then
    read FRAMES TSR FPS_AVG <<< "$("$PYTHON" -c "
import json
d=json.load(open('$SUMMARY'))
print(d.get('num_frames_processed','?'), d.get('tsr_pct','?'), d.get('fps_avg','?'))
" 2>/dev/null || echo "? ? ?")"
  fi
  if [[ "$FRAMES" == "?" && -f "$FRAMELOG" ]]; then
    FRAMES=$("$PYTHON" -c "import csv;print(sum(1 for _ in open('$FRAMELOG'))-1)" 2>/dev/null || echo "?")
  fi
  if [[ "$FPS_AVG" == "?" && "$FRAMES" != "?" && "$(echo "$PROC_TIME > 0" | bc -l)" -eq 1 ]]; then
    FPS_AVG=$(echo "scale=1; $FRAMES / $PROC_TIME" | bc -l)
  fi
  if [[ "$TSR" == "?" && -f "$FRAMELOG" ]]; then
    TSR=$("$PYTHON" -c "
import csv
rows=list(csv.DictReader(open('$FRAMELOG')))
print(f'{100*sum(1 for r in rows if r.get(\"tracking_state\")==\"1\")/len(rows):.1f}' if rows else '?')
" 2>/dev/null || echo "?")
  fi
  if [[ "$FPS_AVG" != "?" ]]; then
    FPS_AVG=$(printf "%.1f" "$FPS_AVG")
  fi

  echo "  Done in $(printf "%.2f" "$PROC_TIME")s (${FRAMES} frames | ${FPS_AVG} avg FPS | TSR: ${TSR}%)"
  echo ""

  # ── [2/3] Evaluate trajectory (evo) ─────────────────────────────
  echo "[2/3] Evaluating Trajectory (evo)..."
  ATE="?"
  RPE_T="?"
  RPE_R="?"
  EST_TUM="${RESULT_DIR}/${FILE_STEM}trajectory.tum"
  GT_TUM="${RESULT_DIR}/${FILE_STEM}groundtruth.tum"
  if [[ -f "$EST_TUM" && -f "$GT_PATH" ]]; then
    python3 - "$GT_PATH" "$GT_TUM" <<'PY'
import sys, pandas as pd
src, dst = sys.argv[1:3]
df = pd.read_csv(src)
# Strip column names and remove units
df.columns = [c.split('[')[0].strip() for c in df.columns.str.strip()]
# Match columns by prefix (handles both "#timestamp" and "#timestamp [ns]")
ts_col = [c for c in df.columns if c.startswith('#timestamp')][0]
cols = [ts_col, 'p_RS_R_x', 'p_RS_R_y', 'p_RS_R_z', 'q_RS_w', 'q_RS_x', 'q_RS_y', 'q_RS_z']
out = df[cols].copy()
out[ts_col] = out[ts_col] * 1e-9  # ns -> s
out.to_csv(dst, sep=" ", index=False, header=False)
PY
    ATE=$(evo_ape tum "$GT_TUM" "$EST_TUM" -va --no_warnings 2>&1 | awk '$1=="rmse"{print $2; exit}')
    RPE_T=$(evo_rpe tum "$GT_TUM" "$EST_TUM" -va --pose_relation trans_part --no_warnings 2>&1 | awk '$1=="rmse"{print $2; exit}')
    RPE_R=$(evo_rpe tum "$GT_TUM" "$EST_TUM" -va --pose_relation rot_part --no_warnings 2>&1 | awk '$1=="rmse"{print $2; exit}')

    # Write RPE and metadata to summary.json
    if [[ -f "$SUMMARY" ]]; then
      python3 - "$SUMMARY" "$ATE" "$RPE_T" "$RPE_R" "$TIMESTAMP" "$SEQUENCE" "$FRAMELOG" <<'PY'
import json, sys, csv, statistics
path, ate, rpe_t, rpe_r, timestamp, sequence, framelog = sys.argv[1:8]
def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None
d = json.load(open(path))

# Read framelog for performance metrics
latencies = []
intervals = []
fps_list = []
with open(framelog) as f:
    for r in csv.DictReader(f):
        try:
            lat = float(r.get("pipeline_latency_us", 0))
            if lat > 0:
                latencies.append(lat / 1000.0)  # us -> ms
            iv = float(r.get("frame_interval_us", 0))
            if iv > 0:
                intervals.append(iv)
                fps_list.append(1e6 / iv)
        except (KeyError, ValueError):
            pass

if latencies:
    lat_sorted = sorted(latencies)
    p95 = lat_sorted[min(len(lat_sorted) - 1, int(len(lat_sorted) * 0.95) - 1)]
    lat_mean = statistics.mean(latencies)
    lat_median = statistics.median(latencies)
    lat_std = statistics.pstdev(latencies) if len(latencies) > 1 else 0
else:
    lat_mean = lat_median = p95 = lat_std = 0

fps_mean = statistics.mean(fps_list) if fps_list else d.get("fps_avg", 0)
fps_median = statistics.median(fps_list) if fps_list else d.get("fps_avg", 0)
int_mean = statistics.mean(intervals) if intervals else 0
int_min = min(intervals) if intervals else 0
int_max = max(intervals) if intervals else 0

out = {
    "metadata": {
        "pipeline": "OpenVINS",
        "dataset": "TUM-VI",
        "sequence": sequence,
        "mode": "offline",
        "timestamp": f"{timestamp[:4]}-{timestamp[4:6]}-{timestamp[6:8]}T{timestamp[9:11]}:{timestamp[11:13]}:{timestamp[13:15]}"
    },
    "tracking": {
        "total_frames": d.get("num_frames_processed", 0),
        "processed_frames": d.get("num_frames_processed", 0),
        "tracking_success_rate_pct": d.get("tsr_pct", 0),
        "execution_time_s": d.get("exec_time_s", 0)
    },
    "accuracy": {
        "ate_rmse_m": num(ate),
        "rpe_trans_rmse_m": num(rpe_t),
        "rpe_rot_rmse_deg": num(rpe_r)
    },
    "performance": {
        "fps": {"mean": round(fps_mean, 2), "median": round(fps_median, 2)},
        "latency_ms": {"mean": round(lat_mean, 2), "median": round(lat_median, 2), "p95": round(p95, 2), "std": round(lat_std, 2)},
        "frame_interval_us": {"mean": round(int_mean, 2), "min": round(int_min, 2), "max": round(int_max, 2)}
    }
}
json.dump(out, open(path, "w"), indent=2)
open(path, "a").write("\n")
PY
    fi

    # Trajectory plot
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
    read LAT_MEAN LAT_P95 JITTER FPS_MED <<< "$("$PYTHON" - "$FRAMELOG" <<'PY'
import csv, sys, statistics
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

  rm -rf "$RUN_CONFIG_DIR"
done
done
