#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
export LC_NUMERIC=C
export PATH="$HOME/.local/bin:$PATH"

# =============================================================================
# LarVIO — TUM-VI Batch processing
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULT_BASE="${SCRIPT_DIR}/result"
PYTHON="python3"
BINARY="${SCRIPT_DIR}/build/larvio_eval"

BASE_DATASET_PATH="$HOME/dataset/tum-vi"
CONFIG_PATH="${SCRIPT_DIR}/config/tum_vi.yaml"

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
      echo "Defaults: --per-frame --monitor enabled; --sequence all; empty output prefix"
      echo ""
      echo "Sequences: room1_512_16  room2_512_16  room3_512_16  room4_512_16"
      echo "           room5_512_16  room6_512_16  corridor1_512_16  corridor2_512_16"
      echo "           corridor3_512_16  corridor4_512_16  corridor5_512_16"
      echo "           magistrale1_512_16  magistrale2_512_16  magistrale3_512_16"
      echo "           magistrale4_512_16  magistrale5_512_16  magistrale6_512_16"
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

  DATASET_PATH="${BASE_DATASET_PATH}/dataset-${SEQUENCE}"
  IMU_CSV="${DATASET_PATH}/mav0/imu0/data.csv"
  CAM_CSV="${DATASET_PATH}/mav0/cam0/data.csv"
  CAM_DIR="${DATASET_PATH}/mav0/cam0/data"
  GT_PATH="${DATASET_PATH}/mav0/mocap0/data.csv"

  if [[ -n "$OUTPUT_NAME" ]]; then
    FILE_STEM="${OUTPUT_NAME}_"
  else
    FILE_STEM=""
  fi

  [[ -x "$BINARY" ]] || { echo "Error: Binary not found or not executable: $BINARY"; echo "Build first with: cmake --build --preset release"; exit 1; }

for ((LOOP_I=1; LOOP_I<=LOOP_COUNT; LOOP_I++)); do
  TIMESTAMP=$(date +%Y%m%d_%H%M%S)
  if [[ "$LOOP_COUNT" -gt 1 ]]; then
    TIMESTAMP="${TIMESTAMP}_$(printf "%02d" "$LOOP_I")"
  fi
  RESULT_DIR="${RESULT_BASE}/tum_vi/${SEQUENCE}/${TIMESTAMP}"

  # ── Header ────────────────────────────────────────────────────────
  echo "========================================================================"
  echo "  LarVIO — TUM-VI [${SEQUENCE}] (OFFLINE)"
  echo "========================================================================"
  echo "  Binary   : $BINARY"
  echo "  Config   : $CONFIG_PATH"
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

  mkdir -p "$RESULT_DIR"

  # ── [1/3] Run VIO pipeline ──────────────────────────────────────
  echo "[1/3] Running VIO Pipeline..."
  START_TIME=$(date +%s.%N)

  # Binary CLI: larvio_eval imu.csv cam0.csv cam0_dir config.yaml [flags]
  ARGS=("$IMU_CSV" "$CAM_CSV" "$CAM_DIR" "$CONFIG_PATH")
  if [[ "$PER_FRAME" == true ]]; then
    ARGS+=(--per-frame-csv "$FRAMELOG" --per-frame-json "$SUMMARY")
  fi
  if [[ "$SHORT_RUN_FRAMES" -gt 0 ]]; then
    ARGS+=(--max-frames "$SHORT_RUN_FRAMES")
  fi
  ARGS+=(--dataset-name "$SEQUENCE")

  if [[ "$DO_MONITOR" == true ]]; then
    "$BINARY" "${ARGS[@]}" >/dev/null 2>/dev/null &
    VIO_PID=$!
    MONITOR_LOG="${RESULT_DIR}/${FILE_STEM}resource.csv"
    (
      trap 'exit 0' INT TERM
      echo "elapsed_sec,cpu_pct,mem_pct,rss_mb,threads"
      while kill -0 "$VIO_PID" 2>/dev/null; do
        TS=$(date +%s.%N)
        if read cpu mem rss threads <<< "$(ps -p "$VIO_PID" -o %cpu=,%mem=,rss=,thcount= 2>/dev/null)" && [[ -n "${cpu:-}" ]]; then
          elapsed=$(echo "$TS - $START_TIME" | bc -l | sed 's/^\./0./')
          rss_mb=$(echo "scale=3; ${rss:-0} / 1024" | bc -l)
          echo "${elapsed},${cpu:-0},${mem:-0},${rss_mb},${threads:-0}"
        fi
        sleep 0.5
      done
    ) > "$MONITOR_LOG" 2>/dev/null &
    MONITOR_PID=$!
    if wait "$VIO_PID"; then VIO_EXIT=0; else VIO_EXIT=$?; fi
  else
    MONITOR_PID=""; MONITOR_LOG=""
    if "$BINARY" "${ARGS[@]}" >/dev/null 2>/dev/null; then VIO_EXIT=0; else VIO_EXIT=$?; fi
  fi
  END_TIME=$(date +%s.%N)
  PROC_TIME=$(echo "$END_TIME - $START_TIME" | bc -l | sed 's/^\./0./')

  # ── Stop process monitor ──
  if [[ -n "$MONITOR_PID" ]]; then
    kill -TERM "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
    if [[ -f "$MONITOR_LOG" ]]; then
      RESOURCE_LINES=$("$PYTHON" - "$MONITOR_LOG" <<'PY'
import sys
try:
    print(max(0, sum(1 for _ in open(sys.argv[1])) - 1))
except FileNotFoundError:
    print(0)
PY
)
      if [[ "$RESOURCE_LINES" -eq 0 ]]; then
        "$PYTHON" - "$MONITOR_LOG" "$PROC_TIME" <<'PY'
import csv, sys
with open(sys.argv[1], "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["elapsed_sec", "cpu_pct", "mem_pct", "rss_mb", "threads"])
    w.writerow([sys.argv[2], 0, 0, 0, 0])
PY
      fi
    fi
  fi

  [[ "$VIO_EXIT" -eq 0 ]] || { echo "Error: LarVIO exited with status $VIO_EXIT"; exit "$VIO_EXIT"; }
  [[ -f "$FRAMELOG" ]] || { echo "Error: LarVIO framelog missing: $FRAMELOG"; exit 1; }

  # Process outputs
  "$PYTHON" - "$FRAMELOG" "$TIMING" "$SUMMARY" "$PROC_TIME" <<'PY'
import csv, json, sys
framelog, timing, summary_path, wall_time = sys.argv[1:5]
with open(framelog, newline="") as f:
    rows = list(csv.DictReader(f))

with open(timing, "w", newline="") as f:
    cols = ["frame_idx", "timestamp_s", "frame_interval_us", "pipeline_latency_us"]
    w = csv.DictWriter(f, fieldnames=cols)
    w.writeheader()
    for row in rows:
        w.writerow({c: row[c] for c in cols})

summary = {}
try:
    with open(summary_path) as f:
        summary = json.load(f)
except FileNotFoundError:
    pass
summary["num_frames_processed"] = len(rows)
summary["exec_time_s"] = float(wall_time)
summary["fps_avg"] = len(rows) / float(wall_time) if float(wall_time) > 0 else None
summary["tsr_pct"] = 100.0 * sum(1 for r in rows if str(r.get("tracking_state", "1")) == "1") / len(rows) if rows else 0.0
summary.setdefault("ate_rmse_m", None)
summary.setdefault("rpe_trans_rmse_m", None)
summary.setdefault("rpe_rot_rmse_deg", None)
with open(summary_path, "w") as f:
    json.dump(summary, f, indent=2)
PY

  if [[ -f "$FRAMELOG" ]]; then
    # Extract trajectory from framelog (timestamp_s,tx,ty,tz,qx,qy,qz,qw — already TUM order)
    "$PYTHON" - "$FRAMELOG" "${RESULT_DIR}/${FILE_STEM}trajectory.tum" <<'PY'
import sys, csv
src, dst = sys.argv[1:3]
with open(src, newline="") as f_in, open(dst, "w") as f_out:
    for r in csv.DictReader(f_in):
        if r.get("tracking_state") == "1":
            f_out.write(f"{r['timestamp_s']} {r['tx']} {r['ty']} {r['tz']} "
                        f"{r['qx']} {r['qy']} {r['qz']} {r['qw']}\n")
PY
  fi

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
        "pipeline": "LarVIO",
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
done
done
