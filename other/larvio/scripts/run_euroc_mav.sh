#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
export LC_NUMERIC=C
export PATH="$HOME/.local/bin:$PATH"

# =============================================================================
# LarVIO — Batch processing test
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
BINARY="${SCRIPT_DIR}/build/larvio_eval"
PYTHON="python3"

BASE_DATASET_PATH="$HOME/dataset/euroc_mav/raw"
CONFIG_PATH="${SCRIPT_DIR}/config/euroc.yaml"

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

  # Derive paths from sequence
  DATASET_PATH="${BASE_DATASET_PATH}/${SEQUENCE}"
  IMU_CSV="${DATASET_PATH}/mav0/imu0/data.csv"
  CAM_CSV="${DATASET_PATH}/mav0/cam0/data.csv"
  CAM_DIR="${DATASET_PATH}/mav0/cam0/data"
  GT_PATH="${DATASET_PATH}/mav0/state_groundtruth_estimate0/data.csv"

  # File stem: empty by default, "<name>_" if --output-name given
  if [[ -n "$OUTPUT_NAME" ]]; then
    FILE_STEM="${OUTPUT_NAME}_"
  else
    FILE_STEM=""
  fi

  # Ensure binary exists
  if [[ ! -x "$BINARY" ]]; then
    echo "Error: Binary not found or not executable: $BINARY"
    echo "Build first with: bash scripts/build.sh"
    exit 1
  fi

  export LD_LIBRARY_PATH="${SCRIPT_DIR}/build:${LD_LIBRARY_PATH:-}"

for ((LOOP_I=1; LOOP_I<=LOOP_COUNT; LOOP_I++)); do
  TIMESTAMP=$(date +%Y%m%d_%H%M%S)
  if [[ "$LOOP_COUNT" -gt 1 ]]; then
    TIMESTAMP="${TIMESTAMP}_$(printf "%02d" "$LOOP_I")"
  fi
  RESULT_DIR="${RESULT_BASE}/euroc_mav/${SEQUENCE}/${TIMESTAMP}"

  # ── Header ────────────────────────────────────────────────────────
  echo "========================================================================"
  echo "  LarVIO — EuRoC [${SEQUENCE}] (OFFLINE)"
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
    echo "[DRY-RUN] Would execute benchmark loop $LOOP_COUNT time(s)."
    echo "[DRY-RUN] No commands were executed."
    exit 0
  fi

  FRAMELOG="${RESULT_DIR}/${FILE_STEM}framelog.csv"
  SUMMARY="${RESULT_DIR}/${FILE_STEM}summary.json"
  TIMING="${RESULT_DIR}/${FILE_STEM}timing.csv"

  mkdir -p "$RESULT_DIR"

  # Binary CLI: larvio_eval imu.csv cam0.csv cam0_dir config.yaml [flags]
  ARGS=("$IMU_CSV" "$CAM_CSV" "$CAM_DIR" "$CONFIG_PATH")
  if [[ "$PER_FRAME" == true ]]; then
    ARGS+=(--per-frame-csv "$FRAMELOG" --per-frame-json "$SUMMARY")
  fi
  if [[ "$SHORT_RUN_FRAMES" -gt 0 ]]; then
    ARGS+=(--max-frames "$SHORT_RUN_FRAMES")
  fi
  ARGS+=(--dataset-name "$SEQUENCE")

  # ── [1/3] Run VIO pipeline ──────────────────────────────────────
  echo "[1/3] Running VIO Pipeline..."
  START_TIME=$(date +%s.%N)

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
    MONITOR_PID=""
    MONITOR_LOG=""
    if "$BINARY" "${ARGS[@]}" >/dev/null 2>/dev/null; then VIO_EXIT=0; else VIO_EXIT=$?; fi
  fi
  END_TIME=$(date +%s.%N)

  PROC_TIME=$(echo "$END_TIME - $START_TIME" | bc -l | sed 's/^\./0./')

  # ── Stop process monitor immediately ──
  if [[ -n "$MONITOR_PID" ]]; then
    kill -TERM "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
    if [[ -f "$MONITOR_LOG" ]]; then
      RESOURCE_LINES=$(python3 - "$MONITOR_LOG" <<'PY'
import sys
try:
    print(max(0, sum(1 for _ in open(sys.argv[1])) - 1))
except FileNotFoundError:
    print(0)
PY
)
      if [[ "$RESOURCE_LINES" -eq 0 ]]; then
        python3 - "$MONITOR_LOG" "$PROC_TIME" <<'PY'
import csv, sys
path, proc_time = sys.argv[1], sys.argv[2]
with open(path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["elapsed_sec", "cpu_pct", "mem_pct", "rss_mb", "threads"])
    writer.writerow([proc_time, 0, 0, 0, 0])
PY
      fi
    fi
  fi

  [[ "$VIO_EXIT" -eq 0 ]] || { echo "Error: LarVIO exited with status $VIO_EXIT"; exit "$VIO_EXIT"; }
  [[ -f "$FRAMELOG" ]] || { echo "Error: LarVIO framelog missing: $FRAMELOG"; exit 1; }

  # Extract timing.csv from framelog.csv
  python3 - "$FRAMELOG" "$TIMING" <<'PY'
import csv, sys
framelog, timing = sys.argv[1:3]
with open(framelog, newline="") as f_in, open(timing, "w", newline="") as f_out:
    reader = csv.DictReader(f_in)
    cols = ["frame_idx", "timestamp_s" if "timestamp_s" in (reader.fieldnames or []) else "timestamp_ns", "frame_interval_us", "pipeline_latency_us"]
    writer = csv.DictWriter(f_out, fieldnames=cols)
    writer.writeheader()
    for row in reader:
        writer.writerow({c: row.get(c, "") for c in cols})
PY

  # Extract trajectory.tum from framelog.csv
  python3 - "$FRAMELOG" "${RESULT_DIR}/${FILE_STEM}trajectory.tum" <<'PY'
import sys, csv
src, dst = sys.argv[1:3]
with open(src, newline="") as f_in, open(dst, "w") as f_out:
    f_out.write("# timestamp tx ty tz qx qy qz qw\n")
    for r in csv.DictReader(f_in):
        if str(r.get("tracking_state")) == "1":
            ts = r.get("timestamp_s")
            if not ts and "timestamp_ns" in r:
                ts = f"{float(r['timestamp_ns']) * 1e-9:.9f}"
            f_out.write(f"{ts} {r['tx']} {r['ty']} {r['tz']} {r['qx']} {r['qy']} {r['qz']} {r['qw']}\n")
PY

  # ── Metrics: frames / FPS / TSR ──────────────────────────────────
  FRAMES="?"
  TSR="?"
  FPS_AVG="?"
  if [[ -f "$SUMMARY" ]]; then
    read FRAMES TSR FPS_AVG <<< "$(python3 -c "
import json
d=json.load(open('$SUMMARY'))
print(d.get('num_frames_processed', d.get('tracking', {}).get('processed_frames', '?')),
      d.get('tsr_pct', d.get('tracking', {}).get('tracking_success_rate_pct', '?')),
      d.get('fps_avg', d.get('performance', {}).get('fps', {}).get('mean', '?')))
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
  EST_TUM="${RESULT_DIR}/${FILE_STEM}trajectory.tum"
  GT_TUM="${RESULT_DIR}/${FILE_STEM}groundtruth.tum"
  if [[ -f "$EST_TUM" && -f "$GT_PATH" ]]; then
    python3 - "$GT_PATH" "$GT_TUM" <<'PY'
import sys, pandas as pd
src, dst = sys.argv[1:3]
df = pd.read_csv(src)
df.columns = df.columns.str.strip()
out = df[["#timestamp", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]",
          "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []"]].copy()
out["#timestamp"] = out["#timestamp"] * 1e-9  # ns -> s (TUM format)
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
      python3 - "$SUMMARY" "$ATE" "$RPE_T" "$RPE_R" "$TIMESTAMP" "$SEQUENCE" "$FRAMELOG" "$PROC_TIME" <<'PY'
import json, sys, csv, statistics
path, ate, rpe_t, rpe_r, timestamp, sequence, framelog, proc_time = sys.argv[1:9]
def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None

try:
    d = json.load(open(path))
except Exception:
    d = {}

latencies = []
intervals = []
fps_list = []
tracked_count = 0
total_count = 0

if framelog:
    try:
        with open(framelog) as f:
            for r in csv.DictReader(f):
                total_count += 1
                if str(r.get("tracking_state", "1")) == "1":
                    tracked_count += 1
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
    except Exception:
        pass

if latencies:
    lat_sorted = sorted(latencies)
    p95 = lat_sorted[min(len(lat_sorted) - 1, int(len(lat_sorted) * 0.95) - 1)]
    lat_mean = statistics.mean(latencies)
    lat_median = statistics.median(latencies)
    lat_std = statistics.pstdev(latencies) if len(latencies) > 1 else 0.0
else:
    lat_mean = lat_median = p95 = lat_std = 0.0

exec_time = float(proc_time) if proc_time and float(proc_time) > 0 else d.get("exec_time_s", 0.0)
fps_mean = statistics.mean(fps_list) if fps_list else (total_count / exec_time if exec_time > 0 else 0.0)
fps_median = statistics.median(fps_list) if fps_list else fps_mean
int_mean = statistics.mean(intervals) if intervals else 0.0
int_min = min(intervals) if intervals else 0.0
int_max = max(intervals) if intervals else 0.0
tsr = (100.0 * tracked_count / total_count) if total_count > 0 else d.get("tsr_pct", 100.0)

out = {
    "metadata": {
        "pipeline": "LarVIO",
        "dataset": "EuRoC",
        "sequence": sequence,
        "mode": "offline",
        "timestamp": f"{timestamp[:4]}-{timestamp[4:6]}-{timestamp[6:8]}T{timestamp[9:11]}:{timestamp[11:13]}:{timestamp[13:15]}"
    },
    "tracking": {
        "total_frames": total_count if total_count > 0 else d.get("num_frames_total", 0),
        "processed_frames": total_count if total_count > 0 else d.get("num_frames_processed", 0),
        "tracking_success_rate_pct": round(float(tsr), 1),
        "execution_time_s": round(exec_time, 6)
    },
    "accuracy": {
        "ate_rmse_m": num(ate),
        "rpe_trans_rmse_m": num(rpe_t),
        "rpe_rot_rmse_deg": num(rpe_r)
    },
    "performance": {
        "fps": {
            "mean": round(fps_mean, 2),
            "median": round(fps_median, 2)
        },
        "latency_ms": {
            "mean": round(lat_mean, 2),
            "median": round(lat_median, 2),
            "p95": round(p95, 2),
            "std": round(lat_std, 2)
        },
        "frame_interval_us": {
            "mean": round(int_mean, 2),
            "min": round(int_min, 2),
            "max": round(int_max, 2)
        }
    }
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
