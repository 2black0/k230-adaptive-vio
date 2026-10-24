#!/usr/bin/env bash
# k230-levio — benchmark runner for host and K230 board.
#
# One command per campaign: converts nothing, runs the estimator, pulls the
# structured artifacts, evaluates against ground truth and plots the result.
# Terminal output is one line per sequence; the estimator's own log goes to
# run.log inside each run directory.
#
# Usage:
#   ./scripts/run_bench.sh --board -s MH01 --stop 1578
#   ./scripts/run_bench.sh --host                       # all 11 EuRoC sequences
#   ./scripts/run_bench.sh --board -s room1             # TUM-VI, detected from the name
#   ./scripts/run_bench.sh --host -s room1 room3
#
# The dataset is inferred from the sequence names (room* = TUM-VI, MH*/V1*/V2* =
# EuRoC), which also picks the data root and the binary built for that profile.
# Use --dataset to override.
#
# Options:
#   --host | --board     where to run (default: host)
#   --dataset euroc|tumvi  override the inferred dataset
#   --resolution baseline|half|quarter (default: baseline for EuRoC,
#                  half/256x256 for TUM-VI)
#   -s SEQ [SEQ ...]     sequences to run (default: all of the dataset)
#   --paced              Mode B, real-time paced; default is Mode A (fast)
#   --start N            first frame index
#   --stop N             one past the last frame index
#   --seed N             fault-injection seed (paced mode)
#   --keep-data          leave the dataset on the board after the run
#   --force              rerun sequences that already have complete artifacts
#   -h | --help
#
# Environment:
#   LEVIO_DATA_ROOT   EuRoC sequences   (default: $HOME/dataset/LEVIO-PGM)
#   LEVIO_TUMVI_ROOT  TUM-VI sequences  (default: $HOME/dataset/LEVIO-PGM-TUMVI)
#   LEVIO_EUROC_HALF_ROOT, LEVIO_EUROC_QUARTER_ROOT
#   LEVIO_TUMVI_QUARTER_ROOT  optional converted-root overrides
#   LEVIO_RESULTS_DIR  optional output directory override
#   LEVIO_BOARD_BIN    optional prebuilt board-binary path
#   K230_SDK          used only to rebuild the board binary if missing
#
# Each run directory ends up with:
#   trajectory.tum  framelog.csv  timing.csv  resource.csv  summary.csv
#   power.csv  run.log  EXITCODE  run.json  trajectory.png  evo/
#
# Board runs write logs to tmpfs (/dev/shm) and are pulled afterwards:
# continuous writes to the SD card measure at ~85 KB/s on this board and would
# land inside the timing being measured.
#
# SPDX-License-Identifier: MIT

set -uo pipefail

export LC_ALL=C
export PATH="$HOME/.local/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$ROOT"

EUROC_ROOT="${LEVIO_DATA_ROOT:-$HOME/dataset/LEVIO-PGM}"
TUMVI_ROOT="${LEVIO_TUMVI_ROOT:-$HOME/dataset/LEVIO-PGM-TUMVI}"
EUROC_HALF_ROOT="${LEVIO_EUROC_HALF_ROOT:-${LEVIO_DATA_ROOT:-$HOME/dataset/LEVIO-PGM-EUROC-HALF}}"
EUROC_QUARTER_ROOT="${LEVIO_EUROC_QUARTER_ROOT:-${LEVIO_DATA_ROOT:-$HOME/dataset/LEVIO-PGM-EUROC-QUARTER}}"
TUMVI_QUARTER_ROOT="${LEVIO_TUMVI_QUARTER_ROOT:-${LEVIO_TUMVI_ROOT:-$HOME/dataset/LEVIO-PGM-TUMVI-QUARTER}}"

EUROC_SEQS=(MH01 MH02 MH03 MH04 MH05 V101 V102 V103 V201 V202 V203)
TUMVI_SEQS=(room1 room2 room3 room4 room5 room6)

TARGET="host"
DATASET="tumvi"
RESOLUTION=""
SEQS=()
MODE="fast"
START=0
STOP=0
SEED="0x12345678"
FORCE=0
KEEP_DATA=0

BOARD_APP="${LEVIO_BOARD_APP:-/root/app/k230-levio}"
BOARD_DATA=""
BOARD_RUN_ROOT="/dev/shm/levio_runs"
BOARD_PIDFILE="/tmp/levio_bench.pid"
ADB_TIMEOUT="${LEVIO_ADB_TIMEOUT:-1800}"
ADB_DEVICE_ARGS=()
if [ -n "${K230_ADB_SERIAL:-}" ]; then
    ADB_DEVICE_ARGS=(-s "$K230_ADB_SERIAL")
fi

# EuRoC and TUM-VI sequence names are disjoint, so the dataset can be inferred
# from them alone; --dataset overrides the inference.
seq_dataset() {
    case "$1" in
        room[1-6])            echo "tumvi" ;;
        MH0[1-5]|V[12]0[1-3]) echo "euroc" ;;
        *)                    echo "" ;;
    esac
}

usage() {
    sed -n '2,/^# SPDX-License-Identifier/p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --host)         TARGET="host" ;;
        --board)        TARGET="board" ;;
        --dataset)      shift; DATASET="${1:-}" ;;
        --resolution)   shift; RESOLUTION="${1:-}" ;;
        --paced)        MODE="paced" ;;
        --start)         shift; START="${1:-0}" ;;
        --stop)          shift; STOP="${1:-0}" ;;
        --seed)          shift; SEED="${1:-0x12345678}" ;;
        --keep-data)     KEEP_DATA=1 ;;
        --force|-f)      FORCE=1 ;;
        -s)
            shift
            while [ $# -gt 0 ] && [ "${1:0:1}" != "-" ]; do
                SEQS+=("$1"); shift
            done
            continue ;;
        -h|--help)      usage 0 ;;
        *)              echo "unknown option '$1'" >&2; usage 1 ;;
    esac
    shift
done


# ------------------------------------------------------------------ #
# Helpers                                                             #
# ------------------------------------------------------------------ #

log() { printf '%s\n' "$*"; }

die() { printf 'error: %s\n' "$*" >&2; exit 1; }

# adb wrapper with a per-call timeout: the USB link drops occasionally and a
# hung adb would stall the whole campaign.
badb() {
    local t="$ADB_TIMEOUT"
    if [ "${1:-}" = "shell" ]; then
        shift
        timeout "$t" adb "${ADB_DEVICE_ARGS[@]}" shell "$@"
    else
        timeout "$t" adb "${ADB_DEVICE_ARGS[@]}" "$@"
    fi
}

# This adb build does not propagate the remote exit status -- `adb shell false`
# returns 0 -- so every remote predicate has to report its answer on stdout
# instead of through its exit code.
board_has() {       # board_has <path>       file or directory exists
    [ "$(badb shell "[ -e $1 ] && echo yes" 2>/dev/null | tr -d '\r\n')" = "yes" ]
}

board_is_exec() {   # board_is_exec <path>   exists and is executable
    [ "$(badb shell "[ -x $1 ] && echo yes" 2>/dev/null | tr -d '\r\n')" = "yes" ]
}

# ---- resolve the dataset, its data root and the matching binaries ----
if [ -z "$DATASET" ]; then
    if [ ${#SEQS[@]} -gt 0 ]; then
        DATASET="$(seq_dataset "${SEQS[0]}")"
        [ -n "$DATASET" ] || die "cannot tell which dataset '${SEQS[0]}' belongs to (use --dataset)"
        for s in "${SEQS[@]}"; do
            [ "$(seq_dataset "$s")" = "$DATASET" ] ||
                die "-s mixes datasets ('$s' is not $DATASET)"
        done
    else
        DATASET="euroc"
    fi
fi

case "$DATASET" in
    euroc)
        [ -n "$RESOLUTION" ] || RESOLUTION="baseline"
        case "$RESOLUTION" in
            baseline) DATA_ROOT="$EUROC_ROOT"; BUILD_SUFFIX=""; RESULT_SUFFIX="" ;;
            half) DATA_ROOT="$EUROC_HALF_ROOT"; BUILD_SUFFIX="-euroc_half"; RESULT_SUFFIX="/euroc-half" ;;
            quarter) DATA_ROOT="$EUROC_QUARTER_ROOT"; BUILD_SUFFIX="-euroc_quarter"; RESULT_SUFFIX="/euroc-quarter" ;;
            *) die "unknown EuRoC resolution '$RESOLUTION' (baseline | half | quarter)" ;;
        esac
        ALL_SEQS=("${EUROC_SEQS[@]}")
        ;;
    tumvi)
        [ -n "$RESOLUTION" ] || RESOLUTION="half"
        case "$RESOLUTION" in
            half) DATA_ROOT="$TUMVI_ROOT"; BUILD_SUFFIX="-tumvi"; RESULT_SUFFIX="/tumvi-half" ;;
            quarter) DATA_ROOT="$TUMVI_QUARTER_ROOT"; BUILD_SUFFIX="-tumvi_quarter"; RESULT_SUFFIX="/tumvi-quarter" ;;
            baseline) DATA_ROOT="$TUMVI_ROOT"; BUILD_SUFFIX="-tumvi"; RESULT_SUFFIX="/tumvi-half" ; RESOLUTION="half" ;;
            *) die "unknown TUM-VI resolution '$RESOLUTION' (half | quarter)" ;;
        esac
        ALL_SEQS=("${TUMVI_SEQS[@]}")
        ;;
    *)
        die "unknown dataset '$DATASET' (euroc | tumvi)"
        ;;
esac

BOARD_DATA="/dev/shm/LEVIO_PGM_${DATASET}_${RESOLUTION}"

HOST_BIN="${ROOT}/build${BUILD_SUFFIX}/levio_replay"
BOARD_BIN="${LEVIO_BOARD_BIN:-${ROOT}/build-k230${BUILD_SUFFIX}/levio_replay}"
# The board carries one binary per profile; a single name would have the two
# builds overwrite each other.
BOARD_BIN_NAME="${LEVIO_BOARD_BIN_NAME:-levio_replay${BUILD_SUFFIX}}"

if [ ${#SEQS[@]} -eq 0 ]; then
    SEQS=("${ALL_SEQS[@]}")
fi

RESULTS_DIR="${LEVIO_RESULTS_DIR:-${ROOT}/results/${TARGET}${RESULT_SUFFIX}}"

# A run is complete when the estimator wrote its trajectory, its summary and a
# success marker. Interrupted campaigns can simply be re-run with the same
# command; finished sequences are skipped.
is_complete() {
    local dir="$1"
    [ -s "${dir}/trajectory.tum" ] &&
    [ -s "${dir}/summary.csv" ] &&
    [ "$(cat "${dir}/EXITCODE" 2>/dev/null)" = "0" ]
}

summary_field() {   # summary_field <file> <metric>
    awk -F, -v k="$2" '$1==k {print $2}' "$1" 2>/dev/null | head -1
}

# A run that processed nothing still exits 0 -- the harness breaks out of the
# loop on a source error and returns cleanly. Catch it here rather than let a
# zero-frame run be recorded as a result.
assert_frames() {   # assert_frames <dir> <seq>
    local n
    n=$(summary_field "$1/summary.csv" frames)
    if [ -z "$n" ] || [ "$n" -eq 0 ]; then
        log "  $2: run processed 0 frames, see $1/run.log"
        return 1
    fi
    return 0
}

build_args() {      # echoes the estimator flags shared by both targets
    local args=""
    [ "$MODE" = "paced" ] && args="$args --mode paced"
    [ "$START" -gt 0 ]  && args="$args --start $START"
    [ "$STOP" -gt 0 ]   && args="$args --stop $STOP"
    args="$args --seed $SEED"
    printf '%s' "$args"
}

write_run_json() {  # write_run_json <dir> <seq> <binary> <binary_sha> <exit>
    local dir="$1" seq="$2" binary="$3" sha="$4" rc="$5"
    local gitrev="null"
    if git -C "$ROOT" rev-parse --short HEAD >/dev/null 2>&1; then
        gitrev="\"$(git -C "$ROOT" rev-parse --short HEAD)\""
    fi
    local verdict="ok"
    if [ "$rc" != "0" ]; then
        verdict="partial"
    elif grep -qiE 'non-finite|diverg' "${dir}/run.log" 2>/dev/null; then
        verdict="diverge"
    fi

    python3 - "$dir" "$seq" "$DATASET" "$RESOLUTION" "$TARGET" "$MODE" "$START" "$STOP" "$SEED" \
             "$binary" "$sha" "$rc" "$verdict" "$gitrev" <<'PY'
import json, sys, datetime
(d, seq, dataset, resolution, target, mode, start, stop, seed, binary, sha, rc, verdict, gitrev) = sys.argv[1:15]
doc = {
    "method": "levio",
    "dataset": dataset,
    "sequence": seq,
    "resolution": resolution,
    "target": target,
    "mode": mode,
    "start": int(start),
    "stop": int(stop),
    "seed": seed,
    "binary": binary,
    "binary_sha256": sha,
    "git_commit": json.loads(gitrev),
    "exit_code": int(rc),
    "verdict": verdict,
    "finished": datetime.datetime.now().isoformat(timespec="seconds"),
}
with open(f"{d}/run.json", "w") as f:
    json.dump(doc, f, indent=2)
PY
}

evaluate() {        # evaluate <dir> <gt> <label>
    local dir="$1" gt="$2" label="$3"
    python3 "${ROOT}/tools/plot_trajectory.py" \
        --est "${dir}/trajectory.tum" --gt "$gt" \
        --out-dir "$dir" --label "$label" 2>&1 | grep -E '^\[(plot|evo)\]' || true
}

report_line() {     # report_line <seq> <dir>
    local seq="$1" dir="$2"
    local fps lat ate frames
    fps=$(summary_field "${dir}/summary.csv" fps_mean)
    lat=$(summary_field "${dir}/summary.csv" latency_mean_us)
    frames=$(summary_field "${dir}/summary.csv" frames)
    ate=$(summary_field "${dir}/evo/ate.csv" ate_rmse)
    printf '  %-6s %7s frames  %7s fps  %9s ms  %s\n' \
        "$seq" "${frames:--}" "${fps:--}" \
        "$(awk -v v="${lat:--}" 'BEGIN{ if (v=="-") print "-"; else printf "%.1f", v/1000 }')" \
        "$(awk -v v="${ate:-}" 'BEGIN{ if (v=="") print ""; else printf "ATE %s m", v }')"
}

# ------------------------------------------------------------------ #
# Host                                                                #
# ------------------------------------------------------------------ #

host_run() {
    local seq="$1"
    local dir="${RESULTS_DIR}/${seq}"
    local data="${DATA_ROOT}/${seq}"

    mkdir -p "$dir"

    [ -d "$data" ] || { log "  $seq: no data at $data (run ./scripts/extract-euroc.sh)"; return 1; }
    [ -x "$HOST_BIN" ] || { log "  $seq: $HOST_BIN missing (run ./scripts/build_host.sh)"; return 1; }

    # shellcheck disable=SC2086
    "$HOST_BIN" --data "$data" --log-dir "$dir" $(build_args) \
        > "${dir}/run.log" 2>&1
    local rc=$?
    printf '%s\n' "$rc" > "${dir}/EXITCODE"

    local sha
    sha=$(sha256sum "$HOST_BIN" | awk '{print $1}')
    write_run_json "$dir" "$seq" "$HOST_BIN" "$sha" "$rc"

    assert_frames "$dir" "$seq" || return 1

    evaluate "$dir" "${data}/groundtruth.txt" "$seq"
    return $rc
}

# ------------------------------------------------------------------ #
# Board                                                              #
# ------------------------------------------------------------------ #

board_preflight() {
    [ -x "$BOARD_BIN" ] || die "$BOARD_BIN missing (run: K230_SDK=../k230_linux_sdk ./scripts/build_k230.sh)"
    [ "$(badb shell 'echo ok' 2>/dev/null | tr -d '\r\n')" = "ok" ] || die "no board on adb"
    file "$BOARD_BIN" | grep -q RISC-V || die "$BOARD_BIN is not a RISC-V binary"

    badb shell "mkdir -p $BOARD_APP" >/dev/null 2>&1
    if [ "$FORCE" -eq 1 ] || ! board_is_exec "$BOARD_APP/$BOARD_BIN_NAME"; then
        badb push "$BOARD_BIN" "$BOARD_APP/$BOARD_BIN_NAME" >/dev/null
        badb shell "chmod +x $BOARD_APP/$BOARD_BIN_NAME" >/dev/null 2>&1
        board_is_exec "$BOARD_APP/$BOARD_BIN_NAME" || die "binary did not land on the board"
    fi
}

board_push_data() {
    local seq="$1"
    if board_has "$BOARD_DATA/$seq/timestamps.txt"; then
        return 0
    fi

    log "  $seq: pushing dataset directly to board tmpfs ..."
    badb shell "mkdir -p $BOARD_DATA; rm -rf $BOARD_DATA/$seq" >/dev/null 2>&1
    # Direct push avoids both the tar+extract peak and the slow SD-card path.
    # ADB shell stdin streaming is unreliable on this board's older daemon.
    if ! badb push "$DATA_ROOT/$seq" "$BOARD_DATA/" >/dev/null; then
        return 1
    fi

    # Verify rather than trust: a silently short push would show up much later
    # as a run that processed 0 frames.
    board_has "$BOARD_DATA/$seq/images/frame000000.pgm" || {
        log "  $seq: dataset is incomplete on the board"
        return 1
    }
}

board_run() {
    local seq="$1"
    local dir="${RESULTS_DIR}/${seq}"
    local rundir="${BOARD_RUN_ROOT}/${seq}"

    mkdir -p "$dir"
    board_push_data "$seq" || { log "  $seq: dataset push failed"; return 1; }

    # nohup/setsid die with the adb session; start-stop-daemon -b daemonises.
    badb shell "rm -rf $rundir; mkdir -p $rundir; \
cat > $rundir/launch.sh <<'EOF'
#!/bin/sh
cd $BOARD_APP
./$BOARD_BIN_NAME --data $BOARD_DATA/$seq --log-dir $rundir $(build_args) > $rundir/run.log 2>&1
echo \$? > $rundir/EXITCODE
EOF
chmod +x $rundir/launch.sh" >/dev/null 2>&1

    badb shell "rm -f $BOARD_PIDFILE; start-stop-daemon -S -b -m -p $BOARD_PIDFILE \
        -x /bin/sh -- $rundir/launch.sh" >/dev/null 2>&1 || true

    # Poll for the exit marker. A dropped link is not fatal: the run is
    # detached, so the next poll simply reconnects.
    local waited=0 rc=""
    while [ "$waited" -lt 3600 ]; do
        rc=$(badb shell "cat $rundir/EXITCODE 2>/dev/null" 2>/dev/null | tr -d '\r\n')
        [ -n "$rc" ] && break
        sleep 5
        waited=$((waited + 5))
    done

    if [ -z "$rc" ]; then
        log "  $seq: timed out waiting for the board"
        return 1
    fi

    # adb nests the remote directory inside the destination when that
    # destination already exists, so pull into a scratch dir and flatten.
    local tmp="${dir}/.pull"
    rm -rf "$tmp"; mkdir -p "$tmp"
    badb pull "$rundir" "$tmp" >/dev/null 2>&1 || true
    if [ -d "${tmp}/${seq}" ]; then
        cp -af "${tmp}/${seq}/." "$dir/" 2>/dev/null || true
    else
        cp -af "${tmp}/." "$dir/" 2>/dev/null || true
    fi
    rm -rf "$tmp"
    printf '%s\n' "$rc" > "${dir}/EXITCODE"

    local sha
    sha=$(sha256sum "$BOARD_BIN" | awk '{print $1}')
    write_run_json "$dir" "$seq" "$BOARD_BIN" "$sha" "$rc"

    assert_frames "$dir" "$seq" || return 1

    evaluate "$dir" "${DATA_ROOT}/${seq}/groundtruth.txt" "$seq"

    # One TUM-VI sequence is ~200 MB against the board's 485 MB tmpfs, so the
    # copy is dropped once it has been used and re-pushed if that sequence is
    # run again. EuRoC sequences would fit anyway, but the rule is the same for
    # both so the runner stays predictable.
    if [ "$KEEP_DATA" -eq 0 ]; then
        badb shell "rm -rf $BOARD_DATA/$seq $rundir" >/dev/null 2>&1 || true
    fi

    return "$rc"
}

# ------------------------------------------------------------------ #
# Main                                                               #
# ------------------------------------------------------------------ #

[ -d "$DATA_ROOT" ] || die "no $DATASET dataset at $DATA_ROOT (run ./scripts/extract-euroc.sh or ./scripts/extract-tumvi.sh)"

log "k230-levio benchmark"
log "  target     : $TARGET"
log "  mode       : $MODE"
log "  sequences  : ${SEQS[*]}"
log "  dataset    : $DATASET"
log "  data root  : $DATA_ROOT"
log "  results    : $RESULTS_DIR"
log ""

[ "$TARGET" = "board" ] && board_preflight

ok=0
skipped=0
failed=0

for seq in "${SEQS[@]}"; do
    dir="${RESULTS_DIR}/${seq}"

    if [ "$FORCE" -eq 0 ] && is_complete "$dir"; then
        log "  $seq: already complete, skipping (--force to rerun)"
        report_line "$seq" "$dir"
        skipped=$((skipped + 1))
        continue
    fi

    if [ "$TARGET" = "board" ]; then
        board_run "$seq"
    else
        host_run "$seq"
    fi
    rc=$?

    if [ "$rc" -eq 0 ]; then
        ok=$((ok + 1))
    else
        failed=$((failed + 1))
    fi
    report_line "$seq" "$dir"
done

log ""
log "done: $ok ok, $skipped skipped, $failed failed   -> $RESULTS_DIR"
[ "$failed" -eq 0 ]
