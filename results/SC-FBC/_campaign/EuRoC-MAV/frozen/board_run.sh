#!/bin/sh
# Runs on the K230, not on the host. One estimator run, detached and self-reporting.
#
# The board's USB gadget drops. Over a three-hour campaign it will drop several
# times, and a run driven synchronously from the host dies with the connection
# even though the board itself is fine and still has the core free. So the host
# starts this script with nohup and then only watches: the estimator keeps
# running through a disconnect, and the host picks the result up whenever it
# gets back.
#
# EXITCODE is written last, and only once, so its presence means "this run is
# finished and complete". That is the whole synchronisation protocol -- the host
# polls for that one file and needs nothing else.
#
# Usage (from the host, via adb):
#   nohup sh board_run.sh <out-dir> <binary> [args...] &
#
# SPDX-License-Identifier: MIT

OUT="$1"
shift

mkdir -p "$OUT"
# A second host invocation after an adb disconnect must not start a second
# estimator over the first one. The host normally detects this before launch;
# this guard is the last line of defence on the board itself.
if [ -f "$OUT/RUNNING" ]; then
    OLD_PID=$(cat "$OUT/ESTIMATOR_PID" 2>/dev/null || true)
    if [ -n "$OLD_PID" ] && [ -d "/proc/$OLD_PID" ]; then
        echo "run already active (pid $OLD_PID)" >&2
        exit 73
    fi
    rm -f "$OUT/RUNNING" "$OUT/ESTIMATOR_PID"
fi
# The host removes the whole output directory only when it has positively
# identified a different or abandoned run. A stale EXITCODE is therefore not
# cleared here during a resume.
rm -f "$OUT/EXITCODE"

"$@" > "$OUT/run.log" 2>&1 &
PID=$!
# Publish the child PID and RUNNING marker atomically. Their presence lets a
# reconnecting host distinguish an active detached estimator from an abandoned
# directory without touching the estimator process.
printf '%s\n' "$PID" > "$OUT/ESTIMATOR_PID.tmp"
mv "$OUT/ESTIMATOR_PID.tmp" "$OUT/ESTIMATOR_PID"
printf '1\n' > "$OUT/RUNNING.tmp"
mv "$OUT/RUNNING.tmp" "$OUT/RUNNING"

# Sampled from /proc, because the host cannot see the target's resident set.
# Same columns as the host-side monitor so one parser reads both.
#
# cpu_pct and mem_pct are computed here rather than left blank: this board's ps
# does not report them per process, but /proc does. CPU% is the delta of
# utime+stime (fields 14 and 15 of /proc/<pid>/stat) over the delta of wall
# clock, which is the same quantity the host-side monitor gets from `ps %cpu`.
# mem_pct is VmRSS against MemTotal.
#
# /proc/<pid>/stat field 2 is the command name and may contain spaces and
# parentheses, so the numeric fields are read after the last ')'.
echo 'elapsed_sec,cpu_pct,mem_pct,rss_mb,threads' > "$OUT/resource.csv"
T0=$(cut -d' ' -f1 /proc/uptime)

HZ=$(getconf CLK_TCK 2>/dev/null || echo 100)
case "$HZ" in ''|*[!0-9]*) HZ=100 ;; esac
MEMTOTAL_KB=$(awk '/MemTotal/{print $2}' /proc/meminfo 2>/dev/null)
case "$MEMTOTAL_KB" in ''|*[!0-9]*) MEMTOTAL_KB=0 ;; esac

read_ticks() {   # read_ticks <pid> -> utime+stime in clock ticks
    sed 's/^.*) //' "/proc/$1/stat" 2>/dev/null | awk '{print $12 + $13}'
}

PREV_TICKS=$(read_ticks "$PID")
PREV_WALL=$(cut -d' ' -f1 /proc/uptime)
INIT_TICKS="$PREV_TICKS"
SAMPLES=0

while [ -d "/proc/$PID" ]; do
    sleep 0.5
    NOW=$(cut -d' ' -f1 /proc/uptime)
    CUR_TICKS=$(read_ticks "$PID")
    RSS=$(awk '/VmRSS/{print $2}' "/proc/$PID/status" 2>/dev/null)
    THR=$(awk '/Threads/{print $2}' "/proc/$PID/status" 2>/dev/null)

    if [ -n "$CUR_TICKS" ]; then
        awk -v now="$NOW" -v t0="$T0" -v ct="$CUR_TICKS" -v pt="${PREV_TICKS:-$CUR_TICKS}" \
            -v pw="$PREV_WALL" -v hz="$HZ" -v rss="${RSS:-0}" -v mt="$MEMTOTAL_KB" \
            -v thr="${THR:-0}" 'BEGIN {
                dt = now - pw;
                cpu = (dt > 0 && ct >= pt) ? 100 * (ct - pt) / hz / dt : 0;
                if (cpu > 100) cpu = 100;
                mem = (mt > 0) ? 100 * rss / mt : 0;
                printf "%.3f,%.2f,%.3f,%.3f,%s\n", now - t0, cpu, mem, rss / 1024, thr;
            }' >> "$OUT/resource.csv"
        SAMPLES=$((SAMPLES + 1))
    fi
    PREV_TICKS="$CUR_TICKS"
    PREV_WALL="$NOW"
done

# A run shorter than one 0.5 s sample would leave a header and nothing else.
# One final row averaged over the whole run is better than an empty file.
if [ "$SAMPLES" -eq 0 ]; then
    END_WALL=$(cut -d' ' -f1 /proc/uptime)
    END_TICKS=$(read_ticks "$PID")
    awk -v now="$END_WALL" -v t0="$T0" -v hz="$HZ" -v mt="$MEMTOTAL_KB" \
        -v ct="${END_TICKS:-0}" -v it="${INIT_TICKS:-0}" \
        'BEGIN {
            dt = now - t0;
            cpu = (dt > 0 && ct >= it) ? 100 * (ct - it) / hz / dt : 0;
            if (cpu > 100) cpu = 100;
            printf "%.3f,%.2f,0.000,0.000,0\n", (dt > 0 ? dt : 0), cpu;
        }' >> "$OUT/resource.csv"
fi

wait "$PID"
RC=$?

# Nothing downstream reads the per-landmark dumps, and at about 5 MB a run they
# would cost a gigabyte of card across a full campaign.
rm -f "$OUT/features.csv" "$OUT/points.csv"

# Thermal state at the end of the run. Sustained load on this board warms the
# die well past its idle ~42 C, and a thermally throttled run is a different
# measurement from a cool one.
#
# NOTE: /sys/class/thermal/thermal_zone0/temp is the raw TS register code,
# not millidegree C. Convert with Kendryte's polynomial (see
# scripts/k230-temp.sh for the derivation and worked examples) so that the
# file named die_milliC_end actually holds millidegree C.
TSCODE="$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | tr -cd '0-9')"
if [ -n "$TSCODE" ]; then
  awk -v r="$TSCODE" 'BEGIN{ c = r % 4096; t = 1e-10*c*c*c*c*1.01472 - 1e-6*c*c*c*1.10063 + 4.36150e-3*c*c - 7.10128*c + 3565.87; printf "%.0f", t*1000 }' \
    > "$OUT/die_milliC_end" 2>/dev/null || true
fi
cut -d' ' -f1 /proc/uptime > "$OUT/uptime_end_s" 2>/dev/null || true

sync
rm -f "$OUT/RUNNING" "$OUT/ESTIMATOR_PID"
# Publish completion atomically. A reconnect must see either no marker or the
# complete exit code, never a half-written marker from a cable/power failure.
printf '%s\n' "$RC" > "$OUT/EXITCODE.tmp"
mv "$OUT/EXITCODE.tmp" "$OUT/EXITCODE"
# The rename and its two payload bytes do not necessarily reach the card
# together: one campaign run came back present but empty, which the host reads
# as "finished" and then waits forever for a code that never arrives. Verify
# the content landed and, if the card ate it, rewrite it in place.
if [ ! -s "$OUT/EXITCODE" ]; then
    printf '%s\n' "$RC" > "$OUT/EXITCODE"
fi
sync
