#!/bin/sh
# One competing runnable process, normal -> contention -> normal. No fixed CPU% claim.
OUT="$1"
LOAD_PID=""
cleanup() {
    if [ -n "$LOAD_PID" ]; then kill "$LOAD_PID" 2>/dev/null; wait "$LOAD_PID" 2>/dev/null; fi
}
trap cleanup EXIT HUP INT TERM
while [ ! -s "$OUT/replay_epoch.txt" ]; do
    [ -s "$OUT/EXITCODE" ] && exit 1
    sleep 0.1
done
sleep 30
echo 'event,monotonic_s' > "$OUT/stress.csv"
printf 'on,%s\n' "$(cut -d' ' -f1 /proc/uptime)" >> "$OUT/stress.csv"
yes > /dev/null &
LOAD_PID=$!
echo "$LOAD_PID" > "$OUT/stress_pid"
sleep 60
cleanup
LOAD_PID=""
printf 'off,%s\n' "$(cut -d' ' -f1 /proc/uptime)" >> "$OUT/stress.csv"
echo done > "$OUT/stress_done"
