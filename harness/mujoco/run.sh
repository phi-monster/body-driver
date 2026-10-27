#!/bin/bash
# run.sh NAME KIND SEED [robot.py options...]: one boot of the driver on one MuJoCo robot (robot.py), in $RUNS/NAME.
# BIN: the driver's build (its body_driver); PORT: the driver's port; MINUTES: the cap (a run past it is a failure, and
# its meta.txt says "stop limit"); STOP: an extended regular expression of the driver's log at which the run is stopped
# (default: the body as measured); GPU: the card the robot draws on; BD_WORKERS as the driver takes it.
# BEATS: the robot's own time a run may take, in beats (a real robot's beat is a tenth of a second: 1200 beats are two
# minutes of it, whatever the hall's pace); the robot stops there, and the run's meta.txt says "stop robot_exit".
# RATE: beats a second the robot keeps to (10: a real robot's, a tenth of a second a beat); unset, as fast as it can.
# At the robot's own pace a beat the driver spends computing apart (Driver.Apart: answered with Hold, the robot moving
# on) costs as many beats as that time takes at it: the hall ran 1.5 to 2.3 times a real robot's pace, so such
# computing cost it that many times the beats, and more or fewer by how loaded the machine was (rounds 4e to 4g2 of
# one driver: 34, 32, 20 and 26 of 60, 10-23).
# TRUTH=1: the robot writes the world's truth beside every observation (truth.jsonl; the bodies' geometry once, in
# TRUTH_GEOMETRY, shared by every run). RECORD=1: the driver records the run, compressed as it is written (run.rec.zst:
# never uncompressed on disk, a cold boot's recording is gigabytes). SCORE=1 (with both): after the run the recording
# is replayed for the driver's estimates (estimates.jsonl) and scored against the truth (score.txt: the arms' tools,
# the eyes, the hands' tips, driver/tools/score), both read straight from the compressed recording, which then goes
# unless KEEP_RECORDING=1.
# Leaves driver.log, robot.log, body.json (what the driver measured), truth.json, scramble.json and meta.txt.
set -u
NAME=$1; KIND=$2; SEED=$3; shift 3
HERE="$(cd "$(dirname "$0")" && pwd)"
RUNS=${RUNS:-/root/runs}; R=$RUNS/$NAME
[ -e "$R" ] && { echo "$R already exists"; exit 5; }
mkdir -p "$R"
PORT=${PORT:-$((31000 + SEED % 2000))}; LIM=${MINUTES:-5}; STOP=${STOP:-"the body as measured:"}
{ echo "kind $KIND"; echo "seed $SEED"; echo "port $PORT"; echo "driver $(md5sum "$BIN/body_driver" | cut -d' ' -f1)";
  echo "options $* ${RATE:+rate $RATE}"; echo "start $(date +%FT%T)"; } > "$R/meta.txt"
# DUMP_CELLS=1: the driver writes every eye's cells as the lock-in saw them at each recognition (Lockin.Dump_Cells).
[ "${DUMP_CELLS:-0}" = 1 ] && export BD_DUMP_CELLS="$R"
# DUMP_FITS=1: the driver writes every arm fit's inputs (BD_DUMP_FITS), for driver-robot-fit_replay to make that fit again
# alone in seconds: a step 2 fit studied without the run's recording.
[ "${DUMP_FITS:-0}" = 1 ] && export BD_DUMP_FITS="$R"
TRUTH_ARGS=(); RECORD_ARGS=()
if [ "${TRUTH:-0}" = 1 ]; then
  TRUTH_ARGS=(--truth "$R/truth.jsonl"); export BD_TRUTH_GEOMETRY=${TRUTH_GEOMETRY:-/root/mj/truth_geometry}
fi
if [ "${RECORD:-0}" = 1 ]; then
  mkfifo "$R/rec.fifo"; zstd -q -T2 -o "$R/run.rec.zst" < "$R/rec.fifo" & ZST=$!
  RECORD_ARGS=(--record "$R/rec.fifo")
fi
# Unpaced (RATE unset or 0: the robot waits for every answer), the driver makes its estimates within the beat that
# asks for them (--in-beat): no beat of the robot held still while it computes, and its time is its beats and its
# estimates' seconds, both in its log.
IN_BEAT_ARGS=(); [ "${RATE:-0}" = 0 ] && IN_BEAT_ARGS=(--in-beat)
"$BIN/body_driver" --listen "$PORT" --body "$R/body.json" "${RECORD_ARGS[@]}" "${IN_BEAT_ARGS[@]}" > "$R/driver.log" 2>&1 &
DRV=$!
for _ in $(seq 1 120); do grep -qE "listening on port|cannot listen" "$R/driver.log" 2>/dev/null && break; sleep 0.5; done
if grep -q "cannot listen" "$R/driver.log"; then
  { echo "end $(date +%FT%T)"; echo "stop port_taken"; } >> "$R/meta.txt"; kill "$DRV" 2>/dev/null; exit 3
fi
MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=${GPU:-0} /root/mj/venv/bin/python "$HERE/robot.py" --connect "127.0.0.1:$PORT" \
  --kind "$KIND" --seed "$SEED" --out "$R" --stop "$R/stop" "${TRUTH_ARGS[@]}" ${BEATS:+--beats "$BEATS"} ${RATE:+--rate "$RATE"} "$@" > "$R/robot.log" 2>&1 &
ROB=$!
end=$(( $(date +%s) + LIM * 60 )); why=limit
while [ "$(date +%s)" -lt "$end" ]; do
  if grep -qE "$STOP" "$R/driver.log"; then why=reached; break; fi
  if grep -qE "could not be measured|the boot failed|the driver failed|the estimates failed" "$R/driver.log"; then why=boot_failed; break; fi
  kill -0 "$ROB" 2>/dev/null || { why=robot_exit; break; }
  kill -0 "$DRV" 2>/dev/null || { why=driver_exit; break; }
  sleep 2
done
touch "$R/stop"
for _ in $(seq 1 20); do kill -0 "$ROB" 2>/dev/null || break; sleep 0.5; done
kill "$ROB" 2>/dev/null; kill "$DRV" 2>/dev/null
# A driver that never opened its recording leaves zstd waiting on the pipe: opening it read-write (never blocks on a
# FIFO) and closing it gives zstd its end, whether or not the driver wrote.
[ -p "$R/rec.fifo" ] && { exec 3<>"$R/rec.fifo"; exec 3>&-; }
wait 2>/dev/null
{ echo "end $(date +%FT%T)"; echo "stop $why"; } >> "$R/meta.txt"
rm -f "$R/rec.fifo"
if [ "${SCORE:-0}" = 1 ] && [ -s "$R/run.rec.zst" ] && [ -s "$R/truth.jsonl" ]; then
  zstd -dc "$R/run.rec.zst" | "$BIN/replay" /dev/stdin --estimates "$R/estimates.jsonl" > "$R/replay.log" 2>&1
  zstd -dc "$R/run.rec.zst" | "$BIN/score" --scramble "$R/scramble.json" "$R/estimates.jsonl" /dev/stdin "$R/truth.jsonl" \
    > "$R/score.txt" 2>&1
  echo "scored $(date +%FT%T)" >> "$R/meta.txt"
  [ "${KEEP_RECORDING:-0}" = 1 ] || rm -f "$R/run.rec.zst"
fi
