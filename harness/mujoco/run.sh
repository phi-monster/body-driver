#!/bin/bash
# run.sh NAME KIND SEED [robot.py options...]: one boot of the driver on one MuJoCo robot (robot.py), in $RUNS/NAME.
# BIN: the driver's build (its body_driver); PORT: the driver's port; MINUTES: the cap (a run past it is a failure, and
# its meta.txt says "stop limit"); STOP: an extended regular expression of the driver's log at which the run is stopped
# (default: the body as measured); GPU: the card the robot draws on; BD_WORKERS as the driver takes it.
# Leaves driver.log, robot.log, body.json (what the driver measured), truth.json, scramble.json and meta.txt.
set -u
NAME=$1; KIND=$2; SEED=$3; shift 3
HERE="$(cd "$(dirname "$0")" && pwd)"
RUNS=${RUNS:-/root/runs}; R=$RUNS/$NAME
[ -e "$R" ] && { echo "$R already exists"; exit 5; }
mkdir -p "$R"
PORT=${PORT:-$((31000 + SEED % 2000))}; LIM=${MINUTES:-5}; STOP=${STOP:-"the body as measured:"}
{ echo "kind $KIND"; echo "seed $SEED"; echo "port $PORT"; echo "driver $(md5sum "$BIN/body_driver" | cut -d' ' -f1)";
  echo "options $*"; echo "start $(date +%FT%T)"; } > "$R/meta.txt"
"$BIN/body_driver" --listen "$PORT" --body "$R/body.json" > "$R/driver.log" 2>&1 &
DRV=$!
for _ in $(seq 1 120); do grep -qE "listening on port|cannot listen" "$R/driver.log" 2>/dev/null && break; sleep 0.5; done
if grep -q "cannot listen" "$R/driver.log"; then
  { echo "end $(date +%FT%T)"; echo "stop port_taken"; } >> "$R/meta.txt"; kill "$DRV" 2>/dev/null; exit 3
fi
MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=${GPU:-0} /root/mj/venv/bin/python "$HERE/robot.py" --connect "127.0.0.1:$PORT" \
  --kind "$KIND" --seed "$SEED" --out "$R" --stop "$R/stop" "$@" > "$R/robot.log" 2>&1 &
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
kill "$ROB" 2>/dev/null; kill "$DRV" 2>/dev/null; wait 2>/dev/null
{ echo "end $(date +%FT%T)"; echo "stop $why"; } >> "$R/meta.txt"
