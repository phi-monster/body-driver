#!/bin/bash
# round.sh ROUND JOBS LIST: one round of cold boots on the big machine. LIST has one boot a line, "NAME SEED [fake_robot
# options...]"; JOBS of them run at once, each through harness/run/fake.sh with its own port, no recording, two
# workers, stopping when the body is measured, the boot fails, or at FAKE_MINUTES. Every boot's line in
# $RUNS/ROUND/status.txt says how it ended. Ports from PORT_BASE (default 11000) up: rounds that run at once need their
# own. Code: BIN (default /root/bd/driver/bin), runs under $RUNS/ROUND. STOP_AT, a regular expression: a boot is stopped
# as soon as a line of its driver's log matches it (STOP_AT='boot: the groups as recognized' runs step 1 alone).
# DUMP_FITS set: every fit's inputs go to $RUNS/ROUND/NAME.fits (BD_DUMP_FITS), for driver-robot-fit_replay.
# RATE: the fake's beats a second at the most (default 0: as fast as the driver answers).
ROUND=$1; JOBS=$2; LIST=$3
export RUNS=${RUNS:-/root/runs}
export FAKE_RUNS=$RUNS/$ROUND BIN=${BIN:-/root/bd/driver/bin} FAKE_RECORD=${FAKE_RECORD:-0} BD_WORKERS=${BD_WORKERS:-2}
export FAKE_MINUTES=${FAKE_MINUTES:-150}
mkdir -p "$FAKE_RUNS"
cd /root/bd
i=0
while read -r NAME SEED OPTS; do
  [ -z "$NAME" ] && continue
  i=$((i + 1))
  while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do sleep 5; done
  ( if [ -n "${DUMP_FITS:-}" ]; then mkdir -p "$FAKE_RUNS/$NAME.fits"; export BD_DUMP_FITS="$FAKE_RUNS/$NAME.fits"; fi
    FAKE_PORT=$((${PORT_BASE:-11000} + i)) bash harness/run/fake.sh "$NAME" "$SEED" --rate ${RATE:-0} $OPTS > "$FAKE_RUNS/$NAME.launch.txt" 2>&1 &
    P=$!
    if [ -n "${STOP_AT:-}" ]; then
      while kill -0 $P 2> /dev/null; do
        if grep -qE "$STOP_AT" "$FAKE_RUNS/$NAME/driver.log" 2> /dev/null; then touch "$FAKE_RUNS/$NAME/stop"; break; fi
        sleep 5
      done
    fi
    wait $P; S=$?
    echo "$NAME $SEED $S $(date +%H:%M) $(tail -1 "$FAKE_RUNS/$NAME/driver.log" 2>/dev/null | grep -oE '^@[0-9]+')" >> "$FAKE_RUNS/status.txt" ) &
done < "$LIST"
wait
echo "round $ROUND done $(date +%H:%M)" >> "$FAKE_RUNS/status.txt"
