#!/bin/bash
# Boot the driver on a fake robot (driver/tools/fake_robot) and score it: one run of one seed.
#
#   bash fake.sh NAME SEED [fake_robot options...]
#
# Output: $FAKE_RUNS/NAME (default $HOME/fakeruns/NAME): run.rec.zst (the driver's own recording,
# compressed as it is written), truth.jsonl and geometry/ (the fake robot's truth, in the simulator's
# format), scramble.json (the draw), body.txt (the body in words), body.json (what the driver
# measured), driver.log, fake.log, meta.txt, and after the run estimates.jsonl, replay.log and
# score.txt (replay --estimates then score, as qscore.sh does). The run ends when the driver's log
# says the body is measured, when the boot fails, when either side exits, or at FAKE_MINUTES
# (default 60). FAKE_BRAIN=HOST:PORT gives the driver a brain service (Qwen behind llama-server, as the kiwi has) with
# the model card's non-thinking sampling; FAKE_UNTIL=task does not stop when the body is measured but when the brain
# says done or the episode fails (the run then goes on to the task the robot hands the brain). # (default 60). BIN names the driver's tools (default the repository's driver/bin); FAKE_PORT the
# port (default 9500 plus the seed modulo 400, so runs of different seeds can go at once).
set -u
NAME=$1; SEED=$2; shift 2
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN=${BIN:-$ROOT/driver/bin}
RUNS=${FAKE_RUNS:-$HOME/fakeruns}
R=$RUNS/$NAME
[ -e "$R" ] && { echo "$R already exists; pick another name"; exit 5; }
mkdir -p "$R/bin"
# FAKE_BODY: a body file the driver starts from (what an earlier run of the same seed measured), copied in.
[ -n "${FAKE_BODY:-}" ] && cp "$FAKE_BODY" "$R/body.json"
# The run keeps its own copies of the tools, so a build during the run changes nothing it runs or scores with.
cp -p "$BIN/body_driver" "$BIN/fake_robot" "$BIN/replay" "$BIN/score" "$R/bin/"
PORT=${FAKE_PORT:-$((9500 + SEED % 400))}
LIM=${FAKE_MINUTES:-60}
{ echo "seed $SEED"; echo "port $PORT"; echo "driver $(md5 -q "$BIN/body_driver" 2>/dev/null || md5sum "$BIN/body_driver" | cut -d' ' -f1)";
  echo "options $*"; echo "start $(date +%FT%T)"; } > "$R/meta.txt"
mkfifo "$R/run.rec.fifo"
zstd -q -T2 -3 -o "$R/run.rec.zst" < "$R/run.rec.fifo" &
ZST=$!
BRAIN=(); if [ -n "${FAKE_BRAIN:-}" ]; then BRAIN=(--eye "$FAKE_BRAIN"); export BL_BRAIN_SAMPLING='{"temperature":1.0,"top_p":1.0,"top_k":20,"min_p":0,"presence_penalty":2.0,"repetition_penalty":1.0,"chat_template_kwargs":{"enable_thinking":false}}'; fi
# FAKE_INST=HOST:PORT: the instrument service (harness/instruments/serve.py; segmentation only by default), as the kiwi runs it.
INST=(); [ -n "${FAKE_INST:-}" ] && INST=(--inst "$FAKE_INST")
"$R/bin/body_driver" --listen "$PORT" --body "$R/body.json" --record "$R/run.rec.fifo" ${BRAIN[@]+"${BRAIN[@]}"} ${INST[@]+"${INST[@]}"} > "$R/driver.log" 2>&1 &
DRV=$!
for _ in $(seq 1 120); do grep -q "listening on port" "$R/driver.log" 2>/dev/null && break; sleep 0.5; done
"$R/bin/fake_robot" --connect "127.0.0.1:$PORT" --seed "$SEED" --out "$R" --stop "$R/stop" "$@" > "$R/fake.log" 2>&1 &
FAKE=$!
end=$(( $(date +%s) + LIM * 60 )); why=limit
while [ "$(date +%s)" -lt "$end" ]; do
  if [ "${FAKE_UNTIL:-measured}" = task ]; then
    if grep -q "the brain said done" "$R/driver.log"; then why=done; break; fi
    if grep -q "the episode failed" "$R/driver.log"; then why=episode_failed; break; fi
  elif grep -q "the body as measured:" "$R/driver.log"; then why=measured; break; fi
  if grep -qE "could not be measured|the boot failed|the driver failed|the estimates failed" "$R/driver.log"; then why=boot_failed; break; fi
  kill -0 "$FAKE" 2>/dev/null || { why=fake_exit; break; }
  kill -0 "$DRV" 2>/dev/null || { why=driver_exit; break; }
  sleep 2
done
touch "$R/stop"
for _ in $(seq 1 30); do kill -0 "$FAKE" 2>/dev/null || break; sleep 1; done
kill "$FAKE" 2>/dev/null
kill "$DRV" 2>/dev/null
sleep 1
kill -9 "$DRV" 2>/dev/null
wait "$ZST" 2>/dev/null
rm -f "$R/run.rec.fifo" "$R/stop"
{ echo "end $(date +%FT%T)"; echo "stop $why"; } >> "$R/meta.txt"
echo "$NAME: $why ($(grep -c "" "$R/truth.jsonl") truth lines)"
# FAKE_REPLAY=0: no replay and no score after the run (a run stopped once its log told what it was for).
[ "${FAKE_REPLAY:-1}" = 0 ] && exit 0
zstd -q -dc "$R/run.rec.zst" | "$R/bin/replay" /dev/stdin --estimates "$R/estimates.jsonl" > "$R/replay.log" 2>&1
tail -2 "$R/replay.log"
SCR=(); [ -f "$R/scramble.json" ] && SCR=(--scramble "$R/scramble.json")
zstd -q -dc "$R/run.rec.zst" | "$R/bin/score" ${SCR[@]+"${SCR[@]}"} "$R/estimates.jsonl" /dev/stdin "$R/truth.jsonl" | tee "$R/score.txt"
