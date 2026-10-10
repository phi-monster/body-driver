#!/bin/bash
# Run the new driver live on one task through the shared simulator slot, recording everything it
# exchanges (driver-side --record, service replies included).
#
#   bash qnew.sh NAME TASK CFG SEED LIMIT_MINUTES DRIVER_BIN [BODY_FILE]
#
# Output: /root/runs/NAME/{run.rec.zst, truth.jsonl.zst, driver.log, sim.log, meta.txt}. The recording is
# compressed while it is written (the driver records into a FIFO that zstd reads), so a live boot,
# about 2.7 MB a beat, never holds a raw recording on disk; truth.jsonl is compressed at the end.
# truth.jsonl is the simulator's side-file truth for scoring (harness/robodojo_truth), which the
# driver never sees; its geometry goes to the shared store /root/rec/geometry. Every run is scrambled (harness/robodojo_scramble):
# the driver is given the readings renamed, reordered and with each channel its own sign, scale and zero; the
# draw goes to scramble.json beside the run, for the scorer, and the seed to sim.log. BD_SCRAMBLE=0 turns it
# off, BD_SCRAMBLE_SEED fixes the draw (a body file is the body under the draw it was kept under). The run stops when
# the driver exits, when /root/q/done_NAME appears, or at the limit.
# The brain (127.0.0.1:8078) and the instrument (127.0.0.1:8077) must already be serving; the
# brain's sampling settings are passed through BL_BRAIN_SAMPLING.
#
# Two runs can go at once, each in its own slot: BD_SLOT=0 (the default) is the first card and port
# 9080, BD_SLOT=1 the second card and port 9081. The second card holds the brain, so a slot-1 run
# needs the brain stopped and is for runs that do not ask it (a boot); each slot queues on its own
# lock and clears only its own leftovers. Nothing it starts keeps the lock open (9>&-): the instrument
# service it once started held slot 1's lock for hours, and the next run there never began.
set -u
K=$1; TASK=$2; CFG=$3; SEED=$4; LIM=$5; BIN=$6; BODY=${7:-}
R=/root/runs/$K
[ -e "$R" ] && { echo "$R already exists; pick another name"; exit 5; }
mkdir -p "$R"
SLOT="${BD_SLOT:-0}"; PORT=$((9080 + SLOT))
LOCK=/root/q/sim.lock; NOW=/root/q/now.txt
[ "$SLOT" = 0 ] || { LOCK=/root/q/slot$SLOT.lock; NOW=/root/q/now$SLOT.txt; }
if [ "$SLOT" != 0 ] && ss -ltn | grep -q ":8078 "; then
  echo "the brain is serving on the card of slot $SLOT; stop it first (or run in slot 0)"; rmdir "$R"; exit 6
fi
QWEN_CARD='{"temperature":0.7,"top_p":0.8,"top_k":20,"min_p":0,"presence_penalty":1.5,"repetition_penalty":1.0}'

exec 9>"$LOCK"
echo "$(date +%T) new-driver run queued: $K ($TASK, $CFG, seed $SEED, slot $SLOT)" >> /root/q/queue.log
flock 9
echo "$(date +%T) new-driver run start: $K" >> /root/q/queue.log
echo "new $K $(date +%s)" > "$NOW"

# Clear this slot's leftovers (the simulator's processes carry its URL, the driver its port); the
# patterns are split so pkill never matches this script.
U1="ws://127.0.0.1"; U2=":$PORT"; L1=--lis; L2="ten $PORT"
pkill -9 -f "$U1$U2" 2>/dev/null; pkill -9 -f -- "$L1$L2" 2>/dev/null
sleep 4
# A loaded box can take minutes to load the instrument's models: wait while its process lives.
ss -ltn | grep -q ":8077 " || (cd /root/instruments && bash run.sh 9>&-) || {
  while pgrep -f "venv_inst/bin/python serve" > /dev/null && ! ss -ltn | grep -q ":8077 "; do sleep 5; done
  ss -ltn | grep -q ":8077 " || { echo "instrument service did not start"; exit 2; }; }

{ echo "task $TASK"; echo "cfg $CFG"; echo "seed $SEED"; echo "driver $(md5sum "$BIN" | cut -d' ' -f1)";
  echo "body ${BODY:-none}"; echo "scramble ${BD_SCRAMBLE:-1} ${BD_SCRAMBLE_SEED:-drawn}"; echo "start $(date +%FT%T)"; } > "$R/meta.txt"

mkfifo "$R/run.rec.fifo"
# Level 17 finds each picture in the one a beat before it: a recording takes a seventh of the room
# level 3 took (a 2-hour boot 30 GB before), and eight threads still compress six times as fast as a
# run records, so the FIFO never holds the driver up. Any zstd reads it back without options.
zstd -q -T8 -17 -o "$R/run.rec.zst" < "$R/run.rec.fifo" 9>&- &
ZST=$!
BL_BRAIN_SAMPLING="${BL_BRAIN_SAMPLING:-$QWEN_CARD}" setsid nohup "$BIN" --listen "$PORT" --eye 127.0.0.1:8078 \
  --inst 127.0.0.1:8077 ${BODY:+--body "$BODY"} --record "$R/run.rec.fifo" </dev/null >"$R/driver.log" 2>&1 9>&- &
sleep 2
cd /root/RoboDojo
BD_TRUTH="$R/truth.jsonl" BD_TRUTH_GEOMETRY=/root/rec/geometry BD_SCRAMBLE_MAP="$R/scramble.json" OMNI_KIT_ACCEPT_EULA=YES PATH=/venv/RoboDojo/bin:$PATH \
  BD_STEP_LIM="${BD_STEP_LIM:-3000}" setsid nohup \
  bash scripts/eval_policy.sh --root_dir /root/RoboDojo --task_name "$TASK" --env_cfg_type "$CFG" --device_id "$SLOT" \
  --policy_name l3_link --port "$PORT" --protocol ws --policy_server_url "ws://127.0.0.1:$PORT" --seed "$SEED" \
  --host 127.0.0.1 --enable_cameras --headless </dev/null >"$R/sim.log" 2>&1 9>&- &

end=$(( $(date +%s) + LIM * 60 )); why=limit
while [ "$(date +%s)" -lt $end ]; do
  [ -f "/root/q/done_$K" ] && { why=done_file; break; }
  pgrep -f -- "$L1$L2" >/dev/null || { why=driver_exit; break; }
  sleep 10
done
pkill -9 -f "$U1$U2" 2>/dev/null; pkill -f -- "$L1$L2" 2>/dev/null
sleep 2
pkill -9 -f -- "$L1$L2" 2>/dev/null
# A driver that never opened the FIFO leaves zstd waiting for a writer: one empty open releases it.
timeout 5 bash -c ": > '$R/run.rec.fifo'" 2>/dev/null
wait $ZST
rm -f "$R/run.rec.fifo"
{ echo "end $(date +%FT%T)"; echo "stop $why"; } >> "$R/meta.txt"
[ -f "$R/truth.jsonl" ] && zstd -q -T0 --rm "$R/truth.jsonl"
grep -h "Success nums" "$R/sim.log" | tail -1 >> "$R/meta.txt"
echo "$(date +%T) new-driver run done: $K ($why)" >> /root/q/queue.log
rm -f "$NOW" "/root/q/done_$K"
