#!/bin/bash
# Run the new driver live on one task through the shared simulator slot, recording everything it
# exchanges (driver-side --record, service replies included).
#
#   bash qnew.sh NAME TASK CFG SEED LIMIT_MINUTES DRIVER_BIN [BODY_FILE]
#
# Output: /root/runs/NAME/{run.rec, truth.jsonl, driver.log, sim.log, meta.txt}, both compressed with
# zstd at the end; truth.jsonl is the simulator's side-file truth for scoring (harness/robodojo_truth),
# which the driver never sees, and its geometry goes to the shared store /root/rec/geometry. The run stops when the driver exits, when /root/q/done_NAME appears, or at the limit.
# The brain (127.0.0.1:8078) and the instrument (127.0.0.1:8077) must already be serving; the
# brain's sampling settings are passed through BL_BRAIN_SAMPLING.
set -u
K=$1; TASK=$2; CFG=$3; SEED=$4; LIM=$5; BIN=$6; BODY=${7:-}
R=/root/runs/$K
[ -e "$R" ] && { echo "$R already exists; pick another name"; exit 5; }
mkdir -p "$R"
QWEN_CARD='{"temperature":0.7,"top_p":0.8,"top_k":20,"min_p":0,"presence_penalty":1.5,"repetition_penalty":1.0}'

exec 9>/root/q/sim.lock
echo "$(date +%T) new-driver run queued: $K ($TASK, $CFG, seed $SEED)" >> /root/q/queue.log
flock 9
echo "$(date +%T) new-driver run start: $K" >> /root/q/queue.log
echo "new $K $(date +%s)" > /root/q/now.txt

# Clear leftovers; patterns are split so pkill never matches this script.
P1=eval_pol; P1="${P1}icy"; P2=eval_cli; P2="${P2}ent"; L1=--lis; L2="ten 9080"
pkill -9 -f "$P1" 2>/dev/null; pkill -9 -f "$P2" 2>/dev/null; pkill -9 -f -- "$L1$L2" 2>/dev/null
sleep 4
ss -ltn | grep -q ":8077 " || (cd /root/instruments && bash run.sh) || { echo "instrument service did not start"; exit 2; }

{ echo "task $TASK"; echo "cfg $CFG"; echo "seed $SEED"; echo "driver $(md5sum "$BIN" | cut -d' ' -f1)";
  echo "body ${BODY:-none}"; echo "start $(date +%FT%T)"; } > "$R/meta.txt"

BL_BRAIN_SAMPLING="${BL_BRAIN_SAMPLING:-$QWEN_CARD}" setsid nohup "$BIN" --listen 9080 --eye 127.0.0.1:8078 \
  --inst 127.0.0.1:8077 ${BODY:+--body "$BODY"} --record "$R/run.rec" </dev/null >"$R/driver.log" 2>&1 &
sleep 2
cd /root/RoboDojo
BD_TRUTH="$R/truth.jsonl" BD_TRUTH_GEOMETRY=/root/rec/geometry OMNI_KIT_ACCEPT_EULA=YES PATH=/venv/RoboDojo/bin:$PATH \
  BD_STEP_LIM="${BD_STEP_LIM:-3000}" setsid nohup \
  bash scripts/eval_policy.sh --root_dir /root/RoboDojo --task_name "$TASK" --env_cfg_type "$CFG" --device_id 0 \
  --policy_name l3_link --port 9080 --protocol ws --policy_server_url "ws://127.0.0.1:9080" --seed "$SEED" \
  --host 127.0.0.1 --enable_cameras --headless </dev/null >"$R/sim.log" 2>&1 &

end=$(( $(date +%s) + LIM * 60 )); why=limit
while [ "$(date +%s)" -lt $end ]; do
  [ -f "/root/q/done_$K" ] && { why=done_file; break; }
  pgrep -f -- "$L1$L2" >/dev/null || { why=driver_exit; break; }
  sleep 10
done
pkill -9 -f "$P2" 2>/dev/null; pkill -9 -f "$P1" 2>/dev/null; pkill -f -- "$L1$L2" 2>/dev/null
sleep 2
{ echo "end $(date +%FT%T)"; echo "stop $why"; } >> "$R/meta.txt"
[ -f "$R/run.rec" ] && zstd -q -T0 --rm "$R/run.rec"
[ -f "$R/truth.jsonl" ] && zstd -q -T0 --rm "$R/truth.jsonl"
grep -h "Success nums" "$R/sim.log" | tail -1 >> "$R/meta.txt"
echo "$(date +%T) new-driver run done: $K ($why)" >> /root/q/queue.log
rm -f /root/q/now.txt "/root/q/done_$K"
