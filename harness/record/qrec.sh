#!/bin/bash
# Record one boot from zero with the wire proxy between the simulator and the driver.
# The driver listens on :9080 as usual; the proxy listens on :9090 and the simulator connects to the proxy.
# Usage: bash qrec.sh NAME TASK CFG [SEED] [LIMIT_MINUTES]
# Output: /root/rec/NAME/{wire.rec.zst, cal.log, sim.log, meta.txt, look/, cal.json, 经历.txt}
# Shares the single simulator slot with everyone else through /root/q/sim.lock.
set -u
K=$1; TASK=$2; CFG=$3; SEED=${4:-0}; LIM=${5:-25}
HERE=$(cd "$(dirname "$0")" && pwd)
R=/root/rec/$K
[ -e "$R" ] && { echo "$R already exists; pick another name"; exit 5; }
mkdir -p "$R/look"
QWEN_CARD='{"temperature":0.7,"top_p":0.8,"top_k":20,"min_p":0,"presence_penalty":1.5,"repetition_penalty":1.0}'

exec 9>/root/q/sim.lock
echo "$(date +%T) rec queued: $K ($TASK, $CFG, seed $SEED)" >> /root/q/queue.log
flock 9
echo "$(date +%T) rec start: $K" >> /root/q/queue.log
echo "rec $K $(date +%s)" > /root/q/now.txt

# Clear leftovers. Patterns are split so pkill never matches this script's own command line.
P1=eval_pol; P1="${P1}icy"; P2=eval_cli; P2="${P2}ent"; L1=--lis; L2="ten 9080 --out"; W1=wire_pro; W1="${W1}xy"
pkill -9 -f "$P1" 2>/dev/null; pkill -9 -f "$P2" 2>/dev/null; pkill -9 -f -- "$L1$L2" 2>/dev/null; pkill -9 -f "$W1" 2>/dev/null
sleep 4
ss -ltn | grep -q ":8077 " || (cd /root/instruments && bash run.sh) || { echo "instrument service did not start"; exit 2; }

{ echo "task $TASK"; echo "cfg $CFG"; echo "seed $SEED"; echo "driver $(md5sum /root/.local/bin/bl-calibrate | cut -d' ' -f1)"; echo "start $(date +%FT%T)"; } > "$R/meta.txt"

cd /root/body-layer
BL_NO_DEPTH=1 BL_MDE_OFF=1 BL_INST=127.0.0.1:8077 BL_BRAIN_SAMPLING="$QWEN_CARD" BL_LIFE="$R/经历.txt" \
  BL_SPAN_LEN=3000 BL_DUMP="$R/look" BL_VID= setsid nohup \
  /root/.local/bin/bl-calibrate --listen 9080 --out "$R/cal.json" --eye 127.0.0.1:8078 </dev/null >"$R/cal.log" 2>&1 &
sleep 3
setsid nohup /venv/RoboDojo/bin/python "$HERE/wire_proxy.py" 9090 ws://127.0.0.1:9080 "$R/wire.rec" </dev/null >"$R/proxy.log" 2>&1 &
PROXY=$!
sleep 2
cd /root/RoboDojo
OMNI_KIT_ACCEPT_EULA=YES PATH=/venv/RoboDojo/bin:$PATH BD_STEP_LIM=3000 setsid nohup bash scripts/eval_policy.sh \
  --root_dir /root/RoboDojo --task_name "$TASK" --env_cfg_type "$CFG" --device_id 0 --policy_name l3_link \
  --port 9090 --protocol ws --policy_server_url "ws://127.0.0.1:9090" --seed "$SEED" --host 127.0.0.1 \
  --enable_cameras --headless </dev/null >"$R/sim.log" 2>&1 &

# Stop at the first brain round (boot finished), when the driver exits, or at the time limit.
end=$(( $(date +%s) + LIM * 60 )); why=limit
while [ "$(date +%s)" -lt $end ]; do
  if grep -q "── 第 1 轮" "$R/cal.log" 2>/dev/null; then why=round1; sleep 15; break; fi
  pgrep -f -- "$L1$L2" >/dev/null || { why=driver_exit; break; }
  sleep 10
done
pkill -9 -f "$P2" 2>/dev/null; pkill -9 -f "$P1" 2>/dev/null
kill -TERM $PROXY 2>/dev/null; sleep 3
pkill -f -- "$L1$L2" 2>/dev/null
{ echo "end $(date +%FT%T)"; echo "stop $why"; echo "raw_bytes $(stat -c %s "$R/wire.rec" 2>/dev/null)"; } >> "$R/meta.txt"
zstd -q -T0 --rm "$R/wire.rec" && echo "zst_bytes $(stat -c %s "$R/wire.rec.zst")" >> "$R/meta.txt"
echo "$(date +%T) rec done: $K ($why)" >> /root/q/queue.log
rm -f /root/q/now.txt
