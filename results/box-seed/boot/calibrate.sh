#!/bin/bash
# 台架件:起驱动的自标定 + 让台子喂长集观测。**驱动侧零参数、零任务名。**
# 用法: bash calibrate.sh <标签> <GPU> [种子]
# 🔴 杀与起分两次调用(仓规);这里只起,不杀。
set -u
TAG=${1:?要个标签}; GPU=${2:?要个GPU}; SEED=${3:-0}
PORT=$((9077+GPU)); OUT=/root/$TAG; mkdir -p "$OUT"
nohup /root/.local/bin/bl-calibrate --listen "$PORT" --out "$OUT/cal.json" > "$OUT/cal.log" 2>&1 &
sleep 3
grep -q "等这台机器人" "$OUT/cal.log" || { echo "[$TAG] 驱动没起来:"; cat "$OUT/cal.log"; exit 2; }
cd /root/RoboDojo || exit 1
export OMNI_KIT_ACCEPT_EULA=YES
if [ -x /venv/RoboDojo/bin/python ]; then export PATH=/venv/RoboDojo/bin:$PATH; fi
# 标定场:长集(20000 步)。这是台子的配置,不是驱动的。
export RD_STEP_LIM=20000
nohup bash scripts/eval_policy.sh \
  --root_dir /root/RoboDojo \
  --task_name cube_pickup --env_cfg_type franka_grasp --device_id "$GPU" \
  --policy_name l3_link --port "$PORT" --protocol ws \
  --policy_server_url "ws://127.0.0.1:$PORT" --seed "$SEED" --host 127.0.0.1 \
  --enable_cameras --headless > "$OUT/sim.log" 2>&1 &
echo "[$TAG] 起了:GPU=$GPU 端口=$PORT 种子=$SEED ⇒ $OUT/{cal,sim}.log"
