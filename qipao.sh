#!/usr/bin/env bash
# 起炮:强制"每炮必须带改动"。同一个二进制指纹不许连起两炮(owner 2026-08-23 定)。
set -e
K="$1"
[ -z "$K" ] && { echo "用法: qipao.sh <炮号,如 107>"; exit 1; }
NEW=$(ssh vast5 'md5sum /root/.local/bin/bl-calibrate | cut -d" " -f1')
PREV=$(ssh vast5 "cat /root/N$((K-1))/BUILD.md5 2>/dev/null || echo none")
if [ "$NEW" = "$PREV" ]; then
  echo "🔴 拒绝起炮 N$K:二进制指纹与上一炮 N$((K-1)) 相同($NEW)= 无改动炮。先带一把刀再来。"
  exit 2
fi
echo "指纹 上一炮 $PREV → 本炮 $NEW(有改动,放行)"
ssh vast5 "mkdir -p /root/N$K/look && echo $NEW > /root/N$K/BUILD.md5 && cd /root/body-layer && BL_DUMP=/root/N$K/look setsid nohup /root/.local/bin/bl-calibrate --listen 9080 --out /root/N$K/cal.json --in /root/M0/cal.json --eye 127.0.0.1:8077 </dev/null >/root/N$K/cal.log 2>&1 & exit 0" || true
sleep 5
ssh vast5 "cd /root/RoboDojo && export OMNI_KIT_ACCEPT_EULA=YES PATH=/venv/RoboDojo/bin:\$PATH RD_STEP_LIM=20000 && setsid nohup bash scripts/eval_policy.sh --root_dir /root/RoboDojo --task_name cube_pickup --env_cfg_type franka_grasp --device_id 0 --policy_name l3_link --port 9080 --protocol ws --policy_server_url 'ws://127.0.0.1:9080' --seed 0 --host 127.0.0.1 --enable_cameras --headless </dev/null >/root/N$K/sim.log 2>&1 & exit 0" || true
echo "N$K 起了(指纹 $NEW)"
