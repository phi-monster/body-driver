#!/usr/bin/env bash
K="$1"
cd /root/RoboDojo
export OMNI_KIT_ACCEPT_EULA=YES PATH=/venv/RoboDojo/bin:$PATH RD_STEP_LIM=20000
setsid nohup bash scripts/eval_policy.sh --root_dir /root/RoboDojo --task_name cube_pickup --env_cfg_type franka_grasp --device_id 0 --policy_name l3_link --port 9080 --protocol ws --policy_server_url "ws://127.0.0.1:9080" --seed 0 --host 127.0.0.1 --enable_cameras --headless </dev/null >/root/N$K/sim.log 2>&1 &
exit 0
