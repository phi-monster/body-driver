#!/usr/bin/env bash
# 起 sim。2026-08-23 换到官方配置:
#   任务  general_pickup(上游把 cube_pickup 改成了它,判据没变:抬 10 cm,200 步)
#   机体  单臂 Franka(env_cfg/franka_grasp.yml,重写版已进 git)
#   x5_grasp = 官方 arx_x5 只把相机深度打开
K="$1"
TASK="${2:-general_pickup}"   # 第二个参数换任务:bootcal=开机自检(长集),general_pickup=记分
cd /root/RoboDojo
export OMNI_KIT_ACCEPT_EULA=YES PATH=/venv/RoboDojo/bin:$PATH RD_STEP_LIM=20000
setsid nohup bash scripts/eval_policy.sh --root_dir /root/RoboDojo --task_name "$TASK" --env_cfg_type franka_grasp --device_id 0 --policy_name l3_link --port 9080 --protocol ws --policy_server_url "ws://127.0.0.1:9080" --seed 0 --host 127.0.0.1 --enable_cameras --headless </dev/null >/root/N$K/sim.log 2>&1 &
exit 0
