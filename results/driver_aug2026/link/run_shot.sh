#!/usr/bin/env bash
# 一炮 = 起驱动 + 起仿真 + 收日志。**这个文件就是复现命令** —— 档案里"跑过但没人记得怎么跑的"
# 那一类事故,靠的就是把命令留在租来的机器上而不是留在库里。
#
# 用法:bash run_shot.sh <标签> <相机> <任务> <集数> [GPU]
#   例:bash run_shot.sh shotA cam_head general_pickup 1 0
#
# 🔴 三件写死的事(改之前先想清楚):
#   ① `--cal` 必给 —— 没有身体常数的闭环会照常跑完并产出一批"看起来正常的零"。
#   ② `--env_cfg_type arx_x5_shot1w` 是**全相机**那份;它的 `config_name` 必须保持
#      `arx_x5_shot1`(那个键**索引布局目录**,改了 `seed_manager.init_eval` 当场崩)。
#   ③ 杀与起分两次执行,`pkill` 的字面量拆开写 —— 否则会杀到自己这条命令。
set -u
TAG="${1:?要个标签}"; CAM="${2:?要个相机名}"; TASK="${3:?要个任务名}"; N="${4:-1}"; GPU="${5:-0}"
CAL=/root/bodycal/116bd6e559e10b06.json
PORT=$((9077 + GPU))
OUT=/root/$TAG
mkdir -p "$OUT"

cd /root/link || exit 1
# `LINK_CALHEAD`(标定集)与 `LINK_HEADJAC`(粗相机 2×2)由调用者的环境透传 ——
# 它们是**这一炮是哪一种炮**的开关,不是每炮都要填的参数。
LINK_CAM="$CAM" LINK_DUMPCLOSE="$OUT" LINK_DUMPNULL="$OUT" LINK_DESCRIBE=1 \
  LINK_CALHEAD="${LINK_CALHEAD:-}" LINK_HEADJAC="${LINK_HEADJAC:-}" LINK_MINPX="${LINK_MINPX:-}" \
  nohup ./target/release/link --port "$PORT" --cal "$CAL" --arm left \
  > "$OUT/link.log" 2>&1 &
sleep 2
grep -q "在听" "$OUT/link.log" || { echo "[$TAG] 驱动没起来:"; cat "$OUT/link.log"; exit 2; }

cd /root/RoboDojo || exit 1
export OMNI_KIT_ACCEPT_EULA=YES
export PATH=/venv/RD511/bin:$PATH
# 🔴 `RD_STEP_LIM` 只准给**标定炮**加长,计分炮一律留在默认的 200。
# 标定量的是身体(这台相机上一米等于多少像素),不是成绩;而一次认手要 ~64 步、
# 三次加两次移动装不进 200(实测每集只做得完两次)。计分炮改这个数就是改题目 ⇒ 不许。
[ -n "${LINK_CALHEAD:-}" ] && export RD_STEP_LIM="${RD_STEP_LIM:-600}"
# 🔴 `--policy_name l3_link`,**不是 `external`**。
# `XPolicyLab/policy/external/deploy.yml` 是个**空文件** ⇒ 缺字段 ⇒ 评测端一遍遍重建场景
# (实测:`Completed setting up the environment` 出现 15 次、一步都没跑、驱动那侧只看到「接上了」)。
# 病相在"卡住",而根因在一个空的 yaml —— 两侧日志都不报错。
# `l3_link/deploy.yml` 是这条链原本就配好的那份(`eval_batch: false` ⇒ num_envs 强制成 1)。
nohup bash scripts/eval_policy.sh \
  --root_dir /root/RoboDojo \
  --task_name "$TASK" --env_cfg_type arx_x5_shot1w --device_id "$GPU" \
  --policy_name l3_link --port "$PORT" --protocol ws \
  --policy_server_url "ws://127.0.0.1:$PORT" --seed 0 --host 127.0.0.1 \
  --enable_cameras --headless \
  > "$OUT/sim.log" 2>&1 &
echo "[$TAG] 起了:相机=$CAM 任务=$TASK 集数=$N GPU=$GPU 端口=$PORT ⇒ $OUT/{link,sim}.log"
