#!/bin/bash
# 大并行 §2 第 34 条:五种相机条件各从零开一次机(主线驱动,只读关节;身体 x5),开到第 1 轮就放锁(harness/scenes/qboot.sh)。
#   distort  镜头径向畸变 k1 −0.15、k2 0.03(真焦距:头顶 288.1、腕 397.0 px)   conditions/distort.json
#   delay2   彩色图晚 2 帧(位姿读数照旧是这一帧的;仿真本来就晚 1 帧,加起来晚 3 帧)  conditions/delay2.json
#   blur     高斯模糊 σ 1.5 px                                                   conditions/blur.json
#   noise    高斯噪声 σ 6 灰度                                                     conditions/noise.json
#   nohead   没有头顶眼:RoboDojo 的 arx_x5_nohead 配置(相机里就没有 cam_head,不是把图涂黑)
# 钩子是 bd_camtest.py(装在 RoboDojo 的 env/observation_manager/,obs_manager.get_obs 里每一步调一次),触发文件按这一炮给
# (BD_CAMTEST 环境变量,不用全箱共用的 /root/camtest.json:那个文件留着别路的炮也会被改图)。
# 每一炮都留落盘和逐帧位姿(BOOT_KEEP=1),跑完用 sum_conditions.py 按仿真真值打分(焦距、畸变、手在哪 —— /root/diag/v1b_score_fk_cur.py)。
# 用法(箱上):bash run_conditions.sh [条件 …](默认五种全跑);证据在 /root/p8/boot/<炮名>/ 和 /root/N<炮名>/
set -u
H=$(cd "$(dirname "$0")" && pwd)
Q=/root/p8/scenes/qboot.sh
declare -A SHOT=([distort]=P8VD [delay2]=P8VL [blur]=P8VB [noise]=P8VN [nohead]=P8VH)
for c in ${@:-distort delay2 blur noise nohead}; do
  k=${SHOT[$c]}
  if [ "$c" = nohead ]; then
    BOOT_KEEP=1 BOOT_CFG=arx_x5_nohead BOOT_SEED=0 bash "$Q" "$k" bootcal zero 25
  else
    BOOT_KEEP=1 BD_CAMTEST="$H/conditions/$c.json" BOOT_CFG=arx_x5 BOOT_SEED=0 bash "$Q" "$k" bootcal zero 25
  fi
  echo "$(date +%T) $c → $k:$(tail -4 /root/p8/boot/$k/meta.txt | tr '\n' ' ')"
done
