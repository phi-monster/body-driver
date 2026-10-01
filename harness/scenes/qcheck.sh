#!/bin/bash
# 路 8 离线核场景(不起驱动、不接脑,只起 Isaac):跟 /root/q/run.sh 拿同一把锁(箱上只有一个仿真位),占着的时候 now.txt 写明是谁。
# 用法:bash qcheck.sh 任务1 [任务2 …]      结果:/root/p8/chk/<标签>/{report.json, *.png},日志 /root/p8/chk/<标签>.log(标签默认 = 任务名)
# 别的身体 / 种子 / 参数用环境变量给(对这一次的每个任务都一样):
#   CHK_CFG(默认 arx_x5)、CHK_SEED(默认 0)、CHK_EXTRA(比如 "--walk_label target --layouts 0")、
#   CHK_TAG(结果目录名的后缀)、CHK_PYPRE(排在 RoboDojo 前面的 PYTHONPATH:拿改过的副本核,不动原文件)
set -u
[ $# -ge 1 ] || { echo "用法:bash qcheck.sh 任务…"; exit 1; }
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=/root/p8/chk
mkdir -p $OUT /root/q
echo "$(date +%T) 路8 排队:离线核场景 $*" >> /root/q/queue.log
exec 9>/root/q/sim.lock
flock 9
echo "$(date +%T) 路8 开跑:离线核场景 $*" >> /root/q/queue.log
echo "路8 离线核场景($*) $(date +%s)" > /root/q/now.txt
FREE_G=$(df -BG /root | awk 'NR==2 {gsub("G","",$4); print $4}')
[ "$FREE_G" -ge 15 ] || { echo "🔴 盘只剩 ${FREE_G}G" | tee -a /root/q/queue.log; rm -f /root/q/now.txt; exit 3; }
cd /root/RoboDojo
export OMNI_KIT_ACCEPT_EULA=YES PATH=/venv/RoboDojo/bin:$PATH CUDA_VISIBLE_DEVICES=0
export PYTHONPATH=${CHK_PYPRE:+$CHK_PYPRE:}/root/RoboDojo:/root/RoboDojo/XPolicyLab
for T in "$@"; do
  TAG="$T${CHK_TAG:+_$CHK_TAG}"
  rm -rf "$OUT/$TAG"
  # 一个任务三张布局约 2.5 分钟;10 分钟封顶(Kit 关机卡住也占不住仿真位),-k:TERM 不理就 KILL
  timeout -k 20 600 python -u "$HERE/check_scenes.py" --task "$T" --tag "$TAG" --out "$OUT" --cfg "${CHK_CFG:-arx_x5}" --seed "${CHK_SEED:-0}" ${CHK_EXTRA:-} \
      --enable_cameras --headless --kit_args " --enable isaacsim.replicator.behavior --enable isaacsim.sensors.camera" > "$OUT/$TAG.log" 2>&1
  echo "$(date +%T) $TAG rc=$? $(grep -o 'ok=[A-Za-z]*' "$OUT/$TAG.log" | tail -1)" | tee -a $OUT/summary.txt
  # RoboDojo 取帧时自己开的流式视频临时文件(这里不要视频):只删这一次核过的那几格
  # 离线核这一回 RoboDojo 开的结果目录(additional_info = p8check,只有这里用这个后缀;任务不分,bootcal 上量身体自己占多少也开)
  rm -rf "/root/RoboDojo/eval_result/RoboDojo/$T/l3_link/"*"/${CHK_SEED:-0}_p8check"
done
echo "$(date +%T) 路8 跑完:离线核场景 $*" >> /root/q/queue.log
rm -f /root/q/now.txt
