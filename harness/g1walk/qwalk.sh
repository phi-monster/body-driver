#!/bin/bash
# 第 40 条走路控制器验收(walk_test.py)的外壳:和 /root/q/run.sh 拿同一把锁(一个仿真位),占卡时 now.txt 写明;不起驱动、不起脑、不开相机。
# 用法(箱上):bash qwalk.sh flat 30 | stairs 3 | squat 2      (模式、仿真分钟数)
set -u
MODE=${1:-flat}; MIN=${2:-30}
H=$(cd "$(dirname "$0")" && pwd)
mkdir -p /root/p8/chk
echo "$(date +%T) 路8 排队:走路控制器验收 $MODE" >> /root/q/queue.log
exec 9>/root/q/sim.lock
flock 9
echo "$(date +%T) 路8 开跑:走路控制器验收 $MODE" >> /root/q/queue.log
echo "路8 走路验收 $MODE $(date +%s)" > /root/q/now.txt
cd /root/RoboDojo
PYTHONPATH=/root/RoboDojo:/root/RoboDojo/XPolicyLab OMNI_KIT_ACCEPT_EULA=YES timeout -k 20 1800 /venv/RoboDojo/bin/python "$H/walk_test.py" \
  --mode "$MODE" --minutes "$MIN" > "/root/p8/chk/g1walk_$MODE.log" 2>&1
rc=$?
echo "$(date +%T) 路8 跑完:走路控制器验收 $MODE(rc=$rc)" >> /root/q/queue.log
rm -f /root/q/now.txt
grep "^\[walk\]" "/root/p8/chk/g1walk_$MODE.log" | tail -1
