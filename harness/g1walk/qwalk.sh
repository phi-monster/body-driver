#!/bin/bash
# 第 40 条走路控制器验收(walk_test.py)的外壳:和 /root/q/run.sh 拿同一把锁(一个仿真位),占卡时 now.txt 写明;不起驱动、不起脑、不开相机。
# 用法(箱上):bash qwalk.sh squat,squatwalk,flat 1 --policy vh --dt 0.004 --decim 5 --tag _vh | bash qwalk.sh stairs 0.5 --steps 1 ...
#   (第一个是模式,可以用逗号连着几样;第二个是 flat 走几分钟 / stairs 最多走几分钟;后面的照传给 walk_test.py)
set -u
MODE=${1:-flat}; MIN=${2:-30}; shift 2 2>/dev/null; EXTRA="$*"
TAG=$(echo "$EXTRA" | grep -o -- "--tag [^ ]*" | cut -d" " -f2)
NAME=$(echo "$MODE" | tr , +)
H=$(cd "$(dirname "$0")" && pwd)
mkdir -p /root/p8/chk
echo "$(date +%T) 路8 排队:走路控制器验收 $MODE" >> /root/q/queue.log
exec 9>/root/q/sim.lock
flock 9
echo "$(date +%T) 路8 开跑:走路控制器验收 $MODE" >> /root/q/queue.log
echo "路8 走路验收 $MODE $(date +%s)" > /root/q/now.txt
cd /root/RoboDojo
PYTHONPATH=/root/RoboDojo:/root/RoboDojo/XPolicyLab OMNI_KIT_ACCEPT_EULA=YES timeout -k 20 1800 /venv/RoboDojo/bin/python "$H/walk_test.py" \
  --mode "$MODE" --minutes "$MIN" $EXTRA > "/root/p8/chk/g1walk_$NAME$TAG.log" 2>&1
rc=$?
echo "$(date +%T) 路8 跑完:走路控制器验收 $MODE(rc=$rc)" >> /root/q/queue.log
rm -f /root/q/now.txt
grep "^\[walk\]" "/root/p8/chk/g1walk_$NAME$TAG.log"
