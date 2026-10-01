#!/bin/bash
# 路 8:11 个小场景每个开一炮(主线驱动、装回 x5 的身体文件),看到第 1 轮就放锁(白桌白墙从零开机是路 2 第 6 条自己的炮,这里不占排队位)。
# 每一炮都自己走 /root/q/run.sh 排队,炮和炮之间别的路可以插进来。结果在 /root/p8/boot/<炮名>/。
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
bash "$HERE/qboot.sh" P8A bd_drawer
bash "$HERE/qboot.sh" P8B bd_lidbox
bash "$HERE/qboot.sh" P8C bd_hinge
bash "$HERE/qboot.sh" P8D bd_peg
bash "$HERE/qboot.sh" P8E bd_hook
bash "$HERE/qboot.sh" P8F bd_knob
bash "$HERE/qboot.sh" P8G bd_trigger
bash "$HERE/qboot.sh" P8H bd_glass
bash "$HERE/qboot.sh" P8I bd_white
bash "$HERE/qboot.sh" P8J bd_walker
bash "$HERE/qboot.sh" P8K bd_cloth
echo "全部开完 $(date +%T)"
