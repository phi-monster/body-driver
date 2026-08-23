#!/usr/bin/env bash
# 收炮:把这一炮的逐帧录像编成 mp4 拉回 ~/Downloads,供人亲眼看。
set -e
K="$1"
[ -z "$K" ] && { echo "用法: shoupao.sh <炮号>"; exit 1; }
N=$(ssh vast5 "ls /root/N$K/vid/*.pgm 2>/dev/null | wc -l")
echo "N$K 录到 $N 帧"
[ "$N" -lt 10 ] && { echo "帧太少,不编片"; exit 1; }
ssh vast5 "cd /root/N$K/vid && ffmpeg -y -loglevel error -framerate 12 -pattern_type glob -i '*.pgm' -c:v libx264 -pix_fmt yuv420p -vf scale=640:-2 /root/N$K/N$K.mp4 && ls -lh /root/N$K/N$K.mp4"
mkdir -p "$HOME/Downloads"
scp vast5:/root/N$K/N$K.mp4 "$HOME/Downloads/N$K.mp4"
echo "已拉回 ~/Downloads/N$K.mp4($N 帧 @12fps)"
