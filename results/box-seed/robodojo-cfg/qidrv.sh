#!/usr/bin/env bash
# 起驱动。不传 --in:旧种子标定是在 Franka 上量的,喂给 X5 是错的先验;
# 从零自校准才是"观众点机体"的真实路径。
K="$1"
mkdir -p /root/N$K/look /root/N$K/vid
md5sum /root/.local/bin/bl-calibrate | cut -d' ' -f1 > /root/N$K/BUILD.md5
cd /root/body-layer
BL_DUMP=/root/N$K/look BL_VID=/root/N$K/vid setsid nohup /root/.local/bin/bl-calibrate \
  --listen 9080 --out /root/N$K/cal.json --eye 127.0.0.1:8077 \
  </dev/null >/root/N$K/cal.log 2>&1 &
exit 0
