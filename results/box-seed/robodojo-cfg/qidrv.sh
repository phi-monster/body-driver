#!/usr/bin/env bash
# 起驱动。
# 🔴 标定跨炮累积:常驻文件 /root/cal_live.json 既读又写。
#    在这台箱子上从零量起(旧箱那份 M0/cal.json 是另一台机器上量的,不认)。
#    驱动每量完一格就落一次盘 ⇒ 集数用完也不白跑,下一炮接着量。
K="$1"
if ss -ltn 2>/dev/null | grep -q ":9080 "; then
  echo "🔴 9080 已被占,先杀掉旧驱动再起"; exit 9
fi
mkdir -p /root/N$K/look /root/N$K/vid
md5sum /root/.local/bin/bl-calibrate | cut -d' ' -f1 > /root/N$K/BUILD.md5
LIVE=/root/cal_live.json
IN=""
[ -s "$LIVE" ] && IN="--in $LIVE"
cd /root/body-layer
# 🔴 钳口那一相加长:真配对产出率实测约 10%,而估计器**每个腕角**要 5 个点(4 个角 ⇒ 20 个)。
# 它是一次性开机自检,量到就存进常驻文件,下次上电不用再量 —— 值得为它多花这一次。
BL_SPAN_LEN=${BL_SPAN_LEN:-3000} BL_DUMP=/root/N$K/look BL_VID=/root/N$K/vid setsid nohup /root/.local/bin/bl-calibrate \
  --listen 9080 --out "$LIVE" $IN --eye 127.0.0.1:8077 \
  </dev/null >/root/N$K/cal.log 2>&1 &
exit 0
