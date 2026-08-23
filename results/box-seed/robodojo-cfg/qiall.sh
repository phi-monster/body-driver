#!/usr/bin/env bash
# 起一炮:先把残留清干净,再起驱动,再起 sim,最后核对"只有一个 sim"。
# ⚠️ 残留 sim 是本仓踩过两次的坑:上一炮的 sim 没死,新炮的驱动会同时被两个 sim 喂,
#    成绩记到谁账上不确定(N119 实锤)。所以清场是起炮的第一步,不是可选项。
set -u
K="$1"
P=eval_pol; P="${P}icy"
pkill -9 -f "$P" 2>/dev/null
pkill -9 -f "bl-cal""ibrate" 2>/dev/null
sleep 4
n=$(pgrep -cf "$P" || true)
if [ "$n" != "0" ]; then echo "🔴 清不干净,还有 $n 个 sim"; exit 1; fi
if ss -ltn 2>/dev/null | grep -q ":9080 "; then echo "🔴 9080 还占着"; exit 2; fi
rm -rf "/root/N$K"
bash /root/qidrv.sh "$K" || exit 3
sleep 3
bash /root/qisim.sh "$K"
sleep 25
n=$(pgrep -cf "$P" || true)
[ -f "/root/N$K/sim.log" ] && L=yes || L=no
echo "起炮 N$K:sim=$n(要 1)· sim.log=$L(要 yes)· 驱动 $(wc -l < /root/N$K/cal.log) 行"
[ "$n" = "1" ] && [ "$L" = "yes" ] || { echo "🔴 起炮没成"; exit 4; }
