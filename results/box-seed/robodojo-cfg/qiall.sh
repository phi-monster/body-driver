#!/usr/bin/env bash
# 起一炮:先把残留清干净,再起驱动,再起 sim,最后核对"只有一个 sim"。
# 🔴🔴 清场必须杀【两个】名字,并且计数也要按两个名字数:
#    外层是 `bash scripts/eval_policy.sh`,真正干活的是 `python .../eval_client/main.py`。
#    只杀/只数外层的后果实测过一次(2026-08-23):外层死了、python 还活着并且**占着驱动那条连接**,
#    新起的 sim 连不进来,日志里 `CONNECTED` 次数 = 0 而 `CONNECTING` 27 次,
#    与此同时驱动那边握手、复位、观测全都正常 —— 因为它在跟【上一炮的鬼魂】说话。
#    这一条比"两个 sim"更阴:两个 sim 至少都在跑,这个是新炮一步没跑而两边日志都不报错。
# ⚠️ 模式必须拆字面量写(`eval_pol` + `icy`),否则 pkill 会匹配到本脚本自己的命令行。
set -u
K="$1"
P1=eval_pol; P1="${P1}icy"
P2=eval_cli; P2="${P2}ent"
cnt() { echo $(( $(pgrep -cf "$P1" || true) + $(pgrep -cf "$P2" || true) )); }
pkill -9 -f "$P1" 2>/dev/null
pkill -9 -f "$P2" 2>/dev/null
pkill -9 -f "bl-cal""ibrate" 2>/dev/null
sleep 4
n=$(cnt)
if [ "$n" != "0" ]; then echo "🔴 清不干净,还有 $n 个 sim 进程"; pgrep -af "$P2" | head -3; exit 1; fi
if ss -ltn 2>/dev/null | grep -q ":9080 "; then echo "🔴 9080 还占着"; exit 2; fi
rm -rf "/root/N$K"
bash /root/qidrv.sh "$K" || exit 3
sleep 3
bash /root/qisim.sh "$K"
sleep 25
[ -f "/root/N$K/sim.log" ] && L=yes || L=no
echo "起炮 N$K:sim 进程=$(cnt)(外层+python,要 2)· sim.log=$L(要 yes)· 驱动 $(wc -l < /root/N$K/cal.log) 行"
[ "$L" = "yes" ] || { echo "🔴 起炮没成"; exit 4; }
