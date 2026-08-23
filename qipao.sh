#!/usr/bin/env bash
# 起炮(owner 2026-08-23 两条死令):
#  ① 不许无改动炮 —— 二进制指纹与上一炮相同就拒起;
#  ② 每炮存视频 —— BL_VID 逐帧落盘,收炮时 shoupao.sh 编片拉回 ~/Downloads,人看完才准起下一炮。
set -e
K="$1"
[ -z "$K" ] && { echo "用法: qipao.sh <炮号,如 107>"; exit 1; }
NEW=$(ssh vast5 'md5sum /root/.local/bin/bl-calibrate | cut -d" " -f1')
PREV=$(ssh vast5 "cat /root/N$((K-1))/BUILD.md5 2>/dev/null || echo none")
if [ "$NEW" = "$PREV" ]; then
  echo "🔴 拒绝起炮 N$K:二进制指纹与上一炮 N$((K-1)) 相同($NEW)= 无改动炮。先带一把刀再来。"
  exit 2
fi
# 上一炮的片子看过没有:没有 SEEN 标记就拒起(人看完才准起下一炮)。
if [ "$PREV" != "none" ] && [ ! -f "$HOME/Downloads/N$((K-1)).SEEN" ]; then
  echo "🔴 拒绝起炮 N$K:上一炮 N$((K-1)) 的视频还没标记看过(缺 ~/Downloads/N$((K-1)).SEEN)。"
  exit 3
fi
# 已有 sim 在跑就拒起(N119 实锤:上一炮的后台起炮任务延迟执行,起了一个 sim
# 写着【上一炮的 sim.log】却连上这一炮的驱动 —— 这一炮真成功了也不会记在自己名下)。
# ⚠️ pgrep -c 数到 0 会返回退出码 1,set -e 会把它当失败 ⇒ 脚本静默退出、什么都没起
# (实测一次:EXIT=1、无输出、N120 目录都没建)。必须 || true 兜住。
RUN=$(ssh vast5 'P=eval_pol; P="${P}icy"; pgrep -cf "$P" || true')
if [ "$RUN" != "0" ]; then
  echo "🔴 拒绝起炮 N$K:箱上还有 $RUN 个 sim 在跑。先清干净再起,免得这一炮的成绩记到别人账上。"
  exit 4
fi
echo "指纹 上一炮 $PREV → 本炮 $NEW(有改动,放行);箱上无残留 sim"
ssh vast5 "mkdir -p /root/N$K/look /root/N$K/vid && echo $NEW > /root/N$K/BUILD.md5 && cd /root/body-layer && BL_DUMP=/root/N$K/look BL_VID=/root/N$K/vid setsid nohup /root/.local/bin/bl-calibrate --listen 9080 --out /root/N$K/cal.json --in /root/M0/cal.json --eye 127.0.0.1:8077 </dev/null >/root/N$K/cal.log 2>&1 & exit 0" || true
sleep 5
# sim 起在箱上写好的脚本里 —— 嵌套引号会把 setsid 那行悄悄吞掉(N107 实测:驱动起了、sim 没起)。
ssh vast5 "bash /root/qisim.sh $K" || true
echo "N$K 起了(指纹 $NEW,录像 /root/N$K/vid)"
