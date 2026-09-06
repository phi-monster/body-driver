#!/usr/bin/env bash
# 🔴🔴🔴 自由棘轮(owner 2026-09-03:"以后只要这些东西进了 driver 就直接停机")
# 驱动里不许有任何【驱动自己决定做什么】的东西。脑说"第几号 → 去哪 → 到什么事件为止",执行器解;完。
# 命中任何一条 ⇒ 退出非零 ⇒ install.sh 失败 ⇒ 驱动装不上。
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
export LC_ALL=C
fail=0
SRC="$ROOT/driver/src"
strip() { sed -E 's#--.*$##' "$1"; }   # 去掉 Ada 行注释(注释里可以记历史,代码里不许有)
# ① 代码里不许出现的【策略/形状规则/动作】标识符:每一个都曾经害过一炮(圆的沿手指捏、两段交接、底沿两块、接触像素…)
for w in 收尾腕 认接触面 两段交接 交接给 退回到下手 竖腕 挪块到格 飞行员 离目标最近 下手点 补抓位 问方向 问格 挪进画面 白转 接触两像素 底沿两块 Contact_Pixel Two_Phase Bottom_Edge Round_Object Long_Object 减半 Halve Grow_Step Disobey 不听话 摘朝向; do
  n=$(for f in "$SRC"/*.ad?; do strip "$f"; done | grep -c -- "$w" || true)
  if [ "$n" != "0" ]; then echo "🔴 驱动代码里有策略词「$w」×$n —— 驱动不许自己决定动作"; fail=1; fi
done
# ② 给脑的提示词里不许有动作例子 / 教程句
for w in "e.g." "how you " "To close" "To lift" "pick it up" "from above" "is how you"; do
  n=$(strip "$SRC/brain.adb" | grep -cF -- "$w" || true)
  if [ "$n" != "0" ]; then echo "🔴 提示词里有教它动作的句子「$w」×$n —— 只许讲格式,不许教怎么做"; fail=1; fi
done
# ③ 运动命令只许从一处发出(selfmap.Go 是唯一的位姿派发;zone/act 的抓握走 Plug.Act 直接发):`Plug.Act (` 的处数是棘轮(只许降)
n=$(for f in "$SRC"/*.adb; do strip "$f"; done | grep -c 'Plug.Act (' || true)
ceil=$(cat "$ROOT/freedom_act_ceiling.txt" 2>/dev/null || echo 999)
if [ "$n" -gt "$ceil" ]; then echo "🔴 发运动命令的地方从 $ceil 处涨到 $n 处 —— 新加的那一处是谁在替模型做决定?"; fail=1; fi
if [ "$n" -lt "$ceil" ]; then echo "$n" > "$ROOT/freedom_act_ceiling.txt"; fi
echo "== 自由棘轮:策略词 0 · 提示词无教程 · 发命令处 $n(上限 $(cat "$ROOT/freedom_act_ceiling.txt"))=="
[ "$fail" = 0 ] && echo "🟢 驱动里没有替模型做决定的东西" || { echo "🔴 停机:驱动装不上,直到删干净"; exit 1; }
