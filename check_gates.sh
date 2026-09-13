#!/usr/bin/env bash
# 🔴🔴 闸门棘轮(owner 2026-09-13:"删掉所有阀门,全用语言解决")
# 两条:
#  ① 身体不许有【意见】。能让它自己停下不动的地方,只许是三种"无能",而且必须附"我试过哪些"。
#  ② 任何能影响【动不动】的门槛,不许写成"量出来的东西 × 一个人拍的系数"。
#     check_constants.sh 只查小数点字面量,查不到 `Z.Span * 0.25` 这种伪装 —— 它绿了一整晚,
#     而 act.adb 里躺着 118 个这样的系数,今晚炸的四个闸全在里面。
# 棘轮:两个数写在 gates_ceiling.txt,只许降不许升。
set -u
export LC_ALL=C
ROOT="$(cd "$(dirname "$0")" && pwd)"
SRC="$ROOT/driver/src"
CEIL="$ROOT/gates_ceiling.txt"
strip() { sed -E 's#--.*$##' "$1"; }

# ① 身体自己决定停下的地方
stops=$(for f in "$SRC"/*.adb; do strip "$f"; done | grep -cE 'Say_Stop *:=|Ok_Pt *:= *False' || true)
# ② 伪装成测量的门槛:量 × 系数 / 量 ÷ 系数(排除纯数学 0.0/1.0/2.0 的向量运算无从分辨,一律计入)
coef=$(for f in "$SRC"/*.adb "$SRC"/*.ads; do strip "$f"; done | grep -oE '[A-Za-z_.]+ *[*/] *[0-9]+\.[0-9]+' | wc -l | tr -d ' ')

read -r c_stops c_coef < <(cat "$CEIL" 2>/dev/null || echo "999 999")
echo "== 闸门棘轮:身体自己停下 $stops 处(上限 $c_stops)· 伪装成测量的门槛 $coef 处(上限 $c_coef) =="
fail=0
if [ "$stops" -gt "$c_stops" ]; then echo "🔴 身体自己决定不动的地方从 $c_stops 涨到 $stops —— 身体不许有意见,只许有无能"; fail=1; fi
if [ "$coef" -gt "$c_coef" ]; then echo "🔴 伪装成测量的门槛从 $c_coef 涨到 $coef —— 门槛必须说得出它是从哪次测量来的"; fail=1; fi
if [ "$fail" = 0 ]; then
  echo "$stops $coef" > "$CEIL"
  echo "🟢 没有新增的闸;上限已收紧到 $stops / $coef"
fi
exit $fail
