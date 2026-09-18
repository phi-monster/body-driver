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
# 🔴 第三种写法:把【这一段要跟的点】整个清空 ⇒ 后面 `if not Pts.Is_Empty` 直接跳过整段,
#    一推不走而日志全绿。HC 实测连着四段零推(45 推那一段一个"步"都没有),而棘轮当时是绿的 ——
#    它只认前两种写法。闸不一定写成"停",也可以写成"没活儿干"。
stops=$(for f in "$SRC"/*.adb; do strip "$f"; done | grep -cE 'Say_Stop *:=|Ok_Pt *:= *False|Pts\.Clear' || true)
# ② 伪装成测量的门槛:量 × 系数 / 量 ÷ 系数(排除纯数学 0.0/1.0/2.0 的向量运算无从分辨,一律计入)
coef=$(for f in "$SRC"/*.adb "$SRC"/*.ads; do strip "$f"; done | grep -oE '[A-Za-z_.]+ *[*/] *[0-9]+\.[0-9]+' | wc -l | tr -d ' ')
# 🔴 第三条(HZ 2026-09-15 补):把【脑交上来的整段程序】半路扔掉,也是一种闸,而且前两条看不见它。
#    实测:脑写了三行,第二行是"记个名字",没记成 ⇒ `Have_Prog := False; return` 把第三行那句
#    "去球上方"一起扔了 ⇒ 整段一推没走,而 stops 和 coef 两个数都是绿的。
#    只许两处:Y_Finished(程序自己跑完)和 Y_Broken(编译期退回,动之前、免费)。多一处都是闸。
disc=$(for f in "$SRC"/*.adb; do strip "$f"; done | grep -cE 'Have_Prog *:= *False' || true)

# 🔴 第四条(2026-09-18 补):【编译期宣称"物理上做不到"】也是一种闸,而且前三条一个都看不见它。
#    实测:plan.adb 里一条"整块比爪口宽 ⇒ 合下去也是空的"把抓剪刀整段挡在动手之前,
#    而三个棘轮全是绿的 —— 它们只数运行期的停。
#    LAB 09-13 总规矩:驱动只准因为【量过期 / 依赖失效 / 量不出来】拒绝;
#    "我物理上做不到"不算理由(owner:"事实不事实的 vlm 难道看不出来吗")。
phys=$(for f in "$SRC"/*.adb; do strip "$f"; done | grep -cE '张不到|张得开|合下去也是空|够不着|太重|太宽|物理上' || true)

read -r c_stops c_coef c_disc c_phys < <(cat "$CEIL" 2>/dev/null || echo "999 999 999 999")
c_phys=${c_phys:-999}
c_disc=${c_disc:-999}
echo "== 闸门棘轮:身体自己停下 $stops(上限 $c_stops)· 伪装成测量的门槛 $coef(上限 $c_coef) · 半路扔整段 $disc(上限 $c_disc) · 编译期宣称物理做不到 $phys(上限 $c_phys) =="
fail=0
if [ "$stops" -gt "$c_stops" ]; then echo "🔴 身体自己决定不动的地方从 $c_stops 涨到 $stops —— 身体不许有意见,只许有无能"; fail=1; fi
if [ "$coef" -gt "$c_coef" ]; then echo "🔴 伪装成测量的门槛从 $c_coef 涨到 $coef —— 门槛必须说得出它是从哪次测量来的"; fail=1; fi
if [ "$disc" -gt "$c_disc" ]; then echo "🔴 半路扔掉整段程序的地方从 $c_disc 涨到 $disc —— 只许"跑完"和"编译期退回"两处"; fail=1; fi
if [ "$phys" -gt "$c_phys" ]; then echo "🔴 编译期宣称"物理上做不到"的地方从 $c_phys 涨到 $phys —— 只准因为量过期/依赖失效/量不出来而拒绝"; fail=1; fi
if [ "$fail" = 0 ]; then
  echo "$stops $coef $disc $phys" > "$CEIL"
  echo "🟢 没有新增的闸;上限已收紧到 $stops / $coef / $disc / $phys"
fi
exit $fail
