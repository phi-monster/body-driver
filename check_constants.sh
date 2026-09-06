#!/usr/bin/env bash
# 🔴🔴 写死的身体常数,机械检查(owner 2026-08-19 定;重写后扫 Ada 树)。
# 判据:代码里每一个带小数点的数字字面量,必须满足三者之一 ——
#   ① 往上 8 行内有一句说明,含以下任一词:无量纲 / 尺度无关 / 协议 / 比例 / 无尺度 / 次数 / dimensionless
#   ② 是 0.0 / 1.0 / 2.0 / 0.5 / 0.25 这类纯数学系数,或在 println/日志/格式化行里
#   ③ 在测试程序(selfcheck.adb)里
# 🔴 棘轮:当前数写在 constants_ceiling.txt,只许降不许升。
set -u
export LC_ALL=C
ROOT="$(cd "$(dirname "$0")" && pwd)"
CEIL_FILE="$ROOT/constants_ceiling.txt"
report=$(
for f in "$ROOT"/driver/src/*.ads "$ROOT"/driver/src/*.adb; do
  [[ "$f" == */selfcheck.adb ]] && continue
  awk -v F="$f" '
    { buf[NR]=$0 }
    {
      line=$0
      sub(/--.*$/, "", line)
      if (line ~ /^[ \t]*$/) next
      if (line ~ /Put_Line|Put \(|Fmt \(|Codec\.Img|Append \(T|Append \(R|Append \(Did|Report :=|Event :=|Cage_Note|Desc :=/) next
      s=line
      while (match(s, /[0-9]+\.[0-9]+(e-?[0-9]+)?/)) {
        v=substr(s, RSTART, RLENGTH)
        pre=substr(s, 1, RSTART-1)
        s=substr(s, RSTART+RLENGTH)
        # 字符串里的数不算
        nq=gsub(/"/, "&", pre)
        if (nq % 2 == 1) continue
        x=v+0
        if (v=="0.0"||v=="1.0"||v=="2.0"||v=="3.0"||v=="4.0"||v=="0.5"||v=="0.25"||v=="0.75"||v=="100.0"||v=="255.0"||v=="256.0"||v=="1.0e-3"||v=="1.0e-4"||v=="1.0e-6"||v=="1.0e-9"||v=="1.0e-12"||v=="1.0e-15"||v=="1.0e-18"||v=="1.0e9"||v=="1.0e6"||v=="1.0e12"||v=="1.0e30"||v=="1.0e-30") continue
        ok=0
        for (i=NR; i>=NR-8 && i>0; i--) if (buf[i] ~ /无量纲|尺度无关|协议|比例|无尺度|次数|dimensionless/) { ok=1; break }
        if (!ok) printf "%s:%d  %s  %s\n", F, NR, v, substr(line,1,90)
      }
    }' "$f"
done)
n=$(printf "%s" "$report" | grep -c . || true)
ceil=$(cat "$CEIL_FILE" 2>/dev/null || echo 99999)
echo "== 写死的常数:$n 处(上限 $ceil)=="
if [ "$n" -gt "$ceil" ]; then echo "$report"; echo "🔴 比上限多了 $((n - ceil)) 处 —— 每一个新写死的数都要么改成量出来的,要么写一句它为什么无量纲。"; exit 1; fi
if [ "$n" -lt "$ceil" ]; then echo "$n" > "$CEIL_FILE"; echo "🟢 降到 $n,上限已收紧(只许降不许升)"; else echo "🟢 持平"; fi
[ "$n" -gt 0 ] && echo "$report"
exit 0
