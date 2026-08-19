#!/usr/bin/env bash
# 🔴🔴 **写死的身体常数,机械检查。**(owner 2026-08-19 定)
#
# 为什么要有它:仓规写着「驱动里不许有手填的身体常数」,而**这条规矩此前没有任何检查**——
# `check_purity.sh` 只查 benchmark 名字和 Python。于是它悄悄失效了,代价是可看得见的:
#   · 腕姿写死成某一具身体的"腕朝下" ⇒ 换 Franka 大步子只交付 8%、一轮 15 格只量到 1 格
#   · `got/cmd < 0.3` 写死 ⇒ Franka 自由交付本来就是 0.105 ⇒ "碰到了"这道闸永远为真
#   · 工具长 `0.1451` 与量出来的 `tool_offset` 同时存在于一个文件里(差 4.3 cm)
#
# 判据:代码里每一个**带物理单位**的数字字面量,必须满足三者之一 ——
#   ① 在 `#[cfg(test)]` 里(测试的假身体不是断言)
#   ② 往上 8 行内有一句说明,含以下任一词:无量纲 / 尺度无关 / 协议 / 比例 / 无尺度 / dimensionless
#   ③ 已经在 `debt.rs` 上挂号(那是"欠着但看得见")
#
# 🔴 **棘轮**:当前数写在 `constants_ceiling.txt` 里,**只许降不许升**。
#   这样既不阻塞今天,又保证明天不会更差 —— 一条"以后再清"的规矩等于没有规矩。
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
CEIL_FILE="$ROOT/constants_ceiling.txt"
# 🔴 插头也是驱动 —— 「驱动里不能含有任何机体参数」这条对它同样成立(owner 2026-08-19)。
DIRS="slow/src contact-set/src contact-gen/src contact-exec/src point-gen/src selfcal/src plug/ws/src"

report=$(
for d in $DIRS; do
  [ -d "$ROOT/$d" ] || continue
  while IFS= read -r f; do
    awk -v F="$f" '
      /#\[cfg\(test\)\]/ { intest=1 }
      { buf[NR]=$0 }
      {
        if (intest) next
        line=$0
        sub(/\/\/.*$/, "", line)                       # 去掉行尾注释
        if (line ~ /^[ \t]*$/) next
        # 打印/格式化行里的数字是展示换算(mm 之类),不驱动任何动作。
        if (line ~ /println!|format!|eprintln!/) next
        # 找带小数点的字面量
        s=line
        while (match(s, /[0-9]+\.[0-9]+/)) {
          v=substr(s, RSTART, RLENGTH)
          s=substr(s, RSTART+RLENGTH)
          x=v+0
          if (x<0.0001) continue
          if (v=="0.0"||v=="1.0"||v=="2.0"||v=="3.0"||v=="4.0"||v=="0.5"||v=="100.0"||v=="180.0"||v=="360.0") continue
          # 字符串里的数(debt.rs 的登记内容)不算
          # 引号里的数字是【被记录的】,不是【被执行的】:`debt.rs` 整本账、HTTP/1.1 之类
          # 全落在这里。判据:这个数左边的双引号个数是奇数 ⇒ 它在字符串里面。
          pre=substr(line, 1, index(line, v)-1)
          nq=gsub(/"/, "&", pre)
          if (nq % 2 == 1) continue
          ok=0
          for (i=NR; i>=NR-8 && i>0; i--) {
            if (buf[i] ~ /无量纲|尺度无关|协议|比例|无尺度|dimensionless/) { ok=1; break }
          }
          if (!ok) printf "%s:%d  %s  %s\n", F, NR, v, substr(line,1,90)
        }
      }
    ' "$f"
  done < <(find "$ROOT/$d" -name '*.rs')
done
)
n=$(printf "%s" "$report" | grep -c . || true)
ceil=$(cat "$CEIL_FILE" 2>/dev/null || echo 99999)

echo "== 写死的身体常数:$n 处(上限 $ceil)=="
if [ "$n" -gt "$ceil" ]; then
  echo "$report"
  echo "🔴 比上限多了 $((n - ceil)) 处 —— 每一个新写死的数都要么改成量出来的,要么写一句它为什么尺度无关。"
  exit 1
fi
if [ "$n" -lt "$ceil" ]; then
  echo "$n" > "$CEIL_FILE"
  echo "🟢 降到 $n,上限已收紧(只许降不许升)"
else
  echo "🟢 持平"
fi
exit 0
