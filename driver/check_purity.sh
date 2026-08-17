#!/usr/bin/env bash
# 🔴 owner 2026-08-13:**driver 是要进入万千世界的,不许照着某个 benchmark 写。**
# 这是那条规矩的**机械检查** —— 一条文档规矩只有写成会响的检查才算数。
#
# 允许:注释里把某个 benchmark 当**例子**引(“RoboDojo 和 CALVIN 收末端位姿,别的收关节角”)。
# 禁止:**代码里**出现任何 benchmark / 任务 / 场景的名字 —— 那意味着这具身体的行为取决于它在哪个榜上。
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
PAT='robodojo|isaac|libero|calvin|general_pickup|stack_blocks|pack_objects|store_tools|eval_result|_result\.json'
bad=0
for d in core/src contact-gen/src contact-exec/src selfcal/src; do
  [ -d "$ROOT/$d" ] || continue
  # 去掉整行注释(// 和 ///)再查 —— 剩下的就是真代码
  while IFS= read -r f; do
    hit=$(sed 's://.*::' "$f" | grep -niE "$PAT" || true)
    if [ -n "$hit" ]; then echo "🔴 $f"; echo "$hit"; bad=1; fi
  done < <(find "$ROOT/$d" -name '*.rs')
done
# 🔴 owner 2026-08-13:**驱动树里一行 Python 都不许有。**
# `bl` 把驱动变成一个进程(一行一问一行一答)之后,那层 676 行的 ctypes 壳就没有存在理由了 ——
# 而它一直躺在 `bind/python/` 里没删。壳住在驱动树里 = "驱动"这个词同时指一份 Rust 和一份跟着它漂的 Python。
py=$(find "$ROOT" -name '*.py' -not -path '*/target/*' 2>/dev/null)
if [ -n "$py" ]; then echo "🔴 驱动树里有 Python:"; echo "$py"; bad=1; fi

if [ "$bad" = 0 ]; then
  echo "🟢 驱动与 ②a:代码里没有 benchmark 名字(注释引例不算),树里零 Python"
fi
exit $bad
