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
for d in slow/src contact-gen/src contact-exec/src selfcal/src contact-set/src point-gen/src plug/ws/src; do
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
# results/ 里的离线分析脚本是仓规要求的记录件(零 GPU 重生成表格),不是驱动代码。
py=$(find "$ROOT" -name '*.py' -not -path '*/target/*' -not -path '*/results/*' -not -path '*/archive/*' 2>/dev/null)
if [ -n "$py" ]; then echo "🔴 驱动树里有 Python:"; echo "$py"; bad=1; fi

# 🔴🔴🔴 **命令里不许出现写死的机器人字段名。**(owner 2026-08-28 定"减法①")
#
# 观测那一头早就不看名字了 —— `discover.rs` 靠**形状**认(6–7 个 ±2π 的浮点 = 关节角、
# 7 个且后 4 个模长≈1 = 位姿、单个 [0,1] = 夹爪、宽×高×3 的字节 = 相机),认不出就拒绝开跑。
# 而**命令那一头**曾经留了一块:`wire::hold_action` 里写着 `left_arm_joint_state` 等四个名字
# (2026-08-28 已删,当时是死代码、零调用者)。`discover.rs` 文件头对此有明令:
#   「一旦驱动里写着 `left_arm_joint_state`,它就只对报这个名字的机器有效,
#     而『装上就能用』这句话立刻不成立。」
#
# ⇒ 判据(机械、可查):`wire.rs` 里**不许把字符串字面量当成发给机器人的 map 键**。
#   正确写法是把**发现出来的路径**传进来(`joint_action` / `pose_action` 就是这么做的)。
#   这条守住,"换一台机器人要手写一次"的最后一块就回不来了。
# 已知欠账(**棘轮:只许减,不许增**):下面四个是**消息信封**的键,不是机器人的字段名。
# 它们是这条链路的协议本身,而且乱动过一次的代价记在 `wire.rs::reply` 的注释里:
#   多填一个字段 ⇒ 对方 `extra="forbid"` ⇒ **整帧静默丢弃**,两侧都不报错,查了一个多小时。
# ⇒ 推断信封是"减法①"剩下的最后一块,风险高、收益小,先挂账;**新出现的字面量键一律红**。
owed=$(printf 'message_type\nmessage_id\nstep\npayload\n')
keyhit=$(sed 's://.*::' "$ROOT/plug/ws/src/wire.rs" 2>/dev/null | grep -oE 'Value::String\("[a-z_]+"' \
         | sed 's/.*"\(.*\)"/\1/' | sort -u | grep -vxF "$owed" || true)
if [ -n "$keyhit" ]; then
  echo "🔴 plug/ws/src/wire.rs 里出现了【新的】写死字段名(信封那四个已挂账):"
  echo "$keyhit"
  echo "   ⇒ 改成吃 discover 出来的路径(见 joint_action / pose_action)"
  bad=1
fi

if [ "$bad" = 0 ]; then
  echo "🟢 驱动与 ②a:没有 benchmark 名字 · 零 Python · **命令里没有写死的机器人字段名**"
fi
exit $bad
