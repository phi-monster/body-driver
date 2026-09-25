#!/usr/bin/env bash
# 🔴 owner 2026-08-13:driver 是要进入万千世界的,不许照着某个 benchmark 写。代码里不许出现 benchmark / 任务 / 场景名;驱动树里零 Python;
# 命令里不许写死机器人字段名(键名全部来自 discover 认出来的路径;下面列出的是线缆协议的信封键,不是机器人字段)。
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
PAT='robodojo|isaac|libero|calvin|general_pickup|stack_blocks|pack_objects|store_tools|eval_result|_result\.json|franka|arx|x5_grasp'
bad=0
for f in "$ROOT"/driver/src/*.ads "$ROOT"/driver/src/*.adb; do
  hit=$(sed 's/--.*$//' "$f" | grep -niE "$PAT" || true)
  if [ -n "$hit" ]; then echo "🔴 $f"; echo "$hit"; bad=1; fi
done
py=$(find "$ROOT/driver" -name '*.py' 2>/dev/null)
if [ -n "$py" ]; then echo "🔴 驱动树里有 Python:"; echo "$py"; bad=1; fi
owed=$(printf 'message_type\nmessage_id\nstep\npayload\nevaluation_id\naction_case_id\ntrial_id\nrepeat_index\nsent_at\nresult\nok\nserver\nserver_instance_id\nfunc_name\nobs\nobservation\ninstruction\nnd\ntype\nshape\ndata\nhello\nprepare_case\nreset\ncall\ninfer\ntrial_end\nheartbeat\nhello_ack\nprepare_case_ack\nreset_result\ncall_result\ninfer_result\ntrial_end_ack\nheartbeat_ack\nxpolicylab_policy_server\nbody-driver\nget_action\n_\n_c\n')
keyhit=$(sed 's/--.*$//' "$ROOT/driver/src/plug.adb" "$ROOT/driver/src/msgpack.adb" "$ROOT/driver/src/layout.adb" 2>/dev/null | grep -oE '"[a-z_]+"' | tr -d '"' | sort -u | grep -vxF "$owed" || true)
if [ -n "$keyhit" ]; then echo "🔴 插头里出现了协议信封之外的写死字段名(机器人字段必须来自 discover):"; echo "$keyhit"; bad=1; fi
# 🔴 owner 2026-09-26:每个量只有一种量法,不许"一个东西几种办法、a 不行再 b";身体另外报的读数(内参、深度)驱动不读。
# 下面这几条是已经删掉的第二种量法,谁把它们加回来,装机就过不去(去注释后查)。
second=""
chk() {   # $1 文件  $2 模式  $3 说明
  if sed 's/--.*$//' "$ROOT/driver/src/$1" | grep -qE "$2"; then second="$second\n  $1:$3"; fi
}
chk plug.adb 'Has_K[[:space:]]*:=[[:space:]]*True' '读了身体给的相机内参(焦距要自己量)'
chk plug.adb 'Has_Depth[[:space:]]*:=[[:space:]]*True' '读了身体给的深度图(远近要自己量)'
chk act.adb 'Picture\.Measure_In_Box' '抠物体又退回按明暗切(只许仪器那一种)'
chk act.adb 'Geom\.Triangulate[[:space:]]*\(G,[[:space:]]*Use_Obs' '东西在哪又退回"我自己挪过的那几眼"(只许两眼同一刻的交点)'
chk act.adb 'Geo_Track[[:space:]]*\(C,[[:space:]]*F,[[:space:]]*Cam,[[:space:]]*Cand' '标定跟点又退回按槽号重切(只许跟点仪器)'
chk act.adb 'Hit_Plane[[:space:]]*\(Geom\.Cam_Pos[[:space:]]*\(G,[[:space:]]*Hp\)' '东西在哪又退回"一条视线落到它躺的面上"(只许两眼同一刻的交点)'
if [ -n "$second" ]; then echo -e "🔴 同一个量出现了第二种量法:$second"; bad=1; fi
[ "$bad" = 0 ] && echo "🟢 驱动:没有 benchmark 名字 · 零 Python · 命令里没有写死的机器人字段名 · 每个量一种量法(六条已删的退路没回来)"
exit $bad
