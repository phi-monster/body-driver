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
[ "$bad" = 0 ] && echo "🟢 驱动:没有 benchmark 名字 · 零 Python · 命令里没有写死的机器人字段名"
exit $bad
