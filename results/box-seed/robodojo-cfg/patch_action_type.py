#!/usr/bin/env python3
"""RoboDojo 单臂 bug 修补之二(装完就跑,幂等)。

症状 A:ValueError: Cannot infer action type from action dict keys: ['ee_pose','ee_joint_state']
症状 B:修完 A 之后走一步就 KeyError: 'arm_ee_pose'

根因:单臂位姿动作的键名,上游三处对不齐 ——
  · validate_action_dict(arm_count==1) 期望不带前缀的 "ee_pose",并且**禁止** left_/right_ 前缀
    (docstring 明写 "Single-arm robots use unprefixed keys (e.g. arm_joint_state, ee_pose)")
  · obs_manager 也为单臂特判成 "ee_pose"
  · 而 get_action_type / 取动作 这两处拼的是 f"{arm_name.split('_')[0]}_ee_pose" ⇒ 单臂得到 "arm_ee_pose"
  ⇒ 单臂的位姿动作没有任何键名能同时过校验器和类型判定;关节动作没事,
    所以上游只发双臂配置时永远碰不到。
修法:按它自己的校验器和 obs_manager 来 —— 臂名不带下划线(单臂 "arm")时位姿键就是 "ee_pose"。
      双臂 "left_arm"/"right_arm" 行为完全不变。
用法:cd /root/RoboDojo && python3 <本文件>
"""
import io, sys

P = "src/eval_client/eval_env.py"
MARK1 = "单臂修:validate_action_dict 期望的是不带前缀"
MARK2 = "单臂修:观测侧 obs_manager 已特判"

OLD1 = '                key_name = f"{robot.arm_name.split(\'_\')[0]}_ee_pose"'
NEW1 = (
    '                # ' + MARK1 + '的 "ee_pose"(且禁止 left_/right_ 前缀),\n'
    '                # 拼出来的 "arm_ee_pose" 会被它判成 Unexpected ⇒ 单臂位姿动作两关必冲突。\n'
    '                key_name = (\n'
    '                    f"{robot.arm_name.split(\'_\')[0]}_ee_pose" if "_" in robot.arm_name else "ee_pose"\n'
    '                )'
)

OLD2 = (
    '                        name = robot.arm_name.split("_")[0]\n'
    '                        key_name = f"{name}_ee_pose"'
)
NEW2 = (
    '                        # ' + MARK2 + '成不带前缀的 "ee_pose",取动作这里必须一致。\n'
    '                        name = robot.arm_name.split("_")[0]\n'
    '                        key_name = f"{name}_ee_pose" if "_" in robot.arm_name else "ee_pose"'
)

s = io.open(P, encoding="utf-8").read()
changed = 0
for mark, old, new, who in ((MARK1, OLD1, NEW1, "get_action_type"), (MARK2, OLD2, NEW2, "取动作")):
    if mark in s:
        continue
    if old not in s:
        print("🔴 上游改过 %s 那段,别盲改 —— 人工比对" % who); sys.exit(1)
    s = s.replace(old, new, 1); changed += 1
io.open(P, "w", encoding="utf-8").write(s)
print("patched %d 处" % changed if changed else "已经打过,跳过")
