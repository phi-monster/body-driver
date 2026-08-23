#!/usr/bin/env python3
"""RoboDojo 单臂 bug 修补之二(装完就跑,幂等)。

症状:单臂 + 位姿动作 ⇒ ValueError: Cannot infer action type from action dict keys: ['ee_pose', 'ee_joint_state']
根因:同一次 take_action 里先后跑的两个函数互相矛盾 ——
      validate_action_dict(arm_count==1) 明写期望 **ee_pose**(不带前缀),并且**禁止** left_/right_ 前缀;
      其 docstring 也写着 "Single-arm robots use unprefixed keys (e.g. arm_joint_state, ee_pose)"。
      而 get_action_type 找的是 f"{arm_name.split('_')[0]}_ee_pose" —— 单臂 arm_name="arm" ⇒ "arm_ee_pose",
      这个键会被 validate 判成 Unexpected。⇒ 单臂的位姿动作**没有任何键名能同时过这两关**。
      单臂的关节动作没事(arm_joint_state 两边都认),所以上游只发双臂配置时永远碰不到。
修法:按它自己的校验器和 docstring 来 —— 臂名不带下划线(单臂 "arm")时,位姿键就是 "ee_pose"。
      双臂 "left_arm"/"right_arm" 行为完全不变。
用法:cd /root/RoboDojo && python3 <本文件>
"""
import io, sys

P = "src/eval_client/eval_env.py"
OLD = '''                key_name = f"{robot.arm_name.split('_')[0]}_ee_pose"'''
NEW = '''                # 单臂修:validate_action_dict 期望的是不带前缀的 "ee_pose"(且禁止 left_/right_ 前缀),
                # 拼出来的 "arm_ee_pose" 会被它判成 Unexpected ⇒ 单臂位姿动作两关必冲突。
                key_name = (
                    f"{robot.arm_name.split('_')[0]}_ee_pose" if "_" in robot.arm_name else "ee_pose"
                )'''

s = io.open(P, encoding="utf-8").read()
if "单臂修:validate_action_dict 期望的是不带前缀" in s:
    print("已经打过,跳过"); sys.exit(0)
if OLD not in s:
    print("🔴 上游改过这段,别盲改 —— 人工比对"); sys.exit(1)
io.open(P, "w", encoding="utf-8").write(s.replace(OLD, NEW, 1))
print("patched")
