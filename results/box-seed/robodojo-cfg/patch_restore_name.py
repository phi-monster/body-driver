#!/usr/bin/env python3
"""RoboDojo 单臂 bug 修补(装完就跑,幂等)。

症状:单臂配置下 env.reset 抛 `ValueError: No robot found with gripper name: arm`,
      55 集全部记成 Unstable,Success/Fail 都是 0。
根因:robot_manager.restore_name 把 "ee_joint_state" 还原成 "arm"(臂名),
      而两个调用点(control_manager:41 / obs_manager:174)要的是【夹爪名】。
      单臂命名是 arm/ee ⇒ 必炸;双臂命名 left_ee/right_ee 不以 "ee" 开头 ⇒ 侥幸躲过,
      所以上游从没暴露过 —— 他们只发双臂配置。
用法:cd /root/RoboDojo && python3 <本文件>
"""
import io, sys

P = "env/robot_manager/robot_manager.py"
OLD = '''    def restore_name(self, processed_name):
        if processed_name.endswith("_joint_state"):
            name = processed_name[:-12]
            if name.startswith("ee"):
                return "arm" + name[2:]
            else:
                return name
        else:
            return processed_name'''
NEW = '''    def restore_name(self, processed_name):
        # 单臂 bug 修:两个调用点都是拿这个名字去 get_robot_by_gripper_name,要的是夹爪名。
        if processed_name.endswith("_joint_state"):
            return processed_name[:-12]
        return processed_name'''

s = io.open(P, encoding="utf-8").read()
if NEW.splitlines()[1].strip() in s:
    print("已经打过,跳过"); sys.exit(0)
if OLD not in s:
    print("🔴 上游改过这段,别盲改 —— 人工比对"); sys.exit(1)
io.open(P, "w", encoding="utf-8").write(s.replace(OLD, NEW))
print("patched")
