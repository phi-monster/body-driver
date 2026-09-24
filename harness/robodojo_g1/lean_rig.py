# -*- coding: utf-8 -*-
"""人形 G1 机体改法(2026-09-25):让它够得着桌子。

离线用仿真自己的规划器量过(/root/g1_reach_test*.py):G1 肩到腕伸直只有 0.36 m,骨盆架在 0.92 m、歇姿肘 1.2 时手腕已在工作空间边界
(往下/往前/往外 4 cm 三种解法全走不了),桌面在手下面 20 cm 永远够不着。改法 = 像小孩趴桌一样:腰前倾 0.5 rad(URDF 限位 ±0.52)、
骨盆直立放低放近 (0, -0.70, 0.78)、歇姿肘 0(手腕离桌面 6 cm、指尖 5 cm),老鼠活动范围收到手够得着的一条(y ∈ [-0.55, -0.32])。
规划器里腰关节锁在同一个 0.5,基座(骨盆)仍直立 ⇒ 桌子碰撞盒不用转。头顶眼是场景里固定的相机,不受影响。
在箱上跑:python3 lean_rig.py /root/RoboDojo
"""
import sys, re

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"


def patch(path, pairs):
    s = open(path, encoding="utf-8").read()
    for old, new in pairs:
        assert s.count(old) == 1, (path, old[:80], s.count(old))
        s = s.replace(old, new)
    open(path, "w", encoding="utf-8").write(s)
    print("改了", path)


patch(f"{R}/env_cfg/robot/g1.yml", [
    ("default_root_pos: [0.0, -0.75, 0.92]", "default_root_pos: [0.0, -0.70, 0.78]"),
])
patch(f"{R}/env/robot_manager/robot_config/g1.py", [
    ('                ".*_elbow_joint": 1.2,\n', '                ".*_elbow_joint": 0.0,\n                "waist_pitch_joint": 0.5,\n'),
    ("            pos=(0.0, -0.75, 0.92),", "            pos=(0.0, -0.70, 0.78),"),
])
for f in ("curobo_left.yml", "curobo_right.yml", "curobo.yml"):
    patch(f"{R}/Assets/Robots/g1/{f}", [("      waist_pitch_joint: 0.0\n", "      waist_pitch_joint: 0.5\n")])
patch(f"{R}/task/RoboDojo/tasks/chase_mouse.py", [
    ("self._bounds = ((-0.35, 0.35), (-0.45, -0.15))", "self._bounds = ((-0.30, 0.30), (-0.55, -0.32))"),
])
print("完成:腰前倾 0.5 rad · 骨盆 (0,-0.70,0.78) · 歇姿肘 0 · 老鼠 y ∈ [-0.55,-0.32]")
