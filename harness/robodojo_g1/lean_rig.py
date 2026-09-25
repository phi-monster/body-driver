# -*- coding: utf-8 -*-
"""人形 G1 机体改法(2026-09-24):让它够得着桌子。

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
    # 腿折起来(膝 1.5、髋 -0.4):骨盆放到 0.78 后垂着的脚踩进地面,整具身体抖(G1T 2026-09-24:静止时手位 6 mm、灰度地板 237,是 G1R 的十倍)
    ('                ".*_elbow_joint": 1.2,\n',
     '                ".*_elbow_joint": 0.0,\n                "waist_pitch_joint": 0.5,\n                ".*_knee_joint": 1.5,\n                ".*_hip_pitch_joint": -0.4,\n'),
    ("            pos=(0.0, -0.75, 0.92),", "            pos=(0.0, -0.70, 0.78),"),
])
# 腕相机往桌面下倾 35°:歇姿前臂放平后相机顺着手指看的是墙和窗(G1T 实拍),看不见桌子;相机前 = 手腕连杆 +x(手指),绕连杆 y 轴转 35° ⇒ 欧拉 [0,-55,-90]
import yaml
p = f"{R}/Assets/Robots/g1/robot_config.yml"
cfg = yaml.safe_load(open(p))
for sd in ("left", "right"):
    cfg["sides"][sd]["camera"][0]["ori"] = [0, -55, -90]
open(p, "w").write("# Unitree G1 (29 DoF) with Inspire 5-finger hands: one articulation, two arm chains. body-driver humanoid test, 2026-09-23.\n"
                   "# 2026-09-24: wrist cameras tilted 35 deg toward the table (ori [0,-55,-90]) for the level-forearm rest posture on the leaned rig.\n"
                   + yaml.safe_dump(cfg, sort_keys=False))
print("改了", p)
for f in ("curobo_left.yml", "curobo_right.yml", "curobo.yml"):
    patch(f"{R}/Assets/Robots/g1/{f}", [("      waist_pitch_joint: 0.0\n", "      waist_pitch_joint: 0.5\n")])
patch(f"{R}/task/RoboDojo/tasks/chase_mouse.py", [
    ("self._bounds = ((-0.35, 0.35), (-0.45, -0.15))", "self._bounds = ((-0.30, 0.30), (-0.55, -0.32))"),
])
print("完成:腰前倾 0.5 rad · 骨盆 (0,-0.70,0.78) · 歇姿肘 0 · 腿折起 · 腕相机下倾 35° · 老鼠 y ∈ [-0.55,-0.32]")
