# -*- coding: utf-8 -*-
"""人形 G1 机体定稿(2026-09-24):直立、放低、贴桌。

前倾(不管腰关节前倾还是整体根位姿前倾,G1T–G1Z)都让整具身体持续震荡(静止时手位 2–6 mm/帧),直立时为零(G1Y)。
直立时够不着桌子的原因是肩太高,所以把骨盆放到桌面以下:(0, -0.72, 0.70),骨盆前沿离桌沿 4 cm。肩高 0.99 m,
桌面高度处往前能到 y ≈ -0.42(手腕)/ -0.32(指尖),和前倾方案一样远。
骨盆低于桌面后,规划器里那块 3 m × 3 m 的"桌子"碰撞板会把躯干裹在里面 ⇒ 改成真桌子的盒子(布局里 Table:1.4 × 1.1 × 0.05,中心 (0,-0.05,0.715)),
转进基座系。腿折起(膝 1.5、髋 -0.4)、腕相机下倾 35°(ori [0,-55,-90])、腰/腿驱动 20000/1000。
歇姿 肩 -0.4 肘 0.1 腕俯仰 +0.3:离线正解(/root/g1_camview_test.py)手腕 (∓0.18,-0.45,0.864)、指尖高 0.863(离桌面 12 cm),
腕相机光轴俯 34°、打在桌面 y = -0.15(东西堆在 y ∈ [-0.2, 0.05]);往下/前/左右 8 cm、往上 16 cm 都够得着。
(G2A 用的腕俯仰 -0.3:手往上翘 33°,相机光轴水平,腕眼画面中间是自己的手和墙。)
顺序(新箱):g1_setup.py → patch_planner_key.py → lean_rig.py → root_lean.py → 本脚本;本脚本把最终值全写一遍,可重复跑。
用法(箱上):/venv/RoboDojo/bin/python rig_upright_low.py /root/RoboDojo
"""
import re, sys

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"


def patch(path, pairs):
    s = open(path, encoding="utf-8").read()
    for old, new in pairs:
        if new in s:
            continue   # 改过了(可重复跑)
        assert s.count(old) == 1, (path, old[:70], s.count(old))
        s = s.replace(old, new)
    open(path, "w", encoding="utf-8").write(s)
    print("改了", path)


# 规划器:真桌子的盒子(不是 3 m 板),中心按根位姿转进基座系
patch(f"{R}/env/planner_manager/curobo_planner.py", [
    ('                    "dims": [3.0, 3.0, 0.05],', '                    "dims": ([1.4, 1.1, 0.05] if table_pose_base is not None else [3.0, 3.0, 0.05]),   # [bd] 给了位姿就是真桌子的盒子'),
])
patch(f"{R}/env/robot_manager/robot_manager.py", [
    ("        p_b = R_root.T @ (_np.array([0.0, 0.0, 0.74 - 0.025]) - p_root)", "        p_b = R_root.T @ (_np.array([0.0, -0.05, 0.74 - 0.025]) - p_root)   # 布局里 Table 中心 (0,-0.05),厚 0.05"),
])
# 根位姿:直立(只偏航 90°)、放低放近
p = f"{R}/env_cfg/robot/g1.yml"; s = open(p).read()
s = re.sub(r"default_root_pos: \[[^\]]*\]", "default_root_pos: [0.0, -0.72, 0.70]", s)
s = re.sub(r"default_root_rot: \[[^\]]*\]", "default_root_rot: [0.707, 0, 0, 0.707]", s)
open(p, "w").write(s); print("改了", p)
p = f"{R}/env/robot_manager/robot_config/g1.py"; s = open(p).read()
s = re.sub(r"pos=\([^)]*\),", "pos=(0.0, -0.72, 0.70),", s)
s = re.sub(r"rot=\([^)]*\),", "rot=(0.707, 0.0, 0.0, 0.707),", s)
s = re.sub(r'"waist_pitch_joint": [0-9.\-]+,', '"waist_pitch_joint": 0.0,', s)
s = re.sub(r'"\.\*_elbow_joint": [0-9.\-]+,', '".*_elbow_joint": 0.1,', s)
if '"_wrist_pitch_joint"' not in s and '".*_wrist_pitch_joint"' not in s:
    s = s.replace('".*_elbow_joint": 0.1,', '".*_elbow_joint": 0.1,\n                ".*_wrist_pitch_joint": 0.3,')
s = re.sub(r'"\.\*_wrist_pitch_joint": [0-9.\-]+,', '".*_wrist_pitch_joint": 0.3,', s)
s = re.sub(r"(joint_names_expr=\[\"\.\*_hip_\.\*\", \"\.\*_knee_joint\", \"\.\*_ankle_\.\*\", \"waist_\.\*\"\],\n\s*effort_limit_sim=300\.0, velocity_limit_sim=10\.0, )stiffness=[0-9.]+, damping=[0-9.]+,",
           r"\1stiffness=20000.0, damping=1000.0,", s)
open(p, "w").write(s); print("改了", p)
for f in ("curobo_left.yml", "curobo_right.yml", "curobo.yml"):
    p = f"{R}/Assets/Robots/g1/{f}"; s = open(p).read()
    open(p, "w").write(re.sub(r"waist_pitch_joint: [0-9.\-]+", "waist_pitch_joint: 0.0", s))
print("完成:直立 · 骨盆 (0,-0.72,0.70) · 腰 0 · 歇姿 肩 -0.4 肘 0.1 腕 +0.3 · 腰腿 20000/1000 · 规划器用真桌子盒")
