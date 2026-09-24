# -*- coding: utf-8 -*-
"""人形 G1 整体前倾(备选机体改法,2026-09-25):腰关节归零,把整具身体绕自己的 y 轴前倾 θ,靠根位姿实现。

腰前倾 0.4–0.5 rad 时整具上身持续震荡(G1T–G1X:静止时手位 2.4–6.3 mm、姿态 0.008–0.019 rad,腰归零的 G1R 只有 0.6 mm),
腰驱动刚度阻尼提十倍、肘角换掉都没用 ⇒ 让关节全在原位、把整个根转过去。根一转,规划器里那块"桌子"碰撞盒必须跟着转进基座系
(原来按基座直立写死:中心 (0,0,桌高−0.025)、姿态单位),否则 IK 会把桌子当成斜的。
用法(箱上):/venv/RoboDojo/bin/python root_lean.py /root/RoboDojo 0.4
"""
import sys, numpy as np
import transforms3d as t3d

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
theta = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4


def patch(path, pairs, marker=None):
    s = open(path, encoding="utf-8").read()
    if marker and marker in s:
        print("已改过", path); return
    for old, new in pairs:
        assert s.count(old) == 1, (path, old[:80], s.count(old))
        s = s.replace(old, new)
    open(path, "w", encoding="utf-8").write(s)
    print("改了", path)


# 1. 规划器:桌子碰撞盒按根位姿转进基座系
patch(f"{R}/env/planner_manager/curobo_planner.py", [
    ("        table_height=0.74,\n    ):", "        table_height=0.74,\n        table_pose_base=None,   # [bd] 桌子(世界里水平)在基座系里的位姿 [x,y,z,qw,qx,qy,qz];根不直立时用它\n    ):"),
    ("        self.robot_cfg, self.scene_model = self._build_robot_and_scene_cfg(yml_data, table_height)",
     "        self.robot_cfg, self.scene_model = self._build_robot_and_scene_cfg(yml_data, table_height, table_pose_base)"),
    ("    def _build_robot_and_scene_cfg(self, yml_data, table_height):", "    def _build_robot_and_scene_cfg(self, yml_data, table_height, table_pose_base=None):"),
    ('''                    "pose": [
                        0.0,
                        0.0,
                        float(table_height) - 0.025,
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                    ],''', '''                    "pose": ([float(v) for v in table_pose_base] if table_pose_base is not None else [
                        0.0,
                        0.0,
                        float(table_height) - 0.025,
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                    ]),'''),
], marker="table_pose_base")

# 2. robot_manager:根不直立时算出桌子在基座系里的位姿
patch(f"{R}/env/robot_manager/robot_manager.py", [
    ('''                table_height=0.74 - root_pose[2],
            )''', '''                table_height=0.74 - root_pose[2],
                table_pose_base=self._bd_table_in_base(root_pose),
            )'''),
    ('''    @staticmethod
    def _bd_pk(robot):''', '''    @staticmethod
    def _bd_table_in_base(root_pose):
        # [bd] 桌子(世界里水平,顶面 0.74,厚 0.05,中心 (0,0,0.715))在基座系里的位姿;根直立时和原来的写法一样
        import numpy as _np
        import transforms3d as _t3d
        p_root = _np.asarray(root_pose[0:3], dtype=float)
        R_root = _t3d.quaternions.quat2mat(_np.asarray(root_pose[3:7], dtype=float))
        p_b = R_root.T @ (_np.array([0.0, 0.0, 0.74 - 0.025]) - p_root)
        q_b = _t3d.quaternions.mat2quat(R_root.T)
        return [float(v) for v in _np.concatenate([p_b, q_b])]

    @staticmethod
    def _bd_pk(robot):'''),
], marker="_bd_table_in_base")

# 3. 根位姿:偏航 90°(面朝桌子)再绕自身 y 轴前倾 θ;腰归零
q_yaw = np.array([0.70710678, 0.0, 0.0, 0.70710678])
q_pitch = np.array([np.cos(theta / 2), 0.0, np.sin(theta / 2), 0.0])
q = t3d.quaternions.qmult(q_yaw, q_pitch)
q_txt = "[%.5f, %.5f, %.5f, %.5f]" % tuple(q)
import re
p = f"{R}/env_cfg/robot/g1.yml"; s = open(p).read()
s2 = re.sub(r"default_root_rot: \[[^\]]*\]", "default_root_rot: " + q_txt, s)
assert s2 != s or q_txt in s; open(p, "w").write(s2); print("改了", p, q_txt)
p = f"{R}/env/robot_manager/robot_config/g1.py"; s = open(p).read()
s2 = re.sub(r'"waist_pitch_joint": [0-9.\-]+,', '"waist_pitch_joint": 0.0,', s)
s2 = re.sub(r"rot=\([^)]*\),", "rot=(%.5f, %.5f, %.5f, %.5f)," % tuple(q), s2)
open(p, "w").write(s2); print("改了", p)
for f in ("curobo_left.yml", "curobo_right.yml", "curobo.yml"):
    p = f"{R}/Assets/Robots/g1/{f}"; s = open(p).read()
    s2 = re.sub(r"waist_pitch_joint: [0-9.\-]+", "waist_pitch_joint: 0.0", s); open(p, "w").write(s2)
print("完成:整体前倾 %.2f rad,腰归零,规划器桌子盒随根转" % theta)
