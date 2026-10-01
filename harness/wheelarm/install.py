# -*- coding: utf-8 -*-
"""装第 39 条的场子进 RoboDojo(路 8):轮子底盘 + 胳膊这具身体(robot_class / robot_config / 机器人配置 / 相机配置 / 资产)、
任务 bd_mouse_floor(地上乱跑、会躲的老鼠)和它的布局。只加新文件:每个要写的路径先查 —— 不在下面这张单子里的不写;
单子里的文件要是已经在、却不是这里装的(第一行没有 body-driver 的记号),也不写。

箱上跑:/venv/RoboDojo/bin/python install.py /root/RoboDojo
(机器人的 USD 由 make_wheelarm.py 单独写:bash ../scenes/usdpy.sh make_wheelarm.py /root/RoboDojo)
"""
import json
import os
import shutil
import sys

import numpy as np

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
HERE = os.path.dirname(os.path.abspath(__file__))
FILES = ["Assets/Robots/wheelarm/robot_config.yml",
         "env/robot_manager/robot_class/wheelarm.py",
         "env/robot_manager/robot_config/wheelarm.py",
         "env_cfg/wheelarm_rgb.yml",
         "env_cfg/camera/camera_wheelarm.yml",
         "task/RoboDojo/tasks/bd_mouse_floor.py"]
MARKS = ("body-driver", "[bd]")
for rel in FILES:
    dst = os.path.join(R, rel)
    if os.path.exists(dst):
        head = open(dst, encoding="utf-8").read(400)
        assert any(m in head for m in MARKS), f"{rel} 已经在、又不是这里装的,不写"
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy(os.path.join(HERE, "rd", rel), dst)
    print("装了", rel)
# 路 8 的运行时(task/RoboDojo/bd/):Walker(老鼠)、新身体的登记(rig.py)
for src, rel in ((f"{HERE}/../scenes/rd/bd/scene.py", "task/RoboDojo/bd/scene.py"), (f"{HERE}/../scenes/rd/bd/rig.py", "task/RoboDojo/bd/rig.py")):
    os.makedirs(os.path.join(R, "task/RoboDojo/bd"), exist_ok=True)
    shutil.copy(src, os.path.join(R, rel))
    print("装了", rel)
cfg = os.path.join(R, "task/RoboDojo/config/bd_mouse_floor.yml")
open(cfg, "w").write("# body-driver 第 39 条(harness/wheelarm/install.py 写出):布局直接给,这里不列东西。\n{}\n")
print("装了 task/RoboDojo/config/bd_mouse_floor.yml")

# ---------------------------------------------------------------- 布局(种子 0,三张):房间、地、背景照 RoboDojo 默认;桌子挪到房间另一头当家具
base = json.load(open(os.path.join(R, "Assets/Eval_Layout/RoboDojo/drone/1/bootcal_0.json")))
LAY = os.path.join(R, "Assets/Eval_Layout/RoboDojo/wheelarm/0")
os.makedirs(LAY, exist_ok=True)
# 地面高:RoboDojo 的 Ground 是一块 cube,中心摆在 default_pos 的 z − 半厚(scene_manager/objects/ground.py)⇒ 顶面就在 default_pos 的 z;
# 这里把它摆到 0.05 m,比默认房间 Simple_Room 自己的地高一点(第 39 条离线核实测:东西落在 0.0475 上),地面就是这一块的顶,高度说得清。
# (第一版按"中心 + 半厚"算出 0.05,算法是错的,碰巧和房间的地差 2.5 mm;大客厅没放房间,东西其实落在 Ground 的顶 0 上、边上的掉出了 7 m 的地)
FLOOR_Z = 0.05
MOUSE_Z = FLOOR_Z + (0.7818 - 0.765)   # chase_mouse 的布局里老鼠的中心比桌面高这么多(它自己的半高);放在地上一样高出地面这么多
BASE_Z = FLOOR_Z + 0.08                # 底盘中心离地 8 cm(make_wheelarm.py:离地 3 cm + 半高 5 cm)
rob = os.path.join(R, "env_cfg/robot/wheelarm.yml")
txt = open(os.path.join(HERE, "rd/env_cfg/robot/wheelarm.yml")).read().replace("[0.0, -1.0, 0.08]", "[0.0, -1.0, %.4f]" % BASE_Z)
open(rob, "w").write(txt)
print("装了 env_cfg/robot/wheelarm.yml(地面 z = %.3f,底盘根 z = %.3f)" % (FLOOR_Z, BASE_Z))
rng = np.random.default_rng(39)
for k in range(3):
    lay = {key: json.loads(json.dumps(base[key])) for key in ("Room", "Ground", "Background")}
    lay["Ground"]["default_pos"] = [0.0, 0.0, FLOOR_Z]
    lay["Table"] = dict(base["Table"], default_pos=[0.0, 1.3, base["Table"]["default_pos"][2]])
    x, y = float(rng.uniform(-0.4, 0.4)), float(rng.uniform(-0.5, -0.1))
    h = float(rng.uniform(0, 2 * np.pi))
    lay["Rigid"] = {"mouse": [{
        "category": "mouse", "category_idx": 8, "label": "target", "default_pos": [x, y, MOUSE_Z],
        "default_ori": [float(np.cos(h / 2)), 0.0, 0.0, float(np.sin(h / 2))], "scale": [1.0, 1.0, 1.0],
        "physics": {"mass": 0.08, "friction": 0.45, "type": "rigid"}, "visual": {}, "relative_plane": "Ground",
        # RoboDojo 核"布局稳不稳"只认桌上的东西:低过桌面 5 cm 一律判站不住(layout_manager.check_layout_stability 的 is_stable),
        # 地上的老鼠一开局就被判掉 ⇒ 这一件不让它核;稳不稳离线核自己量(落稳后挪了多远、歪了多少)
        "need_check_stable": False, "margin": 0.01, "check_mode": "bbox",
        # 区域:底盘前面那一片地(底盘在 y = -1.0、朝 +y;桌子挪到 y = 1.3 以后,它近的那条边在 y = 0.75)
        "bd_walk": {"speed": 0.01, "turn_every": 40, "region": [[-1.2, 1.2], [-0.7, 0.6]], "free_height": 0.005,
                    "seed": int(rng.integers(1 << 30)), "yaw0": 0.0, "flee_radius": 0.25}}]}
    p = os.path.join(LAY, f"bd_mouse_floor_{k}.json")
    json.dump(lay, open(p, "w"), indent=1)
    print("布局", p, "老鼠开局 (%.2f, %.2f)" % (x, y))
