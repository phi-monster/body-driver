# -*- coding: utf-8 -*-
"""装第 40 条的场子进 RoboDojo(路 8):会走的人形 g1walk(机器人类、配置、相机 / 环境配置、走路控制器 AGILE velocity_height_g1 的文件)、
三个任务(bd_livingroom 收拾大客厅;远 5 的 bd_stairs 上台阶、bd_floorpick 从地上捡东西)和它们的布局(同一间客厅)。
只加新文件:不在单子里的不写;单子里的文件要是已经在、却不是这里装的(开头没有 body-driver 的记号),也不写。
先跑 make_livingroom.py(家具、台阶的资产),再跑这个:
  bash ../scenes/usdpy.sh make_livingroom.py /root/RoboDojo
  /venv/RoboDojo/bin/python install.py /root/RoboDojo
"""
import json
import os
import re
import shutil
import subprocess
import sys

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
HERE = os.path.dirname(os.path.abspath(__file__))
TASKS = ["bd_livingroom", "bd_stairs", "bd_floorpick"]
FILES = ["Assets/Robots/g1walk/robot_config.yml",
         "env/robot_manager/robot_class/g1walk.py",
         "env/robot_manager/robot_config/g1walk.py",
         "env_cfg/g1walk_rgb.yml",
         "env_cfg/camera/camera_g1walk.yml"] + [f"task/RoboDojo/tasks/{t}.py" for t in TASKS]
MARKS = ("body-driver", "[bd]")


def put(src, rel):
    dst = os.path.join(R, rel)
    if os.path.exists(dst):
        head = open(dst, encoding="utf-8").read(600)
        assert any(m in head for m in MARKS), f"{rel} 已经在、又不是这里装的,不写"
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy(src, dst)
    print("装了", rel)


for rel in FILES:
    put(os.path.join(HERE, "rd", rel), rel)
# 路 8 的运行时(都在 task/RoboDojo/bd/ 下):Walker 和小场景的判据、新身体的登记、随机题的判据(凸包)、收拾客厅的判据、
# 走路控制器、会走的人形几个任务共用的那一套、上台阶的判据
for src, rel in ((f"{HERE}/../scenes/rd/bd/scene.py", "task/RoboDojo/bd/scene.py"), (f"{HERE}/../scenes/rd/bd/rig.py", "task/RoboDojo/bd/rig.py"),
                 (f"{HERE}/../qexam/rd/bd/question.py", "task/RoboDojo/bd/question.py"), (f"{HERE}/rd/bd/tidy.py", "task/RoboDojo/bd/tidy.py"),
                 (f"{HERE}/rd/bd/agile_vh.py", "task/RoboDojo/bd/agile_vh.py"), (f"{HERE}/rd/bd/walkrig.py", "task/RoboDojo/bd/walkrig.py"),
                 (f"{HERE}/rd/bd/walkjudge.py", "task/RoboDojo/bd/walkjudge.py")):
    os.makedirs(os.path.join(R, "task/RoboDojo/bd"), exist_ok=True)
    shutil.copy(src, os.path.join(R, rel))
    print("装了", rel)
# 走路控制器的文件(AGILE velocity_height_g1):取下来、核过 sha256 才放(fetch_agile.py),放在这具身体的资产目录里
sys.path.insert(0, HERE)
import fetch_agile  # noqa: E402
fetch_agile.fetch(os.path.join(R, "Assets/Robots/g1walk/agile_velocity_height_g1"))
for t in TASKS:
    open(os.path.join(R, f"task/RoboDojo/config/{t}.yml"), "w").write("# body-driver 第 40 条 / 远 5(harness/g1walk/install.py 写出):布局直接给。\n{}\n")
    print(f"装了 task/RoboDojo/config/{t}.yml")
# 机器人配置:骨盆离地 0.75 m(Isaac Lab 的 G1_29DOF_CFG 开局高);地面高和布局(make_livingroom_layout.py)、运行时(walkrig.py)里的是同一个数
LAYOUT_SRC = open(os.path.join(HERE, "make_livingroom_layout.py"), encoding="utf-8").read()
FLOOR_Z = float(re.search(r"^FLOOR_Z = ([0-9.]+)", LAYOUT_SRC, re.M).group(1))
rt = open(os.path.join(R, "task/RoboDojo/bd/walkrig.py"), encoding="utf-8").read()
assert "FLOOR_Z = %.2f" % FLOOR_Z in rt, "运行时写的地面高和布局算的不一样(布局:%.3f)" % FLOOR_Z
# 先写布局:人形开局站哪儿是布局按餐桌的包围盒算的(写在第一件东西的 bd_tidy.robot_start 里),身体配置照它写
subprocess.run([sys.executable, os.path.join(HERE, "make_livingroom_layout.py"), R, "--cfg_name", "g1walk"], check=True)
_lay = json.load(open(os.path.join(R, "Assets/Eval_Layout/RoboDojo/g1walk/0/bd_livingroom_0.json")))
START = [r["bd_tidy"]["robot_start"] for recs in _lay["Rigid"].values() for r in recs if "bd_tidy" in r][0]
txt = open(os.path.join(HERE, "rd/env_cfg/robot/g1walk.yml")).read()
assert "[0.0, -0.6, 0.80]" in txt
txt = txt.replace("[0.0, -0.6, 0.80]", "[%.4f, %.4f, %.4f]" % (START[0], START[1], FLOOR_Z + 0.75))
open(os.path.join(R, "env_cfg/robot/g1walk.yml"), "w").write(txt)
print("装了 env_cfg/robot/g1walk.yml(地面 z = %.3f;人形开局骨盆在 (%.3f, %.3f, %.3f))" % (FLOOR_Z, START[0], START[1], FLOOR_Z + 0.75))
