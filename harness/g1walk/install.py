# -*- coding: utf-8 -*-
"""装第 40 条的场子进 RoboDojo(路 8):会走的人形 g1walk(机器人类、配置、相机 / 环境配置)、任务 bd_livingroom、大客厅的布局。
只加新文件:不在单子里的不写;单子里的文件要是已经在、却不是这里装的(开头没有 body-driver 的记号),也不写。
先跑 make_livingroom.py(家具资产),再跑这个:
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
FILES = ["Assets/Robots/g1walk/robot_config.yml",
         "env/robot_manager/robot_class/g1walk.py",
         "env/robot_manager/robot_config/g1walk.py",
         "env_cfg/g1walk_rgb.yml",
         "env_cfg/camera/camera_g1walk.yml",
         "task/RoboDojo/tasks/bd_livingroom.py"]
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
# 路 8 的运行时(都在 task/RoboDojo/bd/ 下):Walker 和小场景的判据、新身体的登记、随机题的判据(凸包)、收拾客厅的判据
for src, rel in ((f"{HERE}/../scenes/rd/bd/scene.py", "task/RoboDojo/bd/scene.py"), (f"{HERE}/../scenes/rd/bd/rig.py", "task/RoboDojo/bd/rig.py"),
                 (f"{HERE}/../qexam/rd/bd/question.py", "task/RoboDojo/bd/question.py"), (f"{HERE}/rd/bd/tidy.py", "task/RoboDojo/bd/tidy.py")):
    os.makedirs(os.path.join(R, "task/RoboDojo/bd"), exist_ok=True)
    shutil.copy(src, os.path.join(R, rel))
    print("装了", rel)
open(os.path.join(R, "task/RoboDojo/config/bd_livingroom.yml"), "w").write("# body-driver 第 40 条(harness/g1walk/install.py 写出):布局直接给。\n{}\n")
print("装了 task/RoboDojo/config/bd_livingroom.yml")
# 机器人配置:骨盆离地 0.75 m(Isaac Lab 的 G1_29DOF_CFG 开局高);地面高和布局(make_livingroom_layout.py)、任务里的是同一个数
FLOOR_Z = float(re.search(r"^FLOOR_Z = ([0-9.]+)", open(os.path.join(HERE, "make_livingroom_layout.py"), encoding="utf-8").read(), re.M).group(1))
task = open(os.path.join(R, "task/RoboDojo/tasks/bd_livingroom.py"), encoding="utf-8").read()
assert "FLOOR_Z = %.2f" % FLOOR_Z in task, "任务里写的地面高和布局算的不一样(布局:%.3f)" % FLOOR_Z
txt = open(os.path.join(HERE, "rd/env_cfg/robot/g1walk.yml")).read().replace("[0.0, -0.6, 0.80]", "[0.0, -0.6, %.4f]" % (FLOOR_Z + 0.75))
open(os.path.join(R, "env_cfg/robot/g1walk.yml"), "w").write(txt)
print("装了 env_cfg/robot/g1walk.yml(地面 z = %.3f,骨盆 z = %.3f)" % (FLOOR_Z, FLOOR_Z + 0.75))
subprocess.run([sys.executable, os.path.join(HERE, "make_livingroom_layout.py"), R, "--cfg_name", "g1walk"], check=True)
