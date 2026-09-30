# -*- coding: utf-8 -*-
"""chase_mouse 的老鼠走得太快(路 8,10-01):RoboDojo 一个动作 = collect_interval 个物理子步(= 1 /(dt × collect_freq)= 1 /(0.004 × 25)= 10),
任务的 step() 每个子步被调一次;原来的 _wander 每调一次就把老鼠挪一整个 BD_MOUSE_SPEED(它的注释写的是"每个 env step")
⇒ 一个动作走 10 cm,不是 PLAN / 大并行 §1 写的 1 cm/步;"每 40 步换方向"也成了每 4 个动作。另外老鼠被抓起来以后还照样被按平面拽着走。
改成:老鼠按 task/RoboDojo/bd/scene.py 的 Walker 走(harness/scenes 装的,bd_walker 同一个):速度伺服,一个动作正好走 speed、
每 40 个动作随机换方向、碰边反射、被拿离开局那张面 5 mm 以上或翻倒就不走。每一集的随机数种子从 RoboDojo 这一集的全局随机数里抽(可重现)。
接在 g1_setup.py → patch_planner_key.py → lean_rig.py → root_lean.py → rig_upright_low.py 后面跑;可以重复跑(改过就不再改)。
用法:/venv/RoboDojo/bin/python fix_mouse_speed.py /root/RoboDojo      (只换 task/RoboDojo/tasks/chase_mouse.py 的 _wander 这一个方法)
"""
import re
import sys

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
p = f"{R}/task/RoboDojo/tasks/chase_mouse.py"
s = open(p, encoding="utf-8").read()
MARK = "# [bd 10-01] 老鼠按 task/RoboDojo/bd/scene.py 的 Walker 走"
if MARK in s:
    print("已改过", p)
    sys.exit(0)
NEW = '''    def _wander(self):
        ''' + MARK + '''(速度伺服):一个动作正好走 BD_MOUSE_SPEED、每 40 个动作换方向、
        # 碰边反射、被拿起来 / 翻倒就不走。原来每个物理子步挪一整个 speed(一个动作 10 个子步 = 10 cm),抓起来还被拽着走
        if self._mouse_step == 0 or getattr(self, "_bd_walker", None) is None:
            from task.RoboDojo.bd import scene as _bd_scene
            (x0, x1), (y0, y1) = self._bounds
            self._bd_walker = _bd_scene.Walker({"target": {"speed": self._speed, "turn_every": 40, "region": [[x0, x1], [y0, y1]],
                                                           "free_height": 0.005, "seed": random.randrange(2 ** 31), "yaw0": 0.0}})
        self._bd_walker.tick(self)
        self._mouse_step += 1

'''
m = re.search(r"    def _wander\(self\):\n.*?(?=    def step\(self, meta_control_list\):)", s, flags=re.S)
assert m, "chase_mouse.py 的样子变了,找不到 _wander"
assert "\nimport random" in s, "chase_mouse.py 里没有 import random"
s = s[:m.start()] + NEW + s[m.end():]
open(p, "w", encoding="utf-8").write(s)
print("改了", p)
