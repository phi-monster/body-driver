# -*- coding: utf-8 -*-
"""chase_mouse 的老鼠走的区域(路 8,10-01;主代理批了):原来写死 self._bounds = x ±0.30、y −0.55 ~ −0.32 —— 在 G1 歇着的小臂底下、
贴着头顶相机的支架;它自己的布局(task/RoboDojo/config/chase_mouse.yml 的 xlim / ylim)是 x −0.4 ~ 0.4、y −0.2 ~ 0.05。
老鼠按真速度(fix_mouse_speed.py 以后一个动作 1 cm)走满 200 个动作,第 50 个动作前后钻到左小臂和支架那儿,爬高 1.7 cm、歪了,
之后一步不动(/root/p8/chk/walk200.txt)。
改成:区域 = 它布局自己的范围,从 config/chase_mouse.yml 读(和出布局的是同一份),不在任务代码里再写一遍数。
接在 fix_mouse_speed.py 后面跑;可以重复跑(改过就不再改);改之前留一份 chase_mouse.py.bak_1001b。
用法:/venv/RoboDojo/bin/python fix_mouse_region.py /root/RoboDojo      (只换 __init__ 里 self._bounds 那一行)
"""
import os
import re
import shutil
import sys

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
p = f"{R}/task/RoboDojo/tasks/chase_mouse.py"
s = open(p, encoding="utf-8").read()
MARK = "# [bd 10-01b] 老鼠走的区域 = 它布局自己的范围"
if MARK in s:
    print("已改过", p)
    sys.exit(0)
OLD = "        self._bounds = ((-0.30, 0.30), (-0.55, -0.32))\n"
assert s.count(OLD) == 1, "chase_mouse.py 的样子变了,找不到 self._bounds 那一行"
NEW = ('''        ''' + MARK + '''(config/chase_mouse.yml 的 xlim / ylim,出布局用的就是它)。
        # 原来写死 x ±0.30、y −0.55 ~ −0.32:在 G1 歇着的小臂底下、贴着头顶相机的支架,老鼠按真速度走 50 个动作就卡死在那儿
        import yaml as _yaml
        _cm = _yaml.safe_load(open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "chase_mouse.yml"),
                                   encoding="utf-8"))["Rigid"][0]["common"]
        self._bounds = (tuple(float(v) for v in _cm["xlim"]), tuple(float(v) for v in _cm["ylim"]))
''')
assert re.search(r"^import os$", s, flags=re.M), "chase_mouse.py 里没有 import os"
bak = p + ".bak_1001b"
if not os.path.exists(bak):
    shutil.copy(p, bak)
s = s.replace(OLD, NEW)
open(p, "w", encoding="utf-8").write(s)
print("改了", p, "(原样留在", bak, ")")
