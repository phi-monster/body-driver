# -*- coding: utf-8 -*-
"""第 40 条(路 8):大客厅"收拾完没有"的判据。装到 task/RoboDojo/bd/tidy.py(新文件),和 scene.py / question.py 一样绑到 Func_Parser 上。
只读仿真真值:每件东西的位姿和速度、家具的位姿;东西的形状用物件池量的凸包(question.py 的 _corners),家具上放东西的地方从家具自己的
metadata(passive.functional.place)读。每件东西该去哪写在布局里它自己的记录里("bd_place": [家具的标签, 放东西的地方])。

一件东西"放好了" = 它投影的中心在那一面(箱子口)里;平面:最低点在面上 −1.5 ~ +3 cm(和随机题"放上去"一样);箱子:最低点在箱底和箱口之间;
停住了(< 3 cm/s)。判据 bd_tidy 给 1 = 每一件都放好了;放好了几件记在 Func_Parser 上(_bd_tidy_done),录像和报告用。
"""
import math
import types

import numpy as np

from task.RoboDojo.bd.question import _center_xy, _corners
from task.RoboDojo.bd.scene import _inst, _np, _pose, _rotm


def places(layout):
    """布局里每件要收拾的东西:(标签, 家具标签, 放东西的地方)"""
    out = []
    for lst in ((layout or {}).get("Rigid") or {}).values():
        for r in lst:
            if isinstance(r, dict) and r.get("bd_place"):
                out.append((r["label"], r["bd_place"][0], r["bd_place"][1]))
    return out


def find_tidy(layout):
    for lst in ((layout or {}).get("Rigid") or {}).values():
        for r in lst:
            if isinstance(r, dict) and "bd_tidy" in r:
                return r["bd_tidy"]
    return {}


def in_place(fp, env_idx, label, furn, place):
    obj = _inst(fp, env_idx, label)
    fi = _inst(fp, env_idx, furn)
    meta = fp.layout_manager.get_instance_metadata(env_idx=env_idx, inst_name=fi)
    pl = meta["passive"]["functional"]["place"][place]
    fpos, fq = _pose(fp, env_idx, fi)
    Rf = _rotm(fq)
    top = fpos + Rf @ np.asarray(pl["center"], dtype=float)
    half = np.asarray(pl["half"], dtype=float)
    depth = float(pl["depth"])
    c = _corners(fp, env_idx, obj)
    uv = (Rf.T @ np.r_[_center_xy(c) - top[:2], 0.0])[:2]
    inside = bool(np.all(np.abs(uv) <= half))
    low = float(c[:, 2].min())
    if depth > 0:
        level = top[2] - depth - 0.01 <= low <= top[2]
    else:
        level = top[2] - 0.015 <= low <= top[2] + 0.03
    o = fp.layout_manager.get_scene_object(env_idx, obj)
    v = float(np.linalg.norm(_np(o.get_linear_velocity())[:3])) if o is not None else 0.0
    return inside and level and v < 0.03


def bd_tidy(self, args):
    e = args["env_idx"]
    lay = self.layout_manager.saved_layouts[e]
    todo = places(lay)
    done = [lab for lab, furn, place in todo if in_place(self, e, lab, furn, place)]
    self.__dict__.setdefault("_bd_tidy_done", {})[e] = (len(done), len(todo), done)
    return 1.0 if todo and len(done) == len(todo) else 0.0


def install_checks(func_parser):
    func_parser.bd_tidy = types.MethodType(bd_tidy, func_parser)
    func_parser.__dict__.pop("_bd_tidy_done", None)
    func_parser.__dict__.pop("_bdq_shape", None)   # 凸包按实例名缓存(question.py):换一张布局重读
