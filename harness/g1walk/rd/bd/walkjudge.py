# -*- coding: utf-8 -*-
# body-driver(路 8)远 5"上楼梯"的判据:由 harness/g1walk/install.py 装到 task/RoboDojo/bd/walkjudge.py,别手改。
# bd_on_top {"label": 家具的标签, "place": 那件家具上的地方}:两只脚都站在那块面上 ——
#   ① 两只脚那一节(left / right_ankle_roll_link)的位置投到那件家具的资产系里,落在那块面(metadata 的 passive.functional.place)的半长半宽以内;
#   ② 两只脚那一节都比那块面高出"开局站在地上时脚那一节比地面高出的那一截"再减 2 cm(脚那一节不在脚底:站在地上时它离地多高,开局量一次)。
# 只读仿真真值(连杆位姿、家具位姿、metadata),尺寸不写在这里。
import types

import numpy as np

from task.RoboDojo.bd.scene import _inst, _np, _pose, _rotm
from task.RoboDojo.bd.walkrig import FLOOR_Z

FEET = ("left_ankle_roll_link", "right_ankle_roll_link")
SLACK = 0.02


def _feet(fp, env_idx):
    art = fp.robot_manager.robot_key[0]
    ids = [art.body_names.index(n) for n in FEET]
    org = _np(fp.layout_manager.scene_manager.env_origins[env_idx])[:3]
    return art.data.body_pos_w[env_idx, ids].detach().cpu().numpy() - org


def bd_on_top(self, args):
    e = args["env_idx"]
    feet = _feet(self, e)
    ref = self.__dict__.setdefault("_bd_feet_ref", {}).setdefault(e, float(feet[:, 2].min()) - FLOOR_Z)   # 开局(这一集头一回判)脚那一节离地多高
    inst = _inst(self, e, args["label"])
    pos, quat = _pose(self, e, inst)
    meta = self.layout_manager.get_instance_metadata(env_idx=e, inst_name=inst) or {}
    pl = meta["passive"]["functional"]["place"][args["place"]]
    R = _rotm(quat)
    local = (feet - pos) @ R                                  # 两只脚在家具资产系里
    c, half = np.asarray(pl["center"], dtype=float), np.asarray(pl["half"], dtype=float)
    inside = bool(np.all(np.abs(local[:, :2] - c[:2]) <= half))
    top_w = float(pos[2] + (R @ c)[2])
    high = bool(np.all(feet[:, 2] - top_w >= ref - SLACK))
    self.__dict__.setdefault("_bd_on_top", {})[e] = {"inside": inside, "high": high, "feet_over_top": [round(float(v), 3) for v in feet[:, 2] - top_w],
                                                     "feet_over_floor_at_start": round(ref, 3)}
    return 1.0 if (inside and high) else 0.0


def install_checks(func_parser):
    func_parser.bd_on_top = types.MethodType(bd_on_top, func_parser)
    if getattr(func_parser, "_bd_walkjudge_reset", False):
        return
    orig = func_parser.reset

    def reset():
        func_parser.__dict__.pop("_bd_feet_ref", None)     # 每一集重新量开局脚那一节离地多高
        func_parser.__dict__.pop("_bd_on_top", None)
        return orig()

    func_parser.reset = reset
    func_parser._bd_walkjudge_reset = True
