# -*- coding: utf-8 -*-
"""body-driver 小场景的运行时(大并行 §2 第 38 条,路 8)。装到 RoboDojo 的 task/RoboDojo/bd/scene.py(新文件,不改 RoboDojo 原有的任何文件)。

两样东西:
1. 判据:RoboDojo 的 RewardManager 按名字 getattr(func_parser, 名字) 找判据,所以这里的判据在任务 _post_setup_scene 里
   绑到那个 Func_Parser 实例上(install_checks),Func_Parser 的源文件一个字不动。判据只读仿真真值(位姿、关节、粒子),
   几何参数全从资产自己的 metadata.json(passive.functional)里读,任务代码里不写尺寸。
2. 会自己走的东西(Walker):布局里某件 Rigid 带 "bd_walk" 字段,它就在桌面上按步随机走。
   RoboDojo 一个动作 = collect_interval 个物理子步,任务的 step() 每个子步被调一次 ⇒ 每个子步走 speed / collect_interval,
   一个动作正好走 speed(chase_mouse 每个子步走一整个 speed,一个动作走了 10 倍)。只在它还躺在原来那张面上时走:被抬起来就不走了。
"""
import math
import os
import types

import numpy as np
import torch


# ---------------------------------------------------------------- 小工具
def _np(x):
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float).reshape(-1)


def _rotm(q):
    w, x, y, z = [float(v) for v in q]
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _inst(fp, env_idx, label):
    return fp.layout_manager.get_instance_name(label=label, env_idx=env_idx)


def _pose(fp, env_idx, inst):
    pos, rot = fp.layout_manager.get_instance_pose(inst_name=inst, env_idx=env_idx)
    return _np(pos)[:3], _np(rot)[:4]


def _functional(fp, env_idx, inst, tag):
    meta = fp.layout_manager.get_instance_metadata(env_idx=env_idx, inst_name=inst) or {}
    return ((meta.get("passive") or {}).get("functional") or {}).get(tag)


def _points(pos, quat, frames):
    """把物体系里的一串 frame([x,y,z,qw,qx,qy,qz])变成世界(相对本 env 原点)里的点"""
    R = _rotm(quat)
    return [pos + R @ np.asarray(f[:3], dtype=float) for f in frames]


def _table_top(fp, env_idx):
    return float(fp.layout_manager.table_info[env_idx].get("height", 0.0))


# ---------------------------------------------------------------- 判据(每个都是 (self=Func_Parser, args) -> 0.0 / 1.0)
def bd_joint_moved(self, args):
    """某个关节离开局一开始的位置 ≥ amount(关节自己的单位:滑轨米、转轴弧度)。开局位置取 init_state 记下的那一刻。"""
    env_idx, label, joint, amount = args["env_idx"], args["label"], args["joint"], float(args["amount"])
    inst = _inst(self, env_idx, label)
    obj = self.layout_manager.get_scene_object(env_idx, inst)
    q = float(obj.get_joint_info(joint)["position"])
    q0 = float(self.pre_state[env_idx][inst][joint]["position"])
    return 1.0 if abs(q - q0) >= amount else 0.0


def bd_peg_in_hole(self, args):
    """销插进孔里 ≥ depth:销的底(销自己 metadata 的 bottom 点)低于孔口 depth,且水平上在孔口半宽以内。
    孔口(孔那块 metadata 的 hole:frame = 孔口中心,half_width = 孔口半宽)。孔四周是实心的,底低于孔口又在孔口以内 = 只能在孔里。"""
    env_idx = args["env_idx"]
    peg, block = _inst(self, env_idx, args["peg"]), _inst(self, env_idx, args["block"])
    pp, pq = _pose(self, env_idx, peg)
    bp, bq = _pose(self, env_idx, block)
    bottom = _points(pp, pq, _functional(self, env_idx, peg, "bottom")["frame"])[0]
    hole = _functional(self, env_idx, block, "hole")
    mouth = _points(bp, bq, hole["frame"])[0]
    axis = _rotm(bq)[:, 2]
    lateral = (bottom - mouth) - np.dot(bottom - mouth, axis) * axis
    below = -np.dot(bottom - mouth, axis)
    return 1.0 if (below >= float(args["depth"]) and np.linalg.norm(lateral) <= float(hole["half_width"])) else 0.0


def bd_ring_on_hook(self, args):
    """环挂在钩子的横杆上:横杆那条线(钩子 metadata 的 arm:杆根、杆尖两点)穿过环中间 ——
    ① 环心到杆那条线的距离 < 环的内半径(环心 metadata 的 center:frame、inner_radius);② 环心投到杆上落在杆根和杆尖之间;
    ③ 环的法向和杆差不到 45°(杆从环当中穿过去,不是环平躺在杆顶上);④ 环心高出桌面 > 环的外径(不在桌上)。"""
    env_idx = args["env_idx"]
    ring, hook = _inst(self, env_idx, args["ring"]), _inst(self, env_idx, args["hook"])
    rp, rq = _pose(self, env_idx, ring)
    hp, hq = _pose(self, env_idx, hook)
    cen = _functional(self, env_idx, ring, "center")
    c = _points(rp, rq, cen["frame"])[0]
    n = _rotm(rq) @ np.asarray(cen.get("axis", [0, 0, 1]), dtype=float)
    a0, a1 = _points(hp, hq, _functional(self, env_idx, hook, "arm")["frame"])[:2]
    d = a1 - a0
    L = float(np.linalg.norm(d))
    d = d / L
    t = float(np.dot(c - a0, d))
    dist = float(np.linalg.norm((c - a0) - t * d))
    ok = (dist < float(cen["inner_radius"]) and 0.0 <= t <= L and abs(float(np.dot(n, d))) > math.cos(math.radians(45.0))
          and c[2] - _table_top(self, env_idx) > 2.0 * float(cen["outer_radius"]))
    return 1.0 if ok else 0.0


def bd_trigger_while_held(self, args):
    """握住并扣扳机同时成立:枪身比开局抬高 ≥ lift(没人拿着它就会落回桌面)且扳机关节离开局 ≥ amount(弧度)。同一拍判。"""
    env_idx = args["env_idx"]
    gun = _inst(self, env_idx, args["gun"])
    pos, _ = _pose(self, env_idx, gun)
    z0 = float(self.pre_state[env_idx][gun]["pose"][2])
    obj = self.layout_manager.get_scene_object(env_idx, gun)
    q = float(obj.get_joint_info(args["joint"])["position"])
    q0 = float(self.pre_state[env_idx][gun][args["joint"]]["position"])
    return 1.0 if (pos[2] - z0 >= float(args["lift"]) and abs(q - q0) >= float(args["amount"])) else 0.0


def _cloth_points(fp, env_idx, inst):
    obj = fp.layout_manager.get_scene_object(env_idx, inst)
    pts, _, _, _ = obj.sample_mesh_vertices()
    pts = np.asarray(pts.detach().cpu().numpy() if hasattr(pts, "detach") else pts, dtype=float).reshape(-1, 3)
    org = _np(fp.layout_manager.scene_manager.env_origins[env_idx])[:3]
    return pts - org


def bd_cloth_lifted(self, args):
    """布被拿起来:布上最高的那个粒子高出桌面 ≥ height(布是粒子布,位姿不跟着粒子走,只能看粒子)"""
    env_idx = args["env_idx"]
    pts = _cloth_points(self, env_idx, _inst(self, env_idx, args["label"]))
    return 1.0 if float(pts[:, 2].max()) - _table_top(self, env_idx) >= float(args["height"]) else 0.0


CHECKS = {f.__name__: f for f in (bd_joint_moved, bd_peg_in_hole, bd_ring_on_hook, bd_trigger_while_held, bd_cloth_lifted)}


def install_checks(func_parser):
    """把上面的判据绑到这一个 Func_Parser 实例上(RewardManager.call_func_parser 按名字 getattr)"""
    for name, fn in CHECKS.items():
        setattr(func_parser, name, types.MethodType(fn, func_parser))


# ---------------------------------------------------------------- 会自己走的东西
class Walker:
    """布局里带 "bd_walk" 的 Rigid:{"speed": 每个动作走几米, "turn_every": 每几个动作随机换一次方向,
    "region": [[x0, x1], [y0, y1]](碰边就反射), "free_height": 高出它开局那张面多少就算被拿起来了(不走), "seed": 随机数种子,
    "yaw0": 资产自己的前方和 +x 差多少度}。每个物理子步调一次 tick。"""

    def __init__(self):
        self.state = {}
        self._log = os.environ.get("BD_WALK_LOG")

    def reset(self):
        self.state = {}

    def tick(self, env):
        lm = env.scene_manager.layout_manager
        om = getattr(env, "obs_manager", None)
        sub = int(round(float(getattr(om, "collect_interval", 1.0) or 1.0))) if om is not None else 1
        sub = max(sub, 1)
        ended = getattr(env, "end_flag", None)
        for env_idx in range(env.num_envs):
            if ended is not None and ended[env_idx]:
                continue
            for rec in lm.get_layout_records(env_idx, "Rigid"):
                w = rec.get("bd_walk")
                if not w:
                    continue
                inst = rec["inst_name"]
                obj = lm.get_scene_object(env_idx, inst)
                if obj is None:
                    continue
                pos, _ = obj.get_local_pose()
                p = _np(pos)[:3]
                key = (env_idx, inst)
                st = self.state.get(key)
                if st is None:
                    rng = np.random.default_rng(int(w.get("seed", 0)))
                    st = {"rng": rng, "heading": float(rng.uniform(0.0, 2.0 * math.pi)), "z_rest": float(p[2]), "ticks": 0}
                    self.state[key] = st
                if st["ticks"] > 0 and st["ticks"] % (int(w["turn_every"]) * sub) == 0:
                    st["heading"] = float(st["rng"].uniform(0.0, 2.0 * math.pi))
                if p[2] <= st["z_rest"] + float(w["free_height"]):
                    step = float(w["speed"]) / sub
                    h = st["heading"]
                    nx, ny = p[0] + step * math.cos(h), p[1] + step * math.sin(h)
                    (x0, x1), (y0, y1) = w["region"]
                    if nx < x0 or nx > x1:
                        h = math.pi - h
                        nx = min(max(nx, x0), x1)
                    if ny < y0 or ny > y1:
                        h = -h
                        ny = min(max(ny, y0), y1)
                    st["heading"] = h
                    yaw = h + math.radians(float(w.get("yaw0", 0.0)))
                    q = np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])
                    obj.set_local_pose(translation=np.array([nx, ny, p[2]]), orientation=q)
                    obj.set_linear_velocity(torch.zeros(3))
                    obj.set_angular_velocity(torch.zeros(3))
                if self._log and st["ticks"] % sub == 0:
                    with open(self._log, "a") as f:
                        f.write("%s %d %.5f %.5f %.5f\n" % (inst, st["ticks"] // sub, p[0], p[1], p[2]))
                st["ticks"] += 1
