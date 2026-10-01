# -*- coding: utf-8 -*-
"""随机题机的判据(路 8)。装到 RoboDojo 的 task/RoboDojo/bd/question.py(新文件)。和 scene.py 一样绑到 Func_Parser 实例上,源文件不动。
只读仿真真值:物件的位姿、速度、身体末端的位姿。物件的形状从它自己 metadata 里读:make_pool.py 从网格量的凸包顶点(资产系),
按此刻的位姿转到世界里 —— 最低点、最高点、桌面上的投影都按真形状算(第一版用包围盒的 8 个角:东西一歪,角比真东西低出一截,
"放在上面"会判成没放上)。"抬起来"直接用 RoboDojo 自己的 is_lift。"""
import math
import types

import numpy as np

from task.RoboDojo.bd.scene import _inst, _np, _pose, _rotm, _table_top


def bd_of(meta):
    """make_pool.py 量的、出题用的那一块(在 metadata 的 geometry 里:RoboDojo 读 metadata 只留 geometry 等五项)"""
    return meta["geometry"]["bd"]


def _shape(fp, env_idx, inst):
    """资产系里的形状点(make_pool.py 从网格量的凸包顶点),每件只读一次 metadata。题里只用物件池的东西,每件都有;
    没有就是物件池旧了,直接报错(不退回包围盒:第一版就是悄悄退回了包围盒,没人知道)"""
    cache = fp.__dict__.setdefault("_bdq_shape", {})
    if inst not in cache:
        cache[inst] = np.asarray(bd_of(fp.layout_manager.get_instance_metadata(env_idx=env_idx, inst_name=inst))["hull"], dtype=float)
    return cache[inst]


def _corners(fp, env_idx, inst):
    """形状点此刻在世界(相对本 env 原点)里的位置"""
    pos, q = _pose(fp, env_idx, inst)
    return pos + _shape(fp, env_idx, inst) @ _rotm(q).T


def _center_xy(pts):
    """桌面上投影的中心(投影外接框的中点)"""
    return (pts[:, :2].min(axis=0) + pts[:, :2].max(axis=0)) / 2


def _hull(pts):
    """二维凸包的顶点(逆时针)。凸包顶点一件就有几百上千个(香蕉 3359),判据每一拍都要算,用 scipy 的 qhull,不用 Python 一点点绕"""
    from scipy.spatial import ConvexHull
    P = np.asarray(pts, dtype=float)[:, :2]
    return P[ConvexHull(P).vertices]


def _footprint(fp, env_idx, inst):
    return _hull(_corners(fp, env_idx, inst)[:, :2])


def _inside_many(poly, P, shrink=0.0):
    """P 里每个点是不是在凸多边形 poly(逆时针)里面、离每条边至少 shrink"""
    a = np.asarray(poly, dtype=float)
    e = np.roll(a, -1, axis=0) - a
    L = np.linalg.norm(e, axis=1)
    a, e, L = a[L > 1e-12], e[L > 1e-12], L[L > 1e-12]
    P = np.atleast_2d(np.asarray(P, dtype=float))
    d = (e[None, :, 0] * (P[:, None, 1] - a[None, :, 1]) - e[None, :, 1] * (P[:, None, 0] - a[None, :, 0])) / L[None, :]
    return (d >= shrink).all(axis=1)


def _inside(poly, p, shrink=0.0):
    return bool(_inside_many(poly, p, shrink)[0])


def _pts_to_edges(P, Q):
    """P 里的点到凸多边形 Q 各条边最近的距离"""
    a = np.asarray(Q, dtype=float)
    ab = np.roll(a, -1, axis=0) - a
    L2 = (ab * ab).sum(axis=1)
    t = ((P[:, None, :] - a[None]) * ab[None]).sum(axis=-1) / np.where(L2 > 1e-24, L2, 1.0)[None]
    t = np.clip(np.where(L2[None] > 1e-24, t, 0.0), 0.0, 1.0)
    return float(np.linalg.norm(P[:, None, :] - (a[None] + t[..., None] * ab[None]), axis=-1).min())


def _poly_gap(A, B):
    """两个凸多边形之间最近的距离(叠在一起 = 0)"""
    A, B = np.asarray(A, dtype=float), np.asarray(B, dtype=float)
    if _inside_many(B, A).any() or _inside_many(A, B).any():
        return 0.0
    return min(_pts_to_edges(A, B), _pts_to_edges(B, A))


def _start(fp, env_idx, inst):
    s = _np(fp.pre_state[env_idx][inst]["pose"])
    return s[:3], s[3:7]


def bdq_next_to(self, args):
    """A 挪到 B 旁边:两者在桌面上的投影最近处 ≤ gap;A 还在桌上(高度和开局差不到 2 cm);A 挪过 ≥ min_move(开局两者离得远,题目保证)"""
    e = args["env_idx"]
    a, b = _inst(self, e, args["a"]), _inst(self, e, args["b"])
    pa, _ = _pose(self, e, a)
    pa0, _ = _start(self, e, a)
    gap = _poly_gap(_footprint(self, e, a), _footprint(self, e, b))
    ok = gap <= float(args["gap"]) and abs(pa[2] - pa0[2]) <= 0.02 and np.linalg.norm(pa[:2] - pa0[:2]) >= float(args["min_move"])
    return 1.0 if ok else 0.0


def bdq_turned(self, args):
    """原地转过来:绕竖轴转过 ≥ angle(弧度);还是立着(倾斜 ≤ 30°);还在桌上"""
    e = args["env_idx"]
    a = _inst(self, e, args["label"])
    p, q = _pose(self, e, a)
    p0, q0 = _start(self, e, a)
    Rr = _rotm(q) @ _rotm(q0).T
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, Rr[2, 2]))))
    yaw = abs(math.atan2(Rr[1, 0], Rr[0, 0]))
    ok = yaw >= float(args["angle"]) and tilt <= 30.0 and abs(p[2] - p0[2]) <= 0.02
    return 1.0 if ok else 0.0


def bdq_pushed(self, args):
    """推过去:沿 dir(桌面上的单位向量)挪了 ≥ dist、横向偏 ≤ dist / 2,而且这一集里从来没被抬高过 max_lift 以上(推,不是拿起来放过去)。
    最高到过多高记在这个 Func_Parser 上(判据每一拍都判,一直判到成)"""
    e = args["env_idx"]
    a = _inst(self, e, args["label"])
    p, _ = _pose(self, e, a)
    p0, _ = _start(self, e, a)
    hist = self.__dict__.setdefault("_bdq_zmax", {})
    key = (e, a, tuple(np.round(p0, 5)))
    hist[key] = max(hist.get(key, p[2]), p[2])
    d = np.asarray(args["dir"], dtype=float)
    d = d / np.linalg.norm(d)
    dp = p[:2] - p0[:2]
    along = float(dp @ d)
    lateral = abs(float(dp[0] * d[1] - dp[1] * d[0]))
    ok = along >= float(args["dist"]) and lateral <= float(args["dist"]) / 2.0 and hist[key] - p0[2] <= float(args["max_lift"])
    return 1.0 if ok else 0.0


def bdq_on(self, args):
    """A 放到 B 上(mode = on)或放进 B 里(mode = in):A 的中心在 B 的投影里(离边 ≥ 1 cm);
    on:A 的最低点不低于 B 的顶 1.5 cm、不高过 3 cm(搁在上面);in:A 的最低点高出 B 的底 2 mm、不高过 B 的顶;A 停住了(速度 < 3 cm/s)"""
    e = args["env_idx"]
    a, b = _inst(self, e, args["a"]), _inst(self, e, args["b"])
    ca, cb = _corners(self, e, a), _corners(self, e, b)
    pa, _ = _pose(self, e, a)
    obj = self.layout_manager.get_scene_object(e, a)
    v = float(np.linalg.norm(_np(obj.get_linear_velocity())[:3])) if obj is not None else 0.0
    inside = _inside(_hull(cb[:, :2]), _center_xy(ca), shrink=0.01)
    a_bot, b_top, b_bot = ca[:, 2].min(), cb[:, 2].max(), cb[:, 2].min()
    if args.get("mode", "on") == "in":
        level = b_bot + 0.002 <= a_bot <= b_top
    else:
        level = b_top - 0.015 <= a_bot <= b_top + 0.03
    return 1.0 if (inside and level and v < 0.03) else 0.0


def bdq_above(self, args):
    """飞到它正上方:身体末端(无人机就是机身)在它中心正上方 xy_tol 以内,而且高出它的顶 ≥ clear"""
    e = args["env_idx"]
    a = _inst(self, e, args["label"])
    c = _corners(self, e, a)
    robot = self.robot_manager.robot_list[int(args.get("robot", 0))]
    ee = _np(self.robot_manager.get_real_endpose(robot, env_idx_list=[e], is_relative=True)[e])
    ok = np.linalg.norm(ee[:2] - _center_xy(c)) <= float(args["xy_tol"]) and ee[2] >= c[:, 2].max() + float(args["clear"])
    return 1.0 if ok else 0.0


def fits(meta_a, meta_b):
    """A 能不能放到 B 上 / 放进 B 里(出题和离线核都用这一条,两边的尺寸都是 make_pool.py 从网格量的):
    B 是容器:A 最细的那个方向(顺着最长轴看过去,凸包投影的最小外接圆)比 B 的口(半腰那一圈的最大空圆)小 ⇒ "in";
    B 是平顶:A 平放时在桌面上的两条边,短的不比 B 顶面短的那条长、长的不比长的那条长(A 整个搁得下)⇒ "on";否则 None"""
    ba, bb = bd_of(meta_a), bd_of(meta_b)
    if bb.get("container"):
        return "in" if ba["pass_d"] < bb["opening_d"] else None
    if bb.get("flat_top"):
        ea = sorted(meta_a["geometry"]["aligned_bbox"]["extents"][:2])
        eb = sorted(meta_b["geometry"]["aligned_bbox"]["extents"][:2])
        return "on" if ea[0] <= eb[0] and ea[1] <= eb[1] else None
    return None


def find_question(layout):
    """这一集的题:写在目标那件东西的记录里("bd_question" 那一项)。不能放在布局最上面一层:
    RoboDojo 的 SceneManager 把最上面一层除了房间 / 桌 / 地 / 光 / 背景以外的每一项都当成一类东西去生(第一版这样放,开场就 TypeError)"""
    for sect in ("Rigid", "Articulation", "Geometry", "Garment"):
        for lst in ((layout or {}).get(sect) or {}).values():
            for r in lst:
                if isinstance(r, dict) and "bd_question" in r:
                    return r["bd_question"]
    return {}


CHECKS = {f.__name__: f for f in (bdq_next_to, bdq_turned, bdq_pushed, bdq_on, bdq_above)}


def install_checks(func_parser):
    for name, fn in CHECKS.items():
        setattr(func_parser, name, types.MethodType(fn, func_parser))
    func_parser.__dict__.pop("_bdq_zmax", None)
    func_parser.__dict__.pop("_bdq_shape", None)
