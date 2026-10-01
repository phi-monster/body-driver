# -*- coding: utf-8 -*-
"""随机题机的判据(路 8)。装到 RoboDojo 的 task/RoboDojo/bd/question.py(新文件)。和 scene.py 一样绑到 Func_Parser 实例上,源文件不动。
只读仿真真值:物件的位姿、速度、身体末端的位姿。物件多大从它自己 metadata 的包围盒读(资产系),按此刻的位姿转到世界里。
"抬起来"直接用 RoboDojo 自己的 is_lift。"""
import math
import types

import numpy as np

from task.RoboDojo.bd.scene import _inst, _np, _pose, _rotm, _table_top


def _bbox(fp, env_idx, inst):
    meta = fp.layout_manager.get_instance_metadata(env_idx=env_idx, inst_name=inst) or {}
    v = np.asarray(meta["geometry"]["aligned_bbox"]["vertices"], dtype=float)
    return v.min(axis=0), v.max(axis=0)


def _corners(fp, env_idx, inst):
    lo, hi = _bbox(fp, env_idx, inst)
    pos, q = _pose(fp, env_idx, inst)
    R = _rotm(q)
    return np.array([pos + R @ np.array([x, y, z]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def _hull(pts):
    """二维凸包(逆时针)"""
    P = sorted(set(map(tuple, np.round(pts, 6))))
    if len(P) <= 2:
        return np.array(P)
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])


def _footprint(fp, env_idx, inst):
    return _hull(_corners(fp, env_idx, inst)[:, :2])


def _inside(poly, p, shrink=0.0):
    """p 在凸多边形 poly(逆时针)里面,离每条边至少 shrink"""
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        L = float(np.linalg.norm(e))
        if L < 1e-9:
            continue
        if (e[0] * (p[1] - a[1]) - e[1] * (p[0] - a[0])) / L < shrink:
            return False
    return True


def _poly_gap(A, B):
    """两个凸多边形之间最近的距离(叠在一起 = 0)"""
    if any(_inside(B, a) for a in A) or any(_inside(A, b) for b in B):
        return 0.0

    def seg_pt(p, a, b):
        ab = b - a
        t = 0.0 if float(ab @ ab) < 1e-12 else min(1.0, max(0.0, float((p - a) @ ab) / float(ab @ ab)))
        return float(np.linalg.norm(p - (a + t * ab)))

    best = 1e9
    for P, Q in ((A, B), (B, A)):
        for p in P:
            for i in range(len(Q)):
                best = min(best, seg_pt(p, Q[i], Q[(i + 1) % len(Q)]))
    return best


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
    inside = _inside(_hull(cb[:, :2]), ca[:, :2].mean(axis=0), shrink=0.01)
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
    ok = np.linalg.norm(ee[:2] - c[:, :2].mean(axis=0)) <= float(args["xy_tol"]) and ee[2] >= c[:, 2].max() + float(args["clear"])
    return 1.0 if ok else 0.0


CHECKS = {f.__name__: f for f in (bdq_next_to, bdq_turned, bdq_pushed, bdq_on, bdq_above)}


def install_checks(func_parser):
    for name, fn in CHECKS.items():
        setattr(func_parser, name, types.MethodType(fn, func_parser))
    func_parser.__dict__.pop("_bdq_zmax", None)
