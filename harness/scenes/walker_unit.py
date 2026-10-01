# -*- coding: utf-8 -*-
# 会自己走的东西(rd/bd/scene.py 的 Walker)的离线单测:不起 Isaac、不占卡,假物体按 Walker 给的速度积分(带一点摩擦)。
# 查:每个动作 1 cm;不认资产自己的轴(z 朝上 / 侧躺着 y 朝上 / x 朝上都一样走);被拿起来就停;开局在区域外会自己走回来;
# 被推开不弹回去;被挡住时不走、放开后不跳。箱上跑:/venv/RoboDojo/bin/python walker_unit.py
import math, os, sys, types
import numpy as np
import torch
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "rd"))
from bd import scene


def qaxis(axis, deg):
    a = np.asarray(axis, float); a /= np.linalg.norm(a); h = math.radians(deg) / 2
    return np.array([math.cos(h), *(a * math.sin(h))])


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


class Body:
    def __init__(self, pos, quat):
        self.p = np.array(pos, float); self.q = np.array(quat, float); self.v = np.zeros(3); self.w = np.zeros(3)
    def get_local_pose(self): return torch.tensor(self.p), torch.tensor(self.q)
    def get_linear_velocity(self): return torch.tensor(self.v)
    def set_linear_velocity(self, v): self.v = np.asarray(v, float)
    def set_angular_velocity(self, w): self.w = np.asarray(w, float)
    def step(self, dt, friction_decel=0.0, blocked=False):
        vh = self.v[:2].copy(); n = np.linalg.norm(vh)
        if n > 0 and friction_decel > 0:   # crude friction: lose a bit of speed during the step
            vh = vh * max(0.0, 1 - friction_decel * dt / n)
        if blocked:
            vh[:] = 0.0
        self.v[:2] = vh
        self.p[:2] += vh * dt
        ang = self.w[2] * dt
        self.q = qmul(qaxis([0, 0, 1], math.degrees(ang)), self.q)


class LM:
    def __init__(self, recs, objs): self.recs, self.objs = recs, objs
    def get_layout_records(self, env_idx, t): return self.recs if t == "Rigid" else []
    def get_scene_object(self, env_idx, inst): return self.objs[inst]


def run(quat, label="target", lift_at=None, steps=60, params=None, start=(0.0, 0.0), push_at=None, block=None):
    b = Body([start[0], start[1], 0.78], quat)
    rec = {"inst_name": "thing_0_1", "label": label}
    if params is None:
        rec["bd_walk"] = {"speed": 0.01, "turn_every": 25, "region": [[-0.28, 0.28], [-0.12, 0.10]], "free_height": 0.005, "seed": 7, "yaw0": 0.0}
    env = types.SimpleNamespace(num_envs=1, dt=0.004, end_flag=[False], obs_manager=types.SimpleNamespace(collect_interval=10.0),
                                scene_manager=types.SimpleNamespace(layout_manager=LM([rec], {"thing_0_1": b})))
    wk = scene.Walker(params)
    xs = [b.p.copy()]
    for k in range(steps):
        if lift_at is not None and k == lift_at:
            b.p[2] += 0.05
        if push_at is not None and k == push_at:
            b.p[0] += 0.03          # someone shoved it 3 cm sideways
        for _ in range(10):
            wk.tick(env)
            b.step(env.dt, friction_decel=4.9, blocked=(block is not None and block[0] <= k < block[1]))
        xs.append(b.p.copy())
    X = np.array(xs)
    d = np.linalg.norm(np.diff(X[:, :2], axis=0), axis=1)
    return X, d


for name, q in (("z-up", [1, 0, 0, 0]), ("lying: local y up (rot x +90)", qaxis([1, 0, 0], 90)), ("rot y 90 (local x up)", qaxis([0, 1, 0], 90))):
    X, d = run(q)
    inside = (X[:, 0] >= -0.2801).all() and (X[:, 0] <= 0.2801).all() and (X[:, 1] >= -0.1201).all() and (X[:, 1] <= 0.1001).all()
    print("%-32s per-action median %.4f m (min %.4f max %.4f) inside=%s" % (name, np.median(d), d.min(), d.max(), inside))
X, d = run([1, 0, 0, 0], lift_at=20)
print("lifted at action 20: moved before %.4f m/action, after %.4f m/action" % (np.median(d[:19]), np.median(d[21:])))
X, d = run(qaxis([1, 0, 0], 90), params={"target": {"speed": 0.01, "turn_every": 40, "region": [[-0.3, 0.3], [-0.55, -0.32]], "free_height": 0.005, "seed": 3, "yaw0": 0.0}},
           label="target", start=(0.0, -0.30))
print("params_by_label (chase_mouse style), start 2 cm outside region y: median %.4f m/action, y first->last %.3f -> %.3f, inside at end %s"
      % (np.median(d), X[0, 1], X[-1, 1], -0.5501 <= X[-1, 1] <= -0.3199))
X, d = run([1, 0, 0, 0], push_at=30)
print("pushed 3 cm at action 30: step at push %.4f, median after %.4f (no snap back: %s)" % (d[30], np.median(d[32:]), d[31] < 0.02))
X, d = run([1, 0, 0, 0], block=(20, 30))
print("blocked actions 20-29: median while blocked %.4f, first step after %.4f, median after %.4f (no jump: %s)"
      % (np.median(d[21:29]), d[30], np.median(d[32:]), d[30] < 0.02))
