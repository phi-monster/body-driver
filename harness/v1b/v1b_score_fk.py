#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V1b 3c 打分:驱动每一帧按关节读数算出来的手的位姿(vid/fk_poses.txt,世界系、模型单位)和身体自己报的(vid/poses.txt,只给打分)比。
驱动一个真值都没看。对齐:相似变换(世界系之间差一个转、移、倍数)+ 眼离手的偏移,只用每隔一个不同姿势取一半来拟合,另一半考试。
用法:v1b_score_fk.py <跑的目录>"""
import sys, os, math
import numpy as np
from scipy.optimize import least_squares
RUN = sys.argv[1]
def load(p):
    out = {}
    for l in open(p):
        f = l.split()
        if len(f) >= 9:
            out[int(f[0])] = np.array([float(x) for x in f[2:]])
    return out
fk = load(os.path.join(RUN, "vid", "fk_poses.txt")); tr = load(os.path.join(RUN, "vid", "poses.txt"))
def qR(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def rv(x):
    a = np.linalg.norm(x)
    if a < 1e-12: return np.eye(3)
    k = x / a; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
def logR(R):
    c = max(-1, min(1, (np.trace(R) - 1) / 2)); a = math.acos(c)
    if a < 1e-9: return np.zeros(3)
    return a / (2 * math.sin(a)) * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
seqs = sorted(set(fk) & set(tr))
narm = min(len(fk[seqs[0]]), len(tr[seqs[0]])) // 7 if seqs else 0
print("帧 %d(两边都有)· %d 只手" % (len(seqs), narm))
for a in range(narm):
    P = np.array([fk[s][7*a:7*a+7] for s in seqs]); T = np.array([tr[s][7*a:7*a+7] for s in seqs])
    key = np.round(P[:, :3] * 1e4).astype(np.int64)
    _, first = np.unique(key, axis=0, return_index=True)
    first = np.sort(first)
    trn = first[0::2]; tst = first[1::2]
    Rf = np.array([qR(p[3:]) for p in P]); Rt = np.array([qR(t[3:]) for t in T])
    def res(x, idx):
        Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        pe = (s * (P[idx, :3] @ Rg.T) + tg) - (T[idx, :3] + np.einsum('nab,b->na', Rt[idx], tx))
        re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
        return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
    best = None
    rng = np.random.default_rng(0)
    for k in range(20):
        x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
        r = least_squares(lambda x: res(x, trn), x0, method="lm", max_nfev=3000)
        if best is None or r.cost < best.cost: best = r
    x = best.x
    def err(idx):
        Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        pe = (s * (P[idx, :3] @ Rg.T) + tg) - (T[idx, :3] + np.einsum('nab,b->na', Rt[idx], tx))
        return np.linalg.norm(pe, axis=1) * 1000
    et, ee = err(trn), err(tst)
    print("手 %d:不同姿势 %d 个(拟合 %d、考试 %d)· 倍数 %.4f m/单位 · 眼离手 (%.1f, %.1f, %.1f) mm" % (a, len(first), len(trn), len(tst), x[6], *(1000 * x[10:13])))
    print("   拟合那一半:中位 %.2f mm、最大 %.2f mm;考试那一半:中位 %.2f mm、九成 %.2f mm、最大 %.2f mm" %
          (np.median(et), et.max(), np.median(ee), np.quantile(ee, 0.9), ee.max()))


# ── 第二种考法:驱动落盘的运动学(look/kinem_arm<k>.txt)在扫描各格上按关节读数算眼在哪,和扫描时记的仿真真值(sweep.txt 行尾)比 ──
def kinem_fk(W, P, q0, q):
    R = np.eye(3); t = np.zeros(3)
    for i in range(len(W)):
        w = W[i] / np.linalg.norm(W[i]); th = q[i] - q0[i]
        K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        Ri = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
        t = t + R @ (P[i] - Ri @ P[i]); R = R @ Ri
    return R, t
KDIR = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RUN, "look")   # 模型在哪(默认驱动落盘的;离线回放写到别处时给这个)
sp = os.path.join(RUN, "look", "sweep.txt")
if os.path.exists(sp):
    rows = [l for l in open(sp) if "||" in l]
    for arm in range(2):
        kp = os.path.join(KDIR, "kinem_arm%d.txt" % arm)
        if not os.path.exists(kp):
            continue
        L = open(kp).read().split("\n")
        q0 = np.array([float(x) for x in L[1].split()[1:]])
        W, Pp = [], []
        for l in L[2:]:
            if l.startswith("axis"):
                v = [float(x) for x in l.split()[2:]]; W.append(v[:3]); Pp.append(v[3:])
        W = np.array(W); Pp = np.array(Pp)
        Rf, Tf, Tp, Rt = [], [], [], []
        for l in rows:
            left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
            if int(h[1]) != arm:
                continue
            ee = [float(x) for x in right.split()]
            if len(ee) != 7:
                continue
            q = np.array([float(x) for x in parts[1 + arm].split()])
            R, t = kinem_fk(W, Pp, q0, q)
            Rf.append(R); Tf.append(t); Tp.append(np.array(ee[:3])); Rt.append(qR(np.array(ee[3:])))
        Rf = np.array(Rf); Tf = np.array(Tf); Tp = np.array(Tp); Rt = np.array(Rt)
        n = len(Tf); trn = np.arange(0, n, 2); tst = np.arange(1, n, 2)
        def res2(x, idx):
            Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
            pe = (s * (Tf[idx] @ Rg.T) + tg) - (Tp[idx] + np.einsum('nab,b->na', Rt[idx], tx))
            re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
            return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
        best = None; rng = np.random.default_rng(0)
        for k in range(30):
            x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
            r = least_squares(lambda x: res2(x, trn), x0, method="lm", max_nfev=4000)
            if best is None or r.cost < best.cost: best = r
        x = best.x
        Rg = rv(x[0:3]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        def e2(idx):
            return np.linalg.norm((s * (Tf[idx] @ Rg.T) + tg) - (Tp[idx] + np.einsum('nab,b->na', Rt[idx], tx)), axis=1) * 1000
        et, ee_ = e2(trn), e2(tst)
        print("运动学(驱动落盘)· 手 %d:扫描 %d 格(拟合 %d、考试 %d)· 倍数 %.4f m/单位 · 眼离手 (%.1f, %.1f, %.1f) mm" % (arm, n, len(trn), len(tst), s, *(1000 * tx)))
        print("   拟合那一半:中位 %.2f mm、最大 %.2f mm;考试那一半:中位 %.2f mm、九成 %.2f mm、最大 %.2f mm" %
              (np.median(et), et.max(), np.median(ee_), np.quantile(ee_, 0.9), ee_.max()))
