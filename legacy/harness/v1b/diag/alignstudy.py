#!/usr/bin/env python3
# 逐炮:不动的眼的位置按仿真真值(相机配置 pos [0,-0.41,1.308])差多少;第 2 只手对齐差多少(世界系 mm)。真值只打分。
import sys, os, math
import numpy as np
from scipy.optimize import least_squares
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
def kinem_fk(W, P, q0, q):
    R = np.eye(3); t = np.zeros(3)
    for i in range(len(W)):
        w = W[i] / np.linalg.norm(W[i]); th = q[i] - q0[i]
        K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        Ri = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
        t = t + R @ (P[i] - Ri @ P[i]); R = R @ Ri
    return R, t
TRUE_FX = np.array([0.0, -0.41, 1.308])
def study(RUN, KDIR=None):
    KDIR = KDIR or os.path.join(RUN, "look")
    rows = [l for l in open(os.path.join(RUN, "look", "sweep.txt")) if "||" in l]
    FITS = {}
    for arm in range(2):
        L = open(os.path.join(KDIR, "kinem_arm%d.txt" % arm)).read().split("\n")
        q0 = np.array([float(x) for x in L[1].split()[1:]])
        W, Pp = [], []
        for l in L[2:]:
            if l.startswith("axis"):
                v = [float(x) for x in l.split()[2:]]; W.append(v[:3]); Pp.append(v[3:])
        W = np.array(W); Pp = np.array(Pp)
        Rf, Tf, Tp, Rt = [], [], [], []
        for l in rows:
            left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
            if int(h[1]) != arm: continue
            ee = [float(x) for x in right.split()]
            if len(ee) != 7: continue
            q = np.array([float(x) for x in parts[1 + arm].split()])
            R, t = kinem_fk(W, Pp, q0, q)
            Rf.append(R); Tf.append(t); Tp.append(np.array(ee[:3])); Rt.append(qR(np.array(ee[3:])))
        Rf = np.array(Rf); Tf = np.array(Tf); Tp = np.array(Tp); Rt = np.array(Rt)
        idx = np.arange(len(Tf))
        def res2(x):
            Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
            pe = (s * (Tf[idx] @ Rg.T) + tg) - (Tp[idx] + np.einsum('nab,b->na', Rt[idx], tx))
            re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
            return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
        best = None; rng = np.random.default_rng(0)
        for k in range(12):
            x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
            r = least_squares(res2, x0, method="lm", max_nfev=4000)
            if best is None or r.cost < best.cost: best = r
        x = best.x
        FITS[arm] = (x[6], rv(x[0:3]), x[7:10])
        rr = res2(x)[:3*len(idx)].reshape(-1,3)
        en = np.linalg.norm(rr, axis=1)*1000
        print("   手 %d:%d 格 · 单位 %.3f mm · 眼离手 (%.1f, %.1f, %.1f) mm · 眼转 %s · 位置残差 中位 %.2f 最大 %.2f mm" % (arm, len(idx), 1000*x[6], *(1000*x[10:13]), np.round(np.degrees(x[3:6]),2), np.median(en), en.max()))
    s0, Rg0, tg0 = FITS[0]; s1, Rg1, tg1 = FITS[1]
    f_ = open(os.path.join(KDIR, "fixed_eye.txt")).read().split()
    fx = float(f_[1]); pos = np.array([float(v) for v in f_[f_.index("pos") + 1:f_.index("pos") + 4]])
    Rce = np.array([float(v) for v in f_[f_.index("R") + 1:f_.index("R") + 10]]).reshape(3, 3)
    pw = s0 * (Rg0 @ pos) + tg0
    Rw = Rg0 @ Rce
    look = -Rw[:, 2]
    h = open(os.path.join(KDIR, "align_arm1.txt")).readline().split()
    S = float(h[1]); R = np.array([float(v) for v in h[3:12]]).reshape(3, 3); T = np.array([float(v) for v in h[13:16]])
    St = s1 / s0; Rt_ = Rg0.T @ Rg1; Tt = Rg0.T @ (tg1 - tg0) / s0
    dT = s0 * (Rg0 @ (T - Tt)) * 1000
    # 第 2 只手参照眼(它系的原点)按驱动的对齐放到世界 vs 真值
    p1d = s0 * (Rg0 @ T) + tg0; p1t = tg1
    ang = math.degrees(np.linalg.norm(logR(R.T @ Rt_)))
    tilt = math.degrees(math.asin(max(-1, min(1, -look[2]))))
    return dict(f=fx, dfx=(pw - TRUE_FX) * 1000, look=look, tilt=tilt, S=S, St=St, dT=dT, ang=ang, s0=s0, p1t=p1t)
for ARG in ([] if os.environ.get("IK2") else sys.argv[1:]):
    RUN, _, KD = ARG.partition(":")
    try:
        d = study(RUN, KD or None)
    except Exception as e:
        print(os.path.basename(RUN), "错:", e); continue
    print("%s  f %.1f  不动的眼差(世界 mm) (%+.1f, %+.1f, %+.1f) |%.1f|  俯角 %.2f°  第2只手 倍数差 %+.2f%%  转 %.2f°  平移差(世界 mm) (%+.1f, %+.1f, %+.1f) |%.1f|  单位 %.2f mm" %
          (os.path.basename(RUN) + ("(" + os.path.basename(KD) + ")" if KD else ""), d['f'], *d['dfx'], np.linalg.norm(d['dfx']), d['tilt'], 100 * (d['S'] / d['St'] - 1), d['ang'], *d['dT'], np.linalg.norm(d['dT']), d['s0'] * 1000))
# ② 用"全部扫描格拟合的"第一只手相似变换再算一遍(打分脚本用一半格子);两种换算差多少 = 这项考试自己的不确定
def ik2(RUN, KDIR, half):
    KDIR = KDIR or os.path.join(RUN, "look")
    rows = [l for l in open(os.path.join(RUN, "look", "sweep.txt")) if "||" in l]
    out = {}
    for arm in range(2):
        L = open(os.path.join(KDIR, "kinem_arm%d.txt" % arm)).read().split("\n")
        q0 = np.array([float(x) for x in L[1].split()[1:]])
        W, Pp = [], []
        for l in L[2:]:
            if l.startswith("axis"):
                v = [float(x) for x in l.split()[2:]]; W.append(v[:3]); Pp.append(v[3:])
        W = np.array(W); Pp = np.array(Pp)
        Rf, Tf, Tp, Rt = [], [], [], []
        for l in rows:
            left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
            if int(h[1]) != arm: continue
            ee = [float(x) for x in right.split()]
            if len(ee) != 7: continue
            q = np.array([float(x) for x in parts[1 + arm].split()])
            R, t = kinem_fk(W, Pp, q0, q)
            Rf.append(R); Tf.append(t); Tp.append(np.array(ee[:3])); Rt.append(qR(np.array(ee[3:])))
        Rf = np.array(Rf); Tf = np.array(Tf); Tp = np.array(Tp); Rt = np.array(Rt)
        idx = np.arange(0, len(Tf), 2) if half else np.arange(len(Tf))
        def res2(x):
            Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
            pe = (s * (Tf[idx] @ Rg.T) + tg) - (Tp[idx] + np.einsum('nab,b->na', Rt[idx], tx))
            re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
            return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
        best = None; rng = np.random.default_rng(0)
        for k in range(12):
            x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
            r = least_squares(res2, x0, method="lm", max_nfev=4000)
            if best is None or r.cost < best.cost: best = r
        x = best.x
        out[arm] = (x[6], rv(x[0:3]), x[7:10], rv(x[3:6]), x[10:13])
    wv = [float(v) for v in open(os.path.join(RUN, "look", "world.txt")).read().split()]; Rw = np.array(wv[:9]).reshape(3, 3); Ow = np.array(wv[9:12])
    s0, Rg0, tg0 = out[0][:3]
    w2s = lambda p: s0 * (Rg0 @ (Rw.T @ p + Ow)) + tg0
    res = {}
    for l in open(os.path.join(RUN, "look", "ik_check.txt")):
        if "||" not in l: continue
        left, right = l.split("||", 1); parts = left.split("|"); h = parts[0].split()
        sw = int(h[1]); tgt = np.array([float(v) for v in parts[1].split()]); tr = np.array([float(v) for v in right.split()])
        cam = tr[:3] + qR(tr[3:]) @ out[sw][4]
        res.setdefault(sw, []).append(1000 * np.linalg.norm(w2s(tgt[:3]) - cam))
    return res, s0
if os.environ.get("IK2"):
    for ARG in sys.argv[1:]:
        RUN = ARG.split(":")[0]
        for half in (True, False):
            r, s0 = ik2(RUN, None, half)
            print("%s ② 按%s格子拟合的换算(单位 %.3f mm):%s" % (os.path.basename(RUN), "一半" if half else "全部", 1000 * s0, " · ".join("第 %d 只手 %s" % (k, " ".join("%.1f" % e for e in v)) for k, v in sorted(r.items()))))
