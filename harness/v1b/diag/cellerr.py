#!/usr/bin/env python3
# 运动学模型逐格误差(按真值拟合相似变换后):哪根关节、哪个方向、第几格,位置差(mm)、朝向差(°)
import sys, os, math
import numpy as np
from scipy.optimize import least_squares
exec(open('/root/diag/alignstudy.py').read().split("TRUE_FX")[0])
RUN = sys.argv[1]; KDIR = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RUN, "look")
rows = [l for l in open(os.path.join(RUN, "look", "sweep.txt")) if "||" in l]
for arm in range(2):
    L = open(os.path.join(KDIR, "kinem_arm%d.txt" % arm)).read().split("\n")
    q0 = np.array([float(x) for x in L[1].split()[1:]])
    W, Pp = [], []
    for l in L[2:]:
        if l.startswith("axis"):
            v = [float(x) for x in l.split()[2:]]; W.append(v[:3]); Pp.append(v[3:])
    W = np.array(W); Pp = np.array(Pp)
    Rf, Tf, Tp, Rt, tags = [], [], [], [], []
    for l in rows:
        left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
        if int(h[1]) != arm: continue
        ee = [float(x) for x in right.split()]
        if len(ee) != 7: continue
        q = np.array([float(x) for x in parts[1 + arm].split()])
        R, t = kinem_fk(W, Pp, q0, q)
        Rf.append(R); Tf.append(t); Tp.append(np.array(ee[:3])); Rt.append(qR(np.array(ee[3:]))); tags.append("%s%s%s" % (h[2], "+" if int(h[3]) > 0 else ("-" if int(h[3]) < 0 else "0"), h[4]))
    Rf = np.array(Rf); Tf = np.array(Tf); Tp = np.array(Tp); Rt = np.array(Rt)
    idx = np.arange(len(Tf))
    def res2(x, wr=0.05):
        Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        pe = (s * (Tf[idx] @ Rg.T) + tg) - (Tp[idx] + np.einsum('nab,b->na', Rt[idx], tx))
        re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
        return np.concatenate([pe.ravel(), wr * re.ravel()])
    best = None; rng = np.random.default_rng(0)
    for k in range(12):
        x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
        r = least_squares(res2, x0, method="lm", max_nfev=4000)
        if best is None or r.cost < best.cost: best = r
    x = best.x
    Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
    pe = ((s * (Tf @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx))) * 1000
    re = np.array([np.degrees(logR((Rt[i] @ Rx).T @ (Rg @ Rf[i]))) for i in idx])
    print("== 手 %d(%s)单位 %.3f mm" % (arm, os.path.basename(KDIR), 1000 * s))
    for i in idx:
        print("  %-5s 位置差 (%+.2f, %+.2f, %+.2f) |%.2f| mm · 朝向差 (%+.3f, %+.3f, %+.3f)° · 眼离起点 %.0f mm" % (tags[i], *pe[i], np.linalg.norm(pe[i]), *re[i], 1000 * s * np.linalg.norm(Tf[i])))
