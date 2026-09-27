#!/usr/bin/env python3
# 第一只手三角出、配进不动的眼的点:误差(仿真里 − 真视线交真桌面)按"三角它的另一格"分
import sys, os, math
import numpy as np
sys.argv_saved = list(sys.argv)
exec(open('/root/diag/fxstudy.py').read().split("# 真相机投第一只手的点")[0])
tri = a0[:, 15].astype(int)
d = ray(Rtrue, ptrue, ftrue, uv0); lam = (zt - ptrue[2]) / d[:, 2]; Xt = ptrue + lam[:, None] * d
e = (X0s - Xt) * 1000
rows = [l for l in open(os.path.join(RUN, "look", "sweep.txt")) if "||" in l]
tag = {}
k = 0
for l in rows:
    h = l.split("||")[0].split("|")[0].split()
    if int(h[1]) == 0:
        tag[k] = "%s%s%s" % (h[2], "+" if int(h[3]) > 0 else ("-" if int(h[3]) < 0 else "0"), h[4]); k += 1
# 眼离起点(模型单位 → mm)
L = open(os.path.join(RUN, "look", "kinem_arm0.txt")).read().split("\n")
q0 = np.array([float(x) for x in L[1].split()[1:]])
W, Pp = [], []
for l in L[2:]:
    if l.startswith("axis"):
        v = [float(x) for x in l.split()[2:]]; W.append(v[:3]); Pp.append(v[3:])
W = np.array(W); Pp = np.array(Pp)
qs = []
for l in rows:
    left = l.split("||")[0]; parts = left.split("|"); h = parts[0].split()
    if int(h[1]) == 0: qs.append(np.array([float(x) for x in parts[1].split()]))
print("另一格  点数  平均误差(x, y, z)mm   |平均|  中位|e|  基线 mm")
for fr in sorted(set(tri.tolist())):
    m = on & (tri == fr)
    if m.sum() < 8: continue
    R, t = kinem_fk(W, Pp, q0, qs[fr])
    b = 1000 * s0 * np.linalg.norm(t)
    em = e[m].mean(axis=0)
    print("%-6s %5d  (%+.2f, %+.2f, %+.2f)  %.2f  %.2f  %.0f" % (tag.get(fr, str(fr)), m.sum(), *em, np.linalg.norm(em), np.median(np.linalg.norm(e[m], axis=1)), b))
