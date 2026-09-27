#!/usr/bin/env python3
# 扫描配点按真几何的对极误差(Sampson,像素):真几何 = 仿真报的手腕位姿 × 相机安装(第二只手按真值拟合的,两只手同型号);焦距 397
import sys, os, math
import numpy as np
from scipy.optimize import least_squares
exec(open('/root/diag/alignstudy.py').read().split("TRUE_FX")[0])
RUN = sys.argv[1]
rows = [l for l in open(os.path.join(RUN, "look", "sweep.txt")) if "||" in l]
def load_arm(arm):
    L = open(os.path.join(RUN, "look", "kinem_arm%d.txt" % arm)).read().split("\n")
    f = float(L[0].split()[5])
    q0 = np.array([float(x) for x in L[1].split()[1:]])
    W, Pp = [], []
    for l in L[2:]:
        if l.startswith("axis"):
            v = [float(x) for x in l.split()[2:]]; W.append(v[:3]); Pp.append(v[3:])
    W = np.array(W); Pp = np.array(Pp)
    Q, EE, tags = [], [], []
    for l in rows:
        left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
        if int(h[1]) != arm: continue
        Q.append(np.array([float(x) for x in parts[1 + arm].split()])); EE.append(np.array([float(x) for x in right.split()]))
        tags.append("%s%s%s" % (h[2], "+" if int(h[3]) > 0 else ("-" if int(h[3]) < 0 else "0"), h[4]))
    return f, q0, W, Pp, Q, EE, tags
def fit_mount(arm):
    f, q0, W, Pp, Q, EE, tags = load_arm(arm)
    Tf = []; Rf = []
    for q in Q:
        R, t = kinem_fk(W, Pp, q0, q); Rf.append(R); Tf.append(t)
    Tf = np.array(Tf); Rf = np.array(Rf); Tp = np.array([e[:3] for e in EE]); Rt = np.array([qR(e[3:]) for e in EE])
    idx = np.arange(len(Tf))
    def res2(x):
        Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        pe = (s * (Tf @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx))
        re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
        return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
    best = None; rng = np.random.default_rng(0)
    for k in range(12):
        x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
        r = least_squares(res2, x0, method="lm", max_nfev=4000)
        if best is None or r.cost < best.cost: best = r
    return rv(best.x[3:6]), best.x[10:13]
Rx1, tx1 = fit_mount(1)   # 第二只手的模型准(按真值 0.04 / 0.19 mm)⇒ 当两只手共同的相机安装
def samp(Rrel, trel, d1, d2, f):
    # d = 视线方向(相机系,-z 朝前);d2ᵀ [t]x R d1 = 0
    E = np.cross(trel[None, :], (d1 @ Rrel.T))          # [t]x R d1 逐行
    num = np.einsum('ij,ij->i', d2, E)
    # 按像素的一阶近似:∂/∂(u,v) 两边
    Ex1 = E; Etx2 = np.cross(d2, trel[None, :]) @ Rrel   # (Eᵀ d2)
    den = (Ex1[:, 0] ** 2 + Ex1[:, 1] ** 2 + Etx2[:, 0] ** 2 + Etx2[:, 1] ** 2) / (f * f)
    return num / np.sqrt(np.maximum(den, 1e-30)) / f * f / f
for arm in (0, 1):
    f, q0, W, Pp, Q, EE, tags = load_arm(arm)
    C = np.loadtxt(os.path.join(RUN, "look", "corrs_arm%d.txt" % arm))
    I = C[:, 0].astype(int); J = C[:, 1].astype(int)
    def dirs(u, v, ff):
        return np.stack([(u - 320.0) / ff, -(v - 240.0) / ff, -np.ones(len(u))], axis=1)
    mod = np.zeros(len(C)); tru = np.zeros(len(C))
    for (i, j) in sorted(set(zip(I.tolist(), J.tolist()))):
        m = (I == i) & (J == j)
        # 模型
        Ri, ti = kinem_fk(W, Pp, q0, Q[i]); Rj, tj = kinem_fk(W, Pp, q0, Q[j])
        Rr = Rj.T @ Ri; tr_ = Rj.T @ (ti - tj)
        mod[m] = samp(Rr, tr_, dirs(C[m, 2], C[m, 3], f), dirs(C[m, 4], C[m, 5], f), f) * f
        # 真几何
        Rci = qR(EE[i][3:]) @ Rx1; pci = EE[i][:3] + qR(EE[i][3:]) @ tx1
        Rcj = qR(EE[j][3:]) @ Rx1; pcj = EE[j][:3] + qR(EE[j][3:]) @ tx1
        Rr = Rcj.T @ Rci; tr_ = Rcj.T @ (pci - pcj)
        tru[m] = samp(Rr, tr_, dirs(C[m, 2], C[m, 3], 397.0), dirs(C[m, 4], C[m, 5], 397.0), 397.0) * 397.0
    am, at = np.abs(mod), np.abs(tru)
    print("== 手 %d:配点 %d 条 · 按模型 中位 %.3f px、<0.5px %.0f%% · 按真几何 中位 %.3f px、<0.5px %.0f%%" % (arm, len(C), np.median(am), 100 * np.mean(am < 0.5), np.median(at), 100 * np.mean(at < 0.5)))
    # 按格子对:真几何下的中位 / 模型下的中位
    out = []
    for (i, j) in sorted(set(zip(I.tolist(), J.tolist()))):
        m = (I == i) & (J == j)
        out.append((np.median(at[m]), np.median(am[m]), i, j))
    out.sort(reverse=True)
    print("   真几何下最差的 8 对(真 / 模型 中位 px):" + " · ".join("%s-%s %.2f/%.2f" % (tags[i], tags[j], a, b) for a, b, i, j in out[:8]))
    # 按画面区域(起点那一格 0-k 的对,按 cell-0 像素分 4×4 格)
    m0 = I == 0
    gx = np.clip((C[m0, 2] / 160).astype(int), 0, 3); gy = np.clip((C[m0, 3] / 120).astype(int), 0, 3)
    grid = np.full((4, 4), np.nan)
    for a in range(4):
        for b in range(4):
            mm = (gx == a) & (gy == b)
            if mm.sum() > 50: grid[b, a] = np.median(at[m0][mm])
    print("   起点那一格按画面 4×4 格,真几何下对极误差中位(px,行 = 上→下):")
    for b in range(4):
        print("     " + "  ".join("%.2f" % v if not np.isnan(v) else "  - " for v in grid[b]))
