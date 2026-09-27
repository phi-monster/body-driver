#!/usr/bin/env python3
# 对照:每一格的位姿全放开(相对起点那格 6 个数)+ 每点沿起点视线的远近,按同一批轨迹重投影解;焦距 F(给 = 钉住,不给 = 一起解)
import sys, os, math, time
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
exec(open('/root/diag/alignstudy.py').read().split("TRUE_FX")[0])
RUN = sys.argv[1]; ARM = int(sys.argv[2]); FPIN = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
LOOK = os.path.join(RUN, "look")
rows = [l for l in open(os.path.join(LOOK, "sweep.txt")) if "||" in l]
L = open(os.path.join(LOOK, "kinem_arm%d.txt" % ARM)).read().split("\n")
f0 = float(L[0].split()[5]); q0 = np.array([float(x) for x in L[1].split()[1:]])
W0, P0 = [], []
for l in L[2:]:
    if l.startswith("axis"):
        v = [float(x) for x in l.split()[2:]]; W0.append(v[:3]); P0.append(v[3:])
W0 = np.array(W0); P0 = np.array(P0)
Q, EE, TAG = [], [], []
for l in rows:
    left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
    if int(h[1]) != ARM: continue
    Q.append(np.array([float(x) for x in parts[1 + ARM].split()])); EE.append(np.array([float(x) for x in right.split()]))
    TAG.append("%s%s%s" % (h[2], "+" if int(h[3]) > 0 else ("-" if int(h[3]) < 0 else "0"), h[4]))
NF = len(Q)
Z = np.load(os.path.join(RUN, "mv_arm%d_32x24.npz" % ARM)); OBS = Z["obs"]; GRID = Z["grid"]
pi = OBS[:, 0].astype(int); pk = OBS[:, 1].astype(int); puv = OBS[:, 2:4]
cnt = np.bincount(pi, minlength=len(GRID)); keep_pt = np.where(cnt >= 2)[0]
remap = -np.ones(len(GRID), int); remap[keep_pt] = np.arange(len(keep_pt))
m = remap[pi] >= 0; pi = remap[pi[m]]; pk = pk[m]; puv = puv[m]; NP = len(keep_pt); G0 = GRID[keep_pt]; no = len(pi)
# 起步:驱动模型的每格位姿;远近按它三角
Rs0, Ts0 = [], []
for q in Q:
    R, t = kinem_fk(W0, P0, q0, q); Rs0.append(R); Ts0.append(t)
Rs0 = np.array(Rs0); Ts0 = np.array(Ts0)
def d0(f):
    return np.stack([(G0[:, 0] - 320.0) / f, -(G0[:, 1] - 240.0) / f, -np.ones(NP)], axis=1)
def tri(Rs, Ts, f):
    D = d0(f); num = np.zeros(NP); den = np.zeros(NP)
    for n in range(no):
        i, k = pi[n], pk[n]; a = Rs[k].T @ D[i]; b = -Rs[k].T @ Ts[k]
        u = (puv[n, 0] - 320.0) / f; v = -(puv[n, 1] - 240.0) / f
        c1 = np.array([a[0] + u * a[2], a[1] + v * a[2]]); c0 = np.array([b[0] + u * b[2], b[1] + v * b[2]])
        num[i] += -(c1 @ c0); den[i] += c1 @ c1
    return num / np.maximum(den, 1e-12)
lam0 = tri(Rs0, Ts0, f0); good = lam0 > 1e-3
# 只留起步远近合理的点(在眼前面)
sel = good[pi]; pi2 = pi[sel]; pk2 = pk[sel]; puv2 = puv[sel]
keep2 = np.where(good)[0]; rm2 = -np.ones(NP, int); rm2[keep2] = np.arange(len(keep2))
pi2 = rm2[pi2]; G2 = G0[keep2]; NP2 = len(keep2); no2 = len(pi2); lam0 = lam0[keep2]
def d02(f):
    return np.stack([(G2[:, 0] - 320.0) / f, -(G2[:, 1] - 240.0) / f, -np.ones(NP2)], axis=1)
NPOSE = NF - 1
def unpack(x):
    rvs = x[:3 * NPOSE].reshape(NPOSE, 3); ts = x[3 * NPOSE:6 * NPOSE].reshape(NPOSE, 3)
    f = FPIN if FPIN > 0 else math.exp(x[6 * NPOSE]); lam = np.exp(x[6 * NPOSE + 1:])
    Rs = np.concatenate([np.eye(3)[None], np.array([rv(r) for r in rvs])]); Ts = np.concatenate([np.zeros((1, 3)), ts])
    return Rs, Ts, f, lam
def resid(x):
    Rs, Ts, f, lam = unpack(x)
    X = lam[:, None] * d02(f)
    Xc = np.einsum('nji,nj->ni', Rs[pk2], X[pi2] - Ts[pk2])
    z = -Xc[:, 2]
    ru = f * Xc[:, 0] / z + 320.0 - puv2[:, 0]; rvv = -f * Xc[:, 1] / z + 240.0 - puv2[:, 1]
    return np.concatenate([ru, rvv, [1e3 * (math.sqrt(np.mean(np.sum(Ts[1:] ** 2, axis=1))) - math.sqrt(np.mean(np.sum(Ts0[1:] ** 2, axis=1))))]])
x0 = np.concatenate([np.array([logR(R) for R in Rs0[1:]]).ravel(), Ts0[1:].ravel(), [math.log(f0)], np.log(lam0)])
S = lil_matrix((2 * no2 + 1, len(x0)), dtype=np.int8)
for n in range(no2):
    k = pk2[n] - 1
    for rr in (n, no2 + n):
        S[rr, 3 * k:3 * k + 3] = 1; S[rr, 3 * NPOSE + 3 * k:3 * NPOSE + 3 * k + 3] = 1; S[rr, 6 * NPOSE] = 1; S[rr, 6 * NPOSE + 1 + pi2[n]] = 1
S[2 * no2, 3 * NPOSE:6 * NPOSE] = 1
r0 = resid(x0)
t0 = time.time()
sol = least_squares(resid, x0, jac_sparsity=S, loss="soft_l1", f_scale=0.5, max_nfev=400, diff_step=1e-6)
e0 = np.hypot(r0[:no2], r0[no2:2 * no2]); e1 = np.hypot(sol.fun[:no2], sol.fun[no2:2 * no2])
print("手 %d 自由位姿:%d 格 × %d 点、%d 笔 · 起步重投影 中位 %.3f px → 解完 %.3f px(%s,%.0f 秒)" % (ARM, NF, NP2, no2, np.median(e0), np.median(e1), sol.message[:40], time.time() - t0))
Rs, Ts, f, lam = unpack(sol.x)
def grade(Rf, Tf):
    Tp = np.array([e[:3] for e in EE]); Rt = np.array([qR(e[3:]) for e in EE]); idx = np.arange(NF)
    def res2(x):
        Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        pe = (s * (Tf @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx))
        re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
        return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
    best = None; rng = np.random.default_rng(0)
    for k in range(12):
        x0_ = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
        r = least_squares(res2, x0_, method="lm", max_nfev=4000)
        if best is None or r.cost < best.cost: best = r
    x = best.x; s = x[6]; Rg = rv(x[0:3]); tg = x[7:10]; tx = x[10:13]; Rx = rv(x[3:6])
    pe = np.linalg.norm((s * (Tf @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx)), axis=1) * 1000
    ang = np.array([np.degrees(np.linalg.norm(logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])))) for i in idx])
    return s, pe, ang, Rx
for nm, (Rf, Tf, ff) in (("驱动的模型(6 根轴)", (Rs0, Ts0, f0)), ("每格位姿全放开", (Rs, Ts, f))):
    s, pe, ang, Rx = grade(Rf, Tf)
    print("  %s:焦距 %.1f · 按真值 位置 中位 %.2f / 最大 %.2f mm · 朝向 中位 %.3f / 最大 %.3f° · 眼转 %s" % (nm, ff, np.median(pe), pe.max(), np.median(ang), ang.max(), np.round(np.degrees(logR(Rx)), 2)))
    worst = np.argsort(-pe)[:6]
    print("     最差 6 格:" + " · ".join("%s %.2f" % (TAG[i], pe[i]) for i in worst))
