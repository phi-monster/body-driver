#!/usr/bin/env python3
# 多视图一起解(变量投影):每点沿起点视线的远近随模型当场解掉(抗野点的一维高斯牛顿),外层只对运动学 37 个数做 LM(数值差分)
# 用法:vpba.py <跑的目录> <手> [外层轮数]
import sys, os, math, time
import numpy as np
from scipy.optimize import least_squares
exec(open('/root/diag/alignstudy.py').read().split("TRUE_FX")[0])
RUN = sys.argv[1]; ARM = int(sys.argv[2]); NIT = int(sys.argv[3]) if len(sys.argv) > 3 else 60
LOOK = os.path.join(RUN, "look")
rows = [l for l in open(os.path.join(LOOK, "sweep.txt")) if "||" in l]
L = open(os.path.join(LOOK, "kinem_arm%d.txt" % ARM)).read().split("\n")
f0 = float(L[0].split()[5]); q0 = np.array([float(x) for x in L[1].split()[1:]])
W0, P0 = [], []
for l in L[2:]:
    if l.startswith("axis"):
        v = [float(x) for x in l.split()[2:]]; W0.append(v[:3]); P0.append(v[3:])
W0 = np.array(W0); P0 = np.array(P0); NJ = len(W0)
Q, EE, TAG = [], [], []
for l in rows:
    left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
    if int(h[1]) != ARM: continue
    Q.append(np.array([float(x) for x in parts[1 + ARM].split()])); EE.append(np.array([float(x) for x in right.split()]))
    TAG.append("%s%s%s" % (h[2], "+" if int(h[3]) > 0 else ("-" if int(h[3]) < 0 else "0"), h[4]))
Q = np.array(Q); NF = len(Q)
Z = np.load(os.path.join(RUN, "mv_arm%d_32x24.npz" % ARM)); OBS = Z["obs"]; GRID = Z["grid"]
pi = OBS[:, 0].astype(int); pk = OBS[:, 1].astype(int); puv = OBS[:, 2:4]
def fk_all(W, P):
    Rs = np.zeros((NF, 3, 3)); Ts = np.zeros((NF, 3))
    for k in range(NF):
        R, t = kinem_fk(W, P, q0, Q[k]); Rs[k] = R; Ts[k] = t
    return Rs, Ts
def dirs(f):
    return np.stack([(GRID[:, 0] - 320.0) / f, -(GRID[:, 1] - 240.0) / f, -np.ones(len(GRID))], axis=1)
TAU = 0.5   # 抗野点尺度(像素):soft-l1
def solve_depth(Rs, Ts, f, lz, iters):
    D = dirs(f)
    a = np.einsum('nji,nj->ni', Rs[pk], D[pi])            # R_kᵀ d
    b = -np.einsum('nji,nj->ni', Rs[pk], Ts[pk])          # −R_kᵀ t
    for _ in range(iters):
        lam = np.exp(lz)[pi]
        Xc = lam[:, None] * a + b; z = -Xc[:, 2]
        ru = f * Xc[:, 0] / z + 320.0 - puv[:, 0]; rv_ = -f * Xc[:, 1] / z + 240.0 - puv[:, 1]
        # d/dlog λ = λ d/dλ
        dXc = lam[:, None] * a
        du = f * (dXc[:, 0] * z + Xc[:, 0] * dXc[:, 2]) / z ** 2
        dv = -f * (dXc[:, 1] * z + Xc[:, 1] * dXc[:, 2]) / z ** 2
        e2 = ru ** 2 + rv_ ** 2
        w = 1.0 / np.sqrt(1.0 + e2 / TAU ** 2)             # soft-l1 的 IRLS 权
        g = np.bincount(pi, weights=w * (du * ru + dv * rv_), minlength=len(GRID))
        H = np.bincount(pi, weights=w * (du * du + dv * dv), minlength=len(GRID))
        step = np.where(H > 1e-12, -g / np.maximum(H, 1e-12), 0.0)
        lz = lz + np.clip(step, -0.5, 0.5)
    lam = np.exp(lz)[pi]
    Xc = lam[:, None] * a + b; z = -Xc[:, 2]
    ru = f * Xc[:, 0] / z + 320.0 - puv[:, 0]; rv_ = -f * Xc[:, 1] / z + 240.0 - puv[:, 1]
    return lz, ru, rv_, z
def unpack(x):
    return x[:3 * NJ].reshape(NJ, 3), x[3 * NJ:6 * NJ].reshape(NJ, 3), math.exp(x[6 * NJ])
def resid(x, lz, iters=3):
    W, P, f = unpack(x)
    Rs, Ts = fk_all(W, P)
    lz2, ru, rv_, z = solve_depth(Rs, Ts, f, lz, iters)
    e = np.hypot(ru, rv_)
    w = np.sqrt(1.0 / np.sqrt(1.0 + e ** 2 / TAU ** 2))   # soft-l1 ⇒ 每笔乘 √权(IRLS)
    reg = [1e3 * (np.linalg.norm(W[j]) - 1.0) for j in range(NJ)] + [1e3 * (W[j] @ P[j]) for j in range(NJ)] + [1e3 * (math.sqrt(np.mean(np.sum(Ts ** 2, axis=1))) - SC)]
    return np.concatenate([w * ru, w * rv_, reg]), lz2, e, z
x = np.concatenate([W0.ravel(), P0.ravel(), [math.log(f0)]])
Rs0, Ts0 = fk_all(W0, P0); SC = math.sqrt(np.mean(np.sum(Ts0 ** 2, axis=1)))
lz = np.full(len(GRID), math.log(3.0 * SC))
lz, ru, rv_, z = solve_depth(Rs0, Ts0, f0, lz, 30)
good = (np.bincount(pi, weights=(z > 0).astype(float), minlength=len(GRID)) == np.bincount(pi, minlength=len(GRID)))
m = good[pi]; pi = pi[m]; pk = pk[m]; puv = puv[m]
r, lz, e, z = resid(x, lz, 5)
print("手 %d:%d 笔、%d 个点 · 起步(驱动的模型)重投影 中位 %.3f px、九成 %.3f px" % (ARM, len(pi), len(set(pi.tolist())), np.median(e), np.quantile(e, 0.9)))
t0 = time.time(); mu = 1e-3
cost = np.sum(r ** 2)
for it in range(NIT):
    # 数值雅可比(每个参数一次,远近随之当场重解)
    J = np.zeros((len(r), len(x)))
    for j in range(len(x)):
        h = 1e-6 * max(1.0, abs(x[j]))
        xp = x.copy(); xp[j] += h
        J[:, j] = (resid(xp, lz, 2)[0] - r) / h
    A = J.T @ J; g = J.T @ r
    while True:
        dx = -np.linalg.solve(A + mu * np.diag(np.diag(A) + 1e-12), g)
        r2, lz2, e2, _ = resid(x + dx, lz, 4)
        c2 = np.sum(r2 ** 2)
        if c2 < cost:
            x = x + dx; r = r2; lz = lz2; e = e2; cost = c2; mu = max(mu / 3, 1e-9); break
        mu *= 5
        if mu > 1e6: break
    if mu > 1e6 or np.linalg.norm(dx) < 1e-10: break
    if it % 10 == 0:
        print("  第 %d 轮:重投影 中位 %.4f px、九成 %.4f px(%.0f 秒)" % (it, np.median(e), np.quantile(e, 0.9), time.time() - t0))
print("解完(%d 轮,%.0f 秒):重投影 中位 %.4f px、九成 %.4f px" % (it + 1, time.time() - t0, np.median(e), np.quantile(e, 0.9)))
W1, P1, f1 = unpack(x)
out = os.path.join(RUN, "vp"); os.makedirs(out, exist_ok=True)
with open(os.path.join(out, "kinem_arm%d.txt" % ARM), "w") as fo:
    fo.write("arm %d n %d f %.6f cx 320.000 cy 240.000\n" % (ARM, NJ, f1))
    fo.write(L[1] + "\n")
    for j in range(NJ):
        w_ = W1[j] / np.linalg.norm(W1[j])
        fo.write("axis %d %.9f %.9f %.9f %.9f %.9f %.9f\n" % (j, *w_, *P1[j]))
def grade(W, P):
    Rf, Tf = fk_all(W, P)
    Tp = np.array([e_[:3] for e_ in EE]); Rt = np.array([qR(e_[3:]) for e_ in EE]); idx = np.arange(NF)
    def res2(x_):
        Rg = rv(x_[0:3]); Rx = rv(x_[3:6]); s = x_[6]; tg = x_[7:10]; tx = x_[10:13]
        pe = (s * (Tf @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx))
        re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
        return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
    best = None; rng = np.random.default_rng(0)
    for k in range(12):
        x0_ = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
        rr = least_squares(res2, x0_, method="lm", max_nfev=4000)
        if best is None or rr.cost < best.cost: best = rr
    x_ = best.x; s = x_[6]; Rg = rv(x_[0:3]); tg = x_[7:10]; tx = x_[10:13]; Rx = rv(x_[3:6])
    pe = np.linalg.norm((s * (Tf @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx)), axis=1) * 1000
    ang = np.array([np.degrees(np.linalg.norm(logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])))) for i in idx])
    return s, pe, ang, Rx
for nm, (W, P, f) in (("驱动的模型", (W0, P0, f0)), ("多视图一起解", (W1, P1, f1))):
    s, pe, ang, Rx = grade(W, P)
    worst = np.argsort(-pe)[:5]
    print("%s:焦距 %.2f · 按真值 位置 中位 %.2f / 最大 %.2f mm · 朝向 中位 %.3f / 最大 %.3f° · 单位 %.3f mm · 眼转 %s · 最差 %s" %
          (nm, f, np.median(pe), pe.max(), np.median(ang), ang.max(), 1000 * s, np.round(np.degrees(logR(Rx)), 2), " ".join("%s %.2f" % (TAG[i], pe[i]) for i in worst)))
