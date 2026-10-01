#!/usr/bin/env python3
# 同一批跨格轨迹:按 (a) 驱动的模型、(b) 真几何(仿真手腕位姿 × 相机安装、焦距 397)各自只解每点远近,比重投影误差
import sys, os, math
import numpy as np
from scipy.optimize import least_squares
exec(open('/root/diag/alignstudy.py').read().split("TRUE_FX")[0])
RUN = sys.argv[1]; ARM = int(sys.argv[2]); MOUNT_ARM = int(sys.argv[3]) if len(sys.argv) > 3 else 1
LOOK = os.path.join(RUN, "look")
rows = [l for l in open(os.path.join(LOOK, "sweep.txt")) if "||" in l]
def load(arm):
    L = open(os.path.join(LOOK, "kinem_arm%d.txt" % arm)).read().split("\n")
    f0 = float(L[0].split()[5]); q0 = np.array([float(x) for x in L[1].split()[1:]])
    W0, P0 = [], []
    for l in L[2:]:
        if l.startswith("axis"):
            v = [float(x) for x in l.split()[2:]]; W0.append(v[:3]); P0.append(v[3:])
    Q, EE, TAG = [], [], []
    for l in rows:
        left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
        if int(h[1]) != arm: continue
        Q.append(np.array([float(x) for x in parts[1 + arm].split()])); EE.append(np.array([float(x) for x in right.split()]))
        TAG.append("%s%s%s" % (h[2], "+" if int(h[3]) > 0 else ("-" if int(h[3]) < 0 else "0"), h[4]))
    return f0, q0, np.array(W0), np.array(P0), Q, EE, TAG
def mount(arm):
    f0, q0, W0, P0, Q, EE, TAG = load(arm)
    Rf, Tf = [], []
    for q in Q:
        R, t = kinem_fk(W0, P0, q0, q); Rf.append(R); Tf.append(t)
    Rf = np.array(Rf); Tf = np.array(Tf); Tp = np.array([e[:3] for e in EE]); Rt = np.array([qR(e[3:]) for e in EE]); idx = np.arange(len(Q))
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
f0, q0, W0, P0, Q, EE, TAG = load(ARM)
Rx, tx = mount(MOUNT_ARM)
Z = np.load(os.path.join(RUN, "mv_arm%d_32x24.npz" % ARM)); OBS = Z["obs"]; GRID = Z["grid"]
pi = OBS[:, 0].astype(int); pk = OBS[:, 1].astype(int); puv = OBS[:, 2:4]
def poses_model():
    Rs, Ts = [], []
    for q in Q:
        R, t = kinem_fk(W0, P0, q0, q); Rs.append(R); Ts.append(t)
    return np.array(Rs), np.array(Ts)
def poses_true():
    Rc = [qR(e[3:]) @ Rx for e in EE]; pc = [e[:3] + qR(e[3:]) @ tx for e in EE]
    R0 = Rc[0]; p0 = pc[0]
    return np.array([R0.T @ R for R in Rc]), np.array([R0.T @ (p - p0) for p in pc])
def fit_depths(Rs, Ts, f):
    # 每点:起点那格的像素是查询点(精确)⇒ 点在它的视线上;远近按其余各格重投影一维解(抗野点)
    D = np.stack([(GRID[:, 0] - 320.0) / f, -(GRID[:, 1] - 240.0) / f, -np.ones(len(GRID))], axis=1)
    errs = []
    for i in range(len(GRID)):
        m = pi == i
        if m.sum() < 3: continue
        ks = pk[m]; uv = puv[m]
        def r(lz):
            X = math.exp(lz[0]) * D[i]
            Xc = np.einsum('nji,j->ni', Rs[ks], X[None, :].repeat(len(ks), 0)[0] - Ts[ks]) if False else np.array([Rs[k].T @ (X - Ts[k]) for k in ks])
            z = -Xc[:, 2]
            return np.concatenate([f * Xc[:, 0] / z + 320.0 - uv[:, 0], -f * Xc[:, 1] / z + 240.0 - uv[:, 1]])
        best = None
        for lz in np.log([0.3, 1.0, 3.0, 10.0]) + math.log(np.sqrt(np.mean(np.sum(Ts ** 2, axis=1)))):
            s = least_squares(r, [lz], loss="soft_l1", f_scale=0.5)
            if best is None or s.cost < best.cost: best = s
        rr = best.fun; e = np.hypot(rr[:len(ks)], rr[len(ks):])
        errs.extend(e.tolist())
    return np.array(errs)
for nm, (Rs, Ts), f in (("驱动的模型", poses_model(), f0), ("真几何(安装按第 %d 只手)" % MOUNT_ARM, poses_true(), 397.0)):
    e = fit_depths(Rs, Ts, f)
    print("手 %d · %s:每点只解远近 ⇒ 重投影 中位 %.3f px、九成 %.3f px、<0.5 px %.0f%%(%d 笔)" % (ARM, nm, np.median(e), np.quantile(e, 0.9), 100 * np.mean(e < 0.5), len(e)))
