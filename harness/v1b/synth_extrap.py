# 外推考试:拟合只用关节在 ±range 里的停,考试停的关节在 ±30°(驱动真要伸手够东西时的样子)
import math, io, contextlib, numpy as np
import v1b_fit as V
import synth_v1b as Sy

def run(range_deg, test_deg, px, rn, nfit=30, ntest=15, seed=1, f=397.0, depth=0.35, inl=400):
    r = np.random.default_rng(seed)
    n = 6
    dfit = r.uniform(-1, 1, (nfit, n)) * math.radians(range_deg); dfit[0] = 0
    dtest = r.uniform(-1, 1, (ntest, n)) * math.radians(test_deg)
    dQ = np.vstack([dfit, dtest])
    poses = [V.fk(Sy.W_true, Sy.P_true, dq) for dq in dQ]
    train = list(range(nfit)); test = list(range(nfit, nfit + ntest))
    meas = []
    for a, i in enumerate(train):
        for j in train[a + 1:]:
            Ri, ti = poses[i]; Rj, tj = poses[j]
            R = Rj.T @ Ri; t = Rj.T @ (ti - tj)
            base = np.linalg.norm(t); par_rad = base / depth
            w = r.normal(size=3); w /= np.linalg.norm(w)
            Rn = V.rot(w, math.radians(r.normal() * rn)) @ R
            sd = (px / f) / max(par_rad, 1e-4) / math.sqrt(inl)
            d = t / base + r.normal(size=3) * sd; d /= np.linalg.norm(d)
            meas.append(dict(i=i, j=j, R=Rn, t=d, ninl=inl, par=math.degrees(par_rad)))
    Rx = V.rot(np.array([0.3, -0.2, 1.0]), 0.7); tx = np.array([0.0, -0.085, 0.051])
    Pw = []
    for Rk, tk in poses:
        Rc = Sy.R0 @ Rk; pc = Sy.R0 @ tk + Sy.c0
        Ree = Rc @ Rx.T; Pw.append((pc - Ree @ tx, Ree))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        res = V.fit_arm(dQ, meas, train, test, Pw, n, r)
    return res

for rg in (3, 5, 10, 20):
    for px, rn in ((0.5, 0.02), (1.0, 0.05), (2.0, 0.1)):
        res = run(rg, 30, px, rn)
        print("拟合关节 ±%2d° → 考试 ±30° · 像素噪声 %.1f px · 转动噪声 %.2f° ⇒ 眼的位置误差 中位 %6.2f mm、最大 %6.2f mm" % (rg, px, rn, res["test_med_mm"], res["test_max_mm"]), flush=True)
