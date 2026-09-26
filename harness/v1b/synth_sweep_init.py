# 合成考试:新起步(不借焦距)→ 各轴倍数 → 按像素一起解。数据同 synth_sweep_px:板 15 停(±5°)+ 每个关节两个方向扫开到 ±SW°;
# 配点加 10% 乱配(模拟野点);考试 = 全关节同时 ±30° 的姿势。
import math, io, contextlib, sys, os, time
import numpy as np
import v1b_fit as V
import synth_v1b as Sy
import sweep_init as SI

def make(sw_deg, px=0.5, f_true=397.0, seed=3, ntest=40, outl=0.10):
    r = np.random.default_rng(seed)
    n = 6; cx, cy, W_, H_ = 320.0, 240.0, 640, 480
    board = r.uniform(-1, 1, (15, n)) * math.radians(5)
    steps = np.array([2, 4, 8, 14, 20, 28, 36, 45], float) / 45 * sw_deg
    rows = [np.zeros(n)]; meta = [(-1, 0, 0)]
    for j in range(n):
        for d in (-1, 1):
            for k, a in enumerate(steps):
                q = np.zeros(n); q[j] = d * math.radians(a); rows.append(q); meta.append((j, d, k + 1))
    sw = np.array(rows)
    dQ_tr = np.vstack([sw, board]); meta = meta + [(-2, 0, 0)] * 15     # 第 0 帧 = 扫描起点(参照)
    dQ_te = r.uniform(-1, 1, (ntest, n)) * math.radians(30)
    dQ = np.vstack([dQ_tr, dQ_te]); ntr = len(dQ_tr)
    R, t = V.fk_all(Sy.W_true, Sy.P_true, dQ)
    Xw = np.c_[r.uniform(0.2, 1.4, 6000), r.uniform(-0.8, 0.8, 6000), np.zeros(6000)]
    X0 = (Xw - Sy.c0) @ Sy.R0
    def proj(k):
        Xk = (X0 - t[k]) @ R[k]
        z = Xk[:, 2]
        u = f_true * Xk[:, 0] / np.maximum(z, 1e-9) + cx; v = f_true * Xk[:, 1] / np.maximum(z, 1e-9) + cy
        ok = (z > 0.05) & (u >= 0) & (u < W_) & (v >= 0) & (v < 270)
        return np.c_[u, v] + r.normal(0, px, (len(u), 2)), ok
    uv = [proj(k) for k in range(ntr)]
    pairs = set()
    for i in range(ntr):
        dd = sorted(((np.max(np.abs(dQ[i] - dQ[j])), j) for j in range(ntr) if j != i))
        for _, j in dd[:6]:
            pairs.add((min(i, j), max(i, j)))
    prev = {}
    for i in range(1, 1 + 12 * len(steps)):
        key = meta[i][:2]; a_ = prev.get(key, 0)
        pairs.add((min(a_, i), max(a_, i))); pairs.add((0, i)); prev[key] = i
    P = []
    for i, j in sorted(pairs):
        ok = uv[i][1] & uv[j][1]
        if ok.sum() < 60:
            continue
        A = uv[i][0][ok]; B = uv[j][0][ok]
        if len(A) > 800:
            sel = r.choice(len(A), 800, replace=False); A = A[sel]; B = B[sel]
        no = int(outl * len(A))
        B[:no] = np.c_[r.uniform(0, W_, no), r.uniform(0, 270, no)]
        P.append((i, j, A, B))
    Rx = V.rot(np.array([0.3, -0.2, 1.0]), 0.7); tx = np.array([0.0, -0.085, 0.051])
    Pw = []
    for Rk, tk in zip(R, t):
        Rc = Sy.R0 @ Rk; pc = Sy.R0 @ tk + Sy.c0
        Ree = Rc @ Rx.T; Pw.append((pc - Ree @ tx, Ree))
    return dict(dQ=dQ, ntr=ntr, meta=meta, pairs=P, Pw=Pw, cx=cx, cy=cy, W=W_, test=list(range(ntr, ntr + ntest)), n=n)

def run(sw_deg, seed=3):
    D = make(sw_deg, seed=seed)
    dQ, ntr, meta, pairs, n, cx, cy = D["dQ"], D["ntr"], D["meta"], D["pairs"], D["n"], D["cx"], D["cy"]
    t0 = time.time()
    fgrid = np.geomspace(0.35, 1.75, 25) * D["W"]
    th_list, pairs_list = [], []
    for j in range(n):
        fr = [0] + [i for i in range(ntr) if meta[i][0] == j]
        loc = {g: k for k, g in enumerate(fr)}
        pairs_list.append([(loc[i], loc[jj], a, b) for i, jj, a, b in pairs if i in loc and jj in loc])
        th_list.append(dQ[fr, j])
    f0, outs = SI.all_joints(th_list, pairs_list, cx, cy, fgrid, log=lambda s_: print(s_, flush=True))
    Wj, Ph, fj = [], [], []
    for j, o in enumerate(outs):
        w, ph, med, sc = o
        Wj.append(w); Ph.append(ph); fj.append(f0)
        wt = Sy.W_true[j] / np.linalg.norm(Sy.W_true[j])
        print("  关节 %d:轴离真值 %.2f° · 残差中位 %.3f px" % (j, math.degrees(math.acos(min(1, abs(np.dot(w, wt))))), med), flush=True)
    Wj = np.array(Wj); Ph = np.array(Ph); f0 = float(np.median(fj))
    jo = [meta[i][0] if meta[i][0] >= 0 else -1 for i in range(ntr)]
    rho, ref, med = SI.scales(Wj, Ph, f0, dQ[:ntr], jo, pairs, cx, cy)
    Pt = Sy.P_true - np.sum(Sy.P_true * Sy.W_true, 1, keepdims=True) * Sy.W_true   # 真的轴上离眼最近那点
    print("  各轴倍数(参照 %d):%s;真的比例:%s;残差中位 %.3f px" % (ref, np.round(rho, 3).tolist(),
          np.round(np.linalg.norm(Pt, axis=1) / np.linalg.norm(Pt[ref]), 3).tolist(), med))
    # 交给按像素一起解
    # 按起步模型挑内点(同真数据那版:残差 < max(3 px, 3 倍中位))
    I_, J_, A_, B_ = SI.expand(pairs, 10 ** 9)
    r0 = np.abs(SI.pair_res(Wj, Ph * rho[:, None], f0, dQ[:ntr], I_, J_, A_, B_, cx, cy))
    gate = max(3.0, 3 * np.median(r0))
    meas = []
    nin = 0
    for i, j, a, b in pairs:
        rr = np.abs(SI.pair_res(Wj, Ph * rho[:, None], f0, dQ[:ntr], np.full(len(a), i), np.full(len(a), j), a, b, cx, cy))
        m = rr < gate
        nin += m.sum()
        if m.sum() >= 30:
            meas.append(dict(i=i, j=j, a=a[m], b=b[m], par=1.0, R=np.eye(3)))
    print("  挑内点:门 %.2f px,留 %d / %d 个配点(乱配占 10%%)" % (gate, nin, len(A_)), flush=True)
    train = list(range(ntr))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        W, P, f = V.fit_px(dQ, meas, train, n, Wj, Ph * rho[:, None], f0, cx, cy)
        out = V.align_eval(W, P, dQ, train, D["test"], D["Pw"], np.random.default_rng(0))
    ete = out[7]
    print("扫开 ±%d°:起步焦距 %.1f(各关节 %s)⇒ 按像素一起解 焦距 %.1f(真 397.0);考试(全关节 ±30°)中位 %.2f mm、最大 %.2f mm(%.0f s)" %
          (sw_deg, f0, np.round(fj, 1).tolist(), f, np.median(ete), ete.max(), time.time() - t0), flush=True)
    print("   ", [l for l in buf.getvalue().splitlines() if "按像素" in l])

if __name__ == "__main__":
    run(int(sys.argv[1]) if len(sys.argv) > 1 else 45)
