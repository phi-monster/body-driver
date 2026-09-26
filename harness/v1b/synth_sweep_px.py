# 合成预演(V1b 第 2 步):已知胳膊 + 真焦距 397;开机 = 板上 15 停(关节 ±5°)+ 每个关节单独两个方向扫开(累计到 ±SW°);
# 桌面撒点按每一帧投进画面(像素噪声 0.5),真流程造配对(本质矩阵 RANSAC,起步焦距故意错);
# 考试 = 全部关节同时随机转 ±30° 的姿势(干活时的样子),只给关节读数算眼在哪。
import math, io, contextlib, sys, os
import numpy as np, cv2
import v1b_fit as V
import synth_v1b as Sy

def frames(sw_deg, r):
    n = 6
    board = r.uniform(-1, 1, (15, n)) * math.radians(5); board[0] = 0
    steps = np.array([2, 4, 8, 14, 20, 28, 36, 45], float) / 45 * sw_deg
    sw = [np.zeros(n)]
    meta = [(-1, 0, 0)] * 15 + [(0, 0, 0)]
    for j in range(n):
        for d in (-1, 1):
            for k, a in enumerate(steps):
                q = np.zeros(n); q[j] = d * math.radians(a); sw.append(q); meta.append((j, d, k + 1))
    return np.vstack([board, np.array(sw)]), meta

def run(sw_deg, f_init_scale, px=0.5, f_true=397.0, seed=3, ntest=40):
    r = np.random.default_rng(seed)
    n = 6; cx, cy, W_, H_ = 320.0, 240.0, 640, 480
    dQ_tr, meta = frames(sw_deg, r) if sw_deg > 0 else (r.uniform(-1, 1, (15, n)) * math.radians(5), [(-1, 0, 0)] * 15)
    if sw_deg <= 0:
        dQ_tr[0] = 0
    dQ_te = r.uniform(-1, 1, (ntest, n)) * math.radians(30)
    dQ = np.vstack([dQ_tr, dQ_te])
    ntr = len(dQ_tr)
    train = list(range(ntr)); test = list(range(ntr, ntr + ntest))
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
    # 配对:关节最近邻 6 个 + 扫描里连着的两格 + 每格对起点
    pairs = set()
    for i in train:
        dd = sorted(((np.max(np.abs(dQ[i] - dQ[j])), j) for j in train if j != i))
        for _, j in dd[:6]:
            pairs.add((min(i, j), max(i, j)))
    if sw_deg > 0:
        st = 15; prev = {}
        for i in range(16, ntr):
            key = meta[i][:2]; a_ = prev.get(key, st)
            pairs.add((min(a_, i), max(a_, i))); pairs.add((st, i)); prev[key] = i
    f0 = f_true * f_init_scale
    K0 = np.array([[f0, 0, cx], [0, f0, cy], [0, 0, 1.0]])
    meas = []
    for i, j in sorted(pairs):
        ok = uv[i][1] & uv[j][1]
        if ok.sum() < 60:
            continue
        A = uv[i][0][ok]; B = uv[j][0][ok]
        if len(A) > 800:
            sel = r.choice(len(A), 800, replace=False); A = A[sel]; B = B[sel]
        E, mask = cv2.findEssentialMat(A, B, K0, method=cv2.RANSAC, prob=0.99999, threshold=1.0)
        if E is None or E.shape != (3, 3):
            continue
        ninl, Rr, tt, mask2 = cv2.recoverPose(E, A, B, K0, mask=mask)
        m = mask2.ravel() > 0
        if ninl < 50:
            continue
        xa = cv2.undistortPoints(A[m].reshape(-1, 1, 2), K0, None).reshape(-1, 2)
        xb = cv2.undistortPoints(B[m].reshape(-1, 1, 2), K0, None).reshape(-1, 2)
        ra = np.c_[xa, np.ones(len(xa))]; rb = np.c_[xb, np.ones(len(xb))]
        ra /= np.linalg.norm(ra, axis=1, keepdims=True); rb /= np.linalg.norm(rb, axis=1, keepdims=True)
        par = np.degrees(np.median(np.arccos(np.clip(np.sum((ra @ Rr.T) * rb, 1), -1, 1))))
        meas.append(dict(i=i, j=j, R=Rr, t=tt.ravel() / np.linalg.norm(tt), ninl=int(ninl), par=par, a=A[m], b=B[m]))
    Rx = V.rot(np.array([0.3, -0.2, 1.0]), 0.7); tx = np.array([0.0, -0.085, 0.051])
    Pw = []
    for Rk, tk in zip(R, t):
        Rc = Sy.R0 @ Rk; pc = Sy.R0 @ tk + Sy.c0
        Ree = Rc @ Rx.T; Pw.append((pc - Ree @ tx, Ree))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        res = V.fit_arm(dQ, meas, train, test, Pw, n, r)
        W, P, f = V.fit_px(dQ, meas, train, n, res["model"]["W"], res["model"]["P"], f0, cx, cy)
        out = V.align_eval(W, P, dQ, train, test, Pw, r)
    print([l for l in buf.getvalue().splitlines() if "迭代" in l or "按像素" in l])
    # 真参数处的代价(同一份配点):看是"迭代不够"还是"坑"
    if os.environ.get("FROM_TRUTH"):
        rms_t = math.sqrt(np.mean(np.sum(t[train] ** 2, 1)))
        buf2 = io.StringIO()
        with contextlib.redirect_stdout(buf2):
            V.fit_px(dQ, meas, train, n, Sy.W_true, Sy.P_true / rms_t, f_true, cx, cy)
        print("从真参数起步:", [l for l in buf2.getvalue().splitlines() if "迭代" in l or "按像素" in l])
    ete = out[7]
    print("扫开 ±%2d° · 起步焦距 %.1f(真 %.1f)· 配对 %d ⇒ 两两那一份:考试 中位 %.2f mm;按像素一起解:考试(全关节 ±30°)中位 %.2f mm、最大 %.2f mm,焦距 %.1f" %
          (sw_deg, f0, f_true, len(meas), res["test_med_mm"], np.median(ete), ete.max(), f), flush=True)

if __name__ == "__main__":
  for sw, fs in ((0, 1.0), (45, 1.0), (45, 1.15), (45, 0.85)):
      run(sw, fs)
