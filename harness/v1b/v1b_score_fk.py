#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V1b 3c 打分:驱动每一帧按关节读数算出来的手的位姿(vid/fk_poses.txt,世界系、模型单位)和身体自己报的(vid/poses.txt,只给打分)比。
驱动一个真值都没看。对齐:相似变换(世界系之间差一个转、移、倍数)+ 眼离手的偏移,只用每隔一个不同姿势取一半来拟合,另一半考试。
用法:v1b_score_fk.py <跑的目录>"""
import sys, os, math
import numpy as np
from scipy.optimize import least_squares
RUN = sys.argv[1]
def load(p):
    out = {}
    for l in open(p):
        f = l.split()
        if len(f) >= 9:
            out[int(f[0])] = np.array([float(x) for x in f[2:]])
    return out
fk = load(os.path.join(RUN, "vid", "fk_poses.txt")); tr = load(os.path.join(RUN, "vid", "poses.txt"))
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
seqs = sorted(set(fk) & set(tr))
narm = min(len(fk[seqs[0]]), len(tr[seqs[0]])) // 7 if seqs else 0
print("帧 %d(两边都有)· %d 只手" % (len(seqs), narm))
for a in range(narm):
    P = np.array([fk[s][7*a:7*a+7] for s in seqs]); T = np.array([tr[s][7*a:7*a+7] for s in seqs])
    key = np.round(P[:, :3] * 1e4).astype(np.int64)
    _, first = np.unique(key, axis=0, return_index=True)
    first = np.sort(first)
    trn = first[0::2]; tst = first[1::2]
    if len(trn) < 5:   # 拟合 13 个数,每个姿势 6 个残差 ⇒ 至少 5 个姿势(次数)
        print("手 %d:按帧算的位姿只有 %d 个不同姿势,这一段不考" % (a, len(first)))
        continue
    Rf = np.array([qR(p[3:]) for p in P]); Rt = np.array([qR(t[3:]) for t in T])
    def res(x, idx):
        Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        pe = (s * (P[idx, :3] @ Rg.T) + tg) - (T[idx, :3] + np.einsum('nab,b->na', Rt[idx], tx))
        re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
        return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
    best = None
    rng = np.random.default_rng(0)
    for k in range(20):
        x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
        r = least_squares(lambda x: res(x, trn), x0, method="lm", max_nfev=3000)
        if best is None or r.cost < best.cost: best = r
    x = best.x
    def err(idx):
        Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        pe = (s * (P[idx, :3] @ Rg.T) + tg) - (T[idx, :3] + np.einsum('nab,b->na', Rt[idx], tx))
        return np.linalg.norm(pe, axis=1) * 1000
    et, ee = err(trn), err(tst)
    print("手 %d:不同姿势 %d 个(拟合 %d、考试 %d)· 倍数 %.4f m/单位 · 眼离手 (%.1f, %.1f, %.1f) mm" % (a, len(first), len(trn), len(tst), x[6], *(1000 * x[10:13])))
    print("   拟合那一半:中位 %.2f mm、最大 %.2f mm;考试那一半:中位 %.2f mm、九成 %.2f mm、最大 %.2f mm" %
          (np.median(et), et.max(), np.median(ee), np.quantile(ee, 0.9), ee.max()))


# ── 第二种考法:驱动落盘的运动学(look/kinem_arm<k>.txt)在扫描各格上按关节读数算眼在哪,和扫描时记的仿真真值(sweep.txt 行尾)比 ──
def kinem_fk(W, P, q0, q):
    R = np.eye(3); t = np.zeros(3)
    for i in range(len(W)):
        w = W[i] / np.linalg.norm(W[i]); th = q[i] - q0[i]
        K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        Ri = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
        t = t + R @ (P[i] - Ri @ P[i]); R = R @ Ri
    return R, t
KDIR = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RUN, "look")   # 模型在哪(默认驱动落盘的;离线回放写到别处时给这个)
sp = os.path.join(RUN, "look", "sweep.txt")
FITS = {}
if os.path.exists(sp):
    rows = [l for l in open(sp) if "||" in l]
    for arm in range(2):
        kp = os.path.join(KDIR, "kinem_arm%d.txt" % arm)
        if not os.path.exists(kp):
            continue
        L = open(kp).read().split("\n")
        q0 = np.array([float(x) for x in L[1].split()[1:]])
        W, Pp = [], []
        for l in L[2:]:
            if l.startswith("axis"):
                v = [float(x) for x in l.split()[2:]]; W.append(v[:3]); Pp.append(v[3:])
        W = np.array(W); Pp = np.array(Pp)
        Rf, Tf, Tp, Rt, Jr = [], [], [], [], []
        for l in rows:
            left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
            if int(h[1]) != arm:
                continue
            ee = [float(x) for x in right.split()]
            if len(ee) != 7:
                continue
            q = np.array([float(x) for x in parts[1 + arm].split()])
            R, t = kinem_fk(W, Pp, q0, q)
            Rf.append(R); Tf.append(t); Tp.append(np.array(ee[:3])); Rt.append(qR(np.array(ee[3:]))); Jr.append(int(h[2]) if int(h[4]) > 0 else -1)
        Rf = np.array(Rf); Tf = np.array(Tf); Tp = np.array(Tp); Rt = np.array(Rt)
        n = len(Tf); trn = np.arange(0, n, 2); tst = np.arange(1, n, 2)
        def res2(x, idx):
            Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
            pe = (s * (Tf[idx] @ Rg.T) + tg) - (Tp[idx] + np.einsum('nab,b->na', Rt[idx], tx))
            re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
            return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
        best = None; rng = np.random.default_rng(0)
        for k in range(30):
            x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
            r = least_squares(lambda x: res2(x, trn), x0, method="lm", max_nfev=4000)
            if best is None or r.cost < best.cost: best = r
        x = best.x
        Rg = rv(x[0:3]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        FITS[arm] = (s, Rg, tg)
        def e2(idx):
            return np.linalg.norm((s * (Tf[idx] @ Rg.T) + tg) - (Tp[idx] + np.einsum('nab,b->na', Rt[idx], tx)), axis=1) * 1000
        et, ee_ = e2(trn), e2(tst)
        print("运动学(驱动落盘)· 手 %d:扫描 %d 格(拟合 %d、考试 %d)· 倍数 %.4f m/单位 · 眼离手 (%.1f, %.1f, %.1f) mm" % (arm, n, len(trn), len(tst), s, *(1000 * tx)))
        print("   拟合那一半:中位 %.2f mm、最大 %.2f mm;考试那一半:中位 %.2f mm、九成 %.2f mm、最大 %.2f mm" %
              (np.median(et), et.max(), np.median(ee_), np.quantile(ee_, 0.9), ee_.max()))
        ea = e2(np.arange(n)); Jr = np.array(Jr)
        print("   按扫的是哪个关节分(全部格子,mm):" + " · ".join("%s %.1f/%.1f" % ("多关节" if j == 99 else ("起点" if j < 0 else "关节%d" % j), np.median(ea[Jr == j]), ea[Jr == j].max())
                                                       for j in sorted(set(Jr.tolist()))))
        # 每根轴对不对:真值里扫第 j 个关节那几格 = 手绕一条固定的线转(世界系)⇒ 线的方向、位置;模型的轴按上面拟合的相似变换搬到世界里比
        i0 = int(np.where(Jr == -1)[0][0]) if (Jr == -1).any() else 0
        out = []
        for j in range(len(W)):
            ks = np.where(Jr == j)[0]
            if len(ks) == 0:
                continue
            A = []; B = []; ws = []
            for k in ks:
                Rr = Rt[k] @ Rt[i0].T; v = logR(Rr)
                if np.linalg.norm(v) < 1e-3:
                    continue
                ws.append(v / np.linalg.norm(v) * np.sign(q_all[k][j] - q_all[i0][j] if False else 1.0))
                A.append(np.eye(3) - Rr); B.append(Tp[k] - Rr @ Tp[i0])
            if not A:
                continue
            A = np.vstack(A); B = np.concatenate(B)
            c = np.linalg.lstsq(A, B, rcond=None)[0]
            wt = ws[-1]
            wm = Rg @ (W[j] / np.linalg.norm(W[j])); pm = s * (Rg @ Pp[j]) + tg
            ang = math.degrees(math.acos(min(1.0, abs(float(wt @ wm)))))
            # 两条线在"起点那一格眼的位置"附近差多远:眼在真值线上的垂足 vs 在模型线上的垂足
            cam0 = s * (Rg @ Tf[i0]) + tg
            ft = c + wt * float((cam0 - c) @ wt); fm = pm + wm * float((cam0 - pm) @ wm)
            out.append("轴%d 方向差 %.2f° 位置差 %.1f mm(离眼 真 %.0f / 模型 %.0f mm)" % (j, ang, 1000 * np.linalg.norm(ft - fm), 1000 * np.linalg.norm(cam0 - ft), 1000 * np.linalg.norm(cam0 - fm)))
        print("   每根轴(按上面的对齐搬到世界):" + ";".join(out))

# ── 第三种考法:两只手对到一个系(look/align_arm<k>.txt 第一行 = 驱动解的相似变换 X0 = S · R · Xk + T,两边都是各自的模型单位)──
# 真值:每只手"模型 → 世界"的相似变换按上面各自拟合的(s, Rg, tg)⇒ 第 k 只手的模型系 → 第 0 只手的:S = s_k / s_0,R = Rg0ᵀ Rgk,T = Rg0ᵀ (tg_k − tg_0) / s_0
for arm in range(1, 4):
    ap = os.path.join(KDIR, "align_arm%d.txt" % arm)
    if not os.path.exists(ap) or 0 not in FITS or arm not in FITS:
        continue
    h = open(ap).readline().split()
    S = float(h[1]); R = np.array([float(v) for v in h[3:12]]).reshape(3, 3); T = np.array([float(v) for v in h[13:16]])
    s0, Rg0, tg0 = FITS[0]; sk, Rgk, tgk = FITS[arm]
    St = sk / s0; Rt_ = Rg0.T @ Rgk; Tt = Rg0.T @ (tgk - tg0) / s0
    ang = math.degrees(np.linalg.norm(logR(R.T @ Rt_)))
    print("对齐 · 第 %d 只手 → 第 0 只手:长度倍数 驱动 %.4f / 真 %.4f(差 %.1f%%)· 转动差 %.2f° · 平移 驱动 (%.3f, %.3f, %.3f) / 真 (%.3f, %.3f, %.3f) 单位(差 %.1f mm)" %
          (arm, S, St, 100 * (S / St - 1), ang, *T, *Tt, 1000 * s0 * np.linalg.norm(T - Tt)))
    # 第 k 只手扫描格上的眼按驱动的对齐搬到第 0 只手的系、再按第 0 只手的(s, Rg, tg)搬到世界 vs 真值
    L_ = np.loadtxt(ap, skiprows=1)
    if L_.ndim == 2 and len(L_):
        Xa = L_[:, 5:8]; Xb = L_[:, 8:11]
        pred = (S * (R @ Xb.T)).T + T; true_ = (St * (Rt_ @ Xb.T)).T + Tt
        e_drv = 1000 * s0 * np.linalg.norm(pred - Xa, axis=1); e_true = 1000 * s0 * np.linalg.norm(true_ - Xa, axis=1)
        print("   点对 %d:按驱动的对齐 两边点差 中位 %.1f mm;按真值的对齐 中位 %.1f mm、<10 mm 的占 %.0f%%(= 两只手三角出的点本身对得上的比例)" %
              (len(L_), np.median(e_drv), np.median(e_true), 100 * np.mean(e_true < 10)))
