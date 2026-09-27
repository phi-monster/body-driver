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
    trn = trn[::max(1, len(trn) // 200)]   # 拟合最多等间隔取 200 个姿势(长的一炮上千个姿势,20 次重启在 Python 里要几十分钟);考试那一半全算
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
def kinem_fk(W, P, q0, q, slide=None):
    # slide[i] = 第 i 根是"走"的关节(09-27 起落盘第 9 列 turn / slide):沿 W 走 θ 个读数单位(W 的长 = 每单位走多远),不转
    R = np.eye(3); t = np.zeros(3)
    for i in range(len(W)):
        th = q[i] - q0[i]
        if slide is not None and slide[i]:
            t = t + R @ (W[i] * th)
            continue
        w = W[i] / np.linalg.norm(W[i])
        K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        Ri = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
        t = t + R @ (P[i] - Ri @ P[i]); R = R @ Ri
    return R, t


def read_axes(lines):
    W, P, S = [], [], []
    for l in lines:
        if l.startswith("axis"):
            f = l.split()
            W.append([float(x) for x in f[2:5]]); P.append([float(x) for x in f[5:8]]); S.append(len(f) > 8 and f[8] == "slide")
    return np.array(W), np.array(P), S
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
        W, Pp, Sl = read_axes(L[2:])
        Rf, Tf, Tp, Rt, Jr, Qs = [], [], [], [], [], []
        for l in rows:
            left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
            if int(h[1]) != arm:
                continue
            ee = [float(x) for x in right.split()]
            if len(ee) != 7:
                continue
            q = np.array([float(x) for x in parts[1 + arm].split()])
            R, t = kinem_fk(W, Pp, q0, q, Sl)
            Qs.append(q)
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
        # 换算(模型 → 仿真)给后面几种考法用:按【全部】扫描格拟合(09-27:模型在扫描范围里有形变时,一半格子拟合的换算自己晃 2–3 mm,V1B32 第 2 只手 ② 7.3 / 4.1 mm 两种算法)
        ra = least_squares(lambda xx: res2(xx, np.arange(n)), x, method="lm", max_nfev=4000)
        xa = ra.x
        FITS[arm] = (xa[6], rv(xa[0:3]), xa[7:10], rv(xa[3:6]), xa[10:13])
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
            if Sl[j]:
                # 走的关节:真值里扫它那几格 = 眼沿一条线平移 ⇒ 方向、每单位读数走多远;模型的 W 按相似变换搬到世界比
                dq = np.array([Qs[k][j] - Qs[i0][j] for k in ks]); dp = np.array([Tp[k] - Tp[i0] for k in ks])
                g = (dp.T @ dq) / float(dq @ dq)          # 每单位读数走的(世界,米)
                gm = s * (Rg @ W[j])
                ang = math.degrees(math.acos(min(1.0, abs(float(g @ gm)) / (np.linalg.norm(g) * np.linalg.norm(gm)))))
                out.append("轴%d(走)方向差 %.2f° 每单位读数走 真 %.1f / 模型 %.1f mm" % (j, ang, 1000 * np.linalg.norm(g), 1000 * np.linalg.norm(gm)))
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
    s0, Rg0, tg0 = FITS[0][:3]; sk, Rgk, tgk = FITS[arm][:3]
    St = sk / s0; Rt_ = Rg0.T @ Rgk; Tt = Rg0.T @ (tgk - tg0) / s0
    ang = math.degrees(np.linalg.norm(logR(R.T @ Rt_)))
    print("对齐 · 第 %d 只手 → 第 0 只手:长度倍数 驱动 %.4f / 真 %.4f(差 %.1f%%)· 转动差 %.2f° · 平移 驱动 (%.3f, %.3f, %.3f) / 真 (%.3f, %.3f, %.3f) 单位(差 %.1f mm)" %
          (arm, S, St, 100 * (S / St - 1), ang, *T, *Tt, 1000 * s0 * np.linalg.norm(T - Tt)))
    # 第 k 只手扫描格上的眼按驱动的对齐搬到第 0 只手的系、再按第 0 只手的(s, Rg, tg)搬到世界 vs 真值
    L_ = np.loadtxt(ap, skiprows=1)
    if L_.ndim == 2 and len(L_) and L_.shape[1] >= 14:
        # 每条配点投回世界里那只眼:按驱动的对齐 vs 按真值的对齐(像素)⇒ 分得清是"配点 / 各自的点错了"还是"解错了"
        k0 = os.path.join(KDIR, "kinem_arm0.txt"); Lm = open(k0).read().split("\n"); hm = Lm[0].split()
        f0, cx0, cy0 = float(hm[5]), float(hm[7]), float(hm[9]); q00 = np.array([float(x) for x in Lm[1].split()[1:]])
        W0, P0_, S0_ = read_axes(Lm[2:])
        Q0r = [np.array([float(x) for x in l.split("||")[0].split("|")[1].split()]) for l in rows if int(l.split("|")[0].split()[1]) == 0]
        fe_ = open(os.path.join(KDIR, "fixed_eye.txt")).read().split() if os.path.exists(os.path.join(KDIR, "fixed_eye.txt")) else None
        def proj(Rc, pc, f, cx, cy, X):
            Xc = Rc.T @ (X - pc)
            return np.array([f * Xc[0] / -Xc[2] + cx, -f * Xc[1] / -Xc[2] + cy]) if Xc[2] < 0 else np.array([1e4, 1e4])
        ed, et, kinds = [], [], []
        for r in L_:
            va, vf = int(r[0]), int(r[1]); uv = r[3:5]; Xb = r[11:14]
            if va == 0:
                Rc, pc = kinem_fk(W0, P0_, q00, Q0r[vf], S0_); f, cx, cy = f0, cx0, cy0
            elif fe_ is not None:
                f = float(fe_[1]); cx = float(fe_[3]); cy = float(fe_[5])
                pc = np.array([float(v) for v in fe_[fe_.index("pos") + 1: fe_.index("pos") + 4]]); Rc = np.array([float(v) for v in fe_[fe_.index("R") + 1: fe_.index("R") + 10]]).reshape(3, 3)
            else:
                continue
            ed.append(np.linalg.norm(proj(Rc, pc, f, cx, cy, S * (R @ Xb) + T) - uv))
            et.append(np.linalg.norm(proj(Rc, pc, f, cx, cy, St * (Rt_ @ Xb) + Tt) - uv))
            kinds.append(va)
        ed, et, kinds = np.array(ed), np.array(et), np.array(kinds)
        for kk, nm in ((0, "第一只手的格子"), (-1, "不动的眼")):
            m = kinds == kk
            if m.any():
                print("   配点投回%s(%d 条):按驱动的对齐 中位 %.1f px、<3px %.0f%% · 按真值的对齐 中位 %.1f px、<3px %.0f%%" %
                      (nm, m.sum(), np.median(ed[m]), 100 * np.mean(ed[m] < 3), np.median(et[m]), 100 * np.mean(et[m] < 3)))
        L_ = L_[:, 3:]
    if L_.ndim == 2 and len(L_):
        Xa = L_[:, 5:8]; Xb = L_[:, 8:11]
        pred = (S * (R @ Xb.T)).T + T; true_ = (St * (Rt_ @ Xb.T)).T + Tt
        e_drv = 1000 * s0 * np.linalg.norm(pred - Xa, axis=1); e_true = 1000 * s0 * np.linalg.norm(true_ - Xa, axis=1)
        print("   点对 %d:按驱动的对齐 两边点差 中位 %.1f mm;按真值的对齐 中位 %.1f mm、<10 mm 的占 %.0f%%(= 两只手三角出的点本身对得上的比例)" %
              (len(L_), np.median(e_drv), np.median(e_true), 100 * np.mean(e_true < 10)))

# ── 第四种考法:不长在手上的那只眼(look/fixed_eye.txt:焦距、第一只手系里的位置)⇒ 按第一只手"模型 → 世界"的相似变换搬到仿真世界(米)──
fp = os.path.join(KDIR, "fixed_eye.txt")
if os.path.exists(fp) and 0 in FITS:
    f_ = open(fp).read().split()
    fx = float(f_[1]); pos = np.array([float(v) for v in f_[f_.index("pos") + 1:f_.index("pos") + 4]])
    s0, Rg0, tg0 = FITS[0][:3]
    pw = s0 * (Rg0 @ pos) + tg0
    print("不动的眼:焦距 %.1f(仿真 x5 头顶眼 288.1)· 位置 (%.3f, %.3f, %.3f) m(X5E 读位姿那一版标的 (0.000, -0.412, 1.310))· 残差 %s px · 进解 %s / %s" %
          (fx, pw[0], pw[1], pw[2], f_[f_.index("rms") + 1], f_[f_.index("used") + 1], f_[f_.index("of") + 1]))

# ── 第五种考法(V1b ②):开机自检让手走到没去过的地方(look/ik_check.txt:世界系的目标、按读数算到的、身体报的真值)⇒ 目标换到仿真米,和真的眼比 ──
ip = os.path.join(KDIR, "ik_check.txt"); wp = os.path.join(KDIR, "world.txt")
if os.path.exists(ip) and os.path.exists(wp) and 0 in FITS:
    wv = [float(v) for v in open(wp).read().split()]; Rw = np.array(wv[:9]).reshape(3, 3); Ow = np.array(wv[9:12])
    s0, Rg0, tg0 = FITS[0][:3]
    w2s = lambda p: s0 * (Rg0 @ (Rw.T @ p + Ow)) + tg0
    per = {}
    for l in open(ip):
        if "||" not in l: continue
        left, right = l.split("||", 1); parts = left.split("|"); h = parts[0].split()
        a, sw = int(h[0]), int(h[1])
        tgt = np.array([float(v) for v in parts[1].split()]); got = np.array([float(v) for v in parts[2].split()])
        tr = np.array([float(v) for v in right.split()])
        if len(tr) != 7 or sw not in FITS: continue
        RxA, txA = FITS[sw][3], FITS[sw][4]
        cam = tr[:3] + qR(tr[3:]) @ txA
        e_t = 1000 * np.linalg.norm(w2s(tgt[:3]) - cam); e_g = 1000 * np.linalg.norm(w2s(got[:3]) - cam)
        Rt_sim = Rg0 @ Rw.T @ qR(tgt[3:]); Rc = qR(tr[3:]) @ RxA
        ang = math.degrees(np.linalg.norm(logR(Rc.T @ Rt_sim)))
        # 这一处离扫描时去过的地方多远(真的眼;同一只手扫描各格的真值):最近那一格差几 mm、那一格朝向差几度
        near_d, near_a = float("nan"), float("nan")
        if os.path.exists(sp):
            best = None
            for l2 in open(sp):
                if "||" not in l2: continue
                h2 = l2.split("||", 1)[0].split("|")[0].split(); t2 = np.array([float(x) for x in l2.split("||", 1)[1].split()])
                if int(h2[1]) != sw or len(t2) != 7: continue
                cam2 = t2[:3] + qR(t2[3:]) @ txA
                d2 = 1000 * np.linalg.norm(cam2 - cam)
                if best is None or d2 < best[0]:
                    best = (d2, math.degrees(np.linalg.norm(logR((qR(t2[3:]) @ RxA).T @ Rc))))
            if best is not None:
                near_d, near_a = best
        per.setdefault(sw, []).append((e_t, e_g, ang, near_d, near_a))
    for sw, v in sorted(per.items()):
        v = np.array(v)
        print("V1b ② · 第 %d 只手走到没去过的 %d 处:真的眼离目标 %s mm(最大 %.1f)· 朝向差 %s° · 按读数算的位置离真的眼 %s mm" %
              (sw, len(v), " ".join("%.1f" % x for x in v[:, 0]), v[:, 0].max(), " ".join("%.2f" % x for x in v[:, 2]), " ".join("%.1f" % x for x in v[:, 1])))
        print("   这几处离扫描时去过的最近一格:%s mm(那一格朝向差 %s°)" % (" ".join("%.1f" % x for x in v[:, 3]), " ".join("%.1f" % x for x in v[:, 4])))

# ── 第六种考法:指尖(身体文件旁边的几何文件 <身体文件>.geo.json:每只腕眼的 tip = 两瓣指尖中点、gap = 两瓣相距,眼的系、模型单位)
#    ⇒ 乘"米 / 单位"、按拟合的"眼离手"换到手腕(link6)系,和 x5 模型文件里两根手指网格沿夹爪方向最远那一点的中点比(手指对称 ⇒ 中点和张开多少无关)──
try:
    import re, json, struct
    logtxt = open(os.path.join(RUN, "cal.log"), encoding="utf-8", errors="ignore").read()
    mb = re.search(r"身体写进 (/\S+?\.json)", logtxt)
    X5 = "/root/RoboDojo/Assets/Robots/x5"
    if mb and os.path.exists(mb.group(1) + ".geo.json") and os.path.exists(X5 + "/meshes/link7.STL"):
        geo = json.load(open(mb.group(1) + ".geo.json"))
        def stl(p):
            d = open(p, "rb").read(); n = struct.unpack("<I", d[80:84])[0]
            return np.array([struct.unpack("<12f", d[84 + 50 * i:84 + 50 * i + 48])[3:12] for i in range(n)]).reshape(-1, 3)
        #    两种真值一起报:沿夹爪方向最远的那个顶点(端面是 1.5 × 10 mm 的一条窄边,x 最大的顶点落在它的下角)、端面(x 在最大值 1 mm 内的顶点)的中心;
        #    张口按张开到头(两根手指各 44 mm,URDF 的上限;开机碰桌面时爪是张开的)两根手指端面内侧的距离
        tips, faces, inner = [], [], []
        for mesh, org, ax in (("link7", np.array([0.08657, 0.024896, -0.0002436]), 1.0), ("link8", np.array([0.08657, -0.0249, -0.00024366]), -1.0)):
            V = stl(X5 + "/meshes/%s.STL" % mesh) + org + np.array([0.0, ax * 0.044, 0.0])
            tips.append(V[np.argmax(V[:, 0])])
            Fc = V[V[:, 0] >= V[:, 0].max() - 0.001]
            faces.append(Fc.mean(axis=0))
            inner.append(Fc[np.argmin(np.abs(Fc[:, 1]))])
        truth_mid = 0.5 * (tips[0] + tips[1])
        face_mid = 0.5 * (faces[0] + faces[1])
        truth_gap = abs(inner[0][1] - inner[1][1])
        cam_arm = {int(c): int(a) - 1 for c, a in re.findall(r"第(\d+) 台相机\(长在第(\d+) 只手上\)", logtxt)}
        for g in geo["cams"]:
            c = g["cam"]
            if c not in cam_arm or cam_arm[c] not in FITS:
                continue
            a = cam_arm[c]; s_, Rg_, tg_, RxA, txA = FITS[a]
            if not g.get("tip_valid"):
                print("指尖 · 第 %d 只手(第 %d 台眼):没量成" % (a, c)); continue
            tc = np.array(g["tip"]) * s_
            tee = RxA @ tc + txA
            print("指尖 · 第 %d 只手:碰出来的两瓣中点(手腕系)(%.1f, %.1f, %.1f) mm · 模型文件 最远顶点 (%.1f, %.1f, %.1f) 差 %.1f mm / 端面中心 (%.1f, %.1f, %.1f) 差 %.1f mm · 离眼 %.1f mm · 张口 %.1f mm(真 %.1f)(%s)" %
                  (a, *(1000 * tee), *(1000 * truth_mid), 1000 * np.linalg.norm(tee - truth_mid), *(1000 * face_mid), 1000 * np.linalg.norm(tee - face_mid),
                   1000 * np.linalg.norm(tc), 1000 * g["gap"] * s_, 1000 * truth_gap, "碰桌面量的" if g.get("tip_touch") else "不是碰桌面量的"))
            #  逐瓣(09-28 换倾角碰起驱动每一瓣的尖落一行"第A 只手第 K 瓣的尖在眼系 (x, y, z)",眼系、模型单位;同一只手取最后一次)⇒ 各自配离它最近的那根手指
            lobes = {}
            for m in re.finditer(r"第(\d+) 只手第 (\d+) 瓣的尖在眼系 \(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\)", logtxt):
                if int(m.group(1)) - 1 == a:
                    lobes[int(m.group(2))] = np.array([float(m.group(3)), float(m.group(4)), float(m.group(5))])
            for k in sorted(lobes):
                tk = RxA @ (lobes[k] * s_) + txA
                j = int(np.argmin([np.linalg.norm(tk - t) for t in tips]))
                print("   第 %d 瓣(配第 %d 根手指):尖(手腕系)(%.1f, %.1f, %.1f) mm · 最远顶点 (%.1f, %.1f, %.1f) 差 %.1f mm / 端面中心 差 %.1f mm" %
                      (k, j, *(1000 * tk), *(1000 * tips[j]), 1000 * np.linalg.norm(tk - tips[j]), 1000 * np.linalg.norm(tk - faces[j])))
except Exception as e:
    print("指尖:打不了分(%s)" % e)

# ── 第七种考法:桌面高度(驱动的世界 z = 0 就是它量的桌面;板 = 三角出、落在桌面上的点,都在身体文件旁边的 .kin.txt 里,世界系)
#    ⇒ 按第一只手的换算搬到仿真米,和场景配置里的桌面顶(env_cfg/scene/default.yml:Table 的 default_pos z + 厚度一半)比 ──
# ── 第八种考法:头顶眼按像素错多少(V1 判据"头顶眼 < 2 px"):真桌面上铺一片格点,按仿真相机配置(env_cfg/camera/camera_config.yml 的 cam_head:
#    pos、ori 欧拉角度数;内参按 template.py 的焦距 / 横向孔径 × 画幅宽)投一次、按驱动的头顶眼(.kin.txt 的 fixed 行,世界系)投一次,两边差几像素 ──
try:
    kin = (mb.group(1) + ".kin.txt") if mb else None
    if kin and os.path.exists(kin) and 0 in FITS:
        s0, Rg0, tg0 = FITS[0][:3]
        Lk = open(kin).read().split("\n")
        Rw_k = np.array([float(v) for v in [l for l in Lk if l.startswith("rw ")][0].split()[1:10]]).reshape(3, 3)
        Ow_k = np.array([float(v) for v in [l for l in Lk if l.startswith("o ")][0].split()[1:4]])
        w2s_k = lambda p: s0 * (Rg0 @ (Rw_k.T @ p + Ow_k)) + tg0
        board = np.array([[float(v) for v in l.split()[1:4]] for l in Lk if l.startswith("board ")])
        import yaml
        sc = yaml.safe_load(open("/root/RoboDojo/env_cfg/scene/default.yml"))["Table"]
        top = sc["default_pos"][2] + sc["scale"][2] / 2
        if len(board):
            Pb = np.array([w2s_k(p) for p in board]); dz = 1000 * (Pb[:, 2] - top)
            n_s = Rg0 @ (Rw_k.T @ np.array([0.0, 0.0, 1.0])); tilt = math.degrees(math.acos(min(1.0, abs(n_s[2]) / np.linalg.norm(n_s))))
            print("桌面:板上 %d 个点搬到仿真里,比场景配置的桌面顶 %.3f m 高 中位 %+.1f mm、九成 %.1f mm(绝对值)· 驱动的桌面法向和竖直差 %.2f°" %
                  (len(board), top, np.median(dz), np.quantile(np.abs(dz), 0.9), tilt))
        fl = [l for l in Lk if l.startswith("fixed ")]
        if fl:
            t = fl[0].split()
            if len(t) >= 41 and t[1] == "1":
                fd, cxd, cyd = float(t[2]), float(t[3]), float(t[4]); k1, k2 = float(t[5]), float(t[6])
                Rce = np.array([float(v) for v in t[11:20]]).reshape(3, 3); pos_w = np.array([float(v) for v in t[38:41]])
                Rd = Rg0 @ Rw_k.T @ Rce; pd = w2s_k(pos_w)
                cc = yaml.safe_load(open("/root/RoboDojo/env_cfg/camera/camera_config.yml"))["cam_head"]["camera"]
                src = open("/root/RoboDojo/env_cfg/camera/template.py").read()
                blk = src[src.index(cc["type"].upper() + " = {"):]; blk = blk[:blk.index("}")]
                fl_mm = float(re.search(r'"focal_length":\s*([0-9.]+)', blk).group(1)); ha = float(re.search(r'"horizontal_aperture":\s*([0-9.]+)', blk).group(1))
                W_, H_ = [int(v) for v in re.search(r'"resolution":\s*\((\d+),\s*(\d+)\)', blk).groups()]
                ft = fl_mm / ha * W_
                ex, ey, ez = [math.radians(a) for a in cc["ori"]]
                Rx_ = np.array([[1, 0, 0], [0, math.cos(ex), -math.sin(ex)], [0, math.sin(ex), math.cos(ex)]])
                Ry_ = np.array([[math.cos(ey), 0, math.sin(ey)], [0, 1, 0], [-math.sin(ey), 0, math.cos(ey)]])
                Rz_ = np.array([[math.cos(ez), -math.sin(ez), 0], [math.sin(ez), math.cos(ez), 0], [0, 0, 1]])
                Rt_c = Rz_ @ Ry_ @ Rx_; pt_c = np.array(cc["pos"], dtype=float)
                def proj_c(R, p, f, cx, cy, X, k1=0.0, k2=0.0):
                    Xc = R.T @ (X - p)
                    if Xc[2] >= 0: return None
                    x, y = Xc[0] / -Xc[2], Xc[1] / -Xc[2]; r2 = x * x + y * y; d = 1 + k1 * r2 + k2 * r2 * r2
                    return np.array([cx + f * x * d, cy - f * y * d])
                errs = []
                for gx in np.linspace(-0.7, 0.7, 57):
                    for gy in np.linspace(-0.6, 0.5, 45):
                        X = np.array([gx, gy, top]); ut = proj_c(Rt_c, pt_c, ft, W_ / 2, H_ / 2, X)
                        if ut is None or not (0 <= ut[0] < W_ and 0 <= ut[1] < H_): continue
                        ud = proj_c(Rd, pd, fd, cxd, cyd, X, k1, k2)
                        if ud is not None: errs.append(np.linalg.norm(ud - ut))
                errs = np.array(errs)
                ang = math.degrees(np.linalg.norm(logR(Rt_c.T @ Rd)))
                print("头顶眼(按像素):真桌面上 %d 个格点,按驱动的头顶眼投和按仿真相机投 差 中位 %.2f px、九成 %.2f px、最大 %.2f px · 焦距 %.1f / 真 %.1f · 位置差 %.1f mm · 朝向差 %.3f°" %
                      (len(errs), np.median(errs), np.quantile(errs, 0.9), errs.max(), fd, ft, 1000 * np.linalg.norm(pd - pt_c), ang))
                #    被测试钩子转过(sim.log 里 "[camtest] cam_head 绕自己的光轴转了 X°(Fabric" 每次一行)⇒ 真相机 = 配置的朝向再绕自己的光轴(本地 z)转那么多;
                #    驱动重标以后的头顶眼在几何文件(<身体文件>.geo.json 的第 world_cam 台,世界系)里 ⇒ 同样按像素比
                sl = open(os.path.join(RUN, "sim.log"), encoding="utf-8", errors="ignore").read() if os.path.exists(os.path.join(RUN, "sim.log")) else ""
                rolls = [float(x) for x in re.findall(r"\[camtest\] cam_head 绕自己的光轴转了 ([-0-9.]+)°\(Fabric", sl)]
                gp = mb.group(1) + ".geo.json"
                wc = int([l for l in Lk if l.startswith("world_cam")][0].split()[1])
                #    几何文件的快照(harness 的 geosnap.sh:文件每变一次存一份 geosnap/geo_HHMMSS.json)× camseq 的步骤时刻(N<炮>_camseq.txt 的 "== HH:MM:SS 转 …"):
                #    每一份按它存下那一刻之前转过几次 90° 当真相机,各按像素比(开机那份、转 90° 重标那份、转 180° 重标那份)
                snapdir = os.path.join(RUN, "geosnap"); cs_txt = RUN.rstrip("/") + "_camseq.txt"
                if os.path.isdir(snapdir) and os.path.exists(cs_txt):
                    turns = [l.split()[1] for l in open(cs_txt, encoding="utf-8", errors="ignore") if l.startswith("== ") and "转" in l and "挡" not in l and "撤" not in l]
                    tsec = lambda hms: int(hms[0:2]) * 3600 + int(hms[2:4]) * 60 + int(hms[4:6])
                    turn_s = [tsec(t.replace(":", "")) for t in turns]
                    for fn in sorted(os.listdir(snapdir)):
                        if not fn.startswith("geo_"): continue
                        ts = tsec(fn[4:10]); k = sum(1 for t in turn_s if t <= ts)
                        #    仿真一集结束(判成功 / 失败)会复位,头顶眼跟着回到配置的朝向(09-27 V1B41:第二次转之前复位过)⇒ 按步骤时刻累加的转数不一定是真的;
                        #    四个转法都算一遍照实印出来,哪个是真的按 sim.log 的事件(camtest 行 + "Video is saved" 的一集结束)定,不挑最小的
                        errs4 = []
                        gs = [g for g in json.load(open(os.path.join(snapdir, fn)))["cams"] if g["cam"] == wc][0]
                        Rd3 = Rg0 @ Rw_k.T @ np.array(gs["r_ce"]).reshape(3, 3); pd3 = w2s_k(np.array(gs["pos"]))
                        for kk in range(4):
                            th = math.radians(90.0 * kk)
                            Rt_k = Rt_c @ np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
                            e3 = []
                            for gx in np.linspace(-0.7, 0.7, 57):
                                for gy in np.linspace(-0.6, 0.5, 45):
                                    X = np.array([gx, gy, top]); ut = proj_c(Rt_k, pt_c, ft, W_ / 2, H_ / 2, X)
                                    if ut is None or not (0 <= ut[0] < W_ and 0 <= ut[1] < H_): continue
                                    ud = proj_c(Rd3, pd3, gs["f"], gs["cx"], gs["cy"], X, gs.get("k1", 0.0), gs.get("k2", 0.0))
                                    if ud is not None: e3.append(np.linalg.norm(ud - ut))
                            e3 = np.array(e3)
                            errs4.append("%d×90°:中位 %.2f / 九成 %.2f / 最大 %.2f px" % (kk, np.median(e3), np.quantile(e3, 0.9), e3.max()) if len(e3) else "%d×90°:—" % kk)
                        print("头顶眼快照 %s(按步骤时刻转过 %d 次;残差自报 %.2f px;位置差 %.1f mm):%s" % (fn, k, gs.get("rms", 0.0), 1000 * np.linalg.norm(pd3 - pt_c), " · ".join(errs4)))
                if rolls and os.path.exists(gp):
                    th = math.radians(sum(rolls))
                    Rt_r = Rt_c @ np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
                    gg = [g for g in json.load(open(gp))["cams"] if g["cam"] == wc][0]
                    Rd2 = Rg0 @ Rw_k.T @ np.array(gg["r_ce"]).reshape(3, 3); pd2 = w2s_k(np.array(gg["pos"]))
                    e2 = []
                    for gx in np.linspace(-0.7, 0.7, 57):
                        for gy in np.linspace(-0.6, 0.5, 45):
                            X = np.array([gx, gy, top]); ut = proj_c(Rt_r, pt_c, ft, W_ / 2, H_ / 2, X)
                            if ut is None or not (0 <= ut[0] < W_ and 0 <= ut[1] < H_): continue
                            ud = proj_c(Rd2, pd2, gg["f"], gg["cx"], gg["cy"], X, gg.get("k1", 0.0), gg.get("k2", 0.0))
                            if ud is not None: e2.append(np.linalg.norm(ud - ut))
                    e2 = np.array(e2)
                    print("头顶眼被转了 %s°(共 %.0f°)以后驱动重标的那份:真桌面上 %d 个格点差 中位 %.2f px、九成 %.2f px、最大 %.2f px · 位置差 %.1f mm · 朝向差 %.3f°" %
                          (" + ".join("%.0f" % r for r in rolls), sum(rolls), len(e2), np.median(e2), np.quantile(e2, 0.9), e2.max(),
                           1000 * np.linalg.norm(pd2 - pt_c), math.degrees(np.linalg.norm(logR(Rt_r.T @ Rd2)))))
except Exception as e:
    print("桌面 / 头顶眼按像素:打不了分(%s)" % e)
