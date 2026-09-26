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
        FITS[arm] = (s, Rg, tg, rv(x[3:6]), x[10:13])
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
        W0 = np.array([[float(x) for x in l.split()[2:5]] for l in Lm[2:] if l.startswith("axis")])
        P0_ = np.array([[float(x) for x in l.split()[5:8]] for l in Lm[2:] if l.startswith("axis")])
        Q0r = [np.array([float(x) for x in l.split("||")[0].split("|")[1].split()]) for l in rows if int(l.split("|")[0].split()[1]) == 0]
        fe_ = open(os.path.join(KDIR, "fixed_eye.txt")).read().split() if os.path.exists(os.path.join(KDIR, "fixed_eye.txt")) else None
        def proj(Rc, pc, f, cx, cy, X):
            Xc = Rc.T @ (X - pc)
            return np.array([f * Xc[0] / -Xc[2] + cx, -f * Xc[1] / -Xc[2] + cy]) if Xc[2] < 0 else np.array([1e4, 1e4])
        ed, et, kinds = [], [], []
        for r in L_:
            va, vf = int(r[0]), int(r[1]); uv = r[3:5]; Xb = r[11:14]
            if va == 0:
                Rc, pc = kinem_fk(W0, P0_, q00, Q0r[vf]); f, cx, cy = f0, cx0, cy0
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
        tips = []
        for mesh, org in (("link7", np.array([0.08657, 0.024896, -0.0002436])), ("link8", np.array([0.08657, -0.0249, -0.00024366]))):
            V = stl(X5 + "/meshes/%s.STL" % mesh) + org
            tips.append(V[np.argmax(V[:, 0])])
        truth_mid = 0.5 * (tips[0] + tips[1])
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
            print("指尖 · 第 %d 只手:碰出来的两瓣中点(手腕系)(%.1f, %.1f, %.1f) mm · 模型文件 (%.1f, %.1f, %.1f) mm · 差 %.1f mm · 离眼 %.1f mm · 张口 %.1f mm(%s)" %
                  (a, *(1000 * tee), *(1000 * truth_mid), 1000 * np.linalg.norm(tee - truth_mid), 1000 * np.linalg.norm(tc), 1000 * g["gap"] * s_,
                   "碰桌面量的" if g.get("tip_touch") else "不是碰桌面量的"))
except Exception as e:
    print("指尖:打不了分(%s)" % e)
