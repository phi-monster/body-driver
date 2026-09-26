#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V1b 第一步(离线,2026-09-26):只用【关节读数】+【手上那只眼在各停看到的桌面】量出"关节转多少、手上的眼到哪"。
仿真报的手的位姿只用来打分(对齐 + 考试),不进量法。

量法(一种):
  1. 同一只手的各停两两配点(RoMa,/match),本质矩阵 ⇒ 每一对停之间相机转了多少、往哪个方向挪(方向,无尺度)。
  2. 胳膊 = 一串转轴(每个关节一根:方向 ω、过哪一点 p),以第一停为参照,只用关节读数的【差】(不需要零点):
        T(Δq) = exp([S1]Δq1) · … · exp([Sn]Δqn),T = 手上那只眼在"参照停时的眼"系里的位姿。
     先只拟合转动(ω),再线性解 p(平移方向与模型平移平行),最后一起非线性精修。
  3. 考试:停点分成训练 / 考试两份;转轴只用训练停点之间的配对拟合;考试停点只给关节读数 ⇒ 按转轴算出眼在哪。
     打分时把模型系对到仿真世界(相似变换:转、移、一个倍数)+ 眼相对手的固定偏移,都只用训练停点的真值拟合。
     报:考试停点上"算出的眼的位置"和"真值"差多少毫米。

已知的借用(照实报):焦距用驱动自己开机量出来的那个(.geo.json;那个数是驱动用位姿读数量的 —— 下一步要改成自己量);
关节组按名字对到臂(state.left / state.right);关节在读数里的顺序当作从底座到手的顺序。
"""
import sys, os, json, base64, math, time, random, urllib.request
import numpy as np
import cv2
from scipy.optimize import least_squares

RUN = sys.argv[1] if len(sys.argv) > 1 else "."   # 例:/root/NV1B1
GEO = sys.argv[2] if len(sys.argv) > 2 else ""    # 例:/root/cal_v1b1.json.geo.json
INST = os.environ.get("INST", "http://127.0.0.1:8077/match")
NUM = int(os.environ.get("NUM", "2500"))
MAXPAIRS = int(os.environ.get("MAXPAIRS", "400"))
SEED = int(os.environ.get("SEED", "0"))
FINGER_V = float(os.environ.get("FINGER_V", "250"))   # 像素行:手指从这一行往下(按这具身体量到的握区框给)
OUT = os.path.join(RUN, "v1b")
rng = np.random.default_rng(SEED)


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def rot(w, th):
    w = w / np.linalg.norm(w)
    K = skew(w)
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def log_R(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * math.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def quat_to_R(q):   # [w x y z]
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ---------------- 数据 ----------------
def load_groups():
    for line in open(os.path.join(RUN, "cal.log"), encoding="utf-8", errors="replace"):
        if line.startswith("[认] 关节角:"):
            return [s.strip() for s in line.split(":", 1)[1].split("·")]
    return []


def load_stops():
    look = os.path.join(RUN, "look")
    stops = {}
    for line in open(os.path.join(look, "board_stops.txt")):
        f = line.split()
        n, cam, arm = int(f[0]), int(f[1]), int(f[2])
        stops[n] = dict(n=n, cam=cam, arm=arm, pose=np.array([float(x) for x in f[4:11]]))
    for line in open(os.path.join(look, "board_joints.txt")):
        parts = line.strip().split("|")
        n = int(parts[0].split()[0])
        if n in stops:
            stops[n]["groups"] = [np.array([float(x) for x in p.split()]) for p in parts[1:]]
    return [stops[k] for k in sorted(stops) if "groups" in stops[k]]


def load_sweep():
    """开机关节扫描(V1b 第 2 步):look/sweep.txt 每行 = 图名 臂 关节 方向 第几格 拍数 | 各组关节读数 … || 身体报的手的位姿(只打分)"""
    p = os.path.join(RUN, "look", "sweep.txt")
    out = []
    if not os.path.exists(p):
        return out
    for line in open(p):
        if "||" not in line:
            continue
        left, right = line.split("||", 1)
        parts = left.strip().split("|")
        h = parts[0].split()
        nm = h[0]
        ee = [float(x) for x in right.split()]
        if len(ee) != 7:
            continue
        out.append(dict(n="s" + nm.split("_")[1].split(".")[0], img=os.path.join(RUN, "look", nm), arm=int(h[1]), J=int(h[2]),
                        D=int(h[3]), K=int(h[4]), steps=int(h[5]), groups=[np.array([float(x) for x in g.split()]) for g in parts[1:]],
                        pose=np.array(ee), sweep=True))
    return out


def b64(path):
    return base64.b64encode(open(path, "rb").read()).decode()


def match(na, nb, pa=None, pb=None):
    cache = os.path.join(OUT, "m_%s_%s.npy" % (na, nb))
    if os.path.exists(cache):
        return np.load(cache)
    look = os.path.join(RUN, "look")
    pa = pa or os.path.join(look, "board_%s_w.bmp" % na)
    pb = pb or os.path.join(look, "board_%s_w.bmp" % nb)
    body = json.dumps({"a": b64(pa), "b": b64(pb), "num": NUM}).encode()
    req = urllib.request.Request(INST, data=body, headers={"Content-Type": "application/json"})
    r = json.loads(urllib.request.urlopen(req, timeout=300).read())
    s = np.array(r.get("samples", []), float).reshape(-1, 5)
    np.save(cache, s)
    return s


# ---------------- 模型 ----------------
def unpack(x, n):
    W = x[:3 * n].reshape(n, 3)
    P = x[3 * n:6 * n].reshape(n, 3) if len(x) >= 6 * n else np.zeros((n, 3))
    return W, P


def fk(W, P, dq):
    R = np.eye(3)
    t = np.zeros(3)
    for i in range(len(dq)):
        Ri = rot(W[i], dq[i])
        t = t + R @ ((np.eye(3) - Ri) @ P[i])
        R = R @ Ri
    return R, t


def fk_A(W, dq):
    """t = A p(p 按关节依次排成 3n 向量),给定转轴方向时平移对 p 线性"""
    n = len(dq)
    A = np.zeros((3, 3 * n))
    R = np.eye(3)
    for i in range(n):
        Ri = rot(W[i], dq[i])
        A[:, 3 * i:3 * i + 3] = R @ (np.eye(3) - Ri)
        R = R @ Ri
    return R, A


def rod_batch(w, th):
    """一根轴、一串转角 ⇒ 一串转动矩阵 (N,3,3)"""
    w = w / np.linalg.norm(w)
    K = skew(w)
    K2 = K @ K
    s = np.sin(th)[:, None, None]
    c = (1 - np.cos(th))[:, None, None]
    return np.eye(3)[None] + s * K[None] + c * K2[None]


def fk_all(W, P, dQ):
    """所有停一起:眼在参照停眼系里的位姿 R (N,3,3)、t (N,3)"""
    N, n = dQ.shape
    R = np.repeat(np.eye(3)[None], N, 0)
    t = np.zeros((N, 3))
    for i in range(n):
        Ri = rod_batch(W[i], dQ[:, i])
        ti = P[i][None, :] - np.einsum('nab,b->na', Ri, P[i])
        t = t + np.einsum('nab,nb->na', R, ti)
        R = R @ Ri
    return R, t


def fkA_all(W, dQ):
    """给定转轴方向,各停眼的位置对 p 线性:t = A p,A (N,3,3n)"""
    N, n = dQ.shape
    R = np.repeat(np.eye(3)[None], N, 0)
    A = np.zeros((N, 3, 3 * n))
    for i in range(n):
        Ri = rod_batch(W[i], dQ[:, i])
        A[:, :, 3 * i:3 * i + 3] = R @ (np.eye(3)[None] - Ri)
        R = R @ Ri
    return R, A


def log_batch(R):
    tr = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)
    a = np.arccos(tr)
    v = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], 1)
    s = np.sin(a)
    k = np.where(a < 1e-8, 0.5, a / (2 * np.where(np.abs(s) < 1e-12, 1.0, s)))
    return v * k[:, None]


def rotvec(x):
    a = np.linalg.norm(x)
    return rot(x, a) if a > 1e-12 else np.eye(3)



def align_eval(W, P, dQ, train, test, Pw, rng):
    R, t = fk_all(W, P, dQ)
    # 4) 打分:模型系 → 仿真世界(转 Rg、移 tg、倍数 s)+ 眼相对手的偏移(Rx, tx),只用训练停的真值
    pw = np.array([p_ for p_, _ in Pw]); Rw = np.array([R_ for _, R_ in Pw])
    tr = np.array(train)
    def res_align(x):
        Rg = rotvec(x[0:3]); Rx = rotvec(x[3:6])
        s_, tg, tx = x[6], x[7:10], x[10:13]
        rot_e = log_batch(np.transpose(Rw[tr] @ Rx, (0, 2, 1)) @ (Rg[None] @ R[tr]))
        pos_e = (s_ * np.einsum('ab,nb->na', Rg, t[tr]) + tg) - (pw[tr] + np.einsum('nab,b->na', Rw[tr], tx))
        return np.concatenate([0.05 * rot_e.ravel(), pos_e.ravel()])
    bestA = None
    for trial in range(40):
        x0 = np.concatenate([rng.normal(size=3) * math.pi / 2, rng.normal(size=3) * math.pi / 2, [0.1], np.zeros(3), np.zeros(3)])
        ra = least_squares(res_align, x0, method="lm", max_nfev=4000)
        if bestA is None or ra.cost < bestA.cost:
            bestA = ra
    x = bestA.x
    Rg = rotvec(x[0:3]); Rx = rotvec(x[3:6])
    s_, tg, tx = x[6], x[7:10], x[10:13]
    def err(idx):
        idx = np.array(idx)
        dp = (s_ * np.einsum('ab,nb->na', Rg, t[idx]) + tg) - (pw[idx] + np.einsum('nab,b->na', Rw[idx], tx))
        da = np.degrees(np.linalg.norm(log_batch(np.transpose(Rw[idx] @ Rx, (0, 2, 1)) @ (Rg[None] @ R[idx])), axis=1))
        return np.linalg.norm(dp, axis=1) * 1000, da
    etr, atr = err(train); ete, ate = err(test)
    print("对齐:倍数 %.4f(模型单位 → 米),眼离手 (%.1f, %.1f, %.1f) mm" % (s_, *(1000 * tx)))
    print("训练停(%d):眼的位置差 中位 %.2f mm、最大 %.2f mm;朝向差 中位 %.3f°" % (len(train), np.median(etr), etr.max(), np.median(atr)))
    print("考试停(%d,只给关节读数):眼的位置差 中位 %.2f mm、最大 %.2f mm;朝向差 中位 %.3f°、最大 %.3f°" % (len(test), np.median(ete), ete.max(), np.median(ate), ate.max()))
    print("考试逐停(mm):", np.round(ete, 2).tolist())
    return Rg, tg, s_, Rx, tx, etr, atr, ete, ate


def fit_px(dQ, meas, train, n, W0, P0, f0, cx, cy, per_pair=300):
    """所有配对的内点像素一起:x_j^T [t_ij]x R_ij x_i = 0,R/t 由转轴 + 关节读数给;焦距是未知数之一。
    视差太小的对(近乎纯转)只给转动残差(乘焦距换成像素)。尺度按"训练停眼的位置均方根 = 1"钉住"""
    rs = np.random.default_rng(0)
    PI, A, B, rot_pairs = [], [], [], []
    for k, m in enumerate(meas):
        if m["par"] > 0.3 and "a" in m:
            idx = np.arange(len(m["a"]))
            if len(idx) > per_pair:
                idx = rs.choice(idx, per_pair, replace=False)
            PI.append(np.full(len(idx), k)); A.append(m["a"][idx]); B.append(m["b"][idx])
        else:
            rot_pairs.append(k)
    PI = np.concatenate(PI); A = np.vstack(A); B = np.vstack(B)
    I = np.array([m["i"] for m in meas]); J = np.array([m["j"] for m in meas])
    Rm = np.array([m["R"] for m in meas])
    rot_pairs = np.array(rot_pairs, int)
    def res(x):
        W = x[:3 * n].reshape(n, 3); P = x[3 * n:6 * n].reshape(n, 3); f = math.exp(x[6 * n])
        R, t = fk_all(W, P, dQ)
        Rij = np.transpose(R[J], (0, 2, 1)) @ R[I]
        tij = np.einsum('mab,mb->ma', np.transpose(R[J], (0, 2, 1)), t[I] - t[J])
        tn = tij / np.maximum(np.linalg.norm(tij, axis=1, keepdims=True), 1e-12)
        Tx = np.zeros((len(meas), 3, 3))
        Tx[:, 0, 1] = -tn[:, 2]; Tx[:, 0, 2] = tn[:, 1]; Tx[:, 1, 0] = tn[:, 2]
        Tx[:, 1, 2] = -tn[:, 0]; Tx[:, 2, 0] = -tn[:, 1]; Tx[:, 2, 1] = tn[:, 0]
        E = Tx @ Rij
        x1 = np.c_[(A[:, 0] - cx) / f, (A[:, 1] - cy) / f, np.ones(len(A))]
        x2 = np.c_[(B[:, 0] - cx) / f, (B[:, 1] - cy) / f, np.ones(len(B))]
        Ep = E[PI]
        Ex1 = np.einsum('mab,mb->ma', Ep, x1)
        Etx2 = np.einsum('mba,mb->ma', Ep, x2)
        num = np.sum(x2 * Ex1, 1)
        den = np.sqrt(Ex1[:, 0] ** 2 + Ex1[:, 1] ** 2 + Etx2[:, 0] ** 2 + Etx2[:, 1] ** 2) + 1e-12
        samp = f * num / den
        out = [samp]
        if len(rot_pairs):
            Er = np.transpose(Rm[rot_pairs], (0, 2, 1)) @ Rij[rot_pairs]
            out.append(f * log_batch(Er).ravel())
        reg = np.concatenate([np.linalg.norm(W, axis=1) - 1, np.sum(W * P, 1),
                              [math.sqrt(np.mean(np.sum(t[train] ** 2, 1))) - 1.0]])
        out.append(1e3 * reg)
        return np.concatenate(out)
    x0 = np.concatenate([W0.ravel(), P0.ravel(), [math.log(f0)]])
    r = least_squares(res, x0, method="trf", loss="soft_l1", f_scale=1.0, max_nfev=int(os.environ.get("PX_NFEV", "300")), x_scale="jac")
    print("按像素一起解:迭代 %d 次(%s),代价 %.6g" % (r.nfev, r.message, r.cost))
    W = r.x[:3 * n].reshape(n, 3); P = r.x[3 * n:6 * n].reshape(n, 3); f = math.exp(r.x[6 * n])
    W = W / np.linalg.norm(W, axis=1, keepdims=True)
    rr = res(r.x)
    samp = np.abs(rr[:len(PI)])
    print("按像素一起解:%d 个配点、%d 对只给转动;Sampson 残差 中位 %.3f px、九成 %.3f px;焦距 %.1f → %.1f px" %
          (len(PI), len(rot_pairs), np.median(samp), np.quantile(samp, 0.9), f0, f))
    return W, P, f

def fit_arm(dQ, meas, train, test, Pw, n, rng, starts=30):
    """一只手:配对量到的相对转动 / 平移方向 ⇒ 转轴;再用训练停的真值对齐,报考试停的误差(毫米)"""
    I = np.array([m["i"] for m in meas]); J = np.array([m["j"] for m in meas])
    Rm = np.array([m["R"] for m in meas]); Tm = np.array([m["t"] for m in meas])
    good_t = np.array([m["par"] > 0.3 for m in meas])     # 视差太小的对:平移方向不可信(近乎纯转),只用转动
    print("配对 %d(平移方向可信 %d)" % (len(meas), int(good_t.sum())))

    # 1) 只拟合转动:ω(多起点)
    def res_rot(x):
        W = x.reshape(n, 3)
        R, _ = fk_all(W, np.zeros((n, 3)), dQ)
        E = np.transpose(Rm, (0, 2, 1)) @ (np.transpose(R[J], (0, 2, 1)) @ R[I])
        return np.concatenate([log_batch(E).ravel(), np.linalg.norm(W, axis=1) - 1])
    best = None
    for trial in range(starts):
        x0 = rng.normal(size=(n, 3))
        x0 /= np.linalg.norm(x0, axis=1, keepdims=True)
        r = least_squares(res_rot, x0.ravel(), method="trf", loss="soft_l1", f_scale=0.01, max_nfev=3000)
        if best is None or r.cost < best.cost:
            best = r
    W = best.x.reshape(n, 3)
    W = W / np.linalg.norm(W, axis=1, keepdims=True)
    rr = np.degrees(np.linalg.norm(res_rot(W.ravel())[:-n].reshape(-1, 3), axis=1))
    print("转动拟合:每对残差中位 %.3f°、最大 %.3f°" % (np.median(rr), rr.max()))

    # 2) 线性解 p:t_meas × (R_j^T (t_i − t_j)) = 0,‖p‖ = 1;p 沿 ω 的分量不可观 ⇒ 每根轴加一行压成 0
    R, A = fkA_all(W, dQ)
    Ig, Jg, Tg = I[good_t], J[good_t], Tm[good_t]
    M = np.einsum('mab,mbc->mac', np.array([skew(t) for t in Tg]), np.transpose(R[Jg], (0, 2, 1)) @ (A[Ig] - A[Jg]))
    rows = [M.reshape(-1, 3 * n)]
    for i in range(n):
        e = np.zeros((1, 3 * n)); e[0, 3 * i:3 * i + 3] = W[i]
        rows.append(e)
    B = np.vstack(rows)
    _, sv, Vt = np.linalg.svd(B)
    p = Vt[-1]
    tmod = np.einsum('mab,mb->ma', np.transpose(R[Jg], (0, 2, 1)) @ (A[Ig] - A[Jg]), np.repeat(p[None], len(Ig), 0))
    if np.sum(Tg * tmod) < 0:
        p = -p
    P = p.reshape(n, 3)
    print("线性解 p:最小两个奇异值 %.3g / %.3g" % (sv[-1], sv[-2]))

    # 3) 一起精修(转动 + 平移方向),尺度按"训练停眼的位置均方根 = 1"钉住
    def res_all(x):
        W_ = x[:3 * n].reshape(n, 3); P_ = x[3 * n:].reshape(n, 3)
        R_, t_ = fk_all(W_, P_, dQ)
        E = np.transpose(Rm, (0, 2, 1)) @ (np.transpose(R_[J], (0, 2, 1)) @ R_[I])
        tm = np.einsum('mab,mb->ma', np.transpose(R_[J], (0, 2, 1)), t_[I] - t_[J])
        nt = np.linalg.norm(tm, axis=1, keepdims=True)
        # 两个单位向量之差(不是叉积):叉积对"正好反过来"也是 0,右手就收敛到了反过来的那一支(V1B1 第二轮,方向残差中位 173°)
        ct = (Tm - tm / np.maximum(nt, 1e-12)) * good_t[:, None]
        reg = np.concatenate([np.linalg.norm(W_, axis=1) - 1, np.sum(W_ * P_, 1),
                              [math.sqrt(np.mean(np.sum(t_[train] ** 2, 1))) - 1.0]])
        return np.concatenate([log_batch(E).ravel(), ct.ravel(), reg])
    r = least_squares(res_all, np.concatenate([W.ravel(), P.ravel()]), method="trf", loss="soft_l1", f_scale=0.02, max_nfev=20000)
    W = r.x[:3 * n].reshape(n, 3); P = r.x[3 * n:].reshape(n, 3)
    W = W / np.linalg.norm(W, axis=1, keepdims=True)
    R, t = fk_all(W, P, dQ)
    E = np.transpose(Rm, (0, 2, 1)) @ (np.transpose(R[J], (0, 2, 1)) @ R[I])
    rr = np.degrees(np.linalg.norm(log_batch(E), axis=1))
    tm = np.einsum('mab,mb->ma', np.transpose(R[J], (0, 2, 1)), t[I] - t[J])
    cosang = np.sum(Tm * tm, 1) / np.linalg.norm(tm, axis=1)
    tt = np.degrees(np.arccos(np.clip(cosang[good_t], -1, 1)))
    print("精修后:转动残差中位 %.3f° 最大 %.3f°;平移方向残差中位 %.2f° 最大 %.2f°" % (np.median(rr), rr.max(), np.median(tt) if len(tt) else -1, tt.max() if len(tt) else -1))

    Rg, tg, s_, Rx, tx, etr, atr, ete, ate = align_eval(W, P, dQ, train, test, Pw, rng)
    model = dict(W=W, P=P, Rg=Rg, tg=tg, s=s_, Rx=Rx, tx=tx)
    return dict(model=model, fit_rot_med_deg=float(np.median(rr)), fit_dir_med_deg=float(np.median(tt)) if len(tt) else -1.0,
                train_med_mm=float(np.median(etr)), test_med_mm=float(np.median(ete)), test_max_mm=float(ete.max()),
                test_med_deg=float(np.median(ate)), scale=float(s_), axes=W.round(4).tolist(), points=P.round(4).tolist(), test_mm=ete.round(3).tolist())



def eval_frames(arm, gi, q0, Qfit, model, run=None):
    """外推考试:录像里每一帧只给关节读数 ⇒ 按量出来的转轴算眼在哪,和仿真报的手的位姿(真值,经同一个对齐)比。
    按"这一帧离最近的标定停,关节最多差几度"分档报。只用静止帧(前后帧关节读数不变)"""
    vid = os.path.join(run or RUN, "vid")
    pj = os.path.join(vid, "joints.txt"); pp = os.path.join(vid, "poses.txt")
    if not (os.path.exists(pj) and os.path.exists(pp)):
        print("没有录像的关节 / 位姿,外推考试跳过"); return None
    poses = {}
    for line in open(pp):
        f = line.split()
        if len(f) >= 2 + 7 * (arm + 1):
            poses[int(f[0])] = np.array([float(x) for x in f[2 + 7 * arm: 2 + 7 * arm + 7]])
    joints = {}
    for line in open(pj):
        parts = line.strip().split("|")
        seq = int(parts[0].split()[0])
        g = [np.array([float(x) for x in p.split()]) for p in parts[1:]]
        if gi < len(g):
            joints[seq] = g[gi]
    seqs = sorted(set(poses) & set(joints))
    still = [s for s in seqs if (s - 1) in joints and np.max(np.abs(joints[s] - joints[s - 1])) < 1e-6]
    if not still:
        print("外推考试:没有静止帧"); return None
    Q = np.array([joints[s] for s in still]); dQ = Q - q0
    R, t = fk_all(model["W"], model["P"], dQ)
    pw = np.array([poses[s][:3] for s in still]); Rw = np.array([quat_to_R(poses[s][3:]) for s in still])
    pred = model["s"] * np.einsum('ab,nb->na', model["Rg"], t) + model["tg"]
    true = pw + np.einsum('nab,b->na', Rw, model["tx"])
    e = np.linalg.norm(pred - true, axis=1) * 1000
    dist = np.degrees(np.min(np.max(np.abs(Q[:, None, :] - Qfit[None, :, :]), axis=2), axis=1))
    # 静止帧里很多是同一个姿势停着 ⇒ 按"不同姿势"数(关节读数取到 0.001 弧度一样的算一个),每个姿势取一帧
    key = np.round(Q / 1e-3).astype(np.int64)
    _, first = np.unique(key, axis=0, return_index=True)
    uniq = np.zeros(len(Q), bool); uniq[first] = True
    print("外推考试:%d 个静止帧(只给关节读数),其中不同姿势 %d 个" % (len(still), int(uniq.sum())))
    out = []
    for lo, hi in ((0, 2), (2, 5), (5, 10), (10, 20), (20, 40), (40, 180)):
        m = (dist >= lo) & (dist < hi) & uniq
        if m.sum():
            out.append(dict(lo=lo, hi=hi, n=int(m.sum()), med_mm=float(np.median(e[m])), max_mm=float(e[m].max())))
            print("  离标定停 %2d–%3d°:不同姿势 %4d 个,眼的位置误差 中位 %7.2f mm、最大 %7.2f mm" % (lo, hi, m.sum(), np.median(e[m]), e[m].max()))
    return out


def extra_frames(arm, gi, cam, S, k):
    vid = os.path.join(RUN, "vid")
    pj = os.path.join(vid, "joints.txt"); pp = os.path.join(vid, "poses.txt")
    joints, vidn, poses = {}, {}, {}
    for line in open(pj):
        parts = line.strip().split("|")
        h = parts[0].split()
        seq, vn = int(h[0]), int(h[1])
        g = [np.array([float(x) for x in p.split()]) for p in parts[1:]]
        if gi < len(g):
            joints[seq] = g[gi]; vidn[seq] = vn
    for line in open(pp):
        f = line.split()
        if len(f) >= 2 + 7 * (arm + 1):
            poses[int(f[0])] = np.array([float(x) for x in f[2 + 7 * arm: 2 + 7 * arm + 7]])
    cand = []
    for s in sorted(joints):
        if vidn.get(s, -1) < 0 or s not in poses:
            continue
        if all((s - d) in joints and np.max(np.abs(joints[s] - joints[s - d])) < 1e-6 for d in (1, 2, 3)):
            img = os.path.join(vid, "f%06d_c%d.pgm" % (vidn[s], cam))
            if os.path.exists(img):
                cand.append((s, img))
    if not cand or k <= 0:
        return []
    Qc = np.array([joints[s] for s, _ in cand])
    chosen = [s_["groups"][gi] for s_ in S]
    picked = []
    d = np.min(np.max(np.abs(Qc[:, None, :] - np.array(chosen)[None, :, :]), axis=2), axis=1)
    for _ in range(k):
        i = int(np.argmax(d))
        if d[i] < 1e-4:
            break
        s, img = cand[i]
        picked.append(dict(n="v%d" % s, img=img, pose=poses[s], groups=[joints[s] if j == gi else np.zeros(1) for j in range(gi + 1)]))
        d = np.minimum(d, np.max(np.abs(Qc - Qc[i]), axis=1))
    return picked

def main():
    os.makedirs(OUT, exist_ok=True)
    groups = load_groups()
    stops = load_stops()
    global sweep
    sweep = load_sweep()
    print("扫描帧:", len(sweep))
    geo = json.load(open(GEO))["cams"]
    print("关节组:", groups)
    print("板上的停:", len(stops))
    report = {}
    for arm in sorted(set(s["arm"] for s in stops)):
        side = "left" if arm == 0 else "right"
        gi = [i for i, g in enumerate(groups) if g.startswith("state.") and side in g]
        if not gi:
            print("臂 %d:找不到关节组" % arm); continue
        gi = gi[0]
        S = [s for s in stops if s["arm"] == arm]
        cam = S[0]["cam"]
        SW = [s_ for s_ in sweep if s_["arm"] == arm]
        if SW:
            others = [j for j in range(len(SW[0]["groups"])) if j != gi]
            drift = {j: float(np.degrees(np.max(np.abs(np.array([s_["groups"][j] for s_ in SW]) - SW[0]["groups"][j])))) for j in others}
            print("扫描帧 %d 张;扫这只手时别的关节组最多漂了(度):%s" % (len(SW), {groups[j]: round(v, 3) for j, v in drift.items()}))
            runs = {}
            for s_ in SW:
                runs.setdefault((s_["J"], s_["D"]), []).append(s_)
            for (j_, d_), rr in sorted(runs.items()):
                if j_ == 0 and d_ == 0:
                    continue
                q = np.array([r_["groups"][gi][j_] for r_ in rr])
                print("  关节 %d 往%s:%d 格,读数从起点转了 %.1f°(最后一格)" % (j_, "正" if d_ > 0 else "负", len(rr), np.degrees(q[-1] - SW[0]["groups"][gi][j_])))
        f, cx, cy = geo[cam]["f"], geo[cam]["cx"], geo[cam]["cy"]
        if os.environ.get("F0"):
            print("焦距起点改成 %s(驱动量的是 %.1f,只当对照)" % (os.environ["F0"], f))
            f = float(os.environ["F0"])
        K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])
        Q = np.array([s["groups"][gi] for s in S])
        n = Q.shape[1]
        q0 = Q[0]
        dQ = Q - q0
        print("\n===== 臂 %d(%s,眼 %d,焦距 %.1f):%d 停,%d 个关节" % (arm, side, cam, f, len(S), n))
        print("每个关节在这些停里转过的范围(度):", np.round(np.degrees(dQ.max(0) - dQ.min(0)), 1).tolist())
        # 训练 / 考试:板停每三停留一停考试;录像里挑的关节离得远的帧全进训练
        nb = len(S)
        idx = list(range(nb))
        test = [i for i in idx if i % 3 == 2]
        n_board = len(S)
        for s_ in SW:
            S.append(s_)
        extra = extra_frames(arm, gi, cam, S, int(os.environ.get("EXTRA", "0")))
        for e in extra:
            S.append(e)
        if SW:
            Q = np.array([s["groups"][gi] for s in S]); dQ = Q - q0
            print("加进扫描帧 %d 张 ⇒ 训练里每个关节转过的范围(度):" % len(SW),
                  np.round(np.degrees(dQ[[i for i in range(len(S)) if i not in test]].max(0) - dQ[[i for i in range(len(S)) if i not in test]].min(0)), 1).tolist())
        if extra:
            Q = np.array([s["groups"][gi] for s in S]); dQ = Q - q0
            print("加进录像里关节离得远的静止帧 %d 个 ⇒ 训练停里每个关节转过的范围(度):" % len(extra),
                  np.round(np.degrees(dQ[[i for i in range(len(S)) if i not in test]].max(0) - dQ[[i for i in range(len(S)) if i not in test]].min(0)), 1).tolist())
        train = [i for i in range(len(S)) if i not in test]
        # 配对(只在训练停之间):每一停配它关节上最近的 NN 个邻居(视野重叠才配得上),去重
        NN = int(os.environ.get("NN", "6"))
        pairs = set()
        for i in train:
            dd = sorted(((np.max(np.abs(dQ[i] - dQ[j])), j) for j in train if j != i))
            for _, j in dd[:NN]:
                pairs.add((min(i, j), max(i, j)))
        # 扫描:每一格和上一格(同一个关节同一个方向)、每一格和扫描起点
        sw_idx = [i for i in range(len(S)) if S[i].get("sweep")]
        if sw_idx:
            start = [i for i in sw_idx if S[i]["J"] == 0 and S[i]["D"] == 0 and S[i]["K"] == 0]
            st = start[0] if start else sw_idx[0]
            prev = {}
            for i in sw_idx:
                if i == st:
                    continue
                key = (S[i]["J"], S[i]["D"])
                a_ = prev.get(key, st)
                pairs.add((min(a_, i), max(a_, i)))
                pairs.add((min(st, i), max(st, i)))
                prev[key] = i
        pairs = sorted(pairs)
        random.Random(SEED).shuffle(pairs)
        pairs = pairs[:MAXPAIRS]
        print("配对候选 %d" % len(pairs))
        t0 = time.time()
        raw = {}
        for k, (i, j) in enumerate(pairs):
            s = match(S[i]["n"], S[j]["n"], S[i].get("img"), S[j].get("img"))
            if len(s) >= 50:
                raw[(i, j)] = s
            if k % 25 == 0:
                print("  配对 %d/%d(%.0f s)" % (k, len(pairs), time.time() - t0), flush=True)

        def build_meas(Kf):
            out = []
            for (i, j), s in raw.items():
                # 手指跟着眼一起动 ⇒ 画面里不动,配在手指上的点会冒充"眼没动";手指在画面下部(驱动量的握区框从 FINGER_V 行起),两边都去掉
                keep = (s[:, 1] < FINGER_V) & (s[:, 3] < FINGER_V)
                s = s[keep]
                if len(s) < 50:
                    continue
                a, b = s[:, 0:2].astype(np.float64), s[:, 2:4].astype(np.float64)
                E, mask = cv2.findEssentialMat(a, b, Kf, method=cv2.RANSAC, prob=0.99999, threshold=1.0)
                if E is None or E.shape != (3, 3):
                    continue
                ninl, R, t, mask2 = cv2.recoverPose(E, a, b, Kf, mask=mask)
                if ninl < 80:
                    continue
                m = mask2.ravel() > 0
                xa = cv2.undistortPoints(a[m].reshape(-1, 1, 2), Kf, None).reshape(-1, 2)
                xb = cv2.undistortPoints(b[m].reshape(-1, 1, 2), Kf, None).reshape(-1, 2)
                ra = np.c_[xa, np.ones(len(xa))]; rb = np.c_[xb, np.ones(len(xb))]
                ra /= np.linalg.norm(ra, axis=1, keepdims=True); rb /= np.linalg.norm(rb, axis=1, keepdims=True)
                par = np.degrees(np.median(np.arccos(np.clip(np.sum((ra @ R.T) * rb, 1), -1, 1))))
                out.append(dict(i=i, j=j, R=R, t=t.ravel() / np.linalg.norm(t), ninl=int(ninl), par=par, a=a[m], b=b[m]))
            return out
        meas = build_meas(K)
        print("可用配对:%d / %d" % (len(meas), len(pairs)))
        if len(meas) < 3 * n:
            print("配对太少,这只手不解"); continue
        Pw = [(S[i]["pose"][:3], quat_to_R(S[i]["pose"][3:])) for i in range(len(S))]
        res = fit_arm(dQ, meas, train, test, Pw, n, rng)
        scan = []
        for fs in [float(x) for x in os.environ.get("FOCALS", "").split(",") if x.strip()]:
            Kf = K.copy(); Kf[0, 0] *= fs; Kf[1, 1] *= fs
            mf = build_meas(Kf)
            import io, contextlib
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rf = fit_arm(dQ, mf, train, test, Pw, n, np.random.default_rng(SEED))
            scan.append(dict(focal=float(Kf[0, 0]), fit_rot_med_deg=rf["fit_rot_med_deg"], fit_dir_med_deg=rf["fit_dir_med_deg"], test_med_mm=rf["test_med_mm"]))
            print("  焦距 %.1f:拟合残差 转动 %.4f° / 平移方向 %.3f°(不看真值) · 考试停误差 中位 %.2f mm(看真值)" % (Kf[0, 0], rf["fit_rot_med_deg"], rf["fit_dir_med_deg"], rf["test_med_mm"]), flush=True)
        res["focal_scan"] = scan
        res["extrap"] = eval_frames(arm, gi, q0, Q[train], res["model"])
        if os.environ.get("PX", "0") != "0":
            print("—— 按像素一起解(转轴 + 焦距),从两两那一份起步 ——")
            Wp, Pp, fp = fit_px(dQ, meas, train, n, res["model"]["W"], res["model"]["P"], f, cx, cy)
            Rg, tg, s_, Rx, tx, etr, atr, ete, ate = align_eval(Wp, Pp, dQ, train, test, Pw, rng)
            mp = dict(W=Wp, P=Pp, Rg=Rg, tg=tg, s=s_, Rx=Rx, tx=tx)
            res["px"] = dict(focal=fp, train_med_mm=float(np.median(etr)), test_med_mm=float(np.median(ete)), test_max_mm=float(ete.max()),
                             extrap=eval_frames(arm, gi, q0, Q[train], mp))
        res.pop("model")
        res.update(stops=len(S), pairs=len(meas), train=len(train), test=len(test), joint_range_deg=np.degrees(dQ.max(0) - dQ.min(0)).round(2).tolist())
        report[arm] = res
    json.dump(report, open(os.path.join(OUT, "report.json"), "w"), indent=1, ensure_ascii=False)
    print("\n结果存在", os.path.join(OUT, "report.json"))


if __name__ == "__main__":
    main()
