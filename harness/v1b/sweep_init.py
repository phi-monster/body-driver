# -*- coding: utf-8 -*-
"""V1b 第 2 步的起步(不借焦距、不用本质矩阵分解):每个关节单独扫开的那几格,相机的运动只有这一根轴的转动,转角 = 关节读数的差(已知)。
一根轴 = 方向 ω(2 个数)+ 轴离眼的方位 φ(1 个数;远近是这根轴自己的倍数,单看这一根轴量不出)+ 焦距 f(大家共用)。
每个关节:把"起点 ↔ 每一格""相邻两格"的配点放在一起,按像素 Sampson 残差的中位数在 (ω, φ, f) 上铺网格找最小,再就地精修。
然后各轴远近的倍数:用不同关节的格子之间的配点(两根轴一起动)解;最后交给 fit_px 一起解。"""
import math, os
import numpy as np
from scipy.optimize import least_squares


def fib_sphere(n):
    i = np.arange(n) + 0.5
    ph = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.c_[np.cos(th) * np.sin(ph), np.sin(th) * np.sin(ph), np.cos(ph)]


def perp_basis(w):
    a = np.array([1.0, 0, 0]) if abs(w[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(w, a); e1 /= np.linalg.norm(e1)
    return e1, np.cross(w, e1)


def rod(w, th):
    """一根单位轴 w、一串转角 th ⇒ (N,3,3)"""
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    s = np.sin(th)[:, None, None]; c = (1 - np.cos(th))[:, None, None]
    return np.eye(3)[None] + s * K[None] + c * (K @ K)[None]


def samp(Rij, tij, A, B, f, cx, cy):
    """每个配点的 Sampson 残差(像素)。Rij/tij 已按配点展开 (m,3,3)/(m,3):X_j = Rij X_i + tij"""
    tn = tij / np.maximum(np.linalg.norm(tij, axis=1, keepdims=True), 1e-12)
    Tx = np.zeros((len(tn), 3, 3))
    Tx[:, 0, 1] = -tn[:, 2]; Tx[:, 0, 2] = tn[:, 1]; Tx[:, 1, 0] = tn[:, 2]
    Tx[:, 1, 2] = -tn[:, 0]; Tx[:, 2, 0] = -tn[:, 1]; Tx[:, 2, 1] = tn[:, 0]
    E = Tx @ Rij
    x1 = np.c_[(A[:, 0] - cx) / f, (A[:, 1] - cy) / f, np.ones(len(A))]
    x2 = np.c_[(B[:, 0] - cx) / f, (B[:, 1] - cy) / f, np.ones(len(B))]
    Ex1 = np.einsum('mab,mb->ma', E, x1)
    Etx2 = np.einsum('mba,mb->ma', E, x2)
    num = np.sum(x2 * Ex1, 1)
    den = np.sqrt(Ex1[:, 0] ** 2 + Ex1[:, 1] ** 2 + Etx2[:, 0] ** 2 + Etx2[:, 1] ** 2) + 1e-12
    return f * num / den


def one_joint(th, pairs, cx, cy, fgrid, nsph=600, nphi=8, per_pair=200, seed=0, top=10, stages=(8.0, 16.0, 360.0)):
    """th: 这个关节各帧的转角(相对起点,弧度);pairs: [(i, j, A, B)] 帧下标 + 配点像素。返回 (ω, p̂, f, 中位残差)"""
    rs = np.random.default_rng(seed)
    I, J, A, B = [], [], [], []
    for i, j, a, b in pairs:
        idx = np.arange(len(a))
        if len(idx) > per_pair:
            idx = rs.choice(idx, per_pair, replace=False)
        I.append(np.full(len(idx), i)); J.append(np.full(len(idx), j)); A.append(a[idx]); B.append(b[idx])
    I = np.concatenate(I); J = np.concatenate(J); A = np.vstack(A); B = np.vstack(B)
    dth = np.degrees(np.abs(th[J] - th[I]))         # 每个配点所在那一对之间转了多少
    I_all, J_all, A_all, B_all = I, J, A, B
    # 由粗到细:转角大的对让残差的坑很窄,网格点落不进去(合成:肩关节 ±45° 找到的坑残差 1.9 px,真解 0.38 px)
    # ⇒ 网格只用转角最小的那一档;精修一档一档放宽到全部
    m0 = dth <= stages[0]
    if m0.sum() < 200:
        m0 = dth <= np.sort(dth)[min(len(dth) - 1, 199)]
    I, J, A, B = I_all[m0], J_all[m0], A_all[m0], B_all[m0]

    def model(w, phi, fr_th):
        e1, e2 = perp_basis(w)
        p = math.cos(phi) * e1 + math.sin(phi) * e2
        R = rod(w, fr_th)
        t = p[None] - np.einsum('nab,b->na', R, p)
        return R, t

    def score(w, phi, f):
        R, t = model(w, phi, th)
        Rij = np.transpose(R[J], (0, 2, 1)) @ R[I]
        tij = np.einsum('mab,mb->ma', np.transpose(R[J], (0, 2, 1)), t[I] - t[J])
        return np.median(np.abs(samp(Rij, tij, A, B, f, cx, cy)))

    cand = []
    phis = np.arange(nphi) * 2 * math.pi / nphi
    cph, sph = np.cos(phis), np.sin(phis)
    X1 = {f: np.c_[(A[:, 0] - cx) / f, (A[:, 1] - cy) / f, np.ones(len(A))] for f in fgrid}
    X2 = {f: np.c_[(B[:, 0] - cx) / f, (B[:, 1] - cy) / f, np.ones(len(B))] for f in fgrid}
    for w in fib_sphere(nsph):
        R = rod(w, th)
        Rij = np.transpose(R[J], (0, 2, 1)) @ R[I]
        e1, e2 = perp_basis(w)
        # 一根轴的转动:t_ij = (I − R_ij) p,p = cos φ e1 + sin φ e2 ⇒ 对 φ 线性,各个 φ 一次算完
        u1 = e1[None] - np.einsum('mab,b->ma', Rij, e1)
        u2 = e2[None] - np.einsum('mab,b->ma', Rij, e2)
        T = cph[:, None, None] * u1[None] + sph[:, None, None] * u2[None]          # (nφ, m, 3)
        T = T / np.maximum(np.linalg.norm(T, axis=2, keepdims=True), 1e-12)
        for f in fgrid:
            y = np.einsum('mab,mb->ma', Rij, X1[f])                                 # R x1
            x2 = X2[f]
            Ex1 = np.cross(T, y[None])                                              # [t]x R x1
            num = np.sum(x2[None] * Ex1, 2)
            Etx2 = -np.einsum('mba,pmb->pma', Rij, np.cross(T, x2[None]))           # (R^T [t]x^T) x2 = −R^T (t × x2)
            den = np.sqrt(Ex1[:, :, 0] ** 2 + Ex1[:, :, 1] ** 2 + Etx2[:, :, 0] ** 2 + Etx2[:, :, 1] ** 2) + 1e-12
            sc = np.median(np.abs(f * num / den), axis=1)
            for k in range(nphi):
                cand.append((float(sc[k]), w, phis[k], f))
    cand.sort(key=lambda c: c[0])

    def unpack(x):
        w = np.array([math.sin(x[0]) * math.cos(x[1]), math.sin(x[0]) * math.sin(x[1]), math.cos(x[0])])
        return w, x[2], math.exp(x[3])

    def res_on(x, m):
        w, phi, f = unpack(x)
        R, t = model(w, phi, th)
        Ii, Jj = I_all[m], J_all[m]
        Rij = np.transpose(R[Jj], (0, 2, 1)) @ R[Ii]
        tij = np.einsum('mab,mb->ma', np.transpose(R[Jj], (0, 2, 1)), t[Ii] - t[Jj])
        return samp(Rij, tij, A_all[m], B_all[m], f, cx, cy)
    masks = [dth <= st for st in stages]
    masks[0] = m0
    everything = np.ones(len(dth), bool)
    # 网格上最好的 top 个(彼此不同的)各自一档一档精修,按全部配点的残差中位数挑
    picked, out = [], None
    for sc, w0, phi0, f0 in cand:
        if any(abs(f0 - f1) < 1e-9 and np.dot(w0, w1) > math.cos(math.radians(15)) for w1, f1 in picked):
            continue
        picked.append((w0, f0))
        x = np.array([math.acos(np.clip(w0[2], -1, 1)), math.atan2(w0[1], w0[0]), phi0, math.log(f0)])
        for m in masks:
            if m.sum() >= 50:
                x = least_squares(lambda y: res_on(y, m), x, method="trf", loss="soft_l1", f_scale=1.0, max_nfev=400).x
        med = float(np.median(np.abs(res_on(x, everything))))
        if out is None or med < out[0]:
            out = (med, x, sc)
        if len(picked) >= top:
            break
    w, phi, f = unpack(out[1])
    e1, e2 = perp_basis(w)
    return w, math.cos(phi) * e1 + math.sin(phi) * e2, f, out[0], out[2]


def expand(pairs, per_pair, seed=0):
    rs = np.random.default_rng(seed)
    I, J, A, B = [], [], [], []
    for i, j, a, b in pairs:
        idx = np.arange(len(a))
        if len(idx) > per_pair:
            idx = rs.choice(idx, per_pair, replace=False)
        I.append(np.full(len(idx), i)); J.append(np.full(len(idx), j)); A.append(a[idx]); B.append(b[idx])
    return np.concatenate(I), np.concatenate(J), np.vstack(A), np.vstack(B)


def fk_all(W, P, dQ):
    N, n = dQ.shape
    R = np.repeat(np.eye(3)[None], N, 0)
    t = np.zeros((N, 3))
    for i in range(n):
        Ri = rod(W[i] / np.linalg.norm(W[i]), dQ[:, i])
        ti = P[i][None, :] - np.einsum('nab,b->na', Ri, P[i])
        t = t + np.einsum('nab,nb->na', R, ti)
        R = R @ Ri
    return R, t


def pair_res(W, P, f, dQ, I, J, A, B, cx, cy):
    R, t = fk_all(W, P, dQ)
    Rij = np.transpose(R[J], (0, 2, 1)) @ R[I]
    tij = np.einsum('mab,mb->ma', np.transpose(R[J], (0, 2, 1)), t[I] - t[J])
    return samp(Rij, tij, A, B, f, cx, cy)


def scales(W, Ph, f, dQ, joint_of, pairs, cx, cy, per_pair=200):
    """各轴远近的倍数 ρ(可正可负:Sampson 分不出一根轴在眼的这边还是那边)。
    joint_of[i] = 第 i 帧是哪个关节单独扫出来的(-1 = 起点 / 板停 / 多关节)。
    先以扫出来平移最大的那根轴为 1;别的每根轴只用"它的格子 ↔ 参照轴的格子"那些对,一维网格找 ρ;最后全部一起精修。"""
    n = len(W)
    I, J, A, B = expand(pairs, per_pair)
    jo = np.asarray(joint_of)
    grid = np.concatenate([-np.geomspace(0.01, 10, 31), np.geomspace(0.01, 10, 31)])
    # 参照轴:先各自取 ρ = 1,看哪根轴自己那几对的平移方向最确定 —— 这里简单取"跟别的轴配对最多"的那根
    cnt = np.zeros(n)
    for a_, b_ in zip(jo[I], jo[J]):
        if a_ >= 0 and b_ >= 0 and a_ != b_:
            cnt[a_] += 1; cnt[b_] += 1
    ref = int(np.argmax(cnt))
    rho = np.ones(n)
    for j in range(n):
        if j == ref:
            continue
        m = ((jo[I] == j) & (jo[J] == ref)) | ((jo[I] == ref) & (jo[J] == j))
        if m.sum() < 30:
            continue
        best = None
        for g in grid:
            r_ = rho.copy(); r_[j] = g
            P = Ph * r_[:, None]
            s_ = np.median(np.abs(pair_res(W, P, f, dQ, I[m], J[m], A[m], B[m], cx, cy)))
            if best is None or s_ < best[0]:
                best = (s_, g)
        rho[j] = best[1]

    def res(x):
        r_ = np.r_[x[:ref], 1.0, x[ref:]]
        return pair_res(W, Ph * r_[:, None], f, dQ, I, J, A, B, cx, cy)
    x0 = np.r_[rho[:ref], rho[ref + 1:]]
    r = least_squares(res, x0, method="trf", loss="soft_l1", f_scale=1.0, max_nfev=300)
    rho = np.r_[r.x[:ref], 1.0, r.x[ref:]]
    return rho, ref, float(np.median(np.abs(res(r.x))))


def best_phi(Rij, x1, x2, e1, e2, f, A, B, cx, cy, iters=3):
    """轴方向、焦距给定 ⇒ 轴在眼的哪边(φ)直接解:x2ᵀ[t]×R x1 = 0,t = (I − R) p ⇒ 对 p 线性:gᵀp = 0,g = (I − R)ᵀ((R x1) × x2)。
    p 只在垂直于轴的那个平面里(沿轴那一分量不起作用)⇒ 2×2 的最小特征向量;
    再按 Sampson 的换算(像素 = f · 代数残差 / 分母)加权、Huber 1 px,重解几遍(迭代加权最小二乘)。"""
    y = np.einsum('mab,mb->ma', Rij, x1)
    c = np.cross(y, x2)
    g = c - np.einsum('mba,mb->ma', Rij, c)
    G = np.c_[g @ e1, g @ e2]
    w = 1.0 / np.maximum(np.sum(G * G, 1), 1e-18)
    r = None
    for it in range(iters + 1):
        M = (G * w[:, None]).T @ G
        ev, V = np.linalg.eigh(M)
        cph, sph = V[0, 0], V[1, 0]
        p = cph * e1 + sph * e2
        t = p[None] - np.einsum('mab,b->ma', Rij, p)              # 不归一的 t:代数残差 = c·t
        Ex1 = np.cross(t, y)
        Etx2 = -np.einsum('mba,mb->ma', Rij, np.cross(t, x2))
        den = np.sqrt(Ex1[:, 0] ** 2 + Ex1[:, 1] ** 2 + Etx2[:, 0] ** 2 + Etx2[:, 1] ** 2) + 1e-18
        r = f * np.sum(c * t, 1) / den                             # Sampson(像素)
        if it == iters:
            break
        a = np.abs(r)
        hub = np.where(a <= 1.0, 1.0, 1.0 / np.maximum(a, 1e-12))
        w = hub * (f / den) ** 2
    return math.atan2(sph, cph), float(np.median(np.abs(r)))


def one_joint2(th, pairs, cx, cy, fgrid, nsph=3000, per_pair=200, seed=0, top=20, grid_deg=16.0, stages=(16.0, 360.0)):
    """同 one_joint,但 φ 不铺网格(best_phi 直接解);轴方向 3000 点、焦距 fgrid 档;网格用转角 ≤ grid_deg 的对"""
    I_all, J_all, A_all, B_all = expand(pairs, per_pair, seed)
    dth = np.degrees(np.abs(th[J_all] - th[I_all]))
    m0 = dth <= grid_deg
    if m0.sum() < 200:
        m0 = dth <= np.sort(dth)[min(len(dth) - 1, 199)]
    I, J, A, B = I_all[m0], J_all[m0], A_all[m0], B_all[m0]
    X1 = {f: np.c_[(A[:, 0] - cx) / f, (A[:, 1] - cy) / f, np.ones(len(A))] for f in fgrid}
    X2 = {f: np.c_[(B[:, 0] - cx) / f, (B[:, 1] - cy) / f, np.ones(len(B))] for f in fgrid}
    cand = []
    for w in fib_sphere(nsph):
        R = rod(w, th)
        Rij = np.transpose(R[J], (0, 2, 1)) @ R[I]
        e1, e2 = perp_basis(w)
        for f in fgrid:
            phi, sc = best_phi(Rij, X1[f], X2[f], e1, e2, f, A, B, cx, cy, iters=1)
            cand.append((sc, w, phi, f))
    cand.sort(key=lambda c: c[0])

    def unpack(x):
        w = np.array([math.sin(x[0]) * math.cos(x[1]), math.sin(x[0]) * math.sin(x[1]), math.cos(x[0])])
        return w, x[2], math.exp(x[3])

    def res_on(x, m):
        w, phi, f = unpack(x)
        e1, e2 = perp_basis(w)
        p = math.cos(phi) * e1 + math.sin(phi) * e2
        R = rod(w, th)
        Ii, Jj = I_all[m], J_all[m]
        Rij = np.transpose(R[Jj], (0, 2, 1)) @ R[Ii]
        tij = p[None] - np.einsum('mab,b->ma', Rij, p)
        return samp(Rij, tij, A_all[m], B_all[m], f, cx, cy)
    masks = [m0] + [dth <= st for st in stages]
    everything = np.ones(len(dth), bool)
    picked, out = [], None
    for sc, w0, phi0, f0 in cand:
        if any(abs(math.log(f0 / f1)) < 0.05 and np.dot(w0, w1) > math.cos(math.radians(10)) for w1, f1 in picked):
            continue
        picked.append((w0, f0))
        # perp_basis 随轴变:精修用的参数化(两角 + φ)里,φ 要换到同一个基下 —— unpack 里用的就是 perp_basis(w),与网格一致
        x = np.array([math.acos(np.clip(w0[2], -1, 1)), math.atan2(w0[1], w0[0]), phi0, math.log(f0)])
        for m in masks:
            if m.sum() >= 50:
                x = least_squares(lambda y: res_on(y, m), x, method="trf", loss="soft_l1", f_scale=1.0, max_nfev=400).x
        med = float(np.median(np.abs(res_on(x, everything))))
        if out is None or med < out[0]:
            out = (med, x, sc)
        if len(picked) >= top:
            break
    w, phi, f = unpack(out[1])
    e1, e2 = perp_basis(w)
    return w, math.cos(phi) * e1 + math.sin(phi) * e2, f, out[0], out[2]


def joint_table(th, pairs, cx, cy, fgrid, nsph=3000, per_pair=200, seed=0, grid_deg=16.0, keep=20):
    """一个关节:每一档焦距下,轴方向网格上残差最小的 keep 个 (分数, ω, φ)。φ 由 best_phi 直接解。"""
    I_all, J_all, A_all, B_all = expand(pairs, per_pair, seed)
    dth = np.degrees(np.abs(th[J_all] - th[I_all]))
    m0 = dth <= grid_deg
    if m0.sum() < 200:
        m0 = dth <= np.sort(dth)[min(len(dth) - 1, 199)]
    I, J, A, B = I_all[m0], J_all[m0], A_all[m0], B_all[m0]
    X1 = {f: np.c_[(A[:, 0] - cx) / f, (A[:, 1] - cy) / f, np.ones(len(A))] for f in fgrid}
    X2 = {f: np.c_[(B[:, 0] - cx) / f, (B[:, 1] - cy) / f, np.ones(len(B))] for f in fgrid}
    tab = {f: [] for f in fgrid}
    for w in fib_sphere(nsph):
        R = rod(w, th)
        Rij = np.transpose(R[J], (0, 2, 1)) @ R[I]
        e1, e2 = perp_basis(w)
        for f in fgrid:
            phi, sc = best_phi(Rij, X1[f], X2[f], e1, e2, f, A, B, cx, cy, iters=1)
            tab[f].append((sc, w, phi))
    for f in fgrid:
        tab[f].sort(key=lambda c: c[0])
        # 彼此相距 ≥ 10° 的才留(同一个坑只留一个)
        kept = []
        for c in tab[f]:
            if all(np.dot(c[1], k[1]) < math.cos(math.radians(10)) for k in kept):
                kept.append(c)
            if len(kept) >= keep:
                break
        tab[f] = kept
    return dict(tab=tab, data=(I_all, J_all, A_all, B_all, dth, m0))


def refine_fixed_f(th, table, f, cx, cy, top=20, stages=(16.0, 360.0)):
    """焦距固定,从这一档的 top 个候选各自一档一档放宽精修 (ω, φ),按全部配点的残差中位数挑"""
    I_all, J_all, A_all, B_all, dth, m0 = table["data"]

    def unpack(x):
        return np.array([math.sin(x[0]) * math.cos(x[1]), math.sin(x[0]) * math.sin(x[1]), math.cos(x[0])]), x[2]

    def res_on(x, m):
        w, phi = unpack(x)
        e1, e2 = perp_basis(w)
        p = math.cos(phi) * e1 + math.sin(phi) * e2
        R = rod(w, th)
        Ii, Jj = I_all[m], J_all[m]
        Rij = np.transpose(R[Jj], (0, 2, 1)) @ R[Ii]
        tij = p[None] - np.einsum('mab,b->ma', Rij, p)
        return samp(Rij, tij, A_all[m], B_all[m], f, cx, cy)
    masks = [m0] + [dth <= st for st in stages]
    everything = np.ones(len(dth), bool)
    out = None
    for sc, w0, phi0 in table["tab"][f][:top]:
        x = np.array([math.acos(np.clip(w0[2], -1, 1)), math.atan2(w0[1], w0[0]), phi0])
        for m in masks:
            if m.sum() >= 50:
                x = least_squares(lambda y: res_on(y, m), x, method="trf", loss="soft_l1", f_scale=1.0, max_nfev=400).x
        med = float(np.median(np.abs(res_on(x, everything))))
        if out is None or med < out[0]:
            out = (med, x, sc)
    w, phi = unpack(out[1])
    e1, e2 = perp_basis(w)
    return w, math.cos(phi) * e1 + math.sin(phi) * e2, out[0], out[2]


def _table_job(a):
    th, pj, cx, cy, fgrid, kw = a
    return joint_table(th, pj, cx, cy, fgrid, **kw) if len(pj) >= 3 else None


def all_joints(th_list, pairs_list, cx, cy, fgrid, log=print, **kw):
    """全部关节:每档焦距下各关节最好的网格分数加起来,取总和最小的那档焦距;焦距固定后每个关节精修"""
    tables = []
    npar = int(os.environ.get("PAR", "0"))
    if npar > 1:
        # 各关节的网格互不相干 ⇒ 分给几个进程(每个进程单线程,免得抢)
        import multiprocessing as mp
        jobs = [(th, pj, cx, cy, fgrid, kw) for th, pj in zip(th_list, pairs_list)]
        with mp.get_context("fork").Pool(min(npar, len(jobs))) as pool:
            tables = pool.map(_table_job, jobs)
        log("    %d 个关节的网格铺完(%d 个进程)" % (len(tables), npar))
    else:
        for j, (th, pj) in enumerate(zip(th_list, pairs_list)):
            tables.append(joint_table(th, pj, cx, cy, fgrid, **kw) if len(pj) >= 3 else None)
            log("    关节 %d 的网格铺完" % j)
    tot = []
    for f in fgrid:
        tot.append(sum(t["tab"][f][0][0] for t in tables if t is not None))
    fi = int(np.argmin(tot))
    f = fgrid[fi]
    log("    各档焦距的网格总分:" + " ".join("%.0f:%.2f" % (ff, s_) for ff, s_ in zip(fgrid, tot)) + " ⇒ 取 %.1f" % f)
    out = []
    for j, t in enumerate(tables):
        if t is None:
            out.append(None); continue
        w, p, med, sc = refine_fixed_f(th_list[j], t, f, cx, cy)
        out.append((w, p, med, sc))
    return f, out
