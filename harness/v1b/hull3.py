#!/usr/bin/env python3
# 原型(只做诊断):按 hull2 的轮廓和每帧平移切左手指的视觉外壳(眼系、驱动单位),再按网格真值验
import sys, io, contextlib, json, struct, numpy as np
from scipy import ndimage
RUN = sys.argv[1]; SEG = sys.argv[2]; FO = int(sys.argv[3]); ARM = int(sys.argv[4])
meta = json.load(open(SEG + "_seg.json")); seg = np.load(SEG + "_seg.npz")
f, cx, cy = meta["f"], meta["cx"], meta["cy"]
frames = meta["frames"]; t = {int(k): v[0] for k, v in meta["t"].items()}   # 左手指每帧平移(单位,沿眼系 +x)
# 每帧手指轮廓 = 两根手指的并集(合上时两根连成一块、分不清哪根;左手指的点投到任一根手指上都不切 ⇒ 保守)
M = {}
for k in frames:
    if "L%d" % k not in seg.files: continue
    Lk, Rk = seg["L%d" % k], seg["R%d" % k]
    M[k] = ndimage.binary_dilation(Lk if Lk.any() else (Lk | Rk), iterations=1)
ML = ndimage.binary_dilation(seg["L%d" % FO], iterations=1)   # 张开那一帧只取左手指:从它的轮廓出发切
H, W = M[FO].shape
Dg = np.arange(0.6, 2.8, 0.004)          # 离眼多深(单位)
cells = []
for v in range(H):
    us = np.nonzero(ML[v])[0]
    if len(us) == 0: continue
    x_lo = (us.min() - 0.5 - cx) * Dg.max() / f if us.min() - 0.5 - cx < 0 else (us.min() - 0.5 - cx) * Dg.min() / f
    x_hi = (us.max() + 0.5 - cx) * Dg.min() / f if us.max() + 0.5 - cx < 0 else (us.max() + 0.5 - cx) * Dg.max() / f
    xg = np.arange(x_lo, x_hi, 0.004)
    D2, X2 = np.meshgrid(Dg, xg, indexing="ij")
    u0 = np.round(cx + f * X2 / D2).astype(int)
    keep = (u0 >= 0) & (u0 < W)
    keep[keep] = ML[v][u0[keep]]
    for k in frames:
        if k == FO or k not in M or not keep.any(): continue
        u = np.round(cx + f * (X2 + t[k]) / D2).astype(int)
        ins = keep & (u >= 0) & (u < W)
        ok = np.ones_like(keep)
        ok[ins] = M[k][v][u[ins]]
        keep &= ok
    if keep.any():
        dd = D2[keep]; xx = X2[keep]
        cells.append(np.stack([xx, -(v - cy) * dd / f, -dd], axis=1))
P = np.concatenate(cells) if cells else np.zeros((0, 3))
np.save(SEG + "_hull.npy", P)
print("外壳格子 %d 个" % len(P))
# ── 按网格真值验 ──
sys.argv = ["x", RUN]
ns = {"__name__": "scorer"}
with contextlib.redirect_stdout(io.StringIO()):
    exec(open("/root/score/v1b_score_fk.py").read(), ns)
s_, Rg_, tg_, RxA, txA = ns["FITS"][ARM]
X5 = "/root/RoboDojo/Assets/Robots/x5/meshes"
def stl(p):
    d = open(p, "rb").read(); n = struct.unpack("<I", d[80:84])[0]
    return np.array([struct.unpack("<12f", d[84 + 50 * i:84 + 50 * i + 48])[3:12] for i in range(n)]).reshape(-1, 3)
def stl_tri(p):
    d = open(p, "rb").read(); n = struct.unpack("<I", d[80:84])[0]
    return np.array([struct.unpack("<12f", d[84 + 50 * i:84 + 50 * i + 48])[3:12] for i in range(n)]).reshape(n, 3, 3)
Tr = stl_tri(X5 + "/link7.STL") + np.array([0.08657, 0.024896 + 0.044, -0.0002436])
pts = []
rng = np.random.default_rng(0)
for tri in Tr:
    a = np.linalg.norm(np.cross(tri[1] - tri[0], tri[2] - tri[0])) / 2
    n = max(1, int(a / 0.25e-6))
    r1 = rng.random(n); r2 = rng.random(n); sq = np.sqrt(r1)
    pts.append((1 - sq)[:, None] * tri[0] + (sq * (1 - r2))[:, None] * tri[1] + (sq * r2)[:, None] * tri[2])
V = np.concatenate(pts)
E = ((V - np.ravel(txA)) @ np.asarray(RxA)) / float(np.ravel([s_])[0])   # 眼系,单位
D = -E[:, 2]; uu = cx + f * E[:, 0] / np.maximum(D, 1e-9); vv = cy - f * E[:, 1] / np.maximum(D, 1e-9)
vis = (D > 0.3) & (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
E = E[vis]
print("网格上撒了 %d 点,在画面里的 %d 点" % (len(V), len(E)))
# (尺度按真值行程给,这一步不再自标)
from scipy.spatial import cKDTree
tree = cKDTree(P)
dist, _ = tree.query(E)
print("网格(画面里那一截)的点离外壳:中位 %.2f mm、九成 %.2f mm、最远 %.2f mm;在外壳 0.5 mm 以内的 %.0f%%" % (
    1000 * s_ * np.median(dist), 1000 * s_ * np.percentile(dist, 90), 1000 * s_ * dist.max(), 100.0 * np.mean(dist * s_ <= 0.0005)))
# 按离尖多高分段:沿手指(link6 的 x)方向,离尖 h;合拢方向 = link6 的 y;横着 = link6 的 z(都换到眼系)
ax = RxA.T @ np.array([1.0, 0, 0]); ay = RxA.T @ np.array([0, 1.0, 0]); az = RxA.T @ np.array([0, 0, 1.0])
tipE = E[np.argmax(E @ ax)]
print('尖(网格,眼系,单位)', tipE)
for h in (0.0, 0.025, 0.05, 0.075, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6):   # 单位(× 54 mm)
    ph = P[(np.abs((tipE - P) @ ax - h - 0.0125) <= 0.0125)]
    eh = E[(np.abs((tipE - E) @ ax - h - 0.0125) <= 0.0125)]
    if len(ph) == 0 or len(eh) == 0:
        print("离尖 %.2f 单位:外壳 %d 格 · 网格 %d 点" % (h, len(ph), len(eh))); continue
    print("离尖 %.2f 单位(%.0f mm):合拢方向厚 外壳 %.1f / 网格 %.1f mm · 横着宽 外壳 %.1f / 网格 %.1f mm" % (
        h, h * 1000 * s_, 1000 * s_ * np.ptp(ph @ ay), 1000 * s_ * np.ptp(eh @ ay), 1000 * s_ * np.ptp(ph @ az), 1000 * s_ * np.ptp(eh @ az)))
