#!/usr/bin/env python3
# 原型(只做诊断,不喂给驱动):推抓握那一串腕眼帧 → 每帧手指轮廓 → 每一行一个极平面、按每帧手指平移了多少把轮廓对回同一处 → 切出手指截面(视觉外壳)
# 背景:张开 / 合上两帧逐像素比,变了的连成块;贴张开时指尖的块 = 张开时的手指(背景取合上那帧),贴合上时指尖的块 = 合上时的手指(背景取张开那帧)
import sys, json, re, numpy as np
from scipy import ndimage
RUN = sys.argv[1]; CAM = int(sys.argv[2]); FO = int(sys.argv[3]); FC = int(sys.argv[4]); F0 = int(sys.argv[5]); F1 = int(sys.argv[6]); OUT = sys.argv[7]
def rd(k):
    d = open("%s/vid/f%06d_c%d.pgm" % (RUN, k, CAM), "rb").read(); parts = d.split(b"\n", 3)
    w, h = map(int, parts[1].split()); return np.frombuffer(parts[3], dtype=np.uint8)[:w*h].reshape(h, w).astype(np.int16)
logtxt = open(RUN + "/cal.log", encoding="utf-8", errors="ignore").read()
body = re.search(r"身体写进 (/\S+?\.json)", logtxt).group(1)
geo = json.load(open(body + ".geo.json")); g = [c for c in geo["cams"] if c["cam"] == CAM][0]
f, cx, cy = g["f"], g["cx"], g["cy"]
tip = np.array(g["tip"]); Ztip = -tip[2]            # 指尖(两瓣中点)沿光轴多深,单位(驱动的长度单位)
O = rd(FO); C = rd(FC); H, W = O.shape
T = 20                                               # 变了 = 灰度差超过它(诊断用;驱动里按 Picture.Split 分两拨)
ch = ndimage.binary_opening(np.abs(O - C) > T, iterations=1)
lab, n = ndimage.label(ch, structure=np.ones((3, 3)))
sizes = ndimage.sum(ch, lab, range(1, n + 1)); big = [i + 1 for i in range(n) if sizes[i] * 10 >= sizes.max()]
# 张开时的手指在画面两边、合上时在中间:按块心离画面中线多远分(中间那三分之一 = 合上时的,两根合上的手指常连成一块)
cent = np.array([ndimage.center_of_mass(lab == i) for i in big])
closed_ids = [big[j] for j in range(len(big)) if abs(cent[j][1] - W / 2.0) < W / 6.0]
open_ids = [big[j] for j in range(len(big)) if abs(cent[j][1] - W / 2.0) >= W / 6.0]
print("变了的块 %d 个(大的 %d 个):合上时的手指 %s · 张开时的手指 %s" % (n, len(big), [int(sizes[i - 1]) for i in closed_ids], [int(sizes[i - 1]) for i in open_ids]))
bg = O.copy()
for i in open_ids: bg[lab == i] = C[lab == i]
for i in closed_ids: bg[lab == i] = O[lab == i]
np.save(OUT + "_bg.npy", bg)
# 每一帧的轮廓(两根手指分左右:按块心在画面中线哪边)
frames = list(range(F0, F1 + 1)); masks = {}; tips = {}
for k in frames:
    F = rd(k)
    m = ndimage.binary_opening(np.abs(F - bg) > T, iterations=1)
    l2, n2 = ndimage.label(m, structure=np.ones((3, 3)))
    if n2 == 0: continue
    s2 = ndimage.sum(m, l2, range(1, n2 + 1)); keep = [i + 1 for i in range(n2) if s2[i] * 20 >= s2.max()]
    L = np.zeros_like(m); R = np.zeros_like(m)
    for i in keep:
        cm = ndimage.center_of_mass(l2 == i)
        (L if cm[1] < W / 2.0 else R)[l2 == i] = True
    masks[k] = (L, R)
    tk = []
    for M in (L, R):
        ys, xs = np.nonzero(M)
        if len(ys) == 0: tk.append(None); continue
        top = ys.min(); tk.append((float(xs[ys <= top + 1].mean()), int(top)))
    tips[k] = tk
# 每帧每根手指平移了多少(驱动的单位):右指尖横移的像素 × 指尖深 ÷ 焦距(张开那一帧为 0);两根手指对称走 ⇒ 左 = −右。
# 合上静止那几帧两个尖挨着、最上面那点来回跳 ⇒ 取分得开的那几帧的中位(驱动里按抓握读数 × 行程,不用跟尖)
t = {}
base = tips[FO][1][0]
raw = {k: (tips[k][1][0] - base) * Ztip / f for k in frames if k in tips and tips[k][1] is not None}
valid = {k: raw[k] for k in raw if tips[k][1][0] > W / 2.0}          # 右指尖没跑到中线左边(没和左指尖粘上)
plateau = [valid[k] for k in valid if FC - 3 <= k <= FC + 4]          # 合上静止那几帧
full = float(np.median(plateau))
TRUE_STROKE = float(sys.argv[8]) if len(sys.argv) > 8 else None   # 只做诊断:按真值行程(单位)把每帧平移等比放缩
if TRUE_STROKE: scale_t = TRUE_STROKE / abs(full)
else: scale_t = 1.0
for k in raw:
    v = raw[k] if k in valid else full
    v = max(v, full)
    t[k] = [-v * scale_t, v * scale_t]
print("右手指全程平移 %.4f 单位" % full)
for k in frames:
    if k in t:
        print("帧 %d · 左指尖 %s 平移 %s · 右指尖 %s 平移 %s" % (k, tips[k][0], "-" if t[k][0] is None else "%.4f" % t[k][0], tips[k][1], "-" if t[k][1] is None else "%.4f" % t[k][1]))
np.savez_compressed(OUT + "_seg.npz", **{"L%d" % k: masks[k][0] for k in masks}, **{"R%d" % k: masks[k][1] for k in masks})
json.dump({"t": {str(k): t[k] for k in t}, "f": f, "cx": cx, "cy": cy, "Ztip": Ztip, "tip": list(tip), "frames": [k for k in frames if k in t]}, open(OUT + "_seg.json", "w"))
