#!/usr/bin/env python3
# 离线诊断(不喂给驱动):x5 两根手指的网格按张开 / 合上投进一只腕眼,得到真轮廓和每个像素的真深度(眼系,米);
# 眼系 ↔ link6 用打分脚本拟合的那一套(FITS)。输出:真掩膜(npz)和一张叠在 lo / hi 两帧上的图
import sys, io, contextlib, json, struct, re, numpy as np
RUN = sys.argv[1]; ARM = int(sys.argv[2]); CAM = int(sys.argv[3]); OUT = sys.argv[4]
sys.argv = ["x", RUN]
with contextlib.redirect_stdout(io.StringIO()):
    exec(open("/root/score/v1b_score_fk.py").read())
s_, Rg_, tg_, RxA, txA = FITS[ARM]
logtxt = open(RUN + "/cal.log", encoding="utf-8", errors="ignore").read()
body = re.search(r"身体写进 (/\S+?\.json)", logtxt).group(1)
geo = json.load(open(body + ".geo.json"))
g = [c for c in geo["cams"] if c["cam"] == CAM][0]
f, cx, cy = g["f"], g["cx"], g["cy"]
X5 = "/root/RoboDojo/Assets/Robots/x5/meshes"
def stl_tri(p):
    d = open(p, "rb").read(); n = struct.unpack("<I", d[80:84])[0]
    return np.array([struct.unpack("<12f", d[84 + 50 * i:84 + 50 * i + 48])[3:12] for i in range(n)]).reshape(n, 3, 3)
T7 = stl_tri(X5 + "/link7.STL"); T8 = stl_tri(X5 + "/link8.STL")
W, H = 640, 480
def render(open_m):
    masks = []; depth = []
    for T, org, ax in ((T7, np.array([0.08657, 0.024896, -0.0002436]), 1.0), (T8, np.array([0.08657, -0.0249, -0.00024366]), -1.0)):
        P = T + org + np.array([0.0, ax * open_m, 0.0])             # link6 系,米
        E = ((P.reshape(-1, 3) - txA) @ RxA) / s_                   # 眼系,单位(RxA^T (p − t) / s)
        Z = -E[:, 2]
        u = cx + f * E[:, 0] / Z; v = cy - f * E[:, 1] / Z
        u = u.reshape(-1, 3); v = v.reshape(-1, 3); Z = (Z * s_).reshape(-1, 3)   # 深度按米
        M = np.zeros((H, W), bool); D = np.full((H, W), np.inf)
        for k in range(u.shape[0]):
            x0, x1 = int(max(0, np.floor(u[k].min()))), int(min(W - 1, np.ceil(u[k].max())))
            y0, y1 = int(max(0, np.floor(v[k].min()))), int(min(H - 1, np.ceil(v[k].max())))
            if x1 < x0 or y1 < y0 or (Z[k] <= 0).any(): continue
            ys, xs = np.mgrid[y0:y1 + 1, x0:x1 + 1]
            (ax_, ay), (bx, by), (qx, qy) = zip(u[k], v[k])
            den = (by - qy) * (ax_ - qx) + (qx - bx) * (ay - qy)
            if abs(den) < 1e-9: continue
            l1 = ((by - qy) * (xs - qx) + (qx - bx) * (ys - qy)) / den
            l2 = ((qy - ay) * (xs - qx) + (ax_ - qx) * (ys - qy)) / den
            l3 = 1 - l1 - l2
            ins = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
            if not ins.any(): continue
            z = l1 * Z[k][0] + l2 * Z[k][1] + l3 * Z[k][2]
            yy, xx = ys[ins], xs[ins]
            M[yy, xx] = True
            D[yy, xx] = np.minimum(D[yy, xx], z[ins])
        masks.append(M); depth.append(D)
    return masks, depth
mo, do = render(0.044)
mc, dc = render(0.0)
np.savez_compressed(OUT + ".npz", open0=mo[0], open1=mo[1], closed0=mc[0], closed1=mc[1], dopen0=do[0], dopen1=do[1], dclosed0=dc[0], dclosed1=dc[1])
print("真掩膜像素:张开 %d / %d · 合上 %d / %d · 焦距 %.1f" % (mo[0].sum(), mo[1].sum(), mc[0].sum(), mc[1].sum(), f))
for r in (260, 280, 300, 340, 380, 420, 460):
    def seg(M):
        xs = np.nonzero(M[r])[0]
        return "-" if len(xs) == 0 else "%d..%d" % (xs.min(), xs.max())
    print("行 %d · 张开 指0 %s 指1 %s · 合上 指0 %s 指1 %s" % (r, seg(mo[0]), seg(mo[1]), seg(mc[0]), seg(mc[1])))
