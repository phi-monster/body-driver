#!/usr/bin/env python3
# 第一只手的桌面点改按"同一个点进的所有格"一起三角(驱动的模型不动)⇒ 配进不动的眼 ⇒ 按板解不动的眼 ⇒ 和真值比;对照:按起点↔一格一对三角
import sys, os, math, json, base64, io, urllib.request
import numpy as np
from scipy.optimize import least_squares
from PIL import Image
exec(open('/root/diag/alignstudy.py').read().split("TRUE_FX")[0])
RUN = sys.argv[1]; ARM = 0
LOOK = os.path.join(RUN, "look")
def post(path, obj):
    req = urllib.request.Request("http://127.0.0.1:8077" + path, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=600).read())
def put(fn):
    im = Image.open(os.path.join(LOOK, fn)).convert("RGB"); b = io.BytesIO(); im.save(b, format="BMP")
    return post("/frame", {"image": base64.b64encode(b.getvalue()).decode()})["id"]
rows = [l for l in open(os.path.join(LOOK, "sweep.txt")) if "||" in l]
L = open(os.path.join(LOOK, "kinem_arm0.txt")).read().split("\n")
f0 = float(L[0].split()[5]); q0 = np.array([float(x) for x in L[1].split()[1:]])
W0, P0 = [], []
for l in L[2:]:
    if l.startswith("axis"):
        v = [float(x) for x in l.split()[2:]]; W0.append(v[:3]); P0.append(v[3:])
W0 = np.array(W0); P0 = np.array(P0)
Q, EE, FN = [], [], []
for l in rows:
    left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
    if int(h[1]) != 0: continue
    Q.append(np.array([float(x) for x in parts[1].split()])); EE.append(np.array([float(x) for x in right.split()])); FN.append(h[0])
Rs, Ts = [], []
for q in Q:
    R, t = kinem_fk(W0, P0, q0, q); Rs.append(R); Ts.append(t)
Rs = np.array(Rs); Ts = np.array(Ts)
Z = np.load(os.path.join(RUN, "mv_arm0_32x24.npz")); OBS = Z["obs"]; GRID = Z["grid"]
pi = OBS[:, 0].astype(int); pk = OBS[:, 1].astype(int); puv = OBS[:, 2:4]
D = np.stack([(GRID[:, 0] - 320.0) / f0, -(GRID[:, 1] - 240.0) / f0, -np.ones(len(GRID))], axis=1)
# 每点:多视图(所有格)沿起点视线解远近;对照:只用"离起点最远(基线最大)的那一格"、只用"随便一格"(第一个配上的)
def depth_from(i, ks, uv):
    def r(lz):
        X = math.exp(lz[0]) * D[i]
        Xc = np.array([Rs[k].T @ (X - Ts[k]) for k in ks]); z = -Xc[:, 2]
        return np.concatenate([f0 * Xc[:, 0] / z + 320.0 - uv[:, 0], -f0 * Xc[:, 1] / z + 240.0 - uv[:, 1]])
    best = None
    for lz in np.log([2.0, 4.0, 8.0, 16.0]):
        s = least_squares(r, [lz], loss="soft_l1", f_scale=0.5)
        if best is None or s.cost < best.cost: best = s
    e = np.hypot(best.fun[:len(ks)], best.fun[len(ks):])
    return math.exp(best.x[0]), np.median(e)
mv, one = {}, {}
for i in range(len(GRID)):
    m = pi == i
    if m.sum() < 3: continue
    ks = pk[m]; uv = puv[m]
    lam, med = depth_from(i, ks, uv)
    if med < 1.0: mv[i] = lam
    # 对照:起点 ↔ 一格(挑基线最长那格,同驱动"一对三角"但挑得最好)
    b = np.array([np.linalg.norm(Ts[k]) for k in ks]); j = int(np.argmax(b))
    lam1, _ = depth_from(i, ks[j:j + 1], uv[j:j + 1]); one[i] = lam1
print("多视图三角:%d 个点(每点平均 %.1f 格)" % (len(mv), np.mean([np.sum(pi == i) for i in mv])))
# 这些网格点配进不动的眼
cam0 = put(FN[0]); wc = put("world_cam.bmp")
r = post("/match", {"a_id": cam0, "b_id": wc, "num": 0, "coarse": True, "back": True, "points": GRID.round(2).tolist()})
P = np.array(r["points"]); B = np.array(r["back"])
fx = {}
for i in range(len(GRID)):
    if P[i][0] >= 0 and B[i][0] >= 0 and np.hypot(B[i][0] - GRID[i][0], B[i][1] - GRID[i][1]) < 1.0:
        fx[i] = P[i][:2]
print("配进不动的眼(往返 1 px):%d 个网格点" % len(fx))
# 真值换算:第一只手的相似变换(全部格子)
Tp = np.array([e[:3] for e in EE]); Rt = np.array([qR(e[3:]) for e in EE]); idx = np.arange(len(Q))
def res2(x):
    Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
    pe = (s * (Ts @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx))
    re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rs[i])) for i in idx])
    return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
best = None; rng = np.random.default_rng(0)
for k in range(12):
    x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
    rr = least_squares(res2, x0, method="lm", max_nfev=4000)
    if best is None or rr.cost < best.cost: best = rr
s0 = best.x[6]; Rg0 = rv(best.x[0:3]); tg0 = best.x[7:10]
c30, s30 = math.cos(math.radians(30)), math.sin(math.radians(30))
Rtrue = np.array([[1, 0, 0], [0, c30, -s30], [0, s30, c30]]); ptrue = np.array([0.0, -0.41, 1.308]); ftrue = 10.0 / 22.212 * 640
def proj(R, p, f, X):
    Xc = (X - p) @ R; z = -Xc[:, 2]
    return np.stack([f * Xc[:, 0] / z + 320.0, -f * Xc[:, 1] / z + 240.0], axis=1)
def ray(R, p, f, uv):
    d = np.stack([(uv[:, 0] - 320.0) / f, -(uv[:, 1] - 240.0) / f, -np.ones(len(uv))], axis=1) @ R.T
    return d / np.linalg.norm(d, axis=1)[:, None]
def pnp(X, uv):
    def res(x):
        R = rv(x[:3]) @ Rtrue; p = ptrue + x[3:6]; f = ftrue * math.exp(x[6])
        return (proj(R, p, f, X) - uv).ravel()
    r = least_squares(res, np.zeros(7), loss="soft_l1", f_scale=1.0)
    x = r.x; return rv(x[:3]) @ Rtrue, ptrue + x[3:6], ftrue * math.exp(x[6])
for nm, dep in (("多视图(所有格一起)", mv), ("一对(起点 ↔ 基线最长那格)", one)):
    ii = np.array(sorted(set(dep) & set(fx)))
    Xm = np.array([dep[i] * D[i] for i in ii]); Xs = s0 * (Xm @ Rg0.T) + tg0; uv = np.array([fx[i] for i in ii])
    z = Xs[:, 2]; zt = np.median(z); on = np.abs(z - zt) < 5 * 1.4826 * np.median(np.abs(z - zt))
    A = np.c_[Xs[on, 0], Xs[on, 1], np.ones(on.sum())]; cz = np.linalg.lstsq(A, z[on], rcond=None)[0]
    d = ray(Rtrue, ptrue, ftrue, uv); lam = (zt - ptrue[2]) / d[:, 2]; Xt = ptrue + lam[:, None] * d
    e = (Xs - Xt) * 1000
    Rp, pp, fp = pnp(Xs[on], uv[on])
    print("%s:%d 点(桌面上 %d)· 高的离散 %.2f mm、倾 %.3f° / %.3f° · 点 − 真 平均 (%.2f, %.2f, %.2f) mm、中位 |e| %.2f · 按它们解不动的眼:焦距 %.1f、位置差 (%.1f, %.1f, %.1f) mm、转差 %.3f°" %
          (nm, len(ii), on.sum(), 1000 * 1.4826 * np.median(np.abs(z[on] - zt)), math.degrees(math.atan(cz[0])), math.degrees(math.atan(cz[1])), *e[on].mean(axis=0), np.median(np.linalg.norm(e[on], axis=1)),
           fp, *(1000 * (pp - ptrue)), math.degrees(np.linalg.norm(logR(Rp.T @ Rtrue)))))
