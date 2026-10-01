#!/usr/bin/env python3
# 离线实验:扫描起点那一格撒网格点 → 仪器配进每一格(往返 1 px)→ 一个点跨很多格的轨迹 → 运动学 + 每点远近 按像素重投影一起解 → 和真值比
# 用法:mvba.py <跑的目录> <手> [网格 列 行]
import sys, os, math, json, base64, io, time, urllib.request
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from PIL import Image
exec(open('/root/diag/alignstudy.py').read().split("TRUE_FX")[0])
RUN = sys.argv[1]; ARM = int(sys.argv[2]); GX = int(sys.argv[3]) if len(sys.argv) > 3 else 32; GY = int(sys.argv[4]) if len(sys.argv) > 4 else 24
LOOK = os.path.join(RUN, "look")
def post(path, obj):
    req = urllib.request.Request("http://127.0.0.1:8077" + path, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=600).read())
def put(fn):
    im = Image.open(os.path.join(LOOK, fn)).convert("RGB"); b = io.BytesIO(); im.save(b, format="BMP")
    r = post("/frame", {"image": base64.b64encode(b.getvalue()).decode()})
    return r["id"]
rows = [l for l in open(os.path.join(LOOK, "sweep.txt")) if "||" in l]
L = open(os.path.join(LOOK, "kinem_arm%d.txt" % ARM)).read().split("\n")
f0 = float(L[0].split()[5]); q0 = np.array([float(x) for x in L[1].split()[1:]])
W0, P0 = [], []
for l in L[2:]:
    if l.startswith("axis"):
        v = [float(x) for x in l.split()[2:]]; W0.append(v[:3]); P0.append(v[3:])
W0 = np.array(W0); P0 = np.array(P0); NJ = len(W0)
Q, EE, FN, TAG = [], [], [], []
for l in rows:
    left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
    if int(h[1]) != ARM: continue
    Q.append(np.array([float(x) for x in parts[1 + ARM].split()])); EE.append(np.array([float(x) for x in right.split()])); FN.append(h[0])
    TAG.append("%s%s%s" % (h[2], "+" if int(h[3]) > 0 else ("-" if int(h[3]) < 0 else "0"), h[4]))
NF = len(Q)
cache = os.path.join(RUN, "mv_arm%d_%dx%d.npz" % (ARM, GX, GY))
if os.path.exists(cache):
    Z = np.load(cache); OBS = Z["obs"]; GRID = Z["grid"]
else:
    t0 = time.time()
    ids = [put(fn) for fn in FN]
    us = (np.arange(GX) + 0.5) * 640.0 / GX; vs = (np.arange(GY) + 0.5) * 480.0 / GY
    GRID = np.array([[u, v] for v in vs for u in us])
    obs = []
    for k in range(1, NF):
        r = post("/match", {"a_id": ids[0], "b_id": ids[k], "num": 0, "coarse": True, "back": True, "points": GRID.round(2).tolist()})
        if not r.get("ok"): continue
        P = np.array(r["points"]); B = np.array(r["back"])
        for i in range(len(GRID)):
            if P[i][0] < 0 or B[i][0] < 0: continue
            if np.hypot(B[i][0] - GRID[i][0], B[i][1] - GRID[i][1]) < 1.0 and 0 <= P[i][0] < 640 and 0 <= P[i][1] < 480:
                obs.append((i, k, P[i][0], P[i][1]))
    OBS = np.array(obs); np.savez(cache, obs=OBS, grid=GRID)
    print("配点:%d 格 × %d 点,往返 1 px 内 %d 笔(%.0f 秒)" % (NF - 1, len(GRID), len(OBS), time.time() - t0))
pi = OBS[:, 0].astype(int); pk = OBS[:, 1].astype(int); puv = OBS[:, 2:4]
cnt = np.bincount(pi, minlength=len(GRID))
keep_pt = np.where(cnt >= 2)[0]
remap = -np.ones(len(GRID), int); remap[keep_pt] = np.arange(len(keep_pt))
m = remap[pi] >= 0; pi = remap[pi[m]]; pk = pk[m]; puv = puv[m]
NP = len(keep_pt); G0 = GRID[keep_pt]
print("轨迹:%d 个点(至少进 2 格),%d 笔;每点平均进 %.1f 格" % (NP, len(pi), len(pi) / NP))
def fk_all(W, P, qs):
    Rs, Ts = [], []
    for q in qs:
        R, t = kinem_fk(W, P, q0, q); Rs.append(R); Ts.append(t)
    return np.array(Rs), np.array(Ts)
def dir0(uv, f):
    d = np.stack([(uv[:, 0] - 320.0) / f, -(uv[:, 1] - 240.0) / f, -np.ones(len(uv))], axis=1)
    return d
# 起步远近:按起步模型,把每点在各格的观测三角(沿起点视线的深度,最小二乘)
def tri_depth(W, P, f):
    Rs, Ts = fk_all(W, P, Q)
    d0 = dir0(G0, f)
    lam = np.zeros(NP)
    num = np.zeros(NP); den = np.zeros(NP)
    for n in range(len(pi)):
        i, k = pi[n], pk[n]
        # 在第 k 格相机系里:X_k = R_kᵀ (λ d0 − t_k);要投到 puv ⇒ 线性最小二乘 λ
        a = Rs[k].T @ d0[i]; b = -Rs[k].T @ Ts[k]
        u = (puv[n, 0] - 320.0) / f; v = -(puv[n, 1] - 240.0) / f
        # x/(-z) = u ⇒ x + u z = 0;y + v z = 0
        c1 = np.array([a[0] + u * a[2], a[1] + v * a[2]]); c0 = np.array([b[0] + u * b[2], b[1] + v * b[2]])
        num[i] += -(c1 @ c0); den[i] += c1 @ c1
    lam = num / np.maximum(den, 1e-12)
    return lam
def unpack(x):
    W = x[:3 * NJ].reshape(NJ, 3); P = x[3 * NJ:6 * NJ].reshape(NJ, 3); f = math.exp(x[6 * NJ]); lam = np.exp(x[6 * NJ + 1:])
    return W, P, f, lam
def resid(x):
    W, P, f, lam = unpack(x)
    Rs, Ts = fk_all(W, P, Q)
    d0 = dir0(G0, f)
    X = lam[:, None] * d0                                    # 参照眼系里的点
    Xc = np.einsum('nji,nj->ni', Rs[pk], X[pi] - Ts[pk])      # R_kᵀ (X − t_k)
    z = -Xc[:, 2]
    ru = f * Xc[:, 0] / z + 320.0 - puv[:, 0]; rv_ = -f * Xc[:, 1] / z + 240.0 - puv[:, 1]
    reg = []
    for j in range(NJ):
        reg.append(1e3 * (np.linalg.norm(W[j]) - 1.0)); reg.append(1e3 * (W[j] @ P[j]))
    reg.append(1e3 * (math.sqrt(np.mean(np.sum(Ts ** 2, axis=1))) - 1.0))
    return np.concatenate([ru, rv_, np.array(reg)])
lam0 = tri_depth(W0, P0, f0)
ok = lam0 > 1e-3
print("起步远近:%d / %d 个点在眼前面(别的不要)" % (ok.sum(), NP))
sel = ok[pi]; pi = pi[sel]; pk = pk[sel]; puv = puv[sel]
rm = -np.ones(NP, int); rm[np.where(ok)[0]] = np.arange(ok.sum()); pi = rm[pi]; G0 = G0[ok]; NP = int(ok.sum()); lam0 = lam0[ok]
x0 = np.concatenate([W0.ravel(), P0.ravel(), [math.log(f0)], np.log(lam0)])
r0 = resid(x0); no = len(pi)
print("起步(驱动的模型):重投影 中位 %.3f px、九成 %.3f px" % (np.median(np.hypot(r0[:no], r0[no:2 * no])), np.quantile(np.hypot(r0[:no], r0[no:2 * no]), 0.9)))
# 稀疏结构
nres = 2 * no + 2 * NJ + 1; npar = len(x0)
S = lil_matrix((nres, npar), dtype=np.int8)
for n in range(no):
    S[n, :6 * NJ + 1] = 1; S[no + n, :6 * NJ + 1] = 1
    S[n, 6 * NJ + 1 + pi[n]] = 1; S[no + n, 6 * NJ + 1 + pi[n]] = 1
for r in range(2 * no, nres):
    S[r, :6 * NJ + 1] = 1
t0 = time.time()
sol = least_squares(resid, x0, jac_sparsity=S, loss="soft_l1", f_scale=0.5, max_nfev=int(os.environ.get("MAXNFEV", "3000")), diff_step=1e-6, tr_solver="lsmr", verbose=0)
print("优化器:%s · 调用 %d 次 · 代价 %.1f → %.1f" % (sol.message, sol.nfev, 0.5 * np.sum(r0 ** 2), sol.cost))
r1 = sol.fun; e1 = np.hypot(r1[:no], r1[no:2 * no])
print("一起解完(%.0f 秒):重投影 中位 %.3f px、九成 %.3f px" % (time.time() - t0, np.median(e1), np.quantile(e1, 0.9)))
W1, P1, f1, lam1 = unpack(sol.x)
# 写成 kinem 文件格式,给打分用
out = os.path.join(RUN, "mv%d" % ARM); os.makedirs(out, exist_ok=True)
with open(os.path.join(out, "kinem_arm%d.txt" % ARM), "w") as fo:
    fo.write(L[0].split(" f ")[0] + " f %.6f cx 320.000 cy 240.000\n" % f1)
    fo.write(L[1] + "\n")
    for j in range(NJ):
        w = W1[j] / np.linalg.norm(W1[j])
        fo.write("axis %d %.9f %.9f %.9f %.9f %.9f %.9f\n" % (j, *w, *P1[j]))
# 真值打分(同 alignstudy):按全部格子拟合相似变换 + 相机安装
def grade(W, P):
    Rf, Tf = fk_all(W, P, Q)
    Tp = np.array([e[:3] for e in EE]); Rt = np.array([qR(e[3:]) for e in EE]); idx = np.arange(NF)
    def res2(x):
        Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
        pe = (s * (Tf @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx))
        re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
        return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
    best = None; rng = np.random.default_rng(0)
    for k in range(12):
        x0_ = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
        r = least_squares(res2, x0_, method="lm", max_nfev=4000)
        if best is None or r.cost < best.cost: best = r
    x = best.x; s = x[6]; Rg = rv(x[0:3]); tg = x[7:10]; tx = x[10:13]; Rx = rv(x[3:6])
    pe = np.linalg.norm((s * (Tf @ Rg.T) + tg) - (Tp + np.einsum('nab,b->na', Rt, tx)), axis=1) * 1000
    ang = np.array([np.degrees(np.linalg.norm(logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])))) for i in idx])
    return s, np.median(pe), pe.max(), np.median(ang), ang.max(), Rx
for nm, (W, P, f) in (("驱动的模型", (W0, P0, f0)), ("多视图一起解", (W1, P1, f1))):
    s, pm, px, am, ax, Rx = grade(W, P)
    print("%s:焦距 %.1f · 按真值 位置 中位 %.2f / 最大 %.2f mm · 朝向 中位 %.3f / 最大 %.3f° · 单位 %.3f mm · 眼转 %s" % (nm, f, pm, px, am, ax, 1000 * s, np.round(np.degrees(logR(Rx)), 2)))
