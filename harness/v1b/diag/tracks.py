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
print("轨迹缓存好了")
