#!/usr/bin/env python3
# 旧炮的配点换成"问格点"(同驱动 09-27 版的扫描):同一批格子对,第 I 帧上 32 × 24 格点配进第 J 帧(粗配、往返 1 px),
# 轨迹号:I = 0 的对 = 格点号;别的对 = 768 × (1 + 第几对) + 格点号。写成 corrs_arm<k>.txt(7 列)
import sys, os, json, base64, io, urllib.request
import numpy as np
from PIL import Image
RUN, ARM, OUT = sys.argv[1], int(sys.argv[2]), sys.argv[3]
LOOK = os.path.join(RUN, "look")
GX, GY = 32, 24; NG = GX * GY
def post(path, obj):
    req = urllib.request.Request("http://127.0.0.1:8077" + path, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=600).read())
rows = [l for l in open(os.path.join(LOOK, "sweep.txt")) if "||" in l]
fn = [l.split("|")[0].split()[0] for l in rows if int(l.split("|")[0].split()[1]) == ARM]
pairs = []
for l in open(os.path.join(LOOK, "corrs_arm%d.txt" % ARM)):
    f = l.split()
    p = (int(f[0]), int(f[1]))
    if not pairs or pairs[-1] != p:
        if p not in pairs: pairs.append(p)
ids = {}
def pid(k):
    if k not in ids:
        im = Image.open(os.path.join(LOOK, fn[k])).convert("RGB"); b = io.BytesIO(); im.save(b, format="BMP")
        ids[k] = post("/frame", {"image": base64.b64encode(b.getvalue()).decode()})["id"]
    return ids[k]
W, H = Image.open(os.path.join(LOOK, fn[0])).size
grid = [[(gx + 0.5) * W / GX, (gy + 0.5) * H / GY] for gy in range(GY) for gx in range(GX)]
os.makedirs(OUT, exist_ok=True)
n_all = 0
with open(os.path.join(OUT, "corrs_arm%d.txt" % ARM), "w") as fo:
    for serial, (i, j) in enumerate(pairs):
        r = post("/match", {"a_id": pid(i), "b_id": pid(j), "num": 0, "coarse": True, "back": True, "points": [[round(u, 2), round(v, 2)] for u, v in grid]})
        if not r.get("ok"): continue
        P = r["points"]; B = r["back"]
        for g, (u, v) in enumerate(grid):
            if B[g][0] >= 0 and 0 <= P[g][0] < W and 0 <= P[g][1] < H and ((B[g][0] - u) ** 2 + (B[g][1] - v) ** 2) ** 0.5 < 1.0:
                pt = g if i == 0 else NG * (1 + serial) + g
                fo.write("%d %d %.3f %.3f %.3f %.3f %d\n" % (i, j, u, v, P[g][0], P[g][1], pt)); n_all += 1
print("手 %d:%d 对、配点 %d 条" % (ARM, len(pairs), n_all))
