# -*- coding: utf-8 -*-
# 先找出头顶眼里左手离开原处的帧(手区平均亮度变了),再从那之前 6 帧起跟,每 3 帧打一次
import sys, os, glob, numpy as np, torch
sys.path.insert(0, "/root/instruments/track_on"); os.chdir("/root/instruments/track_on")
from PIL import Image
files = sorted(glob.glob("/root/NG1R/vid/f*_c0.pgm"))
idx = [int(os.path.basename(f)[1:7]) for f in files]
def gray(f): return np.asarray(Image.open(f), dtype=np.float32)
base = gray(files[40])[330:410, 145:200].mean()
changed = []
for f, k in zip(files, idx):
    if k < 40: continue
    m = gray(f)[330:410, 145:200].mean()
    if abs(m - base) > 25:
        changed.append(k)
print("总帧", len(files), "手区变了的帧(前 12 个):", changed[:12])
if not changed:
    sys.exit()
first = changed[0]
pos = idx.index(first)
S = idx[max(0, pos - 6)]; E = idx[min(len(idx) - 1, pos + 40)]
from model.trackon_predictor import Predictor
from utils.train_utils import load_args_from_yaml
args = load_args_from_yaml("config/test_dinov2.yaml")
m = Predictor(args, checkpoint_path="/root/instruments/weights/trackon2_dinov2_checkpoint.pt", support_grid_size=0).to("cuda").eval()
def frame(f):
    a = np.asarray(Image.open(f).convert("RGB"), dtype=np.float32)
    return torch.from_numpy(a).permute(2, 0, 1)[None].to("cuda")
q = torch.tensor([[172.0, 358.0], [170.0, 340.0], [150.0, 400.0]], device="cuda")
names = ["左指根", "左指尖", "左手掌"]
m.reset(); first_done = False
for f, k in zip(files, idx):
    if k < S or k > E: continue
    p, v = m.forward_frame(frame(f), new_queries=(None if first_done else q)); first_done = True
    if (idx.index(k) - idx.index(S)) % 3 == 0:
        print("f%d: " % k + " · ".join("%s(%.0f,%.0f)%s" % (n, p[i, 0], p[i, 1], "" if v[i] else "✗") for i, n in enumerate(names)))
