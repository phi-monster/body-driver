# -*- coding: utf-8 -*-
# 离线验 Track-On2:在 V1C 右腕眼(c2)的存图上从第 350 帧跟到第 750 帧,四个查询点:棒球、乐高、地砖角、远墙沿
import sys, time, os
sys.path.insert(0, "/root/instruments/track_on")
os.chdir("/root/instruments/track_on")
import numpy as np, torch
from PIL import Image
from model.trackon_predictor import Predictor
from utils.train_utils import load_args_from_yaml

args = load_args_from_yaml("config/test_dinov2.yaml")
model = Predictor(args, checkpoint_path="/root/instruments/weights/trackon2_dinov2_checkpoint.pt", support_grid_size=0).to("cuda").eval()
print("显存(装完模型) %.2f GB" % (torch.cuda.memory_allocated() / 1e9))

def frame(k, c=2):
    im = Image.open("/root/NV1C/vid/f%06d_c%d.pgm" % (k, c)).convert("RGB")
    a = np.asarray(im, dtype=np.float32)               # H W 3, 0..255
    return torch.from_numpy(a).permute(2, 0, 1)[None].to("cuda")

names = ["棒球", "乐高", "地砖角", "远墙沿"]
q = torch.tensor([[292.0, 240.0], [150.0, 205.0], [355.0, 200.0], [300.0, 120.0]], device="cuda")
model.reset()
ts = []
for k in range(350, 751, 5):
    t = time.time()
    p, v = model.forward_frame(frame(k), new_queries=(q if k == 350 else None))
    torch.cuda.synchronize(); ts.append(time.time() - t)
    if (k - 350) % 50 == 0:
        print("f%d: " % k + " · ".join("%s(%.0f,%.0f)%s" % (n, p[i, 0], p[i, 1], "" if v[i] else "✗") for i, n in enumerate(names)))
print("每帧 %.1f ms(首帧 %.0f ms)· 峰值显存 %.2f GB" % (1000 * np.mean(ts[1:]), 1000 * ts[0], torch.cuda.max_memory_allocated() / 1e9))
