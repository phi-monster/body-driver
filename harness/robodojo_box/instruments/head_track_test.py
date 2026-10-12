# -*- coding: utf-8 -*-
# 头顶眼里能不能跟住人形的手:从第 S 帧点在左手指尖附近的两个点,一路跟到第 E 帧,每 30 帧打一次
import sys, os, numpy as np, torch
sys.path.insert(0, "/root/instruments/track_on"); os.chdir("/root/instruments/track_on")
from PIL import Image
from model.trackon_predictor import Predictor
from utils.train_utils import load_args_from_yaml
args = load_args_from_yaml("config/test_dinov2.yaml")
m = Predictor(args, checkpoint_path="/root/instruments/weights/trackon2_dinov2_checkpoint.pt", support_grid_size=0).to("cuda").eval()
def frame(k):
    a = np.asarray(Image.open("/root/NG1R/vid/f%06d_c0.pgm" % k).convert("RGB"), dtype=np.float32)
    return torch.from_numpy(a).permute(2, 0, 1)[None].to("cuda")
S, E = int(sys.argv[1]), int(sys.argv[2])
q = torch.tensor([[172.0, 358.0], [170.0, 340.0], [150.0, 400.0], [462.0, 357.0]], device="cuda")   # 左手指根、左指尖、左手掌、右手
names = ["左指根", "左指尖", "左手掌", "右手"]
m.reset()
for k in range(S, E + 1, 2):
    p, v = m.forward_frame(frame(k), new_queries=(q if k == S else None))
    if (k - S) % 30 == 0:
        print("f%d: " % k + " · ".join("%s(%.0f,%.0f)%s" % (n, p[i, 0], p[i, 1], "" if v[i] else "✗") for i, n in enumerate(names)))
