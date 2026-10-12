# -*- coding: utf-8 -*-
# SAM 2.1 离线核:SHOT1 脑给剪刀的两个框(头顶眼第 1 轮、左腕眼第 2 轮)⇒ 掩膜多大、框多大、模型自报的 IoU、要多久;叠图落盘看
import time, sys, numpy as np, torch
from PIL import Image
sys.path.insert(0, "/root/instruments/sam2")
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
torch.set_float32_matmul_precision("highest")
t0 = time.time()
model = build_sam2("configs/sam2.1/sam2.1_hiera_s.yaml", "/root/instruments/weights/sam2.1_hiera_small.pt", device="cuda")
pred = SAM2ImagePredictor(model)
print("load %.1f s, GPU %.2f GB" % (time.time() - t0, torch.cuda.max_memory_allocated() / 1e9), flush=True)
cases = [("/root/NSHOT1/look/grid_000001.bmp", [297, 278, 344, 412], "head"), ("/root/NSHOT1/look/grid_000002.bmp", [312, 264, 492, 479], "wrist")]
for path, box, tag in cases:
    img = np.asarray(Image.open(path).convert("RGB"))
    for rep in range(3):
        t = time.time()
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            pred.set_image(img)
            masks, scores, _ = pred.predict(box=np.array(box, dtype=np.float32), multimask_output=False)
        torch.cuda.synchronize()
        dt = time.time() - t
    m = masks[0].astype(bool)
    ys, xs = np.nonzero(m)
    print("%s: 框 %s ⇒ 掩膜 %d px, 包围框 [%d %d %d %d], 自报 IoU %.3f, 一次 %.3f s" % (tag, box, m.sum(), xs.min(), ys.min(), xs.max(), ys.max(), float(scores[0]), dt), flush=True)
    ov = img.copy()
    ov[m] = (0.5 * ov[m] + 0.5 * np.array([255, 0, 255])).astype(np.uint8)
    x0, y0, x1, y1 = box
    ov[y0, x0:x1] = [255, 255, 0]; ov[y1 - 1, x0:x1] = [255, 255, 0]; ov[y0:y1, x0] = [255, 255, 0]; ov[y0:y1, x1 - 1] = [255, 255, 0]
    Image.fromarray(ov).save("/root/instruments/sam_test_%s.png" % tag)
print("peak GPU %.2f GB" % (torch.cuda.max_memory_allocated() / 1e9))
