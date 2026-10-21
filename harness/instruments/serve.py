#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仪器进程:和 vLLM 的眼一样,是驱动旁边的一个小 HTTP 服务。驱动把一帧(BMP24 base64)POST 过来,这里用任务无关的学习型模型量出物理量,
连同不确定度一起还回去。驱动只认数字,不认模型:换模型只改这里。

接口
  GET  /health            → {"instruments": [...], "device": ...}
  POST /segment {"image": ..., "box": [x0,y0,x1,y1], "points": [[u,v,label],...]}
       → {"ok": true, "w": W, "h": H, "area": n, "box": [x0,y0,x1,y1], "score": s, "runs": [r0, r1, ...]}
       框(脑给的)/ 点(label 1 = 在它身上、0 = 不在)⇒ 那件东西的像素:runs = 整幅按行展开的游程,先"不是"一段、再"是"一段……交替。
       score = 模型自报的 IoU(0..1),驱动只报数、不拿它当门

模型:SAM 2.1(Ravi et al. 2024,https://github.com/facebookresearch/sam2 commit 2b90b9f,代码 + 权重 Apache-2.0;hiera-small 46M)。
      2026-09-26 owner 批准装 SAM("装 sam"):驱动自己按明暗切的块常常只是东西的一截、或连着别的东西(SHOT1 剪刀被画面边切着,两只眼的"中心"差 1.7 cm);
      脑给框 ⇒ SAM 出这件东西的整片像素。实测(SHOT1 存图):头顶眼 1725 px、腕眼 6238 px 正好是整把剪刀,一次 0.04 s,显存 0.55 GB。
权重钉死:见 WEIGHTS(sha256);对不上就拒绝起来。
拆掉的(2026-09-24,owner:焦距不准的不留):GeoCalib(单图焦距偏一成、头眼"上"方向反了)、MoGe-2(单目猜深度,尺度核对不了)。
      2026-09-30 拆掉 Track-On2(跟点):驱动里只剩"开机没量出一只眼的朝向时、干活中盯着一块挪几下现量"那一条后备在用它 ——
      一个量两种量法,连同那条后备一起删了(owner:"没用就删了")。
      2026-10-18 拆掉 RoMa(配点 /match、存帧 /frame、整图特征 /describe;owner:"干掉roma"):驱动自己的对准器从眼睛停过的
      各个位置的画面里把点放出来(Driver.World.Sight),丢了的东西也由它在前后两帧里找回来,再没有一处问这个仪器配点。
"""
import base64, io, json, os, threading, hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image

PORT = int(os.environ.get("INST_PORT", "8077"))
WEIGHTS = {
    # 文件名 → sha256(装好后第一次跑校一次;对不上就拒绝起来,不许悄悄换权重)
    "sam2.1_hiera_small.pt": "6d1aa6f30de5c92224f8172114de081d104bbd23dd9dc5c58996f0cad5dc4d38",
    # 2026-10-17 owner:"sam装";Pi 上也要用,一个超小模型。Mac 和 Pi 上用 tiny(39M),dl.fbaipublicfiles.com 092824 原文件
    "sam2.1_hiera_tiny.pt": "7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69",
}
SAM_CKPT = os.environ.get("SAM_CKPT", "/root/instruments/weights/sam2.1_hiera_small.pt")
SAM_CFG = os.environ.get("SAM_CFG", "configs/sam2.1/sam2.1_hiera_s.yaml")   # 仓库在 /root/instruments/sam2_repo(不许放在本文件同级叫 sam2:会遮住包)
# 设备:有 CUDA 用 CUDA(箱子),否则 Mac 的 MPS,否则 CPU(Pi)。半精度只在 CUDA 上开
DEVICE = os.environ.get("INST_DEVICE") or ("cuda" if torch.cuda.is_available() else ("mps" if torch.backends.mps.is_available() else "cpu"))

_lock_seg = threading.Lock()
_sam = None


def _load_sam():
    global _sam
    if _sam is None:
        h = hashlib.sha256(open(SAM_CKPT, "rb").read()).hexdigest()
        name = os.path.basename(SAM_CKPT)
        if h != WEIGHTS.get(name):
            raise RuntimeError("SAM 2.1 权重 %s 哈希对不上:%s" % (name, h))
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        _sam = SAM2ImagePredictor(build_sam2(SAM_CFG, SAM_CKPT, device=DEVICE))
        print("[仪器] %s sha256=%s 设备 %s" % (name, h, DEVICE), flush=True)
    return _sam


def segment(b64, box, points):
    m = _load_sam()
    img = np.asarray(Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB"))
    H, W = img.shape[0], img.shape[1]
    kw = {}
    if box:
        kw["box"] = np.array(box, dtype=np.float32)
    if points:
        kw["point_coords"] = np.array([[p[0], p[1]] for p in points], dtype=np.float32)
        kw["point_labels"] = np.array([int(p[2]) if len(p) > 2 else 1 for p in points], dtype=np.int32)
    if not kw:
        return {"ok": False, "err": "no box / points"}
    with torch.inference_mode(), torch.autocast(DEVICE, dtype=torch.bfloat16, enabled=(DEVICE == "cuda")):
        m.set_image(img)
        masks, scores, _ = m.predict(multimask_output=False, **kw)
    mk = np.asarray(masks[0]).astype(bool).reshape(-1)
    # 游程:先"不是"一段,再"是"一段……交替(第一段可以是 0)
    change = np.flatnonzero(np.diff(mk.astype(np.int8))) + 1
    edges = np.concatenate([[0], change, [mk.size]])
    runs = np.diff(edges).tolist()
    if mk.size > 0 and mk[0]:
        runs = [0] + runs
    area = int(mk.sum())
    if area > 0:
        ys, xs = np.nonzero(mk.reshape(H, W))
        bb = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    else:
        bb = [-1, -1, -1, -1]
    return {"ok": True, "w": W, "h": H, "area": area, "box": bb, "score": _num(float(scores[0]), 4), "runs": runs}


def _num(x, nd):
    """JSON 里的数一律定点小数(驱动的读数器吃不下 1e-05、NaN):非有限数记 -1(像素、可信度都不会是负的)"""
    x = float(x)
    return round(x, nd) if np.isfinite(x) else -1.0


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"instruments": ["segment"], "device": torch.cuda.get_device_name(0) if DEVICE == "cuda" else DEVICE})
        else:
            self._send(404, {"ok": False, "err": "no such instrument"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        try:
            req = json.loads(self.rfile.read(n).decode("utf-8"))
        except Exception as e:
            self._send(400, {"ok": False, "err": "bad json: %s" % e})
            return
        if self.path != "/segment":
            self._send(404, {"ok": False, "err": "no such instrument"})
            return
        try:
            with _lock_seg:
                out = segment(req["image"], req.get("box", []), req.get("points", []))
            self._send(200, out)
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send(500, {"ok": False, "err": str(e)})


if __name__ == "__main__":
    _load_sam()   # 起来就把模型装好,第一帧不慢
    print("[仪器] 听 %d · segment · 设备 %s" % (PORT, DEVICE), flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
