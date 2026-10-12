#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仪器进程:和 vLLM 的眼一样,是驱动旁边的一个小 HTTP 服务。驱动把一帧(BMP24 base64)POST 过来,这里用任务无关的学习型模型量出物理量,
连同不确定度一起还回去。驱动只认数字,不认模型:换模型只改这里。

接口
  GET  /health            → {"instruments": [...], "device": ...}
  POST /frame {"image": ...}                                 → {"ok": true, "id": n}
       一帧先存在这边(解码一次),之后配点只报编号 —— 开机扫描同一帧要和几十帧配,每次都把 1 MB 的图转成文字再传一遍,
       比配点本身还慢(2026-09-26 实测:一对 1.9 秒,里面配点 0.8 秒)。只留最近 1200 帧
  POST /match {"a": ..., "b": ..., "num": N, "points": [[u,v],...]}   (或者 "a_id" / "b_id" 代替 "a" / "b":用 /frame 存过的帧)
       → {"ok": true, "samples": [[ua,va,ub,vb,cert],...], "points": [[ub,vb,cert],...]}
       "back": true ⇒ 另给 "back": [[ua2,va2],...]:每个查询点配到 B 以后再配回 A 落在哪(同一次配点的反向 warp,不另配;往返差 = 配点自己对不对得上)
       两台相机(或同一台相机两个位置)的两帧里,哪两个像素是同一个真实的点:num > 0 抽 num 对对应点;points = A 里的像素,问它们在 B 里在哪。
       cert = 模型自己给的可信度(0..1),驱动不拿它当真,只拿几何去核(三角重投、两停交叉)
  POST /describe {"ids": [n,...]}  → {"ok": true, "vectors": [[...1024 个数...],...]}
       每张存过的帧(/frame 的编号)一个整体特征(DINOv2 图块特征的平均,归一):两张图看起来多像 = 两个向量的点积。驱动只拿它挑先配哪几对
  POST /segment {"image": ..., "box": [x0,y0,x1,y1], "points": [[u,v,label],...]}
       → {"ok": true, "w": W, "h": H, "area": n, "box": [x0,y0,x1,y1], "score": s, "runs": [r0, r1, ...]}
       框(脑给的)/ 点(label 1 = 在它身上、0 = 不在)⇒ 那件东西的像素:runs = 整幅按行展开的游程,先"不是"一段、再"是"一段……交替。
       score = 模型自报的 IoU(0..1),驱动只报数、不拿它当门

模型:RoMa(Edstedt et al., CVPR 2024,https://github.com/Parskatt/RoMa commit 77f8d68,代码 MIT;权重 roma_outdoor.pth + DINOv2 ViT-L/14)。
      2026-09-25 owner 批准装 RoMa:固定的头顶眼只靠看手定不准焦距(分割出来的指尖不是手上一个固定的点),拿腕眼三角出来的桌面点当标定板,要在头顶眼里认出同一批点。
      SAM 2.1(Ravi et al. 2024,https://github.com/facebookresearch/sam2 commit 2b90b9f,代码 + 权重 Apache-2.0;hiera-small 46M)。
      2026-09-26 owner 批准装 SAM("装 sam"):驱动自己按明暗切的块常常只是东西的一截、或连着别的东西(SHOT1 剪刀被画面边切着,两只眼的"中心"差 1.7 cm);
      脑给框 ⇒ SAM 出这件东西的整片像素。实测(SHOT1 存图):头顶眼 1725 px、腕眼 6238 px 正好是整把剪刀,一次 0.04 s,显存 0.55 GB。
权重钉死:见 WEIGHTS(sha256);对不上就拒绝起来。
拆掉的(2026-09-24,owner:焦距不准的不留):GeoCalib(单图焦距偏一成、头眼"上"方向反了)、MoGe-2(单目猜深度,尺度核对不了)。
      2026-09-30 拆掉 Track-On2(跟点):驱动里只剩"开机没量出一只眼的朝向时、干活中盯着一块挪几下现量"那一条后备在用它 ——
      一个量两种量法,连同那条后备一起删了(owner:"没用就删了")。
"""
import base64, io, json, os, sys, time, threading, hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image

PORT = int(os.environ.get("INST_PORT", "8077"))
WEIGHTS = {
    # 文件名 → sha256(装好后第一次跑校一次;对不上就拒绝起来,不许悄悄换权重)
    "roma_outdoor.pth": "c7a45c80d41ad788a63c641d1b686d7cb3f297f40097c6f4e75039889e5cc8ba",
    "dinov2_vitl14_pretrain.pth": "d5383ea8f4877b2472eb973e0fd72d557c7da5d3611bd527ceeb1d7162cbf428",
    "sam2.1_hiera_small.pt": "6d1aa6f30de5c92224f8172114de081d104bbd23dd9dc5c58996f0cad5dc4d38",
}
ROMA_DIR = os.environ.get("ROMA_DIR", "/root/instruments/RoMa")
ROMA_W = os.environ.get("ROMA_W", os.path.expanduser("~/.cache/torch/hub/checkpoints/roma_outdoor.pth"))
ROMA_DINO = os.environ.get("ROMA_DINO", os.path.expanduser("~/.cache/torch/hub/checkpoints/dinov2_vitl14_pretrain.pth"))
SAM_CKPT = os.environ.get("SAM_CKPT", "/root/instruments/weights/sam2.1_hiera_small.pt")
SAM_CFG = os.environ.get("SAM_CFG", "configs/sam2.1/sam2.1_hiera_s.yaml")   # 仓库在 /root/instruments/sam2_repo(不许放在本文件同级叫 sam2:会遮住包)

# 每个模型一把锁(2026-09-26):配点和抠图互不排队
_lock_match = threading.Lock()
_lock_seg = threading.Lock()
_roma = None
_sam = None


def _load_roma():
    """RoMa:两份权重先核 sha256 再交给它(不让它自己去网上下);推理走纯 PyTorch 的局部相关(没装 fused local_corr 扩展)"""
    global _roma
    if _roma is None:
        for fn, path in (("roma_outdoor.pth", ROMA_W), ("dinov2_vitl14_pretrain.pth", ROMA_DINO)):
            h = hashlib.sha256(open(path, "rb").read()).hexdigest()
            if h != WEIGHTS[fn]:
                raise RuntimeError("RoMa 权重 %s 哈希对不上:%s" % (fn, h))
        sys.path.insert(0, ROMA_DIR)
        torch.set_float32_matmul_precision("highest")   # RoMa 自己要求
        from romatch import roma_outdoor
        w = torch.load(ROMA_W, map_location="cuda"); d = torch.load(ROMA_DINO, map_location="cuda")
        _roma = roma_outdoor(device="cuda", weights=w, dinov2_weights=d, use_custom_corr=False)
        print("[仪器] roma_outdoor 权重已核", flush=True)
    return _roma


def _load_sam():
    global _sam
    if _sam is None:
        h = hashlib.sha256(open(SAM_CKPT, "rb").read()).hexdigest()
        if h != WEIGHTS["sam2.1_hiera_small.pt"]:
            raise RuntimeError("SAM 2.1 权重哈希对不上:%s" % h)
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        _sam = SAM2ImagePredictor(build_sam2(SAM_CFG, SAM_CKPT, device="cuda"))
        print("[仪器] sam2.1_hiera_small sha256=%s" % h, flush=True)
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
    with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
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


_frames = {}
_frames_lock = threading.Lock()
_frame_next = [0]
FRAMES_KEEP = 1200   # 存帧上限(次数)


def frame_put(b64):
    img = Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")
    with _frames_lock:
        n = _frame_next[0]
        _frame_next[0] += 1
        _frames[n] = img
        for k in [k for k in _frames if k <= n - FRAMES_KEEP]:
            del _frames[k]
    return {"ok": True, "id": n}


def frame_get(req, key):
    if key + "_id" in req:
        with _frames_lock:
            img = _frames.get(int(req[key + "_id"]))
        if img is None:
            raise KeyError("没有存过的帧 %s" % req[key + "_id"])
        return img
    return Image.open(io.BytesIO(base64.b64decode(req[key]))).convert("RGB")


def match(A, B, num, points, coarse=False, back=False):
    m = _load_roma()
    t0 = time.time()
    Wa, Ha = A.size; Wb, Hb = B.size
    with torch.no_grad():
        up = m.upsample_preds
        m.upsample_preds = not coarse   # coarse = 只在粗分辨率上配(不做最后那一层细化;量快多少、准多少用)
        try:
            warp, cert = m.match(A, B, device="cuda")
        finally:
            m.upsample_preds = up
        out = {"ok": True, "samples": [], "points": [], "model": "roma-outdoor"}
        if num and num > 0:
            mt, ct = m.sample(warp, cert, num=int(num))
            ka, kb = m.to_pixel_coordinates(mt, Ha, Wa, Hb, Wb)
            ka = ka.cpu().numpy(); kb = kb.cpu().numpy(); ct = ct.cpu().numpy()
            out["samples"] = [[_num(ka[i, 0], 3), _num(ka[i, 1], 3), _num(kb[i, 0], 3), _num(kb[i, 1], 3), _num(ct[i], 4)] for i in range(len(ct))]
        if points:
            Ww = warp.shape[2] // 2   # 对称 warp:左半是 A → B
            wAB = warp[0, :, :Ww, 2:].permute(2, 0, 1)[None].float(); cA = cert[0, :, :Ww][None, None].float()
            uv = np.asarray(points, dtype=np.float32).reshape(-1, 2)
            g = torch.tensor(np.stack([2 * uv[:, 0] / Wa - 1, 2 * uv[:, 1] / Ha - 1], 1)[None, :, None, :], device="cuda", dtype=torch.float32)
            xb = torch.nn.functional.grid_sample(wAB, g, align_corners=False)[0, :, :, 0].T.cpu().numpy()
            cb = torch.nn.functional.grid_sample(cA, g, align_corners=False)[0, 0, :, 0].cpu().numpy()
            out["points"] = [[_num(Wb * (xb[i, 0] + 1) / 2, 3), _num(Hb * (xb[i, 1] + 1) / 2, 3), _num(cb[i], 4)] for i in range(len(uv))]
            if back:
                # 往返:同一次配点的 warp 右半是 B → A(对称配点本来就算了),在配过去的那一点上取它 ⇒ 配回 A 的哪里(不必再配一次)
                wBA = warp[0, :, Ww:, :2].permute(2, 0, 1)[None].float()
                gb = torch.tensor(xb[None, :, None, :].astype(np.float32), device="cuda")
                xa = torch.nn.functional.grid_sample(wBA, gb, align_corners=False)[0, :, :, 0].T.cpu().numpy()
                out["back"] = [[_num(Wa * (xa[i, 0] + 1) / 2, 3), _num(Ha * (xa[i, 1] + 1) / 2, 3)] for i in range(len(uv))]
    out["ms"] = (time.time() - t0) * 1000.0
    return out


def describe(ids):
    """每张存过的帧一个整体特征:RoMa 自带的 DINOv2(ViT-L/14)在 448×448 上的图块特征取平均、归一(1024 维)。
    驱动拿它挑"哪几对画面去配"(看起来最像的先配),配上没有照样按几何核;它自己不进任何量"""
    m = _load_roma()
    if next(m.encoder.dinov2_vitl14[0].parameters()).device.type != "cuda":   # RoMa 第一次配点时才把它搬上 GPU(同它自己的做法)
        m.encoder.dinov2_vitl14[0] = m.encoder.dinov2_vitl14[0].to("cuda").to(m.encoder.amp_dtype)
    enc = m.encoder.dinov2_vitl14[0]
    mean = torch.tensor([0.485, 0.456, 0.406], device="cuda").view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], device="cuda").view(1, 3, 1, 1)
    out = []
    with torch.no_grad():
        for i in ids:
            with _frames_lock:
                img = _frames.get(int(i))
            if img is None:
                raise KeyError("没有存过的帧 %s" % i)
            x = torch.from_numpy(np.asarray(img.resize((448, 448)), dtype=np.float32) / 255.0).permute(2, 0, 1)[None].cuda()
            x = (x - mean) / std
            f = enc.forward_features(x.to(next(enc.parameters()).dtype))["x_norm_patchtokens"].float().mean(1)[0]
            f = f / (f.norm() + 1e-12)
            out.append([_num(v, 5) for v in f.cpu().numpy()])
    return {"ok": True, "vectors": out}


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
            self._send(200, {"instruments": ["match", "segment"], "device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu"})
        else:
            self._send(404, {"ok": False, "err": "no such instrument"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        try:
            req = json.loads(self.rfile.read(n).decode("utf-8"))
        except Exception as e:
            self._send(400, {"ok": False, "err": "bad json: %s" % e})
            return
        try:
            if self.path == "/frame":   # 只解码,不用模型 ⇒ 不排那把锁
                self._send(200, frame_put(req["image"]))
                return
            if self.path == "/match":   # 图先解好(不占锁),再排队用模型
                A = frame_get(req, "a"); B = frame_get(req, "b")
            if self.path not in ("/match", "/describe", "/segment"):
                self._send(404, {"ok": False, "err": "no such instrument"})
                return
            lk = _lock_match if self.path in ("/match", "/describe") else _lock_seg
            with lk:
                if self.path == "/match":
                    out = match(A, B, int(req.get("num", 0)), req.get("points", []), bool(req.get("coarse", False)), bool(req.get("back", False)))
                elif self.path == "/segment":
                    out = segment(req["image"], req.get("box", []), req.get("points", []))
                elif self.path == "/describe":
                    out = describe(req.get("ids", []))
                else:
                    self._send(404, {"ok": False, "err": "no such instrument"})
                    return
            self._send(200, out)
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send(500, {"ok": False, "err": str(e)})


if __name__ == "__main__":
    _load_roma()   # 起来就把模型装进显存,第一帧不慢
    _load_sam()
    print("[仪器] 听 %d · match=roma · segment=sam2.1" % PORT, flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
