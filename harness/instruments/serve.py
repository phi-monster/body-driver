#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仪器进程:和 vLLM 的眼一样,是驱动旁边的一个小 HTTP 服务。驱动把一帧(BMP24 base64)POST 过来,这里用任务无关的学习型模型量出物理量,
连同不确定度一起还回去。驱动只认数字,不认模型:换模型只改这里。

接口
  GET  /health            → {"instruments": [...], "device": ...}
  POST /track/start {"image": ..., "points": [[u,v],...]}   → {"ok": true, "id": n}   开一段跟踪,查询点 = 这一帧里的像素
  POST /track/step  {"id": n, "image": ...}                 → {"ok": true, "points": [[u,v,vis,conf],...]}  下一帧里这些点在哪、看不看得见、有多确定
  POST /track/end   {"id": n}                               → {"ok": true}
  POST /frame {"image": ...}                                 → {"ok": true, "id": n}
       一帧先存在这边(解码一次),之后配点只报编号 —— 开机扫描同一帧要和几十帧配,每次都把 1 MB 的图转成文字再传一遍,
       比配点本身还慢(2026-09-26 实测:一对 1.9 秒,里面配点 0.8 秒)。只留最近 1200 帧
  POST /match {"a": ..., "b": ..., "num": N, "points": [[u,v],...]}   (或者 "a_id" / "b_id" 代替 "a" / "b":用 /frame 存过的帧)
       → {"ok": true, "samples": [[ua,va,ub,vb,cert],...], "points": [[ub,vb,cert],...]}
       两台相机(或同一台相机两个位置)的两帧里,哪两个像素是同一个真实的点:num > 0 抽 num 对对应点;points = A 里的像素,问它们在 B 里在哪。
       cert = 模型自己给的可信度(0..1),驱动不拿它当真,只拿几何去核(三角重投、两停交叉)
  POST /segment {"image": ..., "box": [x0,y0,x1,y1], "points": [[u,v,label],...]}
       → {"ok": true, "w": W, "h": H, "area": n, "box": [x0,y0,x1,y1], "score": s, "runs": [r0, r1, ...]}
       框(脑给的)/ 点(label 1 = 在它身上、0 = 不在)⇒ 那件东西的像素:runs = 整幅按行展开的游程,先"不是"一段、再"是"一段……交替。
       score = 模型自报的 IoU(0..1),驱动只报数、不拿它当门

模型:Track-On2(Aydemir et al. 2025,https://github.com/gorkaydemir/track_on 分支 track-on2,DINOv2 版;代码 MIT,权重 MIT,DINOv2 Apache-2.0)。
      RoMa(Edstedt et al., CVPR 2024,https://github.com/Parskatt/RoMa commit 77f8d68,代码 MIT;权重 roma_outdoor.pth + DINOv2 ViT-L/14)。
      2026-09-25 owner 批准装 RoMa:固定的头顶眼只靠看手定不准焦距(分割出来的指尖不是手上一个固定的点),拿腕眼三角出来的桌面点当标定板,要在头顶眼里认出同一批点。
      SAM 2.1(Ravi et al. 2024,https://github.com/facebookresearch/sam2 commit 2b90b9f,代码 + 权重 Apache-2.0;hiera-small 46M)。
      2026-09-26 owner 批准装 SAM("装 sam"):驱动自己按明暗切的块常常只是东西的一截、或连着别的东西(SHOT1 剪刀被画面边切着,两只眼的"中心"差 1.7 cm);
      脑给框 ⇒ SAM 出这件东西的整片像素。实测(SHOT1 存图):头顶眼 1725 px、腕眼 6238 px 正好是整把剪刀,一次 0.04 s,显存 0.55 GB。
权重钉死:见 WEIGHTS(sha256);对不上就拒绝起来。
拆掉的(2026-09-24,owner:焦距不准的不留):GeoCalib(单图焦距偏一成、头眼"上"方向反了)、MoGe-2(单目猜深度,尺度核对不了)。
"""
import base64, io, json, os, sys, time, threading, hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image

PORT = int(os.environ.get("INST_PORT", "8077"))
WEIGHTS = {
    # 文件名 → sha256(装好后第一次跑校一次;对不上就拒绝起来,不许悄悄换权重)
    "trackon2_dinov2_checkpoint.pt": "34c35ea64ea68f3c633c901c2d7876964c34212455dfbb2d508aaea1c4978973",
    "roma_outdoor.pth": "c7a45c80d41ad788a63c641d1b686d7cb3f297f40097c6f4e75039889e5cc8ba",
    "dinov2_vitl14_pretrain.pth": "d5383ea8f4877b2472eb973e0fd72d557c7da5d3611bd527ceeb1d7162cbf428",
    "sam2.1_hiera_small.pt": "6d1aa6f30de5c92224f8172114de081d104bbd23dd9dc5c58996f0cad5dc4d38",
}
ROMA_DIR = os.environ.get("ROMA_DIR", "/root/instruments/RoMa")
ROMA_W = os.environ.get("ROMA_W", os.path.expanduser("~/.cache/torch/hub/checkpoints/roma_outdoor.pth"))
ROMA_DINO = os.environ.get("ROMA_DINO", os.path.expanduser("~/.cache/torch/hub/checkpoints/dinov2_vitl14_pretrain.pth"))
TRACKON_DIR = os.environ.get("TRACKON_DIR", "/root/instruments/track_on")
TRACKON_CKPT = os.environ.get("TRACKON_CKPT", "/root/instruments/weights/trackon2_dinov2_checkpoint.pt")
SAM_CKPT = os.environ.get("SAM_CKPT", "/root/instruments/weights/sam2.1_hiera_small.pt")
SAM_CFG = os.environ.get("SAM_CFG", "configs/sam2.1/sam2.1_hiera_s.yaml")   # 仓库在 /root/instruments/sam2_repo(不许放在本文件同级叫 sam2:会遮住包)

_lock = threading.Lock()   # 跟点(Track-On2)用
# 每个模型一把锁(2026-09-26):原来三件仪器共用 _lock,开机扫描时跟点要排在配点后面等(一对配点 0.4–0.85 秒),扫描慢了一倍
_lock_match = threading.Lock()
_lock_seg = threading.Lock()
_trackon = None
_roma = None
_sam = None
_sessions = {}      # id → 跟踪状态(一个模型,多段各自的记忆)
_next_id = [1]


def _load_trackon():
    global _trackon
    if _trackon is None:
        h = hashlib.sha256(open(TRACKON_CKPT, "rb").read()).hexdigest()
        if h != WEIGHTS["trackon2_dinov2_checkpoint.pt"]:
            raise RuntimeError("Track-On2 权重哈希对不上:%s" % h)
        sys.path.insert(0, TRACKON_DIR)
        cwd = os.getcwd(); os.chdir(TRACKON_DIR)
        try:
            from model.trackon_predictor import Predictor
            from utils.train_utils import load_args_from_yaml
            args = load_args_from_yaml(os.path.join(TRACKON_DIR, "config", "test_dinov2.yaml"))
            _trackon = Predictor(args, checkpoint_path=TRACKON_CKPT, support_grid_size=0).to("cuda").eval()
        finally:
            os.chdir(cwd)
        print("[仪器] trackon2_dinov2 sha256=%s" % h, flush=True)
    return _trackon


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


def match(A, B, num, points, coarse=False):
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
    out["ms"] = (time.time() - t0) * 1000.0
    return out


def _trk_state(m):
    return (m.t, m.point_memory, m.temporal_mask, m.q_init, m.N)


def _trk_restore(m, st):
    m.t, m.point_memory, m.temporal_mask, m.q_init, m.N = st


def _trk_frame(b64):
    return (_decode(b64) * 255.0)[None].to("cuda")   # (1,3,H,W) 0..255


def _trk_step(m, frame, new_queries=None):
    """同 Predictor.forward_frame,但把可见度的概率一起给出来(它自己只给阈值后的真假)"""
    _, _, H, W = frame.shape
    f4, f8, f16, f32, ff = m.model.extract_frame_features(frame)
    if new_queries is not None and new_queries.shape[0] > 0:
        m.init_queries((ff, frame.device), new_queries, H, W)
    if m.q_init is None or m.N == 0:
        return torch.empty(0, 2, device=frame.device), torch.empty(0, device=frame.device)
    p, v_logit, q_new = m.model.track_frame(m.q_init[:m.N], m.temporal_mask[:m.N], m.point_memory[:m.N], (f4, f8, f16, f32, ff), H, W)
    m.point_memory[:m.N] = torch.roll(m.point_memory[:m.N], shifts=-1, dims=1)
    m.point_memory[:m.N, -1] = q_new
    m.temporal_mask[:m.N] = torch.roll(m.temporal_mask[:m.N], shifts=-1, dims=1)
    m.temporal_mask[:m.N, -1] = False
    m.t += 1
    return p, v_logit.sigmoid()


def track_start(b64, points):
    m = _load_trackon()
    m.reset()
    q = torch.tensor(points, dtype=torch.float32, device="cuda").reshape(-1, 2)
    with torch.no_grad():
        p, c = _trk_step(m, _trk_frame(b64), new_queries=q)
    sid = _next_id[0]; _next_id[0] += 1
    _sessions[sid] = _trk_state(m)
    m.reset()
    return {"ok": True, "id": sid, "points": [[float(p[i, 0]), float(p[i, 1]), int(c[i] >= m.delta_v), float(c[i])] for i in range(p.shape[0])],
            "model": "trackon2-dinov2"}


def track_step(sid, b64):
    if sid not in _sessions:
        return {"ok": False, "err": "no such track session %s" % sid}
    m = _load_trackon()
    t0 = time.time()
    _trk_restore(m, _sessions[sid])
    with torch.no_grad():
        p, c = _trk_step(m, _trk_frame(b64))
    _sessions[sid] = _trk_state(m)
    m.reset()
    return {"ok": True, "points": [[float(p[i, 0]), float(p[i, 1]), int(c[i] >= m.delta_v), float(c[i])] for i in range(p.shape[0])],
            "ms": (time.time() - t0) * 1000.0}


def track_end(sid):
    _sessions.pop(sid, None)
    return {"ok": True}


def _decode(b64):
    raw = base64.b64decode(b64)
    im = Image.open(io.BytesIO(raw)).convert("RGB")
    arr = np.asarray(im, dtype=np.float32) / 255.0          # H W 3
    return torch.from_numpy(arr).permute(2, 0, 1).contiguous()  # 3 H W, 0..1


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
            self._send(200, {"instruments": ["track", "match", "segment"], "device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu"})
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
            lk = _lock_match if self.path == "/match" else (_lock_seg if self.path == "/segment" else _lock)
            with lk:
                if self.path == "/track/start":
                    out = track_start(req["image"], req["points"])
                elif self.path == "/track/step":
                    out = track_step(int(req["id"]), req["image"])
                elif self.path == "/track/end":
                    out = track_end(int(req["id"]))
                elif self.path == "/match":
                    out = match(A, B, int(req.get("num", 0)), req.get("points", []), bool(req.get("coarse", False)))
                elif self.path == "/segment":
                    out = segment(req["image"], req.get("box", []), req.get("points", []))
                else:
                    self._send(404, {"ok": False, "err": "no such instrument"})
                    return
            self._send(200, out)
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send(500, {"ok": False, "err": str(e)})


if __name__ == "__main__":
    _load_trackon()   # 起来就把模型装进显存,第一帧不慢
    _load_roma()
    _load_sam()
    print("[仪器] 听 %d · track=trackon2 · match=roma · segment=sam2.1" % PORT, flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
