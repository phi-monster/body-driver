#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仪器进程:和 vLLM 的眼一样,是驱动旁边的一个小 HTTP 服务。驱动把一帧(BMP24 base64)POST 过来,这里用任务无关的学习型模型量出物理量,
连同不确定度一起还回去。驱动只认数字,不认模型:换模型只改这里。

接口
  GET  /health            → {"instruments": [...], "device": ...}
  POST /track/start {"image": ..., "points": [[u,v],...]}   → {"ok": true, "id": n}   开一段跟踪,查询点 = 这一帧里的像素
  POST /track/step  {"id": n, "image": ...}                 → {"ok": true, "points": [[u,v,vis,conf],...]}  下一帧里这些点在哪、看不看得见、有多确定
  POST /track/end   {"id": n}                               → {"ok": true}

模型:Track-On2(Aydemir et al. 2025,https://github.com/gorkaydemir/track_on 分支 track-on2,DINOv2 版;代码 MIT,权重 MIT,DINOv2 Apache-2.0)。
权重钉死:trackon2_dinov2_checkpoint.pt(sha256 见 WEIGHTS)。
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
}
TRACKON_DIR = os.environ.get("TRACKON_DIR", "/root/instruments/track_on")
TRACKON_CKPT = os.environ.get("TRACKON_CKPT", "/root/instruments/weights/trackon2_dinov2_checkpoint.pt")

_lock = threading.Lock()
_trackon = None
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
            self._send(200, {"instruments": ["track"], "device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu"})
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
            with _lock:
                if self.path == "/track/start":
                    out = track_start(req["image"], req["points"])
                elif self.path == "/track/step":
                    out = track_step(int(req["id"]), req["image"])
                elif self.path == "/track/end":
                    out = track_end(int(req["id"]))
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
    print("[仪器] 听 %d · track=trackon2" % PORT, flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
