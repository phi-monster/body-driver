#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仪器进程:和 vLLM 的眼一样,是驱动旁边的一个小 HTTP 服务。驱动把一帧(BMP24 base64)POST 过来,这里用任务无关的学习型模型量出物理量,
连同不确定度一起还回去。驱动只认数字,不认模型:换模型只改这里。

接口
  GET  /health            → {"instruments": [...], "device": ...}
  POST /calib  {"image": "<base64 BMP/PNG>"}
       → {"ok": true, "f": 焦距(px), "f_sd": 焦距不确定度(px), "cx","cy": 主点,
          "up": [x,y,z] 图里"上"的方向(= 重力反向;驱动的相机系:x 右、y 上、z 朝后;单位向量), "up_sd": 弧度,
          "roll","pitch": 弧度, "ms": 毫秒, "model": "geocalib-pinhole-v1.0"}

模型:GeoCalib(Veicht et al. 2024,https://github.com/cvg/GeoCalib;代码 Apache-2.0,权重 CC-BY-4.0)。
权重钉死:v1.0 geocalib-pinhole.tar(sha256 见 WEIGHTS)。
"""
import base64, io, json, os, sys, time, threading, hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image

PORT = int(os.environ.get("INST_PORT", "8077"))
WEIGHTS = {
    # 文件名 → sha256(装好后第一次跑校一次;对不上就拒绝起来,不许悄悄换权重)
    "geocalib/pinhole.tar": os.environ.get("GEOCALIB_SHA256", ""),
}

_lock = threading.Lock()
_geocalib = None


def _load_geocalib():
    global _geocalib
    if _geocalib is None:
        from geocalib import GeoCalib
        _geocalib = GeoCalib(weights="pinhole").to("cuda").eval()
        hub = os.path.join(torch.hub.get_dir(), "geocalib", "pinhole.tar")
        want = WEIGHTS["geocalib/pinhole.tar"]
        if os.path.exists(hub):
            h = hashlib.sha256(open(hub, "rb").read()).hexdigest()
            if want and h != want:
                raise RuntimeError("GeoCalib 权重哈希对不上:%s != %s" % (h, want))
            print("[仪器] geocalib pinhole.tar sha256=%s" % h, flush=True)
    return _geocalib


def _decode(b64):
    raw = base64.b64decode(b64)
    im = Image.open(io.BytesIO(raw)).convert("RGB")
    arr = np.asarray(im, dtype=np.float32) / 255.0          # H W 3
    return torch.from_numpy(arr).permute(2, 0, 1).contiguous()  # 3 H W, 0..1


def calib(b64):
    t0 = time.time()
    m = _load_geocalib()
    img = _decode(b64).to("cuda")
    with torch.no_grad():
        r = m.calibrate(img)
    cam = r["camera"]
    g = r["gravity"]
    f = float(cam.f.flatten()[0])
    cx, cy = [float(x) for x in cam.c.flatten()[:2]] if hasattr(cam, "c") else (0.0, 0.0)
    f_sd = float(r["focal_uncertainty"].flatten()[0]) if "focal_uncertainty" in r else 0.0
    g_sd = float(r["gravity_uncertainty"].flatten()[0]) if "gravity_uncertainty" in r else 0.0
    roll, pitch = [float(x) for x in g.rp.flatten()[:2]]
    # GeoCalib 的 Gravity.vec3d 是图里"上"的方向(roll=pitch=0 时 = (0,-1,0),OpenCV 相机系:x 右、y 下、z 朝前;
    # 2026-09-24 用头眼验过:低头 30° 的相机给 (0,-0.87,+0.5),上方向朝前倾 ⇒ 是"上"不是"下")。
    # 驱动的相机系 x 右、y 上、z 朝后 ⇒ y、z 取反
    v = g.vec3d.flatten().tolist() if hasattr(g, "vec3d") else None
    if v is None:
        import math
        sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
        v = [-sr * cp, -cr * cp, sp]
    up = [v[0], -v[1], -v[2]]
    return {"ok": True, "f": f, "f_sd": f_sd, "cx": cx, "cy": cy, "up": up, "up_sd": g_sd,
            "roll": roll, "pitch": pitch, "ms": (time.time() - t0) * 1000.0, "model": "geocalib-pinhole-v1.0"}


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
            self._send(200, {"instruments": ["calib"], "device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu"})
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
                if self.path == "/calib":
                    out = calib(req["image"])
                else:
                    self._send(404, {"ok": False, "err": "no such instrument"})
                    return
            self._send(200, out)
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send(500, {"ok": False, "err": str(e)})


if __name__ == "__main__":
    _load_geocalib()   # 起来就把模型装进显存,第一帧不慢
    print("[仪器] 听 %d · calib=geocalib" % PORT, flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
