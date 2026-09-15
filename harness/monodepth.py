# 单目出深度:RGB 进,深度图出。当一台【假的深度相机】用 —— 真机只有 RGB,这一层在相机驱动里。
# 尺度是学出来的先验,不是量出来的:所以它属于"眼睛的固件",不属于身体。身体照样只做比较。
# 逐帧独立估会整体漂(实测两帧之间静止处差 3%),而身体靠"上一帧到这一帧变了多少"学表 —— 漂一下表就学歪。
# 锚定分两种(2026-09-16,JI 存图离线实测):
#  · 不动的眼(cam_head):场景几乎不变 ⇒ 每帧按 25/75 分位仿射对齐到第一帧,钉死不再动(原做法)。
#  · 长在手上的眼(*wrist*):画面里最近的那一撮像素是它自己的手指,刚性连着、真实距离恒定,
#    而原始读数会跟着整帧一起漂(指头从 1.01 漂到 0.78,真实距离没变)⇒ 每帧按最近 2% 分位把整幅【只缩不移】回第一帧的尺度。
#    对这只眼用 25/75 仿射是错的:手往桌面走时整幅真的在变近,把分位钉死等于把"我在靠近"抹掉。
#    这不是身体假设:是这台"假深度相机"的固件知道自己装在哪儿,和真深度相机的零点固定在它的底座上是一回事。
import os, numpy as np, torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

_NAME = os.environ.get("BL_MDE", "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf")
_DEV  = os.environ.get("BL_MDE_DEV", "cuda")   # 用这个进程自己看得见的那张卡
_M = None; _P = None; _ANCHOR = {}; _N = 0

def _load():
    #  载不上就退回 CPU;再不行就返回 None(由调用方退回原样)—— 这一层绝不许弄死仿真
    global _M, _P, _DEV
    if _M is None:
        _P = AutoImageProcessor.from_pretrained(_NAME)
        mm = AutoModelForDepthEstimation.from_pretrained(_NAME)
        try:
            _M = mm.to(_DEV).eval().half()
        except Exception as e:
            print("[单目深度] 上卡失败(%s),退回 CPU" % e, flush=True)
            _DEV = "cpu"; _M = mm.eval().float()
    return _P, _M

def _rides_on_hand(key):
    k = str(key).lower()
    return ("wrist" in k) or ("hand" in k)

def depth_from_rgb(rgb, key):
    global _N
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        return None
    h, w = rgb.shape[0], rgb.shape[1]
    p, m = _load()
    with torch.no_grad():
        inp = p(images=rgb[:, :, :3], return_tensors="pt").to(_DEV)
        if _DEV != "cpu":
            inp["pixel_values"] = inp["pixel_values"].half()
        o = m(**inp).predicted_depth
        d = torch.nn.functional.interpolate(o[:, None], size=(h, w), mode="bicubic",
                                            align_corners=False)[0, 0].float().cpu().numpy()
    if _rides_on_hand(key):
        lo = float(np.percentile(d, 2.0))
        a0 = _ANCHOR.get(key)
        if a0 is None:
            _ANCHOR[key] = lo
        elif lo > 1e-6:
            d = d * (a0 / lo)
    else:
        lo, hi = np.percentile(d, [25.0, 75.0])
        a0 = _ANCHOR.get(key)
        if a0 is None:
            _ANCHOR[key] = (float(lo), float(hi))
        elif hi - lo > 1e-6:
            plo, phi = a0
            s = (phi - plo) / (hi - lo)
            d = s * d + (plo - s * lo)
    _N += 1
    if _N <= 3:
        print("[单目深度] %s %dx%d  %.3f..%.3f m  锚=%s" % (key, w, h, float(d.min()), float(d.max()),
              "指头(只缩)" if _rides_on_hand(key) else "25/75 分位(仿射)"), flush=True)
    return np.ascontiguousarray(d.astype(np.float32))
