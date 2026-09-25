# body-driver 测试钩子(2026-09-25,V1:头顶眼被转了 / 被挡了一半 ⇒ 身体要自己发现、重新标、接着干)。仿真这边的,驱动一个字不知道。
# 触发:写 /root/camtest.json(或环境变量 BD_CAMTEST 指的文件),每一步看一次:
#   {"cam": "cam_head", "roll_deg": 90, "tag": "a"} ⇒ 这台相机绕它自己的光轴转 roll_deg 度(USD 相机朝本地 −z 看 ⇒ 绕相机 prim 的本地 z 轴),
#                                                   同一个 (cam, roll_deg, tag) 只转一次;仿真从下一帧起渲染转过的画面(真转,不是把图转一下)。
#   {"cam": "cam_head", "cover": "left"}            ⇒ 从这一帧起这台相机彩色图的左半边(或 "right")置黑:镜头被挡住一半
#   删掉文件 ⇒ 挡的那条撤掉(转过的不回去)
# 转之前、转之后各存一张这台相机的彩色图(/root/camtest_<tag>_before.png / _after.png),核对画面真转了。
# 为什么转渲染用的那台相机、写 Fabric:渲染读的是 Fabric 里的变换;只改 USD 里它上一层 xform 的位姿,画面不变(G2G 2026-09-25 实测)
import json, os, math
import numpy as np

_rolled = set()
_pending_after = {}   # tag → 还要存"转之后"那张的剩余帧数(隔几帧再存,等渲染跟上)


def _qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def _to_np(x):
    return (x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)).astype(float)


def _like(v, ref):
    if hasattr(ref, "detach"):
        import torch
        return torch.tensor(v, dtype=ref.dtype, device=ref.device)
    return np.asarray(v, dtype=np.asarray(ref).dtype)


def _save(obs, env_idx, cam, path):
    try:
        from PIL import Image
        v = obs[env_idx]["vision"].get(cam)
        if v is not None and "color" in v:
            Image.fromarray(np.asarray(v["color"]).astype(np.uint8)).save(path)
            print("[camtest] 存了 %s" % path, flush=True)
    except Exception as e:
        print("[camtest] 存图没成:", e, flush=True)


def apply(om, obs, env_idx_list):
    for tag in list(_pending_after):
        _pending_after[tag] -= 1
        if _pending_after[tag] <= 0:
            cam = tag.split("|")[0]
            _save(obs, env_idx_list[0], cam, "/root/camtest_%s_after.png" % tag.split("|")[1])
            del _pending_after[tag]
    p = os.environ.get("BD_CAMTEST", "/root/camtest.json")
    if not os.path.exists(p):
        return
    try:
        cfg = json.load(open(p))
    except Exception:
        return
    cam = cfg.get("cam", "cam_head")
    cm = om.camera_manager
    cap = getattr(om, "capture_manager", None)
    if "roll_deg" in cfg and cm is not None:
        tag = str(cfg.get("tag", ""))
        key = (cam, float(cfg["roll_deg"]), tag)
        if key not in _rolled:
            _rolled.add(key)
            _save(obs, env_idx_list[0], cam, "/root/camtest_%s_before.png" % tag)
            th = math.radians(float(cfg["roll_deg"]))
            qr = [math.cos(th / 2), 0.0, 0.0, math.sin(th / 2)]   # (w, x, y, z):绕相机 prim 的本地 z
            names = cm.camera_names[0]
            tcs = getattr(cap, "tiled_cameras", None) if cap is not None else None
            if cam not in names or not tcs:
                print("[camtest] 找不到 %s 的渲染相机(tiled_cameras 没有)⇒ 没转" % cam, flush=True)
            else:
                tc = tcs[names.index(cam)]
                for use_usd in (False, True):   # Fabric(渲染读的)和 USD 都写,两边一致
                    pos, q = tc.get_world_poses(usd=use_usd)
                    qv = _to_np(q).reshape(-1, 4)
                    qn = np.array([_qmul(r, qr) for r in qv])
                    qn /= np.linalg.norm(qn, axis=1, keepdims=True)
                    tc.set_world_poses(pos, _like(qn, q), usd=use_usd)
                    print("[camtest] %s 绕自己的光轴转了 %.1f°(%s:四元数 %s → %s)" % (cam, float(cfg["roll_deg"]), "USD" if use_usd else "Fabric",
                                                                    np.round(qv[0], 4).tolist(), np.round(qn[0], 4).tolist()), flush=True)
                _pending_after[cam + "|" + tag] = 5   # 5 帧之后存"转之后"那张(次数)
    side = cfg.get("cover")
    if side in ("left", "right"):
        for env_idx in env_idx_list:
            v = obs[env_idx]["vision"].get(cam)
            if v is not None and "color" in v:
                img = np.array(v["color"], copy=True)
                w = img.shape[1]
                if side == "left":
                    img[:, : w // 2] = 0
                else:
                    img[:, w // 2:] = 0
                v["color"] = img
