# body-driver 测试钩子(2026-09-25,V1:头顶眼被转了 / 被挡了一半 ⇒ 身体要自己发现、重新标、接着干)。仿真这边的,驱动一个字不知道。
# 触发:写 /root/camtest.json(或环境变量 BD_CAMTEST 指的文件),每一步看一次:
#   {"cam": "cam_head", "roll_deg": 90}   ⇒ 这台相机绕它自己的光轴转 roll_deg 度(USD 相机朝本地 −z 看 ⇒ 绕本地 z 轴),同一条只转一次,
#                                          仿真从下一帧起渲染转过的画面(真转,不是把图转一下)。要再转一次就换一个 "tag"
#   {"cam": "cam_head", "cover": "left"}  ⇒ 从这一帧起这台相机彩色图的左半边(或 "right")置黑:镜头被挡住一半
#   删掉文件 ⇒ 挡的那条撤掉(转过的不回去)
import json, os, math
import numpy as np

_rolled = set()


def _qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def apply(om, obs, env_idx_list):
    p = os.environ.get("BD_CAMTEST", "/root/camtest.json")
    if not os.path.exists(p):
        return
    try:
        cfg = json.load(open(p))
    except Exception:
        return
    cam = cfg.get("cam", "cam_head")
    cm = om.camera_manager
    if "roll_deg" in cfg and cm is not None:
        key = (cam, float(cfg["roll_deg"]), str(cfg.get("tag", "")))
        if key not in _rolled:
            th = math.radians(float(cfg["roll_deg"]))
            qr = [math.cos(th / 2), 0.0, 0.0, math.sin(th / 2)]   # (w, x, y, z):绕本地 z
            for env_idx in env_idx_list:
                names = cm.camera_names[env_idx]
                if cam in names:
                    xf = cm.cameras_xform[env_idx][names.index(cam)]
                    pos, q = xf.get_local_pose()
                    is_torch = hasattr(q, "detach")
                    qv = (q.detach().cpu().numpy() if is_torch else np.asarray(q)).astype(float).reshape(-1)
                    qn = np.array(_qmul(qv, qr))
                    qn /= np.linalg.norm(qn)
                    if is_torch:
                        import torch
                        qn = torch.tensor(qn, dtype=q.dtype, device=q.device)
                    xf.set_local_pose(pos, qn)
                    print("[camtest] %s 绕自己的光轴转了 %.1f°(四元数 %s → %s)" % (cam, float(cfg["roll_deg"]), np.round(qv, 4).tolist(),
                                                                       np.round(np.asarray(qn.cpu() if is_torch else qn), 4).tolist()), flush=True)
            _rolled.add(key)
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
