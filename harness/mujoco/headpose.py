# headpose.py OUT KIND CAMERA PAN_JOINT TILT_JOINT TILT PAN...: simulator-side rig code, to choose a head's boot pose by
# sight: the camera's picture at each pan angle (the tilt held), side by side in OUT/KIND_head.png.
import os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
from PIL import Image
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodies
out, kind, cam, pan_j, tilt_j, tilt = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], float(sys.argv[6])
pans = [float(p) for p in sys.argv[7:]]
b = bodies.KINDS[kind](np.random.default_rng(1))
m = b["spec"].compile()
d = mujoco.MjData(m)
r = mujoco.Renderer(m, 240, 320)
imgs = []
for p in pans:
    mujoco.mj_resetData(m, d)
    for name, value in (b.get("init") or {}).items():
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] = value
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, pan_j)]] = p
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, tilt_j)]] = tilt
    mujoco.mj_forward(m, d)
    r.update_scene(d, camera=cam)
    imgs.append(r.render().copy())
r.close()
os.makedirs(out, exist_ok=True)
Image.fromarray(np.concatenate(imgs, axis=1)).save(os.path.join(out, kind + "_head.png"))
print("saved", pans)
