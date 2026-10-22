# cand.py OUT KIND BODY X Y Z: simulator-side rig code, to choose a camera's direction by sight: six cameras on BODY at
# (X, Y, Z) of its frame, looking along +x, -x, +y, -y, +z, -z of it, their pictures in a 3 x 2 grid in OUT/KIND_BODY.png.
import os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
from PIL import Image
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodies
out, kind, body = sys.argv[1], sys.argv[2], sys.argv[3]
pos = [float(v) for v in sys.argv[4:7]]
b = bodies.KINDS[kind](np.random.default_rng(1))
spec = b["spec"]
if b["fix"]:
    for k in list(spec.keys):
        spec.delete(k)
    for j in list(bodies.find_body(spec, b["fix"]).joints):
        if j.type == mujoco.mjtJoint.mjJNT_FREE:
            spec.delete(j)
dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
for i, f in enumerate(dirs):
    up = (0.0, 0.0, 1.0) if f[2] == 0 else (1.0, 0.0, 0.0)
    bodies.find_body(spec, body).add_camera(name="cand%d" % i, pos=pos, xyaxes=bodies.look_axes(f, up), fovy=70.0)
m = spec.compile()
d = mujoco.MjData(m)
if b["key"]:
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, b["key"]))
for name, value in (b.get("init") or {}).items():
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] = value
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 240, 320)
imgs = []
for i in range(6):
    r.update_scene(d, camera="cand%d" % i)
    imgs.append(r.render().copy())
r.close()
grid = np.concatenate([np.concatenate(imgs[0:3], axis=1), np.concatenate(imgs[3:6], axis=1)], axis=0)
os.makedirs(out, exist_ok=True)
Image.fromarray(grid).save(os.path.join(out, "%s_%s.png" % (kind, body)))
print("saved", kind, body)
