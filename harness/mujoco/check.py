# check.py OUT KIND...: simulator-side rig code. Each kind built as robot.py builds it (seed 1, its stand fixed, its
# keyframe taken, a legged robot in its walking program's pose), its groups' actuators and channels counted, and every camera's first picture side by side in
# OUT/KIND.png, to see that each camera is where it was meant to be and sees what it should.
import os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
from PIL import Image
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodies
import legged

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
for kind in sys.argv[2:]:
    try:
        b = bodies.KINDS[kind](np.random.default_rng(1))
        spec = b["spec"]
        if b["fix"]:
            for k in list(spec.keys):
                spec.delete(k)
            body = bodies.find_body(spec, b["fix"])
            for j in list(body.joints):
                if j.type == mujoco.mjtJoint.mjJNT_FREE:
                    spec.delete(j)
        m = spec.compile()
        d = mujoco.MjData(m)
        if b["key"]:
            mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, b["key"]))
        if b.get("walker"):   # a legged robot: posed as its walking program boots it
            legged.firmware(m, d, b["walker"]).pose(b["walker"]["height"])
        for name, value in (b.get("init") or {}).items():   # the pose such a robot is booted in
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] = value
        mujoco.mj_forward(m, d)
        missing = [a for g in b["groups"] for a in g["actuators"] if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) < 0]
        r = mujoco.Renderer(m, 240, 320)
        imgs = []
        for c in b["cameras"]:
            r.update_scene(d, camera=c["name"])
            imgs.append(r.render().copy())
        r.close()
        Image.fromarray(np.concatenate(imgs, axis=1)).save(os.path.join(out, kind + ".png"))
        print("%s: %d actuators in %d groups (%s), %d of the model's %d, missing %s; cameras %s"
              % (kind, sum(len(g["actuators"]) for g in b["groups"]), len(b["groups"]),
                 " ".join("%s:%s:%d" % (g["name"], g["role"], len(g["actuators"])) for g in b["groups"]),
                 sum(len(g["actuators"]) for g in b["groups"]) - len(missing), m.nu, missing or "none",
                 " ".join("%s:%s" % (c["name"], c["mount"]) for c in b["cameras"])), flush=True)
    except Exception as e:
        print("%s: FAILED %s: %s" % (kind, type(e).__name__, e), flush=True)
