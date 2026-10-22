# fall.py KIND JOINT... : simulator-side rig code. The robot as robot.py builds it, holding where it starts, no driver;
# every tenth of a second for two seconds: the named joints' angles and the root's lean, to see what gives way first.
import os, sys, math
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
kind, joints = sys.argv[1], sys.argv[2:]
sys.argv = [sys.argv[0], "--connect", "none:0", "--kind", kind, "--seed", "1", "--out", "/tmp/fall_" + kind]
here = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(here, "robot.py")).read()
g = {"__name__": "fall", "__file__": os.path.join(here, "robot.py")}
exec(compile(src[:src.index("host, port = args.connect.rsplit")], "robot.py", "exec"), g)
m, d, control, substeps = g["m"], g["d"], g["control"], g["substeps"]
import mujoco
ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in joints]
for t in range(21):
    z = np.zeros(9); mujoco.mju_quat2Mat(z, d.qpos[3:7])
    print("%.1f s: lean %5.1f deg; %s" % (t * 0.1, math.degrees(math.acos(max(-1, min(1, z[8])))),
          " ".join("%s %+.3f" % (j, d.qpos[m.jnt_qposadr[i]]) for j, i in zip(joints, ids))), flush=True)
    for _ in range(substeps):
        control()
        mujoco.mj_step(m, d)
