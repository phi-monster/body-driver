# still.py KIND [SECONDS]: simulator-side rig code. The robot as robot.py builds it, holding where it starts (every
# group's target its first reading, a drone's velocities nought), with no driver: every second of the world, how far its
# root has gone and leans (when it is free to), and the largest joint speed and reading change of the second. A body
# that cannot hold still on its own never lets a driver measure it at rest.
import os, sys, math, subprocess
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
sys.argv = [sys.argv[0], "--connect", "none:0", "--kind", sys.argv[1], "--seed", "1", "--out", "/tmp/still_" + sys.argv[1],
            "--beats", str(int(float(sys.argv[2] if len(sys.argv) > 2 else 10) * 10))]
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.py")).read()
src = src[:src.index("host, port = args.connect.rsplit")]   # the robot and its loops, not the wire
g = {"__name__": "still", "__file__": os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.py")}
exec(compile(src, "robot.py", "exec"), g)
m, d, groups, control, substeps, true_reading = g["m"], g["d"], g["groups"], g["control"], g["substeps"], g["true_reading"]
for gr in groups:
    if gr["kind"] == "pd" and os.environ.get("STILL_KP"):
        gr["kp"], gr["kd"] = float(os.environ["STILL_KP"]), float(os.environ["STILL_KD"])
import mujoco
prev = {gr["name"]: true_reading(gr).copy() for gr in groups}
start = d.qpos[:3].copy()
for sec in range(1, int(args_beats := int(sys.argv[sys.argv.index("--beats") + 1])) // 10 + 1):
    vmax = 0.0
    for _ in range(10 * substeps):
        control()
        mujoco.mj_step(m, d)
        vmax = max(vmax, float(np.max(np.abs(d.qvel))) if m.nv else 0.0)
    ch = max(float(np.max(np.abs(true_reading(gr) - prev[gr["name"]]))) for gr in groups)
    prev = {gr["name"]: true_reading(gr).copy() for gr in groups}
    root = ""
    if m.njnt and m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE:
        z = np.zeros(9); mujoco.mju_quat2Mat(z, d.qpos[3:7])
        root = " root moved %.4f m, leans %.2f deg, height %.3f" % (float(np.linalg.norm(d.qpos[:3] - start)),
                                                                     math.degrees(math.acos(max(-1, min(1, z[8])))), d.qpos[2])
    if sec in (1, 2, 5) or sec % 10 == 0:
        print("%s %3d s: fastest joint %.2e, largest reading change in the second %.2e%s" % (sys.argv[4] if False else g["args"].kind, sec, vmax, ch, root), flush=True)
