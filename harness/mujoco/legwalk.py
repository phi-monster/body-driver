# legwalk.py SCENARIO [SEED] [ARGS]: simulator-side rig code, no driver. A legged robot of the exam hall (KIND=g1, the
# default, or go2) on its walking program (legged.py), tried the way the driver will use it. SCENARIO:
#   idle     20 s standing, nothing asked;  idle120  120 s
#   walk     standing, then forward, a stop, a turn, a stop, sideways, a stop, backward, a stop
#   group    standing while the group named in argv 3 (waist, arm1_joints, arm2_joints, or arms: both) is sent to drawn
#            targets over FRACTION (argv 4) of each joint's range, a new one every 1.5 s, for 30 s
#   step     standing, then the joints named in argv 3 (comma-separated, without "robot/") sent AMP rad (argv 4) from
#            where they stand, all at once, over RAMP s (argv 5, 0: a step), then 6 s
#   reach    the arms held reaching forward while it walks forward and stops
#   hands    standing while each hand's joints are sent to drawn targets in their ranges; how close they come
# Every half second: what was asked, the odometry (forward and left in the robot's heading, turning), the root's place,
# height and lean, and the share of the half second a foot was off the floor. NOPROPS=1 takes the floor's boxes away;
# NOSLIP=N sets the no-slip solver's iterations; FIRMWARE=DIR runs another of Unitree's walking programs (a Walker's
# policy folder).
import math
import os
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodies
import legged

scenario = sys.argv[1]
seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1
kind = os.environ.get("KIND", "g1")
rng = np.random.default_rng(seed)
built = bodies.KINDS[kind](rng)
if os.environ.get("NOPROPS") == "1":
    for b in list(built["spec"].bodies):
        if b.name.startswith("prop"):
            built["spec"].delete(b)
m = built["spec"].compile()
if os.environ.get("NOSLIP") is not None:
    m.opt.noslip_iterations = int(os.environ["NOSLIP"])
d = mujoco.MjData(m)
if os.environ.get("FIRMWARE"):
    built["walker"]["policy"] = os.environ["FIRMWARE"]
w = legged.firmware(m, d, built["walker"])
w.pose(built["walker"]["height"])
mujoco.mj_forward(m, d)
groups = {g["name"]: g for g in built["groups"]}
def ids(name):   # a group's servos; none for a group the robot has not (the Go2's arms)
    return [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in groups[name]["actuators"]] if name in groups else []
upper = ids("waist") + ids("arm1_joints") + ids("arm2_joints")
print("legwalk: the walking program drives %d of %d joints" % (len(w.legs), len(w.names)), flush=True)
hands = ids("arm1_closer") + ids("arm2_closer")
# Every user-commanded joint held where it stands: a servo given its target, a torque motor its group's PD each step.
gains = {}
for g in built["groups"]:
    if g["kind"] == "pd":
        for a, kp, kd in zip(ids(g["name"]), np.broadcast_to(g["kp"], len(g["actuators"])),
                             np.broadcast_to(g["kd"], len(g["actuators"]))):
            gains[a] = (float(kp), float(kd))
target = {a: float(d.qpos[m.jnt_qposadr[m.actuator_trnid[a, 0]]]) for a in upper + hands}


def servos():
    for a, x in target.items():
        if a in gains:
            j = m.actuator_trnid[a, 0]
            u = gains[a][0] * (x - d.qpos[m.jnt_qposadr[j]]) - gains[a][1] * d.qvel[m.jnt_dofadr[j]]
            d.ctrl[a] = min(max(u, m.actuator_ctrlrange[a, 0]), m.actuator_ctrlrange[a, 1])
        else:
            d.ctrl[a] = x
# The bodies a foot is on: the G1's ankles' last links, the Go2's calves (its feet are their spheres).
feet = [b for b in range(m.nbody) if m.body(b).name.endswith("_ankle_roll_link") or m.body(b).name.endswith("_calf")]
floor = next(m.geom_bodyid[g] for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE)   # the floor's body
dt = m.opt.timestep
print("legwalk: %s %s, seed %d: %d joints, %d servos, %.2f kg, a step %.3f s, the policy every %d steps"
      % (kind, scenario, seed, m.njnt, m.nu, m.body_subtreemass[0], dt, w.every), flush=True)

t, worst = 0.0, 0.0
def run(seconds, speed=(0.0, 0.0, 0.0), each=None):
    global t, worst
    w.speed = np.array(speed, float)
    end, lifted, n, next_print = t + seconds, 0, 0, t + 0.5
    while t < end - 1e-9:
        if each:
            each(t)
        servos()
        w.control()
        mujoco.mj_step(m, d)
        t += dt
        touching = set()
        for c in range(d.ncon):
            b1, b2 = m.geom_bodyid[d.contact[c].geom1], m.geom_bodyid[d.contact[c].geom2]
            for f in feet:
                if (b1 == f and b2 == floor) or (b2 == f and b1 == floor):
                    touching.add(f)
        lifted += len(touching) < len(feet)
        n += 1
        worst = max(worst, w.lean())
        if t >= next_print - 1e-9:
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, d.qpos[3:7])
            yaw = math.atan2(R[3], R[0])
            o = w.odometry()
            c, s = math.cos(yaw), math.sin(yaw)
            print("t %5.1f asked %5.2f %5.2f %5.2f | odometry in heading %5.2f %5.2f %5.2f | pelvis %6.3f %6.3f yaw %6.1f deg "
                  "height %.3f lean %4.1f deg | a foot off %3.0f%%%s"
                  % (t, speed[0], speed[1], speed[2], c * o[0] + s * o[1], -s * o[0] + c * o[1], o[2], d.qpos[0], d.qpos[1],
                     math.degrees(yaw), d.qpos[2], math.degrees(w.lean()), 100.0 * lifted / max(1, n),
                     "  FALLEN" if w.fallen() else ""), flush=True)
            lifted, n, next_print = 0, 0, next_print + 0.5

def drawn_targets(acts, frac, every):
    state = {"next": 0.0}
    def each(now):
        if now >= state["next"]:
            for a in acts:
                j = m.actuator_trnid[a, 0]
                lo, hi = m.jnt_range[j]
                mid = 0.5 * (lo + hi)
                home = d.qpos[m.jnt_qposadr[j]] if "home" not in state else state["home"][a]
                half = 0.5 * frac * (hi - lo)
                target[a] = float(np.clip(rng.uniform(home - half, home + half), lo, hi))
            state["next"] = now + every
    return each, state

if scenario == "idle":
    run(20.0)
elif scenario == "idle120":
    run(120.0)
elif scenario == "step":
    names = [built["walker"]["prefix"] + n for n in sys.argv[3].split(",")]
    amp, ramp = float(sys.argv[4]), float(sys.argv[5])
    acts = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in names]
    run(2.0)
    start = {a: target[a] for a in acts}
    t0 = t
    def each(now):
        f = 1.0 if ramp <= 0 else min(1.0, (now - t0) / ramp)
        for a in acts:
            lo, hi = m.actuator_ctrlrange[a]
            target[a] = float(np.clip(start[a] + f * amp, lo, hi))
    run(6.0 + max(0.0, ramp), each=each)
elif scenario == "walk":
    for sp, sec in [((0, 0, 0), 3), ((0.5, 0, 0), 5), ((0, 0, 0), 4), ((0, 0, 0.8), 3), ((0, 0, 0), 3), ((0, 0.3, 0), 3),
                    ((0, 0, 0), 3), ((-0.3, 0, 0), 3), ((0, 0, 0), 3)]:
        run(sec, sp)
elif scenario == "group":
    which = sys.argv[3]
    acts = ids("arm1_joints") + ids("arm2_joints") if which == "arms" else ids(which)
    run(2.0)
    home = {a: target[a] for a in acts}
    each, state = drawn_targets(acts, float(sys.argv[4]), 1.5)
    state["home"] = home
    run(30.0, each=each)
    for a in acts:
        target[a] = home[a]
    run(4.0)
elif scenario == "reach":
    run(2.0)
    for side in ("left", "right"):
        target[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, built["walker"]["prefix"] + "%s_shoulder_pitch_joint" % side)] = -1.3
        target[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, built["walker"]["prefix"] + "%s_elbow_joint" % side)] = 0.2
    run(3.0)
    run(5.0, (0.5, 0, 0))
    run(4.0)
elif scenario == "hands":
    run(2.0)
    errs = []
    for trial in range(6):
        targets = {}
        for a in hands:
            j = m.actuator_trnid[a, 0]
            lo, hi = m.jnt_range[j]
            targets[a] = float(rng.uniform(lo, hi))
            target[a] = targets[a]
        run(1.5)
        e = [abs(d.qpos[m.jnt_qposadr[m.actuator_trnid[a, 0]]] - targets[a]) for a in hands]
        errs.append(e)
        print("hands trial %d: after 1.5 s each joint within %.3f rad of its target at worst, %.3f on average"
              % (trial, max(e), float(np.mean(e))), flush=True)
print("legwalk: %s done: the most it leaned %.1f deg%s" % (scenario, math.degrees(worst),
      ", it FELL" if worst > legged.FELL else ", it never fell"), flush=True)
