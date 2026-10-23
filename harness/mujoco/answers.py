# answers.py KIND...: simulator-side rig code, no driver. Whether every channel a robot is commanded through answers its
# command, both ways: each kind booted as robot.py boots it (bodies.build, seed 1), held still for a second, then each
# servo or torque-motored joint (a group of kind servo or pd) asked 0.05 and 0.3 of its own units each way from where it
# stands, every other channel held, for 2 s, and held back where it was for 2 s before the next ask. Prints how far its reading went, as a share of each ask. Found on 10-23:
# ALOHA's two waists, the UR5e's shoulder pan and RB-Y1's four wheels did not answer (bodies.py says why).
import os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodies

ASKS = (0.05, -0.05, 0.3, -0.3)


def reading(m, d, g, i):
    """The channel's reading in its command's units, as robot.py's true reading: a servo's hold target or velocity, a
    torque-motored joint's angle."""
    if g["kind"] == "pd":
        return float(d.qpos[m.jnt_qposadr[m.actuator_trnid[i, 0]]])
    g0, b0, b1, b2 = (m.actuator_gainprm[i, 0], m.actuator_biasprm[i, 0], m.actuator_biasprm[i, 1],
                      m.actuator_biasprm[i, 2])
    return float(-(b0 + b1 * d.actuator_length[i]) / g0 if b1 != 0.0 else -(b0 + b2 * d.actuator_velocity[i]) / g0)


def drive(m, d, channels, walker):
    """Every channel to its target for one step: a servo as robot.py sends it (bodies.servo_command), a torque motor
    its group's PD."""
    for i, (g, k, target) in channels.items():
        if g["kind"] == "pd":
            kp, kd = np.broadcast_to(g["kp"], len(g["actuators"]))[k], np.broadcast_to(g["kd"], len(g["actuators"]))[k]
            j = m.actuator_trnid[i, 0]
            u = kp * (target - d.qpos[m.jnt_qposadr[j]]) - kd * d.qvel[m.jnt_dofadr[j]]
            d.ctrl[i] = min(max(u, m.actuator_ctrlrange[i, 0]), m.actuator_ctrlrange[i, 1])
        else:
            d.ctrl[i] = bodies.servo_command(m, d, g, i, target)
    if walker:
        walker.control()


for kind in sys.argv[1:]:
    built, m, d, walker = bodies.build(kind, np.random.default_rng(1))
    held = {}
    for g in built["groups"]:
        if g["kind"] in ("servo", "pd"):
            for k, a in enumerate(g["actuators"]):
                i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
                held[i] = (g, k, reading(m, d, g, i))
    def hold(channels, seconds):
        for t in range(int(round(seconds / m.opt.timestep))):
            drive(m, d, channels, walker)
            mujoco.mj_step(m, d)
    hold(held, 1.0)
    for i, (g, k, target) in held.items():
        went = []
        for ask in ASKS:   # each ask from where it stands, then back where it was held for as long
            start = reading(m, d, g, i)
            channels = dict(held)
            channels[i] = (g, k, start + ask)
            hold(channels, 2.0)
            went.append((reading(m, d, g, i) - start) / ask)
            hold(held, 2.0)
        lo, hi = m.actuator_ctrlrange[i]
        print("%s %-12s %-24s from %8.4f in [%7.3f %7.3f]: went %s of each ask (%s)"
              % (kind, g["name"], g["actuators"][k], target, lo, hi, " ".join("%5.2f" % w for w in went),
                 " ".join("%+.2f" % a for a in ASKS)), flush=True)
