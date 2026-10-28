# controltest.py: simulator-side rig code. The robot's loops taken a group at once (robot.py control(),
# bodies.servo_commands, legged's controllers) against the same loops joint by joint as they were before 10-28, on two
# copies of each kind built from one seed: every step the same targets, and after every beat the two worlds compared to
# the bit (positions, velocities, commands). Also each loop's own time a beat, both ways.
#   python controltest.py [BEATS] [KIND...]
import math, sys, time
import numpy as np
import mujoco
import bodies, legged

BEATS = int(sys.argv[1]) if len(sys.argv) > 1 else 200
KINDS = sys.argv[2:] or ["panda", "ur5e", "so101", "aloha", "rby1", "g1", "shadow", "stretch", "x2"]
BEAT = 0.1


def setup(kind, seed):
    built, m, d, walker = bodies.build(kind, np.random.default_rng(seed))
    groups = built["groups"]
    for g in groups:
        g["ids"] = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in g["actuators"]]
        g["n"] = len(g["ids"]) if g["ids"] else g["channels"]
        if g["kind"] in ("pd", "sensor"):
            g["qadr"] = [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in g["ids"]]
            g["vadr"] = [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in g["ids"]]
        if g["kind"] in ("pd", "velocity"):
            g["lo"], g["hi"] = m.actuator_ctrlrange[g["ids"], 0].copy(), m.actuator_ctrlrange[g["ids"], 1].copy()
        if g["kind"] == "pd":
            g["kp_n"] = np.array(np.broadcast_to(g["kp"], g["n"]), dtype=np.float64)
            g["kd_n"] = np.array(np.broadcast_to(g["kd"], g["n"]), dtype=np.float64)
    for g in groups:
        if g["kind"] == "servo":
            out = []
            for i in g["ids"]:
                g0, b0, b1, b2 = (m.actuator_gainprm[i, 0], m.actuator_biasprm[i, 0], m.actuator_biasprm[i, 1],
                                  m.actuator_biasprm[i, 2])
                out.append(-(b0 + b1 * d.actuator_length[i]) / g0 if b1 != 0.0 else -(b0 + b2 * d.actuator_velocity[i]) / g0)
            g["ref"] = np.array(out, dtype=float)
        elif g["kind"] in ("pd", "sensor"):
            g["ref"] = np.array([d.qpos[a] for a in g["qadr"]], dtype=float)
        elif g["kind"] == "velocity":
            g["ref"] = np.zeros(g["n"])
        else:
            g["ref"] = np.zeros(g["n"])
    return m, d, walker, groups


def old_gearwbc_control(self):   # legged.GearWbc.control as it was
    d = self.d
    if self.k % self.every == 0:
        cmd = self.asked()
        obs = np.zeros(86, np.float32)
        obs[0:3] = cmd * self.cmd_scale
        obs[3] = self.height_cmd
        obs[4:7] = self.rpy_cmd
        obs[7:10] = d.qvel[self.root_v + 3: self.root_v + 6] * self.ang_vel_scale
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, d.qpos[self.root_q + 3: self.root_q + 7])
        obs[10:13] = R.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])
        n = len(self.names)
        obs[13:13 + n] = (d.qpos[self.q] - self.padded) * self.dof_pos_scale
        obs[13 + n:13 + 2 * n] = d.qvel[self.v] * self.dof_vel_scale
        obs[13 + 2 * n:13 + 2 * n + self.n_act] = self.action
        self.history.append(obs)
        x = np.concatenate(list(self.history))[None, :].astype(np.float32)
        policy = self.balance if np.linalg.norm(cmd) <= 0.05 else self.walk
        self.action = policy.run(None, {policy.get_inputs()[0].name: x})[0][0].astype(float)
        self.target = self.action * self.action_scale + self.default
    tau = self.kp * (self.target - d.qpos[self.q[:self.n_act]]) - self.kd * d.qvel[self.v[:self.n_act]]
    for a, t in zip(self.own_act, tau):
        lo, hi = self.m.actuator_ctrlrange[a]
        d.ctrl[a] = min(max(t, lo), hi) if self.m.actuator_ctrllimited[a] else t
    self.k += 1


def old_control(m, d, groups, walker):   # robot.py control() as it was
    for g in groups:
        if g["kind"] == "servo":
            for k, i in enumerate(g["ids"]):
                d.ctrl[i] = bodies.servo_command(m, d, g, i, g["ref"][k])
        elif g["kind"] == "pd":
            kp, kd = np.broadcast_to(g["kp"], g["n"]), np.broadcast_to(g["kd"], g["n"])
            for k, i in enumerate(g["ids"]):
                u = kp[k] * (g["ref"][k] - d.qpos[g["qadr"][k]]) - kd[k] * d.qvel[g["vadr"][k]]
                d.ctrl[i] = min(max(u, m.actuator_ctrlrange[i, 0]), m.actuator_ctrlrange[i, 1])
        elif g["kind"] == "velocity":
            for k, i in enumerate(g["ids"]):
                u = g["gain"] * (g["ref"][k] - d.actuator_velocity[i])
                d.ctrl[i] = min(max(u, m.actuator_ctrlrange[i, 0]), m.actuator_ctrlrange[i, 1])
        elif g["kind"] == "walk":
            walker.speed = g["ref"]
    if walker:
        if isinstance(walker, legged.GearWbc):
            old_gearwbc_control(walker)
        else:
            walker.control()


def new_control(m, d, groups, walker, bank):   # robot.py control() now
    if bank.ids:
        bank.command(d)
    for g in groups:
        if g["kind"] == "pd":
            u = g["kp_n"] * (g["ref"] - d.qpos[g["qadr"]]) - g["kd_n"] * d.qvel[g["vadr"]]
            d.ctrl[g["ids"]] = np.minimum(np.maximum(u, g["lo"]), g["hi"])
        elif g["kind"] == "velocity":
            u = g["gain"] * (g["ref"] - d.actuator_velocity[g["ids"]])
            d.ctrl[g["ids"]] = np.minimum(np.maximum(u, g["lo"]), g["hi"])
        elif g["kind"] == "walk":
            walker.speed = g["ref"]
    if walker:
        walker.control()


def bits(a):
    return np.ascontiguousarray(a, dtype=np.float64).view(np.uint64)


fails = 0
for kind in KINDS:
    try:
        m1, d1, w1, g1 = setup(kind, 5)
        m2, d2, w2, g2 = setup(kind, 5)
        bank = bodies.ServoBank(m2, [g for g in g2 if g["kind"] == "servo"])
    except SystemExit as e:
        print("%-8s not built here: %s" % (kind, e))
        continue
    substeps = max(1, int(round(BEAT / m1.opt.timestep)))
    asks = np.random.default_rng(17)
    t_old = t_new = t_step = 0.0
    first = None
    for beat in range(1, BEATS + 1):
        if beat % 10 == 1:   # every group asked somewhere new, the same in both worlds
            for a, b in zip(g1, g2):
                if a["kind"] in ("servo", "pd"):
                    a["ref"] = a["ref"] + asks.normal(0.0, 0.05, a["n"])
                    b["ref"] = a["ref"].copy()
                elif a["kind"] == "walk":
                    a["ref"] = asks.normal(0.0, 0.2, a["n"])
                    b["ref"] = a["ref"].copy()
        for _ in range(substeps):
            t0 = time.perf_counter()
            old_control(m1, d1, g1, w1)
            t1 = time.perf_counter()
            new_control(m2, d2, g2, w2, bank)
            t2 = time.perf_counter()
            mujoco.mj_step(m1, d1)
            t3 = time.perf_counter()
            mujoco.mj_step(m2, d2)
            t_old += t1 - t0
            t_new += t2 - t1
            t_step += t3 - t2
        if first is None and not (np.array_equal(bits(d1.qpos), bits(d2.qpos)) and np.array_equal(bits(d1.qvel), bits(d2.qvel))
                                  and np.array_equal(bits(d1.ctrl), bits(d2.ctrl))):
            first = beat
    ok = first is None
    fails += 0 if ok else 1
    print("%-8s %s over %d beats of %d steps; a beat: loops joint by joint %.2f ms, a group at once %.2f ms, mj_step %.2f ms"
          % (kind, "the same to the bit" if ok else "DIFFERENT from beat %d" % first, BEATS, substeps,
             1000 * t_old / BEATS, 1000 * t_new / BEATS, 1000 * t_step / BEATS), flush=True)
print("controltest: %d kinds differ" % fails)
