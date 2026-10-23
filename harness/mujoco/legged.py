# legged.py: simulator-side rig code. A legged robot's own walking, as its maker ships it: a velocity policy (Unitree's
# own, or one trained with Unitree's recipe: its exported ONNX and a deploy.yaml) run the way Unitree's deploy program
# runs it (unitree_rl_mjlab deploy/include/isaaclab/envs/mdp/observations/observations.h, the robots' State_RLBase):
# every policy step (deploy.yaml step_dt) the observation of its training (the root's turn rate and gravity in the
# root's frame, the speed asked, the gait clock's sine and cosine, each joint's angle and speed from its default, the
# last action), the action scaled and offset into the targets of the position servos of the joints it drives
# (deploy.yaml's joint_ids, or all of them) that are the firmware's. The gait clock runs always; while the speed asked
# is under 0.1 (its norm, as the deploy program and the training's phase() measure it) the clock is shown to the policy
# as zero and the robot stands still, as it learned to (5% of its training's commands were standing). The speed asked
# is held to the ranges the deploy program gives (deploy.yaml commands). The robot has fallen when it leans more than
# mjlab's training's fell_over (bad_orientation) allows. The Go2 walks on such a policy (trained here with Unitree's
# recipe: Unitree published the recipe, not the policy); the G1 on NVIDIA's controller (GearWbc, below): Unitree's own
# G1 policy, trained moving its own arms and waist, fell whenever a user moved its arms beyond about a third of their
# ranges (one arm at 60% of its range: 2 falls in 3; NVIDIA's: none at full range, 10-23).
import math
import numpy as np
import mujoco
import onnxruntime as ort
import yaml

STAND = 0.1                     # the deploy program's and the training's standing threshold on the speed asked
FELL = 1.2217304763960306       # the training's fell_over: bad_orientation's limit_angle, 70 degrees


class Walker:
    def __init__(self, m, d, cfg):
        """CFG: the policy's folder (exported/policy.onnx and params/deploy.yaml, as Unitree's deploy program reads
        them), the model's prefix for the robot's names, its root body and its IMU site."""
        self.m, self.d = m, d
        p = yaml.safe_load(open(cfg["policy"] + "/params/deploy.yaml"))
        self.session = ort.InferenceSession(cfg["policy"] + "/exported/policy.onnx", providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0].name
        # The joints in the policy's order: the training's natural order, as the exported policy names them.
        order = self.session.get_modelmeta().custom_metadata_map["joint_names"].split(",")
        self.names = [cfg["prefix"] + n for n in order]
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.names]
        if min(jid) < 0:
            raise SystemExit("legged: the model has no joint among " + ", ".join(self.names))
        self.q = np.array([m.jnt_qposadr[j] for j in jid])
        self.v = np.array([m.jnt_dofadr[j] for j in jid])
        self.default = np.array(p["default_joint_pos"], float)
        act = p["actions"]["JointPositionAction"]
        # The joints its actions drive (deploy.yaml joint_ids; none named: all of them), each through its servo.
        acts = list(act.get("joint_ids") or range(len(order)))
        self.scale = np.array(act["scale"], float)
        self.offset = np.array(act["offset"], float)
        self.own = list(range(len(acts)))
        self.own_act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, self.names[k]) for k in acts]
        if min(self.own_act) < 0:
            raise SystemExit("legged: a joint the firmware drives has no servo")
        self.legs = acts
        r = p["commands"]["base_velocity"]["ranges"]
        self.low = np.array([r["lin_vel_x"][0], r["lin_vel_y"][0], r["ang_vel_z"][0]], float)
        self.high = np.array([r["lin_vel_x"][1], r["lin_vel_y"][1], r["ang_vel_z"][1]], float)
        self.period = float(p["observations"]["gait_phase"]["params"]["period"])
        self.every = max(1, int(round(float(p["step_dt"]) / m.opt.timestep)))
        self.step_dt = self.every * m.opt.timestep
        root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, cfg["root"])
        self.root_q = m.jnt_qposadr[m.body_jntadr[root]]
        self.root_v = m.jnt_dofadr[m.body_jntadr[root]]
        self.imu = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, cfg["imu"])
        self.action = np.zeros(len(acts))
        self.phase = 0.0
        self.k = 0
        self.speed = np.zeros(3)   # forward, left, turning: what the user asked

    def pose(self, height):
        """Every joint at the policy's default, the root at its training's starting height."""
        self.d.qpos[self.root_q + 2] = height
        for k, a in enumerate(self.q):
            self.d.qpos[a] = self.default[k]

    def asked(self):
        return np.clip(np.asarray(self.speed, float), self.low, self.high)

    def control(self):
        """Every step of the model: the policy every policy step, its targets to the legs' servos."""
        d = self.d
        if self.k % self.every == 0:
            cmd = self.asked()
            self.phase = math.fmod(self.phase + self.step_dt / self.period, 1.0)
            clock = [math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)]
            if math.sqrt(float(cmd @ cmd)) < STAND:
                clock = [0.0, 0.0]
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, d.qpos[self.root_q + 3: self.root_q + 7])
            gravity = R.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])
            obs = np.concatenate([d.qvel[self.root_v + 3: self.root_v + 6], gravity, cmd, clock,
                                  d.qpos[self.q] - self.default, d.qvel[self.v], self.action]).astype(np.float32)[None, :]
            self.action = self.session.run(None, {self.input: obs})[0][0].astype(float)
            target = self.action * self.scale + self.offset
            for i, a in zip(self.own, self.own_act):
                d.ctrl[a] = target[i]
        self.k += 1

    def odometry(self):
        """The robot's odometry as Unitree's own simulator gives it (unitree_mujoco: the IMU site's linear velocity in
        the world, the odometry's frame) and its turn rate: velocity forward and left in that frame, turning."""
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_SITE, self.imu, res, 0)
        return np.array([res[3], res[4], res[2]])

    def lean(self):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.d.qpos[self.root_q + 3: self.root_q + 7])
        return math.acos(max(-1.0, min(1.0, R[8])))

    def fallen(self):
        return self.lean() > FELL


class GearWbc:
    """NVIDIA's decoupled whole-body controller for the G1 (GR00T-WholeBodyControl, decoupled_wbc: the controller under
    its GR00T N1.5 and N1.6 models on the real G1), run as its own MuJoCo script runs it
    (sim2mujoco/scripts/run_mujoco_gear_wbc.py, with sim2mujoco/resources/robots/g1/g1_gear_wbc.yaml, whose observation
    is the one its two released policies take: 86 numbers, six frames of history): every policy step the speed asked
    (scaled), the base height and torso angles asked, the pelvis's turn rate (scaled) and gravity in its frame, the 29
    body joints' angles from their defaults and speeds (scaled), the last action; the balance policy while the speed
    asked is at most 0.05 (its norm), the walking policy otherwise; the action, scaled and offset, the targets of the
    legs' and the waist's fifteen joints, held by its PD (its kps and kds) through torque motors at the motors' limits.
    The arms are the user's. The speeds asked are held to its navigation's limits (keyboard_navigation_policy.py: 0.5 m/s
    forward and sideways, 0.5 rad/s turning)."""

    def __init__(self, m, d, cfg):
        import collections
        import xml.etree.ElementTree as ET
        self.m, self.d = m, d
        base = cfg["dir"]
        c = yaml.safe_load(open(base + "/g1_gear_wbc.yaml"))
        self.kp = np.array(c["kps"], float)
        self.kd = np.array(c["kds"], float)
        self.default = np.array(c["default_angles"], float)
        self.n_act = int(c["num_actions"])
        self.every = int(c["control_decimation"])
        self.ang_vel_scale, self.dof_pos_scale = float(c["ang_vel_scale"]), float(c["dof_pos_scale"])
        self.dof_vel_scale, self.action_scale = float(c["dof_vel_scale"]), float(c["action_scale"])
        self.cmd_scale = np.array(c["cmd_scale"], float)
        self.height_cmd = float(c["height_cmd"])
        self.rpy_cmd = np.array(c["rpy_cmd"], float)
        self.limit = np.array([0.5, 0.5, 0.5])
        # The 29 body joints in its model's order (its g1_gear_wbc.xml), found here by name.
        names = [j.get("name") for j in ET.parse(base + "/" + c["xml_path"]).getroot().iter("joint")
                 if j.get("name") and j.get("type") != "free"]
        self.names = [cfg["prefix"] + n for n in names]
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.names]
        if min(jid) < 0:
            raise SystemExit("legged: the model has no joint among " + ", ".join(self.names))
        self.q = np.array([m.jnt_qposadr[j] for j in jid])
        self.v = np.array([m.jnt_dofadr[j] for j in jid])
        self.own_act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self.names[:self.n_act]]
        if min(self.own_act) < 0:
            raise SystemExit("legged: a joint the controller drives has no motor")
        self.legs = list(range(self.n_act))   # the joints it drives (the legs and the waist)
        self.padded = np.zeros(len(self.names))
        self.padded[:self.n_act] = self.default
        sessions = [ort.InferenceSession(base + "/policy/GR00T-WholeBodyControl-%s.onnx" % w,
                                         providers=["CPUExecutionProvider"]) for w in ("Balance", "Walk")]
        self.balance, self.walk = sessions
        self.history = collections.deque([np.zeros(86, np.float32)] * int(c["obs_history_len"]),
                                         maxlen=int(c["obs_history_len"]))
        root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, cfg["root"])
        self.root_q = m.jnt_qposadr[m.body_jntadr[root]]
        self.root_v = m.jnt_dofadr[m.body_jntadr[root]]
        self.imu = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, cfg["imu"])
        self.action = np.zeros(self.n_act)
        self.target = self.default.copy()
        self.k = 0
        self.speed = np.zeros(3)

    def pose(self, height):
        """The legs and the waist at its defaults, the arms straight down (its arms' zeros), the root at HEIGHT."""
        self.d.qpos[self.root_q + 2] = height
        for k, a in enumerate(self.q):
            self.d.qpos[a] = self.padded[k]

    def asked(self):
        return np.clip(np.asarray(self.speed, float), -self.limit, self.limit)

    def control(self):
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

    odometry = Walker.odometry
    lean = Walker.lean
    fallen = Walker.fallen


def firmware(m, d, cfg):
    """The legged robot's firmware its body names: Unitree's own policies (Walker) or NVIDIA's controller (GearWbc)."""
    return GearWbc(m, d, cfg) if cfg.get("kind") == "gear_wbc" else Walker(m, d, cfg)
