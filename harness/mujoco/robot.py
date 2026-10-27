# robot.py --connect HOST:PORT --kind KIND --seed N --out DIR [--beats N] [--stop FILE] [--rate R] [--beat S]
#          [--width W --height H] [--instruction TEXT] [--truth FILE]
#
# Simulator-side rig code (owner 10-02: the only Python we write is the simulator's side). One real robot of the
# MuJoCo Menagerie (bodies.py), spoken to the driver over the body protocol (docs/body-protocol.md) as the lab's
# simulator speaks it: hello, reset, then every beat one call carrying the observation (update_obs) and one asking for
# the action (get_action), which moves the robot for the next beat. Nothing of the robot reaches the driver but
# readings and pictures, and those scrambled as the lab's scrambler scrambles them (harness/robodojo_scramble): every
# key renamed but the top-level instruction, every map in a drawn order, every group's channels in a drawn order with
# a drawn sign, scale and zero; a group read in whole ticks stays whole. DIR/scramble.json holds the draw and
# DIR/truth.json each group's role and each camera's mount, for the scorer; the driver sees neither.
#
# A beat is BEAT seconds of the world (0.1 unless given); the robot's own loops (a torque-motored joint's position
# loop, a wheeled base's and a drone's velocity loops, a legged robot's walking program: legged.py) run at the model's
# step inside it. A legged robot is booted standing on its walking program, settled before the driver's first beat.
# RATE gives beats a second of the wall (0: as fast as the driver answers). The run ends after BEATS beats or when the
# STOP file exists. With --truth FILE, every observation sent writes the world's truth beside it (truth.py), for the
# scorer of steps 2 to 4 (driver/tools/score).
import argparse
import collections
import io
import json
import math
import os
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco
import msgpack
import websocket
from PIL import Image
# The pictures' sensor noise and stream codec made on a GPU where torch has one, the same noise and the same codec's
# quality: on the big machine (10-28) a 320x240 picture took 4.8 ms of noise and 1.6 of JPEG on the robot's CPU, and 60
# robots at once spent 18 to 82 ms a beat on their pictures, beside the drivers' fits on the same cores.
try:
    import torch
    import torchvision
except ImportError:
    torch = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodies
import legged
import truth as truth_file

ap = argparse.ArgumentParser()
ap.add_argument("--connect", required=True)
ap.add_argument("--kind", required=True, choices=sorted(bodies.KINDS))
ap.add_argument("--seed", type=int, required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--beats", type=int, default=3000000)
ap.add_argument("--stop", default="")
ap.add_argument("--rate", type=float, default=0.0)
ap.add_argument("--beat", type=float, default=0.1)
ap.add_argument("--width", type=int, default=320)
ap.add_argument("--height", type=int, default=240)
ap.add_argument("--instruction", default="Pick up the thing by 10 cm.")
ap.add_argument("--truth", default="")
ap.add_argument("--show-refs", type=int, default=0)   # every so many beats, a moving body's commands and root (a diagnostic)
ap.add_argument("--snap", default="")   # beats (comma-separated) at which every camera's picture is saved as it reaches the
                                       # driver, side by side, in OUT/snap_BEAT.png (a diagnostic)
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)
rng = np.random.default_rng(args.seed)

# ---------------------------------------------------------------------------------------------------------------------
# The robot

built, m, d, walker = bodies.build(args.kind, rng)
substeps = max(1, int(round(args.beat / m.opt.timestep)))

groups = built["groups"]
for g in groups:
    g["ids"] = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in g["actuators"]]
    if g["ids"] and min(g["ids"]) < 0:
        raise SystemExit("robot: %s has no actuator among %s" % (args.kind, g["actuators"]))
    g["n"] = len(g["ids"]) if g["ids"] else g["channels"]   # a walking program's speeds are no actuator's
    if g["kind"] in ("pd", "sensor"):
        g["qadr"] = [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in g["ids"]]
        g["vadr"] = [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in g["ids"]]
    g["hinge"] = [m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT
                  and m.jnt_type[m.actuator_trnid[i, 0]] == mujoco.mjtJoint.mjJNT_HINGE for i in g["ids"]] or [False] * g["n"]

# Each sensor as the real one is (bodies.SENSORS): drawn once from the seed in a stream of its own, and the noise of every
# beat from another, so the robot, its cameras and its scramble are the same robot whatever its sensors drew.
bodies.draw_sensors(args.kind, built, np.random.default_rng([args.seed, 7]))
noise_rng = np.random.default_rng([args.seed, 11])
for g in groups:
    s = g["sensor"]
    steps = []
    for k in range(g["n"]):
        if "step" in s:
            steps.append(s["step"])                                     # read in its command's own units
        elif "estimate" in s:
            steps.append(0.0)                                           # an estimate, not a count
        elif s.get("velocity"):
            steps.append(2.0 * math.pi / 2 ** s["bits"] / args.beat)    # counts over a beat
        elif g["hinge"][k]:
            steps.append(2.0 * math.pi / 2 ** s["bits"])                # a turn's counts
        else:
            steps.append(s.get("slide_step", 1.0e-5))                   # a slide's or a tendon's meters a count
    g["steps"] = np.array(steps)
    g["wander"] = np.zeros(g["n"])


def true_reading(g):
    """The group's reading in its command's own units: where a servo stands as the target that would hold it there,
    a torque-motored joint's angle, a velocity loop's velocity, a drone's velocity in its heading's frame."""
    if g["kind"] == "servo":
        out = []
        for i in g["ids"]:
            g0, b0, b1, b2 = (m.actuator_gainprm[i, 0], m.actuator_biasprm[i, 0], m.actuator_biasprm[i, 1],
                              m.actuator_biasprm[i, 2])
            # A position servo stands at the target its spring would hold it at; a velocity servo (no spring, a
            # damper) runs at the velocity its damper would hold it at.
            out.append(-(b0 + b1 * d.actuator_length[i]) / g0 if b1 != 0.0 else -(b0 + b2 * d.actuator_velocity[i]) / g0)
        return np.array(out)
    if g["kind"] in ("pd", "sensor"):
        return np.array([d.qpos[a] for a in g["qadr"]])
    if g["kind"] == "walk":
        return walker.odometry()
    if g["kind"] == "velocity":
        return np.array([d.actuator_velocity[i] for i in g["ids"]])
    if g["kind"] == "flight":
        return flight_reading()
    raise ValueError(g["kind"])


def sensed_reading(g):
    """The reading as its sensor gives it: a count's jitter, then rounded to the encoder's step; a flight controller's
    velocity estimate wandering about the truth over its correlation time."""
    x = true_reading(g)
    s = g["sensor"]
    if "estimate" in s:
        a = math.exp(-args.beat / s["estimate_time"])
        g["wander"] = (a * g["wander"] + math.sqrt(1.0 - a * a) * np.asarray(s["estimate"], float)
                       * noise_rng.standard_normal(x.size))
        return x + g["wander"]
    st = g["steps"]
    return np.round((x + s["jitter"] * st * noise_rng.standard_normal(x.size)) / st) * st


# The drone's flight controller: velocity setpoints (forward, left, up in its heading's frame, and a turn rate) to the
# four rotors' thrusts, as its own board does (a geometric attitude loop under a velocity loop).
drone = None
for g in groups:
    if g["kind"] == "flight":
        root = m.actuator_trnid[g["ids"][0], 0]
        body_id = m.site_bodyid[root]
        mass = float(m.body_subtreemass[body_id])
        inertia = np.array(m.body_inertia[body_id])
        alloc = np.zeros((4, 4))
        mujoco.mj_forward(m, d)
        com = d.xipos[body_id]
        for k, i in enumerate(g["ids"]):
            s = m.actuator_trnid[i, 0]
            r = d.site_xpos[s] - com
            alloc[0, k] = m.actuator_gear[i, 2]
            alloc[1, k] = r[1] * m.actuator_gear[i, 2]
            alloc[2, k] = -r[0] * m.actuator_gear[i, 2]
            alloc[3, k] = m.actuator_gear[i, 5]
        qadr = m.jnt_qposadr[m.body_jntadr[body_id]]
        vadr = m.jnt_dofadr[m.body_jntadr[body_id]]
        drone = {"g": g, "body": body_id, "mass": mass, "J": inertia, "inv": np.linalg.inv(alloc), "qadr": qadr,
                 "vadr": vadr, "yaw_d": None}


def drone_state():
    q = d.qpos[drone["qadr"] + 3: drone["qadr"] + 7]
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    R = R.reshape(3, 3)
    v = np.array(d.qvel[drone["vadr"]: drone["vadr"] + 3])
    w = np.array(d.qvel[drone["vadr"] + 3: drone["vadr"] + 6])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return R, v, w, yaw


def flight_reading():
    R, v, w, yaw = drone_state()
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2], (R @ w)[2]])


def flight_control(ref):
    R, v, w, yaw = drone_state()
    if drone["yaw_d"] is None:
        drone["yaw_d"] = yaw
    drone["yaw_d"] += ref[3] * m.opt.timestep
    c, s = math.cos(yaw), math.sin(yaw)
    v_ref = np.array([c * ref[0] - s * ref[1], s * ref[0] + c * ref[1], ref[2]])
    a = 2.0 * (v_ref - v)
    h = np.linalg.norm(a[:2])
    if h > 5.0:
        a[:2] *= 5.0 / h
    a[2] += 9.81
    b3 = a / np.linalg.norm(a)
    thrust = drone["mass"] * float(a @ R[:, 2])
    b1c = np.array([math.cos(drone["yaw_d"]), math.sin(drone["yaw_d"]), 0.0])
    b2 = np.cross(b3, b1c)
    b2 /= np.linalg.norm(b2)
    Rd = np.column_stack([np.cross(b2, b3), b2, b3])
    E = 0.5 * (Rd.T @ R - R.T @ Rd)
    e_r = np.array([E[2, 1], E[0, 2], E[1, 0]])
    e_w = w - np.array([0.0, 0.0, ref[3]])
    tau = drone["J"] * (-60.0 * e_r - 12.0 * e_w)
    f = drone["inv"] @ np.array([thrust, tau[0], tau[1], tau[2]])
    for k, i in enumerate(drone["g"]["ids"]):
        d.ctrl[i] = min(max(f[k], m.actuator_ctrlrange[i, 0]), m.actuator_ctrlrange[i, 1])


for g in groups:
    g["ref"] = true_reading(g).astype(float)   # every group holds where it stands until it is sent elsewhere
    if g["kind"] in ("flight", "walk"):
        g["ref"] = np.zeros(g["n"])           # no speed asked


def control():
    """The robot's own loops, every step of the model."""
    for g in groups:
        if g["kind"] == "servo":
            for k, i in enumerate(g["ids"]):
                d.ctrl[i] = bodies.servo_command(m, d, g, i, g["ref"][k])
        elif g["kind"] == "pd":
            kp, kd = np.broadcast_to(g["kp"], g["n"]), np.broadcast_to(g["kd"], g["n"])   # one gain, or one a joint
            for k, i in enumerate(g["ids"]):
                u = kp[k] * (g["ref"][k] - d.qpos[g["qadr"][k]]) - kd[k] * d.qvel[g["vadr"][k]]
                d.ctrl[i] = min(max(u, m.actuator_ctrlrange[i, 0]), m.actuator_ctrlrange[i, 1])
        elif g["kind"] == "velocity":
            for k, i in enumerate(g["ids"]):
                u = g["gain"] * (g["ref"][k] - d.actuator_velocity[i])
                d.ctrl[i] = min(max(u, m.actuator_ctrlrange[i, 0]), m.actuator_ctrlrange[i, 1])
        elif g["kind"] == "flight":
            flight_control(g["ref"])
        elif g["kind"] == "walk":
            walker.speed = g["ref"]
    if walker:
        walker.control()


if walker:
    # Booted standing: the walking program holds it up, nothing asked, every other joint held where it was booted, until
    # its body stands still: over a whole beat its root moved and turned less than its own odometry could tell (its
    # errors over a beat), before the driver's first beat. Over a beat, not at an instant: the walking program corrects
    # fifty times a second, and against its instantaneous turn rate the G1 took 195 s of the world to count as still
    # (10-23). A robot that cannot stand is no robot to test a driver on.
    odometer = next(g for g in groups if g["kind"] == "walk")["sensor"]
    t_settle = 0.0
    while True:
        at = d.qpos[walker.root_q: walker.root_q + 3].copy()
        facing = d.qpos[walker.root_q + 3: walker.root_q + 7].copy()
        for _ in range(substeps):
            control()
            mujoco.mj_step(m, d)
            t_settle += m.opt.timestep
            if walker.fallen() or t_settle > 600.0:
                raise SystemExit("robot: %s did not stand on its walking program (%.1f s, leaning %.1f deg)"
                                 % (args.kind, t_settle, math.degrees(walker.lean())))
        back, turned = np.zeros(4), np.zeros(4)
        mujoco.mju_negQuat(back, facing)
        mujoco.mju_mulQuat(turned, d.qpos[walker.root_q + 3: walker.root_q + 7], back)
        moved = np.linalg.norm(d.qpos[walker.root_q: walker.root_q + 2] - at[:2]) / args.beat
        turn = 2.0 * math.acos(min(1.0, abs(turned[0]))) / args.beat
        if moved < min(odometer["estimate"][:2]) and turn < odometer["estimate"][2]:
            break
    print("robot: %s stands on its walking program after %.2f s of the world" % (args.kind, t_settle), flush=True)


# ---------------------------------------------------------------------------------------------------------------------
# What the driver is given: a reading in ticks for a servo read in whole ticks, the lab's scramble over all of it.

def to_wire_units(g, x):
    if g["ticks"]:
        return np.array([int(round(v * g["ticks"] / (2.0 * math.pi))) if h else int(round(v * g["ticks"]))
                         for v, h in zip(x, g["hinge"])], dtype=np.int64)
    return np.asarray(x, dtype=np.float64)


def from_wire_units(g, y):
    if g["ticks"]:
        return np.array([v * 2.0 * math.pi / g["ticks"] if h else v / g["ticks"] for v, h in zip(y, g["hinge"])])
    return np.asarray(y, dtype=np.float64)


names, back, orders, draws = {}, {}, {}, {}
integer_groups = {g["name"] for g in groups if g["ticks"]} | {"env_idx"}


def drawn(key):
    if key not in names:
        while True:
            k = "k%08x" % int(rng.integers(0, 2 ** 32))
            if k not in back:
                break
        names[key] = k
        back[k] = key
    return names[key]


def draw_of(key, n):
    if key not in draws:
        whole = key in integer_groups
        draws[key] = {"order": [int(i) for i in rng.permutation(n)],
                      "sign": [float(s) for s in rng.choice([-1.0, 1.0], n)],
                      # Powers of two (a scale changes only a float's exponent); a whole tick count stays whole.
                      "scale": [float(2.0 ** int(e)) for e in rng.integers(0 if whole else -3, 3 if whole else 4, n)],
                      "zero": [float(int(z)) for z in rng.integers(-2048, 2049, n)] if whole
                      else [float(z) for z in rng.uniform(-math.pi, math.pi, n)]}
    return draws[key]


def scramble_group(key, x):
    a = np.atleast_1d(np.asarray(x))
    g = draw_of(key, a.size)
    y = np.asarray(g["sign"]) * np.asarray(g["scale"]) * a[np.asarray(g["order"])] + np.asarray(g["zero"])
    return y.astype(np.int64) if a.dtype.kind in "iu" else y


def unscramble_group(key, y):
    g = draws[key]
    y = np.asarray(y, dtype=np.float64).ravel()
    x = np.empty(y.size)
    x[np.asarray(g["order"])] = (y - np.asarray(g["zero"])) / (np.asarray(g["sign"]) * np.asarray(g["scale"]))
    return x


def scramble_map(obj, path=()):
    if path not in orders:
        keys = list(obj.keys())
        orders[path] = [keys[i] for i in rng.permutation(len(keys))]
    out = {}
    for key in orders[path] + [k for k in obj if k not in orders[path]]:
        if key not in obj:
            continue
        v = obj[key]
        if path == () and key == "instruction":
            out[key] = v
        elif isinstance(v, dict):
            out[drawn(key)] = scramble_map(v, path + (key,))
        elif isinstance(v, np.ndarray) and v.ndim == 1:
            out[drawn(key)] = scramble_group(key, v)
        elif isinstance(v, (int, float)) and not isinstance(v, bool):
            out[drawn(key)] = scramble_group(key, np.array([v]))
        else:
            out[drawn(key)] = v
    return out


def packable(obj):
    if isinstance(obj, dict):
        return {k: packable(v) for k, v in obj.items()}
    if isinstance(obj, np.ndarray):
        a = np.ascontiguousarray(obj)
        if a.ndim == 1 and a.dtype.kind in "iu":
            a = a.astype("<i8")
        elif a.ndim == 1 and a.dtype.kind == "f":
            a = a.astype("<f8")
        return {"nd": True, "type": a.dtype.str, "shape": list(a.shape), "data": a.tobytes()}
    return obj


# ---------------------------------------------------------------------------------------------------------------------
# Pictures

renderer = mujoco.Renderer(m, args.height, args.width)
cameras = built["cameras"]
for c in cameras:
    c["id"] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, c["name"])
    if c["id"] < 0:
        raise SystemExit("robot: %s has no camera %s" % (args.kind, c["name"]))


# Each camera as the real one is: its sensor's noise (shot noise growing with the level, as the square root of the
# electrons it counts, and read noise), then its stream's compression when it has one, and its latency: the frame it
# gives at a beat was taken that long before, at the step of the world it falls in, beats before when it is longer.
for c in cameras:
    sn = c["sensor"]
    c["K"] = 255.0 / sn["full_well"]                 # levels an electron, the full well at full scale
    c["read_levels"] = sn["read_noise"] * c["K"]
    c["lag_beats"] = int(sn["latency"] // args.beat)
    rest = sn["latency"] - c["lag_beats"] * args.beat
    c["at_step"] = max(0, min(substeps, substeps - int(round(rest / m.opt.timestep))))
    c["queue"] = collections.deque(maxlen=c["lag_beats"] + 1)
capture_at = {}
for c in cameras:
    capture_at.setdefault(c["at_step"], []).append(c)
t_capture = [0.0]


# Only where asked (BD_GPU_PICTURES=1): one CUDA context a robot, 30 robots a GPU beside their renderers, spent the
# GPUs time-slicing (round f2s2 on the big machine, 10-28: the GPUs 84 and 94 % busy, pictures 28 to 64 ms a beat
# against 16 to 82 on the CPU), so a GPU serves the pictures only once one process or the GPU's own sharing does it
# for every robot at once.
GPU = None
if os.environ.get("BD_GPU_PICTURES") == "1" and torch is not None and torch.cuda.is_available():
    GPU = torch.device("cuda:%d" % (int(os.environ.get("MUJOCO_EGL_DEVICE_ID", "0")) % torch.cuda.device_count()))
    gpu_rng = torch.Generator(device=GPU)
    gpu_rng.manual_seed(args.seed * 1000003 + 11)


def capture(c):
    t = time.perf_counter()
    renderer.update_scene(d, camera=c["id"])
    # BD_NO_JPEG=1: a diagnostic, never a hall's world: the pictures as the sensor makes them, no codec's loss (what of a
    # fit's error the compression makes, 10-27: step 3's flows against the rigid-point model on SO-101's base camera).
    coded = c["sensor"]["jpeg"] and not os.environ.get("BD_NO_JPEG")
    if GPU is not None:
        x = torch.from_numpy(renderer.render()).to(GPU).float()
        x += torch.sqrt(c["K"] * x + c["read_levels"] ** 2) * torch.randn(x.shape, device=GPU, generator=gpu_rng)
        x = torch.clamp(torch.round(x), 0, 255).to(torch.uint8)
        if coded:
            enc = torchvision.io.encode_jpeg(x.permute(2, 0, 1).contiguous(), quality=int(c["sensor"]["jpeg"]))
            x = torchvision.io.decode_jpeg(enc.cpu() if enc.is_cuda else enc, device=GPU).permute(1, 2, 0)
        img = x.contiguous().cpu().numpy()
    else:
        img = renderer.render().astype(np.float32)
        img += np.sqrt(c["K"] * img + c["read_levels"] ** 2) * noise_rng.standard_normal(img.shape, dtype=np.float32)
        img = np.clip(np.rint(img), 0, 255).astype(np.uint8)
        if coded:
            buf = io.BytesIO()
            Image.fromarray(img).save(buf, format="JPEG", quality=c["sensor"]["jpeg"])
            img = np.asarray(Image.open(io.BytesIO(buf.getvalue())).convert("RGB")).copy()
    c["queue"].append(img)
    t_capture[0] += time.perf_counter() - t


def pictures():
    return {c["name"]: {"color": c["queue"][0]} for c in cameras}


# ---------------------------------------------------------------------------------------------------------------------
# The wire

host, port = args.connect.rsplit(":", 1)
ws = None
for _ in range(240):
    try:
        ws = websocket.create_connection("ws://%s:%s/" % (host, port), timeout=600)
        break
    except Exception:
        time.sleep(0.5)
if ws is None:
    raise SystemExit("robot: no driver at " + args.connect)


def exchange(msg):
    ws.send(msgpack.packb(packable(msg), use_bin_type=True), opcode=websocket.ABNF.OPCODE_BINARY)
    raw = ws.recv()
    if isinstance(raw, str):
        raw = raw.encode("latin-1")
    return msgpack.unpackb(raw, raw=False, strict_map_key=False)


def call(func, beat, obs=None):
    payload = {"func_name": func}
    if obs is not None:
        payload["obs"] = obs
    return exchange({"message_type": "call", "step": beat, "payload": payload})


def observation():
    state = {g["name"]: to_wire_units(g, sensed_reading(g)) for g in groups}
    # A group read but never commanded (a legged robot's legs under its walking program) echoes no command.
    echo = {g["name"]: to_wire_units(g, g["ref"]) for g in groups if g["kind"] != "sensor"}
    return {"vision": pictures(), "state": state, "action": echo, "data_format_version": "v1.0",
            "additional_info": {"frequency": float(1.0 / args.beat), "env_idx": 0}, "instruction": args.instruction}


def take_action(reply):
    result = (reply.get("payload") or {}).get("result") or []
    if not result or not isinstance(result[0], dict):
        return
    for k, v in result[0].items():
        key = back.get(k)
        target = next((g for g in groups if g["name"] == key and g["kind"] != "sensor"), None)
        if target is None or key not in draws:
            continue
        if isinstance(v, dict) and v.get("nd"):
            v = np.frombuffer(v["data"], dtype=np.dtype(v["type"])).reshape(v["shape"])
        y = unscramble_group(key, v)
        if y.size == target["n"]:
            target["ref"] = from_wire_units(target, y)


exchange({"message_type": "hello", "payload": {}})
exchange({"message_type": "reset", "payload": {}})
print("robot: %s seed %d, %d groups, %d cameras of %d x %d, a beat %.3f s of %d steps"
      % (args.kind, args.seed, len(groups), len(cameras), args.width, args.height, args.beat, substeps), flush=True)

snaps = set(int(b) for b in args.snap.split(",") if b.strip())
beat, t_step, t_render, t_wire = 0, 0.0, 0.0, 0.0
fell = [0]
truth = truth_file.Truth(m, d, args.truth, cameras) if args.truth else None
next_at = time.time()
while True:
    beat += 1
    t0 = time.perf_counter()
    for s in range(substeps):
        for c in capture_at.get(s, ()):
            capture(c)
        control()
        mujoco.mj_step(m, d)
    for c in capture_at.get(substeps, ()):
        capture(c)
    t1 = time.perf_counter()
    obs = observation()
    if truth:
        truth.write(beat, obs["state"], args.width, args.height)
    t2 = time.perf_counter()
    scrambled = scramble_map(obs)
    call("update_obs", beat, scrambled)
    if beat == 1:
        with open(os.path.join(args.out, "scramble.json"), "w") as f:
            json.dump({"seed": args.seed, "names": names, "groups": draws}, f)
        with open(os.path.join(args.out, "truth.json"), "w") as f:
            json.dump({"kind": args.kind, "seed": args.seed,
                       "groups": [{"name": g["name"], "role": g["role"], "channels": g["n"],
                                   "wire": names[g["name"]], "actuators": g["actuators"], "sensor": g["sensor"],
                                   "steps": g["steps"].tolist()} for g in groups]
                       + [{"name": n, "role": "INERT", "channels": 1, "wire": names[n], "actuators": []}
                          for n in ("frequency", "env_idx")],
                       "cameras": [{"name": c["name"], "mount": c["mount"], "by": c["by"], "wire": names[c["name"]],
                                    "sensor": c["sensor"]} for c in cameras]}, f, indent=1)
    if beat in snaps:
        Image.fromarray(np.concatenate([p["color"] for p in pictures().values()], axis=1)).save(
            os.path.join(args.out, "snap_%d.png" % beat))
    take_action(call("get_action", beat))
    t3 = time.perf_counter()
    t_step += t1 - t0
    t_render += t2 - t1
    t_wire += t3 - t2
    if beat % 200 == 0:
        # Where the robot's root stands and how far it leans, when it is free to (a drone, a wheeled or legged base):
        # that it hovers or stands as such a robot does is part of the world being right.
        root = ""
        if m.njnt and m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE:
            z = np.zeros(9)
            mujoco.mju_quat2Mat(z, d.qpos[3:7])
            root = ", its root at %.2f %.2f %.2f m leaning %.1f deg" % (d.qpos[0], d.qpos[1], d.qpos[2],
                                                                      math.degrees(math.acos(max(-1.0, min(1.0, z[8])))))
        print("robot: beat %d, a beat takes the world %.2f ms (the pictures' %.2f), the readings %.2f ms, the scramble and "
              "the driver %.2f ms%s" % (beat, 1000 * t_step / beat, 1000 * t_capture[0] / beat, 1000 * t_render / beat,
                                       1000 * t_wire / beat, root), flush=True)
    if args.show_refs and beat % args.show_refs == 0 and m.njnt and m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE:
        print("robot: beat %d, root %.2f %.2f %.2f, asked %s" % (beat, d.qpos[0], d.qpos[1], d.qpos[2], " ".join(
            "%s %s" % (g["name"], [round(float(x), 3) for x in g["ref"]]) for g in groups if g["kind"] in ("flight", "walk"))),
            flush=True)
    if walker and walker.fallen() and not fell[0]:
        fell[0] = beat
        print("robot: FELL at beat %d, leaning %.1f deg (its training's fell_over: %.1f)"
              % (beat, math.degrees(walker.lean()), math.degrees(legged.FELL)), flush=True)
    if beat >= args.beats or (args.stop and os.path.exists(args.stop)):
        break
    if args.rate > 0:
        next_at += 1.0 / args.rate
        time.sleep(max(0.0, next_at - time.time()))
renderer.close()
if truth:
    truth.close()
ws.close()
print("robot: ended at beat %d" % beat, flush=True)
