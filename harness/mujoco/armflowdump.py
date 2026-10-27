# armflowdump.py RUN FLOWDIR OUT MODE: simulator-side rig code (a diagnostic, not the driver). A flow dump of step 3 made
# from the world itself, for tools/flow_replay: RUN's body and cameras built again from its seed (as robot.py builds
# them), its arm posed (kinematics only, no load) at FLOWDIR's poses and turned either way by FLOWDIR's turns, the fixed
# eye rendered as the hall renders it. MODE clean: the renders as they are (8 bits a channel); MODE sensor: every frame
# through the camera's drawn sensor as robot.py's capture makes it (shot and read noise, then its JPEG), two frames a
# still view, their luma averaged. The index holds the true joints (MuJoCo's axes and anchors in the arm's reference
# frame and unit, as truth3.py takes the frame and unit), units of one radian, readings and turns in radians; truth.txt
# the fixed eye's true lens and pose. What of step 3's error the world's light and the renderer make (MODE clean), and
# what the camera adds (MODE sensor), each against the hall's dump of the same poses.
import io, json, math, os, re, subprocess, sys
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco
from PIL import Image
import bodies

run, flow, out, mode = sys.argv[1:5]
os.makedirs(out, exist_ok=True)
meta = dict(l.split(None, 1) for l in open(run + "/meta.txt").read().splitlines() if " " in l)
kind, seed = meta["kind"].strip(), int(meta["seed"])
built, m, d, walker = bodies.build(kind, np.random.default_rng(seed))
bodies.draw_sensors(kind, built, np.random.default_rng([seed, 7]))
# BD_NOISE_DRAW: another draw of the camera's noise, for the spread a fit shows over draws alone
noise_rng = np.random.default_rng([seed, 1011, int(os.environ.get("BD_NOISE_DRAW", "0"))])

def qrot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

lines = open(flow + "/index.txt").read().splitlines()
head = lines[0].split()
eye, width, height = int(head[2]), int(head[head.index("width") + 1]), int(head[head.index("height") + 1])
arm_id = int(lines[1].split()[1])
joints, poses = [], []
for l in lines[1:]:
    t = l.split()
    if t[0] == "joint":
        v = [float(x) for x in t[1:]]
        joints.append((np.array(v[0:3]), np.array(v[3:6]), v[6]))
    elif t[0] == "pose":
        poses.append({"noise": float(t[3])})
    elif t[0] == "readings":
        poses[-1]["readings"] = np.array([float(x) for x in t[1:]])
    elif t[0] == "steps":
        poses[-1]["steps"] = np.array([float(x) for x in t[1:]])
    elif t[0] == "arm" and int(t[1]) != arm_id:
        break

body = json.load(open(run + "/body.json"))
truth = json.load(open(run + "/truth.json"))
scramble = json.load(open(run + "/scramble.json"))
log = subprocess.run(["sh", "-c", "zstd -dc %s/driver.log.zst 2>/dev/null || cat %s/driver.log" % (run, run)],
                     capture_output=True, text=True, errors="replace").stdout.splitlines()
wire_of = {}
for l in log:
    mm = re.match(r"^camera (\d+): (\S+)", l)
    if mm and int(mm.group(1)) not in wire_of:
        parts = mm.group(2).split("/"); wire_of[int(mm.group(1))] = parts[1] if len(parts) > 1 else parts[0]
name_of_wire = {c["wire"]: c["name"] for c in truth["cameras"]}
def cam_name(e): return name_of_wire[wire_of[e]]
a = {x["arm"]: x for x in body["kinematics"]["arms"]}[arm_id]
kf = {}
for l in log:
    mm = re.search(r"kinematics: arm %d keyframe (\d+) at beat (\d+)" % arm_id, l)
    if mm:
        k = int(mm.group(1))
        if k == 1: kf = {}
        kf[k] = int(mm.group(2))
    if re.search(r"kinematics: arm %d begins" % arm_id, l): kf = {}
n_ref = len(a["reference"]); unit_frames = len(a.get("unit_changes", [])) // n_ref if n_ref else 0
want = set(kf.values())
obs = {}
for l in open(run + "/truth.jsonl"):
    mm = re.search(r'"call":(\d+)', l)
    if mm and int(mm.group(1)) in want:
        obs[int(mm.group(1))] = json.loads(l)
ref = kf.get(1, kf.get(2))
arm_cam, fixed_cam = cam_name(a["eye"]), cam_name(eye)
pa_t = np.array(obs[ref]["cameras"][arm_cam]["pose"][:3])
firsts = [kf[k] for k in sorted(kf)][:unit_frames] if unit_frames else [kf[k] for k in sorted(kf)]
d2 = [np.sum((np.array(obs[b]["cameras"][arm_cam]["pose"][:3]) - pa_t) ** 2) for b in firsts if b in obs]
unit = math.sqrt(sum(d2) / len(d2))

# The arm's group and its joints; the reference pose from the truth's readings at the reference keyframe.
names = scramble["names"]
g = [x for x in built["groups"] if x["role"] == "ARM" and names.get(x["name"]) == a.get("group")]
g = g[0] if g else [x for x in built["groups"] if x["role"] == "ARM"][arm_id - 1]
sc = scramble["groups"][g["name"]]
jids = [m.actuator_trnid[m.actuator(act).id, 0] for act in g["actuators"]]
per = (2.0 * math.pi / g["ticks"]) if g["ticks"] else 1.0
state_ref = np.array(obs[ref]["state"][g["name"]], dtype=float)
qref = d.qpos.copy()
for i, j in enumerate(jids):
    qref[m.jnt_qposadr[j]] = state_ref[i] * (per if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE else 1.0 / (g["ticks"] or 1.0))
for other in built["groups"]:
    if other is g or other["role"] == "ARM" or other["name"] not in obs[ref]["state"] or other["kind"] != "servo":
        continue
    for i, act in enumerate(other["actuators"]):
        aid = m.actuator(act).id
        if m.actuator_trntype[aid] != mujoco.mjtTrn.mjTRN_JOINT:   # a tendon's (Panda's closer): no one joint's reading
            continue
        j = m.actuator_trnid[aid, 0]
        pr = (2.0 * math.pi / other["ticks"]) if other["ticks"] else 1.0
        qref[m.jnt_qposadr[j]] = float(obs[ref]["state"][other["name"]][i]) * pr
d.qpos[:] = qref; mujoco.mj_forward(m, d)
flip = np.diag([1.0, -1.0, -1.0])   # MuJoCo's camera (x right, y up, looking along -z) to the driver's (y down, z ahead)
def cam_pose(name):
    c = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, name)
    return d.cam_xmat[c].reshape(3, 3) @ flip, d.cam_xpos[c].copy(), c
Ra, pa, _ = cam_pose(arm_cam)
Rf, pf, cf = cam_pose(fixed_cam)
print("rebuilt %s seed %d: the arm's eye %.2f mm and %.3f deg from the truth's at the reference, the fixed eye %.2f mm and %.3f deg"
      % (kind, seed, 1000 * np.linalg.norm(pa - pa_t),
         math.degrees(math.acos(min(1.0, (np.trace(Ra.T @ qrot(obs[ref]["cameras"][arm_cam]["pose"][3:7])) - 1) / 2))),
         1000 * np.linalg.norm(pf - np.array(obs[ref]["cameras"][fixed_cam]["pose"][:3])),
         math.degrees(math.acos(min(1.0, (np.trace(Rf.T @ qrot(obs[ref]["cameras"][fixed_cam]["pose"][3:7])) - 1) / 2)))))

# Each chain place of the dump is the true joint of its unit's size, the most parallel axis, the nearest line.
true = []
for i, j in enumerate(jids):
    k = sc["order"].index(i)
    true.append({"j": j, "i": i, "C": per / (sc["sign"][k] * sc["scale"][k]),
                 "W": Ra.T @ d.xaxis[j], "P": Ra.T @ (d.xanchor[j] - pa) / unit})
chain, used = [], set()
for W, P, C in joints:
    best = None
    for t in true:
        if t["i"] in used: continue
        key = (abs(abs(C) / abs(t["C"]) - 1.0) > 0.1, math.degrees(math.acos(min(1.0, abs(float(W @ t["W"]))))) > 10.0,
               float(np.linalg.norm(np.cross(P - t["P"], t["W"]))))
        if best is None or key < best[0]:
            best = (key, t)
    used.add(best[1]["i"]); chain.append(best[1])
print("chain: " + " ".join(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, t["j"]) for t in chain))

renderer = mujoco.Renderer(m, height, width)
camera = [c for c in built["cameras"] if c["name"] == fixed_cam][0]
sn = camera["sensor"]; K_e = 255.0 / sn["full_well"]; read_levels = sn["read_noise"] * K_e
def luma(img):
    img = img.astype(np.float64)
    return 0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]
def frame(img):
    if mode == "clean":
        return luma(img)
    x = img.astype(np.float32)
    x += np.sqrt(K_e * x + read_levels ** 2) * noise_rng.standard_normal(x.shape, dtype=np.float32)
    x = np.clip(np.rint(x), 0, 255).astype(np.uint8)
    if sn["jpeg"]:
        buf = io.BytesIO(); Image.fromarray(x).save(buf, format="JPEG", quality=sn["jpeg"])
        x = np.asarray(Image.open(io.BytesIO(buf.getvalue())).convert("RGB"))
    return luma(x)
def view(q):
    d.qpos[:] = q; mujoco.mj_forward(m, d)
    renderer.update_scene(d, camera=cf); img = renderer.render().copy()
    if mode == "clean":
        return frame(img)
    return (frame(img) + frame(img)) / 2.0
def write(name, lum):
    np.asarray(lum, dtype="<f8").reshape(-1).tofile(os.path.join(out, name))

index = [lines[0], "arm 1 joints %d poses %d" % (len(chain), len(poses))]
for t in chain:
    index.append("joint " + " ".join("%.17E" % v for v in list(t["W"]) + list(t["P"]) + [1.0]) + " 0 " + "%.17E" % 0.0)
for v, p in enumerate(poses, 1):
    dq = np.array([t["C"] * p["readings"][J] for J, t in enumerate(chain)])
    turns = np.array([abs(t["C"] * p["steps"][J]) for J, t in enumerate(chain)])
    # BD_TURN_SCALE: every joint's turn times its own factor (a comma list along the chain), for turns of another size
    if os.environ.get("BD_TURN_SCALE"):
        turns = turns * np.array([float(x) for x in os.environ["BD_TURN_SCALE"].split(",")])
    q = qref.copy()
    for J, t in enumerate(chain):
        q[m.jnt_qposadr[t["j"]]] += dq[J]
    still = view(q)
    if mode == "clean":
        noise = 1.0 / math.sqrt(6.0)   # two renders' 8-bit roundings apart, the least a difference of stills shows
    else:
        diff = view(q) - still
        noise = 1.4826 * float(np.median(np.abs(diff - np.median(diff))))
    write("arm_1_pose_%d_still.bin" % v, still)
    for J, t in enumerate(chain):
        if turns[J] > 0:
            qm = q.copy(); qm[m.jnt_qposadr[t["j"]]] -= turns[J]
            qp = q.copy(); qp[m.jnt_qposadr[t["j"]]] += turns[J]
            write("arm_1_pose_%d_minus_%d.bin" % (v, J + 1), view(qm))
            write("arm_1_pose_%d_plus_%d.bin" % (v, J + 1), view(qp))
        else:
            write("arm_1_pose_%d_minus_%d.bin" % (v, J + 1), still)
            write("arm_1_pose_%d_plus_%d.bin" % (v, J + 1), still)
    index.append("pose %d noise %.17E" % (v, noise))
    index.append("readings " + " ".join("%.17E" % x for x in dq))
    # BD_READ_TURNS: each turn written as the readings would give it (robot.py's sensed_reading: the joint's angle with a
    # count's jitter, rounded to the encoder's step, at the turn either way, half the difference), the render exact
    if os.environ.get("BD_READ_TURNS"):
        st = per if g["ticks"] else 2.0 * math.pi / 2 ** g["sensor"]["bits"]
        jit = g["sensor"]["jitter"]
        read = []
        for J, t in enumerate(chain):
            q0 = qref[m.jnt_qposadr[t["j"]]] + dq[J]
            hi = round((q0 + turns[J]) / st + jit * noise_rng.standard_normal()) * st
            lo = round((q0 - turns[J]) / st + jit * noise_rng.standard_normal()) * st
            read.append(abs(hi - lo) / 2.0)
        turns_written = np.array(read)
    else:
        turns_written = turns
    index.append("steps " + " ".join("%.17E" % x for x in turns_written))
open(os.path.join(out, "index.txt"), "w").write("\n".join(index) + "\n")
f = (height / 2.0) / math.tan(math.radians(m.cam_fovy[cf]) / 2.0)
R = Ra.T @ Rf; tc = Ra.T @ (pf - pa) / unit
open(os.path.join(out, "truth.txt"), "w").write(
    "lens %r %r %r %r\narm 1 rotation %s centre %s\n" % (f, f, width / 2.0, height / 2.0, " ".join(repr(float(x)) for x in R.reshape(-1)),
                                                        " ".join(repr(float(x)) for x in tc)))
print("wrote %s: %d poses, mode %s, unit %.4f m, jpeg %s" % (out, len(poses), mode, unit, sn["jpeg"]))
