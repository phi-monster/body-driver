# trialtruth.py RUN FLOWDIR: simulator-side rig code (a diagnostic, not the driver), built on armturnpx.py's rebuild of the
# run's body, eyes and true joints. Step 3's turn search at FLOWDIR's first pose against the commanded turns' truth: for
# every trial the log names, the true median motion of the fixed eye's pixels on the links the joint moves, and the turn
# that moves them 0.4 px.
import io, json, math, os, re, subprocess, sys
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco
from PIL import Image
import bodies

run, flow = sys.argv[1:3]
mode = "clean"
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



# trialtruth: at the dump's first pose (where Find_Steps searched each joint's turn), the true median motion of the fixed
# eye's pixels on the links a joint moves, for every turn the search tried (the log's "flows: arm A joint J turn T
# reading units: best B" lines), and the turn whose true median motion is 0.4 px.
renderer = mujoco.Renderer(m, height, width)
f = (height / 2.0) / math.tan(math.radians(m.cam_fovy[cf]) / 2.0)
ys, xs = np.mgrid[0:height, 0:width]
xt = (xs + 0.5 - width / 2) / f; yt = (ys + 0.5 - height / 2) / f
parent = m.body_parentid
def descends(b, root):
    while b > 0:
        if b == root: return True
        b = parent[b]
    return False
trials = {}
for l in log:
    mm = re.search(r"flows: arm (\d+) joint (\d+) turn ([-\d.eE+]+) reading units: best (\S+)", l)
    if mm and int(mm.group(1)) == arm_id:
        trials.setdefault(int(mm.group(2)), []).append((float(mm.group(3)), None if mm.group(4) == "none" else float(mm.group(4))))
p = poses[0]
q = qref.copy()
for J, t in enumerate(chain):
    q[m.jnt_qposadr[t["j"]]] += t["C"] * p["readings"][J]
d.qpos[:] = q; mujoco.mj_forward(m, d)
renderer.enable_depth_rendering(); renderer.update_scene(d, camera=cf); depth = renderer.render().copy(); renderer.disable_depth_rendering()
renderer.enable_segmentation_rendering(); renderer.update_scene(d, camera=cf); seg = renderer.render().copy(); renderer.disable_segmentation_rendering()
Rm = d.cam_xmat[cf].reshape(3, 3); Cc = d.cam_xpos[cf].copy()
Xc = np.stack([xt * depth, -yt * depth, -depth], axis=-1); Xw = Xc @ Rm.T + Cc
gb = np.where(seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), m.geom_bodyid[np.clip(seg[..., 0], 0, m.ngeom - 1)], -1)
def median_motion(J, turn):
    t = chain[J - 1]
    th = t["C"] * turn
    jb = m.jnt_bodyid[t["j"]]
    moved = np.isin(gb, [b for b in range(m.nbody) if descends(b, jb)])
    if moved.sum() < 20: return None, int(moved.sum())
    a = d.xaxis[t["j"]].copy(); pj = d.xanchor[t["j"]].copy()
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    Rot = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
    Y = (Xw[moved] - pj) @ Rot.T + pj
    Yc = (Y - Cc) @ Rm
    u = f * (Yc[:, 0] / -Yc[:, 2]) - f * xt[moved]; w = f * (-Yc[:, 1] / -Yc[:, 2]) - f * yt[moved]
    return float(np.median(np.hypot(u, w))), int(moved.sum())
for J in sorted(trials):
    if J > len(chain): continue
    m1, n = median_motion(J, 1.0e-3 * max(abs(x[0]) for x in trials[J]))
    if m1 is None:
        print("j%d: %d pixels of its links seen" % (J, n)); continue
    lo, hi = 0.0, max(abs(x[0]) for x in trials[J])
    while median_motion(J, hi)[0] < 0.4 and abs(chain[J - 1]["C"] * hi) < math.pi / 2:
        hi *= 2.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if median_motion(J, mid)[0] < 0.4: lo = mid
        else: hi = mid
    star = 0.5 * (lo + hi)
    print("j%d (%d px of its links): 0.4 px at turn %.4g; tried %s; kept %.4g = %.2f of it, its median %.2f px" % (
        J, n, star, " ".join("%.3g:%.2fpx" % (T, median_motion(J, T)[0]) for T, B in trials[J]),
        p["steps"][J - 1], p["steps"][J - 1] / star, median_motion(J, p["steps"][J - 1])[0]))
