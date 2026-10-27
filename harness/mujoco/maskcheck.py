# maskcheck.py RUN FLOWDIR: simulator-side rig code (a diagnostic, not the driver), built on armturnpx.py's rebuild of the
# run's body, eyes and true joints. Whether offlink.py's masks (each joint's links as the readings put the arm) hold the
# links as they truly stood at each pose's still (truth.jsonl): each joint's angle apart, and the share of the true
# links' pixels outside the kinematic mask grown by the smoothing's reach.
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



# trialhall: step 3's turn search in a hall run against the truth. Every trial the log names ("flows: arm A joint J turn T
# reading units: best B"), at the beat its picture shows, against the still view before the search: the true links'
# poses there (truth.jsonl) set the model's joints (every hinge from its parent's and child's poses), so the renders
# carry the turn as the joint truly made it and every other joint's sag. Per trial: the true median motion of the
# pixels on the links the joint moves (all joints as they stood; the joint alone); Best_Turn as the driver computes it
# (Flow_Eye: the one Gaussian and its derivative, the filters' noise from the lower quarter, the family gate over the
# picture's pixels) on renders through the camera (its noise and JPEG), of the trial as it stood and of the joint alone,
# against the logged best; which pixels its gate took (on the joint's links or not) and their true motion.
import scipy.ndimage as ndi
from statistics import NormalDist

# maskcheck: per pose of the dump, the arm as its readings put it (kinematics) against the arm as it truly stood at the
# pose's still (truth.jsonl's links at the first beat whose state matches the readings), each joint's angle apart and
# the overlap of the pixels on the links each joint moves (true against kinematic), as offlink.py's masks take them.
import scipy.ndimage as ndi
s_sig = math.sqrt(2.0 * math.log(255.0)) / math.pi
s_rad = int(math.floor(s_sig * math.sqrt(2.0 * math.log(255.0))))
calls = []
for l in open(run + "/truth.jsonl"):
    mm = re.search(r'"call":(\d+)', l)
    if not mm: continue
    o = json.loads(l)
    if "state" in o and g["name"] in o["state"]:
        calls.append((int(mm.group(1)), np.array(o["state"][g["name"]], dtype=float), o["links"]))
body_name = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)]
hinges = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
def angle_about(M, axis):
    v = np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]]) / 2.0
    return math.atan2(float(v @ axis), (np.trace(M) - 1.0) / 2.0)
def q_of(links):
    q = qref.copy()
    for j in hinges:
        b = m.jnt_bodyid[j]; pb = m.body_parentid[b]
        cn, pn = body_name[b], body_name[pb]
        if cn not in links or (pn not in links and pb != 0): continue
        Rp = qrot(links[pn][3:7]) if pb != 0 else np.eye(3)
        M = qrot(m.body_quat[b]).T @ Rp.T @ qrot(links[cn][3:7])
        q[m.jnt_qposadr[j]] = angle_about(M, m.jnt_axis[j])
    return q
state_ref = [s for c, s, _ in calls if c == ref][0]
sc_ = scramble["groups"][g["name"]]
renderer = mujoco.Renderer(m, height, width)
parent = m.body_parentid
def descends(b, root):
    while b > 0:
        if b == root: return True
        b = parent[b]
    return False
def seg_at(q):
    d.qpos[:] = q; mujoco.mj_forward(m, d)
    renderer.enable_segmentation_rendering(); renderer.update_scene(d, camera=cf); sg = renderer.render().copy(); renderer.disable_segmentation_rendering()
    return np.where(sg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), m.geom_bodyid[np.clip(sg[..., 0], 0, m.ngeom - 1)], -1)
pos = 0
tick = 1.0
for v, p in enumerate(poses, 1):
    q_k = qref.copy()
    E = state_ref.copy()
    ss_max = 0.0
    for J, t in enumerate(chain):
        q_k[m.jnt_qposadr[t["j"]]] += t["C"] * p["readings"][J]
        k = sc_["order"].index(t["i"])
        ss = sc_["sign"][k] * sc_["scale"][k]; ss_max = max(ss_max, abs(ss))
        E[t["i"]] += p["readings"][J] / ss
    tick_ = 1.0 if g["ticks"] else 2.0 * math.pi / 2 ** g["sensor"]["bits"]
    while pos < len(calls) and np.max(np.abs(calls[pos][1] - E)) > 1.5 * tick_ * ss_max:
        pos += 1
    if pos >= len(calls):
        print("pose %d: still not found" % v); pos = 0; continue
    q_t = q_of(calls[pos][2])
    gk, gt = seg_at(q_k), seg_at(q_t)
    apart = [abs(q_t[m.jnt_qposadr[t["j"]]] - q_k[m.jnt_qposadr[t["j"]]]) for t in chain]
    ious = []
    for J, t in enumerate(chain):
        bs = [b for b in range(m.nbody) if descends(b, m.jnt_bodyid[t["j"]])]
        mk, mt = np.isin(gk, bs), np.isin(gt, bs)
        keep = ndi.binary_dilation(mk, iterations=s_rad + 1) if mk.any() else mk
        ious.append((float((mt & ~keep).sum()) / max(1, mt.sum())))
    print("pose %d (beat %d): joints apart %s rad; true own-link pixels outside the kinematic mask %s" % (
        v, calls[pos][0], " ".join("%.4f" % x for x in apart), " ".join("%.0f%%" % (100 * x) for x in ious)), flush=True)
