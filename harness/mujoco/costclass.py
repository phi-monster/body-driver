# costclass.py RUN FLOWDIR COSTFILE: simulator-side rig code (a diagnostic, not the driver), built on edgefit.py's
# rebuild of the run's body, eyes and true joints. Which pixels pull the flows' fit off the truth: each pixel of the
# replay's label cost file (BD_STUDY_COSTS: its cost on each link up to its own, nought first, at the fit's eye and at
# the truth's) classed by the truth's segmentation at its pose (inside one body / on an edge; on the arm with its
# link within the evidence's, beyond it, or off the arm); per class the objective's difference fit - truth with each
# eye's best links, and with the segmentation's links.
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




import scipy.ndimage as ndi
costs = sys.argv[3]
renderer = mujoco.Renderer(m, height, width)
s_sig = math.sqrt(2.0 * math.log(255.0)) / math.pi
s_rad = int(math.floor(s_sig * math.sqrt(2.0 * math.log(255.0))))
parent = m.body_parentid
def descends(b, root):
    while b > 0:
        if b == root: return True
        b = parent[b]
    return False
# the link (chain place) each body is on: the last chain joint it descends from; nought off the arm
link_of = {}
for b in range(m.nbody):
    k = 0
    for J, t in enumerate(chain, 1):
        if descends(b, m.jnt_bodyid[t["j"]]): k = J
    link_of[b] = k
segs = {}
def seg_at(v):
    if v in segs: return segs[v]
    p = poses[v - 1]
    q = qref.copy()
    for J, t in enumerate(chain):
        q[m.jnt_qposadr[t["j"]]] += t["C"] * p["readings"][J]
    d.qpos[:] = q; mujoco.mj_forward(m, d)
    renderer.enable_segmentation_rendering(); renderer.update_scene(d, camera=cf); sg = renderer.render().copy(); renderer.disable_segmentation_rendering()
    gb = np.where(sg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), m.geom_bodyid[np.clip(sg[..., 0], 0, m.ngeom - 1)], -1)
    # inside: the whole smoothing window on one body
    mx = ndi.maximum_filter(gb, size=2 * s_rad + 1, mode="nearest"); mn = ndi.minimum_filter(gb, size=2 * s_rad + 1, mode="nearest")
    # the window's links (nought off the arm or on nothing): an edge between two links, or between the arm and the rest
    lk = np.vectorize(lambda b: link_of.get(int(b), 0) if b >= 0 else 0)(gb)
    lmx = ndi.maximum_filter(lk, size=2 * s_rad + 1, mode="nearest"); lmn = ndi.minimum_filter(lk, size=2 * s_rad + 1, mode="nearest")
    segs[v] = (gb, (mx == mn) & (gb >= 0), lmn, lmx)
    return segs[v]
classes = {}
tot = {"best": 0.0, "seg": 0.0, "n": 0}
# with a stem as a fourth argument: the pixels each mask keeps, "pose column row" a line, for the replay's BD_STUDY_KEEP
stem = sys.argv[4] if len(sys.argv) > 4 else None
masks = {"inside": [], "insidearm": [], "nolinkedge": [], "noworldedge": []}
for l in open(costs):
    t = l.split()
    v, u, w, link = int(t[0]), float(t[1]), float(t[2]), int(t[3])
    c = [float("inf") if x == "inf" else float(x) for x in t[4:]]
    at_fit, at_truth = c[0::2], c[1::2]
    kf = min(range(len(at_fit)), key=lambda k: at_fit[k]); kt = min(range(len(at_truth)), key=lambda k: at_truth[k])
    gb, inside, lmn, lmx = seg_at(v)
    cc, rr = min(width - 1, max(0, int(u))), min(height - 1, max(0, int(w)))
    body_link = link_of.get(int(gb[rr, cc]), 0) if gb[rr, cc] >= 0 else 0
    if inside[rr, cc]: where = "inside"
    elif lmn[rr, cc] == lmx[rr, cc]: where = "edge-1"     # two bodies of one link (or of the world)
    elif lmn[rr, cc] >= 1: where = "edge-ll"              # two links of the arm
    else: where = "edge-aw"                               # the arm and the rest
    row = "%d %d %d" % (v, int(u), int(w))
    if where == "inside": masks["inside"].append(row)
    if where == "inside" and body_link >= 1: masks["insidearm"].append(row)
    if where != "edge-ll": masks["nolinkedge"].append(row)
    if where != "edge-aw": masks["noworldedge"].append(row)
    rel = "off the arm" if body_link == 0 else ("link within" if body_link <= link else "link beyond")
    ks = min(body_link, link)
    db = at_fit[kf] - at_truth[kt]
    ds = at_fit[ks] - at_truth[ks]
    if not (math.isfinite(db) and math.isfinite(ds)):
        key = (where, rel, "unbounded")
        classes.setdefault(key, []).append((0.0, 0.0, 0.0, kf == ks, kt == ks)); continue
    key = (where, rel, "best at truth = seg" if kt == ks else "best at truth != seg")
    classes.setdefault(key, []).append((db, ds, at_truth[kt], kf == ks, kt == ks))
    tot["best"] += db; tot["seg"] += ds; tot["n"] += 1
print("pixels %d: the objective fit - truth with the best links %+.1f, with the segmentation's %+.1f" % (tot["n"], tot["best"], tot["seg"]))
if stem:
    for k, rows in masks.items():
        open("%s%s_1.txt" % (stem, k), "w").write("\n".join(rows) + "\n")
    print("masks: " + ", ".join("%s %d" % (k, len(r)) for k, r in masks.items()))
for k in sorted(classes, key=lambda k: sum(x[0] for x in classes[k])):
    x = classes[k]
    print("%-7s %-12s %-22s %5d px: best links %+9.1f, seg links %+9.1f, cost at truth %9.1f, fit's best = seg %3.0f %%"
          % (k[0], k[1], k[2], len(x), sum(y[0] for y in x), sum(y[1] for y in x), sum(y[2] for y in x),
             100.0 * sum(1 for y in x if y[3]) / len(x)))
