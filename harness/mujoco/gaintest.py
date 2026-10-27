# gaintest.py RUN FLOWDIR COSTFILE [STEM]: simulator-side rig code (a diagnostic, not the driver), costclass.py's rebuild
# and classes. Is a pixel of the flows' evidence the world in shade? At each pixel the replay's cost file lists (the
# kept ones), its pose's still against the background (the poses' stills' median, as Evidence takes it) over the
# smoothing's window: the still fitted as the background times a gain that varies linearly across the window, its
# chi square a degree of freedom at the still's noise (the pose's noise of a difference over root two). Per class of
# the truth's segmentation, how many pixels a gate on that chi square calls shade; with STEM, the pixels it keeps
# (STEM_1.txt, "pose column row" a line) for the replay's BD_STUDY_KEEP.
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
import numpy as np
W_, H_ = width, height
idxl = open(flow + "/index.txt").read().splitlines()
noise_of = {}
vv = 0
for l in idxl:
    t = l.split()
    if t and t[0] == "pose": vv = int(t[1]); noise_of[vv] = float(t[3])
P = len(noise_of)
stills = {v: np.fromfile(flow + "/arm_1_pose_%d_still.bin" % v, dtype="<f8").reshape(H_, W_) for v in range(1, P + 1)}
B = np.median(np.stack([stills[v] for v in range(1, P + 1)]), axis=0)
r_ = s_rad
dy, dx = np.mgrid[-r_:r_ + 1, -r_:r_ + 1]
dx = dx.ravel().astype(float); dy = dy.ravel().astype(float)
def gain_chi(v, c, r):
    if r < r_ or c < r_ or r >= H_ - r_ or c >= W_ - r_: return None
    s = stills[v][r - r_:r + r_ + 1, c - r_:c + r_ + 1].ravel(); b = B[r - r_:r + r_ + 1, c - r_:c + r_ + 1].ravel()
    A = np.stack([b, b * dx, b * dy], axis=1)
    a, *_ = np.linalg.lstsq(A, s, rcond=None)
    left = s - A @ a
    sig = noise_of[v] / math.sqrt(2.0)
    return float(np.sum(left ** 2) / (sig ** 2) / (len(s) - 3))
stem = sys.argv[4] if len(sys.argv) > 4 else None
rows = []
for l in open(costs):
    t = l.split()
    v, u, w = int(t[0]), float(t[1]), float(t[2])
    gb, inside, lmn, lmx = seg_at(v)
    cc, rr = min(width - 1, max(0, int(u))), min(height - 1, max(0, int(w)))
    body_link = link_of.get(int(gb[rr, cc]), 0) if gb[rr, cc] >= 0 else 0
    where = "inside" if inside[rr, cc] else "edge"
    rel = "off the arm" if body_link == 0 else "on the arm"
    g = gain_chi(v, cc, rr)
    rows.append((where, rel, g, "%d %d %d" % (v, cc, rr)))
# the gate: the chi square a degree of freedom whose tail over every kept pixel is one (the family's own gate)
from statistics import NormalDist
n = len(rows); dof = (2 * r_ + 1) ** 2 - 3
z = NormalDist().inv_cdf(1.0 - 1.0 / max(2, n))
gate = (1.0 + z * math.sqrt(2.0 / dof))   # Wilson-Hilferty-free normal approximation of the chi square's upper tail
print("%d pixels, gate %.2f a degree of freedom (%d)" % (n, gate, dof))
for cls in sorted(set((r[0], r[1]) for r in rows)):
    g = sorted(x[2] for x in rows if (x[0], x[1]) == cls and x[2] is not None)
    if not g: continue
    shade = sum(1 for x in g if x <= gate)
    print("%-6s %-11s %5d px: gain chi2/dof median %8.2f, 10th %7.2f, shade by the gate %5d (%4.1f %%)" % (cls[0], cls[1], len(g),
          g[len(g) // 2], g[len(g) // 10], shade, 100.0 * shade / len(g)))
if stem:
    open(stem + "_1.txt", "w").write("\n".join(r[3] for r in rows if r[2] is not None and r[2] > gate) + "\n")
