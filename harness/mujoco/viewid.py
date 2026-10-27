# viewid.py RUN FLOWDIR POSE JOINT (from armflowhall.py): simulator-side rig code (a diagnostic, not the driver). As armflowdump.py MODE sensor,
# but every picture rendered at the arm's true state at the beat the hall took it (truth.jsonl: each joint's angle from
# its links' poses, its child's turn about its axis relative to its parent beside the model's rest), so the other joints'
# own motion while one turned (their load changed under them) shows as it did in the hall; the index gives the turned
# joint's true turn and the still's true angles. What of the hall's step 3 error the arm's real motion makes, beyond a
# render of turns made one joint at a time.
import io, json, math, os, re, subprocess, sys
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco
from PIL import Image
import bodies

run, flow = sys.argv[1:3]
want_v, want_j = int(sys.argv[3]), int(sys.argv[4])
out = '/tmp'
# SOLO: the other joints held at the still's true angles while one turns (only the turned joint's own true turn), the A/B
solo = len(sys.argv) > 4 and sys.argv[4] == "solo"
os.makedirs(out, exist_ok=True)
meta = dict(l.split(None, 1) for l in open(run + "/meta.txt").read().splitlines() if " " in l)
kind, seed = meta["kind"].strip(), int(meta["seed"])
built, m, d, walker = bodies.build(kind, np.random.default_rng(seed))
bodies.draw_sensors(kind, built, np.random.default_rng([seed, 7]))
# BD_NOISE_DRAW: another draw of the camera's noise, for the spread a fit shows over draws alone
noise_rng = np.random.default_rng([seed, 1013, int(os.environ.get("BD_NOISE_DRAW", "0"))])

def qrot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
def angle_about(M, axis):
    v = np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]]) / 2.0
    return math.atan2(float(v @ axis), (np.trace(M) - 1.0) / 2.0)

lines = open(flow + "/index.txt").read().splitlines()
head = lines[0].split()
eye, width, height = int(head[2]), int(head[head.index("width") + 1]), int(head[head.index("height") + 1])
beat_of_dump = int(head[4])
arm_id = int(lines[1].split()[1])
joints, poses = [], []
for l in lines[1:]:
    t = l.split()
    if t[0] == "joint":
        v = [float(x) for x in t[1:]]; joints.append((np.array(v[0:3]), np.array(v[3:6]), v[6]))
    elif t[0] == "pose": poses.append({})
    elif t[0] == "readings": poses[-1]["readings"] = np.array([float(x) for x in t[1:]])
    elif t[0] == "steps": poses[-1]["steps"] = np.array([float(x) for x in t[1:]])
    elif t[0] == "arm" and int(t[1]) != arm_id: break
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
ref = kf.get(1, kf.get(2))
names = scramble["names"]
g = [x for x in built["groups"] if x["role"] == "ARM" and names.get(x["name"]) == a.get("group")]
g = g[0] if g else [x for x in built["groups"] if x["role"] == "ARM"][arm_id - 1]
sc = scramble["groups"][g["name"]]
jids = [m.actuator_trnid[m.actuator(act).id, 0] for act in g["actuators"]]
per = (2.0 * math.pi / g["ticks"]) if g["ticks"] else 1.0
tick = 1.0 if g["ticks"] else 2.0 * math.pi / 2 ** g["sensor"]["bits"]
info = []
for i, j in enumerate(jids):
    k = sc["order"].index(i)
    child = m.jnt_bodyid[j]
    info.append({"i": i, "j": j, "ss": sc["sign"][k] * sc["scale"][k], "C": per / (sc["sign"][k] * sc["scale"][k]),
                 "child": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, child),
                 "parent": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[child]),
                 "axis": m.jnt_axis[j].copy(), "rest": qrot(m.body_quat[child])})
chain, used = [], set()
for W, P, C in joints:
    cand = [t for t in info if t["i"] not in used and abs(abs(C) / abs(t["C"]) - 1.0) < 0.1]
    t = sorted(cand, key=lambda t: t["i"])[0]
    used.add(t["i"]); chain.append(t)
calls = []
want_links = {t["child"] for t in chain} | {t["parent"] for t in chain}
arm_cam, fixed_cam = cam_name(a["eye"]), cam_name(eye)
for l in open(run + "/truth.jsonl"):
    mm = re.search(r'"call":(\d+)', l)
    if not mm: continue
    c = int(mm.group(1))
    if c > beat_of_dump: break
    o = json.loads(l)
    calls.append((c, np.array(o["state"][g["name"]], dtype=float), {b: o["links"][b] for b in want_links},
                  o["cameras"][arm_cam]["pose"] if c in set(kf.values()) else None))
by_call = {c: k for k, (c, _, _, _) in enumerate(calls)}
def true_q(k):
    """Every chain joint's true angle at calls[k]: its child's turn relative to its parent, beyond the model's rest."""
    links = calls[k][2]
    return [angle_about(t["rest"].T @ (qrot(links[t["parent"]][3:7]).T @ qrot(links[t["child"]][3:7])), t["axis"]) for t in chain]
state_ref = calls[by_call[ref]][1]
renderer = mujoco.Renderer(m, height, width)
camera = [c for c in built["cameras"] if c["name"] == fixed_cam][0]
sn = camera["sensor"]; K_e = 255.0 / sn["full_well"]; read_levels = sn["read_noise"] * K_e
cf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, fixed_cam)
q_base = d.qpos.copy()
def luma(img):
    img = img.astype(np.float64)
    return 0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]
def frame(img):
    x = img.astype(np.float32)
    x += np.sqrt(K_e * x + read_levels ** 2) * noise_rng.standard_normal(x.shape, dtype=np.float32)
    x = np.clip(np.rint(x), 0, 255).astype(np.uint8)
    if sn["jpeg"]:
        buf = io.BytesIO(); Image.fromarray(x).save(buf, format="JPEG", quality=sn["jpeg"])
        x = np.asarray(Image.open(io.BytesIO(buf.getvalue())).convert("RGB"))
    return luma(x)
def set_q(qs):
    q = q_base.copy()
    for t, v in zip(chain, qs):
        q[m.jnt_qposadr[t["j"]]] = v
    d.qpos[:] = q; mujoco.mj_forward(m, d)
def view(qs):
    set_q(qs)
    renderer.update_scene(d, camera=cf); img = renderer.render().copy()
    return (frame(img) + frame(img)) / 2.0
def write(name, lum):
    np.asarray(lum, dtype="<f8").reshape(-1).tofile(os.path.join(out, name))
# the arm's frame: its eye at the reference's true state; its unit as truth3.py takes it
q_ref = true_q(by_call[ref])
set_q(q_ref)
flip = np.diag([1.0, -1.0, -1.0])
def cam_pose(name):
    c = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, name)
    return d.cam_xmat[c].reshape(3, 3) @ flip, d.cam_xpos[c].copy()
Ra, pa = cam_pose(arm_cam); Rf, pf = cam_pose(fixed_cam)
pa_t = np.array(calls[by_call[ref]][3][:3])
firsts = [kf[k] for k in sorted(kf)][:unit_frames] if unit_frames else [kf[k] for k in sorted(kf)]
d2 = [np.sum((np.array(calls[by_call[b]][3][:3]) - pa_t) ** 2) for b in firsts if b in by_call]
unit = math.sqrt(sum(d2) / len(d2))
print("rebuilt %s seed %d at the reference's true state: the arm's eye %.2f mm from the truth's" % (kind, seed, 1000 * np.linalg.norm(pa - pa_t)))
Wt = [Ra.T @ d.xaxis[t["j"]] for t in chain]; Pt = [Ra.T @ (d.xanchor[t["j"]] - pa) / unit for t in chain]
index = [lines[0], "arm 1 joints %d poses %d" % (len(chain), len(poses))]
for J in range(len(chain)):
    index.append("joint " + " ".join("%.17E" % v for v in list(Wt[J]) + list(Pt[J]) + [1.0]) + " 0 " + "%.17E" % 0.0)

# Which arm state each of the hall's two turned views shows: each against renders of the arm's true state at every beat
# from the pose's still to past the turn's higher reading (rms difference over the fixed eye's picture, the arm's pixels
# where the two renders of the turn differ).
W_, H_ = width, height
def load(name): return np.fromfile(os.path.join(flow, name), dtype="<f8").reshape(H_, W_)
pos = 0
for V, p in enumerate(poses, 1):
    E = state_ref.copy()
    for J, t in enumerate(chain):
        E[t["i"]] += p["readings"][J] / t["ss"]
    while pos < len(calls) and np.max(np.abs(calls[pos][1] - E)) > 1.5 * tick * max(abs(t["ss"]) for t in chain):
        pos += 1
    if V < want_v:
        continue
    J = want_j - 1; t = chain[J]; i = t["i"]; delta = p["steps"][J] / t["ss"]
    others = [k for k in range(len(E)) if k != i]
    # the other joints within a twentieth of the turn (a fine encoder reads their sag), the turned one within that too
    tol = max(2.5 * tick, 0.05 * abs(delta))
    def near(k, target):
        return abs(calls[k][1][i] - target) <= tol and np.max(np.abs(calls[k][1][others] - E[others])) <= tol
    def run_at(target, start):
        k = start
        while k < len(calls) and not near(k, target): k += 1
        if k >= len(calls): return None, k
        while k + 1 < len(calls) and near(k + 1, target): k += 1
        return k, k + 1
    lo, nxt = run_at(E[i] - delta, pos); hi, _ = run_at(E[i] + delta, nxt)
    if lo is None or hi is None:
        print("pose %d joint %d: the turn's beats not found (tolerance %.5f); E %s delta %.5f; still at beat %d" % (V, want_j, tol, np.round(E, 5), delta, calls[pos][0]))
        last = None
        for k in range(pos, min(len(calls), pos + 400)):
            dv = calls[k][1] - E
            row = " ".join("%+.4f" % x for x in dv)
            if row != last:
                print("  beat %d: off the pose %s" % (calls[k][0], row))
                last = row
        break
    print("pose %d joint %d: still beat %d, minus read at %d, plus read at %d" % (V, want_j, calls[pos][0], calls[lo][0], calls[hi][0]))
    hm, hp = load("arm_1_pose_%d_minus_%d.bin" % (V, want_j)), load("arm_1_pose_%d_plus_%d.bin" % (V, want_j))
    rl, rh = view(true_q(lo)), view(true_q(hi))
    mask = np.abs(rh - rl) > 8
    print("pixels the turn changes: %d" % mask.sum())
    for k in range(max(0, lo - 12), min(len(calls), hi + 4)):
        r = view(true_q(k))
        dm = math.sqrt(float(np.mean((hm - r)[mask] ** 2))); dp = math.sqrt(float(np.mean((hp - r)[mask] ** 2)))
        mark = " <- minus read" if k == lo else (" <- plus read" if k == hi else "")
        print("beat %6d: hall minus view off %6.1f, plus view off %6.1f%s" % (calls[k][0], dm, dp, mark), flush=True)
    break
