# wristreg.py RUN FLOWDIR [MODE]: simulator-side rig code (a diagnostic, not the driver). Can an arm's own eye measure
# how the whole arm really moved in each of step 3's turns, the other joints' own motion (their load changed) with it?
# For every turn of a hall flow dump (armflowhall.py's beats and true states), the arm's eye rendered at the turn either
# way (MODE clean, or sensor: the eye's drawn noise and JPEG, two frames a view), its motion between the two estimated from
# the pictures alone but the true depth of the first (direct photometric alignment of the world it sees, six terms, Gauss-
# Newton from the motion the readings predict; its own body's pixels left out), against the true motion and against the motion had the turned joint alone
# moved. The estimate tells the other joints' motion where its error is well under the gap between those two.
import io, json, math, os, re, subprocess, sys
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco
from PIL import Image
import bodies

run, flow = sys.argv[1:3]
mode = sys.argv[3] if len(sys.argv) > 3 else "clean"
meta = dict(l.split(None, 1) for l in open(run + "/meta.txt").read().splitlines() if " " in l)
kind, seed = meta["kind"].strip(), int(meta["seed"])
built, m, d, walker = bodies.build(kind, np.random.default_rng(seed))
bodies.draw_sensors(kind, built, np.random.default_rng([seed, 7]))
noise_rng = np.random.default_rng([seed, 1017])

def qrot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
def angle_about(M, axis):
    v = np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]]) / 2.0
    return math.atan2(float(v @ axis), (np.trace(M) - 1.0) / 2.0)
def rodrigues(w):
    th = np.linalg.norm(w)
    if th < 1e-15: return np.eye(3)
    k = w / th; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
def log_rot(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2)); th = math.acos(c)
    if th < 1e-12: return np.zeros(3)
    return th / (2 * math.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

# --- the dump, the run's arm, its turns' beats (armflowhall.py's matching) ---
lines = open(flow + "/index.txt").read().splitlines()
head = lines[0].split(); beat_of_dump = int(head[4]); arm_id = int(lines[1].split()[1])
joints, poses = [], []
for l in lines[1:]:
    t = l.split()
    if t[0] == "joint": v = [float(x) for x in t[1:]]; joints.append((np.array(v[0:3]), np.array(v[3:6]), v[6]))
    elif t[0] == "pose": poses.append({})
    elif t[0] == "readings": poses[-1]["readings"] = np.array([float(x) for x in t[1:]])
    elif t[0] == "steps": poses[-1]["steps"] = np.array([float(x) for x in t[1:]])
    elif t[0] == "arm" and int(t[1]) != arm_id: break
body = json.load(open(run + "/body.json")); truth = json.load(open(run + "/truth.json")); scramble = json.load(open(run + "/scramble.json"))
log = subprocess.run(["sh", "-c", "zstd -dc %s/driver.log.zst 2>/dev/null || cat %s/driver.log" % (run, run)],
                     capture_output=True, text=True, errors="replace").stdout.splitlines()
wire_of = {}
for l in log:
    mm = re.match(r"^camera (\d+): (\S+)", l)
    if mm and int(mm.group(1)) not in wire_of:
        parts = mm.group(2).split("/"); wire_of[int(mm.group(1))] = parts[1] if len(parts) > 1 else parts[0]
name_of_wire = {c["wire"]: c["name"] for c in truth["cameras"]}
a = {x["arm"]: x for x in body["kinematics"]["arms"]}[arm_id]
arm_cam = name_of_wire[wire_of[a["eye"]]]
kf = {}
for l in log:
    mm = re.search(r"kinematics: arm %d keyframe (\d+) at beat (\d+)" % arm_id, l)
    if mm:
        k = int(mm.group(1))
        if k == 1: kf = {}
        kf[k] = int(mm.group(2))
    if re.search(r"kinematics: arm %d begins" % arm_id, l): kf = {}
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
    k = sc["order"].index(i); child = m.jnt_bodyid[j]
    info.append({"i": i, "j": j, "ss": sc["sign"][k] * sc["scale"][k], "C": per / (sc["sign"][k] * sc["scale"][k]),
                 "child": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, child),
                 "parent": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[child]),
                 "axis": m.jnt_axis[j].copy(), "rest": qrot(m.body_quat[child])})
chain, used = [], set()
for W, P, C in joints:
    cand = [t for t in info if t["i"] not in used and abs(abs(C) / abs(t["C"]) - 1.0) < 0.1]
    t = sorted(cand, key=lambda t: t["i"])[0]; used.add(t["i"]); chain.append(t)
want = {t["child"] for t in chain} | {t["parent"] for t in chain}
calls = []
for l in open(run + "/truth.jsonl"):
    mm = re.search(r'"call":(\d+)', l)
    if not mm: continue
    c = int(mm.group(1))
    if c > beat_of_dump: break
    o = json.loads(l)
    calls.append((c, np.array(o["state"][g["name"]], dtype=float), {b: o["links"][b] for b in want}))
by_call = {c: k for k, (c, _, _) in enumerate(calls)}
def true_q(k):
    links = calls[k][2]
    return [angle_about(t["rest"].T @ (qrot(links[t["parent"]][3:7]).T @ qrot(links[t["child"]][3:7])), t["axis"]) for t in chain]
state_ref = calls[by_call[ref]][1]

# --- the arm's eye: renders, its own body's pixels, its pose ---
W, H = 320, 240
renderer = mujoco.Renderer(m, H, W)
cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, arm_cam)
f = (H / 2.0) / math.tan(math.radians(m.cam_fovy[cid]) / 2.0)
cam = [c for c in built["cameras"] if c["name"] == arm_cam][0]
sn = cam["sensor"]; K_e = 255.0 / sn["full_well"]; read_levels = sn["read_noise"] * K_e
arm_bodies = set()
root = m.jnt_bodyid[jids[0]]
for b in range(m.nbody):
    x = b
    while x > 0:
        if x == root: arm_bodies.add(b); break
        x = m.body_parentid[x]
q_base = d.qpos.copy()
def set_q(qs):
    q = q_base.copy()
    for t, v in zip(chain, qs): q[m.jnt_qposadr[t["j"]]] = v
    d.qpos[:] = q; mujoco.mj_forward(m, d)
def luma(img):
    img = img.astype(np.float64); return 0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]
def frame(img):
    if mode == "clean": return luma(img)
    x = img.astype(np.float32)
    x += np.sqrt(K_e * x + read_levels ** 2) * noise_rng.standard_normal(x.shape, dtype=np.float32)
    x = np.clip(np.rint(x), 0, 255).astype(np.uint8)
    if sn["jpeg"]:
        buf = io.BytesIO(); Image.fromarray(x).save(buf, format="JPEG", quality=sn["jpeg"]); x = np.asarray(Image.open(io.BytesIO(buf.getvalue())).convert("RGB"))
    return luma(x)
def views(qs):
    set_q(qs)
    renderer.update_scene(d, camera=cid); img = renderer.render().copy()
    lum = frame(img) if mode == "clean" else (frame(img) + frame(img)) / 2.0
    renderer.enable_depth_rendering(); renderer.update_scene(d, camera=cid); depth = renderer.render().copy(); renderer.disable_depth_rendering()
    renderer.enable_segmentation_rendering(); renderer.update_scene(d, camera=cid); seg = renderer.render().copy(); renderer.disable_segmentation_rendering()
    gb = np.where(seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), m.geom_bodyid[np.clip(seg[..., 0], 0, m.ngeom - 1)], -1)
    own = np.isin(gb, list(arm_bodies))
    Rm = d.cam_xmat[cid].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0]); p = d.cam_xpos[cid].copy()
    return lum, depth, own, Rm, p
sig = math.sqrt(2 * math.log(255)) / math.pi
def gauss(img):
    r = int(math.ceil(3 * sig)); xs = np.arange(-r, r + 1); k = np.exp(-xs ** 2 / (2 * sig ** 2)); k /= k.sum()
    a1 = np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 1, img)
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 0, a1)
def bilinear(img, x, y):
    x0 = np.clip(np.floor(x).astype(int), 0, W - 2); y0 = np.clip(np.floor(y).astype(int), 0, H - 2)
    fx = x - x0; fy = y - y0
    return (img[y0, x0] * (1 - fx) * (1 - fy) + img[y0, x0 + 1] * fx * (1 - fy) + img[y0 + 1, x0] * (1 - fx) * fy + img[y0 + 1, x0 + 1] * fx * fy)
ys, xs = np.mgrid[0:H, 0:W]
def align(I0, D0, own0, I1, R_start, t_start):
    """The eye's motion (R, t: a point X of the first view's frame is R X + t in the second's) from the two pictures,
    from the motion the readings predict (the turned joint alone, by its turn); None where the view shows too little of
    the world."""
    I0s, I1s = gauss(I0), gauss(I1)
    gy, gx = np.gradient(I1s)
    sel = (D0 > 0) & (D0 < 5.0) & ~own0
    sel[:8, :] = sel[-8:, :] = False; sel[:, :8] = sel[:, -8:] = False
    if sel.sum() < 200: return None
    g0y, g0x = np.gradient(I0s)
    sel &= np.hypot(g0x, g0y) > np.percentile(np.hypot(g0x, g0y)[sel], 30)
    u = xs[sel] + 0.5; v = ys[sel] + 0.5; Z = D0[sel]
    X = np.stack([(u - W / 2) / f * Z, (v - H / 2) / f * Z, Z], axis=1)
    target = I0s[sel]
    R = R_start.copy(); t = t_start.copy()
    for it in range(30):
        Xp = X @ R.T + t
        up = f * Xp[:, 0] / Xp[:, 2] + W / 2 - 0.5; vp = f * Xp[:, 1] / Xp[:, 2] + H / 2 - 0.5
        ok = (up > 1) & (up < W - 2) & (vp > 1) & (vp < H - 2)
        r = bilinear(I1s, up[ok], vp[ok]) - target[ok]
        Gx = bilinear(gx, up[ok], vp[ok]); Gy = bilinear(gy, up[ok], vp[ok])
        Xo = Xp[ok]; x_, y_, z_ = Xo[:, 0], Xo[:, 1], Xo[:, 2]
        du = np.stack([f / z_, np.zeros_like(z_), -f * x_ / z_ ** 2], axis=1)
        dv = np.stack([np.zeros_like(z_), f / z_, -f * y_ / z_ ** 2], axis=1)
        dI = Gx[:, None] * du + Gy[:, None] * dv          # d residual / d X'
        # X' = R X + t, perturbed as exp(w) X' + dt: the residual changes by dI . (w x X') = w . (X' x dI), and by dI . dt
        Jw = np.stack([y_ * dI[:, 2] - z_ * dI[:, 1], z_ * dI[:, 0] - x_ * dI[:, 2], x_ * dI[:, 1] - y_ * dI[:, 0]], axis=1)
        Jm = np.concatenate([Jw, dI], axis=1)
        # robust weights (Huber at 1.5 times the median absolute residual)
        s = 1.4826 * np.median(np.abs(r)) + 1e-6
        wts = np.where(np.abs(r) <= 1.5 * s, 1.0, 1.5 * s / np.abs(r))
        A = (Jm * wts[:, None]).T @ Jm; b = (Jm * wts[:, None]).T @ r
        delta = -np.linalg.solve(A + 1e-9 * np.eye(6), b)
        dR = rodrigues(delta[:3]); R = dR @ R; t = dR @ t + delta[3:]
        if np.linalg.norm(delta[:3]) < 1e-8 and np.linalg.norm(delta[3:]) < 1e-9: break
    return R, t
def relative(Ra, pa, Rb, pb):
    return Rb.T @ Ra, Rb.T @ (pa - pb)

pos = 0; rows = []
for V, p in enumerate(poses, 1):
    E = state_ref.copy()
    for J, t in enumerate(chain): E[t["i"]] += p["readings"][J] / t["ss"]
    while pos < len(calls) and np.max(np.abs(calls[pos][1] - E)) > 1.5 * tick * max(abs(t["ss"]) for t in chain): pos += 1
    if pos >= len(calls): break
    q_still = true_q(pos)
    for J, t in enumerate(chain):
        i = t["i"]; delta = p["steps"][J] / t["ss"]
        if p["steps"][J] == 0: continue
        others = [k for k in range(len(E)) if k != i]
        def near(k, target):
            return abs(calls[k][1][i] - target) <= 2.0 * tick and np.max(np.abs(calls[k][1][others] - E[others])) <= 2.5 * tick
        def run_at(target, start):
            k = start
            while k < len(calls) and not near(k, target): k += 1
            if k >= len(calls): return None, k
            while k + 1 < len(calls) and near(k + 1, target): k += 1
            return k, k + 1
        lo, nxt = run_at(E[i] - delta, pos)
        hi, _ = run_at(E[i] + delta, nxt) if lo is not None else (None, pos)
        if lo is None or hi is None: continue
        q_lo, q_hi = true_q(lo), true_q(hi)
        I0, D0, own0, R0, p0 = views(q_lo)
        I1, _, _, R1, p1 = views(q_hi)
        Rt, tt = relative(R0, p0, R1, p1)
        solo_hi = [q_hi[k] if k == J else q_lo[k] for k in range(len(chain))]
        _, _, _, R1s, p1s = views(solo_hi)
        Rs, ts = relative(R0, p0, R1s, p1s)
        est = align(I0, D0, own0, I1, Rs, ts)
        if est is None: continue
        Re, te = est
        rows.append((V, J + 1, np.linalg.norm(log_rot(Re @ Rt.T)), np.linalg.norm(te - tt) * 1000,
                     np.linalg.norm(log_rot(Rs @ Rt.T)), np.linalg.norm(ts - tt) * 1000, np.linalg.norm(log_rot(Rt))))
        pos = max(pos, hi)
        print("pose %2d joint %d: estimate off %.5f rad %.3f mm; the other joints moved it %.5f rad %.3f mm; the turn %.4f rad"
              % rows[-1], flush=True)
r = np.array([x[2:] for x in rows])
print("median: estimate off %.5f rad %.3f mm; the other joints' share %.5f rad %.3f mm; turns %.4f rad" % tuple(np.median(r, axis=0)))
print("turns where the estimate is under a fifth of the other joints' share: rotation %d of %d, translation %d of %d"
      % (int(np.sum(r[:, 0] < 0.2 * r[:, 2])), len(r), int(np.sum(r[:, 1] < 0.2 * r[:, 3])), len(r)))
