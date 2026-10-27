# turnest.py RUN FLOWDIR [POSE] (from turnlin.py): simulator-side rig code (a diagnostic, not the driver). A flow dump of step 3 made
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

run, flow = sys.argv[1:3]
want_pose = int(sys.argv[3]) if len(sys.argv) > 3 else 1
mode = "sensor"
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



# One pose's turns of every joint at sizes from a sixteenth to eight times the dump's, through the camera's noise and
# JPEG: per size the true median motion of the pixels the turn moves (depth and segmentation), Best_Turn's estimate of it
# (the median of |smoothed change| / |smoothed gradient| over the pixels whose change and gradient stand out), and the
# linearity of the change (the median over those pixels of |change at twice the turn| / 2 |change at the turn|).
renderer = mujoco.Renderer(m, height, width)
camera = [c for c in built["cameras"] if c["name"] == fixed_cam][0]
sn = camera["sensor"]; K_e = 255.0 / sn["full_well"]; read_levels = sn["read_noise"] * K_e
f = (height / 2.0) / math.tan(math.radians(m.cam_fovy[cf]) / 2.0)
ys, xs = np.mgrid[0:height, 0:width]
xt = (xs + 0.5 - width / 2) / f; yt = (ys + 0.5 - height / 2) / f
sig = math.sqrt(2 * math.log(255)) / math.pi
def gauss(img):
    r = int(math.ceil(3 * sig)); k = np.exp(-np.arange(-r, r + 1) ** 2 / (2 * sig ** 2)); k /= k.sum()
    a1 = np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 1, img)
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 0, a1)
def luma(img):
    img = img.astype(np.float64); return 0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]
def frame(img):
    x = img.astype(np.float32)
    x += np.sqrt(K_e * x + read_levels ** 2) * noise_rng.standard_normal(x.shape, dtype=np.float32)
    x = np.clip(np.rint(x), 0, 255).astype(np.uint8)
    if sn["jpeg"]:
        buf = io.BytesIO(); Image.fromarray(x).save(buf, format="JPEG", quality=sn["jpeg"]); x = np.asarray(Image.open(io.BytesIO(buf.getvalue())).convert("RGB"))
    return luma(x)
def view(q):
    d.qpos[:] = q; mujoco.mj_forward(m, d)
    renderer.update_scene(d, camera=cf); img = renderer.render().copy()
    return (frame(img) + frame(img)) / 2.0
parent = m.body_parentid
def descends(b, root):
    while b > 0:
        if b == root: return True
        b = parent[b]
    return False
p = poses[want_pose - 1]
q = qref.copy()
for J, t in enumerate(chain):
    q[m.jnt_qposadr[t["j"]]] += t["C"] * p["readings"][J]
still = view(q); still2 = view(q)
noise = 1.4826 * float(np.median(np.abs((still2 - still) - np.median(still2 - still))))
S0 = gauss(still)
gy, gx = np.gradient(S0); G = np.hypot(gx, gy)
sigma_c = noise * 0.2   # the smoothed difference's noise (Flow_Eye: Noise times the smoothing's square sum, about a fifth)
# BD_MEASURED_NOISE: the smoothed difference's noise measured (two still views' smoothed difference, its robust spread)
if os.environ.get("BD_MEASURED_NOISE"):
    sm_ = gauss(still2 - still)[10:-10, 10:-10]
    sigma_c = 1.4826 * float(np.median(np.abs(sm_ - np.median(sm_))))
d.qpos[:] = q; mujoco.mj_forward(m, d)
renderer.enable_depth_rendering(); renderer.update_scene(d, camera=cf); depth = renderer.render().copy(); renderer.disable_depth_rendering()
renderer.enable_segmentation_rendering(); renderer.update_scene(d, camera=cf); seg = renderer.render().copy(); renderer.disable_segmentation_rendering()
Rm = d.cam_xmat[cf].reshape(3, 3); C = d.cam_xpos[cf].copy()
Xc = np.stack([xt * depth, -yt * depth, -depth], axis=-1); Xw = Xc @ Rm.T + C
gb = np.where(seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), m.geom_bodyid[np.clip(seg[..., 0], 0, m.ngeom - 1)], -1)
print("pose %d, noise of a difference %.2f levels, the smoothed change's noise taken as %.3f" % (want_pose, noise, sigma_c))
# Two measures of how far a turn moves the picture, against the truth (the median motion of the pixels the turn moves,
# depth and segmentation): (1) windows' shifts by Lucas-Kanade between the still and the turned view (9 x 9 windows of
# texture, a few Gauss-Newton steps, the median shift over windows whose shift stands out of its own sigma); (2) the
# change over the gradient at the pixels whose change stands out of the noise by a family-wide gate (Z for every pixel),
# the median.
Iy, Ix = np.gradient(S0)
def lk_shifts(A, B):
    """Each 9 x 9 window's shift from A to B (A's gradient, iterated), its sigma; windows of texture only."""
    out = []
    Ag = gauss(A); Bg = gauss(B)
    gy0, gx0 = np.gradient(Ag)
    for cy in range(12, height - 12, 6):
        for cx in range(12, width - 12, 6):
            sl = (slice(cy - 4, cy + 5), slice(cx - 4, cx + 5))
            gx, gy = gx0[sl].ravel(), gy0[sl].ravel()
            H = np.array([[gx @ gx, gx @ gy], [gx @ gy, gy @ gy]])
            if np.linalg.eigvalsh(H)[0] < 9.0 * 81 * sigma_c ** 2:
                continue
            u = np.zeros(2)
            yy, xx = np.mgrid[cy - 4:cy + 5, cx - 4:cx + 5]
            for it in range(6):
                xs_ = np.clip(xx + u[0], 0, width - 2); ys_ = np.clip(yy + u[1], 0, height - 2)
                x0 = np.floor(xs_).astype(int); y0 = np.floor(ys_).astype(int); fx = xs_ - x0; fy = ys_ - y0
                Bw = (Bg[y0, x0] * (1 - fx) * (1 - fy) + Bg[y0, x0 + 1] * fx * (1 - fy) + Bg[y0 + 1, x0] * (1 - fx) * fy
                      + Bg[y0 + 1, x0 + 1] * fx * fy).ravel()
                e = Bw - Ag[sl].ravel()
                du = -np.linalg.solve(H, np.array([gx @ e, gy @ e]))
                u += du
                if np.hypot(*du) < 1e-3: break
            sig_u = sigma_c * math.sqrt(np.trace(np.linalg.inv(H)))
            out.append((np.hypot(*u), sig_u))
    return out
pixels = width * height
Zfam = 4.5    # about the family-wide gate for some 77 000 pixel tests at the driver's Z (Driver.Uncertain), for this study
for J, t in enumerate(chain):
    base = abs(t["C"] * p["steps"][J])
    if base == 0: continue
    jb = m.jnt_bodyid[t["j"]]
    moved = np.isin(gb, [b for b in range(m.nbody) if descends(b, jb)])
    a = d.xaxis[t["j"]].copy(); pj = d.xanchor[t["j"]].copy()
    Kx = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    print("joint %d:" % (J + 1))
    for k in range(-4, 3):
        th = base * 2.0 ** k
        Rot = np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * Kx @ Kx
        Y = (Xw[moved] - pj) @ Rot.T + pj; Yc = (Y - C) @ Rm
        uu = f * (Yc[:, 0] / -Yc[:, 2]) - f * xt[moved]; ww = f * (-Yc[:, 1] / -Yc[:, 2]) - f * yt[moved]
        true_med = float(np.median(np.hypot(uu, ww)))
        qp = q.copy(); qp[m.jnt_qposadr[t["j"]]] += th
        turned = view(qp)
        ch = gauss(turned - still)
        strict = (np.abs(ch) > Zfam * sigma_c) & (G > Zfam * sigma_c)
        est2 = float(np.median(np.abs(ch[strict]) / G[strict])) if strict.sum() > 20 else float("nan")
        sh = lk_shifts(still, turned)
        movers = [s for s, sg in sh if s > Zfam * sg]
        est1 = float(np.median(movers)) if len(movers) > 5 else float("nan")
        print("  turn x%-5s true median %6.2f px | windows' shifts %6.2f px (%d of %d windows) | strict change/gradient %6.2f px (%d px)"
              % (("1/%d" % 2 ** -k) if k < 0 else str(2 ** k), true_med, est1, len(movers), len(sh), est2, int(strict.sum())), flush=True)
