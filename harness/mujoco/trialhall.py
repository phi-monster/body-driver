# trialhall.py RUN FLOWDIR: simulator-side rig code (a diagnostic, not the driver), built on armturnpx.py's rebuild of the
# run's body, eyes and true joints. Step 3's turn search against the hall's own truth: every trial at the true links'
# poses of its beat (truth.jsonl), Best_Turn computed as Flow_Eye computes it on renders of them, beside the logged best;
# where its gate's pixels lie and how far they truly moved. BD_JOINTS: only those joints; BD_BREAKDOWN: the gate's
# pixels by where they lie.
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
trial_re = re.compile(r"^@(\d+) \[robot\] flows: arm (\d+) joint (\d+) turn ([-\d.eE+]+) reading units: best (\S+)")
trials = []
for l in log:
    mm = trial_re.match(l)
    if mm and int(mm.group(2)) == arm_id:
        trials.append((int(mm.group(1)), int(mm.group(3)), float(mm.group(4)), None if mm.group(5) == "none" else float(mm.group(5))))
if not trials:
    print("no trials logged"); sys.exit(0)
first, last = trials[0][0], trials[-1][0]
calls = {}
for l in open(run + "/truth.jsonl"):
    mm = re.search(r'"call":(\d+)', l)
    if not mm: continue
    c = int(mm.group(1))
    if c < first - 60: continue
    if c > last + 2: break
    o = json.loads(l)
    calls[c] = (o["state"][g["name"]], o["links"])
# The still: back from the first trial's beat past the beats at its turn, past the push, to the arm at rest before it.
c = first - 1
at_turn = calls[c][0]
while c - 1 in calls and calls[c - 1][0] == at_turn: c -= 1
while c - 1 in calls and calls[c - 1][0] != calls[c][0]: c -= 1
still_beat = c - 1 if c - 1 in calls else c
print("still at beat %d (state %s), first trial at %d (state %s)" % (still_beat, calls[still_beat][0], first, calls[first - 1][0]))
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
q_s = q_of(calls[still_beat][1])
d.qpos[:] = q_s; mujoco.mj_forward(m, d)
err = max(1000 * np.linalg.norm(d.xpos[b] - np.array(calls[still_beat][1][body_name[b]][:3])) for b in range(1, m.nbody) if body_name[b] in calls[still_beat][1])
print("the still's links rebuilt within %.3f mm of the truth's" % err)
renderer = mujoco.Renderer(m, height, width)
camera = [cm for cm in built["cameras"] if cm["name"] == fixed_cam][0]
sn = camera["sensor"]; K_e = 255.0 / sn["full_well"]; read_levels = sn["read_noise"] * K_e
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
# The driver's filters (Flow_Eye): the least Gaussian a picture's levels resolve, its weights to one level of the peak;
# the slope its derivative scaled so that a ramp's is its slope.
Levels = 255.0
s_sig = math.sqrt(2.0 * math.log(Levels)) / math.pi
s_rad = int(math.floor(s_sig * math.sqrt(2.0 * math.log(Levels))))
xk = np.arange(-s_rad, s_rad + 1)
w = np.exp(-xk ** 2 / (2 * s_sig ** 2)); w /= w.sum()
sl = xk * w / float(np.sum(xk ** 2 * w))
def filt(img, kx, ky):
    # correlate rows with kx (along x) and columns with ky (along y), as Flow_Eye.Filter
    a = ndi.correlate1d(img, kx, axis=1, mode="nearest")
    return ndi.correlate1d(a, ky, axis=0, mode="nearest")
quarter = NormalDist().inv_cdf(0.625)
def filtered_spread(vals):
    inner = np.abs(vals[s_rad:height - s_rad, s_rad:width - s_rad]).ravel()
    k = (inner.size + 3) // 4
    return float(np.partition(inner, k - 1)[k - 1]) / quarter
pixels = width * height
tail = 2 * (1 - NormalDist().cdf(3.0))
gate = NormalDist().inv_cdf(1 - tail / pixels / 2)
sq_w, sq_s = float(np.sum(w ** 2)), float(np.sum(sl ** 2))
def best_turn(still, turned, turn, own_mask=None):
    gu = filt(still, sl, w); gv = filt(still, w, sl); G = np.hypot(gu, gv)
    diff = turned - still
    ch = filt(diff, w, w)
    raw = diff.ravel(); noise = 1.4826 * float(np.median(np.abs(raw - np.median(raw))))
    sig_c = max(filtered_spread(ch), noise * sq_w)
    sig_g = max(filtered_spread(filt(diff, sl, w)) / math.sqrt(2.0), noise / math.sqrt(2.0) * math.sqrt(sq_s * sq_w))
    moved = (G > gate * sig_g) & (np.abs(ch) > gate * sig_c)
    if own_mask is not None: moved &= own_mask
    if moved.sum() == 0: return None, 0, moved
    per_unit = float(np.median(np.abs(ch[moved]) / G[moved])) / turn
    slope_m = float(np.median(G[moved]))
    target = (3.0 * s_sig ** 2 * sig_c / (math.sqrt(2.0) * slope_m)) ** (1.0 / 3.0)
    return target / per_unit, int(moved.sum()), moved
# The still: depth and segmentation, every pixel's body and world point, the still picture through the camera.
d.qpos[:] = q_s; mujoco.mj_forward(m, d)
renderer.enable_depth_rendering(); renderer.update_scene(d, camera=cf); depth = renderer.render().copy(); renderer.disable_depth_rendering()
renderer.enable_segmentation_rendering(); renderer.update_scene(d, camera=cf); seg = renderer.render().copy(); renderer.disable_segmentation_rendering()
f = (height / 2.0) / math.tan(math.radians(m.cam_fovy[cf]) / 2.0)
ys, xs = np.mgrid[0:height, 0:width]
xt = (xs + 0.5 - width / 2) / f; yt = (ys + 0.5 - height / 2) / f
Rm_s = d.cam_xmat[cf].reshape(3, 3).copy(); C_s = d.cam_xpos[cf].copy()
Xw = np.stack([xt * depth, -yt * depth, -depth], axis=-1) @ Rm_s.T + C_s
gb = np.where(seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), m.geom_bodyid[np.clip(seg[..., 0], 0, m.ngeom - 1)], -1)
xpos_s, xmat_s = d.xpos.copy(), d.xmat.copy().reshape(-1, 3, 3)
still_img = view(q_s)
parent = m.body_parentid
def descends(b, root):
    while b > 0:
        if b == root: return True
        b = parent[b]
    return False
def motion(q):
    d.qpos[:] = q; mujoco.mj_forward(m, d)
    Rm = d.cam_xmat[cf].reshape(3, 3); Cc = d.cam_xpos[cf]
    out = np.zeros((height, width))
    for b in np.unique(gb):
        if b <= 0: continue
        sel = gb == b
        R = d.xmat[b].reshape(3, 3) @ xmat_s[b].T
        Y = (Xw[sel] - xpos_s[b]) @ R.T + d.xpos[b]
        Yc = (Y - Cc) @ Rm
        u = f * (Yc[:, 0] / -Yc[:, 2]) - f * xt[sel]; v = f * (-Yc[:, 1] / -Yc[:, 2]) - f * yt[sel]
        out[sel] = np.hypot(u, v)
    return out
want_joints = set(int(x) for x in os.environ.get("BD_JOINTS", "").split(",") if x)
arm_bodies = set()
for u in chain:
    arm_bodies |= {b for b in range(m.nbody) if descends(b, m.jnt_bodyid[u["j"]])}
for beat, J, T, B in trials:
    if J > len(chain) or (want_joints and J not in want_joints): continue
    t = chain[J - 1]
    own = np.isin(gb, [b for b in range(m.nbody) if descends(b, m.jnt_bodyid[t["j"]])])
    if beat - 1 not in calls: continue
    q_b = q_of(calls[beat - 1][1])
    q_o = q_s.copy(); a_ = m.jnt_qposadr[t["j"]]; q_o[a_] = q_b[a_]
    mot_b, mot_o = motion(q_b), motion(q_o)
    turned, alone = view(q_b), view(q_o)
    b_all, n_all, moved = best_turn(still_img, turned, T)
    b_alone, n_alone, _ = best_turn(still_img, alone, T)
    b_ownpx, n_ownpx, _ = best_turn(still_img, turned, T, own)
    fmt = lambda x: "none" if x is None else "%.3g" % x
    others = [(abs(q_b[m.jnt_qposadr[u["j"]]] - q_s[m.jnt_qposadr[u["j"]]]), k + 1) for k, u in enumerate(chain) if u is not t]
    worst = max(others) if others else (0.0, 0)
    on_own = float((moved & own).sum()) / max(1, n_all)
    print("j%d T %-7.3g log %-7s | renders: as stood %-7s alone %-7s own px %-7s | true own-link median %.2f px (alone %.2f), "
          "turned %.3g of the commanded | gate %d px, %.0f%% on its links, their true median %.2f, the rest's %.2f px"
          % (J, T, fmt(B), fmt(b_all), fmt(b_alone), fmt(b_ownpx), float(np.median(mot_b[own])) if own.any() else float("nan"),
             float(np.median(mot_o[own])) if own.any() else float("nan"),
             (q_b[a_] - q_s[a_]) / (t["C"] * T) if T else float("nan"), n_all, 100 * on_own,
             float(np.median(mot_b[moved & own])) if (moved & own).any() else float("nan"),
             float(np.median(mot_b[moved & ~own])) if (moved & ~own).any() else float("nan"))
          + " | others off the still up to %.2g rad (j%d), its own %.2g rad" % (worst[0], worst[1], abs(q_b[a_] - q_s[a_])), flush=True)
    if os.environ.get("BD_BREAKDOWN"):
        # the gate's pixels off the joint's links: within the smoothing's reach of its links (an edge's spill), on other
        # links of the arm, on the rest of the world; each group's share and median |change| / |gradient| (px)
        gu = filt(still_img, sl, w); gv = filt(still_img, w, sl); G = np.hypot(gu, gv); ch = filt(turned - still_img, w, w)
        ratio = np.abs(ch) / np.maximum(G, 1e-9)
        near = ndi.binary_dilation(own, iterations=s_rad) & ~own
        arm_other = np.isin(gb, list(arm_bodies)) & ~own
        groups = [("own", moved & own), ("spill", moved & near), ("arm", moved & arm_other & ~near),
                  ("world", moved & ~own & ~near & ~arm_other)]
        print("    gate's pixels: " + ", ".join("%s %d (%.2f px)" % (nm, int(gm.sum()), float(np.median(ratio[gm])) if gm.any() else float("nan"))
                                            for nm, gm in groups)
              + "; bodies of the world's: " + ", ".join("%s %d" % (body_name[b] if b >= 0 else "none", n) for b, n in
                  sorted(zip(*np.unique(gb[moved & ~own & ~near & ~arm_other], return_counts=True)), key=lambda x: -x[1])[:4]), flush=True)
