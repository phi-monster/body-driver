# armtracktest.py KIND SEED: simulator-side rig code (a diagnostic, not the driver). Whether points on the arm can be
# followed in a still camera's pictures as an arm's hinge turns (step 3 with tracked points in place of brightness
# rates): textured places on the arm's links away from their outline (the segmentation the same over the patch), each
# one's exact motion from the depth buffer and the hinge's world axis, followed by a plain patch match (the patch's
# zero-mean normalized correlation over a window around the truth, sub-pixel by a parabola either way); per hinge and
# turn, how many points and how far the matches stand from the truth (median, 90th, share within 0.5 px).
import os, sys, math
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco
import bodies
kind, seed = sys.argv[1], int(sys.argv[2])
built, m, d, walker = bodies.build(kind, np.random.default_rng(seed))
cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "base_cam")
W, H, HALF, SEARCH = 320, 240, 8, 3
f = (H / 2) / math.tan(math.radians(m.cam_fovy[cam]) / 2)
def luma(rgb): return 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
def render(what):
    r = mujoco.Renderer(m, H, W)
    if what == "depth": r.enable_depth_rendering()
    if what == "seg": r.enable_segmentation_rendering()
    r.update_scene(d, camera=cam); img = r.render(); r.close()
    return luma(img.astype(np.float64)) if what == "rgb" else img
q0 = d.qpos.copy()
hinges = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE and "jaw" not in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "")]
parent = m.body_parentid
def descends(b, root):
    while b > 0:
        if b == root: return True
        b = parent[b]
    return False
d.qpos[:] = q0; mujoco.mj_forward(m, d)
I0 = render("rgb"); depth = render("depth"); seg = render("seg")
R = d.cam_xmat[cam].reshape(3, 3); C = d.cam_xpos[cam].copy()
ys, xs = np.mgrid[0:H, 0:W]
xt = (xs + 0.5 - W / 2) / f; yt = (ys + 0.5 - H / 2) / f
Xw = np.stack([xt * depth, -yt * depth, -depth], axis=-1) @ R.T + C
body = np.where(seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), m.geom_bodyid[np.clip(seg[..., 0], 0, m.ngeom - 1)], -1)
gy, gx = np.gradient(I0)
def box(a, r=HALF):
    c = np.cumsum(np.cumsum(np.pad(a, ((1, 0), (1, 0))), 0), 1)
    out = np.zeros_like(a)
    out[r:H - r, r:W - r] = (c[2 * r + 1:, 2 * r + 1:] - c[:H - 2 * r, 2 * r + 1:] - c[2 * r + 1:, :W - 2 * r] + c[:H - 2 * r, :W - 2 * r])
    return out
Sxx, Syy, Sxy = box(gx * gx), box(gy * gy), box(gx * gy)
least = (Sxx + Syy) / 2 - np.sqrt(((Sxx - Syy) / 2) ** 2 + Sxy ** 2)
arm_root = m.jnt_bodyid[hinges[0]]
def patch_same(y, x):
    p = seg[y - HALF:y + HALF + 1, x - HALF:x + HALF + 1, 0]
    return np.all(p == seg[y, x, 0])
cand = []
for y in range(HALF + SEARCH + 2, H - HALF - SEARCH - 2, 6):
    for x in range(HALF + SEARCH + 2, W - HALF - SEARCH - 2, 6):
        b = body[y, x]
        if b >= 0 and descends(b, arm_root) and patch_same(y, x):
            cand.append((least[y, x], y, x))
cand.sort(reverse=True)
pts = [(y, x) for _, y, x in cand[:max(10, len(cand) // 2)]]
print("points on the arm away from outlines, the textured half: %d" % len(pts))
def match(I1, y, x, ty, tx):
    P = I0[y - HALF:y + HALF + 1, x - HALF:x + HALF + 1]; P = P - P.mean(); pn = np.sqrt((P * P).sum())
    if pn == 0: return None
    cy, cx = int(round(ty)), int(round(tx)); best = None; scores = {}
    for dy in range(-SEARCH, SEARCH + 1):
        for dx in range(-SEARCH, SEARCH + 1):
            yy, xx = cy + dy, cx + dx
            if yy - HALF < 0 or xx - HALF < 0 or yy + HALF >= H or xx + HALF >= W: continue
            Q = I1[yy - HALF:yy + HALF + 1, xx - HALF:xx + HALF + 1]; Q = Q - Q.mean(); qn = np.sqrt((Q * Q).sum())
            s = (P * Q).sum() / (pn * qn) if qn > 0 else -1
            scores[(yy, xx)] = s
            if best is None or s > best[0]: best = (s, yy, xx)
    s, by, bx = best
    def sub(a, b, c): den = a - 2 * b + c; return 0.0 if den == 0 else 0.5 * (a - c) / den
    oy = sub(scores.get((by - 1, bx), s), s, scores.get((by + 1, bx), s))
    ox = sub(scores.get((by, bx - 1), s), s, scores.get((by, bx + 1), s))
    return by + oy, bx + ox, s
for j in hinges:
    jb = m.jnt_bodyid[j]; a = d.xaxis[j].copy(); p = d.xanchor[j].copy()
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    mine = [(y, x) for (y, x) in pts if descends(body[y, x], jb)]
    if len(mine) < 5: continue
    row = []
    for theta in (0.01, 0.03, 0.1):
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        Rot = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K @ K
        qa = q0.copy(); qa[m.jnt_qposadr[j]] += theta; d.qpos[:] = qa; mujoco.mj_forward(m, d)
        I1 = render("rgb"); d.qpos[:] = q0; mujoco.mj_forward(m, d)
        errs = []; moves = []
        for (y, x) in mine:
            Y = (Xw[y, x] - p) @ Rot.T + p; Yc = (Y - C) @ R
            tx = f * (Yc[0] / -Yc[2]) + W / 2 - 0.5; ty = f * (-Yc[1] / -Yc[2]) + H / 2 - 0.5
            moves.append(math.hypot(tx - x, ty - y))
            r = match(I1, y, x, ty, tx)
            if r is None: continue
            errs.append(math.hypot(r[0] - ty, r[1] - tx))
        e = np.array(errs)
        row.append("%.2f rad (moves %.1f px): %d pts, error median %.2f, 90th %.2f, within 0.5 px %.0f%%" % (theta, np.median(moves), len(e), np.median(e), np.percentile(e, 90), 100 * (e < 0.5).mean()))
    print("%-14s %s" % (name, " | ".join(row)), flush=True)
