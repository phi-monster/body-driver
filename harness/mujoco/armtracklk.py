# armtracklk.py KIND SEED: simulator-side rig code (a diagnostic, not the driver). How many points on the arm a good
# tracker follows in a still camera's pictures as a hinge turns: corners on the arm's links (the smaller eigenvalue of
# the gradient tensor over a 9 x 9 window, the window on one link, local maxima 5 px apart), each tracked by Lucas and
# Kanade with an affine warp of its window (bilinear, Gaussian-smoothed pictures) from the truth's motion put off by
# about a pixel (a prediction's error), against the exact motion (depth buffer, the hinge's world axis).
import os, sys, math
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco
from scipy import ndimage
import bodies
kind, seed = sys.argv[1], int(sys.argv[2])
built, m, d, walker = bodies.build(kind, np.random.default_rng(seed))
cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "base_cam")
W, H, HALF = 320, 240, 4
f = (H / 2) / math.tan(math.radians(m.cam_fovy[cam]) / 2)
def luma(rgb): return 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
def render(what):
    r = mujoco.Renderer(m, H, W)
    if what == "depth": r.enable_depth_rendering()
    if what == "seg": r.enable_segmentation_rendering()
    r.update_scene(d, camera=cam); img = r.render(); r.close()
    return ndimage.gaussian_filter(luma(img.astype(np.float64)), 0.8) if what == "rgb" else img
q0 = d.qpos.copy()
hinges = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE and "jaw" not in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "") and "finger" not in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "")]
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
k = np.ones((2 * HALF + 1, 2 * HALF + 1))
Sxx = ndimage.convolve(gx * gx, k, mode="constant"); Syy = ndimage.convolve(gy * gy, k, mode="constant"); Sxy = ndimage.convolve(gx * gy, k, mode="constant")
least = (Sxx + Syy) / 2 - np.sqrt(((Sxx - Syy) / 2) ** 2 + Sxy ** 2)
arm_root = m.jnt_bodyid[hinges[0]]
segid = seg[..., 0]
same = np.ones((H, W), dtype=bool)
for dy in range(-HALF, HALF + 1):
    for dx in range(-HALF, HALF + 1):
        same &= np.roll(np.roll(segid, dy, 0), dx, 1) == segid
onarm = np.vectorize(lambda b: b >= 0 and descends(b, arm_root))(body) & same
onarm[:HALF + 6, :] = False; onarm[-HALF - 6:, :] = False; onarm[:, :HALF + 6] = False; onarm[:, -HALF - 6:] = False
noise_floor = np.percentile(least[onarm], 50) if onarm.any() else 0
mx = ndimage.maximum_filter(np.where(onarm, least, 0), size=5)
cand = np.argwhere(onarm & (least == mx) & (least > noise_floor))
print("arm pixels away from outlines %d; corners (local maxima above the median) %d" % (onarm.sum(), len(cand)))
off = np.arange(-HALF, HALF + 1)
OY, OX = np.meshgrid(off, off, indexing="ij")
def track(I1, y, x, py, px):
    T = I0[y - HALF:y + HALF + 1, x - HALF:x + HALF + 1]
    Ty, Tx = gy[y - HALF:y + HALF + 1, x - HALF:x + HALF + 1], gx[y - HALF:y + HALF + 1, x - HALF:x + HALF + 1]
    # affine warp: x' = px + (1+a) ox + b oy, y' = py + c ox + (1+e) oy
    p = np.array([px, py, 0.0, 0.0, 0.0, 0.0])
    J = np.stack([Tx.ravel(), Ty.ravel(), (Tx * OX).ravel(), (Tx * OY).ravel(), (Ty * OX).ravel(), (Ty * OY).ravel()], axis=1)
    A = J.T @ J
    if np.linalg.cond(A) > 1e8: return None
    for _ in range(30):
        wx = p[0] + (1 + p[2]) * OX + p[3] * OY
        wy = p[1] + p[4] * OX + (1 + p[5]) * OY
        Iw = ndimage.map_coordinates(I1, [wy.ravel(), wx.ravel()], order=1, mode="nearest").reshape(T.shape)
        e = (Iw - T).ravel()
        dp = np.linalg.solve(A, J.T @ e)
        # inverse compositional, first order: subtract the increment
        p[0] -= dp[0]; p[1] -= dp[1]; p[2] -= dp[2]; p[3] -= dp[3]; p[4] -= dp[4]; p[5] -= dp[5]
        if abs(dp[0]) + abs(dp[1]) < 1e-3: break
    return p[1], p[0]
rng = np.random.default_rng(3)
for j in hinges:
    jb = m.jnt_bodyid[j]; a = d.xaxis[j].copy(); pa = d.xanchor[j].copy()
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    mine = [(y, x) for (y, x) in cand if descends(body[y, x], jb)]
    if len(mine) < 3: print("%-14s %d points" % (name, len(mine))); continue
    row = []
    for theta in (0.01, 0.03, 0.1):
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        Rot = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K @ K
        qa = q0.copy(); qa[m.jnt_qposadr[j]] += theta; d.qpos[:] = qa; mujoco.mj_forward(m, d)
        I1 = render("rgb"); d.qpos[:] = q0; mujoco.mj_forward(m, d)
        errs = []; moves = []
        for (y, x) in mine:
            Y = (Xw[y, x] - pa) @ Rot.T + pa; Yc = (Y - C) @ R
            tx = f * (Yc[0] / -Yc[2]) + W / 2 - 0.5; ty = f * (-Yc[1] / -Yc[2]) + H / 2 - 0.5
            moves.append(math.hypot(tx - x, ty - y))
            r = track(I1, y, x, ty + rng.normal(0, 0.7), tx + rng.normal(0, 0.7))
            if r is None: continue
            errs.append(math.hypot(r[0] - ty, r[1] - tx))
        e = np.array(errs)
        row.append("%.2f rad (moves %.1f px): %d pts, median %.2f, 90th %.2f, within 0.5 px %.0f%%" % (theta, np.median(moves), len(e), np.median(e), np.percentile(e, 90), 100 * (e < 0.5).mean()))
    print("%-14s %s" % (name, " | ".join(row)), flush=True)
