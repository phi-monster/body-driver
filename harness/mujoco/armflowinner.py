# armflowinner.py KIND SEED: simulator-side rig code (a diagnostic, not the driver). As armflowtest.py, but only the pixels
# inside a part: the segmentation the same over the 5 x 5 window around (no edge whose depth or motion the buffer mixes)
# beside all the moved pixels as before. Step 3's own setting: the base
# camera still, one hinge of the arm turned either way by a small angle; every pixel on a link the hinge moves (the
# renderer's segmentation, the same body at both turns and at rest) has its exact motion from the depth buffer and the
# hinge's world axis; the measured change (plus less minus, halved, luma smoothed as Flow_Eye smooths) against the
# gradient times that motion: slope, correlation, residual rms against the signal's, per hinge and turn size, for the
# hall's MSAA 4 and 2x2 supersampled. Shading that changes as a surface turns under fixed lights shows here, and not in
# a turn of the camera.
import os, sys, math
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco
import bodies
kind, seed = sys.argv[1], int(sys.argv[2])
built, m, d, walker = bodies.build(kind, np.random.default_rng(seed))
cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "base_cam")
W, H = 320, 240
f = (H / 2) / math.tan(math.radians(m.cam_fovy[cam]) / 2)
sig = math.sqrt(2 * math.log(255)) / math.pi
def gauss(img):
    r = int(math.ceil(3 * sig)); xs = np.arange(-r, r + 1); k = np.exp(-xs ** 2 / (2 * sig ** 2)); k /= k.sum()
    a = np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 1, img)
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 0, a)
def luma(rgb): return 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
def render(mode, what):
    k = 2 if mode == "ss2" else 1
    r = mujoco.Renderer(m, k * H, k * W)
    if what == "depth": r.enable_depth_rendering()
    if what == "seg": r.enable_segmentation_rendering()
    r.update_scene(d, camera=cam); img = r.render(); r.close()
    if what == "rgb":
        img = img.astype(np.float64)
        if k == 2: img = img.reshape(H, 2, W, 2, 3).mean(axis=(1, 3))
        return luma(img)
    return img
q0 = d.qpos.copy()
hinges = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE and "jaw" not in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "")]
parent = m.body_parentid
def descends(b, root):
    while b > 0:
        if b == root: return True
        b = parent[b]
    return False
ys, xs = np.mgrid[0:H, 0:W]
xt = (xs + 0.5 - W / 2) / f; yt = (ys + 0.5 - H / 2) / f
for mode in ("msaa4",):
    m.vis.quality.offsamples = 4
    d.qpos[:] = q0; mujoco.mj_forward(m, d)
    I0 = gauss(render(mode, "rgb")); depth = render("msaa4", "depth"); seg = render("msaa4", "seg")
    gy, gx = np.gradient(I0)
    R = d.cam_xmat[cam].reshape(3, 3); C = d.cam_xpos[cam].copy()
    Xc = np.stack([xt * depth, -yt * depth, -depth], axis=-1)            # camera frame (x right, y up, looking -z)
    Xw = Xc @ R.T + C                                                      # world
    geom_body = np.where(seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), m.geom_bodyid[np.clip(seg[..., 0], 0, m.ngeom - 1)], -1)
    inner = np.zeros((H, W), dtype=bool); inner[8:-8, 8:-8] = True
    lines = []
    for j in hinges:
        jb = m.jnt_bodyid[j]
        moved = inner & np.vectorize(lambda b: b >= 0 and descends(b, jb))(geom_body)
        if moved.sum() < 200: continue
        a = d.xaxis[j].copy(); p = d.xanchor[j].copy()
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        row = []
        for target in (0.3, 1.0):
            # the turn whose median moved pixel moves TARGET px: the motion is linear in a small turn, so measure at 1e-4
            def flow(theta):
                K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                Rot = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K @ K
                Y = (Xw - p) @ Rot.T + p
                Yc = (Y - C) @ R
                u = f * (Yc[..., 0] / -Yc[..., 2]) - f * xt
                v = f * (-Yc[..., 1] / -Yc[..., 2]) - f * yt
                return u, v
            u1, v1 = flow(1e-4)
            disp = np.hypot(u1, v1)[moved]
            theta = target / np.median(disp) * 1e-4
            u, v = flow(theta)
            qa = q0.copy(); qa[m.jnt_qposadr[j]] += theta; d.qpos[:] = qa; mujoco.mj_forward(m, d)
            Ip = gauss(render(mode, "rgb")); sp = render("msaa4", "seg")
            qa[m.jnt_qposadr[j]] -= 2 * theta; d.qpos[:] = qa; mujoco.mj_forward(m, d)
            Im = gauss(render(mode, "rgb")); sm = render("msaa4", "seg")
            d.qpos[:] = q0; mujoco.mj_forward(m, d)
            same = (sp[..., 0] == seg[..., 0]) & (sm[..., 0] == seg[..., 0])
            inner5 = np.ones((H, W), dtype=bool)
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    inner5 &= np.roll(np.roll(seg[..., 0], dy, 0), dx, 1) == seg[..., 0]
            g = np.hypot(gx, gy)
            parts = []
            for label, extra in (("all", np.ones((H, W), dtype=bool)), ("inside", inner5)):
                sel = moved & same & extra & (g > np.percentile(g[moved], 50))
                if sel.sum() < 20:
                    parts.append("%s n%d" % (label, sel.sum())); continue
                D = ((Ip - Im) / 2.0)[sel]; P = (-(gx * u + gy * v))[sel]
                slope = np.dot(D, P) / np.dot(P, P); corr = np.corrcoef(D, P)[0, 1]
                parts.append("%s n%d slope %.2f corr %.2f" % (label, sel.sum(), slope, corr))
            row.append("%.1f px: %s" % (target, ", ".join(parts)))
        lines.append("%-14s %s" % (name, " | ".join(row)))
    print("== " + mode); print("\n".join(lines), flush=True)
