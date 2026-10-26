# aliastest.py KIND SEED: simulator-side rig code (a diagnostic, not the driver). Whether the renderer's picture
# changes with a sub-pixel motion as brightness constancy says (step 3's flows, 10-27): the robot's base camera turned
# about its own vertical axis by a tiny angle (a flow known at every pixel, no depth), the picture before and after
# (luma, smoothed as Flow_Eye smooths: Gaussian sigma sqrt(2 ln 255)/pi px), the measured change against the
# gradient times the flow over the textured pixels: slope, correlation and the residual's rms, for MSAA 4 (the hall's),
# no MSAA, and 2x2 supersampled MSAA 4.
import os, sys, math
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco
import bodies
kind, seed = sys.argv[1], int(sys.argv[2])
built, m, d, walker = bodies.build(kind, np.random.default_rng(seed))
cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "base_cam")
W, H = 320, 240
fovy = m.cam_fovy[cam]
f = (H / 2) / math.tan(math.radians(fovy) / 2)
sig = math.sqrt(2 * math.log(255)) / math.pi
def gauss(img):
    r = int(math.ceil(3 * sig)); xs = np.arange(-r, r + 1); k = np.exp(-xs ** 2 / (2 * sig ** 2)); k /= k.sum()
    a = np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 1, img)
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 0, a)
def luma(rgb): return 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
q0 = m.cam_quat[cam].copy()
def render(mode, delta):
    # turn the camera about its own y axis by delta (quaternion product q0 * (cos, 0, sin, 0))
    c, s = math.cos(delta / 2), math.sin(delta / 2)
    w0, x0, y0, z0 = q0
    m.cam_quat[cam] = [w0 * c - y0 * s, x0 * c - z0 * s, y0 * c + w0 * s, z0 * c + x0 * s]
    mujoco.mj_forward(m, d)
    if mode == "ss2":
        r = mujoco.Renderer(m, 2 * H, 2 * W); r.update_scene(d, camera=cam); img = r.render().astype(np.float64); r.close()
        img = img.reshape(H, 2, W, 2, 3).mean(axis=(1, 3))
    else:
        r = mujoco.Renderer(m, H, W); r.update_scene(d, camera=cam); img = r.render().astype(np.float64); r.close()
    m.cam_quat[cam] = q0
    return luma(img)
ys, xs = np.mgrid[0:H, 0:W]
xt = (xs + 0.5 - W / 2) / f; yt = (ys + 0.5 - H / 2) / f
for mode, samples in (("msaa4", 4), ("nomsaa", 0), ("ss2", 4)):
    m.vis.quality.offsamples = samples
    I0 = gauss(render(mode, 0.0))
    gy, gx = np.gradient(I0)
    g = np.hypot(gx, gy)
    inner = np.zeros_like(g, dtype=bool); inner[8:-8, 8:-8] = True   # the smoothing's zero padding makes edges
    sel = inner & (g > np.percentile(g[inner], 75))
    out = []
    for px in (0.1, 0.3, 1.0):
        delta = px / f
        Ip = gauss(render(mode, delta)); Im = gauss(render(mode, -delta))
        D = (Ip - Im) / 2.0
        # the exact flow: each pixel's ray (camera frame: x right, y up, looking down -z) seen from the camera turned
        # by delta about its y axis is the ray turned by -delta, projected again (image y down)
        rx, ry, rz = xt, -yt, -np.ones_like(xt)
        cd, sd = math.cos(-delta), math.sin(-delta)
        nx, nz = cd * rx + sd * rz, -sd * rx + cd * rz
        ux = f * (nx / -nz - xt); uy = f * ((-ry / -nz) - yt)
        P = -(gx * ux + gy * uy)
        a, b = D[sel], P[sel]
        if np.dot(a, b) < 0: b = -b
        slope = np.dot(a, b) / np.dot(b, b); corr = np.corrcoef(a, b)[0, 1]
        res = a - slope * b
        out.append("%.1f px: slope %.3f corr %.3f residual rms %.2f of signal rms %.2f" % (px, slope, corr, res.std(), b.std()))
    print("%-6s %s" % (mode, " | ".join(out)), flush=True)
