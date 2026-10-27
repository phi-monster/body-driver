# smoothnoise.py RUN FLOWDIR: simulator-side rig code (a diagnostic, not the driver). The noise of a smoothed difference
# of two still views of the fixed eye, as the camera gives them (its drawn noise and JPEG, two frames a view, or noise
# alone without the JPEG), against the white-noise formula the flows' gates take it by (the difference's own noise over
# its pixels times the root of the smoothing's summed squared weights).
import io, math, os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "turnlin.py")).read().split("p = poses[want_pose - 1]")[0].replace(
    "run, flow = sys.argv[1:3]\nwant_pose = int(sys.argv[3]) if len(sys.argv) > 3 else 1", "run, flow = sys.argv[1:3]\nwant_pose = 1"))
p = poses[0]
q = qref.copy()
for J, t in enumerate(chain):
    q[m.jnt_qposadr[t["j"]]] += t["C"] * p["readings"][J]
r = int(math.ceil(3 * sig)); k1 = np.exp(-np.arange(-r, r + 1) ** 2 / (2 * sig ** 2)); k1 /= k1.sum()
w2 = float(np.sum(np.outer(k1, k1) ** 2))
for use_jpeg in (True, False):
    sn["jpeg"] = sn["jpeg"] if use_jpeg else None
    a, b = view(q), view(q)
    dd = b - a
    raw = 1.4826 * float(np.median(np.abs(dd - np.median(dd))))
    sm = gauss(dd)[10:-10, 10:-10]
    smoothed = 1.4826 * float(np.median(np.abs(sm - np.median(sm))))
    print("%s: a difference's noise %.3f levels; smoothed %.4f, the white-noise formula %.4f (x%.2f)"
          % ("noise and JPEG" if use_jpeg else "noise alone", raw, smoothed, raw * math.sqrt(w2), smoothed / (raw * math.sqrt(w2))))
