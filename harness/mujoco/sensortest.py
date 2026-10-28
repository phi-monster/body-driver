# sensortest.py: simulator-side rig code. sensor.c's camera sensor against what it stands for: flat pictures at
# several levels with a webcam's noise, the noise each leaves (the rounded picture less the level) against the
# sensor's own (its variance the level's sd squared and a twelfth for the rounding, its mean nought, its kurtosis a
# Gaussian's, its tails beyond three and four sds a Gaussian's), the same key the same picture and the next key
# another; and its time against numpy's draws and sums, as robot.py made them before.
import ctypes, math, os, subprocess, sys, time
import numpy as np
here = os.path.dirname(os.path.abspath(__file__))
lib = os.path.join(here, "libsensor.so")
if not os.path.exists(lib) or os.path.getmtime(lib) < os.path.getmtime(os.path.join(here, "sensor.c")):
    subprocess.run(["cc", "-O2", "-ffp-contract=off", "-fPIC", "-shared", os.path.join(here, "sensor.c"), "-o", lib,
                    "-lm"], check=True)
so = ctypes.CDLL(lib)
so.sensor_setup.argtypes = []
so.sensor_sense.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_long, ctypes.c_void_p, ctypes.c_uint64]
so.sensor_setup()
K, read = 255.0 / 6000.0, 2.5 * 255.0 / 6000.0
sd = np.sqrt(np.float32(K) * np.arange(256, dtype=np.float32) + np.float32(read) ** 2).astype(np.float32)
# A level's pictures against their exact law: out = x + sd n rounded and held to 0 .. 255, so P(out = j) is the normal's
# mass between j - 1/2 and j + 1/2 about x in sd's, the ends taking the rest; a chi square over the values (those
# expected fewer than five times pooled), against its 0.999 point (Wilson and Hilferty's).
def phi(z):
    return 0.5 * math.erfc(-z / math.sqrt(2.0))
bad = 0
n = 2_000_000
for level in (0, 3, 5, 40, 128, 220, 252, 255):
    a = np.full(n, level, dtype=np.uint8)
    out = np.empty_like(a)
    so.sensor_sense(a.ctypes.data, out.ctypes.data, n, sd.ctypes.data, 1000 + level)
    s = float(sd[level])
    p = np.array([(phi((0.5 - level) / s) if j == 0 else 1.0 - phi((254.5 - level) / s) if j == 255
                   else phi((j + 0.5 - level) / s) - phi((j - 0.5 - level) / s)) for j in range(256)])
    seen = np.bincount(out, minlength=256).astype(np.float64)
    want = p * n
    order = np.argsort(want)
    chi, df, pool_w, pool_s = 0.0, -1, 0.0, 0.0
    for j in order:
        pool_w += want[j]; pool_s += seen[j]
        if pool_w >= 5.0:
            chi += (pool_s - pool_w) ** 2 / pool_w; df += 1; pool_w = pool_s = 0.0
    df = max(df, 1)
    crit = df * (1.0 - 2.0 / (9.0 * df) + 3.090 * math.sqrt(2.0 / (9.0 * df))) ** 3
    e = out.astype(np.float64) - level
    ok = chi < crit
    bad += not ok
    print("level %3d: sd %.3f, the noise's mean %+.4f sd %.4f; chi square %.1f over %d degrees (0.999 point %.1f)%s"
          % (level, s, e.mean(), e.std(), chi, df, crit, "" if ok else "  FAILED"))
pic = np.random.default_rng(3).integers(0, 256, (240, 320, 3), dtype=np.uint8)
o1, o2, o3 = (np.empty_like(pic) for _ in range(3))
so.sensor_sense(pic.ctypes.data, o1.ctypes.data, pic.size, sd.ctypes.data, 77)
so.sensor_sense(pic.ctypes.data, o2.ctypes.data, pic.size, sd.ctypes.data, 77)
so.sensor_sense(pic.ctypes.data, o3.ctypes.data, pic.size, sd.ctypes.data, 78)
same, other = bool(np.array_equal(o1, o2)), bool(not np.array_equal(o1, o3))
bad += (not same) + (not other)
N = 200
t = time.perf_counter()
for k in range(N):
    so.sensor_sense(pic.ctypes.data, o1.ctypes.data, pic.size, sd.ctypes.data, k)
ours = (time.perf_counter() - t) / N * 1000
g = np.random.default_rng([1, 13, 0])
t = time.perf_counter()
for k in range(N):
    noise = g.standard_normal(pic.shape, dtype=np.float32)
    img = pic.astype(np.float32)
    img += np.sqrt(np.float32(K) * img + np.float32(read) ** 2) * noise
    np.clip(np.rint(img), 0, 255).astype(np.uint8)
numpys = (time.perf_counter() - t) / N * 1000
print("sensortest: the same key the same picture %s, the next key another %s; a picture %.3f ms against numpy's %.3f"
      % (same, other, ours, numpys))
print("sensortest: %d failed" % bad)
sys.exit(1 if bad else 0)
