# masktest.py: simulator-side rig code. wiremask.mask_frame, the robot's, against websocket-client's own masking, on
# frames of every length up to 4100 bytes and of the robot's sizes, with random keys: the same bytes; and the time of
# each on frames of two and three cameras' pictures, ours to be faster (np.resize made it 13 to 15 times slower, 10-28).
import array, sys, time
import numpy as np
import websocket._abnf as ws_abnf
from wiremask import mask_frame
own = ws_abnf._mask

rng = np.random.default_rng(3)
bad = 0
sizes = list(range(0, 4101)) + [65535, 65536, 100001, 230400, 230403, 460800, 691200, 691202]
for n in sizes:
    key = array.array("B", rng.integers(0, 256, 4, dtype=np.uint8).tobytes())
    data = array.array("B", rng.integers(0, 256, n, dtype=np.uint8).tobytes())
    if own(key, data) != mask_frame(key, data):
        bad += 1
slower = 0
for n in (460800, 691200):
    key = array.array("B", rng.integers(0, 256, 4, dtype=np.uint8).tobytes())
    data = array.array("B", rng.integers(0, 256, n, dtype=np.uint8).tobytes())
    ms = []
    for f in (own, mask_frame):
        t = time.perf_counter()
        for _ in range(20):
            f(key, data)
        ms.append((time.perf_counter() - t) / 20 * 1000)
    slower += ms[1] >= ms[0]
    print("masktest: %d bytes, websocket-client's %.3f ms, ours %.3f ms" % (n, ms[0], ms[1]))
print("masktest: %d of %d frames differ; ours slower on %d of 2 sizes" % (bad, len(sizes), slower))
sys.exit(1 if bad or slower else 0)
