# masktest.py: simulator-side rig code. robot.py's mask_frame against websocket-client's own masking, on frames of every
# length up to a few and of the robot's sizes, with random keys: the same bytes, or the first that differs.
import array, os, sys
import numpy as np
import websocket._abnf as ws_abnf
own = ws_abnf._mask


def mask_frame(mask_value, data_value):
    data = np.frombuffer(data_value, dtype=np.uint8)
    return np.bitwise_xor(data, np.resize(np.frombuffer(mask_value, dtype=np.uint8), data.size)).tobytes()


rng = np.random.default_rng(3)
bad = 0
for n in list(range(0, 20)) + [1023, 4096, 65535, 65536, 100001, 230403]:
    for _ in range(5):
        key = array.array("B", rng.integers(0, 256, 4, dtype=np.uint8).tobytes())
        data = array.array("B", rng.integers(0, 256, n, dtype=np.uint8).tobytes())
        if own(key, data) != mask_frame(key, data):
            bad += 1
print("masktest: %d of %d frames differ" % (bad, 26 * 5))
sys.exit(1 if bad else 0)
