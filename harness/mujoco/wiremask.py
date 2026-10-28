# wiremask.py: simulator-side rig code. websocket-client's masking of a frame the robot sends, its result byte for byte:
# the frame's bytes XOR its four-byte key repeated. websocket-client does it through two integers the frame's size and
# back: 1.0 ms for 460 KB and 1.6 ms for 690 KB on the big machine (10-28). numpy's np.resize of the key to the frame's
# size took 13 and 23 ms (it joins the key's copies one by one; round r27s2 ran so). The key repeated as bytes: 0.05 and
# 0.07 ms (masktest.py).
import numpy as np


def mask_frame(mask_value, data_value):
    data = np.frombuffer(data_value, dtype=np.uint8)
    key = np.frombuffer(bytes(mask_value) * ((data.size + 3) // 4), dtype=np.uint8, count=data.size)
    return np.bitwise_xor(data, key).tobytes()
