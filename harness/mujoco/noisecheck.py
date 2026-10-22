# noisecheck.py OUT KIND: simulator-side rig code. The robot as robot.py builds it, every camera's picture as its sensor
# gives it, twice in a row with the world still, and between them the frame-to-frame difference magnified eight times,
# in OUT/KIND_noise.png (a row a camera: the picture, the difference); and each camera's and group's drawn sensor.
import os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
from PIL import Image
out, kind = sys.argv[1], sys.argv[2]
sys.argv = [sys.argv[0], "--connect", "none:0", "--kind", kind, "--seed", "1", "--out", "/tmp/noise_" + kind]
here = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(here, "robot.py")).read()
g = {"__name__": "noisecheck", "__file__": os.path.join(here, "robot.py")}
exec(compile(src[:src.index("host, port = args.connect.rsplit")], "robot.py", "exec"), g)
rows = []
for c in g["cameras"]:
    g["capture"](c); a = c["queue"][-1].astype(np.int16)
    g["capture"](c); b = c["queue"][-1].astype(np.int16)
    diff = np.clip(128 + 8 * (b - a), 0, 255).astype(np.uint8)
    rows.append(np.concatenate([b.astype(np.uint8), diff], axis=1))
    print("%s %s: %s; frame-to-frame difference, levels: std %.2f, largest %d" % (kind, c["name"], c["sensor"],
          float(np.std(b - a)), int(np.max(np.abs(b - a)))))
for gr in g["groups"]:
    print("%s %s: %s, steps %s" % (kind, gr["name"], gr["sensor"], np.unique(gr["steps"]).round(8).tolist()))
os.makedirs(out, exist_ok=True)
Image.fromarray(np.concatenate(rows, axis=0)).save(os.path.join(out, kind + "_noise.png"))
