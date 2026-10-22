# bench.py MODEL [SECONDS]: simulator-side rig code (not the driver). One real robot model from the Menagerie, three
# cameras added (one on its last body, one on its root body, one fixed in the world), every actuator driven to random
# targets; prints the physics' milliseconds a step, the renderer's a 320x240 picture, and how many of each a second.
import os, sys, time, random
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np

path = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
spec = mujoco.MjSpec.from_file(path)
bodies = [b for b in spec.bodies if b.name != "world"]
last, root = bodies[-1], bodies[0]
last.add_camera(name="bench_wrist", pos=[0.0, 0.0, 0.05], quat=[1.0, 0.0, 0.0, 0.0], fovy=60.0)
root.add_camera(name="bench_body", pos=[0.1, 0.0, 0.1], quat=[0.5, 0.5, -0.5, -0.5], fovy=60.0)
spec.worldbody.add_camera(name="bench_fixed", pos=[1.5, -1.5, 1.2], xyaxes=[1.0, 1.0, 0.0, -0.4, 0.4, 0.8], fovy=50.0)
m = spec.compile()
d = mujoco.MjData(m)
r = mujoco.Renderer(m, 240, 320)
cams = ["bench_wrist", "bench_body", "bench_fixed"]
lo = m.actuator_ctrlrange[:, 0] if m.nu else np.zeros(0)
hi = m.actuator_ctrlrange[:, 1] if m.nu else np.zeros(0)
steps = frames = 0
t_phys = t_rend = 0.0
frame_every = max(1, int(round((1.0 / 30.0) / m.opt.timestep)))   # pictures at 30 a second of simulated time
start = time.time()
while time.time() - start < seconds:
    if steps % 200 == 0 and m.nu:
        d.ctrl[:] = lo + (hi - lo) * np.random.rand(m.nu)
    a = time.perf_counter(); mujoco.mj_step(m, d); t_phys += time.perf_counter() - a; steps += 1
    if steps % frame_every == 0:
        for c in cams:
            a = time.perf_counter(); r.update_scene(d, camera=c); img = r.render(); t_rend += time.perf_counter() - a
            frames += 1
wall = time.time() - start
r.close()
print("%s: %d joints %d actuators timestep %.4f s | physics %.3f ms a step | render %.2f ms a picture | %d steps %d pictures in %.1f s (%.0f pictures a second)"
      % (os.path.basename(os.path.dirname(path)), m.njnt, m.nu, m.opt.timestep, 1000 * t_phys / max(1, steps),
         1000 * t_rend / max(1, frames), steps, frames, wall, frames / wall), flush=True)
