import time
t0=time.time()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "enable_cameras": True})
print("APP_UP", round(time.time()-t0,1), flush=True)
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
w = World(stage_units_in_meters=1.0)
w.scene.add_default_ground_plane()
w.scene.add(DynamicCuboid(prim_path="/World/box", name="box", position=np.array([0,0,0.5]), size=0.2))
cam = Camera(prim_path="/World/cam", position=np.array([2.0,0.0,1.0]), frequency=20, resolution=(160,120),
             orientation=rot_utils.euler_angles_to_quats(np.array([0,15,180]), degrees=True))
w.reset(); cam.initialize()
print("WORLD_READY", round(time.time()-t0,1), flush=True)
for i in range(30):
    w.step(render=True)
img = cam.get_rgba()
print("RGBA", None if img is None else img.shape, "耗时", round(time.time()-t0,1), flush=True)
if img is not None:
    print("非零像素比", float((img[...,:3].sum(-1)>0).mean()))
    np.save("/root/render_test.npy", img[...,:3])
print("RENDER_TEST_DONE", flush=True)
app.close()
