"""Install the fisheye test rig into RoboDojo: an F-theta fisheye for the fixed head camera.

- Patches env/camera_manager/camera_manager.py so that a camera whose config has a `lens` entry
  gets Isaac Sim's F-theta lens model (Camera.set_ftheta_properties) right after it is created.
  The patch is marked and idempotent.
- Writes env_cfg/camera/camera_fisheye.yml (the official camera config with a lens on cam_head)
  and env_cfg/x5_fisheye.yml (the official arx_x5 config using it).

The truth is the F-theta polynomial angle(r) = k1 r + k2 r^2 (r in pixels from the image
centre), which is neither Kannala-Brandt nor the double sphere model, so comparing those two
on it is fair. Usage on the box: /venv/RoboDojo/bin/python install.py
"""

import pathlib
import re

R = pathlib.Path("/root/RoboDojo")
MARK = "# [bd] fisheye rig"

manager = R / "env/camera_manager/camera_manager.py"
src = manager.read_text()
if MARK not in src:
    anchor = '                    cur_camera.set_lens_distortion_model("pinhole")\n'
    assert anchor in src, "camera_manager.py changed; the rig needs a new anchor"
    patch = anchor + (
        f"                    {MARK}: a camera with a lens entry gets the F-theta model\n"
        "                    _lens = camera_config.camera.get(\"lens\")\n"
        "                    if _lens is not None:\n"
        "                        _w, _h = args_info[\"resolution\"]\n"
        "                        cur_camera.set_ftheta_properties(nominal_height=_h, nominal_width=_w,\n"
        "                                                         optical_center=(_w / 2.0, _h / 2.0),\n"
        "                                                         max_fov=float(_lens[\"max_fov\"]),\n"
        "                                                         distortion_coefficients=list(_lens[\"coefficients\"]))\n"
        "                        print(\"[bd] fisheye lens on\", camera_name, dict(_lens), flush=True)\n")
    manager.write_text(src.replace(anchor, patch, 1))

official = (R / "env_cfg/camera/camera_config.yml").read_text()
lens = ("    lens:\n"
        "      max_fov: 170.0\n"
        "      coefficients: [0.0, 0.0032, 1.2e-6, 0.0, 0.0]\n")
fisheye = re.sub(r"(cam_head:\n  camera:\n(?:    .*\n)*?    ori: .*\n)", lambda m: m.group(1) + lens, official, count=1)
assert fisheye != official, "cam_head block not found"
(R / "env_cfg/camera/camera_fisheye.yml").write_text(fisheye)

x5 = (R / "env_cfg/arx_x5.yml").read_text().replace("camera: camera_config", "camera: camera_fisheye")
(R / "env_cfg/x5_fisheye.yml").write_text(x5.replace("config_name: arx_x5", "config_name: arx_x5"))
print("fisheye rig installed: env_cfg/x5_fisheye.yml")
