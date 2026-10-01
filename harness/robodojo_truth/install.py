"""Install the truth hook into RoboDojo: with BD_TRUTH set to a file path, every observation the
simulator produces appends one JSON line to that file with the world pose of every link of every
robot, {"links": {"<robot>/<link>": [x, y, z, qw, qx, qy, qz]}}, relative to the environment origin.

The observation itself is untouched, so the driver never sees the truth; line k of the file
belongs to the k-th observation (the scorer checks the alignment against the end-effector poses
the robot reports). The patch is marked and idempotent.
Usage on the box: /venv/RoboDojo/bin/python install.py
"""

import pathlib

R = pathlib.Path("/root/RoboDojo")
MARK = "# [bd] truth hook"

manager = R / "env/observation_manager/obs_manager.py"
src = manager.read_text()
if MARK not in src:
    anchor = "        return obs\n"
    assert src.count(anchor) == 1, "obs_manager.py changed; the hook needs a new anchor"
    hook = (
        f"        {MARK}: link poses to a side file for truth-probe runs only (BD_TRUTH)\n"
        "        import os as _bd_os\n"
        "        if _bd_os.environ.get(\"BD_TRUTH\") and self.robot_manager is not None:\n"
        "            import json as _bd_json\n"
        "            _bd_links = {}\n"
        "            _bd_origin = self.robot_manager.scene.env_origins.cpu().numpy()\n"
        "            for _bd_i, _bd_robot in enumerate(self.robot_manager.robot_list):\n"
        "                _bd_key = self.robot_manager.robot_key[_bd_i]\n"
        "                _bd_poses = _bd_key.data.body_link_pose_w.clone().cpu().numpy()\n"
        "                _bd_env = env_idx_list[0]\n"
        "                for _bd_j, _bd_name in enumerate(_bd_key.body_names):\n"
        "                    _bd_p = [float(v) for v in _bd_poses[_bd_env][_bd_j]]\n"
        "                    _bd_p[0] -= float(_bd_origin[_bd_env][0]); _bd_p[1] -= float(_bd_origin[_bd_env][1])\n"
        "                    _bd_p[2] -= float(_bd_origin[_bd_env][2])\n"
        "                    _bd_links[str(_bd_robot.robot_name) + \"/\" + str(_bd_name)] = _bd_p\n"
        "            with open(_bd_os.environ[\"BD_TRUTH\"], \"a\") as _bd_f:\n"
        "                _bd_f.write(_bd_json.dumps({\"links\": _bd_links}) + \"\\n\")\n"
    )
    idx = src.index(anchor)
    manager.write_text(src[:idx] + hook + src[idx:])
print("truth hook installed in env/observation_manager/obs_manager.py")
