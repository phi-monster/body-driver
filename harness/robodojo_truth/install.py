"""Install the truth hook into RoboDojo (see bd_truth.py for what it writes).

With BD_TRUTH=FILE set, every observation the simulator builds appends one line of truth to
FILE; the observation itself is untouched, so the driver never sees the truth. The hook is a
marked block in obs_manager.get_obs; installing replaces any earlier block, so it is idempotent.
Usage on the box: /venv/RoboDojo/bin/python install.py
"""

import pathlib
import shutil

R = pathlib.Path("/root/RoboDojo")
HERE = pathlib.Path(__file__).resolve().parent
MARK = "# [bd] truth hook"
ANCHOR = "        return obs\n"

manager = R / "env/observation_manager/obs_manager.py"
shutil.copy(HERE / "bd_truth.py", manager.parent / "bd_truth.py")

src = manager.read_text()
assert src.count(ANCHOR) == 1, "obs_manager.py changed; the hook needs a new anchor"
if MARK in src:
    start = src.index("        " + MARK)
    src = src[:start] + src[src.index(ANCHOR):]
hook = (
    f"        {MARK}: truth to a side file for scoring only (BD_TRUTH); the observation is untouched\n"
    "        import os as _bd_os\n"
    "        if _bd_os.environ.get(\"BD_TRUTH\"):\n"
    "            if not hasattr(self, \"_bd_truth\"):\n"
    "                import importlib.util as _bd_ilu\n"
    "                _bd_sp = _bd_ilu.spec_from_file_location(\n"
    "                    \"bd_truth\", _bd_os.path.join(_bd_os.path.dirname(__file__), \"bd_truth.py\"))\n"
    "                self._bd_truth = _bd_ilu.module_from_spec(_bd_sp)\n"
    "                _bd_sp.loader.exec_module(self._bd_truth)\n"
    "            self._bd_truth.write(self, obs, env_idx_list, _bd_os.environ[\"BD_TRUTH\"])\n"
)
idx = src.index(ANCHOR)
manager.write_text(src[:idx] + hook + src[idx:])
print("truth hook installed in env/observation_manager/obs_manager.py")
