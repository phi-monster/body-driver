# 把 bd_camtest 钩子接进 RoboDojo 的 get_obs(幂等:接过就不再接)。用法:/venv/RoboDojo/bin/python patch_camtest.py /root/RoboDojo
import sys, shutil, os
root = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
om = os.path.join(root, "env/observation_manager/obs_manager.py")
shutil.copy(os.path.join(os.path.dirname(os.path.abspath(__file__)), "bd_camtest.py"), os.path.join(root, "env/observation_manager/bd_camtest.py"))
s = open(om, encoding="utf-8").read()
mark = "# body-driver camtest hook"
if mark in s:
    print("已经接过"); sys.exit(0)
anchor = "            if self.collect_intrinsic_matrix or self.collect_extrinsic_matrix:\n"
assert s.count(anchor) == 1, "get_obs 的样子变了,钩子没接"
hook = ("            " + mark + "(V1 测试:头顶眼被转了 / 被挡了一半;触发文件 /root/camtest.json)\n"
        "            try:\n"
        "                if not hasattr(self, '_bd_camtest'):\n"
        "                    import importlib.util as _ilu, os as _os\n"
        "                    _sp = _ilu.spec_from_file_location('bd_camtest', _os.path.join(_os.path.dirname(__file__), 'bd_camtest.py'))\n"
        "                    self._bd_camtest = _ilu.module_from_spec(_sp)\n"
        "                    _sp.loader.exec_module(self._bd_camtest)\n"
        "                self._bd_camtest.apply(self, obs, env_idx_list)\n"
        "            except Exception as _e:\n"
        "                print('[camtest] 钩子出错:', _e, flush=True)\n")
s = s.replace(anchor, hook + anchor)
open(om, "w", encoding="utf-8").write(s)
print("接上了:", om)
