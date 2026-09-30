# -*- coding: utf-8 -*-
"""chase_mouse 的老鼠走得太快(路 8,10-01):RoboDojo 一个动作 = collect_interval 个物理子步(= 1 /(dt × collect_freq)= 1 /(0.004 × 25)= 10),
任务的 step() 每个子步被调一次;原来的 _wander 每调一次就走一整个 BD_MOUSE_SPEED(它的注释写的是"每个 env step")
⇒ 一个动作走 10 cm,不是 PLAN / 大并行 §1 写的 1 cm/步;换方向"每 40 步"也成了每 4 个动作。另外老鼠被抓起来以后还照样被按平面拽着走。
改成:每个子步走 speed / collect_interval(一个动作正好 speed)、每 40 个动作换一次方向、离开它开局那张面 5 mm 以上就不走。
接在 g1_setup.py → patch_planner_key.py → lean_rig.py → root_lean.py → rig_upright_low.py 后面跑;可以重复跑(改过就不再改)。
用法:/venv/RoboDojo/bin/python fix_mouse_speed.py /root/RoboDojo      (只改 task/RoboDojo/tasks/chase_mouse.py 的 _wander 这一个方法)
"""
import re
import sys

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
p = f"{R}/task/RoboDojo/tasks/chase_mouse.py"
s = open(p, encoding="utf-8").read()
MARK = "# [bd 10-01] 每个子步走 speed / collect_interval"
if MARK in s:
    print("已改过", p)
    sys.exit(0)
NEW = '''    def _wander(self):
        ''' + MARK + '''(一个动作 = collect_interval 个物理子步,step() 每个子步调一次);
        # 每 40 个动作换一次方向;离开开局那张面 5 mm 以上(被拿起来了)就不走
        lm = self.scene_manager.layout_manager
        om = getattr(self, "obs_manager", None)
        sub = max(int(round(float(getattr(om, "collect_interval", 1.0) or 1.0))), 1) if om is not None else 1
        if self._mouse_step == 0 or not hasattr(self, "_mouse_z0"):
            self._mouse_z0 = {}
        for env_idx in range(self.num_envs):
            name = lm.get_instance_name(env_idx, "target")
            if name is None:
                continue
            obj = lm.get_scene_object(env_idx, name)
            if obj is None:
                continue
            pos, rot = obj.get_local_pose()
            p = np.array(pos.detach().cpu() if hasattr(pos, "detach") else pos, dtype=float).reshape(-1)[:3]
            z0 = self._mouse_z0.setdefault(env_idx, float(p[2]))
            if env_idx not in self._mouse_vel or self._mouse_step % (40 * sub) == 0:
                a = random.uniform(0, 2 * np.pi)
                self._mouse_vel[env_idx] = np.array([np.cos(a), np.sin(a)]) * self._speed
            if p[2] > z0 + 0.005:
                continue
            v = self._mouse_vel[env_idx]
            nx, ny = p[0] + v[0] / sub, p[1] + v[1] / sub
            (x0, x1), (y0, y1) = self._bounds
            if nx < x0 or nx > x1:
                v[0] = -v[0]
                nx = min(max(nx, x0), x1)
            if ny < y0 or ny > y1:
                v[1] = -v[1]
                ny = min(max(ny, y0), y1)
            q = rot.detach().cpu() if hasattr(rot, "detach") else rot
            obj.set_local_pose(translation=np.array([nx, ny, p[2]]), orientation=np.array(q, dtype=float).reshape(-1)[:4])
        self._mouse_step += 1

'''
m = re.search(r"    def _wander\(self\):\n.*?(?=    def step\(self, meta_control_list\):)", s, flags=re.S)
assert m, "chase_mouse.py 的样子变了,找不到 _wander"
s = s[:m.start()] + NEW + s[m.end():]
open(p, "w", encoding="utf-8").write(s)
print("改了", p)
