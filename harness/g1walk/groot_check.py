# -*- coding: utf-8 -*-
"""第 40 条(路 8):我们接的 GR00T Decoupled WBC(rd/bd/groot_wbc.py)和它自己的 MuJoCo 跑法(run_mujoco_gear_wbc.py)一个数一个数地对。
它的跑法要 mujoco、pynput,这里不装:从它的源文件里把 GearWbcController 的 compute_observation / quat_rotate_inverse / get_gravity_orientation
原样摘出来(ast),用一个假的 MuJoCo data(qpos / qvel 随机给)喂;历史、两份策略怎么换、输出怎么换成目标,照它 run() 里那几行。
两边喂同一串读数(关节在它那边按它的 XML 顺序、在我们这边按打乱了的身体顺序,再加 14 个手指关节),每一拍比一帧观测和 15 个目标。
不进仿真、不占卡。用法(箱上):/venv/RoboDojo/bin/python groot_check.py <fetch_groot.py 放文件的目录>
"""
import ast
import os
import sys
import types

import numpy as np
import torch
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "rd", "bd"))
import groot_wbc  # noqa: E402

B = sys.argv[1] if len(sys.argv) > 1 else "/root/p8/groot"
rng = np.random.default_rng(0)

# ---- 它的代码原样摘出来
src = open(os.path.join(B, "run_mujoco_gear_wbc.py"), encoding="utf-8").read()
tree = ast.parse(src)
cls = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "GearWbcController"][0]
keep = [f for f in cls.body if isinstance(f, ast.FunctionDef) and f.name in ("compute_observation", "quat_rotate_inverse", "get_gravity_orientation")]
assert len(keep) == 3
stub = ast.Module(body=[ast.ClassDef(name="Theirs", bases=[], keywords=[], body=keep, decorator_list=[])], type_ignores=[])
ns = {"np": np}
exec(compile(ast.fix_missing_locations(stub), "run_mujoco_gear_wbc.py(摘出来的三个方法)", "exec"), ns)
theirs = ns["Theirs"]()

cfg = yaml.safe_load(open(os.path.join(B, "g1_gear_wbc.yaml")))
for k in ["kps", "kds", "default_angles", "cmd_scale", "cmd_init"]:   # 它的 load_config 里同样转成数组
    cfg[k] = np.array(cfg[k], dtype=np.float32)
n_joints = 29
# 它算躯干那几项要 self.data.xquat / xmat / cvel(算出来不进观测),给个占位
theirs.data = types.SimpleNamespace(xquat=np.tile([1.0, 0, 0, 0], (64, 1)), xmat=np.tile(np.eye(3).reshape(9), (64, 1)), cvel=np.zeros((64, 6)))
theirs.torso_index = 0

names29 = [n for n, _ in groot_wbc.xml_joints(os.path.join(B, "g1_gear_wbc.xml"))[0]]
hand = [f"{s}_hand_{f}_{i}_joint" for s in ("left", "right") for f, n in (("index", 2), ("middle", 2), ("thumb", 3)) for i in range(n)]
body = names29 + hand
rng.shuffle(body)
ctrl = groot_wbc.GrootWBC(B, body, num_envs=1)
print("[check] 它的 XML 里的关节顺序:", names29)
print("[check] 输出的 15 个:", ctrl.act_names)
print("[check] kp", ctrl.kp.tolist(), "· kd", ctrl.kd.tolist())
print("[check] 力矩上限", [ctrl.effort[n] for n in ctrl.act_names], "· 关节默认", ctrl.joint_default)
print("[check] 比例:命令", ctrl.cmd_scale.tolist(), "角速度", ctrl.ang_vel_scale, "关节", ctrl.dof_pos_scale, "关节速度", ctrl.dof_vel_scale,
      "输出", ctrl.action_scale, "· 历史 %d 帧 · %.0f Hz · 开局胯高命令 %.2f" % (ctrl.history, ctrl.freq, ctrl.stand_h))

perm = [body.index(n) for n in names29]
hist = [np.zeros(86, dtype=np.float32)] * cfg["obs_history_len"]
their_action = np.zeros(cfg["num_actions"], dtype=np.float32)
worst = {"一帧观测": 0.0, "目标": 0.0}
switch = {"balance": 0, "walk": 0}
for t in range(60):
    walking = t % 3 != 0
    cmd = np.array([rng.uniform(-0.6, 0.8), rng.uniform(-0.4, 0.4), rng.uniform(-0.8, 0.8)], dtype=np.float32) if walking else \
        np.array([0.01, -0.02, 0.01], dtype=np.float32)          # 模 ≤ 0.05 ⇒ 站着那一份
    height = float(rng.uniform(0.3, 0.8))
    rpy = np.array([rng.uniform(-0.3, 0.3), rng.uniform(-0.5, 0.5), rng.uniform(-0.3, 0.3)], dtype=np.float32)
    q = rng.normal(0, 1, 4)
    q[1:3] *= 0.2
    q = q / np.linalg.norm(q)                                   # w, x, y, z(MuJoCo 和 Isaac Lab 都是 w 在前)
    omega = rng.normal(0, 1.0, 3)                               # 机身系角速度(MuJoCo 浮动根 qvel[3:6] 就是机身系的)
    qj = (np.r_[ctrl.d15.numpy(), np.zeros(14)] + rng.normal(0, 0.3, 29)).astype(np.float64)
    dqj = rng.normal(0, 2.0, 29)
    d = types.SimpleNamespace(qpos=np.r_[[0.0, 0.0, 0.75], q, qj], qvel=np.r_[[0.0, 0.0, 0.0], omega, dqj])
    control = {"loco_cmd": cmd.copy(), "height_cmd": height, "rpy_cmd": rpy.copy(), "freq_cmd": 1.0}
    single, _ = theirs.compute_observation(d, cfg, their_action, control, n_joints)
    hist = hist[1:] + [single]
    obs = np.concatenate(hist).astype(np.float32)[None]
    which = "balance" if np.linalg.norm(np.array(control["loco_cmd"])) <= 0.05 else "walk"
    switch[which] += 1
    their_action = ctrl.models[which].run(None, {"input": obs})[0].squeeze()
    their_target = their_action * cfg["action_scale"] + cfg["default_angles"]
    # 我们这边:Isaac Lab 的量 —— 重力方向用 Isaac Lab 2.x 的 quat_apply_inverse(w 在前)
    w_, xyz = q[0], q[1:]
    g = np.array([0.0, 0.0, -1.0])
    tt = 2.0 * np.cross(xyz, g)
    grav = g - w_ * tt + np.cross(xyz, tt)
    jp = np.zeros((1, len(body)), np.float32)
    jv = np.zeros((1, len(body)), np.float32)
    jp[0, perm], jv[0, perm] = qj, dqj
    jp[0, [body.index(n) for n in hand]] = rng.normal(0, 1, len(hand))
    f_ours = ctrl.frame(torch.tensor(cmd)[None], torch.tensor([height]), torch.tensor(rpy)[None], torch.tensor(omega, dtype=torch.float32)[None],
                        torch.tensor(grav, dtype=torch.float32)[None], torch.tensor(jp), torch.tensor(jv)).numpy()[0]
    tgt = ctrl.step(torch.tensor(cmd)[None], torch.tensor([height]), torch.tensor(rpy)[None], torch.tensor(omega, dtype=torch.float32)[None],
                    torch.tensor(grav, dtype=torch.float32)[None], torch.tensor(jp), torch.tensor(jv)).numpy()[0]
    worst["一帧观测"] = max(worst["一帧观测"], float(np.abs(f_ours - single).max()))
    worst["目标"] = max(worst["目标"], float(np.abs(tgt - their_target).max()))
    if t == 1:
        print("[check] 第 2 拍它的目标:", np.round(their_target, 4).tolist())
        print("[check] 第 2 拍我们的目标:", np.round(tgt, 4).tolist())
print("[check] 60 拍(站着那一份 %d 拍、走的那一份 %d 拍;我们这边 %s),两边差得最多的:" % (switch["balance"], switch["walk"], ctrl.used),
      {k: float("%.2e" % v) for k, v in worst.items()})
ok = max(worst.values()) < 1e-4 and switch == ctrl.used
print("[check] %s" % ("一样" if ok else "🔴 对不上"))
sys.exit(0 if ok else 1)
