# -*- coding: utf-8 -*-
"""第 40 条(路 8):我们自己拼的那一套(rd/bd/agile_vh.py:TorchScript + 观测 / 历史 / 动作换算)和厂商导出的整张 ONNX 图一个数一个数地对。
ONNX 用 onnx 自带的参考实现(onnx.reference.ReferenceEvaluator,纯 numpy)跑,不装 onnxruntime。
两边从同一份历史初值(ONNX 附带的 _initial_values.safetensors)起,喂同一串原始读数(命令、角速度、根的朝向、29 个关节的位置 / 速度),
每一拍比:12 个腿关节的目标、6 项历史、上一拍动作;再比一回"复位后第一拍拿这一拍填满 5 帧"。身体的关节顺序故意打乱、加上 14 个手指关节,
核按名字找关节那一步。不进仿真、不占卡。
用法(箱上):/venv/RoboDojo/bin/python agile_check.py [velocity_height_g1 那几个文件放在哪儿;默认 install.py 放的地方]
"""
import json
import os
import struct
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "rd", "bd"))
import agile_vh  # noqa: E402

B = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo/Assets/Robots/g1walk/agile_velocity_height_g1"
M = agile_vh.MODEL
rng = np.random.default_rng(0)


def safetensors(path):
    b = open(path, "rb").read()
    n = struct.unpack("<Q", b[:8])[0]
    h = json.loads(b[8:8 + n])
    out = {}
    for k, v in h.items():
        if k == "__metadata__":
            continue
        s, e = v["data_offsets"]
        out[k.split("/")[-1]] = np.frombuffer(b[8 + n + s:8 + n + e], dtype=np.float32).reshape(v["shape"]).copy()
    return out


def quat_apply_inverse_wxyz(q, v):
    """Isaac Lab 2.x 的 quat_apply_inverse(四元数 w 在前)"""
    w, xyz = q[..., :1], q[..., 1:]
    t = 2.0 * np.cross(xyz, v)
    return v - w * t + np.cross(xyz, t)


from onnx.reference import ReferenceEvaluator  # noqa: E402

sess = ReferenceEvaluator(os.path.join(B, M + ".onnx"))
init = safetensors(os.path.join(B, M + "_initial_values.safetensors"))
hand = [f"{s}_hand_{f}_{i}_joint" for s in ("left", "right") for f, n in (("index", 2), ("middle", 2), ("thumb", 3)) for i in range(n)]
# 身体的关节:29 个 + 14 个手指,顺序打乱
import yaml  # noqa: E402
names29 = {i["name"]: i for i in yaml.safe_load(open(os.path.join(B, M + ".yaml")))["models"][M]["inputs"]}["robot_joint_pos"]["element_names"][0]
body = list(names29) + hand
rng.shuffle(body)
ctrl = agile_vh.AgileVH(B, body, num_envs=1, device="cpu")
c = ctrl.const
print("[check] 从 ONNX 图里找到的数:")
print("  关节默认位置(29,yaml 顺序):", np.round(c["q0"], 4).tolist())
print("  关节速度:减", np.unique(c["qd0"]).tolist(), "再乘", c["vel_scale"], "· 上一拍动作进观测前夹到", c["act_obs_clip"])
print("  输出 → 目标:比例", np.round(c["scale"], 5).tolist())
print("              偏移", np.round(c["offset"], 4).tolist(), "· 夹到", float(c["lo"].min()), "~", float(c["hi"].max()))
print("  kp", c["kp"].tolist(), "\n  kd", np.round(c["kd"], 4).tolist())
print("  频率 %.0f Hz · 历史 %d 帧 · 腿关节顺序 %s" % (ctrl.freq, ctrl.history, ctrl.leg_names))

order = ["velocity_height_commands", "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "actions"]
# 两边从同一份历史初值起
for h, k in zip(ctrl.hist, order):
    h.copy_(torch.as_tensor(init["h_policy_%s_in" % k]).permute(1, 0, 2))
ctrl.fresh[:] = False
ctrl.last.copy_(torch.as_tensor(init["last_action_in"]))
feed = {"h_policy_%s_in" % k: init["h_policy_%s_in" % k] for k in order}
feed["last_action_in"] = init["last_action_in"]
perm = [body.index(n) for n in names29]
worst = {"目标": 0.0, "上一拍动作": 0.0, **{"历史 " + k: 0.0 for k in order}}
T = 40
for t in range(T):
    cmd = np.array([[rng.uniform(-0.5, 1.5), rng.uniform(-0.5, 0.5), rng.uniform(-1, 1), rng.uniform(0.4, 0.72)]], dtype=np.float32)
    w = rng.normal(0, 1.0, (1, 3)).astype(np.float32)
    q = rng.normal(0, 1, 4)
    q[1:3] *= 0.15                          # 歪一点、朝向随便
    q = (q / np.linalg.norm(q)).astype(np.float32)[None]          # w, x, y, z
    qxyzw = np.concatenate([q[:, 1:], q[:, :1]], axis=1)
    g = quat_apply_inverse_wxyz(q.astype(np.float64), np.array([[0.0, 0.0, -1.0]])).astype(np.float32)
    jp29 = (c["q0"] + rng.normal(0, 0.3, 29)).astype(np.float32)[None]
    jv29 = rng.normal(0, 2.0, (1, 29)).astype(np.float32)
    jp = np.zeros((1, len(body)), np.float32)
    jv = np.zeros((1, len(body)), np.float32)
    jp[0, perm], jv[0, perm] = jp29[0], jv29[0]
    jp[0, [body.index(n) for n in hand]] = rng.normal(0, 1, len(hand))   # 手指关节随便给,不该进策略
    feed.update({"base_velocity": cmd, "robot_root_ang_vel_b": w, "robot_root_quat_w": qxyzw, "robot_joint_pos": jp29, "robot_joint_vel": jv29})
    out = dict(zip(sess.output_names, sess.run(None, feed)))
    tgt = ctrl.step(torch.as_tensor(cmd), torch.as_tensor(w), torch.as_tensor(g), torch.as_tensor(jp), torch.as_tensor(jv)).numpy()
    worst["目标"] = max(worst["目标"], float(np.abs(tgt - out["joint_pos"]).max()))
    worst["上一拍动作"] = max(worst["上一拍动作"], float(np.abs(ctrl.last.numpy() - out["last_action_out"]).max()))
    for h, k in zip(ctrl.hist, order):
        worst["历史 " + k] = max(worst["历史 " + k], float(np.abs(h.numpy() - out["h_policy_%s_out" % k].transpose(1, 0, 2)).max()))
    if t == 0:
        print("[check] 第 1 拍 ONNX 的目标:", np.round(out["joint_pos"][0], 4).tolist())
        print("[check] 第 1 拍我们的目标:", np.round(tgt[0], 4).tolist())
    for k in order:
        feed["h_policy_%s_in" % k] = out["h_policy_%s_out" % k]
    feed["last_action_in"] = out["last_action_out"]
assert np.allclose(out["joint_pos_kp_gains"], c["kp"]) and np.allclose(out["joint_pos_kd_gains"], c["kd"])
print("[check] %d 拍,两边差得最多的(绝对值):" % T, {k: float("%.2e" % v) for k, v in worst.items()})

# 复位后第一拍:我们拿这一拍填满 5 帧;ONNX 这边把这一拍处理好的值(从 ONNX 自己出的那一帧取)摆满 5 帧再跑,两边该一样
ctrl.reset()
feed2 = dict(feed)
feed2["last_action_in"] = np.zeros_like(feed["last_action_in"])
probe = sess.run(None, feed2)
probe = dict(zip(sess.output_names, probe))
for k in order:
    feed2["h_policy_%s_in" % k] = np.repeat(probe["h_policy_%s_out" % k][-1:], ctrl.history, axis=0)
out2 = dict(zip(sess.output_names, sess.run(None, feed2)))
tgt2 = ctrl.step(torch.as_tensor(cmd), torch.as_tensor(w), torch.as_tensor(g), torch.as_tensor(jp), torch.as_tensor(jv)).numpy()
d2 = float(np.abs(tgt2 - out2["joint_pos"]).max())
print("[check] 复位后第一拍(5 帧填满)两边目标差得最多:%.2e" % d2)
ok = max(worst.values()) < 1e-4 and d2 < 1e-4
print("[check] %s" % ("一样" if ok else "🔴 对不上"))
sys.exit(0 if ok else 1)
