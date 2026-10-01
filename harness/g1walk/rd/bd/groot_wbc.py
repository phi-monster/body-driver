# -*- coding: utf-8 -*-
# body-driver 第 40 条(路 8):会走的人形,厂商那一侧的走路控制器的另一份候选 —— NVIDIA GR00T-WholeBodyControl 的 Decoupled WBC 下半身策略
# (GR00T N1.5 / N1.6 用的那一份;github.com/NVlabs/GR00T-WholeBodyControl,代码 Apache-2.0,权重 NVIDIA Open Model License;
#  取下来、核 sha256 用 harness/g1walk/fetch_groot.py)。
#
# 推理:直接跑它的两份 ONNX(Balance:速度命令的模 ≤ 0.05 时用;Walk:其余时候),用 onnx 自带的参考实现(onnx.reference.ReferenceEvaluator,
# 纯 numpy;箱上没装 onnxruntime,也不往共用的环境里装)。两份都是一张图:估计器 516 → 35(3 + 32 归一化)+ 当前一帧,接 121 → 15 的 MLP。
# 接口照它自己的 MuJoCo 跑法(scripts/run_mujoco_gear_wbc.py)和配置(g1_gear_wbc.yaml),不自己定:
#   - 关节顺序:它的 MuJoCo 身体(g1_gear_wbc.xml)里浮动根以后那 29 个关节的顺序(左腿 6、右腿 6、腰 3、左胳膊 7、右胳膊 7);
#     输出的 15 个 = 前 15 个(两条腿 + 腰)。这里按名字到身体的关节体里去找。
#   - 一帧观测 86 个数:[速度命令 vx, vy, wz × cmd_scale, 胯高命令, 躯干 roll / pitch / yaw 命令, 机身系角速度 × ang_vel_scale,
#     机身系重力方向, 29 个关节 − 默认(腿、腰按 default_angles,胳膊 0)× dof_pos_scale, 29 个关节速度 × dof_vel_scale, 上一拍的 15 个原始输出];
#     6 帧历史,旧的在前;开局 6 帧都是 0(它的跑法就是 deque 里先放 6 个零,不是拿第一帧填满)。
#   - 目标 = 原始输出 × action_scale + default_angles;力矩 = kp(目标 − 位置) − kd·速度,每个物理步算(它的跑法:物理 0.005 s,每 4 步算一回策略)。
#   这一套和它的跑法逐项对:harness/g1walk/groot_check.py。
import os
import xml.etree.ElementTree as ET

import numpy as np
import torch
import yaml

BALANCE, WALK = "GR00T-WholeBodyControl-Balance.onnx", "GR00T-WholeBodyControl-Walk.onnx"


def xml_joints(path):
    """它的 MuJoCo 身体里的关节(浮动根不算):[(名字, 力矩上限)],按顺序;外加关节的默认 armature / damping / frictionloss"""
    r = ET.parse(path).getroot()
    default = {}
    for d in r.iter("default"):
        for c in d:
            if c.tag == "joint":
                default.update(c.attrib)
    out = []
    for j in r.iter("joint"):
        n = j.attrib.get("name")
        if n is None or j.attrib.get("type") == "free":
            continue
        lim = j.attrib.get("actuatorfrcrange")
        out.append((n, float(lim.split()[1]) if lim else None))
    return out, {k: float(v) for k, v in default.items()}


class GrootWBC:
    """厂商那一侧的走路控制器:每 1/50 s 调一次 step(),给两条腿、腰 15 个关节的目标位置"""

    def __init__(self, bundle, joint_names, num_envs=1, device="cpu"):
        from onnx.reference import ReferenceEvaluator
        c = yaml.safe_load(open(os.path.join(bundle, "g1_gear_wbc.yaml"), encoding="utf-8"))
        self.cfg = c
        joints, self.joint_default = xml_joints(os.path.join(bundle, "g1_gear_wbc.xml"))
        self.obs_names = [n for n, _ in joints]
        assert len(self.obs_names) == 29, len(self.obs_names)
        self.effort = {n: e for n, e in joints}
        n_act = int(c["num_actions"])
        self.act_names = self.obs_names[:n_act]
        self.kp = np.asarray(c["kps"], dtype=np.float32)
        self.kd = np.asarray(c["kds"], dtype=np.float32)
        d15 = np.asarray(c["default_angles"], dtype=np.float32)
        assert len(self.kp) == len(self.kd) == len(d15) == n_act
        self.history = int(c["obs_history_len"])
        assert int(c["num_obs"]) == 86 * self.history, (c["num_obs"], self.history)
        self.freq = 1.0 / (float(c["simulation_dt"]) * int(c["control_decimation"]))
        missing = [n for n in self.obs_names if n not in joint_names]
        assert not missing, f"身体里没有这几个关节:{missing}"
        self.obs_ids = [joint_names.index(n) for n in self.obs_names]
        self.leg_ids = [joint_names.index(n) for n in self.act_names]      # 叫 leg_ids 和 agile_vh 一样:这里是两条腿 + 腰
        self.leg_names = list(self.act_names)
        dev = torch.device(device)
        self.dev = dev
        pad = np.zeros(29, dtype=np.float32)
        pad[:n_act] = d15
        self.q0 = torch.as_tensor(pad, device=dev)
        self.d15 = torch.as_tensor(d15, device=dev)
        self.cmd_scale = torch.as_tensor(np.asarray(c["cmd_scale"], dtype=np.float32), device=dev)
        self.ang_vel_scale, self.dof_pos_scale = float(c["ang_vel_scale"]), float(c["dof_pos_scale"])
        self.dof_vel_scale, self.action_scale = float(c["dof_vel_scale"]), float(c["action_scale"])
        self.stand_h = float(c["height_cmd"])
        self.models = {"balance": ReferenceEvaluator(os.path.join(bundle, BALANCE)), "walk": ReferenceEvaluator(os.path.join(bundle, WALK))}
        self.hist = torch.zeros(num_envs, self.history, 86, device=dev)
        self.last = torch.zeros(num_envs, n_act, device=dev)
        self.calls = 0
        self.used = {"balance": 0, "walk": 0}

    def reset(self, env_ids=None):
        if env_ids is None:
            self.hist.zero_()
            self.last.zero_()
        else:
            self.hist[env_ids] = 0.0
            self.last[env_ids] = 0.0

    def frame(self, cmd3, height, rpy, ang_vel_b, gravity_b, joint_pos, joint_vel):
        return torch.cat([cmd3 * self.cmd_scale, height.reshape(-1, 1), rpy, ang_vel_b * self.ang_vel_scale, gravity_b,
                          (joint_pos[:, self.obs_ids] - self.q0) * self.dof_pos_scale, joint_vel[:, self.obs_ids] * self.dof_vel_scale,
                          self.last], dim=-1)

    @torch.no_grad()
    def step(self, cmd3, height, rpy, ang_vel_b, gravity_b, joint_pos, joint_vel):
        """cmd3 (N, 3) = [vx, vy, wz](不乘比例,它自己乘);height (N,) 胯高命令;rpy (N, 3) 躯干朝向命令;ang_vel_b、gravity_b (N, 3) 机身系;
        joint_pos、joint_vel (N, 身体全部关节,身体的顺序) ⇒ (N, 15) 两条腿 + 腰的目标位置(顺序 = self.leg_ids)"""
        f = self.frame(cmd3.float(), height.float(), rpy.float(), ang_vel_b.float(), gravity_b.float(), joint_pos.float(), joint_vel.float())
        self.hist.copy_(torch.cat([self.hist[:, 1:], f[:, None, :]], dim=1))
        obs = self.hist.reshape(self.hist.shape[0], -1).cpu().numpy().astype(np.float32)
        walking = (torch.linalg.norm(cmd3.float(), dim=-1) > 0.05).cpu().numpy()
        act = np.zeros((obs.shape[0], self.last.shape[1]), dtype=np.float32)
        for name, sel in (("balance", ~walking), ("walk", walking)):
            if sel.any():
                act[sel] = self.models[name].run(None, {"input": obs[sel]})[0]
                self.used[name] += int(sel.sum())
        a = torch.as_tensor(act, device=self.dev)
        self.last = a.clone()
        self.calls += 1
        return a * self.action_scale + self.d15

    def gains_ok(self, art):
        """身体上两条腿、腰的执行器 kp / kd、力矩上限和它要的一样没有:返回对不上的 [(关节, 身体的 kp, kd, 上限, 要的 kp, kd, 上限)]"""
        bad = []
        want = {n: (float(p), float(d), self.effort[n]) for n, p, d in zip(self.act_names, self.kp, self.kd)}
        for act in art.actuators.values():
            for j, n in enumerate(act.joint_names):
                if n in want:
                    got = (float(act.stiffness[0, j]), float(act.damping[0, j]), float(act.effort_limit[0, j]))
                    if any(abs(g - w) > 1e-4 for g, w in zip(got, want[n])):
                        bad.append((n,) + got + want[n])
        return bad
