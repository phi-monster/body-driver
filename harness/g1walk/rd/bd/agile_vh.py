# -*- coding: utf-8 -*-
# body-driver 第 40 条(路 8):会走的人形,厂商那一侧的走路控制器 —— NVIDIA WBC-AGILE 的 G1 下半身策略 velocity_height_g1
# (github.com/nvidia-isaac/WBC-AGILE,Apache-2.0;取下来、核 sha256 用 harness/g1walk/fetch_agile.py)。由 harness/g1walk 装进来,别手改。
#
# 推理只用它的 TorchScript(unitree_g1_velocity_height_history_torchscript.pt:400 → 512 → 256 → 128 → 12 的 MLP,ELU,不带归一化)。
# 接口照它自己的 LEAPP 说明(Velocity-Height-G1-History-v0.yaml)和同一个策略导出的整张 ONNX 图来,不自己定:
#   - 关节顺序:yaml 的 robot_joint_pos / robot_joint_vel 写明了 29 个身体关节的顺序,输出 joint_pos 写明了 12 个腿关节的顺序;
#     这里按名字到身体的关节体里去找(身体多出来的手指关节不进策略)。
#   - 默认姿势、kp / kd、频率(50 Hz)、历史长度(5 帧):yaml 里读(agile.articulations.robot、pipeline.configs.frequency、h_* 的形状)。
#   - yaml 里没写、只在 ONNX 图里的几个数:关节速度乘的 0.1、上一拍动作进观测前夹到 ±10、输出怎么换成目标
#     (目标 = 夹到 ±6(原始输出 × 每个关节的比例 + 默认)),都顺着 ONNX 图从具名的输入、输出往回找出来(onnx_constants),不抄数。
#   - 观测一拍 80 个数,按训练时的顺序 [命令 4(vx, vy, wz, 骨盆高), 机身系角速度 3, 机身系重力方向 3, 29 个关节 − 默认, 29 个关节速度 × 0.1,
#     上一拍的原始输出 12];每一项各存 5 帧(旧的在前),各自摊平以后按上面的顺序接起来 = 400。复位以后第一拍拿这一拍把 5 帧填满
#     (Isaac Lab 的 CircularBuffer 也是这样;ONNX 附带的初值文件 5 帧也都一样)。
#   这一整套和 ONNX 一个数一个数地对过:harness/g1walk/agile_check.py。
# 命令的意思照它训练时的命令生成器(agile/rl_env/mdp/commands,velocity_height_env_cfg.py 的 CommandsCfg):vx −0.5 ~ 1.5 m/s、vy ±0.5、
# wz ±1 rad/s、骨盆高 0.4 ~ 0.72 m;random_height_during_walking=False —— 训练时只要在走,骨盆高就被改回 0.72,蹲只在站着(速度 0)时练过。
import os

import numpy as np
import torch
import yaml

MODEL = "Velocity-Height-G1-History-v0"
TORCHSCRIPT = "unitree_g1_velocity_height_history_torchscript.pt"
TERMS = ["commands", "ang_vel", "gravity", "joint_pos", "joint_vel", "actions"]   # 一拍观测里各项的顺序(= ONNX 的 cat 那一步)


def onnx_constants(path):
    """顺着 ONNX 图从具名的输入、输出往回找:关节默认位置 / 默认速度 / 速度比例、上一拍动作的夹子、输出的比例 / 偏移 / 夹子、kp / kd"""
    import onnx
    from onnx import numpy_helper
    g = onnx.load(path).graph
    init = {t.name: numpy_helper.to_array(t) for t in g.initializer}
    made_by = {o: n for n in g.node for o in n.output}

    def const(name):
        if name in init:
            return np.asarray(init[name], dtype=np.float32)
        n = made_by[name]
        assert n.op_type == "Constant", (name, n.op_type)
        return np.asarray(numpy_helper.to_array(n.attribute[0].t), dtype=np.float32)

    def user(inp, op):
        hits = [n for n in g.node if n.op_type == op and n.input and n.input[0] == inp]
        assert len(hits) == 1, (inp, op, len(hits))
        return hits[0]

    c = {}
    c["q0"] = const(user("robot_joint_pos", "Sub").input[1]).reshape(-1)
    s = user("robot_joint_vel", "Sub")
    c["qd0"] = const(s.input[1]).reshape(-1)
    c["vel_scale"] = float(const(user(s.output[0], "Mul").input[1]))
    k = user("last_action_in", "Clip")
    c["act_obs_clip"] = (float(const(k.input[1])), float(const(k.input[2])))
    n_min = made_by["joint_pos"]
    n_max = made_by[n_min.input[0]]
    n_add = made_by[n_max.input[0]]
    n_mul = made_by[n_add.input[0]]
    assert (n_min.op_type, n_max.op_type, n_add.op_type, n_mul.op_type) == ("Min", "Max", "Add", "Mul") and n_mul.input[0] == "last_action_out"
    c["scale"], c["offset"] = const(n_mul.input[1]).reshape(-1), const(n_add.input[1]).reshape(-1)
    c["lo"], c["hi"] = const(n_max.input[1]).reshape(-1), const(n_min.input[1]).reshape(-1)
    c["kp"], c["kd"] = const("joint_pos_kp_gains").reshape(-1), const("joint_pos_kd_gains").reshape(-1)
    return c


class AgileVH:
    """厂商那一侧的走路控制器:每 1/50 s 调一次 step(),给两条腿 12 个关节的目标位置"""

    def __init__(self, bundle, joint_names, num_envs=1, device="cpu"):
        y = yaml.safe_load(open(os.path.join(bundle, MODEL + ".yaml"), encoding="utf-8"))
        m = y["models"][MODEL]
        ins = {i["name"]: i for i in m["inputs"]}
        outs = {o["name"]: o for o in m["outputs"]}
        self.obs_names = list(ins["robot_joint_pos"]["element_names"][0])
        assert list(ins["robot_joint_vel"]["element_names"][0]) == self.obs_names
        self.leg_names = list(outs["joint_pos"]["element_names"][0])
        for kk in ("joint_pos_kp_gains", "joint_pos_kd_gains"):
            assert list(outs[kk]["element_names"][0]) == self.leg_names
        self.history = int(ins["h_policy_joint_pos_in"]["shape"][0])
        self.freq = float(y["pipeline"]["configs"]["frequency"])
        robot = y["agile"]["articulations"]["robot"]
        assert list(robot["joint_names"]) == self.obs_names
        c = onnx_constants(os.path.join(bundle, MODEL + ".onnx"))
        # yaml 写的默认姿势、kp / kd 和 ONNX 图里用的是同一套
        assert np.allclose(c["q0"], np.asarray(robot["default_joint_pos"], dtype=np.float32))
        kp = dict(zip(self.obs_names, robot["default_joint_stiffness"]))
        kd = dict(zip(self.obs_names, robot["default_joint_damping"]))
        assert np.allclose(c["kp"], [kp[n] for n in self.leg_names]) and np.allclose(c["kd"], [kd[n] for n in self.leg_names])
        self.const = c
        missing = [n for n in self.obs_names if n not in joint_names]
        assert not missing, f"身体里没有这几个关节:{missing}"
        self.obs_ids = [joint_names.index(n) for n in self.obs_names]
        self.leg_ids = [joint_names.index(n) for n in self.leg_names]
        dev = torch.device(device)
        t = lambda a: torch.as_tensor(np.array(a, dtype=np.float32), device=dev)
        self.q0, self.qd0 = t(c["q0"]), t(c["qd0"])
        self.vel_scale = c["vel_scale"]
        self.a_lo, self.a_hi = c["act_obs_clip"]
        self.scale, self.offset, self.lo, self.hi = t(c["scale"]), t(c["offset"]), t(c["lo"]), t(c["hi"])
        self.kp, self.kd = c["kp"], c["kd"]
        self.policy = torch.jit.load(os.path.join(bundle, TORCHSCRIPT), map_location=dev).eval()
        n_act = int(ins["last_action_in"]["shape"][-1])
        dims = [int(ins[nm]["shape"][-1]) for nm in ("base_velocity", "robot_root_ang_vel_b", "h_policy_projected_gravity_in",
                                                     "robot_joint_pos", "robot_joint_vel", "last_action_in")]
        assert sum(dims) * self.history == getattr(self.policy.mlp, "0").weight.shape[1], (dims, self.history)
        self.hist = [torch.zeros(num_envs, self.history, d, device=dev) for d in dims]
        self.fresh = torch.ones(num_envs, dtype=torch.bool, device=dev)
        self.last = torch.zeros(num_envs, n_act, device=dev)
        self.calls = 0

    def reset(self, env_ids=None):
        """复位以后:上一拍动作清零,下一拍拿那一拍的观测把 5 帧填满"""
        if env_ids is None:
            self.fresh[:] = True
            self.last.zero_()
        else:
            self.fresh[env_ids] = True
            self.last[env_ids] = 0.0

    def frame(self, cmd, ang_vel_b, gravity_b, joint_pos, joint_vel):
        return [cmd, ang_vel_b, gravity_b, joint_pos[:, self.obs_ids] - self.q0,
                (joint_vel[:, self.obs_ids] - self.qd0) * self.vel_scale, self.last.clamp(self.a_lo, self.a_hi)]

    @torch.no_grad()
    def step(self, cmd, ang_vel_b, gravity_b, joint_pos, joint_vel):
        """cmd (N, 4) = [vx, vy, wz, 骨盆高];ang_vel_b、gravity_b (N, 3) 机身系;joint_pos、joint_vel (N, 身体全部关节,身体的顺序)
        ⇒ (N, 12) 腿关节的目标位置(顺序 = self.leg_ids)"""
        f = self.frame(cmd.float(), ang_vel_b.float(), gravity_b.float(), joint_pos.float(), joint_vel.float())
        fill = self.fresh[:, None, None]
        for h, x in zip(self.hist, f):
            h.copy_(torch.where(fill, x[:, None, :].expand_as(h), torch.cat([h[:, 1:], x[:, None, :]], dim=1)))
        self.fresh[:] = False
        obs = torch.cat([h.reshape(h.shape[0], -1) for h in self.hist], dim=-1)
        raw = self.policy(obs)
        self.last = raw.clone()
        self.calls += 1
        return torch.maximum(torch.minimum(raw * self.scale + self.offset, self.hi), self.lo)

    def gains_ok(self, art):
        """身体腿上的执行器 kp / kd 和策略要的一样没有(一个关节一个关节对):返回对不上的 [(关节, 身体的 kp, kd, 要的 kp, kd)]"""
        bad = []
        want = {n: (p, d) for n, p, d in zip(self.leg_names, self.kp, self.kd)}
        for act in art.actuators.values():
            names = list(act.joint_names)
            for j, n in enumerate(names):
                if n in want:
                    p, d = float(act.stiffness[0, j]), float(act.damping[0, j])
                    if abs(p - want[n][0]) > 1e-4 or abs(d - want[n][1]) > 1e-4:
                        bad.append((n, p, d) + want[n])
        return bad
