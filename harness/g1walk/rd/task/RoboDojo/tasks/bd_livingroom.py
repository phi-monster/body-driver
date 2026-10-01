# -*- coding: utf-8 -*-
# body-driver 第 40 条(路 8):会走的人形收拾大客厅。由 harness/g1walk 装进来,别手改。
# 身体:Isaac 资产库的 G1(g1walk,根不固定)。两条腿归厂商那一侧的走路控制器 —— Isaac Lab 自带的 Agile 下半身策略
# (ISAACLAB_NUCLEUS_DIR/Policies/Agile/agile_locomotion.pt):收 [vx, vy, wz, 胯高],每 0.02 s(50 Hz,RoboDojo 物理 250 Hz ⇒ 每 5 个子步)
# 出两条腿 12 个关节的目标。输入照 Isaac Lab 那一份拼(locomanipulation/pick_place 的 AgileTeacherPolicyObservationsCfg + AgileBasedLowerBodyAction):
#   [命令 4, 机身线速度 3(机身系), 角速度 3, 重力在机身系的方向 3, 29 个身体关节 − 默认, (关节速度 − 默认) × 0.1, 上一拍的输出 12],
#   关节按关节体里的顺序挑(和 Isaac Lab 的 find_joints 一样);输出 × 0.25 + 默认 = 腿的目标。
# 走路控制器的命令那一组在身体报的读数里叫 base_cmd_joint_state(4 个:机身系的 vx、vy、wz 量到的,和骨盆离地多高),
# 收的命令也叫 base_cmd_joint_state(4 个:[vx, vy, wz, 胯高])—— 这一组怎么叫、怎么报,等路 7 的身体协议文档定了照它改。
# 判据:task/RoboDojo/bd/tidy.py 的 bd_tidy(每一件都放到它该去的地方)。一集 30 分钟 = 45000 个动作(25 Hz;BD_STEP_LIM 可改)。
import os

import numpy as np
import torch

from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager
from task.RoboDojo.bd import rig, scene, tidy

rig.register("g1walk", "g1walk", ["G1WalkLeft", "G1WalkRight"])
rig.no_planner("g1walk")
rig.dims("g1walk")

OBS_JOINTS = [".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_.*_joint", ".*_hip_.*_joint", ".*_knee_joint", ".*_ankle_.*_joint", "waist_.*_joint"]
LEG_JOINTS = [".*_hip_.*_joint", ".*_knee_joint", ".*_ankle_.*_joint"]
CMD_KEY = "base_cmd_joint_state"
STAND = [0.0, 0.0, 0.0, 0.72]    # 站着不动:Isaac Lab 自己给 G1 的胯高(g1_lower_body_standing.py 的 hip_height)


class WalkController:
    """厂商那一侧的走路控制器(每个物理子步 tick 一次,每 decim 个子步算一回策略)"""

    def __init__(self, env):
        from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR, retrieve_file_path
        from isaaclab.utils.io.torchscript import load_torchscript_model
        self.env = env
        self.art = env.robot_manager.robot_key[0]
        dev = self.art.device
        self.policy = load_torchscript_model(retrieve_file_path(f"{ISAACLAB_NUCLEUS_DIR}/Policies/Agile/agile_locomotion.pt"), device=dev)
        self.obs_ids, _ = self.art.find_joints(OBS_JOINTS)
        self.leg_ids, _ = self.art.find_joints(LEG_JOINTS)
        self.q0 = self.art.data.default_joint_pos.clone()
        self.qd0 = self.art.data.default_joint_vel.clone()
        self.last = torch.zeros(self.q0.shape[0], len(self.leg_ids), device=dev)
        self.decim = max(1, int(round(0.02 / float(env.dt))))
        self.sub = 0

    def reset(self):
        self.last.zero_()
        self.sub = 0

    def tick(self, cmd):
        if self.sub % self.decim == 0:
            d = self.art.data
            c = torch.tensor(cmd, dtype=torch.float32, device=self.last.device).unsqueeze(0).repeat(self.last.shape[0], 1)
            x = torch.cat([c, d.root_lin_vel_b, d.root_ang_vel_b, d.projected_gravity_b,
                           (d.joint_pos - self.q0)[:, self.obs_ids], 0.1 * (d.joint_vel - self.qd0)[:, self.obs_ids], self.last], dim=-1)
            with torch.inference_mode():
                raw = self.policy(x)
            self.last = raw.clone()
            self.art.set_joint_position_target(raw * 0.25 + self.q0[:, self.leg_ids], joint_ids=self.leg_ids)
        self.sub += 1

    def reading(self, env_idx=0):
        """走路那一组的读数:机身系量到的 vx、vy、wz,骨盆离地多高"""
        d = self.art.data
        z = float(d.root_pos_w[env_idx, 2] - self.env.scene_manager.env_origins[env_idx][2]) - FLOOR_Z
        v, w = d.root_lin_vel_b[env_idx].cpu().numpy(), d.root_ang_vel_b[env_idx].cpu().numpy()
        return [float(v[0]), float(v[1]), float(w[2]), z]


FLOOR_Z = 0.05   # 地面 = RoboDojo 布局里 Ground 那一块的中心 + 半厚(安装时按布局核过,见 harness/g1walk/install.py)


def _patch_eval_env(cls):
    """评测环境:动作里 base_cmd_joint_state 那一项先拿下来给走路控制器(RoboDojo 核动作时不认它),读数里加上这一组"""
    if getattr(cls, "_bd_walk_patched", False):
        return
    orig_take, orig_obs = cls.take_action, cls.get_obs_batch

    def take_action(self, action):
        a = dict(action)
        cmd = a.pop(CMD_KEY, None)
        if cmd is not None:
            self.bd_cmd = [float(v) for v in np.asarray(cmd, dtype=float).reshape(-1)[:4]]
        return orig_take(self, a)

    def get_obs_batch(self, env_idx_list=None, last_frame=False):
        out = orig_obs(self, env_idx_list=env_idx_list, last_frame=last_frame)
        if getattr(self, "bd_walk", None) is not None:
            for d in out:
                e = int(d.get("env_idx", 0))
                d.setdefault("state", {})[CMD_KEY] = np.asarray(self.bd_walk.reading(e), dtype=np.float32)
                d.setdefault("action", {})[CMD_KEY] = np.asarray(self.bd_cmd, dtype=np.float32)
        return out

    cls.take_action, cls.get_obs_batch = take_action, get_obs_batch
    cls._bd_walk_patched = True


class BdLivingroomCommon:
    """大客厅:几十件东西各放到该去的地方"""

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = int(os.environ.get("BD_STEP_LIM", "45000"))
        self.bd_cmd = list(STAND)
        self.bd_walk = None
        _patch_eval_env(type(self))
        # RoboDojo 开局核"布局稳不稳"要白走 300 个物理子步(1.2 s),那时走路控制器还没接上、人形站不稳;这一集的东西全不让它核
        # (布局里 need_check_stable 都是 False,核也是白核),这一关在这具身体这一集里跳过
        self.scene_manager.layout_manager.check_layout_stability = lambda env, render=False: (True, [])

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)
        scene.install_checks(self.reward_manager.func_parser)
        tidy.install_checks(self.reward_manager.func_parser)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()
        self.bd_cmd = list(STAND)
        if self.bd_walk is None:
            self.bd_walk = WalkController(self)
        self.bd_walk.reset()

    def step(self, meta_control_list):
        if self.bd_walk is not None:
            self.bd_walk.tick(self.bd_cmd)
        super().step(meta_control_list)

    def run_reward(self):
        self.reward_manager.check([("bd_tidy", {})])

    def gen_instruction(self, env_idx):
        return [tidy.find_tidy(self.scene_manager.layout_manager.saved_layouts[env_idx]).get("sentence", "")]


class bd_livingroom(BdLivingroomCommon, TaskEnv):
    pass
