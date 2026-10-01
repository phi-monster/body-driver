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
# 走路控制器挂在每一个物理子步上(_hook_physics);评测环境把场景全摆好以后,再把人形摆回站着的样子、物理走到停(bd_stand_and_settle)。
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
        self.calls, self.last_cmd, self.last_out = 0, None, None   # 离线核看:策略算了几回、最后一回用的命令、输出多大

    def reset(self):
        self.last.zero_()
        self.sub = 0

    def stand_up(self):
        """把人形摆回开局站着的样子:根的位姿、速度、关节都按配置的开局写进仿真。RoboDojo 的 robot_manager.reset 只设关节目标、不摆根
        (固定在桌边的胳膊用不着摆),根不固定的身体摔过一回,下一集开局还躺着"""
        a = self.art
        ids = torch.arange(a.num_instances, device=a.device)
        rs = a.data.default_root_state.clone()
        org = torch.stack([torch.as_tensor(np.asarray(_np_cpu(o), dtype=np.float32)[:3]) for o in self.env.scene_manager.env_origins]).to(a.device)
        rs[:, :3] += org[: rs.shape[0]]
        a.write_root_pose_to_sim(rs[:, :7], env_ids=ids)
        a.write_root_velocity_to_sim(torch.zeros_like(rs[:, 7:]), env_ids=ids)
        a.write_joint_state_to_sim(a.data.default_joint_pos.clone(), a.data.default_joint_vel.clone(), env_ids=ids)
        a.set_joint_position_target(a.data.default_joint_pos.clone(), env_ids=ids)
        self.reset()

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
            self.calls += 1
            self.last_cmd = list(cmd)
            self.last_out = float(raw.abs().max())
        self.sub += 1

    def reading(self, env_idx=0):
        """走路那一组的读数:机身系量到的 vx、vy、wz,骨盆离地多高"""
        d = self.art.data
        z = float(d.root_pos_w[env_idx, 2] - self.env.scene_manager.env_origins[env_idx][2]) - FLOOR_Z
        v, w = d.root_lin_vel_b[env_idx].cpu().numpy(), d.root_ang_vel_b[env_idx].cpu().numpy()
        return [float(v[0]), float(v[1]), float(w[2]), z]


FLOOR_Z = 0.05   # 地面高:客厅布局把 RoboDojo 的 Ground 顶面和客厅底板都摆在这儿(make_livingroom_layout.py;安装时核过两边一样)


def _np_cpu(x):
    return x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)


def _hook_physics(task, sim):
    """走路控制器挂在每一个物理子步上(不只挂在动作里):RoboDojo 开局复位要白走 300 个子步(task_env.reset),场景、相机也各自走几步
    (scene_manager / camera_manager 直接调 sim.sim_step),这些时候没人管腿,人形就摔了(第一版离线核:开局骨盆已经在 0.37 m)"""
    if getattr(sim, "_bd_walk_hooked", False):
        return
    orig = sim.sim_step

    def sim_step(render=True):
        w = task._walk()
        if w is not None:
            w.tick(task.bd_cmd)
        return orig(render=render)

    sim.sim_step = sim_step
    sim._bd_walk_hooked = True


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

    orig_setup = cls.setup_scene

    def setup_scene(self):
        orig_setup(self)
        self.bd_stand_and_settle()

    cls.take_action, cls.get_obs_batch, cls.setup_scene = take_action, get_obs_batch, setup_scene
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
        # 手的读数报每个关节此刻的位置(RoboDojo 自己报的是上一拍的命令)
        rig.real_ee_readings(type(self))
        # 布局里的东西 need_check_stable 都是 False(地上、家具上的东西 RoboDojo 那一关只认桌上的),核也是白核:这一关跳过
        self.scene_manager.layout_manager.check_layout_stability = lambda env, render=False: (True, [])

    def _walk(self):
        """走路控制器:身体那个关节体在仿真里起来了才建(第一次用到的时候)"""
        if self.bd_walk is None:
            keys = getattr(self.robot_manager, "robot_key", None)
            if keys and getattr(keys[0], "is_initialized", False):
                self.bd_walk = WalkController(self)
        return self.bd_walk

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)
        scene.install_checks(self.reward_manager.func_parser)
        tidy.install_checks(self.reward_manager.func_parser)
        for s in {id(o): o for o in (sim, getattr(self, "sim", None)) if o is not None}.values():
            _hook_physics(self, s)

    def reset(self, seed=None, options=None):
        self.bd_cmd = list(STAND)
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()

    def bd_stand_and_settle(self):
        """把人形摆回站着的样子,物理走到停(机身速度 < 1 cm/s、角速度 < 0.1 rad/s,每 10 个子步看一次,最多 2 秒)。
        在评测环境把场景全摆好以后调(_patch_eval_env 接在 setup_scene 后面):RoboDojo 复位时先删掉地、家具再生出来,setup_scene 里又把
        每件东西(连地、墙、家具)按布局重新摆一遍,中间都走物理步 —— 人形的脚趁地不在掉下去,地回来的时候脚埋在地里,卡着抬不起来
        (离线核:站得住、走不动,腿跟不上目标 1.7 rad;抬高 0.2 m 放手,落回去高了 4 cm,之后就走得动了)"""
        w = self._walk()
        if w is None:
            return
        w.stand_up()
        a = w.art
        for _ in range(50):
            for _ in range(10):
                self.sim_step(render=False)
            if float(a.data.root_lin_vel_w[0].norm()) < 0.01 and float(a.data.root_ang_vel_w[0].norm()) < 0.1:
                break

    def run_reward(self):
        self.reward_manager.check([("bd_tidy", {})])

    def gen_instruction(self, env_idx):
        return [tidy.find_tidy(self.scene_manager.layout_manager.saved_layouts[env_idx]).get("sentence", "")]


class bd_livingroom(BdLivingroomCommon, TaskEnv):
    pass
