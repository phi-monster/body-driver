# -*- coding: utf-8 -*-
# body-driver 第 40 条(路 8):会走的人形(g1walk)的几个任务共用的运行时 —— 大客厅(bd_livingroom)、上台阶(bd_stairs)、从地上捡东西(bd_floorpick)。
# 由 harness/g1walk/install.py 装到 task/RoboDojo/bd/walkrig.py(在 tasks/ 外面,不进任务清单),别手改。RoboDojo 原有的文件一个字不动,全在运行时补。
#
# 身体:Isaac 资产库的 G1(根不固定,Dex3 手)。两条腿归厂商那一侧的走路控制器 —— NVIDIA WBC-AGILE 的 velocity_height_g1
# (task/RoboDojo/bd/agile_vh.py;TorchScript 推理,接口照它自己的 LEAPP yaml / ONNX,harness/g1walk/agile_check.py 和 ONNX 一个数一个数对过;
#  文件在 Assets/Robots/g1walk/agile_velocity_height_g1/,装的时候核 sha256):收 [vx, vy, wz, 骨盆高],每 1/50 s(RoboDojo 物理 250 Hz ⇒ 每 5 个子步)
# 出两条腿 12 个关节的目标;腿、脚的电机照它训练时的配(robot_config/g1walk.py)。
# 走路控制器的命令那一组在身体报的读数里叫 base_cmd_joint_state(4 个:机身系的 vx、vy、wz 量到的,和骨盆离地多高),
# 收的命令也叫 base_cmd_joint_state(4 个:[vx, vy, wz, 骨盆高])—— 这一组怎么叫、怎么报,等路 7 的身体协议文档定了照它改。
# 走路控制器挂在每一个物理子步上(_hook_physics);评测环境把场景全摆好以后,再把人形摆回站着的样子、物理走到停(bd_stand_and_settle)。
import os

import numpy as np
import torch

from env.reward_manager.reward_manager import RewardManager
from task.RoboDojo.bd import agile_vh, rig, scene

rig.register("g1walk", "g1walk", ["G1WalkLeft", "G1WalkRight"])
rig.no_planner("g1walk")
rig.dims("g1walk")

CMD_KEY = "base_cmd_joint_state"
STAND = [0.0, 0.0, 0.0, 0.72]    # 站着不动:骨盆高 0.72 m(velocity_height_g1 训练时的站直高度,unitree_g1.DEFAULT_PELVIS_HEIGHT)
BUNDLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../Assets/Robots/g1walk/agile_velocity_height_g1")
FLOOR_Z = 0.05   # 地面高:布局把 RoboDojo 的 Ground 顶面和客厅底板都摆在这儿(make_livingroom_layout.py;安装时核过两边一样)


def _np_cpu(x):
    return x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)


class WalkController:
    """厂商那一侧的走路控制器(每个物理子步 tick 一次,每 decim 个子步算一回策略)"""

    def __init__(self, env):
        self.env = env
        self.art = env.robot_manager.robot_key[0]
        self.ctrl = agile_vh.AgileVH(os.path.normpath(BUNDLE), list(self.art.joint_names), num_envs=self.art.num_instances, device=self.art.device)
        bad = self.ctrl.gains_ok(self.art)
        assert not bad, f"腿上的 kp / kd 和走路控制器要的不一样:{bad}"
        self.decim = int(round(1.0 / self.ctrl.freq / float(env.dt)))
        assert abs(self.decim * float(env.dt) * self.ctrl.freq - 1.0) < 1e-6, (env.dt, self.ctrl.freq)
        self.sub = 0
        self.calls, self.last_cmd, self.last_out = 0, None, None   # 离线核看:策略算了几回、最后一回用的命令、输出多大

    def reset(self):
        self.ctrl.reset()
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
            c = torch.tensor(cmd, dtype=torch.float32, device=self.art.device).unsqueeze(0).repeat(self.art.num_instances, 1)
            tgt = self.ctrl.step(c, d.root_ang_vel_b, d.projected_gravity_b, d.joint_pos, d.joint_vel)
            self.art.set_joint_position_target(tgt, joint_ids=self.ctrl.leg_ids)
            self.calls += 1
            self.last_cmd = list(cmd)
            self.last_out = float(self.ctrl.last.abs().max())
        self.sub += 1

    def reading(self, env_idx=0):
        """走路那一组的读数:机身系量到的 vx、vy、wz,骨盆离地多高"""
        d = self.art.data
        z = float(d.root_pos_w[env_idx, 2] - self.env.scene_manager.env_origins[env_idx][2]) - FLOOR_Z
        v, w = d.root_lin_vel_b[env_idx].cpu().numpy(), d.root_ang_vel_b[env_idx].cpu().numpy()
        return [float(v[0]), float(v[1]), float(w[2]), z]


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
    """评测环境:动作里 base_cmd_joint_state 那一项先拿下来给走路控制器(RoboDojo 核动作时不认它),读数里加上这一组;
    场景全摆好以后(setup_scene)把人形摆回站着、物理走到停"""
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


class WalkCommon:
    """会走的人形的任务都用的这一套;子类给 STEP_LIM(一集多少个动作,BD_STEP_LIM 可改)、判据、给脑的话"""
    STEP_LIM = 750

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = int(os.environ.get("BD_STEP_LIM", str(self.STEP_LIM)))
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
