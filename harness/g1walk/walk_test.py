# -*- coding: utf-8 -*-
"""第 40 条(路 8):会走的人形,厂商那一侧的走路控制器验收 —— "按 [vx, vy, wz] 走 30 分钟不摔,能上台阶、能蹲下"。
走路控制器用公开的:Isaac Lab 自带的 G1 下半身策略(Agile,TorchScript,ISAACLAB_NUCLEUS_DIR/Policies/Agile/agile_locomotion.pt),
它收 [vx, vy, wz, hip_height](胯高 0.72 m 站着,往下就是蹲),出两条腿 12 个关节的目标;上半身(腰、胳膊、手)不归它管,这里按默认姿势拿着。
身体是 Isaac Lab 的 G1_29DOF_CFG(Isaac 资产库 Robots/Unitree/G1/g1.usd,根不固定)。观测、动作都直接用 Isaac Lab 那一份的配置类
(locomanipulation/pick_place 的 AgileBasedLowerBodyActionCfg、AgileTeacherPolicyObservationsCfg),不自己拼策略的输入。

这是测试台的验收(测的是厂商那一侧的控制器),不是驱动的事;驱动以后只发 [vx, vy, wz, hip_height] 和上半身的关节。
命令按种子随机抽(每几秒换一回),只验"按命令走得动、不摔",不替谁做任务。

用法(箱上,走排队):bash run_walk.sh flat 30 | stairs 3 | squat 2
出:/root/p8/chk/g1walk_<名>/report.json(摔了几回、每一段命令和实际走的速度、台阶上去没有、蹲到多低)
"""
import argparse
import json
import math
import os
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["flat", "stairs", "squat"], default="flat")
ap.add_argument("--minutes", type=float, default=30.0, help="仿真时间(分钟)")
ap.add_argument("--seed", type=int, default=40)
ap.add_argument("--out", default="/root/p8/chk/g1walk")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import numpy as np
import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab_assets.robots.unitree import G1_29DOF_CFG
from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.action_cfg import AgileBasedLowerBodyActionCfg
from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.agile_locomotion_observation_cfg import AgileTeacherPolicyObservationsCfg

OUT = f"{args.out}_{args.mode}"
os.makedirs(OUT, exist_ok=True)
LOWER = [".*_hip_.*_joint", ".*_knee_joint", ".*_ankle_.*_joint"]


def stairs_terrain():
    """台阶:一段往上的楼梯(每级高 STEP_H、深 0.30 m,五级),前后各一段平地;用 Isaac Lab 的 mesh 地形(金字塔台阶的一个格子)"""
    from isaaclab.terrains import TerrainGeneratorCfg
    from isaaclab.terrains.trimesh import MeshPyramidStairsTerrainCfg
    gen = TerrainGeneratorCfg(size=(8.0, 8.0), border_width=4.0, num_rows=1, num_cols=1, horizontal_scale=0.1, vertical_scale=0.005,
                              slope_threshold=0.75, use_cache=False, curriculum=False,
                              sub_terrains={"stairs": MeshPyramidStairsTerrainCfg(proportion=1.0, step_height_range=(STEP_H, STEP_H),
                                                                                  step_width=0.30, platform_width=2.0, border_width=1.0)})
    return TerrainImporterCfg(prim_path="/World/ground", terrain_type="generator", terrain_generator=gen, max_init_terrain_level=0,
                              physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0))


STEP_H = 0.10   # 一级台阶 10 cm(家里的台阶 15–18 cm;先验 10 cm,过了再往上加)


@configclass
class SceneCfg(InteractiveSceneCfg):
    terrain = (stairs_terrain() if args.mode == "stairs" else
               TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane",
                                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0)))
    robot = G1_29DOF_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))


@configclass
class ActionsCfg:
    lower_body_joint_pos = AgileBasedLowerBodyActionCfg(asset_name="robot", joint_names=LOWER, policy_output_scale=0.25,
                                                        obs_group_name="lower_body_policy",
                                                        policy_path=f"{ISAACLAB_NUCLEUS_DIR}/Policies/Agile/agile_locomotion.pt")
    # 上半身不归走路控制器管:目标 = 默认姿势(动作给 0)
    upper_body = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[r"^(?!.*(_hip_|_knee_|_ankle_)).*$"], scale=1.0,
                                            use_default_offset=True)


@configclass
class ObservationsCfg:
    lower_body_policy: AgileTeacherPolicyObservationsCfg = AgileTeacherPolicyObservationsCfg()


@configclass
class EventsCfg:
    reset_robot = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class EnvCfg(ManagerBasedEnvCfg):
    scene = SceneCfg(num_envs=1, env_spacing=4.0)
    actions = ActionsCfg()
    observations = ObservationsCfg()
    events = EventsCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 1 / 200   # 和 Isaac Lab 的 locomanipulation G1 一样:物理 200 Hz,策略 50 Hz


env = ManagerBasedEnv(EnvCfg())
robot = env.scene["robot"]
n_upper = env.action_manager.get_term("upper_body").action_dim
dt = env.step_dt
rng = np.random.default_rng(args.seed)


def state():
    d = robot.data
    p = d.root_pos_w[0].cpu().numpy() - env.scene.env_origins[0].cpu().numpy()
    q = d.root_quat_w[0].cpu().numpy()
    g = d.projected_gravity_b[0].cpu().numpy()
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -g[2]))))
    v = d.root_lin_vel_b[0].cpu().numpy()
    w = d.root_ang_vel_b[0].cpu().numpy()
    yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    return p, yaw, tilt, v, w


def act(cmd):
    a = torch.zeros(1, 4 + n_upper, device=env.device)
    a[0, :4] = torch.tensor(cmd, device=env.device)
    return a


rep = {"mode": args.mode, "step_dt_s": dt, "segments": [], "falls": [], "seed": args.seed,
       # 这具身体的关节、连杆叫什么、按什么顺序排(搭 RoboDojo 那一具会走的人形要照着它配手、眼)
       "joint_names": list(robot.joint_names), "body_names": list(robot.body_names),
       "lower_body_joints": list(env.action_manager.get_term("lower_body_joint_pos")._joint_names),
       "usd": G1_29DOF_CFG.spawn.usd_path}
obs, _ = env.reset()
t0 = time.time()
T = int(round(args.minutes * 60 / dt))
fell_at = None
k = 0
if args.mode == "flat":
    # 每 4 秒随机换一回命令:vx −0.5 ~ 0.8、vy −0.4 ~ 0.4、wz −0.8 ~ 0.8 rad/s、胯高 0.55 ~ 0.75 m;每回记命令和这几秒实际的平均
    seg = int(round(4.0 / dt))
    while k < T:
        cmd = [float(rng.uniform(-0.5, 0.8)), float(rng.uniform(-0.4, 0.4)), float(rng.uniform(-0.8, 0.8)), float(rng.uniform(0.55, 0.75))]
        vs, ws, hs = [], [], []
        for _ in range(seg):
            env.step(act(cmd))
            k += 1
            p, yaw, tilt, v, w = state()
            if p[2] < 0.35 or tilt > 60.0:
                rep["falls"].append({"t_s": round(k * dt, 2), "cmd": cmd, "pelvis_z": round(float(p[2]), 3), "tilt_deg": round(tilt, 1)})
                env.reset()
                break
            if _ >= seg // 2:          # 后一半(起步过了)才算跟得上没有
                vs.append(v[:2]); ws.append(w[2]); hs.append(p[2])
        if vs:
            vm, wm = np.mean(vs, axis=0), float(np.mean(ws))
            rep["segments"].append({"t_s": round(k * dt, 1), "cmd": [round(c, 3) for c in cmd], "v_xy": [round(float(x), 3) for x in vm],
                                    "wz": round(wm, 3), "pelvis_z": round(float(np.mean(hs)), 3)})
elif args.mode == "squat":
    # 站着(胯高 0.72)→ 每 5 秒往下 0.05 m 蹲到 0.40 → 再起来;记每一档实际的骨盆高
    seq = [0.72 - 0.05 * i for i in range(7)] + [0.40] + [0.40 + 0.05 * i for i in range(1, 7)]
    seg = int(round(5.0 / dt))
    while k < T:
        for h in seq:
            hs = []
            for _ in range(seg):
                env.step(act([0.0, 0.0, 0.0, h]))
                k += 1
                p, yaw, tilt, v, w = state()
                if p[2] < 0.2 or tilt > 60.0:
                    rep["falls"].append({"t_s": round(k * dt, 2), "hip_cmd": h, "pelvis_z": round(float(p[2]), 3), "tilt_deg": round(tilt, 1)})
                    env.reset()
                    break
                if _ >= seg // 2:
                    hs.append(p[2])
            rep["segments"].append({"hip_cmd": round(h, 3), "pelvis_z": round(float(np.mean(hs)), 3) if hs else None})
            if k >= T:
                break
else:
    # 台阶:从台阶外的平地朝台阶正中走(vx 0.4,金字塔台阶的格子中心是最高的平台),看骨盆能不能升到平台那一层
    start = None
    seg = int(round(1.0 / dt))
    best = -1e9
    while k < T:
        for _ in range(seg):
            env.step(act([0.4, 0.0, 0.0, 0.72]))
            k += 1
        p, yaw, tilt, v, w = state()
        if start is None:
            start = p.copy()
        best = max(best, float(p[2] - start[2]))
        rep["segments"].append({"t_s": round(k * dt, 1), "xy": [round(float(x), 3) for x in p[:2]], "pelvis_rise_m": round(float(p[2] - start[2]), 3),
                                "tilt_deg": round(tilt, 1)})
        if p[2] - start[2] < -0.3 or tilt > 60.0:
            rep["falls"].append({"t_s": round(k * dt, 2), "pelvis_rise_m": round(float(p[2] - start[2]), 3), "tilt_deg": round(tilt, 1)})
            break
    rep["step_height_m"] = STEP_H
    rep["max_pelvis_rise_m"] = round(best, 3)
rep["sim_s"] = round(k * dt, 1)
rep["wall_s"] = round(time.time() - t0, 1)
if rep["segments"] and args.mode == "flat":
    e = [math.hypot(s["v_xy"][0] - s["cmd"][0], s["v_xy"][1] - s["cmd"][1]) for s in rep["segments"]]
    ew = [abs(s["wz"] - s["cmd"][2]) for s in rep["segments"]]
    rep["vel_err_median_mps"] = round(float(np.median(e)), 3)
    rep["vel_err_p90_mps"] = round(float(np.quantile(e, 0.9)), 3)
    rep["yaw_rate_err_median"] = round(float(np.median(ew)), 3)
json.dump(rep, open(os.path.join(OUT, "report.json"), "w"), indent=1)
print("[walk] %s:仿真 %.0f s(墙钟 %.0f s)· 摔了 %d 回 · %s" % (args.mode, rep["sim_s"], rep["wall_s"], len(rep["falls"]),
      {k2: rep[k2] for k2 in rep if k2.endswith("_mps") or k2.endswith("_median") or k2.startswith("max_")}), flush=True)
os._exit(0)
