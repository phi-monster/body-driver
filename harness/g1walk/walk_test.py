# -*- coding: utf-8 -*-
"""第 40 条(路 8):会走的人形,厂商那一侧的走路控制器验收。不起 RoboDojo、不起驱动、不起脑、不开相机:Isaac Lab 自己的场子里只放人形和地。
两份控制器(--policy):
  vh  :NVIDIA WBC-AGILE 的 velocity_height_g1(rd/bd/agile_vh.py;TorchScript 推理,接口照它自己的 yaml / ONNX,agile_check.py 对过)。
  groot:NVIDIA GR00T-WholeBodyControl 的 Decoupled WBC(rd/bd/groot_wbc.py;它的两份 ONNX 用 onnx 的参考实现跑,接口照它自己的 MuJoCo 跑法,
        groot_check.py 对过)。它出两条腿 + 腰 15 个;身体 = Isaac Lab 的 G1_29DOF_CFG,两条腿 + 腰换成它跑法里的 PD(kp / kd / 力矩上限 / armature)。
        身体 = 测试台用的那一具(rd/env/robot_manager/robot_config/g1walk.py:腿、脚的电机照它训练时的配)。
  old :Isaac Lab 自带的 agile_locomotion.pt(老的那份,留着对照):身体 = Isaac Lab 的 G1_29DOF_CFG 原样,输入输出照 Isaac Lab 的
        AgileBasedLowerBodyActionCfg / AgileTeacherPolicyObservationsCfg。
都收 [vx, vy, wz, 骨盆高](机身系;骨盆高 0.72 m 是站直)。上半身不归走路控制器管,按默认姿势拿着。

验收(--mode,可以用逗号连着跑几样;stairs 要另起,地不一样):
  squat     :站着不动,骨盆高从 0.72 每 4 秒往下给 0.04 到 0.40,再一档一档回到 0.72;每一档量后 2 秒骨盆实际多高。
  squatwalk :骨盆高给 0.72 / 0.66 / 0.60 / 0.55 / 0.50,每一档先站 3 秒,再 vx 0.4 走 5 秒,再停 2 秒;量走的时候骨盆多高、走了多远。
  flat      :平地随机命令,每 4 秒换一回(vx −0.5 ~ 0.8、vy ±0.4、wz ±0.8 rad/s;骨盆高 0.55 ~ 0.75,--hold_height 就固定 0.72),
              一共 --minutes 分钟;数摔了几回,速度跟得上没有。
  flatmany  :--num_envs 个人形一起走 flat 那一套(各抽各的命令),谁摔了只复位谁;一共 num_envs × minutes 机器人分钟。
  stairs    :正前方 1 m 起 --steps 级台阶(每级高 --step_h、深 0.30 m,最上面一级 2 m 深),vx 0.4 往前走;看上去没有、摔没摔。
摔了 = 身子歪过 60°,或者骨盆离脚下的地不到 0.25 m,或者除了两只脚以外哪一节离地不到 5 cm(跪下、坐下、躺下)。

用法(箱上,走排队):bash qwalk.sh squat,squatwalk,flat 1 --policy vh --dt 0.004 --decim 5 --friction_mode multiply --tag _vh
出:/root/p8/chk/g1walk_<模式>_<tag>/report.json
"""
import argparse
import importlib.util
import json
import math
import os
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--policy", choices=["vh", "old", "groot"], default="vh")
ap.add_argument("--groot_bundle", default="/root/p8/groot", help="GR00T Decoupled WBC 那几个文件放在哪儿(fetch_groot.py 取下来、核过 sha256 的)")
ap.add_argument("--squat_min", type=float, default=0.40, help="squat 往下给到多低(每档 0.04)")
ap.add_argument("--reach_h", type=float, default=0.40, help="reach2 蹲到多低(胯高命令)")
ap.add_argument("--bundle", default="/root/RoboDojo/Assets/Robots/g1walk/agile_velocity_height_g1",
                help="velocity_height_g1 那几个文件放在哪儿(install.py 用 fetch_agile.py 取下来、核过 sha256 放的地方)")
ap.add_argument("--mode", default="squat,squatwalk,flat")
ap.add_argument("--minutes", type=float, default=1.0, help="flat / flatmany 走多久(仿真分钟);stairs 最多走多久")
ap.add_argument("--hold_height", action="store_true", help="flat 走的时候骨盆高固定 0.72(训练时就是这样)")
ap.add_argument("--num_envs", type=int, default=16, help="flatmany 几个人形")
ap.add_argument("--steps", type=int, default=1, help="stairs 几级")
ap.add_argument("--step_h", type=float, default=0.10, help="stairs 一级多高(m)")
ap.add_argument("--seed", type=int, default=40)
ap.add_argument("--out", default="/root/p8/chk/g1walk")
ap.add_argument("--dt", type=float, default=1 / 200, help="物理一步多少秒(AGILE 训练用 1/200;RoboDojo 是 0.004)")
ap.add_argument("--decim", type=int, default=4, help="几个物理步算一回策略(dt × decim = 0.02 s = 50 Hz)")
ap.add_argument("--friction_mode", default="multiply", help="地面摩擦和脚的怎么合(AGILE 训练、RoboDojo 的地都是 multiply)")
ap.add_argument("--tag", default="", help="结果目录名后缀")
ap.add_argument("--reach_grid", default="0,0.5;-0.5,0,0.5,1.0;-1.0,0,1.0",
                help="reach2 摆哪几档:腰俯仰;肩前后;肘(弧度,离默认姿势),分号隔开三组,逗号隔开每组的几档")
ap.add_argument("--faithful", action="store_true",
                help="只拿来查原因:vh —— 身体整个照 AGILE 训练时的 G1_29DOF(不要手 —— USD 的 left_hand / right_hand 变体选 None;腰、胳膊也换成它的 DC 电机);"
                     "groot —— 胳膊也照它自己的 MuJoCo 跑法(目标 0、kp 100、kd 0.5 的 PD)")
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
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab_assets.robots.unitree import G1_29DOF_CFG

HERE = os.path.dirname(os.path.abspath(__file__))
MODES = args.mode.split(",")
assert "stairs" not in MODES or MODES == ["stairs"], "stairs 要另起一回(地不一样)"
N = args.num_envs if "flatmany" in MODES else 1
assert "flatmany" not in MODES or MODES == ["flatmany"], "flatmany 要另起一回(人形个数不一样)"
OUT = f"{args.out}_{args.mode.replace(',', '+')}{args.tag}"
os.makedirs(OUT, exist_ok=True)
LOWER = [".*_hip_.*_joint", ".*_knee_joint", ".*_ankle_.*_joint"]
STAND_H = 0.72          # vh、old:站直的胯高命令;groot 换成它自己配置里的(0.74,见下)
# 上半身(腰、胳膊、手)归测试台按默认姿势拿着;groot 连腰也归走路控制器管(它出两条腿 + 腰 15 个)
UPPER_RE = r"^(?!.*(_hip_|_knee_|_ankle_|waist_)).*$" if args.policy == "groot" else r"^(?!.*(_hip_|_knee_|_ankle_)).*$"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


if args.policy == "vh":
    ROBOT = load(os.path.join(HERE, "rd/env/robot_manager/robot_config/g1walk.py"), "g1walk_robot").get_robot_config()
    agile_vh = load(os.path.join(HERE, "rd/bd/agile_vh.py"), "agile_vh")
    if args.faithful:
        # agile/rl_env/assets/robots/unitree_g1.py 的 G1_29DOF:不要手;腰、胳膊是它的 DC 电机(延时不加:训练时 0–4 个物理步随机)
        from isaaclab.actuators import DCMotorCfg
        ROBOT.spawn.variants = {"Physics": "PhysX", "left_hand": "None", "right_hand": "None"}
        acts = {k: v for k, v in ROBOT.actuators.items() if k in ("legs", "feet")}
        acts["waist"] = DCMotorCfg(joint_names_expr=["waist_.*_joint"],
                                   effort_limit={"waist_yaw_joint": 88.0, "waist_roll_joint": 50.0, "waist_pitch_joint": 50.0},
                                   effort_limit_sim={"waist_yaw_joint": 88.0, "waist_roll_joint": 50.0, "waist_pitch_joint": 50.0},
                                   velocity_limit={"waist_yaw_joint": 32.0, "waist_roll_joint": 37.0, "waist_pitch_joint": 37.0},
                                   velocity_limit_sim={"waist_yaw_joint": 32.0, "waist_roll_joint": 37.0, "waist_pitch_joint": 37.0},
                                   stiffness=300.0, damping=5.0, armature=0.03, saturation_effort=120.0)
        ae = {".*_shoulder_pitch_joint": 25.0, ".*_shoulder_roll_joint": 25.0, ".*_shoulder_yaw_joint": 25.0, ".*_elbow_joint": 25.0,
              ".*_wrist_roll_joint": 25.0, ".*_wrist_pitch_joint": 5.0, ".*_wrist_yaw_joint": 5.0}
        av = {".*_shoulder_pitch_joint": 37.0, ".*_shoulder_roll_joint": 37.0, ".*_shoulder_yaw_joint": 37.0, ".*_elbow_joint": 37.0,
              ".*_wrist_roll_joint": 37.0, ".*_wrist_pitch_joint": 22.0, ".*_wrist_yaw_joint": 22.0}
        acts["arms"] = DCMotorCfg(joint_names_expr=[".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_.*_joint"],
                                  effort_limit=dict(ae), effort_limit_sim=dict(ae), velocity_limit=dict(av), velocity_limit_sim=dict(av),
                                  stiffness={".*_shoulder_pitch_joint": 90.0, ".*_shoulder_roll_joint": 60.0, ".*_shoulder_yaw_joint": 20.0,
                                             ".*_elbow_joint": 60.0, ".*_wrist_.*_joint": 4.0},
                                  damping={".*_shoulder_pitch_joint": 2.0, ".*_shoulder_roll_joint": 1.0, ".*_shoulder_yaw_joint": 0.4,
                                           ".*_elbow_joint": 1.0, ".*_wrist_.*_joint": 0.2},
                                  armature=0.03, saturation_effort=40.0)
        ROBOT.actuators = acts
elif args.policy == "groot":
    # 身体 = 测试台那一具(Isaac Lab 的 G1_29DOF_CFG,Dex3 手),两条腿 + 腰换成 GR00T 自己跑法里的电机:每个物理步算
    # 力矩 = kp(目标 − 位置) − kd·速度、夹在它 MuJoCo 身体写的上限里(IdealPD,显式);armature 照它 XML 的默认 0.01
    groot_wbc = load(os.path.join(HERE, "rd/bd/groot_wbc.py"), "groot_wbc")
    import yaml as _yaml
    from isaaclab.actuators import IdealPDActuatorCfg
    _gc = _yaml.safe_load(open(os.path.join(args.groot_bundle, "g1_gear_wbc.yaml")))
    _gj, _gdef = groot_wbc.xml_joints(os.path.join(args.groot_bundle, "g1_gear_wbc.xml"))
    _eff = dict(_gj)
    _names15 = [n for n, _ in _gj][: int(_gc["num_actions"])]
    ROBOT = G1_29DOF_CFG.copy()
    ROBOT.spawn.articulation_props.fix_root_link = False
    acts = {k: v for k, v in ROBOT.actuators.items() if k not in ("legs", "feet", "waist")}
    acts["groot_lower"] = IdealPDActuatorCfg(
        joint_names_expr=list(_names15), stiffness={n: float(v) for n, v in zip(_names15, _gc["kps"])},
        damping={n: float(v) for n, v in zip(_names15, _gc["kds"])}, effort_limit={n: _eff[n] for n in _names15},
        effort_limit_sim={n: _eff[n] for n in _names15}, armature=float(_gdef.get("armature", 0.0)))
    if args.faithful:
        _arms = [n for n, _ in _gj][int(_gc["num_actions"]):]
        acts.pop("arms", None)
        acts["groot_arms"] = IdealPDActuatorCfg(joint_names_expr=_arms, stiffness=100.0, damping=0.5,
                                                effort_limit={n: _eff[n] for n in _arms}, effort_limit_sim={n: _eff[n] for n in _arms},
                                                armature=float(_gdef.get("armature", 0.0)))
    ROBOT.actuators = acts
    STAND_H = float(_gc["height_cmd"])
else:
    ROBOT = G1_29DOF_CFG.copy()
q = ROBOT.init_state.rot
YAW0 = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))   # 开局朝哪(G1_29DOF_CFG 开局转了 90°,朝 +y)
HEAD = np.array([math.cos(YAW0), math.sin(YAW0)])
STEP_D, TOP_D, STEP_X0, STEP_W = 0.30, 2.0, 1.0, 3.0      # 台阶:每级深 0.30 m,最上面一级 2 m;离开局 1 m 起;宽 3 m


def stair_boxes():
    """台阶是摆在地上的几块静止的方块(有碰撞、不动);第 k 级顶面高 (k+1) × step_h"""
    out = {}
    for k in range(args.steps):
        d = TOP_D if k == args.steps - 1 else STEP_D
        s0 = STEP_X0 + k * STEP_D
        h = (k + 1) * args.step_h
        c = HEAD * (s0 + d / 2)
        size = (d, STEP_W, h) if abs(HEAD[0]) > 0.5 else (STEP_W, d, h)
        out[f"step{k}"] = AssetBaseCfg(prim_path=f"/World/step{k}",
                                       spawn=sim_utils.CuboidCfg(size=size, collision_props=sim_utils.CollisionPropertiesCfg(),
                                                                 physics_material=sim_utils.RigidBodyMaterialCfg(
                                                                     static_friction=1.0, dynamic_friction=1.0, friction_combine_mode=args.friction_mode),
                                                                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.5, 0.4))),
                                       init_state=AssetBaseCfg.InitialStateCfg(pos=(float(c[0]), float(c[1]), h / 2)))
    return out


def ground_under(s):
    """沿开局朝向走了 s 米,脚下的地多高"""
    if MODES != ["stairs"] or s < STEP_X0:
        return 0.0
    k = int((s - STEP_X0) // STEP_D)
    return min(k + 1, args.steps) * args.step_h


@configclass
class SceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane",
                                 physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0,
                                                                                 friction_combine_mode=args.friction_mode))
    robot = ROBOT.replace(prim_path="{ENV_REGEX_NS}/Robot")
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))


if args.policy == "old":
    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
    from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.action_cfg import AgileBasedLowerBodyActionCfg
    from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.agile_locomotion_observation_cfg import AgileTeacherPolicyObservationsCfg

    @configclass
    class ActionsCfg:
        lower_body_joint_pos = AgileBasedLowerBodyActionCfg(asset_name="robot", joint_names=LOWER, policy_output_scale=0.25,
                                                            obs_group_name="lower_body_policy",
                                                            policy_path=f"{ISAACLAB_NUCLEUS_DIR}/Policies/Agile/agile_locomotion.pt")
        upper_body = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[r"^(?!.*(_hip_|_knee_|_ankle_)).*$"], scale=1.0,
                                                use_default_offset=True)

    @configclass
    class ObservationsCfg:
        lower_body_policy: AgileTeacherPolicyObservationsCfg = AgileTeacherPolicyObservationsCfg()
else:
    @configclass
    class ActionsCfg:
        # 腿归走路控制器(每一拍直接写腿的目标);这里只管上半身:目标 = 默认姿势(动作给 0)
        upper_body = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[UPPER_RE], scale=1.0, use_default_offset=True)

    @configclass
    class ObservationsCfg:
        @configclass
        class Unused(ObsGroup):
            ang = ObsTerm(func=mdp.base_ang_vel)
        unused: Unused = Unused()


@configclass
class EventsCfg:
    reset_robot = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class EnvCfg(ManagerBasedEnvCfg):
    scene = SceneCfg(num_envs=N, env_spacing=4.0)
    actions = ActionsCfg()
    observations = ObservationsCfg()
    events = EventsCfg()

    def __post_init__(self):
        self.decimation = args.decim
        self.sim.dt = args.dt
        if MODES == ["stairs"]:      # 台阶的几块方块挂在场景上(InteractiveScene 按 cfg 的属性一个个生出来)
            for k, v in stair_boxes().items():
                setattr(self.scene, k, v)


env = ManagerBasedEnv(EnvCfg())
robot = env.scene["robot"]
dev = env.device
n_upper = env.action_manager.get_term("upper_body").action_dim
dt = env.step_dt
rng = np.random.default_rng(args.seed)
ctrl = None
rep = {"policy": args.policy, "faithful": args.faithful, "modes": MODES, "step_dt_s": dt, "physics_dt_s": args.dt, "decimation": args.decim,
       "friction_mode": args.friction_mode, "seed": args.seed, "num_envs": N, "usd": ROBOT.spawn.usd_path,
       "joint_names": list(robot.joint_names), "falls": []}
if args.policy in ("vh", "groot"):
    if args.policy == "vh":
        ctrl = agile_vh.AgileVH(args.bundle, list(robot.joint_names), num_envs=N, device=dev)
    else:
        ctrl = groot_wbc.GrootWBC(args.groot_bundle, list(robot.joint_names), num_envs=N, device=dev)
    assert abs(1.0 / dt - ctrl.freq) < 1e-6, f"策略要 {ctrl.freq} Hz,这里一拍 {dt} s"
    bad = ctrl.gains_ok(robot)
    assert not bad, f"腿上的 kp / kd 和策略要的不一样:{bad}"
    leg = list(ctrl.leg_ids)
    rep["legs_in_sim"] = {}
    for a in robot.actuators.values():
        for j, n in enumerate(a.joint_names):
            if n in ctrl.leg_names:
                rep["legs_in_sim"][n] = {"kp": round(float(a.stiffness[0, j]), 4), "kd": round(float(a.damping[0, j]), 4),
                                         "effort": round(float(a.effort_limit[0, j]), 2), "vel_curve": round(float(a.velocity_limit[0, j]), 2),
                                         "armature": round(float(a.armature[0, j]), 6), "type": type(a).__name__}
    jl = getattr(robot.data, "joint_vel_limits", getattr(robot.data, "joint_velocity_limits", None))
    if jl is not None:
        rep["physx_joint_vel_limit"] = {n: round(float(jl[0, robot.joint_names.index(n)]), 2) for n in ctrl.leg_names}
    rep["policy_files"] = {"bundle": args.groot_bundle if args.policy == "groot" else args.bundle, "freq_hz": ctrl.freq,
                           "history": ctrl.history, "leg_order": ctrl.leg_names, "stand_height_cmd": STAND_H}
non_feet = [i for i, n in enumerate(robot.body_names) if "ankle" not in n]
try:   # 这具身体的 USD 有哪些变体、选的是哪个(--faithful 去掉手靠的就是它)
    import omni.usd
    _prim = omni.usd.get_context().get_stage().GetPrimAtPath(env.scene.env_prim_paths[0] + "/Robot")
    _vs = _prim.GetVariantSets()
    rep["usd_variants"] = {n: {"choices": list(_vs.GetVariantSet(n).GetVariantNames()), "selected": _vs.GetVariantSet(n).GetVariantSelection()}
                           for n in _vs.GetNames()}
except Exception as _e:  # noqa: BLE001
    rep["usd_variants"] = repr(_e)
rep["mass_kg"] = round(float(robot.root_physx_view.get_masses()[0].sum()), 3)


def do_step(cmd):
    """cmd:(N, 4) [vx, vy, wz, 骨盆高]。走一拍(0.02 s)"""
    c = torch.as_tensor(np.asarray(cmd, dtype=np.float32), device=dev).reshape(-1, 4)
    if c.shape[0] == 1 and N > 1:      # 一条命令给所有人形
        c = c.expand(N, 4).contiguous()
    a = torch.zeros(N, (4 if args.policy == "old" else 0) + n_upper, device=dev)
    if UPPER is not None:                 # 上半身(腰、胳膊、手)的目标:离默认姿势多少(reach 那一样用)
        a[:, (4 if args.policy == "old" else 0):] = UPPER
    if args.policy == "old":
        a[:, :4] = c
    elif args.policy == "groot":
        d = robot.data
        tgt = ctrl.step(c[:, :3], c[:, 3], RPY.expand(N, 3), d.root_ang_vel_b, d.projected_gravity_b, d.joint_pos, d.joint_vel)
        robot.set_joint_position_target(tgt, joint_ids=leg)
    else:
        d = robot.data
        tgt = ctrl.step(c, d.root_ang_vel_b, d.projected_gravity_b, d.joint_pos, d.joint_vel)
        robot.set_joint_position_target(tgt, joint_ids=leg)
    env.step(a)


RPY = torch.zeros(1, 3, device=dev)    # groot:躯干 roll / pitch / yaw 命令(它的观测里有这三个;reach2 用 pitch 往前弯)


UPPER = None
_up_names = list(env.action_manager.get_term("upper_body")._joint_names)


def upper(**targets):
    """上半身这几个关节的目标(弧度,离默认姿势);别的照默认。不给 = 全回默认"""
    global UPPER
    if not targets:
        UPPER = None
        return
    u = torch.zeros(N, n_upper, device=dev)
    for n, v in targets.items():
        u[:, _up_names.index(n)] = float(v)
    UPPER = u


def reset(env_ids=None):
    if env_ids is None:
        env.reset()
    else:
        env.reset(env_ids=torch.as_tensor(env_ids, device=dev))
    if ctrl is not None:
        ctrl.reset(None if env_ids is None else torch.as_tensor(env_ids, device=dev))


def along(i=0):
    p = (robot.data.root_pos_w[i] - env.scene.env_origins[i]).cpu().numpy()
    return float(np.dot(p[:2], HEAD)), p


def state():
    d = robot.data
    p = (d.root_pos_w - env.scene.env_origins).cpu().numpy()
    g = d.projected_gravity_b.cpu().numpy()
    tilt = np.degrees(np.arccos(np.clip(-g[:, 2], -1.0, 1.0)))
    low = (d.body_pos_w[:, non_feet, 2] - env.scene.env_origins[:, None, 2]).min(dim=1).values.cpu().numpy()
    ground = np.array([ground_under(float(np.dot(p[i, :2], HEAD))) for i in range(N)])
    fallen = (tilt > 60.0) | (p[:, 2] - ground < 0.25) | (low - ground < 0.05)
    return p, tilt, fallen, d.root_lin_vel_b.cpu().numpy(), d.root_ang_vel_b.cpu().numpy()


def fall(t, extra):
    p, tilt, fallen, v, w = state()
    rep["falls"].append({"t_s": round(t, 2), "pelvis_z": round(float(p[0, 2]), 3), "tilt_deg": round(float(tilt[0]), 1), **extra})


def hold(cmd, seconds, keep_from=0.0):
    """命令 cmd 拿着走 seconds 秒;返回 (摔没摔, 从 keep_from 秒起每一拍的 (骨盆 xyz, 机身系速度, 角速度))"""
    k = int(round(seconds / dt))
    k0 = int(round(keep_from / dt))
    rec = []
    for j in range(k):
        do_step([cmd])
        p, tilt, fallen, v, w = state()
        if fallen[0]:
            return True, rec
        if j >= k0:
            rec.append((p[0].copy(), v[0].copy(), w[0].copy()))
    return False, rec


t0 = time.time()
sim_t = 0.0
reset()
hold([0.0, 0.0, 0.0, STAND_H], 2.0)   # 先站稳 2 秒
sim_t += 2.0
for mode in MODES:
    if mode == "squat":
        n_down = int(round((STAND_H - args.squat_min) / 0.04))
        seq = [round(STAND_H - 0.04 * i, 2) for i in range(n_down + 1)] + [round(STAND_H - 0.04 * i, 2) for i in range(n_down - 1, -1, -1)]
        out = []
        for h in seq:
            fell, rec = hold([0.0, 0.0, 0.0, h], 4.0, keep_from=2.0)
            sim_t += 4.0
            z = np.array([r[0][2] for r in rec]) if rec else np.array([np.nan])
            xy = np.array([r[0][:2] for r in rec]) if rec else np.zeros((1, 2))
            o = {"hip_cmd": h, "pelvis_z": round(float(np.mean(z)), 3), "pelvis_z_range": [round(float(np.min(z)), 3), round(float(np.max(z)), 3)],
                 "err": round(float(np.mean(z)) - h, 3), "fell": fell}
            if ctrl is not None:   # 腿到没到策略给的目标(差得多 = 扛不住;差得少 = 策略自己就只给了这么深)
                d = robot.data
                q, qt = d.joint_pos[0, leg].cpu().numpy(), d.joint_pos_target[0, leg].cpu().numpy()
                o["leg_target_minus_pos_rad"] = {n: round(float(a - b), 3) for n, a, b in zip(ctrl.leg_names, qt, q) if n.startswith("left_")}
                o["leg_pos_rad"] = {n: round(float(b), 3) for n, b in zip(ctrl.leg_names, q) if n.startswith("left_")}
            out.append(o)
            if fell:
                fall(sim_t, {"mode": mode, "hip_cmd": h})
                reset()
                hold([0.0, 0.0, 0.0, STAND_H], 2.0)
        rep["squat"] = out
        rep["squat_lowest_pelvis_z"] = min(o["pelvis_z"] for o in out)
        print("[walk] squat:" + " ".join("%.2f→%.3f" % (o["hip_cmd"], o["pelvis_z"]) for o in out), flush=True)
    elif mode == "squatwalk":
        out = []
        for h in [0.72, 0.66, 0.60, 0.55, 0.50]:
            fell1, rec1 = hold([0.0, 0.0, 0.0, h], 3.0, keep_from=2.0)
            s0, _ = along()
            fell2, rec2 = (True, []) if fell1 else hold([0.4, 0.0, 0.0, h], 5.0, keep_from=2.0)
            s1, _ = along()
            fell3, rec3 = (True, []) if (fell1 or fell2) else hold([0.0, 0.0, 0.0, h], 2.0, keep_from=1.0)
            sim_t += 10.0
            o = {"hip_cmd": h,
                 "pelvis_z_standing": round(float(np.mean([r[0][2] for r in rec1])), 3) if rec1 else None,
                 "pelvis_z_walking": round(float(np.mean([r[0][2] for r in rec2])), 3) if rec2 else None,
                 "vx_walking": round(float(np.mean([r[1][0] for r in rec2])), 3) if rec2 else None,
                 "walked_m_in_5s": round(s1 - s0, 3),
                 "pelvis_z_stopped": round(float(np.mean([r[0][2] for r in rec3])), 3) if rec3 else None,
                 "fell": bool(fell1 or fell2 or fell3)}
            out.append(o)
            if o["fell"]:
                fall(sim_t, {"mode": mode, "hip_cmd": h})
            reset()
            hold([0.0, 0.0, 0.0, STAND_H], 2.0)
        rep["squatwalk"] = out
        print("[walk] squatwalk:" + " ".join("%.2f:站%.3f/走%.3f(%.2fm)%s" % (o["hip_cmd"], o["pelvis_z_standing"] or -1, o["pelvis_z_walking"] or -1,
                                                                            o["walked_m_in_5s"], "摔" if o["fell"] else "") for o in out), flush=True)
    elif mode == "flat":
        segs = []
        T = int(round(args.minutes * 60 / 4.0))
        n_fall0 = len(rep["falls"])
        for i in range(T):
            cmd = [float(rng.uniform(-0.5, 0.8)), float(rng.uniform(-0.4, 0.4)), float(rng.uniform(-0.8, 0.8)),
                   STAND_H if args.hold_height else float(rng.uniform(0.55, 0.75))]
            fell, rec = hold(cmd, 4.0, keep_from=2.0)
            sim_t += 4.0
            if fell:
                fall(sim_t, {"mode": mode, "cmd": [round(c, 3) for c in cmd]})
                reset()
                continue
            vm = np.mean([r[1][:2] for r in rec], axis=0)
            segs.append({"cmd": [round(c, 3) for c in cmd], "v_xy": [round(float(x), 3) for x in vm],
                         "wz": round(float(np.mean([r[2][2] for r in rec])), 3), "pelvis_z": round(float(np.mean([r[0][2] for r in rec])), 3)})
        e = [math.hypot(s["v_xy"][0] - s["cmd"][0], s["v_xy"][1] - s["cmd"][1]) for s in segs]
        ew = [abs(s["wz"] - s["cmd"][2]) for s in segs]
        eh = [abs(s["pelvis_z"] - s["cmd"][3]) for s in segs]
        rep["flat"] = {"minutes": args.minutes, "hold_height": args.hold_height, "segments": segs, "falls": len(rep["falls"]) - n_fall0,
                       "vel_err_median_mps": round(float(np.median(e)), 3) if e else None,
                       "vel_err_p90_mps": round(float(np.quantile(e, 0.9)), 3) if e else None,
                       "yaw_rate_err_median": round(float(np.median(ew)), 3) if ew else None,
                       "height_err_median_m": round(float(np.median(eh)), 3) if eh else None}
        print("[walk] flat:%.1f 分钟 摔 %d 回 · 速度差中位 %s m/s · 九成 %s · 转速差中位 %s · 骨盆高差中位 %s" % (
            args.minutes, rep["flat"]["falls"], rep["flat"]["vel_err_median_mps"], rep["flat"]["vel_err_p90_mps"],
            rep["flat"]["yaw_rate_err_median"], rep["flat"]["height_err_median_m"]), flush=True)
    elif mode == "flatmany":
        seg = int(round(4.0 / dt))
        T = int(round(args.minutes * 60 / dt))
        k = 0
        errs, werrs = [], []
        while k < T:
            cmd = np.stack([rng.uniform(-0.5, 0.8, N), rng.uniform(-0.4, 0.4, N), rng.uniform(-0.8, 0.8, N),
                            np.full(N, STAND_H) if args.hold_height else rng.uniform(0.55, 0.75, N)], axis=1)
            for j in range(seg):
                do_step(cmd)
                k += 1
                p, tilt, fallen, v, w = state()
                bad = np.where(fallen)[0]
                if len(bad):
                    rep["falls"] += [{"env": int(i), "t_s": round(k * dt, 2), "cmd": [round(float(x), 3) for x in cmd[i]],
                                      "pelvis_z": round(float(p[i, 2]), 3), "tilt_deg": round(float(tilt[i]), 1)} for i in bad]
                    reset(bad.tolist())
                if j >= seg // 2:
                    errs.append(np.linalg.norm(v[:, :2] - cmd[:, :2], axis=1))
                    werrs.append(np.abs(w[:, 2] - cmd[:, 2]))
        sim_t += k * dt
        e, ew = np.concatenate(errs), np.concatenate(werrs)
        rep["flatmany"] = {"robot_minutes": round(N * k * dt / 60.0, 1), "falls": len(rep["falls"]),
                           "vel_err_median_mps": round(float(np.median(e)), 3), "vel_err_p90_mps": round(float(np.quantile(e, 0.9)), 3),
                           "yaw_rate_err_median": round(float(np.median(ew)), 3)}
        print("[walk] flatmany:%s" % rep["flatmany"], flush=True)
    elif mode == "still":
        # 站着不动到底有多不动(驱动开机先量"静止噪声":每只眼不动时一拍里画面变多少;身体自己晃,眼就跟着晃 ——
        # P8XH / P8XH2 两回开机,头上那只眼的灰度地板 163 / 200,固定在桌边的 G1 是 17、x5 是 6):站 10 秒,每拍记躯干(头上那只眼装在它上面)
        # 的朝向和位置,算一拍里转了多少、10 秒里晃了多大;按 640 宽、横向视场 69°(d435)折成头上那只眼一拍里画面挪几个像素
        tid = robot.body_names.index("torso_link")
        f_px = 320.0 / math.tan(math.radians(69.0 / 2))
        qs, ps = [], []
        for _ in range(int(round(10.0 / dt))):
            do_step([[0.0, 0.0, 0.0, STAND_H]])
            qs.append(robot.data.body_quat_w[0, tid].cpu().numpy().astype(np.float64))
            ps.append((robot.data.body_pos_w[0, tid] - env.scene.env_origins[0]).cpu().numpy().astype(np.float64))
        sim_t += 10.0
        qs, ps = np.array(qs), np.array(ps)
        dots = np.abs(np.sum(qs[1:] * qs[:-1], axis=1)).clip(0, 1)
        dang = np.degrees(2 * np.arccos(dots))                       # 一拍里转了多少度
        q_mean = qs.mean(axis=0) / np.linalg.norm(qs.mean(axis=0))
        dev_ang = np.degrees(2 * np.arccos(np.abs(qs @ q_mean).clip(0, 1)))
        dpos = np.linalg.norm(np.diff(ps, axis=0), axis=1)
        rep["still"] = {"seconds": 10.0, "torso_turn_per_beat_deg": {"median": round(float(np.median(dang)), 4), "p90": round(float(np.quantile(dang, 0.9)), 4),
                                                                    "max": round(float(dang.max()), 4)},
                        "torso_turn_per_beat_px": {"median": round(float(f_px * np.radians(np.median(dang))), 2),
                                                   "p90": round(float(f_px * np.radians(np.quantile(dang, 0.9))), 2),
                                                   "max": round(float(f_px * np.radians(dang.max())), 2)},
                        "torso_sway_deg_max": round(float(dev_ang.max()), 3), "torso_move_per_beat_mm_p90": round(float(np.quantile(dpos, 0.9) * 1000), 2),
                        "torso_drift_m": round(float(np.linalg.norm(ps[-1, :2] - ps[0, :2])), 4)}
        print("[walk] still:%s" % rep["still"], flush=True)
    elif mode == "reach":
        # 手最低能到多低(从地上 / 矮处拿东西要它):胳膊照默认姿势垂着(G1 胳膊各关节 0 = 垂在身体两边),骨盆高 0.72 和 0.40 各站 4 秒,
        # 量手那几节(名字里带 hand 的连杆,连杆原点;指尖的形状再往外几厘米)最低在哪;再在 0.40 上把腰往前弯(waist_pitch +0.3、+0.5 rad,
        # 测试台现在没把腰交给驱动 —— 这是"要是交出去了"能到多低,velocity_height_g1 训练时腰的俯仰一直是 0)
        hands = [i for i, n in enumerate(robot.body_names) if "hand" in n]
        out = []
        for h, wp in ((0.72, 0.0), (0.40, 0.0), (0.40, 0.3), (0.40, 0.5)):
            upper(waist_pitch_joint=wp) if wp else upper()
            fell, rec = hold([0.0, 0.0, 0.0, h], 4.0, keep_from=3.0)
            sim_t += 4.0
            zs = (robot.data.body_pos_w[0, hands, 2] - env.scene.env_origins[0, 2]).cpu().numpy()
            o = {"hip_cmd": h, "waist_pitch": wp, "pelvis_z": round(float(np.mean([r[0][2] for r in rec])), 3) if rec else None,
                 "lowest_hand_link": robot.body_names[hands[int(np.argmin(zs))]], "lowest_hand_link_z": round(float(zs.min()), 3), "fell": fell}
            out.append(o)
            if fell:
                fall(sim_t, {"mode": mode, "hip_cmd": h, "waist_pitch": wp})
                reset()
                upper()
                hold([0.0, 0.0, 0.0, STAND_H], 2.0)
        upper()
        rep["reach"] = out
        print("[walk] reach:" + " ".join("骨盆高令%.2f 腰%.1f → 骨盆 %s、手最低 %.3f m(%s)%s" % (o["hip_cmd"], o["waist_pitch"], o["pelvis_z"], o["lowest_hand_link_z"],
                                                                                o["lowest_hand_link"], " 摔" if o["fell"] else "") for o in out), flush=True)
    elif mode == "reach2":
        # 手最低能到多低,胳膊也摆一摆:骨盆高令 0.40(蹲到它能蹲的最低),腰 0 / +0.5 rad,两条胳膊一样摆 ——
        # 肩前后(shoulder_pitch)× 肘(elbow)几档,每档站 2 秒,量手那几节最低在哪、摔没摔(胳膊往前伸重心跟着往前)
        hands = [i for i, n in enumerate(robot.body_names) if "hand" in n]
        out = []
        W_, S_, E_ = [[float(v) for v in g.split(",")] for g in args.reach_grid.split(";")]
        for wp in W_:
            for sp in S_:
                for el in E_:
                    arms = {"left_shoulder_pitch_joint": sp, "right_shoulder_pitch_joint": sp, "left_elbow_joint": el, "right_elbow_joint": el}
                    if args.policy == "groot":
                        # 腰归 GR00T 管:往前弯给它的躯干 pitch 命令(它观测里的那一项),不直接写腰的关节
                        RPY[0, 1] = wp
                        upper(**arms)
                    else:
                        upper(waist_pitch_joint=wp, **arms)
                    fell, rec = hold([0.0, 0.0, 0.0, args.reach_h], 2.0, keep_from=1.5)
                    sim_t += 2.0
                    zs = (robot.data.body_pos_w[0, hands, 2] - env.scene.env_origins[0, 2]).cpu().numpy()
                    out.append({"waist_pitch": wp, "shoulder_pitch": sp, "elbow": el, "lowest_hand_link_z": round(float(zs.min()), 3),
                                "pelvis_z": round(float(np.mean([r[0][2] for r in rec])), 3) if rec else None, "fell": fell})
                    if fell:
                        fall(sim_t, {"mode": mode, "waist_pitch": wp, "shoulder_pitch": sp, "elbow": el})
                        reset()
                        upper()
                        RPY.zero_()
                        hold([0.0, 0.0, 0.0, STAND_H], 2.0)
        upper()
        RPY.zero_()
        rep["reach2"] = out
        ok = [o for o in out if not o["fell"]]
        best = min(ok, key=lambda o: o["lowest_hand_link_z"]) if ok else None
        print("[walk] reach2:%d 档里摔了 %d 档;站得住的里手最低 %s" % (len(out), len(out) - len(ok), best), flush=True)
    elif mode == "stairs":
        # 上去了 = 骨盆走到最上面一级往里 0.5 m,而且两只脚(脚那一节)都比开局高出台阶总高 − 1 cm(站在最上面那一层上)。
        # 不按骨盆升了多少判:走起来的站姿和站着不一样(velocity_height_g1 站着骨盆 0.67、走起来 0.76),骨盆升高掺着站姿
        top = args.steps * args.step_h
        goal = STEP_X0 + (args.steps - 1) * STEP_D + 0.5
        feet = [robot.body_names.index(n) for n in ("left_ankle_roll_link", "right_ankle_roll_link")]
        rec = []
        s0, p0 = along()
        z0 = float(p0[2])
        f0 = robot.data.body_pos_w[0, feet, 2].cpu().numpy() - env.scene.env_origins[0, 2].item()
        k = 0
        res = "没走到"
        frise = np.zeros(2)
        while k * dt < args.minutes * 60:
            do_step([[0.4, 0.0, 0.0, STAND_H]])
            k += 1
            p, tilt, fallen, v, w = state()
            s, _ = along()
            frise = robot.data.body_pos_w[0, feet, 2].cpu().numpy() - env.scene.env_origins[0, 2].item() - f0
            if k % 25 == 0:
                rec.append({"t_s": round(k * dt, 2), "along_m": round(s, 3), "pelvis_z": round(float(p[0, 2]), 3), "tilt_deg": round(float(tilt[0]), 1),
                            "feet_rise_m": [round(float(x), 3) for x in frise]})
            if fallen[0]:
                res = "摔了"
                fall(k * dt, {"mode": mode, "along_m": round(s, 3)})
                break
            if s >= goal and float(frise.min()) > top - 0.01:
                res = "上去了"
                break
        sim_t += k * dt
        rep["stairs"] = {"steps": args.steps, "step_h_m": args.step_h, "step_depth_m": STEP_D, "result": res, "t_s": round(k * dt, 2),
                         "pelvis_rise_m": round(float(p[0, 2]) - z0, 3), "feet_rise_m": [round(float(x), 3) for x in frise], "along_m": round(s, 3),
                         "track": rec}
        print("[walk] stairs:%d 级 × %.2f m —— %s(%.1f s,往前 %.2f m,两只脚升了 %s m,骨盆升了 %.3f m)" % (
            args.steps, args.step_h, res, k * dt, s, [round(float(x), 3) for x in frise], float(p[0, 2]) - z0), flush=True)
rep["sim_s"] = round(sim_t, 1)
rep["wall_s"] = round(time.time() - t0, 1)
json.dump(rep, open(os.path.join(OUT, "report.json"), "w"), indent=1, ensure_ascii=False)
print("[walk] %s / %s:仿真 %.0f s(墙钟 %.0f s)· 一共摔了 %d 回" % (args.policy, args.mode, rep["sim_s"], rep["wall_s"], len(rep["falls"])), flush=True)
os._exit(0)
