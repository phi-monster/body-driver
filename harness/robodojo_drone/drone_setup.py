# -*- coding: utf-8 -*-
# body-driver drone test rig (2026-09-24): a flying camera in RoboDojo, built the same way as the humanoid rig (sim-side shim, zero driver code).
# The "drone" is a small body hanging from a virtual 6-DoF gantry (x/y/z prismatic + yaw/pitch/roll revolute) fixed above the table:
# RoboDojo/curobo see a 6-joint arm whose end link IS the body; the driver sees one "arm" that reports a pose and takes pose commands,
# one camera on the body, and NO grip channel at all (ee_type "none", 10-01; 大并行 §5 路 1 "无人机不报抓握也开得了机").
# Until 10-01 the rig reported a grip channel that moved nothing (a 1 cm placeholder joint inside the body); the driver measured it as a
# mute group. A drone has no grip, so the observation now carries none and the action takes none. The placeholder joint stays in the
# URDF only because RoboDojo's robot manager indexes one end-effector joint per arm (find_joints, control tensors); nobody sees or commands it.
# Run on the box with the RoboDojo venv python:  python drone_setup.py   then convert the URDF to USD (needs Isaac):
#   cd /root/RoboDojo && /venv/RoboDojo/bin/python third_party/IsaacLab/scripts/tools/convert_urdf.py Assets/Robots/drone/drone.urdf \
#       Assets/Robots/drone/drone.usd --fix-base --headless
# RD=<dir> writes into a copy of the RoboDojo tree instead (dry run: diff it against the live tree before touching the live one).
import os, json, yaml, shutil

R = os.environ.get("RD", "/root/RoboDojo"); D = f"{R}/Assets/Robots/drone"
os.makedirs(D, exist_ok=True)

# ---- URDF: base_link (fixed) -> x -> y -> z -> yaw -> pitch -> roll -> body_link (+ the placeholder joint inside the body, see above)
JOINTS = ["gx_joint", "gy_joint", "gz_joint", "yaw_joint", "pitch_joint", "roll_joint"]
def link(name, size="0.02 0.02 0.02", mass=0.05):
    return f"""  <link name="{name}">
    <inertial><mass value="{mass}"/><origin xyz="0 0 0"/><inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><box size="{size}"/></geometry></visual>
    <collision><geometry><box size="{size}"/></geometry></collision>
  </link>
"""
def joint(name, parent, child, jtype, axis, lo, hi, vel="5.0", effort="200.0"):
    return f"""  <joint name="{name}" type="{jtype}">
    <parent link="{parent}"/><child link="{child}"/><origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="{axis}"/><limit lower="{lo}" upper="{hi}" effort="{effort}" velocity="{vel}"/>
  </joint>
"""
urdf = '<?xml version="1.0"?>\n<robot name="drone">\n'
urdf += link("base_link", "0.02 0.02 0.02")
chain = ["base_link", "gx_link", "gy_link", "gz_link", "yaw_link", "pitch_link"]
for l in chain[1:]:
    urdf += link(l)
urdf += link("body_link", "0.12 0.12 0.03", mass=0.5)
urdf += link("grip_link", "0.01 0.01 0.01", mass=0.01)
urdf += joint("gx_joint", "base_link", "gx_link", "prismatic", "1 0 0", "-1.0", "1.0")
urdf += joint("gy_joint", "gx_link", "gy_link", "prismatic", "0 1 0", "-1.0", "1.0")
urdf += joint("gz_joint", "gy_link", "gz_link", "prismatic", "0 0 1", "-1.0", "0.5")
urdf += joint("yaw_joint", "gz_link", "yaw_link", "revolute", "0 0 1", "-3.14", "3.14")
urdf += joint("pitch_joint", "yaw_link", "pitch_link", "revolute", "0 1 0", "-1.5", "1.5")
urdf += joint("roll_joint", "pitch_link", "body_link", "revolute", "1 0 0", "-1.5", "1.5")
urdf += joint("grip_joint", "body_link", "grip_link", "revolute", "0 0 1", "0.0", "0.01", vel="1.0", effort="1.0")
urdf += "</robot>\n"
open(f"{D}/drone.urdf", "w").write(urdf)

# ---- robot_config.yml (one side; the body is the end link; camera on the body looking straight down)
# ori: RoboDojo builds the mount with euler_angles_to_quat(ori, degrees) on a USD camera, which looks along its own -Z
# (franka robot_config.yml measured it: ori [45,0,0] puts the +Z axis at (0,-0.707,0.707)). body_link is level at joint zero, so
# [0,0,0] looks straight down at the table. The first version had [0,-90,0] = looking along +x, level: DR1 (09-28) saw only the room wall
# and the driver fitted its "table" to that wall (normal 79 deg off vertical).
side = dict(ee_joints="roll_joint", ee_link="body_link", arm_joints_name=JOINTS, gripper_joints_name=["grip_joint"],
            gripper_move=dict(base="grip_joint", sign=1.0, mimic=[]), gripper_bias=0.0, gripper_scale=[0.0, 0.01], curobo="curobo_left.yml",
            camera=[dict(link="body_link", name="cam_wrist", type="d435", mesh="pinhole", pos=[0.0, 0.0, -0.03], ori=[0, 0, 0])])
# ee_type "none" (10-01): no end effector is reported or commanded (the shim below teaches RoboDojo's managers this third kind);
# the gripper_* fields only describe the placeholder joint RoboDojo indexes
cfg = dict(urdf_path="./drone.urdf", base_link="base_link", ee_type="none", dual_arm=False, delta_matrix=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
           global_trans_matrix=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], grasp_camera_reference_axis=[1, 0, 0], sides=dict(left=side))
open(f"{D}/robot_config.yml", "w").write("# body-driver drone rig (2026-09-24): a body on a virtual 6-DoF gantry above the table; one camera on the body; no grip (ee_type none, 10-01).\n" + yaml.safe_dump(cfg, sort_keys=False))

# ---- curobo (6-joint chain, prismatic + revolute)
links = chain + ["body_link"]
spheres = {l: [dict(center=[0.0, 0.0, 0.0], radius=0.02)] for l in links}
spheres["body_link"] = [dict(center=[0.0, 0.0, 0.0], radius=0.07)]
# curobo 要至少一对自碰撞要查,否则它的调试日志除以零(DR1 2026-09-24 实测 ZeroDivisionError);虚拟龙门吊的各节都叠在机身上,
# 查哪对都会把机身拦住 ⇒ 底座的球挪到 5 m 外(永远碰不到),只留"底座 × 机身"这一对要查
spheres["base_link"] = [dict(center=[0.0, 0.0, 5.0], radius=0.02)]
ignore = {l: [m for m in links if m != l and not ({l, m} == {"base_link", "body_link"})] for l in links}
n = 6
kin = dict(add_object_link=False, asset_root_path=D, base_link="base_link", collision_link_names=links, collision_sphere_buffer=0.0,
           collision_spheres=spheres,
           cspace=dict(acceleration_scale=[1.0] * n, cspace_distance_weight=[1.0] * n, default_joint_position=[0.0] * n,
                       jerk_scale=[1.0] * n, joint_names=JOINTS, max_acceleration=[10.0] * n, max_jerk=[500.0] * n,
                       null_space_maximum_distance=[1.0] * n, null_space_weight=[1.0] * n, position_limit_clip=0.0,
                       velocity_scale=[1.0] * n, retract_config=[0.0] * n),
           debug=None, ee_link="body_link", external_asset_path=None, external_robot_configs_path=None,
           extra_collision_spheres=None, extra_links={}, format_version=2.0, grasp_contact_link_names=None, load_meshes=False,
           load_tool_frames_with_mesh=False, lock_joints={},   # 假夹爪关节长在机身外、不在 curobo 的链上,锁它会 KeyError(DR1 2026-09-24)
           mesh_link_names=links, self_collision_buffer={l: -0.01 for l in links}, self_collision_ignore=ignore,
           tool_frames=["body_link"], urdf_path=f"{D}/drone.urdf", use_external_assets=False, use_global_cumul=True)
open(f"{D}/curobo_left.yml", "w").write(yaml.safe_dump(dict(robot_cfg=dict(kinematics=kin), planner=dict(frame_bias=[0.0, 0.0, 0.0])), sort_keys=False))
shutil.copy(f"{D}/curobo_left.yml", f"{D}/curobo.yml")

# ---- robot_class/drone.py
CLASS = '''import os

import numpy as np

from utils.pipeline_utils import get_embodiment_config_by_robot_type


class Drone:
    # body-driver drone rig: one 6-DoF chain (virtual gantry) whose end link is the flying body with the camera.
    side = "left"

    def __init__(self, cfg: dict):
        self.robot_type = cfg.get("robot_type", None)
        self.robot_name = cfg.get("robot_name", None)
        self.is_coupled = cfg.get("coupled", False)
        self.default_root_pos = cfg.get("default_root_pos", None)
        self.default_root_rot = cfg.get("default_root_rot", None)
        self.grasp_perfect_direction = cfg.get("grasp_perfect_direction", None)
        self.SceneCfg = None
        self.static_camera_list = None
        self.robot_cfg = get_embodiment_config_by_robot_type(robot_type=self.robot_type, robot_name=self.robot_name)
        self.robot_file = self.robot_cfg["robot_file"]
        common = self.robot_cfg["robot_config"]
        sd = common["sides"][self.side]
        self.robot_args = dict(common)
        self.robot_args.update(sd)
        self.urdf_path = os.path.join(self.robot_file, common.get("urdf_path"))
        self.srdf_path = os.path.join(self.robot_file, common.get("srdf_path", ""))
        self.curobo_yml_path = os.path.join(self.robot_file, sd["curobo"])
        self.ee_joint_name = sd["ee_joints"]
        self.ee_link_name = sd["ee_link"]
        self.arm_joints_name = list(sd["arm_joints_name"])
        self.gripper_move = sd["gripper_move"]
        self.gripper_joints_name = list(sd["gripper_joints_name"])
        self.gripper_bias = sd["gripper_bias"]
        self.gripper_scale = sd["gripper_scale"]
        self.base_link = common.get("base_link", "base_link")
        self.delta_matrix = np.array(common.get("delta_matrix", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        self.grasp_camera_reference_axis = common.get("grasp_camera_reference_axis", [1, 0, 0])
        self.inv_delta_matrix = np.linalg.inv(self.delta_matrix)
        self.global_trans_matrix = np.array(common.get("global_trans_matrix", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        self.ee_type = common.get("ee_type", "gripper")
        self.rotate_lim = common.get("rotate_lim", [0, 0])
        self.entity_origin_pose = self.default_root_pos + self.default_root_rot
        self.camera = sd.get("camera", None)
        self.mesh_dir = common.get("mesh_dir", self.robot_file)
        self.save_gripper_joints_name = sd.get("save_gripper_joints_name", self.gripper_joints_name)
'''
open(f"{R}/env/robot_manager/robot_class/drone.py", "w").write(CLASS)

# ---- robot_config/drone.py (ArticulationCfg)
ROBOTCFG = '''from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import isaaclab.sim as sim_utils

from env.global_configs import ROBOTS_PATH


def get_robot_config():
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ROBOTS_PATH}/drone/drone.usd",
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, max_depenetration_velocity=5.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
                fix_root_link=True,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={".*": 0.0},
            joint_vel={".*": 0.0},
            pos=(0.0, -0.2, 1.4),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        actuators={
            "gantry": ImplicitActuatorCfg(
                joint_names_expr=["gx_joint", "gy_joint", "gz_joint", "yaw_joint", "pitch_joint", "roll_joint"],
                effort_limit_sim=500.0, velocity_limit_sim=5.0, stiffness=5000.0, damping=200.0, armature=0.01,
            ),
            "grip": ImplicitActuatorCfg(   # the placeholder joint inside the body (ee_type none: never reported, never commanded)
                joint_names_expr=["grip_joint"],
                effort_limit_sim=1.0, velocity_limit_sim=1.0, stiffness=10.0, damping=0.5, armature=0.001,
            ),
        },
    )
'''
open(f"{R}/env/robot_manager/robot_config/drone.py", "w").write(ROBOTCFG)

# ---- registries
p = f"{R}/env/robot_manager/robot_manager.py"; s = open(p).read()
if '"drone"' not in s:
    s = s.replace('    "g1": {\n        "module": "g1",\n        "classes": ("G1Left", "G1Right"),\n    },\n}',
                  '    "g1": {\n        "module": "g1",\n        "classes": ("G1Left", "G1Right"),\n    },\n    "drone": {\n        "module": "drone",\n        "classes": ("Drone",),\n    },\n}', 1)
    s = s.replace('    "g1": "g1",\n}', '    "g1": "g1",\n    "drone": "drone",\n}', 1)
    open(p, "w").write(s)
assert '"drone"' in open(p).read(), "registry patch failed"

p = f"{R}/env_cfg/robot/_robot_info.json"; d = json.load(open(p)); d["drone"] = {"arm_dim": [6], "ee_dim": [1]}; json.dump(d, open(p, "w"), indent=4)

open(f"{R}/env_cfg/robot/drone.yml", "w").write("""# body-driver drone rig: virtual gantry base fixed 1.4 m above the table centre; the body hangs at joint zero and flies within the joint ranges.
robots:
  - {
    robot_type: arm,
    robot_name: drone,
    coupled: False,
    default_root_pos: [0.0, -0.2, 1.4],
    default_root_rot: [1.0, 0, 0, 0],
    grasp_perfect_direction: "top_down",
    enabled_self_collisions: False
  }
""")
open(f"{R}/env_cfg/drone_rgb.yml", "w").write("""# body-driver drone rig, official-style observation: RGB + poses only (no depth, no intrinsics, no extrinsics)
config_name: drone

config:
  sim: sim_config
  scene: default
  robot: drone
  camera: camera_config

observation:
  collect_freq: 25
  robot:
    joint_states: true
    world_ee_state: true
  vision:
    approximate_depth: false
    depth: false
    intrinsic_matrix: false
    extrinsic_matrix: false
    shape: true
""")

# ---- 没有联动关节的抓握通道(无人机:抓握通道只有 grip_joint 一个关节,mimic = [])。人形那份垫片(g1_setup.py)把 RoboDojo 里几处
#      "一个基关节 + 联动关节"的写法改成了"一个或 N 个联动",都没管"零个":mimic 为空时下标越界,仿真一开机就崩(09-27 V1BD2)。
#      四处都补上"没有联动 ⇒ 只有基关节那一个目标"(同一句,放在每处最前面)
NO_MIMIC = "if not mimic:   # [bd] no mimic joints (drone grip): the base joint only"
for f, indent, ret in ((f"{R}/env/robot_manager/control_manager.py", "            ", "return [val]"),
                       (f"{R}/env/robot_manager/robot_manager.py", "    ", "return [val]"),
                       (f"{R}/src/eval_client/eval_env.py", "    ", "return [val]")):
    s2 = open(f).read()
    if "[bd] no mimic joints" not in s2:
        key = "\n" + indent + 'mimic = robot.gripper_move["mimic"]\n'   # 行首、正好这一档缩进(更深的那行另补,见下)
        assert s2.count(key) == 1, (f, "mimic line not found once", s2.count(key))
        s2 = s2.replace(key, key + indent + NO_MIMIC + "\n" + indent + "    " + ret + "\n")
        open(f, "w").write(s2)
    print("no-mimic patch:", f.split("/")[-1], s2.count("[bd] no mimic joints"))
p = f"{R}/env/robot_manager/robot_manager.py"; s2 = open(p).read()
old_init = "                    if mimic and isinstance(mimic[0], (list, tuple)):   # [bd] N mimic joints\n"
new_init = ("                    if not mimic:   # [bd] no mimic joints (drone grip): the base joint only\n"
            "                        gripper_list = [[val] for _ in range(len(joint_list))]\n"
            "                    elif mimic and isinstance(mimic[0], (list, tuple)):   # [bd] N mimic joints\n")
if "gripper_list = [[val] for _ in range(len(joint_list))]" not in s2:
    assert s2.count(old_init) == 1, "init-state mimic line not found"
    s2 = s2.replace(old_init, new_init); open(p, "w").write(s2)
print("init-state no-mimic:", "gripper_list = [[val] for _ in range(len(joint_list))]" in open(p).read())

# ---- 不报末端(ee_type "none",10-01):RoboDojo 只认 "gripper" / "hand" 两种末端。动作那一边(eval_env 取动作、插值)和
#      control_manager 本来就是"gripper / hand / 别的照旧不管",别的这一种不用补;要补的只有两处,同一个记号 [bd] no end effector:
#      ① robot_manager.set_robot_init_state 只给前两种算末端的初值,第三种 gripper_list 没定义 ⇒ 给占位关节一个 0(没人看、没人发);
#      ② obs_manager 把上一条命令里的 ee_joint_state 原样放进观测(state 和 action 两处)⇒ 第三种不放:驱动看到的身体没有抓握通道。
#      动作里不带 ee_joint_state 时 control_manager 按上一条命令补齐(update_current_missing_ctrl_info),占位关节一直停在 0
NO_EE = "# [bd] no end effector"
p = f"{R}/env/robot_manager/robot_manager.py"; s2 = open(p).read()
old_hand = ('                elif robot.ee_type == "hand":\n'
            '                    gripper_list = self.get_end_effector_real_val(robot=robot)\n')
new_hand = ('                elif robot.ee_type == "none":   ' + NO_EE + ' (drone rig): the placeholder joint holds 0\n'
            '                    gripper_list = [[0.0] * len(robot.gripper_joints_name) for _ in range(len(joint_list))]\n' + old_hand)
if NO_EE not in s2:
    assert s2.count(old_hand) == 1, ("init-state hand branch not found once", s2.count(old_hand))
    s2 = s2.replace(old_hand, new_hand); open(p, "w").write(s2)
print("init-state no-ee:", NO_EE in open(p).read())
p = f"{R}/env/observation_manager/obs_manager.py"; s2 = open(p).read()
old_obs = ("                                robot = self.robot_manager.get_robot_by_gripper_name(end_effector_name)\n"
           '                                if robot.ee_type == "gripper":\n')
new_obs = ("                                robot = self.robot_manager.get_robot_by_gripper_name(end_effector_name)\n"
           '                                if robot.ee_type == "none":   ' + NO_EE + " (drone rig): not reported\n"
           "                                    continue\n"
           '                                if robot.ee_type == "gripper":\n')
if NO_EE not in s2:
    assert s2.count(old_obs) == 1, ("obs ee branch not found once", s2.count(old_obs))
    s2 = s2.replace(old_obs, new_obs); open(p, "w").write(s2)
print("obs no-ee:", NO_EE in open(p).read())

# ---- layout for config drone, seed 1 (a static pickup table: the drone only calibrates over it)
os.makedirs(f"{R}/Assets/Eval_Layout/RoboDojo/drone/1", exist_ok=True)
shutil.copy(f"{R}/Assets/Eval_Layout/RoboDojo/arx_x5/1/general_pickup_0.json", f"{R}/Assets/Eval_Layout/RoboDojo/drone/1/general_pickup_0.json")
# 开机长场次(bootcal = 同一个场景、同一个判据,只放开集长):同一张桌子(09-27 V1BD1 第一炮起不来:没有 bootcal 的布局)
shutil.copy(f"{R}/Assets/Eval_Layout/RoboDojo/drone/1/general_pickup_0.json", f"{R}/Assets/Eval_Layout/RoboDojo/drone/1/bootcal_0.json")
print("written:", sorted(os.listdir(D)), os.listdir(f"{R}/Assets/Eval_Layout/RoboDojo/drone/1"))
