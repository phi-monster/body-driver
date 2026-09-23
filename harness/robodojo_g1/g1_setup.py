# -*- coding: utf-8 -*-
# body-driver humanoid test rig (2026-09-23): integrate Unitree G1 + Inspire hands into RoboDojo and add a moving-mouse task.
import os, json, yaml, shutil
R = "/root/RoboDojo"; G = f"{R}/Assets/Robots/g1"

def arm(s): return [f"{s}_shoulder_pitch_joint", f"{s}_shoulder_roll_joint", f"{s}_shoulder_yaw_joint", f"{s}_elbow_joint", f"{s}_wrist_roll_joint", f"{s}_wrist_pitch_joint", f"{s}_wrist_yaw_joint"]
def armlinks(s): return [f"{s}_shoulder_pitch_link", f"{s}_shoulder_roll_link", f"{s}_shoulder_yaw_link", f"{s}_elbow_link", f"{s}_wrist_roll_link", f"{s}_wrist_pitch_link", f"{s}_wrist_yaw_link"]
def hand(P): return [f"{P}_index_proximal_joint", f"{P}_index_intermediate_joint", f"{P}_middle_proximal_joint", f"{P}_middle_intermediate_joint", f"{P}_ring_proximal_joint", f"{P}_ring_intermediate_joint", f"{P}_pinky_proximal_joint", f"{P}_pinky_intermediate_joint", f"{P}_thumb_proximal_yaw_joint", f"{P}_thumb_proximal_pitch_joint", f"{P}_thumb_intermediate_joint", f"{P}_thumb_distal_joint"]

# ---- robot_config.yml (common + per side)
def side(s, P):
    return dict(ee_joints=f"{s}_wrist_yaw_joint", ee_link=f"{s}_wrist_yaw_link", arm_joints_name=arm(s), gripper_joints_name=hand(P),
                gripper_move=dict(base=hand(P)[0], sign=-1.0, mimic=[[j, 1.0, 0.0] for j in hand(P)[1:]]),
                gripper_bias=0.0, gripper_scale=[0.0, 1.5], curobo=f"curobo_{s}.yml",
                camera=[dict(link=f"{s}_wrist_yaw_link", name="cam_wrist", type="d435", mesh="pinhole", pos=[0.0, 0.0, 0.08], ori=[0, -90, 0])])
cfg = dict(urdf_path="./g1.urdf", base_link="pelvis", ee_type="gripper", dual_arm=True, delta_matrix=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
           global_trans_matrix=[[1, 0, 0], [0, -1, 0], [0, 0, -1]], grasp_camera_reference_axis=[1, 0, 0],
           sides=dict(left=side("left", "L"), right=side("right", "R")))
open(f"{G}/robot_config.yml", "w").write("# Unitree G1 (29 DoF) with Inspire 5-finger hands: one articulation, two arm chains. body-driver humanoid test, 2026-09-23.\n" + yaml.safe_dump(cfg, sort_keys=False))

# ---- curobo ymls (IK for each arm chain; pelvis base, waist locked)
def curobo(s):
    links = ["pelvis", "torso_link"] + armlinks(s)
    spheres = {l: [dict(center=[0.0, 0.0, 0.0], radius=0.04)] for l in links}
    spheres["torso_link"] = [dict(center=[0.0, 0.0, 0.1], radius=0.09), dict(center=[0.0, 0.0, 0.25], radius=0.09)]
    # ignore self-collision only inside the two link groups that overlap by construction (spheres sit at joint origins);
    # everything else (torso/shoulders vs elbow/wrists) stays checked, so curobo has pairs to check (all-ignored ⇒ its debug log divides by zero)
    upper = ["pelvis", "torso_link"] + armlinks(s)[:3]
    lower = armlinks(s)[3:]
    ignore = {}
    for grp in (upper, lower, [armlinks(s)[2], armlinks(s)[3]]):
        for l in grp:
            ignore.setdefault(l, [])
            ignore[l] += [m for m in grp if m != l and m not in ignore[l]]
    n = 7
    kin = dict(add_object_link=False, asset_root_path=G, base_link="pelvis", collision_link_names=links, collision_sphere_buffer=0.0,
               collision_spheres=spheres,
               cspace=dict(acceleration_scale=[1.0] * n, cspace_distance_weight=[1.0] * n, default_joint_position=[0.0] * n,
                           jerk_scale=[1.0] * n, joint_names=arm(s), max_acceleration=[10.0] * n, max_jerk=[500.0] * n,
                           null_space_maximum_distance=[1.0] * n, null_space_weight=[1.0] * n, position_limit_clip=0.0,
                           velocity_scale=[1.0] * n, retract_config=[0.0] * n),
               debug=None, ee_link=f"{s}_wrist_yaw_link", external_asset_path=None, external_robot_configs_path=None,
               extra_collision_spheres=None, extra_links={}, format_version=2.0, grasp_contact_link_names=None, load_meshes=False,
               load_tool_frames_with_mesh=False, lock_joints={"waist_yaw_joint": 0.0, "waist_roll_joint": 0.0, "waist_pitch_joint": 0.0},
               mesh_link_names=links, self_collision_buffer={l: -0.01 for l in links}, self_collision_ignore=ignore,
               tool_frames=[f"{s}_wrist_yaw_link"], urdf_path=f"{G}/g1.urdf", use_external_assets=False, use_global_cumul=True)
    open(f"{G}/curobo_{s}.yml", "w").write(yaml.safe_dump(dict(robot_cfg=dict(kinematics=kin), planner=dict(frame_bias=[0.0, 0.0, 0.0])), sort_keys=False))
curobo("left"); curobo("right"); shutil.copy(f"{G}/curobo_left.yml", f"{G}/curobo.yml")

# ---- robot_class/g1.py
CLASS = """import os

import numpy as np

from utils.pipeline_utils import get_embodiment_config_by_robot_type


class _G1Side:
    # One arm chain of the Unitree G1 humanoid (one articulation shared by both sides: a coupled robot).
    side = "left"

    def __init__(self, cfg: dict):
        self.robot_type = cfg.get("robot_type", None)
        self.robot_name = cfg.get("robot_name", None)
        self.is_coupled = cfg.get("coupled", True)
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
        self.base_link = common.get("base_link", "pelvis")
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


class G1Left(_G1Side):
    side = "left"


class G1Right(_G1Side):
    side = "right"
"""
open(f"{R}/env/robot_manager/robot_class/g1.py", "w").write(CLASS)

# ---- robot_config/g1.py (ArticulationCfg)
ROBOTCFG = """from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import isaaclab.sim as sim_utils

from env.global_configs import ROBOTS_PATH


def get_robot_config():
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ROBOTS_PATH}/g1/g1_29dof_inspire_hand.usd",
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
            joint_pos={
                ".*": 0.0,
                ".*_elbow_joint": 1.2,
                "left_shoulder_roll_joint": 0.25,
                "right_shoulder_roll_joint": -0.25,
            },
            joint_vel={".*": 0.0},
            pos=(0.0, -0.75, 0.8),
            rot=(0.707, 0.0, 0.0, 0.707),
        ),
        actuators={
            "legs_waist": ImplicitActuatorCfg(
                joint_names_expr=[".*_hip_.*", ".*_knee_joint", ".*_ankle_.*", "waist_.*"],
                effort_limit_sim=300.0, velocity_limit_sim=10.0, stiffness=2000.0, damping=100.0, armature=0.01,
            ),
            "arms": ImplicitActuatorCfg(
                joint_names_expr=[".*_shoulder_.*", ".*_elbow_joint", ".*_wrist_.*"],
                effort_limit_sim=300.0, velocity_limit_sim=10.0, stiffness=3000.0, damping=100.0, armature=0.001,
            ),
            "hands": ImplicitActuatorCfg(
                joint_names_expr=[".*_index_.*", ".*_middle_.*", ".*_thumb_.*", ".*_ring_.*", ".*_pinky_.*"],
                effort_limit_sim=30.0, velocity_limit_sim=10.0, stiffness=10.0, damping=0.2, armature=0.001,
            ),
        },
    )
"""
open(f"{R}/env/robot_manager/robot_config/g1.py", "w").write(ROBOTCFG)

# ---- registries
p = f"{R}/env/robot_manager/robot_manager.py"; s = open(p).read()
if '"g1"' not in s:
    s = s.replace('    "x5": {\n        "module": "x5",\n        "classes": ("X5",),\n    },\n}',
                  '    "x5": {\n        "module": "x5",\n        "classes": ("X5",),\n    },\n    "g1": {\n        "module": "g1",\n        "classes": ("G1Left", "G1Right"),\n    },\n}', 1)
    s = s.replace('ROBOT_CONFIG_REGISTRY = {\n    "franka": "franka",\n    "x5": "x5",\n}',
                  'ROBOT_CONFIG_REGISTRY = {\n    "franka": "franka",\n    "x5": "x5",\n    "g1": "g1",\n}', 1)
    open(p, "w").write(s)
assert '"g1"' in open(p).read()

# ---- control_manager: N mimic joints + sign-aware command
p = f"{R}/env/robot_manager/control_manager.py"; s = open(p).read()
old = ('        def process_gripper_val(robot_manager, robot, position, gripper_eps=0.2, env_idx=None):\n'
       '            real_gripper_val = robot_manager.get_end_effector_real_val(robot, env_idx_list=[env_idx])[env_idx]\n'
       '            real_gripper_val = real_gripper_val[0]\n'
       '            scale = robot.gripper_scale')
new = old + ('\n            # [bd] hands whose joint closes toward the upper limit (sign -1) are observed normalised (1 = open, 0 = closed);\n'
             '            # the command arrives in that same normalised space, so map it back to a joint position here.\n'
             '            if robot.gripper_move.get("sign", 1) == -1:\n'
             '                position = scale[1] - float(position) * (scale[1] - scale[0])')
if "[bd] hands whose joint closes" not in s:
    assert s.count(old) == 1; s = s.replace(old, new)
old2 = '            return [val, val * robot.gripper_move["mimic"][1] + robot.gripper_move["mimic"][2]]'
new2 = ('            mimic = robot.gripper_move["mimic"]\n'
        '            if mimic and isinstance(mimic[0], (list, tuple)):   # [bd] N mimic joints: [[name, scale, offset], ...]\n'
        '                return [val] + [val * m[1] + m[2] for m in mimic]\n'
        '            return [val, val * mimic[1] + mimic[2]]')
if "[bd] N mimic joints" not in s:
    assert s.count(old2) == 1; s = s.replace(old2, new2)
open(p, "w").write(s)

# ---- _robot_info.json
p = f"{R}/env_cfg/robot/_robot_info.json"; d = json.load(open(p)); d["g1"] = {"arm_dim": [7, 7], "ee_dim": [1, 1]}; json.dump(d, open(p, "w"), indent=4)

# ---- env_cfg/robot/g1.yml + env_cfg/g1_mouse.yml
open(f"{R}/env_cfg/robot/g1.yml", "w").write("""# Unitree G1 humanoid (fixed pelvis, legs hanging), standing behind the near table edge facing the table.
robots:
  - {
    robot_type: arm,
    robot_name: g1,
    coupled: True,
    default_root_pos: [0.0, -0.75, 0.8],
    default_root_rot: [0.707, 0, 0, 0.707],
    grasp_perfect_direction: "top_down",
    enabled_self_collisions: False
  }
""")
open(f"{R}/env_cfg/g1_mouse.yml", "w").write("""# body-driver humanoid test: G1 + Inspire hands, official scene/table/cameras, depth/intrinsics on (the driver ignores depth).
config_name: g1

config:
  sim: sim_config
  scene: default
  robot: g1
  camera: camera_rgbd

observation:
  collect_freq: 25
  robot:
    joint_states: true
    world_ee_state: true
  vision:
    approximate_depth: false
    depth: true
    intrinsic_matrix: true
    extrinsic_matrix: true
    shape: true
""")

# ---- task chase_mouse
TASK = """import os
import random

import numpy as np

from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager


class ChaseMouseCommon:
    # body-driver test (2026-09-23): the target mouse wanders randomly on the table; success = lifted 0.1 m (official judge).

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = int(os.environ.get("BD_STEP_LIM", "3000"))
        self._mouse_vel = {}
        self._mouse_step = 0
        self._speed = float(os.environ.get("BD_MOUSE_SPEED", "0.01"))   # metres per env step
        self._bounds = ((-0.35, 0.35), (-0.45, -0.15))

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()
        self._mouse_vel = {}
        self._mouse_step = 0

    def _wander(self):
        lm = self.scene_manager.layout_manager
        for env_idx in range(self.num_envs):
            name = lm.get_instance_name(env_idx, "target")
            if name is None:
                continue
            obj = lm.get_scene_object(env_idx, name)
            if obj is None:
                continue
            pos, rot = obj.get_local_pose()
            p = np.array(pos, dtype=float).reshape(-1)[:3]
            if env_idx not in self._mouse_vel or self._mouse_step % 40 == 0:
                a = random.uniform(0, 2 * np.pi)
                self._mouse_vel[env_idx] = np.array([np.cos(a), np.sin(a)]) * self._speed
            v = self._mouse_vel[env_idx]
            nx, ny = p[0] + v[0], p[1] + v[1]
            (x0, x1), (y0, y1) = self._bounds
            if nx < x0 or nx > x1:
                v[0] = -v[0]
                nx = min(max(nx, x0), x1)
            if ny < y0 or ny > y1:
                v[1] = -v[1]
                ny = min(max(ny, y0), y1)
            obj.set_local_pose(translation=np.array([nx, ny, p[2]]), orientation=np.array(rot, dtype=float).reshape(-1)[:4])
        self._mouse_step += 1

    def step(self, meta_control_list):
        self._wander()
        super().step(meta_control_list)

    def run_reward(self):
        self.reward_manager.check([self.reward_manager.is_lift(label="target", z_threshold=0.1)])

    def gen_instruction(self, env_idx):
        return ["Catch the mouse that is running around on the table and lift it 10 cm."]


class chase_mouse(ChaseMouseCommon, TaskEnv):
    pass
"""
open(f"{R}/task/RoboDojo/tasks/chase_mouse.py", "w").write(TASK)

# ---- layout for config g1, seed 1
src = json.load(open(f"{R}/Assets/Eval_Layout/RoboDojo/arx_x5/1/general_pickup_1.json"))
m = src["Rigid"]["mouse"][0]; m["default_pos"] = [0.0, -0.3, 0.7818]; m["label"] = "target"
lay = dict(src); lay["Rigid"] = {"mouse": [m]}
os.makedirs(f"{R}/Assets/Eval_Layout/RoboDojo/g1/1", exist_ok=True)
json.dump(lay, open(f"{R}/Assets/Eval_Layout/RoboDojo/g1/1/chase_mouse_0.json", "w"))
print("written:", sorted(os.listdir(G)), os.listdir(f"{R}/Assets/Eval_Layout/RoboDojo/g1/1"))
