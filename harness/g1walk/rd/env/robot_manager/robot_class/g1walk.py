import os

import numpy as np

from utils.pipeline_utils import get_embodiment_config_by_robot_type


class _G1WalkSide:
    # body-driver walking humanoid rig (大并行 §2 第 40 条,路 8): one arm chain of the free-standing G1 (one articulation shared by both sides).
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


class G1WalkLeft(_G1WalkSide):
    side = "left"


class G1WalkRight(_G1WalkSide):
    side = "right"
