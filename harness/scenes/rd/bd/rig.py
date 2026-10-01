# -*- coding: utf-8 -*-
"""路 8 新加的身体(第 39 条轮子底盘 wheelarm、第 40 条会走的人形 g1walk)接进 RoboDojo 要补的几处,都在运行时补,RoboDojo 原有的文件一个字不动。
任务模块被导入的时候调(那时评测环境、机器人管理器都还没建):
  register(名字, 模块, 类名…)       robot_manager 的两张登记表加上它(机器人类在 env/robot_manager/robot_class/<模块>.py,配置在 robot_config/<模块>.py)
  no_planner(名字)                  不给它建 cuRobo 规划器(驱动只发关节目标,用不着;它要的碰撞球、URDF 这些身体没有)
  dims(名字)                         评测环境核动作维数时查的 env_cfg/robot/_robot_info.json 里没有它 ⇒ 按它自己的机器人配置数(每条胳膊几个关节、
                                     每只手几个关节:夹爪一个,手按 gripper_joints_name 的个数)
"""
import os
import sys

import yaml

from env.robot_manager import robot_manager as _rm


def register(name, module, classes):
    _rm.ROBOT_CLASS_REGISTRY.setdefault(name, {"module": module, "classes": tuple(classes)})
    _rm.ROBOT_CONFIG_REGISTRY.setdefault(name, module)


_NO_PLANNER = set()


def no_planner(name):
    _NO_PLANNER.add(name)
    if getattr(_rm.RobotManager, "_bd_no_planner", False):
        return
    orig = _rm.RobotManager._setup_planner

    def _setup_planner(self, robot):
        if getattr(robot, "robot_name", None) in _NO_PLANNER:
            return
        return orig(self, robot)

    _rm.RobotManager._setup_planner = _setup_planner
    _rm.RobotManager._bd_no_planner = True


_DIMS = set()


def dims(name):
    _DIMS.add(name)
    ee = sys.modules.get("src.eval_client.eval_env")
    if ee is None or getattr(ee, "_bd_dims", False):
        return
    orig = ee.get_robot_action_dim_info

    def _dims(env_cfg):
        rn = env_cfg["config"]["robot"]
        if rn not in _DIMS:
            return orig(env_cfg)
        from env.global_configs import ROBOTS_PATH
        cfg = yaml.safe_load(open(os.path.join(ROBOTS_PATH, rn, "robot_config.yml")))
        sides = [cfg["sides"][s] for s in ("left", "right") if s in cfg["sides"]]
        hand = cfg.get("ee_type", "gripper") == "hand"
        return {"arm_dim": [len(s["arm_joints_name"]) for s in sides],
                "ee_dim": [len(s["gripper_joints_name"]) if hand else 1 for s in sides]}

    ee.get_robot_action_dim_info = _dims
    ee._bd_dims = True
