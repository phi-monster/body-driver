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


def joint_group(cls, key, joint_names):
    """身体上不归 RoboDojo 那几条胳膊、那几只手管的一组关节(比如轮子),照实单独报一组、单独收一组:
    读数里加 state.<key>(这几个关节此刻的位置)、action.<key>(上一拍给的目标,命令的回声);动作里的 <key> 先拿下来(RoboDojo 核动作时不认它),
    当这几个关节的位置目标写进仿真。目标一直保持到下一回给;复位时目标 = 此刻的位置(站着不动)。cls 是评测环境的类(type(self))"""
    if key in getattr(cls, "_bd_groups", {}):
        return
    import numpy as np
    import torch
    groups = dict(getattr(cls, "_bd_groups", {}))
    groups[key] = list(joint_names)
    cls._bd_groups = groups
    if getattr(cls, "_bd_group_patched", False):
        return
    orig_take, orig_obs, orig_reset = cls.take_action, cls.get_obs_batch, cls.reset

    def _ids(self, k):
        cache = self.__dict__.setdefault("_bd_group_ids", {})
        if k not in cache:
            art = self.robot_manager.robot_key[0]
            cache[k] = art.find_joints(self._bd_groups[k], preserve_order=True)[0]
        return cache[k]

    def _targets(self):
        return self.__dict__.setdefault("_bd_group_targets", {})

    def take_action(self, action):
        a = dict(action)
        art = self.robot_manager.robot_key[0]
        for k in self._bd_groups:
            v = a.pop(k, None)
            if v is not None:
                t = torch.as_tensor(np.asarray(v, dtype=np.float32).reshape(1, -1), device=art.device)
                _targets(self)[k] = t
                art.set_joint_position_target(t, joint_ids=_ids(self, k))
        return orig_take(self, a)

    def get_obs_batch(self, env_idx_list=None, last_frame=False):
        out = orig_obs(self, env_idx_list=env_idx_list, last_frame=last_frame)
        art = self.robot_manager.robot_key[0]
        for d in out:
            e = int(d.get("env_idx", 0))
            for k in self._bd_groups:
                ids = _ids(self, k)
                q = art.data.joint_pos[e, ids].cpu().numpy().astype(np.float32)
                t = _targets(self).get(k)
                d.setdefault("state", {})[k] = q
                d.setdefault("action", {})[k] = t[0].cpu().numpy().astype(np.float32) if t is not None else q.copy()
        return out

    def reset(self, seed=None, options=None):
        out = orig_reset(self, seed=seed, options=options)
        art = self.robot_manager.robot_key[0]
        for k in self._bd_groups:
            ids = _ids(self, k)
            t = art.data.joint_pos[:, ids].clone()
            _targets(self)[k] = t
            art.set_joint_position_target(t, joint_ids=ids)
        return out

    cls.take_action, cls.get_obs_batch, cls.reset = take_action, get_obs_batch, reset
    cls._bd_group_patched = True


def real_ee_readings(cls):
    """手指 / 夹爪的读数报真的:RoboDojo 的 obs_manager 报 state.*ee_joint_state 用的是上一拍的命令(control_manager.prev_control),
    不是手指此刻在哪 —— 推到 24 读数跟着到 24,手指其实早到头了(第 39 条开机炮:"推到 24.0696 读数跟着走,哪只眼里都没变")。
    真机的夹爪报的是它自己编码器的位置。这里把读数换成手指关节此刻的位置,单位和命令一样:
    夹爪(gripper)= 带头的那个关节的位置按 gripper_scale 折回 0 – 1(和命令同一个折法);手(hand)= 每个关节的位置(弧度,和命令一样)。
    命令的回声(action.*ee_joint_state)照旧是上一拍的命令"""
    if getattr(cls, "_bd_real_ee", False):
        return
    import numpy as np
    orig_obs = cls.get_obs_batch

    def get_obs_batch(self, env_idx_list=None, last_frame=False):
        out = orig_obs(self, env_idx_list=env_idx_list, last_frame=last_frame)
        rm = self.robot_manager
        for robot in rm.robot_list:
            if getattr(robot, "type", "target") != "target":
                continue
            key = rm.process_name(robot.gripper_name)
            art = rm.robot_key[rm.robot_list.index(robot)]
            for d in out:
                st = d.get("state")
                if st is None or key not in st:
                    continue
                e = int(d.get("env_idx", 0))
                q = art.data.joint_pos[e, robot.gripper_joint_indices].cpu().numpy().astype(np.float64)
                if robot.ee_type == "gripper":
                    base = robot.gripper_joints_name.index(robot.gripper_move["base"])
                    lo, hi = robot.gripper_scale
                    v = (q[base] - lo) / (hi - lo) if robot.gripper_move["sign"] == 1 else (hi - q[base]) / (hi - lo)
                    st[key] = [float(v)]
                else:
                    st[key] = q.astype(np.float32)
        return out

    cls.get_obs_batch = get_obs_batch
    cls._bd_real_ee = True
