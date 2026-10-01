# -*- coding: utf-8 -*-
# body-driver 第 39 条(路 8):轮子底盘 + 一条胳膊,地上一只乱跑、会躲的老鼠。由 harness/wheelarm 装进来,别手改。
# 老鼠(RoboDojo 自带的 mouse 资产)在地上按 task/RoboDojo/bd/scene.py 的 Walker 走:一个动作 1 cm(owner:"老鼠随机乱跑(1 cm/步)")、
# 每 40 个动作随机换方向、碰边反射;身体哪一节进了 flee_radius 就朝正背着它跑("还会躲");被拿离地面 5 mm 以上就不走。
# 判据:老鼠离开局高 > 10 cm(RoboDojo 的 is_lift;"30 秒内抓起来、离地 10 cm")。一集 30 秒 = 750 个动作(25 Hz;BD_STEP_LIM 可改)。
#
# 这具身体不在 RoboDojo 的机器人登记里:这里(任务模块被导入的时候,机器人管理器、评测环境都还没建)补三处,RoboDojo 原有的文件一个字不动:
#   ① robot_manager 的两张登记表加上它;② 不给它建 cuRobo 规划器(驱动只发关节目标,用不着;它要的碰撞球、URDF 这具身体没有);
#   ③ 评测环境核动作维数时查的 env_cfg/robot/_robot_info.json 里没有它 ⇒ 按它自己的机器人配置数(胳膊那一串几个关节、夹爪一个)。
import os
import sys

import yaml

from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager
from env.robot_manager import robot_manager as _rm
from task.RoboDojo.bd import scene

_rm.ROBOT_CLASS_REGISTRY.setdefault("wheelarm", {"module": "wheelarm", "classes": ("WheelArm",)})
_rm.ROBOT_CONFIG_REGISTRY.setdefault("wheelarm", "wheelarm")
if not getattr(_rm.RobotManager, "_bd_wheelarm_planner", False):
    _orig_setup_planner = _rm.RobotManager._setup_planner

    def _setup_planner(self, robot):
        if getattr(robot, "robot_name", None) == "wheelarm":
            return
        return _orig_setup_planner(self, robot)

    _rm.RobotManager._setup_planner = _setup_planner
    _rm.RobotManager._bd_wheelarm_planner = True


_ee = sys.modules.get("src.eval_client.eval_env")
if _ee is not None and not getattr(_ee, "_bd_wheelarm_dims", False):
    _orig_dims = _ee.get_robot_action_dim_info

    def _dims(env_cfg):
        if env_cfg["config"]["robot"] == "wheelarm":
            from env.global_configs import ROBOTS_PATH
            sd = yaml.safe_load(open(os.path.join(ROBOTS_PATH, "wheelarm", "robot_config.yml")))["sides"]["left"]
            return {"arm_dim": [len(sd["arm_joints_name"])], "ee_dim": [1]}
        return _orig_dims(env_cfg)

    _ee.get_robot_action_dim_info = _dims
    _ee._bd_wheelarm_dims = True


class BdMouseFloorCommon:
    """地上一只乱跑、会躲的老鼠;抓起来离地 10 cm 算成"""

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = int(os.environ.get("BD_STEP_LIM", "750"))
        self.walker = scene.Walker()

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)
        scene.install_checks(self.reward_manager.func_parser)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()
        self.walker.reset()

    def step(self, meta_control_list):
        self.walker.tick(self)
        super().step(meta_control_list)

    def run_reward(self):
        self.reward_manager.check([self.reward_manager.is_lift(label="target", z_threshold=0.1)])

    def gen_instruction(self, env_idx):
        return ["Catch the mouse that is running around on the floor and lift it 10 cm."]


class bd_mouse_floor(BdMouseFloorCommon, TaskEnv):
    pass
