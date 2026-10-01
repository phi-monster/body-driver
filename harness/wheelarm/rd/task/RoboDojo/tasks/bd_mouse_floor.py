# -*- coding: utf-8 -*-
# body-driver 第 39 条(路 8):轮子底盘 + 一条胳膊,地上一只乱跑、会躲的老鼠。由 harness/wheelarm 装进来,别手改。
# 老鼠(RoboDojo 自带的 mouse 资产)在地上按 task/RoboDojo/bd/scene.py 的 Walker 走:一个动作 1 cm(owner:"老鼠随机乱跑(1 cm/步)")、
# 每 40 个动作随机换方向、碰边反射;身体哪一节进了 flee_radius 就朝正背着它跑("还会躲");被拿离地面 5 mm 以上就不走。
# 判据:老鼠离开局高 > 10 cm(RoboDojo 的 is_lift;"30 秒内抓起来、离地 10 cm")。一集 30 秒 = 750 个动作(25 Hz;BD_STEP_LIM 可改)。
#
# 这具身体不在 RoboDojo 的机器人登记里:task/RoboDojo/bd/rig.py 在任务模块被导入的时候补三处(登记表、不建 cuRobo 规划器、动作维数按它自己的
# 机器人配置数),RoboDojo 原有的文件一个字不动。
import os

from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager
from task.RoboDojo.bd import rig, scene

rig.register("wheelarm", "wheelarm", ["WheelArm"])
rig.no_planner("wheelarm")
rig.dims("wheelarm")


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
