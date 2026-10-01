# -*- coding: utf-8 -*-
# body-driver 第 40 条(路 8):会走的人形收拾大客厅。由 harness/g1walk 装进来,别手改。
# 身体、走路控制器(NVIDIA WBC-AGILE 的 velocity_height_g1)、读数和命令那一组、复位:task/RoboDojo/bd/walkrig.py(会走的人形的几个任务共用)。
# 判据:task/RoboDojo/bd/tidy.py 的 bd_tidy(每一件都放到它该去的地方)。一集 30 分钟 = 45000 个动作(25 Hz;BD_STEP_LIM 可改)。
# 人形开局站在餐桌跟前、面朝餐桌(开机量身体要一张桌面;布局 make_livingroom_layout.py 算的、install.py 写进身体配置)。
from env.environment.task_env import TaskEnv
from task.RoboDojo.bd import tidy, walkrig


class BdLivingroomCommon(walkrig.WalkCommon):
    """大客厅:几十件东西各放到该去的地方"""
    STEP_LIM = 45000

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        tidy.install_checks(self.reward_manager.func_parser)

    def run_reward(self):
        self.reward_manager.check([("bd_tidy", {})])

    def gen_instruction(self, env_idx):
        return [tidy.find_tidy(self.scene_manager.layout_manager.saved_layouts[env_idx]).get("sentence", "")]


class bd_livingroom(BdLivingroomCommon, TaskEnv):
    pass
