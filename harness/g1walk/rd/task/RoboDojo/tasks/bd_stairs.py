# -*- coding: utf-8 -*-
# body-driver 远 5(路 8):会走的人形上台阶。由 harness/g1walk 装进来,别手改。
# 同一间客厅(人形开局站在餐桌跟前),东边空地上一段台阶(3 级 × 10 cm,上面一块平台;make_livingroom.py 的 bd_lr_stairs)。
# 身体、走路控制器、读数和命令那一组、复位:task/RoboDojo/bd/walkrig.py。
# 判据:task/RoboDojo/bd/walkjudge.py 的 bd_on_top(两只脚都站在平台那一块面上)。一集 3 分钟 = 4500 个动作(BD_STEP_LIM 可改)。
from env.environment.task_env import TaskEnv
from task.RoboDojo.bd import walkjudge, walkrig


class BdStairsCommon(walkrig.WalkCommon):
    """上台阶:走到台阶那儿,一级一级上去,站在最上面的平台上"""
    STEP_LIM = 4500

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        walkjudge.install_checks(self.reward_manager.func_parser)

    def run_reward(self):
        self.reward_manager.check([("bd_on_top", {"label": "stairs", "place": "top"})])

    def gen_instruction(self, env_idx):
        return ["Walk over to the stairs, climb them, and stand on the platform at the top."]


class bd_stairs(BdStairsCommon, TaskEnv):
    pass
