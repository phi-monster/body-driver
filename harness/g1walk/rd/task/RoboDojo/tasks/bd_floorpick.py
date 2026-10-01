# -*- coding: utf-8 -*-
# body-driver 远 5(路 8):会走的人形蹲下捡地上的东西。由 harness/g1walk 装进来,别手改。
# 同一间客厅(人形开局站在餐桌跟前),地上一件东西(随机题机的物件池 bdq_*,离人形 1–2 m)。
# 身体、走路控制器、读数和命令那一组、复位:task/RoboDojo/bd/walkrig.py。
# 判据:RoboDojo 自己的 is_lift(标签 target,离开局抬高 10 cm)。一集 3 分钟 = 4500 个动作(BD_STEP_LIM 可改)。
from env.environment.task_env import TaskEnv
from task.RoboDojo.bd import walkrig


class BdFloorpickCommon(walkrig.WalkCommon):
    """从地上捡起一件东西,抬离地面 10 cm"""
    STEP_LIM = 4500

    def run_reward(self):
        self.reward_manager.check([self.reward_manager.is_lift(label="target", z_threshold=0.1)])

    def gen_instruction(self, env_idx):
        lay = self.scene_manager.layout_manager.saved_layouts[env_idx]
        say = [r.get("bd_say") for recs in (lay.get("Rigid") or {}).values() for r in recs if r.get("label") == "target"]
        return [say[0] if say and say[0] else "Pick up the thing on the floor and lift it 10 cm."]


class bd_floorpick(BdFloorpickCommon, TaskEnv):
    pass
