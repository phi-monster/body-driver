# l3_link:把 RoboDojo 的评测循环接到零机体假设驱动(bl-calibrate)上。
# 契约照抄上游样板(XPolicyLab/policy/Pi_0/deploy.py):
#   model_client.call(func_name=...) 打到 ws 策略服务器(就是驱动),TASK_ENV 负责步进。
# 与样板的唯一区别:驱动一次只回一个动作(它是闭环的,每拍都要看新观测),
# 所以 get_action 的返回按"可能是单个动作、也可能是动作块"两种都兼容。
def eval_one_episode(TASK_ENV, model_client):
    model_client.call(func_name="reset")

    while not TASK_ENV.is_episode_end():
        obs = TASK_ENV.get_obs()
        model_client.call(func_name="update_obs", obs=obs)
        actions = model_client.call(func_name="get_action")

        if actions is None:
            break
        # 单个动作 ⇒ 包一层;动作块 ⇒ 原样。判据是"第一个元素是不是序列",不假设维度。
        if len(actions) == 0:
            break
        first = actions[0]
        if not hasattr(first, "__len__"):
            actions = [actions]

        for action_idx, action in enumerate(actions):
            TASK_ENV.take_action(action)
            if TASK_ENV.is_episode_end() or action_idx + 1 == len(actions):
                break
            obs = TASK_ENV.get_obs()
            model_client.call(func_name="update_obs", obs=obs)


def eval_one_episode_batch(TASK_ENV, model_client):
    raise NotImplementedError("l3_link 只跑单环境(deploy.yml 里 eval_batch: false)")
