# RoboDojo 上的人形测试台(2026-09-23)

`g1_setup.py` 在箱子上跑一次(`python3 /root/g1_setup.py`,用 `/venv/RoboDojo/bin/python3`),把 Unitree G1 29 自由度 + Inspire 五指手接进 RoboDojo,并加一个"老鼠在桌上随机走"的任务 `chase_mouse`。

前置:`Assets/Robots/g1/` 里要先放好 `g1_29dof_inspire_hand.usd` + `configuration/` 四个子层(NVIDIA 资产库 `Assets/Isaac/5.1/Isaac/IsaacLab/Robots/Unitree/G1/`)、`g1.urdf`(unitree_ros 的 `g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`)和 `meshes/`。

起炮:`SEED=1 CAL=/root/cal_g1.json BL_NO_DEPTH=1 BL_MDE_OFF=1 ROBODOJO_RUN_ID=<名> BL_LIFE=/root/经历_g1.txt CFG=g1_mouse DRVMODE=work bash /root/qiall.sh <名> chase_mouse`

驱动一个字不改:身体是什么、有几条臂、几只眼、哪只手能合,都由驱动开机自己量。

**老鼠走得太快(路 8,10-01 量的)**:`chase_mouse` 的 `_wander` 在任务的 `step()` 里,而 RoboDojo 一个动作 = 10 个物理子步、每个子步都调 `step()`,
原来每调一次走一整个 `BD_MOUSE_SPEED` ⇒ 离线核(`harness/scenes/check_scenes.py --walk_label target`,人形 g1_rgb、种子 1)量到**每个动作中位 9.51 cm**
(最大 15.2 cm)、每 4 个动作换方向,而且一路在蹦(高 0.772–0.811 m);抓起来以后还被按平面拽着走。
修法是 `fix_mouse_speed.py`(接在 `rig_upright_low.py` 后面跑,只换 `_wander` 一个方法,可重复跑):老鼠改按 `harness/scenes` 装的
`task/RoboDojo/bd/scene.py` 的 Walker 走(速度伺服);在副本上核过:**每个动作中位 1.00 cm**、高度不变、开局在区域外 2 cm 自己走回区域里。
它改的是箱上大家共用的任务文件,等主代理点头再在箱上用。
