# RoboDojo 上的人形测试台(2026-09-23)

`g1_setup.py` 在箱子上跑一次(`python3 /root/g1_setup.py`,用 `/venv/RoboDojo/bin/python3`),把 Unitree G1 29 自由度 + Inspire 五指手接进 RoboDojo,并加一个"老鼠在桌上随机走"的任务 `chase_mouse`。

前置:`Assets/Robots/g1/` 里要先放好 `g1_29dof_inspire_hand.usd` + `configuration/` 四个子层(NVIDIA 资产库 `Assets/Isaac/5.1/Isaac/IsaacLab/Robots/Unitree/G1/`)、`g1.urdf`(unitree_ros 的 `g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`)和 `meshes/`。

起炮:`SEED=1 CAL=/root/cal_g1.json BL_NO_DEPTH=1 BL_MDE_OFF=1 ROBODOJO_RUN_ID=<名> BL_LIFE=/root/经历_g1.txt CFG=g1_mouse DRVMODE=work bash /root/qiall.sh <名> chase_mouse`

驱动一个字不改:身体是什么、有几条臂、几只眼、哪只手能合,都由驱动开机自己量。
