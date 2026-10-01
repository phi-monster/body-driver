# 无人机载体(RoboDojo 仿真侧垫片,驱动零改动)

2026-09-24,owner:自标定必须 x5 / 人形 / 无人机全能。无人机在 RoboDojo 里没有现成的,这里把"一台会飞的相机"做成一个 6 自由度的虚拟龙门吊:
`base_link` 固定在桌子上方 1.4 m,x/y/z 三个滑动关节 + 偏航/俯仰/横滚三个转动关节,末端 `body_link` 就是机身,机身下面一台相机朝下前方看。
RoboDojo 和 curobo 看到的是一条 6 关节的"手臂",驱动看到的是:一条报位姿、吃位姿命令的"臂",一台长在它身上的眼,一个什么都不动的抓握通道(无人机没有手指)。

- `drone_setup.py`:在箱子上用 RoboDojo 的 python 跑,写 `Assets/Robots/drone/{drone.urdf, robot_config.yml, curobo*.yml}`、
  `env/robot_manager/robot_class/drone.py`、`robot_config/drone.py`、注册表、`env_cfg/robot/drone.yml`、`env_cfg/drone_rgb.yml`(只给 RGB + 位姿)、布局。
- USD:`convert_urdf.py Assets/Robots/drone/drone.urdf Assets/Robots/drone/drone.usd --fix-base --headless`(Isaac Lab 自带的转换器)。
- 起炮:`SEED=1 CAL=/root/cal_drone.json BL_NO_DEPTH=1 BL_MDE_OFF=1 ROBODOJO_RUN_ID=DR1 BL_LIFE=/root/经历_dr1.txt CFG=drone_rgb DRVMODE=work BD_STEP_LIM=3000 BL_INST=127.0.0.1:8077 bash /root/qiall.sh DR1 general_pickup`

要验的(owner 的规矩:相机装法先看图再信):相机朝向、驱动对"没有手指的臂"怎么办、"上"没有桌面可摸时从哪来。

- 09-28 DR1(第一次开机)第一帧:机身相机拍到的是房间的墙,不是桌面 —— `ori [0,-90,0]` 把相机转成水平朝 +x 看(RoboDojo 的相机朝自己的 −Z 看,
  `ori` 是 XYZ 欧拉角,franka 那份配置里实测过)。驱动于是把墙当成桌面拟合(法向和竖直差 79°)。改成 `[0,0,0]` = 机身水平时正朝下。
- `pos [0,0,-0.03]`(相机在机身中心下 3 cm)仿真里有没有真生效没有独立真值:打分脚本的"眼离手"是拟合出来的,DR1 两种考法都是 0–2 mm,
  而对着几米外的墙转动看不出 3 cm 的偏;相机朝下看 0.6 m 处的桌面时能看出来,看驱动量到的转轴离眼多远。
- 10-01 不报抓握(大并行 §5 路 1):`robot_config.yml` 的 `ee_type` 改成 `none`。观测里没有 `ee_joint_state`(state、action 两处都不放),
  动作也不收它;驱动看到的身体就是一条报位姿、吃命令的臂,加一台长在它上面的眼。以前的做法是报一个什么都不动的抓握通道,驱动把它量成哑巴组。
  URDF 里那个 1 cm 的占位关节还留着(在机身盒子里面):RoboDojo 的机器人管理器每条臂都按下标找一个末端关节(`find_joints`、控制张量),
  没人看它,也没人发它的命令。RoboDojo 侧补了两处,记号 `[bd] no end effector`:
  - `set_robot_init_state`:第三种末端给占位关节初值 0;
  - `obs_manager`:第三种末端不放进观测。
  取动作、插值、`control_manager` 本来就是"gripper / hand / 别的不管",不用补。
  `RD=<目录>` 让脚本写进一份拷贝(先在拷贝上跑、和现场比过再动现场;10-01 比过:只多那两处补丁和 `ee_type`)。
