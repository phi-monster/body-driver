# 无人机载体(RoboDojo 仿真侧垫片,驱动零改动)

2026-09-25,owner:自标定必须 x5 / 人形 / 无人机全能。无人机在 RoboDojo 里没有现成的,这里把"一台会飞的相机"做成一个 6 自由度的虚拟龙门吊:
`base_link` 固定在桌子上方 1.4 m,x/y/z 三个滑动关节 + 偏航/俯仰/横滚三个转动关节,末端 `body_link` 就是机身,机身下面一台相机朝下前方看。
RoboDojo 和 curobo 看到的是一条 6 关节的"手臂",驱动看到的是:一条报位姿、吃位姿命令的"臂",一台长在它身上的眼,一个什么都不动的抓握通道(无人机没有手指)。

- `drone_setup.py`:在箱子上用 RoboDojo 的 python 跑,写 `Assets/Robots/drone/{drone.urdf, robot_config.yml, curobo*.yml}`、
  `env/robot_manager/robot_class/drone.py`、`robot_config/drone.py`、注册表、`env_cfg/robot/drone.yml`、`env_cfg/drone_rgb.yml`(只给 RGB + 位姿)、布局。
- USD:`convert_urdf.py Assets/Robots/drone/drone.urdf Assets/Robots/drone/drone.usd --fix-base --headless`(Isaac Lab 自带的转换器)。
- 起炮:`SEED=1 CAL=/root/cal_drone.json BL_NO_DEPTH=1 BL_MDE_OFF=1 ROBODOJO_RUN_ID=DR1 BL_LIFE=/root/经历_dr1.txt CFG=drone_rgb DRVMODE=work BD_STEP_LIM=3000 BL_INST=127.0.0.1:8077 bash /root/qiall.sh DR1 general_pickup`

要验的(owner 的规矩:相机装法先看图再信):相机朝向(`ori` 的约定和 G1 腕眼一样是 `[0,-90,0]`,第一炮落一帧看)、驱动对"没有手指的臂"怎么办、"上"没有桌面可摸时从哪来。
