# 轮子底盘 + 胳膊 + 地上会躲的老鼠(大并行 §2 第 39 条,路 8)

目标 B("LeKiwi 抓老鼠")的仿真那一半:一具带轮子的身体在地上抓一只乱跑、会躲的老鼠,30 秒内抓起来离地 10 cm。真 LeKiwi 归硬件组。

## 身体(`make_wheelarm.py` 写 `Assets/Robots/wheelarm/wheelarm.usd`)
几何这里自己定,不照 LeKiwi(LeKiwi 是三个全向轮;这里是两轮差速 + 前后两个万向球):
- 底盘 0.36 × 0.30 × 0.10 m,离地 3 cm,6 kg;两个驱动轮(半径 5 cm、宽 3 cm,在两侧 y = ±0.17,连续转、没有限位);前后两个万向球(摩擦 0,只撑着);
- 胳膊装在底盘顶上靠前:转腰 → 肩 → 大臂 24 cm → 肘 → 小臂 22 cm → 腕俯仰 → 腕转 → 掌 + 两根平行手指(各走 0–4 cm);
- 前眼在底盘前沿(朝前、往下 15°),腕眼在掌上(顺着手指看);没有头顶眼。
- 关节体的根是底盘那一节(第一版放在最外层的 Xform 上,PhysX 自己挑了大臂当根,RoboDojo 摆根摆的是大臂,底盘被歪着摆、离地高了 9 cm)。

身体报的、收的就是关节:两个轮子的转角排在前面、胳膊五个关节在后面(一串)、手指一组(RoboDojo 一条夹爪通道)、两只眼的彩色图。
轮子和别的关节一样收位置目标(目标转得比轮子快多少,推得就多大劲)。驱动不知道哪个是轮子:推一下这两个关节,两只眼里的世界一起流,
就是"扛着全身的那组通道"(大并行 §2 甲 1)。底盘按 [vx, vy, wz] 收速度的那种接法(厂商控制器)等路 7 的身体协议文档定了再加。

## 装进 RoboDojo(只加新文件,原有的一个字不动)
```
bash ../scenes/usdpy.sh make_wheelarm.py /root/RoboDojo      # 机器人 USD
/venv/RoboDojo/bin/python install.py /root/RoboDojo          # 机器人类、执行器、机器人 / 相机 / 环境配置、任务 bd_mouse_floor、三张布局
```
- `env/robot_manager/robot_class/wheelarm.py`、`robot_config/wheelarm.py`(根不固定;执行器:轮子刚度 10 N·m/rad、胳膊 400、手指 400 N/m);
- `env_cfg/wheelarm_rgb.yml`(CFG)、`env_cfg/robot/wheelarm.yml`(根放在地面上 8 cm,地面高按布局里 Ground 那一块算:中心 + 半厚 = 0.05 m)、
  `env_cfg/camera/camera_wheelarm.yml`(只有身体自己的两只眼);
- 任务 `task/RoboDojo/tasks/bd_mouse_floor.py`:这具身体不在 RoboDojo 的登记里,任务模块被导入时补三处(robot_manager 的两张登记表加上它;
  不给它建 cuRobo 规划器;评测环境核动作维数时按它自己的机器人配置数)—— 都是运行时补的,RoboDojo 原有的文件不改;
- 布局 `Assets/Eval_Layout/RoboDojo/wheelarm/0/bd_mouse_floor_{0,1,2}.json`:房间、地、背景照 RoboDojo 默认,桌子挪到房间另一头;
  老鼠(RoboDojo 自带的 mouse)在底盘前面地上。RoboDojo 核"布局稳不稳"把低过桌面 5 cm 的东西一律判站不住,地上的老鼠关掉这一关。

## 老鼠(`task/RoboDojo/bd/scene.py` 的 Walker)
一个动作 1 cm(owner:"老鼠随机乱跑(1 cm/步)")、每 40 个动作随机换方向、碰区域的边反射、被拿离地面 5 mm 以上就不走;
"还会躲":身体哪一节(仿真真值:每一节连杆的位置)进了 `flee_radius`(0.25 m)就朝正背着最近那一节跑。

## 判据
老鼠离开局高 > 10 cm(RoboDojo 的 is_lift)。一集 30 秒 = 750 个动作(25 Hz)。

## 离线核(`harness/scenes/check_scenes.py --task bd_mouse_floor`,CFG `wheelarm_rgb`)
装得起、老鼠落稳(挪 2.4 mm、歪 0°)、两只眼的图;底盘真在地上走:两个轮子一起转(目标每个动作 0.08 rad = 4 mm)50 个动作走了 0.190 m(该 0.2 m),
反着转原地转了 50.6°、挪了 2.7 cm;老鼠离地 8 cm 判 0、12 cm 判 1。
