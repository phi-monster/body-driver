# 通用小场景(大并行 §2 第 38 条,路 8)

给路 5、路 6 在自己的炮上核机制用的 11 个 RoboDojo 新任务。身体是 RoboDojo 自己的双臂 x5(`CFG=arx_x5`),每个任务自己带评分(只读仿真真值),每个任务 3 张布局。

**只往 RoboDojo 里加文件,原有文件一个不动。** 生成器每写一个路径先检查(名字里带 `bd_`,或在 `task/RoboDojo/bd/` 下);装完以后 RoboDojo 的 `git diff` 哈希不变。

## 11 个任务

| 任务 | 场景 | 判据(仿真真值) | 给脑的话 |
|---|---|---|---|
| `bd_drawer` | 带抽屉的柜子(关节体,底座固定;抽屉是滑轨 0–20 cm,带阻尼,拉到哪停在哪) | 抽屉离开局 ≥ 10 cm | Open the drawer of the cabinet. |
| `bd_lidbox` | 带盖的盒子(不固定,放在桌上;盖子铰链在后上沿 0–110°;盒里一块红方块) | 盖子转开 ≥ 80° | Open the lid of the box. |
| `bd_hinge` | 带铰链的板(一扇竖轴小门,两边都能开 ±100°) | 板转开 ≥ 60° | Swing the hinged board open. |
| `bd_peg` | 孔和销:方孔 20.8 mm、方销 20.0 mm(每边 0.4 mm,合起来 0.8 mm < 1 mm),孔深 5 cm | 销底低于孔口 ≥ 2 cm 且在孔口以内 | Insert the blue peg into the square hole of the yellow block. |
| `bd_hook` | 挂钩和环(钩子:立柱 + 朝机器人伸出的横杆,杆尖上翘;环:外径 8 cm、内径 5.6 cm) | 横杆从环当中穿过(环心离杆 < 内半径、法向和杆差 < 45°、投影在杆上)且环离桌 > 8 cm | Hang the red ring on the hook. |
| `bd_knob` | 旋钮(竖轴,±340°,转半圈就可能超过手腕一次的行程) | 离开局转过 ≥ 180° | Turn the knob half a turn. |
| `bd_trigger` | 带扳机的枪形东西(关节体,不固定;扳机是转轴 0–25°,回位弹簧 0.2 N·m/rad;侧躺在桌上,握把朝机器人) | 同一拍里枪身抬离 ≥ 5 cm 且扳机扣下 ≥ 17.5° | Pick up the water gun and pull its trigger. |
| `bd_glass` | 玻璃杯(Isaac 自带的 OmniGlass,透明;碰撞是杯底 + 16 片薄壁) | 抬离 ≥ 10 cm(RoboDojo 的 is_lift) | Pick up the glass cup by 10 cm. |
| `bd_white` | 白桌白墙:桌面纯白 OmniPBR(`Assets/Material/bd_white`)、四面白墙 + 白地;桌上一只喷漆罐(RoboDojo 自带的 `can/00000`) | 抬离 ≥ 10 cm | Pick up the spray can by 10 cm. |
| `bd_walker` | 会自己走的东西:RoboDojo 自带的玩具校车(`toy_car/00000`)在桌面上按步随机走:每个动作 1 cm、每 25 个动作随机换方向、碰边反射;被拿离桌面 5 mm 以上或翻倒就不走。走法是速度伺服:心里"该在哪"的点每个动作前进 1 cm,物理按速度去追(摩擦、碰撞照算:被挡住就停,被推开就从那儿接着走) | 抬离 ≥ 10 cm | Catch the toy bus that drives around on the table and lift it 10 cm. |
| `bd_cloth` | 一块布:30 × 30 cm 粒子布(31 × 31 个点),平铺在桌上;粒子参数和 RoboDojo 自己的布(fold_clothes)一样 | 布上最高的粒子高出桌面 ≥ 10 cm | Pick up the cloth by a corner and lift it 10 cm. |

几何都是这里自己定的,不照 RoboDojo 的布局、不照我们手上的硬件。判据里的尺寸不写在任务代码里,全从资产自己的 `metadata.json`(`passive.functional`)读:孔口位置和半宽、横杆两端、环心和内外半径、销底。

**摆布局**(每个任务 3 张,种子固定、可重现):每件东西的碰撞盒——关节体再加上动的那一节从下限到上限扫过的地方——都离身体歇着的两只手够远,东西之间不叠。手占的地方按离线核量到的 x5 每一节连杆开局位置定:腕 (±0.30, −0.352, 0.922),手指朝前平伸(指尖约 y −0.21);每只手一个盒子 x = 臂 ± 0.11、y ∈ [−0.40, −0.17]、高于 0.85 m,外加以腕为心 15 cm 的圆。查法:东西的角点不进手的地方,手的地方里铺满的点也不在东西的任何一个碰撞盒里(大块板子横穿手也查得出)。第一版没这一条,`bd_drawer` 布局 2 的抽屉拉开 11 cm 就顶在歇着的左手手指上。

## 文件

| 文件 | 干什么 |
|---|---|
| `make_scenes.py` | 生成器:资产(ASCII USD)、metadata / description、白桌面 MDL、任务、任务配置、布局。箱上跑:`bash usdpy.sh make_scenes.py /root/RoboDojo` |
| `usdpy.sh` | 箱上离线的 USD python(Isaac 包里自带的 pxr,不起 Isaac、不占卡) |
| `rd/bd/scene.py` | 装到 `task/RoboDojo/bd/scene.py`:五个判据(`bd_joint_moved` `bd_peg_in_hole` `bd_ring_on_hook` `bd_trigger_while_held` `bd_cloth_lifted`)在任务 `_post_setup_scene` 里绑到 RoboDojo 的 `Func_Parser` 实例上(`Func_Parser` 源文件不动);会自己走的东西 `Walker` |
| `check_scenes.py` | 离线核:不接驱动、不接脑,照 RoboDojo 的 `main.py` 装场景,逐张布局核装得起、稳、东西在、存图、判据 0 → 1(见下) |
| `qcheck.sh` | 离线核的外壳:和 `/root/q/run.sh` 拿同一把锁(一个仿真位),占卡时 `now.txt` 写明;`bash qcheck.sh bd_drawer bd_peg …` |

装到 RoboDojo 里的(全是新文件):
- `Assets/Object/RoboDojo/Articulation/{bd_cabinet, bd_lidbox, bd_hinge_board, bd_knob, bd_water_gun}/00000/`
- `Assets/Object/RoboDojo/Rigid/{bd_cube, bd_peg, bd_ring, bd_glass_cup}/00000/`
- `Assets/Object/RoboDojo/Geometry/{bd_hole_block, bd_hook, bd_white_room}/00000/`
- `Assets/Object/RoboDojo/Garment/bd_cloth/00000/`
- `Assets/Material/bd_white/bd_white.mdl`(名字不以 `material` 开头,不会混进 RoboDojo 随机桌面材质的池子)
- `task/RoboDojo/bd/scene.py`(在 `tasks/` 外面,不进 RoboDojo 的任务清单)
- `task/RoboDojo/tasks/bd_*.py`、`task/RoboDojo/config/bd_*.yml`
- `Assets/Eval_Layout/RoboDojo/arx_x5/0/bd_*_{0,1,2}.json`(种子 0;环境那几块——房间、桌、地、背景、头顶相机支架——照 RoboDojo 默认)

## 离线核(`check_scenes.py`)核什么

1. 装得起来,RoboDojo 自己的"布局稳不稳"那一关过了;
2. 每件东西都在、落稳后离布局给的位置多远;存三只相机的图(取帧前多渲几次:只渲一次拿到的是上一次渲染的样子);第 0 张布局记下身体每一节连杆开局在哪;
3. 评分对:把仿真真值摆成"做成了 / 没做成"的几种样子,判据给 1 / 0;物理上站得住的状态(抽屉拉开、盖子翻过头、销插进孔、环挂上钩)再走几十个物理步,判据还是 1;扳机松手后回位弹簧把它拉回 0、判据跟着变 0;布拎起一角再撒手,那一角掉回去、判据跟着变 0(证明读到的粒子是仿真在动的,不是 USD 里的死数);最后走一遍 RoboDojo 自己的 `reward_manager.step → get_reward`,做成了的样子真判成 1;
4. 会自己走的东西:原地不动地走 60 个动作(动作 = 身体此刻的关节读数),量每个动作走多远、出不出界。

摆状态只动场景里的东西,从不替身体动手。摆关节用 Isaac 的 `set_joint_positions`,它会顺手把驱动目标也设成新位置(`isaacsim.core.prims` 的 articulation.py),带弹簧的扳机摆完要把目标放回 0 才是"松手"。

出了错(比如 Isaac 抛异常)先把报告写下来再直接退——Kit 在异常以后正常关机会卡住,第一版就这样占了仿真位 5 分钟;`qcheck.sh` 每个任务 10 分钟封顶。

## 开炮(排队位,主线驱动)

身体文件别用原件(驱动干活时会把量到的写回 `--out`):先拷一份,
```
cp /root/cal_v1b78.json /root/p8/cal_p8a.json; cp /root/cal_v1b78.json.geo.json /root/p8/cal_p8a.json.geo.json
CAL=/root/p8/cal_p8a.json BL_LIFE=/root/p8/经历_p8a.txt CFG=arx_x5 DRVMODE=work BD_STEP_LIM=3000 bash /root/q/run.sh 8 P8A bd_drawer 12
```
开得了机 = 驱动日志走到 `[身] 身体量完 ⇒ 开始干活` 和 `── 第 1 轮`;看完就 `touch /root/q/done_P8A` 放锁。
