# 随机题机(路 8)

随机一件东西 × 随机一个要求 × 随机一具身体,每题一句给脑的话 + 一张布局 + 一个按仿真真值判的判据;一题一集走排队跑,每集存落盘和成败,汇总成功率;做成过的题进守门的一套,以后每批重跑。出题、评分、跑法都在这里,驱动里一个字不加。

## 物件池(`make_pool.py`)
Isaac 资产库里自带的 YCB 物件(`Isaac/5.1/Isaac/Props/YCB/Axis_Aligned`),16 件:饼干盒、糖盒、番茄汤罐、芥末瓶、布丁盒、果冻盒、午餐肉罐、香蕉、碗、马克杯、电钻、木块、剪刀、马克笔、夹子、泡沫砖。装成 RoboDojo 的新 Rigid 类别 `bdq_<短名>`(只加 `bdq_` 开头的目录):

- 原样存下 YCB 的 USD;贴图 8–14 MB 的 PNG 下下来缩成 1024 px 的 JPEG(相机 640×480,物件在画面里一两百像素),原图不留,整个池子约 10 MB;
- 自己写一个 `object.usd`:刚体 + 质量(YCB 数据集的)、网格加碰撞(凸包 / 凸分解)、物理材质;
- **朝向**:Isaac 的 YCB 网格是 Y 轴顺着物件的高画的,而且口朝 −Y(舞台写着 Z 朝上,Isaac 自己的 `Axis_Aligned_Physics` 也没转它)。立着的(罐子、瓶子、杯子、碗、电钻、香蕉、糖盒)绕 x 转 −90°;扁盒子、木块、剪刀、马克笔、夹子、泡沫砖不转,平躺在最大那一面上。第一版转的是 +90°:量网格才看出碗底、杯底的顶点全在最上面(扣着放),芥末瓶倒立在瓶盖上,物件池稳不稳那一炮倒了 89.8°。现在每个容器建完自己核一遍"口朝上"(口中间那一圈往下看,底在下半截),不对就报错;
- `metadata.json` 的 `geometry.bd` 里放出题、判据用的(放在 `geometry` 里面,因为 RoboDojo 读 metadata 只留 physics / visual / geometry / active / passive 五项;第一版放在最上面一层,仿真里拿不到,判据悄悄退回了包围盒):
  - 平顶 / 容器 / 转 180° 看得出来(按上面的摆法说);
  - 从网格量的形状:凸包顶点(判据找最低点、最高点、桌面上的投影)、`pass_d`(最细能从多大的圆口塞进去:顺着最长轴看过去,凸包投影的最小外接圆直径)、容器的 `opening_d` / `opening_center`(高过它一半的顶点投到桌面上,围在中间的最大空圆:碗 12.7 cm、杯子 6.7 cm)。

**物件池核过**(`make_questions.py --pool_check` 写两张布局,种子 29998 / 29999,每张 8 件隔 30 cm;`check_scenes.py` 的 `pool_scenario`):物理走 300 个子步(和 RoboDojo 核布局稳不稳一样长),16 件都站得住(按 RoboDojo 的规矩:歪 ≤ 30°、每根轴挪 ≤ 4 cm;实测最多挪 7.1 mm、最多歪 12.5°,都是香蕉);每个平顶 / 容器拿同一张里最小的放得上 / 放得进的那件真放上去 / 放进去、物理走到停,题里的判据判 1:马克笔竖着放进杯子(最低点高出杯底 4.4 mm)、放进碗、放到饼干盒上,泡沫砖放到果冻盒、布丁盒、木块上。

## 出题(`make_questions.py`)
```
/venv/RoboDojo/bin/python make_questions.py /root/RoboDojo --batch b2 --n 30 --seed 2
/venv/RoboDojo/bin/python make_questions.py /root/RoboDojo --batch x --feasibility 20   # 只看每一对(身体, 要求)摆不摆得下
```
- 身体:`x5`(CFG `arx_x5`)、`humanoid`(G1 + 五指手,CFG `g1_rgb`)、`drone`(CFG `drone_rgb`),都是箱上现成的测试台;
- 要求:`lift`(抬多高)、`next_to`(挪到另一件旁边)、`turn`(原地转过来)、`push`(往某个方向推过去,不许拿起来)、`on`(放到平顶上 / 放进容器);无人机只出 `above`(飞到它正上方);
- 每题先抽(身体, 要求)这一对,每一对机会一样;再在这一对里抽东西、摆布局,摆不下换东西重抽。出题前先看每一对摆不摆得下(另一路随机数抽 200 回),一回都摆不下的那一对不出,打印出来;
- 出题人只按"这具身体够得着、拿得住"挑题,数都是量的:
  - 拿得住 = 平放时窄的那一边比手张到头的空窄:x5 8.8 cm(ARX.usd 的 joint7 / joint8 两指各走 0–0.044 m,0 时两指碰撞面贴着);人形 6.6 cm(`g1_29dof_inspire_hand.usd` 默认姿势,拇指末节和食指中节最近的距离);
  - 够得着 = 桌上能出题的那一块、歇着的手占的地方(离线核量的连杆位置):x5 一块 60 × 28 cm;人形每只手只能伸 8 cm 左右,两只手各一小块(两手中间够不着);
  - 放得上 / 放得进 = `question.fits`(和判据、离线核用同一条):进容器要 A 的 `pass_d` 比 B 的口小(杯子的口只过得去马克笔);放平顶要 A 平放的两条边都不比 B 顶面的长;
  - 要两件的题(挨着、放上去):两件在同一块出题区里(人形 A 只能在 B 那只手的地方里挪),开局空得比"挨着"远(外接圆之间 > 5 cm);
- 一集多少步(驱动开机的步也算在里面,和 RoboDojo 官方任务一样;x5 带着存好的身体文件开机 68 步):照 RoboDojo 自己最像的那个官方任务给的步数 —— lift / push / above 200(`general_pickup`:"Pick up the <target> by 10 cm."),next_to / on / turn 300(`deposit_coin`:"Pick up the coin … and insert it … into the coin bank.")。官方唯一推东西的 `push_T` 是 600,但它要推到一个位姿上;
- 一题一个种子目录(种子 = 20000 + 题号):`Assets/Eval_Layout/RoboDojo/<配置>/<种子>/bd_question_0.json`。题(题号、给脑的话、判据、步数)写在目标那件东西的记录里(`bd_question` 那一项;布局最上面一层的每一项 RoboDojo 都当成一类东西去生,第一版放那儿开场就 TypeError);任务 `bd_question` 对所有题都一样;同一个种子上回出给了别的身体,那份旧题删掉;
- 题单 `batches/<批>.json`(箱上 `/root/p8/qexam/batches/`,同一份拷进这里):`b1` = 头一批 30 道 YCB 题(种子 20000–20029);
- **小场景那一批**(大并行 §6"再加路 8 的小场景"):`make_questions.py … --batch s1 --scenes --seed 11`,`harness/scenes` 的 11 个任务各从种子 0 的 3 张布局里随机挑一张,原样拷进这一题自己的种子目录(RoboDojo 一个种子目录里有几张布局就连着跑几集,一题一集就得一题一个目录;拷完对过 md5 一样),判据就是那个任务自己的(各张布局离线核过,见 `harness/scenes/README.md`);步数照官方最像的任务:一件事(抬起来、拉开、掀开、推开)= `general_pickup` 的 200,两件事(拿起来再插进去 / 挂上去 / 扣扳机;拧半圈要松手再拧)= `deposit_coin` 的 300。`s1` = 题 30–40(种子 20030–20040)。

## 判据(`rd/bd/question.py`,装到 `task/RoboDojo/bd/question.py`)
只读仿真真值;形状用 metadata 里量的凸包顶点、按此刻位姿转到世界里(第一版用包围盒 8 个角:东西一歪,角比真东西低一截)。"挨着""挪过"的 5 cm 和 RoboDojo 自己判据的默认距离一样(`is_lift`、`is_moved` 都是 0.05)。

| 要求 | 判成"做成了" |
|---|---|
| lift | RoboDojo 自己的 `is_lift`:比开局高 > h |
| next_to | 两件桌面投影(凸包)最近处 ≤ 5 cm;A 高度和开局差 ≤ 2 cm(还在桌上);A 挪过 ≥ 5 cm |
| turn | 绕竖轴转过 ≥ 135°,倾斜 ≤ 30°,高度差 ≤ 2 cm |
| push | 沿那个方向挪了 ≥ 八成的距离、横向偏 ≤ 它的一半,而且这一集里从没被抬高 2 cm 以上 |
| on | A 的中心在 B 的投影里(离边 ≥ 1 cm);放上面:A 的最低点在 B 的顶 −1.5 cm 到 +3 cm 之间(扫描的盒子顶不平,饼干盒上实测 −5 mm);放进去:A 的最低点高出 B 的底 2 mm、不高过 B 的顶;A 停住了(< 3 cm/s) |
| above | 机身在它中心正上方 5 cm 以内,而且高出它的顶 ≥ 5 cm |

## 离线核(`harness/scenes/check_scenes.py --task bd_question --qseeds …`,`qcheck.sh` 排队)
不接驱动、不动身体,只摆场景里的东西;同一具身体的几题在一个进程里挨个核。每题:装得起来(RoboDojo 自己的稳不稳那一关过了)、东西都在、存三只相机的图,然后:

| 要求 | 摆成的样子 → 该判 |
|---|---|
| 都有 | 开局 → 0 |
| lift | 抬高 h + 2 cm → 1;只抬 h − 2 cm → 0 |
| turn | 原地转 90° → 0;转 180° → 1 |
| push | 沿那个方向挪过去(没抬)→ 1(这时走一遍 RoboDojo 自己的 `step → get_reward`);同一处但中间被拿起来过 5 cm → 0 |
| next_to | 沿两件连线挪到离 B 正好 2 cm → 1;同一处但悬在半空 6 cm → 0 |
| on | 在 B 旁边、悬在 B 的顶 / 半腰那个高度 → 0;真放上去 / 放进去(放不进口就竖着放)、物理走到停 → 1 |
| above | 东西在机身正下方偏 10 cm → 0;正下方 → 1 |

最后停在做成了的样子,走一遍 RoboDojo 自己的 `reward_manager.step → get_reward`,该是 1。

## 跑(`run_questions.py`)
```
python3 run_questions.py --batch b1            # 这一批全跑(一题一集,每集走 /root/q/run.sh 排队)
python3 run_questions.py --batch b1 --qids 3,4
python3 run_questions.py --guard               # 守门的一套
python3 run_questions.py --summary --batch b1  # 按身体 × 要求汇总
```
- 每集从 x5 `/root/cal_v1b78.json`、人形 `/root/cal_h4.json`、无人机 `/root/cal_dr2.json`(或 `--body x5=…`)拷一份新的身体文件(连 `.geo.json`、`.kin.txt`、参照图)装回,一集和一集之间不带经验;`SEED` = 题的种子,`BD_STEP_LIM` = 题的步数;
- 驱动用主线那一份(`BL_BIN` / `BL_HOME` 给了就用那一份);RoboDojo 一写出这一集的 `_result.json`(`{"success_rate", "details": {"0": {"success"}}}`)就放锁;
- 每集留下:`runs/<炮名>/{result.json, run.log, look 里的文字, 第一张和最后一张给脑看的图, sim 报错}`,驱动日志就是 `/root/N<炮名>/cal.log`;成败一行进 `results/<批>.jsonl`(做成没有、用了几拍、叫了几次脑、用时);RoboDojo 的录像默认删(`--keep_video` 留);
- 做成过的题记进 `guard.json`。
- 注意:身体文件的拷贝(`cal_*`,每集约 3 MB,含参照图)按规矩不删,跑得多了要主代理定留不留。
