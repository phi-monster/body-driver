# 随机题机(路 8)

随机一件东西 × 随机一个要求 × 随机一具身体,每题一句给脑的话 + 一张布局 + 一个按仿真真值判的判据;一题一集走排队跑,每集存落盘和成败,汇总成功率;做成过的题进守门的一套,以后每批重跑。出题、评分、跑法都在这里,驱动里一个字不加。

## 物件池(`make_pool.py`)
Isaac 资产库里自带的 YCB 物件(`Isaac/5.1/Isaac/Props/YCB/Axis_Aligned`),16 件:饼干盒、糖盒、番茄汤罐、芥末瓶、布丁盒、果冻盒、午餐肉罐、香蕉、碗、马克杯、电钻、木块、剪刀、马克笔、夹子、泡沫砖。装成 RoboDojo 的新 Rigid 类别 `bdq_<短名>`(只加 `bdq_` 开头的目录):

- 原样存下 YCB 的 USD;贴图 8–14 MB 的 PNG 下下来缩成 1024 px 的 JPEG(相机 640×480,物件在画面里一两百像素),原图不留,整个池子约 9.5 MB;
- 自己写一个 `object.usd`:刚体 + 质量(YCB 数据集的)、网格加碰撞(凸包 / 凸分解)、物理材质;
- **朝向**:Isaac 的 YCB 网格是 Y 朝上画的(舞台写着 Z 朝上,Isaac 自己的 `Axis_Aligned_Physics` 也没转它,照原样放罐子、碗都侧躺着)。每件按它平常怎么放写明哪根轴朝上:罐子、瓶子、杯子、碗、电钻、香蕉、糖盒立着(Y 朝上),扁盒子、木块、剪刀、马克笔、夹子、泡沫砖平躺在最大那一面上;
- `metadata.json` 带包围盒和出题用的三条:平顶(上面能放东西)、容器(里面能放东西)、转 180° 看得出来。

## 出题(`make_questions.py`)
```
/venv/RoboDojo/bin/python make_questions.py /root/RoboDojo --batch b2 --n 30 --seed 2
```
- 身体:`x5`(CFG `arx_x5`)、`humanoid`(G1 + 五指手,CFG `g1_rgb`)、`drone`(CFG `drone_rgb`),都是箱上现成的测试台;
- 要求:`lift`(抬多高)、`next_to`(挪到另一件旁边)、`turn`(原地转过来)、`push`(往某个方向推过去,不许拿起来)、`on`(放到另一件上面 / 放进容器);无人机只出 `above`(飞到它正上方);
- 出题人只用"这具身体够得着、拿得住"来挑题(题目可行),不碰驱动:x5 张口约 9 cm、桌上能出题的那一块、歇着的手占的地方(离线核量的连杆位置);人形只能伸 8 cm 左右,题只出在两只手各自够得着的那两小块(两手中间够不着);
- 一题一个种子目录(种子 = 20000 + 题号):`Assets/Eval_Layout/RoboDojo/<配置>/<种子>/bd_question_0.json`,布局里 `bd_question` 那一块写着题号、给脑的话、判据、步数;任务 `bd_question` 对所有题都一样;
- 题单 `batches/<批>.json`(箱上 `/root/p8/qexam/batches/`,同一份拷进这里)。

## 判据(`rd/bd/question.py`,装到 `task/RoboDojo/bd/question.py`)
只读仿真真值;物件多大从它 metadata 的包围盒读、按此刻位姿转到世界里。

| 要求 | 判成"做成了" |
|---|---|
| lift | RoboDojo 自己的 `is_lift`:比开局高 h |
| next_to | 两件在桌面上的投影最近处 ≤ 5 cm;A 还在桌上(高度差 < 2 cm);A 挪过 ≥ 5 cm(开局两件之间空着 ≥ 10 cm,人形 ≥ 6 cm) |
| turn | 绕竖轴转过 ≥ 135°,倾斜 ≤ 30°,还在桌上 |
| push | 沿那个方向挪了 ≥ 八成的距离、横向偏 ≤ 一半,而且这一集里从没被抬高 2 cm 以上 |
| on | A 的中心在 B 的投影里(离边 ≥ 1 cm);放上面:A 的最低点在 B 的顶 −1.5 cm 到 +3 cm 之间;放进去:A 的最低点高出 B 的底 2 mm、不高过 B 的顶;A 停住了(< 3 cm/s) |
| above | 机身在它中心正上方 5 cm 以内,而且高出它的顶 ≥ 5 cm |

离线核(不接驱动、不动身体,只摆场景里的东西):`harness/scenes/check_scenes.py --task bd_question --qseeds 20003,20004,…`(同一具身体的几题在一个进程里挨个核)。

## 跑(`run_questions.py`)
```
python3 run_questions.py --batch b1            # 这一批全跑(一题一集,每集走 /root/q/run.sh 排队)
python3 run_questions.py --batch b1 --qids 3,4
python3 run_questions.py --guard               # 守门的一套
python3 run_questions.py --summary --batch b1  # 按身体 × 要求汇总
```
- 每集从 x5 `/root/cal_v1b78.json`、人形 `/root/cal_h4.json`、无人机 `/root/cal_dr2.json`(或 `--body x5=…`)拷一份新的身体文件(连 `.geo.json`、`.kin.txt`、参照图)装回,一集和一集之间不带经验;
- 驱动用主线那一份(`BL_BIN` / `BL_HOME` 给了就用那一份);RoboDojo 一写出这一集的 `_result.json` 就放锁;
- 每集留下:`runs/<炮名>/{result.json, run.log, look 里的文字, 第一张和最后一张给脑看的图, sim 报错}`,驱动日志就是 `/root/N<炮名>/cal.log`;成败一行进 `results/<批>.jsonl`(做成没有、用了几拍、叫了几次脑、用时);RoboDojo 的录像默认删(`--keep_video` 留);
- 做成过的题记进 `guard.json`。
- 注意:身体文件的拷贝(`cal_*`,每集约 3 MB,含参照图)按规矩不删,跑得多了要主代理定留不留。
