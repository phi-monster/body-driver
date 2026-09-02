# body layer

> ## **world model + body layer**
> ## **世界模型 + 身体层**
> ### **世界靠学，身体靠量。**
> ### **我一动，跟着动的那一块，就是我。**

Whatever belongs to **this body** is measured at power-on and kept measured; it never enters the
weights. Whatever belongs to **the world** is learned.

Today every VLA bakes the body into the weights, so a new robot costs hours of data and a gradient
pass — the best public figure is "a few hours of data / under 200 demonstrations". This layer is
the other half of that split, and the point of it is that **the body half was never in the weights
to begin with**. Each robot measures itself; the model does not change by one byte.

🔴 **And the layer publishes what it still owes.** *"We removed the hand-filled constants"* was not
true on 2026-08-09 and is not true today; the honest number is a
[**ledger**](slow/src/debt.rs) — **60 rows, 12 outstanding body constants**, two of them this
layer's own. Read it before the claim below: [what this layer still owes](#what-this-layer-still-owes).

---

## 📄 论文:**《我动,故我在》** · *I Move, Therefore I Am*
### 副题:**机器人怎么知道自己是机器人**

**标题不是修辞,是判据本身** —— 驱动里那行代码的原话:**我一动,画面里跟着动的那一块,就是我。**
笛卡尔从"思考"推出存在;这具身体不思考,**它先动,再从"什么跟着我动"里推出自己有多大、有几只手、够得到哪儿。**

**主贡献 = 这个 driver 本身。** 主目标 = RoboDojo 官方分逼近人类遥操(≥42/55 = 76.03%)。
**第二篇的位置已经留好**:第一篇是**出生**,第二篇是**擂台**(*The First Fight of Physical AI*)——
先有身体,才有对手。

## 🧠 脑子放哪、快到什么程度、换谁都能开(owner 2026-09-02 逐条问定)

### 真正的约束是【装不装得下】,不是快慢

Kimi3 / Fable5 那种体量**永远上不了机器人** —— 不是慢,是**放不进去**(显存/功耗/散热/成本),
而且模型只会更大。真问题是**哪些必须在机器人身上**:

| | 在哪 | 为什么 |
|---|---|---|
| 大脑(任何大模型) | **不在机器人上** | 装不下;它一秒只需说一两句 |
| **身体层** | **必须在机器人上** | **它没有权重** —— Rust + Ada,零依赖零分配,什么板子都装得下 |
| 反射 / 安全 | **必须在机器人上** | 断网也得能自己停住 |

🔴 **这一层的核心价值一句话:它把"大脑必须每秒发一千条命令"变成"大脑每几秒说一句话" ——
而这正是一个装不下的大脑能开机器人的唯一办法。**
⇒ **别人的模型越大越强,这一层越值钱。我们永远不去训更大的脑子,也不需要。**

### 延迟是**量出来的身体常数**,不是要去解决的工程问题

上层 VLM **可以换成任何一个**,客户算力平台**我们不知道**,甚至可能直接连云端。
⇒ **任何"把这个模型在这台服务器上跑快"的手段都不属于这一层**(云厂商自己在做)。
属于这一层的是**让它慢也没关系**:

| | |
|---|---|
| 不问的情况下能自己跑多久 | 量出来的 |
| **该问的那一刻** | 预测崩了就是(方向盘本身是预测器) |
| 等回话时 | **不许停** |
| 断网 / 回话没来 | **永远正确的反射**:冻住 + 不松手 + **沿自己刚走过的路退回去**(唯一一条有正面证据是空的路) |

🔴 **开机顺手量一次"我这个脑子回一句话要多久"**:慢(云端 2 s)⇒ 一次多要一点、更靠自己扛;
快(本地 0.1 s)⇒ 问得更勤。**延迟也是量的,不是填的。**

### 换成 Fable5 / 任何 API:**一行都不用改**

接口从头到尾只是"发一张图 + 一段话,收回一个很小的 JSON"。**驱动不要权重、不要隐状态、
不要模型在本地** ⇒ 换脑子 = 换一个地址。
🔴 **纪律:最终那个官方分要用【开源本地模型】再跑一遍** ——
*"同一套驱动、换两个完全不同的脑子都跑得出来"* 本身就是更强的结果。

### 🔴 快路只对【我们自己跑得起来的模型】成立

"权重开源"和"摸得到隐状态"是**两回事** —— 走 API(哪怕对面是 Kimi3 这种开源权重)
拿回来的**只有文字**,和闭源一样。而那种体量自己跑就回到"装不下"。
⇒ **那条 20 倍的快路只在小到我们自己能跑的模型上成立。本地小模型不是给别人用的,它是快路的唯一载体。**

### 两种完全不同的东西都被叫成"小头" —— 混起来会做错决定

| | **甲:读数头** | **乙:蒸馏成小模型** |
|---|---|---|
| 大模型还跑不跑 | **还跑** | 不跑了 |
| 替掉什么 | 只替掉"一个字一个字拼答案"那 3.71 秒 | 整个大模型 |
| 快多少 / 要不要装大模型 | **约 20×** / **要** | 几百× / **不要** |
| 能力 | **就是大模型的能力** | **只有蒸馏过的那一片**;没教过的**会自信地答错** |

训甲:**冻住主干,只训外挂头**,教师是我们自己的慢版本。
🔴 红线:**不许微调主干**(本仓有数:同数据整模微调 12.5% vs 只训受保护小模块 94.5%)。
⚠️ 实测前提:`why` 排在答案前面才对(它写"该去第 6 格"而答案填 **12**)⇒ **推理就是计算**,
头必须训成"**想完之后**的答案",不能天真直读隐状态。
白拿:头给**概率分布 ⇒ 有了"我有多确信"**,不确信自动落回慢路;快慢答得不一样 = 免费的"该醒了"信号。

### 能力拆两半,而快的那一半**不需要大模型的能力**

| | 需要什么 | 需要多快 |
|---|---|---|
| **知道该干什么**("现在是打架,别把手伸直") | 世界知识 ⇒ 必须大模型 | **几 Hz 就够** |
| **在这具身体上做出来** | **一点世界知识都不需要**,只要量出来的身体 | 要快 |

⇒ **能力是通过"它一秒前说的那句话"进来的,不是通过快头的权重。**
🔴 由此,**"没教过战斗会不会打"取决于输出词表是不是任务无关的**:
词表是"哪一块 → 哪一格 → 多快 → 到什么事件为止" ⇒ **战斗不必教给头**(1000 Hz 上出拳和抓取是同一个原语);
词表是"抓/推/撬"这种动词 ⇒ 没教过就是不会,**而且装作会**。
**⇒ 删动词表不是洁癖,它决定这条路走不走得通。**
⚠️ 诚实的另一半:**词表能迁移 ≠ 本事能迁移** —— 只在"慢慢靠近"训过的头没学过甩得快、
没学过中途松手。**说得出**新任务,**做得到**只到训过的物理区间为止。

### 1000 Hz:大模型永远到不了,大模型的**知识**到得了

| 每步跑什么 | 要搬 | 3090(936 GB/s) | H100(3.35 TB/s) |
|---|---|---|---|
| **完整 9B 一次前向** | 18 GB | **52 Hz** | **180 Hz** |
| 1B 蒸馏头 | 2 GB | ~460 Hz | ~1600 Hz |
| **50M 小头 + 缓存好的上下文** | 0.1 GB | **~9000 Hz** | — |

🔴 **9B 要 1000 Hz 需 18 TB/s 带宽 —— 没有这种芯片。这是物理,不是工程。**
⇒ 1000 Hz 那层的每步模块必须 **≲100M 参数**;而且**那一层没有图像**(相机 30–120 Hz)
⇒ **它读力和关节** —— 正是 `fast/` 该有的样子。

🔴🔴 **没人做过的那一点不在速度上,在【小头被什么条件化】**:别人的小头条件是**任务**;
**把"量出来的这具身体"(通道表 / 部件图 / 交付率)放进条件** ⇒
**同一个小头换一具身体不用重训,因为身体是【输入】不是【权重】。**
慢环那半已经在做,**快环这半是它的下一步。**

### 今天不换大模型的理由(实测,不是保守)

今天的失败**全在驱动,不在脑子**:眼挑目标 **6/6 全对**(渲图为证)· 推理也对
(*"球在第 11 格,要拿起来得去正上方那一格,第 5 格"*)· **唯一答错那次是我的 schema 字段顺序写反**。
⇒ 换 Fable5 在今天这个阻塞上买不到东西。**等题变难再换**(同一份人形任务榜:最好 16.8% vs 3.4%,**差五倍**)。

**逐条读数、加速清单与出处** ⇒ [`results/eyespeed-sep2026/`](results/eyespeed-sep2026/)
(实测 3.90 s / 524 进 98 出 / 固定开销仅 0.18 s ⇒ **判死"缩小图"和"缓存提示词"**;
三条逐 token 等价的加速;**量化陷阱:总分只掉 1% 而 19–21% 的答案变了、6% 从对变错**)。

### 这条路上世界有什么、没有什么(自由授权搜索,2026-09-01)

🔴 **一句话:这条路被劈成两半,两半各自有人做,而【没有任何人把它们接起来】。**

| 半 | 谁在做 | 缺的那一半 |
|---|---|---|
| **机器人自己动一动、量出自己的身体** | Lipson 组(Science Robotics 2019 / 2022)· π-graphs(SciRob 2023)· AutoURDF(CVPR 2025)· **MAVRIC**(RA-L 2020,Levine)· DIJE(2507.00446) | **这一整条线里一个大模型都没有**,而且**每换一具身体都要重训一次** |
| **通用大模型当机器人的脑子** | HumanCLAW · Butter-Bench · Anthropic robotics · FAEA · Prompt2Walk | **这些模型完全不去量自己的身体** |

**测到那道缝的那篇是 HumanCLAW(2607.27180)**,原文两句:
> **"No current VLM perceives its own body."**
> **"it behaves like a ghost: it holds no instinctive model of which pixels are its own limbs, and it never tries to infer where they are."**

### 前沿真实的数(论文里的,不是宣传)

| 出处 | 任务 | 数 |
|---|---|---|
| HumanCLAW | VLM 指挥人形 找→走→坐 | **最好 Gemini-3.1 16.8%**(找到 64.9 / 走到 42.4 / 坐上 16.8)· GPT-5.5 **3.4%** · 腿脚 **28–45% 的步子在撞** · 交互失败 **58% 是坐进空气里** |
| Butter-Bench | LLM 开 TurtleBot4 | 最好 **40%** vs 人类 **95%**;**"察觉东西不在" 所有模型 0% / 人类 100%**;"直接朝目标画直线,**完全不管墙**" |
| Anthropic | 直接给力矩 | 整任务 **0–5.5%**;Go2 平衡 **"nearly two full seconds"**;**给指南针工具效果远好于其他** ⇒ 反证模型缺自我朝向感 |
| LAP-3B(2602.10556) | 没见过的机体 | 自己 3 机体 6 任务平均 >50%;**"all open-sourced VLAs collapse to zero success rate"** |
| RoboInspector(2508.21378) | LLM 写控制代码 | 216 组;主导失败是 **Badpose(末端姿态错)** |
| RoboVista(2607.04610) | VLM 空间量测 | 最好 **56.5%**(随机 20%);**30.2% 是把东西认错**;**思维链让场景理解掉最多 12%** |
| TRI(2507.05331) | 大行为模型 | **"Pretrained LBM is statistically indistinguishable from single-task"**;某任务微调后 **0.335 < 单任务基线** |

⚠️ **这三条直接打在我们身上,不许当成"接上 VLM 就好了"**:VLM 自己就是那个不知道身体在哪的鬼魂(16.8%)· LLM 写控制代码主要死在末端姿态(= 我们"转手腕被整条拒掉"那条)· VLM 空间量测 56.5% / 30% 认错东西(= 本仓 `"问眼睛 246.9"` 那条负面前科的同族)。

### 口号 vs 证据

| 谁 | 说的 | 实际 |
|---|---|---|
| **Skild AI**(融资 $1.4B) | "omni-bodied""**even without knowing its exact body**""换全新身体不重训" | **博客里一个数字都没有** · 无论文 · 无技术报告 · 无权重 · 无消融 |
| **Generalist GEN-0/1.5** | "works on different robots by design" | 只说测过 6/7/16+ DoF,**没有任何"训练里没见过的机体"的说法,也没有分机体数字** |
| **π 0.7** | "zero-shot cross-embodiment" | **那台机器在训练数据里**,只是任务没见过 |
| **Gemini Robotics 1.5** | 三种机体"无需 robot-specific post-training" | **三台都在多机体预训练集里** |
| **NVIDIA GR00T** | "cross-embodiment model" | 🟢 **最诚实的一家**:官方文档明写新机体要采演示 + 微调 embodiment-specific encoder |
| **波士顿动力 ZEST** | "Zero-shot Embodied Skill Transfer" | 那个 zero-shot 指 **sim-to-real**,不是换身体。术语撞车 |

### 🔴 地图上的空白 —— 我们站的位置

多种措辞搜过,**找不到任何工作报告过**:

1. **没有人把"自己量身体"和"大模型当主体"接起来。零篇。**
2. **没有人在运行时发现"我有几个自由度"** —— 跨形态方法全都把动作维度当已知(MetaMorph/AnyMorph/GET-Zero/AnyBody 吃形态图 · XMoP/Mirage 吃 URDF · Cloak 吃 known geometry · CrossFormer 明写不支持变自由度)
3. **没有人从观测里判断"我是吸盘 / 两指 / 五指"**(GET-Zero 明写 >16 关节预期失败)
4. **没有一套发现流程同时管 臂 + 腿 + 轮**(Lipson 组自己是分开的:4-DoF 臂 / 12-DoF 腿 / 纯本体感觉拓扑)
5. **没有人给"量身体"定过【一集里花几步】的预算** —— 别人报的是 7,888 点 / 300–500 张图 / "about a day on a single GPU";唯一例外是 MAVRIC 的"约 20 秒 / 约 100 个动作"(**2019 年**)
6. **没有人量过"完全不给 URDF,代价是多少"**(只有边角消融:XMoP 去掉 SE(3) 本体感觉 ⇒ **0%**)
7. **没有任何基准带"换身体、不改代码"这条轴**(AnyBody 最接近,但把形态向量喂给策略;RoboTwin-XE 的跨机体初始位姿 **"aligned via IK"** ⇒ 运动学已知)
8. **没有人让大模型自己设计"怎么量自己"的流程**(最接近的 ICWM 探测动作是**写死的**,模型还专门训过)
9. **没有人用自己量出来的身体做"不许碰到"**(Lipson 96.84% 避障是单台 4-DoF + 全标定 + 训一天;XMoP 77 台但读 URDF)
10. **自模型本身不迁移** —— **不存在"自建模的基础模型"**
11. **没有人报过"量身体阶段的大模型调用预算"**(分层 VLA 摊薄的是**任务**调用:规划器每 ~100 步一次、~2 Hz)
12. **"在图像里认出自己"这个原语,2019 年的 MAVRIC 至今仍是最好结果**,没人用现代基础模型重做过;唯一近期后继是 **DIJE** —— 就是本仓自图那篇

**相邻立场论文**(2606.06556 *Robots Need More Than VLAs & World Models*)撞在同一点上:
> *"The embodiment gap … is not how to copy human motion, but how to **preserve the task-relevant physical effect** of that motion when executed by a different body."*

它主张机器人要对自己的形态/运动学做**显式推理**,但给的方案仍是**人给的**结构化重定向,**没有走到"自己量出来"这一步**。

⇒ **本仓的位置:第 1 条那道缝里,而且第 5/6/7/11 条正好是这一层天生要回答的问题(预算按集内步数、零 URDF、换体不改代码)。**

<details><summary>出处(点开)</summary>

自模型/身体发现:[SciRob 2019](https://www.science.org/doi/10.1126/scirobotics.aau9354) · [Full-Body Visual Self-Modeling](https://ar5iv.labs.arxiv.org/html/2111.06389) · [Egocentric Self-Modeling](https://arxiv.org/abs/2207.03386) · [π-graphs](https://www.science.org/doi/10.1126/scirobotics.adh0972) · [AutoURDF](https://arxiv.org/abs/2412.05507) · [MAVRIC](https://ar5iv.labs.arxiv.org/html/1912.13360) · [DIJE](https://arxiv.org/pdf/2507.00446) · [Body Discovery of Embodied AI](https://arxiv.org/abs/2503.19941) · [Sensorimotor Self-Recognition in MLLM Robots](https://arxiv.org/html/2505.19237v2)

跨形态:[LAP](https://arxiv.org/html/2602.10556v1) · [Cloak](https://arxiv.org/abs/2606.22836) · [Mirage](https://arxiv.org/html/2402.19249v2) · [XMoP](https://arxiv.org/html/2409.15585) · [CrossFormer](https://arxiv.org/html/2402.19432v1) · [GET-Zero](https://arxiv.org/html/2407.15002v1) · [Embodiment Scaling Laws](https://arxiv.org/html/2505.05753v2) · [AnyBody](https://arxiv.org/html/2505.14986v1) · [ICWM](https://www.alphaxiv.org/abs/2606.26025v2)

负结果:[HumanCLAW](https://arxiv.org/html/2607.27180) · [Butter-Bench](https://arxiv.org/html/2510.21860v1) · [Anthropic](https://www.anthropic.com/research/claude-plays-robotics) · [RoboInspector](https://arxiv.org/html/2508.21378v2) · [RoboVista](https://arxiv.org/html/2607.04610) · [TRI LBM](https://arxiv.org/html/2507.05331v1)

产业:[Skild](https://www.skild.ai/blogs/building-the-general-purpose-robotic-brain) · [Generalist GEN-0](https://generalistai.com/blog/gen-0) · [π0.7](https://www.pi.website/blog/pi07) · [Gemini Robotics 1.5](https://arxiv.org/abs/2510.03342) · [GR00T new-embodiment docs](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/3_0_new_embodiment_finetuning.md) · [ZEST](https://arxiv.org/abs/2602.00401)
</details>

---

## 🔴 三条减法(owner 2026-08-28,别人给的建议 + 我方判断)

**减法的意思**:删掉一个"人为了方便而发明、但世界本身不需要"的中间概念。
仓里已经用这条理由删过:复位点 · 手腕四元数 · 末端位姿 · "第几列是开合轴" · "东西放在一个平面上"。

| # | 减什么 | 状态 | 判断 |
|---|---|---|---|
| ① | **plug**(换一台机器人要手写的那一层) | 🟢 **本轮做掉大半** | 观测端本来就靠**形状**认(不看键名);命令端删掉了最后一处写死的机器人字段名(`hold_action` 的 `left_arm_joint_state` 等四个,当时是**死代码零调用**),深度通道改成**靠形状认**(浮点 dtype + 二维 shape),不再把彩色路径末段换成字面量 `depth`(原来 6 处)。加了机械检查 `check_purity.sh`(棘轮) |
| ② | **13 个动词表** | 🟢🟢 **2026-09-02 整片删除**(owner:*"必须现在删,不然算作弊"*) | 删掉四处:`问身体` 8 词 · `pick` 6 词 + 3 档力度 · `ask` 同一份 · `verb.rs` 的 `Verb` 枚举/`decide`/`demand`,以及 C 接口的 `bl_verb`、`bl_world_ref.verb`、`.manner`、`bl_after_check`。替代品 = 接触集第③格**在画面里的说法**:**"那个东西最后要落到第几格。"** 抓 = 落到它上方那格;砸 = 落到小人那格;合爪/换手/退回全变成驱动自己解的中间步骤。"用多大力"换成**顶到读数自己不再动为止** |
| ③ | **VLM 给的像素坐标 u,v** | 🟢 **换了生产者(2026-08-28)** | **已做**:候选由 `point_gen::分块` 从深度图切(形态学闭运算 —— 物体是深度上的坑,闭运算填平,差就是鼓多高)⇒ **不认识任何物体、不用任何检测器、不用相机内参**,所以那三个被判死的检测器一个都没碰。眼只负责从画了编号框的图里挑一个,并且有 `0`("一块都不是")这个出口。挑中那块的**掩膜**直接交给 `算一把`,那个猜出来的半径彻底删掉。真实深度图上验过:立方体/球/钞票/小车四件全中,三个伪影(背景 · 细缝 · **我自己的胳膊**)靠"贴画面边的丢掉"一次清干净 |

### ⚠️ 给这三条配的刹车(2026-08-28 用一整天换来的)

**只在替代品已经跑起来之后,才准删。** 当天犯的三个最贵的错全是这一条:
删掉写死的手腕姿态却没装上解出来的姿态(胳膊一路趴着扫过去)· 把"认手前等手臂停稳"删错了地方
(它把自己的胳膊当成了手)· 在一套早就写好、更完整的实现**上面**又写了一套更差的(十来炮白跑)。
**"删掉一个中间概念"和"什么都不做"在代码上长得一模一样,在行为上差十万八千里。**


## 现在在哪(2026-09-02 深夜,一屏读完)

🔴 **官方 `general_pickup`:0 / 55。至今没有抓起来过任何一个东西。** 目标 **≥42/55**(超人类遥操 76.03%)。
这一栏是唯一算数的成绩,不许拿中间指标替代。

**机体**:RoboDojo 官方 `arx_x5` 双臂。**它只吃末端命令,不吃关节命令。**

| 环节 | 状态 | 数 |
|---|---|---|
| 眼找球 | 🟢 | 单炮内 **6/6 · 13/13** 逐位相同 |
| **越用越强**(装回不重量) | 🟢 **今晚第一次成立** | 部件图 16 通道 × 3 相机、手指厚度、腕相机、通道表、雅可比**全部装回**;第一段模型决策从第 **3400** 行提前到第 **314** 行 |
| **动词表** | 🟢 **整片删除** | 模型每段只说两个号:**动第几号(我身上的块 / 世界里的块)→ 到第几格**,加一个事件、一个完没完 |
| **模型自主伸手** | 🟢 **今晚第一次出现** | 它自己说"动我身上那一块 → 球的格子,直到碰上";驱动逐通道试正反、沿最好的走到不再靠近(CG:0.106 → 0.091) |
| 眼(VLM)速度 | 🟢 | vLLM 0.28.0 + 自带 MTP 头,钉 GPU 1:**3.16 s → 2.08 s**(≈1.4×);决定 5/6 同旧眼 |
| 合爪 | 🟢 闸在 | 没有"物体在两指之间"的正面证据**一律不合**(以前 4–8 次空合/炮,今晚 **0**) |
| **记忆** | 🟢 **接入**(2026-09-03) | `memory.rs` 从"驱动调用 0 次"到每拍 `observed()`、换集 `NewTask`;任务/目标/试过几个下手点/手里有没有 随每次提示词给模型 |
| **N 个接触面** | 🟢 **改完**(2026-09-03) | 通道表列长 = 3×面数;追/退回/刚体闸全按面数;两指时逐式还原旧公式。⚠️ 只在两指上跑过 |
| **抓起来** | 🔴 | **0 次** |

**这一版对七个目标各有什么、验过没有**(owner 2026-09-03:*"这一版必须得能完成所有任务才能跑"*):

| 目标 | 这一版里的机制 | 跑过没有 |
|---|---|---|
| RoboDojo 抓 | 全链 | **0/55**;炮 CJ(飞行员 v3 + N 面)在跑 |
| LeKiwi 抓老鼠 | 每拍重看 · `快`(步间不等停)· 地点记忆 `Scope::Place` | 未跑 |
| 客厅收拾 | 任务/地点记忆已接;轮子 = 通道(由布局报) | 未跑 |
| 五指扣扳机 | N 面机器;握着时"动第 k 号手指块 → 格,直到顶住" | 未跑;**认接触面仍只交 2 块**,多面得来自身体文件 |
| 楼梯 / 蹲捡 | 腿 = 厂商控制器三个数(通路 18/18);蹲 = 通道 | 未跑 |
| 躲拳 | `别碰`:模型点名的块并入硬障碍 | 未跑 |
| 格斗 | `快` + `到什么为止`(事件)+ 边动边松 | 未跑 |

**现在具名的阻塞(按证据排序)**:

0. 🔴🔴 **2026-09-03 删光了驱动里所有替模型做决定的东西**(CJ:模型一次没被问;CK:23 段全被我的 keep 优先吃掉;CL:一步给太大被拒;CM:执行器第一次把点名的块推到位但它认错了号)。现在驱动只剩"列编号 → 问 → 一起解 → 报"。具名的下一个阻塞:**编号每轮重排、碎块混入,模型认错球的号**(CM)。
1. 🔴 **收尾伺服收不敛。** 腕相机里跟到的"物体"深度差一直在 **−0.17 ~ −0.21 m**(比手指远 20 厘米),`追 12/15` 读数冻住,`跟丢了` 每炮 5 次。识别球的身份键(鼓多高)已换上,但收敛仍未发生。**这是今晚每一炮的终点。**
2. 🟡 **模型自主伸手很慢**:一段 = 6 通道 × 3 探针 + 沿通道几步 + 一次问眼,只换来画幅 0.01–0.02 的靠近。形状对,效率差。
3. 🟡 **它有时把"第 11 格"和"第 11 号"混** —— 编号已画到图上,要盯。

**今晚删掉/撤回的**(每条都有炮号为证,见 `LAB.md`):8 词动词表 · 6 词 + 3 档力度 · C 接口 `bl_verb`/`manner`/`bl_after_check` · 我手写的三个"失败就挪一步"触发器 · 基于"够不着"错判的拉黑 · "正上方一格 = 抬离桌面"这条世界假设 · "同一格 = 什么都别做"这条规则。

**下一步(判据先写死)**:
- **炮 CN**(删光版首炮,目标固定棒球):**赢 = 球离开桌面**。看三件:模型每段说的号对不对(grid_NNN.bmp 对照)· 执行器把点名的块推到位没有(误差曲线)· 抓握那一号被点名后读数停在哪。**每炮出视频 + 大白话分析。**
- **加速第二条**:suffix decoding(`arctic-inference` 已装)单独对照 —— 我们每步同一个 JSON 骨架,是它的最佳工况。

### 🔴🔴 架构上的定论(owner 2026-08-27,已落地)

**凡是只在某一具身体或某一种场景上成立的东西,一律不许进驱动** —— 哪怕它能让这一炮抓起来。
> *"这一炮夹起来了,真机照样夹不起来。"*

**已经从主路上删掉的**(每一条都曾经"让这一炮更接近成功"):
末端位姿 · 手腕四元数 · 工具轴 · "第几列是开合轴" · 钳口张开量 · 指尖偏移 ·
够多远 · 世界的上下 · 原位 · **"东西放在一个平面上"**

**现在的主路只有四件事(2026-09-03 起,owner:"驱动里不许有替模型做决定的东西"):**
```
列编号(部件图切出的我身上的块 · 每只手的手腕和抓握 · 深度切出的世界块,全画在图上)
  → 问模型:1–4 条「第几号 → 格子 / 相对某号的方位(贴着·上·下·左·右·前·后)/ 别动」+ 到什么事件为止 + 快不快 + 别碰哪几号
  → 执行器把几条一起解:每块切模板跟,表 = 通道表初值 + 现场探 + 每步修(Broyden),别的块全当障碍
  → 报回发生了什么(事件 / 剩多少 / 各块现在在哪 / 抓握读数),下一拍再问
```
**驱动自己不决定任何动作**:用哪只手 = 模型点名的块在哪只手上;进场角度 = 离现在最近的可行解;合 = 模型点名"抓握去某物"。
门槛全是现场量的噪声地板(实到抖动 / 各块位置抖动 / 静帧噪声 / 读数噪声);接口和安全上限(网格几格、每段最多四条、一段最多 400 步)写明。
**通道 = 观测里报出来的每一个能下命令的自由度**(关节 / 手指 / 桨 / 轮 / 舵),数量由布局给。
🔴 **自由棘轮 `check_freedom.sh`(进 `install.sh`)**:代码里出现策略词、提示词里出现教程句、发命令处超过上限 ⇒ 装不上 = 停机。

### 🔴 架构上的另一条定论(owner 2026-08-27):**驱动里不许有复位键**

> *"直播的时候机器人永远没有复位键。"*

原来量通道表时每列之前会 `落` 回一个记住的基准位姿 —— 那就是复位。**已删。**
现在:探一个通道 = 走一个**来回**(+δ 再 −δ,相对命令),两遍各除以自己的实到位移 ⇒ 应当相等,
`分歧 < 共识` 才收;净位移为零,既不漂也不需要记住任何位姿。
**而且表不再只在开头量一次** —— 干活的每一拍用 Broyden 秩一修正把刚发生的事记回表里,
⇒ **"量身体阶段"这回事不存在了**;物体在跑 / 身体被改造 / 相机被撞歪都自己跟上。

**下一步:** 跑最终版,报官方 55 集的绝对数;然后按 200 步原配置重跑那一栏。
陷阱与逐炮读数见 [`LAB.md`](LAB.md)。
   现行的 `撑阈 = 0.01` 会被**一次正常的空合自己触发**(爪子先快后慢,后段每拍变化本来就 < 0.01)。


**🔴 已定案(2026-08-25,owner):**
- **删掉「15 格量不完就不干活」。** 缺的格不再回声不动;合爪判据(每根手指读数都停在半途)**零尺度**,爪宽只当长度尺用。
- **米制要从驱动和文档里删干净。** 人不知道自己手掌几厘米,只知道"这个杯子一把能攥住" —— 那是**比较**不是尺寸。
- **UX:没有独立的自标定流程。** 用户只有「用驱动干活」一个功能;缺什么在任务里当场补,无感、越用越准。缺东西的代价是**慢**不是**不能**,且必须**说出来**。
  例外:安全那几件不能靠干着干着学会 —— 起点就最弱(慢、力小、碰到阻力就停),量到再放开。
- **不全改 Ada。** `fast/` 已是 Ada/SPARK;要加深的是成色(`gnatprove` 证明义务清零 + 进 CI),不是扩大面积。

陷阱 / 教训 / 逐炮读数 ⇒ [`LAB.md`](LAB.md)。

---

## The invariant, stated once

The nearest ancestor is **UP-OSI** (Yu/Tan/Liu/Turk, RSS 2017, arXiv 1702.02453): a universal
policy plus online identification of body parameters. The single difference carries the whole
claim:

> UP-OSI feeds the measured body parameters **into the policy** — the policy is body-conditioned.
> Here they go **only to the execution layer. The policy's input contains no body parameter at all.**

"Measure the body, then hand it to the policy" is the entire field's reflex, and it *looks*
compliant: the body really was measured, nothing was baked into the weights. But once a body
parameter is inside the policy's input distribution, swapping the body **degrades quietly** instead
of failing loudly. Lose this and we are 2017.

So the enforcement is **structural, not procedural**. Read [`abi/body_layer.h`](abi/body_layer.h):

| port | what may pass | what **cannot be expressed** |
|---|---|---|
| `bl_world_ref` — any VLM / WM | normalised pixel `u,v`, region `extent`, **which numbered cell the thing must end up in** | no `z`, no pose, no object id, no task id, **no verb** |
| `bl_policy_in` — the action model | image + that reference | no joint angles, no link lengths, no camera matrix, no gripper span, no payload, no robot name |
| `bl_execute` — this layer | everything measured about *this* body | — |

**A pointer that cannot express a pose cannot leak one.** An auditor checking for a privileged
channel reads the struct definitions; if no member can carry it, no amount of downstream code can.

---

## A layer that cannot say REFUSE is not a body layer

Every measured quantity carries **value / uncertainty / probed range / timestamp / dependency list
/ self-test / the version it replaced** — [`slow/src/measurement.rs`](slow/src/measurement.rs). A
bare `f64` cannot be refused on.

`bl_admit` refuses when a quantity was never measured, has gone stale, is being asked outside the
range it was actually probed over, is not precise enough for the ask, fails its own self-test, or —
the case a wall-clock TTL cannot catch — **something it was measured *against* has since moved**.

That last one is the whole reason `deps` exists. A hand-written `"my maximum payload is 500 grams"`
does not know the arm is sagging today, and **nothing in that system will ever notice**. The
category's admission test is exactly this:

> Change a hardware condition — add a weight, loosen a joint, knock the camera — and see whether
> the system **notices**. If it cannot notice, this layer does not exist in that system.

And the other half:

> Put it on a body with **different kinematics**, give it a fresh body layer, retrain **nothing**.
> Not "under 200 demonstrations". **Zero.**

---

## Layout

```
上层        ── VLM / WM   ── 看世界、听指令、判"擦干净没"    ← 学 · 秒级 · 可走 API
动作模型    ── 世界层意图 → 轨迹                            ← 学 · 几十毫秒 · 本地权重
════════ 量 / 学 分界线（body layer 的边界定义）════════
body layer 慢面 ── 量身体 · 存标定 · 判过期 · 拒绝           ← 量 · 秒级 · Rust
body layer 快面 ── 限位 · 力限 · 看门狗 · 急停               ← 量 · 硬实时 · Ada/SPARK
```

| path | what |
|---|---|
| [`abi/body_layer.h`](abi/body_layer.h) | **the contract.** Stable C ABI, so binding it does not require adopting our language |
| [`slow/`](slow/) | Rust. Measure, store with provenance, schedule, judge expiry, refuse, **and state its own debt**. **Zero dependencies, zero allocation.** |
| [`fast/`](fast/) | Ada/SPARK. Limits, force cap, watchdog, e-stop. **`gnatprove --level=2`: 40/40 checks proved, 0 unproved** (18 run-time, 7 functional contracts, 2 assertions, 3 initialization, 10 termination) |
| [`bind/python/`](bind/python/) | ctypes binding, stdlib only. The stack this layer must serve is Python, and until this existed **nothing could call it** |
| [`realdata/`](realdata/) | real episode logs the probes are asserted against in `cargo test`, plus the script that regenerates them |
| [`conformance/`](conformance/) | `abi_check.sh` — header ↔ library symbols, both directions. `python_check.sh` — the ABI driven from a third language, refusals first, **and the header's enum values compared against the built library**, which the symbol check cannot see |

Both faces hold the **same** numbers; the fast face exists only because *a force limit checked once
a second is not a safety limit*.

**Why a C ABI**: the claim is "anybody's mind + anybody's body". If plugging in required linking
Rust, half the field is excluded on day one. A standard only one language can consume is not a
standard.

**Why no allocator in either face**: a hard-real-time layer must not depend on one, and a layer
that cannot build for the target it has to run on is not a deliverable. An earlier draft had a
single `Box` behind the opaque handle; the caller supplies the storage instead
(`bl_sizeof_body` / `bl_init`).

🔴 **And that second clause is currently false, measured 2026-08-09.** `cargo build
--no-default-features` fails with 26 errors, and it failed before this was checked —
`git show HEAD:slow/src/probe.rs` already used `sort_by` (in `alloc`) and `sqrt`/`hypot`/`powi`
(in `std`, not `core`). The `no_std` attribute, the Cargo comment and the sentence above have all
been describing something that has never compiled. It is not fixed here because the fix is a
decision, not a tidy-up: the float methods need either a dependency — against the zero-dependency
rule, which is itself load-bearing — or a hand-written shim that then needs its own proof. It is
recorded at the attribute in `slow/src/lib.rs` so the next reader does not believe it.

**Why SPARK from day one, not "Rust now, SPARK later"**: the standing order, and its stated reason
— *you will not get around to it*. This repository's hit rate on "we will switch later" is zero.
The commercial shape is selling compliance, so a kernel with a machine-checked proof is a **product
feature**, not engineering vanity.

---

## What the prover found that review did not

Three things, and they are the argument for doing this in SPARK on day one rather than later:

1. **A contract that was simply wrong.** `Clear`'s postcondition claimed the failing branch leaves
   the state halted. False whenever `Clear` is called on a state that was never halted — a legal
   call. Corrected to "on failure nothing changes", which is also the stronger guarantee.
2. **A window where the invariant did not hold.** Setting `Is_Halted := True` and then
   `Why := <reason>` leaves one statement in between during which a halt carries no reason. An
   interrupt landing there observes a state that must not exist. Fixed with one `with delta`
   update per transition.
3. 🔴 **An invented body constant, caught as an unprovable assertion.** `Install_Limits` used to
   seat the safe hold at the midpoint of each joint's range. The prover could not discharge
   `Lo + (Hi−Lo)/2 ≤ Hi` over floats — and chasing that proof would have been solving the wrong
   problem, because *the midpoint of an arm's travel may be through the table*. The safe place to
   hold an arm is **where the arm is**. It is now an argument with a precondition, checked, rather
   than a number this package makes up. That is the same rule as the rest of the layer:
   **nothing that describes the body may be invented.**

## The rule about guards

> A guard that has never failed has never been tested, and in the output it is indistinguishable
> from a guard that does not exist.

So every self-test in [`fast/fast_selftest.adb`](fast/fast_selftest.adb) and every unit test in
[`slow/src/lib.rs`](slow/src/lib.rs) is a case the guard **must** refuse, and the build fails if
one of them is admitted. Exactly one "must be admitted" case sits at the end of each suite, because
a layer that refuses everything is also not a body layer.

This rule is not abstract here. Instances already paid for: a two-state occlusion control whose
second clause made `0 > 0` print PASS; a docstring promising a `--ref` positive control that the
argument parser never implemented; a watchdog whose counter was broken, so its "no new episodes"
predicate was permanently true and it deleted a healthy leg's 15 episodes. **In every case the log
said the guard was fine.**

---

## How much of it exists

Asked directly on 2026-08-09 ("is the self-calibration already done?"), and it was not written down
anywhere, which is its own finding. **A named slot in an enum is not a probe.** The answer then was
**5 of 10**; five quantities were names.

**Now 11 of 11.** Every quantity has an estimator in
[`slow/src/probe.rs`](slow/src/probe.rs), and `slow/src/lib.rs` asserts it mechanically —
`debt::declared_only() == 0` fails the build the moment a slot is added without one.

| quantity | measured by |
|---|---|
| `hand_pixel` · `image_jacobian` · `arm_weight` · `latency` | the original four |
| `step_delivery` | added from a measurement — see below |
| `reach` | a band, not a radius; validated on 2174 real episodes |
| `gripper_span` · `backlash` · `contact_threshold` · `self_occlusion` | added 2026-08-09 |
| **`tool_offset`** | added 2026-08-09 **from a census of the live stack** — see below |

### 🔴🔴 "11 of 11" 说的是**代码在**,不是**量得出来** —— 别把这两件事读成一件

上面那张表回答的是"每一格有没有估计器"。它**不回答**"这具身体上真的量到了没有",
更不回答"量到的那个数是不是一个常数"。2026-08-18 用 **15 次从零开始的独立标定**把后两件事量了:

| | 绝对数 |
|---|---|
| 一炮从零跑完能拿到几格 | **4–10 格**,多数停在 **7/15** |
| 15 炮合起来有数的 | **12/15** |
| **跨 15 炮真的对得上的(最大相对散布 <2%)** | **1 格**(`step_delivery`,散布 1.0%) |
| 从没量到过的 | `friction` · `hand_pixel` · `gripper_span` |
| 抓取成功 | **0 次** |

散布(15 炮之间最大相对差):`contact_threshold` **1603%** · `image_jacobian` **564%** ·
`arm_weight` **238%** · `home_pose` **1954%** · `backlash` **72%**。
⇒ **"量到了"和"它是这具身体的一个常数"是两件事,而我们此前只验过前一件。**

零 GPU 重跑这张表:`python3 results/all15-aug18/collect.py results/all15-aug18/json/*.json`。
逐条 bug 与修法见 [`LAB.md`](LAB.md)(原 DRIVER_GOAL 已整篇并入) §五(2026-08-18 那几行)。

Two of them are checked against **real logs**, not only against synthetic cases a test author
imagined — `contact_threshold` against 520 rows of a press-depth staircase with PhysX contact as
ground truth, `backlash` against three 300-step sweeps of a 7-joint arm
([`realdata/`](realdata/), asserted in `cargo test`). The real data earned its keep immediately: see
*"what real data found that the unit tests did not"* below.

### ⛔ 开机日程已删除(2026-08-25,owner 定)

**装上就下命令,没有第二步。** 干到需要某个身体量而手上没有 ⇒ 它自己动一下去量,量完接着干。
🔴 **代价照记:旧版 N128–N143 共 14 炮,进入干活模式 0 次** —— 用户让它拿东西,它先量自己四十分钟。
而「装机量一次、永久有效」本身是个手填假设:换只手、挂个武器,爪宽和指尖长当场全变。

### `step_delivery` is the first quantity added because a body demanded it

Two arms on the same harness, same waypoint controller, same 45 mm commanded step: one delivered
**0.76** of it per control period, the other **0.11**. The per-waypoint step budget had been set
from the first arm, so the second could never reach a waypoint — **0.136 m of residual on every
episode**, surfacing as *"the arm stopped short of the pre-grasp waypoint"*, which reads like a
planner, reachability or wrist-convention fault. It was none of those, and every scalar in the log
was ordinary.

The instinct at that point was to open the simulator's actuator config and raise the second arm's
stiffness until it kept up. That is wrong twice over: **it types a body constant** (the debt this
layer exists to drive to zero) and **it is not portable** — a real robot has no config file to
read. Worse, it changes the physics the demonstrations are collected under, so the data quietly
becomes a different dataset.

Measuring instead and sizing the budget from the arm's own progress: residual **0.136 m → 0.0058 m**
— the first arm's own figure — with nothing about the robot changed.

⇒ it is deliberately **not** `latency` (dead time; both arms answered 1 period) and **not**
`backlash` (a dead band at a reversal; this is a shortfall on every step in one direction).
⚠️ And it is still measured *in the experiment's own Python*, not through this ABI. **Named debt,
not a solved problem.**

### `tool_offset` is the second, and it was found by counting what the live stack types in

One number, written by hand in **four** places in the deployed system, with three values for three
bodies:

| where | value | what it says |
|---|---|---|
| `L3_GRIPPER_BIAS` (env, deployed executor) | **0.145** | *"x5 = 0.145, franka = 0.102"* — copy it by hand out of `Assets/Robots/<body>/robot_config.yml` |
| the teacher's `flange_for()` | **0.145** | `tcp = flange + 0.145 · R[:,0]`, hardcoded — so setting the knob above fixes one of the two |
| the teacher's wrist-tilt ceiling | **0.145** | *"the flange sits 0.145 m back along the tool axis"* |
| a third rig's harness | **0.1034** | `tcp_off` |

**4.3 cm apart between two bodies, and 0.145 is the default.** A machine that forgets to pass it
does not fail — it executes with another robot's geometry. That is precisely the *quiet* degradation
this README opens by saying the design exists to prevent, running in production.

It is measurable by acting on itself: **turn the wrist and the working point sweeps an arc whose
radius is the offset.** No kinematics, no declared frame — the geometry is in the picture.

### What real data found that the unit tests did not

`backlash` scores each reversal against the body's own same-direction delivery, so it needs that
control ratio. Fed three real 300-step sweeps, one joint's continuation ratios scattered around
**0.00025 with a standard error of 0.279** — and the estimator divided by it and reported a dead
band of **1.01 rad, about 58°, on a simulated arm that has none.** Every unit test passed. The guard
is now that the control ratio must be separable from zero by its own spread, and the same real logs
are the regression test: the free-space sweep is answered on 6 of 7 joints (all ≤ 2.6e-4 rad, i.e.
zero), and the sweeps where the leg is pressed into a surface — where a joint fighting contact has
no established free-motion ratio — are **refused**.

`gripper_span` was caught by its own test rather than by data, and the shape is the same: on a
perfectly stuck gripper the true slope and the true residual are both zero, so in floating point
they come out as noise of the same size, and the sign test then decided the verdict from the sign of
a rounding error — reporting a jammed gripper as *"the jaws close as you command them open"*.

## What this layer still owes

🔴 **The most expensive thing in this repository was a true number that flattered us.**

`hand_filled_constants()` returns `0`. It is true, and it is a **structural** zero: nothing can
enter through `bl_measure` without a passing self-test, so it counts a set that is empty by
construction. It says nothing whatever about the constants that never came near this API — and
those are the ones running the robot.

The proof arrived as a measurement. A parameter search over the deployed teacher on 2026-08-09
found its **dominant** constant:

> `TEACH_HIGH_FRAC` — how far up an object's long axis to place the grasp.
> **≤ 0.30: 32 of 44 (73%). > 0.31: 10 of 100 (10%). Fisher p = 9.3e-14.**

The largest effect anybody has measured on that stack, and **this layer had never heard of it.** A
census of the same two files then found **45 environment knobs and a hardcoded camera matrix**
against ten declared quantities. So the honest statement is not *"we removed the hand-filled
constants"*; it is *"we removed the ones we thought of, and the biggest one was found for us by a
search."*

[`slow/src/debt.rs`](slow/src/debt.rs) is the correction, and it is readable through the ABI
(`bl_debt_total` / `bl_debt_outstanding` / `bl_debt_line`) so an auditor gets both numbers without
reading any Rust. **60 rows**, one per constant, each with where it is set, what this layer can do
about it, and what would discharge it. **12 are body constants this layer cannot supply today.**

| the entries that matter most | standing |
|---|---|
| **`TEACH_HIGH_FRAC`** (32/44 vs 10/100, p=9.3e-14) | **outstanding — no slot, on purpose** |
| `FX` · `CX,CY` · `CAM_POS` · `CAM_EULER_DEG`, hardcoded under *"FROZEN P1 rig; do not re-derive"* | the image Jacobian exists so that no intrinsic or extrinsic has to be written down — the claim is true of this layer and **false of the system it serves** |
| **`bl_spec.step_m`** — this layer's own | **outstanding.** Every command is scaled by it, no probe produces it, and it is the metric ruler `gripper_span` and `tool_offset` divide by |
| **`bl_spec.damping`** — this layer's own | **outstanding.** Documented as *"from the measured Jacobian's own conditioning"*, and nothing computes it — a promise kept by a comment |
| `TEACH_SETTLE` · `TEACH_REHOME_STEPS` | outstanding, and the cheapest to discharge: both are `latency` + `step_delivery`, already measured, not yet wired |
| `L3_GRIPPER_BIAS` · `TEACH_JAW_MAX` · `BPD_REACH_BOX` · … | **replaceable** — a probe exists |

Three things about this table are deliberate.

1. **`TEACH_HIGH_FRAC` did not get an enum slot.** Whether a grasp 30% up an object holds depends on
   the **object**, so it has not been shown to be measurable off the body — and this README's own
   finding is that a named slot with no probe is worth nothing while reading as covered. Its
   discharge test is pre-registered instead: with `gripper_span` and `tool_offset` measured, derive
   the clearance the jaws need and re-run the sweep. If the effect disappears it was a body constant
   in disguise; if it survives, it belongs to the model and not to this layer. Both outcomes are
   informative, which is the only reason to write the test down before running it.
2. **"Replaceable" is not "replaced."** A probe existing does not connect it to anything, and
   nothing outside this directory reads this layer. Collapsing those two into one green cell is
   exactly the move this file exists to refuse.
3. **The ledger audits this layer too.** A ledger that only counts other people's constants is an
   advertisement. Two of the twelve are in `bl_spec`, in the middle of the execution path.

## What is actually hard here, and what is already done

Recognising the hand is **done**: 1.7 cm → **0.62 cm**, reproduced across three independent
processes. What is *not* done is keeping it during the servo, and the archive is precise about it:

* *"three times the localisation reading improved markedly and the closed loop gained nothing."*
  Fixing the fit from 1.7 cm to 0.62 cm moved the latch **not at all**, over 32 paired layouts.
* fit-time error **2.0 px**; error **at the moment the hand is closest to the target**
  **4.9–14.6 px = 1.5–4.6 cm** — at or above the 2.0 cm latch radius.
  *"The version that fits best is the one that drifts worst."*
* the whole family of "give the fit more evidence" is refuted: repainting the robot took usable
  candidate pixels **11 → 173 (15×)** and the loop stayed **0/9**.

⇒ the specification for [`slow/src/hand.rs`](slow/src/hand.rs), written by that verdict: **re-measure
every control step, and abstain rather than guess.**

The trap it is built against is named in the file. The old selector was *"whichever rigid thing
responds most to my command is me"* — derived when the competitors were **the hand and its shadow**.
On a different rig the competitors became **different links of the same arm**, and the elbow, nearer
the camera at 0.393 m against the fingertip's 0.438 m, won the rule. The loop then aimed the elbow
at the mark and reported **0.04–9.3 px** of error while the truth was **167 px**.

> A selection rule derived for two candidates does not report an error when the candidate set
> changes. It just quietly picks wrong.

So the estimator does **not** take a maximum. It enumerates candidates and, when the top two are
within `min_separation` (default 1.50 — the fingertip/elbow gain ratio was about **1.11**), it
returns a refusal instead of the better of them.

---

---

# What the layer above says to it: **the contact set**

> Merged 2026-08-13 from `universal-grounding/ARCH.md` and `ARCH_NEW_TRY.md`, both now deleted.
> One conclusion, in one place, latest version only.

## The cut is fixed by mechanics, not taste

```
object side:  G · F_contact = F_object      G depends only on WHERE on the surface + the normal — body-independent
body side:    J_hand(q) · q̇ = v_contact     J_hand depends only on the BODY — object-independent
```

The only quantities both sides share are **contact point / normal / relative direction at that point**.
⇒ that is the unique place where *the layer above never names a body and the layer below never names
an object, and the information is still sufficient.* One notch up (end-effector pose + a gripper
scalar) has already assumed a two-fingered hand; one notch down (joint angles) has already assumed a
DoF count. Murray–Li–Sastry ch.5.

⚠️ **Correction on the record (owner, 2026-08-13):** the deleted `ARCH.md` framed this as *"the old
interface was two-fingered"*. That is wrong about **this** architecture and was repeated in chat
before being checked. The `bl_policy_in` port above has never carried a gripper span — the two-finger
assumption lives in the **deployed RoboDojo/L3 action space** (`dx dy dz drx dry drz grip`), which is
glue, not this contract. The contact set replaces *that*, and its argument is the mechanics above,
not a flaw in this layer.

## The contact set, stated once

> ① which points on the object surface are touched · ② the normal at each and the **cone** of force
> allowed there (direction, no magnitude) · ③ how the **object** must move (a twist) · ④ tolerance

Thirteen verbs collapse into one template — *touch these points, push this way at each, and the
object does this*:

| verb | filled in as |
|---|---|
| push · sweep | one point (or edge), lateral force, object translates on its support |
| press · tap | one point, normal force, object does not move (tap = with velocity) |
| pry · flip · scoop | one point at an edge, up/side lever, object rotates about that edge |
| grasp | ≥2 opposed points, inward force, object follows the hand |
| pour · twist · insert | already held, object rotates/translates about an axis |

⇒ what has to be learned is no longer *thirteen skills* but **how to fill this template's
parameters** — half computed from geometry, half supplied by semantics.

**Reach forbids two fingers by construction**: nothing in ①–④ mentions how many fingers exist. A
suction cup fills the same table with one point and a normal-only cone.

## Four producers, and what each is forbidden to name

| layer | produces | from | 🔴 must never contain |
|---|---|---|---|
| **① body layer** (this directory) | constants of *this* body | **measurement** | any scene or task quantity |
| **②a contact generator** — `contact-gen/`, its own crate, 13/13 tests | where this shape can be grasped (point · normal · jaw direction), filtered by ① | **computation** (geometry) | semantics, task |
| **②b closed-form executor** — `contact-exec/`, its own crate, 5/5 tests | contact set + object twist → joint trajectory | **computation** | — |
| **③ eye + weights** | which object · what to do · where it is **useful** to touch (not merely stable) | **VLM / learned** | metres, joint angles, DoF count, finger count, link geometry, gripper opening |

`contact-gen` **does not read this layer**: body constants are passed in as arguments. The reverse
dependency would destroy the mechanical guarantee that ②a is body-independent. It speaks the same
process protocol as `bl` — `cg` with `body` / `grid` / `pts` / `gen` / `at`.

### Rules ②a already encodes, each paid for by a measurement

| rule | why |
|---|---|
| **height above the support is a THRESHOLD, not a maximise** | written as a maximise it dominates every later term and picks the topmost soft skin — a shoe's collar, the top face of a flat-lying hammer. Render-decided, and the only one of four revisions that raised the score: 26% → **34%** |
| 🔴 **never delete a candidate for being "too wide"** | the one grasp-ability rule in this repo forbids using the jaw opening as a filter. Width enters the **ordering** only; segments that do not fit stay in the table, ranked last |
| **a refusal must be able to state its reason** | `Refusal::{JawSpanUnknown, TooFewPoints, Flat, NoSection}` — never a silently empty list |
| **jaw span unmeasured ⇒ the whole layer refuses** | `JawSpan::{Measured, Declared, Unknown}` |

⚠️ **Measuring a width correctly is not grasping better.** Changing band thickness to the jaw face
height *did* fix the measured width (a block stopped reading 0.0048 m) and cost **12 pp** — it also
mixed two heights of material into one band, and *closed-on-nothing* went 21% → 33%.

### Still owed by ②a

| owed | status |
|---|---|
| **"can it be pinched"** | once measured locally, *"the jaws fit"* is almost always true for a convex solid ⇒ what actually decides is friction / depth / stability, which geometry alone cannot answer. **Slip is 41% of failures** |
| surface normals · friction cones · centre-of-mass alignment | the current last-place sort key ("how deep the material is") is an invented proxy; no published grasp-quality metric is that |
| jaw-face height | no slot in this layer; passed as a parameter today |

## Body / world / semantics — the spine

| | belongs to | obtained by | valid for |
|---|---|---|---|
| tool offset · jaw span · contact threshold · reach · home pose · **how much force a command produces** | **body** | **measured** (this layer) | one calibration, long-lived; new battery / worn part ⇒ measure again |
| how heavy this thing is · where its centre is · whether it slips · how wide it is *there* | **world** | **measured by touching it** | asked per object, remembered once asked |
| which object · what it should become · where it is **useful** to hold | **semantics** | VLM / learned | — |

### 🔴 Force must be stored as two halves and never multiplied together

> owner 2026-08-13: *"抓苹果这种任务做多了是不是就不用再碰了?但这个发力跟机体状态有关吗,满电/新旧都有差别。"*

**How many newtons an apple needs** is a world property and can be remembered. **How many newtons a
command produces** is a body property that drifts with charge, wear and temperature. Store the
product — *"grasp apple at 0.37"* — and a battery swap invalidates it. Store the first, ask this
layer for the second, every time.

⇒ this is where measuring beats practising: a human cerebellum re-trained for a new arm takes months;
re-measuring takes minutes.

### Brain / cerebellum, and why the analogy earned its place

Cortex = ③ the eye (decide what is wanted). Cerebellum = ① this layer + ②b (internal model + fast
loop). The model a cerebellum holds of its own body **is** the set of measured body constants —
a forward model.

It predicted the observed symptoms before they were explained: cerebellar damage presents as *intent
intact, but overshoot, mis-graded grip, jerky motion.* The robot: knows it wants the block, reaches
roughly, knocks it away, cannot hold it, flings it 0.27 m in one step. Textbook.

**One difference, and it is the advantage: theirs is trained, ours is measured.**

## More constraints ⇒ a smaller answer set

An unconstrained search is infinite; every **true** constraint collapses it by an order of magnitude.

| constraint | removes | from |
|---|---|---|
| what my body can do | everything this body cannot execute | **① measured** |
| only these points can be touched | every unreachable contact | **②a computed** |
| what the world must become | every action irrelevant to the goal | **③ eye** |
| physics | everything impossible | the world model |

🔴 **Measured facts are HARD constraints (this arm simply cannot reach); learned facts are SOFT
preferences. Hard constraints cut the space; soft ones cannot.**
🔴 This is also why *refuse rather than invent* pays: an invented number is a **false** constraint —
it removes the correct answer, and nothing downstream can tell.

### Scenarios, and the constraint each one forces

| scenario | constraint it forces |
|---|---|
| real robot (not sim) | must run in real time; **cannot see the far side** — no pretending a full model exists |
| new body (6-axis → 7-axis → dual-arm → wheeled → humanoid) | the algorithm may not contain *"how many joints"* |
| new end-effector (2-finger → 3 → 5 → suction) | may not contain *"how far open"* as a single scalar |
| cheap hardware (LeKiwi) | no high-precision feedback, no force sensor |
| grab a scurrying toy mouse | must be able to **re-plan at any instant**; latency dominates |
| tidy a living room for 30 min | long tasks must decompose, and mistakes must be recoverable |
| **dodge a punch** | 🔴 must be able to express **"do not touch"** — zero contact points plus a clearance |
| a person nearby | no unpredictable large motions |

🔴 **The dodge row is a real finding**: *wanting to touch* and *wanting to avoid* are the same
constraint with opposite sign, so one solver covers grasping and evasion — not two.

## Three solvers, one language

| tier | rate | job |
|---|---|---|
| slow | s–min | what the world should become → a sequence of contact sets |
| middle | 10–100 ms | given a contact set, how the joints move |
| fast | ~1 ms | did it touch, did it slip → fix in place |

All three speak contact sets. This layer answers *what this body is* alongside them, **and refuses
when it does not know.**

## Probing the world: three questions, all through channels that already exist

No force sensor required.

| question | how it is asked | which number is read |
|---|---|---|
| how wide is it there | close gently until it stops | jaw reading × jaw span |
| how heavy | lift 3 cm | commanded travel vs achieved travel |
| does it slip | same lift | **do the jaws keep closing** — measured: held median 0.0049 vs not-held 0.1755, zero overlap |
| is the centre off | same lift | how much it rotated relative to the hand once lifted |

**"Touch it first" is not a hard-coded step — it falls out of the arithmetic.** Roll the plan once
with each unknown parameter at its pessimistic end; if the pessimistic roll cannot succeed, probing
is worth its cost, otherwise act now. Second time the same object is seen, its table is still there
and nothing is probed. *(This supersedes the earlier `+ λ·unknown` cost term, which needed a
hand-picked λ.)*

## 🔴 A driver may not be written against a benchmark

**This layer is going into the world, not onto a leaderboard.** It answers *what is this body like*,
never *how does this benchmark score*.

| | allowed | forbidden |
|---|---|---|
| comments | citing a benchmark as an **example** | citing *"this leaderboard requires X"* as the **reason** a quantity exists |
| code | — | **any** benchmark / task / scene name |

Second clause: **no Python anywhere in the driver tree.** Once `bl` is a process (one line in, one
line out), the 725-line ctypes shell has no reason to exist and its presence made *"the driver"* mean
one Rust program plus a Python file drifting behind it. Deleted.

**A documentation rule only counts once it is a check that can fail** ⇒ `body-layer/check_purity.sh`,
run before每次提交: ① no benchmark names in `body-layer/slow/src`, `body-layer/contact-gen/src` or `body-layer/contact-exec/src` after stripping
comments; ② no `*.py` in the driver tree. Non-zero exit on either.

## What must genuinely be learned (everything else is measured or computed)

| item | why it is not computable | evidence |
|---|---|---|
| **scooping to a target mass** | a missing scalar, not insufficient precision: vision gives volume, volume→mass needs density, density needs force/weighing/acoustics | every work reporting gram-level accuracy reads it from a forbidden channel; pure-vision works report volume or fill fraction |
| **in-hand manipulation with many fingers** | published solutions are RL-only and cover single-axis continuous rotation; *rotate to a specified angle* is unsolved. ⚠️ the premise matters: that claim holds **without a model, without touch, without depth** | — |
| **deformables / granular / fluid** | no object frame and no finite contact set ⇒ the interface degenerates into a field, and the field's evolution is exactly what must be learned | folding has a closed form (G-fold, 50/50 on real towels); **flattening has only learned solutions** |
| **semantics** | force closure says where it is stable, never where it is useful (do not grasp a blade; pour by the handle) | — |
| **object priors** (μ / mass / fragility) | no direct measurement without force; geometry cannot imply them | initialised by the VLM, narrowed by interaction |

🟢 **Two-finger regrasping is the most complete non-learned region** — using external contact or
gravity to regrasp, flipping against a surface, putting down and re-taking — verified on real robots
without force, depth or touch sensing. The one gap is local contact geometry, and the three routes to
that are each published separately; **nobody has joined the two halves.**

## Honest limits

1. **Contact-set planning stops scaling as points multiply.** Classical results cover a few simple
   geometries; a five-finger hand has many contacts.
2. **Computed correct ≠ holds.** Force closure and ε-metrics destabilise within seconds when real
   friction differs from the assumption or a lateral disturbance arrives, and ε predicts robustness
   to pose error poorly.
3. **Global physical parameters cannot be learned from feedback** (measured on real hardware) — the
   signal is too weak. Feedback can adjust the moment of first contact, nothing more.
4. **"What humans do" here is inferred, not measured** — the 76.03% teleoperation figure is from a
   paper; it has never been run on this rig.
5. **Nobody has reported 76% averaged over 42 tasks.**
6. **Multi-step tasks multiply**: 38% per step ⇒ stacking (2 steps) 14%, packing (4 steps) 2%.
   **Below ~90% per step, matching a human teleoperator is arithmetically impossible.**
7. **Modularity's own cost**: six links at 0.7–0.9 each ⇒ 20–40% end to end. **"Retry on failure" is
   therefore not optional** — it is the only thing that pulls the product back up.
8. 🔴 **Covering 99% and minimising what is learned are in direct opposition.** Rigid bodies and
   articulated objects are almost entirely computable; the remaining ~30% (cloth, powder, liquid) is
   almost entirely learned. Engineering cannot dissolve this — one side has to be chosen.

## Lessons paid for in GPU time (2026-08-13)

Each of these produced a run that had to be thrown away.

| lesson | fingerprint it showed | fix |
|---|---|---|
| 🔴 **an unavailable value must default to the UNFAVOURABLE side** | `held = jaw_width < 0.9·(thickness or 1e9)` ⇒ with no material `0.0803 < 9e8` is always true ⇒ believed it was holding from step one; `touched_any` **26/26 = 100%** while object displacement was **0.000 m** | never fall to the favourable side. This layer's *refuse* discipline held; the layer above broke it |
| 🔴 **the same gate, second form** | closing to *a computed width* reached the command **exactly** (3.10 cm commanded, 3.10 cm reached) — i.e. it closed on air — and the old test still scored it as held: **10/11 by my record, 0/8 official** | **close until blocked and read where it stopped.** Independent of any width estimate and of the wrist convention |
| **feed all four slots of the contact set, or the cost is flat** | cost from slot ③ alone ⇒ nothing before contact changes anything ⇒ every candidate ties: 12/12 steps hit the search cap, displacement all 0.0000, held 0/12 | — |
| 🔴 **tolerance is per contact point, not per plan** (slot ④, defined and never used) | rendered frames show the jaws **already parked over the object, open, straddling it**, while the code re-planned forever because the *hover* waypoint was 5 cm off | millimetres where it touches; centimetres where it merely approaches |
| 🔴 **ask this layer before measuring anything yourself** | a whole run spent measuring "cycles of dead time" (`latency` = **0**, already stored) and another measuring per-cycle travel (`step_delivery` = **0.9999**, already stored) | `bl list` first. 14 quantities are stored; the glue was asking for 6 |
| 🔴 **do not rebuild what exists** | a Python grasp-candidate generator was written from scratch, re-deriving three rules `contact-gen` already encodes — and **violating** the *never delete a wide candidate* rule by truncating to the top 6 | `cg` is a process; call it |
| **a silent fallback is worse than a crash** | an asset path with the wrong case, swallowed by `except: return None`, left the point cloud **empty for five consecutive runs** while the code silently used a degenerate fallback that looks exactly like a poorly-performing feature | log which branch was taken, every episode |
| **ask the simulator for geometry; do not guess file paths** | as above | read the mesh from the live stage — it also removes the pose transform |

## Verification, fixed before the numbers exist

Ordered by *cheapest thing that can kill the idea*, not by *most impressive*. Any shot that fails
stops the sequence.

| shot | question | target | 🟢 win | 🔴 lose |
|---|---|---|---|---|
| **1 · expressiveness** | can the new interface state an action the old one structurally cannot | twisting a bottle cap — **0/16** on the old interface, named cause *"orientation is not in the control law's fixed point"* | official **> 0/16**, no regression elsewhere | still 0 ⇒ the derivation is wrong; the cut is not there |
| **2 · coverage** | is a whole class bought, or one task | the 18 RoboDojo tasks needing a final object **orientation**, now **1/14** | **≥7/18** | unchanged ⇒ expressiveness was not the bottleneck |
| **3 · cross-body** | does a kinematically different body run with **not one byte** of upper-layer change | ARX X5 (6-axis) + Franka (7-axis), both already on the rig | Franka absolute successes **≥70%** of X5's | needs an upper-layer edit ⇒ name the body quantity still leaking through |

**Instrumentation gates** (any one failing ⇒ the run does not count): `body_const_source` must read
`body_layer(...)` · `contact_thr_from` must read `body_layer(contact_threshold)` · `touched_any` must
contain a real True — **and must be read together with displacement**; held-without-movement is a
false gate, by construction.

**Accounting**: official `_result.json` `success` only · headline is **N attempts, M successes**,
percentages descriptive only · a temporary drop during a rewrite is expected and is not a loss.

---

## Acceptance: one long task, end to end, through this layer

🔴 **The leaderboard runs go *through* this ABI. No stitching alongside it** — a stitched path is
where cheating hides, and nobody, including us, would know it had happened.

**Target: `classify_objects`** — 1100 steps, a pile sorted into three baskets. Chosen because it is
long, it is inside what the interface can express (reach / grasp / place only), it needs the
multi-object loop *and* the return-home phase, and it is the one task whose target selection does
**not** go through the hand-maintained per-task table (that table is derived from each task's own
scoring function, which is precisely the kind of cheat this layer exists to make impossible).

Acceptance is not "it scored". It is all of:

1. every reference the eye emits passes through `bl_admit`, and refusals are **counted separately**
   from failures — *no data*, *not applicable* and *ran and scored zero* are three different things;
2. `bl_policy_in` carries no body parameter (checked by reading the struct, not by trusting a log);
3. the hand point is re-measured every step, with abstentions reported;
4. 🔴 **both** constant counts are reported, never the flattering one alone —
   `hand_filled_constants() == 0` (structural, counts only what came through this API) **and**
   `bl_debt_outstanding()`, which is **12** and is the number that describes the robot;
5. ~~the arm returns to origin~~ ⛔ **删除(owner 2026-08-23 死命令):直播删复位键 ⇒
   「回原位」这个概念本身就是复位的残余,上一个任务停在哪就是哪,**不存在可回的家**;**
6. and the numbers are reported as **N attempts, M successes** — absolute counts, never a
   percentage alone.

---

## Licence and shape

Whatever occupies this position has historically been licensed **per unit**. The shape here:
**fully open (copyleft) + a commercial licence + an ad-valorem per-unit royalty** — a flat annual
fee is hostile to small teams, and a $500 arm should not pay what a $200k industrial robot pays.

No patent application for now, deliberately, with one irreversible fact on the record: once
published, patent rights outside the United States are permanently lost (most jurisdictions have no
grace period).

---

# 记忆 —— 已经在驱动里,而驱动一次没调过

**实现:[`slow/src/memory.rs`](slow/src/memory.rs)(567 行,9 个测试),走 `bl_memory_*` 出接口。**
2026-08-11 从 Python 搬进 Rust(owner:*"我们整个 body driver 里面为什么会有任何一行 python"*)。
设计规则不再写在文档里 —— 它们**在那份代码里是结构性的**,读那个文件。

🟢 **2026-09-03 接入驱动**(此前 grep 调用 0 次 —— 能被调用不等于被调用了):`Scope::Task`,换集 `NewTask`,每拍 `observed()`;
格子只写不会自己动的事实(任务 / 目标 / 试过几个下手点 / 手里有没有 / 上一步结果),渲染成一段随每次提示词给模型。
接入前的代价:模型每一段拿到的"历史"只有**上一段那一句话**,于是它一遍遍说 *"上次没够着,再靠近一次"*。

**记忆按【多快过期】分层,每一层已经有主:**

| 层 | 例子 | 什么时候死 | 归谁 |
|---|---|---|---|
| 这一帧 | 那个杯子此刻在哪 | 下一帧 | **不存** —— 再看一眼 |
| 这个任务 | 我在干什么、干过什么、在等什么 | 任务结束 | `memory.rs`(`Scope::Task`) |
| 这个地方 | 垃圾桶在那个角 | 离开这个地方 | `memory.rs`(`Scope::Place` + `PlaceKey`,**会弃权**) |
| 这具身体 | 指尖离法兰多远 | 换机器或换工具 | 身体层 |
| 这个世界 | 刀要握把手 | 永不 | 权重 |

🔴 **第一层那条("会自己动的东西不许存")是场景相关的,不许照抄到客厅** ——
传送带上写下的位置几秒后就错;而沙发、垃圾桶、桌子的位置是任务里**最耐久**的事实。
一般形式是**按"它会不会自己动"分类**,不是"位置一律不许存"。

## 为什么"弃权"是全部的重点

实测(RoboDojo 传送带,5 集 / 4 腿 / 4 名词 / 4 布局,一致):名词→像素那个接口
**离【同类的错的那一个】22.8–50.5 px,离对的那一个 732–996 px**,而画幅只有 640 px ——
指称物根本不在画面里,**而接口没有办法说出这件事**。每一个答案都很自信。
⇒ **一个不会弃权的接口,会把"我看不见它"变成"它在那边"。** 模型再强也治不了。

## LeKiwi 测试(owner 2026-08-11 定的配方,原话不许转述)

1. 让它移动到某个物体上 2. 打乱物体 3. 说 **"移动到上次的物体上"**

🔴 **这句话要一字不改地给。** 换成"移动到红杯子"就什么都没测到,而且看起来还是过了。
三步共用**同一个**记忆实例;记录逐帧的格子内容当证据。

## 还够不到的(直接问过,owner 2026-08-11:半小时一次性收拾大客厅)

| | 今天 | 半小时收拾 |
|---|---|---|
| 吞吐 | 看一眼 ≈ 5 秒 | 1800 秒 ⇒ **最多 360 次**,而世界不等你 |
| 容量 | 8 个格子 | 几十个物体,各有去处 |
| 失败记忆 | 无 | "那个杯子滑了两次,换个抓法" |
| 检索 | 全量塞进提示词 | 得能**被查询** —— **这一段设计我没想清楚** |

5 秒/次这个数是算术不是意见:眼只能当**慢**环(决定下一步做什么),
快环归跟踪器(实测 146.4 Hz)和身体层。**这个分工就是架构,而它从没在长任务上跑过。**
