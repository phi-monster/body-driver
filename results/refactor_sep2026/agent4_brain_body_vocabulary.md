# agent4 — 脑↔身体词表自由搜索(2026-09-07)

派出条件:只给目标+约束+交付物,没给候选方法。原始报告全文如下(§0–§5 事实,§6 是 agent 自己的方案草案,已按 CLAUDE.md §3.8 折价看)。

---

研究做完了。下面是交付物。

---

# 脑↔身体"最小完备词表"调研报告

## 0. 大白话结论(老板只看这一节也够)

1. **老板记得的"Anthropic 做过类似的事",最可能是三件事之一,我都查到了**:
   - **2024-10 的"电脑操作"(computer use)**:模型只看屏幕截图,只用一小套固定动作(点、拖、打字、滚动……)去操作任何软件。Anthropic 明说这是刻意选择:**"Instead of making specific tools to help Claude complete individual tasks, we're teaching it *general* computer skills"**。而且他们发现模型**数像素**(定坐标)是瓶颈,专门训练了它数像素。动作集从 10 个 → 16 个 → 17 个(加了 zoom "放大看"),每次加词都是因为**模型做不到某个动作**(滚动/拖拽/看小字)。
   - **2025-11 的 Project Fetch**:8 个没做过机器人的员工分两队编程一只机器狗,有 Claude 的队 7/8 任务完成、用时约一半。这里 Claude 是**写代码**,不是直接发动作。
   - **2026-07-09 的《How Claude performs on robotics tasks》**:Anthropic 给模型四种控制接口(直接发关节力矩 / 写控制程序 / 用高层指令"监督"一个预训练控制器 / 训 RL)。结论一句话:**"When they must drive the joints themselves they mostly fail. But when they supervise a pretrained controller or use simple orientation tools, they can complete real navigation and manipulation tasks."** 直接控机械臂成功率 **0–5.5%**;给它一个"在夹爪相机上可移动、可询问的红叉光标"工具,10 任务子集成功率从 **6% 涨到 32%**。**这和我们"脑子只从编号里选、身体自己量"的路线是同一个判断。**
2. **各领域独立演化出来的"决策者↔执行体"接口,形状惊人一致**:一句话 = **动到哪(目标关系)+ 怎么动(每个方向是"管位置"还是"管力")+ 动到什么事件为止(守卫条件)**。机器人界 1981 年就把它写成理论(Mason 的 task frame formalism,"任务坐标系形式化":六个方向,每个方向二选一——控位置或控力),2010 年做成工业接口(Kröger 的 Manipulation Primitives:混合运动 + 工具命令 + **停止条件 λ**)。数控机床的 G 代码 4 条运动指令(G0/G1/G2/G3)+ 一条"探到为止"(G38.2 "probe toward workpiece, stop on contact")就覆盖全部加工;ABB 的 SearchL "the robot stops as the input bit goes high";无人机 MAVLink 规定"同一时刻最多一条运动命令 + 一条辅助命令"。**我们词表里的 `until`,就是这条 60 年历史的"守卫运动"(guarded move)。**
3. **"词有限但任务无穷"这件事,在几个地方有定理级证据**:Goldberg 1993 证明**只用一个动作(平行夹爪夹一下)**,任何多边形零件都能被摆正(到对称为止);Lynch & Mason 1996 证明"稳定推"这一个动作在平面里可控;Frazzoli 2005 证明一个**有限**的机动动作库(trim + maneuver)拼成的"正则语言"是可控的、可达集有内点。反面也有定理:一般化的"带不确定性的精细运动规划"(LMT 1984 框架)可计算但**双指数时间**(Canny 1989)——所以"脑子只说话,身体自己算"必须限制在每句话可局部核对的形式,不能指望身体做全局规划。
4. **人和动物的证据说:底层是少量"合成基元",上层是"选哪个+怎么搭配"**。青蛙脊髓刺激的力场**线性相加**(83% 情况等价于向量和);人抓 57 种物体的手形,**2 个主成分解释 >80%**;猴子运动皮层刺激 500 ms 直接得到"手到嘴""抓取"等**整套姿势**而不是单关节;小鼠行为可分成 40–100 个亚秒"音节"加转移概率(MoSeq),果蝇 >100 种刻板行为。**结论对我们的启发:脑子该说"要什么姿势/关系",不该说"哪个关节动多少"。**
5. **"词太多没人用"是各领域共同的失败**:多抓型假肢手有 14 种抓法,家用时 3 种占绝大多数(power 抓 55%);2023 年对照研究结论"多抓型 **did not show relevant differences** 比单抓型";民航驾驶舱"模式太多"直接写进事故报告(Asiana 214:**"complexities of the autothrottle and autopilot flight director systems ... increased the likelihood of mode error"**)。**词表膨胀不是免费的。**
6. **大模型公司"模型控制外部世界"的接口设计原则,Anthropic 写得最直白**(《Building effective agents》):"Put yourself in the model's shoes"、把格式做成模型在互联网上见过的样子、**去掉"formatting overhead"**(不要让模型数行号、转义字符串)、并举例:让工具**只收绝对路径**后模型"used this method flawlessly"。这和老板"脑子在定量几何上弱、在从编号里选上强,词表要顺着这个来"是一个意思。
7. **我们现在的词表最像谁**:图像上加编号网格 + 编号框让模型选,是 Set-of-Mark(2023)→ PIVOT / MOKA(2024)这条线;MOKA 甚至就是 5×5 网格(列 a–e、行 1–5)+ 每个物体上编号候选点 + 朝向只给 6 个离散选项。它们后来的演化方向:**(a)** 从"选点"到"选关系约束"(CoPa 用"A 的轴指向 B"这类句子;ReKep 用关键点上的约束函数);**(b)** 加"语言级中间动作"(RT-H 的 "move arm forward" 之类,>2500 组合;ECoT;Gemini Robotics 1.5 在动作前先用自然语言想"move gripper to the left so that it is closer to the clothes");**(c)** 2026 年出现让 VLM 直接设"动作基元参数"(Language Movement Primitives,31 个真实任务 65% vs 基线 35%)。**没有任何一家公开接口同时覆盖瞄准、力度、速度、节奏、多手、单指**——这是空白,不是老板漏看了。
8. 我的方案放在最后一节(§6),核心是:**把身体自己的接触部位也编号成"东西"**(于是不需要任何身体词),**用四个二值/三值"方式"槽**(来自舞蹈记谱 Laban 的 Effort 四因子:重/轻、急/缓、直/绕、紧/松)覆盖力度、速度、路径、顺从性,**用"相对于另一个东西"+"until 事件"覆盖多手与节奏**。以及它明确覆盖不了的:弹道式动作(投掷、击打)、精细定量(缝合)、连续步态、多点同时接触换抓。

---

## 1. Anthropic 到底做过什么(三件,按时间)

### 1.1 Computer use(2024-10-22 起)——"看截图 + 一小套固定动作"

- 设计理由(原文):"Instead of making specific tools to help Claude complete individual tasks, we're teaching it *general* computer skills—allowing it to use a wide range of standard tools and software programs designed for people." — https://www.anthropic.com/news/3-5-models-and-computer-use
- 模型弱项在坐标:"Training Claude to count pixels accurately was critical. Without this skill, the model finds it difficult to give mouse commands—similar to how models often struggle with simple-seeming questions like 'how many A's in the word banana?'"(仅在计算器、文本编辑器等几个简单软件上训练,不联网;"the model would even self-correct and retry")— https://www.anthropic.com/news/developing-computer-use
- 首发时承认做不到的:"Some actions that people perform effortlessly—scrolling, dragging, zooming—currently present challenges for Claude"。
- **动作集演化(来自官方 quickstart 源码的 Literal 定义)** — https://raw.githubusercontent.com/anthropics/anthropic-quickstarts/main/computer-use-demo/computer_use_demo/tools/computer.py
  | 版本 | 动作 | 数量 |
  |---|---|---|
  | `computer_20241022` | key, type, mouse_move, left_click, left_click_drag, right_click, middle_click, double_click, screenshot, cursor_position | 10 |
  | `computer_20250124` | + left_mouse_down, left_mouse_up, scroll, hold_key, wait, triple_click | 16 |
  | `computer_20251124` | + zoom | 17 |
  | `computer_toolset_20260801` | 同 17 个,改成"成员工具"(每个动作一个 tool) | 17 |
- 现行文档的关键规定 — https://platform.claude.com/docs/en/agents-and-tools/tool-use/computer-use-tool
  - 坐标一律用截图像素:"Every `coordinate`, `start_coordinate`, and `region` value ... is in the pixel space of the full-display screenshots you return ... after a `zoom`, Claude still expresses coordinates in the full screenshot's space, never relative to the zoomed image."(**放大只用来看,不改坐标系**——这是防混淆的设计)
  - "Providing the target description before the image is processed improves click accuracy."(**先说要点什么,再给图**)
  - 分辨率建议 1024×768 / 1280×800,"Avoid resolutions above 1920x1080"。
- **浏览器工具(`browser_toolset_20260801`)加了"按引用操作元素"**,理由原文:"A reference survives layout shifts and reflows that make pixel coordinates fragile, and lets Claude act on controls that are hard to hit with a pointer." — https://platform.claude.com/docs/en/agents-and-tools/tool-use/browser-use-tool (成员含 navigate/screenshot/zoom/各种 click/hover/drag/scroll/scroll_to/type/key/hold_key/wait/read_page/find/get_page_text/form_input/file_upload/tab 管理等)
- 文本编辑工具:`view / create / str_replace / insert` 四个命令覆盖全部编辑 — https://platform.claude.com/docs/en/agents-and-tools/tool-use/text-editor-tool
- **接口设计原则**(《Building effective agents》)— https://www.anthropic.com/research/building-effective-agents
  - "Put yourself in the model's shoes. Is it obvious how to use this tool, based on the description and parameters, or would you need to think carefully about it?"
  - "Keep the format close to what the model has seen naturally occurring in text on the internet."
  - "Make sure there's no formatting 'overhead' such as having to keep an accurate count of thousands of lines of code, or string-escaping any code it writes."
  - SWE-bench 案例:相对路径出错 → "changed the tool to always require absolute filepaths—and we found that the model used this method flawlessly."
- **MCP(2024-11)**:"a universal, open standard for connecting AI systems with data sources, replacing fragmented integrations with a single protocol" — https://www.anthropic.com/news/model-context-protocol 。协议只定义 3 个服务器原语:Tools(可执行)、Resources(数据)、Prompts(模板);客户端原语 Elicitation(向用户要输入);Sampling/Logging 在 2026-07-28 版已弃用 — https://modelcontextprotocol.io/docs/learn/architecture

### 1.2 Project Fetch(2025-11)——Claude 帮人写机器狗代码

- 8 名研究员分两队编程 Unitree Go2 捡沙滩球;三阶段(厂商遥控器 → 自写软件连机器人用传感器 → 全自主找球)。"Team Claude succeeded in about half the time"、完成 7/8 vs 6/8、"Only Team Claude made substantial progress toward the final goal: programming the robot to fully autonomously retrieve the ball."。有队员做了"a natural language controller"(如 "walk forward")。**Claude 不直接发动作,是写程序。** — https://www.anthropic.com/research/project-fetch-robot-dog

### 1.3 《How Claude performs on robotics tasks》(2026-07-09)——最贴我们问题的一篇

来源:https://www.anthropic.com/research/claude-plays-robotics(测试了 Opus 4/4.1/4.5/4.6/4.7、Mythos Preview、GPT-5.4、Gemini 3.1 等)
- 四种接口:**直接控制**(每步输出力矩/末端 7 维命令)、**程序控制**(写 Python 控制器)、**策略监督**(用高层指令驱动预训练策略,四足是 "velocity commands (forward, lateral, yaw)")、**RL 监督**(用 PPO 训);机械臂还测 **VLA 监督**("It can accept the proposed action, adjust it, or replace it entirely")。
- 观测:JPEG 相机帧 + 本体感知;"After every movement, it receives images of the scene along with readings from the gripper's force sensors."
- 结果:直接控机械臂 **"0 to 5.5%"**;人形站起 "No model successfully stood the robot up from a collapsed pose even once";四足高层导航最好 54/100;VLA 监督下 "every tested model still performs substantially worse than MolmoAct does on its own"。
- **有效的工具**:"a 'compass' which just gives the model its orientation in degrees ... handily beats the other ones";"a small red X on the gripper cam that the model can move and query for the object and distance at that point" → "success on the 10-task subset goes from 6% to 32%"。深度热图、分割叠加 "roughly neutral"。
- 失败模式:"fail at tasks that require stable spatial memory, self-localization, or long open-loop plans";"the models rely much more on the recent past than on a broad accumulated understanding";实机例子:某模型 "saw the target table in the reflection of the glass door and started charging for the glass door"。
- 速度:直接控制需 ~83 Hz,当前 API 0.2–0.4 Hz,"roughly two orders of magnitude latency improvement" 才能实时。

---

## 2. 各领域的"决策者↔执行体"最小动作集(事实)

### 2.1 机器人:工业语言 / 任务坐标系 / 操作原语 / 技能库 / 遥编程

| 体系 | 动作集(条数) | 为什么是这几条 | 完备性 | 已知失败 | 出处 |
|---|---|---|---|---|---|
| **G 代码(ISO 6983,数控机床)** | G0 "Coordinated Motion at Rapid Rate", G1 "at Feed Rate", G2/G3 "Coordinated Helical Motion", G4 "Dwell", **G38.2 "probe toward workpiece, stop on contact, signal error if failure"** | 刚体路径 = 直线 + 圆弧 + 停;接触 = 守卫探测 | 覆盖全部切削加工几十年 | 没有力控;所有几何是数字 | https://linuxcnc.org/docs/html/gcode/g-code.html |
| **PLCopen 运动控制(IEC 61131-3)** | 单轴:MC_Power/Home/Stop/Halt/MoveAbsolute/MoveRelative/MoveAdditive/MoveSuperimposed/MoveVelocity/MoveContinuous/TouchProbe/PositionProfile/TorqueControl…;多轴:MC_CamIn/Out、GearIn/Out/GearInPos、Phasing(约 30 个功能块)+ 一张轴状态机 | 目标:"simplicity, efficiency, consistency, universality, flexibility and completeness";硬件无关可复用 | 工业事实标准 | 是"数字接口",不解决"要多大力"的语义 | https://infosys.beckhoff.com/content/1033/tccncmcplatform/15303208587.html ; https://www.plcopen.org/application/files/3817/3868/2057/plcopen_motion_control_overview.pdf |
| **ABB RAPID** | MoveJ/MoveL/MoveC + **SearchL**("The robot stops as the input bit goes high")+ TriggL(路径上触发事件)+ Force Control("Set up desired force and movement pattern, Set up end condition, Activate force control … Wait for end condition to occur") | 运动三条 + 守卫一条 + 触发一条 + 力控一套 | 工业 | SearchL 没搜到 → ERR_MAXREACHED 需错误处理 | https://library.e.abb.com/public/b227fcd260204c4dbeb8a58f8002fe64/Rapid_instructions.pdf ; https://library.e.abb.com/public/bde123851c3c419580ee0512a055df98/3HAC050377%20AM%20Force%20Control%20RW%206-en.pdf |
| **Universal Robots URScript** | movej/movel/movep/movec + servoj/speedj/speedl/stopj/stopl + **force_mode(task_frame, selection_vector, wrench, type, limits)**("selection_vector: A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame")+ freedrive | 每个方向"顺从/不顺从"二选一 = 任务坐标系形式化的工业版 | 工业 | "Avoid movements parallel to compliant axes and high deceleration" | https://s3-eu-west-1.amazonaws.com/ur-support-site/115824/scriptManual_SW5.11.pdf |
| **CRTK(手术机器人通用 API)** | 命令名 = {servo, interpolate, move} × {j 关节, c 笛卡尔} × {p 位置, r 相对, v 速度, f 力};查询 measured_/setpoint_/goal_;状态 enable/disable/pause/resume/home | 三个"谁生成轨迹"的层级:servo "at a rate close to the low-level rate",interpolate "compute intermediary positions",move "send a single command and wait for completion" | dVRK、Raven-II、工业臂共用 | 只是运动学接口,不含任务语义 | https://crtk-robotics.readthedocs.io/en/latest/pages/api.html ; https://ieeexplore.ieee.org/document/9287933/ |
| **任务坐标系形式化 TFF(Mason 1981;Bruyninckx & De Schutter 1996 综述)** | 一个正交坐标系,"the available degrees of freedom of the objects in contact can be modeled as either pure motion freedoms or pure contact forces along one or more of the frame's axes";"The six 'directions' of the Task Frame are divided into motion-controlled directions and force-controlled directions" | 接触约束天然把 6 维分成"能动的"和"顶着的" | "coming up with an appropriate Task Frame is not too difficult for most simple contacts";有 "extensive catalog" | **"it cannot model all possible contact situations, even for contacts between polyhedral objects; difficulties sometimes arise when two or more primitive contact situations occur at the same time"**;力/速度方向"not necessarily orthogonal but reciprocal" | https://webpages.charlotte.edu/xiao/pdf/ar05.pdf (Lefebvre, Xiao, Bruyninckx, De Gersem 综述);https://www.semanticscholar.org/paper/f891df6eab7e66556c4e53b24b66d8fad73b3251 |
| **Manipulation Primitives(Kröger, Finkemeyer, Wahl 2010/2011)** | 一条 MP = 混合运动(任务坐标系 + 每轴控制模式/设定值)+ 工具命令 + **停止条件 λ**;同组 2014 年形式化为 Elemental Action = ⟨Device, DeviceCommands, λ, μ⟩,机器人命令 = ⟨flag, TF, GF, JV, ST, DA⟩,"a stop condition: λ → {true, false} which maps sensor values to a boolean value. It is a conjunction or disjunction about measurable values." | 标题就叫 "A Universal Interface between Sensor-Based Motion Control and Robot Programming";可"executing sensor-guided and sensor-guarded motion commands simultaneously ... any kind and any number of sensors" | 装配任务验证 | 章节付费墙,摘要未能逐字核;定义取自同组 arXiv 论文 | https://link.springer.com/chapter/10.1007/978-3-642-16785-0_17 ; https://arxiv.org/pdf/1409.6601 |
| **守卫运动/精细运动规划(LMT 1984)** | 基元 = 顺从运动 + 守卫终止;"preimage" = 从哪些起点出发目标"both guaranteed and recognizable" | 不确定性下的正确性判据 | 理论上完备但见 §3 复杂度 | Canny 1989:一般情形"double exponential in the number of plan steps" | https://journals.sagepub.com/doi/10.1177/027836498400300101 ; https://ieeexplore.ieee.org/document/99986/ |
| **技能库(SayCan 2022)** | 551 个技能,7 族(pick/place/move/knock/upright/drawer/navigation);LLM 打分 × 价值函数(可行性) | 把"想做"和"能做"分开 | 厨房场景 | 每个技能要单独训 | https://arxiv.org/abs/2204.01691 |
| **Code as Policies(2022)** | LLM 每次写程序调用感知与控制 API | "prescribe precise values (e.g., velocities) to ambiguous descriptions" | — | 就是老板不要的"脑子每步写程序" | https://arxiv.org/abs/2209.07753 |
| **工业"技能"(Pedersen 2016)** | "a relatively small set of skills are derived from current factory worker instructions",技能自带前后置条件("self-asserting") | 工人任务级编程 | 已在产线部署 | 技能仍是手工写 | https://vbn.aau.dk/en/publications/robot-skills-for-manufacturing-from-concept-to-industrial-deploym/ |
| **遥编程(Funda, Lindsay, Paul 1992)** | 操作员在仿真里动,软件实时生成"a sequence of elementary robot instructions",远端延迟执行 | 时延下必须发符号指令而非连续流 | 空间/水下 | 依赖仿真与真实一致 | https://direct.mit.edu/pvar/article/1/1/29/58758/ |
| **火星车驾驶** | 三种模式:blind drive(逐条 "drive 2 metres forward, turn 90 degrees right")、visual odometry(每米停一次拍地面)、AutoNav("A general direction and goal is given");毅力号五种含 "Guarded" 模式("consults the onboard map to autonomously determine whether the planned step is safe") | 单向时延 → 一天一批高层目标 | 现役 | 保守、慢 | https://robotics.jpl.nasa.gov/what-we-do/flight-projects/mars-exploration-rovers/surface-navigation/ ; https://www.science.org/doi/10.1126/scirobotics.adi3099 |

### 2.2 人类运动控制(运动基元 / 协同 / 小脑与皮层 / 婴儿)

| 发现 | 数 | 出处 |
|---|---|---|
| 脊髓"力场基元"线性叠加(青蛙):两点同时刺激 = 两个力场的向量和,"co-stimulation fields and summation fields were found to be equivalent in 83% of the cases" | 少量基元 + 线性组合 | https://pubmed.ncbi.nlm.nih.gov/8052615/ (Mussa-Ivaldi, Giszter, Bizzi 1994 PNAS) |
| 肌肉协同:青蛙踢腿各方向 = **3 个**时变肌肉协同的组合 | 3 | https://www.nature.com/articles/nn1010 (d'Avella, Saltiel, Bizzi 2003) |
| 手的姿势协同:抓 57 种物体、15 个关节角,"the first two components could account for >80% of the variance";高阶成分不是噪声而"provided additional information about the object" | 2 主成分 >80% | https://pubmed.ncbi.nlm.nih.gov/9822764/ (Santello, Flanders, Soechting 1998) |
| 运动皮层刺激 500 ms(猴)得到整套姿势:"stimulation of one site caused the mouth to open and also caused the hand to shape into a grip posture and move to the mouth ... regardless of the direction of movement required to reach the posture";皮层是 "a map of hand positions around the body" | 词汇 = 终态姿势/位置 | https://pubmed.ncbi.nlm.nih.gov/12062029/ (Graziano, Taylor, Moore 2002) |
| 最优反馈控制/最小干预原则:"correcting only those deviations that interfere with task goals" | — | https://www.nature.com/articles/nn963 (Todorov & Jordan 2002) |
| 小脑 = 前向/逆向内部模型("predict the consequences of actions ... overcome time delays") | — | https://www.cell.com/trends/cognitive-sciences/abstract/S1364-6613(98)01221-2 (Wolpert, Miall, Kawato 1998) |
| 婴儿伸手是从自身"intrinsic dynamics"里涌现并逐步适配任务,不是先有词表 | — | https://srcd.onlinelibrary.wiley.com/doi/abs/10.1111/j.1467-8624.1993.tb04188.x (Thelen 1993) |
| 机器人"身体图式"综述(运动咿呀 → 自模型) | — | https://dl.acm.org/doi/abs/10.1109/TAMD.2010.2086454 (Hoffmann et al. 2010) |
| 双手分工(Guiard 1987):两手"as if assembled in series, forming a kinematic chain",非惯用手定参考系、惯用手做精细 | — | https://pubmed.ncbi.nlm.nih.gov/15136274/ |
| 手内操作分类(Elliott & Connolly 1984):三类(simple synergies / reciprocal synergies / sequential patterns),13 种模式(pinch、dynamic tripod、squeeze、twiddle、rock、roll…) | 13 | https://onlinelibrary.wiley.com/doi/abs/10.1111/j.1469-8749.1984.tb04445.x |
| 抓取分类:Iberall "opposition space" = 三种对置(pad/palm/side)+ "virtual fingers";Feix 等 GRASP 分类:33 种抓法,按 "i) opposition type, ii) the virtual finger assignments, iii) type in terms of power, precision or intermediate grasp and iv) the position of the thumb" | 3 对置 / 33 抓法 | https://www.ijcai.org/Proceedings/87-2/Papers/114.pdf ; https://www.csc.kth.se/grasp/taxonomyGRASP.pdf ; https://dblp.org/rec/journals/thms/Feix0SDK16.html |
| **舞蹈记谱(Laban)**:Effort 四因子各两极——Weight(strong/light)、Time(sudden/sustained)、Space(direct/indirect)、Flow(bound/free),组合成 8 种 basic efforts;Labanotation 方向 27 个(9 水平方向 × 3 高度),**"The length of a symbol defines the duration of the movement"** | 4×2 方式;27 方向 | https://exhibitions.lib.umd.edu/bartenieff/laban-theory ; https://en.wikipedia.org/wiki/Labanotation |

### 2.3 动物行为学

| 发现 | 出处 |
|---|---|
| MoSeq(Wiltschko 2015):小鼠 3D 姿态 "structured at the sub-second timescale",行为 = "a series of reused and stereotyped modules with defined transition probabilities";2024 协议:音节 "typically numbering 40-100" | https://www.sciencedirect.com/science/article/pii/S0896627315010375 ; https://pubmed.ncbi.nlm.nih.gov/38926589/ |
| 果蝇(Berman 2014):"over 100 distinguishable, stereotyped behavioural states",刻板行为占约 50% 时间 | https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4233753/ |

### 2.4 飞机 / 无人机

| 体系 | 事实 | 出处 |
|---|---|---|
| ArduPilot Copter 飞行模式 | "Copter has 25 flight built-in flight modes, 10 of which are regularly used";每个模式一句话(Stabilize "Self-levels the roll and pitch axis";Loiter "Holds altitude and position";RTL "Returns above takeoff location";Guided "Navigates to single points commanded by GCS";Brake "Brings copter to an immediate stop"…) | https://ardupilot.org/copter/docs/flight-modes.html |
| MAVLink 任务命令 | 三类:NAV_*(WAYPOINT/TAKEOFF/LOITER_UNLIM/LOITER_TURNS/LOITER_TIME/RETURN_TO_LAUNCH/LAND/SPLINE_WAYPOINT/DELAY/PAYLOAD_PLACE)"control the movement of the vehicle";DO_*(CHANGE_SPEED/SET_SERVO/SET_ROI/GRIPPER/WINCH…)"do not affect the vehicle's position";CONDITION_*(DELAY/DISTANCE/YAW)"delay DO commands until some condition is met";**"at most one 'Navigation' command and one 'Do' or 'Condition' command can be running at one time"**;位置永远放 param5/6/7 | https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html ; https://mavlink.io/en/services/mission.html |
| 模式混淆(Sarter & Woods 1995) | "when designers proliferate modes without supporting these new cognitive demands, new mode-related error forms and failure paths can result";"automation surprises" | https://journals.sagepub.com/doi/10.1518/001872095779049516 |
| Asiana 214(NTSB 2014) | "complexities of the autothrottle and autopilot flight director systems ... increased the likelihood of mode error";"FLCH trap":选 FLCH 后自动油门进 HOLD,"none of the flight crew noticed" | https://www.ntsb.gov/investigations/pages/DCA13MA120.aspx ; https://www.flightglobal.com/ntsb-cites-autothrottle-system-crew-failures-in-asiana-214-crash/113587.article |
| Sheridan & Verplank 1978 十级自动化 | 从"计算机不提供任何帮助"到"计算机决定一切并自主行动、无视人";第 2 级 "the computer offers a complete set of action alternatives"(**逐字表未从一手源核到**,多次 403) | https://www.researchgate.net/publication/23882567_Human_and_Computer_Control_of_Undersea_Teleoperators |

### 2.5 手术机器人主从接口

| 事实 | 出处 |
|---|---|
| da Vinci:主手 "read the position, orientation, and grip commands from the surgeon's finger tips";震颤滤波 6 Hz;运动缩放(例 5:1);**clutch**(离合:松开映射、重摆手位) | https://www.sciencedirect.com/topics/engineering/vinci-system ; https://www.laparoscopyhospital.com/Book/Ch-03.pdf |
| CRTK 见 §2.1 | |
| "手术语言"JIGSAWS:15 个手势(G1 "Reaching for needle with right hand"、G2 "Positioning needle"、G3 "Pushing needle through tissue" … G15 "Pulling suture with both hands"),覆盖缝合/穿针/打结三任务 | https://arxiv.org/pdf/2102.00027 |

### 2.6 假肢 / 脑机接口

| 事实 | 出处 |
|---|---|
| DEKA 多功能手家用抓型统计:"power (55% of time), pinch open, and lateral pinch" 三种为主;测试环境 power 仅 23% | https://pubmed.ncbi.nlm.nih.gov/28914583/ |
| bebionic:14 种抓法,用户可编程 8 种(4+4) | https://www.progoandp.com/wp-content/uploads/2018/02/14112_bebionic_user_guide_lo-2.pdf |
| 多抓型 vs 单抓型对照(JNER 2023):"MHPs did not show relevant differences in outcomes compared to SHPs on any of the ICF-categories";单抓型在噪声、握力、耐久、穿衣、控制费力 5 项更好 | https://pmc.ncbi.nlm.nih.gov/articles/PMC9930076/ |
| BrainGate 皮层内 BCI:同一群神经元同时解码"连续(光标速度)+ 离散(点击)",Kalman 滤波 + 线性分类器 | https://pubmed.ncbi.nlm.nih.gov/21278024/ |

### 2.7 游戏引擎 / 游戏 AI 的输入抽象

| 事实 | 出处 |
|---|---|
| Unity Input System:动作(Action)与设备解耦,"when you use Actions in your code, you do not need to refer to specific devices because the binding defines which device's controls are used" | https://docs.unity3d.com/Packages/com.unity.inputsystem@1.14/manual/Actions.html |
| Steam/SteamVR Input:"abstracts away the device specific parts of your code so you can focus on the intent of the user - their actions" | https://valvesoftware.github.io/steamvr_unity_plugin/articles/SteamVR-Input.html |
| Genie(DeepMind 2024):从无标注视频学出的"潜在动作"码本 **只有 8 个**,足以玩 2D 平台游戏 | https://huggingface.co/blog/vladbogo/genie-generative-interactive-environments |
| VPT(Minecraft):动作 = 全部按键(W/S/A/D/跳/背包/潜行/冲刺…)+ 鼠标位移分桶 | https://arxiv.org/pdf/2206.11795 |
| PySC2(星际):约 300 个"函数动作",13 种参数类型(屏幕坐标、选择等),"C-style function call" | https://github.com/google-deepmind/pysc2/blob/master/docs/environment.md |
| Crawford:互动叙事需要 "thousands of verbs"(反例:开放叙事要的是大词表) | https://www.erasmatazz.com/library/course-description-2018/actual-verbs.html |

### 2.8 电脑操作 agent 的动作集

| 体系 | 动作集 | 定位方式 | 出处 |
|---|---|---|---|
| Anthropic computer use | 17(见 §1.1) | 截图像素坐标;zoom 只看不改坐标 | 见 §1.1 |
| Anthropic browser toolset | ~33 成员 | 坐标 **或** 元素引用(read_page/find 返回引用) | 见 §1.1 |
| OpenAI CUA | click, double_click, drag, move, scroll, keypress, type, wait, screenshot(9) | 像素坐标;"return screenshots after a short group of actions" | https://developers.openai.com/api/docs/guides/tools-computer-use |
| Google Gemini 2.5 Computer Use | open_web_browser, wait_5_seconds, go_back, go_forward, search, navigate, click_at, hover_at, type_text_at, key_combination, scroll_document, scroll_at, drag_and_drop(13) | **归一化 0–999 网格坐标**;"not yet optimized for desktop OS-level control";购买等动作需用户确认 | https://ai.google.dev/gemini-api/docs/computer-use ; https://blog.google/technology/google-deepmind/gemini-computer-use-model/ |
| WebArena | click/hover/type/press/scroll/tab_focus/new_tab/tab_close/go_back/go_forward/goto/noop/**stop(answer)**(13) | 无障碍树 + 元素 ID(每个元素前缀一个编号) | https://arxiv.org/abs/2307.13854 |
| ALFRED(家务仿真) | 5 导航(MoveAhead/RotateRight/RotateLeft/LookUp/LookDown)+ 7 交互(Pickup/Put/Open/Close/ToggleOn/ToggleOff/Slice)= 12;交互目标用像素掩码,82 类物体 | 像素掩码 | https://arxiv.org/abs/1912.01734 |

### 2.9 大模型公司做机器人"高层接口"的公开设计

| 体系 | 接口形状 / 字段 | 限制 | 出处 |
|---|---|---|---|
| **Gemini Robotics-ER(API 文档)** | 输出 JSON 点列表 `{"point":[y,x],"label":...}`,"The points are in [y, x] format normalized to 0-1000";还有框、轨迹(点序列)、任务编排、成功检测;"You can pass this output directly to a robotics API or a VLA model" | 不直接控;"Complex queries, high-resolution inputs or high thinking levels can lead to increased processing times" | https://ai.google.dev/gemini-api/docs/robotics-overview |
| **Gemini Robotics 1.5(论文)** | 动作前"多层次"自然语言思考:任务级("pick up the yellow tennis ball" → "put ... in the white bag")到运动级("move gripper to the left so that it is closer to the clothes");"two-step decomposition proves more robust than a single, end-to-end translation";Motion Transfer 跨机体 | "its dexterity remains on par with the previous generation";机体差距大时迁移变弱 | https://arxiv.org/abs/2510.03342 |
| **π0.5(Physical Intelligence)** | 两段推理:先出高层子任务文本("pick up the pillow"),再出低层动作 | 子任务是自由文本 | https://arxiv.org/html/2504.16054v1 |
| **Figure Helix** | S2(7B VLM, 7–9 Hz)→ **潜向量** → S1(80M, 200 Hz) | 接口是不可读的向量 | https://www.figure.ai/news/helix |
| **NVIDIA GR00T N1** | System 2 VLM → System 1 扩散 Transformer 120 Hz | 同上 | https://arxiv.org/abs/2503.14734 |
| **RT-H(Google 2024)** | 中间层"language motions"("move arm forward"/"rotate arm right"/"close gripper"),从 9 维动作(3 位移+3 转动+2 底盘+1 夹爪)按"小动作阈值"自动打标,"over 2500 language motions";RT-H 比 RT-2 高 15%,可被人用语言纠正 | 作者自认不确定 "the best abstraction level for the intermediate layers" | https://arxiv.org/abs/2403.01823 |
| **ECoT(2024)** | 动作前生成:计划、子任务、**移动基元**、物体框、夹爪位置;OpenVLA +28% | 文本推理慢 | https://arxiv.org/abs/2407.08693 |
| **MOKA(2024)** | VLM 从标注图选:grasp 点、function 点(在手物体上将接触的部位)、target 点、pre-contact / post-contact 路点;朝向只从 6 个离散选项选(forward/backward/upside/downside/left/right);图上 "5×5 grid tiles, marked as columns a-e (left to right) and rows 1-5 (bottom to top)";每物体 K+1 个编号候选点 | "Current VLMs are not capable of reliably predicting 6-DoF motions";只做 4-DoF 桌面;无力控;API 延迟不适合动态任务 | https://arxiv.org/abs/2403.03174 |
| **PIVOT(2024)** | 图上画编号箭头/圆圈,VLM 选最好的几个,拟合新分布再画,迭代 | "VLMs only take 2D images";"greedy, myopic behaviors";遮挡;"current performance is far from perfect" | https://arxiv.org/abs/2402.07872 |
| **Set-of-Mark(2023)** | 分割后叠编号/掩码/框,把"回归坐标"变成"选编号";RefCOCOg 零样本 75.6 mIoU vs 微调模型 67.2 | 标记冲突、"The centered marks are not always the best"、图中原有数字干扰 | https://arxiv.org/abs/2310.11441 |
| **CoPa(2024)** | 部件建模为"向量"(细长件)或"面";VLM 输出约束句:"Vector A and Vector B are on the same line, with the opposite direction"、"Point A is xx cm above the table surface" | "constrained by its reliance on simplistic geometric elements such as surfaces and vectors";VLM "lack a genuine grounding in the 3D physical world" | https://arxiv.org/abs/2403.08248 |
| **ReKep(2024)** | 任务 = 阶段序列,每阶段"sub-goal constraints"+"path constraints",都是 VLM 写的 Python 函数(3D 关键点 → 代价);68.6% vs VoxPoser 10% | "relies on a forward model based on rigidity assumption";依赖点跟踪;"assumes fixed stage sequences" | https://arxiv.org/abs/2409.01652 |
| **HAMSTER(2025)** | VLM 画一条 2D 路径,低层策略跟随 | 只有路径,无力/速度 | https://arxiv.org/abs/2502.05485 |
| **UniAct(CVPR 2025)** | 学出的"通用动作码本"256×128,机体专用解码器 | 学出来的,不可读 | https://arxiv.org/abs/2501.10105 |
| **Language Movement Primitives(2026-02)** | VLM 直接设 DMP(动态运动基元)的少量可解释参数;31 个真实任务 65% vs 最好基线 35% | 参数是数字 | https://arxiv.org/abs/2602.02839 |
| **AVP(2026-05)** | VLM 先出"视觉基元 token"再由 flow 动作头执行;比 π0.5 +37.04% | 基元不可读 | https://arxiv.org/abs/2605.22183 |
| **PrimitiveVLA(2026-05)** | 示教自动拆成可复用基元,VLM 规划器排序,LLM 生成切换模块 | 未给基元数 | https://arxiv.org/abs/2605.28634 |
| **《10 Open Challenges for VLA》(2025-11)** | 指出方向:"universal atomic actions via a codebook" + 机体专用解码器 | — | https://arxiv.org/html/2511.05936 |

---

## 3. 理论:关于"最小完备动作集"的定理与经典论证

| 结果 | 内容(原文) | 对我们的含义 | 出处 |
|---|---|---|---|
| 刚体运动 = 螺旋(Chasles 定理) | 任意刚体位移 = 绕一条轴转 + 沿该轴平移 | "沿/绕一条线动"对刚体是完备的;TFF 的 6 维分解由此来 | https://en.wikipedia.org/wiki/Chasles%27_theorem_(kinematics) |
| Goldberg 1993 | "Theorem 2 For any polygonal part, we can always find a plan to orient the part up to symmetry";"Theorem 3 ... the algorithm runs in time O(n² log n)";"Since the gripper actions do not require feedback, this result implies that any polygonal part can be oriented without sensors" | **一个动作词**(squeeze)+ 序列 = 一整类任务完备 | https://goldberg.berkeley.edu/pubs/algo93.pdf |
| Lynch & Mason 1996 | "Pushing imposes a set of nonholonomic velocity constraints on the motion of the object, and we study the issues of local and global controllability during pushing with point contact or stable line contact" | "推"这一个词在平面可控 | https://publications.ri.cmu.edu/stable-pushing-mechanics-controllability-and-planning/ |
| Erdmann & Mason 1988 | 托盘倾斜:无传感器序列可把某些零件朝向完全确定,"mechanical decisions in place of environmental inquiries" | 机械本身可当"传感器" | https://ieeexplore.ieee.org/document/800/ |
| LMT 1984 / Canny 1989 | LMT 给出"criteria for correctness of compliant-motion strategies";Canny:LMT 框架下计划"computable",但一般情形"double exponential in the number of plant steps" | 全局精细规划不能放在身体层;每句话必须局部可核 | https://journals.sagepub.com/doi/10.1177/027836498400300101 ; https://ieeexplore.ieee.org/document/99986/ |
| Frazzoli, Dahleh, Feron 2005(Maneuver Automaton) | 基元 = trim(定常)+ maneuver(有限时间过渡);拼接规则是"a regular language, defined through a finite-state machine";"Theorem 5.1 (MA Controllability)";"Theorem 5.2: The reachable set has a nonempty interior" | **有限词库 + 正则语法**可以可控且可达——对有对称性(平移/绕竖轴旋转不变)的系统 | https://web.mit.edu/dahleh/www/pubs/2005.1.pdf |
| Kress-Gazit, Fainekos, Pappas 2009 | 任务用 LTL 片段写,"automatically generate a hybrid controller that guarantees that the robot can achieve its task" | "until"就是 LTL 的 Until;有限事件字母表的 Until 监视器是有限自动机 → 常数内存 | https://dl.acm.org/doi/10.1109/TRO.2009.2030225 ; https://en.wikipedia.org/wiki/Linear_temporal_logic |
| Options 框架(Sutton, Precup, Singh 1999) | 时间延展动作 = (起始集, 策略, 终止条件) | 与 MP 的 λ 同构 | https://www.sciencedirect.com/science/article/pii/S0004370299000521 |
| Wörgötter et al. 2013 操作本体 | 以"touching / untouching"为时间锚,"We find less than 30 fundamental manipulations";"There are only three fundamentally different manipulation types ... Hand-only-actions / Separation actions / Release determined actions";"only six manipulation goal categories";**"It is not possible to arrive at rigorous proof of this claim"**;只处理单手 | 单手操作的"字母表"可以很小,且以接触事件为界 | https://iis.uibk.ac.at/public/papers/Worgotter-2013-TAMD.pdf |
| Bullock, Ma, Dollar 2013 | 5 个二值描述(contact / prehensile / motion / within-hand motion / motion-at-contact)→ 15 个互斥类 | 用二值问题枚举操作类型 | https://pubmed.ncbi.nlm.nih.gov/24808298/ |
| Feix 2016 / Iberall 1987 / Santello 1998 | 33 抓法由 4 个离散判据生成;3 种对置 + 虚拟手指;2 个主成分 >80% | 抓取的"词"可以是对置方向 + 虚拟手指数,不需要具体手指 | 见 §2.2 |
| Guiard 1987 | 两手 = 串联运动链,非惯用手定参考系 | 多手 = "一个相对另一个" | 见 §2.2 |
| Laban Effort | 4 因子 × 2 极 | 力度/速度/路径/顺从性各一个二值槽即可表达"方式" | 见 §2.2 |

---

## 4. 我们现在这套词表最像谁,它们后来怎么演化

| 我们的词 | 最像 | 那家后来加了什么、为什么 |
|---|---|---|
| 编号网格 + 编号框,脑子选编号 | Set-of-Mark → PIVOT / MOKA(MOKA 就是 a–e × 1–5 网格 + 每物体编号候选点) | SoM 发现"选编号"比"回归坐标"强;MOKA 发现 VLM 做不了 6-DoF → 朝向只给 6 个离散选项;PIVOT 用"多轮缩小候选"补精度;Anthropic 2026 用"可移动可询问的红叉"把成功率 6%→32% |
| move → 第几格 / 相对第几号 (at/above/below/…) | MOKA 的 target 点 + "same/above" 高度;CoPa/ReKep 的关系约束 | CoPa/ReKep 把"点"升级成"关系"(轴指向、平行、垂直、距离),但都退回了数字("xx cm")或代码 |
| amount = small/medium/large | RT-H 的"small action threshold"分档;Laban 的 Weight/Time 极 | RT-H 靠 >2500 组合覆盖,代价是词表爆;Laban 用 4 个二值槽 |
| `until` steps/contact/resist/slip/settle | 守卫运动谱系:AL/VAL(1970s)→ LMT 1984 → MP 的 λ → ABB SearchL / G38.2 / MAVLink NAV | 工业体系全部只保留"传感器布尔条件"作为终止;MAVLink 另加 CONDITION 类作为"等到再做"的门 |
| grip close/open | ALFRED 的 Pickup/Put;假肢的抓型选择 | 假肢 14 抓型 → 用户只用 3 种;Feix 说明抓型可由 4 个离散判据生成 |
| look(换相机) | Anthropic zoom(不改坐标系)、browser scroll_to | zoom 加进来是因为"看小字";坐标系不变是为防混淆 |
| fast | Laban Time;UR/ABB 速度是数字 | 无一家把"快慢"做成脑子可选的离散槽,除了 Laban |
| done / say | WebArena stop(answer)、ECoT 推理文本 | "stop"是每个 agent 环境都有的;推理文本被 Gemini Robotics 1.5 变成正式机制 |
| 缺:瞄准、力度、节奏、多手、单指 | CoPa 的"向量指向"(瞄准)、TFF/UR selection_vector(力/位每轴)、Guiard(双手)、Elliott-Connolly(手内) | **没有任何公开接口把这几项做成脑子可选的有限词** |

---

## 5. 硬约束逐条对照(评估任何方案时用)

| 约束 | 各领域给的证据 |
|---|---|
| 脑子定量弱、选编号强 | SoM(+8.4 mIoU 零样本超微调模型);Anthropic 训练数像素、后来加 zoom、浏览器改元素引用;Anthropic 2026 直接控 0–5.5% vs 监督预训练策略可完成任务;MOKA 朝向只给 6 选项 |
| 身体层无策略、无机体参数 | PLCopen / Unity / Steam Input 的"动作与设备解耦";UniAct 的"通用码本 + 机体解码器";Gemini Robotics 1.5 的 Motion Transfer(但"less effective when the embodiment gap is substantially larger") |
| 每句话可机械执行、可核对、可进常数内存监视器 | MP 的 λ 是"conjunction or disjunction about measurable values";LTL 片段可综合出带保证的控制器;MAVLink 一次只跑一条 NAV |
| 一句话覆盖任意执行器 | Iberall 的"对置 + 虚拟手指"不依赖手指数;吸盘/磁铁 = 单虚拟手指的"附着";MAVLink 同一套 NAV 词跑多旋翼与固定翼("Multicopter vehicles stop at the specified point. Forward-moving vehicles circle the point") |

---

## 6. 这是我的看法(此节之前全是事实;此节是方案草案)

### 6.1 设计原则(从上面证据推出来的)

1. **一句话 = 目标关系 + 方式 + 终止事件**。这是 TFF/MP/G-code/MAVLink 各自独立收敛到的形状,不是我发明的。
2. **所有"名词"都是编号,包括身体自己**。身体层已经量出"哪些像素是自己";把自己能碰东西的部位(夹爪指、手指腹、吸盘面、无人机起落架、脚)也当成画面里的编号项("我的接触点 #k"),脑子就能说"用 #3 碰 #7"而不说"手指"。这一步同时解决"单指精细动作"和"不许有身体词"。
3. **"方式"用 Laban 的四个二值槽,不用数字**。重/轻 = 力度;急/缓 = 速度;直/绕 = 路径是否绕障;紧/松 = 顺从性(顶到东西时是硬顶还是让)。身体层把每个极映射成自己量出来的量(它知道自己"轻"是多少牛)。四个二值槽 = 16 种组合,比 RT-H 的 2500 组合小三个量级,比"small/medium/large 一个槽"表达力大得多。
4. **终止事件是唯一的"时间"词**。节奏/时序不用数字,用事件链:一句话结束的事件就是下一句开始的时刻;"等到"用 `wait_until` 单独一句(等于 MAVLink 的 CONDITION)。
5. **多手 = 一个 mover 相对于另一个 mover**(Guiard):`move #A relative #B` 已经在我们词表里,只要允许 B 是"正在动的东西"。
6. **每句话只允许一个"运动主体 + 一个辅助动作"**(MAVLink 规则),监视器内存才是常数。

### 6.2 词表草案

```
utterance := movers? attach? aim? wait? look? done? say?
movers    := up to 4 × MOVE(who, rel, how, until)
  who     : item#  (可以是我的接触点#,也可以是手里的东西#)
  rel     : at | above | below | left | right | front | back | away | between(item#, item#)
            + of item#/cell#  (目标也可以是另一个 mover)
  how     : weight ∈ {strong, light}  time ∈ {sudden, sustained}
            space ∈ {direct, indirect}  flow ∈ {bound, free}   (缺省由身体层保持上一次)
  until   : contact | resist | slip | settle | steps(n∈{1,2,3}) | event_of(other mover)
ATTACH(who_contacts: {item#...}, on: item#, mode: hold | release)     -- 取代 grip
  · 两指夹爪 = 两个接触点 hold;五指 = 任意子集;吸盘/磁铁 = 一个接触点 hold
AIM(who: item#, edge: 该物体上编号的端点#, toward: item#/cell#, until: settle)   -- 瞄准
  · 身体层在每个物体上画编号端点(MOKA/CoPa 做法),"让 #5 的端点 b 指向 #9"
WAIT(until: event)      -- 节奏/时序的门(=MAVLink CONDITION)
HOLD(who: item#)        -- 显式"别动"(=stay_put),给多手协同用
LOOK(camera#)  DONE  SAY(text)
```

覆盖对照:
- 瞄准:`AIM`——物体端点编号是身体层从画面上量出来的(细长物两端、面的中心+法线),脑子只选编号。
- 力度:`how.weight`;"用多大劲"变成二值,加上 `until resist` 做闭环。
- 速度:`how.time`。
- 节奏:`until` 链 + `WAIT` + `HOLD`;"敲三下"= 三句 `MOVE ... until contact` + `MOVE away until steps(1)`。
- 多手:两个 `MOVE`,其中一个 `rel ... of` 另一个 mover 或 `until event_of(other)`。
- 单指:`who` = 我的接触点 #k;`ATTACH` 的接触点子集。

### 6.3 它在哪些任务上会不够(诚实列出)

1. **弹道式/动态任务**(投、击、接):需要毫秒级时序和预测,Helix/GR00T 都要 120–200 Hz 内环;脑子按事件说话来不及,只能把整段"甩"交给身体层——而我们规定身体层不许有策略。这是硬矛盾。
2. **定量精度任务**(缝合、插针、拧到指定扭矩):JIGSAWS 手势之下是亚毫米;`strong/light`、`small/medium/large` 表达不了"0.3 mm"。CoPa/ReKep 在这里都退回数字或代码。
3. **连续步态/平衡**:Anthropic 2026 实测"No model successfully stood the robot up ... even once";步态必须是身体层自带的量出来的能力,词表只能说"往哪走"。
4. **多点同时接触的换抓/手内翻转**:TFF 明说 "cannot model all possible contact situations ... when two or more primitive contact situations occur at the same time";我们的 `ATTACH` 一次只描述一个接触集合的开/合。
5. **柔性物体**:ReKep 的 "rigidity assumption" 同样适用于我们的端点编号。
6. **长程空间记忆**:Anthropic 2026 "fail at tasks that require stable spatial memory, self-localization";词表解决不了脑子忘了东西在哪。
7. **词表的"完备性"没有证明,只有归纳**——Wörgötter 自己说 "It is not possible to arrive at rigorous proof of this claim"。能证明的只有:每一句都能被有限自动机监视(LTL Until 片段),以及在 Frazzoli 意义下"有限基元的正则语言可控"——那是对有对称性的运动系统,不是对任意操作任务。

### 6.4 没核到的东西(照实说)

- Kröger 2010 章节和 Feix 2016 摘要在付费墙后,MP 定义取自同组 2014 年 arXiv 论文的逐字定义与检索摘要。
- Sheridan & Verplank 十级逐字表连试 5 个源都 403,只核到第 2 级措辞。
- 本地仓库里没找到 `body-layer/` 目录(git 快照写的 `body-layer/constants_ceiling.txt` 在当前工作树不存在),所以"我们现在的词表"完全按你消息里的描述对照,没有对照代码。
