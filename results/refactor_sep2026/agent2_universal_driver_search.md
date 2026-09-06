# Free-search agent 2 (2026-09-06): 本质抬高 driver + 机器人公司采用第三方身体层的事实

Mandate: goal + hard constraints only; told to read agent 1's report first and not repeat it. 134 tool uses, 25 min. 事实照录;"看法"一节是 agent 的,不是我们的(§3.8)。

## 结论(agent 原话,人话)
1. "我动一动量出自己"别的行业早成产品且有数学预算:航空的控制效能辨识 + 控制分配(翼面被打掉后在线重量每个舵面还能干什么);对偶控制(每条命令既干活又探测,1960s);持续激励 / Willems 引理给出**最少要多少段数据**才能不用模型预测系统;继电器自整定(1984 起装进几乎每台 PID)。最硬真机证据:参数完全未知的四旋翼被随手一扔,450 ms 内在线拟合 52 个参数,57/57 接住(arXiv 2406.11723)。全都不要机体参数、URDF、权重。
2. 第三方层历史:坐在厂商认证安全控制器**之上**的活着(Mujin、Intrinsic、Skild);坐在中间收费的死了(ForgeOS 2024、H-ROS 2019)。全行业采用只在三种情况:买方逼(VDA 5050、AUTOSAR)、监管逼(OBD-II)、免版税 + 设备自描述(USB HID、IPP、MIDI、MAVLink)。安全功能必须**不自我演化、双通道、PL d Cat 3**;EU 机械法规 2027 起把"带自我演化行为的安全部件"列入必须第三方审的高风险清单 ⇒ 自己量出来的限位不能是安全功能本身,只能被不变的监督器兜住。

## 一、本质提升方向(按抬升幅度)
### 1. 持续激励 + 行为系统理论(DeePC)+ 对偶控制
Willems 引理:输入持续激励(order L+n)⇒ 输入-输出记录本身能预测任何未来轨迹,不需辨识模型;数据长度 T ≥ (m+1)L−1(arXiv 2205.06636, 2202.07930)。DeePC 真机:纳米四旋翼位置控制 "without system identification or state estimation"(PMC9291934);线驱软体臂硬件、零样本迁移(arXiv 2510.08953, 2606.26048)。对偶控制综述 IEE Proc. 2000;带在线实验设计的 MPC(S0959152415000876)。
改我们:table —— 探针不再"每通道 ±δ 来回",而是"直到激励阶数够了为止" ⇒ **探测预算第一次有停机判据**;"别碰 X"是 QP 约束不是策略词。失败边界:线性时不变才严格;接触前后是两个系统;噪声要正则化;每步一个小 QP。

### 2. 航空控制效能辨识 + 控制分配 + INDI
效能矩阵 B(每执行器对加速度的效果)与整机模型分开;INDI 用加速度反馈替掉模型非输入项;控制分配处理冗余、饱和、速率限、故障(Johansen & Fossen Automatica 2013;INDI survey CJA 2025;Smeur JGCD 2015 arXiv 1701.07254)。真机:未知四旋翼单次抛出,RLS 拟 52 参数,450 ms 激励,57/57(arXiv 2406.11723);NASA NF-15B IFCS 在线辨识与故障(NTRS 20100025868, 20080034509);L1 自适应 Learjet 16 种失效全恢复(JGCD 10.2514/1.G001730);PX4 v1.13 起效能矩阵为运行时参数(PR #18776)。
改我们:act 的最小二乘 → 带饱和/速率/优先级的加权最小二乘分配;"被拒减半、不听话禁用、×1.5"全换成**效能重估**(RLS/卡尔曼);探针幅度 = 激励设计。失败边界:INDI 要执行器快于被控动力学且反馈够快(30 fps 相机 + 2 s 脑只能用分配部分);效能错 30% 以上性能退化。

### 3. 把 until 编译成可证监视器;任务 = 漏斗顺序组合
Burridge–Rizzi–Koditschek IJRR 1999(漏斗组合保证收敛);NASA Copilot "C99 code generated is constant in memory and time";R2U2 + Copilot 已集成进飞机(NTRS 20150023544);STL 在线鲁棒度监视 RTAMT(arXiv 2501.18608);机械臂 STL(arXiv 2110.00339);Lang2LTL、LTLCodeGen(BNF 约束 LLM 输出);行为树综述 RAS 2022;BTGenBot-2。
改我们:brain 的 until → STL 小子集,谓词阈值 = 现场量的噪声地板;act 的"一段" → 一个漏斗(域 = 表预测准的区域,底 = 事件为真);fast/ 加监视器 —— **SPARK 真正能拿到东西的层**。对着最贵的病"日志全绿而世界没发生"。失败边界:漏斗域无模型只能用经验域;阈值要量;时序逻辑不擅长连续量。

### 4. MOSAIC(多对局部模型 + 责任信号)
Wolpert & Kawato 1998:每对(前向,逆)模型一个责任信号 = 当前预测准不准;Haruno 2001 模块 "switched almost perfectly"。接触前/后 = 两对模型;"碰上了" = 责任切换,不要力传感器、不要手写"实到少于命令十分之一"阈值。工具并入身体图式:Iriki 1996 猕猴用耙子后感受野延伸到耙子全长。
改我们:table → 一组表(空手/顶住/握着);contact/resist 从责任切换长出来。失败边界:模块数无界;切换滞后抖动;都不准时要 refuse 出口。

### 5. 高自由度量法:目标牙牙学语 + Fisher 信息 + 继电器自整定
Goal babbling 到 50 DOF(Rolf, Steil, Gienger TAMD 2010;Online Goal Babbling honda-ri 1732);Fisher 信息轨迹合成;贝叶斯最优实验设计主动碰表面辨识(arXiv 2605.12084);继电器自整定(Åström & Hägglund 1984);BCI 解码器 3 分钟标定(PMC3638090);外骨骼人在环优化(Zhang Science 2017)。网络同构:DPLPMTUD RFC 8899(探测/退避,有收敛证明)= "幅从小起翻倍到画面动过地板";BBR 量 RTprop/BtlBw(ACM Queue 2016)= 延迟/交付率量法。
改我们:探针调度从"逐通道 ±δ"改成"朝目标走的路上采样",每步选信息量最大的通道;步幅阶梯用 DPLPMTUD 规则。失败边界:要可观测目标空间;Fisher 要参数化;继电器法接触瞬间不适用。

### 6. Simplex / 运行时保障
NASA CR-2015-218702:"Current civil certification processes are based on the idea that the correct behavior of a system must be completely specified and verified prior to operation";Simplex 下 "the safety of the vehicle never depends soley upon the adaptive function";"the current certification process does not allow for this type of reasoning about time-varying levels of assurance"(NTRS 20150005863 §6–7.5)。ISO/IEC TR 5469:2024 三种安全 AI 架构:Redundancy / Supervision / Back-up(Mariani 讲稿 p.17)。CBF 把 ISO 10218 SSM/PFL 写成安全滤波器(arXiv 2606.13203)。HACMS/seL4 红队在飞的直升机上失败(DARPA HACMS;CACM)。
改我们:fast/ = 监视器 + 备份 + 切换;切换判据 = 表残差超地板 / 脑没回话 / 通道被拒;备份 = 冻住 + 不松手 + 沿来路退。失败边界:切换瞬态;监视器限值不能自己量(TR 5469 使用等级 D),起点最弱。

### 7. 模式混淆(mode confusion)
Sarter & Woods 1994;Asiana 214(NTSB:自动油门模式理解不足);737 MAX MCAS 单迎角传感器 → 双传感器差 >5.5° 不激活(FAA 737 RTS Summary)。改我们:每次回脑的 JSON 带"我现在哪个模式、没新话我接下来自己做什么、什么条件下停"。

### 8. 动作合同而非设备合同(佐证删动词表)
Steam Input "The game receives 'actions' rather than raw inputs";Unity Mecanim muscle space 跨骨架重定向;Industrie 4.0 Capabilities–Skills–Services + AAS。

### 9. 自描述是"一个驱动通吃"的通行做法(与我们硬约束相反的输入侧;有价值的是输出侧)
USB HID report descriptor;IPP Everywhere "supported by over 98% of printers";IO-Link IODD(Schmalz 吸盘);OPC UA for Robotics(VDMA 40010)。价值:驱动量完之后用这类格式**发布自己量到的东西**。

## 二、机器人公司采用第三方身体层:事实清单
### A. 标准与它要你证明的
| 标准 | 要点 | 意味 |
|---|---|---|
| ISO 10218-1/-2:2025 | 2025 发布替 2011;TS 15066 PFL 并入;首次含网络安全;功能安全明示;PL d Cat 3 一刀切改为按功能给默认 PL 表或完整风险评估(Robot Report;A3 FAQ;arXiv 2602.17822) | 急停/限速/模式各要 PL + ISO 13849-1 验证文件 |
| ISO 13849-1 PL d Cat 3 | Cat 3 = 单一故障不丢安全功能,两通道 + 诊断;UR 17 项安全功能全 PL d Cat 3,PFHd < 1.8E-07,容差 力 25 N、TCP 速 50 mm/s、停止 50 ms、40 mm、关节 5° | **单 CPU 一份 SPARK 程序不构成 Cat 3**;安全功能 = 硬件架构 + 软件一起认证 |
| IEC 61508 | 形式化方法 SIL 4 HR、SIL 2–3 R(二手);Route 3S 既有软件;compliant item 配 safety manual | 第三方软件进认证系统两条路 |
| ISO 26262 SEooC | "developed based on assumptions",集成方验证假设 | 不知最终机体的安全部件有现成交付形式 |
| IEC 62304 SOUP | 第三方软件 = software of unknown provenance | 医疗厂商把你当 SOUP 管 |
| IEC 80601-2-77:2019 | 手术机器人基本安全 | |
| FDA PCCP 2024-12 | 预批"修改包络",包络内自更新不再提交 | 监管接受"运行中变"的唯一形式:范围事先写死 |
| ISO/IEC TR 5469:2024 | 使用等级 D "Dynamic (online) teaching or learning possible" ⇒ "No specific functional safety requirements for AI technology, but application of risk reduction concepts of existing functional safety standards";技术类 III not recommended | 在线自适应部分**不能承担安全功能** |
| ISO/PAS 8800:2024 | 车辆 AI 安全 | |
| EU 机械法规 2023/1230 | 2027-01-20 全面适用;Annex I A(二手转述):"safety components with fully or partially self-evolving behaviour using machine learning approaches ensuring safety functions" ⇒ 强制公告机构评估;Annex III 1.2.1 声明预期行为/限制/自主程度;1.1.9 防篡改;substantial modification ⇒ 修改者成制造商 | 字面点名 ML;在线自适应非 ML 层不在字面内,但 1.2.1 照样适用 |
| EU AI Act Annex I | NLF 产品安全部件 AI = 高风险;适用期推迟到 2028-08-02 | |
| EU 产品责任指令 2024/2853 | 软件明确为产品;缺陷判断含 self-learning;2026-12-09 起 | 第三方软件层作者进产品责任链 |
| ISO 13482 / UL 3300 / ISO/CD 25785-1 | 13482 FDIS;UL 3300 2025-12-31 进 NRTL;25785-1(主动稳定移动机器人 = 人形/四足)草案,工作组含 Agility、BD、A3 | 人形还没有可引用的安全标准 |
| DO-178C / DO-333 / EASA SC Light-UAS | DO-333 形式化补充 | |
| 医疗自主等级 | Yang 2017 六级(Science Robotics) | |

### B. 接口事实
Franka FCI 1 kHz、PREEMPT_RT、20 连续丢包停机、读写 <500 µs · UR RTDE 500 Hz,安全 PL d Cat 3 TÜV,UR+ 由 UR 测试上架 · KUKA FRI 1–2 ms,RSI 4/12 ms 付费 · ABB EGM 4 ms UDP,选项 689-1 付费 · Fanuc Stream Motion J519 付费 8 ms · Yaskawa MotoROS2 · Unitree G1 SDK2 CycloneDDS 500 Hz LowCmd 每电机 q/dq/kp/kd/tau · BD Spot Joint Control API 特许 license,100–333 Hz,end_time 必填 · Agility Digit 研究口 UDP 2 kHz/1 kHz,商用只给 Arc 云 WMS 级,安全 Cat 1 停 + Safety PLC + FSoE · Figure 无外部接口(Helix S2 7–9 Hz + S1 200 Hz) · GR00T N1 S2 10 Hz + S1 120 Hz,embodiment-specific encoders/decoders · openpi WebSocket 20/50 Hz · Robotiq 2F-85 Modbus RTU;Shadow Hand EtherCAT 1 kHz;Schmalz IO-Link · MAVLink 事实标准(DJI 私有) · ros2_control 从 URDF 解析硬件;MoveIt 要 URDF+SRDF;ROS 2 "yet incomplete to be applicable to hard real-time or safety-critical" · ISO 9409-1 法兰 = 机械层早已通吃。

### C. 他们自己怎么做这一层
Figure 2025-02 退出 OpenAI,大模型 "had become a smaller problem" 相比 "high rate robot control",S1 自研 · Agility 自研 whole-body control foundation model(<1M 参数 LSTM,Digit 专用,"motor cortex";上层给 "dense free-space position and orientation objectives",不给关节命令;"always on safety layer") · BD + DeepMind Gemini 部署 Atlas/Spot,接口/延迟/责任未披露 · DeepMind Gemini Robotics 2 "natively multi-embodiment",On-Device 新双臂 "just a few hours … less than 200 examples",单权重跨 Apollo/Franka 45.7–76.3%(全身)、74.2–89.6%(夹爪)、32–92%(多指) · NVIDIA GR00T 伙伴 1X/Agility/Apptronik/BD/Figure/Fourier/Sanctuary/Unitree/XPENG;2026-06 参考人形(Unitree H2 + Sharpa 手,75 DoF) · Skild AI B2B "omni-bodied" 脑,2026-03 与 ABB、UR/MiR、NVIDIA 合作;接口/认证/责任未披露 · ABB/KUKA/Fanuc 控制器封闭,实时外控付费 · Intuitive 封闭,Restore Robotics 反垄断诉讼 2019–2025;研究口 dVRK。

### D. 抱怨与案例
LeRobot 论文 "Hardware from different manufacturers uses different communication protocols, coordinate systems, joint names, calibration procedures, and safety rules"(arXiv 2602.22818);社区统一 RemotePolicy 桥(Positronic) · Holbrook v. Prodomax(2015 密歇根工人被压死):同时起诉集成商与部件厂 FANUC/Nachi/Lincoln · NASA 报告:自适应认证阻力在 "complete description of desired behavior"、"misunderstandings … 'nondeterministic'",对 L1 类 "there seem to be no actual barriers to their certification" · ROS 2 执行器不支持回调抢占(MPG;RTSS'25 ROSRT)。

### E. 第三方层被采用/拒绝的历史
AUTOSAR(OEM 发起,"Cooperate on standards, compete on implementation")· ARINC 653(航空公司主导)· VDA 5050(汽车厂买方 + VDMA)· OBD-II(CARB 强制)· USB HID / IPP / MIDI(自描述 + 免版税)· MAVLink(开源 + DIU 采购)· OPC UA Robotics / MassRobotics AMR(协会驱动)· Mujin 活着(坐控制器之上)· Intrinsic Flowstate 2026-02 并入 Google(FANUC/UR/KUKA/Comau,ABB 不在)· **READY Robotics ForgeOS 2024-08 关门**(250+ 机型,$42M,"had difficulty persuading robot manufacturers to work with ForgeOS")· **Acutronic H-ROS 2019 关门**("hit the market too early")。

### F. Ada/SPARK 对"公司想不想用"
IEC 61508-3 形式化推荐随 SIL 升;DO-333;NASA:DO-333 "has not been demonstrated for an adaptive system" · NVIDIA 把安全关键固件 C→Ada/SPARK 对齐 ISO 26262(AdaCore case study);seL4/HACMS · **反向:Rust 已可认证 —— Ferrocene 取得 ISO 26262 ASIL D、IEC 61508 SIL 4、IEC 62304 Class C,核心库子集 SIL 2**(ferrocene.dev)· **语言不是认证单位**:认证的是安全功能 + 架构 + 工具链 + 安全手册;PL d Cat 3 要双通道,单 SPARK 二进制单核不满足;TR 5469 等级 D 要求安全由非自适应手段提供。

## 三、agent 的看法(标明)
1. 抬升最大 = 第 1+2 条:把"表 + Broyden + 减半/放大/禁用"整套换成"效能矩阵在线辨识 + 带约束控制分配 + 有停机判据的激励设计";DJ→EA 每个坑在飞控里都是效能重估和分配器约束的标准情形;所有规则变成 QP 约束,堵住策略词源头。
2. 第二 = 第 3 条:until 编译成监视器,"一段"= 漏斗;SPARK 拿得到东西的层;"不全改 Ada"是对的,证监视器和备份,不证估计器。
3. "所有公司想用"历史答案不是技术:活的坐在厂商安全控制器之上且不碰安全功能;产品形状 = SEooC(假设清单 + 安全手册)+ 自描述输出(OPC UA/AAS)+ 走厂商实时口 + 安全留给厂商 + 自己的 Ada 核只做监督器和备份;窄腰放在厂商安全层之上而非替代。
4. 机械法规点名 ML;我们不是 ML 但是在线自我演化;Annex III 1.2.1 + TR 5469 等级 D 都推到同一处:自己量的限位不能当安全限位,起点最弱、量到再放开。
5. 边界:九条没有一条在"刚性 25 DOF 手 + 只有头相机"上证明过;四旋翼 450 ms 靠高频本体感受 ⇒ 只靠 30 fps 相机慢一个量级;解相机判死不需翻案(DeePC/分配都活在"命令 ↔ 画面/读数"空间)。
未核到原文两处(二手):EUR-Lex 2023/1230 附件;IEC 61508-3 附表 A。
