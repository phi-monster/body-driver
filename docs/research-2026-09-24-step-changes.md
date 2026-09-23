# 断层式提升候选:面向"量身体、不学策略"驱动器的调研(2023–2026)

(2026-09-24,只搜不改的调研 agent 的报告原文;作为参考,不是决定)

## 0. 先说结论(对我们三个瓶颈的直接诊断)

1. **11 px 残差不是噪声,是系统误差。** Kalib 的敏感性分析:跟踪点噪声 σ≤10 px 时,标定平移误差仍 <1 cm(前提是位姿分布好)。拟合后残差 11 px 说明有未建模的系统项——最常见三种:图像与关节读数不同步(几十 ms 的时间差在运动中直接变成像素偏差)、内参错、位姿集合病态(Kalib 明确列出"机械臂主要沿直线运动"是失败模式)。修法是量出时间偏移、按信息增益挑标定位姿,不是加更多帧。
2. **双相机深度病态的根源是基线/景深比太小。** 解法不是更好的算法,而是让手腕相机自己动出一条你选定的基线——标定后的手腕相机在两个位姿上就是一对基线可控的标定立体相机。MG-Grasp(2026)证明:已知内外参 + 稀疏 RGB 视图 + 三角化定尺度,得到的度量点云足以支撑 87.5% 的真实杂乱桌面抓取。
3. **在 Isaac Sim 里(开发阶段)根本不该标定。** RoboDojo 的观测配置就有 `intrinsic_matrix` / `extrinsic_matrix` / `depth` / `approximate_depth` 开关;Isaac Sim Camera API 直接给 `get_intrinsics_matrix()` 和 `get_world_pose()`。(官方评测不给这些 —— 见 LAB 2026-09-24;开发时可用它们当尺子。)真机上外参也有 CAD/URDF 标称值可作先验,但要警惕:CtRNet-X 在 DROID 数据集上用数据集自带外参渲染机器人掩膜,IoU 只有 0.0186——真机"提供的外参"经常是垃圾,仿真里的是精确的。

口径:RoboDojo 论文写的是 42 个仿真任务 + 18 个真机任务(布局目录里的 55 = 42 + 13 个 random 变体);`collect_freq: 25`、`dt: 0.004`,若评测步率也是 25 Hz,200 步只有 8 秒墙钟——任何秒级感知(如 MASt3R 每次 2–3 s)必须异步跑。

---

## 1. 相机–机器人 / 手眼标定(从自身图像出发)

| 方法 | 年份 | 需要什么 | 精度(真机) | 代价 | 是否符合"量不学" |
|---|---|---|---|---|---|
| 经典 AX=XB(棋盘/AprilTag) | — | 标定板 | Tsai 约 4.4 mm、Daniilidis 约 3.4 mm 定位误差(PLOS One 2022);EasyHeC++ 论文里 marker-based 基线 2.0 cm / 0.87° | 低 | 是,但要标定板 |
| **Kalib**(2024) | 2024 | 无标记、无网格、无训练;用 SpatialTracker 跟踪一个法兰/指尖参考点 + FK + SQPnP | 仿真 0.3–0.6 cm / 0.01 rad;10–20 帧即可;跟踪 55 FPS;失败模式:直线运动、参考点出画、运动模糊 | 低–中(需一个点跟踪模型) | 基本是(点跟踪器是学习型感知,但任务无关) |
| **ARC-Calib**(2025) | 2025 | 无标记、无网格、无预训练模型;单关节旋转产生椭圆轨迹,用光流跟踪角点,用旋转轴共线 + 平面共面约束做凸优化 | 仿真 25 次运动后 0.0042 rad / 6.5 mm;真机 6 组相机、平均 26.5 次运动收敛,掩膜 IoU 0.94(AprilTag 基线 0.84) | 低(纯几何) | **完全符合**,"用身体自己的运动生成标定图案"就是我们的哲学 |
| EasyHeC / EasyHeC++(2023/2024) | 2024 | **需要机器人网格**,SAM/GroundedSAM 掩膜,可微渲染 | 真机 0.3 cm(eye-to-hand)/0.31 cm(eye-in-hand);新臂 15 min、复标 5 min | 中 | 否(要网格,不是 body-agnostic) |
| CtRNet-X(2024) | 2024 | 单帧 RGB + 关节角;需按机器人预训练 | DREAM-real 平均 ADD 0.014 m;自采数据单帧 ADD 0.056–0.059 m,批处理 0.022 m | 高(每种身体要训) | 否 |
| Dr. Robot / RoboPEPP(2024/2025) | 2024–25 | 需 URDF、按机器人训练 | 单帧估关节+位姿 | 高 | 否 |
| **NBV 标定位姿选择**(2023) | 2023 | Fisher 信息矩阵挑下一位姿(AX=YB,用圆点板) | 真机加 5 个位姿:重投影 0.717 px、平移 1.18 mm、旋转 0.159°(随机 0.766 px / 1.39 mm) | 低 | **是**——"该往哪动才能把外参量准"的原则,可直接嫁接到无标记方案 |

**要点:** 单帧法精度是厘米级,只适合粗初始化;想到毫米级必须多视角 + 运动。我们的"指尖位置已测量"是一个天然参考点,Kalib 的流程几乎零改动可用;ARC-Calib 的椭圆约束是纯几何,与 Ada 驱动器最搭。先量时间偏移:乒乓系统把每个环节的延迟都当成经验高斯分布单独测。

## 2. 用关节力矩 / 电机电流当触觉与负载传感

**信号本来就在:** Isaac Lab `root_physx_view.get_measured_joint_efforts()`;ARX5 SDK 后台 500 Hz 收发并带力矩限幅;Unitree G1 `LowState.motor_state` 含 `tau_est`,低层 500 Hz;Franka FCI 1 kHz 给 `tau_ext_hat` 和估计外力旋量。

| 用途 | 方法 | 证据 | 精度/效果 | 符合性 |
|---|---|---|---|---|
| 抓取验证 | 夹爪停在指令宽度之前 = 有物体(Robotiq gOBJ;Franka `grasp()` 用 ε_inner/ε_outer,默认 5 mm) | 工业标配 | 零成本,可靠 | **是**,夹爪通道已有读数 |
| 电流→法向力 | 线性标定 | Current as Touch(2026):Dex3 RMSE 10.09 g、R²=0.99;LEAP RMSE 17.75 g、R²=0.95 | 电流本身就是力的好代理 | 线性标定部分完全符合;开机量一次即可 |
| 接触/碰撞检测 | 广义动量观测器(De Luca / Haddadin) | 不需要加速度信号;需要动力学模型或**开机实测的力矩基线图**(无负载走遍工作空间,记录 τ(q, q̇),外力矩 = 读数 − 基线) | 电流型工业臂上模型法 GMO 残差 3.9–9.6% 负载能力(Denso VS060,ICRA 2024);推理 0.77 ms | 基线图法是纯测量,**完全符合**;精度在电流型臂上是"几牛到十几牛",够做"碰到了就停" |
| 人形 | MOB-Net:只用关节编码器 + 骨盆 IMU 估全身外力矩 | IJRR 2025 | — | 基础 MOB 部分符合 |
| 滑移 | 电流/宽度在抬起时的下降 | 工程做法 | 中等可靠 | 是 |

**具体收益:** "reach until contact"把视觉精度要求从毫米放宽到厘米——接触由力矩触发终止,而不是由视觉判定到位。这直接对冲第 1 节的标定误差。

## 3. 视觉伺服与主动感知(够到移动目标)

- **经典 IBVS/PBVS**(Chaumette & Hutchinson 2006):无学习。目标匀速以外的运动会带来跟踪滞后,需要目标速度前馈或预测。
- **训练无关的 ViT-VS**(IROS 2025):DINOv2-S 特征做对应 + 经典 IBVS;无扰动收敛 100%,末端误差 18.6 mm / 1.5°;扰动下 76.6%;但只有 5–10 Hz。
- **最关键的一条:相对测量而非绝对测量。** 在同一张图里把"实测的指尖像素"伺服到"目标像素",外参误差变成二阶项;两台相机同时做 = 三维对齐而不需要深度。"手腕相机抖导致指尖掩膜是垃圾"——用点跟踪(SpatialTrackerV2、CoTracker3)或带记忆/遮挡头的 SAM2(30–44 FPS)替代逐帧分割,是现成解法。
- **移动目标:** KARL(2025)在感知与控制之间放卡尔曼层;多旋翼 IBVS 拦截(2024)用比例导引补偿延迟。做法一致:预测到绝对未来时刻,而不是伺服当前观测。
- **主动感知:** 几何准则——选让目标–指尖三角化视差角最大、且不遮挡的手腕位姿,和第 1 节的 Fisher 信息准则同源。
- **事件相机伺服**:1–2 kHz 环路、亚 3 ms 延迟——硬件换代,留给"死斗"阶段。

## 4. 快速动态操作——需要什么环路率与延迟模型

| 系统 | 感知 | 控制 | 延迟 | 结果 |
|---|---|---|---|---|
| DeepMind 乒乓(2023/2024) | 125 fps,相机延迟 838 μs,球感知 40±8.2 ms | 策略 100 Hz,ABB EGM 248 Hz | 观测 29–33 ms,动作 64.5–71 ms,各环节按高斯分布测出并在仿真中复现 | 延迟错配 50% 就从 1.83 掉到 1.33 |
| ETH ANYmal 羽毛球(2025) | 立体相机 60 Hz | 策略 100 Hz,状态估计 400 Hz | 对手击球到首个挥拍指令 0.375 s | 瓶颈:视野、相机误差、执行器速度 |
| EV-Catcher(2023) | 事件相机 | Jetson NX | 端到端 <1 ms | 13 m/s 球 81%,落点误差 1.9 cm |
| 双臂杂耍 AthenaZero(2026) | ToF+RGB 30 Hz | 1 kHz | 感知约 0.1 s,抛物线拟合预测到绝对未来时刻 | — |
| Figure Helix(2025) | — | S1 200 Hz 输出腕位姿 + 手指,S2 7–9 Hz | 训练时人为加入与部署一致的时间偏移 | 闭源 |

**可移植的原则(不需要学习):** (a) 每个传感器帧和指令都打时戳,开机实测"指令→读数"阶跃延迟与"图像→关节"偏移;(b) 目标状态用简单动力学(匀速/抛物线)外推到执行时刻;(c) 慢脑快体两层——我们已有,缺的是 100 Hz 级别的身体环路和延迟测量。

## 5. 仅 RGB 的抓取候选生成:学习型 vs 几何型

| 方法 | 输入 | 学习? | 杂乱场景成功率 | 备注 |
|---|---|---|---|---|
| ten Pas & Platt 2015 | 点云 | **否**(几何必要条件) | 单物 88%,杂乱 73% | 纯几何天花板参考 |
| GPD 2017 | 点云 | 候选几何、打分 CNN | 杂乱 93% | 差距≈20 个点来自学习打分 |
| AnyGrasp 2023 | 深度 | 是 | 93.3% | 学习 |
| GraspGen 2025 | 点云 | 扩散 | 真机杂乱 81.3% | 学习 |
| **MG-Grasp 2026** | 4–5 张 RGB + 已知内外参 | MASt3R 对应(学习)+ 三角化定尺度(几何)+ 抓取网络(学习) | 真机 87.5%(35/40) | 每次 2.1–2.9 s;反光/近球形物体不可靠 |

**符合"量不学"的抓取链:** 手腕运动立体 → 点云 + 法向 → 反向法向对偶点采样 + 摩擦锥/力闭合检查 → 我们的接触集。纯几何在杂乱桌面的实证天花板约 73–75%;差的那 20 个点是学习打分器买来的。

## 6. 人形 / 全身操作栈:谁真的给你"位姿接口"

| 栈 | 接口 | 末端跟踪精度 | 开源 |
|---|---|---|---|
| GR00T-WholeBodyControl | Decoupled WBC:下身 RL + 上身 IK;21-D 策略接口 | HERO 实测 SONIC 末端误差 13.38±1.43 cm | 是 |
| Holosoma | 速度控制 / 全身跟踪预录动作 | — | 是 |
| HOVER | 关键点 / 关节角 / 根部速度 | 真机关键点误差 47.5 mm | 部分 |
| HERO(2026) | **机器人坐标系 6-DoF 末端位姿 + 速度**,50 Hz,cuRobo 约 20 ms 重规划 | 真机 2.44±0.86 cm / 8.22° | 未见代码 |
| Unitree SDK2 | `rt/lowcmd` PD 500 Hz;`rt/arm_sdk`;LowState 有 q/dq/ddq/tau_est + IMU | — | 是;**没有 IK / 笛卡尔接口** |
| Agility Digit / Figure Helix | 末端位姿目标接口 + 小网络 WBC | 未给数字 | 否 |

**结论:** 今天没有一个开源人形栈能给出 <1 cm 的可信末端位姿接口;2–13 cm 的误差必须由驱动器自己的视觉/触觉闭环吸收。

## 7. 顶级实验室里可搬的"非策略"部件

NVIDIA cuRobo(GPU 运动生成 20–50 ms,不是学习)、FoundationPose(6D 跟踪)、FoundationStereo;DeepMind Gemini Robotics-ER 的"VLM 指点"产品形态(输出归一化点/框/轨迹)和我们"脑只指、身体做"同构,可作 Qwen 指点能力的对照;Figure Helix 的"训练时注入实测延迟偏移"原则;其余(π0.5、GO-1、GraspVLA、Redwood)都是端到端策略,不可搬。

## 8. 按"预期收益 / 代价"排名(前 8)

| # | 项 | 预期收益 | 代价 | 证据强度 |
|---|---|---|---|---|
| 1 | **读身体已提供的信号**(开发期内外参当尺子;关节外力矩;夹爪"未到指令宽度即停") | 消灭开发期的标定误差与深度病态;抓取验证零成本;接触触发终止 | 近零 | 强 |
| 2 | **开机实测延迟 + 全链路时戳 + 预测到执行时刻** | 静态精度和移动目标同时受益 | 低 | 强(乒乓:延迟错配 50% 即崩) |
| 3 | **同图相对伺服**:实测指尖点→目标点,双相机同时,点跟踪替代掩膜 | 外参误差降为二阶;不需要深度;修"抖动掩膜" | 低–中 | 中强 |
| 4 | **手腕运动立体**:自选基线的两位姿三角化 → 度量点云 | 修双相机深度病态;为几何抓取供料 | 中 | 强(MG-Grasp 87.5%) |
| 5 | **开机力矩基线图 + 动量观测器 + 电流→力线性标定** | 触觉级接触检测、负载判断 | 中 | 中 |
| 6 | **信息增益选标定位姿 + 纯几何 ARC-Calib 约束** | 标定从"多拍几张"变为"可收敛":1 mm / 0.16° 级 | 中 | 强 |
| 7 | **几何反向法向抓取采样 + 摩擦锥/力闭合** 接到接触集 | 杂乱桌面预计 70–75% | 中 | 中 |
| 8 | **cuRobo 式避碰运动生成** | 杂乱场景少碰倒东西 | 中–高 | 强 |

不进前 8 的:人形位姿接口(现有开源栈 2–13 cm);事件相机(硬件换代);任何学习型抓取网络/VLA。

## 9. 唯一需要 owner 拍板的岔路

"任务无关的学习型感知模型(点跟踪器、SAM2、MASt3R/FoundationStereo 稠密对应)能否作为**测量仪器**使用?"两条路都有实证:纯几何(ARC-Calib、ten Pas 2015)能做标定和 73% 抓取;学习型感知作仪器(Kalib 的 SpatialTracker、MG-Grasp 的 MASt3R)把杂乱桌面推到 87.5%。这一决定影响第 3、4、7 项的整条实现。它们不是策略,不含任务词汇,不随任务增长——但它们是学出来的。

## Sources

标定:https://arxiv.org/html/2409.10441 · https://arxiv.org/html/2410.09293 · https://arxiv.org/abs/2305.01191 · https://arxiv.org/html/2408.10562 · https://arxiv.org/html/2503.14701 · https://ar5iv.labs.arxiv.org/html/2303.06766 · https://github.com/cvlab-columbia/drrobot · https://arxiv.org/abs/2411.17662 · https://journals.plos.org/plosone/article?id=10.1371%2Fjournal.pone.0273261 · https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.sensors.camera/docs/index.html · https://robodojo-benchmark.com/doc/usage/configurations/ · https://arxiv.org/html/2607.04434v1

力矩/电流:https://arxiv.org/html/2607.03529 · https://d-nb.info/1211397823/34 · https://arxiv.org/html/2309.16219 · https://arxiv.org/abs/2402.11221 · https://github.com/isaac-sim/IsaacLab/discussions/1867 · https://blog.robotiq.com/knowledge/how-object-detection-works-on-robotiq-grippers · https://docs.ros.org/en/humble/p/libfranka/generated/classfranka_1_1Gripper.html · https://github.com/real-stanford/arx5-sdk · https://deepwiki.com/unitreerobotics/unitree_sdk2/3-g1-humanoid-robot

视觉伺服:https://faculty.cc.gatech.edu/~seth/ResPages/pdfs/ChaHut06.pdf · https://arxiv.org/html/2503.04545 · https://arxiv.org/pdf/2506.15945 · https://arxiv.org/html/2409.17497v2 · https://docs.ultralytics.com/models/sam-2 · https://arxiv.org/abs/2507.12462 · https://arxiv.org/html/2511.04199

动态操作:https://ar5iv.labs.arxiv.org/html/2309.03315 · https://ethz.ch/en/news-and-events/eth-news/news/2025/08/playing-badminton-against-a-robot.html · https://arxiv.org/abs/2304.07200 · https://arxiv.org/pdf/2409.10319 · https://arxiv.org/html/2608.26800 · https://arxiv.org/html/2607.15129 · https://www.figure.ai/news/helix · https://www.prophesee.ai/2025/02/20/demystifying-event-based-camera-latency/

抓取/几何:https://arxiv.org/html/2603.16270 · https://arxiv.org/abs/1501.03100 · https://arxiv.org/abs/1706.09911 · https://arxiv.org/abs/2212.08333 · https://arxiv.org/pdf/2507.13097 · https://github.com/NVlabs/FoundationStereo · https://arxiv.org/abs/2512.11130 · https://github.com/bytedance-seed/depth-anything-3 · https://arxiv.org/abs/2503.17316 · https://curobo.org/ · https://github.com/NVlabs/FoundationPose

人形/实验室:https://github.com/NVlabs/GR00T-WholeBodyControl · https://arxiv.org/pdf/2606.22174 · https://github.com/amazon-far/holosoma · https://arxiv.org/html/2602.16705 · https://arxiv.org/html/2410.21229 · https://arxiv.org/abs/2602.06341 · https://www.agilityrobotics.com/content/training-a-whole-body-control-foundation-model · https://arxiv.org/pdf/2603.06280 · https://ai.google.dev/gemini-api/docs/robotics-overview · https://www.pi.website/blog/pi05
