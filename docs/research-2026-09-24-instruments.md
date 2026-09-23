# 仪器层选型:任务无关的学习型感知"量具"(截至 2026-09-24)

(只搜不改的调研 agent 报告;每条许可证结论都给了原文链接;数字分 **实测(论文/README 给的)** 和 **估计(本报告推的,必须自己量)** 两种,估计的都标了"估"。)

## (a) 一句话结论 + 最小完美集

**结论:现在的 6 件清单里有 3 件该换。** 点跟踪从 TAPNext/BootsTAPIR 换成 **Track-On2(DINOv2 版,MIT,52M,自带不确定度头)**;单目度量深度从 DA3-METRIC-LARGE 换成 **MoGe-2(MIT,一次前向同时给焦距 + 米制点图 + 法线 + 掩膜,可喂已知 FoV)**;光流若要保留,选 **SEA-RAFT(S)(BSD-3,输出逐像素方差)** 而不是没有不确定度的 NeuFlow v2。GeoCalib、SAM 2.1、MapAnything(Apache 版)保留。**最小集 = 5 件(第 6 件光流可选)**,权重文件合计约 7 GB,每帧链路在 4090 上估 40–60 ms 关键路径,能塞进 GPU0 剩下的 16 GB。

| # | 模型 | 顶哪些槽 | 代码 / 权重许可证 | 参数 | 权重文件 | VRAM | 延迟 | 不确定度 |
|---|---|---|---|---|---|---|---|---|
| 1 | **GeoCalib**(pinhole 权重) | (1) 焦距 + 重力方向;多帧共享内参;多相机刚性 rig 联合重力 | Apache-2.0 / **CC-BY 4.0** | 未公布(tar 111 MB) | `geocalib-pinhole.tar` 116,074,121 B | 估 <1 GB(320 px) | 论文 ≈100 ms/张 | **有**(LM 收敛处协方差 Σ=H⁻¹) |
| 2 | **MoGe-2**(ViT-B 每帧 / ViT-L 或 **MoGe-3** 开机) | (2) 米制深度 + 点图 + 法线 + 焦距/内参 + 有效掩膜;(9) 物体尺寸 | MIT / MIT | S 35M · B 104M · L 326M;MoGe-3-L 370M | vits 141 MB · vitb 419 MB · vitl 1.31 GB · moge-3-vitl 1.48 GB(SHA256 见 (c)) | ViT-L fp16 估 2–3 GB | 实测 A100 fp16 ViT-L:29 ms@484² / 39 ms@700²;MoGe-3-L 121 ms;ViT-S/B 未公布 | 无(只有有效掩膜)——要靠驱动自己跨帧对账 |
| 3 | **MapAnything**(`facebook/map-anything-apache`) | (3) 图 + 已知位姿/内参 ⇒ 米制点图/深度;也能单目米制;(11) 无位姿时估位姿 | Apache-2.0 / Apache-2.0 | 1B(DINOv2 ViT-G 编码器 + 16 层交替注意力) | `model.safetensors` 4.91 GB F32 | 估 6–8 GB(≤16 视图,518 px,bf16,同级 1B 模型 VGGT-Ω 实测 6.0 GB@1 帧 / 13.4 GB@100 帧) | 未公布(H200 上的曲线只有图);开机异步跑,秒级 | **有**(逐像素置信度 + 有效掩膜);但"不对几何输入的噪声建模" |
| 4 | **Track-On2**(`trackon2_dinov2_checkpoint.pt`) | (4) 在线逐帧点跟踪(指尖、目标、网格 = 稀疏光流);(7) 与已知相机运动一起三角化 ⇒ 物体 3D 点与刚体位姿;(10) 顺带给 DINOv2-S 稠密特征 | MIT / MIT(HF 标)+ DINOv2 Apache | 52.3M(23.6M 可训) | 93.9 MB,SHA256 `34c35ea6…8973` | 实测 A100 FP32:0.64 GB@64 点,<0.5 GB@256 点 | 实测 A100:>35 FPS@64 点,>30 FPS@256 点 | **有**(可见性 + 不确定度头) |
| 5 | **SAM 2.1**(hiera-tiny / small) | (5) 框/点提示 ⇒ 掩膜 + 视频流式传播;自我/世界分割;轮廓 ⇒ 接触几何 | Apache-2.0 / Apache-2.0 | 38.9M / 46M | tiny 156,008,466 B · small 184,416,285 B | 估 2–4 GB(视频模式,未公布;large 在 HD 视频约 11 GB) | 实测 A100:91.2 / 84.8 FPS | **有**(IoU 分数 / 对象分数) |
| 6(可选) | **SEA-RAFT(S)**(或速度优先时 NeuFlow v2) | (6) 稠密光流:抖动补偿、动/静分离 | BSD-3 / BSD-3(NeuFlow v2 Apache) | S 未公布(M 权重 78.8 MB) | `Tartan-C-T-TSKH-spring540x960-M` SHA256 `cb8cfbf1…6102` | 估 <1 GB | 实测 RTX 3090:S 47.5 ms@540×960(640×480 在 4090 上估 15–25 ms);NeuFlow v2 RTX 2080 15 ms@1024×436 | SEA-RAFT **有**(Laplace 混合的 α、β 参数 ⇒ 逐像素方差);NeuFlow **无** |

**预算核对(GPU0 空余 16 GB):** 每帧四件 MoGe-2-B(估 1.5 GB)+ Track-On2(实测 0.64 GB)+ SAM 2.1 small(估 3 GB)+ SEA-RAFT-S(估 1 GB)≈ 6 GB;开机件 MapAnything(估 7 GB)在开机时加载、量完卸掉,峰值 ≈ 13 GB < 16 GB。GeoCalib(估 1 GB)放 GPU1(vLLM 缩到 16 GB 后余 8 GB)或 GPU0 均可。每帧关键路径(并行流):估 40–60 ms;串行估 80–110 ms。**这些估计一个都没量过,V1 第一件事就是量。**

**"一个模型顶几个槽"的答案:** 有两条路。(i) MoGe-2 一次前向同时给焦距 + 米制点图 + 法线 + 掩膜,把"焦距估计"和"单目深度"合成一件,但它没有重力方向、没有不确定度,所以 GeoCalib 仍要留(重力 + 协方差是它独有的);(ii) MapAnything 同时顶单目米制、已知位姿多视角、无位姿时估位姿,理论上能取代 MoGe-2 + DA3 + 大半个 VO,但它是 1B、每帧用不起,所以只做开机件。真正能砍掉的是:光流(用 Track-On2 网格查询代替,先量够不够)、专门的稠密特征模型(复用 Track-On2 冻结的 DINOv2-S)、6-DoF 物体位姿网络(掩膜 + 跟踪 + 已知相机运动 ⇒ 三角化 ⇒ 刚体拟合,零模型)、手眼标定网络(全都要 URDF/CAD,零模型)。

---

## (b) 分槽排名(赢家、亚军、为什么输)

### 槽 1:单图内参(焦距 / 主点)+ 重力 / roll / pitch

| 名次 | 模型 | 许可证 | 输出 | 判决 |
|---|---|---|---|---|
| **1** | GeoCalib(ETH,ECCV 2024) | 代码 Apache-2.0,权重 CC-BY 4.0 [1] | 焦距、重力向量(roll/pitch)、LM 协方差不确定度;2025-03 加 `shared_intrinsics=True` 多帧共享内参 [2];2026-06-28 加 `camera_R_rig` 多相机刚性 rig 联合重力 [3];4 种相机模型(pinhole / simple_radial / radial / simple_divisional) | **赢**:唯一同时给重力 + 不确定度的;主点固定在图像中心(与我们 R2 假设一致) |
| 2 | AnyCalib(ICCV 2025) | 代码 + 权重 Apache-2.0 [4] | 焦距、主点、畸变(12 种相机模型族)、光线场;ViT-L(DINOv2);4090 上 ≈25 ms | 焦距比 GeoCalib 准(Stanford2D3D 角误差 2.55° vs 3.23°,重投影 12.11 px vs 15.16 px)[5],但 **不给重力、不给不确定度**;若 CC-BY 署名不可接受,用它当焦距备胎 |
| 3 | CalibAnyView(2026-05,arXiv 2605.14615) | 无代码(没找到仓库) | N≥1 帧共享内参 + 每帧重力,单帧 roll 1.07° vs GeoCalib 1.60°,FoV 3.64° vs 4.93° [6] | 论文而已 |
| — | MoGe-2 / MoGe-3 的内参输出 | MIT | 与点图一起回归 | MoGe-2 论文不报焦距精度 [7];留作对账 |
| — | DA3 any-view 的内参输出 | Apache(SMALL/BASE) | 无位姿时估内参 + 位姿 | 小模型无位姿时位姿 AUC3 只有 8.6–19 [8],不能当量具 |
| ✗ | WildCamera(NeurIPS 2023) | Apache-2.0 [9] | 焦距 + 主点(4 DoF)、裁剪检测 | 老、Swin-L、无新对比数据;输 |
| ✗ | PerspectiveFields | **Adobe Research License,仅非商用** [10] | roll/pitch/FoV(+cx,cy) | 排除 |
| ✗ | DiffCalib(AAAI 2025) | 仓库注明非商用 [11] | 扩散生成入射图 | 排除;且扩散模型秒级 |

### 槽 2:单张 RGB ⇒ 米制深度 / 点图

| 名次 | 模型 | 许可证 | 给焦距? | 不确定度? | 判决 |
|---|---|---|---|---|---|
| **1** | MoGe-2(CVPR'25 Oral / NeurIPS 2026) | MIT / MIT [12] | **给**(内参 3×3),还可喂已知 `fov_x` | 无(有效掩膜) | 米制点图相对误差 8.19 vs Depth Pro 13.7 vs UniDepth V2 10.1 [7];A100 fp16 39 ms@700²;缺点:细线 / 头发、前后景尺度差大时直线不直 |
| 1′ | MoGe-3(2026-08-18) | MIT | 给 | 无 | 同一族,加 3D 稀疏体素精修,专治 **薄结构 / 小物体**(剪刀!);A100 fp16 121 ms(ViT-L)[13];依赖 FlexGEMM/Triton(Linux 可,macOS 不行);边界仍有飞点 |
| 2 | DA3METRIC-LARGE | Apache-2.0 [14] | 不给,反而 **要**:`metric = f·out/300` [15] | 论文只说训练时有置信度 | 0.35B、1.34 GB;ETH3D 上最好但室内外标准集落后 UniDepthv2 [8];我们有 R2 的焦距,所以"要焦距"不是缺点——但 MoGe-2 更全 |
| 3 | OptiGeo(CoRL 2026,HKU) | 代码 MIT;权重 **DINOv3 ViT-S 派生** [16][17] | 给 FoV | 无 | 30M,fp16 1000 token 32.8 ms / 0.42 GB;透明 / 反光专项:ClearGrasp Real AbsRel 0.019 = MoGe-2,δ 97.9 vs 97.1;TransCG 4.36 vs 4.64 [17];见槽 9 |
| 4 | MetricAnything(ECCV 2026) | 代码 Apache-2.0;student pointmap 权重 Apache(HF 标)= MoGe-2 ViT-L 微调 [18] | 给 | 无 | 326M;没找到对 MoGe-2 的头对头数字;teacher(稀疏度量提示)未放 |
| 5 | Metric3Dv2 | BSD-2 [19] | 要焦距 | **有**(法线置信度) | 2024;精度被 MoGe-2 甩开 |
| 6 | Depth-Anything-V2-Metric-Hypersim-Small | Apache-2.0,24.8M,Hypersim 合成训练 [20] | 无焦距处理 | 无 | 便宜但焦距依赖导致跨相机尺度飘 |
| ✗ | Depth Pro(Apple) | GitHub LICENSE 文本像 Apple Sample Code License,但 HF 标 **apple-amlr = 仅研究用** [21] | 给 | 无 | 许可证矛盾 ⇒ 只能 dev-only;0.3 s/2.25 MP 也慢 |
| ✗ | UniDepthV2 | CC BY-NC 4.0 [22] | 给 | 有 | 排除 |
| ✗ | DA V2 Base/Large、DA3-LARGE/GIANT/NESTED-1.1 | CC-BY-NC 4.0 [14][20] | — | — | 排除 |
| ✗ | Marigold V2 / Pixel-Perfect Depth(2026,扩散) | 未查许可证 | — | — | 扩散模型秒级,不进每帧链路 |

### 槽 3:多视角 + **已知位姿**(手臂给)⇒ 米制重建

| 名次 | 模型 | 许可证 | 吃已知位姿/内参? | 判决 |
|---|---|---|---|---|
| **1** | MapAnything(Apache 版) | 代码 Apache-2.0;`facebook/map-anything-apache` Apache-2.0;`facebook/map-anything` CC-BY-NC [23] | **吃**:`intrinsics`、`camera_poses`(cam2world,OpenCV)、`is_metric_scale`、`depth_z`、`ray_directions`;"任一视图有位姿则第一视图必须有" [24] | 唯一原生把"位姿 + 内参 + 米制标志"当输入、输出带置信度的;Apache 版只用 6 个数据集训练(BlendedMVS、Mapillary、ScanNet++ v2、Spring、TartanAirV2-WB、UnrealStereo4K),论文说仍"与 VGGT 相当",几何输入越多越好(AbsRel 0.08→0.01,内点率 57.5%→82.0%)[25];缺点:1B、518 px、"不对几何输入噪声建模" |
| 2 | DA3-SMALL / DA3-BASE(any-view) | Apache-2.0 [14] | **吃**:`extrinsics (N,4,4)` + `intrinsics (N,3,3)` ⇒ 相机 token;`align_to_input_ext_scale=True` 时用 Umeyama 把深度缩到输入位姿的米制尺度 [26] | 0.08B / 0.12B,A100 504×336 多视图 160 / 126 FPS [8];无位姿时估位姿很弱(Small AUC3 8.6–19),但给了真位姿后大小模型差距缩小 [8];**与 MapAnything 无头对头,要在自己的仿真上 A/B** |
| ✗ | VGGT / VGGT-Ω | VGGT 原权重非商用;`VGGT-1B-Commercial` 需申请(Llama 式审批)[27];VGGT-Ω 权重 FAIR Noncommercial Research License [28] | 不吃位姿 | 排除(不吃位姿 + 门槛许可证) |
| ✗ | π³ / Pi3X | 代码 BSD-3,权重 CC BY-NC 4.0 [29] | Pi3X 吃 | 排除 |
| ✗ | CUT3R、StreamVGGT | CC BY-NC-SA 4.0 [30][31] | — | 排除 |
| ✗ | Fast3R | FAIR NC Research License [32] | — | 排除 |
| ✗ | MASt3R / DUSt3R | CC BY-NC-SA | — | 排除 |
| ✗ | X-Lens(2026-07,0.04B,41 FPS,吃标定多视图) | CC BY-NC 4.0 [33] | 吃内参,位姿可选 | 可惜,排除 |
| — | Rig3R(Wayve,NeurIPS 2025) | 无官方代码(只有社区复现)| 吃 rig 元数据 | 无货 |
| — | DAGE(CVPR 2026)、Mix3R | 未见代码/许可证 | — | 论文 |

### 槽 4:在线 2D / 3D 点跟踪

| 名次 | 模型 | 许可证 | 参数 / 速度 | DAVIS AJ | RoboTAP AJ | 不确定度 | 判决 |
|---|---|---|---|---|---|---|---|
| **1** | Track-On2(DINOv2 版,TPAMI 2026) | MIT(仓库 + HF 标)[34][35] | 52.3M;A100 FP32 >35 FPS@64 点 / >30 FPS@256 点,0.64 GB [36] | 66.8 | 67.1 | **有**(可见性 + 不确定度头) | 只用 Kubric 合成训练;`forward_frame` 逐帧在线,可随时插新查询点;DINOv2 变体在合成 PointOdyssey 上反而比 DINOv3 变体好 [36] |
| 2 | TAPNext++(CVPR 2026 Findings) | Apache-2.0(代码 + ckpt)[37] | 12 层 ViT+SSM(B 号 194M);H100 256×256 193 FPS / 5.18 ms@256 点 [38];PyTorch 版用 torchvision `EncoderBlock`(SDPA),**不依赖 FA3** [39] | 65.6(256)/ 67.0(512) | 61.1(256)/ 66.0(512) | 位置头是 256+256 bin 的 softmax ⇒ 可自己算离散度 [39];可见性 logits | 长期遮挡后 **重检测** 更强(自定义 RD-AJ 指标,Track-On2 没测);ckpt 文件 2.53 GB(见 (f)) |
| 3 | BootsTAPIR online(causal) | Apache-2.0 | Quadro RTX 4000 上 ≈17 FPS@480×480 | 老 | — | 无 | 2024 的东西,被上面两个超 |
| 4 | LocoTrack | Apache-2.0 [40] | 2024 | — | — | 无 | 偏离线;无在线 API |
| ✗ | CoTracker3 | CC-BY-NC(主体)[41] | — | — | — | — | 排除 |
| ✗ | SpatialTrackerV2 | CC BY-NC 4.0 [42] | 在线版仍是 TODO | — | — | — | 排除 |
| ✗ | TAPIP3D | Apache-2.0 [43] | 单目要先跑 MegaSaM+MoGe 出深度,离线 | — | — | — | 管线太重;我们的 3D 从两只腕眼 + 已知位姿三角化 |
| ✗ | DELTA(Snap) | 仓库只写"见 LICENSE.md",未核 | 100 帧要 20–40 GB [44] | — | — | — | 排除 |
| ✗ | MVTracker(ETH) | 没找到许可证 [45] | 要深度或先跑 DUSt3R/VGGT | — | — | — | 排除 |

### 槽 5:可提示分割 + 视频传播

| 名次 | 模型 | 许可证 | 参数 | A100 FPS | SA-V J&F | 判决 |
|---|---|---|---|---|---|---|
| **1** | SAM 2.1 hiera-tiny / small | Apache-2.0 [46] | 38.9M / 46M | 91.2 / 84.8 | test 76.5 / 76.6 | 流式记忆,逐帧 `SAM2VideoPredictor`,输出 IoU 分数;桌面 GPU 上"高效版"并不更快 |
| 2 | EfficientTAM-S(ICCV 2025) | Apache-2.0 [47] | 34M | 85(1024)/ 134(512) | val 74.5 / 71.5(512) | 只在手机上有优势 |
| 3 | EdgeTAM(CVPR 2025) | Apache-2.0 [48] | RepViT-M1 | 150.9(torch.compile) | val 72.3 | 快但掉 4.5 点 |
| — | TinySAM 2(2026-05)、SAM-MT(2026-07) | 未见代码 | — | — | — | 论文 |
| ✗ | SAM 3 / 3.1(2026-03-27) | **SAM License**:允许商用,但禁军事/武器/ITAR、须原样转许可、权重门控 [49] | 848M | — | — | owner 已排除;也太大 |
| — | MobileSAM / SAM-HQ | Apache | — | — | — | 只有图像,无视频记忆 |

### 槽 6:光流

| 名次 | 模型 | 许可证 | 速度 | 不确定度 | 判决 |
|---|---|---|---|---|---|
| **1** | SEA-RAFT(S/M)(ECCV 2024 Oral) | BSD-3 [50] | RTX 3090 540×960:S 47.5 ms、M 70.9 ms;1080p 21 FPS [51] | **有**(Laplace 混合 α、β₂) | TartanAir 合成预训练;Spring EPE 0.363 |
| 2 | NeuFlow v2 | Apache-2.0 [52] | RTX 2080 15 ms@1024×436;Jetson Orin Nano 106 ms;9M 参数 [53] | 无 | 只用 FlyingThings 训练,作者自述 1/1 尺度特征过拟合训练集 [53];纯速度选它 |
| 3 | MemFlow | Apache-2.0 [54] | 慢(记忆解码 15 步) | 无 | 输 |
| — | DPFlow | 仓库 404,ptlflow 页不写权重许可证 | — | — | 未核实 |
| — | RAFT | BSD-3 | 125 ms | 无 | 老 |

### 槽 7:无 CAD 的 6-DoF 物体位姿跟踪(RGB)

**没有一个既许可证干净、又不要 CAD、又不要深度、又小的。** 结论:不上模型,用 SAM 2.1 掩膜 + Track-On2 跟踪 + 已知相机运动(手臂位姿)⇒ 三角化 3D 点 ⇒ Kabsch/Umeyama 刚体拟合,每帧一个带协方差的位姿——这就是"一个机制"。

排除原因:FoundationPose(NVIDIA 非商用 + RGB-D);Any6D(依赖 FoundationPose + 需 RGB-D 锚图)[55];RGBTrack(FoundationPose 架构 + 需真尺度 CAD)[56];BundleSDF/BundleTrack(RGB-D,NVIDIA);Gen6D(**GPL-3.0** 传染,需 5+ 张参考图)[57];2026-07 RA-L 那篇 57.6 FPS 跟踪器(RGB-D)[58];B2TFPose(2026-09,RGB-only 但要 CAD 渲染模板 + DINOv3)[59];G6D(2026-09,RGB-D)。

### 槽 8:机器人在图中 / 手眼标定网络

全部要 URDF/CAD 或按机器人训练,驱动没有,全排除:CtRNet(MIT,但要 URDF + mesh 渲染 + 每种机器人训关键点网,只有 Panda/Baxter)[60];CtRNet-X 同;EasyHeC(MIT,要 mesh + 可微渲染)[61];RoboPEPP(要 URDF,仓库未见许可证)[62];DREAM(NVIDIA Source Code License 非商用,要 URDF/CAD)[63]。Kalib(无标记,用 SpatialTracker 跟一个点 + PnP)**仓库没有 LICENSE 文件 = 保留所有权利**,不能用其代码 [64];但它的方法就是我们 R2 已在做的事(跟指尖 + 解 PnP),用 Track-On2 代替 SpatialTracker 即可,零模型。

### 槽 9:其它物理量

| 量 | 结论 |
|---|---|
| 透明 / 反光物体深度 | **OptiGeo**(MIT 代码,30M,30 FPS,ClearGrasp/TransCG 上 ≥ MoGe-2)是唯一小而新的;但权重是 DINOv3 ViT-S 派生 ⇒ DINOv3 License 条款("Built with DINOv3"署名、AUP、同许可证转发)跟着走,MIT 标签洗不掉 [16][17][65]。MODEST(MIT,ICRA 2025,Syn-TODD/ClearPose 训练,窄域)[66]。建议:先量 MoGe-2 在仿真玻璃上的误差,不够再上 OptiGeo |
| 物体尺寸 | 不需要模型:MoGe-2 米制点图 × SAM 掩膜 |
| 遮挡 / amodal 补全 | 没有干净且小的:SAMEO(CVPR 2025,基于 EfficientSAM,项目页 CC BY-SA,权重未明);saraao/amodal(Apache,但是扩散补全,重);TABE 管线。不推荐 |
| 接触 / 滑移 / 视觉力 | RGB 单目没有可信的开源小模型;Sparsh 等是给 DIGIT/GelSight 触觉传感器的。用夹爪通道读数 + 跟踪点相对手指的漂移 |
| 材质 / 质量 | 没有可信的;排除 |

### 槽 10:稠密通用特征

| 名次 | 模型 | 许可证 | 判决 |
|---|---|---|---|
| **1** | DINOv2 ViT-S/B(21M / 86M) | Apache-2.0(代码 + 权重)[67] | 干净;并且已经在 Track-On2、MoGe、MapAnything 里当骨干——**不用单独再放一件** |
| 2 | LingBot-Vision ViT-S/B(蚂蚁 Robbyant,2026-07,masked boundary modeling) | GitHub + HF 标 Apache-2.0 [68][69];**论文写 CC BY 4.0** [70] ⇒ 矛盾待问 | 稠密任务号称超同尺寸 DINOv2/v3(1.1B 在 NYUv2 RMSE 0.296 胜 DINOv3-7B 0.309),小号数字未见;有意思的 2026 挑战者 |
| ✗ | DINOv3 | **DINOv3 License**:允许商用,但要"Built with DINOv3"署名、AUP(禁军事/ITAR)、派生物须同许可证、下载要登记 [65][71] | dev-only;派生物(Track-On2 DINOv3 版、Track-On-R、OptiGeo、C-RADIOv4 的老师)都带这条尾巴 |
| ✗ | RADIO / C-RADIOv4 | **代码 NSCL 非商用**;C-RADIO 权重 NVIDIA Open Model License [72] | 代码就不干净,排除 |
| — | Perception Encoder(Meta) | Apache-2.0 [73] | PE-Spatial 只有 G/14,太大 |

### 槽 11:单目 VO / SLAM(移动底盘、人形头眼,后用)

| 名次 | 模型 | 许可证 | 数字 | 判决 |
|---|---|---|---|---|
| **1** | DPVO | MIT [74] | RTX 3090:默认 60 FPS / 4.9 GB,Fast 120 FPS / 2.5 GB;EuRoC ATE 0.105 m vs DROID-VO 0.186;**全部用 TartanAir 合成训练** [75] | 要内参(我们量得出);单目尺度靠轮式里程计或 MoGe-2 米制锚 |
| 2 | DROID-SLAM | BSD-3 [76] | 推理 ≥11 GB;VO 模式 40 FPS / 8.7 GB | 太占显存 |
| — | MapAnything / DA3-SMALL | Apache | 序列帧也能估位姿 | 备用,不专门为此加件 |
| ✗ | MASt3R-SLAM | CC BY-NC-SA 4.0 [77] | — | 排除 |
| ✗ | VGGT-SLAM 2.0(RSS 2026) | 代码 BSD-2,但吃 VGGT 权重(非商用/门控)[78] | — | 排除 |
| △ | ViPE(NVIDIA,v1.2 2026-06) | 代码 Apache,但 UniK3D 组件 CC BY-NC-SA + 下载的第三方模型各带许可证 [79] | 离线管线 | 混合许可证,dev-only |

---

## (c) 逐模型事实卡

### GeoCalib
- 仓库:https://github.com/cvg/GeoCalib · 论文 arXiv 2409.06704(ECCV 2024)
- 许可证原文:"The code is provided under the Apache-2.0 License while the weights of the trained model are provided under the Creative Commons Attribution 4.0 International Public License." [1]
- 架构:SegNeXt 编码-解码 + 可微 LM 优化;训练输入 320×320;论文报 ≈100 ms/张(GPU 未指明)[80];参数量未公布。
- 输入:`[C,H,W]` 张量,支持 batch;`calibrate(batch, shared_intrinsics=True)` 多帧共享内参;`calibrate(batch, camera_R_rig=...)` 已知 rig 内相对旋转时联合重力 [2][3]。
- 输出:`camera`(焦距;主点固定图像中心)、`gravity`;论文:LM 收敛处协方差 Σ_θ=H⁻¹ 给不确定度,可标失败 [80]——README 输出键只列 camera/gravity,具体不确定度键名要跑一遍确认。
- 合成域证据:TartanAir(合成)roll 0.43° / pitch 1.49° / FoV 4.90°,远好于 ParamNet、UVP [80];作者自述"重力比 FoV 准,纬度约束弱"。
- 最近更新:v1.0 2024-09-08(唯一 release);commits 2025-03-30 shared intrinsics(#27)、2026-05-18、2026-06-28 rig 联合重力(#43)[3]。
- 安装:`pip install -e "git+https://github.com/cvg/GeoCalib#egg=geocalib"`;权重自动下到 `torch.hub` 目录。
- 权重:`https://github.com/cvg/GeoCalib/releases/download/v1.0/geocalib-pinhole.tar`(116,074,121 B,ETag `0x8DCD005D5C6275F`)、`geocalib-distorted.tar`(116,143,955 B);官方没发 SHA256。
- 失败模式(推测,未验):腕眼贴桌 10–30 cm、画面里没有地平线/竖直线时重力方向没约束;FoV 本就是它的弱项——所以焦距一定要和 R2 对账。

### MoGe-2 / MoGe-3
- 仓库:https://github.com/microsoft/MoGe · MoGe-2 arXiv 2507.02546 · MoGe-3 arXiv 2607.17967(2026-07-21)
- 许可证原文:"MoGe code is released under the MIT license, except for DINOv2 code which is released by Meta AI under the Apache 2.0 license."权重 HF 标 MIT [12][81]。
- 变体:`Ruicheng/moge-2-vits-normal`(35M,141 MB,SHA256 `79a16621928c2bf0ed04659218c55c01075e950507f40bb3332fb4c873d3e1dc`)、`Ruicheng/moge-2-vitb-normal`(104M,419 MB,`16b8110e86d5dc5a849db120ca96ef3a223fd30b0c9146d1d81db504073da5f6`)、`Ruicheng/moge-2-vitl`(326M,1.31 GB,`3eefd4abb2102f38f12b2d1992e5ff15e4923e5431c67dd494afe157e0111cd5`)、`Ruicheng/moge-3-vitl`(370M,1.48 GB,`9b41b7b9f65ad80aab7ad686f5e9cc0d1fd33f1964022618dfbcd52fc1fb7925`)、`Ruicheng/moge-3-vitg`(1.25B)。
- 输入:单张 RGB(2:1–1:2 任意比例);**可选 `fov_x`(水平 FoV,度)**——我们有 R2/GeoCalib 的焦距就该喂。
- 输出:米制点图 (H,W,3,OpenCV)、米制深度、法线(S/B 的 -normal 版)、内参 3×3(归一化)、有效掩膜;MoGe-3 另给逐步精修中间量。**无置信度。**
- 速度:MoGe-2 ViT-L A100 fp16 29 ms@484² / 39 ms@700² / 55 ms@840²(fp32 慢 3–4 倍)[7];README 另写"60 ms(A100/3090,fp16,ViT-L)";MoGe-3-L 121 ms、-G 177 ms(A100 fp16,3 步精修)[13];ViT-S/B 未公布(估 4090 上 ViT-B ≤20 ms)。
- 合成域证据:评测含 Synth4K、Spring、Sintel(MoGe-3)[13];MoGe-2 在 Sintel/Spring 有边界 F1 [7]。
- 失败模式:细线、头发;前后景尺度差大时直线不直 [7];回归范式边界飞点 [13];腕眼近景米制尺度先验可能偏(训练多为房间尺度)——未验,V2 要量。
- 安装:`pip install git+https://github.com/microsoft/MoGe.git`;MoGe-3 需 FlexGEMM/Triton。
- 最近更新:MoGe-3 发布 2026-08-18 [12]。

### MapAnything(Apache 版)
- 仓库:https://github.com/facebookresearch/map-anything · arXiv 2509.13414 · HF `facebook/map-anything-apache`
- 许可证:代码 Apache-2.0;`facebook/map-anything-apache` Apache-2.0,`facebook/map-anything` CC-BY-NC 4.0(另有 `-v1` 旧版)[23][24]。
- 架构:DINOv2 ViT-G 编码器 + 16 层交替注意力(1536 维,24 头)≈1B;输入最长边 518 px [25]。
- 权重:`model.safetensors` 4.91 GB(F32),SHA256 `fa06c0fdccefc5048e072c85935d5789b1e36b307f3859033c17f9dcb9fd5201`;HF 写"Latest release on Jan 20th 2026" [82];GitHub releases v1.0.1/v1.1(新 ckpt、AerialMegaDepth)→ v1.1.3(2026-07,关掉 DINO hub 初始化)[83]。
- 输入:`images` + 可选 `intrinsics` 或 `ray_directions`(二选一)、`camera_poses`(cam2world,4×4 或四元数+平移)、`depth_z`(需标定)、`is_metric_scale` [24]。
- 输出:`pts3d`/`pts3d_cam`、深度(z 和沿光线)、光线、内参、位姿、**逐像素置信度**、有效掩膜。
- 训练集(Apache 版):BlendedMVS、Mapillary Planet-Scale Depth、ScanNet++ v2、Spring、TartanAirV2-WB、UnrealStereo4K——6 个里 4 个是渲染/合成 [25]。
- 速度/显存:论文只有 H200 上的曲线图(Fig S.1),无数字;memory-efficient 模式"负 trade-off"(2000 视图 140 GB)。**4090 数字必须自己跑 `scripts/profile_memory_runtime.py`。**
- 已知限制:不对几何输入噪声建模(我们的位姿有协方差,它当真值用);第一视图必须有位姿;联网下载 HF 权重。
- 安装:clone 后 `pip install -e .`(PyPI 页没打开,未确认有包)。

### Track-On2
- 仓库:https://github.com/gorkaydemir/track_on(ICLR 25 / TPAMI 26 / CVPR 26)· arXiv 2509.19115
- 许可证:仓库 MIT(Görkay Aydemir 2025)[34];HF `gorkaydemir/track_on2` 标 MIT [35]。
- 权重:`trackon2_dinov2_checkpoint.pt` 93.9 MB,SHA256 `34c35ea64ea68f3c633c901c2d7876964c34212455dfbb2d508aaea1c4978973`(**用这个**);`trackon2_dinov3_checkpoint.pt`(README 默认;需另行申请 DINOv3 权重,带 DINOv3 条款);`track_on_r.pt` + `verifier.pt`(真实视频微调,同样 DINOv3)。DINOv2 版代码在 `track-on2` 分支 [84]。
- 数字:52.3M(23.6M 可训);A100 FP32 >35 FPS@64 点 0.64 GB、>30 FPS@256 点 <0.5 GB;DINOv2 版 DAVIS AJ 66.8 / RoboTAP 67.1 / Kinetics 55.2(DINOv3 版 67.0 / 68.1 / 55.3)[36]。
- 输入/输出:逐帧 RGB + 任意时刻插入查询点;输出像素坐标、可见性、**不确定度**。
- 训练:只用 TAP-Vid Kubric(合成)[36]。
- 未知:640×480 在 4090 上的延迟、记忆长度 72 帧之外的长期重检测。

### TAPNext++(亚军,留作对照)
- 仓库:https://github.com/google-deepmind/tapnet(`tapnet/tapnextpp/`)· arXiv 2604.10582(2026-04-12,CVPR 2026 Findings)
- 许可证原文:"All pre-trained model checkpoints released in this repository... are also licensed under Apache 2.0";tapnextpp README:"The TAPNext++ model checkpoint is released under the Apache License, Version 2.0" [37][85]。
- 权重:`https://storage.googleapis.com/dm-tapnet/tapnextpp/tapnextpp_ckpt.pt`(2,532,282,370 B,MD5 `a6388e0b911f2e3644fafa8a9a118dfb`,2026-03-27)、`https://storage.googleapis.com/gresearch/tapnextpp/tapnextpp_512.ckpt`(2,532,283,010 B,MD5 `eea4fffa043f4f28503c130d826b116c`,2026-06-22)。2.5 GB 对 194M 参数偏大(可能含优化器/EMA),要核。
- 数字:H100 256×256 193 FPS(5.18 ms)@256 点、512×512 191 FPS [38];DAVIS First 65.6(256)/ 67.0(512),RoboTAP AJ 61.1 / 66.0,RD-AJ 54.6 [85][86]。
- PyTorch 实现 `tapnet/tapnext/tapnext_torch.py`:torchvision `EncoderBlock`(SDPA),无 flash_attn 导入,`TAPNextTrackingState` 逐帧状态;位置头 512 bin softmax ⇒ 可算离散度 [39]。论文测速用了 FlashAttention-3(H100),4090 上速度会不同。

### SAM 2.1
- 仓库:https://github.com/facebookresearch/sam2;许可证原文:"The SAM 2 model checkpoints, SAM 2 demo code (front-end and back-end), and SAM 2 training code are licensed under Apache 2.0." [46]
- 表(A100,torch 2.5.1,cuda 12.4):tiny 38.9M 91.2 FPS SA-V test 76.5;small 46M 84.8 FPS 76.6;base_plus 80.8M 64.1 FPS 78.2;large 224.4M 39.5 FPS 79.5 [46]。
- 权重:`https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt`(156,008,466 B)、`sam2.1_hiera_small.pt`(184,416,285 B);官方无 SHA。
- 安装:clone + `pip install -e .`(Python ≥3.10,torch ≥2.5.1);PyPI 上的 `sam2` 是第三方打包 [87];HF transformers 已有 SAM2 Video 文档页可作备选部署路径 [88]。
- 视频模式显存未公布(估 tiny 2–4 GB);发布 2024-09-29,之后无新版。
- 失败模式:细长物(剪刀)和纹理相近背景会漏/溢(PLAN 里 H61 半片桌纹);用 IoU 分数门控。

### SEA-RAFT(S/M)/ NeuFlow v2
- SEA-RAFT:https://github.com/princeton-vl/SEA-RAFT,BSD-3 [50];S = ResNet-18 前 6 层 4 次迭代,M = ResNet-34 前 13 层;RTX 3090 540×960 S 47.5 ms / M 70.9 ms / L 108 ms;Spring EPE 0.363;Laplace 混合损失 ⇒ 逐像素 α、β₂(方差)[51];TartanAir 预训练。权重 `MemorySlices/Tartan-C-T-TSKH-spring540x960-M/model.safetensors` 78,778,760 B,SHA256 `cb8cfbf14c5e0f6734b64add383708b7ff68cc6089a0007c67165d4761346102`(S 版权重名未核)。
- NeuFlow v2:https://github.com/neufieldrobotics/NeuFlow_v2,Apache-2.0 [52];9M;RTX 2080 15 ms@1024×436、Jetson Orin Nano 106 ms;Sintel clean/final 1.24/2.67,KITTI 4.33/15.3(均 FlyingThings 单训)[53];权重 `neuflow_mixed.pth`;无不确定度。

### DA3-SMALL / DA3-BASE / DA3METRIC-LARGE(备胎)
- 仓库:https://github.com/ByteDance-Seed/Depth-Anything-3;PyPI 官方 `pip install depth-anything-3`(0.1.1,2026-03-04)[89]。
- 许可证:SMALL/BASE/METRIC-LARGE/MONO-LARGE Apache-2.0;LARGE-1.1/GIANT-1.1/NESTED-1.1 CC BY-NC 4.0("-1.1 是修了训练 bug 后重训的")[14]。
- 权重:`depth-anything/DA3-SMALL` 137 MB SHA256 `364492e38a3a06d221ac75da7f6621ada3f2361cd24fde11ba79091e9f40efcf`;`DA3-BASE` 542 MB `e01067dc1659613083d9145a9a2547ccdbe6ccbbf83c4fe7b3e8a4e2bdae78b5`;`DA3METRIC-LARGE` 1.34 GB `bbea5b0b3ee389849cffa7ddae89de064a90abd2b055fc5aa99aac68db324776`。
- API:`model.inference(image, extrinsics=(N,4,4), intrinsics=(N,3,3), align_to_input_ext_scale=True)` ⇒ `depth (N,H,W)`、`conf`、`extrinsics (N,3,4)`、`intrinsics` [26];METRIC:`metric = focal·out/300` [15]。
- 速度:A100 504×336、32 图/场景:Small 160.5 FPS、Base 126.5、Large 78.4 [8]。

### OptiGeo(槽 9 备胎)
- 仓库:https://github.com/mx-liu6/OptiGeo(CoRL 2026)· arXiv 2608.29881 · HF `mxliu-hku/OptiGeo`(`OptiGeo.pt` 121 MB,SHA256 `d0941b669d41eea9ccc63b831232f3d70791980ea339d2bf3421e437bb3f9292`)
- 许可证:代码 MIT [16];但论文:"a ViT-Small encoder pre-trained with DINOv3" [17] ⇒ 权重是 DINOv3 派生物。
- 输出:相对点图(MoGe 式)+ 深度 + FoV + 可靠性掩膜;fp16 1000 token 32.8 ms / 0.42 GB [17]。
- 数据:9M 样本 21 数据集 + Infinigen 渲染 77K 帧桌面玻璃器皿 [17]。

### DPVO(槽 11,后用)
- https://github.com/princeton-vl/DPVO,MIT [74];RTX 3090 60 FPS/4.9 GB(默认)、120 FPS/2.5 GB(Fast);TartanAir 合成训练;需内参文件 [75]。

---

## (d) 仿真域证据(RoboDojo / Isaac 渲染图能不能直接用)

- **点跟踪是合成原生的**:Track-On2 只用 Kubric 训练,TAPNext++ 用 Kubric-1024 + PointOdyssey;Track-On2 论文明说 DINOv2 变体在合成 PointOdyssey 上优于 DINOv3 变体 [36][38]。⇒ 仿真里没有域差,真机才有。
- **GeoCalib** 在合成 TartanAir 上 roll 0.43° / pitch 1.49° / FoV 4.90° [80]。
- **MoGe-2/3** 评测含 Synth4K、Spring、Sintel [7][13];**DA3** 的教师模型完全用合成数据训练,训练集含 TartanAir、Objaverse、Trellis [8][90];**MapAnything Apache 版** 6 个训练集里 4 个是渲染的(TartanAirV2-WB、Spring、UnrealStereo4K、BlendedMVS)[25];**DA V2-Metric-Small** 直接在 Hypersim 上训 [20];**DPVO** 全 TartanAir 训 [75];**SEA-RAFT** TartanAir 预训练,**NeuFlow v2** FlyingThings 单训 [51][53]。⇒ 深度/几何这一层对合成图是"回家",风险反而在真机。
- **SAM 2**:训练集是真实 SA-V;找到的旁证是 1400 万张含合成数据的瞳孔分割研究和手术视频扰动鲁棒性研究 [91][92],**没有 Isaac Sim 专项评测**。SAM2 在仿真桌面上的表现要自己量(PLAN 里 H61 那种半片桌纹就是证据)。
- **OptiGeo** 的透明物训练本身就是渲染的(Infinigen)[17]。
- 尚无任何一篇给"腕眼 10–30 cm 近景 + 渲染桌面"的度量深度误差;这是 V2 要量的第一个数。

---

## (e) 排除清单(原因)

**许可证:** CoTracker3(CC-BY-NC)[41];SpatialTrackerV2(CC BY-NC 4.0)[42];CUT3R、StreamVGGT、MASt3R/DUSt3R/MASt3R-SLAM(CC BY-NC-SA 4.0)[30][31][77];Fast3R(FAIR NC)[32];VGGT 原权重非商用、`VGGT-1B-Commercial` 需申请 [27];VGGT-Ω(FAIR Noncommercial Research License,门控)[28];π³/Pi3X 权重 CC BY-NC 4.0 [29];UniDepthV2(CC BY-NC 4.0)[22];Depth Pro(HF 标 apple-amlr 仅研究,与 GitHub LICENSE 文本矛盾)[21];Depth-Anything-V2 Base/Large/Giant、DA3 LARGE/GIANT/NESTED-1.1(CC-BY-NC 4.0)[14][20];PerspectiveFields(Adobe Research License 非商用)[10];DiffCalib(仓库注明非商用)[11];X-Lens(CC BY-NC 4.0)[33];FoundationPose / BundleSDF / DREAM(NVIDIA 非商用)[63];RADIO(代码 NSCL 非商用)[72];Gen6D(GPL-3.0 传染)[57];SAM 3/3.1(SAM License:商用可但 AUP + 转许可 + 门控;owner 已定不用)[49];DINOv3 及其派生物(Track-On2 DINOv3 版、Track-On-R、OptiGeo 权重、C-RADIOv4)——商用可但署名 + AUP + 同许可证转发 + 登记 [65][71];Kalib(无 LICENSE 文件)[64];RoboPEPP(未见许可证)[62];DELTA、MVTracker(许可证未核出)[44][45]。

**要 CAD / URDF / 按机器人训练:** CtRNet、CtRNet-X、EasyHeC/EasyHeC++、RoboPEPP、DREAM(槽 8 全部);RGBTrack、B2TFPose、SAM-6D、GigaPose(要物体 CAD);Any6D(RGB-D 锚图 + FoundationPose)。

**要深度传感器:** TAPIP3D 单目管线(MegaSaM)、DELTA、MVTracker、LingBot-Depth(RGB-D 融合,RGB-only 未说明)[93]、MetricAnything teacher(稀疏度量提示,未发布)、2026-07 RA-L 跟踪器、G6D。

**决策者(owner 规则,不属仪器):** 抓取位姿网络(AnyGrasp、GraspNet、Contact-GraspNet、GraspGen 等)、VLA/策略——一律不看。

**太慢:** 扩散类(Marigold V2、Pixel-Perfect Depth、DiffCalib、扩散 amodal)。

---

## (f) 没能核实的(必须自己量或去问)

1. **4090 上 640×480 的实际延迟与显存**:Track-On2、MoGe-2 ViT-S/B、SAM 2.1 tiny/small 视频模式、MapAnything(≤16 视图)、GeoCalib——一个都没有公开数字。表里的都是估的。
2. **MapAnything(Apache)vs DA3-SMALL/BASE 在"已知位姿 + 近景桌面"下的精度**:无头对头;DA3 小模型给真位姿后差距缩小 [8],但没和 MapAnything 比过。
3. **Track-On2 vs TAPNext++ 的长期遮挡重检测**:TAPNext++ 自定义 RD-AJ 指标上 Track-On2 未测;标准三集 Track-On2 全胜 [36][85]。
4. **MoGe-2 焦距精度 vs GeoCalib / AnyCalib**:MoGe-2 论文不报 [7]。
5. **GeoCalib 在腕眼近景(无地平线)上的重力可靠性**:无证据。
6. **OptiGeo 的许可证实质**:MIT 标签 vs DINOv3 派生权重,需法务口径。
7. **LingBot-Vision 许可证矛盾**(仓库/HF Apache-2.0,论文 CC BY 4.0)及小号模型的稠密任务数字。
8. **CalibAnyView** 有无代码(没找到)。
9. **TAPNext++ ckpt 为何 2.53 GB**、发布的是 S(56M)还是 B(194M)。
10. **DA3METRIC-LARGE 推理时是否真给 conf**(API 文档写 conf 可选)。
11. **DA3 位姿条件是否原生用输入平移的米制尺度**:API 是事后 Umeyama 缩放(`align_to_input_ext_scale`),不是原生 [26]。
12. SEA-RAFT(S) 官方权重文件名/校验和(只核了 M)。

---

## 引用

[1] https://github.com/cvg/GeoCalib (README License 段)
[2] https://raw.githubusercontent.com/cvg/GeoCalib/main/README.md (`shared_intrinsics`, `camera_R_rig`)
[3] https://github.com/cvg/GeoCalib/commits/main (2025-03-30 #27, 2026-06-28 #43)
[4] https://github.com/javrtg/AnyCalib ("Code and weights are provided under the Apache 2.0 license")
[5] https://arxiv.org/html/2503.12701v2
[6] https://arxiv.org/html/2605.14615
[7] https://arxiv.org/html/2507.02546 (MoGe-2, Table B.3)
[8] https://arxiv.org/html/2511.10647v1 (DA3, Table 8 / scaling tables)
[9] https://github.com/ShngJZ/WildCamera
[10] https://raw.githubusercontent.com/jinlinyi/PerspectiveFields/main/LICENSE
[11] https://github.com/zjutcvg/DiffCalib
[12] https://github.com/microsoft/MoGe (README license 段, MoGe-3 2026-08-18)
[13] https://arxiv.org/html/2607.17967 (MoGe-3)
[14] https://github.com/ByteDance-Seed/Depth-Anything-3 (模型表 License 列)
[15] https://huggingface.co/depth-anything/DA3METRIC-LARGE
[16] https://github.com/mx-liu6/OptiGeo
[17] https://arxiv.org/html/2608.29881
[18] https://github.com/metric-anything/metric-anything ; https://huggingface.co/yjh001/metricanything_student_pointmap
[19] https://github.com/YvanYin/Metric3D
[20] https://github.com/DepthAnything/Depth-Anything-V2 ; https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small
[21] https://huggingface.co/apple/DepthPro/blob/main/LICENSE ; https://raw.githubusercontent.com/apple/ml-depth-pro/main/LICENSE
[22] https://github.com/lpiccinelli-eth/UniDepth
[23] https://huggingface.co/facebook/map-anything-apache
[24] https://raw.githubusercontent.com/facebookresearch/map-anything/main/README.md
[25] https://arxiv.org/html/2509.13414v3
[26] https://raw.githubusercontent.com/ByteDance-Seed/Depth-Anything-3/main/docs/API.md
[27] https://github.com/facebookresearch/vggt
[28] https://huggingface.co/facebook/VGGT-Omega ; https://github.com/facebookresearch/vggt-omega
[29] https://github.com/yyfz/Pi3
[30] https://raw.githubusercontent.com/CUT3R/CUT3R/main/LICENSE
[31] https://raw.githubusercontent.com/wzzheng/StreamVGGT/main/LICENSE.txt
[32] https://github.com/facebookresearch/fast3r
[33] https://github.com/zhouhengamerica/XLens
[34] https://github.com/gorkaydemir/track_on/blob/main/LICENSE
[35] https://huggingface.co/gorkaydemir/track_on2/tree/main
[36] https://arxiv.org/html/2509.19115v1 ; https://arxiv.org/html/2509.19115
[37] https://raw.githubusercontent.com/google-deepmind/tapnet/main/README.md
[38] https://arxiv.org/html/2604.10582v1 ; https://tap-next-plus-plus.github.io/
[39] https://raw.githubusercontent.com/google-deepmind/tapnet/main/tapnet/tapnext/tapnext_torch.py
[40] https://github.com/cvlab-kaist/locotrack
[41] https://github.com/facebookresearch/co-tracker
[42] https://raw.githubusercontent.com/henry123-boy/SpaTrackerV2/main/LICENSE.txt
[43] https://github.com/zbw001/TAPIP3D
[44] https://github.com/snap-research/DELTA_densetrack3d
[45] https://github.com/ethz-vlg/mvtracker
[46] https://raw.githubusercontent.com/facebookresearch/sam2/main/README.md
[47] https://github.com/yformer/EfficientTAM ; https://arxiv.org/html/2411.18933
[48] https://github.com/facebookresearch/EdgeTAM ; https://arxiv.org/html/2501.07256
[49] https://raw.githubusercontent.com/facebookresearch/sam3/main/LICENSE
[50] https://github.com/princeton-vl/SEA-RAFT
[51] https://arxiv.org/html/2405.14793
[52] https://github.com/neufieldrobotics/NeuFlow_v2
[53] https://arxiv.org/html/2408.10161v1 (Table I)
[54] https://github.com/DQiaole/MemFlow
[55] https://github.com/taeyeopl/Any6D
[56] https://arxiv.org/abs/2506.17119
[57] https://github.com/liuyuan-pal/Gen6D
[58] https://arxiv.org/abs/2607.23468
[59] https://arxiv.org/abs/2609.06726
[60] https://github.com/ucsdarclab/CtRNet-robot-pose-estimation
[61] https://github.com/ootts/EasyHeC
[62] https://github.com/raktimgg/RoboPEPP
[63] https://github.com/NVlabs/DREAM
[64] https://github.com/Learner209/Kalib (LICENSE 404)
[65] https://ai.meta.com/resources/models-and-libraries/dinov3-license/
[66] https://github.com/L-J-Yuan/MODEST
[67] https://github.com/facebookresearch/dinov2
[68] https://github.com/robbyant/lingbot-vision
[69] https://huggingface.co/robbyant/lingbot-vision-vit-small
[70] https://arxiv.org/html/2607.05247v1
[71] https://github.com/facebookresearch/dinov3
[72] https://github.com/NVlabs/RADIO
[73] https://github.com/facebookresearch/perception_models
[74] https://github.com/princeton-vl/DPVO
[75] https://arxiv.org/pdf/2208.04726 (Fig. 8/10, Tab. 2)
[76] https://github.com/princeton-vl/DROID-SLAM
[77] https://raw.githubusercontent.com/rmurai0610/MASt3R-SLAM/main/LICENSE.md
[78] https://github.com/MIT-SPARK/VGGT-SLAM
[79] https://github.com/nv-tlabs/vipe
[80] https://arxiv.org/html/2409.06704v1
[81] https://huggingface.co/Ruicheng/moge-2-vits-normal/blob/main/model.pt (及 vitb-normal, vitl, moge-3-vitl)
[82] https://huggingface.co/facebook/map-anything-apache/blob/main/model.safetensors
[83] https://github.com/facebookresearch/map-anything/releases
[84] https://raw.githubusercontent.com/gorkaydemir/track_on/main/README.md
[85] https://github.com/google-deepmind/tapnet/tree/main/tapnet/tapnextpp
[86] https://github.com/google-deepmind/tapnet (README TAPNext++ 表)
[87] https://pypi.org/project/sam2/
[88] https://huggingface.co/docs/transformers/en/model_doc/sam2_video
[89] https://pypi.org/project/depth-anything-3/
[90] https://arxiv.org/html/2511.10647v1 (数据一节)
[91] https://arxiv.org/html/2410.08926v2
[92] https://arxiv.org/html/2408.04593v1
[93] https://github.com/robbyant/lingbot-depth
