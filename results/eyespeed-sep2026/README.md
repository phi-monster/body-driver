# 眼(上层 VLM)有多快 —— 量到的,以及自己部署时怎么让它更快

🔴 **这一份是【文档】,不是代码。** 加速手段属于"某个模型在某台服务器上"的性质,**不属于身体层** ——
上层 VLM 可以换成任何一个,客户的算力平台我们不知道,甚至可能直接连云端。
身体层要做的是**让它慢也没关系**(见 `../../README.md` 那一节),不是让它快。

## 一、我们自己量到的(2026-09-02,RTX 3090,Qwen3.5-9B bf16,vLLM 0.27.1)

一次完整决策(带图 + JSON schema 约束):

| | 读数 |
|---|---|
| 端到端 | **3.90 秒**(三次重复 3.94 / 3.90 / 3.90) |
| 进 prompt token | **524**(640×480 一张图约 300 个 + 文字 224) |
| 出 token | **98** |
| 把出的上限压到 40 | **1.70 秒** |

斜率:**每多吐一个 token = 37.9 ms**(≈26.4 tok/s);**固定开销 0.18 秒**。
⇒ **96% 的时间在逐 token 解码**,与图多大、提示词多长基本无关。

**由此直接判死两条**(别再花力气):
- **缩小图 / 剪视觉 token** —— 只影响 prefill,上限收益 **1.5%**
- **缓存固定提示词(prefix caching)** —— 同上,能省的只有那 0.18 秒

**由此白拿一条结论**:prefill 524 token ≤ 0.18 s,而顺序解码 524 token 要 19.9 s
⇒ **一次前向核对 4 个 token 和核对 1 个,代价几乎一样** ⇒ 投机解码在这台机器上**必然**有效。

## 二、这台机器上查到的三个具体问题(2026-09-02,读 `/root/eye_serve.log`)

| # | 查到什么 | 证据(日志原文) |
|---|---|---|
| 1 | **全解码的 CUDA graph 根本没捕获**,只捕了残缺版 | `Capturing CUDA graphs (mixed prefill-decode, PIECEWISE): 5/5`,**没有 FULL 那一行**;佐证 `CUDAGraph memory: 0.05 GiB` |
| 2 | **vLLM 0.27.1,而 Qwen3.5 的 MTP 支持是 0.28.0 才进的** | `Initializing a V1 LLM engine (v0.27.1) … speculative_config=None` |
| 3 | **KV cache 被掐到 1.32 GiB,有 2 GB 白放着** | vLLM 自己提示 `Replace … --kv-cache-memory=3460924928 (3.22 GiB) to fully utilize gpu memory` |

物理上限:3090 带宽 936 GB/s,每 token 要搬 16–18 GB 权重 ⇒ **理论 50–58 tok/s**。
**我们实测 26.4 ⇒ 大约跑在硬件上限的 45–55%。**

## 三、排序清单(自由授权搜索 agent 带回,2026-09-02)

### 逐 token 完全等价(答案一个字不变)—— 先做这三条

| # | 是什么 | 报告的倍数 | 今天哪个软件 | 边缘成不成立 |
|---|---|---|---|---|
| 1 | **MTP**:模型**自带**一个多 token 预测头(`config.json` 里 `mtp_num_hidden_layers: 1`),猜几个字、主模型一次核对 | 同卡同族实测 **2.6×**;**带图片验过**:接受率 83.4%、平均接受长度 3.50、**答案完全相同** | vLLM **0.28.0** `--speculative-config '{"method":"mtp","num_speculative_tokens":3}'` | 🟢 成立,且边缘上最值钱(拿闲置算力换带宽)。NVIDIA Jetson Thor 上同类做法 **2.5×** |
| 2 | **修引擎配置**:让 FULL decode graph 真的捕上、留出显存余量 | 极端(被迫 eager)差 **7×**;温和 13–18% | vLLM 0.28.0 | 🟢 边缘上更重要(CPU 更弱,kernel 启动占比更高) |
| 3 | **suffix decoding**:记住之前吐过什么直接填,再核对。**我们每步吐同一个 JSON 骨架,这是它的最佳工况** | 官方 **2.3–6.3×**,点名 "agentic loops" | vLLM + `arctic-inference`,`{"method":"suffix","num_speculative_tokens":32}` | 🟢 最边缘友好:不加权重、不占显存、不挑硅片 |

### 会改变答案的(要自己量,别看总分)

🔴 **量化是本清单唯一"看着没退化、其实退化了"的东西**:
W4A16 总体准确率只掉 **0.9–1.5%**,而 **答案改变率(DR)19–21%、其中 5.7–6.6% 从对变错(NDR)**。
LLM.int8() 好得多(DR **4.71%** / NDR **1.21%**)。
⇒ **要量化就先 INT8**;**必须用自己的任务量 DR/NDR,不许看总准确率**。

### 改变问题的形状(比加速更值钱)

| | 大白话 | 量级 |
|---|---|---|
| **动作分块** | 一次决策管未来 K 步 | 每步延迟 **÷K**,不动推理栈 |
| **异步 / Real-Time Chunking** | 执行当前段的同时后台算下一段 | **延迟不再是延迟**;LeRobot 已有实现 |
| **双系统** | 大模型 1–5 Hz 出子目标,小策略 20–50 Hz 出动作 | 这**正是本仓已有的分层** —— 独立查证,没走错路 |
| 🔴 **不吐文本,从隐状态直接读动作** | 别让它把决策"拼写"成 JSON | **3.71 s → 0**,约 **20×**;vLLM 有官方隐状态导出。代价:要训一个头 |

## 四、一条必须记住的事故

**"原理上无损" ≠ "实现上无损"。** vLLM issue #40875:**Qwen 系 + 结构化输出**下,ngram 投机的默认
`prompt_lookup_min=2` 会**污染输出** —— 30 次里只有 16 次干净(53%);改成 8 之后 30/30 全对。
**我们正好就是"Qwen + JSON schema"那个组合。**
⇒ **任何投机解码上线前,固定随机种子跑一次逐字节 A/B。**

## 五、出处

模型与引擎:[Qwen3.5-9B config.json](https://huggingface.co/Qwen/Qwen3.5-9B/raw/main/config.json) ·
[vLLM MTP](https://docs.vllm.ai/en/latest/features/speculative_decoding/mtp/) ·
[vLLM suffix](https://docs.vllm.ai/en/latest/features/speculative_decoding/suffix/) ·
[vLLM CUDA graphs 设计](https://github.com/vllm-project/vllm/blob/main/docs/design/cuda_graphs.md) ·
[隐状态导出](https://github.com/vllm-project/vllm/blob/main/docs/features/speculative_decoding/extract_hidden_states.md)

实测复现:[MTP 多模态 vllm#52481](https://github.com/vllm-project/vllm/issues/52481) ·
[ngram 污染 vllm#40875](https://github.com/vllm-project/vllm/issues/40875) ·
[3090 实测 syv-ai](https://github.com/syv-ai/qwen38-27b-rtx3090) ·
[3090 实测 tfriedel](https://github.com/tfriedel/qwen3.6-rtx3090-lab)

质量代价:[实例级发散 arXiv 2503.06794](https://arxiv.org/abs/2503.06794) ·
[VLM 投机解码基准 MMSpec arXiv 2603.14989](https://arxiv.org/abs/2603.14989)

形状:[RTC arXiv 2506.07339](https://arxiv.org/abs/2506.07339) ·
[LeRobot 异步推理](https://huggingface.co/docs/lerobot/main/async) ·
[FAST arXiv 2501.09747](https://arxiv.org/abs/2501.09747) ·
[分层 VLA arXiv 2606.10267](https://arxiv.org/abs/2606.10267)

## 六、没核实的(照实列)

- 4090 上 Qwen3.5-9B ~126 tok/s(llama.cpp Q4)那个数,原页面打不开(HTTP 403),只有搜索摘要
- **DFlash 是否"greedy 下逐字节相同"没有任何一方明说** —— 四种措辞都在回避这句;MTP 那边有实测支持
- DFlash 在边缘硅片上**零实测数字**
- 我们那 98 个输出 token 里有多少是**语法强制**的(只有我们自己的 schema 能回答)
- 量化的 DR/NDR 是在 LLaVA-1.5 / Qwen2-VL 上量的,**不是 Qwen3.5**

---

## 六b、实测对照(2026-09-02,同一批 6 张真实画面,温度 0,固定种子)

| 引擎 | 平均 | 决定与旧眼相同 | 文本与旧眼相同 |
|---|---|---|---|
| **旧眼** vLLM 0.27.1 | **3.16 s** | — | — |
| 0.28.0,**不开** MTP | 3.20 s | 5/6 | 2/6 |
| 0.28.0 + **MTP k=3** | 2.59 s(去掉首张预热 **2.08 s**)| 5/6 | 2/6 |
| 同引擎 MTP vs 无 MTP | — | **5/6** | **4/6** |

- **单换引擎版本输出就变了**(bf16 内核不同,贪心下临界 token 翻转)⇒ "跨版本逐字节一样"不存在。
- 6 张里那张 NBZ 三个配置答了三个不同的决定((3,12) / (11,11) / (11,12))⇒ **模型在刀刃上,任何扰动都翻它**,不是 MTP 的账。
- **MTP 接受率**(vLLM metrics):371 个猜测里 **220 个被接受(59%)**,位置 0 接受 96/172;每次前向多拿约 1.3 个 token ⇒ 与墙钟 **≈1.4×** 对得上(未到报告的 2.6×)。
- 0.28.0 上 **FULL 解码 CUDA graph 真的捕获了**(`Capturing CUDA graphs (decode, FULL)`),KV cache 35,354 → 45,452 token;**但不开 MTP 时没有可测的速度差**。
- 🔴 **结论**:MTP **不是**逐字节无损(agent 那条警告应验),但其漂移**不超过换一次引擎版本的漂移**;决定层面 5/6。**已切换驱动到它**(端口 8078,`CUDA_VISIBLE_DEVICES=1`)。
- 合成对照提示词里发现一个坑:只列了 4 个 item,模型却答 `move_item=11` —— **把"第 11 格"和"第 11 号"混了**。真炮里编号已画在图上,要盯。
- 待测:suffix decoding(`arctic-inference 0.3.0` 已装)—— 每步同一个 JSON 骨架,是它的最佳工况。

# 七、定论去 README

这一份只留**读数和出处**。由它推出来的架构定论(快路只对自跑得起的模型成立 · 两种"小头" ·
能力拆两半 · 1000 Hz 的物理墙 · 真约束是装不装得下 · 换脑不改代码)
⇒ 见 [`../../README.md`](../../README.md) 的《脑子放哪、快到什么程度、换谁都能开》一节。
