# 换箱手册(2026-08-23 定,踩过三台机器才凑齐)

## 租机硬约束(选错就得重来一遍)
| 项 | 要求 | 为什么 |
|---|---|---|
| 驱动版本 | **550 ≤ driver < 581** | 下界:vllm 0.27(眼)要 CUDA 12.8+,535.x 直接报 "NVIDIA driver too old (found 12020)";上界:≥581 的 Vulkan 回归让 IsaacSim 开场景段错误 |
| GPU | ≥2 张(sim 一张、眼一张) | 眼和 sim 抢同一张卡会 OOM(旧箱实测:sim 占 7.8G 后 vllm 90% 配额起不来) |
| 盘 | ≥200 GB | RoboDojo 39G + 眼模型 19G + 其余 |
| 下行 | 越快越好(≥400 Mbps) | 要下 58 GB,带宽就是时间 |
| 创建参数 | `--env '-e NVIDIA_DRIVER_CAPABILITIES=all'` | 事后补不了 |

## 重建顺序
1. `eye_setup.sh`(装 vllm)→ `eye_dl.sh`(下 Qwen/Qwen3.5-9B,19G)→ `eye_serve.sh`(钉到 sim 不用的那张卡)
2. `git clone https://github.com/robodojo-benchmark/RoboDojo` → `boot_rd.sh` → **失败在 conda 激活是正常的** → `boot_rd2.sh`(软链 /venv/RoboDojo 进 conda envs,从 base_deps 续)→ `boot_rd3.sh`(from isaacsim)
3. 传 body-layer 源码:**plug + slow + contact-gen + selfcal + point-gen + contact-exec + contact-set + abi + fast + battery + conformance + realdata**(少一个 cargo 就报 "No such file or directory",实测漏了两轮)
4. `cargo build --release` → `cp target/release/bl-calibrate /root/.local/bin/`
5. `M0/cal.json` 放好(种子标定,省一次全量自校准)
6. `qipao.sh <炮号>` 起炮

## 血账
- 宿主 **133997**:8 张 GPU 全掉总线,stop+start 无效 —— 拉黑
- 宿主 **113571**:镜像拉完卡在 loading 16 分钟不给端口 —— 拉黑
- 宿主 **43488**:机器好,但驱动 535.247 太旧,vllm 起不来 —— 只适合纯 sim
- 宿主 **24850**(驱动 570.195.03):当前在用
