# 换箱手册(2026-08-23 定,踩过三台机器才凑齐)

## 租机硬约束(选错就得重来一遍)
| 项 | 要求 | 为什么 |
|---|---|---|
| 驱动版本 | **580.x**(窗口只有一个大版本宽) | 下界实测三次:535.247 报 "driver too old (found 12020)"、570.195 报 "too old (12080)" —— vllm 0.27 要 CUDA 13.0,只有 580 起才带;上界:≥581 的 Vulkan 回归让 IsaacSim 开场景段错误 |
| 眼环境 Python | **3.12**,不能更低 | vllm 0.27 模块级 import flashinfer,而 flashinfer 在 3.10 崩 `type[...]`、3.11 崩 `array.array[...]`,3.12 才合法。卸掉 flashinfer ⇒ vllm ModuleNotFoundError;`--enforce-eager` 绕不开(是 import 期,不是运行期)。装法:`uv python install 3.12 && uv venv --python 3.12 /root/eyeenv3 && VIRTUAL_ENV=/root/eyeenv3 uv pip install vllm` |
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

### 🔴 渲染必装:`apt-get install -y libglu1-mesa`(缺它的症状完全不像缺库)
少了 `libGLU.so.1`,`omni.iray.libs/bin/iray/libneuray.so` 加载失败 ⇒ 材质/着色器系统整片起不来 ⇒
日志刷几十条 `Cannot load shader file 'rtx/raytracing/*.hlsl'` + `HydraEngine rtx failed creating scene renderer`,
**相机返回空图**,而 IsaacSim 自己在两分钟后弹 `Kit appears to be hanging`(容器里没有 zenity,于是只留一行 `sh: 1: zenity: not found`)。
表面看像"光追着色器没发布/驱动不对/箱子坏了",实际只差一个 6 MB 的系统包。
**判定法**:`grep "libneuray" 日志` —— 有 `libGLU.so.1: cannot open shared object file` 就是它。
**验证法**:跑 `rendertest.py`(最小场景 + 相机 30 步),要拿到非零图像;这一步把"渲染坏了"和"RoboDojo 配置错了"彻底分开。
⚠️ 老箱能渲染是因为它碰巧装过这个包 —— 换箱后**必须先跑最小渲染测试再起炮**,否则会把渲染故障误判成驱动/任务的问题。

## 血账
- 宿主 **133997**:8 张 GPU 全掉总线,stop+start 无效 —— 拉黑
- 宿主 **113571**:镜像拉完卡在 loading 16 分钟不给端口 —— 拉黑
- 宿主 **43488**:机器好,但驱动 535.247 太旧,vllm 起不来 —— 只适合纯 sim
- 宿主 **24850**:驱动 570.195.03,vllm 报 "too old (12080)" —— 拉黑
- 宿主 **33061**(驱动 580.159.04 / CUDA 13.0,2×3090):当前在用,眼与 sim 同箱跑通

## 场子(2026-08-23 换到官方配置)
上游 RoboDojo 已把 `cube_pickup` 改名 **`general_pickup`**(判据没变:抬 10 cm 算成、200 步),
并且 **env_cfg 里没有任何 Franka 单臂配置** —— 之前 N1–N127 跑的 `franka_grasp.yml` + `cube_pickup.py`
是我自己写的、**从没进过上游仓库也没进过本仓 git**,随旧箱一起没了。教训:场子定义属于实验的一部分,必须进 git。
跑的仍然是 **单臂 Franka**(重写版,已进 git);双臂 X5 那套留着备用。配置全在 `robodojo-cfg/`:
| 文件 | 放到箱上哪 | 是什么 |
|---|---|---|
| `x5_grasp.yml` | `/root/RoboDojo/env_cfg/` | 官方 `arx_x5.yml` 的唯一改动:depth/intrinsic/extrinsic 打开 |
| `camera_rgbd.yml` | `/root/RoboDojo/env_cfg/camera/` | 官方 `camera_config.yml` 把注释掉的 `distance_to_image_plane` 放开 |
| `qisim.sh` | `/root/` | 起 sim(官方任务 + 官方机体) |
| `franka_grasp.yml` | `/root/RoboDojo/env_cfg/` | **在跑的场子**:单臂 Franka + 官方 `general_pickup` |
| `franka_single.yml` | `/root/RoboDojo/env_cfg/robot/` | 一条居中的 Franka(官方那条 franka 是 `type: support`/`need_planner: False` 的配角,不能拿来抓) |
| `camera_rgbd_franka.yml` | `/root/RoboDojo/env_cfg/camera/` | 只留头相机 + 打开深度(腕部相机挂在 X5 臂上,franka 没有) |
| `x5_grasp.yml` / `camera_rgbd.yml` | 同上 | 双臂 X5 版,备用,**当前不跑** |
| `qidrv.sh` | `/root/` | 起驱动。**不传 `--in`** —— 旧种子是 Franka 上量的,换机体才是错先验 |

### 装完必跑的两步(漏了会在开场炸,报错完全不像缺步骤)
1. **`/venv/RoboDojo/bin/python utils/update_embodiment_config_path.py`**(在 `/root/RoboDojo` 下跑)
   把 `Assets/Robots/*/curobo_tmp.yml` 里的 `${ASSETS_PATH}` 换成绝对路径,生成 `curobo.yml`。
   不跑 ⇒ `FileNotFoundError: .../Assets/Robots/x5/curobo.yml`。
2. **自写的 env_cfg 里 `config_name` 必须写 `arx_x5`**,不能写文件名。
   布局按 `Assets/Eval_Layout/RoboDojo/<config_name>/<seed>/` 查,官方只发 `arx_x5` 的那套;
   写成别的 ⇒ `FileNotFoundError: .../Eval_Layout/RoboDojo/<你的名字>/0`。
   `arx_x5/0/` 下 `general_pickup_*.json` 共 **55** 个 ⇒ **官方一轮 = 55 集**,这就是绝对分母。
官方把深度注释掉了,而驱动靠深度找东西;Gemini_345Lg 本来就是 RGBD 相机,打开的是它已有的通道,不是加特权观测。
