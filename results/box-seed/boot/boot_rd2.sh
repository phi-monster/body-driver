#!/usr/bin/env bash
# RoboDojo 装机 · 续装(跳过 system/conda 两步)。
# 🔴 为什么要跳:vast 镜像**自带**一个叫 `RoboDojo` 的环境,位置是 `/venv/RoboDojo`,
# 而不在 conda 默认的 `~/miniconda3/envs/` 下。装机脚本因此两头不着:
#   `conda env list | grep "^RoboDojo "` 看得见它 ⇒ **跳过创建**
#   `source ~/miniconda3/bin/activate RoboDojo` 在 envs/ 下找不到 ⇒ **激活失败 ⇒ 整脚本 exit 1**
# 修法:把它软链进 envs/(`ln -sfn /venv/RoboDojo ~/miniconda3/envs/RoboDojo`),
# 再从 `base_deps` 续装 —— 那之后的步骤只是**尽力**激活、不会硬失败。
set -u
export DEBIAN_FRONTEND=noninteractive
export OMNI_KIT_ACCEPT_EULA=YES
ln -sfn /venv/RoboDojo "$HOME/miniconda3/envs/RoboDojo"
cd /root/RoboDojo || exit 1
bash scripts/install.sh --from base_deps
echo "=== INSTALL EXIT $? ==="
