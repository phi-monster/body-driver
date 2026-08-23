#!/usr/bin/env bash
# RoboDojo 装机(官方路径)。**这个文件就是复现命令** —— 上一台箱销毁时才发现
# RoboDojo 的安装步骤只在箱上、没进库,这次先写下来。
# 前置(租机时就要满足,事后补不了):
#   · 驱动 **535.129–580**(≥581 的 Vulkan 回归会让 IsaacSim 在开场景时段错误)
#   · `NVIDIA_DRIVER_CAPABILITIES=all`(镜像自带)
#   · Vulkan ICD **只留一份**(重复 ⇒ IsaacSim 直接 abort)
set -u
export DEBIAN_FRONTEND=noninteractive
export OMNI_KIT_ACCEPT_EULA=YES
cd /root/RoboDojo || exit 1
bash scripts/install.sh -i
echo "=== INSTALL EXIT $? ==="
