#!/usr/bin/env bash
# RoboDojo 装机 · 续装第三段(子模块已用浅克隆自己拉完,见 /root/subs.sh)。
# 从 `isaacsim` 起,剩下 isaacsim / isaaclab / curobo 三步。
set -u
export DEBIAN_FRONTEND=noninteractive
export OMNI_KIT_ACCEPT_EULA=YES
cd /root/RoboDojo || exit 1
bash scripts/install.sh --from isaacsim
echo "=== INSTALL EXIT $? ==="
