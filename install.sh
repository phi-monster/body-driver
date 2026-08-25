#!/usr/bin/env bash
# 官方安装:构建驱动、跑完全部自证、把它放上 PATH。不碰任何机器人。
#
# 🔴🔴 **装完只有一个用法,没有第二步**(owner 2026-08-25 定):
#
#   bl-calibrate --listen <端口> --out 身体.json [--eye host:port]
#
# **下命令就去干。** 观测里给什么指令,就做什么。
# 干到需要某个身体量而手上没有 ⇒ **它自己动一下去问**,量完接着干,你看不见这一步。
#
# 🔴 **没有"自标定阶段"了。** 旧版是开机先按一张表把 15 个量挨个量完才准干活;
# 代价照记:N128–N143 共 **14 炮,进入干活模式 0 次** —— 用户让它拿东西,
# 它先坐下来量自己四十分钟;而评测里每 200 步打断一次,于是**永远量不完,也永远不干活**。
# 而且「装机量一次、永久有效」本身是个手填的假设:换只手、挂个武器,
# 爪宽和指尖长**当场全变**,时间型的过期管不住它。
#
# ⇒ 缺什么当场量,量完存进 `身体.json`;下次开机装回来,**用到就核对,对不上就重量**。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
command -v cargo >/dev/null || { echo "need a Rust toolchain: https://rustup.rs"; exit 1; }

echo "== driver: no robot names, no Python in the tree =="
bash "$ROOT/check_purity.sh"

# 🔴 写死的身体常数 —— 棘轮式,只许降不许升。见该脚本文件头记的三条代价。
echo "== driver: no hand-filled body constants (ratchet) =="
bash "$ROOT/check_constants.sh"

echo "== driver: build + test =="
for c in slow contact-set contact-gen contact-exec point-gen selfcal; do
  ( cd "$ROOT/$c" && cargo test --release --quiet )
done

# The fast face is Ada/SPARK. Building it needs GNAT; if that is not here we say so out loud
# rather than skipping quietly -- a safety face that silently did not build is worse than absent.
if command -v gprbuild >/dev/null; then
  echo "== driver fast face (Ada/SPARK): build =="
  ( cd "$ROOT/fast" && gprbuild -q -P body_layer_fast_lib.gpr )
  bash "$ROOT/conformance/ada_check.sh" || echo "   (ada_check reported a mismatch above)"
else
  echo "== driver fast face (Ada/SPARK): SKIPPED -- no gprbuild on PATH =="
  echo "   the limits/force-cap/watchdog/e-stop face is NOT built. Install GNAT to get it."
fi

echo "== plug: build + test =="
( cd "$ROOT/plug/ws" && cargo test --release --quiet && cargo build --release --quiet )

mkdir -p "$HOME/.local/bin"
cp "$ROOT/plug/ws/target/release/bl-calibrate" "$HOME/.local/bin/"
echo
echo "installed: $HOME/.local/bin/bl-calibrate"
echo "next, with your robot's controller running:   bl-calibrate --listen 9077 --out cal.json"
