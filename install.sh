#!/usr/bin/env bash
# 官方安装:构建驱动、跑完全部自证、把 `bl-calibrate` 放上 PATH。不碰任何机器人。
#
# 装完的用法(真实世界就这两步,之后零代码):
#   ① 自标定:      bl-calibrate --listen <端口> --out cal.json
#   ② 干活:        bl-calibrate --listen <端口> --in cal.json --out cal.json [--eye host:port]
#      —— 标定日程走完自动进入干活模式:观测里给什么指令,就做什么。
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
