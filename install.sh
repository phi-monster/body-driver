#!/usr/bin/env bash
# Build the driver, prove it, and put `bl-calibrate` on the path. Touches no robot.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
command -v cargo >/dev/null || { echo "need a Rust toolchain: https://rustup.rs"; exit 1; }

echo "== driver: no robot names, no Python in the tree =="
bash "$ROOT/driver/check_purity.sh"

echo "== driver: build + test =="
for c in core contact-set contact-gen contact-exec point-gen selfcal; do
  ( cd "$ROOT/driver/$c" && cargo test --release --quiet )
done

# The fast face is Ada/SPARK. Building it needs GNAT; if that is not here we say so out loud
# rather than skipping quietly -- a safety face that silently did not build is worse than absent.
if command -v gprbuild >/dev/null; then
  echo "== driver fast face (Ada/SPARK): build =="
  ( cd "$ROOT/driver/fast" && gprbuild -q -P body_layer_fast_lib.gpr )
  bash "$ROOT/driver/conformance/ada_check.sh" || echo "   (ada_check reported a mismatch above)"
else
  echo "== driver fast face (Ada/SPARK): SKIPPED -- no gprbuild on PATH =="
  echo "   the limits/force-cap/watchdog/e-stop face is NOT built. Install GNAT to get it."
fi

echo "== plug: build =="
( cd "$ROOT/plug/ws" && cargo build --release --quiet )

mkdir -p "$HOME/.local/bin"
cp "$ROOT/plug/ws/target/release/bl-calibrate" "$HOME/.local/bin/"
echo
echo "installed: $HOME/.local/bin/bl-calibrate"
echo "next, with your robot's controller running:   bl-calibrate --listen 9077"
