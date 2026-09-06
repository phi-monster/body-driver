#!/usr/bin/env bash
# 装驱动:三道棘轮 → 离线自检 → 证明监视器/备份 → 编译 → 装到 ~/.local/bin/bl-calibrate(名字沿用,箱上脚本不用改)。
set -eu
ROOT="$(cd "$(dirname "$0")" && pwd)"
bash "$ROOT/check_purity.sh"
bash "$ROOT/check_constants.sh"
bash "$ROOT/check_freedom.sh"
cd "$ROOT/driver"
command -v alr >/dev/null || export PATH="$HOME/.alire/bin:$HOME/alire/bin:/root/alire/bin:$PATH"
command -v alr >/dev/null || { echo "need Alire (alr) with gnat_native + gprbuild: https://alire.ada.dev"; exit 1; }
alr -n build
./bin/selfcheck
if alr -n exec -- which gnatprove >/dev/null 2>&1 && alr -n exec -- gnatprove --version >/dev/null 2>&1; then
  echo "== SPARK: proving monitor + backup =="
  alr -n exec -- gnatprove -P body_driver.gpr --level=2 -j8 -u monitor.adb -u backup.adb --report=fail || { echo "🔴 proof failed"; exit 1; }
  grep -E "^Total" obj/gnatprove/gnatprove.out || true
else
  echo "== SPARK: gnatprove not runnable here (prove on a machine that has it) =="
fi
mkdir -p "$HOME/.local/bin"
rm -f "$HOME/.local/bin/bl-calibrate"
cp bin/body_driver "$HOME/.local/bin/bl-calibrate"
echo "== installed: $HOME/.local/bin/bl-calibrate ($(md5sum "$HOME/.local/bin/bl-calibrate" 2>/dev/null | cut -c1-12 || md5 -q "$HOME/.local/bin/bl-calibrate" | cut -c1-12)) =="
