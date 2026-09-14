#!/usr/bin/env bash
# 装驱动:三道棘轮 → 离线自检 → 证明监视器/备份 → 编译 → 装到 ~/.local/bin/bl-calibrate(名字沿用,箱上脚本不用改)。
set -eu
ROOT="$(cd "$(dirname "$0")" && pwd)"
bash "$ROOT/check_purity.sh"
bash "$ROOT/check_constants.sh"
bash "$ROOT/check_freedom.sh"
bash "$ROOT/check_gates.sh"
cd "$ROOT/driver"
command -v alr >/dev/null || export PATH="$HOME/.alire/bin:$HOME/alire/bin:/root/alire/bin:$PATH"
command -v alr >/dev/null || { echo "need Alire (alr) with gnat_native + gprbuild: https://alire.ada.dev"; exit 1; }
alr -n build
./bin/selfcheck
# 🔴 "证明器自己跑不起来" ≠ "代码没通过证明" —— 崩掉的工具什么也没证明,两件事必须分开报。
# `gnatprove --version` 不调后端就能答(FSF 16.1.0),所以它过了【不代表】证明跑得动:
# 实测这台箱子上后端 gnatwhy3 需要 GLIBC_2.38 而系统更旧,一跑就是 GNAT BUG DETECTED,
# 旧写法把它当成 proof failed 直接 exit 1 —— 于是二进制永远装不上,而原因和代码毫无关系。
if alr -n exec -- which gnatprove >/dev/null 2>&1 && alr -n exec -- gnatprove --version >/dev/null 2>&1; then
  echo "== SPARK: proving monitor + backup =="
  set +e
  PROOF_OUT=$(alr -n exec -- gnatprove -P body_driver.gpr --level=2 -j8 -u monitor.adb -u backup.adb --report=fail 2>&1)
  PROOF_RC=$?
  set -e
  echo "$PROOF_OUT" | tail -20
  if [ "$PROOF_RC" -ne 0 ]; then
    if echo "$PROOF_OUT" | grep -qE "GNAT BUG DETECTED|GLIBC_|error while loading shared libraries|Executable not found in PATH"; then
      echo "🟡 SPARK:证明器【自己启动不了】(不是证明没过)—— 这台机器证不了,换一台证。"
      echo "🟡 本次安装不含证明结论;监视器/备份的行为仍由离线自检钉着(selfcheck 已在上一步全过)。"
    else
      echo "🔴 proof failed"; exit 1
    fi
  else
    grep -E "^Total" obj/gnatprove/gnatprove.out || true
  fi
else
  echo "== SPARK: gnatprove not runnable here (prove on a machine that has it) =="
fi
mkdir -p "$HOME/.local/bin"
rm -f "$HOME/.local/bin/bl-calibrate"
cp bin/body_driver "$HOME/.local/bin/bl-calibrate"
echo "== installed: $HOME/.local/bin/bl-calibrate ($(md5sum "$HOME/.local/bin/bl-calibrate" 2>/dev/null | cut -c1-12 || md5 -q "$HOME/.local/bin/bl-calibrate" | cut -c1-12)) =="
