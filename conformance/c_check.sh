#!/usr/bin/env bash
# c_check.sh -- build and run the C client. THE compile is half the check.
#
# 🔴 A signature that drifts from the header breaks this BUILD. That is the property a hand-mirrored
# ctypes binding cannot have: a Python mirror that disagrees with the header reads shifted bytes and
# reports BL_OK, so every end-to-end claim made through it rests on the copy being right.
set -uo pipefail
cd "$(dirname "$0")/.."

LIB=$(ls -t slow/target/release/libbody_layer.dylib slow/target/release/libbody_layer.so 2>/dev/null | head -1)
if [ -z "${LIB:-}" ]; then
  echo "c_check: no cdylib built -- run: cargo build --release --manifest-path slow/Cargo.toml"
  exit 2
fi
# Same staleness guard as abi_check: validating an artifact older than the contract is the bug
# these scripts exist to prevent.
newest=$(ls -t abi/body_layer.h slow/src/*.rs 2>/dev/null | head -1)
if [ "$newest" -nt "$LIB" ]; then
  echo "c_check: FAIL -- $LIB is OLDER than $newest; rebuild first."
  exit 1
fi

CC=${CC:-cc}
OUT=$(mktemp -d)/c_client
echo "c_check: compiling against abi/body_layer.h, linking $LIB"
if ! $CC -std=c11 -Wall -Wextra -Werror -I abi conformance/c_client.c "$LIB" -lm -o "$OUT" 2>&1; then
  echo "c_check: FAIL -- the client does not COMPILE against the header."
  echo "         That is the check working: the contract and the caller disagree."
  exit 1
fi
echo "c_check: compiled clean (-Wall -Wextra -Werror)"
"$OUT"
rc=$?
[ $rc -eq 0 ] && echo "c_check: PASS" || echo "c_check: FAIL (client exit $rc)"
exit $rc
