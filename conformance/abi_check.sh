#!/usr/bin/env bash
# abi_check.sh -- every function the header PROMISES must exist as a symbol in the built library.
#
# 🔴 WHY THIS EXISTS AS A SCRIPT AND NOT AS A NOTE IN A README.
#
# This repository has a recorded case of a module docstring promising a `--ref` positive control
# that the argument parser never implemented.  Everyone who read the docstring -- including the
# author -- believed the control had run.  It had not, and "three arms all read zero" could not be
# separated from "the pipeline is broken" for weeks.
#
# A header is a docstring with a linker attached.  The same failure is available to it, and the
# same fix applies: make a machine check the promise instead of a person.
#
# It also catches the reverse, which is quieter and worse: a symbol the library exports that the
# header never declared.  That is an undeclared entry point -- a way into the layer that nobody
# audited, in a layer whose entire value is that its entry points can be audited by reading one
# file.
#
# Usage:  ./conformance/abi_check.sh
# Exit 0 only if the header and the library agree in BOTH directions.

set -uo pipefail
cd "$(dirname "$0")/.."

HDR=abi/body_layer.h

# 🔴 CHECK THE ARTIFACT THAT WAS ACTUALLY BUILT, AND SAY WHICH ONE.
#
# This preferred `target/debug` and fell back to `target/release`, so a stale debug archive left
# over from an earlier build was validated in place of the one just produced -- and it reported a
# missing symbol that the fresh library exported. A checker that inspects the wrong artifact can
# fail for the wrong reason, and can just as easily PASS for the wrong reason, which is worse and
# quieter. That is precisely the failure mode this script was written to catch, in the script
# itself.
#
# Newest wins, the choice is printed, and an artifact older than the header or the sources is a
# hard failure rather than something to validate.
# The artifact that matters is the one a CONSUMER LOADS -- the cdylib, which is what
# `bind/python/body_layer.py` opens with ctypes. The static archive was checked instead, and under
# the release profile `nm` reports zero `bl_*` symbols from it, so the check silently graded a file
# nobody links against. Prefer the shared object; fall back to the archive only if there is none.
LIB=$(ls -t slow/target/*/libbody_layer.dylib slow/target/*/libbody_layer.so 2>/dev/null | head -1)
if [ -z "${LIB:-}" ]; then
  LIB=$(ls -t slow/target/*/libbody_layer.a 2>/dev/null | head -1)
fi
if [ -z "${LIB:-}" ] || [ ! -f "$LIB" ]; then
  echo "FAIL: no built library found. Build it first:"
  echo "  cd slow && cargo build --release"
  exit 2
fi
echo "abi_check: inspecting $LIB"
newest_src=$(ls -t "$HDR" slow/src/*.rs 2>/dev/null | head -1)
if [ -n "${newest_src:-}" ] && [ "$newest_src" -nt "$LIB" ]; then
  echo "FAIL: $LIB is OLDER than $newest_src -- rebuild before checking."
  echo "      Validating a stale artifact is the bug this script exists to prevent."
  exit 2
fi

# Declared: every `bl_*(` that appears at the start of a declaration line in the header.
# 🔴 STRIP COMMENTS FIRST.  This grepped the raw header, so a `bl_predict()` mentioned inside a
# comment -- explaining why that function deliberately does NOT exist -- was read as a promise, and
# the check reported the library missing an entry point nobody ever declared.
#
# A conformance check that misfires on prose is worse than none: the first false alarm is
# investigated, the third is ignored, and by the fifth nobody reads the output at all.  The header
# is documentation AND contract in one file, so the contract has to be extracted from the parts that
# are contract.
declared=$(sed -E 's@/\*([^*]|\*+[^*/])*\*+/@@g' "$HDR" |
           sed -E '\@^[[:space:]]*/?\*@d; \@^[[:space:]]*//@d' |
           grep -oE '\bbl_[a-z_]+\(' | tr -d '(' | sort -u)

# Exported: every `bl_*` symbol the archive actually defines (T = text, uppercase = defined).
exported=$(nm -g "$LIB" 2>/dev/null | awk '$2 ~ /^[TDS]$/ {print $3}' \
           | sed 's/^_//' | grep -E '^bl_[a-z_]+$' | sort -u)

fail=0

echo "== promised by $HDR but NOT exported =="
missing=$(comm -23 <(echo "$declared") <(echo "$exported"))
if [ -n "$missing" ]; then
  echo "$missing" | sed 's/^/  MISSING  /'
  fail=1
else
  echo "  (none)"
fi

echo "== exported but NOT declared in the header =="
# `bl_reason_str` / `bl_quantity_str` are declared; anything else undeclared is an unaudited door.
extra=$(comm -13 <(echo "$declared") <(echo "$exported"))
if [ -n "$extra" ]; then
  echo "$extra" | sed 's/^/  UNDECLARED  /'
  fail=1
else
  echo "  (none)"
fi

echo
if [ "$fail" -eq 0 ]; then
  echo "abi_check: PASS -- the header and the library agree in both directions"
  echo "  declared: $(echo "$declared" | wc -l | tr -d ' ')   exported: $(echo "$exported" | wc -l | tr -d ' ')"
else
  echo "abi_check: FAIL -- the contract and the artifact disagree"
fi
exit "$fail"
