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
LIB=slow/target/debug/libbody_layer.a

if [ ! -f "$LIB" ]; then
  LIB=slow/target/release/libbody_layer.a
fi
if [ ! -f "$LIB" ]; then
  echo "FAIL: no built library found. Build it first:"
  echo "  cd slow && cargo build --features fast"
  exit 2
fi

# Declared: every `bl_*(` that appears at the start of a declaration line in the header.
declared=$(grep -oE '\bbl_[a-z_]+\(' "$HDR" | tr -d '(' | sort -u)

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
