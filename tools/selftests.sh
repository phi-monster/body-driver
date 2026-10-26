#!/usr/bin/env bash
# Every self test of a built driver, each alone in its own process and named exactly, several at a
# time, so the gates take minutes rather than the length of the whole suite run in one process.
#
#   tools/selftests.sh BIN_DIR [JOBS] [LIMIT_SECONDS]
#
# BIN_DIR holds selftest, relative to driver/ (bin, or bin-asan built with BD_BUILD=asan) or absolute;
# the environment passes through (ASAN_OPTIONS included). JOBS defaults to the machine's processors,
# LIMIT_SECONDS to the box's limit (tools/boxtest.sh). A name that is also a prefix of others runs
# only itself (a filter ending in $). Every test that fails or exceeds the limit is printed with what
# it printed; the last line counts them. Exit status 0 when every test passes. With BD_KEEP_FAILED
# set to a directory, every failing test's whole log is copied there (the run's own are deleted with
# it: robot.boot.step failed once in a box suite on 10-26 and passed 86 times alone, its log gone).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$1"
JOBS="${2:-$(sysctl -n hw.ncpu 2>/dev/null || nproc)}"
LIMIT="${3:-1200}"
OUT=$(mktemp -d "${TMPDIR:-/tmp}/bd_selftests.XXXXXX")
trap 'rm -rf "$OUT"' EXIT
# A registration may break its line before the name (Register\n  ("alignment.affine", ...)): a one-line
# grep missed the 26 alignment tests of 504 (10-21).
find "$ROOT/driver" -name '*.ad[sb]' -not -path '*/obj/*' -print0 \
  | xargs -0 perl -0777 -ne 'print "$1\n" while /\bRegister\s*\(\s*"([A-Za-z0-9_.]+)"/g' | sort -u > "$OUT/names"
(cd "$ROOT/driver" && xargs -P "$JOBS" -I{} sh -c \
   'timeout "$2" "$0/selftest" "{}\$" > "$1/{}.log" 2>&1; echo "{} $?"' "$BIN" "$OUT" "$LIMIT" \
   < "$OUT/names") > "$OUT/results"
awk '$2 != 0' "$OUT/results" | sort > "$OUT/bad"
while read -r name status; do
  if [ "$status" = 124 ]; then echo "TIMEOUT  $name (over $LIMIT s)"; else echo "FAIL     $name"; fi
  if [ -n "${BD_KEEP_FAILED:-}" ]; then mkdir -p "$BD_KEEP_FAILED" && cp "$OUT/$name.log" "$BD_KEEP_FAILED/"; fi
  grep -v '^\[' "$OUT/$name.log" | grep -v '^pass\|passed,' | head -5 | sed 's/^/         /'
done < "$OUT/bad"
echo "$(awk '$2 == 0' "$OUT/results" | wc -l | tr -d ' ') of $(wc -l < "$OUT/results" | tr -d ' ') passed"
[ ! -s "$OUT/bad" ]
