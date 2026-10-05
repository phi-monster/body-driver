#!/usr/bin/env bash
# Every self test on the box, the x86-64 Linux the driver runs on, each test alone in its own process.
#
#   tools/boxtest.sh [COMMIT] [LIMIT_SECONDS]      (defaults: HEAD, 1200)
#
# The Mac gates (tools/check.sh) cannot see what differs on the box: the order in which a call's
# actuals are evaluated, fused multiply-adds, libm. So COMMIT is built on the box in
# /root/work/core/boxtest, from a git archive, with the alire state kept in /root/work/core/deps,
# and every registered test runs alone under its own time limit, sixteen at a time, so one test
# that does not end cannot hide the others. It prints every test that fails or exceeds the limit,
# with what the test printed, and a count. Exit status 0 when every test passes.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
COMMIT="${1:-HEAD}"
LIMIT="${2:-1200}"
BOX="${BOX:-vast5d}"
HASH=$(git -C "$ROOT" rev-parse --short "$COMMIT") || exit 2
NAMES=$(git -C "$ROOT" grep -h -o -E 'Register \("[A-Za-z0-9_.]+"' "$COMMIT" -- driver \
        | sed -E 's/Register \("//; s/"$//' | sort -u)
COUNT=$(echo "$NAMES" | wc -l | tr -d ' ')
echo "building $HASH on $BOX for $COUNT tests"
if ! git -C "$ROOT" archive --format=tar "$COMMIT" driver docs | zstd -q -c | ssh -o ConnectTimeout=20 "$BOX" '
    set -e; D=/root/work/core/boxtest; rm -rf "$D"; mkdir -p "$D"; cd "$D"
    zstd -dc | tar -x; cp -r /root/work/core/deps/alire driver/; cd driver
    PATH=/root/alire/bin:$HOME/.alire/bin:$PATH nice -n 5 alr -n build > ../build.log 2>&1
    ! grep -E " error" ../build.log'; then
  echo "the build on the box failed: /root/work/core/boxtest/build.log"; exit 3
fi
echo "$NAMES" | ssh -o ConnectTimeout=20 "$BOX" "cd /root/work/core/boxtest/driver && mkdir -p ../out && \
  xargs -P 16 -I{} sh -c 'timeout $LIMIT ./bin/selftest {} > ../out/{}.log 2>&1; echo \"{} \$?\"' \
  > ../results.txt; awk '\$2 != 0' ../results.txt | sort > ../bad.txt; \
  while read -r name status; do \
    if [ \"\$status\" = 124 ]; then echo \"TIMEOUT  \$name (over $LIMIT s)\"; else echo \"FAIL     \$name\"; fi; \
    grep -v '^\[' ../out/\$name.log | grep -v '^pass\|passed,' | head -5 | sed 's/^/         /'; \
  done < ../bad.txt; \
  echo \"\$(awk '\$2 == 0' ../results.txt | wc -l) of \$(wc -l < ../results.txt) passed\"; \
  [ ! -s ../bad.txt ]" 2>&1 | grep -v "Welcome\|Have fun\|authentication"
exit "${PIPESTATUS[1]}"
