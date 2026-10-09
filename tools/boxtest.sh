#!/usr/bin/env bash
# Every self test on the box, the x86-64 Linux the driver runs on, each test alone in its own process.
#
#   tools/boxtest.sh [COMMIT] [LIMIT_SECONDS]      (defaults: HEAD, 1200)
#
# The Mac gates (tools/check.sh) cannot see what differs on the box: the order in which a call's
# actuals are evaluated, fused multiply-adds, libm. So COMMIT is built on the box in a directory of
# its own under /root/work/core/boxtest (two runs never share one), from a git archive, with the
# alire state kept in /root/work/core/deps. Every registered test then runs alone, named exactly (a
# name that is also a prefix of others runs only itself), under its own time limit, sixteen at a
# time, so one test that does not end cannot hide the others. It prints every test that fails or
# exceeds the limit, with what the test printed, and a count; the directory is removed at the end.
# Exit status 0 when every test passes.
#
# The tests run detached on the box and are only polled from here: a connection held open for the
# whole run was cut under load ("Operation timed out"), and merge.sh took a passing merge back out
# for it. Box tests queue on one lock, /root/q/boxtest.lock, so the merges and the paths never run
# two at once and no test is pushed over its limit by another run's load.
set -u
set -o pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
COMMIT="${1:-HEAD}"
LIMIT="${2:-1200}"
BOX="${BOX:-vast5d}"
SSH=(ssh -o ConnectTimeout=20 -o ServerAliveInterval=30 -o ServerAliveCountMax=20)
quiet() { grep -v "Welcome\|Have fun\|authentication" || true; }
HASH=$(git -C "$ROOT" rev-parse --short "$COMMIT") || exit 2
D="/root/work/core/boxtest/$HASH-$$-$(date +%s)"
NAMES=$(git -C "$ROOT" grep -h -o -E 'Register \("[A-Za-z0-9_.]+"' "$COMMIT" -- driver \
        | sed -E 's/Register \("//; s/"$//' | sort -u)
COUNT=$(echo "$NAMES" | wc -l | tr -d ' ')
echo "building $HASH on $BOX in $D for $COUNT tests"
# The sources go up as one file, sent again while the connection fails (ssh's own status 255; a
# connection held for a whole build was cut under load, and merge.sh took a passing merge back out
# for a build that never started), and the build runs detached and is polled, like the tests.
ARCHIVE=$(mktemp)
trap 'rm -f "$ARCHIVE"' EXIT
git -C "$ROOT" archive --format=tar "$COMMIT" driver docs | zstd -q -c > "$ARCHIVE" || exit 2
sent=0
for try in 1 2 3 4 5 6; do
  if "${SSH[@]}" "$BOX" "mkdir -p '$D' && cat > '$D/sources.tar.zst'" < "$ARCHIVE" 2>/dev/null; then sent=1; break; fi
  sleep 30
done
[ "$sent" = 1 ] || { echo "the sources could not be sent to $BOX"; exit 3; }
#  Started once: a retry after a cut connection finds build.started and starts no second build.
until "${SSH[@]}" "$BOX" "cd '$D' && { mkdir build.started 2>/dev/null || exit 0; } && setsid nohup sh -c 'zstd -dc sources.tar.zst | tar -x && cp -r /root/work/core/deps/alire driver/ && cd driver && PATH=/root/alire/bin:\$HOME/.alire/bin:\$PATH nice -n 5 alr -n build > ../build.log 2>&1; echo \$? > ../build.status' > /dev/null 2>&1 < /dev/null &" 2>/dev/null; do sleep 30; done
until "${SSH[@]}" "$BOX" "test -f '$D/build.status'" 2>/dev/null; do sleep 30; done
if ! "${SSH[@]}" "$BOX" "[ \"\$(cat '$D/build.status')\" = 0 ] && ! grep -E ': (error|warning)[: ]' '$D/build.log'" 2>&1 | quiet; then
  echo "the build on the box failed: $D/build.log"; exit 3
fi
# The runner waits its turn on the lock, then writes results.txt and, last, finished.
RUNNER=$(mktemp)
trap 'rm -f "$ARCHIVE" "$RUNNER"' EXIT
cat > "$RUNNER" <<'EOF'
#!/bin/sh
cd "$1/driver" && mkdir -p ../out
exec 9>/root/q/boxtest.lock
flock 9
xargs -P 16 -I{} sh -c 'timeout "$0" ./bin/selftest "{}\$" > ../out/{}.log 2>&1; echo "{} $?"' "$2" < ../names > ../results.txt
awk '$2 != 0' ../results.txt | sort > ../bad.txt
touch ../finished
EOF
"${SSH[@]}" "$BOX" "cat > '$D/run.sh'" < "$RUNNER" 2>&1 | quiet
echo "$NAMES" | "${SSH[@]}" "$BOX" "cat > '$D/names'" 2>&1 | quiet
"${SSH[@]}" "$BOX" "cd '$D' && setsid nohup sh run.sh '$D' '$LIMIT' > runner.log 2>&1 < /dev/null &" 2>&1 | quiet
until "${SSH[@]}" "$BOX" "test -f '$D/finished'" 2>/dev/null; do
  if "${SSH[@]}" "$BOX" "test -d '$D'" 2>/dev/null; then sleep 30; else
    "${SSH[@]}" "$BOX" true 2>/dev/null && { echo "the box test's directory is gone: $D"; exit 4; }
    sleep 30
  fi
done
"${SSH[@]}" "$BOX" "cd '$D/driver' && \
  while read -r name status; do \
    if [ \"\$status\" = 124 ]; then echo \"TIMEOUT  \$name (over $LIMIT s)\"; else echo \"FAIL     \$name\"; fi; \
    grep -v '^\[' ../out/\$name.log | grep -v '^pass\|passed,' | head -5 | sed 's/^/         /'; \
  done < ../bad.txt; \
  echo \"\$(awk '\$2 == 0' ../results.txt | wc -l) of \$(wc -l < ../results.txt) passed\"; \
  ok=0; [ -s ../bad.txt ] && ok=1; [ \$(wc -l < ../results.txt) -eq $COUNT ] || ok=1; cd /; rm -rf '$D'; exit \$ok" 2>&1 | quiet
exit "${PIPESTATUS[0]}"
