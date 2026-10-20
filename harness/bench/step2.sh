#!/usr/bin/env bash
# step2.sh ARM... : step 2 of the boot (driver/tools/driver-robot-step2_bench) on the named arms of the fake robot's
# bodies, all at once, one process an arm with BD_WORKERS workers (default 1), each under the two-minute cap (cap.sh).
# An arm is SEED:ARM (the bench draws the channels' scramble itself) or RUN:SEED:ARM (the draw of that live run,
# $FAKE_RUNS/RUN/scramble.json, default ~/fakeruns, so its channels are the run's). Prints every arm's verdict against
# the truth, the tally and the wall time; the rounds are in OUT (default a fresh folder under $TMPDIR). BIN names the
# bench (default the repository's); BENCH_* settings pass through (BENCH_NEAR, BENCH_NOISE, BENCH_GRID default 24).
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN=${BIN:-$HERE/../../driver/bin/driver-robot-step2_bench}
RUNS=${FAKE_RUNS:-$HOME/fakeruns}
OUT=${OUT:-$(mktemp -d "${TMPDIR:-/tmp}/step2.XXXXXX")}
export BD_WORKERS=${BD_WORKERS:-1} BENCH_GRID=${BENCH_GRID:-24}
T0=$(perl -MTime::HiRes=time -e 'printf "%.1f", time')
for x in "$@"; do
  IFS=: read -r a b c <<< "$x"
  if [ -n "$c" ]; then echo "$a $b $c $RUNS/$a/scramble.json"; else echo "s$a $a $b -"; fi
done | xargs -P 10 -L 1 bash -c '
  if [ "$3" = - ]; then unset BENCH_SCRAMBLE; else export BENCH_SCRAMBLE=$3; fi
  "'"$HERE"'"/cap.sh "'"$BIN"'" $1:$2 > "'"$OUT"'"/$0_$2.txt 2>&1; echo $? > "'"$OUT"'"/$0_$2.rc' 2>/dev/null
T1=$(perl -MTime::HiRes=time -e 'printf "%.1f", time')
for f in "$OUT"/*.rc; do rc=$(cat "$f"); [ "$rc" = 0 ] || echo "$(basename "$f" .rc): killed at the cap or failed (exit $rc)"; done
grep -hE "^seed" "$OUT"/*.txt | cut -c1-200
echo "tally: $(grep -hE '^seed' "$OUT"/*.txt | grep -oE 'TOLD RIGHT|TOLD WRONG|untold, best right|untold, best wrong|NOT FITTED' | sort | uniq -c | tr '\n' ' ')"
echo "wall: $(perl -e "printf '%.1f', $T1 - $T0") s; rounds in $OUT"
