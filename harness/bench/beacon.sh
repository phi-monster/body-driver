#!/usr/bin/env bash
# beacon.sh SEED[:ARM]... : step 3's closer-flick route (driver/tools/driver-robot-beacon_bench) on the named seeds of the
# fake robot's table bodies, one process a seed, each under the two-minute cap (cap.sh). Prints every strategy's line:
# how many poses it took, how many the eye saw, and, when the fit told the eye, after how many sightings and how far the
# lens and the place are from the body's (the fit's claimed sigmas beside). BEACON_PLACE=1 puts the other arms into the
# world instead (the table both arms' eyes saw is the link); BEACON_FAST=1 runs only the strategies that reach the joints'
# ends; BEACON_NOISE, BEACON_ROUNDS, BEACON_UNIT, BEACON_UNIT2 set the flick centre's noise, the draws of it, and the
# arms' units. BIN names the bench (default the repository's).
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN=${BIN:-$HERE/../../driver/bin/driver-robot-beacon_bench}
OUT=${OUT:-$(mktemp -d "${TMPDIR:-/tmp}/beacon.XXXXXX")}
mkdir -p "$OUT"
export BD_WORKERS=${BD_WORKERS:-1}
T0=$(perl -MTime::HiRes=time -e 'printf "%.1f", time')
printf '%s\n' "$@" | xargs -P 10 -L 1 bash -c '"'"$HERE"'"/cap.sh "'"$BIN"'" "$0" > "'"$OUT"'"/"$0".txt 2>&1; echo $? > "'"$OUT"'"/"$0".rc' 2>/dev/null
T1=$(perl -MTime::HiRes=time -e 'printf "%.1f", time')
for f in "$OUT"/*.rc; do rc=$(cat "$f"); [ "$rc" = 0 ] || echo "$(basename "$f" .rc): killed at the cap or failed (exit $rc)"; done
cat "$OUT"/*.txt | grep -E "^seed" | cut -c1-260
echo "wall: $(perl -e "printf '%.1f', $T1 - $T0") s; lines in $OUT"
