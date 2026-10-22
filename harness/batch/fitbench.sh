#!/bin/bash
# fitbench.sh BIN OUT THREADS JOBS DUMP...: every dumped fit replayed by BIN's driver-robot-fit_replay, JOBS at once,
# each with THREADS threads; per dump a line: its name, seconds, rounds, the order line and the fit line.
BIN=$1; OUT=$2; T=$3; J=$4; shift 4
mkdir -p $OUT.d
one() {
  f=$1; n=$(echo $f | sed -E 's#.*/r7./##; s#/#_#g')
  s=$(date +%s.%N)
  BD_WORKERS=$((T-1)) $BIN/driver-robot-fit_replay $f > $OUT.d/$n.log 2>&1
  e=$(date +%s.%N)
  printf "%s\t%.1f\t%s\t%s\t%s\n" "$n" "$(echo "$e - $s" | bc)" "$(grep -oE "and [0-9]+ rounds" $OUT.d/$n.log | awk '{s+=$2} END {print s+0}')" \
    "$(grep -oE "order of the joints along arm [0-9]+, [0-9 ]+, is (NOT )?told" $OUT.d/$n.log | tail -1)" \
    "$(grep -oE "(fitted from [0-9]+ sightings of [0-9]+ keyframes.*focal [0-9.]+ x [0-9.]+ px|not fitted \(stage [0-9]\))" $OUT.d/$n.log | tail -1 | grep -oE "focal [0-9.]+ x [0-9.]+ px|not fitted \(stage [0-9]\)")"
}
export -f one; export BIN OUT T
printf "%s\n" "$@" | xargs -P $J -I{} bash -c 'one {}' > $OUT
echo done >> $OUT
