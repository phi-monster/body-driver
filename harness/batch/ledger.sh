#!/bin/bash
# ledger.sh ROUNDDIR...: where each boot's wall time went
for D in "$@"; do
for R in "$D"/*/; do
  n=$(basename $R); L=$R/driver.log; [ -f $L ] || continue
  s=$(date -d "$(grep ^start $R/meta.txt | cut -d' ' -f2)" +%s 2>/dev/null) || continue
  e=$(stat -c %Y $L); wall=$((e-s))
  beats=$(tail -1 $L | grep -oE '^@[0-9]+' | tr -d @)
  kept=$(grep -oE "models are back after [0-9]+ messages" $L | awk '{s+=$5} END {print s+0}')
  est=$(grep -oE "estimated from [0-9]+ beats in ?[0-9.]+ s" $L | grep -oE "in ?[0-9.]+ s" | grep -oE "[0-9.]+" | awk '{s+=$1} END {printf "%.0f", s}')
  lock=$(grep -oE "lock-in [0-9.]+, alignment" $L | awk '{s+=$2} END {printf "%.0f", s}')
  kin=$(grep -oE "kinematics [0-9.]+\)" $L | grep -oE "[0-9.]+" | awk '{s+=$1} END {printf "%.0f", s}')
  aln=$(grep -oE "alignment [0-9.]+, graph" $L | awk '{s+=$2} END {printf "%.0f", s}')
  nest=$(grep -c "estimated from" $L)
  fk=$(grep "a beat takes" $R/fake.log 2>/dev/null | tail -1 | grep -oE "render [0-9.E+-]+ ms, body [0-9.E+-]+ ms, driver [0-9.E+-]+" | awk '{printf "render %.1f body %.1f driver %.1f", $2, $5, $8}')
  p1=$(grep -m1 -oE "^@[0-9]+ \[robot\] boot: the body is kept in .* after recognizing" $L | grep -oE "^@[0-9]+")
  p2=$(grep -m1 -oE "^@[0-9]+ \[robot\] boot: the body is kept in .* after sweeping" $L | grep -oE "^@[0-9]+")
  p3=$(grep -m1 -oE "^@[0-9]+ \[robot\] boot: the body is kept in .* after placing" $L | grep -oE "^@[0-9]+")
  printf "%-6s wall %5ds beats %6s (kept back %6s) | %3d estimates %5ss: lock-in %5s align %4s kin %5s | fake/beat %s | step1 %s step2 %s step3 %s\n" \
    $n $wall "$beats" "$kept" $nest "$est" "$lock" "$aln" "$kin" "$fk" "${p1:--}" "${p2:--}" "${p3:--}"
done; done
