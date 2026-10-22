#!/bin/bash
# score3.sh RUN: step 3: every fixed eye told, its lens within 2% of the true fixed camera's fx (its place against the
# truth is not read yet).
R=$1; L=$R/driver.log; B=$R/body.txt; N=$(basename $R)
beat=$(grep -oE "^@[0-9]+" $L | tail -1); placed=$(grep -c "after placing" $L)
tf=$(grep "^camera [^ ]* on the world" $B | grep -oE "fx [0-9.E+-]+" | awk '{printf "%.2f ", $2}')
ok=1; out=""; n=0
for e in $(grep -oE "flows: fixed eye [0-9]+ from" $L | grep -oE "[0-9]+" | sort -u); do
  n=$((n + 1)); l=$(grep -E "flows: fixed eye $e from" $L | tail -1)
  told=$(echo "$l" | grep -c ": told"); f=$(echo "$l" | grep -oE "focal [0-9.]+" | grep -oE "[0-9.]+" | head -1)
  d=$(for t in $tf; do awk -v x=$f -v t=$t 'BEGIN{a=100*(x-t)/t; printf "%.1f %.1f\n", (a<0?-a:a), a}'; done | sort -n | head -1)
  set -- $d; [ "$told" = 1 ] && awk -v m=$1 'BEGIN{exit !(m <= 2)}' || ok=0
  out="$out eye$e told=$told focal ${2}%;"
done
nt=$(echo $tf | wc -w); [ $n -lt $nt ] && ok=0; [ $placed = 0 ] && ok=0
echo "$(date +%H:%M) $N step3 $([ $ok = 1 ] && echo PASS || echo FAIL) fixed eyes $n/$nt$out $beat"
