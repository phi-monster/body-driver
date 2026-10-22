#!/bin/bash
# score2.sh RUN: step 2 by the finish line (驱动重写2.md): every true arm with an eye fitted, its order told, its lens
# (fx and fy alike) within 2% of its eye's true fx, in at most 4000 beats an arm (the sweep's first beat to its end).
R=$1; L=$R/driver.log; B=$R/body.txt; N=$(basename $R)
ta=$(grep -c "with an eye of its own" $B); beat=$(grep -oE "^@[0-9]+" $L | tail -1)
t0=$(grep -m1 -oE "^@[0-9]+ \[robot\] boot: sweeping arm" $L | grep -oE "^@[0-9]+" | tr -d @)
t1=$(grep -m1 -oE "^@[0-9]+ \[robot\] boot: the body is kept in .* after sweeping" $L | grep -oE "^@[0-9]+" | tr -d @)
truth=$(grep -E "^camera [^ ]+ on arm[0-9]+/" $B | grep -oE "fx [0-9.E+-]+" | awk '{printf "%.2f ", $2}')
out=""; ok=1; n=0
for a in $(grep -oE "kinematics: arm [0-9]+ fitted from" $L | grep -oE "[0-9]+" | sort -u); do
  n=$((n + 1))
  read fx fy <<< "$(grep -E "kinematics: arm $a fitted from" $L | tail -1 | grep -oE "focal [0-9.]+ x [0-9.]+" | grep -oE "[0-9.]+" | tr '\n' ' ')"
  told=$(grep -E "order of the joints along arm $a, " $L | tail -1 | grep -c " is told")
  e=$(for t in $truth; do awk -v x=$fx -v y=$fy -v t=$t 'BEGIN{a=100*(x-t)/t; b=100*(y-t)/t; m=(a<0?-a:a); if ((b<0?-b:b)>m) m=(b<0?-b:b); printf "%.1f %.1f %.1f\n", m, a, b}'; done | sort -n | head -1)
  set -- $e; [ "$told" = 1 ] && awk -v m=$1 'BEGIN{exit !(m <= 2)}' || ok=0
  out="$out arm$a fx ${2}% fy ${3}% told=$told;"
done
[ $n != $ta ] && ok=0
per=""; if [ -n "$t0" ] && [ -n "$t1" ] && [ $ta -gt 0 ]; then per=$(( (t1 - t0) / ta )); [ $per -gt 4000 ] && ok=0; else ok=0; fi
[ $ta = 0 ] && ok=2
v=$([ $ok = 1 ] && echo PASS || ([ $ok = 2 ] && echo "NONE(no arm with an eye)" || echo FAIL))
echo "$(date +%H:%M) $N step2 $v arms $n/$ta beats an arm ${per:-unfinished}$out $beat"
