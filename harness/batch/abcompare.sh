#!/bin/bash
# abcompare.sh A B: per boot in both rounds, wall seconds, beats, the steps' beats, fits and their seconds, the arms' sweeps
for n in $(ls -d /root/runs/$1/*/ | xargs -n1 basename); do
  for r in $1 $2; do
    R=/root/runs/$r/$n; L=$R/driver.log; [ -f $L ] || continue
    s=$(date -d "$(grep ^start $R/meta.txt | cut -d' ' -f2)" +%s); now=$(stat -c %Y $L)
    b=$(tail -1 $L | grep -oE '^@[0-9]+' | tr -d @)
    p2=$(grep -m1 -oE '^@[0-9]+ \[robot\] boot: the body is kept in .* after sweeping' $L | grep -oE '^@[0-9]+')
    kin=$(grep -oE 'kinematics [0-9.]+\)' $L | grep -oE '[0-9.]+' | awk '{s+=$1} END {printf "%.0f", s}')
    nf=$(grep -cE 'kinematics: arm [0-9]+ (fitted from|not fitted)' $L)
    sw=$(grep -E 'boot: arm [0-9]+ swept' $L | sed -E 's/^@[0-9]+ \[robot\] boot: arm ([0-9]+) swept in ([0-9]+) passes of ([0-9]+) rounds, /a\1:\2p /' | tr '\n' ' ')
    tl=$(grep -oE 'order of the joints along arm [0-9]+, [0-9 ]+, is (NOT )?told' $L | tail -2 | sed -E 's/order of the joints along arm ([0-9]+), ([0-9 ]+), is NOT told/a\1 \2?/; s/order of the joints along arm ([0-9]+), ([0-9 ]+), is told/a\1 \2!/' | tr '\n' ' ')
    printf "%-5s %-4s %5ss @%-6s step2 %-7s fits %3s %5ss | %s| %s\n" $n $r $((now-s)) "$b" "${p2:--}" "$nf" "$kin" "$sw" "$tl"
  done
done
