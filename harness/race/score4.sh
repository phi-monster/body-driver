#!/bin/bash
# score4.sh RUN: steps 4-5: every closer channel pushed in its eye made lobes with tips at both readings, no hand left
# unmade; and the hands' sizes and depth measured or not.
R=$1; L=$R/driver.log; N=$(basename $R)
beat=$(grep -oE "^@[0-9]+" $L | tail -1); m=$(grep -c "the body as measured:" $L)
ch=$(grep -oE "hand: closer group [0-9]+ channel [0-9]+ in eye [0-9]+: +[0-9]+ lobes" $L | sort -u)
nch=$(echo "$ch" | grep -c lobes); with=$(echo "$ch" | grep -vc ": *0 lobes"); [ -z "$ch" ] && { nch=0; with=0; }
unmade=$(grep -c "no hand is made of it" $L)
tips=$(grep -cE "hand: closer group [0-9]+ channel [0-9]+ in eye [0-9]+: lobe [0-9]+: .*tip \(" $L)
sizes=$(grep -E "hand [0-9]+ sizes:" $L | tail -3 | grep -oE "(width|thickness|depth) [a-z0-9.]+" | sort | uniq -c | tr -s ' ' | tr '\n' ',')
ok=FAIL; [ $m -gt 0 ] && [ $nch -gt 0 ] && [ $with = $nch ] && [ $unmade = 0 ] && ok=PASS
grep -qE "^arm [0-9]+ ends in" $R/body.txt && grep -qvE "ends in HOOK" <(grep -E "^arm [0-9]+ ends in" $R/body.txt) || ok="NONE(no closer)"
echo "$(date +%H:%M) $N step4 $ok channels with lobes $with/$nch, hands unmade $unmade, tip lines $tips, sizes [$sizes] $beat"
