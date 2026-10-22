#!/bin/bash
# score1.sh RUN: step 1 against the truth (body.txt). Each true group's role from its name (joints: ARM; a closer or
# finger: CLOSER; base, motors: CARRIER; frequency, env_idx: INERT; a humanoid's one joints group: not judged), as
# (channels, role) pairs, against the pairs the driver recognized; each eye's role from where the truth mounts it
# (on an arm: ARM_CARRIED, on the world: WORLD_FIXED, elsewhere: CARRIER_CARRIED) against the roles recognized.
R=$1; L=$R/driver.log; B=$R/body.txt; N=$(basename $R)
blk=$(awk '/the groups as recognized:/{f=1;next} f&&/^$/{exit} f' $L 2>/dev/null)
beat=$(grep -oE "^@[0-9]+" $L | tail -1)
if [ -z "$blk" ]; then echo "$(date +%H:%M) $N step1 UNFINISHED $beat $(grep -m1 -oE "boot failed.*" $L | cut -c1-100)"; exit; fi
human=$(head -1 $B | grep -c humanoid)
tg=$(grep -E "^group " $B | perl -ne 'next unless /^group (\S+): (\d+) channels/; my ($n,$c)=($1,$2);
  my $r = $n =~ /^(base|motors)$/ ? "CARRIER" : $n =~ /^(frequency|env_idx)$/ ? "INERT" : $n =~ /_(closer|finger\d+)$/ ? "CLOSER"
        : $n =~ /^arm\d+_joints$/ ? "ARM" : $n eq "joints" ? "?" : "?"; print "$c$r\n"' | sort | tr '\n' ' ')
sg=$(echo "$blk" | perl -ne 'print "$1$3\n" if /^group \d+: (\d+) values, (not )?commandable, ([A-Z_]+)/' | sort | tr '\n' ' ')
te=$(grep -E "^camera " $B | perl -ne 'print((/ on arm\d+\// ? "ARM_CARRIED" : / on the world/ ? "WORLD_FIXED" : "CARRIER_CARRIED"), "\n")' | sort | tr '\n' ' ')
se=$(echo "$blk" | perl -ne 'print "$1\n" if /^eye \d+: image lag [^,]+, ([A-Z_]+)/' | sort | tr '\n' ' ')
tq=$(echo "$tg" | tr ' ' '\n' | grep -v "?" | tr '\n' ' ')
ok=PASS
[ "$human" = 0 ] && [ "$tg" != "$sg" ] && ok=FAIL
[ "$te" != "$se" ] && ok=FAIL
[ "$human" = 1 ] && [ $ok = PASS ] && ok="PASS(eyes; groups not judged)"
echo "$(date +%H:%M) $N step1 $ok groups true [$tg] seen [$sg] eyes true [$te] seen [$se] $beat"
