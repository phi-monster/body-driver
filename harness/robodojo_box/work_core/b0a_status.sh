#!/bin/bash
# One screen of B0a: each run's beats, phase, and the beat each phase began at; the fakes; disk.
date +%T
for N in B0a_1 B0a_2; do
  L=/root/runs/$N/driver.log
  [ -f "$L" ] || { echo "$N: no log"; continue; }
  last=$(grep -o "^@[0-9]*" "$L" | tail -1)
  rec=$(grep -m1 "boot: the groups as recognized" "$L" | grep -o "^@[0-9]*")
  swp=$(grep -m1 "boot: sweeping arm" "$L" | grep -o "^@[0-9]*")
  fit=$(grep -m1 "kinematics: arm . fitted\b\|the arms fitted in [1-9]" "$L" | grep -o "^@[0-9]*")
  hand=$(grep -m1 "hand: closer group .* from " "$L" | grep -o "^@[0-9]*")
  done_=$(grep -m1 "hand: measured\|the body as measured" "$L" | grep -o "^@[0-9]*")
  echo "=== $N: at $last | recognized $rec | sweep $swp | fitted $fit | hand $hand | done $done_"
  grep "boot: arm . swept in\|kinematics: arm . \(not \)\?fitted (stage\|boot: group . pushed .* ways at" "$L" | tail -3 | cut -c1-150
  tail -1 "$L" | cut -c1-120
done
for F in B0aF1 B0aF2; do
  L=/root/fakeruns/$F/driver.log
  [ -f "$L" ] || continue
  echo "=== $F: at $(grep -o "^@[0-9]*" "$L" | tail -1) | $(grep "boot:" "$L" | tail -1 | cut -c1-100) | $(tail -1 /root/fakeruns/$F/fake.log 2>/dev/null | cut -c1-60)"
done
df -h /root | tail -1 | awk '{print $4" free"}'
