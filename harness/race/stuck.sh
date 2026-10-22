#!/bin/bash
# stuck.sh: every live driver of the race whose models have been away (an estimate running) for more than 3000 beats,
# and each of its threads' processor use over 5 s and where it waits in the kernel (no debugger attaches on the box).
for P in $(ps -eo pid,args | awk '$2 ~ /\/root\/race\/runs\/[0-9]\/.*\/bin\/body_driver/ {print $1}'); do
  a=$(tr "\0" "\n" < /proc/$P/cmdline 2>/dev/null | grep -oE "^/root/race/runs/[0-9]/[^/]+" | head -1); L=$a/driver.log
  [ -f "$L" ] || continue
  last=$(grep -oE "^@[0-9]+" $L | tail -1 | tr -d @); back=$(grep "models are back" $L | tail -1 | grep -oE "^@[0-9]+" | tr -d @)
  [ $(( ${last:-0} - ${back:-0} )) -gt 3000 ] || continue
  echo "== $P $(basename $a): last @$last, models back last at @${back:-0}"
  for t in /proc/$P/task/*; do echo "$(basename $t) $(cut -d' ' -f14,15 $t/stat | awk '{print $1+$2}') $(cat $t/wchan)"; done | sort > /dev/shm/s1.$P
  sleep 5
  for t in /proc/$P/task/*; do echo "$(basename $t) $(cut -d' ' -f14,15 $t/stat | awk '{print $1+$2}')"; done | sort > /dev/shm/s2.$P
  join /dev/shm/s1.$P /dev/shm/s2.$P | awk '{printf "  tid %s: %d ticks in 5 s, waits in %s\n", $1, $4-$2, $3}'
  rm -f /dev/shm/s1.$P /dev/shm/s2.$P
done
