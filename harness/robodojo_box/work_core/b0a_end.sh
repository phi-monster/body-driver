#!/bin/bash
# Stops each B0a run once its boot has ended (the hand measured, or the brain phase begun: nothing more
# is measured after that and there is no brain to ask), so the runs do not idle to the 180-minute ceiling.
while :; do
  left=0
  for N in B0a_1 B0a_2; do
    L=/root/runs/$N/driver.log
    if pgrep -f "qnew.sh $N " > /dev/null; then
      if grep -q "hand: measured\|the body as measured\|\[brain\] round" "$L" 2>/dev/null; then
        touch /root/q/done_$N; echo "$(date +%T) $N boot ended at $(grep -o "^@[0-9]*" "$L" | tail -1): stopping"
      else
        left=$((left+1))
      fi
    fi
  done
  [ "$left" = 0 ] && { echo "$(date +%T) both runs ended"; exit 0; }
  sleep 30
done
