#!/bin/bash
# watch.sh: every minute one line of the race's state, and an ALERT line for every line that stopped or idles, for
# cores left idle, and for the brain down. Monitor tails $RACE/watch.log for ALERT.
RACE=${RACE:-/root/race}
while [ ! -e $RACE/stop ]; do
  load=$(cut -d' ' -f1 /proc/loadavg); s="$(date +%H:%M) load $load"
  for L in 1 2 3 4 6; do
    alive=$(ps -eo args | grep -c "^bash $RACE/../bd/harness/race/line.sh $L \|^bash /root/bd/harness/race/line.sh $L ")
    run=$(ps -eo args | grep -c "^$RACE/runs/$L/[^ ]*/bin/body_driver")
    case $L in 2) bn=step2_bench;; 3) bn=flow_bench;; 4) bn=step4_bench;; *) bn=none;; esac
    run=$((run + $(ps -eo args | grep -c "^$RACE/bin/driver-robot-$bn ")))
    pend=$(ls $RACE/ckpt/$L/*.json 2>/dev/null | wc -l); led=$(wc -l < $RACE/ledger/line$L.txt 2>/dev/null || echo 0)
    s="$s | L$L run $run pend $pend done $led"
    [ "$alive" = 0 ] && echo "$(date +%H:%M) ALERT line $L STOPPED (its runner is gone)"
    [ "$alive" != 0 ] && [ "$run" = 0 ] && echo "$(date +%H:%M) ALERT line $L IDLE: no run going, $pend cases waiting"
  done
  b=$(ps -eo args | grep -c "^/root/brain/llama.cpp/build/bin/llama-server"); s="$s | brain $b"
  [ "$b" = 0 ] && [ -s /root/brain/Qwen3.5-9B-Q8_0.gguf ] && echo "$(date +%H:%M) ALERT brain down"
  awk -v l=$load 'BEGIN{exit !(l < 96)}' && echo "$(date +%H:%M) ALERT cores idle: load $load of 128"
  # The load held near the box's 128 threads, never far over: an oversubscribed box ran every fit 3 to 7 times
  # slower and the beats a fit holds the arm still for with it (load 282 on 10-23, 19:04). Under 110, line 1 (fresh
  # cold boots, always work to do) gets 2 slots more; over 140, line 1 gives 2 back down to 4, then line 2 down to 4.
  s1=$(cat $RACE/state/slots.1 2>/dev/null || echo 8); s2=$(cat $RACE/state/slots.2 2>/dev/null || echo 6)
  # Under 110: line 2 gets the 2 slots while more cases wait for it than it runs (line 1 made 787 runs and line 2
  # had 404 waiting on 10-23, 19:47), line 1 otherwise; line 2 at most 30.
  p2=$(ls $RACE/ckpt/2/*.json 2>/dev/null | wc -l)
  if awk -v l=$load 'BEGIN{exit !(l < 110)}'; then if [ $p2 -gt $s2 ] && [ $s2 -lt 30 ]; then s2=$((s2 + 2)); else s1=$((s1 + 2)); fi
  elif awk -v l=$load 'BEGIN{exit !(l > 140)}'; then if [ $s1 -gt 4 ]; then s1=$((s1 - 2)); elif [ $s2 -gt 4 ]; then s2=$((s2 - 2)); fi; fi
  [ $s1 -gt 40 ] && s1=40; echo $s1 > $RACE/state/slots.1; echo $s2 > $RACE/state/slots.2; s="$s | slots L1 $s1 L2 $s2"
  free=$(df --output=avail -BG / | tail -1 | tr -dc 0-9); s="$s | disk ${free}G free"
  [ "$free" -lt 10 ] && echo "$(date +%H:%M) ALERT disk: ${free} GB free"
  echo "$s"
  sleep 60
done
