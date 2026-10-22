#!/bin/bash
# line.sh LINE SLOTS: one line of the race (驱动重写2.md §3), SLOTS runs at once, until $RACE/stop exists. A run starts
# from the line's next case, is stopped at the line's step, is scored into $RACE/ledger/lineLINE.txt, and its body
# file becomes the next line's case when it reached the step. Line 1 draws fresh seeds of every class; the others take
# the oldest case the line before them left. LINE: 1 groups, 2 arms, 3 fixed eyes, 4 hands, 6 brain and task.
LINE=$1; SLOTS=$2
# The slots are read again every round from $RACE/state/slots.LINE when it exists, so they change without a restart.
RACE=${RACE:-/root/race}; BD=${BD:-/root/bd}
mkdir -p $RACE/runs/$LINE $RACE/ckpt/$LINE/taken $RACE/ledger $RACE/state
export FAKE_BIN_FROZEN=1 FAKE_RUNS=$RACE/runs/$LINE FAKE_RECORD=0 FAKE_REPLAY=0 FAKE_GPU=${FAKE_GPU:-/root/libfakegpu.so} BIN=$RACE/bin
CLASSES="t k b d"
opts_of() { case $1 in t) echo "--beats 3000000";; k) echo "--servo yes --ticks 4096 --beats 3000000";;
  b) echo "--class base --beats 3000000";; d) echo "--class drone --beats 3000000";; esac; }
first_seed() { case $1 in t) echo 600;; k) echo 700;; b) echo 800;; d) echo 1000;; esac; }
case $LINE in
  1) STOP="boot: the body is kept in .* after recognizing|the body as measured:"; MIN=5; W=2; NEXT=2 ;;
  2) STOP="boot: the body is kept in .* after sweeping|the body as measured:"; MIN=8; W=8; NEXT=3 ;;
  3) STOP="boot: the body is kept in .* after placing|the body as measured:"; MIN=5; W=3; NEXT=4 ;;
  4) STOP="the body as measured:"; MIN=6; W=3; NEXT=6 ;;
  6) STOP=""; MIN=10; W=3; NEXT=""; export FAKE_UNTIL=task FAKE_BRAIN=${BRAIN:-127.0.0.1:18081} FAKE_TRUTH=1 ;;
esac
PORT0=$((21000 + LINE * 200)); n=0
next_case() {   # prints "NAME SEED BODY|-" and claims the case
  # $RACE/state/seedlist (lines "NAME SEED"): line 1 takes its cases from it, in order, instead of fresh seeds, to run
  # bodies another build already ran (the same seeds, two builds side by side).
  if [ "$LINE" = 1 ] && [ -s $RACE/state/seedlist ]; then
    k=$(cat $RACE/state/seedlist.at 2>/dev/null || echo 1); l=$(sed -n "${k}p" $RACE/state/seedlist); [ -z "$l" ] && return 1
    echo $((k + 1)) > $RACE/state/seedlist.at; set -- $l; echo "$1 $2 -"
  elif [ "$LINE" = 1 ]; then
    i=$(cat $RACE/state/class 2>/dev/null || echo 0); set -- $CLASSES; shift $((i % 4)); c=$1; echo $((i + 1)) > $RACE/state/class
    s=$(cat $RACE/state/seed.$c 2>/dev/null || first_seed $c); echo $((s + 1)) > $RACE/state/seed.$c
    echo "$c$s $s -"
  else
    f=$(ls -tr $RACE/ckpt/$LINE/*.json 2>/dev/null | head -1); [ -z "$f" ] && return 1
    b=$(basename $f .json); mv $f $RACE/ckpt/$LINE/taken/ 2>/dev/null || return 1; echo "$b $(cat $RACE/ckpt/$LINE/$b.seed) $RACE/ckpt/$LINE/taken/$b.json"
  fi
}
# A line with no case waiting runs its step's bench on the next fresh seed instead (seconds to minutes a seed, the
# fake world's truth beside every number): line 2 the arm fits (step2_bench), line 3 the fixed eyes' flows
# (flow_bench), line 4 the hands' lobes (step4_bench), each when the build has it. Its output goes to
# $RACE/bench/LINE/SEED.txt, its last lines to $RACE/ledger/benchLINE.txt.
bench_of() { case $LINE in 2) echo driver-robot-step2_bench;; 3) echo driver-robot-flow_bench;; 4) echo driver-robot-step4_bench;; esac; }
bench_one() {
  local b=$1 s=$(cat $RACE/state/bench.$LINE 2>/dev/null || echo 3000); echo $((s + 1)) > $RACE/state/bench.$LINE
  mkdir -p $RACE/bench/$LINE; local o=$RACE/bench/$LINE/$s.txt
  ( cd $RACE/bench/$LINE && BD_WORKERS=4 timeout 600 $RACE/bin/$b $s > $o 2>&1; echo "exit $?" >> $o )
  echo "$(date +%H:%M) bench $b seed $s: $(grep -vE "^exit|^\s*$" $o | tail -3 | cut -c1-200 | tr "\n" "|") $(tail -1 $o)" >> $RACE/ledger/bench$LINE.txt
}
run_one() {   # NAME SEED BODY PORT
  local name=$1 seed=$2 body=$3 port=$4 c=${1:0:1} tag="${1}_L${LINE}_$(date +%H%M%S)_$RANDOM"
  ( [ "$body" != - ] && export FAKE_BODY=$body
    FAKE_MINUTES=$MIN BD_WORKERS=$W FAKE_PORT=$port bash $BD/harness/run/fake.sh $tag $seed --rate 0 $(opts_of $c) \
      > $FAKE_RUNS/$tag.launch.txt 2>&1 ) &
  local P=$!
  if [ -n "$STOP" ]; then
    while kill -0 $P 2>/dev/null; do
      grep -qE "$STOP" $FAKE_RUNS/$tag/driver.log 2>/dev/null && { touch $FAKE_RUNS/$tag/stop; break; }
      sleep 3
    done
  fi
  wait $P
  echo "$(bash $BD/harness/race/score$LINE.sh $FAKE_RUNS/$tag 2>&1) [$(readlink $FAKE_RUNS/$tag/bin | grep -oE "bd[0-9]+")]" >> $RACE/ledger/line$LINE.txt
  if [ -n "$NEXT" ] && grep -qE "$STOP" $FAKE_RUNS/$tag/driver.log 2>/dev/null; then
    mkdir -p $RACE/ckpt/$NEXT; cp $FAKE_RUNS/$tag/body.json $RACE/ckpt/$NEXT/$name.json.tmp
    echo $seed > $RACE/ckpt/$NEXT/$name.seed; mv $RACE/ckpt/$NEXT/$name.json.tmp $RACE/ckpt/$NEXT/$name.json
  fi
  # What a run keeps once scored: its log, compressed, its body and its meta; its truth, geometry and scrambler go.
  zstd -q --rm -T1 $FAKE_RUNS/$tag/driver.log 2>/dev/null
  rm -rf $FAKE_RUNS/$tag/truth.jsonl $FAKE_RUNS/$tag/geometry $FAKE_RUNS/$tag/scramble.json $FAKE_RUNS/$tag/bin
}
while [ ! -e $RACE/stop ] && [ ! -e $RACE/stop.$LINE ]; do
  [ -s $RACE/state/slots.$LINE ] && SLOTS=$(cat $RACE/state/slots.$LINE)
  # No launch while the disk has less than 5 GB free (a full disk broke every run on 10-23, 18:48).
  while [ "$(jobs -rp | wc -l)" -lt "$SLOTS" ] && [ "$(df --output=avail -BG / | tail -1 | tr -dc 0-9)" -ge 5 ]; do
    if C=$(next_case); then run_one $C $((PORT0 + n % 200)) &
    else b=$(bench_of); [ -n "$b" ] && [ -x $RACE/bin/$b ] || break; bench_one $b &
    fi
    n=$((n + 1))
    sleep 1
  done
  sleep 5
done
wait
