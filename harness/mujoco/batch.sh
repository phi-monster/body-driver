#!/bin/bash
# batch.sh NAME BIN LIST [P]: a batch of driver boots on MuJoCo robots, P at once (each a driver of BD_WORKERS threads
# and a robot of one, its pictures on GPU index mod 2). LIST holds lines "KIND SEED COPY": the same KIND and SEED under
# two COPY letters is the same robot, cameras and draw booted twice, to see whether it comes out the same. Each run
# stops at STOP (default: step 1, the body kept after recognizing) or its cap (MINUTES, 5), is scored (SCORE, default
# score1.pl) into /root/runs/NAME/ledger.txt with its stop and its seconds, and keeps its log compressed.
NAME=$1; BIN=$2; LIST=$3; P=${4:-40}
# Each batch its own ports (from its name), so a run left of another batch never holds one (m1's SO-101 met m0's
# driver still on 32001, 10-23), and below the box's ephemeral range (32768-60999), where a connection's own port
# could hold one (r4g's SO-101 seed 2 met port 55042 taken, 10-23); off tensorboard's 16006.
PORT_BASE=${PORT_BASE:-$((20000 + $(echo -n "$NAME" | cksum | cut -d' ' -f1) % 127 * 100))}
HERE="$(cd "$(dirname "$0")" && pwd)"
export RUNS=/root/runs/$NAME BIN HERE PORT_BASE MINUTES=${MINUTES:-5} BD_WORKERS=${BD_WORKERS:-2}
export STOP=${STOP:-"boot: the body is kept in .* after recognizing|the body as measured:"} SCORE=${SCORE:-$HERE/score1.pl}
mkdir -p $RUNS
one() {
  local i=$1 kind=$2 seed=$3 copy=$4 tag="$2$3$4"
  PORT=$((PORT_BASE + i)) GPU=$((i % 2)) bash $HERE/run.sh $tag $kind $seed > $RUNS/$tag.out 2>&1
  local s=$(date -d "$(sed -n 's/^start //p' $RUNS/$tag/meta.txt)" +%s) e=$(date -d "$(sed -n 's/^end //p' $RUNS/$tag/meta.txt)" +%s)
  echo "$(date +%H:%M) $(perl $SCORE $RUNS/$tag) stop $(sed -n 's/^stop //p' $RUNS/$tag/meta.txt) $((e - s)) s" >> $RUNS/ledger.txt
  zstd -q --rm -T1 $RUNS/$tag/driver.log 2>/dev/null
}
export -f one
nl -v0 -w1 -s' ' $LIST | xargs -P $P -L1 bash -c 'one "$@"' _
echo "batch $NAME done $(date +%H:%M:%S)" >> $RUNS/ledger.txt
