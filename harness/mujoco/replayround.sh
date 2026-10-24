#!/bin/bash
# replayround.sh ROUND REPLAY_BIN OUT (simulator-side rig code): every boot of a recorded round (batch.sh with RECORD=1
# KEEP_RECORDING=1) replayed with REPLAY_BIN's code (driver/tools/replay --graphs: the recording fed through the
# estimators as the run did, the decider's notes made where it made them, the graph derived at the same points), its
# graph at the run's last groups as recognized (the replay's last estimate at or before that beat) scored as step 1 is
# (score1.pl), beside the boot's own verdict: one line a boot in OUT/verdicts.txt, a boot unfinished at its cap staying
# unfinished. A rule of the graph is tried so on the same boots without what timing alone moves a round by (2 to 4 of
# 60); what it would make the decider do next is not replayed. P replays at once (30).
ROUND=$1; BIN=$2; OUT=$3
HERE="$(cd "$(dirname "$0")" && pwd)"; export HERE
RUNS=${RUNS:-/root/runs}
mkdir -p $OUT
one() {
  local d=$1 bin=$2 out=$3 tag=$(basename $1)
  [ -f $d/run.rec.zst ] || return
  local w=$out/$tag; mkdir -p $w
  zstd -dc $d/run.rec.zst | $bin/replay /dev/stdin --graphs > $w/replay.log 2>&1
  cp $d/truth.json $w/
  local at=$(zstd -dc $d/driver.log.zst | grep -E "^@[0-9]+ \[robot\] boot: the groups as recognized:" | tail -1 | sed -E 's/^@([0-9]+) .*/\1/')
  { zstd -dc $d/driver.log.zst | awk '/\[core\] the robot reports:/{p=1} p{print} p && /^[[:space:]]*$/{exit}'
    if [ -n "$at" ]; then
      perl -e 'my $at = shift; my (@cur, @best, $in); while (<>) { if (/^\@(\d+) \[robot\] boot: the groups as recognized:/) { last if $1 > $at; @cur = ($_); $in = 1; next } if ($in) { if (/^\s*$/) { @best = @cur; $in = 0 } else { push @cur, $_ } } } @best = @cur if $in; print @best, "\n"' $at $w/replay.log
    fi; } > $w/driver.log
  local mine=$(perl $HERE/score1.pl $w | cut -d' ' -f3-)
  local own=$(grep " $tag step1 " $(dirname $d)/ledger.txt | head -1 | sed -E "s/.* $tag step1 //; s/ stop .*//")
  echo "$tag | own: $own | replay: $mine" > $w/verdict.txt
}
export -f one
ls -d $RUNS/$ROUND/*/ | sed 's#/$##' | xargs -P ${P:-30} -I{} bash -c 'one {} '$BIN' '$OUT
cat $OUT/*/verdict.txt | sort > $OUT/verdicts.txt
echo "agree $(awk -F'|' '{split($2,a,": "); split($3,b,": "); x=(a[2] ~ /^PASS/); y=(b[2] ~ /^PASS/); if (x==y) n++} END {print n+0}' $OUT/verdicts.txt) of $(wc -l < $OUT/verdicts.txt); replay passes $(grep -c "replay: PASS" $OUT/verdicts.txt), own passes $(grep -c "own: PASS" $OUT/verdicts.txt)"
