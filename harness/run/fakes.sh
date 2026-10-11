#!/bin/bash
# A batch of fake boots: one run of fake.sh per seed, several at once, then one table of what each
# measured against its truth.
#
#   bash fakes.sh PREFIX FIRST_SEED LAST_SEED [AT_ONCE] [fake_robot options...]
#
# Runs are named PREFIX<seed> under $FAKE_RUNS (fake.sh); AT_ONCE (default 4) go at the same time,
# each on its own port. When all have ended, every run's score.txt is read and one line a run is
# printed and written to $FAKE_RUNS/PREFIX.summary: how it stopped, the beats, each tool's position
# error, each eye's position error, and each lobe's tip error (open and closed), in millimetres;
# "-" where the scorer had nothing to score.
set -u
PREFIX=$1; FIRST=$2; LAST=$3; shift 3
AT_ONCE=${1:-4}; [ $# -gt 0 ] && shift
HERE="$(cd "$(dirname "$0")" && pwd)"
RUNS=${FAKE_RUNS:-$HOME/fakeruns}
mkdir -p "$RUNS"
running=0
for SEED in $(seq "$FIRST" "$LAST"); do
  bash "$HERE/fake.sh" "$PREFIX$SEED" "$SEED" "$@" > "$RUNS/$PREFIX$SEED.out" 2>&1 &
  running=$((running + 1))
  if [ "$running" -ge "$AT_ONCE" ]; then
    wait -n 2>/dev/null || wait
    running=$((running - 1))
  fi
done
wait
summarize() {
  local R=$1 S=$R/score.txt
  local why beats tools eyes lobes
  why=$(awk '$1 == "stop" {print $2}' "$R/meta.txt" 2>/dev/null)
  beats=$(grep -c '"kind":"observation"' "$R/truth.jsonl" 2>/dev/null)
  tools=$(grep -E '^tool [0-9]+' "$S" 2>/dev/null | sed -E 's/^tool ([0-9]+).*position median ([0-9.]+) mm.*/t\1=\2/; s/^tool ([0-9]+):.*/t\1=-/' | tr '\n' ' ')
  eyes=$(grep -E '^eye [0-9]+ .*position median' "$S" 2>/dev/null | sed -E 's/^eye ([0-9]+) .*position median ([0-9.]+) mm.*/e\1=\2/' | tr '\n' ' ')
  lobes=$(grep -E '^  (open|closed) lobe' "$S" 2>/dev/null | sed -E 's/^  (open|closed) lobe ([0-9]+)[^:]*: ([0-9.]+) mm from the support.*/\1\2=\3/; s/^  (open|closed) lobe ([0-9]+): no press.*/\1\2=-/' | tr '\n' ' ')
  echo "$(basename "$R") ${why:-?} ${beats:-0} beats | ${tools:--} | ${eyes:--} | ${lobes:--}"
}
{
  for SEED in $(seq "$FIRST" "$LAST"); do
    summarize "$RUNS/$PREFIX$SEED"
  done
} | tee "$RUNS/$PREFIX.summary"
