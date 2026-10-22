#!/bin/bash
# rescore.sh LINE: every finished run of the line scored again with the line's judge as it is now, its log read from
# its compressed copy (into /dev/shm for the judge, then removed); to stdout.
RACE=${RACE:-/root/race}; LINE=$1; T=/dev/shm/rescore.$$; mkdir -p $T
for d in $RACE/runs/$LINE/*/; do
  n=$(basename $d); [ -f $d/driver.log.zst ] || continue
  mkdir -p $T/$n; zstd -q -dc $d/driver.log.zst > $T/$n/driver.log; ln -sf $d/body.txt $T/$n/body.txt
  [ -f $d/truth.jsonl ] && ln -sf $d/truth.jsonl $T/$n/truth.jsonl
  echo "$(bash /root/bd/harness/race/score$LINE.sh $T/$n) [$(grep -oE "driver [0-9a-f]+" $d/meta.txt | cut -c8-15)]"
  rm -rf $T/$n
done
rm -rf $T
