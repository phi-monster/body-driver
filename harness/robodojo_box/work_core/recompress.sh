#!/bin/bash
# Recompress used recordings at level 17 (qnew.sh records at it from 10-08); a name that other links
# share is replaced at every link, so the old copy is freed once its readers close it.
set -o pipefail
for f in "$@"; do
  nice -n 10 zstd -q -dc "$f" | nice -n 10 zstd -q -T24 -17 -o "$f.l17" && zstd -q -t "$f.l17" || { echo "FAILED $f"; rm -f "$f.l17"; continue; }
  old=$(stat -c %s "$f"); new=$(stat -c %s "$f.l17"); ino=$(stat -c %i "$f")
  for g in $(find /root/runs /root/work -samefile "$f" 2>/dev/null); do [ "$g" = "$f" ] || ln -f "$f.l17" "$g"; done
  mv "$f.l17" "$f"
  echo "$f: $old -> $new bytes"
done
