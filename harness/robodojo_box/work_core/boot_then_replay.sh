#!/bin/bash
# Builds and starts the two boots with HASH, then replays a recording with the same build.
H=$1; N0=$2; N1=$3; REC=$4; OUT=$5; shift 5
bash /root/work/core/build_and_boot.sh $H $N0 $N1 > /root/work/core/bab.out 2>&1
bash /root/work/core/replayrun3.sh $H $REC $OUT "$@"
