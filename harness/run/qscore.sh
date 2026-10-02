#!/bin/bash
# Score one run against the simulator's truth: replay its recording through the estimators of a
# driver build, then compare the estimates with the side-file truth.
#
#   bash qscore.sh RUN_DIR BIN_DIR [replay options...]
#
# RUN_DIR holds run.rec.zst (qnew.sh) or wire.rec.zst (qrec.sh), and truth.jsonl.zst. BIN_DIR holds
# the driver's tools (replay, score). Extra options go to replay (for example --inst 127.0.0.1:8077
# for a recording without service replies). The decompressed files live in a scratch directory
# that is removed at the end; the report goes to stdout and to RUN_DIR/score.txt.
set -u
RUN=$1; BIN=$2; shift 2
REC=$(ls "$RUN"/run.rec.zst "$RUN"/wire.rec.zst 2>/dev/null | head -1)
[ -n "$REC" ] || { echo "no recording in $RUN"; exit 2; }
[ -f "$RUN/truth.jsonl.zst" ] || { echo "no truth in $RUN"; exit 2; }
W=$(mktemp -d /root/work/score.XXXXXX)
trap 'rm -rf "$W"' EXIT
zstd -q -d "$REC" -o "$W/run.rec" && zstd -q -d "$RUN/truth.jsonl.zst" -o "$W/truth.jsonl" || exit 3
"$BIN/replay" "$W/run.rec" --estimates "$W/estimates.jsonl" "$@" > "$W/replay.log" 2>&1
tail -3 "$W/replay.log"
"$BIN/score" "$W/estimates.jsonl" "$W/run.rec" "$W/truth.jsonl" | tee "$RUN/score.txt"
