#!/bin/bash
# Score one run against the simulator's truth: replay its recording through the estimators of a
# driver build, then compare the estimates with the side-file truth.
#
#   bash qscore.sh RUN_DIR BIN_DIR [replay options...]
#
# RUN_DIR holds run.rec.zst (qnew.sh) or wire.rec.zst (qrec.sh), and truth.jsonl.zst. BIN_DIR holds
# the driver's tools (replay, score). Extra options go to replay (for example --inst 127.0.0.1:8077
# for a recording without service replies). The recording is streamed from its compressed file
# (it is never unpacked on disk); only the truth is unpacked, into a scratch directory removed at
# the end. The report goes to stdout and to RUN_DIR/score.txt, and the replay's log to
# RUN_DIR/replay.log.
set -u
RUN=$1; BIN=$2; shift 2
REC=$(ls "$RUN"/run.rec.zst "$RUN"/wire.rec.zst 2>/dev/null | head -1)
[ -n "$REC" ] || { echo "no recording in $RUN"; exit 2; }
[ -f "$RUN/truth.jsonl.zst" ] || { echo "no truth in $RUN"; exit 2; }
W=$(mktemp -d /root/work/score.XXXXXX)
trap 'rm -rf "$W"' EXIT
zstd -q -d "$RUN/truth.jsonl.zst" -o "$W/truth.jsonl" || exit 3
zstd -q -dc "$REC" | "$BIN/replay" /dev/stdin --estimates "$W/estimates.jsonl" "$@" > "$W/replay.log" 2>&1
cp "$W/replay.log" "$RUN/replay.log"
tail -3 "$W/replay.log"
zstd -q -dc "$REC" | "$BIN/score" "$W/estimates.jsonl" /dev/stdin "$W/truth.jsonl" | tee "$RUN/score.txt"
