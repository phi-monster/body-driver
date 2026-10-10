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
# the end. A scrambled run (harness/robodojo_scramble) is scored through the draw written beside it
# (scramble.json), which the scorer pairs the recorded beats with the truth by. # the end. The report goes to stdout and to RUN_DIR/score-BUILD.txt, the replay's log to
# RUN_DIR/replay-BUILD.log and its estimates to RUN_DIR/estimates-BUILD.jsonl.zst, BUILD being the
# name of the build the tools come from (the directory above BIN_DIR's driver), so scorings by
# different builds never overwrite each other; one run is scored by one scoring at a time. The
# report's first line names the tools, and says when they are not the run's own build (meta.txt's
# driver md5): a replay by other code asks what the run never asked, and without --inst those
# calls are never answered (A29 replayed by a newer build never fitted arm 1, and was once read
# as the run's own).
set -u
RUN=$1; BIN=$2; shift 2
BUILD=$(basename "$(dirname "$(dirname "$(cd "$BIN" && pwd)")")")
[ "$BUILD" = "/" ] && BUILD=tools
exec 8> "$RUN/.scoring.lock"
flock -n 8 || { echo "$RUN is being scored by another scoring; not starting a second"; exit 4; }
RUN_MD5=$(awk '$1 == "driver" {print $2}' "$RUN/meta.txt" 2>/dev/null)
OWN_MD5=$(md5sum "$BIN/body_driver" 2>/dev/null | cut -c1-32)
if [ -n "$RUN_MD5" ] && [ "$RUN_MD5" = "$OWN_MD5" ]; then
  HEAD_LINE="tools: $BIN, the run's own build"
else
  HEAD_LINE="tools: $BIN, NOT the run's own build (driver $OWN_MD5, the run's ${RUN_MD5:-unknown}): its replay asks what the run never asked$(case " $* " in *" --inst "*) echo ", answered live";; *) echo ", and without --inst those calls are never answered";; esac)"
fi
REC=$(ls "$RUN"/run.rec.zst "$RUN"/wire.rec.zst 2>/dev/null | head -1)
[ -n "$REC" ] || { echo "no recording in $RUN"; exit 2; }
[ -f "$RUN/truth.jsonl.zst" ] || { echo "no truth in $RUN"; exit 2; }
W=$(mktemp -d /root/work/score.XXXXXX)
trap 'rm -rf "$W"' EXIT
zstd -q -d "$RUN/truth.jsonl.zst" -o "$W/truth.jsonl" || exit 3
zstd -q -dc "$REC" | "$BIN/replay" /dev/stdin --estimates "$W/estimates.jsonl" "$@" > "$W/replay.log" 2>&1
cp "$W/replay.log" "$RUN/replay-$BUILD.log"
zstd -q -f "$W/estimates.jsonl" -o "$RUN/estimates-$BUILD.jsonl.zst"
tail -3 "$W/replay.log"
SCR=(); [ -f "$RUN/scramble.json" ] && SCR=(--scramble "$RUN/scramble.json")
{ echo "$HEAD_LINE"; zstd -q -dc "$REC" | "$BIN/score" "${SCR[@]}" "$W/estimates.jsonl" /dev/stdin "$W/truth.jsonl"; } | tee "$RUN/score-$BUILD.txt"
