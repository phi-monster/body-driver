#!/bin/bash
# After A16 ends: replay its recording with the same build and score it against truth (qscore.sh keeps the
# replay log beside the score), then compare the live and replayed estimate and hand lines (replay fidelity).
until grep -q "run done: A16" /root/q/queue.log; do sleep 30; done
sleep 60
R=/root/runs/A16; W=/root/work/core
bash /root/rectools/harness/run/qscore.sh $R $W/bin_1e85ed8 > $W/a16_score.log 2>&1
grep -o "estimated from [0-9]* beats" $R/driver.log > $W/a16_live_estimates.txt
grep -o "estimated from [0-9]* beats" $R/replay.log > $W/a16_replay_estimates.txt
diff $W/a16_live_estimates.txt $W/a16_replay_estimates.txt > $W/a16_estimates.diff; echo "diff exit $?" >> $W/a16_estimates.diff
grep -E "hand: closer|lobe [0-9]:|kinematics: arm [0-9] fitted" $R/driver.log > $W/a16_live_hand.txt
grep -E "hand: closer|lobe [0-9]:|kinematics: arm [0-9] fitted" $R/replay.log > $W/a16_replay_hand.txt
diff $W/a16_live_hand.txt $W/a16_replay_hand.txt > $W/a16_hand.diff; echo "diff exit $?" >> $W/a16_hand.diff
date -u +%T > $W/a16_checked
