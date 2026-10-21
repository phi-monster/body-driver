#!/bin/bash
# B0a: B0 again with its four fixes (a86e3f8 lag cost, 26eb8c36 sweep ladder and recognition hold,
# 23b0ced6 view wait): the official arx_x5, scrambled, from zero, two runs at once (slot 0 and slot 1,
# task seed 0 both: seed 1 ends the episode at reset), 180 minutes the ceiling; and two random fake
# bodies at once for 150 minutes (seeds 1 and 2; a fake records 0.22 GB a minute, 0.8 alone, and the
# box has about 60 GB), each with a disk watchdog that stops it below 12 GB free.
H=${1:?hash}; D=/root/work/core/run_$H; B=$D/driver/bin/body_driver
[ -x "$B" ] || { echo "no driver built at $B"; exit 2; }
[ -f "$D/harness/run/fakes.sh" ] || { echo "no harness in $D (archive from the repo root)"; exit 2; }
for N in B0a_1 B0a_2; do [ -e /root/runs/$N ] && { echo "/root/runs/$N exists"; exit 5; }; done
cd /root/work/core || exit 1
BD_NO_VIDEO=1 BD_TIMING=1 BD_SLOT=0 setsid nohup bash /root/rectools/harness/run/qnew.sh B0a_1 bootcal arx_x5 0 180 "$B" > /root/work/core/b0a_1_qnew.out 2>&1 < /dev/null &
BD_NO_VIDEO=1 BD_TIMING=1 BD_SLOT=1 setsid nohup bash /root/rectools/harness/run/qnew.sh B0a_2 bootcal arx_x5 0 180 "$B" > /root/work/core/b0a_2_qnew.out 2>&1 < /dev/null &
# The fake bodies only with FAKES=1: the fake arms rise under their hold (B0F1: arm 2 fingers from
# z 0.91 to 1.63 m over 8000 beats, a fake_world sag bug), to be fixed before a fake batch means anything.
if [ "${FAKES:-0}" = 1 ]; then
  FAKE_RUNS=/root/fakeruns BIN=$D/driver/bin FAKE_MINUTES=150 setsid nohup bash "$D/harness/run/fakes.sh" B0aF 1 2 2 > /root/fakeruns/B0aF.out 2>&1 < /dev/null &
  sleep 5
  for F in B0aF1 B0aF2; do setsid nohup bash /root/work/core/fakewatch.sh $F > /root/work/core/fakewatch_$F.out 2>&1 < /dev/null & done
fi
date +%T; echo started
ps -eo pid,etime,args | grep -E "qnew.sh|fakes.sh|fake.sh|fakewatch" | grep -v grep | cut -c1-140
