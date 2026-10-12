#!/bin/bash
# B0: the first acceptance run (驱动重写.md §6 item 5; the owner named it B0 on 10-12, fixes B0a, B0b...):
# the official arx_x5, scrambled, from zero (no body file), two runs at once, 180 minutes each
# (slot 0 with task seed 0, slot 1 with task seed 1), and six random fake bodies at once for
# 150 minutes (the discovery loop, seeds 1..6 as the Z batch had), all on the branch commit 985e89a.
H=985e89a; D=/root/work/core/run_$H; B=$D/driver/bin/body_driver
[ -x "$B" ] || { echo "no driver built at $B"; exit 2; }
for N in B0_1 B0_2; do [ -e /root/runs/$N ] && { echo "/root/runs/$N exists"; exit 5; }; done
cd /root/work/core || exit 1
BD_NO_VIDEO=1 BD_TIMING=1 BD_SLOT=0 setsid nohup bash /root/rectools/harness/run/qnew.sh B0_1 bootcal arx_x5 0 180 "$B" > /root/work/core/b0_1_qnew.out 2>&1 &
BD_NO_VIDEO=1 BD_TIMING=1 BD_SLOT=1 setsid nohup bash /root/rectools/harness/run/qnew.sh B0_2 bootcal arx_x5 1 180 "$B" > /root/work/core/b0_2_qnew.out 2>&1 &
FAKE_RUNS=/root/fakeruns BIN=$D/driver/bin FAKE_MINUTES=150 setsid nohup bash "$D/harness/run/fakes.sh" B0F 1 6 6 > /root/fakeruns/B0F.out 2>&1 &
sleep 3
date +%T; echo started
ps -eo pid,etime,args | grep -E "qnew.sh|fakes.sh|fake.sh" | grep -v grep | cut -c1-140
