#!/bin/bash
# B0_2 again with task seed 0: with seed 1 the simulator connected and shut itself down before
# sending one observation, twice (04:12 and 04:21), the driver untouched (0-byte recordings).
touch /root/q/done_B0_2
for _ in $(seq 1 40); do pgrep -f "qnew.sh B0_2" > /dev/null || break; sleep 2; done
pkill -9 -f "qnew.sh B0_2" 2>/dev/null; pkill -9 -f -- "--listen 9081" 2>/dev/null; rm -f /root/q/done_B0_2 /root/q/now1.txt
mv /root/runs/B0_2 /root/runs/B0_2_dead2
B=/root/work/core/run_985e89a/driver/bin/body_driver
cd /root/work/core
BD_NO_VIDEO=1 BD_TIMING=1 BD_SLOT=1 setsid nohup bash /root/rectools/harness/run/qnew.sh B0_2 bootcal arx_x5 0 180 "$B" > /root/work/core/b0_2_qnew.out 2>&1 < /dev/null &
sleep 3; date +%T; tail -2 /root/q/queue.log
