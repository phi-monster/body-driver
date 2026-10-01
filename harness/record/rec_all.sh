#!/bin/bash
# Record the five reference bodies booting from zero, one after another (each waits for the simulator slot).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
bash "$HERE/qrec.sh" RX5 bootcal arx_x5 0 30
bash "$HERE/qrec.sh" RG1 bootcal g1_rgb 1 30
bash "$HERE/qrec.sh" RDR bootcal drone_rgb 1 20
bash "$HERE/qrec.sh" RWA bd_mouse_floor wheelarm_rgb 0 12
bash "$HERE/qrec.sh" RGW bd_livingroom g1walk_rgb 0 12
echo "all recordings done $(date +%T)" >> /root/q/queue.log
