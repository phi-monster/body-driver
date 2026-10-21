#!/bin/bash
# Builds run_<hash> from its uploaded archive, then starts a from-zero boot on each slot with it:
# slot 0 the dev config (arx_x5_fabric), slot 1 the official one (arx_x5).
#   build_and_boot.sh HASH NAME0 NAME1
H=$1; N0=$2; N1=$3
D=/root/work/core/run_$H
rm -rf $D && mkdir -p $D && cd $D && zstd -dc ../run_$H.tar.zst | tar -x && rm ../run_$H.tar.zst && cp -r /root/work/core/deps/alire driver/ && cd driver || exit 1
export PATH=/root/alire/bin:$HOME/.alire/bin:$PATH
nice -n 5 alr -n build > ../build.log 2>&1; echo build exit $? >> ../build.log
B=$D/driver/bin/body_driver
[ -x $B ] || { echo "no driver built"; exit 2; }
cd /root/work/core
BD_NO_VIDEO=1 BD_TIMING=1 BD_SLOT=0 setsid nohup bash /root/rectools/harness/run/qnew.sh $N0 bootcal arx_x5_fabric 0 150 $B /root/work/core/${N0,,}_body.json > /root/work/core/${N0,,}_qnew.log 2>&1 < /dev/null &
BD_NO_VIDEO=1 BD_TIMING=1 BD_SLOT=1 setsid nohup bash /root/rectools/harness/run/qnew.sh $N1 bootcal arx_x5 0 180 $B /root/work/core/${N1,,}_body.json > /root/work/core/${N1,,}_qnew.log 2>&1 < /dev/null &
echo started $N0 $N1
