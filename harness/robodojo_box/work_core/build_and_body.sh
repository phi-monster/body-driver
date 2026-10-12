#!/bin/bash
# Builds run_<hash> from its uploaded archive, then starts one boot from a copy of a kept body file on one slot.
#   build_and_body.sh HASH NAME SLOT CFG BODY_FROM MINUTES
H=$1; N=$2; SLOT=$3; CFG=$4; FROM=$5; MIN=$6
D=/root/work/core/run_$H
if [ ! -x $D/driver/bin/body_driver ]; then
  rm -rf $D && mkdir -p $D && cd $D && zstd -dc ../run_$H.tar.zst | tar -x && rm ../run_$H.tar.zst && cp -r /root/work/core/deps/alire driver/ && cd driver || exit 1
  export PATH=/root/alire/bin:$HOME/.alire/bin:$PATH
  nice -n 5 alr -n build > ../build.log 2>&1; echo build exit $? >> ../build.log
fi
B=$D/driver/bin/body_driver
[ -x $B ] || { echo "no driver built"; exit 2; }
cd /root/work/core
cp $FROM /root/work/core/${N,,}_body.json
BD_NO_VIDEO=1 BD_TIMING=1 BD_SLOT=$SLOT setsid nohup bash /root/rectools/harness/run/qnew.sh $N bootcal $CFG 0 $MIN $B /root/work/core/${N,,}_body.json > /root/work/core/${N,,}_qnew.log 2>&1 < /dev/null &
echo started $N
