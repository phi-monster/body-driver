#!/bin/bash
# Builds a driver from the uploaded tar into run_<hash> and replays the start of a recording once per eye.
H=$1; REC=$2; BYTES=$3; TAG=$4
rm -rf /root/work/core/run_$H && mkdir -p /root/work/core/run_$H && cd /root/work/core/run_$H && zstd -dc ../run_$H.tar.zst | tar -x && rm ../run_$H.tar.zst && cp -r /root/work/core/deps/alire driver/ && cd driver || exit 1
export PATH=/root/alire/bin:$HOME/.alire/bin:$PATH
nice -n 5 alr -n build > ../build.log 2>&1; echo build exit $? >> ../build.log
cd /root/work/core/prof && rm -f lag_$TAG.finished
for e in 1 2 3; do
  zstd -q -dc $REC | head -c $BYTES | /root/work/core/run_$H/driver/bin/replay /dev/stdin --watch $e 2>&1 | grep "eye $e:\|answers a push" > lag_${TAG}_e$e.log
done
echo done > lag_$TAG.finished
