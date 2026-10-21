#!/bin/bash
# Builds run_<hash> from its uploaded archive and replays a whole recording with the live instrument, the replay's
# lines kept (the estimate lines say how long each recompute stage took).
H=$1; REC=$2; OUT=$3
D=/root/work/core/run_$H
rm -rf $D && mkdir -p $D && cd $D && zstd -dc ../run_$H.tar.zst | tar -x && rm ../run_$H.tar.zst && cp -r /root/work/core/deps/alire driver/ && cd driver || exit 1
export PATH=/root/alire/bin:$HOME/.alire/bin:$PATH
nice -n 5 alr -n build > ../build.log 2>&1; echo build exit $? >> ../build.log
cd /root/work/core/prof && rm -f $OUT.finished
zstd -q -dc $REC | $D/driver/bin/replay /dev/stdin --inst 127.0.0.1:8077 > $OUT.log 2>&1
echo done > $OUT.finished
