#!/bin/bash
# Builds run_<hash> with the gprof build and replays a recording to its end with the live instrument, the profile
# written where the replay ran.
H=$1; REC=$2; OUT=$3
D=/root/work/core/prof_$H
rm -rf $D && mkdir -p $D && cd $D && zstd -dc ../run_$H.tar.zst | tar -x && cp -r /root/work/core/deps/alire driver/ && cd driver || exit 1
export PATH=/root/alire/bin:$HOME/.alire/bin:$PATH
BD_BUILD=profile nice -n 5 alr -n build > ../build.log 2>&1; echo build exit $? >> ../build.log
mkdir -p /root/work/core/prof/$OUT && cd /root/work/core/prof/$OUT && rm -f gmon.out
zstd -q -dc $REC | $D/driver/bin-profile/replay /dev/stdin --inst 127.0.0.1:8077 > replay.log 2>&1
gprof -b -p $D/driver/bin-profile/replay gmon.out > flat.txt 2>&1
echo done > /root/work/core/prof/$OUT.finished
