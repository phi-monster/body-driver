#!/bin/bash
# A gprof profile of the replay of a recording slice by a build: what the main task spends its time on.
D=$1; REC=$2; BYTES=$3; OUT=$4
cd /root/work/core/$D/driver || exit 1
export PATH=/root/alire/bin:$HOME/.alire/bin:$PATH
nice -n 5 alr -n build -- -XBD_BUILD=profile > ../build_profile.log 2>&1; echo build exit $? >> ../build_profile.log
mkdir -p $OUT && cd $OUT && rm -f gmon.out
zstd -q -dc $REC | head -c $BYTES | /root/work/core/$D/driver/bin-profile/replay /dev/stdin --inst 127.0.0.1:8077 > replay.log 2>&1
gprof -b -p /root/work/core/$D/driver/bin-profile/replay gmon.out > flat.txt 2>/dev/null
gprof -b -q /root/work/core/$D/driver/bin-profile/replay gmon.out > graph.txt 2>/dev/null
echo done > finished
