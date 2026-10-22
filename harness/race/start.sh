#!/bin/bash
# start.sh: the race's lines, the brain service and the watch, each detached.
RACE=${RACE:-/root/race}; mkdir -p $RACE/ledger
d() { (setsid nohup "$@" < /dev/null >> $RACE/ledger/daemons.log 2>&1 &); }
d bash /root/bd/harness/race/line.sh 1 10
d bash /root/bd/harness/race/line.sh 2 8
d bash /root/bd/harness/race/line.sh 3 6
d bash /root/bd/harness/race/line.sh 4 4
d bash /root/bd/harness/race/line.sh 6 2
d bash /root/bd/harness/race/brain.sh
(setsid nohup bash /root/bd/harness/race/watch.sh < /dev/null >> $RACE/watch.log 2>&1 &)
