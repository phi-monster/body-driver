#!/bin/bash
# Builds the watch replay from the uploaded tar and replays A56's start once per eye, each line of that eye kept.
H=$1
rm -rf /root/work/core/run_watch && mkdir -p /root/work/core/run_watch && cd /root/work/core/run_watch && zstd -dc ../run_$H.tar.zst | tar -x && rm ../run_$H.tar.zst && cp -r /root/work/core/deps/alire driver/ && cd driver || exit 1
export PATH=/root/alire/bin:$HOME/.alire/bin:$PATH
nice -n 5 alr -n build > ../build.log 2>&1; echo build exit $? >> ../build.log
cd /root/work/core/prof && rm -f lag_finished
for e in 1 2 3; do
  zstd -q -dc /root/runs/A56/run.rec.zst | head -c 2500000000 | /root/work/core/run_watch/driver/bin/replay /dev/stdin --watch $e 2>&1 | grep "eye $e:" > lag_a56_e$e.log
done
echo done > lag_finished
