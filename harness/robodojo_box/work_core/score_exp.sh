#!/bin/bash
# Builds run_<name> from its archive when not built, then scores a run with it (qscore, live instrument).
#   score_exp.sh NAME RUN_DIR
N=$1; RUN=$2
D=/root/work/core/run_$N
if [ ! -x $D/driver/bin/replay ]; then
  rm -rf $D && mkdir -p $D && cd $D && zstd -dc ../run_$N.tar.zst | tar -x 2>/dev/null && rm ../run_$N.tar.zst && cp -r /root/work/core/deps/alire driver/ && cd driver || exit 1
  export PATH=/root/alire/bin:$HOME/.alire/bin:$PATH
  nice -n 5 alr -n build > ../build.log 2>&1; echo build exit $? >> ../build.log
fi
bash /root/rectools/harness/run/qscore.sh $RUN $D/driver/bin --inst 127.0.0.1:8077
