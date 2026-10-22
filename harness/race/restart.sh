#!/bin/bash
# restart.sh: the line runners and the watch started again with the scripts as they are now; runs in flight finish
# under their old runners. Body files a run left half written (a full disk) are taken out of the queues first.
RACE=${RACE:-/root/race}
for p in $(ps -eo pid,ppid,args | awk '$2 == 1 && ($4 ~ /harness\/race\/line.sh/ || $4 ~ /harness\/race\/watch.sh/) {print $1}'); do kill $p; done
bad=0
for f in $RACE/ckpt/[0-9]/*.json; do
  [ -e "$f" ] || continue
  if [ "$(tail -c 16 "$f" | tr -d "\n \t" | tail -c 1)" != "}" ]; then mkdir -p $RACE/ckpt/broken; mv "$f" $RACE/ckpt/broken/; bad=$((bad+1)); fi
done
echo "half-written bodies taken out: $bad"
for L in 1 2 3 4 6; do [ -s $RACE/state/slots.$L.keep ] && cp $RACE/state/slots.$L.keep $RACE/state/slots.$L; done
d() { (setsid nohup "$@" < /dev/null >> $RACE/ledger/daemons.log 2>&1 &); }
for L in 1 2 3 4 6; do d bash /root/bd/harness/race/line.sh $L $(cat $RACE/state/slots.$L); done
(setsid nohup bash /root/bd/harness/race/watch.sh < /dev/null >> $RACE/watch.log 2>&1 &)
sleep 3; ps -eo ppid,args | awk '$1 == 1 && $2 == "bash"' | grep -c "race/line.sh"
