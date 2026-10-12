#!/bin/bash
# While fake run $1 goes: when the free space falls below 12 GB, stop it through its stop file, so the
# two B0 runs keep their room (a fake body records about 0.22 GB a minute).
R=/root/fakeruns/$1
while pgrep -f "fake.sh $1 " > /dev/null; do
  free=$(df -B1 --output=avail /root | tail -1)
  if [ "$free" -lt $(( 12 * 1024 * 1024 * 1024 )) ]; then
    touch "$R/stop"; echo "stopped $1 at $(date -u +%T): free $((free >> 30)) GB"; exit 3
  fi
  sleep 30
done
echo "$1 ended on its own at $(date -u +%T), free $(( $(df -B1 --output=avail /root | tail -1) >> 30 )) GB"
