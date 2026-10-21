#!/bin/bash
# While run $1 is going: if the free space falls below 8 GB, stop it cleanly through its done file.
K=$1
while true; do
  grep -q "run done: $K" /root/q/queue.log && { echo "$K done at $(date -u +%T), free $(( $(df -B1 --output=avail /root | tail -1) >> 30 )) GB"; exit 0; }
  free=$(df -B1 --output=avail /root | tail -1)
  if [ "$free" -lt $(( 8 * 1024 * 1024 * 1024 )) ]; then
    touch /root/q/done_$K; echo "stopped $K at $(date -u +%T): free $((free >> 30)) GB"; exit 3
  fi
  sleep 30
done
