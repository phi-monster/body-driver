#!/bin/bash
# Keeps a copy of a run's body file as it was when the arms were swept (groups and arms measured, no hand), for
# runs that measure only the hands from there.
R=$1; B=$2; OUT=$3
until grep -q "after sweeping the arms" /root/runs/$R/driver.log 2>/dev/null; do sleep 5; done
cp "$B" "$OUT" && echo "kept $OUT at $(grep "after sweeping the arms" /root/runs/$R/driver.log | cut -c1-8)"
