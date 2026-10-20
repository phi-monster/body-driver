#!/usr/bin/env bash
# cap.sh CMD... : run CMD, killed at 120 s (owner 10-20: a run past two minutes is a failure); exit 142 when killed.
exec perl -e 'alarm 120; exec @ARGV or die "cannot run $ARGV[0]\n"' "$@"
