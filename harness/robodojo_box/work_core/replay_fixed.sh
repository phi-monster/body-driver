#!/bin/bash
# 1. B0F1 with the lag fix (a86e3f8): the estimates that took 141 and 731 s on 985e89a.
( time ( zstd -dc /root/fakeruns/B0F1/run.rec.zst | head -c 1400000000 | /root/work/core/run_a86e3f8/bin/replay /dev/stdin > /root/work/core/b0f1_replay_fixed.log 2>&1 ) ) 2> /root/work/core/b0f1_replay_fixed.time
# 2. B0_2, eye 3 (arm 1 own eye) watched through the first sweep rounds: why the base joints first pushes were not followed.
( time ( zstd -dc /root/runs/B0_2/run.rec.zst | head -c 800000000 | /root/work/core/run_a86e3f8/bin/replay /dev/stdin --watch 3 > /root/work/core/b0_2_replay_watch3.log 2>&1 ) ) 2> /root/work/core/b0_2_replay_watch3.time
# 3. B0_1, eye 3 watched through the recognition rounds (its settling grew 504 -> 1728 beats a round).
( time ( zstd -dc /root/runs/B0_1/run.rec.zst 2>/dev/null | head -c 1500000000 | /root/work/core/run_a86e3f8/bin/replay /dev/stdin --watch 3 > /root/work/core/b0_1_replay_watch3.log 2>&1 ) ) 2> /root/work/core/b0_1_replay_watch3.time
