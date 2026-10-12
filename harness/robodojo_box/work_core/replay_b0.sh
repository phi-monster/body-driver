#!/bin/bash
# head -c counts the decompressed stream: an x5 beat is about 2.8 MB raw (0.37 MB in the zstd).
# B0_2, eye 3 watched through the first sweep rounds (sweep began at beat 1879): ~2300 beats.
( time ( zstd -dc /root/runs/B0_2/run.rec.zst | head -c 6500000000 | /root/work/core/run_a86e3f8/bin/replay /dev/stdin --watch 3 > /root/work/core/b0_2_replay_watch3.log 2>&1 ) ) 2> /root/work/core/b0_2_replay_watch3.time
# B0_1, eye 3 watched through the recognition rounds up to the 16x round (beats 4300-6611): ~7000 beats.
( time ( zstd -dc /root/runs/B0_1/run.rec.zst 2>/dev/null | head -c 19600000000 | /root/work/core/run_a86e3f8/bin/replay /dev/stdin --watch 3 > /root/work/core/b0_1_replay_watch3.log 2>&1 ) ) 2> /root/work/core/b0_1_replay_watch3.time
