#!/bin/bash
# brain.sh: Qwen3.5-9B (Q8_0, with its vision projector) behind llama-server on the second GPU, served as "eye", once
# the model is fetched; started again if it exits.
M=/root/brain/Qwen3.5-9B-Q8_0.gguf; P=/root/brain/mmproj-F16.gguf
until [ -s $M ] && [ -s $P ]; do sleep 10; done
while [ ! -e ${RACE:-/root/race}/stop ]; do
  CUDA_VISIBLE_DEVICES=1 /root/brain/llama.cpp/build/bin/llama-server -m $M --mmproj $P --alias eye --host 127.0.0.1 \
    --port 18081 -ngl 99 --ctx-size 32768 --parallel 2 >> /root/brain/server.log 2>&1
  sleep 5
done
