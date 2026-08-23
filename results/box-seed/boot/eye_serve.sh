export CUDA_VISIBLE_DEVICES=1
exec /root/eyeenv/bin/vllm serve /root/eye_model \
  --port 8077 --served-model-name eye \
  --gpu-memory-utilization 0.90 --max-model-len 8192 --max-num-seqs 8
