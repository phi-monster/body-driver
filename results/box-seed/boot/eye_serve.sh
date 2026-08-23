#!/usr/bin/env bash
# 眼服务。三条硬约束,每条都是踩出来的:
# 1) 钉到 sim 不用的那张卡(同卡 OOM)。
# 2) 必须 Python 3.12(/root/eyeenv3):vllm 0.27 硬依赖 flashinfer,而 flashinfer
#    在 py3.10 上 type[...] 崩、py3.11 上 array.array[...] 崩,3.12 才合法。
#    卸掉它 vllm 直接 ModuleNotFoundError;--enforce-eager 绕不开(模块级导入)。
# 3) 驱动必须 580.x:低于它 vllm 报 "driver too old";≥581 的 Vulkan 回归让 IsaacSim 崩。
export CUDA_VISIBLE_DEVICES=1
exec /root/eyeenv3/bin/vllm serve /root/eye_model \
  --port 8077 --served-model-name eye \
  --gpu-memory-utilization 0.90 --max-model-len 8192 --max-num-seqs 8
