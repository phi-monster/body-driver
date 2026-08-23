set -e
echo "=== 装 vllm ==="
python3 -m venv /root/eyeenv 2>/dev/null || true
/root/eyeenv/bin/pip install -q --upgrade pip
/root/eyeenv/bin/pip install -q vllm
echo "VLLM OK $(/root/eyeenv/bin/python -c "import vllm;print(vllm.__version__)")"
echo "=== 下模型 ==="
/root/eyeenv/bin/pip install -q huggingface_hub[hf_transfer]
HF_HUB_ENABLE_HF_TRANSFER=1 /root/eyeenv/bin/huggingface-cli download Qwen/Qwen3.5-9B --local-dir /root/eye_model
echo "MODEL OK"
