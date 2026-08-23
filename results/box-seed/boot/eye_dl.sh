set -e
export HF_HUB_ENABLE_HF_TRANSFER=1
/root/eyeenv/bin/hf download Qwen/Qwen3.5-9B --local-dir /root/eye_model
echo "MODEL OK"
