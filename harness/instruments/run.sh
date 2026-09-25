#!/usr/bin/env bash
# 起仪器进程(GPU0,端口 8077)。按 pid 文件杀旧的;pkill 的模式拆成字面量,免得杀到自己
cd /root/instruments || exit 1
if [ -f serve.pid ]; then kill -9 "$(cat serve.pid)" 2>/dev/null; fi
P=serve; P="${P}.py"
pkill -9 -f "bin/python $P" 2>/dev/null
# 杀掉之后口要过一会儿才放(2026-09-25 实测:等 1 s 还占着,新进程就没起)⇒ 最多等 30 s
for i in $(seq 1 30); do ss -ltn 2>/dev/null | grep -q ":8077 " || break; sleep 1; done
if ss -ltn 2>/dev/null | grep -q ":8077 "; then echo "🔴 8077 还占着"; ss -ltnp | grep ":8077 "; exit 3; fi
CUDA_VISIBLE_DEVICES=0 PYTHONIOENCODING=utf-8 nohup /root/venv_inst/bin/python serve.py > serve.log 2>&1 < /dev/null &
echo $! > serve.pid
disown
for i in $(seq 1 90); do ss -ltn 2>/dev/null | grep -q ":8077 " && break; sleep 1; done
ss -ltn | grep -q ":8077 " && echo "仪器进程起来了($i s,pid $(cat serve.pid))" || { echo "🔴 仪器进程没起来"; tail -5 serve.log; exit 2; }
