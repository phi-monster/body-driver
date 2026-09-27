#!/usr/bin/env bash
# 一炮的离线全套:两只手的跨格轨迹 → 多视图一起解 → 对齐回放 → 打分
R=$1; D=/root/NV1B$R
for a in 0 1; do timeout 600 /root/venv_inst/bin/python -W ignore /root/diag/tracks.py $D $a 2>&1 | grep "配点" ; done
for a in 0 1; do timeout 900 /root/venv_inst/bin/python -W ignore /root/diag/vpba.py $D $a 60 2>&1 | grep "^驱动的模型\|^多视图" | sed "s/^/V1B$R 手 $a /" | cut -c1-200; done
mkdir -p $D/lookvp $D/alvp
for f in $D/look/*; do n=$(basename $f); case $n in kinem_arm*) ;; *) ln -sf $f $D/lookvp/$n;; esac; done
cp $D/vp/kinem_arm0.txt $D/vp/kinem_arm1.txt $D/lookvp/
timeout 900 /root/body-layer/driver/bin/alignexam $D/lookvp $D/alvp 127.0.0.1 8077 > $D/alvp/out.txt 2>&1
cp $D/vp/kinem_arm*.txt $D/alvp/
timeout 1200 /root/venv_inst/bin/python -W ignore /root/diag/alignstudy.py $D $D:$D/alvp 2>&1 | grep -v "^   手"
