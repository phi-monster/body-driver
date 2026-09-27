#!/usr/bin/env bash
# 新版 Ada(问格点 + 多视图一起解 + 轨迹点三角)按旧炮落盘的图离线重跑:配点 → kinexam → alignexam → 真值打分
R=$1; D=/root/NV1B$R; B=/root/body-layer/driver/bin; L=$D/lookada; A=$D/alada
mkdir -p $L $A
for f in $D/look/*; do n=$(basename $f); case $n in kinem_arm*|corrs_arm*) ;; *) ln -sf $f $L/$n;; esac; done
for a in 0 1; do timeout 900 /root/venv_inst/bin/python -W ignore /root/diag/gridcorrs.py $D $a $L; done
T0=$(date +%s)
for a in 0 1; do timeout 1800 $B/kinexam $L $a $L $a > $L/kinexam_arm$a.txt 2>&1 & done; wait
echo "V1B$R 运动学两只手一起:$(( $(date +%s) - T0 )) 秒"
for a in 0 1; do grep "焦距 起步\|多视图\|一起解" $L/kinexam_arm$a.txt | cut -c1-240 | head -3; done
timeout 1800 $B/alignexam $L $A 127.0.0.1 8077 > $A/out.txt 2>&1
cp $L/kinem_arm*.txt $A/
grep "三角出\|放进世界:不长\|一起精修以后\|对齐用了" $A/out.txt | cut -c1-220
timeout 1200 /root/venv_inst/bin/python -W ignore /root/diag/alignstudy.py $D $D:$A 2>&1 | grep -v "^   手"
