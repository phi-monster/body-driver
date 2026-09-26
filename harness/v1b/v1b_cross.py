#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨炮考试:拿一次开机(例 V1B2)解出来的模型 —— 转轴、焦距、参照读数,连同打分用的"模型系 → 世界"对齐 —— 一个数不改,
去算另一次开机(例 V1B1)录像里每一个静止帧手在哪(只给那一帧的关节读数),和那一炮仿真报的手的位姿比。
同一台机器人、同一个场景(底座在世界里不动),另一次开机的姿势,那一炮一帧都没参与拟合。
用法:v1b_cross.py <模型所在的跑的目录> <被考的跑的目录>"""
import sys, os
import numpy as np
sys.argv = [sys.argv[0], sys.argv[1]] + sys.argv[2:]
import v1b_fit as V

SRC, DST = sys.argv[1], sys.argv[2]
for arm in (0, 1):
    p = os.path.join(SRC, "v1b", "model_arm%d.npz" % arm)
    if not os.path.exists(p):
        print("臂 %d:没有模型 %s" % (arm, p)); continue
    z = np.load(p)
    model = dict(W=z["W"], P=z["P"], Rg=z["Rg"], tg=z["tg"], s=float(z["s"]), Rx=z["Rx"], tx=z["tx"])
    print("\n===== 臂 %d:模型来自 %s(焦距 %.1f),考 %s 的录像" % (arm, SRC, float(z["f"]), DST))
    V.eval_frames(arm, int(z["gi"]), z["q0"], z["Qtrain"], model, run=DST)
