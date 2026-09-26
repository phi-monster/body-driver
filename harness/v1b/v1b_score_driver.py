#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V1b 3b 打分:驱动开机自己量出来的运动学(look/kinem_arm<k>.txt:每根轴的方向 W、过的点 P、焦距、参照读数,驱动的相机约定 -z 朝前)
拿仿真真值打分 —— 驱动一个真值都没看。对齐(模型系 → 世界的相似变换 + 眼离手的偏移)只用扫描帧的真值拟合;
考试 = 全部板停 + 录像里的静止帧(按离扫描帧多远分档、按不同姿势数报)。用法:v1b_score_driver.py <跑的目录>"""
import sys, os
import numpy as np
sys.argv = [sys.argv[0], sys.argv[1], ""]
import v1b_fit as V
RUN = sys.argv[1]
V.RUN = RUN
groups = V.load_groups()
stops = V.load_stops() if os.path.exists(os.path.join(RUN, "look", "board_stops.txt")) else []
sweep = V.load_sweep()
for arm in (0, 1):
    p = os.path.join(RUN, "look", "kinem_arm%d.txt" % arm)
    if not os.path.exists(p):
        print("臂 %d:驱动没落盘运动学" % arm); continue
    L = open(p).read().split("\n")
    h = L[0].split()
    f = float(h[h.index("f") + 1])
    q0 = np.array([float(x) for x in L[1].split()[1:]])
    W, P = [], []
    for l in L[2:]:
        if l.startswith("axis"):
            v = [float(x) for x in l.split()[2:]]
            W.append(v[:3]); P.append(v[3:])
    W = np.array(W); P = np.array(P); n = len(W)
    side = "left" if arm == 0 else "right"
    gi = [i for i, g in enumerate(groups) if g.startswith("state.") and side in g][0]
    SW = [s for s in sweep if s["arm"] == arm]
    BS = [s for s in stops if s["arm"] == arm]
    S = SW + BS
    Q = np.array([s["groups"][gi] for s in S]); dQ = Q - q0
    train = list(range(len(SW))); test = list(range(len(SW), len(S)))
    print("\n===== 臂 %d:驱动量的焦距 %.1f(真 397),扫描 %d 帧拟合对齐,板停 %d 个考试" % (arm, f, len(SW), len(BS)))
    Pw = [(s["pose"][:3], V.quat_to_R(s["pose"][3:])) for s in S]
    Rg, tg, s_, Rx, tx, etr, atr, ete, ate = V.align_eval(W, P, dQ, train, test if test else train[:1], Pw, np.random.default_rng(0))
    V.eval_frames(arm, gi, q0, Q[train], dict(W=W, P=P, Rg=Rg, tg=tg, s=s_, Rx=Rx, tx=tx))
