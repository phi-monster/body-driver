#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V1b 第 2 步离线:开机关节扫描(look/sweep.txt + sweep_*.bmp)+ 板停,只用关节读数 + 腕眼画面量出"关节转多少、腕眼到哪",焦距也自己量。
量法(一种):① 每个关节单独扫的那几格:转角已知,(轴方向, 轴在眼的哪边, 焦距) 铺网格 + 精修(sweep_init.one_joint);
② 各轴离眼远近的比例:不同关节的格子之间的配点(sweep_init.scales);③ 全部配点按像素一起解(v1b_fit.fit_px,焦距是未知数)。
仿真报的手的位姿只用来打分:训练帧上拟合"模型系 → 世界"的相似变换 + 眼离手的偏移;考试 = 留出的板停 + 录像里的静止帧(按离训练帧多远分档)。
用法:v1b_sweep.py <跑的目录> <身体文件.geo.json>(geo 只拿主点 cx, cy;焦距不用)"""
import sys, os, math, time, json, random
import numpy as np
import v1b_fit as V
import sweep_init as SI

RUN = sys.argv[1]; GEO = sys.argv[2]
FINGER_V = float(os.environ.get("FINGER_V", "250"))
NN = int(os.environ.get("NN", "6"))
PER_PAIR = int(os.environ.get("PER_PAIR", "200"))
ARMS = [int(x) for x in os.environ.get("ARMS", "0,1").split(",")]
V.RUN = RUN; V.OUT = os.path.join(RUN, "v1b"); V.FINGER_V = FINGER_V
os.makedirs(V.OUT, exist_ok=True)


def main():
    groups = V.load_groups()
    stops = V.load_stops()
    sweep = V.load_sweep()
    geo = json.load(open(GEO))["cams"]
    print("关节组:", groups, "· 板停", len(stops), "· 扫描帧", len(sweep))
    report = {}
    for arm in ARMS:
        side = "left" if arm == 0 else "right"
        gi = [i for i, g in enumerate(groups) if g.startswith("state.") and side in g][0]
        SW = [s for s in sweep if s["arm"] == arm]
        BS = [s for s in stops if s["arm"] == arm]
        if not SW:
            print("臂 %d:没有扫描帧" % arm); continue
        cam = BS[0]["cam"] if BS else (1 + arm)
        cx, cy = geo[cam]["cx"], geo[cam]["cy"]
        start = [s for s in SW if s["J"] == 0 and s["D"] == 0 and s["K"] == 0][0]
        S = [start] + [s for s in SW if s is not start] + BS
        nb0 = len(S) - len(BS)
        Q = np.array([s["groups"][gi] for s in S]); q0 = start["groups"][gi]; dQ = Q - q0
        n = Q.shape[1]
        others = [j for j in range(len(start["groups"])) if j != gi and groups[j].startswith("state.")]
        for j in others:
            dr = np.degrees(np.max(np.abs(np.array([s["groups"][j] for s in SW]) - start["groups"][j])))
            print("臂 %d 扫描时,另一组 %s 最多漂 %.3f°" % (arm, groups[j], dr))
        print("\n===== 臂 %d(%s,眼 %d):扫描 %d 帧 + 板停 %d;扫描里每个关节转过的范围(度):%s" % (
            arm, side, cam, nb0, len(BS), np.round(np.degrees(dQ[:nb0].max(0) - dQ[:nb0].min(0)), 1).tolist()))
        # 考试:板停每三停留一停
        test = [nb0 + k for k in range(len(BS)) if k % 3 == 2]
        train = [i for i in range(len(S)) if i not in test]
        # 配对:每个关节 起点↔每格、相邻两格;全部训练帧 关节最近邻 NN 个
        pairs = set()
        prev = {}
        for i in range(1, nb0):
            key = (S[i]["J"], S[i]["D"]); a_ = prev.get(key, 0)
            pairs.add((min(a_, i), max(a_, i))); pairs.add((0, i)); prev[key] = i
        for i in train:
            dd = sorted(((np.max(np.abs(dQ[i] - dQ[j])), j) for j in train if j != i))
            for _, j in dd[:NN]:
                pairs.add((min(i, j), max(i, j)))
        pairs = sorted(pairs)
        print("配对 %d" % len(pairs), flush=True)
        t0 = time.time()
        raw = []
        for k, (i, j) in enumerate(pairs):
            s = V.match(S[i]["n"], S[j]["n"], S[i].get("img"), S[j].get("img"))
            if len(s):
                keep = (s[:, 1] < FINGER_V) & (s[:, 3] < FINGER_V)
                s = s[keep]
            if len(s) >= 50:
                raw.append((i, j, s[:, 0:2].astype(float), s[:, 2:4].astype(float)))
            if k % 50 == 0:
                print("  配对 %d/%d(%.0f s)" % (k, len(pairs), time.time() - t0), flush=True)
        print("配得上的对 %d / %d" % (len(raw), len(pairs)))
        # ① 每个关节单独扫的那几格:焦距各关节共用 ⇒ 每档焦距下各关节最好的网格分数加起来取最小;焦距定了再逐个精修
        fgrid = np.geomspace(0.35, 1.75, 25) * (2 * cx)     # 视场 30°–110°(画幅宽 = 2 cx),每档约 7%
        th_list, pairs_list, frs = [], [], []
        # 起步的模型是"只有这一个关节在动";碰上东西那几格别的关节会被顶偏(V1B2:扫肩时肘偏到 22°)⇒ 起步只用别的关节几乎没动的格子:
        # 别的关节偏 δ 让画面挪 f·δ,要 < 1 像素(按最长那档焦距算,最严)。被顶偏的格子读数是真的,照样进最后一起解
        dmax = 1.0 / fgrid.max()
        for j in range(n):
            allj = [i for i in range(1, nb0) if S[i]["J"] == j]
            fr = [0] + [i for i in allj if np.max(np.abs(np.delete(dQ[i], j))) < dmax]
            if len(fr) - 1 < len(allj):
                print("  关节 %d:%d 格里有 %d 格别的关节被带偏(最多 %.2f°)⇒ 起步不用" % (j, len(allj), len(allj) - len(fr) + 1,
                      np.degrees(max(np.max(np.abs(np.delete(dQ[i], j))) for i in allj))))
            loc = {g: k for k, g in enumerate(fr)}
            pairs_list.append([(loc[i], loc[jj], a, b) for i, jj, a, b in raw if i in loc and jj in loc])
            th_list.append(dQ[fr, j]); frs.append(fr)
            print("  关节 %d:%d 帧 %d 对,转角 %.1f…%.1f°" % (j, len(fr), len(pairs_list[-1]), math.degrees(th_list[-1].min()), math.degrees(th_list[-1].max())), flush=True)
        t1 = time.time()
        f0, outs = SI.all_joints(th_list, pairs_list, cx, cy, fgrid, log=lambda s_: print(s_, flush=True), per_pair=PER_PAIR)
        Wj, Ph, fj = [], [], []
        for j, o in enumerate(outs):
            if o is None:
                print("  关节 %d:配得上的对太少 ⇒ 量不了" % j)
                Wj.append(np.array([0, 0, 1.0])); Ph.append(np.array([1.0, 0, 0])); fj.append(np.nan); continue
            w, ph, med, sc = o
            Wj.append(w); Ph.append(ph); fj.append(f0)
            print("  关节 %d ⇒ 轴 %s · 残差中位 %.3f px(网格 %.3f)" % (j, np.round(w, 3).tolist(), med, sc), flush=True)
        print("  起步焦距 %.1f(%.0f s)" % (f0, time.time() - t1), flush=True)
        Wj = np.array(Wj); Ph = np.array(Ph); f0 = float(np.nanmedian(fj))
        jo = [S[i]["J"] if (0 < i < nb0) else -1 for i in range(len(S))]
        rho, ref, med = SI.scales(Wj, Ph, f0, dQ, jo, [p for p in raw if p[0] in train and p[1] in train], cx, cy, per_pair=PER_PAIR)
        print("  各轴离眼远近的比例(以关节 %d 为 1):%s · 残差中位 %.3f px" % (ref, np.round(rho, 4).tolist(), med), flush=True)
        # ③ 按像素一起解:按起步模型挑内点(残差 < 3 px 或 3 倍中位)
        I_, J_, A_, B_ = SI.expand([p for p in raw if p[0] in train and p[1] in train], 10 ** 9)
        r0 = np.abs(SI.pair_res(Wj, Ph * rho[:, None], f0, dQ, I_, J_, A_, B_, cx, cy))
        gate = max(3.0, 3 * np.median(r0))
        meas = []
        for i, j, a, b in raw:
            if i in train and j in train:
                rr = np.abs(SI.pair_res(Wj, Ph * rho[:, None], f0, dQ, np.full(len(a), i), np.full(len(a), j), a, b, cx, cy))
                m = rr < gate
                if m.sum() >= 30:
                    meas.append(dict(i=i, j=j, a=a[m], b=b[m], par=1.0, R=np.eye(3)))
        print("  按起步模型挑内点(门 %.2f px):%d 对" % (gate, len(meas)))
        Wp, Pp, fp = V.fit_px(dQ, meas, train, n, Wj, Ph * rho[:, None], f0, cx, cy, per_pair=300)
        Pw = [(s["pose"][:3], V.quat_to_R(s["pose"][3:])) for s in S]
        Rg, tg, s_, Rx, tx, etr, atr, ete, ate = V.align_eval(Wp, Pp, dQ, train, test, Pw, np.random.default_rng(0))
        mp = dict(W=Wp, P=Pp, Rg=Rg, tg=tg, s=s_, Rx=Rx, tx=tx)
        ex = V.eval_frames(arm, gi, q0, Q[train], mp)
        report[arm] = dict(focal_start=f0, focal_joints=[float(x) for x in fj], focal=float(fp), train_med_mm=float(np.median(etr)),
                           test_med_mm=float(np.median(ete)), test_max_mm=float(ete.max()), extrap=ex, rho=rho.tolist(), ref=ref,
                           range_deg=np.degrees(dQ[:nb0].max(0) - dQ[:nb0].min(0)).round(1).tolist(), pairs=len(meas))
    json.dump(report, open(os.path.join(V.OUT, "report_sweep.json"), "w"), indent=1, ensure_ascii=False)
    print("\n结果存在", os.path.join(V.OUT, "report_sweep.json"))


if __name__ == "__main__":
    main()
