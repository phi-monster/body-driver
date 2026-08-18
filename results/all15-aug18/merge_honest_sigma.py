#!/usr/bin/env python3
"""把 N 炮独立标定合成一份,**σ 取跨炮实测散布**。零 GPU 可重跑。

🔴 为什么必须这么做:一炮之内的残差**审计不了它自己**。一炮里样本一致 ⇒ 拟得很"精",
   而跨炮的那个变化**一炮之内根本看不见**。实测(2026-08-18,八炮):
   自报 σ 让接触阈跨炮差 **3513 个 1σ**、画面尺 **654 个**,而它们的【值】其实挺稳。
   ⇒ **精密 ≠ 可复现,而估计器只量了前者却按后者用。**

做法:每一格每一维,值取中位数,σ 取 max(自报 σ 的中位数, 跨炮标准差)。
      **只放大,不缩小** —— 跨炮一致不代表 σ 可以变小(可能是八炮同错)。

用法:merge_honest_sigma.py 出口.json 输入1.json 输入2.json ...
"""
import json, sys, statistics as st

def main(out, paths):
    runs = []
    for p in paths:
        try:
            runs.append(json.load(open(p)))
        except Exception:
            pass
    if not runs:
        print("没有能读的输入"); return
    names = set()
    for r in runs:
        names |= set(r.get("quantities", {}).keys())
    merged = {"fingerprint": "self-measured", "quantities": {}}
    print(f"合 {len(runs)} 炮\n")
    print(f"{'格':<20} {'炮数':>4}  {'自报 σ':>10}  {'跨炮 σ':>10}  {'放大':>8}")
    for name in sorted(names):
        ms = [r["quantities"][name] for r in runs if name in r.get("quantities", {})]
        if not ms: continue
        d = min(int(m.get("dim") or len(m["value"])) for m in ms)
        base = dict(ms[0])
        val, sig = [], []
        worst_own, worst_cross = 0.0, 0.0
        for i in range(d):
            col = [float(m["value"][i]) for m in ms]
            own = st.median([abs(float(m.get("uncertainty", [0]*d)[i])) for m in ms])
            cross = st.pstdev(col) if len(col) > 1 else 0.0
            val.append(st.median(col))
            sig.append(max(own, cross))          # 只放大,不缩小
            worst_own = max(worst_own, own)
            worst_cross = max(worst_cross, cross)
        v = list(base["value"]); u = list(base.get("uncertainty", [0.0]*len(v)))
        for i in range(d):
            v[i] = val[i]; u[i] = sig[i]
        base["value"] = v; base["uncertainty"] = u
        merged["quantities"][name] = base
        amp = worst_cross / worst_own if worst_own > 1e-12 else float("inf")
        print(f"{name:<20} {len(ms):>4}  {worst_own:>10.5f}  {worst_cross:>10.5f}  "
              f"{'∞' if amp==float('inf') else f'{amp:>7.1f}×'}")
    json.dump(merged, open(out, "w"), indent=2)
    print(f"\n写到 {out} —— {len(merged['quantities'])} 格,σ 已换成跨炮实测")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
