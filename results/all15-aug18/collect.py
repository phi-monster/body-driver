#!/usr/bin/env python3
"""把 N 次独立自标定的输出合成一张【跨炮复现】表。零 GPU 可重跑。

用法:collect.py <full0.json> <full1.json> ...
每格给出:量到几次 / 中位数 / 相对散布 (max-min)/|median| / 每次的值。
🔴 复现性看的是**跨炮**,不是一炮内的 σ —— 一炮内的 σ 只说明拟合紧,
   说明不了"它是这具身体的一个常数"。
"""
import json, sys, statistics as st

FIFTEEN = ["step_delivery","home_pose","latency","backlash","friction","reach",
           "contact_threshold","floor","arm_weight","hand_pixel","image_jacobian",
           "self_occlusion","tool_offset","tool_axis_column","gripper_span"]

def load(p):
    try:
        d = json.load(open(p))
    except Exception:
        return None
    return d.get("quantities", {})

def main(paths):
    runs = [(p, load(p)) for p in paths]
    runs = [(p, q) for p, q in runs if q is not None]
    print(f"读到 {len(runs)} 炮\n")
    rows = []
    for name in FIFTEEN:
        vecs = []
        for p, q in runs:
            m = q.get(name)
            if m is None:
                continue
            v = m.get("value", [])
            dim = m.get("dim") or len(v)
            d = max(1, min(int(dim), len(v)))
            vecs.append([float(x) for x in v[:d]])
        n = len(vecs)
        if n == 0:
            rows.append((name, 0, "—", "—", "从没量到"))
            continue
        # 🔴 逐维比,不是只比第 0 维 —— 只比第 0 维会把一个 24 维的量判成"完全一样"。
        d = min(len(v) for v in vecs)
        med = [st.median([v[i] for v in vecs]) for i in range(d)]
        rel = 0.0
        for i in range(d):
            col = [v[i] for v in vecs]
            sp = max(col) - min(col)
            base = abs(med[i])
            rel = max(rel, sp / base if base > 1e-12 else (0.0 if sp == 0 else float("inf")))
        # 🔴 **一炮谈不上复现。** 这条不加,n=1 会被印成"散布 0.0% 🟢",而那是假的。
        if n == 1:
            flag = "⚪ 只有一炮 —— 谈不上复不复现"
        elif all(v == vecs[0] for v in vecs) and any(abs(x) > 1e-12 for x in vecs[0]):
            flag = f"🔴 {n} 炮【逐位一模一样】—— 查它读的是不是自己的命令"
        elif rel < 0.02:
            flag = f"🟢 跨 {n} 炮复现(最大相对散布 <2%)"
        elif rel < 0.20:
            flag = f"🟡 跨 {n} 炮有漂(2–20%)"
        else:
            flag = f"🔴 跨 {n} 炮不复现(>20%)"
        shown = f"{med[0]:.6g}" + (f" (+{d-1} 维)" if d > 1 else "")
        rows.append((name, n, shown, ("inf" if rel == float("inf") else f"{rel*100:.1f}%"), flag))
    w = max(len(r[0]) for r in rows)
    print(f"{'格':<{w}}  量到   中位数            跨炮相对散布  判定")
    for name, n, med, rel, flag in rows:
        print(f"{name:<{w}}  {n:>2}/{len(runs)}  {med:<16}  {rel:>10}  {flag}")
    got = sum(1 for r in rows if r[1] > 0)
    solid = sum(1 for r in rows if "🟢" in r[4])
    print(f"\n绝对数:15 格里 **{got} 格有数**,其中 **{solid} 格跨【多炮】复现**(≥2 炮且最大相对散布 <2%)")

if __name__ == "__main__":
    main(sys.argv[1:])
