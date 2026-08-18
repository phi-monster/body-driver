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
        vecs, sigs = [], []
        for p, q in runs:
            m = q.get(name)
            if m is None:
                continue
            v = m.get("value", [])
            u = m.get("uncertainty", [0.0] * len(v))
            dim = m.get("dim") or len(v)
            d = max(1, min(int(dim), len(v)))
            vecs.append([float(x) for x in v[:d]])
            sigs.append([abs(float(x)) for x in u[:d]])
        n = len(vecs)
        if n == 0:
            rows.append((name, 0, "—", "—", "从没量到"))
            continue
        d = min(len(v) for v in vecs)
        med = [st.median([v[i] for v in vecs]) for i in range(d)]
        # 🔴 **按它自己报的 σ 量,不按相对百分比。**
        # 相对百分比在**中位数接近零**的分量上会爆炸(实测:home_pose 报 96152%、
        # backlash 报 4113%,而它们的中位数是 −0.31 和 −0.000153)—— 那是度量的毛病,
        # 不是数据的毛病,而它会让人把"没变化"读成"大幅回退"。
        # σ 是估计器自己给的、带单位的,在零附近照样有意义;而"跨炮差了几个 1σ"
        # 同时也在**审计那个 σ**:σ 诚实的话,跨炮该落在 1–2 个 σ 之内。
        worst_z, worst_i = 0.0, 0
        for i in range(d):
            col = [v[i] for v in vecs]
            sp = max(col) - min(col)
            σ = st.median([s2[i] for s2 in sigs]) if sigs else 0.0
            z = sp / σ if σ > 1e-12 else (0.0 if sp < 1e-12 else float("inf"))
            if z > worst_z:
                worst_z, worst_i = z, i
        if n == 1:
            flag = "⚪ 只有一炮 —— 谈不上复不复现"
        elif all(v == vecs[0] for v in vecs) and any(abs(x) > 1e-12 for x in vecs[0]):
            flag = f"🔴 {n} 炮【逐位一模一样】—— 查它读的是不是自己的命令"
        elif worst_z <= 2.0:
            flag = f"🟢 跨 {n} 炮复现(最差 {worst_z:.1f} 个 1σ)"
        elif worst_z <= 6.0:
            flag = f"🟡 跨 {n} 炮有漂({worst_z:.1f} 个 1σ)"
        else:
            flag = f"🔴 跨 {n} 炮不复现({'∞' if worst_z == float('inf') else f'{worst_z:.0f}'} 个 1σ)· σ 本身可能是假的"
        shown = f"{med[0]:.6g}" + (f" (+{d-1} 维)" if d > 1 else "")
        z = "∞" if worst_z == float("inf") else f"{worst_z:.1f}σ"
        rows.append((name, n, shown, f"{z}(第{worst_i}维)", flag))
    w = max(len(r[0]) for r in rows)
    print(f"{'格':<{w}}  量到   中位数            最差跨炮分歧   判定")
    for name, n, med, z, flag in rows:
        print(f"{name:<{w}}  {n:>2}/{len(runs)}  {med:<16}  {z:>12}  {flag}")
    got = sum(1 for r in rows if r[1] > 0)
    solid = sum(1 for r in rows if "🟢" in r[4])
    print(f"\n绝对数:15 格里 **{got} 格有数**,其中 **{solid} 格跨【多炮】复现**(≥2 炮且最差分歧 ≤2 个 1σ)")

if __name__ == "__main__":
    main(sys.argv[1:])
