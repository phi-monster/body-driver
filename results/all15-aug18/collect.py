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
        vals = []
        for p, q in runs:
            m = q.get(name)
            if m is None:
                continue
            v = m.get("value", [])
            dim = m.get("dim") or len(v)
            vals.append(tuple(round(x, 6) for x in v[:max(1, min(dim, 4))]))
        n = len(vals)
        if n == 0:
            rows.append((name, 0, "—", "—", "从没量到"))
            continue
        first = [t[0] for t in vals]
        med = st.median(first)
        spread = (max(first) - min(first))
        rel = spread / abs(med) if med else float("inf")
        # 完全相同到浮点位 = 可疑(读的可能是自己的命令,不是身体)
        flag = ""
        if n > 1 and len(set(first)) == 1:
            flag = "🔴 每炮【一模一样】到浮点位 —— 查它读的是不是自己的命令"
        elif rel < 0.02:
            flag = "🟢 跨炮复现(相对散布 <2%)"
        elif rel < 0.20:
            flag = "🟡 跨炮有漂(2–20%)"
        else:
            flag = "🔴 跨炮不复现(>20%)"
        rows.append((name, n, f"{med:.6g}", f"{rel*100:.1f}%", flag))
    w = max(len(r[0]) for r in rows)
    print(f"{'格':<{w}}  量到  中位数        跨炮相对散布  判定")
    for name, n, med, rel, flag in rows:
        print(f"{name:<{w}}  {n:>2}/{len(runs)}  {med:<12}  {rel:>10}  {flag}")
    got = sum(1 for r in rows if r[1] > 0)
    solid = sum(1 for r in rows if "🟢" in r[4])
    print(f"\n绝对数:15 格里 **{got} 格有数**,其中 **{solid} 格跨炮复现**(相对散布 <2%)")

if __name__ == "__main__":
    main(sys.argv[1:])
