# -*- coding: utf-8 -*-
"""把离线核的 report.json 汇成一张表:python3 sum_checks.py /root/p8/chk [任务…]"""
import json
import os
import sys

root = sys.argv[1] if len(sys.argv) > 1 else "/root/p8/chk"
names = sys.argv[2:] or sorted(d for d in os.listdir(root) if os.path.isfile(os.path.join(root, d, "report.json")))
for n in names:
    p = os.path.join(root, n, "report.json")
    if not os.path.isfile(p):
        print(n, "没有 report.json")
        continue
    r = json.load(open(p))
    print("%s  ok=%s  用时 %ss" % (n, r.get("ok"), r.get("total_s")))
    for L in r["layouts"]:
        if not L.get("loaded"):
            print("   布局 %d 装不起来:%s" % (L["layout"], L.get("error")))
            continue
        drift = max([o.get("drift_m", 0.0) for o in L.get("objects", [])] or [0.0])
        bad = [t["state"] for t in L["tests"] if not t["ok"]]
        extra = []
        for k in ("joint_after_hold", "peg_bottom_below_mouth_after_60", "trigger_after_release", "cloth_flat_max_above_table", "cloth_set_path"):
            if k in L:
                extra.append("%s=%s" % (k, round(L[k], 4) if isinstance(L[k], float) else L[k]))
        if "trigger_release_on_table_deg_every_2_steps" in L:
            extra.append("扳机松开 %s°" % L["trigger_release_on_table_deg_every_2_steps"])
        if "walk" in L:
            w = L["walk"]
            extra.append("每步中位 %.4f m(%.4f–%.4f)出界 %s" % (w["per_step_median_m"], w["per_step_min_m"], w["per_step_max_m"], not w["inside_region"]))
        print("   布局 %d:装 %ss · 东西 %d 件都在=%s · 最大漂 %.4f m · 判据 %d/%d 对%s · 管线判 %s · %s" % (
            L["layout"], L.get("reset_s"), len(L.get("objects", [])), all(o["present"] for o in L.get("objects", [])), drift,
            sum(t["ok"] for t in L["tests"]), len(L["tests"]), ("(错:%s)" % bad) if bad else "", L.get("pipeline_reward_in_success_state"),
            " · ".join(extra)))
