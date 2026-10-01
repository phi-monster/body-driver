# -*- coding: utf-8 -*-
"""一批题 × 离线核的结果,逐题一行(markdown 表):题号、身体、要求、种子、步数、给脑的话、东西摆在哪、判据、离线核(每个摆法判得对不对、
RoboDojo 自己的管线在做成了的样子里给不给分、放上去 / 放进去落稳后量的)。
用法(箱上):python3 sum_batch.py /root/p8/qexam/batches/b1.json /root/p8/chk/bd_question_b1_x5 /root/p8/chk/bd_question_b1_humanoid …"""
import json
import os
import sys

batch = json.load(open(sys.argv[1]))
checked = {}
for d in sys.argv[2:]:
    p = os.path.join(d, "report.json")
    if not os.path.exists(p):
        continue
    for L in json.load(open(p))["layouts"]:
        checked[int(L["layout"])] = L


def judge(name, kw):
    if name == "is_lift":
        return "is_lift(RoboDojo 自带):比开局高 > %.0f cm" % (100 * kw["z_threshold"])
    if name == "bdq_next_to":
        return "bdq_next_to:A、B 桌面投影最近 ≤ %.0f cm,A 高度差 ≤ 2 cm,A 挪过 ≥ %.0f cm" % (100 * kw["gap"], 100 * kw["min_move"])
    if name == "bdq_turned":
        return "bdq_turned:绕竖轴转 ≥ 135°,歪 ≤ 30°,高度差 ≤ 2 cm"
    if name == "bdq_pushed":
        return "bdq_pushed:沿 %s 走 ≥ %.0f cm、横偏 ≤ %.0f cm,这一集从没被抬高过 %.0f cm" % (
            kw["dir"], 100 * kw["dist"], 50 * kw["dist"], 100 * kw["max_lift"])
    if name == "bdq_on":
        return ("bdq_on(in):A 中心在 B 投影里(离边 ≥ 1 cm),A 最低点高过 B 底 2 mm、不高过 B 顶,A 停了(< 3 cm/s)" if kw.get("mode") == "in" else
                "bdq_on(on):A 中心在 B 投影里(离边 ≥ 1 cm),A 最低点在 B 顶 −1.5 ~ +3 cm,A 停了(< 3 cm/s)")
    if name == "bdq_above":
        return "bdq_above:机身在 A 中心正上方 %.0f cm 以内、高出 A 顶 ≥ %.0f cm" % (100 * kw["xy_tol"], 100 * kw["clear"])
    return "%s %s" % (name, json.dumps(kw, ensure_ascii=False))


ok_n = 0
print("| 题 | 身体 | 要求 | 种子 | 步 | 给脑的话 | 东西(标签 = 东西 @ x, y 米) | 判据 | 离线核 |")
print("|---|---|---|---|---|---|---|---|---|")
for q in batch["questions"]:
    L = checked.get(q["seed"])
    objs = "; ".join("%s=%s @ %.2f, %.2f" % (o["label"], o["desc"], o["pos"][0], o["pos"][1]) for o in q["objects"])
    if L is None:
        res = "没核"
    elif not L.get("loaded"):
        res = "🔴 装不起来:%s" % (L.get("error", "")[:80])
    else:
        tests = ";".join("%s → %d%s" % (t["state"], t["got"], "" if t["ok"] else "(该 %d)🔴" % t["expect"]) for t in L["tests"])
        extra = ""
        if "on_after_settle" in L:
            a = L["on_after_settle"]
            extra = ";落稳后 A 最低点比 B 底高 %.1f mm、比 B 顶高 %.1f mm%s" % (1000 * a["a_low_minus_b_low"], 1000 * a["a_low_minus_b_top"],
                                                                     "(竖着放进去)" if a.get("upright") else "")
        res = "%s · %s · 管线 %s%s" % ("对" if L.get("ok") else "🔴", tests, L.get("pipeline_reward_in_success_state"), extra)
        ok_n += 1 if L.get("ok") else 0
    name, kw = q["check"]
    print("| %d | %s | %s | %d | %d | %s | %s | %s | %s |" % (q["qid"], q["body"], q["requirement"], q["seed"], q["steps"], q["sentence"], objs,
                                                            judge(name, kw), res))
print()
print("离线核过了 %d / %d 题" % (ok_n, len(batch["questions"])))
