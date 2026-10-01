#!/usr/bin/env python3
# 对照:同样两道换成厘米(没有 MY RULER 那句),看错是不是单位带来的
import json, urllib.request
QS = [
 ("I commanded a step of 5.0 cm toward the thing and my hand only went 0.8 cm. What most likely happened? One sentence.", "blocked / something stopped it"),
 ("The scissors are 24 cm away from my fingertips. Each step I take moves my hand at most 4.5 cm. How many steps at least do I need to reach them? Answer with a number first.", "6"),
 # 同一道步数题换个说法再问一次指尖长(看是不是它这一类算术本来就不稳)
 ("The scissors are 2.4 hand-lengths away from my fingertips. One step moves my hand 0.45 hand-lengths or less. What is 2.4 divided by 0.45, and so how many whole steps at least? Answer with the division first.", "5.33 → 6"),
]
for q, want in QS:
    body = {"model": "eye", "max_tokens": 120, "temperature": 0, "chat_template_kwargs": {"enable_thinking": False},
            "messages": [{"role": "user", "content": "You ARE this robot.\n\n" + q}]}
    req = urllib.request.Request("http://127.0.0.1:8078/v1/chat/completions", data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    r = json.loads(urllib.request.urlopen(req, timeout=120).read())
    a = r["choices"][0]["message"]["content"].strip().replace("\n", " ")
    print("问:", q[:90], "…\n  标准答案:", want, "\n  它答:", a[:300], "\n")
