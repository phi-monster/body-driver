#!/usr/bin/env python3
# ④ 验脑看不看得懂"指尖长"(驱动给脑的那句 MY RULER 原样当前提;纯文字、温度 0;标准答案写在题后,人判)
import json, urllib.request
RULER = ("MY RULER: every length I tell you is in hand-lengths. One hand-length is the distance from my eye to my fingertips, "
         "which I measured myself by touching the table. I have no other ruler and I do not know centimetres.")
QS = [
 ("My jaw opens 0.90 hand-lengths. The cup in front of me is 1.30 hand-lengths wide. Can I close my open jaw around the whole cup? Answer 'yes' or 'no' first, then one sentence why.", "no"),
 ("After my last move my fingertips are 0.03 hand-lengths from the point I wanted. Is my hand basically at that point, or still far from it? Answer 'at it' or 'far' first, then one sentence.", "at it"),
 ("I commanded a step of 0.50 hand-lengths toward the thing and my hand only went 0.08 hand-lengths. What most likely happened? One sentence.", "blocked / something stopped it"),
 ("The scissors are 2.4 hand-lengths away from my fingertips. Each step I take moves my hand at most 0.45 hand-lengths. How many steps at least do I need to reach them? Answer with a number first.", "6"),
 ("Thing A is 0.6 hand-lengths from my fingertips, thing B is 1.8 hand-lengths away. Which is closer, and about how many times farther is the other one? Answer first with A or B.", "A, 3 times"),
]
for q, want in QS:
    body = {"model": "eye", "max_tokens": 120, "temperature": 0, "chat_template_kwargs": {"enable_thinking": False},
            "messages": [{"role": "user", "content": "You ARE this robot. " + RULER + "\n\n" + q}]}
    req = urllib.request.Request("http://127.0.0.1:8078/v1/chat/completions", data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    r = json.loads(urllib.request.urlopen(req, timeout=120).read())
    a = r["choices"][0]["message"]["content"].strip().replace("\n", " ")
    print("问:", q[:90], "…\n  标准答案:", want, "\n  它答:", a[:300], "\n")
