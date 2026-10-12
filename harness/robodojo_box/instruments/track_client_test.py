# -*- coding: utf-8 -*-
import base64, json, urllib.request, time
def post(path, obj):
    r = urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8077" + path, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"}), timeout=120)
    return json.loads(r.read())
def b64(p): return base64.b64encode(open(p, "rb").read()).decode()
t = time.time(); s = post("/track/start", {"image": b64("/root/instruments/f350_c2.png"), "points": [[292, 240], [150, 205], [355, 200], [300, 120]]}); print("start", s, "%.2fs" % (time.time() - t))
for k in (400, 550, 600):
    t = time.time(); r = post("/track/step", {"id": s["id"], "image": b64("/root/instruments/f%d_c2.png" % k)})
    print("f%d" % k, [[round(x, 1) for x in p] for p in r["points"]], "%.0f ms wall %.2fs" % (r["ms"], time.time() - t))
print(post("/track/end", {"id": s["id"]}))
