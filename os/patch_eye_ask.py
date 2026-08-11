"""Add an `/ask` endpoint to the eye server.  `/point` is not touched.

WHY A SECOND ENDPOINT AND NOT A BIGGER PROMPT ON THE OLD ONE.  `/point` takes a NOUN and returns a
PIXEL, and it has no way to say "the thing you named is not in this picture" -- `noun_rules` either
finds a noun or refuses the sentence, and once a noun is found the model always answers with a
point.  Measured consequence, 5 episodes / 4 legs / 4 nouns / 4 layouts, unanimous: the answer sat
22.8-50.5 px from `target0` and 732-996 px from `target1` in a 640 px frame -- the referent was not
in the picture at all and the eye had no way to say so.  An interface that cannot express "not yet"
makes that failure structural, not a matter of model quality.

`/ask` passes a caller-written question through verbatim and returns whatever the model said, plus
a parsed point when the reply contains one.  The DECISION about what the reply means stays with the
caller (the thin OS), which is the same split `/point` uses: the collector ranks, the estimator
decides.
"""
import re
import sys

P = "/root/l3eye42/eye_server.py"
s = open(P).read()
if "/ask" in s:
    print("already patched")
    sys.exit(0)

# a raw generate that takes the prompt verbatim, beside _answer which formats PROMPT with a noun
anchor = "def _salvage(raw):"
assert anchor in s, "anchor not found"
new_fn = '''def _answer_raw(im, prompt, maxtok=None):
    """The model, asked the caller's question verbatim.  No noun rule, no prompt template."""
    msgs = [{"role": "user", "content": [{"type": "image", "image": im},
                                         {"type": "text", "text": prompt}]}]
    try:
        text = PROC.apply_chat_template(msgs, tokenize=False, add_generation_prompt=True,
                                        enable_thinking=False)
    except TypeError:
        text = PROC.apply_chat_template(msgs, tokenize=False, add_generation_prompt=True)
    inp = PROC(text=[text], images=[im], return_tensors="pt").to(MODEL.device)
    torch.cuda.synchronize()
    t1 = time.time()
    with torch.no_grad():
        out = MODEL.generate(**inp, max_new_tokens=int(maxtok or MAXTOK), do_sample=False)
    torch.cuda.synchronize()
    ms = (time.time() - t1) * 1000.0
    raw = PROC.batch_decode(out[:, inp["input_ids"].shape[1]:], skip_special_tokens=True)[0]
    return raw, ms


'''
s = s.replace(anchor, new_fn + anchor, 1)

# route /ask before the /point check
anchor2 = '''    def do_POST(self):
        if not self.path.startswith("/point"):
            return self._send(404, {"error": "no such path"})'''
assert anchor2 in s, "post anchor not found"
new_post = '''    def do_POST(self):
        if self.path.startswith("/ask"):
            return self._ask()
        if not self.path.startswith("/point"):
            return self._send(404, {"error": "no such path"})'''
s = s.replace(anchor2, new_post, 1)

# the handler itself, added just before do_POST
anchor3 = "    def do_POST(self):"
ask_fn = '''    def _ask(self):
        """Free-form question about THIS frame.  Returns the reply verbatim plus a parsed point if
        one is present -- and `uv=None` with the text intact is a legitimate answer, not an error."""
        try:
            n = int(self.headers.get("Content-Length", "0"))
            h = int(self.headers["X-H"]); w = int(self.headers["X-W"])
            prompt = urllib.parse.unquote(self.headers.get("X-Prompt", ""))
            mt = self.headers.get("X-Maxtok")
            body = b""
            while len(body) < n:
                chunk = self.rfile.read(n - len(body))
                if not chunk:
                    break
                body += chunk
            arr = np.frombuffer(body, np.uint8)
            if arr.size != h * w * 3:
                return self._send(400, {"error": "body %d != %d" % (arr.size, h * w * 3)})
            im = Image.fromarray(arr.reshape(h, w, 3))
        except Exception as e:
            return self._send(400, {"error": "%s: %s" % (type(e).__name__, e)})
        if not prompt.strip():
            return self._send(400, {"error": "X-Prompt is empty"})
        try:
            raw, ms = _answer_raw(im, prompt, mt)
        except Exception as e:
            return self._send(500, {"error": "%s: %s" % (type(e).__name__, e)})
        uvn, kind = rd_eye._parse_point(raw)
        if uvn is None:
            uvn, kind = _salvage(raw)
        uv = None if uvn is None else [float(uvn[0]) * w, float(uvn[1]) * h]
        _N["calls"] += 1
        _N["ms"] += ms
        print("[ask] %.0fms uv=%s raw=%r" % (ms, uv, raw[:90].replace(chr(10), " ")), flush=True)
        return self._send(200, {"uv": uv, "kind": kind, "raw": raw, "ms": round(ms, 1),
                                "wh": [w, h], "model": MID})

'''
s = s.replace(anchor3, ask_fn + anchor3, 1)
open(P, "w").write(s)
print("patched /ask into", P)
