#!/usr/bin/env python3
"""A fake brain service, written only from docs/brain-service.md (not from the driver's source), to check the wiring.

It never makes the body move: the program question gets one `say` line, the "where is it" question always gets
"I cannot see it here". A brain that writes motion for the body is not allowed (owner); this one only proves that
a service built from the document is understood by the driver.

    python3 tools/fake_brain.py 8097            # then: body_driver --listen 9080 --eye 127.0.0.1:8097 ...

Every request is printed as one line: which question (from the request's shape, as the document describes it) and
which keys it carried, so the run can be checked against the document afterwards.
"""
import json
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer

SAY = "say I am a fake brain that only checks the wiring, so I will not move you\n"
NOT_HERE = {"found": False, "bbox_2d": [0, 0, 0, 0]}
COUNT = {"program": 0, "locate": 0, "other": 0}


def reply(content):
    return {"choices": [{"index": 0, "message": {"role": "assistant", "content": content}, "finish_reason": "stop"}]}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        try:
            req = json.loads(self.rfile.read(n).decode("utf-8"))
        except Exception as e:
            self._send(400, {"error": "bad json: %s" % e})
            return
        if self.path != "/v1/chat/completions":
            self._send(404, {"error": "no such path"})
            return
        msgs = req.get("messages", [])
        parts = msgs[0].get("content", []) if msgs else []
        has_image = any(p.get("type") == "image_url" and str(p.get("image_url", {}).get("url", "")).startswith("data:image/bmp;base64,")
                        for p in parts)
        text = " ".join(p.get("text", "") for p in parts if p.get("type") == "text")
        if "response_format" in req:                  # section 3: "where is it" - a json_schema named where_is_it
            kind = "locate"
            name = text.split("\n", 1)[0].replace("Locate what someone would call: ", "")
            out = reply(json.dumps(NOT_HERE))
            detail = "name=%r temperature=%r" % (name, req.get("temperature"))
        elif "structured_outputs" in req:             # section 2: write a program - a grammar for constrained decoding
            kind = "program"
            out = reply(SAY)
            g = req["structured_outputs"].get("grammar", "")
            detail = "grammar %d chars, prompt %d chars, sampling keys %s" % (
                len(g), len(text), sorted(k for k in req if k not in ("model", "messages", "structured_outputs", "chat_template_kwargs")))
        else:
            kind = "other"
            out = reply("")
            detail = "keys %s" % sorted(req)
        COUNT[kind] += 1
        print("[fake brain] %s #%d model=%r image=%s %s" % (kind, COUNT[kind], req.get("model"), has_image, detail), flush=True)
        self._send(200, out)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8097
    print("[fake brain] listening on 127.0.0.1:%d" % port, flush=True)
    HTTPServer(("127.0.0.1", port), Handler).serve_forever()
