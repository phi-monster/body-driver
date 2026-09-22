# M3: can the project own brain (Qwen3.5-9B) point at a named thing directly, without numbered blobs?
import base64, io, json, sys, re, urllib.request, os
from PIL import Image, ImageDraw
URL="http://127.0.0.1:8078/v1/chat/completions"
OUT="/root/_m3"
frames = {
 "sc1_head_gray": "/root/NSC1/vid/f000000_c0.pgm",
 "sc2_head_gray": "/root/NSC2/vid/f000000_c0.pgm",
 "sc3_head_gray": "/root/NSC3/vid/f000000_c0.pgm",
 "sc4_head_gray": "/root/NSC4/vid/f000000_c0.pgm",
 "sc4_wristR_gray": "/root/NSC4/vid/f000000_c2.pgm",
 "sc4_wristL_gray": "/root/NSC4/vid/f000000_c1.pgm",
 "sc4_head_overlay_color": "/root/NSC4/look/grid_000001.bmp",
}
queries = ["scissors", "baseball", "jeans", "toy figure", "banana", "hammer"]   # last two are ABSENT (negative control)
def ask(img, what):
    buf=io.BytesIO(); img.save(buf, format="PNG"); b64=base64.b64encode(buf.getvalue()).decode()
    prompt=(f"Locate the {what} in this image. If it is there, answer with JSON only: "
            f"{{\"found\": true, \"bbox_2d\": [x1, y1, x2, y2]}}. If it is not in the image, answer {{\"found\": false}}.")
    body={"model":"eye","temperature":0,"max_tokens":120,
          "chat_template_kwargs":{"enable_thinking":False},
          "messages":[{"role":"user","content":[
              {"type":"image_url","image_url":{"url":"data:image/png;base64,"+b64}},
              {"type":"text","text":prompt}]}]}
    req=urllib.request.Request(URL, data=json.dumps(body).encode(), headers={"Content-Type":"application/json"})
    r=json.load(urllib.request.urlopen(req, timeout=300))
    return r["choices"][0]["message"]["content"]
res={}
for name,path in frames.items():
    if not os.path.exists(path): print("missing",path); continue
    im=Image.open(path).convert("RGB")
    if name.endswith("overlay_color"): im=im.crop((0,0,640,480))
    W,H=im.size
    can=im.copy(); d=ImageDraw.Draw(can)
    res[name]={}
    for q in queries:
        try: txt=ask(im,q)
        except Exception as e: txt="ERR "+repr(e)
        res[name][q]=txt
        print(name,"|",q,"|",txt.replace("\n"," ")[:160], flush=True)
        m=re.search(r"\[\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\]", txt)
        if m and "false" not in txt.lower():
            x1,y1,x2,y2=[float(g) for g in m.groups()]
            # interpretation A: normalised 0..1000 (red) ; interpretation B: absolute pixels (blue)
            d.rectangle([x1*W/1000,y1*H/1000,x2*W/1000,y2*H/1000], outline=(255,0,0), width=2)
            d.text((x1*W/1000+2,y1*H/1000+2), q, fill=(255,0,0))
            d.rectangle([x1,y1,x2,y2], outline=(0,80,255), width=1)
    can.save(f"{OUT}/{name}.jpg", quality=85)
json.dump(res, open(f"{OUT}/results.json","w"), indent=1)
