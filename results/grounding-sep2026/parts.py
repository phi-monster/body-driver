import base64, io, json, re, urllib.request
from PIL import Image, ImageDraw
URL="http://127.0.0.1:8078/v1/chat/completions"
frames={"sc1_head":"/root/NSC1/vid/f000000_c0.pgm","sc4_head":"/root/NSC4/vid/f000000_c0.pgm","sc4_wristL":"/root/NSC4/vid/f000000_c1.pgm","sc1_wristR":"/root/NSC1/vid/f000000_c2.pgm"}
parts=["handles of the scissors","blades of the scissors","robot gripper fingers"]
def ask(img, what):
    buf=io.BytesIO(); img.save(buf, format="PNG"); b64=base64.b64encode(buf.getvalue()).decode()
    body={"model":"eye","temperature":0,"max_tokens":160,"chat_template_kwargs":{"enable_thinking":False},
          "messages":[{"role":"user","content":[{"type":"image_url","image_url":{"url":"data:image/png;base64,"+b64}},
          {"type":"text","text":f"Locate the {what} in this image. Answer with JSON only: {{\"found\": true, \"bbox_2d\": [x1, y1, x2, y2]}} or {{\"found\": false}}."}]}]}
    req=urllib.request.Request(URL, data=json.dumps(body).encode(), headers={"Content-Type":"application/json"})
    return json.load(urllib.request.urlopen(req, timeout=300))["choices"][0]["message"]["content"]
cols=[(255,0,0),(0,200,0),(255,160,0)]
for name,path in frames.items():
    im=Image.open(path).convert("RGB"); W,H=im.size; d=ImageDraw.Draw(im)
    for q,c in zip(parts,cols):
        t=ask(im.copy() if False else Image.open(path).convert("RGB"),q)
        boxes=re.findall(r"\[\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\]", t)
        print(name,"|",q,"|",t.replace("\n"," ")[:150], flush=True)
        for b in boxes:
            x1,y1,x2,y2=[float(v) for v in b]; d.rectangle([x1*W/1000,y1*H/1000,x2*W/1000,y2*H/1000], outline=c, width=2)
    im.save(f"/root/_m3/parts_{name}.jpg", quality=85)
