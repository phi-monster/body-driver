# In-box measurement: given ONLY the brain box, can plain pixel statistics inside it recover the thing and its long axis?
import json, math
from PIL import Image, ImageDraw
import numpy as np
R=json.load(open("/root/_m3/results.json"))
import re
frames={"sc1_head_gray":"/root/NSC1/vid/f000000_c0.pgm","sc2_head_gray":"/root/NSC2/vid/f000000_c0.pgm","sc3_head_gray":"/root/NSC3/vid/f000000_c0.pgm","sc4_head_gray":"/root/NSC4/vid/f000000_c0.pgm","sc4_wristL_gray":"/root/NSC4/vid/f000000_c1.pgm"}
for name,path in frames.items():
    t=R[name]["scissors"]; m=re.search(r"\[\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\]", t)
    g=np.asarray(Image.open(path).convert("L")).astype(float); H,W=g.shape
    x1,y1,x2,y2=[float(v) for v in m.groups()]; x1,x2=x1*W/1000,x2*W/1000; y1,y2=y1*H/1000,y2*H/1000
    pad=6; X1,Y1,X2,Y2=[int(max(0,v)) for v in (x1-pad,y1-pad,min(W,x2+pad),min(H,y2+pad))]
    roi=g[Y1:Y2,X1:X2]
    # background = the ring of pixels on the box border (the table right around the thing); foreground = pixels that differ from it by more than the ring own spread
    ring=np.concatenate([roi[0,:],roi[-1,:],roi[:,0],roi[:,-1]]); mu,sd=np.median(ring),ring.std()+1e-6
    fg=np.abs(roi-mu)>3*sd
    ys,xs=np.nonzero(fg); n=len(xs)
    cx,cy=xs.mean(),ys.mean(); C=np.cov(np.vstack([xs-cx,ys-cy])); w,v=np.linalg.eigh(C); ax=v[:,1]; ang=math.degrees(math.atan2(ax[1],ax[0]))
    L=2*math.sqrt(w[1]); S=2*math.sqrt(w[0])
    print(f"{name}: box {x2-x1:.0f}x{y2-y1:.0f}px  ring mu={mu:.0f} sd={sd:.1f}  fg={n}px ({100*n/roi.size:.0f}% of box)  centre=({X1+cx:.1f},{Y1+cy:.1f})  long-axis angle={ang:.1f}deg  half-lengths {L:.1f}/{S:.1f}px  elong={L/S:.1f}")
    im=Image.open(path).convert("RGB"); d=ImageDraw.Draw(im)
    for yy,xx in zip(ys,xs): im.putpixel((X1+int(xx),Y1+int(yy)),(0,255,0))
    d.rectangle([x1,y1,x2,y2],outline=(255,0,0)); d.line([X1+cx-ax[0]*L,Y1+cy-ax[1]*L,X1+cx+ax[0]*L,Y1+cy+ax[1]*L],fill=(255,255,0),width=2)
    im.crop((max(0,X1-60),max(0,Y1-60),min(W,X2+60),min(H,Y2+60))).resize((4*(min(W,X2+60)-max(0,X1-60)),4*(min(H,Y2+60)-max(0,Y1-60)))).save(f"/root/_m3/axis_{name}.jpg",quality=85)
