#!/usr/bin/env python3
# 验收视频(只做展示,不喂给驱动):开机"两只手同时碰桌面量指尖"那一段。
# 上:头顶眼;下:两只腕眼。按驱动自己的几何(身体文件 .geo.json + 逐拍运动学 fk_poses.txt + 日志里每一瓣的尖)把最后量出来的每一瓣指尖
# 投进头顶眼和它自己那只腕眼,倒回去画在每一帧上(核对用):手压着桌子时圆圈正好在碰桌的那个指尖上 = 量对了。字幕是驱动那一刻在做什么(日志)。
# 用法:touchvideo.py <炮目录> <身体文件.geo.json> <输出.mp4> [标题]
import sys, re, json, subprocess, numpy as np
from PIL import Image, ImageDraw, ImageFont

RUN, GEO, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
TITLE = sys.argv[4] if len(sys.argv) > 4 else RUN.rstrip("/").split("/")[-1]
FONT = "/root/fonts/NotoSansSC.otf"
F_BIG = ImageFont.truetype(FONT, 22)
F_SM = ImageFont.truetype(FONT, 17)

geo = {c["cam"]: c for c in json.load(open(GEO))["cams"]}
head = next(c for c in geo.values() if c.get("fixed"))

def qR(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z); w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

def pixel(g, pc):   # 驱动的约定:眼系 x 右、y 上、z 朝后;u = cx + f·x/(−z),v = cy − f·y/(−z);径向畸变 K1 / K2
    z = -pc[2]
    if z <= 1e-6: return None
    x, y = pc[0] / z, pc[1] / z
    r2 = x * x + y * y; d = 1 + g["k1"] * r2 + g["k2"] * r2 * r2
    return (g["cx"] + g["f"] * x * d, g["cy"] - g["f"] * y * d)

# 逐拍:图号、驱动按关节读数算的每只手的腕眼位姿
img_of, fk = {}, {}
for l in open(RUN + "/vid/poses.txt"):
    f = l.split()
    if len(f) >= 2: img_of[int(f[0])] = int(f[1])
for l in open(RUN + "/vid/fk_poses.txt"):
    f = l.split()
    if len(f) >= 16: fk[int(f[0])] = [float(x) for x in f[2:16]]

lines = open(RUN + "/cal.log", encoding="utf-8", errors="ignore").read().split("\n")
i0 = next(i for i, l in enumerate(lines) if "同时碰桌面量指尖" in l)
i1 = next(i for i, l in enumerate(lines) if "同时碰完" in l)
cams = {}   # 手 → 它的腕眼(日志"第 K 只手 … 第 C 台相机";没有就按顺序 1、2)
for l in lines[:i0]:
    m = re.search(r"第 ?(\d+) 只手.*第 ?(\d+) 台相机", l)
    if m and "长在这只手上" in l: cams[int(m.group(1))] = int(m.group(2))
for h in (1, 2): cams.setdefault(h, h)

def label(t):
    if "⇒ 碰到" in t and "轻碰" in t: return "轻碰碰到 —— 就在这一刻读位姿"
    if "⇒ 碰到" in t: return "粗找碰到了面"
    if "轻碰" in t: return "轻碰:一档一档往下"
    if "大步" in t and "找" in t: return "粗找:一大步一大步往下"
    if "小步" in t and "找" in t: return "粗找:一小步一小步往下"
    if "等自己那只眼里画面停下" in t: return "等手指回过来"
    if "朝下" in t and ("转" in t or "斜" in t): return "转手,让一瓣手指朝下"
    if "指尖碰桌面量好" in t: return "指尖量好"
    if "量不成" in t: return "这一瓣没量成"
    return None

events = {1: [], 2: []}     # (拍, 字幕)
tips = {1: {}, 2: {}}       # 手 → {瓣: 眼系坐标}
tip_beat = {}               # 手 → 指尖量好的那一拍
last_beat = {1: None, 2: None}
b_first, b_last = None, None
for l in lines[i0:i1 + 1]:
    m = re.search(r"〔手(\d)〕", l)
    if not m: continue
    h = int(m.group(1))
    mb = re.search(r"· 拍 (\d+)→(\d+)", l)
    if mb:
        s, e = int(mb.group(1)), int(mb.group(2))
        last_beat[h] = e
        b_first = s if b_first is None else min(b_first, s)
        b_last = e if b_last is None else max(b_last, e)
    mt = re.search(r"第 (\d+) 瓣的尖在眼系 \(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\)", l)
    if mt:
        tips[h][int(mt.group(1))] = np.array([float(mt.group(2)), float(mt.group(3)), float(mt.group(4))])
        if last_beat[h] is not None: tip_beat[h] = last_beat[h]
        continue
    lb = label(l)
    if lb and last_beat[h] is not None:
        events[h].append((last_beat[h], lb))

def pgm(beat, c):
    p = "%s/vid/f%06d_c%d.pgm" % (RUN, img_of[beat], c)
    with open(p, "rb") as f:
        d = f.read()
    parts = d.split(b"\n", 3)
    w, hh = map(int, parts[1].split())
    return Image.frombytes("L", (w, hh), parts[3][:w * hh]).convert("RGB")

COL = {1: (255, 70, 70), 2: (70, 200, 255)}
def tip_world(h, beat, t):
    v = fk[beat][7 * (h - 1):7 * h]
    R = qR(*v[3:7]); g = geo[cams[h]]
    Rce = np.array(g["r_ce"]).reshape(3, 3); off = np.array(g["off"])
    return np.array(v[:3]) + R @ off + (R @ Rce) @ t, R @ Rce

def frame(beat, freeze_note=None):
    H = pgm(beat, 0)
    W1, W2 = pgm(beat, cams[1]), pgm(beat, cams[2])
    dh, d1, d2 = ImageDraw.Draw(H), ImageDraw.Draw(W1), ImageDraw.Draw(W2)
    for h, dw in ((1, d1), (2, d2)):
        if tips[h] and beat in fk:
            for k, t in sorted(tips[h].items()):
                pw, Rc = tip_world(h, beat, t)
                Rh = np.array(head["r_ce"]).reshape(3, 3)
                p = pixel(head, Rh.T @ (pw - np.array(head["pos"])))
                if p:
                    dh.ellipse([p[0] - 8, p[1] - 8, p[0] + 8, p[1] + 8], outline=COL[h], width=3)
                q = pixel(geo[cams[h]], t)   # 腕眼里:尖就在眼系里
                if q:
                    dw.ellipse([q[0] - 12, q[1] - 12, q[0] + 12, q[1] + 12], outline=COL[h], width=4)
    canvas = Image.new("RGB", (640, 480 + 240 + 96), (18, 18, 18))
    canvas.paste(H, (0, 96))
    canvas.paste(W1.resize((320, 240)), (0, 576)); canvas.paste(W2.resize((320, 240)), (320, 576))
    d = ImageDraw.Draw(canvas)
    d.text((10, 6), TITLE + "  ·  拍 %d" % beat, font=F_BIG, fill=(235, 235, 235))
    for h, y in ((1, 38), (2, 64)):
        cur = [e for b, e in events[h] if b <= beat]
        txt = "第 %d 只手:%s" % (h, cur[-1] if cur else "—")
        txt += "(圆圈 = 最后量出来的指尖,画回每一帧核对)" if not (h in tip_beat and beat >= tip_beat[h]) else "(圆圈 = 量出来的指尖)"
        d.text((10, y), txt, font=F_SM, fill=COL[h])
    if freeze_note:
        d.rectangle([0, 96, 640, 126], fill=(0, 0, 0))
        d.text((10, 100), freeze_note, font=F_SM, fill=(255, 230, 120))
    return canvas

beats = [b for b in range(b_first, b_last + 1) if b in img_of]
tail = [b for b in range(b_last + 1, b_last + 40) if b in img_of and b in fk]
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "640x816", "-r", "10", "-i", "-",
                       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "24", "-preset", "veryfast", OUT], stdin=subprocess.PIPE)
for b in beats + tail:
    ff.stdin.write(frame(b).tobytes())
if tail:
    last = frame(tail[-1], "量完,手回到原处:圆圈是驱动自己量出来的每一瓣指尖,应该正好落在手指尖上")
    for _ in range(40):
        ff.stdin.write(last.tobytes())
ff.stdin.close(); ff.wait()
print("写好", OUT, "· 拍", b_first, "→", b_last, "· 手 → 腕眼", cams, "· 指尖量好在拍", tip_beat, "· 每只手的瓣数", {h: len(t) for h, t in tips.items()})
