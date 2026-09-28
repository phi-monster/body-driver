#!/usr/bin/env python3
# 离线台架(只做诊断,不喂给驱动):碰指尖那一段每一下"挪 … ⇒ 实到 …"按驱动逐拍的运动学位姿(fk_poses)对到拍上,
# 按仿真真值(poses.txt 的 link6 位姿 + x5 手指网格,张开)算出这一下结束时手指最低点离桌面多高,和驱动当时的判断摆在一起。
import sys, re, struct, numpy as np
RUN = sys.argv[1]
X5 = "/root/RoboDojo/Assets/Robots/x5/meshes"
TOP = 0.765
def stl(p):
    d = open(p, "rb").read(); n = struct.unpack("<I", d[80:84])[0]
    return np.array([struct.unpack("<12f", d[84 + 50 * i:84 + 50 * i + 48])[3:12] for i in range(n)]).reshape(-1, 3)
L7 = np.unique(stl(X5 + "/link7.STL").round(6), axis=0) + np.array([0.08657, 0.024896 + 0.044, -0.0002436])
L8 = np.unique(stl(X5 + "/link8.STL").round(6), axis=0) + np.array([0.08657, -0.0249 - 0.044, -0.00024366])
FING = np.vstack([L7, L8])
def qR(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = {}
for l in open(RUN + "/vid/poses.txt"):
    f = l.split()
    if len(f) >= 16:
        tr[int(f[0])] = np.array([float(x) for x in f[2:16]])
fk = {}
for l in open(RUN + "/vid/fk_poses.txt"):
    f = l.split()
    if len(f) >= 16:
        fk[int(f[0])] = np.array([float(x) for x in f[2:16]])
def low(seq, arm):
    v = tr.get(seq)
    if v is None: return None
    p = v[7 * arm:7 * arm + 3]; q = v[7 * arm + 3:7 * arm + 7]
    return float((FING @ qR(q).T + p)[:, 2].min() - TOP)
pat_mv = re.compile(r"〔手(\d)〕挪 \(([-0-9.]+) 单位,([-0-9.]+) 单位,([-0-9.]+) 单位\) ⇒ 实到 \(([-0-9.]+) 单位,([-0-9.]+) 单位,([-0-9.]+) 单位\),差 ([0-9.]+) 单位")
pat_hit = re.compile(r"〔手(\d)〕  (一大步一大步找|一小步一小步找|轻碰\(一档一档\)|退回碰到的那一大步开始的地方、一小步一小步找|接着一小步一小步找):第 (\d+) 步\(一步 ([0-9.]+) 单位\)少走 ([0-9.]+) 单位,空走时少走 ([0-9.]+) 单位\(门 ([0-9.]+) 单位\)")
pat_false = re.compile(r"〔手(\d)〕.*(虚的)")
lines = open(RUN + "/cal.log", encoding="utf-8", errors="ignore").read().split("\n")
start = next(i for i, l in enumerate(lines) if "同时碰桌面量指尖" in l)
end = next(i for i, l in enumerate(lines) if "同时碰完" in l)
seqs = sorted(fk)
nb = [int(re.search(r"开机量身体一共用了 (\d+) 拍", l).group(1)) for l in lines if "开机量身体一共用了" in l][0]
cur = {0: nb, 1: nb}
first_of = {0: True, 1: True}
rows = []
first = True
for i in range(start, end):
    l = lines[i]
    m = pat_mv.search(l)
    if m:
        h = int(m.group(1)) - 1; arm = h
        act = np.array([float(m.group(5)), float(m.group(6)), float(m.group(7))])
        cmd = np.array([float(m.group(2)), float(m.group(3)), float(m.group(4))])
        ms = re.search(r"· 拍 (\d+)→(\d+)", l)
        if ms:   # 驱动自己记了起止拍号(09-29 起)
            s, e = int(ms.group(1)), int(ms.group(2))
            err = float(np.linalg.norm(fk[e][7 * arm:7 * arm + 3] - fk[s][7 * arm:7 * arm + 3] - act)) if (s in fk and e in fk) else 9.0
            cur[h] = e
            rows.append(dict(line=i + 1, hand=h + 1, cmd=cmd, act=act, s=s, e=e, err=err, lo_s=low(s, arm), lo_e=low(e, arm), verdict=""))
            continue
        s0 = cur[h]
        best = None
        starts = range(s0 - 10, s0 + 60) if first_of[h] else range(s0, s0 + 3)
        for s in starts:
            if s not in fk: continue
            for e in range(s + 1, s + 100):
                if e not in fk: break
                d = fk[e][7 * arm:7 * arm + 3] - fk[s][7 * arm:7 * arm + 3]
                err = np.linalg.norm(d - act)
                if err < 0.0015:
                    if best is None or best[0] >= 0.0015 or e < best[2]:
                        best = (err, s, e)
                    break
                if best is None or (best[0] >= 0.0015 and err < best[0]):
                    best = (err, s, e)
            if best is not None and best[0] < 0.0015 and not first_of[h]:
                break
        first_of[h] = False
        if best is None: continue
        err, s, e = best
        cur[h] = e
        rows.append(dict(line=i + 1, hand=h + 1, cmd=cmd, act=act, s=s, e=e, err=err, lo_s=low(s, arm), lo_e=low(e, arm), verdict=""))
        continue
    m = pat_hit.search(l)
    if m:
        h = int(m.group(1))
        for r in reversed(rows):
            if r["hand"] == h:
                r["verdict"] = "碰到(%s 少走 %s 底 %s 门 %s)" % (m.group(2)[:4], m.group(5), m.group(6), m.group(7)); break
        continue
    m = pat_false.search(l)
    if m:
        h = int(m.group(1))
        for r in reversed(rows):
            if r["hand"] == h:
                r["verdict"] += " → 后来判虚"; break
H = sys.argv[2] if len(sys.argv) > 2 else None
bad_align = sum(1 for r in rows if r["err"] > 0.0015)
print("%s:碰指尖段 %d 下(对不上拍的 %d 下)" % (RUN, len(rows), bad_align))
for r in rows:
    if H and str(r["hand"]) != H: continue
    if abs(r["cmd"][2]) < 1e-6 or r["cmd"][0] ** 2 + r["cmd"][1] ** 2 > 1e-6: continue   # 只看竖着往下 / 往上的
    print("L%-5d 手%d 拍 %4d→%4d 要 %+.3f 实到 %+.3f 少走 %.3f · 真值:手指最低点离桌面 %s → %s mm %s" % (
        r["line"], r["hand"], r["s"], r["e"], r["cmd"][2], r["act"][2], abs(r["cmd"][2]) - abs(r["act"][2]),
        "-" if r["lo_s"] is None else "%.1f" % (1000 * r["lo_s"]), "-" if r["lo_e"] is None else "%.1f" % (1000 * r["lo_e"]), r["verdict"]))

# ── 按"一段往下找"汇总(一段 = 同一只手从上一个判断句到这一个判断句之间的竖直往下的那几步)──
pat_verdict = re.compile(r"〔手(\d)〕  (一大步一大步找|一小步一小步找|轻碰\(一档一档\)|退回碰到的那一大步开始的地方、一小步一小步找|接着一小步一小步找)[::].*?(⇒ 碰到|都没认出碰到)")
print()
print("── 每一段往下找:驱动在第几步说碰到 / 真值(手指最低点离桌面 ≤ 0.5 mm)第几步真碰上 ──")
stats = {}
last_line = {1: start, 2: start}
for i in range(start, end):
    m = pat_verdict.search(lines[i])
    if not m: continue
    h = int(m.group(1)); kind = m.group(2)
    kind = "大步" if kind.startswith("一大步") else ("轻碰" if kind.startswith("轻碰") else "小步")
    steps = [r for r in rows if r["hand"] == h and last_line[h] < r["line"] <= i + 1 and r["cmd"][2] < 0 and r["cmd"][0] ** 2 + r["cmd"][1] ** 2 < 1e-6]
    last_line[h] = i + 1
    if not steps: continue
    fired = m.group(3) == "⇒ 碰到"
    truth_k = next((k for k, r in enumerate(steps) if r["lo_e"] is not None and r["lo_e"] <= 0.0005), None)
    aligned = all(r["err"] < 0.0015 for r in steps)
    if fired and truth_k is None:
        cls = "虚认"
    elif fired:
        cls = "认对" if truth_k >= len(steps) - 2 else "晚认"
    else:
        cls = "漏认" if truth_k is not None else "都空"
    key = (kind, cls if aligned else cls + "?")
    stats[key] = stats.get(key, 0) + 1
    hs = " ".join("%.1f" % (1000 * r["lo_e"]) if r["lo_e"] is not None else "-" for r in steps[-6:])
    print("L%-5d 手%d %s %d 步 %s%s · 真值第 %s 步碰上 · 最后几步结束时离桌面 (mm) %s%s" % (
        i + 1, h, kind, len(steps), "说碰到" if fired else "没认出", "", "-" if truth_k is None else str(truth_k + 1), hs, "" if aligned else "  (有几步对不上拍)"))
print()
for k in sorted(stats):
    print("  %s %s:%d 段" % (k[0], k[1], stats[k]))
