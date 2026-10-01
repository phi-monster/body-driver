# -*- coding: utf-8 -*-
"""一炮从零量出来的身体,按仿真真值打分(路 8):给人形、无人机新量的身体文件(P8S、P8T)和第 34 条五种相机条件(P8V*)用。
- 手在哪:跑 /root/diag/v1b_score_fk_cur.py <炮的目录>(主代理的 V1b 打分:驱动按关节读数算的手的位姿 vs 仿真真值,先按身体自己的尺寸定倍数再比);
  这一炮得是 qboot.sh 带 BOOT_KEEP=1 开的(look/ 和 vid/ 都留着);
- 眼:身体文件 .geo.json 里每台相机的焦距、主点、畸变 vs 真值(RoboDojo 相机配置算出来的:焦距 = 画幅宽 × focal_length / horizontal_aperture;
  主点 = 画幅中心;畸变 = 钩子加的那一份,没加就是 0)。
用法(箱上):python3 score_boot.py P8S [--distort=-0.15,0.03](负号开头的值要写成 = 连着,不然 argparse 当成另一个选项)
"""
import argparse
import json
import os
import subprocess

ap = argparse.ArgumentParser()
ap.add_argument("shot")
ap.add_argument("--distort", default="0,0", help="钩子加的 k1,k2(没加 = 0,0)")
ap.add_argument("--truth_f", default="cam_head=288.1,cam_left_wrist=397.0,cam_right_wrist=397.0,cam_wrist=397.0",
                help="每台相机的真焦距(px):RoboDojo env_cfg/camera/template.py 的 Gemini_345Lg 10.0 / 22.212 × 640、d435 13.0 / 20.955 × 640")
args = ap.parse_args()
K = args.shot
k = K.lower()
RUN = f"/root/N{K}"
out = {"shot": K}
geo = f"/root/p8/cal_{k}.json.geo.json"
names = []
for l in open(f"{RUN}/cal.log", errors="replace"):
    if l.startswith("[认] 相机:"):
        names = [c.strip().split(".")[1] for c in l.split(":", 1)[1].split("·")]
        break
tf = dict(kv.split("=") for kv in args.truth_f.split(","))
k1t, k2t = (float(v) for v in args.distort.split(","))
rows = []
if os.path.exists(geo):
    g = json.load(open(geo))
    for c in g["cams"]:
        nm = names[c["cam"]] if c["cam"] < len(names) else "cam%d" % c["cam"]
        ft = float(tf[nm]) if nm in tf else None
        rows.append({"cam": c["cam"], "name": nm, "f": c["f"], "f_true": ft, "f_err_pct": round(100 * (c["f"] - ft) / ft, 3) if ft else None,
                     "cx": c["cx"], "cy": c["cy"], "k1": c["k1"], "k2": c["k2"], "k1_true": k1t, "k2_true": k2t, "fixed": c.get("fixed"),
                     "tip_valid": c.get("tip_valid"), "gap": c.get("gap")})
out["cams"] = rows
txt = open(f"{RUN}/cal.log", errors="replace").read()
out["boot_steps"] = next((int(l.split("用了")[1].split("拍")[0]) for l in txt.splitlines() if "开机量身体一共用了" in l), None)
out["round1"] = "── 第 1 轮" in txt
# 打分脚本的第一种考法要逐帧的位姿(vid/fk_poses.txt、vid/poses.txt);开机炮默认不录(BOOT_VID 才录,一炮近 1 GB)⇒ 没录就给两份空的,
# 那一种考法就不考(脚本自己按"0 只手"跳过),第二种考法(扫描各格的运动学、走到没去过的地方、指尖、桌面、头顶眼)照常
vid = os.path.join(RUN, "vid")
out["per_frame_poses"] = os.path.exists(os.path.join(vid, "fk_poses.txt"))
if not out["per_frame_poses"]:
    os.makedirs(vid, exist_ok=True)
    for f in ("fk_poses.txt", "poses.txt"):
        open(os.path.join(vid, f), "a").close()
sc = subprocess.run(["/venv/RoboDojo/bin/python", "/root/diag/v1b_score_fk_cur.py", RUN], capture_output=True, text=True, timeout=3000)
out["v1b_score"] = (sc.stdout + sc.stderr).strip().splitlines()
json.dump(out, open(f"/root/p8/boot/{K}/score.json", "w"), indent=1, ensure_ascii=False)
print("== %s:开机 %s 拍,到第 1 轮 %s" % (K, out["boot_steps"], out["round1"]))
for r in rows:
    print("   相机 %d %-16s 焦距 %.1f(真 %s,差 %s%%)主点 (%.1f, %.1f) 畸变 k1 %.4f k2 %.4f(加的 %.2f / %.2f)%s" % (
        r["cam"], r["name"], r["f"], r["f_true"], r["f_err_pct"], r["cx"], r["cy"], r["k1"], r["k2"], k1t, k2t, " 不动的眼" if r["fixed"] else ""))
for l in out["v1b_score"]:
    print("   " + l)
