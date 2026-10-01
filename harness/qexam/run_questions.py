#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""随机题机的跑法(路 8):一题一集,每集都走排队(/root/q/run.sh,主线驱动;BL_BIN / BL_HOME 给了就用那一份),
这一集 RoboDojo 一写出结果就放锁;每集存落盘和成败,最后按身体 × 要求汇总成功率;做成过的题记进守门的一套(guard.json),以后每批重跑。
不往驱动里加一个字;题、判据都在布局里(make_questions.py 出的)。

用法(箱上):
    python3 run_questions.py --batch b1                 # 这一批全跑
    python3 run_questions.py --batch b1 --qids 3,4      # 只跑这几题
    python3 run_questions.py --guard                    # 守门的一套全跑
    python3 run_questions.py --summary --batch b1       # 只汇总已经跑过的
身体文件:每集从下面这几份(或 --body x5=路径)拷一份新的再装回(一集和一集之间不带经验),驱动量到的写回拷的那一份。
"""
import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--batch", default="")
ap.add_argument("--qids", default="")
ap.add_argument("--guard", action="store_true")
ap.add_argument("--summary", action="store_true")
ap.add_argument("--lim", type=int, default=30, help="一集最多占排队位几分钟")
ap.add_argument("--home", default="/root/p8/qexam", help="题单、结果、每集落盘放这儿")
ap.add_argument("--body", action="append", default=[], help="身体=身体文件,比如 x5=/root/cal_v1b78.json")
ap.add_argument("--keep_video", action="store_true", help="留 RoboDojo 这一集的录像(挪进 runs/<炮名>/;默认删)")
ap.add_argument("--shard", default="", help="k/n:这一份只跑第 k 份(从 0 数,共 n 份);有几个仿真位就起几个,各跑一份")
ap.add_argument("--path", default="8", help="排队时报的路号")
args = ap.parse_args()
# 身体文件:x5 用 /root/cal_v1b78.json(装回来开机 68 拍);人形、无人机原来的 /root/cal_h4.json、/root/cal_dr2.json 是 09-28 的旧格式,
# 现在的驱动不认、每集都从零量(人形 710 拍,比题的步数还多)⇒ 10-01 用当时的主线驱动在 bootcal 上从零各量了一份(炮 P8P、P8R)
BODY_FILE = {"x5": "/root/cal_v1b78.json", "humanoid": "/root/p8/cal_p8p.json", "drone": "/root/p8/cal_p8r.json"}
for kv in args.body:
    k, v = kv.split("=", 1)
    BODY_FILE[k] = v
RD = "/root/RoboDojo"
os.makedirs(f"{args.home}/runs", exist_ok=True)
os.makedirs(f"{args.home}/results", exist_ok=True)
GUARD = f"{args.home}/guard.json"


def load_batch(name):
    return json.load(open(f"{args.home}/batches/{name}.json"))


BATCH_FLAGS = {}   # 题单上的:guard(做成了进不进守门的一套,默认进;RoboDojo 那一批不进)、keep_video(默认看 --keep_video)


def questions():
    if args.guard:
        g = json.load(open(GUARD)) if os.path.exists(GUARD) else {}
        out = []
        for key, ent in sorted(g.items()):
            qs = {q["qid"]: q for q in load_batch(ent["batch"])["questions"]}
            out.append((ent["batch"], qs[ent["qid"]]))
        return out
    b = load_batch(args.batch)
    BATCH_FLAGS.update({k: b[k] for k in ("guard", "keep_video") if k in b})
    want = {int(s) for s in args.qids.split(",") if s != ""}
    out = [(args.batch, q) for q in b["questions"] if not want or q["qid"] in want]
    if args.shard:
        k, n = (int(v) for v in args.shard.split("/"))
        out = out[k::n]
    return out


def copy_body(src, dst):
    shutil.copy(src, dst)
    for f in glob.glob(src + ".geo.json") + glob.glob(src + ".kin.txt") + glob.glob(src + ".kin.txt_*.bmp"):
        shutil.copy(f, dst + f[len(src):])


def tidy(shot, out):
    """这一炮的目录:驱动日志 cal.log(cal 开头的不删)、look/ 里的文字留下,第一张、最后一张给脑看的图转成 jpg 留下,别的删"""
    n = f"/root/N{shot}"
    look = f"{n}/look"
    grids = sorted(glob.glob(f"{look}/grid_*.bmp"))
    try:
        from PIL import Image
        for tag, g in (("first", grids[:1]), ("last", grids[-1:])):
            if g:
                Image.open(g[0]).convert("RGB").save(f"{out}/{tag}_grid.jpg", quality=85)
    except Exception as e:   # PIL 不在就不转图
        print("  图没转:", e)
    for t in glob.glob(f"{look}/*.txt"):
        shutil.copy(t, out)
    sim = f"{n}/sim.log"
    if os.path.exists(sim):
        keep = [l for l in open(sim, errors="replace") if re.search(r"Traceback|Error|Exception|Unstable", l)
                and not re.search(r"omni\.kit\.test|CXXABI|libXt|omni\.graph|MaterialX|usdBakeMtlx|circular import", l)]
        open(f"{out}/sim_errors.txt", "w").writelines(keep[:60])
    for p in glob.glob(f"{n}/*"):
        base = os.path.basename(p)
        if base.startswith("cal") or base.startswith("经历"):
            continue
        shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)


def run_one(batch, q):
    shot = None
    for k in range(1000):
        cand = "P8Q%d_%d" % (q["qid"], k)
        if not os.path.exists(f"/root/N{cand}") and not os.path.exists(f"{args.home}/runs/{cand}"):
            shot = cand
            break
    out = f"{args.home}/runs/{shot}"
    os.makedirs(out)
    cal = f"{out}/cal_{shot.lower()}.json"
    copy_body(BODY_FILE[q["body"]], cal)
    env = dict(os.environ, CAL=cal, BL_LIFE=f"{out}/经历_{shot.lower()}.txt", CFG=q["cfg"], SEED=str(q["seed"]), DRVMODE="work", BL_VID="")
    if q.get("steps") is not None:   # RoboDojo 官方任务那一批不给:各任务用自己写死的步数(general_pickup 读 BD_STEP_LIM,run.sh 默认给 200 = 官方)
        env["BD_STEP_LIM"] = str(q["steps"])
    else:
        env.pop("BD_STEP_LIM", None)
    t0 = time.time()
    print("== 题 %d(%s / %s,种子 %d,%s 步)%s · 炮 %s" % (q["qid"], q["body"], q["requirement"], q["seed"],
                                                     q["steps"] if q.get("steps") is not None else "任务自己的", q["sentence"], shot), flush=True)
    log = open(f"{out}/run.log", "w")
    task = q.get("task", "bd_question")   # 小场景那一批(make_questions.py --scenes)是各自的任务;YCB 题都是 bd_question
    proc = subprocess.Popen(["bash", "/root/q/run.sh", args.path, shot, task, str(args.lim)], env=env, stdout=log, stderr=subprocess.STDOUT)
    res_glob = f"{RD}/eval_result/RoboDojo/{task}/l3_link/{q['cfg_name']}/{q['seed']}_/{shot}/_result.json"
    result = None
    started = None
    while proc.poll() is None:
        hit = glob.glob(res_glob)
        if hit:
            time.sleep(3)
            result = json.load(open(hit[0]))
            open(f"/root/q/done_{shot}", "w").close()
            break
        if started is None and os.path.exists(f"/root/N{shot}/cal.log"):
            started = time.time()
        time.sleep(10)
    proc.wait()
    wall = time.time() - (started or t0)
    calog = f"/root/N{shot}/cal.log"
    steps = rounds = boot = None
    if os.path.exists(calog):
        txt = open(calog, errors="replace").read()
        m = re.findall(r"这一集已用 (\d+) 拍", txt)
        steps = int(m[-1]) if m else None
        rounds = len(re.findall(r"── 第 \d+ 轮", txt))
        mb = re.findall(r"开机量身体一共用了 (\d+) 拍", txt)
        boot = int(mb[-1]) if mb else None   # 身体文件装回来是几十拍;几百拍 = 从零量了(身体文件格式旧了 / 对不上),这一集的步数大半花在开机上
    success = None
    if result is not None:
        det = list((result.get("details") or {}).values())
        success = bool(det[0]["success"]) if det else bool(result.get("success_rate", 0) > 0.5)
    rec = {"batch": batch, "qid": q["qid"], "seed": q["seed"], "body": q["body"], "requirement": q["requirement"], "sentence": q["sentence"],
           "success": success, "outcome": ("success" if success else "fail") if result is not None else "no_result",
           "steps_used": steps, "boot_steps": boot, "brain_rounds": rounds, "wall_s": round(wall), "shot": shot, "driver_log": calog,
           "finished": time.strftime("%Y-%m-%d %H:%M:%S")}
    json.dump(rec, open(f"{out}/result.json", "w"), indent=1, ensure_ascii=False)
    with open(f"{args.home}/results/{batch if not args.guard else 'guard'}.jsonl", "a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    tidy(shot, out)
    edir = os.path.dirname(res_glob.replace("_result.json", ""))
    if args.keep_video or BATCH_FLAGS.get("keep_video"):   # 录像挪进这一集的落盘(RoboDojo 按做成 / 没做成给文件名:episode_*.mp4)
        for v in glob.glob(f"{edir}/**/*.mp4", recursive=True):
            shutil.move(v, os.path.join(out, os.path.basename(v)))
        rec["videos"] = sorted(os.path.basename(v) for v in glob.glob(f"{out}/*.mp4"))
        json.dump(rec, open(f"{out}/result.json", "w"), indent=1, ensure_ascii=False)
    shutil.rmtree(edir, ignore_errors=True)
    for r in glob.glob(f"{os.path.dirname(edir)}/_resume_{shot}.json"):
        os.remove(r)
    if success and BATCH_FLAGS.get("guard", True):
        g = json.load(open(GUARD)) if os.path.exists(GUARD) else {}
        g.setdefault("%s/%d" % (batch, q["qid"]), {"batch": batch, "qid": q["qid"], "first_success": rec["finished"], "shot": shot})
        json.dump(g, open(GUARD, "w"), indent=1, ensure_ascii=False)
    print("   ⇒ %s · 用了 %s 拍、叫了 %s 次脑 · %d 秒" % (rec["outcome"], steps, rounds, wall), flush=True)
    return rec


def summary(recs):
    by = {}
    for r in recs:
        k = (r["body"], r["requirement"])
        by.setdefault(k, [0, 0])
        by[k][1] += 1
        by[k][0] += 1 if r["success"] else 0
    tot = sum(v[1] for v in by.values())
    ok = sum(v[0] for v in by.values())
    print("成功 %d / %d%s" % (ok, tot, (" = %.0f%%" % (100.0 * ok / tot)) if tot else ""))
    for (b, rq), (s, n) in sorted(by.items()):
        print("   %-9s %-8s %d / %d" % (b, rq, s, n))


if args.summary:
    f = f"{args.home}/results/{args.batch or 'guard'}.jsonl"
    summary([json.loads(l) for l in open(f)] if os.path.exists(f) else [])
    sys.exit(0)
recs = [run_one(b, q) for b, q in questions()]
summary(recs)
