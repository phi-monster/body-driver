# -*- coding: utf-8 -*-
"""RoboDojo 通关机器的题单(路 8,大并行 §2 第 42 条):RoboDojo 自己的每个任务文件一集(owner 10-01)。

箱上跑(不起 Isaac):python3 make_rdexam.py /root/RoboDojo --batch rd54
出来 /root/p8/qexam/batches/rd54.json,跑法和随机题一样:python3 run_questions.py --batch rd54(一题一集、走排队、数分、录像挪进落盘、清帧)。
- 任务 = RoboDojo 仓库里 git 记着的 task/RoboDojo/tasks/*.py(54 个:42 个 + 12 个 _random;config/ 下 55 个 yml 多的那个是 _task.yml,
  所有任务共用的底子,不是任务 —— owner 口径的 55 就是数了 config/);箱上后加的(bootcal、chase_mouse、bd_*)不算;
- 身体 x5(CFG arx_x5),种子 0,这个任务种子 0 下的第一张布局(RoboDojo 一个种子目录里的布局连着跑,跑完第一集、写出 _result.json 就放锁);
- 步数用各任务自己写死的(题单里不给 steps,跑的时候不设 BD_STEP_LIM;general_pickup 读它,run.sh 默认 200 = 官方);
- 题单里 guard = false(RoboDojo 是最后单独考的那一场,不进守门的一套),keep_video = true(owner 看每集的录像判)。
"""
import argparse
import json
import os
import re
import subprocess

ap = argparse.ArgumentParser()
ap.add_argument("root", nargs="?", default="/root/RoboDojo")
ap.add_argument("--batch", default="rd54")
ap.add_argument("--out", default="/root/p8/qexam/batches")
ap.add_argument("--start", type=int, default=1000, help="第一题的题号(和随机题的题号分开)")
args = ap.parse_args()
R = args.root

files = subprocess.run(["git", "-C", R, "ls-files", "task/RoboDojo/tasks/"], capture_output=True, text=True, check=True).stdout.split()
tasks = sorted(os.path.basename(f)[:-3] for f in files if f.endswith(".py") and not os.path.basename(f).startswith("_"))
cfgs = subprocess.run(["git", "-C", R, "ls-files", "task/RoboDojo/config/"], capture_output=True, text=True, check=True).stdout.split()
ymls = sorted(os.path.basename(f)[:-4] for f in cfgs if f.endswith(".yml"))
assert set(ymls) - set(tasks) == {"_task"} and set(tasks) <= set(ymls), "任务文件和配置对不上:%s" % (set(ymls) ^ set(tasks))
qs = []
for k, t in enumerate(tasks):
    lay = os.path.join(R, "Assets/Eval_Layout/RoboDojo/arx_x5/0")
    n = len([f for f in os.listdir(lay) if re.fullmatch(r"%s_\d+\.json" % re.escape(t), f)])
    assert n > 0, f"{t}:种子 0 下没有布局"
    src = open(os.path.join(R, "task/RoboDojo/tasks", t + ".py"), encoding="utf-8").read()
    m = re.search(r"def gen_instruction.*?\n(.*?)\n\s*return", src, re.S)
    instr = re.findall(r"[\"']([A-Z][^\"']{8,})[\"']", m.group(1)) if m else []
    qs.append({"qid": args.start + k, "seed": 0, "task": t, "body": "x5", "cfg": "arx_x5", "cfg_name": "arx_x5", "requirement": "robodojo",
               "sentence": instr[0] if instr else "(任务自己出的话)", "check": ["RoboDojo 自己的判据", {"layouts_in_seed0": n}], "steps": None,
               "objects": []})
batch = {"batch": args.batch, "bodies": ["x5"], "guard": False, "keep_video": True, "questions": qs}
os.makedirs(args.out, exist_ok=True)
json.dump(batch, open(os.path.join(args.out, args.batch + ".json"), "w"), indent=1, ensure_ascii=False)
print("RoboDojo 任务文件 %d 个(config/ 下 %d 个 yml,多的那个是 _task.yml)→ %s" % (len(tasks), len(ymls), os.path.join(args.out, args.batch + ".json")))
for q in qs:
    print("%5d  %-36s 种子 0 下 %2d 张布局  %s" % (q["qid"], q["task"], q["check"][1]["layouts_in_seed0"], q["sentence"][:90]))
