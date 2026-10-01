# -*- coding: utf-8 -*-
"""随机题机的出题(路 8,大并行 §5 路 8 / §6):随机一件东西 × 随机一个要求 × 随机一具身体,每题一句给脑的话 + 一张布局 + 一个判据。

箱上跑(RoboDojo 那个 venv 的 python,不起 Isaac):
    /venv/RoboDojo/bin/python make_questions.py /root/RoboDojo --batch b1 --n 30 --seed 1
出来的(都是新文件):
    task/RoboDojo/tasks/bd_question.py、task/RoboDojo/config/bd_question.yml(一个任务对所有题都一样:题写在每一集的布局里)
    task/RoboDojo/bd/question.py(判据)
    Assets/Eval_Layout/RoboDojo/<身体的配置>/<种子>/bd_question_0.json —— 一题一个种子目录(种子 = 20000 + 题号),一题一集、单独跑
    /root/p8/qexam/batches/<批>.json(题单:题号、种子、身体、要求、给脑的话、判据、步数),同一份拷回 harness/qexam/batches/
物件只用 make_pool.py 装的 Isaac YCB 物件(bdq_*),不用 RoboDojo 的物件、布局、任务。

身体(都是箱上现成的测试台,照它们本来的样子):
    x5 = 双臂 x5(CFG arx_x5);humanoid = G1 + 五指手(CFG g1_rgb,只能伸 8 cm 左右,题出在手指够得着的那一小块);
    drone = 无人机(CFG drone_rgb,只出"飞到它正上方"这一类)。
要求:lift(抬多高)、next_to(挪到另一件旁边)、turn(原地转过来)、push(往某个方向推过去,不许拿起来)、on(放到另一件上面 / 放进去)。
摆布局:东西不叠,离身体歇着的手够远(按离线核量到的连杆位置,见 harness/scenes/README.md),要两件的题开局两件离得远。
"""
import argparse
import json
import math
import os
import shutil

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("root", nargs="?", default="/root/RoboDojo")
ap.add_argument("--batch", required=True)
ap.add_argument("--n", type=int, default=30)
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--start", type=int, default=None, help="第一题的题号(默认接着已有的题号往后排)")
ap.add_argument("--bodies", default="x5,humanoid,drone")
ap.add_argument("--out", default="/root/p8/qexam/batches")
args = ap.parse_args()
R = args.root
HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = f"{R}/Assets/Object/RoboDojo/Rigid"
SEED0 = 20000
WRITTEN = []


def mine(path):
    rel = os.path.relpath(path, R) if path.startswith(R) else path
    ok = (not path.startswith(R)) or ("bd_question" in rel or rel.startswith("task/RoboDojo/bd/"))
    assert ok and ".." not in rel, f"不许写这个路径: {rel}"
    WRITTEN.append(rel)
    return path


# ---------------------------------------------------------------- 身体:配置、桌上能出题的那一块、歇着的手占的地方(离线核量的连杆位置)
TABLE_TOP = 0.765
BODIES = {
    # x5:腕 (±0.30, −0.352, 0.922),手指朝前平伸到 y ≈ −0.21;张口约 9 cm
    "x5": {"cfg": "arx_x5", "cfg_name": "arx_x5", "base": "arx_x5/0/bootcal_0.json", "zones": [[[-0.30, 0.30], [-0.18, 0.10]]],
           "keepout": [(-0.41, -0.19, -0.40, -0.17, 0.85), (0.19, 0.41, -0.40, -0.17, 0.85)], "grip": 0.085,
           "reqs": ["lift", "next_to", "turn", "push", "on"], "lift_cm": [5, 10, 15], "push_cm": [10], "pair_gap": [0.10, 0.40], "clutter": [1, 2]},
    # G1:腕 (±0.178, −0.448, 0.864),手指朝前到 y ≈ −0.238、高 0.834–0.894(最低的小指 0.834);能伸 8 cm 左右(rig_upright_low.py 的离线正解)
    #     ⇒ 题只出在两只手各自够得着的那一小块(两手中间够不着)
    "humanoid": {"cfg": "g1_rgb", "cfg_name": "g1", "base": "g1/1/bootcal_0.json", "zones": [[[-0.26, -0.10], [-0.30, -0.15]], [[0.10, 0.26], [-0.30, -0.15]]],
                 "keepout": [(-0.26, -0.10, -0.52, -0.20, 0.82), (0.10, 0.26, -0.52, -0.20, 0.82)], "grip": 0.08,
                 "reqs": ["lift", "next_to", "turn", "push", "on"], "lift_cm": [5, 10], "push_cm": [5], "pair_gap": [0.06, 0.16], "clutter": [0, 1]},
    # 无人机:龙门架的底座在 (0, −0.2, 1.4),机身在桌子上方飞;只出"飞到它正上方"
    "drone": {"cfg": "drone_rgb", "cfg_name": "drone", "base": "drone/1/bootcal_0.json", "zones": [[[-0.35, 0.35], [-0.25, 0.20]]],
              "keepout": [], "grip": 0.0, "reqs": ["above"], "clutter": [1, 3]},
}
STEPS = {"lift": 400, "next_to": 600, "turn": 500, "push": 500, "on": 700, "above": 300}


def q_yaw(deg):
    h = math.radians(deg) / 2
    return [math.cos(h), 0.0, 0.0, math.sin(h)]


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# ---------------------------------------------------------------- 物件池
POOL = {}
for d in sorted(os.listdir(OBJ)):
    if not d.startswith("bdq_"):
        continue
    m = json.load(open(f"{OBJ}/{d}/00000/metadata.json"))
    v = np.asarray(m["geometry"]["aligned_bbox"]["vertices"], dtype=float)
    POOL[d] = {"cat": d, "lo": v.min(axis=0), "hi": v.max(axis=0), "mass": m["physics"]["mass"], **m["bd"]}
assert POOL, "物件池是空的:先跑 make_pool.py"


def corners(o, x, y, yaw):
    lo, hi = o["lo"], o["hi"]
    Rz = rot_z(yaw)
    z = TABLE_TOP - lo[2] + 0.002
    return np.array([[x, y, z] + Rz @ np.array([cx, cy, cz]) for cx in (lo[0], hi[0]) for cy in (lo[1], hi[1]) for cz in (lo[2], hi[2])]), z


def footprint_r(o):
    return 0.5 * float(np.hypot(o["hi"][0] - o["lo"][0], o["hi"][1] - o["lo"][1]))


def min_w(o):
    return float(min(o["hi"][0] - o["lo"][0], o["hi"][1] - o["lo"][1]))


def height(o):
    return float(o["hi"][2] - o["lo"][2])


def clear(body, pts):
    for x0, x1, y0, y1, zmin in body["keepout"]:
        if any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and p[2] >= zmin for p in pts):
            return False
    # 每个角都在某一块出题区里(放宽 3 cm)
    return all(any(z[0][0] - 0.03 <= p[0] <= z[0][1] + 0.03 and z[1][0] - 0.03 <= p[1] <= z[1][1] + 0.03 for z in body["zones"]) for p in pts)


def place(rng, body, objs, pair=None):
    """objs:要摆的物件;pair = (i, j, 最小空隙, 中心最远):第 i、j 件开局离多远(空隙 = 中心距 − 两件的半对角线,
    比判"挨着"的 5 cm 大,开局就不会已经挨着)。返回 [(x, y, yaw, z)] 或 None"""
    zones = body["zones"]
    for _ in range(4000):
        out = []
        ok = True
        for k, o in enumerate(objs):
            (rx0, rx1), (ry0, ry1) = zones[int(rng.integers(len(zones)))]
            x, y, yaw = float(rng.uniform(rx0, rx1)), float(rng.uniform(ry0, ry1)), float(rng.uniform(0, 360))
            pts, z = corners(o, x, y, yaw)
            if not clear(body, pts):
                ok = False
                break
            for (x2, y2, _, _), o2 in zip(out, objs):
                if math.hypot(x - x2, y - y2) < footprint_r(o) + footprint_r(o2) + 0.03:
                    ok = False
                    break
            if not ok:
                break
            out.append((x, y, yaw, z))
        if not ok:
            continue
        if pair:
            i, j, lo, hi = pair
            dd = math.hypot(out[i][0] - out[j][0], out[i][1] - out[j][1])
            if not (dd - footprint_r(objs[i]) - footprint_r(objs[j]) >= lo and dd <= hi):
                continue
        return out
    return None


# ---------------------------------------------------------------- 出一道题
DIRS = {"to your left": [-1.0, 0.0], "to your right": [1.0, 0.0], "away from you": [0.0, 1.0]}


def make_one(rng, body_name):
    body = BODIES[body_name]
    req = str(rng.choice(body["reqs"]))
    names = list(POOL)
    graspable = [n for n in names if min_w(POOL[n]) <= body["grip"]]
    if req == "above":
        a = str(rng.choice(names))
        other = []
        sent = str(rng.choice(["Fly right above the {A}.", "Fly over to the {A} and hover right above it."]))
        check = ["bdq_above", {"label": "target", "xy_tol": 0.05, "clear": 0.05}]
    elif req == "lift":
        a = str(rng.choice(graspable))
        h = int(rng.choice(body["lift_cm"]))
        other = []
        sent = "Lift the {A} %d cm off the table." % h
        check = ["is_lift", {"label": "target", "z_threshold": h / 100.0}]
    elif req == "turn":
        a = str(rng.choice([n for n in graspable if POOL[n]["turnable"]]))
        other = []
        sent = str(rng.choice(["Turn the {A} around.", "Turn the {A} around so it faces the other way."]))
        check = ["bdq_turned", {"label": "target", "angle": math.radians(135.0)}]
    elif req == "push":
        a = str(rng.choice([n for n in names if height(POOL[n]) <= 0.12]))
        dname = str(rng.choice(list(DIRS)))
        dist = int(rng.choice(body["push_cm"]))
        other = []
        sent = "Push the {A} about %d cm %s, without lifting." % (dist, dname)
        check = ["bdq_pushed", {"label": "target", "dir": DIRS[dname], "dist": 0.8 * dist / 100.0, "max_lift": 0.02}]
    elif req == "next_to":
        a = str(rng.choice(graspable))
        b = str(rng.choice([n for n in names if n != a]))
        other = [b]
        sent = "Move the {A} next to the {B}."
        check = ["bdq_next_to", {"a": "target", "b": "other", "gap": 0.05, "min_move": 0.05}]
    elif req == "on":
        bs = [n for n in names if POOL[n]["flat_top"] or POOL[n]["container"]]
        b = str(rng.choice(bs))
        # A 窄的那一边放得进 B 窄的那一边(平顶:九成;容器:八成)
        fits = [n for n in graspable if n != b and min_w(POOL[n]) <= (0.8 if POOL[b]["container"] else 0.9) * min_w(POOL[b])]
        if not fits:
            return None
        a = str(rng.choice(fits))
        other = [b]
        mode = "in" if POOL[b]["container"] else "on"
        sent = "Put the {A} %s the {B}." % ("in" if mode == "in" else "on")
        check = ["bdq_on", {"a": "target", "b": "other", "mode": mode}]
    else:
        raise KeyError(req)
    rest = [n for n in names if n != a and n not in other]
    n_cl = int(rng.integers(body["clutter"][0], body["clutter"][1] + 1))
    clutter = [str(c) for c in rng.choice(rest, size=min(n_cl, len(rest)), replace=False)] if n_cl else []
    objs = [a] + other + clutter
    pair = None
    if other:
        lo, hi = body["pair_gap"]
        pair = (0, 1, lo, hi)
    spots = place(rng, body, [POOL[n] for n in objs], pair)
    if spots is None:
        return None
    labels = ["target"] + (["other"] if other else []) + ["clutter_%d" % k for k in range(len(clutter))]
    sentence = sent.format(A=POOL[a]["desc"], B=POOL[other[0]]["desc"] if other else "")
    return {"body": body_name, "cfg": body["cfg"], "cfg_name": body["cfg_name"], "requirement": req, "sentence": sentence,
            "check": check, "steps": STEPS[req], "objects": [{"label": l, "cat": n, "desc": POOL[n]["desc"], "pos": [x, y, z], "yaw": yaw}
                                                             for l, n, (x, y, yaw, z) in zip(labels, objs, spots)]}


def layout_for(q):
    base = json.load(open(f"{R}/Assets/Eval_Layout/RoboDojo/{BODIES[q['body']]['base']}"))
    lay = {k: base[k] for k in ("Room", "Table", "Ground", "Background")}
    lay["Geometry"] = {"camera_stand": base["Geometry"]["camera_stand"]}
    lay["Rigid"] = {}
    for o in q["objects"]:
        po = POOL[o["cat"]]
        rec = {"category": o["cat"], "category_idx": 0, "label": o["label"], "default_pos": o["pos"], "default_ori": q_yaw(o["yaw"]),
               "scale": [1.0, 1.0, 1.0], "physics": {"mass": po["mass"], "static_friction": 0.6, "dynamic_friction": 0.5, "type": "rigid"},
               "visual": {}, "relative_plane": "Table", "need_check_stable": True, "margin": 0.01, "check_mode": "bbox"}
        lay["Rigid"].setdefault(o["cat"], []).append(rec)
    lay["bd_question"] = {k: q[k] for k in ("qid", "body", "requirement", "sentence", "check", "steps")}
    return lay


TASK = '''# -*- coding: utf-8 -*-
# body-driver 随机题机(路 8)。由 harness/qexam/make_questions.py 写出,别手改。
# 一题一集:题(给脑的话、判据、步数)写在这一集的布局里("bd_question" 那一块),任务代码对所有题、所有身体都一样。
import os

from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager
from task.RoboDojo.bd import question, scene


class BdQuestionCommon:
    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = int(os.environ.get("BD_STEP_LIM", "600"))

    def _question(self, env_idx=0):
        lay = self.scene_manager.layout_manager.saved_layouts[env_idx] or {}
        return lay.get("bd_question", {})

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)
        scene.install_checks(self.reward_manager.func_parser)
        question.install_checks(self.reward_manager.func_parser)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()

    def run_reward(self):
        name, kw = self._question()["check"]
        self.reward_manager.check([(name, dict(kw))])

    def gen_instruction(self, env_idx):
        return [self._question(env_idx).get("sentence", "")]


class bd_question(BdQuestionCommon, TaskEnv):
    pass
'''

os.makedirs(f"{R}/task/RoboDojo/bd", exist_ok=True)
shutil.copy(f"{HERE}/rd/bd/question.py", mine(f"{R}/task/RoboDojo/bd/question.py"))
open(mine(f"{R}/task/RoboDojo/tasks/bd_question.py"), "w").write(TASK)
open(mine(f"{R}/task/RoboDojo/config/bd_question.yml"), "w").write(
    "# body-driver 随机题机(harness/qexam/make_questions.py 写出):题写在每一集的布局里,这里不列东西。\n{}\n")

os.makedirs(args.out, exist_ok=True)
used = set()
for f in os.listdir(args.out):
    if f.endswith(".json"):
        used |= {q["qid"] for q in json.load(open(os.path.join(args.out, f)))["questions"]}
qid = args.start if args.start is not None else (max(used) + 1 if used else 0)
rng = np.random.default_rng(args.seed)
bodies = args.bodies.split(",")
qs = []
while len(qs) < args.n:
    q = make_one(rng, str(rng.choice(bodies)))
    if q is None:
        continue
    q["qid"] = qid
    q["seed"] = SEED0 + qid
    d = f"{R}/Assets/Eval_Layout/RoboDojo/{q['cfg_name']}/{q['seed']}"
    os.makedirs(d, exist_ok=True)
    for old in os.listdir(d):
        assert old.startswith("bd_question"), f"{d} 里有别的东西:{old}"
    json.dump(layout_for(q), open(mine(f"{d}/bd_question_0.json"), "w"), indent=1)
    qs.append(q)
    qid += 1
batch = {"batch": args.batch, "seed": args.seed, "bodies": bodies, "questions": qs}
json.dump(batch, open(mine(os.path.join(args.out, f"{args.batch}.json")), "w"), indent=1, ensure_ascii=False)
for q in qs:
    print("%4d  %-8s %-8s 种子 %d  %s" % (q["qid"], q["body"], q["requirement"], q["seed"], q["sentence"]))
by = {}
for q in qs:
    by.setdefault((q["body"], q["requirement"]), 0)
    by[(q["body"], q["requirement"])] += 1
print("一共 %d 题:%s" % (len(qs), ", ".join("%s/%s %d" % (k[0], k[1], v) for k, v in sorted(by.items()))))
