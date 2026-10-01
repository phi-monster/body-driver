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
import sys

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("root", nargs="?", default="/root/RoboDojo")
ap.add_argument("--batch", required=True)
ap.add_argument("--n", type=int, default=30)
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--start", type=int, default=None, help="第一题的题号(默认接着已有的题号往后排)")
ap.add_argument("--bodies", default="x5,humanoid,drone")
ap.add_argument("--out", default="/root/p8/qexam/batches")
ap.add_argument("--scenes", action="store_true",
                help="这一批出路 8 的小场景(harness/scenes 的 11 个任务,每个随机挑一张布局,一题一个种子目录),不出 YCB 题")
ap.add_argument("--feasibility", type=int, default=0, help="只看每一对(身体, 要求)摆不摆得下:每对试这么多回,报成了几回、一回多久,不写文件")
ap.add_argument("--pool_check", action="store_true",
                help="只写一张核物件池稳不稳的布局(种子 29999,x5):池里每件按它的摆法隔开放一排排,离线核时逐件量站不站得住")
args = ap.parse_args()
R = args.root
HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = f"{R}/Assets/Object/RoboDojo/Rigid"
SEED0 = 20000
WRITTEN = []


def mine(path):
    rel = os.path.relpath(path, R) if path.startswith(R) else path
    ok = (not path.startswith(R)) or ("bd_question" in rel or rel.startswith("task/RoboDojo/bd/")
                                      or (rel.startswith("Assets/Eval_Layout/RoboDojo/") and os.path.basename(rel).startswith("bd_")))
    assert ok and ".." not in rel, f"不许写这个路径: {rel}"
    WRITTEN.append(rel)
    return path


# ---------------------------------------------------------------- 身体:配置、桌上能出题的那一块、歇着的手占的地方(离线核量的连杆位置)
TABLE_TOP = 0.765
BODIES = {
    # x5:腕 (±0.30, −0.352, 0.922),手指朝前平伸到 y ≈ −0.21。
    #     grip = 夹爪张到头的指缝:ARX.usd 的 joint7 / joint8 两指各走 0 – 0.044 m,0 时两指的碰撞面贴着(缝 0.8 mm)⇒ 8.8 cm
    "x5": {"cfg": "arx_x5", "cfg_name": "arx_x5", "base": "arx_x5/0/bootcal_0.json", "zones": [[[-0.30, 0.30], [-0.18, 0.10]]],
           "keepout": [(-0.41, -0.19, -0.40, -0.17, 0.85), (0.19, 0.41, -0.40, -0.17, 0.85)], "grip": 0.088,
           "reqs": ["lift", "next_to", "turn", "push", "on"], "lift_cm": [5, 10, 15], "push_cm": [10], "clutter": [1, 2]},
    # G1:腕 (±0.178, −0.448, 0.864),手指朝前到 y ≈ −0.238、高 0.834–0.894(最低的小指 0.834);能伸 8 cm 左右(rig_upright_low.py 的离线正解)
    #     ⇒ 题只出在两只手各自够得着的那一小块(两手中间够不着)。
    #     grip = 手张到头时拇指和食指之间最近的空:g1_29dof_inspire_hand.usd 默认姿势(各关节 0:手指伸直、拇指没转过来)
    #     拇指末节和食指中节两块的最近距离 6.6 cm(中心距 11.3 cm;握拳式的包着拿能拿更粗的,这里只按捏得住算)
    "humanoid": {"cfg": "g1_rgb", "cfg_name": "g1", "base": "g1/1/bootcal_0.json", "zones": [[[-0.26, -0.10], [-0.30, -0.15]], [[0.10, 0.26], [-0.30, -0.15]]],
                 "keepout": [(-0.26, -0.10, -0.52, -0.20, 0.82), (0.10, 0.26, -0.52, -0.20, 0.82)], "grip": 0.066,
                 "reqs": ["lift", "next_to", "turn", "push", "on"], "lift_cm": [5, 10], "push_cm": [5], "clutter": [0, 1]},
    # 无人机:龙门架的底座在 (0, −0.2, 1.4),机身在桌子上方飞;只出"飞到它正上方"
    "drone": {"cfg": "drone_rgb", "cfg_name": "drone", "base": "drone/1/bootcal_0.json", "zones": [[[-0.35, 0.35], [-0.25, 0.20]]],
              "keepout": [], "grip": 0.0, "reqs": ["above"], "clutter": [1, 3]},
}
# 一集多少步(驱动开机用的步也算在里面,和 RoboDojo 官方任务一样:x5 带着存好的身体文件开机 68 步):照 RoboDojo 自己最像的那个官方任务给的步数,
#   lift / push / above = general_pickup 的 200("Pick up the <target> by 10 cm.":一个动作、10 cm 上下);
#   next_to / on / turn = deposit_coin 的 300("Pick up the coin … and insert it … into the coin bank.":拿起来、放到指定的地方)。
#   官方唯一的推的任务 push_T 是 600,但它要把 T 形块推到一个位姿上(准),这里的推只要往一个方向推 10 cm,按 general_pickup 算
STEPS = {"lift": 200, "push": 200, "above": 200, "next_to": 300, "on": 300, "turn": 300}
# "挨着" = 桌面上的投影最近处 ≤ 5 cm,A 至少挪过 5 cm:和 RoboDojo 自己判据的默认距离一样(is_lift 的 z_threshold、is_moved 的 dis_threshold 都是 0.05)
NEXT_GAP = 0.05


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
    POOL[d] = {"cat": d, "lo": v.min(axis=0), "hi": v.max(axis=0), "mass": m["physics"]["mass"], "meta": m, **m["geometry"]["bd"]}
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
    """objs:要摆的物件;pair = (i, j, 最小空隙):第 i、j 件开局离得比"挨着"远(空隙 = 中心距 − 两件的半对角线 > 判"挨着"的那个距离,
    外接圆都隔得开,真形状只会隔得更开),而且在同一块出题区里(一块 = 一只手够得着的地方:人形两只手中间够不着,
    A 得在 B 所在的那只手的地方里挪过去)。返回 [(x, y, yaw, z)] 或 None"""
    zones = body["zones"]
    for _ in range(4000):
        out = []
        ok = True
        zone_of = {}
        for k, o in enumerate(objs):
            zi = int(rng.integers(len(zones)))
            if pair and k == pair[1]:
                zi = zone_of[pair[0]]
            zone_of[k] = zi
            (rx0, rx1), (ry0, ry1) = zones[zi]
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
            i, j, lo = pair
            dd = math.hypot(out[i][0] - out[j][0], out[i][1] - out[j][1])
            if not dd - footprint_r(objs[i]) - footprint_r(objs[j]) > lo:
                continue
        return out
    return None


# ---------------------------------------------------------------- 出一道题
DIRS = {"to your left": [-1.0, 0.0], "to your right": [1.0, 0.0], "away from you": [0.0, 1.0]}


def make_one(rng, body_name, req):
    body = BODIES[body_name]
    names = list(POOL)
    graspable = [n for n in names if min_w(POOL[n]) < body["grip"]]   # 平放时窄的那一边比手张到头的空窄
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
        check = ["bdq_next_to", {"a": "target", "b": "other", "gap": NEXT_GAP, "min_move": NEXT_GAP}]
    elif req == "on":
        bs = [n for n in names if POOL[n]["flat_top"] or POOL[n]["container"]]
        b = str(rng.choice(bs))
        # 放得上 / 放得进:按两件从网格量的尺寸(question.fits;第一版用"窄边的八成 / 九成",糖盒能出成"放进杯子",其实塞不进)
        ok = [n for n in graspable if n != b and Q.fits(POOL[n]["meta"], POOL[b]["meta"])]
        if not ok:
            return None
        a = str(rng.choice(ok))
        other = [b]
        mode = Q.fits(POOL[a]["meta"], POOL[b]["meta"])
        sent = "Put the {A} %s the {B}." % ("in" if mode == "in" else "on")
        check = ["bdq_on", {"a": "target", "b": "other", "mode": mode}]
    else:
        raise KeyError(req)
    rest = [n for n in names if n != a and n not in other]
    n_cl = int(rng.integers(body["clutter"][0], body["clutter"][1] + 1))
    clutter = [str(c) for c in rng.choice(rest, size=min(n_cl, len(rest)), replace=False)] if n_cl else []
    objs = [a] + other + clutter
    pair = (0, 1, NEXT_GAP) if other else None
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
        if o["label"] == "target":   # 题写在目标那件东西的记录里(最上面一层的每一项 RoboDojo 都当成一类东西去生)
            rec["bd_question"] = {k: q[k] for k in ("qid", "body", "requirement", "sentence", "check", "steps")}
        lay["Rigid"].setdefault(o["cat"], []).append(rec)
    return lay


TASK = '''# -*- coding: utf-8 -*-
# body-driver 随机题机(路 8)。由 harness/qexam/make_questions.py 写出,别手改。
# 一题一集:题(给脑的话、判据、步数)写在这一集布局里目标那件东西的记录里("bd_question" 那一项),任务代码对所有题、所有身体都一样。
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
        return question.find_question(self.scene_manager.layout_manager.saved_layouts[env_idx])

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
sys.path.insert(0, R)
from task.RoboDojo.bd import question as Q   # 放得上 / 放得进的那一条(Q.fits)和判据、离线核用的是同一份
open(mine(f"{R}/task/RoboDojo/tasks/bd_question.py"), "w").write(TASK)
open(mine(f"{R}/task/RoboDojo/config/bd_question.yml"), "w").write(
    "# body-driver 随机题机(harness/qexam/make_questions.py 写出):题写在每一集的布局里,这里不列东西。\n{}\n")

if args.pool_check:
    # 两张布局(种子 29998、29999)各 8 件,格子隔 30 cm(最大那件半对角线 13.4 cm,两件加起来再空 3 cm),都在两只手前面 30 cm 以外。
    # 第一版固定的格子没管大小,剪刀和木块开局就叠在一起,木块被挤翻 78°。
    # 离线核这两张(check_scenes.py 的 pool_scenario):逐件量落稳后挪了多远、歪了多少;每个平顶 / 容器拿同一张里最小的、放得上 / 放得进的那件
    # 真放上去 / 放进去(物理走到停),判据判一遍。分组让每个平顶 / 容器同一张里都有放得上 / 放得进的东西(杯子的口只过得去马克笔)
    groups = [["bdq_mug", "bdq_marker", "bdq_bowl", "bdq_cracker_box", "bdq_banana", "bdq_drill", "bdq_clamp", "bdq_scissors"],
              ["bdq_gelatin_box", "bdq_pudding_box", "bdq_wood_block", "bdq_foam_brick", "bdq_soup_can", "bdq_sugar_box", "bdq_mustard", "bdq_meat_can"]]
    assert sorted(sum(groups, [])) == sorted(POOL), "两张核物件池的布局要把池里每件都放上,一件一次"
    spots = [(x, y) for y in (0.30, 0.02) for x in (-0.45, -0.15, 0.15, 0.45)]
    for li, (seed, group) in enumerate(zip((29998, 29999), groups)):
        objs = []
        for k, n in enumerate(group):
            x, y = spots[k]
            _, z = corners(POOL[n], x, y, 0.0)
            objs.append({"label": "target" if k == 0 else "pool_%s" % n[4:], "cat": n, "desc": POOL[n]["desc"], "pos": [x, y, z], "yaw": 0.0})
        q = {"qid": -1 - li, "body": "x5", "cfg": "arx_x5", "cfg_name": "arx_x5", "requirement": "pool", "sentence": "(物件池稳不稳、放得上放得进)",
             "check": ["is_lift", {"label": "target", "z_threshold": 0.1}], "steps": 50, "objects": objs}
        d = f"{R}/Assets/Eval_Layout/RoboDojo/arx_x5/{seed}"
        os.makedirs(d, exist_ok=True)
        json.dump(layout_for(q), open(mine(f"{d}/bd_question_0.json"), "w"), indent=1)
        print("物件池稳不稳的布局:%s(%s)" % (d, ", ".join(group)))
    for b in sorted(POOL):
        if POOL[b]["flat_top"] or POOL[b]["container"]:
            ok = [a for a in sorted(POOL) if a != b and Q.fits(POOL[a]["meta"], POOL[b]["meta"])]
            print("  %-16s %s %s:%s" % (b, "口 %.3f" % POOL[b]["opening_d"] if POOL[b]["container"] else "顶 %.3f × %.3f" % tuple(sorted(POOL[b]["meta"]["geometry"]["aligned_bbox"]["extents"][:2])),
                                        "放得进" if POOL[b]["container"] else "放得上", ", ".join(a[4:] for a in ok) or "没有"))
    raise SystemExit(0)

if args.feasibility:
    import time as _t
    r_ = np.random.default_rng(args.seed)
    for b_ in args.bodies.split(","):
        for rq in BODIES[b_]["reqs"]:
            t_ = _t.time()
            ok_ = sum(make_one(r_, b_, rq) is not None for _ in range(args.feasibility))
            print("%-9s %-8s 试 %d 回成 %d 回,一回 %.2f s" % (b_, rq, args.feasibility, ok_, (_t.time() - t_) / args.feasibility), flush=True)
    raise SystemExit(0)

os.makedirs(args.out, exist_ok=True)
used = set()
for f in os.listdir(args.out):
    if f.endswith(".json"):
        used |= {q["qid"] for q in json.load(open(os.path.join(args.out, f)))["questions"]}
qid = args.start if args.start is not None else (max(used) + 1 if used else 0)
rng = np.random.default_rng(args.seed)

if args.scenes:
    # 路 8 的小场景进随机题(大并行 §6"再加路 8 的小场景"):每个场景从它种子 0 的 3 张布局里随机挑一张,原样拷进这一题自己的种子目录
    # (RoboDojo 一个种子目录里有几张布局就连着跑几集;一题一集就得一题一个目录,目录里只有这一张)。判据就是那个任务自己的。
    # 步数照 RoboDojo 官方最像的任务:一件事(抬起来 / 拉开 / 掀开 / 推开)= general_pickup 的 200;
    # 两件事(拿起来再插进去 / 挂上去 / 扣扳机;拧半圈要松手再拧)= deposit_coin 的 300
    SCENE_STEPS = {"bd_drawer": 200, "bd_lidbox": 200, "bd_hinge": 200, "bd_glass": 200, "bd_white": 200, "bd_walker": 200, "bd_cloth": 200,
                   "bd_peg": 300, "bd_hook": 300, "bd_trigger": 300, "bd_knob": 300}
    import hashlib
    import re
    qs = []
    for task in sorted(SCENE_STEPS):
        k = int(rng.integers(3))
        src = f"{R}/Assets/Eval_Layout/RoboDojo/arx_x5/0/{task}_{k}.json"
        instr = re.search(r"return \[(['\"])(.*)\1\]", open(f"{R}/task/RoboDojo/tasks/{task}.py").read()).group(2)
        seed = SEED0 + qid
        d = f"{R}/Assets/Eval_Layout/RoboDojo/arx_x5/{seed}"
        os.makedirs(d, exist_ok=True)
        for old in os.listdir(d):
            assert old.startswith("bd_"), f"{d} 里有别的东西:{old}"
        dst = mine(f"{d}/{task}_0.json")
        shutil.copy(src, dst)
        same = hashlib.md5(open(src, "rb").read()).hexdigest() == hashlib.md5(open(dst, "rb").read()).hexdigest()
        lay = json.load(open(src))
        objs = [{"label": r.get("label"), "cat": c, "desc": c, "pos": r["default_pos"]} for sect in ("Rigid", "Articulation", "Geometry", "Garment")
                for c, lst in (lay.get(sect) or {}).items() for r in lst if c != "camera_stand"]
        qs.append({"qid": qid, "seed": seed, "task": task, "body": "x5", "cfg": "arx_x5", "cfg_name": "arx_x5", "requirement": "scene",
                   "sentence": instr, "check": ["%s 自己的判据" % task, {"layout": f"arx_x5/0/{task}_{k}.json", "copy_md5_same": same}],
                   "steps": SCENE_STEPS[task], "objects": objs})
        qid += 1
    batch = {"batch": args.batch, "seed": args.seed, "bodies": ["x5"], "questions": qs}
    json.dump(batch, open(mine(os.path.join(args.out, f"{args.batch}.json")), "w"), indent=1, ensure_ascii=False)
    for q in qs:
        print("%4d  %-11s 种子 %d  布局 %s(拷得一样 %s)  %d 步  %s" % (q["qid"], q["task"], q["seed"], q["check"][1]["layout"],
                                                                q["check"][1]["copy_md5_same"], q["steps"], q["sentence"]))
    raise SystemExit(0)
bodies = args.bodies.split(",")
# 每题先抽(身体, 要求)这一对 —— 每一对机会一样(无人机只有一种要求,就只占一份);再在这一对里抽东西、摆布局,摆不下换东西重抽。
# 第一版身体、要求一起抽,摆不下就连身体带要求整个重抽:人形的小块地方摆不下两件,"挪到旁边""放上去"总被换掉,30 题里人形 8 道是"转过来"
PAIRS = []
r_chk = np.random.default_rng(args.seed + 1)   # 另一路随机数:看摆不摆得下,不搅出题那一路
for b_ in bodies:
    for rq in BODIES[b_]["reqs"]:
        if any(make_one(r_chk, b_, rq) is not None for _ in range(200)):
            PAIRS.append((b_, rq))
        else:
            print("不出 %s / %s:抽 200 回东西都摆不下(这具身体够得着的地方放不下这种题)" % (b_, rq))
qs = []
while len(qs) < args.n:
    body_name, req = PAIRS[int(rng.integers(len(PAIRS)))]
    for _ in range(500):
        q = make_one(rng, body_name, req)
        if q is not None:
            break
    assert q is not None, f"{body_name} / {req}:抽 500 回东西都摆不下"
    q["qid"] = qid
    q["seed"] = SEED0 + qid
    d = f"{R}/Assets/Eval_Layout/RoboDojo/{q['cfg_name']}/{q['seed']}"
    os.makedirs(d, exist_ok=True)
    for old in os.listdir(d):
        assert old.startswith("bd_question"), f"{d} 里有别的东西:{old}"
    for b in BODIES.values():   # 同一个种子上一回出给了别的身体:那份旧题删掉(只删 bd_question_0.json,目录里没别的才删目录)
        od = f"{R}/Assets/Eval_Layout/RoboDojo/{b['cfg_name']}/{q['seed']}"
        if b["cfg_name"] != q["cfg_name"] and os.path.exists(f"{od}/bd_question_0.json"):
            os.remove(mine(f"{od}/bd_question_0.json"))
            if not os.listdir(od):
                os.rmdir(od)
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
