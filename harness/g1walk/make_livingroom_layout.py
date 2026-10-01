# -*- coding: utf-8 -*-
"""第 40 条(路 8):大客厅的布局和"收拾完没有"的评分。家具是 make_livingroom.py 装的 bd_lr_*,东西是随机题机的物件池(bdq_*,Isaac 的 YCB)。

箱上跑(不起 Isaac):/venv/RoboDojo/bin/python make_livingroom_layout.py /root/RoboDojo --cfg_name <会走的人形那具身体的配置名> --n 30
写:
  Assets/Eval_Layout/RoboDojo/<配置名>/0/bd_livingroom_{0,1,2}.json(三张,东西乱放的地方不同)
  task/RoboDojo/bd/tidy.py(判据 bd_tidy:每件东西在不在它该去的地方)
每件东西该去哪(写在布局里那件东西的记录里,"bd_place": [家具的标签, 那件家具上放东西的地方]):
  吃的、碗碟 → 厨房台面;香蕉 → 餐桌上的果盘;玩具(泡沫砖、木块)→ 玩具箱;工具(电钻、夹子、剪刀)→ 工具箱;马克笔 → 书架。
给脑的话把这张对照表照实说出来(该去哪儿不让它猜)。
"""
import argparse
import json
import math
import os
import shutil

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("root", nargs="?", default="/root/RoboDojo")
ap.add_argument("--cfg_name", required=True)
ap.add_argument("--n", type=int, default=30)
ap.add_argument("--seed", type=int, default=40)
args = ap.parse_args()
R = args.root
HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = f"{R}/Assets/Object/RoboDojo"

# ---------------------------------------------------------------- 家具摆在哪(客厅 x −4 ~ 4、y −3 ~ 3;yaw 度)
FURN = {   # 标签: (类别, x, y, yaw)
    "sofa": ("bd_lr_sofa", 0.0, 2.45, 180.0),
    "coffee_table": ("bd_lr_coffee_table", 0.0, 1.3, 0.0),
    "tv_stand": ("bd_lr_tv_stand", 0.0, -2.75, 0.0),
    "bookshelf": ("bd_lr_bookshelf", 3.75, 0.5, 90.0),
    "counter": ("bd_lr_counter", -3.6, 0.5, -90.0),
    "dining_table": ("bd_lr_dining_table", -2.0, -1.6, 0.0),
    "toy_box": ("bd_lr_toy_box", 1.9, 2.3, 0.0),
    "tool_box": ("bd_lr_tool_box", 2.6, -2.4, 0.0),
    "room": ("bd_lr_room", 0.0, 0.0, 0.0),
}
WHERE = {   # 东西 → (家具, 放东西的地方)
    "bdq_soup_can": ("counter", "top"), "bdq_meat_can": ("counter", "top"), "bdq_sugar_box": ("counter", "top"),
    "bdq_cracker_box": ("counter", "top"), "bdq_pudding_box": ("counter", "top"), "bdq_gelatin_box": ("counter", "top"),
    "bdq_mustard": ("counter", "top"), "bdq_mug": ("counter", "top"), "bdq_bowl": ("counter", "top"),
    "bdq_banana": ("dining_table", "fruit_bowl"),
    "bdq_foam_brick": ("toy_box", "inside"), "bdq_wood_block": ("toy_box", "inside"),
    "bdq_drill": ("tool_box", "inside"), "bdq_clamp": ("tool_box", "inside"), "bdq_scissors": ("tool_box", "inside"),
    "bdq_marker": ("bookshelf", "shelf1"),
}
SAY = ("Tidy up the living room: put the food and the dishes on the kitchen counter, the banana in the fruit bowl on the dining table, "
       "the toys in the toy box, the tools in the toolbox, and the marker on the bookshelf.")
ROBOT_START = (0.0, -0.6)        # 会走的人形开局站在客厅中间偏南,朝 +y(沙发那边)


def rotz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def meta(sect, cat):
    return json.load(open(f"{OBJ}/{sect}/{cat}/00000/metadata.json"))


def bbox(m):
    v = np.asarray(m["geometry"]["aligned_bbox"]["vertices"], dtype=float)
    return v.min(axis=0), v.max(axis=0)


def footprint(lo, hi, x, y, yaw):
    c = np.array([[lo[0], lo[1], 0], [hi[0], lo[1], 0], [hi[0], hi[1], 0], [lo[0], hi[1], 0]])
    return (c @ rotz(yaw).T)[:, :2] + [x, y]


def overlap(P, Q):
    for poly in (P, Q):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            ax = np.array([-e[1], e[0]])
            if (P @ ax).max() < (Q @ ax).min() or (Q @ ax).max() < (P @ ax).min():
                return False
    return True


def q_yaw(deg):
    h = math.radians(deg) / 2
    return [math.cos(h), 0.0, 0.0, math.sin(h)]


# 东西开局乱放的地方:地上(避开家具和人形开局那一块)、沙发座、茶几面、电视柜面、餐桌面(果盘外)
FLOOR_Z = 0.05          # 地面 = RoboDojo 布局里 Ground 那一块的中心 + 半厚(和第 39 条一样按布局算;这里读同一份)
base = json.load(open(f"{R}/Assets/Eval_Layout/RoboDojo/drone/1/bootcal_0.json"))
G = base["Ground"]
FLOOR_Z = float(G["default_pos"][2]) + 0.5 * float(G["thickness"])
furn_foot = {}
for lab, (cat, x, y, yaw) in FURN.items():
    if lab == "room":
        continue
    lo, hi = bbox(meta("Geometry", cat))
    furn_foot[lab] = footprint(lo, hi, x, y, yaw)


def surface(lab, place):
    """家具 lab 上放东西的地方:世界里的中心(顶面)、半长半宽、转角、深"""
    cat, x, y, yaw = FURN[lab]
    pl = meta("Geometry", cat)["passive"]["functional"]["place"][place]
    c = np.asarray(pl["center"]) @ rotz(yaw).T + [x, y, FLOOR_Z]
    return c, np.asarray(pl["half"]), yaw, pl["depth"]


MESSY = [("floor", None)] * 6 + [("sofa", "seat"), ("coffee_table", "top"), ("tv_stand", "top_left"), ("dining_table", "top")]
rng = np.random.default_rng(args.seed)
pool = sorted(WHERE)
os.makedirs(f"{R}/task/RoboDojo/bd", exist_ok=True)
shutil.copy(f"{HERE}/rd/bd/tidy.py", f"{R}/task/RoboDojo/bd/tidy.py")
D = f"{R}/Assets/Eval_Layout/RoboDojo/{args.cfg_name}/0"
os.makedirs(D, exist_ok=True)
for k in range(3):
    lay = {key: json.loads(json.dumps(base[key])) for key in ("Ground", "Background")}
    lay["Table"] = dict(base["Table"], default_pos=[0.0, 30.0, base["Table"]["default_pos"][2]])   # RoboDojo 的桌子挪出客厅(它要有这一项)
    lay["Geometry"] = {}
    for lab, (cat, x, y, yaw) in FURN.items():
        lay["Geometry"].setdefault(cat, []).append({"category": cat, "category_idx": 0, "label": lab, "default_pos": [x, y, FLOOR_Z],
                                                    "default_ori": q_yaw(yaw), "scale": [1.0, 1.0, 1.0], "physics": {"type": "geometry"}, "visual": {}})
    taken = [footprint(np.array([-0.35, -0.35, 0]), np.array([0.35, 0.35, 0]), ROBOT_START[0], ROBOT_START[1], 0.0)]   # 人形站的地方
    taken += list(furn_foot.values())
    lay["Rigid"] = {}
    placed = []
    for i in range(args.n):
        cat = pool[i % len(pool)] if i < len(pool) else str(rng.choice(pool))
        m = meta("Rigid", cat)
        lo, hi = bbox(m)
        for _ in range(2000):
            where, place = MESSY[int(rng.integers(len(MESSY)))]
            yaw = float(rng.uniform(0, 360))
            if where == "floor":
                x, y = float(rng.uniform(-3.6, 3.6)), float(rng.uniform(-2.6, 2.6))
                z = FLOOR_Z - lo[2] + 0.003
                fp = footprint(lo, hi, x, y, yaw)
                if any(overlap(fp, t) for t in taken):
                    continue
            else:
                c, half, fyaw, _ = surface(where, place)
                u, v = float(rng.uniform(-half[0], half[0])), float(rng.uniform(-half[1], half[1]))
                x, y = (np.array([u, v, 0]) @ rotz(fyaw).T)[:2] + c[:2]
                z = c[2] - lo[2] + 0.003
                fp = footprint(lo, hi, x, y, yaw)
                if any(overlap(fp, t) for t in taken[len(furn_foot) + 1:]):    # 和别的东西不叠(家具的投影这回本来就在底下)
                    continue
                # 整个投影都在这一面上
                inside = ((fp - c[:2]) @ rotz(fyaw)[:2, :2]) / half
                if np.abs(inside).max() > 1.0:
                    continue
            taken.append(fp)
            break
        else:
            raise AssertionError(f"{cat}:2000 回都摆不下")
        rec = {"category": cat, "category_idx": 0, "label": f"item_{i:02d}", "default_pos": [float(x), float(y), float(z)],
               "default_ori": q_yaw(yaw), "scale": [1.0, 1.0, 1.0], "physics": {"mass": m["physics"]["mass"], "static_friction": 0.6,
                                                                               "dynamic_friction": 0.5, "type": "rigid"},
               "visual": {}, "relative_plane": "Ground", "need_check_stable": False, "margin": 0.01, "check_mode": "bbox",
               "bd_place": list(WHERE[cat]), "bd_start": where}
        if i == 0:
            rec["bd_tidy"] = {"sentence": SAY, "n": args.n}
        lay["Rigid"].setdefault(cat, []).append(rec)
        placed.append((cat, where))
    json.dump(lay, open(f"{D}/bd_livingroom_{k}.json", "w"), indent=1)
    by = {}
    for cat, w in placed:
        by[w] = by.get(w, 0) + 1
    print("布局 %s/bd_livingroom_%d.json:%d 件东西,开局在 %s" % (D, k, len(placed), by))
print("给脑的话:", SAY)
