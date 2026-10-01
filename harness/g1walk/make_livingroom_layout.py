# -*- coding: utf-8 -*-
"""第 40 条(路 8):大客厅的布局和"收拾完没有"的评分;同一间客厅里远 5 的两样(上台阶、从地上捡东西)的布局。家具是 make_livingroom.py 装的 bd_lr_*,东西是随机题机的物件池(bdq_*,Isaac 的 YCB)。

箱上跑(不起 Isaac):/venv/RoboDojo/bin/python make_livingroom_layout.py /root/RoboDojo --cfg_name <会走的人形那具身体的配置名> --n 30
写:
  Assets/Eval_Layout/RoboDojo/<配置名>/0/bd_livingroom_{0,1,2}.json(三张,东西乱放的地方不同)
  Assets/Eval_Layout/RoboDojo/<配置名>/0/bd_stairs_{0,1,2}.json、bd_floorpick_{0,1,2}.json(远 5)
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
# 会走的人形开局站在餐桌南边、面朝餐桌(朝 +y),骨盆离桌子近的那条边 START_GAP:开机量身体要一张手够得着、眼看得见的桌面
# (P8XH 开局站在客厅当中,前面 1.9 m 才是茶几:"定不了世界(第一只手的眼没三角出桌面)");量完就在这间屋里接着干,地、桌子都是同一张。
# 和 RoboDojo 那具固定在桌边的 G1 差不多的站法(那具骨盆在桌子近边后面 0.12 m,桌面比骨盆高 6.5 cm;这里桌面离地 0.76、骨盆 0.75)。
# 站在哪儿按餐桌资产自己的包围盒算(下面 robot_start()),写进布局(第一件东西的 bd_tidy 里),install.py 从布局里读了写进身体配置
START_AT, START_GAP = "dining_table", 0.15


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
# 地面高:RoboDojo 的 Ground 是一块 cube,中心摆在 default_pos 的 z − 半厚(scene_manager/objects/ground.py)⇒ 顶面就在 default_pos 的 z;
# 这里把它摆到 0.05 m,比默认房间 Simple_Room 自己的地高一点(第 39 条离线核实测:东西落在 0.0475 上),地面就是这一块的顶,高度说得清。
# (第一版按"中心 + 半厚"算出 0.05,算法是错的,碰巧和房间的地差 2.5 mm;大客厅没放房间,东西其实落在 Ground 的顶 0 上、边上的掉出了 7 m 的地)
FLOOR_Z = 0.05      # 和第 39 条(harness/wheelarm/install.py)一样;客厅自己的地(bd_lr_room 那块 8 × 6 m 的底板)顶面也在这儿
base = json.load(open(f"{R}/Assets/Eval_Layout/RoboDojo/drone/1/bootcal_0.json"))
furn_foot = {}
for lab, (cat, x, y, yaw) in FURN.items():
    if lab == "room":
        continue
    lo, hi = bbox(meta("Geometry", cat))
    furn_foot[lab] = footprint(lo, hi, x, y, yaw)


def robot_start():
    """人形开局站哪儿(x, y):START_AT 那件家具朝 −y 那条边的正中,再往 −y 退 START_GAP(人形朝 +y)"""
    cat, x, y, yaw = FURN[START_AT]
    assert abs(yaw) < 1e-9, "这里只按没转过的家具算"
    lo, hi = bbox(meta("Geometry", cat))
    return (float(x + (lo[0] + hi[0]) / 2), float(y + lo[1] - START_GAP))


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
def room_layout():
    """客厅本身:地(RoboDojo 的 Ground 摆到 FLOOR_Z)、背景、家具;RoboDojo 的桌子挪出客厅(布局里要有这一项)"""
    lay = {key: json.loads(json.dumps(base[key])) for key in ("Ground", "Background")}
    lay["Ground"]["default_pos"] = [0.0, 0.0, FLOOR_Z]
    lay["Table"] = dict(base["Table"], default_pos=[0.0, 30.0, base["Table"]["default_pos"][2]])
    lay["Geometry"] = {}
    for lab, (cat, x, y, yaw) in FURN.items():
        lay["Geometry"].setdefault(cat, []).append({"category": cat, "category_idx": 0, "label": lab, "default_pos": [x, y, FLOOR_Z],
                                                    "default_ori": q_yaw(yaw), "scale": [1.0, 1.0, 1.0], "physics": {"type": "geometry"}, "visual": {}})
    return lay


ROBOT_START = robot_start()
ROBOT_FOOT = footprint(np.array([-0.35, -0.35, 0]), np.array([0.35, 0.35, 0]), ROBOT_START[0], ROBOT_START[1], 0.0)   # 人形站的地方
for k in range(3):
    lay = room_layout()
    taken = [ROBOT_FOOT] + list(furn_foot.values())
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
            rec["bd_tidy"] = {"sentence": SAY, "n": args.n, "robot_start": [round(v, 4) for v in ROBOT_START]}
        lay["Rigid"].setdefault(cat, []).append(rec)
        placed.append((cat, where))
    json.dump(lay, open(f"{D}/bd_livingroom_{k}.json", "w"), indent=1)
    by = {}
    for cat, w in placed:
        by[w] = by.get(w, 0) + 1
    print("布局 %s/bd_livingroom_%d.json:%d 件东西,开局在 %s" % (D, k, len(placed), by))
print("人形开局站在:", [round(v, 4) for v in robot_start()], "(%s 前 %.2f m)" % (START_AT, START_GAP))
print("给脑的话:", SAY)


# ---------------------------------------------------------------- 远 5:上台阶(bd_stairs)、蹲下捡地上的东西(bd_floorpick)
# 同一间客厅、同一个开局(人形站在餐桌跟前);每样三张布局,台阶 / 那件东西摆的地方不同
def floor_ok(fp):
    """在客厅的地上(离墙 0.4 m 以内不放)、不压家具、不压人形站的地方"""
    if np.abs(fp[:, 0]).max() > 3.6 or np.abs(fp[:, 1]).max() > 2.6:
        return False
    return not any(overlap(fp, t) for t in [ROBOT_FOOT] + list(furn_foot.values()))


def item_rec(cat, label, x, y, yaw, **extra):
    m = meta("Rigid", cat)
    lo, hi = bbox(m)
    rec = {"category": cat, "category_idx": 0, "label": label, "default_pos": [float(x), float(y), float(FLOOR_Z - lo[2] + 0.003)],
           "default_ori": q_yaw(yaw), "scale": [1.0, 1.0, 1.0], "physics": {"mass": m["physics"]["mass"], "static_friction": 0.6,
                                                                           "dynamic_friction": 0.5, "type": "rigid"},
           "visual": {}, "relative_plane": "Ground", "need_check_stable": False, "margin": 0.01, "check_mode": "bbox"}
    rec.update(extra)
    return rec


st_lo, st_hi = bbox(meta("Geometry", "bd_lr_stairs"))
for k in range(3):
    lay = room_layout()
    for _ in range(2000):
        # 台阶朝 +x 往上(人形从西边、餐桌那儿走过来),摆在东边那块空地上;台阶前面(西边)再空出 1 m 走过来、站上第一级
        x, y = float(rng.uniform(0.3, 1.2)), float(rng.uniform(-1.3, 0.2))
        fp = footprint(st_lo, st_hi, x, y, 0.0)
        ahead = footprint(np.array([-1.0, st_lo[1], 0.0]), np.array([0.0, st_hi[1], 0.0]), x, y, 0.0)
        if floor_ok(fp) and floor_ok(ahead):
            break
    else:
        raise AssertionError("台阶摆不下")
    lay["Geometry"]["bd_lr_stairs"] = [{"category": "bd_lr_stairs", "category_idx": 0, "label": "stairs", "default_pos": [x, y, FLOOR_Z],
                                        "default_ori": q_yaw(0.0), "scale": [1.0, 1.0, 1.0], "physics": {"type": "geometry"}, "visual": {}}]
    lay["Rigid"] = {}
    json.dump(lay, open(f"{D}/bd_stairs_{k}.json", "w"), indent=1)
    print("布局 %s/bd_stairs_%d.json:台阶脚下在 (%.2f, %.2f),平台顶 %.2f m" % (D, k, x, y, FLOOR_Z + st_hi[2]))

# 捡得起来的:平放时窄的那一边 < 7 cm(手张开能包住的;人形手的张口这里没量,按随机题机给 G1 的 6.6 cm 取整)
PICK = []
for cat in sorted(WHERE):
    lo, hi = bbox(meta("Rigid", cat))
    if min(hi[0] - lo[0], hi[1] - lo[1]) < 0.07:
        PICK.append(cat)
for k in range(3):
    lay = room_layout()
    cat = str(rng.choice(PICK))
    lo, hi = bbox(meta("Rigid", cat))
    for _ in range(2000):
        r, ang, yaw = float(rng.uniform(1.0, 2.0)), float(rng.uniform(0.0, 2 * math.pi)), float(rng.uniform(0, 360))
        x, y = ROBOT_START[0] + r * math.cos(ang), ROBOT_START[1] + r * math.sin(ang)
        if floor_ok(footprint(lo, hi, x, y, yaw)):
            break
    else:
        raise AssertionError(f"{cat} 摆不下")
    desc = meta("Rigid", cat)["geometry"].get("bd", {}).get("desc") or cat.replace("bdq_", "").replace("_", " ")
    say = "Pick up the %s from the floor and lift it 10 cm." % desc
    lay["Rigid"] = {cat: [item_rec(cat, "target", x, y, yaw, bd_say=say)]}
    json.dump(lay, open(f"{D}/bd_floorpick_{k}.json", "w"), indent=1)
    print("布局 %s/bd_floorpick_%d.json:%s 在地上 (%.2f, %.2f),离人形 %.2f m —— %s" % (D, k, cat, x, y, r, say))
