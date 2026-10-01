# -*- coding: utf-8 -*-
"""body-driver 小场景的运行时(大并行 §2 第 38 条,路 8)。装到 RoboDojo 的 task/RoboDojo/bd/scene.py(新文件,不改 RoboDojo 原有的任何文件)。

两样东西:
1. 判据:RoboDojo 的 RewardManager 按名字 getattr(func_parser, 名字) 找判据,所以这里的判据在任务 _post_setup_scene 里
   绑到那个 Func_Parser 实例上(install_checks),Func_Parser 的源文件一个字不动。判据只读仿真真值(位姿、关节、粒子),
   几何参数全从资产自己的 metadata.json(passive.functional)里读,任务代码里不写尺寸。
2. 会自己走的东西(Walker):布局里某件 Rigid 带 "bd_walk" 字段,它就在桌面上按步随机走。
   RoboDojo 一个动作 = collect_interval 个物理子步,任务的 step() 每个子步被调一次 ⇒ 每个子步走 speed / collect_interval,
   一个动作正好走 speed(chase_mouse 每个子步走一整个 speed,一个动作走了 10 倍)。只在它还躺在原来那张面上时走:被抬起来就不走了。
"""
import math
import os
import types

import numpy as np
import torch


# ---------------------------------------------------------------- 小工具
def _np(x):
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float).reshape(-1)


def _rotm(q):
    w, x, y, z = [float(v) for v in q]
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _inst(fp, env_idx, label):
    return fp.layout_manager.get_instance_name(label=label, env_idx=env_idx)


def _pose(fp, env_idx, inst):
    pos, rot = fp.layout_manager.get_instance_pose(inst_name=inst, env_idx=env_idx)
    return _np(pos)[:3], _np(rot)[:4]


def _functional(fp, env_idx, inst, tag):
    meta = fp.layout_manager.get_instance_metadata(env_idx=env_idx, inst_name=inst) or {}
    return ((meta.get("passive") or {}).get("functional") or {}).get(tag)


def _points(pos, quat, frames):
    """把物体系里的一串 frame([x,y,z,qw,qx,qy,qz])变成世界(相对本 env 原点)里的点"""
    R = _rotm(quat)
    return [pos + R @ np.asarray(f[:3], dtype=float) for f in frames]


def _table_top(fp, env_idx):
    return float(fp.layout_manager.table_info[env_idx].get("height", 0.0))


# ---------------------------------------------------------------- 判据(每个都是 (self=Func_Parser, args) -> 0.0 / 1.0)
def bd_joint_moved(self, args):
    """某个关节离开局一开始的位置 ≥ amount(关节自己的单位:滑轨米、转轴弧度)。开局位置取 init_state 记下的那一刻。"""
    env_idx, label, joint, amount = args["env_idx"], args["label"], args["joint"], float(args["amount"])
    inst = _inst(self, env_idx, label)
    obj = self.layout_manager.get_scene_object(env_idx, inst)
    q = float(obj.get_joint_info(joint)["position"])
    q0 = float(self.pre_state[env_idx][inst][joint]["position"])
    return 1.0 if abs(q - q0) >= amount else 0.0


def bd_peg_in_hole(self, args):
    """销插进孔里 ≥ depth:销的底(销自己 metadata 的 bottom 点)低于孔口 depth,且水平上在孔口半宽以内。
    孔口(孔那块 metadata 的 hole:frame = 孔口中心,half_width = 孔口半宽)。孔四周是实心的,底低于孔口又在孔口以内 = 只能在孔里。"""
    env_idx = args["env_idx"]
    peg, block = _inst(self, env_idx, args["peg"]), _inst(self, env_idx, args["block"])
    pp, pq = _pose(self, env_idx, peg)
    bp, bq = _pose(self, env_idx, block)
    bottom = _points(pp, pq, _functional(self, env_idx, peg, "bottom")["frame"])[0]
    hole = _functional(self, env_idx, block, "hole")
    mouth = _points(bp, bq, hole["frame"])[0]
    axis = _rotm(bq)[:, 2]
    lateral = (bottom - mouth) - np.dot(bottom - mouth, axis) * axis
    below = -np.dot(bottom - mouth, axis)
    return 1.0 if (below >= float(args["depth"]) and np.linalg.norm(lateral) <= float(hole["half_width"])) else 0.0


def bd_ring_on_hook(self, args):
    """环挂在钩子的横杆上:横杆那条线(钩子 metadata 的 arm:杆根、杆尖两点)穿过环中间 ——
    ① 环心到杆那条线的距离 < 环的内半径(环心 metadata 的 center:frame、inner_radius);② 环心投到杆上落在杆根和杆尖之间;
    ③ 环的法向和杆差不到 45°(杆从环当中穿过去,不是环平躺在杆顶上);④ 环心高出桌面 > 环的外径(不在桌上)。"""
    env_idx = args["env_idx"]
    ring, hook = _inst(self, env_idx, args["ring"]), _inst(self, env_idx, args["hook"])
    rp, rq = _pose(self, env_idx, ring)
    hp, hq = _pose(self, env_idx, hook)
    cen = _functional(self, env_idx, ring, "center")
    c = _points(rp, rq, cen["frame"])[0]
    n = _rotm(rq) @ np.asarray(cen.get("axis", [0, 0, 1]), dtype=float)
    a0, a1 = _points(hp, hq, _functional(self, env_idx, hook, "arm")["frame"])[:2]
    d = a1 - a0
    L = float(np.linalg.norm(d))
    d = d / L
    t = float(np.dot(c - a0, d))
    dist = float(np.linalg.norm((c - a0) - t * d))
    ok = (dist < float(cen["inner_radius"]) and 0.0 <= t <= L and abs(float(np.dot(n, d))) > math.cos(math.radians(45.0))
          and c[2] - _table_top(self, env_idx) > 2.0 * float(cen["outer_radius"]))
    return 1.0 if ok else 0.0


def bd_trigger_while_held(self, args):
    """握住并扣扳机同时成立:枪身比开局抬高 ≥ lift(没人拿着它就会落回桌面)且扳机关节离开局 ≥ amount(弧度)。同一拍判。"""
    env_idx = args["env_idx"]
    gun = _inst(self, env_idx, args["gun"])
    pos, _ = _pose(self, env_idx, gun)
    z0 = float(self.pre_state[env_idx][gun]["pose"][2])
    obj = self.layout_manager.get_scene_object(env_idx, gun)
    q = float(obj.get_joint_info(args["joint"])["position"])
    q0 = float(self.pre_state[env_idx][gun][args["joint"]]["position"])
    return 1.0 if (pos[2] - z0 >= float(args["lift"]) and abs(q - q0) >= float(args["amount"])) else 0.0


def _cloth_points(fp, env_idx, inst):
    obj = fp.layout_manager.get_scene_object(env_idx, inst)
    pts, _, _, _ = obj.sample_mesh_vertices()
    pts = np.asarray(pts.detach().cpu().numpy() if hasattr(pts, "detach") else pts, dtype=float).reshape(-1, 3)
    org = _np(fp.layout_manager.scene_manager.env_origins[env_idx])[:3]
    return pts - org


def bd_cloth_lifted(self, args):
    """布被拿起来:布上最高的那个粒子高出桌面 ≥ height(布是粒子布,位姿不跟着粒子走,只能看粒子)"""
    env_idx = args["env_idx"]
    pts = _cloth_points(self, env_idx, _inst(self, env_idx, args["label"]))
    return 1.0 if float(pts[:, 2].max()) - _table_top(self, env_idx) >= float(args["height"]) else 0.0


def bd_not_touched(self, args):
    """不要碰(远 6):会出拳的东西出完 punches 拳、拳头一回都没碰到身体。碰没碰 = PhysX 的接触报告里拳头那一节和身体的哪一节
    有过接触(scene.Puncher 记在这件东西上,_bd_punch);拳还没出完判 0"""
    env_idx = args["env_idx"]
    obj = self.layout_manager.get_scene_object(env_idx, _inst(self, env_idx, args["label"]))
    st = getattr(obj, "_bd_punch", None) or {}
    return 1.0 if (st.get("done", 0) >= int(args["punches"]) and not st.get("touched")) else 0.0


CHECKS = {f.__name__: f for f in (bd_joint_moved, bd_peg_in_hole, bd_ring_on_hook, bd_trigger_while_held, bd_cloth_lifted, bd_not_touched)}


def install_checks(func_parser):
    """把上面的判据绑到这一个 Func_Parser 实例上(RewardManager.call_func_parser 按名字 getattr)"""
    for name, fn in CHECKS.items():
        setattr(func_parser, name, types.MethodType(fn, func_parser))


# ---------------------------------------------------------------- 会自己走的东西
def _nearest_robot_xy(env, env_idx, xy):
    """身体所有连杆里,桌面 / 地面上离 xy 最近的那一节的位置(本 env 原点下;仿真真值)"""
    rm = getattr(env, "robot_manager", None)
    if rm is None:
        return None
    org = _np(env.scene_manager.env_origins[env_idx])[:2]
    best, bd = None, 1e9
    for key in getattr(rm, "robot_key", []):
        d = key.data
        P = getattr(d, "body_pos_w", None)
        if P is None:
            P = getattr(d, "body_link_pos_w")
        P = P[env_idx].detach().cpu().numpy()[:, :2] - org
        k = int(np.argmin(np.linalg.norm(P - xy, axis=1)))
        dist = float(np.linalg.norm(P[k] - xy))
        if dist < bd:
            best, bd = P[k], dist
    return best


class Walker:
    """布局里带 "bd_walk" 的 Rigid:{"speed": 每个动作走几米, "turn_every": 每几个动作随机换一次方向,
    "region": [[x0, x1], [y0, y1]](碰边就反射), "free_height": 高出它开局那张面多少就算被拿起来了(不走), "seed": 随机数种子,
    "yaw0": 资产自己的前方和 +x 差多少度, "flee_radius": 可选 —— 身体哪一节(仿真真值:每一节连杆的位置)进了这么近,就朝正背着最近那一节的
    方向跑(会躲的老鼠,第 39 条);没进来照旧随机走}。每个物理子步调一次 tick。

    怎么走:它心里有一个"该在哪"的点,每个物理子步往走的方向挪 speed / collect_interval(一个动作正好 speed);
    每个子步按"该在哪 − 此刻在哪"给它一个水平速度,位置由物理自己积分(摩擦、碰撞都算:被挡住就过不去,不会穿过去)。
    差出去超过两个动作的路 = 被挡住或被推开了 ⇒ "该在哪"拉回它此刻在哪,从那儿接着走。朝向按限定的角速度转向走的方向。
    第一版是每个子步直接把位姿挪一小段:挪完物理的摩擦又把它往回拽,一个动作只走了 9.5 mm(bd_walker 离线核量的)。
    被拿离开局那张面 free_height 以上、或者翻倒了(竖轴偏过 30°),就不走。"""

    def __init__(self, params_by_label=None):
        """params_by_label = {标签: 同 bd_walk 的参数}:给了就按它走这件(不看布局里的 bd_walk;chase_mouse 用这个)"""
        self.state = {}
        self.params_by_label = params_by_label or {}
        self._log = os.environ.get("BD_WALK_LOG")

    def reset(self):
        self.state = {}

    def tick(self, env):
        lm = env.scene_manager.layout_manager
        om = getattr(env, "obs_manager", None)
        sub = int(round(float(getattr(om, "collect_interval", 1.0) or 1.0))) if om is not None else 1
        sub = max(sub, 1)
        dt = float(env.dt)
        ended = getattr(env, "end_flag", None)
        for env_idx in range(env.num_envs):
            if ended is not None and ended[env_idx]:
                continue
            for rec in lm.get_layout_records(env_idx, "Rigid"):
                w = self.params_by_label.get(rec.get("label")) or rec.get("bd_walk")
                if not w:
                    continue
                inst = rec["inst_name"]
                obj = lm.get_scene_object(env_idx, inst)
                if obj is None:
                    continue
                pos, rot = obj.get_local_pose()
                p, qn = _np(pos)[:3], _np(rot)[:4]
                key = (env_idx, inst)
                st = self.state.get(key)
                Rb = _rotm(qn)
                if st is None:
                    rng = np.random.default_rng(int(w.get("seed", 0)))
                    # "朝上"和"朝前"都按它开局躺着的样子定,不认资产自己的轴(chase_mouse 的鼠标资产不是 z 朝上:按 z 认它一开局就"翻倒了",一步不走)
                    up0 = Rb.T @ np.array([0.0, 0.0, 1.0])
                    fwd0 = np.array([1.0, 0.0, 0.0]) if np.linalg.norm((Rb @ np.array([1.0, 0.0, 0.0]))[:2]) > 0.5 else np.array([0.0, 1.0, 0.0])
                    st = {"rng": rng, "heading": float(rng.uniform(0.0, 2.0 * math.pi)), "z_rest": float(p[2]), "ticks": 0,
                          "target": p[:2].copy(), "up0": up0, "fwd0": fwd0}
                    self.state[key] = st
                if st["ticks"] > 0 and st["ticks"] % (int(w["turn_every"]) * sub) == 0:
                    st["heading"] = float(st["rng"].uniform(0.0, 2.0 * math.pi))
                if w.get("flee_radius"):
                    near = _nearest_robot_xy(env, env_idx, p[:2])
                    if near is not None and float(np.linalg.norm(p[:2] - near)) < float(w["flee_radius"]):
                        st["heading"] = math.atan2(p[1] - near[1], p[0] - near[0])   # 正背着最近的那一节跑
                upright = (Rb @ st["up0"])[2] > math.cos(math.radians(30.0))
                if p[2] > st["z_rest"] + float(w["free_height"]) or not upright:
                    st["target"] = p[:2].copy()          # 被拿起来 / 翻倒了:不走;放下、立起来以后从那儿接着走
                else:
                    step = float(w["speed"]) / sub
                    h = st["heading"]
                    (x0, x1), (y0, y1) = w["region"]
                    t = st["target"] + step * np.array([math.cos(h), math.sin(h)])
                    # 碰边反射:这一步会往区域外面走就把那个方向反过来(不夹到边上:开局就在区域外的(chase_mouse 的老鼠开局 y −0.30、
                    # 区域 y ≤ −0.32)夹一下"该在哪"就跳出去 2 cm,被当成被推开了、一步不走;反射以后它自己走回区域里)
                    if (t[0] < x0 and math.cos(h) < 0) or (t[0] > x1 and math.cos(h) > 0):
                        h = math.pi - h
                    if (t[1] < y0 and math.sin(h) < 0) or (t[1] > y1 and math.sin(h) > 0):
                        h = -h
                    t = st["target"] + step * np.array([math.cos(h), math.sin(h)])
                    st["heading"] = h
                    err = t - p[:2]
                    if np.linalg.norm(err) > 2.0 * float(w["speed"]):
                        t = p[:2].copy()                  # 被挡住 / 被推开了
                        err = t - p[:2]
                    st["target"] = t
                    v = _np(obj.get_linear_velocity())[:3]
                    obj.set_linear_velocity(torch.tensor([err[0] / dt, err[1] / dt, v[2]], dtype=torch.float32))
                    f = Rb @ st["fwd0"]
                    yaw_now = math.atan2(f[1], f[0])
                    dyaw = (h + math.radians(float(w.get("yaw0", 0.0))) - yaw_now + math.pi) % (2.0 * math.pi) - math.pi
                    wmax = math.pi   # 转向最快半圈一秒
                    obj.set_angular_velocity(torch.tensor([0.0, 0.0, max(-wmax, min(wmax, dyaw / dt))], dtype=torch.float32))
                if self._log and st["ticks"] % sub == 0:
                    with open(self._log, "a") as f:
                        f.write("%s %d %.5f %.5f %.5f\n" % (inst, st["ticks"] // sub, p[0], p[1], p[2]))
                st["ticks"] += 1


# ---------------------------------------------------------------- 会出拳的东西(远 6 不要碰)
def _robot_links(env, env_idx):
    """身体每一节连杆此刻的位置(本 env 原点下;仿真真值)和名字"""
    rm = getattr(env, "robot_manager", None)
    org = _np(env.scene_manager.env_origins[env_idx])[:3]
    P, names, seen = [], [], set()
    for key in (getattr(rm, "robot_key", []) if rm is not None else []):
        if id(key) in seen:
            continue
        seen.add(id(key))
        d = key.data
        X = getattr(d, "body_pos_w", None)
        if X is None:
            X = getattr(d, "body_link_pos_w")
        P.append(X[env_idx].detach().cpu().numpy() - org)
        names += list(getattr(key, "body_names", []))
    return (np.concatenate(P) if P else np.zeros((0, 3))), names


def _robot_prims(env, env_idx):
    """身体在舞台上的 prim 路径(接触报告里另一方的路径在它下面 = 碰到身体了)"""
    out = []
    for key in getattr(env.robot_manager, "robot_key", []):
        pp = str(key.cfg.prim_path)
        p = pp.replace("env_.*", f"env_{env_idx}").replace(".*", str(env_idx))
        if p not in out:
            out.append(p)
    return out


class Puncher:
    """布局里带 "bd_punch" 的关节体(会出拳的东西,make_scenes.py 的 bd_puncher):每个物理子步调一次 tick。
    参数 {"start", "every", "count"(第几个动作开始第一拳、每几个动作一拳、一共几拳), "aim_s"(对准用几秒), "speed"(伸出去多快,m/s),
    "overshoot"(伸过目标多远), "hold_s"(伸到头停几秒), "back_speed"(收回多快), "pivot"(转轴在资产系哪儿), "fist"(歇着时拳头中心在转轴前多远),
    "stroke"(臂最多伸多远)}。
    每一拳:对准那一刻身体离转轴最近的那一节(仿真真值:每一节连杆的位置)—— 转台转到朝着它、臂俯仰到朝着它;对准完再伸,
    伸到"转轴到那一节的距离 − 拳头歇着的位置 + overshoot"为止(那一节一直不动就一定打到);停一下,收回。对准以后不再跟:拳头是直着打出去的。
    碰没碰到身体:拳头那一节带 PhysX 的接触报告(资产里就带着 PhysxContactReportAPI),报告里另一方在身体的 prim 下面就记下来。
    结果记在这件东西上(obj._bd_punch:done 出完几拳、touched 第一回碰到的那一拳 / 那一节 / 什么时候、aims 每一拳对准的是哪一节),
    判据 bd_not_touched 读它。"""

    def __init__(self):
        self.state = {}
        self._sub = None
        self._watch = {}
        self._log = os.environ.get("BD_PUNCH_LOG")

    def reset(self):
        self.state = {}
        self._watch = {}

    def _subscribe(self):
        """每回新场子(复位重新生出这件东西)都调:接触处理打开、重新订阅接触报告。
        Isaac Lab 的 SimulationContext 开局就把 omni.physx 的接触处理关了(/physics/disableContactProcessing = True,
        isaaclab/sim/simulation_context.py),只有建 ContactSensor 时才打开(contact_sensor.py 里同一行设 False)。
        不打开,拳头碰到哪儿接触报告都是空的(第一版离线核:拳头被手指挡在 0.243 m、报告 0 回)⇒ 照 ContactSensor 的做法打开。
        第二版只在头一回订阅:第一张布局收得到,换布局(RoboDojo 删了重生)以后一条都收不到(拳头被手指挡住、拐开 14°)⇒ 每回重订"""
        import carb
        cs = carb.settings.get_settings()
        was = cs.get("/physics/disableContactProcessing")
        cs.set_bool("/physics/disableContactProcessing", False)
        from omni.physx import get_physx_simulation_interface
        self._sub = None
        self._sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact)
        self.headers_seen = getattr(self, "headers_seen", 0)
        self.subscriptions = getattr(self, "subscriptions", 0) + 1
        return bool(was)

    def _on_contact(self, headers, data):
        self.headers_seen = getattr(self, "headers_seen", 0) + len(headers)
        if not self._watch:
            return
        from pxr import PhysicsSchemaTools
        for h in headers:
            a0, a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            for fist, (st, prims) in self._watch.items():
                other = a1 if a0 == fist else (a0 if a1 == fist else None)
                if other is None:
                    continue
                st["contacts"] = st.get("contacts", 0) + 1
                st["last_other"] = other
                if any(other == p or other.startswith(p + "/") for p in prims) and not st.get("touched"):
                    st["touched"] = {"punch": st.get("k"), "link": other, "t_s": round(st.get("t", 0.0), 3)}

    def tick(self, env):
        lm = env.scene_manager.layout_manager
        om = getattr(env, "obs_manager", None)
        sub = max(int(round(float(getattr(om, "collect_interval", 1.0) or 1.0))) if om is not None else 1, 1)
        dt = float(env.dt)
        for env_idx in range(env.num_envs):
            for rec in lm.get_layout_records(env_idx, "Articulation"):
                w = rec.get("bd_punch")
                if not w:
                    continue
                inst = rec["inst_name"]
                obj = lm.get_scene_object(env_idx, inst)
                if obj is None:
                    continue
                key = (env_idx, inst)
                st = self.state.get(key)
                if st is None:
                    from pxr import UsdPhysics
                    fist = None
                    root = getattr(obj, "prim", None)
                    for prim in (root.GetChildren() if root is not None else []):
                        if prim.GetName() == "arm" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            fist = str(prim.GetPath())
                    names = list(obj.dof_names)
                    st = {"ticks": 0, "k": 0, "t": 0.0, "done": 0, "touched": None, "aims": [], "phase": "idle",
                          "ids": [names.index(n) for n in ("yaw_joint", "pitch_joint", "punch_joint")], "fist_path": fist,
                          "robot_prims": _robot_prims(env, env_idx), "tgt": [0.0, 0.0, 0.0], "ext": 0.0}
                    self.state[key] = st
                    obj._bd_punch = st
                    if fist is not None:
                        self._watch[fist] = (st, st["robot_prims"])
                        st["contact_processing_was_disabled"] = self._subscribe()
                        st["headers_at_start"] = self.headers_seen
                st["t"] = st["ticks"] * dt
                t = st["t"]
                k = st["k"]
                if k < int(w["count"]):
                    t0 = (float(w["start"]) + k * float(w["every"])) * sub * dt     # 这一拳几时开始(秒)
                    if st["phase"] == "idle" and t >= t0:
                        pos, rot = obj.get_local_pose()
                        p, Rq = _np(pos)[:3], _rotm(_np(rot)[:4])
                        pivot = p + Rq @ np.asarray(w.get("pivot", [0.0, 0.0, 0.15]), dtype=float)
                        P, names = _robot_links(env, env_idx)
                        j = int(np.argmin(np.linalg.norm(P - pivot, axis=1)))
                        # aim_offset:只给离线核用(对准偏开一点,核"打不到就判 1"),题里不给
                        da = Rq.T @ (P[j] + np.asarray(w.get("aim_offset", [0.0, 0.0, 0.0]), dtype=float) - pivot)
                        yaw_ = math.atan2(da[1], da[0])
                        elev = math.atan2(da[2], math.hypot(da[0], da[1]))
                        dist = float(np.linalg.norm(da))
                        st["tgt"] = [max(-math.radians(90), min(math.radians(90), yaw_)), max(-math.radians(60), min(math.radians(60), -elev)),
                                     max(0.0, min(float(w.get("stroke", 0.5)), dist - float(w.get("fist", 0.10)) + float(w["overshoot"])))]
                        st["aims"].append({"punch": k, "link": names[j] if j < len(names) else int(j), "dist_m": round(dist, 3),
                                           "yaw_deg": round(math.degrees(yaw_), 1), "elev_deg": round(math.degrees(elev), 1), "t_s": round(t, 2),
                                           "ext_m": round(st["tgt"][2], 3)})
                        st["phase"], st["t_phase"] = "aim", t
                    elif st["phase"] == "aim" and t >= st["t_phase"] + float(w["aim_s"]):
                        st["phase"], st["t_phase"] = "extend", t
                    elif st["phase"] == "extend":
                        st["ext"] = min(st["tgt"][2], st["ext"] + float(w["speed"]) * dt)
                        if st["ext"] >= st["tgt"][2] - 1e-9:
                            st["phase"], st["t_phase"] = "hold", t
                    elif st["phase"] == "hold" and "at_hold" not in st["aims"][-1] and t >= st["t_phase"] + 0.5 * float(w["hold_s"]):
                        q = _np(obj.get_joint_positions())
                        st["aims"][-1]["at_hold"] = {"yaw_deg": round(math.degrees(q[st["ids"][0]]), 1), "pitch_deg": round(math.degrees(q[st["ids"][1]]), 1),
                                                     "ext_m": round(float(q[st["ids"][2]]), 3)}
                    elif st["phase"] == "hold" and t >= st["t_phase"] + float(w["hold_s"]):
                        st["phase"] = "retract"
                    elif st["phase"] == "retract":
                        st["ext"] = max(0.0, st["ext"] - float(w["back_speed"]) * dt)
                        if st["ext"] <= 0.0:
                            st["phase"], st["k"], st["done"] = "idle", k + 1, st["done"] + 1
                cur = obj.get_joint_positions()
                v = _np(cur).reshape(1, -1).copy()
                aimed = st["phase"] != "idle" or st["k"] > 0
                v[0, st["ids"][0]] = st["tgt"][0] if aimed else 0.0
                v[0, st["ids"][1]] = st["tgt"][1] if aimed else 0.0
                v[0, st["ids"][2]] = st["ext"]
                tv = torch.as_tensor(v, dtype=torch.float32)
                if hasattr(cur, "device"):
                    tv = tv.to(cur.device)
                obj._articulation_view.set_joint_position_targets(tv)
                if self._log and st["ticks"] % sub == 0:
                    with open(self._log, "a") as f:
                        f.write("%s %d %s %.4f %s\n" % (inst, st["ticks"] // sub, st["phase"], st["ext"], bool(st["touched"])))
                st["ticks"] += 1
