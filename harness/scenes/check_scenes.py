# -*- coding: utf-8 -*-
"""离线核小场景(路 8):不接驱动、不接脑,在 Isaac 里按 RoboDojo 自己的 main.py 那一套把场景装起来,逐张布局核:
  ① 装得起来、RoboDojo 自己的"布局稳不稳"那一关过了(不稳它会抛 UnStableError);
  ② 场景里每件东西都在、落稳后离布局给的位置不远;存每只相机的第一帧图(肉眼看东西在画面里);
  ③ 评分对:把仿真真值直接摆成"做成了"/"没做成"的几种样子(关节位置、位姿、布的粒子),看这个任务注册的判据给 1 / 0;
     能靠物理站住的(抽屉拉开、盖子翻过头、销插进孔、环挂上钩)再走几十个物理步,看判据还是 1(= 这个状态物理上真站得住);
     最后走一遍 RoboDojo 自己的 reward_manager.step → get_reward,看"做成了"那个样子真判成 1;
  ④ 会自己走的东西:原地不动地走 N 个动作(动作 = 身体此刻的关节读数,手臂不动),量它每个动作走多远、换不换方向、出不出界。
这里摆状态只动场景里的东西,从不替身体动手(不是脚本脑)。

用法(箱上,拿着排队锁,见 qcheck.sh):
  cd /root/RoboDojo && python -u check_scenes.py --task bd_drawer --out /root/p8/chk --enable_cameras --headless --kit_args "..."
"""
import argparse
import json
import math
import os
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", required=True)
parser.add_argument("--cfg", default="arx_x5")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--layouts", default="0,1,2")
parser.add_argument("--out", required=True)
parser.add_argument("--walk_steps", type=int, default=60)
parser.add_argument("--walk_label", default="", help="量任意一件东西每个动作走多远(比如 chase_mouse 的 target)")
parser.add_argument("--tag", default="", help="结果放 out/<tag>(默认 = 任务名)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np
import torch
from omegaconf import OmegaConf

from env.global_configs import BENCHMARK, ENV_CONFIG_PATH, ROOT_DIR
import src.eval_client.eval_env as ee
from utils.load_file import load_yaml
from utils.pipeline_utils import process_config, process_randomization


class _NoPolicy:
    """代替 ws 策略客户端:离线核不接驱动"""

    def __init__(self, **kw):
        pass

    def call(self, func_name=None, obs=None, **kw):
        return None

    def close(self):
        pass


ee.WsModelClient = _NoPolicy
T0 = time.time()
OUT = os.path.join(args.out, args.tag or args.task)
os.makedirs(OUT, exist_ok=True)
REPORT = {"task": args.task, "cfg": args.cfg, "layouts": []}


def log(*a):
    print("[chk %6.1fs]" % (time.time() - T0), *a, flush=True)


# ---------------------------------------------------------------- 和 main.py 一样拼配置
def make_env():
    task_name = args.task
    eval_cfg = load_yaml(os.path.join(ENV_CONFIG_PATH, args.cfg + ".yml"))
    eval_cfg.update(task_name=task_name, num_envs=1, device_id=0, eval_batch=False, policy_name="l3_link",
                    additional_info="p8check", seed=args.seed, physx_monitor_enabled=False)
    deploy_cfg = {"policy_name": "l3_link", "port": 1, "host": "127.0.0.1", "protocol": "ws", "policy_server_url": "ws://127.0.0.1:1",
                  "evaluation_id": "p8check", "trial_id": f"{task_name}-p8check", "action_case_id": f"{task_name}_case", "repeat_index": None}
    tr = __import__(f"task.{BENCHMARK}.task_registry", fromlist=["task_config_path"])
    env_cfg = OmegaConf.create({
        "sim": load_yaml(os.path.join(ENV_CONFIG_PATH, "sim", eval_cfg["config"]["sim"] + ".yml")),
        "scene": load_yaml(os.path.join(ENV_CONFIG_PATH, "scene", eval_cfg["config"]["scene"] + ".yml")),
        "camera": load_yaml(os.path.join(ENV_CONFIG_PATH, "camera", eval_cfg["config"]["camera"] + ".yml")),
        "robot": load_yaml(os.path.join(ENV_CONFIG_PATH, "robot", eval_cfg["config"]["robot"] + ".yml")),
        "task_env": load_yaml(tr.task_config_path(os.path.join(ROOT_DIR, "task", BENCHMARK, "config"), task_name)),
        "eval_cfg": eval_cfg, "deploy_cfg": deploy_cfg})
    OmegaConf.update(env_cfg, "sim.scene.num_envs", 1, force_add=True)
    env_cfg = process_randomization(env_cfg)
    env_cfg, _ = process_config(env_cfg, task_name=task_name)
    OmegaConf.update(env_cfg, "camera.default_frequency", eval_cfg["observation"].get("collect_freq", 0), force_add=True)
    env_cfg.sim.seed = [0]
    return ee.create_eval_env(env_cfg, app)


# ---------------------------------------------------------------- 小工具
def _np(x):
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float).reshape(-1)


def _like(v, ref):
    if isinstance(ref, torch.Tensor):
        return torch.as_tensor(np.asarray(v), dtype=ref.dtype, device=ref.device)
    return np.asarray(v, dtype=np.asarray(ref).dtype)


def rotm(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def q_from_z_to(d):
    z = np.array([0.0, 0.0, 1.0])
    d = np.asarray(d, dtype=float) / np.linalg.norm(d)
    ax = np.cross(z, d)
    s = np.linalg.norm(ax)
    ang = math.atan2(s, float(np.dot(z, d)))
    if s < 1e-9:
        return [1.0, 0.0, 0.0, 0.0] if ang < 1.0 else [0.0, 1.0, 0.0, 0.0]
    ax = ax / s
    return [math.cos(ang / 2)] + list(ax * math.sin(ang / 2))


class Scene:
    def __init__(self, env):
        self.env = env
        self.lm = env.scene_manager.layout_manager
        self.rm = env.reward_manager

    def inst(self, label):
        return self.lm.get_instance_name(env_idx=0, label=label)

    def obj(self, label):
        return self.lm.get_scene_object(0, self.inst(label))

    def pose(self, label):
        p, q = self.lm.get_instance_pose(env_idx=0, inst_name=self.inst(label))
        return _np(p)[:3], _np(q)[:4]

    def meta(self, label):
        return self.lm.get_instance_metadata(env_idx=0, inst_name=self.inst(label))

    def set_pose(self, label, pos, quat):
        o = self.obj(label)
        o.set_local_pose(np.asarray(pos, dtype=float), np.asarray(quat, dtype=float))
        if hasattr(o, "set_linear_velocity"):
            o.set_linear_velocity(torch.zeros(3))
            o.set_angular_velocity(torch.zeros(3))

    def set_joint(self, label, joint, value):
        o = self.obj(label)
        q = o.get_joint_positions()
        v = _np(q).copy()
        v[o.dof_names.index(joint)] = value
        o.set_joint_positions(_like(v, q))
        qd = o.get_joint_velocities()
        o.set_joint_velocities(_like(np.zeros_like(_np(qd)), qd))

    def release(self, label, joint, target=0.0):
        """放手:把这个关节驱动的目标放回 target。Isaac 的 set_joint_positions 会顺手把驱动目标也设成新位置
        (isaacsim.core.prims 的 articulation.py:set_dof_positions 之后紧跟 set_dof_position_targets),
        带弹簧的关节(扳机)不放回去,弹簧就会把它"钉"在摆到的位置"""
        o = self.obj(label)
        cur = o.get_joint_positions()
        v = _np(cur).reshape(1, -1).copy()
        v[0, o.dof_names.index(joint)] = target
        o._articulation_view.set_joint_position_targets(_like(v, cur))

    def joint(self, label, joint):
        return float(self.obj(label).get_joint_info(joint)["position"])

    def cloth_points(self, label):
        o = self.obj(label)
        pts, _, _, _ = o.sample_mesh_vertices()
        return np.asarray(pts.detach().cpu().numpy() if hasattr(pts, "detach") else pts, dtype=float).reshape(-1, 3)

    def cloth_shift(self, label, mask, dz):
        o = self.obj(label)
        if getattr(o, "_device", "cpu") != "cpu" and getattr(o, "_cloth_prim_view", None) is not None:
            w = o._cloth_prim_view.get_world_positions()
            v = w.detach().clone()
            m = torch.as_tensor(mask, device=v.device)
            v[0, m, 2] += dz
            o._cloth_prim_view.set_world_positions(v)
            return "view"
        from pxr import Vt
        attr = o._prim.GetAttribute("points")
        p = np.asarray(attr.Get(), dtype=np.float32)
        p[mask, 2] += dz
        attr.Set(Vt.Vec3fArray.FromNumpy(p))
        return "usd"

    def graded(self):
        """这个任务注册的第一关判据,逐条用 RoboDojo 的 check_once 判(不弹出,好反复判)"""
        stage = self.rm.check_list[0][0] if self.rm.check_list[0] else []
        return all(self.rm.check_once(c, 0) for c in stage)

    def steps(self, n):
        for _ in range(n):
            self.env.sim_step(render=False)

    def hold_action(self):
        """动作 = 身体此刻的关节读数(手臂不动);夹爪取上一拍发过的那个值(RoboDojo 的 take_action 每只夹爪都要有)"""
        obs = self.env.get_obs()
        src = dict(obs.get("action") or {})
        src.update(obs.get("state") or {})
        keys = [k for k in src if k.endswith("arm_joint_state") or k.endswith("ee_joint_state")]
        act = {k: np.asarray(src[k], dtype=float).reshape(-1) for k in keys}
        for k in [k for k in act if k.endswith("arm_joint_state")]:
            g = k.replace("arm_joint_state", "ee_joint_state")
            act.setdefault(g, np.array([1.0]))
        return act


def test(rep, name, expect, got):
    ok = bool(got) == bool(expect)
    rep["tests"].append({"state": name, "expect": int(bool(expect)), "got": int(bool(got)), "ok": ok})
    log("   %s 判据 %d(该 %d)%s" % (name, int(bool(got)), int(bool(expect)), "" if ok else "  🔴"))


# ---------------------------------------------------------------- 每个任务的真值摆法
def scenario(S, rep):
    t = args.task
    if t in ("bd_drawer", "bd_lidbox", "bd_hinge", "bd_knob"):
        label, joint, cases, hold = {
            "bd_drawer": ("cabinet", "drawer_joint", [("关着", 0.0, 0), ("拉开 5 cm", 0.05, 0), ("拉开 15 cm", 0.15, 1)], ("拉开 15 cm 后走 60 步", 0.15, 1)),
            "bd_lidbox": ("box", "lid_joint", [("盖着", 0.0, 0), ("开 45°", math.radians(45), 0), ("开 100°", math.radians(100), 1)], ("开 100° 后走 120 步", math.radians(100), 1)),
            "bd_hinge": ("board", "board_joint", [("关着", 0.0, 0), ("开 34°", 0.6, 0), ("往外开 69°", 1.2, 1), ("往里开 69°", -1.2, 1)], ("开 69° 后走 60 步", 1.2, 1)),
            "bd_knob": ("knob", "knob_joint", [("没转", 0.0, 0), ("转 90°", math.pi / 2, 0), ("正转 190°", math.radians(190), 1), ("反转 190°", -math.radians(190), 1)], ("转 190° 后走 60 步", math.radians(190), 1)),
        }[t]
        for name, v, e in cases:
            S.set_joint(label, joint, v)
            test(rep, name, e, S.graded())
        name, v, e = hold
        S.set_joint(label, joint, v)
        traj = []
        for _ in range(12 if t == "bd_lidbox" else 6):
            S.steps(10)
            traj.append(round(S.joint(label, joint), 4))
        rep["joint_after_hold"] = S.joint(label, joint)
        rep["joint_hold_every_10_steps"] = traj
        log("   %s:关节 %.4f(每 10 步 %s)" % (name, rep["joint_after_hold"], traj))
        test(rep, name, e, S.graded())
        S.set_joint(label, joint, v)
        return True
    if t == "bd_trigger":
        p0, q0 = S.pose("gun")
        o = S.obj("gun")
        props = o.dof_properties
        rep["dof_properties"] = {n: {k: float(props[k][i]) for k in props.dtype.names if np.issubdtype(props[k].dtype, np.number)}
                                 for i, n in enumerate(o.dof_names)}
        try:
            kps, kds = o._articulation_view.get_gains()
            rep["gains"] = {"kps": _np(kps).tolist(), "kds": _np(kds).tolist()}
        except Exception as e:
            rep["gains"] = str(e)
        log("   扳机关节(仿真里实际装上的):%s · 增益 %s" % (rep["dof_properties"], rep["gains"]))
        S.set_joint("gun", "trigger_joint", math.radians(22))
        test(rep, "躺在桌上扣扳机 22°", 0, S.graded())
        S.release("gun", "trigger_joint")
        tr = []
        for _ in range(10):
            S.steps(2)
            tr.append(round(math.degrees(S.joint("gun", "trigger_joint")), 2))
        rep["trigger_release_on_table_deg_every_2_steps"] = tr
        log("   躺在桌上扣到 22° 松开:每 2 步 %s°" % tr)
        test(rep, "松开 20 步后扳机回到 1° 以内(回位弹簧)", 1, abs(tr[-1]) < 1.0)
        S.set_joint("gun", "trigger_joint", 0.0)
        S.set_pose("gun", p0 + [0, 0, 0.10], q0)
        test(rep, "抬起 10 cm 不扣", 0, S.graded())
        S.set_joint("gun", "trigger_joint", math.radians(22))
        test(rep, "抬起 10 cm 扣 22°", 1, S.graded())
        S.release("gun", "trigger_joint")
        S.steps(20)
        rep["trigger_after_release"] = S.joint("gun", "trigger_joint")
        log("   抬着松开扳机走 20 步:扳机 %.2f°(回位弹簧)" % math.degrees(rep["trigger_after_release"]))
        test(rep, "抬着松开 20 步后不再算扣着", 0, S.graded())
        S.set_pose("gun", p0 + [0, 0, 0.10], q0)
        S.set_joint("gun", "trigger_joint", math.radians(22))
        return True
    if t == "bd_peg":
        bp, bq = S.pose("block")
        hole = S.meta("block")["passive"]["functional"]["hole"]
        mouth = bp + rotm(bq) @ np.asarray(hole["frame"][0][:3])
        test(rep, "销在桌上", 0, S.graded())
        S.set_pose("peg", mouth + [0, 0, 0.01], bq)
        test(rep, "销悬在孔口上方 1 cm", 0, S.graded())
        S.set_pose("peg", mouth + rotm(bq) @ np.array([0.035, 0.0, 0.0]), bq)
        test(rep, "销立在孔旁边的块顶上", 0, S.graded())
        S.set_pose("peg", mouth + [0, 0, -0.03], bq)
        test(rep, "销插进孔 3 cm", 1, S.graded())
        S.steps(60)
        pp, _ = S.pose("peg")
        rep["peg_bottom_below_mouth_after_60"] = float(mouth[2] - pp[2])
        log("   插进去走 60 步:销底在孔口下 %.4f m(孔深 %.3f)" % (rep["peg_bottom_below_mouth_after_60"], hole["depth"]))
        test(rep, "插进去走 60 步", 1, S.graded())
        return True
    if t == "bd_hook":
        hp, hq = S.pose("hook")
        arm = S.meta("hook")["passive"]["functional"]["arm"]["frame"]
        a0, a1 = [hp + rotm(hq) @ np.asarray(f[:3]) for f in arm]
        cen = S.meta("ring")["passive"]["functional"]["center"]
        c_local = np.asarray(cen["frame"][0][:3])
        test(rep, "环在桌上", 0, S.graded())
        d = (a1 - a0) / np.linalg.norm(a1 - a0)
        q_hang = q_from_z_to(d)
        at = a0 + 0.6 * (a1 - a0)
        c = at - np.array([0, 0, cen["inner_radius"] - 0.006 - 0.001])   # 杆(粗 12 mm)贴着环里圈的顶
        S.set_pose("ring", c - rotm(q_hang) @ c_local, q_hang)
        test(rep, "环套在杆上", 1, S.graded())
        S.steps(90)
        rp, rq = S.pose("ring")
        rep["ring_center_after_90"] = [float(v) for v in rp + rotm(rq) @ c_local]
        log("   套上走 90 步:环心 %s" % np.round(rep["ring_center_after_90"], 4).tolist())
        test(rep, "套上走 90 步", 1, S.graded())
        S.set_pose("ring", at + np.array([0, 0, 0.006 + 0.0005]) - c_local, [1, 0, 0, 0])
        test(rep, "环平躺在杆顶上", 0, S.graded())
        S.set_pose("ring", c - rotm(q_hang) @ c_local, q_hang)
        return True
    if t in ("bd_glass", "bd_white", "bd_walker"):
        label = {"bd_glass": "glass", "bd_white": "target", "bd_walker": "target"}[t]
        p0, q0 = S.pose(label)
        test(rep, "在桌上", 0, S.graded())
        S.set_pose(label, p0 + [0, 0, 0.12], q0)
        test(rep, "抬起 12 cm", 1, S.graded())
        return True
    if t == "bd_cloth":
        P = S.cloth_points("cloth")
        top = S.lm.table_info[0]["height"]
        rep["cloth_flat_max_above_table"] = float(P[:, 2].max() - top)
        rep["cloth_flat_min_above_table"] = float(P[:, 2].min() - top)
        log("   平铺:粒子高出桌面 %.4f – %.4f m,%d 个粒子" % (rep["cloth_flat_min_above_table"], rep["cloth_flat_max_above_table"], len(P)))
        test(rep, "平铺在桌上", 0, S.graded())
        c = P[np.argmin(P[:, 0] + P[:, 1])]
        mask = np.linalg.norm(P[:, :2] - c[:2], axis=1) < 0.05
        rep["cloth_set_path"] = S.cloth_shift("cloth", mask, 0.12)
        test(rep, "一角拎高 12 cm", 1, S.graded())
        return True
    return False


def walk(S, rep, label=None):
    """原地不动走 N 个动作,量会自己走的东西每个动作走多远(label 不给 = 布局里带 bd_walk 的那一件)"""
    recs = [r for r in S.lm.get_layout_records(0, "Rigid") if (r.get("label") == label if label else r.get("bd_walk"))]
    rec = recs[0]
    label = rec["label"]
    region = (rec.get("bd_walk") or {}).get("region", [[-1e9, 1e9], [-1e9, 1e9]])
    xs = []
    for k in range(args.walk_steps):
        p, _ = S.pose(label)
        xs.append(p.copy())
        S.env.take_action(S.hold_action())
    p, _ = S.pose(label)
    xs.append(p.copy())
    X = np.array(xs)
    d = np.linalg.norm(np.diff(X[:, :2], axis=0), axis=1)
    hd = np.degrees(np.arctan2(np.diff(X[:, 1]), np.diff(X[:, 0])))
    (x0, x1), (y0, y1) = region
    rep["walk"] = {"steps": int(len(d)), "per_step_median_m": float(np.median(d)), "per_step_min_m": float(d.min()), "per_step_max_m": float(d.max()),
                   "z_range_m": [float(X[:, 2].min()), float(X[:, 2].max())],
                   "inside_region": bool((X[:, 0] >= x0 - 1e-6).all() and (X[:, 0] <= x1 + 1e-6).all() and (X[:, 1] >= y0 - 1e-6).all() and (X[:, 1] <= y1 + 1e-6).all()),
                   "heading_deg_every_5": [float(round(v, 1)) for v in hd[::5]], "xy_first": X[0, :2].tolist(), "xy_last": X[-1, :2].tolist()}
    log("   走了 %d 个动作:每个动作中位 %.4f m(%.4f – %.4f),高 %.4f – %.4f,出界 %s,方向每 5 步 %s" % (
        len(d), np.median(d), d.min(), d.max(), X[:, 2].min(), X[:, 2].max(), not rep["walk"]["inside_region"], rep["walk"]["heading_deg_every_5"]))


def save_images(env, rep, tag):
    """取帧前先多渲几次:渲染出来的图比仿真状态晚一帧(和驱动那边量到的"画面晚读数一拍"是同一件事),
    只渲一次拿到的是上一次渲染的样子(bd_drawer 第一版:关节读数 0.15、存下的图里抽屉还关着)"""
    from PIL import Image
    for _ in range(4):
        env.render()
    obs = env.get_obs()
    rep.setdefault("images", [])
    for cam, v in obs["vision"].items():
        if isinstance(v, dict) and "color" in v:
            path = os.path.join(OUT, "%s_%s.png" % (tag, cam))
            Image.fromarray(np.asarray(v["color"]).astype(np.uint8)).save(path)
            rep["images"].append(path)


# ---------------------------------------------------------------- 主循环(每张布局:关掉 → 重装,和 main.py 一样)
env = make_env()
for lid in [int(s) for s in args.layouts.split(",") if s != ""]:
    rep = {"layout": lid, "tests": []}
    REPORT["layouts"].append(rep)
    t1 = time.time()
    try:
        env.reset(seed=[lid])
    except Exception as e:
        rep["loaded"] = False
        rep["error"] = "%s: %s" % (type(e).__name__, e)
        log("布局 %d 装不起来:%s" % (lid, rep["error"]))
        try:
            env.close()
        except Exception:
            pass
        continue
    rep["loaded"] = True
    rep["reset_s"] = round(time.time() - t1, 1)
    env.run_reward()
    S = Scene(env)
    log("布局 %d 装好了(%.0f s),判据:%s" % (lid, rep["reset_s"], env.reward_manager.check_list[0]))
    # ② 每件东西在不在、落稳后离布局给的位置多远
    rep["objects"] = []
    saved = S.lm.saved_layouts[0]
    for sect in ("Rigid", "Articulation", "Geometry", "Garment"):
        for r in S.lm.get_layout_records(0, sect):
            inst = r["inst_name"]
            o = S.lm.get_scene_object(0, inst)
            item = {"type": sect, "label": r.get("label"), "inst": inst, "present": o is not None}
            if o is not None and sect != "Garment":
                p, _ = S.lm.get_instance_pose(env_idx=0, inst_name=inst)
                item["drift_m"] = float(np.linalg.norm(_np(p)[:3] - np.asarray(r["default_pos"], dtype=float)))
            rep["objects"].append(item)
            log("   %s %s(%s)在=%s 漂 %s" % (sect, r.get("label"), inst, item["present"], ("%.4f m" % item["drift_m"]) if "drift_m" in item else "-"))
    save_images(env, rep, "L%d" % lid)
    st = env.get_obs()["state"]
    rep["ee_rest"] = {k: [round(float(v), 4) for v in np.asarray(st[k]).reshape(-1)[:3]] for k in st if k.endswith("ee_pose")}
    log("   身体歇着时手在哪(ee 位置):%s" % rep["ee_rest"])
    if lid == 0:   # 身体每一节连杆开局在哪(本 env 原点下):摆布局时离手多远按它定
        org = _np(env.scene_manager.env_origins[0])[:3]
        links = {}
        for i, key in enumerate(env.robot_manager.robot_key):
            d = key.data
            P = getattr(d, "body_pos_w", None)
            if P is None:
                P = getattr(d, "body_link_pos_w")
            P = P[0].detach().cpu().numpy()
            for n, p in zip(key.body_names, P):
                links["%d/%s" % (i, n)] = [round(float(v), 4) for v in (p - org)]
        rep["robot_links_rest"] = links
        log("   身体每一节开局在哪:%s" % links)
    if args.task == "bd_walker" or args.walk_label:
        walk(S, rep, args.walk_label or None)
    # ③ 评分对不对
    if scenario(S, rep):
        env.reward_manager.step(env_idx_list=[0])
        r = env.reward_manager.get_reward(final_check=True)
        rep["pipeline_reward_in_success_state"] = float(r[0])
        log("   RoboDojo 自己的 step → get_reward(停在做成了的样子):%s" % r)
        save_images(env, rep, "L%d_done" % lid)
    rep["ok"] = rep["loaded"] and all(t["ok"] for t in rep["tests"]) and rep.get("pipeline_reward_in_success_state", 1.0) > 0.999 \
        and all(o["present"] for o in rep["objects"])
    env.close()

REPORT["total_s"] = round(time.time() - T0, 1)
REPORT["ok"] = all(l.get("ok") for l in REPORT["layouts"])
json.dump(REPORT, open(os.path.join(OUT, "report.json"), "w"), indent=1, ensure_ascii=False)
log("完:%s ok=%s" % (args.task, REPORT["ok"]))
app.close()
