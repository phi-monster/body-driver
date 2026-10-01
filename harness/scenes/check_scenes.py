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
import sys
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
parser.add_argument("--qseeds", default="", help="bd_question:同一具身体的几道题(种子)在一个进程里挨个核")
parser.add_argument("--no_stability", action="store_true",
                    help="物件池稳不稳:这个进程里跳过 RoboDojo 的'布局稳不稳'那一关,落稳后逐件量它挪了多远、歪了多少(哪件站不住一眼看出来)")
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
        p = np.array(attr.Get(), dtype=np.float32)   # 拷一份(Vt 数组给出来的是只读视图)
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
    if t == "bd_question":
        return question_scenario(S, rep)
    if t == "bd_mouse_floor":
        return mouse_floor_scenario(S, rep)
    if t == "bd_livingroom":
        return livingroom_scenario(S, rep)
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
        # 读到的粒子是不是活的(仿真在动它):撒手走 60 步,拎高的那一角该掉回去、判据跟着变 0
        hs = []
        for _ in range(6):
            S.steps(10)
            hs.append(round(float(S.cloth_points("cloth")[:, 2].max() - top), 4))
        rep["cloth_corner_after_release_every_10_steps"] = hs
        log("   撒手后布最高点每 10 步:%s m" % hs)
        test(rep, "撒手走 60 步(那一角掉回去了)", 0, S.graded())
        test(rep, "粒子是活的(撒手后最高点降了 2 cm 以上)", 1, hs[-1] < 0.12 - 0.02)
        # 最后停在"做成了"的样子给 RoboDojo 的管线判:整块布抬高 12 cm(撒手时拎高的那一角被拉向中间、掉下来以后已不在原来那一角的位置)
        rep["cloth_set_path"] = S.cloth_shift("cloth", np.ones(len(P), dtype=bool), 0.12)
        return True
    return False


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def settle(S, label, max_steps=500):
    """物理往前走,直到这件东西停下(每 10 步看一次:线速度 < 1 cm/s、角速度 < 0.1 rad/s),最多 max_steps 个子步(2 秒)。返回走了几步"""
    o = S.obj(label)
    n = 0
    while n < max_steps:
        S.steps(10)
        n += 10
        if float(np.linalg.norm(_np(o.get_linear_velocity())[:3])) < 0.01 and float(np.linalg.norm(_np(o.get_angular_velocity())[:3])) < 0.1:
            break
    return n


def put_on(S, la, lb, mode):
    """把 A 放到 B 上(on:A 平放,落到 B 投影外接框的中点、最低点比 B 的顶高 5 mm)/ 放进 B 里(in:落到 B 的口的中心、最低点在 B 的半腰;
    A 平放时投影放得进口就平放,放不进就把最长的那根轴竖起来 —— 出题时"放得进"就是按竖着算的),然后物理走到停。
    返回放下去之后量的:A 最低点比 B 的底 / 顶高多少、走了几步、A 竖没竖"""
    from task.RoboDojo.bd import question as Q
    fp = S.rm.func_parser
    ia, ib = S.inst(la), S.inst(lb)
    pa, ra = S.pose(la)
    pb, rb = S.pose(lb)
    cb = Q._corners(fp, 0, ib)
    sa = Q._shape(fp, 0, ia)
    q = np.asarray(ra, dtype=float)
    upright = False
    if mode == "in":
        mb = Q.bd_of(S.meta(lb))
        xy = (pb + rotm(rb) @ np.r_[np.asarray(mb["opening_center"], dtype=float), 0.0])[:2]
        flat = sa @ rotm(q).T
        if 2 * float(np.linalg.norm(flat[:, :2] - Q._center_xy(flat), axis=1).max()) >= float(mb["opening_d"]):
            k = int(np.argmax(sa.max(axis=0) - sa.min(axis=0)))
            h = math.sqrt(0.5)
            q_local = {0: [h, 0.0, -h, 0.0], 1: [h, h, 0.0, 0.0], 2: [1.0, 0.0, 0.0, 0.0]}[k]   # 资产系的这根轴转到竖直
            q = quat_mul(q, q_local)
            upright = True
        low = (cb[:, 2].min() + cb[:, 2].max()) / 2
    else:
        xy = Q._center_xy(cb)
        low = cb[:, 2].max() + 0.005
    world = sa @ rotm(q).T
    S.set_pose(la, np.r_[xy - Q._center_xy(world) + 0.0, low - world[:, 2].min()], q)
    n = settle(S, la)
    ca = Q._corners(fp, 0, ia)
    cb = Q._corners(fp, 0, ib)
    info = {"mode": mode, "upright": upright, "settle_steps": n, "a_low_minus_b_low": round(float(ca[:, 2].min() - cb[:, 2].min()), 4),
            "a_low_minus_b_top": round(float(ca[:, 2].min() - cb[:, 2].max()), 4),
            "a_center_minus_b_center": [round(float(v), 4) for v in Q._center_xy(ca) - Q._center_xy(cb)]}
    log("   %s %s %s(竖起来 %s):走了 %d 步停下,A 最低点比 B 底高 %.4f、比 B 顶高 %.4f,中心差 %s" % (
        la, "放进" if mode == "in" else "放到", lb, upright, n, info["a_low_minus_b_low"], info["a_low_minus_b_top"], info["a_center_minus_b_center"]))
    return info


def pool_scenario(S, rep):
    """核物件池的两张布局:① 物理走 300 个子步(和 RoboDojo 自己核布局稳不稳一样长),逐件量离摆的位置挪了多远、歪了多少,
    按 RoboDojo 的规矩(歪 ≤ 30°、每根轴挪 ≤ 4 cm)算站不站得住;② 这张里每个平顶 / 容器,拿同一张里最小的、放得上 / 放得进(question.fits)
    的那件真放上去 / 放进去,物理走到停,用题里的判据(bdq_on)判一遍,该判 1;判完放回原处"""
    from task.RoboDojo.bd import question as Q
    S.steps(300)
    recs = S.lm.get_layout_records(0, "Rigid")
    rep["pool"] = []
    for r in recs:
        p, qq = S.pose(r["label"])
        d = p - np.asarray(r["default_pos"], dtype=float)
        Rr = rotm(qq) @ rotm(np.asarray(r["default_ori"], dtype=float)).T
        tilt = float(math.degrees(math.acos(max(-1.0, min(1.0, Rr[2, 2])))))
        ok = tilt <= 30.0 and bool((np.abs(d) <= 0.04).all())
        rep["pool"].append({"cat": r["category"], "label": r["label"], "move_m": [round(float(v), 4) for v in d], "tilt_deg": round(tilt, 2), "stands": ok})
        test(rep, "%s 落稳(挪 %.1f mm、歪 %.1f°)" % (r["category"], 1000 * float(np.linalg.norm(d)), tilt), 1, ok)
    metas = {r["label"]: S.meta(r["label"]) for r in recs}
    size = lambda m: float(np.prod(sorted(m["geometry"]["aligned_bbox"]["extents"][:2])))
    for rb in recs:
        mb = metas[rb["label"]]
        if not (Q.bd_of(mb)["flat_top"] or Q.bd_of(mb)["container"]):
            continue
        cand = [ra for ra in recs if ra is not rb and Q.fits(metas[ra["label"]], mb)]
        if not cand:
            rep["pool"].append({"b": rb["category"], "a": None})
            log("   %s:这张里没有放得上 / 放得进的东西" % rb["category"])
            continue
        mode = Q.fits(metas[cand[0]["label"]], mb)
        key = (lambda ra: Q.bd_of(metas[ra["label"]])["pass_d"]) if mode == "in" else (lambda ra: size(metas[ra["label"]]))
        ra = min(cand, key=key)
        p0, r0 = S.pose(ra["label"])
        info = put_on(S, ra["label"], rb["label"], mode)
        got = S.rm.check_once(("bdq_on", {"a": ra["label"], "b": rb["label"], "mode": mode}), 0)
        rep["pool"].append({"b": rb["category"], "a": ra["category"], **info, "graded": int(bool(got))})
        test(rep, "%s %s %s、物理走到停" % (ra["category"], "放进" if mode == "in" else "放到", rb["category"]), 1, got)
        S.set_pose(ra["label"], p0, r0)
        settle(S, ra["label"])
    return False


def robot_root(S):
    """身体的根(底盘)此刻在本 env 原点下的位置、朝向(仿真真值)"""
    key = S.env.robot_manager.robot_key[0]
    org = _np(S.env.scene_manager.env_origins[0])[:3]
    d = key.data
    return d.root_pos_w[0].detach().cpu().numpy() - org, d.root_quat_w[0].detach().cpu().numpy()


def drive(S, rep, name, d_left, d_right, n):
    """n 个动作里两个轮子的目标每个动作各往前转 d_left / d_right 弧度(胳膊、手指照读数不动),量底盘走了多远、转了多少"""
    p0, q0 = robot_root(S)
    yaw = lambda q: math.degrees(math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
    act = S.hold_action()
    key = [k for k in act if k.endswith("arm_joint_state")][0]
    tgt = np.asarray(act[key], dtype=float).copy()
    # 胳膊那一串的顺序是 RoboDojo 按关节体里的顺序重排过的(robot_manager:find_joints 以后把 arm_joints_name 换成了它的顺序),按名字找轮子
    names = list(S.env.robot_manager.robot_list[0].arm_joints_name)
    il, ir = names.index("wheel_left_joint"), names.index("wheel_right_joint")
    rep.setdefault("arm_joint_order", names)
    w0 = tgt[[il, ir]].copy()
    for _ in range(n):
        tgt[il] += d_left
        tgt[ir] += d_right
        a = S.hold_action()
        a[key] = tgt.copy()
        S.env.take_action(a)
    p1, q1 = robot_root(S)
    st = S.env.get_obs()["state"]
    w1 = np.asarray(st[key], dtype=float).reshape(-1)[[il, ir]]
    out = {"actions": n, "d_left_rad": d_left, "d_right_rad": d_right, "wheel_turned_rad": [round(float(v), 3) for v in (w1 - w0)],
           "moved_xy_m": [round(float(v), 4) for v in (p1 - p0)[:2]],
           "dist_m": round(float(np.linalg.norm((p1 - p0)[:2])), 4), "yaw_deg": round(yaw(q1) - yaw(q0), 2), "z_m": [round(float(p0[2]), 4), round(float(p1[2]), 4)]}
    rep.setdefault("drive", {})[name] = out
    log("   底盘 %s:%d 个动作、轮子每个动作 %.3f / %.3f rad(轮子读数转了 %s)⇒ 走了 %s m(%.4f m)、转了 %.2f°、根高 %s;胳膊那一串的顺序 %s" % (
        name, n, d_left, d_right, out["wheel_turned_rad"], out["moved_xy_m"], out["dist_m"], out["yaw_deg"], out["z_m"], names))
    return out


def mouse_floor_scenario(S, rep):
    """第 39 条:底盘真在地上走(两个轮子一起转 ⇒ 往前走;反着转 ⇒ 原地转),老鼠离地 12 cm 判 1、8 cm 判 0"""
    r = 0.05   # 轮子半径(make_wheelarm.py)
    a = drive(S, rep, "往前", 0.004 / r, 0.004 / r, 50)          # 一个动作该走 4 mm(0.1 m/s),50 个动作该走 0.2 m
    test(rep, "两个轮子一起转,底盘往前走了 %.3f m(该 0.2 m 上下)" % a["dist_m"], 1, 0.1 < a["dist_m"] < 0.3)
    b = drive(S, rep, "原地转", -0.004 / r, 0.004 / r, 50)
    test(rep, "两个轮子反着转,底盘原地转了 %.1f°、挪了 %.3f m" % (b["yaw_deg"], b["dist_m"]), 1, abs(b["yaw_deg"]) > 10 and b["dist_m"] < 0.05)
    # 会躲:老鼠摆到离身体最近那一节 0.15 m(布局里 flee_radius 0.25 m 以内),走 5 个动作;它该正背着那一节跑,一个动作 1 cm ⇒ 远出 ~5 cm
    from task.RoboDojo.bd import scene as SC
    p0, q0 = S.pose("target")
    near = SC._nearest_robot_xy(S.env, 0, p0[:2])
    u = (p0[:2] - near) / max(float(np.linalg.norm(p0[:2] - near)), 1e-9)
    S.set_pose("target", np.r_[near + u * 0.15, p0[2]], q0)
    S.steps(10)
    pa, _ = S.pose("target")
    da = float(np.linalg.norm(pa[:2] - SC._nearest_robot_xy(S.env, 0, pa[:2])))
    for _ in range(5):
        S.env.take_action(S.hold_action())
    pb, _ = S.pose("target")
    db = float(np.linalg.norm(pb[:2] - SC._nearest_robot_xy(S.env, 0, pb[:2])))
    rep["flee"] = {"start_m": round(da, 4), "after5_m": round(db, 4)}
    test(rep, "老鼠离身体最近那一节 %.3f m,5 个动作以后 %.3f m(躲开了 ≥ 3 cm)" % (da, db), 1, db - da >= 0.03)
    S.set_pose("target", p0, q0)
    S.steps(10)
    p0, q0 = S.pose("target")
    test(rep, "老鼠在地上", 0, S.graded())
    S.set_pose("target", p0 + [0, 0, 0.08], q0)
    test(rep, "老鼠离地 8 cm", 0, S.graded())
    S.set_pose("target", p0 + [0, 0, 0.12], q0)
    test(rep, "老鼠离地 12 cm", 1, S.graded())
    return True


def livingroom_scenario(S, rep):
    """第 40 条:会走的人形站得住、按走路控制器的命令走得动;大客厅的判据:开局 0,每件都摆到它该去的地方、物理走到停 ⇒ 1,挪走一件 ⇒ 0"""
    from task.RoboDojo.bd import question as Q
    from task.RoboDojo.bd import tidy as TD
    env = S.env
    p0, _ = robot_root(S)
    for _ in range(50):
        env.take_action(S.hold_action())
    p1, _ = robot_root(S)
    rep["stand"] = {"pelvis_z": [round(float(p0[2]), 3), round(float(p1[2]), 3)], "moved_m": round(float(np.linalg.norm((p1 - p0)[:2])), 3)}
    test(rep, "站着不动 50 个动作:骨盆高 %.3f → %.3f m、挪了 %.3f m" % (p0[2], p1[2], rep["stand"]["moved_m"]), 1, p1[2] > p0[2] - 0.1 and rep["stand"]["moved_m"] < 0.1)
    env.bd_cmd = [0.3, 0.0, 0.0, 0.72]
    for _ in range(50):
        env.take_action(S.hold_action())
    p2, _ = robot_root(S)
    env.bd_cmd = [0.0, 0.0, 0.0, 0.72]
    for _ in range(25):
        env.take_action(S.hold_action())
    p3, _ = robot_root(S)
    rep["walk"] = {"moved_xy_m": [round(float(v), 3) for v in (p2 - p1)[:2]], "dist_m": round(float(np.linalg.norm((p2 - p1)[:2])), 3),
                   "pelvis_z_after": round(float(p3[2]), 3)}
    test(rep, "走路控制器 vx 0.3 m/s 走 2 s:走了 %.3f m(该 0.6 m 上下)、停下以后骨盆高 %.3f m" % (rep["walk"]["dist_m"], p3[2]), 1,
         0.4 < rep["walk"]["dist_m"] < 0.8 and p3[2] > p0[2] - 0.1)
    save_images(env, rep, "L0_walked")
    fp = S.rm.func_parser
    test(rep, "开局(东西乱放着)", 0, S.graded())
    lay = S.lm.saved_layouts[0]
    todo = TD.places(lay)
    starts = {lab: S.pose(lab) for lab, _, _ in todo}
    for k, (lab, furn, place) in enumerate(todo):
        fi = S.inst(furn)
        pl = S.meta(furn)["passive"]["functional"]["place"][place]
        fpos, fq = S.pose(furn)
        Rf = rotm(fq)
        top = fpos + Rf @ np.asarray(pl["center"], dtype=float)
        half = np.asarray(pl["half"], dtype=float)
        # 同一处放好几件:沿那一面排开(按序号在面上铺格子),从面上(箱子:箱底往上)放下去
        n = sum(1 for _, f2, p2 in todo if (f2, p2) == (furn, place))
        i = sum(1 for _, f2, p2 in todo[:k] if (f2, p2) == (furn, place))
        cols = int(math.ceil(math.sqrt(n)))
        u = -half[0] + (2 * half[0]) * ((i % cols) + 0.5) / cols
        v = -half[1] + (2 * half[1]) * ((i // cols) + 0.5) / max(1, int(math.ceil(n / cols)))
        xy = (top + Rf @ np.array([u, v, 0.0]))[:2]
        _, q = starts[lab]
        sa = Q._shape(fp, 0, S.inst(lab))
        world = sa @ rotm(q).T
        floor = top[2] - float(pl["depth"])
        S.set_pose(lab, np.r_[xy - Q._center_xy(world), floor + 0.01 - world[:, 2].min()], q)
    S.steps(500)
    got = S.graded()
    done = fp.__dict__.get("_bd_tidy_done", {}).get(0)
    rep["tidy_all_placed"] = {"placed": done[0] if done else None, "of": done[1] if done else None}
    test(rep, "每件都摆到它该去的地方、物理走到停(%s / %s 件放好了)" % (rep["tidy_all_placed"]["placed"], rep["tidy_all_placed"]["of"]), 1, got)
    lab0 = todo[0][0]
    S.set_pose(lab0, *starts[lab0])
    S.steps(100)
    test(rep, "挪回一件(%s)" % lab0, 0, S.graded())
    return False


def question_scenario(S, rep):
    """随机题机的一道题:按它的要求把仿真真值摆成做成了 / 没做成的几种样子(只动场景里的东西,不动身体)"""
    from task.RoboDojo.bd import question as Q
    q = Q.find_question(S.lm.saved_layouts[0])
    req, (name, kw) = q["requirement"], q["check"]
    if req == "pool":
        return pool_scenario(S, rep)
    rep["question"] = {k: q[k] for k in ("qid", "body", "requirement", "sentence")}
    log("   题 %s(%s / %s):%s" % (q["qid"], q["body"], req, q["sentence"]))
    p0, r0 = S.pose("target")
    S.graded = lambda: S.rm.check_once((name, dict(kw)), 0)   # 直接判这一题的判据(不碰 RoboDojo 那张待判的单子,最后管线还要用它)
    test(rep, "开局", 0, S.graded())
    e = 0
    ia = S.inst("target")
    if req == "lift":
        h = float(kw["z_threshold"])
        S.set_pose("target", p0 + [0, 0, h + 0.02], r0)
        test(rep, "抬高 %.0f cm" % ((h + 0.02) * 100), 1, S.graded())
        S.set_pose("target", p0 + [0, 0, h - 0.02], r0)
        test(rep, "只抬 %.0f cm" % ((h - 0.02) * 100), 0, S.graded())
        S.set_pose("target", p0 + [0, 0, h + 0.02], r0)
        return True
    if req == "turn":
        def yawed(deg):
            c, s = math.cos(math.radians(deg) / 2), math.sin(math.radians(deg) / 2)
            w, x, y, z = r0
            return np.array([c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w])   # 绕世界 z 转 deg(左乘)
        S.set_pose("target", p0, yawed(90))
        test(rep, "原地转 90°", 0, S.graded())
        S.set_pose("target", p0, yawed(180))
        test(rep, "原地转 180°", 1, S.graded())
        return True
    if req == "push":
        d = np.asarray(kw["dir"], dtype=float)
        dist = float(kw["dist"])
        goal = p0 + np.r_[d * (dist + 0.01), 0.0]
        S.set_pose("target", goal, r0)
        test(rep, "沿着推过去 %.0f cm(没抬)" % ((dist + 0.01) * 100), 1, S.graded())
        # "被拿起来过"记一次就一直算(判据里记着这一集到过的最高),所以管线判要在拿起来之前做
        S.env.reward_manager.step(env_idx_list=[0])
        rep["pipeline_reward_in_success_state"] = float(S.env.reward_manager.get_reward(final_check=True)[0])
        log("   RoboDojo 自己的 step → get_reward(推过去了、没抬):%s" % rep["pipeline_reward_in_success_state"])
        S.set_pose("target", goal + [0, 0, 0.05], r0)
        S.graded()
        S.set_pose("target", goal, r0)
        test(rep, "同一处,但中间被拿起来过 5 cm", 0, S.graded())
        return "done"
    if req == "next_to":
        ib = S.inst("other")
        pb, _ = S.pose("other")
        u = (p0[:2] - pb[:2]) / max(1e-9, np.linalg.norm(p0[:2] - pb[:2]))
        lo_, hi_ = 0.0, float(np.linalg.norm(p0[:2] - pb[:2]))
        for _ in range(30):   # 沿两者连线往 B 挪,找到离 B 正好 2 cm 的地方
            mid = (lo_ + hi_) / 2
            S.set_pose("target", np.r_[pb[:2] + u * mid, p0[2]], r0)
            gap = Q._poly_gap(Q._footprint(S.rm.func_parser, e, ia), Q._footprint(S.rm.func_parser, e, ib))
            lo_, hi_ = (mid, hi_) if gap < 0.02 else (lo_, mid)
        S.set_pose("target", np.r_[pb[:2] + u * hi_, p0[2]], r0)
        test(rep, "挪到 B 旁边 2 cm", 1, S.graded())
        S.set_pose("target", np.r_[pb[:2] + u * hi_, p0[2] + 0.06], r0)
        test(rep, "在 B 旁边,但悬在半空 6 cm", 0, S.graded())
        S.set_pose("target", np.r_[pb[:2] + u * hi_, p0[2]], r0)
        return True
    if req == "on":
        mode = kw.get("mode", "on")
        pb, _ = S.pose("other")
        ca = Q._corners(S.rm.func_parser, e, ia)
        cb = Q._corners(S.rm.func_parser, e, S.inst("other"))
        z = (cb[:, 2].min() + cb[:, 2].max()) / 2 if mode == "in" else cb[:, 2].max() + 0.005
        S.set_pose("target", np.r_[pb[:2] + (p0[:2] - pb[:2]) * 0.6, z + p0[2] - ca[:, 2].min()], r0)
        test(rep, "在 B 旁边、悬在它%s那个高度(没在它%s)" % (("半腰", "里面") if mode == "in" else ("上面", "上面")), 0, S.graded())
        info = put_on(S, "target", "other", mode)
        rep["on_after_settle"] = info
        test(rep, "放%s B、物理走到停" % ("进" if mode == "in" else "到"), 1, S.graded())
        return True
    if req == "above":
        robot = S.env.robot_manager.robot_list[0]
        ee = _np(S.env.robot_manager.get_real_endpose(robot, env_idx_list=[0], is_relative=True)[0])
        rep["drone_body"] = [float(v) for v in ee[:3]]
        S.set_pose("target", np.r_[ee[:2] + [0.10, 0.0], p0[2]], r0)
        test(rep, "东西在机身正下方偏 10 cm", 0, S.graded())
        S.set_pose("target", np.r_[ee[:2], p0[2]], r0)
        test(rep, "东西在机身正下方", 1, S.graded())
        return True
    raise KeyError(req)


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
if args.no_stability:   # 只在这个进程里:跳过 RoboDojo 的"布局稳不稳"那一关,落稳后逐件量挪了多远、歪了多少
    env.scene_manager.layout_manager.check_layout_stability = lambda e_, render=False: (True, [])
def check_layout(lid, rep, tag=None):
    tag = tag or "L%d" % lid
    t1 = time.time()
    env.reset(seed=[lid])
    rep["loaded"] = True
    rep["reset_s"] = round(time.time() - t1, 1)
    env.run_reward()
    S = Scene(env)
    log("布局 %d 装好了(%.0f s),判据:%s" % (lid, rep["reset_s"], env.reward_manager.check_list[0]))
    # ② 每件东西在不在、落稳后离布局给的位置多远
    rep["objects"] = []
    for sect in ("Rigid", "Articulation", "Geometry", "Garment"):
        for r in S.lm.get_layout_records(0, sect):
            inst = r["inst_name"]
            o = S.lm.get_scene_object(0, inst)
            item = {"type": sect, "label": r.get("label"), "inst": inst, "present": o is not None}
            if o is not None and sect != "Garment":
                p, qq = S.lm.get_instance_pose(env_idx=0, inst_name=inst)
                item["drift_m"] = float(np.linalg.norm(_np(p)[:3] - np.asarray(r["default_pos"], dtype=float)))
                Rr = rotm(_np(qq)[:4]) @ rotm(np.asarray(r["default_ori"], dtype=float)).T   # 开局摆的样子 → 落稳后
                item["tilt_deg"] = float(math.degrees(math.acos(max(-1.0, min(1.0, Rr[2, 2])))))
            rep["objects"].append(item)
            log("   %s %s(%s)在=%s 漂 %s 歪 %s" % (sect, r.get("label"), inst, item["present"], ("%.4f m" % item["drift_m"]) if "drift_m" in item else "-",
                                               ("%.1f°" % item["tilt_deg"]) if "tilt_deg" in item else "-"))
    save_images(env, rep, tag)
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
    if args.task in ("bd_walker", "bd_mouse_floor") or args.walk_label:
        walk(S, rep, args.walk_label or None)
        save_images(env, rep, tag + "_walked")   # 走完以后它在哪(chase_mouse 走满 200 个动作那一炮:老鼠停在 G1 左小臂底下)
    # ③ 评分对不对
    res = scenario(S, rep)
    if res is True:
        env.reward_manager.step(env_idx_list=[0])
        r = env.reward_manager.get_reward(final_check=True)
        rep["pipeline_reward_in_success_state"] = float(r[0])
        log("   RoboDojo 自己的 step → get_reward(停在做成了的样子):%s" % r)
    if res:
        save_images(env, rep, tag + "_done")
    rep["ok"] = rep["loaded"] and all(t["ok"] for t in rep["tests"]) and rep.get("pipeline_reward_in_success_state", 1.0) > 0.999 \
        and all(o["present"] for o in rep["objects"])


def finish(code):
    """写报告、退出。出了错就不走 Kit 的正常关机(第一版 bd_cloth 抛了异常以后 Kit 关机卡住,占着仿真位 5 分钟):
    报告先落盘,正常结束也只给 app.close() 30 秒,卡住就直接退"""
    REPORT["total_s"] = round(time.time() - T0, 1)
    REPORT["ok"] = code == 0 and all(l.get("ok") for l in REPORT["layouts"])
    json.dump(REPORT, open(os.path.join(OUT, "report.json"), "w"), indent=1, ensure_ascii=False)
    log("完:%s ok=%s" % (args.task, REPORT["ok"]))
    sys.stdout.flush()
    sys.stderr.flush()
    if code != 0:
        os._exit(code)
    import threading
    threading.Timer(30.0, lambda: os._exit(0)).start()
    app.close()
    os._exit(0)


QSEEDS = [int(s) for s in args.qseeds.split(",") if s != ""]
RUNS = [(0, qs) for qs in QSEEDS] if QSEEDS else [(int(s), None) for s in args.layouts.split(",") if s != ""]
for lid, qs in RUNS:
    rep = {"layout": lid if qs is None else qs, "tests": [], "loaded": False}
    REPORT["layouts"].append(rep)
    try:
        if qs is not None:   # 随机题机:一题一个种子目录;同一具身体的几题在一个进程里挨个核(换种子 = 换题)
            env.seed_manager.config["seed"] = qs
            env.seed_manager.init_eval()
        check_layout(lid, rep, None if qs is None else "Q%d" % qs)
    except Exception as e:
        import traceback
        rep["error"] = "%s: %s" % (type(e).__name__, e)
        rep["traceback"] = traceback.format_exc()
        log("布局 %d 出错:%s\n%s" % (rep["layout"], rep["error"], rep["traceback"]))
        if type(e).__name__ != "UnStableError":
            finish(3)
        # 布局站不住:RoboDojo 自己的 main.py 也是记下、关掉、接着下一张(这一张算没过)
        rep["unstable"] = True
    env.close()
finish(0)
