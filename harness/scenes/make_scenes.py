# -*- coding: utf-8 -*-
"""body-driver 小场景生成器(大并行 §2 第 38 条,路 8)。

在箱上跑(Isaac 自带的 pxr,不起 Isaac、不占卡):
    bash harness/scenes/usdpy.sh harness/scenes/make_scenes.py /root/RoboDojo
只往 RoboDojo 里【加】文件,原有文件一个都不动(每一个要写的路径先过 _mine():名字里带 bd_,且不是 RoboDojo 原有的东西):
    Assets/Object/RoboDojo/{Articulation,Rigid,Geometry,Garment}/bd_*/00000/{object.usd, metadata.json, description.json}
    Assets/Material/bd_white/bd_white.mdl            (名字不以 material 开头 ⇒ 不会混进 RoboDojo 随机桌面材质的池子)
    task/RoboDojo/bd/scene.py                        (判据 + 会自己走的东西;在 tasks/ 外面 ⇒ 不进任务清单)
    task/RoboDojo/tasks/bd_*.py、task/RoboDojo/config/bd_*.yml
    Assets/Eval_Layout/RoboDojo/arx_x5/0/bd_*_<n>.json
几何都是这里自己定的(不照 RoboDojo 的布局抄、不照我们手上硬件的 CAD 抄);现成资产能用的就用(玩具校车、喷漆罐、OmniGlass 材质、RoboDojo 的布料参数)。
"""
import json
import math
import os
import shutil
import sys

import numpy as np
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = f"{R}/Assets/Object/RoboDojo"
LAYOUT_DIR = f"{R}/Assets/Eval_Layout/RoboDojo/arx_x5/0"
N_LAYOUTS = 3
SCENE_CFG = yaml.safe_load(open(f"{R}/env_cfg/scene/default.yml"))
TABLE = SCENE_CFG["Table"]
TABLE_TOP = float(TABLE["default_pos"][2]) + float(TABLE["scale"][2]) / 2.0   # 0.74 + 0.025 = 0.765
WRITTEN = []


def _mine(path):
    """只许写我们自己加的东西:路径里带 bd_(或 task/RoboDojo/bd/),而且不是 git 里 RoboDojo 原有的文件"""
    rel = os.path.relpath(path, R)
    ok = ("/bd_" in "/" + rel or rel.startswith("task/RoboDojo/bd/")) and ".." not in rel
    assert ok, f"不许写这个路径(不是 bd_ 开头的新文件): {rel}"
    WRITTEN.append(rel)
    return path


# ---------------------------------------------------------------- 四元数(w, x, y, z)
def q_axis(axis, deg):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    h = math.radians(deg) / 2.0
    return [math.cos(h)] + list(a * math.sin(h))


def q_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def rotm(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


QI = [1.0, 0.0, 0.0, 0.0]


def gq(q):
    return Gf.Quatf(float(q[0]), Gf.Vec3f(float(q[1]), float(q[2]), float(q[3])))


# ---------------------------------------------------------------- USD 资产
class Asset:
    def __init__(self, kind, cat, idx=0):
        self.kind, self.cat, self.idx = kind, cat, idx
        self.dir = _mine(f"{OBJ}/{kind}/{cat}/{idx:05d}")
        WRITTEN.pop()
        os.makedirs(self.dir, exist_ok=True)
        path = _mine(f"{self.dir}/object.usd")
        if os.path.exists(path):
            os.remove(path)
        self.layer = Sdf.Layer.CreateNew(path, args={"format": "usda"})
        self.st = Usd.Stage.Open(self.layer)
        UsdGeom.SetStageUpAxis(self.st, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.st, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.st, 1.0)
        self.root = UsdGeom.Xform.Define(self.st, "/World")
        self.st.SetDefaultPrim(self.root.GetPrim())
        UsdGeom.Scope.Define(self.st, "/World/Looks")
        self.mats = {}
        self.cols = []          # (中心, 尺寸, 四元数, 连杆):碰撞盒,算包围盒、"躺在桌上时底有多低"、动的那一节扫过哪儿
        self.joints = []        # (子连杆, 类型, 轴点, 轴, 关节系, 下限, 上限):摆布局时查动的那一节整个行程扫过的地方
        self.functional = {}
        self.physics = {}
        self.desc = []

    # 材质
    def mat(self, name, rgb, rough=0.5, metal=0.0):
        if name not in self.mats:
            m = UsdShade.Material.Define(self.st, f"/World/Looks/{name}")
            s = UsdShade.Shader.Define(self.st, f"/World/Looks/{name}/Shader")
            s.CreateIdAttr("UsdPreviewSurface")
            s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
            s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
            m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
            self.mats[name] = m
        return self.mats[name]

    def glass(self, name="glass"):
        """Isaac 自带的 OmniGlass(Kit 的 mdl/core/Base 里就有,和 RoboDojo 房间窗户用的是同一个)"""
        if name not in self.mats:
            m = UsdShade.Material.Define(self.st, f"/World/Looks/{name}")
            s = UsdShade.Shader.Define(self.st, f"/World/Looks/{name}/Shader")
            s.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
            s.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
            s.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.96, 0.98, 1.0))
            s.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(1.5)
            out = s.CreateOutput("out", Sdf.ValueTypeNames.Token)
            m.CreateSurfaceOutput("mdl").ConnectToSource(out)
            m.CreateVolumeOutput("mdl").ConnectToSource(out)
            m.CreateDisplacementOutput("mdl").ConnectToSource(out)
            self.mats[name] = m
        return self.mats[name]

    def phys(self, name, static, dynamic, restitution=0.0):
        key = "phys_" + name
        if key not in self.mats:
            m = UsdShade.Material.Define(self.st, f"/World/Looks/{key}")
            api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
            api.CreateStaticFrictionAttr(float(static))
            api.CreateDynamicFrictionAttr(float(dynamic))
            api.CreateRestitutionAttr(float(restitution))
            self.mats[key] = m
        return self.mats[key]

    @staticmethod
    def bind(prim, mat, purpose=None):
        api = UsdShade.MaterialBindingAPI.Apply(prim)
        if purpose:
            api.Bind(mat, UsdShade.Tokens.weakerThanDescendants, purpose)
        else:
            api.Bind(mat)

    # 形状(坐标都是资产系:所有连杆在原位时的变换都是单位阵)
    def box(self, path, center, size, mat=None, collide=True, visible=True, quat=None, pmat=None):
        c = UsdGeom.Cube.Define(self.st, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
        if quat is not None:
            c.AddOrientOp().Set(gq(quat))
        c.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
        c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
            self.cols.append((np.asarray(center, dtype=float), np.asarray(size, dtype=float), quat or QI, path.rsplit("/", 1)[0]))
        if not visible:
            c.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        if mat is not None:
            self.bind(c.GetPrim(), mat)
        if pmat is not None:
            self.bind(c.GetPrim(), pmat, "physics")
        return c

    def cyl(self, path, center, radius, height, mat=None, collide=True, visible=True, pmat=None):
        c = UsdGeom.Cylinder.Define(self.st, path)
        c.CreateRadiusAttr(float(radius))
        c.CreateHeightAttr(float(height))
        c.CreateAxisAttr(UsdGeom.Tokens.z)
        c.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2), Gf.Vec3f(radius, radius, height / 2)])
        c.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
            self.cols.append((np.asarray(center, dtype=float), np.array([2 * radius, 2 * radius, height]), QI, path.rsplit("/", 1)[0]))
        if not visible:
            c.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        if mat is not None:
            self.bind(c.GetPrim(), mat)
        if pmat is not None:
            self.bind(c.GetPrim(), pmat, "physics")
        return c

    def mesh(self, path, pts, counts, idx, mat=None, normals=None):
        m = UsdGeom.Mesh.Define(self.st, path)
        P = np.asarray(pts, dtype=float)
        m.CreatePointsAttr([Gf.Vec3f(*p) for p in P])
        m.CreateFaceVertexCountsAttr([int(c) for c in counts])
        m.CreateFaceVertexIndicesAttr([int(i) for i in idx])
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateExtentAttr([Gf.Vec3f(*P.min(axis=0)), Gf.Vec3f(*P.max(axis=0))])
        if normals is not None:
            m.CreateNormalsAttr([Gf.Vec3f(*n) for n in normals])
            m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        if mat is not None:
            self.bind(m.GetPrim(), mat)
        return m

    # 物理
    def rigid(self, mass):
        UsdPhysics.RigidBodyAPI.Apply(self.root.GetPrim())
        UsdPhysics.MassAPI.Apply(self.root.GetPrim()).CreateMassAttr(float(mass))

    def articulated(self):
        # 只用 USD 自带的 UsdPhysics:这里的关节体都只有两节、父子相连(关节自己就把这一对的碰撞滤掉),
        # PhysX 默认的求解迭代(位置 32、速度 1)也够,用不着 PhysxArticulationAPI
        UsdPhysics.ArticulationRootAPI.Apply(self.root.GetPrim())

    def link(self, name, mass):
        x = UsdGeom.Xform.Define(self.st, f"/World/{name}")
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(float(mass))
        return f"/World/{name}"

    def fix_to_world(self, link):
        j = UsdPhysics.FixedJoint.Define(self.st, f"{link}/fix_to_world")
        j.CreateBody1Rel().SetTargets([Sdf.Path(link)])

    def joint(self, kind, name, parent, child, anchor, axis, lo, hi, frame=QI, stiffness=0.0, damping=0.0):
        """kind = revolute(限位 度;刚度、阻尼按 每弧度 给,USD 角驱动的单位是 每度 ⇒ 乘 π/180)/ prismatic(米)"""
        J = UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint
        j = J.Define(self.st, f"{child}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateAxisAttr(axis)
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        a = Gf.Vec3f(*[float(v) for v in anchor])
        j.CreateLocalPos0Attr(a)
        j.CreateLocalPos1Attr(a)
        j.CreateLocalRot0Attr(gq(frame))
        j.CreateLocalRot1Attr(gq(frame))
        if stiffness or damping:
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "revolute" else "linear")
            per = math.pi / 180.0 if kind == "revolute" else 1.0
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(float(stiffness) * per)
            d.CreateDampingAttr(float(damping) * per)
            d.CreateTargetPositionAttr(0.0)
            d.CreateMaxForceAttr(1.0e4)
        self.joints.append((child, kind, np.asarray(anchor, dtype=float), axis, frame, float(lo), float(hi)))
        return f"{child}/{name}"

    # 几何(资产系)
    @staticmethod
    def _box_corners(c, s, q):
        Rb = rotm(q)
        return [c + Rb @ (s * np.array([sx, sy, sz])) for sx in (-0.5, 0.5) for sy in (-0.5, 0.5) for sz in (-0.5, 0.5)]

    def corners(self, joint_values=None):
        """所有碰撞盒的角点(资产系);joint_values = {子连杆: 关节值(度 / 米)} ⇒ 那一节按关节动过去"""
        out = []
        moves = {j[0]: j for j in self.joints}
        for c, s, q, link in self.cols:
            pts = self._box_corners(c, s, q)
            if joint_values and link in moves and link in joint_values:
                _, kind, anchor, axis, frame, _, _ = moves[link]
                e = rotm(frame) @ np.eye(3)["XYZ".index(axis)]
                v = joint_values[link]
                if kind == "prismatic":
                    pts = [p + v * e for p in pts]
                else:
                    Rj = rotm(q_axis(e, v))
                    pts = [anchor + Rj @ (p - anchor) for p in pts]
            out += pts
        return out

    def swept(self, samples=9):
        """静止时 + 每个关节从下限到上限几个位置时,所有碰撞盒的角点(资产系)"""
        out = self.corners()
        for child, kind, anchor, axis, frame, lo, hi in self.joints:
            for v in np.linspace(lo, hi, samples):
                out += self.corners({child: float(v)})
        return out

    # 落盘
    def rest_minz(self, quat):
        """这件东西按 quat 摆着时,碰撞盒最低点在资产原点下面多少(负数)"""
        Rq = rotm(quat)
        return min(0.0, min((Rq @ p)[2] for p in self.corners()))

    def save(self):
        self.layer.Save()
        P = np.array(self.corners())
        lo, hi = P.min(axis=0), P.max(axis=0)
        verts = [[float(x), float(y), float(z)] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        ext = [float(v) for v in (hi - lo)]
        geo = {"aligned_bbox": {"vertices": verts, "extents": ext}, "oriented_bbox": {"vertices": verts, "extents": ext},
               "radius": float(np.linalg.norm(hi - lo) / 2.0)}
        meta = {"uuid": f"bd-{self.cat}-{self.idx:05d}", "physics": self.physics, "geometry": geo, "active": {"place": {}, "functional": {}},
                "passive": {"support": {}, "functional": self.functional}}
        json.dump(meta, open(_mine(f"{self.dir}/metadata.json"), "w"), indent=1)
        json.dump({"uuid": meta["uuid"], "description": self.desc, "caption": ""}, open(_mine(f"{self.dir}/description.json"), "w"), indent=1)
        return self


ASSETS = {}


def build(fn):
    a = fn()
    ASSETS[a.cat] = a.save()
    return fn


# ---------------------------------------------------------------- 1 带抽屉的柜子(关节体,底座固定;资产系里开口朝 +Y,摆的时候转 180° 朝机器人)
@build
def cabinet():
    a = Asset("Articulation", "bd_cabinet")
    a.articulated()
    T, W, D, H = 0.012, 0.32, 0.28, 0.22
    wood, front, inner, dark = a.mat("wood", (0.70, 0.52, 0.34), 0.6), a.mat("front", (0.58, 0.40, 0.25), 0.55), a.mat("inner", (0.66, 0.50, 0.34), 0.7), a.mat("handle", (0.12, 0.12, 0.14), 0.3, 0.6)
    grip = a.phys("grip", 0.9, 0.8)
    b = a.link("cabinet_body", 4.0)
    a.fix_to_world(b)
    a.box(f"{b}/bottom", (0, 0, T / 2), (W, D, T), wood)
    a.box(f"{b}/top", (0, 0, H - T / 2), (W, D, T), wood)
    a.box(f"{b}/left", (-W / 2 + T / 2, 0, H / 2), (T, D, H), wood)
    a.box(f"{b}/right", (W / 2 - T / 2, 0, H / 2), (T, D, H), wood)
    a.box(f"{b}/back", (0, -D / 2 + T / 2, H / 2), (W, T, H), wood)
    d = a.link("drawer", 0.4)
    a.box(f"{d}/bottom", (0, 0.015, 0.025), (0.284, 0.25, 0.01), inner)
    a.box(f"{d}/left", (-0.137, 0.015, 0.08), (0.01, 0.25, 0.12), inner)
    a.box(f"{d}/right", (0.137, 0.015, 0.08), (0.01, 0.25, 0.12), inner)
    a.box(f"{d}/back", (0, -0.105, 0.08), (0.284, 0.01, 0.12), inner)
    a.box(f"{d}/front", (0, D / 2 + 0.008, 0.11), (0.30, 0.016, 0.19), front)
    a.box(f"{d}/handle_bar", (0, D / 2 + 0.016 + 0.028, 0.13), (0.12, 0.016, 0.016), dark, pmat=grip)
    for s in (-1, 1):
        a.box(f"{d}/handle_post{'lr'[s > 0]}", (0.05 * s, D / 2 + 0.016 + 0.012, 0.13), (0.014, 0.024, 0.014), dark, pmat=grip)
    a.joint("prismatic", "drawer_joint", b, d, (0, D / 2, 0.11), "Y", 0.0, 0.20, damping=8.0)
    a.functional = {"drawer": {"base_link": "drawer", "parent_joint": "drawer_joint", "frame": [[0.0, D / 2 + 0.044, 0.13, 1.0, 0.0, 0.0, 0.0]]}}
    a.physics = {"mass": 4.4, "friction": 0.8}
    a.desc = ["wooden cabinet with a drawer", "small wooden drawer cabinet"]
    return a


# ---------------------------------------------------------------- 2 带盖的盒子(关节体,不固定:放在桌上的盒子;盖子铰链在后上沿)
@build
def lidbox():
    a = Asset("Articulation", "bd_lidbox")
    a.articulated()
    Wb, Db, Hb, t = 0.20, 0.14, 0.08, 0.008
    body, lidc, knob = a.mat("box", (0.12, 0.48, 0.52), 0.5), a.mat("lid", (0.18, 0.60, 0.64), 0.45), a.mat("knob", (0.93, 0.93, 0.90), 0.4)
    grip = a.phys("grip", 0.9, 0.8)
    b = a.link("box_body", 0.6)
    a.box(f"{b}/bottom", (0, 0, t / 2), (Wb, Db, t), body)
    a.box(f"{b}/left", (-Wb / 2 + t / 2, 0, Hb / 2), (t, Db, Hb), body)
    a.box(f"{b}/right", (Wb / 2 - t / 2, 0, Hb / 2), (t, Db, Hb), body)
    a.box(f"{b}/front", (0, -Db / 2 + t / 2, Hb / 2), (Wb, t, Hb), body)
    a.box(f"{b}/back", (0, Db / 2 - t / 2, Hb / 2), (Wb, t, Hb), body)
    lid = a.link("lid", 0.08)
    a.box(f"{lid}/plate", (0, 0, Hb + 0.005), (Wb + 0.006, Db + 0.006, 0.01), lidc)
    a.box(f"{lid}/knob", (0, -0.055, Hb + 0.010 + 0.010), (0.05, 0.016, 0.02), knob, pmat=grip)
    # 铰链轴 = 资产系 -X(关节系绕 Z 转 180°)⇒ 角度往正走 = 盖子前沿往上翻开
    a.joint("revolute", "lid_joint", b, lid, (0, Db / 2 + 0.003, Hb + 0.005), "X", 0.0, 110.0, frame=q_axis((0, 0, 1), 180.0), damping=0.01)
    a.functional = {"lid": {"base_link": "lid", "parent_joint": "lid_joint", "frame": [[0.0, -0.055, Hb + 0.03, 1.0, 0.0, 0.0, 0.0]]}}
    a.physics = {"mass": 0.68, "friction": 0.8}
    a.desc = ["teal box with a hinged lid", "teal lidded box"]
    return a


@build
def cube():
    a = Asset("Rigid", "bd_cube")
    a.rigid(0.03)
    a.box("/World/cube", (0, 0, 0.015), (0.03, 0.03, 0.03), a.mat("red", (0.85, 0.10, 0.10), 0.5))
    a.physics = {"mass": 0.03, "friction": 0.6}
    a.desc = ["small red cube"]
    return a


# ---------------------------------------------------------------- 3 带铰链的板(一扇立着的小门:板绕竖轴转,轴在一根固定的立柱上)
@build
def hinge_board():
    a = Asset("Articulation", "bd_hinge_board")
    a.articulated()
    gray, orange, black = a.mat("frame", (0.25, 0.25, 0.28), 0.5, 0.3), a.mat("board", (0.92, 0.52, 0.16), 0.5), a.mat("handle", (0.08, 0.08, 0.08), 0.4)
    grip = a.phys("grip", 0.9, 0.8)
    f = a.link("frame", 2.0)
    a.fix_to_world(f)
    a.box(f"{f}/base", (0, 0, 0.006), (0.07, 0.07, 0.012), gray)
    a.box(f"{f}/post", (0, 0, 0.012 + 0.135), (0.02, 0.02, 0.27), gray)
    bd = a.link("board", 0.25)
    a.box(f"{bd}/panel", (0.125, 0, 0.15), (0.21, 0.012, 0.22), orange)
    a.box(f"{bd}/handle_bar", (0.20, -0.006 - 0.026, 0.15), (0.014, 0.014, 0.08), black, pmat=grip)
    for k, z in (("lo", 0.12), ("hi", 0.18)):
        a.box(f"{bd}/handle_post_{k}", (0.20, -0.006 - 0.012, z), (0.014, 0.024, 0.014), black, pmat=grip)
    a.joint("revolute", "board_joint", f, bd, (0, 0, 0.15), "Z", -100.0, 100.0, damping=0.2)
    a.functional = {"board": {"base_link": "board", "parent_joint": "board_joint", "frame": [[0.20, -0.032, 0.15, 1.0, 0.0, 0.0, 0.0]]}}
    a.physics = {"mass": 2.25, "friction": 0.8}
    a.desc = ["orange board on a hinge", "small orange hinged door"]
    return a


# ---------------------------------------------------------------- 4 孔和销(方孔 20.8 mm、方销 20.0 mm ⇒ 每边 0.4 mm、合起来 0.8 mm < 1 mm)
@build
def hole_block():
    a = Asset("Geometry", "bd_hole_block")
    B, Hh, s, tb = 0.12, 0.06, 0.0208, 0.01
    yel = a.mat("block", (0.95, 0.78, 0.18), 0.55)
    pm = a.phys("block", 0.6, 0.5)
    g = UsdGeom.Xform.Define(a.st, "/World/block").GetPath().pathString
    a.box(f"{g}/bottom", (0, 0, tb / 2), (B, B, tb), yel, pmat=pm)
    w = (B / 2 - s / 2)
    zc, hz = tb + (Hh - tb) / 2, Hh - tb
    a.box(f"{g}/left", (-(B / 2 + s / 2) / 2, 0, zc), (w, B, hz), yel, pmat=pm)
    a.box(f"{g}/right", ((B / 2 + s / 2) / 2, 0, zc), (w, B, hz), yel, pmat=pm)
    a.box(f"{g}/front", (0, -(B / 2 + s / 2) / 2, zc), (s, w, hz), yel, pmat=pm)
    a.box(f"{g}/back", (0, (B / 2 + s / 2) / 2, zc), (s, w, hz), yel, pmat=pm)
    a.functional = {"hole": {"base_link": None, "parent_joint": None, "frame": [[0.0, 0.0, Hh, 1.0, 0.0, 0.0, 0.0]],
                             "half_width": s / 2, "depth": Hh - tb}}
    a.physics = {"mass": 0.0, "friction": 0.6}
    a.desc = ["yellow block with a square hole"]
    return a


@build
def peg():
    a = Asset("Rigid", "bd_peg")
    a.rigid(0.06)
    a.box("/World/peg", (0, 0, 0.035), (0.020, 0.020, 0.07), a.mat("peg", (0.16, 0.34, 0.86), 0.45))
    a.functional = {"bottom": {"frame": [[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]}, "top": {"frame": [[0.0, 0.0, 0.07, 1.0, 0.0, 0.0, 0.0]]}}
    a.physics = {"mass": 0.06, "friction": 0.4}
    a.desc = ["blue square peg"]
    return a


# ---------------------------------------------------------------- 5 挂钩和环
@build
def hook():
    a = Asset("Geometry", "bd_hook")
    base, metal = a.mat("base", (0.35, 0.35, 0.38), 0.5), a.mat("hook", (0.20, 0.20, 0.22), 0.35, 0.7)
    pm = a.phys("hook", 0.5, 0.4)
    g = UsdGeom.Xform.Define(a.st, "/World/hook").GetPath().pathString
    a.box(f"{g}/base", (0, 0, 0.006), (0.10, 0.10, 0.012), base, pmat=pm)
    a.box(f"{g}/post", (0, 0, 0.012 + 0.15), (0.02, 0.02, 0.30), metal, pmat=pm)
    a.box(f"{g}/arm", (0, -0.01 - 0.07, 0.29), (0.012, 0.14, 0.012), metal, pmat=pm)
    a.box(f"{g}/tip", (0, -0.15 + 0.006, 0.296 + 0.014), (0.012, 0.012, 0.028), metal, pmat=pm)
    a.functional = {"arm": {"base_link": None, "parent_joint": None,
                            "frame": [[0.0, -0.01, 0.29, 1.0, 0.0, 0.0, 0.0], [0.0, -0.15, 0.29, 1.0, 0.0, 0.0, 0.0]]}}
    a.physics = {"mass": 0.0, "friction": 0.5}
    a.desc = ["hook on a stand"]
    return a


@build
def ring():
    a = Asset("Rigid", "bd_ring")
    a.rigid(0.04)
    Rm, r, n = 0.034, 0.006, 16
    red = a.mat("ring", (0.85, 0.12, 0.12), 0.4)
    # 看得见的:圆环面(32 × 12)
    nu, nv = 32, 12
    pts, cnt, idx = [], [], []
    for i in range(nu):
        u = 2 * math.pi * i / nu
        for j in range(nv):
            v = 2 * math.pi * j / nv
            pts.append(((Rm + r * math.cos(v)) * math.cos(u), (Rm + r * math.cos(v)) * math.sin(u), r + r * math.sin(v)))
    for i in range(nu):
        for j in range(nv):
            i2, j2 = (i + 1) % nu, (j + 1) % nv
            cnt.append(4)
            idx += [i * nv + j, i2 * nv + j, i2 * nv + j2, i * nv + j2]
    a.mesh("/World/visual", pts, cnt, idx, red)
    # 碰撞:一圈 16 个方块(截面 12 mm,和圆环的粗细一样)
    L = 2 * Rm * math.sin(math.pi / n) * 1.05
    for k in range(n):
        th = 360.0 * k / n
        c = (Rm * math.cos(math.radians(th)), Rm * math.sin(math.radians(th)), r)
        a.box(f"/World/col_{k:02d}", c, (2 * r, L, 2 * r), None, visible=False, quat=q_axis((0, 0, 1), th))
    a.functional = {"center": {"frame": [[0.0, 0.0, r, 1.0, 0.0, 0.0, 0.0]], "axis": [0.0, 0.0, 1.0], "inner_radius": Rm - r, "outer_radius": Rm + r}}
    a.physics = {"mass": 0.04, "friction": 0.5}
    a.desc = ["red ring"]
    return a


# ---------------------------------------------------------------- 6 旋钮(竖轴,±340°,能转过手腕的行程)
@build
def knob():
    a = Asset("Articulation", "bd_knob")
    a.articulated()
    dark, white, red = a.mat("base", (0.20, 0.22, 0.26), 0.5), a.mat("knob", (0.90, 0.90, 0.88), 0.4), a.mat("mark", (0.90, 0.10, 0.10), 0.4)
    grip = a.phys("grip", 1.0, 0.9)
    b = a.link("knob_base", 1.5)
    a.fix_to_world(b)
    a.box(f"{b}/base", (0, 0, 0.025), (0.10, 0.10, 0.05), dark)
    a.box(f"{b}/tick", (0.038, 0, 0.0505), (0.014, 0.003, 0.001), white, collide=False)
    k = a.link("knob", 0.04)
    a.cyl(f"{k}/cap", (0, 0, 0.05 + 0.0125), 0.022, 0.025, white, pmat=grip)
    a.box(f"{k}/fin", (0, 0, 0.075 + 0.008), (0.044, 0.010, 0.016), white, pmat=grip)
    a.box(f"{k}/pointer", (0.0225, 0, 0.066), (0.006, 0.004, 0.012), red, collide=False)
    a.joint("revolute", "knob_joint", b, k, (0, 0, 0.05), "Z", -340.0, 340.0, damping=0.02)
    a.functional = {"knob": {"base_link": "knob", "parent_joint": "knob_joint", "frame": [[0.0, 0.0, 0.083, 1.0, 0.0, 0.0, 0.0]]}}
    a.physics = {"mass": 1.54, "friction": 0.9}
    a.desc = ["white knob on a dark box", "rotary knob"]
    return a


# ---------------------------------------------------------------- 7 带扳机的枪形东西(关节体,不固定;扳机绕枪的横轴转,有回位弹簧)
@build
def water_gun():
    a = Asset("Articulation", "bd_water_gun")
    a.articulated()
    green, orange, yellow, black = a.mat("body", (0.10, 0.62, 0.30), 0.45), a.mat("tank", (0.95, 0.55, 0.10), 0.45), a.mat("trigger", (0.98, 0.85, 0.15), 0.4), a.mat("nozzle", (0.10, 0.10, 0.10), 0.4)
    grip = a.phys("grip", 0.9, 0.8)
    g = a.link("gun_body", 0.25)
    a.box(f"{g}/barrel", (0.04, 0, 0.0), (0.20, 0.03, 0.04), green, pmat=grip)
    a.box(f"{g}/tank", (-0.005, 0, 0.035), (0.09, 0.036, 0.03), orange, pmat=grip)
    a.box(f"{g}/nozzle", (0.15, 0, 0.0), (0.02, 0.014, 0.014), black, pmat=grip)
    a.box(f"{g}/grip", (-0.04, 0, -0.065), (0.036, 0.028, 0.09), green, pmat=grip)
    tr = a.link("trigger", 0.008)
    a.box(f"{tr}/blade", (0.004, 0, -0.037), (0.008, 0.012, 0.030), yellow, pmat=grip)
    # 绕枪的 +Y 转正角 = 扳机下端往握把那边去(扣);0–25°,回位弹簧 0.2 N·m/rad(扣到底 ≈ 3 cm 力臂上 2 N 多)
    a.joint("revolute", "trigger_joint", g, tr, (0.004, 0, -0.021), "Y", 0.0, 25.0, stiffness=0.2, damping=0.002)
    a.functional = {"trigger": {"base_link": "trigger", "parent_joint": "trigger_joint", "frame": [[0.004, 0.0, -0.045, 1.0, 0.0, 0.0, 0.0]]},
                    "grip": {"base_link": "gun_body", "parent_joint": None, "frame": [[-0.04, 0.0, -0.065, 1.0, 0.0, 0.0, 0.0]]}}
    a.physics = {"mass": 0.258, "friction": 0.8}
    a.desc = ["green water gun", "toy water pistol"]
    return a


# ---------------------------------------------------------------- 8 玻璃杯(看得见的是旋转体网格 + OmniGlass;碰撞是杯底圆柱 + 一圈 16 片薄壁)
@build
def glass_cup():
    a = Asset("Rigid", "bd_glass_cup")
    a.rigid(0.2)
    ro, ri, H, tb, n = 0.034, 0.031, 0.10, 0.008, 48
    prof = [(0.0, 0.0), (ro, 0.0), (ro, H), (ri, H), (ri, tb), (0.0, tb)]
    pts, cnt, idx, nrm = [], [], [], []
    for k in range(len(prof) - 1):
        (r0, z0), (r1, z1) = prof[k], prof[k + 1]
        dr, dz = r1 - r0, z1 - z0
        nl = math.hypot(dr, dz)
        nr, nz = dz / nl, -dr / nl          # 轮廓按逆时针走,外法向 = (dz, -dr)
        for i in range(n):
            u0, u1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            base = len(pts)
            for (rr, zz, uu) in ((r0, z0, u0), (r0, z0, u1), (r1, z1, u1), (r1, z1, u0)):
                pts.append((rr * math.cos(uu), rr * math.sin(uu), zz))
            um = (u0 + u1) / 2
            cnt.append(4)
            idx += [base, base + 1, base + 2, base + 3]
            nrm += [(nr * math.cos(um), nr * math.sin(um), nz)] * 4
    a.mesh("/World/visual", pts, cnt, idx, a.glass(), normals=nrm)
    a.cyl("/World/col_base", (0, 0, tb / 2), ro, tb, None, visible=False)
    Lw = 2 * ro * math.sin(math.pi / 16) * 1.08
    for k in range(16):
        th = 360.0 * k / 16
        rm = (ro + ri) / 2
        a.box(f"/World/col_wall_{k:02d}", (rm * math.cos(math.radians(th)), rm * math.sin(math.radians(th)), tb + (H - tb) / 2),
              (0.004, Lw, H - tb), None, visible=False, quat=q_axis((0, 0, 1), th))
    a.physics = {"mass": 0.2, "friction": 0.5}
    a.desc = ["clear glass cup", "transparent drinking glass"]
    return a


# ---------------------------------------------------------------- 9 白墙白地(静止几何:三面墙 + 身后一面 + 一块地;配白桌面材质)
@build
def white_room():
    a = Asset("Geometry", "bd_white_room")
    w = a.mat("white", (0.92, 0.92, 0.92), 0.8)
    g = UsdGeom.Xform.Define(a.st, "/World/room").GetPath().pathString
    a.box(f"{g}/back", (0, 0.80, 1.0), (3.0, 0.02, 2.0), w)
    a.box(f"{g}/front", (0, -1.40, 1.0), (3.0, 0.02, 2.0), w)
    a.box(f"{g}/left", (-1.30, -0.30, 1.0), (0.02, 2.2, 2.0), w)
    a.box(f"{g}/right", (1.30, -0.30, 1.0), (0.02, 2.2, 2.0), w)
    a.box(f"{g}/floor", (0, -0.30, 0.051), (2.6, 2.2, 0.002), w)
    a.physics = {"mass": 0.0, "friction": 0.5}
    a.desc = ["white walls"]
    return a


# ---------------------------------------------------------------- 11 一块布(粒子布:30 × 30 cm、31 × 31 个点,三角面;物理参数跟 RoboDojo 自己的布一样)
@build
def cloth():
    a = Asset("Garment", "bd_cloth")
    N, S = 31, 0.30
    pts = [(-S / 2 + S * i / (N - 1), -S / 2 + S * j / (N - 1), 0.0) for j in range(N) for i in range(N)]
    cnt, idx = [], []
    for j in range(N - 1):
        for i in range(N - 1):
            v0, v1, v2, v3 = j * N + i, j * N + i + 1, (j + 1) * N + i + 1, (j + 1) * N + i
            tri = [(v0, v1, v2), (v0, v2, v3)] if (i + j) % 2 == 0 else [(v0, v1, v3), (v1, v2, v3)]
            for t in tri:
                cnt.append(3)
                idx += list(t)
    m = a.mesh("/World/cloth", pts, cnt, idx, a.mat("cloth", (0.20, 0.33, 0.72), 0.9))
    m.CreateDoubleSidedAttr(True)
    a.cols = [(np.array([0.0, 0.0, 0.0]), np.array([S, S, 0.002]), QI, "/World")]   # 布没有碰撞盒;这一块只用来算包围盒、摆布局
    a.physics = {"mass": 0.0, "friction": 0.5}
    a.desc = ["blue cloth", "square blue cloth"]
    return a


# ---------------------------------------------------------------- 白桌面材质(OmniPBR,纯色、没有纹理)
MDL_DIR = _mine(f"{R}/Assets/Material/bd_white")
WRITTEN.pop()
os.makedirs(MDL_DIR, exist_ok=True)
open(_mine(f"{MDL_DIR}/bd_white.mdl"), "w").write("""mdl 1.4;

import ::OmniPBR::OmniPBR;

// body-driver 小场景:白桌面(纯色,没有纹理 ⇒ 配点仪器在桌面上配不出点)
export material bd_white(*)
 = ::OmniPBR::OmniPBR(
    diffuse_color_constant: color(0.92f, 0.92f, 0.92f),
    diffuse_tint: color(1.f, 1.f, 1.f),
    albedo_brightness: 1.f,
    reflection_roughness_constant: 0.7f,
    metallic_constant: 0.f,
    specular_level: 0.3f);
""")


# ---------------------------------------------------------------- 现成的资产(RoboDojo 自带):只读它们 metadata 里的包围盒(躺在桌上时底有多低、占哪儿)
def reuse_corners(kind, cat, idx):
    d = json.load(open(f"{OBJ}/{kind}/{cat}/{idx:05d}/metadata.json"))
    return [np.asarray(v, dtype=float) for v in d["geometry"]["aligned_bbox"]["vertices"]]


def reuse_minz(kind, cat, idx):
    return min(v[2] for v in reuse_corners(kind, cat, idx))


# ---------------------------------------------------------------- 布局
BASE_LAYOUT = json.load(open(f"{LAYOUT_DIR}/bootcal_0.json"))   # 只拿环境那几块(房间、桌、地、背景、头顶相机的支架),东西一件不拿
ENV_BLOCKS = {k: BASE_LAYOUT[k] for k in ("Room", "Table", "Ground", "Background")}
CAMERA_STAND = BASE_LAYOUT["Geometry"]["camera_stand"][0]


def yaw(deg):
    return q_axis((0, 0, 1), deg)


def rec(cat, idx, label, pos, quat, ptype, physics=None, **extra):
    r = {"category": cat, "category_idx": idx, "label": label, "default_pos": [float(v) for v in pos], "default_ori": [float(v) for v in quat],
         "scale": [1.0, 1.0, 1.0], "physics": dict(physics or {}, type=ptype), "visual": {}, "relative_plane": "Table",
         "need_check_stable": True, "margin": 0.01, "check_mode": "bbox"}
    r.update(extra)
    return r


def on_table(asset_cat, x, y, quat, lift=0.001):
    return [x, y, TABLE_TOP - ASSETS[asset_cat].rest_minz(quat) + lift]


def layout(objs, table=None):
    out = {}
    for sect, cat, r in objs:
        out.setdefault(sect, {}).setdefault(cat, []).append(r)
    out.setdefault("Geometry", {})["camera_stand"] = [CAMERA_STAND]
    for k, v in ENV_BLOCKS.items():
        out[k] = json.loads(json.dumps(v))
    if table:
        out["Table"]["default"] = table
    return out


def U(rng, lo, hi):
    return float(rng.uniform(lo, hi))


CLOTH_PHYS = {   # 和 RoboDojo 自己的布(fold_clothes)同一套粒子参数:这个仿真里验过能稳
    "particle_system": {"particle_system_enabled": True, "enable_ccd": True, "solver_position_iteration_count": 32, "max_depenetration_velocity": None,
                        "global_self_collision_enabled": True, "non_particle_collision_enabled": True, "contact_offset": 0.008, "rest_offset": 0.004,
                        "particle_contact_offset": 0.005, "fluid_rest_offset": 0.0025, "solid_rest_offset": 0.0025, "wind": None,
                        "max_neighborhood": None, "max_velocity": 5.0},
    "particle_material": {"adhesion": 0.1, "adhesion_offset_scale": 0.0, "cohesion": 0.0, "particle_adhesion_scale": 0.2, "particle_friction_scale": 0.6,
                          "drag": 0.0, "lift": 0.0, "friction": 0.5, "damping": 0.05, "gravity_scale": 2, "viscosity": None,
                          "vorticity_confinement": None, "surface_tension": None},
    "garment_config": {"particle_mass": 0.01, "self_collision": True, "self_collision_filter": True, "stretch_stiffness": 100000000.0,
                       "bend_stiffness": 5000.0, "shear_stiffness": 5000.0, "spring_damping": 10.0},
}


# ---------------------------------------------------------------- 摆布局:每件东西(连同动的那一节整个行程)离身体歇着的两只手够远、东西之间不叠
# 身体(RoboDojo 的双臂 x5)开局歇着时两只手占的地方:check_scenes 在这张桌上量的每一节连杆的位置(见 README「离线核」);
# 手指朝上,比 HAND_Z 低的(平放在桌上的)碰不到手。第一版 bd_drawer 布局 2 就是抽屉拉开 11 cm 顶在歇着的左手上。
HANDS = [(-0.30, -0.352), (0.30, -0.352)]
HAND_R = 0.15
HAND_Z = 0.85


def _world(pts, pos, quat):
    Rq = rotm(quat)
    return [np.asarray(pos, dtype=float) + Rq @ np.asarray(p, dtype=float) for p in pts]


class Item:
    def __init__(self, sect, cat, r, pts, overlap=True, hands=True):
        self.sect, self.cat, self.r, self.pts, self.overlap, self.hands = sect, cat, r, pts, overlap, hands

    def footprint(self):
        P = np.array(self.pts)[:, :2]
        c = (P.min(axis=0) + P.max(axis=0)) / 2
        return c, float(np.linalg.norm(P - c, axis=1).max())


def item(sect, cat, label, x, y, q, ptype, physics=None, lift=0.001, reuse=None, extra_pts=(), **extra):
    """摆一件东西。占的点 = 资产自己的碰撞盒角点,关节体再加上动的那一节从下限到上限扫过的;RoboDojo 自带的资产用它 metadata 里的包围盒"""
    if reuse is not None:   # reuse = RoboDojo 自带资产的编号(0 也算)
        local = reuse_corners("Rigid", cat, reuse)
        minz = min(0.0, min((rotm(q) @ p)[2] for p in local))
    else:
        local = ASSETS[cat].swept()
        minz = ASSETS[cat].rest_minz(q)
    pos = [x, y, TABLE_TOP - minz + lift]
    return Item(sect, cat, rec(cat, 0, label, pos, q, ptype, physics, **extra), _world(local, pos, q) + list(extra_pts))


def _placed_ok(items):
    for it in items:
        if it.hands:
            for p in it.pts:
                if p[2] >= HAND_Z and any(math.hypot(p[0] - hx, p[1] - hy) < HAND_R for hx, hy in HANDS):
                    return False
    fps = [it.footprint() for it in items if it.overlap]
    for i in range(len(fps)):
        for j in range(i):
            if np.linalg.norm(fps[i][0] - fps[j][0]) < fps[i][1] + fps[j][1] + 0.02:
                return False
    return True


def _sample(name, rng):
    art = lambda cat, label, x, y, q: item("Articulation", cat, label, x, y, q, "articulation")
    rig = lambda cat, label, x, y, q, **ph: item("Rigid", cat, label, x, y, q, "rigid", ph)
    # 静止几何:碰撞体在资产自己的 USD 里(和 RoboDojo 的 key_slot 一样不在根上另加碰撞),正好坐在桌面上
    geo = lambda cat, label, x, y, q: item("Geometry", cat, label, x, y, q, "geometry", lift=0.0)
    if name == "bd_drawer":
        return [art("bd_cabinet", "cabinet", U(rng, -0.12, 0.12), U(rng, 0.05, 0.16), yaw(180 + U(rng, -12, 12)))], None
    if name == "bd_lidbox":
        q = yaw(U(rng, -15, 15))
        box = art("bd_lidbox", "box", U(rng, -0.18, 0.18), U(rng, -0.16, 0.02), q)
        x, y = box.r["default_pos"][:2]
        # 盒底板 8 mm,盒子自己离桌 1 mm;方块的原点在它的底面
        inside = Item("Rigid", "bd_cube", rec("bd_cube", 0, "inside", [x, y, TABLE_TOP + 0.001 + 0.008 + 0.001], yaw(U(rng, 0, 90)), "rigid",
                                              {"mass": 0.03, "static_friction": 0.6, "dynamic_friction": 0.5}), [], overlap=False, hands=False)
        return [box, inside], None
    if name == "bd_hinge":
        return [art("bd_hinge_board", "board", U(rng, -0.22, 0.02), U(rng, -0.04, 0.10), yaw(U(rng, -10, 10)))], None
    if name == "bd_peg":
        return [geo("bd_hole_block", "block", U(rng, -0.20, 0.20), U(rng, -0.16, 0.06), yaw(U(rng, 0, 90))),
                rig("bd_peg", "peg", U(rng, -0.20, 0.20), U(rng, -0.16, 0.06), yaw(U(rng, 0, 90)), mass=0.06, static_friction=0.4, dynamic_friction=0.3)], None
    if name == "bd_hook":
        return [geo("bd_hook", "hook", U(rng, -0.12, 0.15), U(rng, 0.03, 0.12), yaw(U(rng, -10, 10))),
                rig("bd_ring", "ring", U(rng, -0.25, 0.25), U(rng, -0.20, 0.0), yaw(U(rng, 0, 90)), mass=0.04, static_friction=0.5, dynamic_friction=0.4)], None
    if name == "bd_knob":
        return [art("bd_knob", "knob", U(rng, -0.18, 0.18), U(rng, -0.16, 0.04), yaw(U(rng, 0, 90)))], None
    if name == "bd_trigger":
        q = q_mul(yaw(U(rng, -20, 20)), q_axis((1, 0, 0), -90.0))   # 左侧着地躺着,握把朝机器人
        return [art("bd_water_gun", "gun", U(rng, -0.18, 0.18), U(rng, -0.14, 0.04), q)], None
    if name == "bd_glass":
        return [rig("bd_glass_cup", "glass", U(rng, -0.18, 0.18), U(rng, -0.16, 0.04), yaw(U(rng, 0, 360)), mass=0.2, static_friction=0.6, dynamic_friction=0.5)], None
    if name == "bd_white":
        can = item("Rigid", "can", "target", U(rng, -0.18, 0.18), U(rng, -0.16, 0.04), yaw(U(rng, 0, 360)), "rigid",
                   {"mass": 0.32, "static_friction": 0.6, "dynamic_friction": 0.5}, reuse=0)
        walls = Item("Geometry", "bd_white_room", rec("bd_white_room", 0, "walls", [0.0, 0.0, 0.0], QI, "geometry", relative_plane="Ground"), [],
                     overlap=False, hands=False)
        return [can, walls], "bd_white"
    if name == "bd_walker":
        # 它会走遍这一块:这一块的四角(外扩它自己的半径)按手的高度查,离歇着的手够远
        (x0, x1), (y0, y1) = region = [[-0.28, 0.28], [-0.13, 0.10]]
        rr = max(np.linalg.norm(p[:2]) for p in reuse_corners("Rigid", "toy_car", 0))
        corners = [np.array([x + sx * rr, y + sy * rr, HAND_Z]) for x in (x0, x1) for y in (y0, y1) for sx in (-1, 1) for sy in (-1, 1)]
        bus = item("Rigid", "toy_car", "target", U(rng, -0.20, 0.20), U(rng, -0.10, 0.06), yaw(U(rng, 0, 360)), "rigid",
                   {"mass": 0.35, "static_friction": 0.6, "dynamic_friction": 0.5}, reuse=0, extra_pts=corners,
                   bd_walk={"speed": 0.01, "turn_every": 25, "region": region, "free_height": 0.005,
                            "seed": int(rng.integers(0, 2 ** 31 - 1)), "yaw0": 0.0})
        return [bus], None
    if name == "bd_cloth":
        q = yaw(U(rng, 0, 90))
        x, y = U(rng, -0.12, 0.12), U(rng, -0.14, 0.04)
        c = Item("Garment", "bd_cloth", rec("bd_cloth", 0, "cloth", [x, y, TABLE_TOP + 0.004], q, "garment", CLOTH_PHYS),
                 _world(ASSETS["bd_cloth"].corners(), [x, y, TABLE_TOP + 0.004], q))
        return [c], None
    raise KeyError(name)


def scene_layouts(name, k):
    rng = np.random.default_rng(2026_1001 + 97 * k + sum(map(ord, name)))
    for _ in range(20000):
        items, table = _sample(name, rng)
        if _placed_ok(items):
            return layout([(it.sect, it.cat, it.r) for it in items], table=table)
    raise RuntimeError(f"{name} 第 {k} 张布局摆不开(离手够远、东西不叠)")


# ---------------------------------------------------------------- 任务(每个任务 = 一份 task/*.py + config/*.yml + 3 张布局)
SCENES = [
    # 名字, 一句话说明(中文,进 docstring), 判据(python 表达式), 给脑的话, 默认步数, 配置里的类别
    ("bd_drawer", "带抽屉的柜子。判据:抽屉关节(滑轨)离开局 ≥ 10 cm(仿真真值:关节位置)。",
     '("bd_joint_moved", {"label": "cabinet", "joint": "drawer_joint", "amount": 0.10})', "Open the drawer of the cabinet.", 600,
     {"Articulation": [("bd_cabinet", "cabinet")]}),
    ("bd_lidbox", "带盖的盒子(盒子没固定,里面有一块小红方块)。判据:盖子铰链转开 ≥ 80°。",
     '("bd_joint_moved", {"label": "box", "joint": "lid_joint", "amount": 1.3963})', "Open the lid of the box.", 600,
     {"Articulation": [("bd_lidbox", "box")], "Rigid": [("bd_cube", "inside")]}),
    ("bd_hinge", "带铰链的板(一扇竖轴的小门,两边都能开 ±100°)。判据:板绕铰链转开 ≥ 60°。",
     '("bd_joint_moved", {"label": "board", "joint": "board_joint", "amount": 1.0472})', "Swing the hinged board open.", 600,
     {"Articulation": [("bd_hinge_board", "board")]}),
    ("bd_peg", "孔和销(方孔 20.8 mm、方销 20.0 mm,缝合起来 0.8 mm)。判据:销底低于孔口 ≥ 2 cm 且在孔口以内。",
     '("bd_peg_in_hole", {"peg": "peg", "block": "block", "depth": 0.02})', "Insert the blue peg into the square hole of the yellow block.", 600,
     {"Rigid": [("bd_peg", "peg")], "Geometry": [("bd_hole_block", "block")]}),
    ("bd_hook", "挂钩和环。判据:钩子的横杆从环当中穿过、环离开桌面挂着。",
     '("bd_ring_on_hook", {"ring": "ring", "hook": "hook"})', "Hang the red ring on the hook.", 600,
     {"Rigid": [("bd_ring", "ring")], "Geometry": [("bd_hook", "hook")]}),
    ("bd_knob", "旋钮(竖轴,±340°,转半圈要超过手腕一次能转的行程时就得松手再拧)。判据:旋钮离开局转过 ≥ 180°。",
     '("bd_joint_moved", {"label": "knob", "joint": "knob_joint", "amount": 3.1416})', "Turn the knob half a turn.", 600,
     {"Articulation": [("bd_knob", "knob")]}),
    ("bd_trigger", "带扳机的枪形东西(扳机是一个带回位弹簧的转轴,0–25°)。判据:同一拍里枪身抬离开局 ≥ 5 cm 且扳机扣下 ≥ 17.5°。",
     '("bd_trigger_while_held", {"gun": "gun", "joint": "trigger_joint", "amount": 0.3054, "lift": 0.05})',
     "Pick up the water gun and pull its trigger.", 600, {"Articulation": [("bd_water_gun", "gun")]}),
    ("bd_glass", "玻璃杯(OmniGlass 材质,透明)。判据:杯子抬离开局 ≥ 10 cm(RoboDojo 的 is_lift)。",
     'self.reward_manager.is_lift(label="glass", z_threshold=0.1)', "Pick up the glass cup by 10 cm.", 400,
     {"Rigid": [("bd_glass_cup", "glass")]}),
    ("bd_white", "白桌白墙(桌面纯白 OmniPBR、四面白墙白地,没有纹理)上放一只喷漆罐。判据:罐子抬离开局 ≥ 10 cm。",
     'self.reward_manager.is_lift(label="target", z_threshold=0.1)', "Pick up the spray can by 10 cm.", 400,
     {"Rigid": [("can", "target")], "Geometry": [("bd_white_room", "walls")]}),
    ("bd_walker", "会自己走的东西:一辆玩具校车在桌面上按步随机走(每个动作 1 cm、每 25 个动作换一次方向、碰边反射;被拿离桌面就不走)。判据:抬离开局 ≥ 10 cm。",
     'self.reward_manager.is_lift(label="target", z_threshold=0.1)', "Catch the toy bus that drives around on the table and lift it 10 cm.", 600,
     {"Rigid": [("toy_car", "target")]}),
    ("bd_cloth", "一块布(30 × 30 cm 粒子布,平铺在桌上)。判据:布上最高的点高出桌面 ≥ 10 cm。",
     '("bd_cloth_lifted", {"label": "cloth", "height": 0.10})', "Pick up the cloth by a corner and lift it 10 cm.", 600,
     {"Garment": [("bd_cloth", "cloth")]}),
]

TASK_TMPL = '''# -*- coding: utf-8 -*-
# body-driver 小场景(大并行 §2 第 38 条,路 8)。由 harness/scenes/make_scenes.py 生成,别手改。
import os

from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager
from task.RoboDojo.bd import scene


class {Cls}Common:
    """{doc}"""

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = int(os.environ.get("BD_STEP_LIM", "{step_lim}")){init_extra}

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)
        scene.install_checks(self.reward_manager.func_parser)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset(){reset_extra}{step_extra}

    def run_reward(self):
        self.reward_manager.check([{check}])

    def gen_instruction(self, env_idx):
        return [{instr!r}]


class {name}({Cls}Common, TaskEnv):
    pass
'''

WALK_INIT = "\n        self.walker = scene.Walker()"
WALK_RESET = "\n        self.walker.reset()"
WALK_STEP = '''

    def step(self, meta_control_list):
        # 一个动作 = collect_interval 个物理子步,这里每个子步走一小段(scene.Walker)
        self.walker.tick(self)
        super().step(meta_control_list)'''

os.makedirs(f"{R}/task/RoboDojo/bd", exist_ok=True)
shutil.copy(f"{HERE}/rd/bd/scene.py", _mine(f"{R}/task/RoboDojo/bd/scene.py"))
os.makedirs(LAYOUT_DIR, exist_ok=True)
for name, doc, check, instr, step_lim, cats in SCENES:
    cls = "".join(p.capitalize() for p in name.split("_"))
    walk = name == "bd_walker"
    src = TASK_TMPL.format(Cls=cls, name=name, doc=doc, step_lim=step_lim, check=check, instr=instr,
                           init_extra=WALK_INIT if walk else "", reset_extra=WALK_RESET if walk else "", step_extra=WALK_STEP if walk else "")
    open(_mine(f"{R}/task/RoboDojo/tasks/{name}.py"), "w").write(src)
    cfg = {}
    for sect, lst in cats.items():
        cfg[sect] = [{"category": [{"name": c, "index": [0]} for c, _ in lst],
                      "select_mode": {"nums": len(lst), "mode": "unique", "label": [lab for _, lab in lst]}}]
    head = f"# body-driver 小场景 {name}(harness/scenes/make_scenes.py 生成):{doc}\n# 布局在 Assets/Eval_Layout/RoboDojo/arx_x5/0/{name}_<n>.json(回放用);这里只列用到的东西。\n"
    open(_mine(f"{R}/task/RoboDojo/config/{name}.yml"), "w").write(head + yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
    for k in range(N_LAYOUTS):
        json.dump(scene_layouts(name, k), open(_mine(f"{LAYOUT_DIR}/{name}_{k}.json"), "w"), indent=1)

print("写了 %d 个文件:" % len(WRITTEN))
for w in WRITTEN:
    print("  ", w)
