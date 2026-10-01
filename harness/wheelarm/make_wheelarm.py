# -*- coding: utf-8 -*-
"""轮子底盘 + 一条胳膊(大并行 §2 第 39 条,路 8):几何这里自己定,不照 LeKiwi(LeKiwi 是三个全向轮;这里是两轮差速 + 前后两个万向球)。
有前眼(底盘前沿)和腕眼(夹爪上),没有头顶眼。

箱上跑(Isaac 自带的 pxr,不起 Isaac):bash ../scenes/usdpy.sh make_wheelarm.py /root/RoboDojo
写 Assets/Robots/wheelarm/wheelarm.usd(新目录,RoboDojo 原有的文件一个不动)。

身体(机器人自己的系:x 朝前、y 朝左、z 朝上;原点在底盘几何中心):
- 底盘:0.36 × 0.30 × 0.10 m 的盒子,离地 3 cm,6 kg;
- 两个驱动轮:半径 5 cm、宽 3 cm,在底盘两侧(y = ±0.17)、前后居中;轮子是连续转的转轴(没有限位),每个 0.3 kg;
- 两个万向球:半径 2.5 cm,在底盘前后(x = ±0.14)底下,摩擦 0(只撑着,不拽);
- 胳膊装在底盘顶上靠前(x = 0.10):转腰(绕 z)→ 肩(绕 y)→ 大臂 24 cm → 肘(绕 y)→ 小臂 22 cm → 腕俯仰(绕 y)→ 腕 5 cm →
  腕转(绕腕的长轴)→ 掌 + 两根平行手指(各走 0 – 4 cm);
- 关节都在零位时胳膊竖直朝上;歇着的样子(RoboDojo 配置里给)是胳膊往前折、腕眼往前下看。
机器人自己报的、收的就是这些关节(两个轮子的转角、胳膊五个关节、手指),驱动不知道哪个是轮子、哪个是胳膊。
"""
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
D = f"{R}/Assets/Robots/wheelarm"
assert os.path.relpath(D, R) == "Assets/Robots/wheelarm"
os.makedirs(D, exist_ok=True)
PATH = f"{D}/wheelarm.usd"
if os.path.exists(PATH):
    os.remove(PATH)
layer = Sdf.Layer.CreateNew(PATH, args={"format": "usda"})
st = Usd.Stage.Open(layer)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
ROOT = "/wheelarm"
root = UsdGeom.Xform.Define(st, ROOT)
st.SetDefaultPrim(root.GetPrim())
UsdGeom.Scope.Define(st, f"{ROOT}/Looks")
MATS = {}


def mat(name, rgb, rough=0.6, metal=0.0):
    if name not in MATS:
        m = UsdShade.Material.Define(st, f"{ROOT}/Looks/{name}")
        sh = UsdShade.Shader.Define(st, f"{ROOT}/Looks/{name}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        MATS[name] = m
    return MATS[name]


def phys(name, static, dynamic):
    key = "phys_" + name
    if key not in MATS:
        m = UsdShade.Material.Define(st, f"{ROOT}/Looks/{key}")
        api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        MATS[key] = m
    return MATS[key]


def bind(prim, m, purpose=None):
    api = UsdShade.MaterialBindingAPI.Apply(prim)
    if purpose:
        api.Bind(m, UsdShade.Tokens.weakerThanDescendants, purpose)
    else:
        api.Bind(m)


def link(name, mass):
    x = UsdGeom.Xform.Define(st, f"{ROOT}/{name}")
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(float(mass))
    return f"{ROOT}/{name}"


def _place(g, center, quat=None):
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
    if quat is not None:
        x.AddOrientOp().Set(Gf.Quatf(*[float(v) for v in quat]))


def box(lk, name, center, size, rgb, pm=None, quat=None):
    g = UsdGeom.Cube.Define(st, f"{lk}/{name}")
    g.CreateSizeAttr(1.0)
    _place(g, center, quat)
    UsdGeom.Xformable(g).AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    bind(g.GetPrim(), mat(name + "_m", rgb))
    if pm is not None:
        bind(g.GetPrim(), pm, "physics")
    return g


def cyl(lk, name, center, radius, height, axis, rgb, pm=None):
    g = UsdGeom.Cylinder.Define(st, f"{lk}/{name}")
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(float(height))
    g.CreateAxisAttr(axis)
    _place(g, center)
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    bind(g.GetPrim(), mat(name + "_m", rgb))
    if pm is not None:
        bind(g.GetPrim(), pm, "physics")
    return g


def sphere(lk, name, center, radius, rgb, pm=None):
    g = UsdGeom.Sphere.Define(st, f"{lk}/{name}")
    g.CreateRadiusAttr(float(radius))
    _place(g, center)
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    bind(g.GetPrim(), mat(name + "_m", rgb))
    if pm is not None:
        bind(g.GetPrim(), pm, "physics")
    return g


JOINTS = []


def joint(kind, name, parent, child, anchor, axis, lo=None, hi=None):
    """所有连杆的系都和根重合(只是几何摆在各自的地方),轴点给在这个共同的系里 ⇒ 两边的局部位置一样。
    lo / hi 不给 = 没有限位(轮子)。驱动的刚度、阻尼在 RoboDojo 那一侧(robot_config/wheelarm.py 的执行器)给"""
    J = UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint
    j = J.Define(st, f"{ROOT}/joints/{name}")
    j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
    j.CreateAxisAttr(axis)
    if lo is not None:
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
    a = Gf.Vec3f(*[float(v) for v in anchor])
    j.CreateLocalPos0Attr(a)
    j.CreateLocalPos1Attr(a)
    UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "revolute" else "linear").CreateTypeAttr("force")
    JOINTS.append(name)


UsdGeom.Scope.Define(st, f"{ROOT}/joints")
GREY, DARK, BLUE, ORANGE = (0.55, 0.57, 0.6), (0.12, 0.12, 0.13), (0.15, 0.35, 0.75), (0.95, 0.45, 0.1)
rubber = phys("rubber", 1.0, 0.9)
slick = phys("slick", 0.0, 0.0)
grip = phys("grip", 1.2, 1.0)

BZ = 0.08   # 底盘中心离地(离地 3 cm + 半高 5 cm);RoboDojo 那一侧按这个高度摆根
base = link("base_link", 6.0)
# 关节体的根放在底盘这一节上(不放在最外面那层 Xform 上):第一版放在 Xform 上,PhysX 自己挑了大臂当根(关节顺序成了
# 肩、肘、腰、腕、轮子…),RoboDojo 摆根的位姿摆的是大臂,底盘被歪着摆、离地高了 9 cm;自由浮动的身体,根就该是那块底盘
UsdPhysics.ArticulationRootAPI.Apply(st.GetPrimAtPath(base))
box(base, "chassis", (0.0, 0.0, 0.0), (0.36, 0.30, 0.10), GREY)
box(base, "bumper", (0.185, 0.0, -0.02), (0.01, 0.30, 0.04), DARK)
sphere(base, "caster_front", (0.14, 0.0, 0.025 - BZ), 0.025, DARK, slick)
sphere(base, "caster_back", (-0.14, 0.0, 0.025 - BZ), 0.025, DARK, slick)
box(base, "front_eye_housing", (0.17, 0.0, 0.065), (0.03, 0.06, 0.03), DARK)

for side, y in (("left", 0.17), ("right", -0.17)):
    w = link(f"wheel_{side}", 0.3)
    cyl(w, "tyre", (0.0, y, 0.05 - BZ), 0.05, 0.03, "Y", DARK, rubber)
    box(w, "spoke", (0.0, y + (0.016 if y > 0 else -0.016), 0.05 - BZ), (0.07, 0.004, 0.012), ORANGE)   # 看得出轮子在转
    joint("revolute", f"wheel_{side}_joint", base, w, (0.0, y, 0.05 - BZ), "Y")

MZ = 0.05          # 胳膊装在底盘顶面
MX = 0.10
l1 = link("arm_link1", 0.4)
box(l1, "column", (MX, 0.0, MZ + 0.04), (0.06, 0.06, 0.08), BLUE)
joint("revolute", "arm_joint1", base, l1, (MX, 0.0, MZ), "Z", -180.0, 180.0)
SZ = MZ + 0.08
l2 = link("arm_link2", 0.35)
box(l2, "upper_arm", (MX, 0.0, SZ + 0.12), (0.04, 0.045, 0.24), GREY)
joint("revolute", "arm_joint2", l1, l2, (MX, 0.0, SZ), "Y", -100.0, 100.0)
EZ = SZ + 0.24
l3 = link("arm_link3", 0.25)
box(l3, "forearm", (MX, 0.0, EZ + 0.11), (0.035, 0.04, 0.22), BLUE)
joint("revolute", "arm_joint3", l2, l3, (MX, 0.0, EZ), "Y", -150.0, 150.0)
WZ = EZ + 0.22
l4 = link("arm_link4", 0.12)
box(l4, "wrist", (MX, 0.0, WZ + 0.025), (0.04, 0.04, 0.05), GREY)
joint("revolute", "arm_joint4", l3, l4, (MX, 0.0, WZ), "Y", -120.0, 120.0)
PZ = WZ + 0.05
palm = link("gripper_link", 0.15)
box(palm, "palm", (MX, 0.0, PZ + 0.015), (0.03, 0.09, 0.03), DARK)
box(palm, "wrist_eye_housing", (MX + 0.025, 0.0, PZ + 0.012), (0.02, 0.03, 0.024), DARK)
joint("revolute", "arm_joint5", l4, palm, (MX, 0.0, PZ), "Z", -180.0, 180.0)
FZ = PZ + 0.03
for side, s in (("left", 1.0), ("right", -1.0)):
    f = link(f"finger_{side}", 0.03)
    # 合上时两指内侧贴着(手指 1 cm 厚,中心离中线 5 mm);手指沿 ±y 各走 0 – 4 cm ⇒ 张到头 8 cm
    box(f, "pad", (MX, s * 0.005, FZ + 0.025), (0.02, 0.01, 0.05), ORANGE, grip)
    jn = f"finger_{side}_joint"
    joint("prismatic", jn, palm, f, (MX, 0.0, FZ), "Y", 0.0 if s > 0 else -0.04, 0.04 if s > 0 else 0.0)
layer.Save()
print("写了 %s:关节 %s" % (PATH, ", ".join(JOINTS)))
