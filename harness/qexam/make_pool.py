# -*- coding: utf-8 -*-
"""随机题机的物件池(路 8):Isaac 资产库里自带的 YCB 物件(Isaac/Props/YCB/Axis_Aligned),装成 RoboDojo 的新 Rigid 类别 bdq_<短名>。
不用 RoboDojo 自己的物件、布局和任务。只往 RoboDojo 里加 bdq_ 开头的新目录。

箱上跑(Isaac 自带的 pxr,不起 Isaac、不占卡):bash ../scenes/usdpy.sh make_pool.py /root/RoboDojo
每一件:
  - 下 <名字>.usd 原样存进 Assets/Object/RoboDojo/Rigid/bdq_<短名>/00000/ycb/;
  - 它材质里用到的贴图(8–14 MB 的 PNG)下下来缩成 1024 px 的 JPEG 存 ycb/tex/,原图不留(相机 640×480,物件在画面里一两百像素,用不上原图);
  - 写 object.usd:根上刚体 + 质量(YCB 数据集的),引用 ycb/<名字>.usd,它的网格加碰撞(凸包或凸分解),贴图那一项改指缩过的 JPEG,
    绑物理材质;metadata.json(包围盒 + 出题用的几条:平顶、容器、转 180° 看得出来;
    + 从网格量的形状:凸包顶点(判据找最低点、桌面上的投影用)、最细能从多大的圆口塞进去、容器的口多大)、description.json(给脑的叫法)。
"""
import io
import json
import math
import os
import sys
import urllib.request

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
SRC = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Props/YCB/Axis_Aligned"
OBJ = f"{R}/Assets/Object/RoboDojo/Rigid"
TEX_PX = 1024
# (YCB 名字, 短名, 给脑的叫法, 质量 kg(YCB 数据集), 碰撞近似, 哪根轴朝上, 平顶(上面能放东西), 容器(里面能放东西), 转 180° 看得出来)
# 哪根轴朝上:Isaac 的 YCB 网格是 Y 轴顺着物件的高画的(舞台写的是 Z 朝上,网格只有 0.01 的缩放;Isaac 自己的 Axis_Aligned_Physics 也没转它,
# 照原样放罐子、碗都是侧躺着),而且口朝 −Y:第一版绕 x 转 +90°,量网格才知道碗底、杯底的顶点全在最上面(碗、杯子是扣着的,
# 芥末瓶倒立在瓶盖上 ⇒ 稳不稳那一炮倒了 89.8°)。"y" = 绕 x 转 −90°(网格的 −Y 转到世界的 +Z:罐子、瓶子、杯子、碗、电钻、香蕉、糖盒
# 立着,口朝上;容器建完自己核一遍口朝上,见 build());"z" = 不转(扁盒子、木块、剪刀、马克笔、夹子、泡沫砖平躺在最大的那一面上)。
# 平顶 / 容器 / 转 180° 看得出来 都是按这个摆法说的
POOL = [
    ("003_cracker_box", "cracker_box", "cracker box", 0.411, "convexHull", "z", True, False, True),
    ("004_sugar_box", "sugar_box", "sugar box", 0.514, "convexHull", "y", False, False, True),
    ("005_tomato_soup_can", "soup_can", "tomato soup can", 0.349, "convexHull", "y", False, False, False),
    ("006_mustard_bottle", "mustard", "mustard bottle", 0.603, "convexHull", "y", False, False, True),
    ("008_pudding_box", "pudding_box", "pudding box", 0.187, "convexHull", "z", True, False, True),
    ("009_gelatin_box", "gelatin_box", "gelatin box", 0.097, "convexHull", "z", True, False, True),
    ("010_potted_meat_can", "meat_can", "potted meat can", 0.370, "convexHull", "y", False, False, True),
    ("011_banana", "banana", "banana", 0.066, "convexHull", "y", False, False, True),
    ("024_bowl", "bowl", "bowl", 0.147, "convexDecomposition", "y", False, True, False),
    ("025_mug", "mug", "mug", 0.118, "convexDecomposition", "y", False, True, True),
    ("035_power_drill", "drill", "power drill", 0.895, "convexDecomposition", "y", False, False, True),
    ("036_wood_block", "wood_block", "wooden block", 0.729, "convexHull", "z", True, False, False),
    ("037_scissors", "scissors", "scissors", 0.082, "convexDecomposition", "z", False, False, True),
    ("040_large_marker", "marker", "marker", 0.016, "convexHull", "z", False, False, True),
    ("051_large_clamp", "clamp", "clamp", 0.125, "convexDecomposition", "z", False, False, True),
    ("061_foam_brick", "foam_brick", "foam brick", 0.028, "convexHull", "z", True, False, False),
]
WRITTEN = []


def mine(path):
    rel = os.path.relpath(path, R)
    assert ("/bdq_" in "/" + rel) and ".." not in rel, f"不许写这个路径(不是 bdq_ 开头的新目录): {rel}"
    WRITTEN.append(rel)
    return path


def fetch(url, retries=3):
    last = None
    for _ in range(retries):
        try:
            with urllib.request.urlopen(url, timeout=120) as r:
                return r.read()
        except Exception as e:   # 网络抖一下就再来
            last = e
    raise RuntimeError(f"下不来 {url}: {last}")


def asset_inputs(stage):
    """这一层里所有贴图那样的资产路径:(shader 路径, 输入名, 相对路径)"""
    out = []
    for p in stage.Traverse():
        if p.GetTypeName() != "Shader":
            continue
        for i in UsdShade.Shader(p).GetInputs():
            v = i.Get()
            if isinstance(v, Sdf.AssetPath) and v.path:
                out.append((p.GetPath(), i.GetBaseName(), v.path))
    return out


def mesh_points(root):
    """这件东西所有网格的顶点,在物件自己的系里(根下面的变换 —— YCB 自己的 0.01 缩放、立起来那 90° —— 都算进去);
    和网格边长的中位(按面的顶点顺序,相邻两点算一条边)"""
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    out, edges = [], []
    for p in Usd.PrimRange(root):
        if p.IsA(UsdGeom.Mesh):
            M = xc.GetLocalToWorldTransform(p)
            m = UsdGeom.Mesh(p)
            V = np.asarray([tuple(M.Transform(Gf.Vec3d(*map(float, v)))) for v in m.GetPointsAttr().Get()], dtype=float)
            idx, cnt = np.asarray(m.GetFaceVertexIndicesAttr().Get()), np.asarray(m.GetFaceVertexCountsAttr().Get())
            k = 0
            for n in cnt:
                f = idx[k:k + n]
                edges.append(np.linalg.norm(V[f] - V[np.roll(f, -1)], axis=1))
                k += n
            out.append(V)
    return np.concatenate(out), float(np.median(np.concatenate(edges)))


def min_circle_d(P2):
    """平面上一堆点的最小外接圆直径(Welzl 的增量法;点先打乱,期望线性)"""
    pts = [tuple(p) for p in np.random.default_rng(0).permutation(P2)]

    def circ2(a, b):
        c = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        return c, math.dist(a, c)

    def circ3(a, b, c):
        ax, ay, bx, by, cx, cy = *a, *b, *c
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(d) < 1e-15:
            return max((circ2(a, b), circ2(a, c), circ2(b, c)), key=lambda t: t[1])
        ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
        uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
        return (ux, uy), math.dist(a, (ux, uy))

    inside = lambda c, p: math.dist(c[0], p) <= c[1] * (1 + 1e-12) + 1e-12
    c = (pts[0], 0.0)
    for i, p in enumerate(pts):
        if inside(c, p):
            continue
        c = (p, 0.0)
        for j, q in enumerate(pts[:i]):
            if inside(c, q):
                continue
            c = circ2(p, q)
            for r in pts[:j]:
                if not inside(c, r):
                    c = circ3(p, q, r)
    return 2 * c[1]


def pass_d(H, ext):
    """要从圆口塞进去,最细能多细:顺着最长的那根轴看过去(YCB 这批网格是摆正了的,包围盒的轴就是它的长宽高),凸包投影的最小外接圆直径。
    罐子 = 它的圆;盒子 = 小的两条边的对角线"""
    k = int(np.argmax(ext))
    return min_circle_d(np.delete(H, k, axis=1))


def opening_d(P, step):
    """容器的口能过多大的东西:高过它一半的那些顶点投到桌面上,围在中间的最大空圆的直径和圆心(圆心要在这些点的凸包里;
    碗往下收窄,量到的是半腰那一圈,比碗沿小 —— 东西得能下去一半才算放进去)。step = 圆心网格(网格边长的量级)"""
    from scipy.spatial import Delaunay, cKDTree
    z0, z1 = P[:, 2].min(), P[:, 2].max()
    U = P[P[:, 2] >= (z0 + z1) / 2][:, :2]
    lo, hi = U.min(axis=0), U.max(axis=0)
    gx, gy = np.meshgrid(np.arange(lo[0], hi[0], step), np.arange(lo[1], hi[1], step))
    C = np.c_[gx.ravel(), gy.ravel()]
    C = C[Delaunay(U).find_simplex(C) >= 0]
    d, _ = cKDTree(U).query(C)
    k = int(np.argmax(d))
    return 2 * float(d[k]), C[k]


def build(ycb, short, desc, mass, approx, up, flat_top, container, turnable):
    from PIL import Image
    d = mine(f"{OBJ}/bdq_{short}/00000")
    WRITTEN.pop()
    os.makedirs(f"{d}/ycb/tex", exist_ok=True)
    src_usd = f"{d}/ycb/{ycb}.usd"
    if not os.path.exists(src_usd):
        open(mine(src_usd), "wb").write(fetch(f"{SRC}/{ycb}.usd"))
    src = Usd.Stage.Open(src_usd)
    root_name = src.GetDefaultPrim().GetName()
    tex = []
    for shader, name, rel in asset_inputs(src):
        jpg = f"ycb/tex/{os.path.splitext(os.path.basename(rel))[0]}.jpg"
        if not os.path.exists(f"{d}/{jpg}"):
            im = Image.open(io.BytesIO(fetch(f"{SRC}/{rel}"))).convert("RGB")
            im.thumbnail((TEX_PX, TEX_PX))
            im.save(mine(f"{d}/{jpg}"), quality=88)
        tex.append((str(shader).replace(f"/{root_name}", "/World/ycb", 1), name, jpg))
    path = mine(f"{d}/object.usd")
    if os.path.exists(path):
        os.remove(path)
    layer = Sdf.Layer.CreateNew(path, args={"format": "usda"})
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(float(mass))
    y = st.DefinePrim("/World/ycb", "Xform")
    y.GetReferences().AddReference(f"./ycb/{ycb}.usd")
    if up == "y":
        UsdGeom.Xformable(y).AddRotateXOp().Set(-90.0)   # 网格的 −Y 转到世界的 +Z(立起来、口朝上)
    meshes = [p for p in Usd.PrimRange(y) if p.IsA(UsdGeom.Mesh)]
    assert meshes, f"{ycb} 里没有网格"
    for p in meshes:
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(approx)
    for shader, name, jpg in tex:
        sh = UsdShade.Shader(st.GetPrimAtPath(shader))
        sh.GetInput(name).Set(Sdf.AssetPath(f"./{jpg}"))
    pm = UsdShade.Material.Define(st, "/World/PhysicsMaterial")
    api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    api.CreateStaticFrictionAttr(0.6)
    api.CreateDynamicFrictionAttr(0.5)
    api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
    layer.Save()
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(root.GetPrim()).ComputeAlignedRange()
    lo, hi = [list(map(float, bb.GetMin())), list(map(float, bb.GetMax()))]
    verts = [[x, yy, z] for x in (lo[0], hi[0]) for yy in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    ext = [hi[i] - lo[i] for i in range(3)]
    # 形状(判据和出题都用量的,不用包围盒加系数):凸包的顶点(找最低点、桌面上的投影)、最细能从多大的圆口塞进去、容器的口多大
    from scipy.spatial import ConvexHull
    P, edge = mesh_points(root.GetPrim())
    H = P[ConvexHull(P).vertices]
    shape = {"hull": np.round(H, 4).tolist(), "pass_d": round(pass_d(H, ext), 4), "n_vertices": int(len(P))}
    if container:
        od, oc = opening_d(P, max(edge, 1e-4))
        # 口朝上才算容器:口中间那一圈(半径 = 口的四分之一)往下看,底的顶点得在下半截(第一版碗、杯子扣着放,就是这里没核)
        z0, z1 = P[:, 2].min(), P[:, 2].max()
        core = P[np.hypot(P[:, 0] - oc[0], P[:, 1] - oc[1]) < od / 4]
        assert len(core) and np.median(core[:, 2]) < (z0 + z1) / 2, f"{ycb}:口中间的顶点在上半截(高 {np.median(core[:, 2]):.3f}),扣着放了"
        shape["opening_d"] = round(od, 4)
        shape["opening_center"] = [round(float(v), 4) for v in oc]
    # 出题、判据用的这一块放在 geometry 里面:RoboDojo 读 metadata.json 只留 physics / visual / geometry / active / passive 五项
    # (utils/load_file.py 的 load_object_metadata),第一版放在最上面一层("bd"),判据在仿真里拿不到,悄悄退回了包围盒
    meta = {"uuid": f"bdq-{short}", "physics": {"mass": mass, "friction": 0.5},
            "geometry": {"aligned_bbox": {"vertices": verts, "extents": ext}, "oriented_bbox": {"vertices": verts, "extents": ext},
                         "radius": float(sum(e * e for e in ext) ** 0.5 / 2),
                         "bd": {"ycb": ycb, "desc": desc, "up": up, "flat_top": flat_top, "container": container, "turnable": turnable,
                                "approx": approx, **shape}},
            "active": {"place": {}, "functional": {}}, "passive": {"support": {}, "functional": {}}}
    json.dump(meta, open(mine(f"{d}/metadata.json"), "w"), indent=1)
    json.dump({"uuid": meta["uuid"], "description": [desc], "caption": ""}, open(mine(f"{d}/description.json"), "w"), indent=1)
    print("%-20s %-12s 尺寸 %.3f × %.3f × %.3f m  质量 %.3f  贴图 %d  网格 %d  顶点 %d(边长中位 %.4f)  凸包 %d 点  最细 %.3f%s" % (
        ycb, short, ext[0], ext[1], ext[2], mass, len(tex), len(meshes), len(P), edge, len(H), shape["pass_d"],
        "  口 %.3f" % shape["opening_d"] if container else ""))


for row in POOL:
    build(*row)
print("写了 %d 个文件" % len(WRITTEN))
