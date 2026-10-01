# -*- coding: utf-8 -*-
"""第 40 条(路 8):大客厅 —— 家具、几十件东西、每件该去的地方,外加判"收拾完没有"的评分。几何这里自己定。

箱上跑(Isaac 自带的 pxr,不起 Isaac):bash ../scenes/usdpy.sh make_livingroom.py /root/RoboDojo
写(都是新的,名字都带 bd_lr_):
  Assets/Object/RoboDojo/Geometry/bd_lr_<家具>/00000/{object.usd, metadata.json, description.json}
家具的"放东西的地方"(台面、箱子里)写在 metadata 的 passive.functional 里:{"place": {"center": [x,y,z 顶面], "half": [半长, 半宽], "depth": 箱子深(台面 0)}},
资产系里(摆的时候按家具的位姿转到世界里)。布局和判据在 make_livingroom_layout.py(要先有会走的人形那一具身体的配置)。

客厅 8 × 6 m(x −4 ~ 4、y −3 ~ 3),四面墙 2.5 m 高:
- 沙发(北墙下)、茶几(沙发前)、电视柜(南墙下)、书架(东墙,四层隔板)、厨房台面(西墙)、餐桌(西边,桌上一只果盘)、
  玩具箱、工具箱(敞口的箱子,地上)。
"""
import json
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

R = sys.argv[1] if len(sys.argv) > 1 else "/root/RoboDojo"
OBJ = f"{R}/Assets/Object/RoboDojo/Geometry"
WRITTEN = []


def mine(path):
    rel = os.path.relpath(path, R)
    assert "/bd_lr_" in "/" + rel and ".." not in rel, f"不许写这个路径: {rel}"
    WRITTEN.append(rel)
    return path


class Furn:
    """一件家具:几块盒子拼的静态几何(RoboDojo 的 Geometry),资产系原点在它落地那一面的中心"""

    def __init__(self, cat, desc):
        self.cat, self.desc = cat, desc
        self.dir = f"{OBJ}/{cat}/00000"
        os.makedirs(mine(self.dir), exist_ok=True)
        path = mine(f"{self.dir}/object.usd")
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
        self.mats, self.boxes, self.places = {}, [], {}

    def mat(self, name, rgb, rough=0.7):
        if name not in self.mats:
            m = UsdShade.Material.Define(self.st, f"/World/Looks/{name}")
            sh = UsdShade.Shader.Define(self.st, f"/World/Looks/{name}/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
            m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self.mats[name] = m
        return self.mats[name]

    def box(self, name, center, size, rgb):
        g = UsdGeom.Cube.Define(self.st, f"/World/{name}")
        g.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
        x.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.mat(name + "_m", rgb))
        self.boxes.append((center, size))

    def place(self, name, center, half, depth=0.0):
        """放东西的地方:顶面中心 center(资产系)、半长半宽 half;depth > 0 = 敞口箱子(东西放进去,底在 center 往下 depth)"""
        self.places[name] = {"center": [float(v) for v in center], "half": [float(v) for v in half], "depth": float(depth)}

    def save(self):
        self.layer.Save()
        lo = [min(c[i] - s[i] / 2 for c, s in self.boxes) for i in range(3)]
        hi = [max(c[i] + s[i] / 2 for c, s in self.boxes) for i in range(3)]
        verts = [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        ext = [hi[i] - lo[i] for i in range(3)]
        meta = {"uuid": f"bd-{self.cat}", "physics": {"mass": 0.0, "friction": 0.6},
                "geometry": {"aligned_bbox": {"vertices": verts, "extents": ext}, "oriented_bbox": {"vertices": verts, "extents": ext},
                             "radius": float(math.sqrt(sum(e * e for e in ext)) / 2)},
                "active": {"place": {}, "functional": {}}, "passive": {"support": {}, "functional": {"place": self.places}}}
        json.dump(meta, open(mine(f"{self.dir}/metadata.json"), "w"), indent=1)
        json.dump({"uuid": meta["uuid"], "description": [self.desc], "caption": ""}, open(mine(f"{self.dir}/description.json"), "w"), indent=1)
        print("%-22s %.2f × %.2f × %.2f m  放东西的地方 %s" % (self.cat, ext[0], ext[1], ext[2], list(self.places) or "-"))


WOOD, WHITE, GREY, BEIGE, DARK, BLUE, RED, GREEN = ((0.55, 0.38, 0.22), (0.9, 0.9, 0.88), (0.5, 0.5, 0.52), (0.82, 0.75, 0.62),
                                                    (0.15, 0.15, 0.16), (0.2, 0.35, 0.7), (0.75, 0.2, 0.15), (0.25, 0.55, 0.3))

f = Furn("bd_lr_room", "living room walls")
for nm, c, s in (("north", (0, 3.0, 1.25), (8.0, 0.1, 2.5)), ("south", (0, -3.0, 1.25), (8.0, 0.1, 2.5)),
                 ("west", (-4.0, 0, 1.25), (0.1, 6.0, 2.5)), ("east", (4.0, 0, 1.25), (0.1, 6.0, 2.5))):
    f.box(nm, c, s, WHITE)
# 客厅自己的地:8 × 6 m 的底板,顶面在资产原点(摆在地面高上)。RoboDojo 的 Ground 只有 7 × 7 m(env_spacing),第一版没底板,
# 靠墙(x ±3.6 m)的东西掉出了地(糖盒落下去 3.8 m)
f.box("floor", (0, 0, -0.05), (8.0, 6.0, 0.1), WOOD)
f.save()

f = Furn("bd_lr_sofa", "sofa")
f.box("seat", (0, 0, 0.22), (2.0, 0.9, 0.44), BLUE)
f.box("back", (0, 0.38, 0.6), (2.0, 0.15, 0.4), BLUE)
f.box("arm_l", (-0.95, 0, 0.35), (0.1, 0.9, 0.3), BLUE)
f.box("arm_r", (0.95, 0, 0.35), (0.1, 0.9, 0.3), BLUE)
f.place("seat", (0, -0.07, 0.44), (0.85, 0.3))
f.save()

f = Furn("bd_lr_coffee_table", "coffee table")
f.box("top", (0, 0, 0.40), (1.2, 0.6, 0.04), WOOD)
for i, (x, y) in enumerate(((-0.55, -0.25), (0.55, -0.25), (-0.55, 0.25), (0.55, 0.25))):
    f.box(f"leg{i}", (x, y, 0.19), (0.05, 0.05, 0.38), WOOD)
f.place("top", (0, 0, 0.42), (0.55, 0.25))
f.save()

f = Furn("bd_lr_tv_stand", "TV stand")
f.box("body", (0, 0, 0.25), (1.6, 0.4, 0.5), DARK)
f.box("tv", (0, 0.1, 0.85), (1.2, 0.05, 0.7), DARK)
f.place("top_left", (-0.55, -0.08, 0.5), (0.2, 0.1))
f.save()

f = Furn("bd_lr_bookshelf", "bookshelf")
f.box("side_l", (-0.44, 0, 0.8), (0.02, 0.35, 1.6), WOOD)
f.box("side_r", (0.44, 0, 0.8), (0.02, 0.35, 1.6), WOOD)
f.box("back", (0, 0.17, 0.8), (0.9, 0.01, 1.6), WOOD)
for i, z in enumerate((0.02, 0.42, 0.82, 1.22, 1.59)):
    f.box(f"shelf{i}", (0, 0, z), (0.86, 0.34, 0.03), WOOD)
for i, z in enumerate((0.435, 0.835)):    # 人形够得着的两层
    f.place(f"shelf{i + 1}", (0, -0.02, z), (0.38, 0.12))
f.save()

f = Furn("bd_lr_counter", "kitchen counter")
f.box("body", (0, 0, 0.44), (2.0, 0.6, 0.88), WHITE)
f.box("top", (0, 0, 0.9), (2.04, 0.64, 0.04), GREY)
f.place("top", (0, -0.02, 0.92), (0.9, 0.25))
f.save()

f = Furn("bd_lr_dining_table", "dining table with a fruit bowl")
f.box("top", (0, 0, 0.74), (1.4, 0.9, 0.04), WOOD)
for i, (x, y) in enumerate(((-0.65, -0.4), (0.65, -0.4), (-0.65, 0.4), (0.65, 0.4))):
    f.box(f"leg{i}", (x, y, 0.36), (0.05, 0.05, 0.72), WOOD)
# 果盘:桌面上一只浅盘(底 + 四边),放水果的地方
f.box("bowl_base", (0.3, 0, 0.765), (0.30, 0.30, 0.01), BEIGE)
for nm, c, s in (("bowl_n", (0.3, 0.145, 0.79), (0.30, 0.01, 0.05)), ("bowl_s", (0.3, -0.145, 0.79), (0.30, 0.01, 0.05)),
                 ("bowl_w", (0.155, 0, 0.79), (0.01, 0.30, 0.05)), ("bowl_e", (0.445, 0, 0.79), (0.01, 0.30, 0.05))):
    f.box(nm, c, s, BEIGE)
f.place("fruit_bowl", (0.3, 0, 0.815), (0.13, 0.13), depth=0.045)
f.place("top", (-0.3, 0, 0.76), (0.3, 0.35))
f.save()

for cat, desc, rgb in (("bd_lr_toy_box", "toy box", RED), ("bd_lr_tool_box", "toolbox", GREEN)):
    f = Furn(cat, desc)
    f.box("base", (0, 0, 0.01), (0.6, 0.4, 0.02), rgb)
    for nm, c, s in (("n", (0, 0.195, 0.16), (0.6, 0.01, 0.32)), ("s", (0, -0.195, 0.16), (0.6, 0.01, 0.32)),
                     ("w", (-0.295, 0, 0.16), (0.01, 0.4, 0.32)), ("e", (0.295, 0, 0.16), (0.01, 0.4, 0.32))):
        f.box(nm, c, s, rgb)
    f.place("inside", (0, 0, 0.32), (0.27, 0.17), depth=0.30)
    f.save()
print("写了 %d 个文件" % len(WRITTEN))
