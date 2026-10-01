"""Truth for scoring, written beside the observation stream and never into it.

With BD_TRUTH=FILE, every observation the simulator builds appends one JSON line to FILE:

  {"kind": "observation", "call": N,
   "state":   {key: [values]},
   "links":   {"robot0/link": [x, y, z, qw, qx, qy, qz]},
   "cameras": {name: {"pose": [x, y, z, qw, qx, qy, qz], "resolution": [w, h], "K": K or null,
                      "lens": {...} or null}},
   "objects": {inst_name: {"label": label, "type": type, "pose": [x, y, z, qw, qx, qy, qz]}}}

"state" repeats the observation's own state values, so the scorer pairs every line with its
recorded observation exactly instead of trusting that each line was sent. Positions are in the
environment's frame (world less the environment origin). A camera pose is that of its optical
frame (+x right, +y down, +z forward); K is null when the lens is not a pinhole, and "lens" then
holds the rig's F-theta parameters. Links, cameras and objects are the physics state when the
observation is built; the image of the same observation may show an earlier state, as the
renderer lags. An object is reported at its root: the parts of an articulated object are not.

The first line of a run is {"kind": "geometry", "links": {...}, "objects": {...}}: for every
link and object, the vertices of its meshes in its own frame, in "collision" (what touches) and
"visual" (what the cameras see; a mesh can be both), and its other shapes (spheres, capsules,
...) with their attributes and their transform into that frame (column vectors, translation in
the last column), so the scorer computes tips, outlines and support points itself.
"""

import json

import numpy as np

_calls = 0
_geometry_written = False


def _values(v):
    if hasattr(v, "detach"):
        v = v.detach().cpu().numpy()
    return np.asarray(v, dtype=np.float64).ravel().tolist()


def _pose(position, orientation, origin):
    p = np.asarray(_values(position)) - origin
    return [round(float(x), 7) for x in list(p) + _values(orientation)]


def _articulations(om):
    unique = []
    for art in om.robot_manager.robot_key:
        if all(art is not seen for seen in unique):
            unique.append(art)
    return [(art.cfg.prim_path.rstrip("/").split("/")[-1], art) for art in unique]


def _links(om, env, origin):
    out = {}
    for name, art in _articulations(om):
        poses = art.data.body_link_pose_w[env].detach().cpu().numpy().astype(np.float64)
        for j, link in enumerate(art.body_names):
            out[f"{name}/{link}"] = _pose(poses[j][:3], poses[j][3:], origin)
    return out


def _cameras(om, env, origin):
    from omegaconf import OmegaConf

    cm = om.camera_manager
    out = {}
    if cm is None:
        return out
    for i in range(cm.num_cams):
        name = cm.camera_names[env][i]
        cam = cm.cameras[env][i]
        position, orientation = cam.get_world_pose(camera_axes="ros")
        entry = {"pose": _pose(position, orientation, origin), "resolution": list(cam.get_resolution())}
        try:
            entry["K"] = np.asarray(_values(cam.get_intrinsics_matrix())).reshape(3, 3).tolist()
        except Exception:
            entry["K"] = None
        lens = cm.camera_config[name].camera.get("lens")
        entry["lens"] = None if lens is None else OmegaConf.to_container(lens, resolve=True)
        out[name] = entry
    return out


def _object_records(om, env):
    lm = om.env.scene_manager.layout_manager
    for kind in ("Rigid", "Dynamic", "Geometry", "Articulation"):
        records = lm.object_records_by_type.get(kind)
        if records is None:
            continue
        for rec in records.layout_records_by_env[env]:
            yield kind.lower(), rec, lm.get_scene_object(env, rec["inst_name"])


def _objects(om, env, origin):
    out = {}
    for kind, rec, obj in _object_records(om, env):
        entry = {"label": rec.get("label"), "type": kind}
        try:
            position, orientation = obj.get_world_pose()
            entry["pose"] = _pose(position, orientation, origin)
        except Exception as e:
            entry["error"] = repr(e)
        out[rec["inst_name"]] = entry
    return out


def _matrix(m):
    return [[float(m[j][i]) for j in range(4)] for i in range(4)]


def _shapes_of(root, stop_at_bodies):
    from pxr import Usd, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    out = {"collision": [], "visual": [], "shapes": []}
    it = iter(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
    for prim in it:
        if prim != root and stop_at_bodies and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            it.PruneChildren()
            continue
        if not prim.IsA(UsdGeom.Gprim):
            continue
        collision = False
        p = prim
        while p.IsValid():
            if p.HasAPI(UsdPhysics.CollisionAPI):
                collision = True
                break
            if p == root:
                break
            p = p.GetParent()
        imageable = UsdGeom.Imageable(prim)
        visual = imageable.ComputePurpose() in ("default", "render") and imageable.ComputeVisibility() != "invisible"
        m = cache.ComputeRelativeTransform(prim, root)[0]
        if prim.IsA(UsdGeom.Mesh):
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get() or []
            moved = [[round(float(c), 6) for c in m.Transform(q)] for q in points]
            if collision:
                out["collision"].extend(moved)
            if visual:
                out["visual"].extend(moved)
        else:
            attributes = {}
            for a in prim.GetAttributes():
                if a.GetName() in ("size", "radius", "height", "axis", "radiusTop", "radiusBottom"):
                    v = a.Get()
                    attributes[a.GetName()] = v if isinstance(v, str) else float(v)
            out["shapes"].append({"type": prim.GetTypeName(), "collision": collision, "visual": visual,
                                  "attributes": attributes, "transform": _matrix(m)})
    return out


def _geometry(om, env):
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    links = {}
    for name, art in _articulations(om):
        path = art.cfg.prim_path.replace("{ENV_REGEX_NS}", "/World/envs/env_.*").replace("env_.*", f"env_{env}")
        root = stage.GetPrimAtPath(path)
        bodies = {}
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies.setdefault(prim.GetName(), prim)
        for link in art.body_names:
            if link in bodies:
                links[f"{name}/{link}"] = _shapes_of(bodies[link], True)
    objects = {}
    for kind, rec, obj in _object_records(om, env):
        try:
            objects[rec["inst_name"]] = _shapes_of(stage.GetPrimAtPath(obj.prim_path), False)
        except Exception as e:
            objects[rec["inst_name"]] = {"error": repr(e)}
    return {"kind": "geometry", "links": links, "objects": objects}


def write(om, obs, env_idx_list, path):
    global _calls, _geometry_written
    if om.robot_manager is None:
        return
    env = list(env_idx_list)[0]
    origin = np.asarray(_values(om.robot_manager.scene.env_origins[env]))
    lines = []
    if not _geometry_written:
        lines.append(_geometry(om, env))
        _geometry_written = True
    _calls += 1
    lines.append({
        "kind": "observation",
        "call": _calls,
        "state": {k: _values(v) for k, v in obs[env]["state"].items()},
        "links": _links(om, env, origin),
        "cameras": _cameras(om, env, origin),
        "objects": _objects(om, env, origin),
    })
    with open(path, "a") as f:
        for line in lines:
            f.write(json.dumps(line, separators=(",", ":")) + "\n")
