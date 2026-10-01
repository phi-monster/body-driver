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

The first line of a run is {"kind": "geometry", "store": DIR, "links": {"robot0/link": KEY},
"joints": [{"parent": "robot0/link", "child": "robot0/link", "type": "PhysicsRevoluteJoint"}],
"objects": {inst_name: KEY}}; the joints give the tree of each robot (a finger is a link below the
tool link through a joint that moves). Each KEY names DIR/KEY.json, the geometry of that link or object in
its own frame, written once, the first time any run meets it (KEY is the SHA-1 of the content):
{"meshes": [{"collision": bool, "visual": bool, "vertices": n, "faces": f, "points": FILE,
"counts": FILE, "indices": FILE}], "shapes": [{"type", "attributes", "transform", "collision",
"visual"}]}. Each FILE, beside it in DIR, is raw little-endian: points x y z per vertex as float32,
counts one int32 per face (its corners), indices the faces' vertex numbers as int32, as authored.
A mesh is collision when it touches (PhysX collides it) and visual when the cameras see it; it can
be both. Shapes are the other primitives (spheres, capsules, ...) with their transform into that
frame (column vectors, translation in the last column). DIR is BD_TRUTH_GEOMETRY, or "geometry"
beside FILE. The scorer computes tips, support points, outlines and surface distances from these.
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


def _shapes_of(root, stop_at_bodies):
    from pxr import Usd, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    meshes, shapes = [], []
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
        # USD transforms row vectors: a point p maps to p m.
        m = np.array(cache.ComputeRelativeTransform(prim, root)[0], dtype=np.float64)
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            points = np.asarray(mesh.GetPointsAttr().Get() or [], dtype=np.float64).reshape(-1, 3)
            meshes.append((collision, visual, (points @ m[:3, :3] + m[3, :3]).astype("<f4"),
                           np.asarray(mesh.GetFaceVertexCountsAttr().Get() or [], dtype="<i4"),
                           np.asarray(mesh.GetFaceVertexIndicesAttr().Get() or [], dtype="<i4")))
        else:
            attributes = {}
            for a in prim.GetAttributes():
                if a.GetName() in ("size", "radius", "height", "axis", "radiusTop", "radiusBottom"):
                    v = a.Get()
                    if v is not None:
                        attributes[a.GetName()] = v if isinstance(v, str) else float(v)
            shapes.append({"type": prim.GetTypeName(), "collision": collision, "visual": visual,
                           "attributes": attributes, "transform": m.T.tolist()})
    return meshes, shapes


def _stored(meshes, shapes, store):
    """The key of a geometry in the store, writing it there the first time it is seen."""
    import hashlib
    import os

    shapes_text = json.dumps(shapes, separators=(",", ":"), sort_keys=True)
    h = hashlib.sha1(shapes_text.encode())
    for collision, visual, points, counts, indices in meshes:
        h.update(bytes([collision, visual]))
        for a in (points, counts, indices):
            h.update(len(a).to_bytes(8, "little"))
            h.update(a.tobytes())
    key = h.hexdigest()
    manifest = os.path.join(store, key + ".json")
    if not os.path.exists(manifest):
        os.makedirs(store, exist_ok=True)
        entries = []
        for i, (collision, visual, points, counts, indices) in enumerate(meshes):
            files = {}
            for part, a in (("points", points), ("counts", counts), ("indices", indices)):
                files[part] = f"{key}.{i}.{part}"
                a.tofile(os.path.join(store, files[part]))
            entries.append({"collision": collision, "visual": visual, "vertices": len(points), "faces": len(counts),
                            **files})
        with open(manifest + ".part", "w") as f:
            json.dump({"meshes": entries, "shapes": json.loads(shapes_text)}, f, separators=(",", ":"))
        os.replace(manifest + ".part", manifest)
    return key


def _geometry(om, env, store):
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    links = {}
    joints = []
    for name, art in _articulations(om):
        path = art.cfg.prim_path.replace("{ENV_REGEX_NS}", "/World/envs/env_.*").replace("env_.*", f"env_{env}")
        root = stage.GetPrimAtPath(path)
        bodies = {}
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies.setdefault(prim.GetName(), prim)
            if prim.IsA(UsdPhysics.Joint):
                ends = [UsdPhysics.Joint(prim).GetBody0Rel().GetTargets(), UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()]
                if ends[0] and ends[1]:
                    joints.append({"parent": f"{name}/{ends[0][0].name}", "child": f"{name}/{ends[1][0].name}",
                                   "type": prim.GetTypeName()})
        for link in art.body_names:
            if link in bodies:
                links[f"{name}/{link}"] = _stored(*_shapes_of(bodies[link], True), store)
    objects = {}
    for kind, rec, obj in _object_records(om, env):
        try:
            objects[rec["inst_name"]] = _stored(*_shapes_of(stage.GetPrimAtPath(obj.prim_path), False), store)
        except Exception as e:
            objects[rec["inst_name"]] = {"error": repr(e)}
    return {"kind": "geometry", "store": store, "links": links, "joints": joints, "objects": objects}


_failures = 0


def write(om, obs, env_idx_list, path):
    """Never lets the truth break the run: the first failure is printed whole, later ones counted."""
    global _failures
    try:
        _write(om, obs, env_idx_list, path)
    except Exception:
        _failures += 1
        if _failures == 1:
            import traceback
            print("[bd] truth hook failed; the run goes on without truth:", flush=True)
            traceback.print_exc()
        elif _failures % 1000 == 0:
            print(f"[bd] truth hook failed {_failures} times", flush=True)


def _write(om, obs, env_idx_list, path):
    global _calls, _geometry_written
    if om.robot_manager is None:
        return
    env = list(env_idx_list)[0]
    origin = np.asarray(_values(om.robot_manager.scene.env_origins[env]))
    lines = []
    if not _geometry_written:
        import os
        store = os.environ.get("BD_TRUTH_GEOMETRY") or os.path.join(os.path.dirname(os.path.abspath(path)), "geometry")
        lines.append(_geometry(om, env, store))
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
