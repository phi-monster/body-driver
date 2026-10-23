# truth.py: simulator-side rig code. The truth of a MuJoCo run for scoring, written beside the observation stream and
# never into it, in the form the driver's scorer reads (driver/tools/score.adb; the RoboDojo hook it was written for,
# harness/robodojo_truth/bd_truth.py, is in the git history): one JSON line per observation sent,
#   {"kind": "observation", "call": BEAT, "state": {group: [values as they went into the scramble]},
#    "links": {body: [x, y, z, qw, qx, qy, qz]},
#    "cameras": {name: {"pose": its optical frame (+x right, +y down, +z forward), "resolution": [w, h],
#                       "K": [[f, 0, w/2], [0, f, h/2], [0, 0, 1]], "lens": null}},
#    "objects": {name: {"label": name, "type": "rigid", "pose": [...]}}},
# the first line of a run {"kind": "geometry", "store": DIR, "links": {body: KEY}, "joints": [{"parent", "child",
# "type"}], "objects": {name: KEY}}: each KEY names DIR/KEY.json, a body's geometry in its own frame, written once
# (KEY is the SHA-1 of the content): {"meshes": [{"collision", "visual", "vertices", "faces", "points", "counts",
# "indices"}], "shapes": []}, each array file raw little-endian (points float32 x y z, counts and indices int32). The
# scorer reads the collision meshes' points (a finger's support point along a direction): a mesh geom gives its
# vertices and triangles; a primitive its points: a box its corners, a sphere, a capsule's ends and an ellipsoid an
# icosphere of 162 points (a support point off by under 1% of the radius), a cylinder its rims (64 points each). A body
# under a hinge or a slide hangs from its parent through a moving joint (PhysicsRevoluteJoint, PhysicsPrismaticJoint);
# a body with no joint is fixed to it; the boxes on the floor (bodies named prop*) are objects, not links.
import hashlib
import json
import os

import numpy as np
import mujoco


def _icosphere(levels):
    t = (1.0 + 5.0 ** 0.5) / 2.0
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6),
         (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10),
         (8, 6, 7), (9, 8, 1)]
    v = [np.array(p, float) / np.linalg.norm(p) for p in v]
    for _ in range(levels):
        mid, nf = {}, []
        def middle(a, b):
            key = (min(a, b), max(a, b))
            if key not in mid:
                p = v[a] + v[b]
                v.append(p / np.linalg.norm(p))
                mid[key] = len(v) - 1
            return mid[key]
        for a, b, c in f:
            ab, bc, ca = middle(a, b), middle(b, c), middle(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    return np.array(v)


SPHERE = _icosphere(2)   # 162 points on the unit sphere
RIM = np.array([[np.cos(a), np.sin(a)] for a in np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False)])


def _geom_points(m, g):
    """A geom's points and triangles in its body's frame, or None for what no finger is (a plane, a height field)."""
    t, size = m.geom_type[g], m.geom_size[g]
    faces = np.zeros((0, 3), np.int32)
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        k = m.geom_dataid[g]
        p = m.mesh_vert[m.mesh_vertadr[k]: m.mesh_vertadr[k] + m.mesh_vertnum[k]].astype(float)
        faces = m.mesh_face[m.mesh_faceadr[k]: m.mesh_faceadr[k] + m.mesh_facenum[k]].astype(np.int32)
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        p = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * size
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
        p = SPHERE * size[0]
    elif t == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        p = SPHERE * size
    elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
        top, bottom = SPHERE[SPHERE[:, 2] >= 0.0], SPHERE[SPHERE[:, 2] <= 0.0]
        p = np.vstack([top * size[0] + [0.0, 0.0, size[1]], bottom * size[0] - [0.0, 0.0, size[1]]])
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        p = np.vstack([np.column_stack([RIM * size[0], np.full(len(RIM), z)]) for z in (-size[1], size[1])])
    else:
        return None
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, m.geom_quat[g])
    return p @ R.reshape(3, 3).T + m.geom_pos[g], faces


def _stored(meshes, store):
    """The key of a body's geometry in the store, written there the first time it is seen."""
    h = hashlib.sha1()
    for collision, visual, points, faces in meshes:
        h.update(bytes([collision, visual]))
        for a in (points, faces):
            h.update(len(a).to_bytes(8, "little"))
            h.update(a.tobytes())
    key = h.hexdigest()
    manifest = os.path.join(store, key + ".json")
    if not os.path.exists(manifest):
        os.makedirs(store, exist_ok=True)
        entries = []
        for i, (collision, visual, points, faces) in enumerate(meshes):
            files = {part: "%s.%d.%s" % (key, i, part) for part in ("points", "counts", "indices")}
            points.astype("<f4").tofile(os.path.join(store, files["points"]))
            np.full(len(faces), 3, "<i4").tofile(os.path.join(store, files["counts"]))
            faces.astype("<i4").ravel().tofile(os.path.join(store, files["indices"]))
            entries.append({"collision": collision, "visual": visual, "vertices": len(points), "faces": len(faces),
                            **files})
        with open(manifest + ".part", "w") as f:
            json.dump({"meshes": entries, "shapes": []}, f, separators=(",", ":"))
        os.replace(manifest + ".part", manifest)
    return key


def _pose(p, q):
    return [round(float(x), 7) for x in list(p) + list(q)]


class Truth:
    def __init__(self, m, d, path, cameras):
        self.m, self.d, self.path, self.cameras = m, d, path, cameras
        self.store = os.environ.get("BD_TRUTH_GEOMETRY") or os.path.join(os.path.dirname(path) or ".", "geometry")
        self.names = [m.body(b).name or "body%d" % b for b in range(m.nbody)]
        self.objects = [b for b in range(1, m.nbody) if self.names[b].startswith("prop")]
        self.links = [b for b in range(1, m.nbody) if b not in self.objects]
        self.file = open(path, "w")
        self.file.write(json.dumps(self._geometry(), separators=(",", ":")) + "\n")

    def _body_meshes(self, b):
        m = self.m
        out = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b:
                continue
            got = _geom_points(m, g)
            if got is None:
                continue
            collision = bool(m.geom_contype[g] or m.geom_conaffinity[g])
            visual = bool(m.geom_group[g] <= 2 and m.geom_rgba[g][3] > 0.0)   # what MuJoCo draws by default
            out.append((collision, visual, got[0].astype("<f4"), got[1]))
        return out

    def _geometry(self):
        m = self.m
        joints = []
        for b in self.links:
            kinds = {m.jnt_type[m.body_jntadr[b] + k] for k in range(m.body_jntnum[b])}
            kind = ("PhysicsFixedJoint" if not kinds else
                    "PhysicsRevoluteJoint" if mujoco.mjtJoint.mjJNT_HINGE in kinds else
                    "PhysicsPrismaticJoint" if mujoco.mjtJoint.mjJNT_SLIDE in kinds else "PhysicsFreeJoint")
            joints.append({"parent": self.names[m.body_parentid[b]], "child": self.names[b], "type": kind})
        return {"kind": "geometry", "store": self.store,
                "links": {self.names[b]: _stored(self._body_meshes(b), self.store) for b in self.links},
                "joints": joints,
                "objects": {self.names[b]: _stored(self._body_meshes(b), self.store) for b in self.objects}}

    def write(self, beat, state, width, height):
        m, d = self.m, self.d
        cams = {}
        for c in self.cameras:
            R = d.cam_xmat[c["id"]].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])   # MuJoCo's camera looks down -z, y up
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, R.ravel())
            f = 0.5 * height / np.tan(np.radians(m.cam_fovy[c["id"]]) / 2.0)
            cams[c["name"]] = {"pose": _pose(d.cam_xpos[c["id"]], q), "resolution": [width, height],
                               "K": [[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]], "lens": None}
        line = {"kind": "observation", "call": beat,
                "state": {k: np.asarray(v).tolist() for k, v in state.items()},
                "links": {self.names[b]: _pose(d.xpos[b], d.xquat[b]) for b in self.links},
                "cameras": cams,
                "objects": {self.names[b]: {"label": self.names[b], "type": "rigid", "pose": _pose(d.xpos[b], d.xquat[b])}
                            for b in self.objects}}
        self.file.write(json.dumps(line, separators=(",", ":")) + "\n")

    def close(self):
        self.file.close()
