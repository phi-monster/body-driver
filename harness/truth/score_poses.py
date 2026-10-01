"""Score a replay's estimates against simulator truth, whatever model produced them.

Inputs
  --est    the JSON lines written by `replay RECORDING --estimates FILE`: per beat after boot,
           each arm's tool pose and each eye's pose (4 x 4, world frame, the driver's own unit);
           the last line holds, per eye, the lines of sight of a pixel grid in the eye's frame
  --rec    the same recording (uncompressed BDWIRE1): the robot-reported end-effector poses in
           it are the truth for the arms (the driver never reads them)
  --probe  optional: a recording of the same rig with intrinsic matrices in the observation
           (a "truth probe"), the truth for the lines of sight

Arms: the estimated tool poses T_est(b) are related to the reported poses T_true(b) by an
unknown similarity S (the world frames and units differ) and a constant offset X (the tool
frame the driver chose versus the reported one): T_true = S T_est X. Both are fitted on every
other distinct pose and the errors are reported on the poses not used for fitting. Eyes carried
by an arm are scored the same way against that arm. Lines of sight are compared, in each eye's
own frame, with the pinhole rays of the true intrinsics.
"""

import argparse
import json
import math
import struct

import msgpack
import numpy as np
from scipy.optimize import least_squares


def records(path):
    with open(path, "rb") as f:
        assert f.read(8) == b"BDWIRE1\n"
        while True:
            head = f.read(13)
            if len(head) < 13:
                return
            kind, _, length = struct.unpack("<BQI", head)
            yield chr(kind), f.read(length)


def normalized(tree):
    """msgpack-numpy writes its keys as bytes; make every map key text, keep values as they are."""
    if isinstance(tree, dict):
        return {(k.decode() if isinstance(k, bytes) else k): normalized(v) for k, v in tree.items()}
    if isinstance(tree, list):
        return [normalized(v) for v in tree]
    return tree


def flatten(tree, prefix=()):
    if isinstance(tree, dict) and not tree.get("nd"):
        for k, v in tree.items():
            yield from flatten(v, prefix + (str(k),))
    else:
        yield prefix, tree


def nd_values(v):
    if isinstance(v, dict) and v.get("nd"):
        return np.frombuffer(v["data"], dtype=np.dtype(v["type"])).astype(float).reshape(v["shape"])
    return np.asarray(v, dtype=float)


def observations(path):
    """Yield the observation map of every robot message that carries one, in order."""
    for kind, payload in records(path):
        if kind != "R":
            continue
        m = normalized(msgpack.unpackb(payload, raw=False, strict_map_key=False))
        obs = (m.get("payload") or {}).get("obs") or (m.get("payload") or {}).get("observation")
        if isinstance(obs, dict):
            yield obs


def quat_matrix(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotvec(w):
    a = np.linalg.norm(w)
    if a < 1e-12:
        return np.eye(3)
    k = w / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def angle(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def fit(est, true):
    """Similarity S and right offset X with true ~ S est X; est, true: lists of (R, p)."""
    def unpack(x):
        return math.exp(x[0]), rotvec(x[1:4]), x[4:7], rotvec(x[7:10]), x[10:13]

    def residuals(x):
        s, Rs, ts, Rx, tx = unpack(x)
        r = []
        for (Re, pe), (Rt, pt) in zip(est, true):
            R = Rs @ Re @ Rx
            p = s * (Rs @ (Re @ tx + pe)) + ts
            r.extend(p - pt)
            r.extend(0.1 * (R - Rt).ravel())
        return np.array(r)

    scale0 = (np.std([p for _, p in true], axis=0).sum() / max(1e-9, np.std([p for _, p in est], axis=0).sum()))
    best = None
    for start in range(8):
        x0 = np.zeros(13)
        x0[0] = math.log(max(scale0, 1e-6))
        x0[1:4] = np.random.default_rng(start).normal(0, 1.0, 3) if start else 0.0
        r = least_squares(residuals, x0)
        if best is None or r.cost < best.cost:
            best = r
    return unpack(best.x)


def errors(S, est, true):
    s, Rs, ts, Rx, tx = S
    pos, rot = [], []
    for (Re, pe), (Rt, pt) in zip(est, true):
        pos.append(1000 * np.linalg.norm(s * (Rs @ (Re @ tx + pe)) + ts - pt))
        rot.append(angle((Rs @ Re @ Rx).T @ Rt))
    return np.array(pos), np.array(rot)


def describe(name, pos, rot):
    print(f"  {name}: {len(pos)} test poses, position median {np.median(pos):.2f} mm, 90% {np.quantile(pos, 0.9):.2f}"
          f" mm, max {pos.max():.2f} mm; rotation median {np.median(rot):.3f} deg, max {rot.max():.3f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--est", required=True)
    ap.add_argument("--rec", required=True)
    ap.add_argument("--probe")
    a = ap.parse_args()

    lines = [json.loads(l) for l in open(a.est) if l.strip()]
    beats = {l["beat"]: l for l in lines if "beat" in l}
    rays = next((l["rays"] for l in lines if "rays" in l), None)

    truth = {}
    for beat, obs in enumerate(observations(a.rec)):
        poses = [nd_values(v) for k, v in flatten(obs) if k and k[-1].endswith("ee_pose")]
        truth[beat] = poses
    common = sorted(set(beats) & set(truth))
    print(f"beats with estimates and truth: {len(common)}")
    if not common:
        return
    n_true = len(truth[common[0]])
    n_tools = len(beats[common[0]]["tools"])
    n_eyes = len(beats[common[0]]["eyes"])

    def as_pose(m):
        m = np.asarray(m, dtype=float).reshape(4, 4)
        return m[:3, :3], m[:3, 3]

    for t in range(n_true):
        tp = [(quat_matrix(truth[b][t][3:7]), truth[b][t][:3]) for b in common]
        key = np.round(np.array([p for _, p in tp]) * 1e4).astype(np.int64)
        _, first = np.unique(key, axis=0, return_index=True)
        first = np.sort(first)
        train, test = first[0::2], first[1::2]
        if len(train) < 5 or len(test) < 1:
            print(f"true arm {t}: too few distinct poses ({len(first)})")
            continue
        candidates = [("tool", i) for i in range(n_tools)] + [("eye", e) for e in range(n_eyes)]
        results = []
        for what, i in candidates:
            ep = [as_pose(beats[b]["tools" if what == "tool" else "eyes"][i]) for b in common]
            S = fit([ep[j] for j in train], [tp[j] for j in train])
            pos, rot = errors(S, [ep[j] for j in test], [tp[j] for j in test])
            results.append((np.median(pos), what, i, pos, rot, S[0]))
        results.sort(key=lambda r: r[0])
        print(f"true arm {t} ({len(first)} distinct poses):")
        for med, what, i, pos, rot, s in results:
            if med < 50:
                describe(f"{what} {i + 1} (scale {s:.5f} m per driver unit)", pos, rot)

    if rays and a.probe:
        probe = next(observations(a.probe))
        leaves = list(flatten(probe))
        intrinsics = [nd_values(v) for k, v in leaves if k and k[-1] == "intrinsic_matrix"]
        sizes = [nd_values(v).shape[:2] for k, v in leaves
                 if isinstance(v, dict) and v.get("nd") and len(v["shape"]) == 3 and v["shape"][2] == 3]
        print(f"lines of sight against {len(intrinsics)} true intrinsic matrices:")
        for e, grid in enumerate(rays):
            if e >= len(intrinsics) or e >= len(sizes):
                break
            K = intrinsics[e].reshape(3, 3)
            h, w = sizes[e]
            errs = []
            for u, v, dx, dy, dz in grid:
                #  The render is a symmetric frustum: the optical axis meets the image at its
                #  centre, (w/2, h/2) in the driver's corner-origin pixel convention. Only the
                #  focal lengths are taken from K, whose principal-point convention is unknown.
                d_true = np.array([(u - w / 2) / K[0, 0], (v - h / 2) / K[1, 1], 1.0])
                d_true /= np.linalg.norm(d_true)
                d = np.array([dx, dy, dz]) / np.linalg.norm([dx, dy, dz])
                errs.append(math.degrees(math.acos(max(-1.0, min(1.0, d @ d_true)))))
            errs = np.array(errs)
            print(f"  eye {e + 1}: median {np.median(errs):.4f} deg, max {errs.max():.4f} deg"
                  f" (about {math.radians(np.median(errs)) * K[0, 0]:.2f} px at f = {K[0, 0]:.1f} px)")


if __name__ == "__main__":
    main()
