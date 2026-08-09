#!/usr/bin/env python3
"""Re-generate the real-episode CSVs the probes are checked against.

WHY THESE FILES ARE COMMITTED.  `slow/examples/reach_on_real_data.rs` was written to take a CSV
path and no such CSV was ever committed, so the one real-data check in this repository could not be
re-run by anybody -- including its author.  A validation that cannot be re-run is a validation
nobody can contradict, which is the same failure shape as a guard that never fires.  So the inputs
live next to the code, small and in plain text, and this script says exactly where they came from.

Source (not part of this repository; an Isaac/PhysX rig log from the same project):
    universal-grounding/results/qcontact_aug2026/stair.npz
    universal-grounding/results/qcontact_aug2026/leg_{hover,near,press}.npz

Usage:  python3 realdata/extract.py [SRC_DIR]
"""
import os
import sys

import numpy as np

SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/Project/phi/research/universal-grounding/results/qcontact_aug2026"
)
HERE = os.path.dirname(os.path.abspath(__file__))


def contact():
    """A staircase of press depths against a measured touch height.

    `depth < 0` is clear of the surface, `depth > 0` is pressed into it, and PhysX reports the
    ground truth in `touch` -- 0 on every clear row, 1 on every pressed row.  That makes it a
    genuine two-class sample for `contact_threshold`, which refuses to fit a threshold from one
    class alone.
    """
    d = np.load(os.path.join(SRC, "stair.npz"))
    dep, f, t = d["depth"], d["f"], d["touch"]
    out = os.path.join(HERE, "contact_stair.csv")
    with open(out, "w") as fh:
        fh.write("depth_m,force_n,physx_touch\n")
        for a, b, c in zip(dep, f, t):
            fh.write("%.6f,%.6f,%d\n" % (a, b, int(c)))
    print("%s  n=%d  clear=%d pressed=%d" % (out, len(dep), int((dep < 0).sum()), int((dep > 0).sum())))


def reversals():
    """Per-step commanded vs achieved joint motion, over a sweep that reverses many times.

    `q_cmd` / `q_act` are joint POSITIONS, so the per-step deltas are the signed commanded and
    achieved steps `backlash` wants.  Three legs are kept on purpose: `hover` sweeps in free space,
    `near` and `press` sweep with the leg against a surface -- and a joint fighting a contact has no
    established free-motion delivery ratio, so those are the rows the probe must REFUSE.
    """
    out = os.path.join(HERE, "reversals.csv")
    with open(out, "w") as fh:
        fh.write("leg,joint,cmd_delta_rad,act_delta_rad\n")
        for leg in ("hover", "near", "press"):
            d = np.load(os.path.join(SRC, "leg_%s.npz" % leg))
            qc, qa = d["q_cmd"], d["q_act"]
            for j in range(7):  # arm joints only; 7,8 are the fingers
                dc, da = np.diff(qc[:, j]), np.diff(qa[:, j])
                for a, b in zip(dc, da):
                    if a == 0.0:
                        continue
                    fh.write("%s,%d,%.9f,%.9f\n" % (leg, j, a, b))
    print(out)


if __name__ == "__main__":
    contact()
    reversals()
