"""The scrambler: nothing of how this robot's numbers happen to be laid out reaches the driver.

It sits on the simulator's side, between the observation the simulator builds and the driver
(XPolicyLab/policy/l3_link/deploy.py calls it), so the driver cannot see or undo it. On every run:

- every key of the observation, at every depth, gets a name drawn at random, except the top-level
  "instruction" (docs/body-protocol.md, section 3), and keeps that name at every beat;
- every map of the observation lists its keys in an order drawn at random, the same at every beat;
- every group of readings (a number, or a one-dimensional numeric array) has its channels in an order
  drawn at random and each channel its own sign, scale and zero:
      reading' = sign * scale * reading + zero.
  A group and the echo of the command it was given share their last key, and with it the draw, so a
  command and its echo stay in one space;
- pictures and every other leaf go through unchanged under their drawn names.

The driver's action, {drawn command key: [values]}, goes back through the draw before the simulator
takes it (unscramble_action).

BD_SCRAMBLE=0 turns it off. BD_SCRAMBLE_SEED fixes the draw: a body file the driver kept under one
draw is that body under that draw only. Otherwise the seed comes from the operating system. Either
way it is printed on the simulator's log. BD_SCRAMBLE_MAP names a file the draw is written to once
the first observation has fixed it, for the scorer (driver/tools/score --scramble FILE); the driver
never sees it:
    {"seed": N, "names": {real key: drawn key}, "groups": {real last key:
     {"order": [...], "sign": [...], "scale": [...], "zero": [...]}}}
where channel i of the drawn group is channel order[i] of the real one.

A test of the draw itself (no simulator): python bd_scramble.py.
"""

import json
import os

import numpy as np

ON = os.environ.get("BD_SCRAMBLE", "1") != "0"

_rng = None
_seed = None
_names = {}    # real key -> drawn key
_back = {}     # drawn key -> real key
_orders = {}   # path of real keys to a map -> its real keys in the drawn order
_groups = {}   # real last key of a group -> its draw
_written = [False]


def _start():
    global _rng, _seed
    if _rng is None:
        given = os.environ.get("BD_SCRAMBLE_SEED")
        _seed = int(given) if given else int.from_bytes(os.urandom(4), "little")
        _rng = np.random.default_rng(_seed)
        print("[scramble] on, seed %d" % _seed, flush=True)


def _name(key):
    if key not in _names:
        while True:
            drawn = "k%08x" % int(_rng.integers(0, 2 ** 32))
            if drawn not in _back and drawn not in _names:
                break
        _names[key] = drawn
        _back[drawn] = key
    return _names[key]


def _is_group(v):
    if isinstance(v, (bool, np.bool_)):
        return False
    if isinstance(v, (int, float, np.integer, np.floating)):
        return True
    if isinstance(v, (list, tuple, np.ndarray)):
        try:
            a = np.asarray(v)
        except Exception:
            return False
        return a.ndim == 1 and a.size > 0 and a.dtype.kind in "fiu"
    return False


def _draw(key, n):
    g = _groups.get(key)
    if g is None:
        # Scales are powers of two, so a scale changes no bit of a reading but its exponent; zeros are drawn over
        # a turn of a joint read in radians, signs either way, the channels in any order.
        g = {"order": [int(i) for i in _rng.permutation(n)],
             "sign": [float(s) for s in _rng.choice([-1.0, 1.0], n)],
             "scale": [float(2.0 ** int(e)) for e in _rng.integers(-3, 4, n)],
             "zero": [float(z) for z in _rng.uniform(-np.pi, np.pi, n)]}
        _groups[key] = g
    elif len(g["order"]) != n:
        raise ValueError("[scramble] the group %s changed size from %d to %d" % (key, len(g["order"]), n))
    return g


def _forward(key, v):
    scalar = not isinstance(v, (list, tuple, np.ndarray))
    a = np.asarray(v, dtype=np.float64).ravel()
    g = _draw(key, a.size)
    y = np.asarray(g["sign"]) * np.asarray(g["scale"]) * a[np.asarray(g["order"])] + np.asarray(g["zero"])
    return float(y[0]) if scalar else y


def _map(d, path):
    if path not in _orders:
        keys = list(d.keys())
        _orders[path] = [keys[i] for i in _rng.permutation(len(keys))]
    order = _orders[path] + [k for k in d.keys() if k not in _orders[path]]
    out = {}
    for key in order:
        if key not in d:
            continue
        v = d[key]
        if path == () and key == "instruction":
            out[key] = v
        elif isinstance(v, dict):
            out[_name(key)] = _map(v, path + (key,))
        elif _is_group(v):
            out[_name(key)] = _forward(key, v)
        else:
            out[_name(key)] = v
    return out


def _write_map():
    where = os.environ.get("BD_SCRAMBLE_MAP")
    if where and not _written[0]:
        with open(where, "w") as f:
            json.dump({"seed": _seed, "names": _names, "groups": _groups}, f)
        _written[0] = True


def scramble_obs(obs):
    """The observation as the driver is given it."""
    if not ON or not isinstance(obs, dict):
        return obs
    _start()
    out = _map(obs, ())
    _write_map()
    return out


def unscramble_action(action):
    """The driver's action as the simulator takes it."""
    if not ON or not isinstance(action, dict):
        return action
    out = {}
    for drawn, values in action.items():
        if drawn not in _back:
            raise KeyError("[scramble] the driver commanded %r, a key no observation had" % (drawn,))
        key = _back[drawn]
        g = _groups[key]
        y = np.asarray(values, dtype=np.float64).ravel()
        if y.size != len(g["order"]):
            raise ValueError("[scramble] the driver commanded %d values for a group of %d" % (y.size, len(g["order"])))
        x = np.empty(y.size)
        x[np.asarray(g["order"])] = (y - np.asarray(g["zero"])) / (np.asarray(g["sign"]) * np.asarray(g["scale"]))
        out[key] = x.tolist()
    return out


def _test():
    global ON
    ON = True
    os.environ["BD_SCRAMBLE_SEED"] = "12345"
    rng = np.random.default_rng(7)
    obs = {"vision": {"cam_head": {"color": np.zeros((4, 5, 3), np.uint8), "shape": np.array([4, 5, 3])},
                      "cam_left_wrist": {"color": np.ones((4, 5, 3), np.uint8), "shape": np.array([4, 5, 3])}},
           "state": {"left_arm_joint_state": rng.normal(size=6), "left_ee_joint_state": np.array([1.0]),
                     "left_ee_pose": rng.normal(size=7)},
           "action": {"left_arm_joint_state": rng.normal(size=6), "left_ee_joint_state": np.array([0.0])},
           "env_idx": 0, "instruction": "pick up the cup", "data_format_version": "1"}
    s = scramble_obs(obs)
    assert "instruction" in s and s["instruction"] == "pick up the cup", "the instruction moved"
    assert "state" not in s and "vision" not in s, "a key kept its name"
    st = s[_names["state"]]
    assert set(st.keys()) == {_names[k] for k in obs["state"]}, "a group was lost"
    # Twice the same observation, the same draw.
    s2 = scramble_obs(obs)
    assert list(s2.keys()) == list(s.keys()), "the order changed between beats"
    # A group read back through the draw is the group, to the rounding of the zero.
    for key, v in obs["state"].items():
        y = st[_names[key]]
        back = unscramble_action({_names[key]: y})[key]
        assert np.allclose(back, v, rtol=0, atol=8 * np.finfo(float).eps * (np.pi + np.abs(v)).max()), key
    # The echo of a command shares the draw of the reading: a command sent as the driver read it comes back as sent.
    cmd = {_names["left_arm_joint_state"]: s[_names["action"]][_names["left_arm_joint_state"]]}
    assert np.allclose(unscramble_action(cmd)["left_arm_joint_state"], obs["action"]["left_arm_joint_state"]), "echo"
    # The pictures pass unchanged.
    cam = s[_names["vision"]][_names["cam_left_wrist"]][_names["color"]]
    assert cam is obs["vision"]["cam_left_wrist"]["color"], "a picture changed"
    # Something is actually scrambled: the channels' order or signs or scales differ from none.
    g = _groups["left_arm_joint_state"]
    assert g["order"] != list(range(6)) or any(x != 1.0 for x in g["sign"] + g["scale"]), "nothing scrambled"
    print("[scramble] test passed: seed %d, %d keys renamed, %d groups drawn" % (_seed, len(_names), len(_groups)))


if __name__ == "__main__":
    _test()
