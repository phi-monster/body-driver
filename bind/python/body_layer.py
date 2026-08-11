"""body layer — Python binding over the stable C ABI.  ctypes only, no dependencies.

WHY THIS FILE EXISTS.  On 2026-08-09 a grep answered the question "who reads the body layer?" with
**zero**.  The running teacher is Python; this layer is Rust behind a C ABI; and the distance
between those two facts was the entire reason a library with 40 passing tests was not connected to
anything.  A C ABI that half the field can consume in principle and nobody consumes in practice is
a header file.

WHAT IT DELIBERATELY DOES NOT DO.  It does not re-declare the enums.  Every name table below is
built at import time by asking the library (`bl_quantity_str`, `bl_reason_str`, `bl_need_str`),
because a second copy of an enum in another language is a copy that drifts, and the drift is silent:
both sides keep returning success while meaning different things.  `Quantity.NAMES` is therefore
whatever the linked library says it is, and cannot disagree with it.

Usage:

    from body_layer import BodyLayer, Quantity
    bl = BodyLayer()                      # loads the cdylib, checks the ABI version
    body = bl.new_body()
    for q, need in body.plan(now_ns):     # what this machine still owes itself
        m = run_my_probe(q)               # your rig's probing code
        body.measure(m)                   # refused unless it carries its own provenance
"""

import ctypes
import os
import sys

ABI_VERSION = 2
MAX_DIM = 48
MAX_DEPS = 8
MAX_JOINTS = 16
REASON_LEN = 96

OK, REFUSE, EINVAL, EVERSION, ENOSPACE, EINTERNAL = range(6)
STATUS_NAMES = {
    OK: "ok",
    REFUSE: "refuse",
    EINVAL: "einval",
    EVERSION: "eversion",
    ENOSPACE: "enospace",
    EINTERNAL: "einternal",
}

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))


def _default_lib():
    stem = "libbody_layer"
    ext = ".dylib" if sys.platform == "darwin" else ".so"
    for profile in ("release", "debug"):
        p = os.path.join(_ROOT, "slow", "target", profile, stem + ext)
        if os.path.exists(p):
            return p
    raise OSError(
        "no built cdylib found under slow/target/{release,debug}. Build it:\n"
        "  cd slow && cargo build --release"
    )


class Measurement(ctypes.Structure):
    """Mirror of `bl_measurement`.  Field order and types match the header exactly.

    🔴 There is no constructor that fills in a default uncertainty, a default validity window, or a
    passing self-test.  A measurement that cannot say how well it knows itself is exactly the bare
    f64 this layer exists to abolish, and `bl_measure` refuses it.
    """

    _fields_ = [
        ("quantity", ctypes.c_uint32),
        ("dim", ctypes.c_uint32),
        # bl_axis_kind per axis: 0 interval, 1 categorical, 2 unmeasured. Zero is what a
        # zero-initialised struct gets, which is exactly the behaviour from before this field.
        ("axis_kind", ctypes.c_uint32 * MAX_DIM),
        ("value", ctypes.c_double * MAX_DIM),
        ("uncertainty", ctypes.c_double * MAX_DIM),
        ("valid_lo", ctypes.c_double * MAX_DIM),
        ("valid_hi", ctypes.c_double * MAX_DIM),
        ("measured_at_ns", ctypes.c_uint64),
        ("valid_for_ns", ctypes.c_uint64),
        ("n_deps", ctypes.c_uint32),
        ("deps", ctypes.c_uint32 * MAX_DEPS),
        ("dep_epoch", ctypes.c_uint64 * MAX_DEPS),
        ("epoch", ctypes.c_uint64),
        ("selftest_passed", ctypes.c_uint32),
        ("prev_epoch", ctypes.c_uint64),
    ]


class WorldRef(ctypes.Structure):
    """Mirror of `bl_world_ref` — the entire vocabulary any VLM or world model may use.

    Look at what is absent: no z, no pose, no object id, no task id.  A pointer that cannot express
    a pose cannot leak one, and that absence is the invariant, not an oversight.
    """

    _fields_ = [
        ("u", ctypes.c_double),
        ("v", ctypes.c_double),
        ("extent", ctypes.c_double),
        ("verb", ctypes.c_uint32),
        ("manner", ctypes.c_double),
        ("frame_id", ctypes.c_uint64),
    ]


class PolicyOut(ctypes.Structure):
    """Mirror of `bl_policy_out`.  `dir` is a UNIT vector: how far to travel is the body's business.

    Magnitude is derivable from proprioception, so it is a shortcut; direction is not.  The library
    rejects a non-unit vector rather than normalising it, because silently normalising would let a
    policy encode distance in the magnitude and put the shortcut straight back.
    """

    _fields_ = [
        ("dir", ctypes.c_double * 3),
        ("drot", ctypes.c_double * 3),
        ("grip", ctypes.c_double),
        ("base", ctypes.c_double * 3),
    ]


class Spec(ctypes.Structure):
    """Mirror of `bl_spec`.

    ⚠️ `step_m` and `damping` are the two hand-set body constants this layer still carries; they are
    row entries in the ledger (`BodyLayer.debt()`), not a clean part of the design.
    """

    _fields_ = [
        ("step_m", ctypes.c_double),
        ("period_ms", ctypes.c_uint32),
        ("damping", ctypes.c_double),
        ("n_joints", ctypes.c_uint32),
    ]


X_MOVE, X_REFUSED, X_HALTED, X_BAD_INTENT = range(4)


class Quantity:
    """Names filled in at import from the linked library.  Never re-typed here."""

    NAMES = {}
    IDS = {}


class Reason:
    NAMES = {}


class Need:
    NAMES = {}


class BodyLayer:
    """The loaded library."""

    def __init__(self, path=None):
        self.path = path or _default_lib()
        self.lib = ctypes.CDLL(self.path)
        L = self.lib
        L.bl_sizeof_body.restype = ctypes.c_size_t
        L.bl_alignof_body.restype = ctypes.c_size_t
        L.bl_init.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint32]
        L.bl_init.restype = ctypes.c_uint32
        L.bl_close.argtypes = [ctypes.c_void_p]
        # the thin OS's memory -- the same caller-owned-storage discipline as the body
        u32p, u64p = ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint64)
        # probes -- the MEASURING half. Unreachable from any language but Rust until 2026-08-11.
        dp, mp = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(Measurement)
        L.bl_declined_str.restype = ctypes.c_char_p
        L.bl_solve.argtypes = [ctypes.c_void_p, ctypes.POINTER(Spec), dp, dp, u32p]
        L.bl_solve.restype = ctypes.c_uint32
        L.bl_probe_image_jacobian.argtypes = [dp, dp, ctypes.POINTER(ctypes.c_uint64),
                                              ctypes.c_size_t, ctypes.c_size_t, ctypes.c_uint64,
                                              ctypes.c_double, mp, u32p]
        L.bl_probe_image_jacobian.restype = ctypes.c_uint32
        L.bl_probe_hand_pixel.argtypes = [dp, dp, dp, dp, ctypes.POINTER(ctypes.c_uint32), dp,
                                          ctypes.c_size_t, ctypes.c_uint64, ctypes.c_uint64,
                                          ctypes.c_uint64, ctypes.c_uint64, mp, u32p]
        L.bl_probe_hand_pixel.restype = ctypes.c_uint32
        L.bl_probe_arm_weight.argtypes = [dp, dp, ctypes.c_size_t, ctypes.c_uint64, mp, u32p]
        L.bl_probe_arm_weight.restype = ctypes.c_uint32
        L.bl_probe_contact_threshold.argtypes = [dp, ctypes.c_size_t, dp, ctypes.c_size_t,
                                                 ctypes.c_uint32, ctypes.c_uint64, ctypes.c_uint64,
                                                 mp, u32p]
        L.bl_probe_contact_threshold.restype = ctypes.c_uint32
        L.bl_probe_step_delivery.argtypes = [dp, dp, ctypes.c_size_t, ctypes.c_uint64, mp, u32p]
        L.bl_probe_step_delivery.restype = ctypes.c_uint32
        L.bl_probe_reach.argtypes = [dp, ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t,
                                     ctypes.c_uint64, mp, u32p]
        L.bl_probe_reach.restype = ctypes.c_uint32
        L.bl_probe_latency.argtypes = [ctypes.c_int64, ctypes.c_uint32, ctypes.c_uint64, mp, u32p]
        L.bl_probe_latency.restype = ctypes.c_uint32
        L.bl_probe_backlash.argtypes = [dp, dp, ctypes.c_size_t, ctypes.c_uint64, mp, u32p]
        L.bl_probe_backlash.restype = ctypes.c_uint32
        L.bl_probe_gripper_span.argtypes = [dp, dp, ctypes.c_size_t, ctypes.c_double,
                                            ctypes.c_double, ctypes.c_uint64, ctypes.c_uint64,
                                            mp, u32p]
        L.bl_probe_gripper_span.restype = ctypes.c_uint32
        L.bl_probe_tool_offset.argtypes = [dp, dp, dp, ctypes.c_size_t, ctypes.c_double,
                                           ctypes.c_double, ctypes.c_uint64, ctypes.c_uint64,
                                           mp, u32p]
        L.bl_probe_tool_offset.restype = ctypes.c_uint32
        L.bl_predict_horizon.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double,
                                         u32p, u32p]
        L.bl_predict_horizon.restype = ctypes.c_uint32
        L.bl_predict_admit.argtypes = [ctypes.POINTER(Predicted), ctypes.c_uint32,
                                       ctypes.c_double, ctypes.c_uint32, u32p, ctypes.c_char_p]
        L.bl_predict_admit.restype = ctypes.c_uint32
        L.bl_predict_admit_chase.argtypes = [ctypes.c_void_p, ctypes.POINTER(Predicted),
                                            ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                            ctypes.c_uint32, u32p, ctypes.c_char_p]
        L.bl_predict_admit_chase.restype = ctypes.c_uint32
        L.bl_memory_sizeof.restype = ctypes.c_size_t
        L.bl_memory_alignof.restype = ctypes.c_size_t
        L.bl_memory_init.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint32,
                                     ctypes.c_uint32]
        L.bl_memory_init.restype = ctypes.c_uint32
        L.bl_memory_declare.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32, u32p]
        L.bl_memory_declare.restype = ctypes.c_uint32
        L.bl_memory_observed.argtypes = [ctypes.c_void_p]
        L.bl_memory_observed.restype = ctypes.c_uint32
        L.bl_memory_write.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p,
                                      ctypes.c_uint32, u32p]
        L.bl_memory_write.restype = ctypes.c_uint32
        L.bl_memory_get.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p,
                                    ctypes.c_size_t, u32p]
        L.bl_memory_get.restype = ctypes.c_uint32
        L.bl_memory_event.argtypes = [ctypes.c_void_p, ctypes.c_uint32, u32p]
        L.bl_memory_event.restype = ctypes.c_uint32
        L.bl_memory_stats.argtypes = [ctypes.c_void_p, u64p, u64p, u64p, u32p, u32p]
        L.bl_memory_stats.restype = ctypes.c_uint32
        L.bl_place_matches.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_char_p,
                                       ctypes.c_double, u32p]
        L.bl_place_matches.restype = ctypes.c_uint32
        L.bl_measure.argtypes = [ctypes.c_void_p, ctypes.POINTER(Measurement)]
        L.bl_measure.restype = ctypes.c_uint32
        L.bl_get.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.POINTER(Measurement)]
        L.bl_get.restype = ctypes.c_uint32
        L.bl_admit.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(WorldRef),
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_char_p,
        ]
        L.bl_admit.restype = ctypes.c_uint32
        L.bl_selftest.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint64)]
        L.bl_selftest.restype = ctypes.c_uint32
        L.bl_execute.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(PolicyOut),
            ctypes.POINTER(Spec),
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_uint32),
        ]
        L.bl_execute.restype = ctypes.c_uint32
        L.bl_save_max_bytes.restype = ctypes.c_size_t
        L.bl_save.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_size_t),
        ]
        L.bl_save.restype = ctypes.c_uint32
        L.bl_load.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t]
        L.bl_load.restype = ctypes.c_uint32
        L.bl_measure_plan.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_size_t),
        ]
        L.bl_measure_plan.restype = ctypes.c_uint32
        L.bl_debt_total.restype = ctypes.c_uint32
        L.bl_debt_outstanding.restype = ctypes.c_uint32
        L.bl_debt_line.argtypes = [ctypes.c_uint32, ctypes.c_char_p, ctypes.c_size_t]
        L.bl_debt_line.restype = ctypes.c_uint32
        for fn in ("bl_reason_str", "bl_quantity_str", "bl_need_str"):
            getattr(L, fn).argtypes = [ctypes.c_uint32]
            getattr(L, fn).restype = ctypes.c_char_p
        self._fill_names()

    def _fill_names(self):
        """Ask the library what its enums are.  Walking until 'unknown' means adding a quantity in
        Rust needs no edit here, and — more importantly — REMOVING one cannot leave a stale Python
        name pointing at a different id."""
        for table, fn in (
            (Quantity.NAMES, self.lib.bl_quantity_str),
            (Reason.NAMES, self.lib.bl_reason_str),
            (Need.NAMES, self.lib.bl_need_str),
        ):
            i = 0
            while True:
                name = fn(i).decode()
                if name == "unknown":
                    break
                table[i] = name
                i += 1
                if i > 256:  # a runaway would mean bl_*_str never says 'unknown'
                    raise RuntimeError("enum walk did not terminate")
        Quantity.IDS = {v: k for k, v in Quantity.NAMES.items()}

    def new_body(self):
        return Body(self)

    @staticmethod
    def _arr(xs):
        a = (ctypes.c_double * len(xs))()
        for i, x in enumerate(xs):
            a[i] = float(x)
        return a

    def _probe(self, call):
        """-> (Measurement, None) or (None, why). A DECLINE is an answer: the probe ran and could
        not say anything, which is different from a gate refusing to admit what it did say."""
        out, why = Measurement(), ctypes.c_uint32(0)
        st = call(ctypes.byref(out), ctypes.byref(why))
        if st == OK:
            return out, None
        return None, self.lib.bl_declined_str(why.value).decode()

    def solve(self, body, dir_img, step_m, n_axes, period_ms=40, damping=0.01):
        """-> (deltas, reason). Image direction in, actuator deltas out, scaled to the body's step.

        The deltas are in whatever axes the Jacobian was probed over -- that is the whole of the
        actuator-agnostic property.
        """
        sp = Spec(step_m=float(step_m), period_ms=int(period_ms), damping=float(damping),
                  n_joints=int(n_axes))
        d = (ctypes.c_double * 3)(*[float(x) for x in dir_img])
        out = (ctypes.c_double * 16)()
        why = ctypes.c_uint32(0)
        st = self.lib.bl_solve(body._ptr, ctypes.byref(sp), d, out, ctypes.byref(why))
        if st != OK:
            return None, Reason.NAMES.get(why.value, "unknown")
        return [out[i] for i in range(int(n_axes))], "none"

    def probe_image_jacobian(self, cmds, uvs, at_ns, n_axes, now_ns=0, min_response_px=1e-4):
        """🔴 How this body's own commands move its own image. `cmds` is [n_samples][n_axes].

        `n_axes` IS the actuator: probe with joints and `solve` returns joint deltas; probe with
        end-effector axes and it returns end-effector deltas. The one rule is that the probe and
        the executor use the SAME axes.
        """
        flat = [x for row in cmds for x in row]
        uvf = [x for p in uvs for x in p]
        c, u = self._arr(flat), self._arr(uvf)
        t = (ctypes.c_uint64 * len(at_ns))(*[int(x) for x in at_ns])
        return self._probe(lambda o, w: self.lib.bl_probe_image_jacobian(
            c, u, t, len(at_ns), int(n_axes), now_ns, float(min_response_px), o, w))

    def probe_hand_pixel(self, cands, now_ns=0, epoch=1, prev_epoch=0, jac_epoch=0):
        """`cands`: list of dicts with u, v, gain, rigidity, pixels, spread.

        Refuses when the field is not separable -- the rule that exists because a selection tuned
        for "the hand vs its shadow" silently picked the ELBOW, 167 px from the truth, while
        reporting 0.04-9.3 px of error.
        """
        f = lambda k: self._arr([c[k] for c in cands])  # noqa: E731
        u, v, g, r, sp = f("u"), f("v"), f("gain"), f("rigidity"), f("spread")
        px = (ctypes.c_uint32 * len(cands))(*[int(c["pixels"]) for c in cands])
        return self._probe(lambda o, w: self.lib.bl_probe_hand_pixel(
            u, v, g, r, px, sp, len(cands), now_ns, epoch, prev_epoch, jac_epoch, o, w))

    def probe_arm_weight(self, joint_angle, hold_torque, now_ns=0):
        """🔴 What holding still against gravity costs. Its valid range is THE POSES ACTUALLY
        VISITED -- a gravity self-calibration here once had its entire residual in interpolation
        between them, so asking outside is where the number stops meaning anything."""
        a, t = self._arr(joint_angle), self._arr(hold_torque)
        return self._probe(lambda o, w: self.lib.bl_probe_arm_weight(a, t, len(a), now_ns, o, w))

    def probe_contact_threshold(self, free, touching, polarity, now_ns=0, arm_weight_epoch=0):
        """`polarity` 0 = higher on contact (force/current/torque), 1 = LOWER (delivered motion).

        🔴 Required, no default. This binding kept the OLD four-argument call for hours after the
        ABI grew this parameter -- `python_check.sh` passed the whole time because it never
        exercises this function, so the mirror drifted exactly the way the C client exists to
        prevent on the other edge. Every argument after the missing one was silently shifted.
        """
        f, t = self._arr(free), self._arr(touching)
        return self._probe(lambda o, w: self.lib.bl_probe_contact_threshold(
            f, len(f), t, len(t), int(polarity), now_ns, arm_weight_epoch, o, w))

    def probe_step_delivery(self, commanded, achieved, now_ns=0):
        c, a = self._arr(commanded), self._arr(achieved)
        return self._probe(lambda o, w: self.lib.bl_probe_step_delivery(c, a, len(c), now_ns, o, w))

    def probe_reach(self, radius, attained, now_ns=0):
        r = self._arr(radius)
        at = (ctypes.c_uint32 * len(attained))(*[1 if x else 0 for x in attained])
        return self._probe(lambda o, w: self.lib.bl_probe_reach(r, at, len(r), now_ns, o, w))

    def probe_latency(self, first_motion_step, steps_observed, now_ns=0):
        """`first_motion_step < 0` = nothing moved. That is a refusal, not a latency."""
        return self._probe(lambda o, w: self.lib.bl_probe_latency(
            int(first_motion_step), int(steps_observed), now_ns, o, w))

    def probe_backlash(self, commanded, observed, now_ns=0):
        c, ob = self._arr(commanded), self._arr(observed)
        return self._probe(lambda o, w: self.lib.bl_probe_backlash(c, ob, len(c), now_ns, o, w))

    def probe_gripper_span(self, opening, separation, units_per_m, units_per_m_sigma=0.0,
                           now_ns=0, jac_epoch=0):
        op, sep = self._arr(opening), self._arr(separation)
        return self._probe(lambda o, w: self.lib.bl_probe_gripper_span(
            op, sep, len(op), units_per_m, units_per_m_sigma, now_ns, jac_epoch, o, w))

    def probe_tool_offset(self, wrist_angle, u, v, units_per_m, units_per_m_sigma=0.0,
                          now_ns=0, jac_epoch=0):
        a, uu, vv = self._arr(wrist_angle), self._arr(u), self._arr(v)
        return self._probe(lambda o, w: self.lib.bl_probe_tool_offset(
            a, uu, vv, len(a), units_per_m, units_per_m_sigma, now_ns, jac_epoch, o, w))

    def predict_horizon(self, body, distance_m, tol_frac=0.01):
        """-> (periods, reason).  How long THIS body is blind while it covers that distance.

        From the body's own measured delivery, never from a guess — which is the half the conveyor
        loop never asked for.
        """
        out, why = ctypes.c_uint32(0), ctypes.c_uint32(0)
        st = self.lib.bl_predict_horizon(body._ptr, float(distance_m), float(tol_frac),
                                         ctypes.byref(out), ctypes.byref(why))
        if st != OK:
            return None, Reason.NAMES.get(why.value, "unknown")
        return out.value, "none"

    def predict_admit(self, predicted, need_periods, tol_uv=None):
        """-> (ok, reason, detail). The gate on its own, when the caller already knows the horizon.

        `ok` True with reason `no_evidence` is the THIRD rung: proceed, and nothing has validated
        this model at this horizon.
        """
        why = ctypes.c_uint32(0)
        detail = ctypes.create_string_buffer(REASON_LEN)
        st = self.lib.bl_predict_admit(
            ctypes.byref(predicted), ctypes.c_uint32(int(need_periods)),
            ctypes.c_double(0.0 if tol_uv is None else float(tol_uv)),
            ctypes.c_uint32(0 if tol_uv is None else 1), ctypes.byref(why), detail)
        return st == OK, Reason.NAMES.get(why.value, "unknown"), detail.value.decode()

    def predict_admit_chase(self, body, predicted, distance_m, tol_frac=0.01, tol_uv=None):
        """-> (ok, reason, detail).  May I chase this thing across that distance?

        🔴 `ok` True with reason `no_evidence` is the THIRD rung: admitted, and nothing has
        validated the model at this horizon.  A caller that reads only `ok` is acting on an
        unvalidated prediction.
        """
        why = ctypes.c_uint32(0)
        detail = ctypes.create_string_buffer(REASON_LEN)
        st = self.lib.bl_predict_admit_chase(
            body._ptr, ctypes.byref(predicted), float(distance_m), float(tol_frac),
            ctypes.c_double(0.0 if tol_uv is None else float(tol_uv)),
            ctypes.c_uint32(0 if tol_uv is None else 1), ctypes.byref(why), detail)
        return st == OK, Reason.NAMES.get(why.value, "unknown"), detail.value.decode()

    def new_memory(self, scope="task"):
        return Memory(self, scope)

    def place_matches(self, a, a_conf, b, b_conf):
        """-> "same" | "new" | "unsure".  The third is the point: acting on a map of somewhere
        else, confidently, is worse than having no map."""
        out = ctypes.c_uint32(0)
        st = self.lib.bl_place_matches(bytes(a), float(a_conf), bytes(b), float(b_conf),
                                       ctypes.byref(out))
        if st != OK:
            raise RuntimeError("bl_place_matches = %s" % STATUS_NAMES.get(st, st))
        return ("same", "new", "unsure")[out.value]

    def debt(self):
        """Every hand-set constant the ledger knows about: (name, site, standing, note)."""
        rows = []
        buf = ctypes.create_string_buffer(1024)
        for i in range(self.lib.bl_debt_total()):
            st = self.lib.bl_debt_line(i, buf, len(buf))
            if st != OK:
                raise RuntimeError("bl_debt_line(%d) = %s" % (i, STATUS_NAMES.get(st, st)))
            rows.append(tuple(buf.value.decode().split("\t")))
        return rows

    def debt_outstanding(self):
        """Body constants this layer cannot supply today.

        🔴 Report this next to a body's `hand_filled_constants`, never instead of it.  That one is a
        structural zero — it counts only what came through this API — and quoting it alone is the
        shape of claim this layer exists to stop other people making.
        """
        return self.lib.bl_debt_outstanding()


SLOT_BYTES = 64
MAX_SLOTS = 8
FINGERPRINT_BYTES = 16


class Predicted(ctypes.Structure):
    """Mirror of `bl_predicted`.  Where a learned model says a reference will be.

    🔴 No z, no pose, no object id — the same vocabulary as `WorldRef`.  This is the most natural
    place in the design for a 3-D pose to enter ("just tell me where it will BE"), and one that
    could return a pose would be a leak with a respectable name.
    """

    _fields_ = [
        ("u", ctypes.c_double),
        ("v", ctypes.c_double),
        ("extent", ctypes.c_double),
        ("at_period", ctypes.c_uint32),
        ("sigma_uv", ctypes.c_double),
        ("verified_periods", ctypes.c_uint32),
    ]

    @staticmethod
    def none_at(u, v, extent=0.1):
        """What a loop with NO prediction is implicitly asserting: it will still be there.

        Spelled out because that assertion is normally invisible — not predicting looks like not
        doing anything.
        """
        return Predicted(u, v, extent, 0, 0.0, 0)


class Memory:
    """One memory: a bounded, compacting context.  Storage is caller-owned, like the body's."""

    SCOPES = {"task": 0, "place": 1}
    EVENTS = {"new_task": 0, "unrecognised_place": 1, "body_changed": 2}

    def __init__(self, bl, scope="task"):
        self.bl = bl
        n = bl.lib.bl_memory_sizeof()
        self._storage = ctypes.create_string_buffer(n + bl.lib.bl_memory_alignof())
        self._ptr = ctypes.cast(self._storage, ctypes.c_void_p)
        st = bl.lib.bl_memory_init(self._ptr, n, Memory.SCOPES[scope], ABI_VERSION)
        if st != OK:
            raise RuntimeError("bl_memory_init = %s" % STATUS_NAMES.get(st, st))

    def declare(self, name, pins=False):
        why = ctypes.c_uint32(0)
        st = self.bl.lib.bl_memory_declare(self._ptr, name.encode(), 1 if pins else 0,
                                           ctypes.byref(why))
        return st == OK, Reason.NAMES.get(why.value, "unknown")

    def observed(self):
        """🔴 The clock the mechanical pin runs on.  Nothing in the model can decline to tick it."""
        self.bl.lib.bl_memory_observed(self._ptr)

    def write(self, name, value, durable=True):
        """-> (ok, reason).  A PERISHABLE fact is refused, not stored: rung 1 is look-again."""
        why = ctypes.c_uint32(0)
        st = self.bl.lib.bl_memory_write(self._ptr, name.encode(), value.encode(),
                                         1 if durable else 0, ctypes.byref(why))
        return st == OK, Reason.NAMES.get(why.value, "unknown")

    def get(self, name):
        """The stored value, or None -- which means never written, not written-empty."""
        buf = ctypes.create_string_buffer(SLOT_BYTES + 1)
        why = ctypes.c_uint32(0)
        st = self.bl.lib.bl_memory_get(self._ptr, name.encode(), buf, len(buf), ctypes.byref(why))
        return buf.value.decode() if st == OK else None

    def event(self, kind):
        """-> True if THIS memory was cleared.  A new errand does not clear the room."""
        cleared = ctypes.c_uint32(0)
        self.bl.lib.bl_memory_event(self._ptr, Memory.EVENTS[kind], ctypes.byref(cleared))
        return bool(cleared.value)

    def stats(self):
        o, u, r = (ctypes.c_uint64(0) for _ in range(3))
        f, d = ctypes.c_uint32(0), ctypes.c_uint32(0)
        self.bl.lib.bl_memory_stats(self._ptr, ctypes.byref(o), ctypes.byref(u), ctypes.byref(r),
                                    ctypes.byref(f), ctypes.byref(d))
        return {"observations": o.value, "unreadable": u.value,
                "refused_perishable": r.value, "filled": f.value, "declared": d.value}


class Body:
    """One physical robot's measured self-knowledge.  Storage is caller-owned; this library never
    allocates, because a hard-real-time safety layer must not depend on an allocator."""

    def __init__(self, bl):
        self.bl = bl
        n = bl.lib.bl_sizeof_body()
        self._storage = ctypes.create_string_buffer(n + bl.lib.bl_alignof_body())
        self._ptr = ctypes.cast(self._storage, ctypes.c_void_p)
        st = bl.lib.bl_init(self._ptr, n, ABI_VERSION)
        if st != OK:
            raise RuntimeError("bl_init = %s" % STATUS_NAMES.get(st, st))

    def close(self):
        if self._ptr:
            self.bl.lib.bl_close(self._ptr)
            self._ptr = None

    def measure(self, m):
        return self.bl.lib.bl_measure(self._ptr, ctypes.byref(m))

    def get(self, quantity):
        out = Measurement()
        st = self.bl.lib.bl_get(self._ptr, quantity, ctypes.byref(out))
        return st, (out if st == OK else None)

    def admit(self, ref, now_ns):
        """Returns (status, reason_name, detail).  🔴 A REFUSE is an ANSWER, not an error: do not
        retry it away and do not fold it into 'the task failed'.  'No data', 'not applicable' and
        'ran and scored zero' are three different facts."""
        why = ctypes.c_uint32(0)
        detail = ctypes.create_string_buffer(REASON_LEN)
        st = self.bl.lib.bl_admit(self._ptr, ctypes.byref(ref), now_ns, ctypes.byref(why), detail)
        return st, Reason.NAMES.get(why.value, "unknown"), detail.value.decode()

    def admit_quantity(self, quantity, at=None, tol=None, now_ns=0):
        """The gate for ONE quantity: (ok, reason_name, detail).

        This is what a caller asking "may I use this measurement, over this range" needs, and its
        absence is why a second copy of these four checks grew in Python. Same verdict source as
        `admit`, so the two can no longer disagree.
        """
        why = ctypes.c_uint32(0)
        detail = ctypes.create_string_buffer(REASON_LEN)
        st = self.bl.lib.bl_admit_quantity(
            self._ptr, ctypes.c_uint32(quantity),
            ctypes.c_double(0.0 if at is None else float(at)), ctypes.c_uint32(0 if at is None else 1),
            ctypes.c_double(0.0 if tol is None else float(tol)), ctypes.c_uint32(0 if tol is None else 1),
            ctypes.c_uint64(int(now_ns)), ctypes.byref(why), detail)
        return st == OK, Reason.NAMES.get(why.value, "unknown"), detail.value.decode()

    def execute(self, intent, spec, now_ms):
        cmd = (ctypes.c_double * MAX_JOINTS)()
        outcome = ctypes.c_uint32(0)
        st = self.bl.lib.bl_execute(
            self._ptr, ctypes.byref(intent), ctypes.byref(spec), now_ms, cmd, ctypes.byref(outcome)
        )
        return st, outcome.value, list(cmd)

    def plan(self, now_ns):
        """What this machine still owes itself, dependencies first: [(quantity, need)] by name.

        An empty plan means the measuring half has finished on this body.  It does NOT mean the
        stack around it carries no hand-set constants — that is `BodyLayer.debt_outstanding()`.
        """
        cap = len(Quantity.NAMES)
        qs = (ctypes.c_uint32 * cap)()
        ns = (ctypes.c_uint32 * cap)()
        n = ctypes.c_size_t(0)
        st = self.bl.lib.bl_measure_plan(self._ptr, now_ns, qs, ns, cap, ctypes.byref(n))
        if st != OK:
            raise RuntimeError("bl_measure_plan = %s" % STATUS_NAMES.get(st, st))
        return [
            (Quantity.NAMES.get(qs[i], "unknown"), Need.NAMES.get(ns[i], "unknown"))
            for i in range(n.value)
        ]

    def selftest(self):
        mask = ctypes.c_uint64(0)
        st = self.bl.lib.bl_selftest(self._ptr, ctypes.byref(mask))
        return st, mask.value

    def save(self):
        cap = self.bl.lib.bl_save_max_bytes()
        buf = (ctypes.c_uint8 * cap)()
        n = ctypes.c_size_t(0)
        st = self.bl.lib.bl_save(self._ptr, buf, cap, ctypes.byref(n))
        if st != OK:
            raise RuntimeError("bl_save = %s" % STATUS_NAMES.get(st, st))
        return bytes(bytearray(buf[: n.value]))

    def load(self, blob):
        buf = (ctypes.c_uint8 * len(blob)).from_buffer_copy(blob)
        return self.bl.lib.bl_load(self._ptr, buf, len(blob))
