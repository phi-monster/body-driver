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

ABI_VERSION = 1
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
