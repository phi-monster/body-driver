#!/usr/bin/env bash
# python_check.sh -- drive the C ABI from Python, REFUSALS FIRST.
#
# 🔴 WHY REFUSALS FIRST, AND WHY THIS EXISTS AT ALL.
#
# `abi_check.sh` proves the header and the library agree about which SYMBOLS exist.  It says nothing
# about whether the two agree about what those symbols MEAN -- an enum value added on one side and
# not the other passes it, and every call keeps returning success while the two sides are talking
# about different quantities.  This script closes that: it parses the header's `BL_Q_*` values and
# compares them against what the built library reports, from a third language.
#
# And it drives guaranteed refusals before any success, for the reason the Rust-to-Ada conformance
# gives: a binding exercised only on the happy path is a binding whose error codes have never been
# observed.  If a struct were mislaid, a status misnumbered, or a pointer misaligned, the happy path
# would very often still return BL_OK.
#
# The concrete gap it closes: on 2026-08-09 a grep for who reads this layer answered ZERO.  The
# running teacher is Python.  A C ABI nobody can call from the language the stack is written in is
# a header file with tests.
#
# Usage:  ./conformance/python_check.sh
set -uo pipefail
cd "$(dirname "$0")/.."

if ! ls slow/target/*/libbody_layer.dylib slow/target/*/libbody_layer.so >/dev/null 2>&1; then
  echo "building the cdylib first..."
  (cd slow && cargo build) || exit 2
fi

PYTHONPATH="bind/python${PYTHONPATH:+:$PYTHONPATH}" python3 - "$@" <<'PY'
import ctypes, re, sys
from body_layer import (BodyLayer, Body, Measurement, WorldRef, PolicyOut, Spec,
                        Quantity, Reason, MAX_DIM, ABI_VERSION,
                        OK, REFUSE, EINVAL, EVERSION, ENOSPACE,
                        X_REFUSED, STATUS_NAMES)

fail = 0
def check(name, got, want):
    global fail
    ok = got == want
    if not ok:
        fail = 1
    print("  %-4s %-52s got=%r want=%r" % ("PASS" if ok else "FAIL", name, got, want))

bl = BodyLayer()
print("library: %s" % bl.path)

# ---------------------------------------------------------------- the enums must agree
# A quantity added in Rust and forgotten in the header passes abi_check.sh, because the symbols are
# unchanged.  Both sides are then confidently naming different things by the same integer.
hdr = open("abi/body_layer.h").read()
hdr_q = {int(v): k.lower()[5:] for k, v in
         re.findall(r"(BL_Q_[A-Z_]+)\s*=\s*(\d+)", hdr) if k != "BL_Q_COUNT"}
hdr_count = int(re.search(r"BL_Q_COUNT\s*=\s*(\d+)", hdr).group(1))
print("== header enum vs library ==")
check("BL_Q_COUNT == quantities the library names", hdr_count, len(Quantity.NAMES))
for i, name in sorted(hdr_q.items()):
    check("header BL_Q id %d" % i, Quantity.NAMES.get(i), name)

# ---------------------------------------------------------------- refusals, before anything works
print("== must refuse ==")
n = bl.lib.bl_sizeof_body()
storage = ctypes.create_string_buffer(n + 64)
p = ctypes.cast(storage, ctypes.c_void_p)
check("bl_init with the wrong ABI version", bl.lib.bl_init(p, n, ABI_VERSION + 1), EVERSION)
check("bl_init with storage too small",     bl.lib.bl_init(p, 1, ABI_VERSION),     ENOSPACE)
check("bl_init with a null pointer",        bl.lib.bl_init(None, n, ABI_VERSION),  EINVAL)

body = bl.new_body()
ref = WorldRef(u=0.5, v=0.5, extent=0.0, verb=0, manner=0.5, frame_id=1)
st, why, detail = body.admit(ref, 1_000_000_000)
check("fresh body refuses to admit", st, REFUSE)
check("...and says why", why, "never_measured")

def jac(epoch_dep=None, selftest=1, lo=-1.0, hi=1.0):
    m = Measurement()
    m.quantity = Quantity.IDS["image_jacobian"]
    m.dim = 18
    for k in range(18):
        m.value[k] = 800.0 if k % 7 == 0 else 5.0
        m.uncertainty[k] = 0.01
        m.valid_lo[k] = lo
        m.valid_hi[k] = hi
    m.measured_at_ns = 1_000_000_000
    m.valid_for_ns = 0
    m.selftest_passed = selftest
    return m

check("a measurement whose self-test failed", body.measure(jac(selftest=0)), EINVAL)
check("an empty validity window (lo >= hi)",  body.measure(jac(lo=1.0, hi=1.0)), EINVAL)
bad = jac(); bad.quantity = 9999
check("an unknown quantity id",               body.measure(bad), EINVAL)
check("bl_get on an unknown quantity id",     body.get(9999)[0], EINVAL)
check("bl_get on one never measured",         body.get(Quantity.IDS["arm_weight"])[0], REFUSE)
check("bl_load of a foreign buffer",          body.load(b"XXXX\0\0\0\0"), EINVAL)

qs = (ctypes.c_uint32 * 1)(); ns = (ctypes.c_uint32 * 1)(); cnt = ctypes.c_size_t(0)
check("bl_measure_plan with cap too small",
      bl.lib.bl_measure_plan(body._ptr, 0, qs, ns, 1, ctypes.byref(cnt)), ENOSPACE)

buf = ctypes.create_string_buffer(64)
check("bl_debt_line past the end of the ledger",
      bl.lib.bl_debt_line(bl.lib.bl_debt_total(), buf, 64), EINVAL)

# ---------------------------------------------------------------- then the things that must work
print("== must work ==")
plan = body.plan(1_000_000_000)
check("a fresh body owes every quantity", len(plan), len(Quantity.NAMES))
check("...and the Jacobian comes before the hand point",
      [q for q, _ in plan].index("image_jacobian") < [q for q, _ in plan].index("hand_pixel"), True)
check("...each with a reason", plan[0][1], "never_measured")

check("a well-formed measurement is accepted", body.measure(jac()), OK)
st, got = body.get(Quantity.IDS["image_jacobian"])
check("...and reads back", st, OK)
# A functional layout check: if the Python struct disagreed with the Rust one by a single byte of
# padding, these fields would come back shifted while every status stayed BL_OK.
check("...with value[0] intact",       got.value[0], 800.0)
check("...with dim intact",            got.dim, 18)
check("...with uncertainty intact",    round(got.uncertainty[0], 6), 0.01)
check("...with the epoch stamped",     got.epoch >= 1, True)
check("...and prev_epoch = 0 (first)", got.prev_epoch, 0)

check("the plan shrank by exactly one", len(body.plan(1_000_000_000)), len(Quantity.NAMES) - 1)

blob = body.save()
fresh = bl.new_body()
check("save/load round trip", fresh.load(blob), OK)
check("...and the value survived", fresh.get(Quantity.IDS["image_jacobian"])[1].value[0], 800.0)
check("a truncated file is refused", fresh.load(blob[:-9]), EINVAL)

intent = PolicyOut(); intent.dir[0] = 3.0     # not a unit vector
spec = Spec(step_m=0.004, period_ms=40, damping=0.05, n_joints=6)
st, outcome, _ = body.execute(intent, spec, 1000)
check("a non-unit direction is a bad intent, not normalised", (st, outcome), (OK, 3))
intent2 = PolicyOut(); intent2.dir[0] = 1.0; intent2.grip = 1.0
st, outcome, _ = body.execute(intent2, spec, 1000)
check("an under-measured body refuses to execute", (st, outcome), (OK, X_REFUSED))

# ---------------------------------------------------------------- the debt is readable from here
print("== debt ==")
rows = bl.debt()
check("the ledger is readable through the ABI", len(rows), bl.lib.bl_debt_total())
check("...and every row has four fields", all(len(r) == 4 for r in rows), True)
names = [r[0] for r in rows]
check("TEACH_HIGH_FRAC is in it", "TEACH_HIGH_FRAC" in names, True)
check("this layer audits itself too", "bl_spec.step_m" in names, True)
out = bl.debt_outstanding()
check("outstanding is not zero", out > 0, True)
print("  ledger: %d rows, %d outstanding body constants" % (len(rows), out))
print("  (a body's own hand_filled_constants is a STRUCTURAL zero -- report both or neither)")

print()
if fail:
    print("python_check: FAIL")
else:
    print("python_check: PASS -- the ABI is callable from Python and refuses before it works")
sys.exit(fail)
PY
