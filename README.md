# body layer

> ## **world model + body layer**
> ## **世界模型 + 身体层**
> ### **世界靠学，身体靠量。**

Whatever belongs to **this body** is measured at power-on and kept measured; it never enters the
weights. Whatever belongs to **the world** is learned.

Today every VLA bakes the body into the weights, so a new robot costs hours of data and a gradient
pass — the best public figure is "a few hours of data / under 200 demonstrations". This layer is
the other half of that split, and the point of it is that **the body half was never in the weights
to begin with**. Each robot measures itself; the model does not change by one byte.

🔴 **And the layer publishes what it still owes.** *"We removed the hand-filled constants"* was not
true on 2026-08-09 and is not true today; the honest number is a
[**ledger**](slow/src/debt.rs) — **60 rows, 12 outstanding body constants**, two of them this
layer's own. Read it before the claim below: [what this layer still owes](#what-this-layer-still-owes).

---

## The invariant, stated once

The nearest ancestor is **UP-OSI** (Yu/Tan/Liu/Turk, RSS 2017, arXiv 1702.02453): a universal
policy plus online identification of body parameters. The single difference carries the whole
claim:

> UP-OSI feeds the measured body parameters **into the policy** — the policy is body-conditioned.
> Here they go **only to the execution layer. The policy's input contains no body parameter at all.**

"Measure the body, then hand it to the policy" is the entire field's reflex, and it *looks*
compliant: the body really was measured, nothing was baked into the weights. But once a body
parameter is inside the policy's input distribution, swapping the body **degrades quietly** instead
of failing loudly. Lose this and we are 2017.

So the enforcement is **structural, not procedural**. Read [`abi/body_layer.h`](abi/body_layer.h):

| port | what may pass | what **cannot be expressed** |
|---|---|---|
| `bl_world_ref` — any VLM / WM | normalised pixel `u,v`, region `extent`, verb, coarse effort | no `z`, no pose, no object id, no task id |
| `bl_policy_in` — the action model | image + that reference | no joint angles, no link lengths, no camera matrix, no gripper span, no payload, no robot name |
| `bl_execute` — this layer | everything measured about *this* body | — |

**A pointer that cannot express a pose cannot leak one.** An auditor checking for a privileged
channel reads the struct definitions; if no member can carry it, no amount of downstream code can.

---

## A layer that cannot say REFUSE is not a body layer

Every measured quantity carries **value / uncertainty / probed range / timestamp / dependency list
/ self-test / the version it replaced** — [`slow/src/measurement.rs`](slow/src/measurement.rs). A
bare `f64` cannot be refused on.

`bl_admit` refuses when a quantity was never measured, has gone stale, is being asked outside the
range it was actually probed over, is not precise enough for the ask, fails its own self-test, or —
the case a wall-clock TTL cannot catch — **something it was measured *against* has since moved**.

That last one is the whole reason `deps` exists. A hand-written `"my maximum payload is 500 grams"`
does not know the arm is sagging today, and **nothing in that system will ever notice**. The
category's admission test is exactly this:

> Change a hardware condition — add a weight, loosen a joint, knock the camera — and see whether
> the system **notices**. If it cannot notice, this layer does not exist in that system.

And the other half:

> Put it on a body with **different kinematics**, give it a fresh body layer, retrain **nothing**.
> Not "under 200 demonstrations". **Zero.**

---

## Layout

```
上层        ── VLM / WM   ── 看世界、听指令、判"擦干净没"    ← 学 · 秒级 · 可走 API
动作模型    ── 世界层意图 → 轨迹                            ← 学 · 几十毫秒 · 本地权重
════════ 量 / 学 分界线（body layer 的边界定义）════════
body layer 慢面 ── 量身体 · 存标定 · 判过期 · 拒绝           ← 量 · 秒级 · Rust
body layer 快面 ── 限位 · 力限 · 看门狗 · 急停               ← 量 · 硬实时 · Ada/SPARK
```

| path | what |
|---|---|
| [`abi/body_layer.h`](abi/body_layer.h) | **the contract.** Stable C ABI, so binding it does not require adopting our language |
| [`slow/`](slow/) | Rust. Measure, store with provenance, schedule, judge expiry, refuse, **and state its own debt**. **Zero dependencies, zero allocation.** |
| [`fast/`](fast/) | Ada/SPARK. Limits, force cap, watchdog, e-stop. **`gnatprove --level=2`: 40/40 checks proved, 0 unproved** (18 run-time, 7 functional contracts, 2 assertions, 3 initialization, 10 termination) |
| [`bind/python/`](bind/python/) | ctypes binding, stdlib only. The stack this layer must serve is Python, and until this existed **nothing could call it** |
| [`realdata/`](realdata/) | real episode logs the probes are asserted against in `cargo test`, plus the script that regenerates them |
| [`conformance/`](conformance/) | `abi_check.sh` — header ↔ library symbols, both directions. `python_check.sh` — the ABI driven from a third language, refusals first, **and the header's enum values compared against the built library**, which the symbol check cannot see |

Both faces hold the **same** numbers; the fast face exists only because *a force limit checked once
a second is not a safety limit*.

**Why a C ABI**: the claim is "anybody's mind + anybody's body". If plugging in required linking
Rust, half the field is excluded on day one. A standard only one language can consume is not a
standard.

**Why no allocator in either face**: a hard-real-time layer must not depend on one, and a layer
that cannot build for the target it has to run on is not a deliverable. An earlier draft had a
single `Box` behind the opaque handle; the caller supplies the storage instead
(`bl_sizeof_body` / `bl_init`).

🔴 **And that second clause is currently false, measured 2026-08-09.** `cargo build
--no-default-features` fails with 26 errors, and it failed before this was checked —
`git show HEAD:slow/src/probe.rs` already used `sort_by` (in `alloc`) and `sqrt`/`hypot`/`powi`
(in `std`, not `core`). The `no_std` attribute, the Cargo comment and the sentence above have all
been describing something that has never compiled. It is not fixed here because the fix is a
decision, not a tidy-up: the float methods need either a dependency — against the zero-dependency
rule, which is itself load-bearing — or a hand-written shim that then needs its own proof. It is
recorded at the attribute in `slow/src/lib.rs` so the next reader does not believe it.

**Why SPARK from day one, not "Rust now, SPARK later"**: the standing order, and its stated reason
— *you will not get around to it*. This repository's hit rate on "we will switch later" is zero.
The commercial shape is selling compliance, so a kernel with a machine-checked proof is a **product
feature**, not engineering vanity.

---

## What the prover found that review did not

Three things, and they are the argument for doing this in SPARK on day one rather than later:

1. **A contract that was simply wrong.** `Clear`'s postcondition claimed the failing branch leaves
   the state halted. False whenever `Clear` is called on a state that was never halted — a legal
   call. Corrected to "on failure nothing changes", which is also the stronger guarantee.
2. **A window where the invariant did not hold.** Setting `Is_Halted := True` and then
   `Why := <reason>` leaves one statement in between during which a halt carries no reason. An
   interrupt landing there observes a state that must not exist. Fixed with one `with delta`
   update per transition.
3. 🔴 **An invented body constant, caught as an unprovable assertion.** `Install_Limits` used to
   seat the safe hold at the midpoint of each joint's range. The prover could not discharge
   `Lo + (Hi−Lo)/2 ≤ Hi` over floats — and chasing that proof would have been solving the wrong
   problem, because *the midpoint of an arm's travel may be through the table*. The safe place to
   hold an arm is **where the arm is**. It is now an argument with a precondition, checked, rather
   than a number this package makes up. That is the same rule as the rest of the layer:
   **nothing that describes the body may be invented.**

## The rule about guards

> A guard that has never failed has never been tested, and in the output it is indistinguishable
> from a guard that does not exist.

So every self-test in [`fast/fast_selftest.adb`](fast/fast_selftest.adb) and every unit test in
[`slow/src/lib.rs`](slow/src/lib.rs) is a case the guard **must** refuse, and the build fails if
one of them is admitted. Exactly one "must be admitted" case sits at the end of each suite, because
a layer that refuses everything is also not a body layer.

This rule is not abstract here. Instances already paid for: a two-state occlusion control whose
second clause made `0 > 0` print PASS; a docstring promising a `--ref` positive control that the
argument parser never implemented; a watchdog whose counter was broken, so its "no new episodes"
predicate was permanently true and it deleted a healthy leg's 15 episodes. **In every case the log
said the guard was fine.**

---

## How much of it exists

Asked directly on 2026-08-09 ("is the self-calibration already done?"), and it was not written down
anywhere, which is its own finding. **A named slot in an enum is not a probe.** The answer then was
**5 of 10**; five quantities were names.

**Now 11 of 11.** Every quantity has an estimator in
[`slow/src/probe.rs`](slow/src/probe.rs), and `slow/src/lib.rs` asserts it mechanically —
`debt::declared_only() == 0` fails the build the moment a slot is added without one.

| quantity | measured by |
|---|---|
| `hand_pixel` · `image_jacobian` · `arm_weight` · `latency` | the original four |
| `step_delivery` | added from a measurement — see below |
| `reach` | a band, not a radius; validated on 2174 real episodes |
| `gripper_span` · `backlash` · `contact_threshold` · `self_occlusion` | added 2026-08-09 |
| **`tool_offset`** | added 2026-08-09 **from a census of the live stack** — see below |

Two of them are checked against **real logs**, not only against synthetic cases a test author
imagined — `contact_threshold` against 520 rows of a press-depth staircase with PhysX contact as
ground truth, `backlash` against three 300-step sweeps of a 7-joint arm
([`realdata/`](realdata/), asserted in `cargo test`). The real data earned its keep immediately: see
*"what real data found that the unit tests did not"* below.

### And there is now a schedule

[`slow/src/schedule.rs`](slow/src/schedule.rs). Plugging in a new machine is `plan()` → run the
probes it names → `submit()` each → repeat until `is_ready()`; the order comes from what each
quantity is *expressed in terms of*, and nothing about it is typed in per robot. The part that could
not be left to a person is the **cascade**: re-measuring the image Jacobian invalidates the hand
point, the gripper span, the occlusion map and the tool offset **while all four of their own clocks
still read fresh**, so the plan schedules them before they go bad rather than after.

⚠️ Still true: **nothing outside this directory reads this layer.** A grep on 2026-08-09 for who
consumes it returned **zero** — eight prose mentions, no imports. The running teacher is Python, so
the first move against that is [`bind/python/body_layer.py`](bind/python/body_layer.py) (ctypes,
no dependencies) and [`conformance/python_check.sh`](conformance/python_check.sh), which drives the
ABI from Python **refusals first** and cross-checks the header's enum against the built library —
something `abi_check.sh` cannot do, because a quantity added on one side and forgotten on the other
leaves every symbol unchanged and every call returning success. Being callable is not being called;
wiring it into the teacher is the next thing, not a done thing.

### `step_delivery` is the first quantity added because a body demanded it

Two arms on the same harness, same waypoint controller, same 45 mm commanded step: one delivered
**0.76** of it per control period, the other **0.11**. The per-waypoint step budget had been set
from the first arm, so the second could never reach a waypoint — **0.136 m of residual on every
episode**, surfacing as *"the arm stopped short of the pre-grasp waypoint"*, which reads like a
planner, reachability or wrist-convention fault. It was none of those, and every scalar in the log
was ordinary.

The instinct at that point was to open the simulator's actuator config and raise the second arm's
stiffness until it kept up. That is wrong twice over: **it types a body constant** (the debt this
layer exists to drive to zero) and **it is not portable** — a real robot has no config file to
read. Worse, it changes the physics the demonstrations are collected under, so the data quietly
becomes a different dataset.

Measuring instead and sizing the budget from the arm's own progress: residual **0.136 m → 0.0058 m**
— the first arm's own figure — with nothing about the robot changed.

⇒ it is deliberately **not** `latency` (dead time; both arms answered 1 period) and **not**
`backlash` (a dead band at a reversal; this is a shortfall on every step in one direction).
⚠️ And it is still measured *in the experiment's own Python*, not through this ABI. **Named debt,
not a solved problem.**

### `tool_offset` is the second, and it was found by counting what the live stack types in

One number, written by hand in **four** places in the deployed system, with three values for three
bodies:

| where | value | what it says |
|---|---|---|
| `L3_GRIPPER_BIAS` (env, deployed executor) | **0.145** | *"x5 = 0.145, franka = 0.102"* — copy it by hand out of `Assets/Robots/<body>/robot_config.yml` |
| the teacher's `flange_for()` | **0.145** | `tcp = flange + 0.145 · R[:,0]`, hardcoded — so setting the knob above fixes one of the two |
| the teacher's wrist-tilt ceiling | **0.145** | *"the flange sits 0.145 m back along the tool axis"* |
| a third rig's harness | **0.1034** | `tcp_off` |

**4.3 cm apart between two bodies, and 0.145 is the default.** A machine that forgets to pass it
does not fail — it executes with another robot's geometry. That is precisely the *quiet* degradation
this README opens by saying the design exists to prevent, running in production.

It is measurable by acting on itself: **turn the wrist and the working point sweeps an arc whose
radius is the offset.** No kinematics, no declared frame — the geometry is in the picture.

### What real data found that the unit tests did not

`backlash` scores each reversal against the body's own same-direction delivery, so it needs that
control ratio. Fed three real 300-step sweeps, one joint's continuation ratios scattered around
**0.00025 with a standard error of 0.279** — and the estimator divided by it and reported a dead
band of **1.01 rad, about 58°, on a simulated arm that has none.** Every unit test passed. The guard
is now that the control ratio must be separable from zero by its own spread, and the same real logs
are the regression test: the free-space sweep is answered on 6 of 7 joints (all ≤ 2.6e-4 rad, i.e.
zero), and the sweeps where the leg is pressed into a surface — where a joint fighting contact has
no established free-motion ratio — are **refused**.

`gripper_span` was caught by its own test rather than by data, and the shape is the same: on a
perfectly stuck gripper the true slope and the true residual are both zero, so in floating point
they come out as noise of the same size, and the sign test then decided the verdict from the sign of
a rounding error — reporting a jammed gripper as *"the jaws close as you command them open"*.

## What this layer still owes

🔴 **The most expensive thing in this repository was a true number that flattered us.**

`hand_filled_constants()` returns `0`. It is true, and it is a **structural** zero: nothing can
enter through `bl_measure` without a passing self-test, so it counts a set that is empty by
construction. It says nothing whatever about the constants that never came near this API — and
those are the ones running the robot.

The proof arrived as a measurement. A parameter search over the deployed teacher on 2026-08-09
found its **dominant** constant:

> `TEACH_HIGH_FRAC` — how far up an object's long axis to place the grasp.
> **≤ 0.30: 32 of 44 (73%). > 0.31: 10 of 100 (10%). Fisher p = 9.3e-14.**

The largest effect anybody has measured on that stack, and **this layer had never heard of it.** A
census of the same two files then found **45 environment knobs and a hardcoded camera matrix**
against ten declared quantities. So the honest statement is not *"we removed the hand-filled
constants"*; it is *"we removed the ones we thought of, and the biggest one was found for us by a
search."*

[`slow/src/debt.rs`](slow/src/debt.rs) is the correction, and it is readable through the ABI
(`bl_debt_total` / `bl_debt_outstanding` / `bl_debt_line`) so an auditor gets both numbers without
reading any Rust. **60 rows**, one per constant, each with where it is set, what this layer can do
about it, and what would discharge it. **12 are body constants this layer cannot supply today.**

| the entries that matter most | standing |
|---|---|
| **`TEACH_HIGH_FRAC`** (32/44 vs 10/100, p=9.3e-14) | **outstanding — no slot, on purpose** |
| `FX` · `CX,CY` · `CAM_POS` · `CAM_EULER_DEG`, hardcoded under *"FROZEN P1 rig; do not re-derive"* | the image Jacobian exists so that no intrinsic or extrinsic has to be written down — the claim is true of this layer and **false of the system it serves** |
| **`bl_spec.step_m`** — this layer's own | **outstanding.** Every command is scaled by it, no probe produces it, and it is the metric ruler `gripper_span` and `tool_offset` divide by |
| **`bl_spec.damping`** — this layer's own | **outstanding.** Documented as *"from the measured Jacobian's own conditioning"*, and nothing computes it — a promise kept by a comment |
| `TEACH_SETTLE` · `TEACH_REHOME_STEPS` | outstanding, and the cheapest to discharge: both are `latency` + `step_delivery`, already measured, not yet wired |
| `L3_GRIPPER_BIAS` · `TEACH_JAW_MAX` · `BPD_REACH_BOX` · … | **replaceable** — a probe exists |

Three things about this table are deliberate.

1. **`TEACH_HIGH_FRAC` did not get an enum slot.** Whether a grasp 30% up an object holds depends on
   the **object**, so it has not been shown to be measurable off the body — and this README's own
   finding is that a named slot with no probe is worth nothing while reading as covered. Its
   discharge test is pre-registered instead: with `gripper_span` and `tool_offset` measured, derive
   the clearance the jaws need and re-run the sweep. If the effect disappears it was a body constant
   in disguise; if it survives, it belongs to the model and not to this layer. Both outcomes are
   informative, which is the only reason to write the test down before running it.
2. **"Replaceable" is not "replaced."** A probe existing does not connect it to anything, and
   nothing outside this directory reads this layer. Collapsing those two into one green cell is
   exactly the move this file exists to refuse.
3. **The ledger audits this layer too.** A ledger that only counts other people's constants is an
   advertisement. Two of the twelve are in `bl_spec`, in the middle of the execution path.

## What is actually hard here, and what is already done

Recognising the hand is **done**: 1.7 cm → **0.62 cm**, reproduced across three independent
processes. What is *not* done is keeping it during the servo, and the archive is precise about it:

* *"three times the localisation reading improved markedly and the closed loop gained nothing."*
  Fixing the fit from 1.7 cm to 0.62 cm moved the latch **not at all**, over 32 paired layouts.
* fit-time error **2.0 px**; error **at the moment the hand is closest to the target**
  **4.9–14.6 px = 1.5–4.6 cm** — at or above the 2.0 cm latch radius.
  *"The version that fits best is the one that drifts worst."*
* the whole family of "give the fit more evidence" is refuted: repainting the robot took usable
  candidate pixels **11 → 173 (15×)** and the loop stayed **0/9**.

⇒ the specification for [`slow/src/hand.rs`](slow/src/hand.rs), written by that verdict: **re-measure
every control step, and abstain rather than guess.**

The trap it is built against is named in the file. The old selector was *"whichever rigid thing
responds most to my command is me"* — derived when the competitors were **the hand and its shadow**.
On a different rig the competitors became **different links of the same arm**, and the elbow, nearer
the camera at 0.393 m against the fingertip's 0.438 m, won the rule. The loop then aimed the elbow
at the mark and reported **0.04–9.3 px** of error while the truth was **167 px**.

> A selection rule derived for two candidates does not report an error when the candidate set
> changes. It just quietly picks wrong.

So the estimator does **not** take a maximum. It enumerates candidates and, when the top two are
within `min_separation` (default 1.50 — the fingertip/elbow gain ratio was about **1.11**), it
returns a refusal instead of the better of them.

---

## Acceptance: one long task, end to end, through this layer

🔴 **The leaderboard runs go *through* this ABI. No stitching alongside it** — a stitched path is
where cheating hides, and nobody, including us, would know it had happened.

**Target: `classify_objects`** — 1100 steps, a pile sorted into three baskets. Chosen because it is
long, it is inside what the interface can express (reach / grasp / place only), it needs the
multi-object loop *and* the return-home phase, and it is the one task whose target selection does
**not** go through the hand-maintained per-task table (that table is derived from each task's own
scoring function, which is precisely the kind of cheat this layer exists to make impossible).

Acceptance is not "it scored". It is all of:

1. every reference the eye emits passes through `bl_admit`, and refusals are **counted separately**
   from failures — *no data*, *not applicable* and *ran and scored zero* are three different things;
2. `bl_policy_in` carries no body parameter (checked by reading the struct, not by trusting a log);
3. the hand point is re-measured every step, with abstentions reported;
4. 🔴 **both** constant counts are reported, never the flattering one alone —
   `hand_filled_constants() == 0` (structural, counts only what came through this API) **and**
   `bl_debt_outstanding()`, which is **12** and is the number that describes the robot;
5. the arm returns to origin, because the task's own scorer requires it;
6. and the numbers are reported as **N attempts, M successes** — absolute counts, never a
   percentage alone.

---

## Licence and shape

Whatever occupies this position has historically been licensed **per unit**. The shape here:
**fully open (copyleft) + a commercial licence + an ad-valorem per-unit royalty** — a flat annual
fee is hostile to small teams, and a $500 arm should not pay what a $200k industrial robot pays.

No patent application for now, deliberately, with one irreversible fact on the record: once
published, patent rights outside the United States are permanently lost (most jurisdictions have no
grace period).
