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
| [`slow/`](slow/) | Rust. Measure, store with provenance, judge expiry, refuse. **Zero dependencies, zero allocation.** |
| [`fast/`](fast/) | Ada/SPARK. Limits, force cap, watchdog, e-stop, with machine-checked absence of runtime errors |

Both faces hold the **same** numbers; the fast face exists only because *a force limit checked once
a second is not a safety limit*.

**Why a C ABI**: the claim is "anybody's mind + anybody's body". If plugging in required linking
Rust, half the field is excluded on day one. A standard only one language can consume is not a
standard.

**Why no allocator in either face**: a hard-real-time layer must not depend on one, and a layer
that cannot build for the target it has to run on is not a deliverable. An earlier draft had a
single `Box` behind the opaque handle; the caller supplies the storage instead
(`bl_sizeof_body` / `bl_init`).

**Why SPARK from day one, not "Rust now, SPARK later"**: the standing order, and its stated reason
— *you will not get around to it*. This repository's hit rate on "we will switch later" is zero.
The commercial shape is selling compliance, so a kernel with a machine-checked proof is a **product
feature**, not engineering vanity.

---

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
4. `hand_filled_constants() == 0`;
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
