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

## 现在在哪(2026-08-25 22:00,一屏读完)

🔴 **官方 `general_pickup` 55 集的成绩:从未产生过。0 次。** 不是跑得不好,是**从来没跑到那一步**。
普查(2026-08-25):此前 14 炮 **进入干活模式 0 次** —— 全部死在开机自标定的一格上,
而此前汇报的一切"进展"都只是**标定内部指标**。

| 项 | 绝对数 |
|---|---|
| **官方 55 集成绩** | **0 / 55**(50 失败 + 5 不稳定,2026-08-25 首次跑完)· 目标 **≥42/55**(超人类遥操 76.03%) |
| 进入干活模式 | 2026-08-25 **首次**(此前 14 炮全 0) |
| 走完全程(计划→下探→合爪→抬) | 2026-08-25 **首次**(`BL_ZHIQU=1` 直取模式;默认模式走不完) |
| 已量到的身体量 | **8 / 15**(home_pose · latency · backlash · reach · step_delivery · image_jacobian · hand_pixel · contact_threshold) |
| 量不到的 | `gripper_span`(相机够不着:指行程 1.8 cm @ 0.56 m ⇒ **9 像素**)· `tool_offset`(三种量法全败,见 LAB) |

**今晚修掉的(全部已验):**
1. ~~三维重建上下颠倒~~(USD −z 朝前 vs CV +z 朝前)⇒ 桌面法向 `[0.44,0.21,0.87]` → `[0,0,-1]`。
2. ~~我们自家的 −0.15 m 头相机偏移把评测场景的图整个弄黑~~ ⇒ 改回官方 `[0.0,-0.41,1.308]`。
3. ~~`0÷0=NaN` 让每把都报 `NoFrame`~~ · ~~解相机被塞在「量爪宽」那一相当副产品~~ · ~~回原位/解相机/看爪子的顺序~~。

🔴 **当前唯一的具名阻塞:`tool_offset`(法兰到指尖多长)量不出来。**
三种在任务里量的办法全败,原因各不相同(见 [`LAB.md`](LAB.md) 陷阱节)。
后果是爪子停在物体**上方**合爪 ⇒ **每一把都是空的** —— 而这是**驱动自检自己判的**,
它没有把假抓报成成功。

**下一步(待 owner 定):工具长换一条来路。** 候选:
(a) 换个能真的下得去的位形/腕姿再用「走得到的最低处」量;
(b) 走**机体自报**那条路 —— 相机内外参已经这么拿了,手的几何应当同源,而不是靠戳。


**🔴 已定案(2026-08-25,owner):**
- **删掉「15 格量不完就不干活」。** 缺的格不再回声不动;合爪判据(每根手指读数都停在半途)**零尺度**,爪宽只当长度尺用。
- **米制要从驱动和文档里删干净。** 人不知道自己手掌几厘米,只知道"这个杯子一把能攥住" —— 那是**比较**不是尺寸。
- **UX:没有独立的自标定流程。** 用户只有「用驱动干活」一个功能;缺什么在任务里当场补,无感、越用越准。缺东西的代价是**慢**不是**不能**,且必须**说出来**。
  例外:安全那几件不能靠干着干着学会 —— 起点就最弱(慢、力小、碰到阻力就停),量到再放开。
- **不全改 Ada。** `fast/` 已是 Ada/SPARK;要加深的是成色(`gnatprove` 证明义务清零 + 进 CI),不是扩大面积。

陷阱 / 教训 / 逐炮读数 ⇒ [`LAB.md`](LAB.md)。

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

### 🔴🔴 "11 of 11" 说的是**代码在**,不是**量得出来** —— 别把这两件事读成一件

上面那张表回答的是"每一格有没有估计器"。它**不回答**"这具身体上真的量到了没有",
更不回答"量到的那个数是不是一个常数"。2026-08-18 用 **15 次从零开始的独立标定**把后两件事量了:

| | 绝对数 |
|---|---|
| 一炮从零跑完能拿到几格 | **4–10 格**,多数停在 **7/15** |
| 15 炮合起来有数的 | **12/15** |
| **跨 15 炮真的对得上的(最大相对散布 <2%)** | **1 格**(`step_delivery`,散布 1.0%) |
| 从没量到过的 | `friction` · `hand_pixel` · `gripper_span` |
| 抓取成功 | **0 次** |

散布(15 炮之间最大相对差):`contact_threshold` **1603%** · `image_jacobian` **564%** ·
`arm_weight` **238%** · `home_pose` **1954%** · `backlash` **72%**。
⇒ **"量到了"和"它是这具身体的一个常数"是两件事,而我们此前只验过前一件。**

零 GPU 重跑这张表:`python3 results/all15-aug18/collect.py results/all15-aug18/json/*.json`。
逐条 bug 与修法见 [`DRIVER_GOAL.md`](DRIVER_GOAL.md) §五(2026-08-18 那几行)。

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

⚠️ **Stale as of 2026-08-15 — see [`DRIVER_GOAL.md`](DRIVER_GOAL.md).** The layer HAS been driven on a real robot: `phi-monster/lekiwi` records the full chain running on a Pi through the C ABI, with grounding / aiming / contact verified on hardware. The sentence below described 2026-08-09 and was never updated; it misled a reader on 2026-08-15 into reporting "never plugged into a real robot". Original text kept below for the audit trail. ⚠️ Still true *at that date*: **nothing outside this directory reads this layer.** A grep on 2026-08-09 for who
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

---

# What the layer above says to it: **the contact set**

> Merged 2026-08-13 from `universal-grounding/ARCH.md` and `ARCH_NEW_TRY.md`, both now deleted.
> One conclusion, in one place, latest version only.

## The cut is fixed by mechanics, not taste

```
object side:  G · F_contact = F_object      G depends only on WHERE on the surface + the normal — body-independent
body side:    J_hand(q) · q̇ = v_contact     J_hand depends only on the BODY — object-independent
```

The only quantities both sides share are **contact point / normal / relative direction at that point**.
⇒ that is the unique place where *the layer above never names a body and the layer below never names
an object, and the information is still sufficient.* One notch up (end-effector pose + a gripper
scalar) has already assumed a two-fingered hand; one notch down (joint angles) has already assumed a
DoF count. Murray–Li–Sastry ch.5.

⚠️ **Correction on the record (owner, 2026-08-13):** the deleted `ARCH.md` framed this as *"the old
interface was two-fingered"*. That is wrong about **this** architecture and was repeated in chat
before being checked. The `bl_policy_in` port above has never carried a gripper span — the two-finger
assumption lives in the **deployed RoboDojo/L3 action space** (`dx dy dz drx dry drz grip`), which is
glue, not this contract. The contact set replaces *that*, and its argument is the mechanics above,
not a flaw in this layer.

## The contact set, stated once

> ① which points on the object surface are touched · ② the normal at each and the **cone** of force
> allowed there (direction, no magnitude) · ③ how the **object** must move (a twist) · ④ tolerance

Thirteen verbs collapse into one template — *touch these points, push this way at each, and the
object does this*:

| verb | filled in as |
|---|---|
| push · sweep | one point (or edge), lateral force, object translates on its support |
| press · tap | one point, normal force, object does not move (tap = with velocity) |
| pry · flip · scoop | one point at an edge, up/side lever, object rotates about that edge |
| grasp | ≥2 opposed points, inward force, object follows the hand |
| pour · twist · insert | already held, object rotates/translates about an axis |

⇒ what has to be learned is no longer *thirteen skills* but **how to fill this template's
parameters** — half computed from geometry, half supplied by semantics.

**Reach forbids two fingers by construction**: nothing in ①–④ mentions how many fingers exist. A
suction cup fills the same table with one point and a normal-only cone.

## Four producers, and what each is forbidden to name

| layer | produces | from | 🔴 must never contain |
|---|---|---|---|
| **① body layer** (this directory) | constants of *this* body | **measurement** | any scene or task quantity |
| **②a contact generator** — `contact-gen/`, its own crate, 13/13 tests | where this shape can be grasped (point · normal · jaw direction), filtered by ① | **computation** (geometry) | semantics, task |
| **②b closed-form executor** — `contact-exec/`, its own crate, 5/5 tests | contact set + object twist → joint trajectory | **computation** | — |
| **③ eye + weights** | which object · what to do · where it is **useful** to touch (not merely stable) | **VLM / learned** | metres, joint angles, DoF count, finger count, link geometry, gripper opening |

`contact-gen` **does not read this layer**: body constants are passed in as arguments. The reverse
dependency would destroy the mechanical guarantee that ②a is body-independent. It speaks the same
process protocol as `bl` — `cg` with `body` / `grid` / `pts` / `gen` / `at`.

### Rules ②a already encodes, each paid for by a measurement

| rule | why |
|---|---|
| **height above the support is a THRESHOLD, not a maximise** | written as a maximise it dominates every later term and picks the topmost soft skin — a shoe's collar, the top face of a flat-lying hammer. Render-decided, and the only one of four revisions that raised the score: 26% → **34%** |
| 🔴 **never delete a candidate for being "too wide"** | the one grasp-ability rule in this repo forbids using the jaw opening as a filter. Width enters the **ordering** only; segments that do not fit stay in the table, ranked last |
| **a refusal must be able to state its reason** | `Refusal::{JawSpanUnknown, TooFewPoints, Flat, NoSection}` — never a silently empty list |
| **jaw span unmeasured ⇒ the whole layer refuses** | `JawSpan::{Measured, Declared, Unknown}` |

⚠️ **Measuring a width correctly is not grasping better.** Changing band thickness to the jaw face
height *did* fix the measured width (a block stopped reading 0.0048 m) and cost **12 pp** — it also
mixed two heights of material into one band, and *closed-on-nothing* went 21% → 33%.

### Still owed by ②a

| owed | status |
|---|---|
| **"can it be pinched"** | once measured locally, *"the jaws fit"* is almost always true for a convex solid ⇒ what actually decides is friction / depth / stability, which geometry alone cannot answer. **Slip is 41% of failures** |
| surface normals · friction cones · centre-of-mass alignment | the current last-place sort key ("how deep the material is") is an invented proxy; no published grasp-quality metric is that |
| jaw-face height | no slot in this layer; passed as a parameter today |

## Body / world / semantics — the spine

| | belongs to | obtained by | valid for |
|---|---|---|---|
| tool offset · jaw span · contact threshold · reach · home pose · **how much force a command produces** | **body** | **measured** (this layer) | one calibration, long-lived; new battery / worn part ⇒ measure again |
| how heavy this thing is · where its centre is · whether it slips · how wide it is *there* | **world** | **measured by touching it** | asked per object, remembered once asked |
| which object · what it should become · where it is **useful** to hold | **semantics** | VLM / learned | — |

### 🔴 Force must be stored as two halves and never multiplied together

> owner 2026-08-13: *"抓苹果这种任务做多了是不是就不用再碰了?但这个发力跟机体状态有关吗,满电/新旧都有差别。"*

**How many newtons an apple needs** is a world property and can be remembered. **How many newtons a
command produces** is a body property that drifts with charge, wear and temperature. Store the
product — *"grasp apple at 0.37"* — and a battery swap invalidates it. Store the first, ask this
layer for the second, every time.

⇒ this is where measuring beats practising: a human cerebellum re-trained for a new arm takes months;
re-measuring takes minutes.

### Brain / cerebellum, and why the analogy earned its place

Cortex = ③ the eye (decide what is wanted). Cerebellum = ① this layer + ②b (internal model + fast
loop). The model a cerebellum holds of its own body **is** the set of measured body constants —
a forward model.

It predicted the observed symptoms before they were explained: cerebellar damage presents as *intent
intact, but overshoot, mis-graded grip, jerky motion.* The robot: knows it wants the block, reaches
roughly, knocks it away, cannot hold it, flings it 0.27 m in one step. Textbook.

**One difference, and it is the advantage: theirs is trained, ours is measured.**

## More constraints ⇒ a smaller answer set

An unconstrained search is infinite; every **true** constraint collapses it by an order of magnitude.

| constraint | removes | from |
|---|---|---|
| what my body can do | everything this body cannot execute | **① measured** |
| only these points can be touched | every unreachable contact | **②a computed** |
| what the world must become | every action irrelevant to the goal | **③ eye** |
| physics | everything impossible | the world model |

🔴 **Measured facts are HARD constraints (this arm simply cannot reach); learned facts are SOFT
preferences. Hard constraints cut the space; soft ones cannot.**
🔴 This is also why *refuse rather than invent* pays: an invented number is a **false** constraint —
it removes the correct answer, and nothing downstream can tell.

### Scenarios, and the constraint each one forces

| scenario | constraint it forces |
|---|---|
| real robot (not sim) | must run in real time; **cannot see the far side** — no pretending a full model exists |
| new body (6-axis → 7-axis → dual-arm → wheeled → humanoid) | the algorithm may not contain *"how many joints"* |
| new end-effector (2-finger → 3 → 5 → suction) | may not contain *"how far open"* as a single scalar |
| cheap hardware (LeKiwi) | no high-precision feedback, no force sensor |
| grab a scurrying toy mouse | must be able to **re-plan at any instant**; latency dominates |
| tidy a living room for 30 min | long tasks must decompose, and mistakes must be recoverable |
| **dodge a punch** | 🔴 must be able to express **"do not touch"** — zero contact points plus a clearance |
| a person nearby | no unpredictable large motions |

🔴 **The dodge row is a real finding**: *wanting to touch* and *wanting to avoid* are the same
constraint with opposite sign, so one solver covers grasping and evasion — not two.

## Three solvers, one language

| tier | rate | job |
|---|---|---|
| slow | s–min | what the world should become → a sequence of contact sets |
| middle | 10–100 ms | given a contact set, how the joints move |
| fast | ~1 ms | did it touch, did it slip → fix in place |

All three speak contact sets. This layer answers *what this body is* alongside them, **and refuses
when it does not know.**

## Probing the world: three questions, all through channels that already exist

No force sensor required.

| question | how it is asked | which number is read |
|---|---|---|
| how wide is it there | close gently until it stops | jaw reading × jaw span |
| how heavy | lift 3 cm | commanded travel vs achieved travel |
| does it slip | same lift | **do the jaws keep closing** — measured: held median 0.0049 vs not-held 0.1755, zero overlap |
| is the centre off | same lift | how much it rotated relative to the hand once lifted |

**"Touch it first" is not a hard-coded step — it falls out of the arithmetic.** Roll the plan once
with each unknown parameter at its pessimistic end; if the pessimistic roll cannot succeed, probing
is worth its cost, otherwise act now. Second time the same object is seen, its table is still there
and nothing is probed. *(This supersedes the earlier `+ λ·unknown` cost term, which needed a
hand-picked λ.)*

## Home pose belongs to ①, and not because a benchmark asks for it

`all_robot_back_to_origin` is enforced in **30 of 42** RoboDojo tasks, but that is evidence, not the
reason. The reason:

> **Every action must end with the body in a known, repeatable configuration, or the next action
> starts from an unknown one — this is the precondition for actions to compose at all.**

Which arm's home it is → **①**. Whether it is home now → **①**. How to get back without sweeping
things off the table → **②b** (lift straight up first; skimming the surface drags objects).

**The tolerance is measured, never chosen**: homing once yields a pose with no spread, and *"am I
home"* is entirely a question about spread. Refuse when asked for a tolerance tighter than the body's
own repeatability, rather than answering "not home" forever. Measured here: repeatability **0.74 mm**
on synthetic records; **0.000 mm spread over n=11** on this body (2026-08-13).

## 🔴 A driver may not be written against a benchmark

**This layer is going into the world, not onto a leaderboard.** It answers *what is this body like*,
never *how does this benchmark score*.

| | allowed | forbidden |
|---|---|---|
| comments | citing a benchmark as an **example** | citing *"this leaderboard requires X"* as the **reason** a quantity exists |
| code | — | **any** benchmark / task / scene name |

Second clause: **no Python anywhere in the driver tree.** Once `bl` is a process (one line in, one
line out), the 725-line ctypes shell has no reason to exist and its presence made *"the driver"* mean
one Rust program plus a Python file drifting behind it. Deleted.

**A documentation rule only counts once it is a check that can fail** ⇒ `body-layer/check_purity.sh`,
run before每次提交: ① no benchmark names in `body-layer/slow/src`, `body-layer/contact-gen/src` or `body-layer/contact-exec/src` after stripping
comments; ② no `*.py` in the driver tree. Non-zero exit on either.

## What must genuinely be learned (everything else is measured or computed)

| item | why it is not computable | evidence |
|---|---|---|
| **scooping to a target mass** | a missing scalar, not insufficient precision: vision gives volume, volume→mass needs density, density needs force/weighing/acoustics | every work reporting gram-level accuracy reads it from a forbidden channel; pure-vision works report volume or fill fraction |
| **in-hand manipulation with many fingers** | published solutions are RL-only and cover single-axis continuous rotation; *rotate to a specified angle* is unsolved. ⚠️ the premise matters: that claim holds **without a model, without touch, without depth** | — |
| **deformables / granular / fluid** | no object frame and no finite contact set ⇒ the interface degenerates into a field, and the field's evolution is exactly what must be learned | folding has a closed form (G-fold, 50/50 on real towels); **flattening has only learned solutions** |
| **semantics** | force closure says where it is stable, never where it is useful (do not grasp a blade; pour by the handle) | — |
| **object priors** (μ / mass / fragility) | no direct measurement without force; geometry cannot imply them | initialised by the VLM, narrowed by interaction |

🟢 **Two-finger regrasping is the most complete non-learned region** — using external contact or
gravity to regrasp, flipping against a surface, putting down and re-taking — verified on real robots
without force, depth or touch sensing. The one gap is local contact geometry, and the three routes to
that are each published separately; **nobody has joined the two halves.**

## Honest limits

1. **Contact-set planning stops scaling as points multiply.** Classical results cover a few simple
   geometries; a five-finger hand has many contacts.
2. **Computed correct ≠ holds.** Force closure and ε-metrics destabilise within seconds when real
   friction differs from the assumption or a lateral disturbance arrives, and ε predicts robustness
   to pose error poorly.
3. **Global physical parameters cannot be learned from feedback** (measured on real hardware) — the
   signal is too weak. Feedback can adjust the moment of first contact, nothing more.
4. **"What humans do" here is inferred, not measured** — the 76.03% teleoperation figure is from a
   paper; it has never been run on this rig.
5. **Nobody has reported 76% averaged over 42 tasks.**
6. **Multi-step tasks multiply**: 38% per step ⇒ stacking (2 steps) 14%, packing (4 steps) 2%.
   **Below ~90% per step, matching a human teleoperator is arithmetically impossible.**
7. **Modularity's own cost**: six links at 0.7–0.9 each ⇒ 20–40% end to end. **"Retry on failure" is
   therefore not optional** — it is the only thing that pulls the product back up.
8. 🔴 **Covering 99% and minimising what is learned are in direct opposition.** Rigid bodies and
   articulated objects are almost entirely computable; the remaining ~30% (cloth, powder, liquid) is
   almost entirely learned. Engineering cannot dissolve this — one side has to be chosen.

## Lessons paid for in GPU time (2026-08-13)

Each of these produced a run that had to be thrown away.

| lesson | fingerprint it showed | fix |
|---|---|---|
| 🔴 **an unavailable value must default to the UNFAVOURABLE side** | `held = jaw_width < 0.9·(thickness or 1e9)` ⇒ with no material `0.0803 < 9e8` is always true ⇒ believed it was holding from step one; `touched_any` **26/26 = 100%** while object displacement was **0.000 m** | never fall to the favourable side. This layer's *refuse* discipline held; the layer above broke it |
| 🔴 **the same gate, second form** | closing to *a computed width* reached the command **exactly** (3.10 cm commanded, 3.10 cm reached) — i.e. it closed on air — and the old test still scored it as held: **10/11 by my record, 0/8 official** | **close until blocked and read where it stopped.** Independent of any width estimate and of the wrist convention |
| **feed all four slots of the contact set, or the cost is flat** | cost from slot ③ alone ⇒ nothing before contact changes anything ⇒ every candidate ties: 12/12 steps hit the search cap, displacement all 0.0000, held 0/12 | — |
| 🔴 **tolerance is per contact point, not per plan** (slot ④, defined and never used) | rendered frames show the jaws **already parked over the object, open, straddling it**, while the code re-planned forever because the *hover* waypoint was 5 cm off | millimetres where it touches; centimetres where it merely approaches |
| 🔴 **ask this layer before measuring anything yourself** | a whole run spent measuring "cycles of dead time" (`latency` = **0**, already stored) and another measuring per-cycle travel (`step_delivery` = **0.9999**, already stored) | `bl list` first. 14 quantities are stored; the glue was asking for 6 |
| 🔴 **do not rebuild what exists** | a Python grasp-candidate generator was written from scratch, re-deriving three rules `contact-gen` already encodes — and **violating** the *never delete a wide candidate* rule by truncating to the top 6 | `cg` is a process; call it |
| **a silent fallback is worse than a crash** | an asset path with the wrong case, swallowed by `except: return None`, left the point cloud **empty for five consecutive runs** while the code silently used a degenerate fallback that looks exactly like a poorly-performing feature | log which branch was taken, every episode |
| **ask the simulator for geometry; do not guess file paths** | as above | read the mesh from the live stage — it also removes the pose transform |

## Two stored quantities that are not trustworthy today

| quantity | reading | verdict |
|---|---|---|
| `backlash` | **refuses**: *"samples imply mutually inconsistent answers"* | 🟢 the refusal is correct — leave it refusing, do not invent a value |
| `floor` | **0.9208** while `home` z is **0.9215** — 0.7 mm apart | 🔴 degenerate: the press-down probe never descended. Must be re-measured against an actual support surface before anything uses it |

## What is missing and is blocking today

**`reach` answers a radial band `[0.13425, 0.6019]`** — validated on 2174 real episodes, and still
unable to answer *"wrist down, at this xy, descending to this z — can I hold that pose?"* Every
failure in the 2026-08-13 runs landed there: the flange residual and the tool-point residual were
**identical (0.0482 m)**, which rules out a tool-offset error and leaves *the pose is outside the
workspace*. Until that slot exists, the layer above has to ask by acting — which is a measurement,
not a hand-filled constant, but it belongs here.

## Verification, fixed before the numbers exist

Ordered by *cheapest thing that can kill the idea*, not by *most impressive*. Any shot that fails
stops the sequence.

| shot | question | target | 🟢 win | 🔴 lose |
|---|---|---|---|---|
| **1 · expressiveness** | can the new interface state an action the old one structurally cannot | twisting a bottle cap — **0/16** on the old interface, named cause *"orientation is not in the control law's fixed point"* | official **> 0/16**, no regression elsewhere | still 0 ⇒ the derivation is wrong; the cut is not there |
| **2 · coverage** | is a whole class bought, or one task | the 18 RoboDojo tasks needing a final object **orientation**, now **1/14** | **≥7/18** | unchanged ⇒ expressiveness was not the bottleneck |
| **3 · cross-body** | does a kinematically different body run with **not one byte** of upper-layer change | ARX X5 (6-axis) + Franka (7-axis), both already on the rig | Franka absolute successes **≥70%** of X5's | needs an upper-layer edit ⇒ name the body quantity still leaking through |

**Instrumentation gates** (any one failing ⇒ the run does not count): `body_const_source` must read
`body_layer(...)` · `contact_thr_from` must read `body_layer(contact_threshold)` · `touched_any` must
contain a real True — **and must be read together with displacement**; held-without-movement is a
false gate, by construction.

**Accounting**: official `_result.json` `success` only · headline is **N attempts, M successes**,
percentages descriptive only · a temporary drop during a rewrite is expected and is not a loss.

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

---

# thin OS — memory

Merged in here (owner 2026-08-11: *"os layer 不要独立成项目,合并进 body driver 的文件夹"*). It is a
sibling of the body layer in the architecture and a subdirectory of it on disk.

⚠️ **`SYSTEM.md` no longer exists.** It was deleted by `eebd68fd` *"删除 SYSTEM.md 与
EMBODIMENTS.md(owner 指令)"* from `universal-grounding/results/system_aug2026/`. Any instruction
that says "read the root SYSTEM.md" is stale; the memory design lives here.

---

# THE DESIGN, stated once

## Memory is classified by HOW FAST IT GOES STALE, and each rung already has an owner

| rung | example | dies when | owner |
|---|---|---|---|
| **this frame** | where the moving cup is right now | next frame | **not stored** — look again |
| **this task** | what I am doing, what I have done, what I am waiting for | the task ends | **thin OS** |
| **this place** | the bin is in that corner; the sofa faces the window | you leave the place | **thin OS — 任务记忆已建(`os_layer.py`,529 行,`ThinOS`+`SlotOS`);地点记忆全仓 grep 无实现** |
| **this body** | the fingertip is 0.1451 m from the flange | the robot or tool changes | **body layer — built** |
| **the world** | knives are held by the handle | never | **the weights** |

This is not a claim to handle everything. It is a partition with a named owner per rung, and the

🔴 **Rung 1's rule is scene-specific and must not be generalised.** "Never store a position" is
right on a conveyor, where everything moves. In a room it is backwards: the sofa, the bin and the
table do not move, and their positions are the most durable facts in the task. The general form is
**classify by whether the thing moves by itself**, not "positions are forbidden".

## What starts a NEW memory: three independent events, never a timer

| event | opens | keeps |
|---|---|---|
| a new task is given | a new TASK memory | place memory (same room) · body |
| the place is not recognised | a new PLACE memory | task memory (walking through a door does not change the goal) · body |
| the body or tool changes | a new body calibration | both of the above |

The current implementation collapses all three into "one episode, wipe everything", which is why
walking out of a room would also forget the errand.

## 🔴 The place key is to a room what the body fingerprint is to a robot

`bl_body` already solved this exact problem: *one calibration per BODY, stored under a fingerprint
computed from what the body itself reports, containing no benchmark name and no task name.*

Place memory takes the same shape: **one memory per PLACE, keyed by a fingerprint computed from
what the place itself looks like.**

- return to the same room → the key matches → carry on with the map you had
- a room never seen → no match → open an empty one and fill it as you go
- outdoors, everything new → every place is a new key → no history, which is correct

🔴 **And it must be able to refuse.** *"I do not know whether I have been here"* is a legitimate
third answer, and **misidentifying a place is worse than having no memory at all** — you would go
looking for a bin in a corner that has none. Same discipline as the body layer: measure, and refuse
when the measurement does not separate.

## Every stored fact carries WHEN IT WAS LAST CONFIRMED

Maps go stale — somebody moves the bin. So each `(what, where)` records the last time it was seen
with one's own eyes, and:

- the longer since confirmation, the more it must be re-checked before being acted on
- checked and wrong ⇒ rewrite in place **and record that this spot changes**, so it is re-checked
  more often thereafter

## Retrieval, not recitation

Dozens of objects cannot all go in the prompt. Place memory has to be **queryable** — "what do I
know about the bin?" — rather than dumped. At that point it is a small database, and **this is the
part of the design I have not worked out.**

# thin OS

`universal-grounding/README.md` 当初给它的定位:

> 记忆 / 多步规划归**薄 OS 那一层(与 body layer 并列,不进动作模型)**
> 记住顺序 / 多步规划 | 不归这个接口,归薄 OS ——(owner 订正 2026-08-06:VLM 天生就会记事,给它一层薄 OS 即可)

Confirmed absent before writing: all nineteen `bl_*` entry points in `body-layer/abi/body_layer.h`
are measurement, execution or debt. `bl_save`/`bl_load` serialise a CALIBRATION, not an episode.
Nothing in the body layer remembers anything that happened.

| layer | owns | does NOT own |
|---|---|---|
| **thin OS** (here) | memory · which one · when · multi-step order | anything about the robot |
| body layer | measured body constants, refusals | anything that happened |
| weights | **primitives only** | memory, timing, referent choice |

## What it is

A **compacting context**, not a memory store (owner 2026-08-11: *"我们的记忆不是长期记忆吧,是类似
于 claude code 的上下文 compact 这种"*). Shape taken from `claude-code/src/services/compact/`:

- `microCompact.ts` → old turns keep their one-line note and lose their raw reply. No model call.
- `compact.ts` + `autoCompact.ts` → at a threshold, one model call rewrites the notes into a fixed
  brief and the notes are dropped.

🔴 **The mechanism transfers; the REASON does not, and that changes what to keep.** Claude Code
compacts because the window fills, and text does not expire — a file read 200 turns ago is probably
still true. Here an episode is 10–40 frames and never fills anything, while *"frame 3: a green
object is on the belt"* was true when written and describes something that has since driven away.
**We compact for freshness, they compact for size.** Two consequences, both live in the code:

1. Our memory holds two kinds of thing and they must not be treated alike — **perishable** (where
   something is right now: never remember it, look again) and **durable** (what the first object
   looked like; whether we have already grasped; which phase we are in: never expires within an
   episode).
2. **Our forgetting is unrecoverable.** Claude Code can re-read the file; we cannot re-see an
   object that has left the frame. Anything durable has to survive compaction by name, not by a
   summariser's judgement.

*(Both of those consequences are why the **slot** strategy below is the default: it is the same
idea with the durable facts pinned by name and the perishable ones given nowhere to live. `notes`
is kept so the two can be compared on identical episodes rather than argued about.)*

## Why abstention is the whole point

Measured on RoboDojo `match_and_pick_from_conveyor`, 5 episodes / 4 legs / 4 nouns / 4 layouts,
unanimous: the old noun→pixel interface answered **22.8–50.5 px from the WRONG same-category
instance** and **732–996 px from the right one**, in a 640 px frame — the referent was off-screen
and the interface had no way to say so. Every one of those answers was confident.

**An interface that cannot abstain turns "I cannot see it" into "it is over there."** No model
quality fixes that. So `/ask` exists and `NOT_YET` is a first-class reply.

Welded consequences of that, in `os_layer.py`:

- an abstention **may not carry a point** — `/ask` runs the point parser on every reply, so a
  NOT_YET that happens to contain a bracketed pair still arrives with a `uv` (observed:
  `status=NOT_YET, uv=[0.0, 0.0]`). The status decides; the discarded point is kept in
  `server_uv_ignored` rather than dropped silently.
- **"unreadable reply" and "not yet" are counted separately.** One says the world is not ready, the
  other says the instrument is not, and they have opposite next moves.
- a compaction that fails to parse **does not drop the notes it was meant to replace** — that would
  lose the memory and read as the eye having forgotten.
- 🔴 this layer never reads a pose. It sees frames and its own notes. Ground truth in here would be
  an oracle wearing a memory's clothes and the measurement would be circular.

## Install

`os_layer.py` talks to an eye server over `POST /ask` (free-form prompt + frame → reply). Add that
endpoint to an existing `eye_server.py` with:

```
python patch_eye_ask.py          # idempotent; /point is untouched
```

Then restart the server **by port owner, not by pidfile** — a pidfile-based restart silently left
the old code serving while `/health` looked fine, and the only symptom was a 404 on the new route:

```
pid=$(ss -lptnH "sport = :48611" | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2); kill -9 $pid
```

## Use

```python
import os_layer
OS = os_layer.make_os(task_sentence)        # L3_OS_KIND=slot (default) | notes
ans = OS.observe(rgb, k)                    # once per look
if ans["ready"]:
    drive_to_pixel(ans["uv"])               # uv is in PIXELS of this frame
else:
    step_the_world()                        # ans["why"] in {not_yet, no_point, unparsed}
```

`OS.summary()` → `os_kind` · `os_calls` · `os_log`, plus per-strategy: slot →
`os_slots` / `os_pinned` / `os_changes`; notes → `os_compactions` / `os_turns_live` / `os_brief`.

Env: `L3_EYE_URL` · `L3_OS_KEEP_RAW` (turns keeping raw text, default 3) · `L3_OS_COMPACT_AT`
(turns before a rewrite, default 10).

## 🔴 Scope: per-episode vs per-session

An OS instance holds ONE episode. On a benchmark, carrying memory across episodes is a leak that looks
like skill, so the caller constructs a new one per episode.

**On a real robot the opposite is wanted** — "the object from last time" spans commands. Keep one
instance alive for the whole session and call `observe()` across commands. Nothing in the class
needs to change; it is the caller's choice, and it must be a deliberate one.

## LeKiwi test (owner's recipe, 2026-08-11)

1. tell it to move to an object
2. shuffle the objects
3. say **"移动到上次的物体上"**

🔴 **Give that sentence verbatim.** Do not let a coding assistant paraphrase it into "move to the
red cup" — the whole thing being tested is whether the layer resolves *"上次的"* from its own
memory. A translated instruction tests nothing, and it will still look like it passed.

Keep ONE OS instance across all three steps (per the scope note above), and record `os_log` — the
per-frame notes are the evidence for what it actually remembered, as distinct from what it guessed.

## Two strategies, same contract

`make_os()` returns one of two, both exposing the same `observe()`, so they can be A/B'd on
identical episodes instead of argued about. `L3_OS_KIND=slot` (default) or `notes`.

### `notes` — the Claude Code shape

Keep a note per frame; at a threshold spend one model call rewriting them into a brief. Correct
when the reason to compress is SIZE.

### `slot` — recommended here

Five named slots the eye rewrites every frame, riding the observation:

| slot | |
|---|---|
| `first_object` | what was seen first, described well enough to recognise again. **Write-once.** |
| `phase` | `observing` / `waiting` / `target_visible` / `done` |
| `already_done` | what has been accomplished or missed |
| `other` | escape hatch for a fact we did not name |
| `status` + `point_2d` | this frame only — **never stored** |

Three properties the notes shape cannot give:

1. **No blind window.** Compaction costs one 5 s inference during which the robot is not looking;
   on a 10–40 frame episode that is a large fraction of the episode. The slot rewrite is free.
2. **Durable facts are pinned by name**, not by whether a summariser happened to carry them. Our
   forgetting is unrecoverable — the object has left the frame.
3. 🔴 **Perishable facts have nowhere to be written.** There is no slot for "where it is now". A
   position exists only as `point_2d` for the current frame and is consumed immediately. Driving on
   a stale position stops being a rule to obey and becomes unrepresentable.

🔴🔴 **AND THAT THIRD RULE IS SCENE-SPECIFIC, NOT UNIVERSAL — do not copy it forward.** It is right
here because a conveyor moves everything: a position written down is wrong seconds later. In a room
it is **backwards**. The sofa does not move, the bin does not move, the table does not move, and
their positions are the most durable facts in the task — a design that structurally forbids storing
them cannot tidy a living room. The correct general form is not "never store positions" but
**"classify memory by whether the thing moves by itself"**: a belt object's position is perishable,
a fixture's position is durable, and the robot's own pose is somewhere in between (durable for
seconds, re-measurable on demand). Written down because the next build will be tempted to reuse
this file's rule verbatim, and it is exactly the shape of mistake this repo keeps paying for — a
lesson derived from one setting, applied where its premise does not hold.

Cost, stated: fixed slots are less general than prose. `other` is the seam to watch if this ever
loses to `notes`.

### 🔴 The pin rule, and the version of it that was wrong

`first_object` pins on the frame AFTER it is first written — mechanical, and nothing the model does
can affect it. By then the eye has seen a different picture, "the first one" is history, and any
later edit would be overwriting a memory it can no longer check against the world.

The first version pinned when `phase` left `observing`. The smoke test showed **the model never
updates `phase`** (it answered `TARGET_VISIBLE` with `phase` still `observing`), so `pinned` stayed
empty for the whole run: **a guard that only fires when the thing it guards against cooperates is
not a guard.** Recorded because the failure was invisible — every other field looked right.

### Smoke test (3 synthetic frames: red → green → red)

```
k=0  status=NOT_YET         first_object='red square'   pinned=[]
k=1  status=NOT_YET         first_object='red square'   pinned=['first_object']
k=2  status=TARGET_VISIBLE  first_object='red square'   pinned=['first_object']
```

It recorded the first object, abstained while a different one passed, and answered only when the
match returned — from its own memory, with no ground truth anywhere in the path. A refused
overwrite of a pinned slot is recorded in `os_changes` as `REFUSED`, never dropped silently.

## Measured, first readings

- `NOT_YET` on **11 of 83** asks on real conveyor frames (13%). Before abstention existed this rate
  was structurally 0%.
- On frame 1 of an episode the `notes` version still answered `TARGET_VISIBLE` and pointed at the
  first object, where the task wants "not yet, I am still remembering". That is this layer's own
  error rate, now countable, and it is the first thing the `slot` A/B should move.

## Still open

**Slot A/B on real episodes.** The smoke test is synthetic. `notes` vs `slot` on the same conveyor
layouts, comparing abstention rate, wrong-instance rate and official score, is not run yet.

## Scale: what this does NOT reach yet

Asked directly (owner, 2026-08-11) whether this supports a **30-minute one-shot living-room tidy**.
It does not, in three separate ways, and only one of them is a matter of tuning.

| | this layer today | a 30-min tidy |
|---|---|---|
| **throughput** | one look ≈ 5 s wall-clock | 1800 s ⇒ **≤360 looks total**, for dozens of objects, and the world does not wait |
| **capacity** | ONE remembered object; `already_done` is one free-text string | dozens of objects, each with a destination; progress; what is left |
| **failure memory** | none | "that mug slipped twice, change the grasp" |

The throughput number is arithmetic, not opinion: at 5 s per look the eye can only ever be the
**slow** loop — deciding what to do next — while a tracker (LAB: measured at **146.4 Hz**) and the
body layer close the fast loop. That split is already the architecture; it has never been run on a
long task.

What does carry over: the **abstention contract** (a long task needs "I have not been to that room
yet" and "I cannot reach that" far more than a short one), **pinning by name** (over 30 minutes the
commonest failure is a later observation quietly overwriting an earlier one), and **look-rate
follows the world** (which is what makes real-time possible at all).
