//! # body layer — the slow face
//!
//! > **world model + body layer**
//! > **世界模型 + 身体层**
//! > **世界靠学，身体靠量。**
//!
//! Whatever belongs to **this body** is measured at power-on and kept measured; it never enters
//! the weights. Whatever belongs to **the world** is learned. This crate is the first half.
//!
//! ## The invariant, stated once
//!
//! The nearest ancestor is UP-OSI (Yu/Tan/Liu/Turk, RSS 2017): a universal policy plus online
//! identification of body parameters. The single difference carries the whole claim:
//!
//! > UP-OSI feeds the measured body parameters **into the policy**. Here they go **only to the
//! > execution layer — the policy's input contains no body parameter at all.**
//!
//! "Measure the body, then hand it to the policy" is the field's reflex and it *looks* compliant.
//! But once a body parameter is in the policy's input distribution, swapping the body degrades
//! **quietly** instead of failing loudly. The enforcement is therefore structural: see
//! `../../abi/body_layer.h`, where `bl_policy_in` simply has no member such a parameter could
//! arrive through.
//!
//! ## What this crate must do, and how it is checked
//!
//! * Hold each measured quantity with **value, uncertainty, probed range, timestamp, dependency
//!   list, self-test, and the version it replaced** — see [`measurement::Measurement`]. A bare
//!   `f64` cannot be refused on.
//! * **Refuse.** A layer that cannot say REFUSE is not a body layer — see [`refuse`].
//! * Re-measure the hand **every control step**, and abstain rather than guess — see [`hand`].
//!
//! ## The admission test for the whole category
//!
//! Put it on a body with **different kinematics**, give it a fresh body layer, retrain **nothing**,
//! and see whether it still works. Not "under 200 demonstrations". Zero.
//!
//! And the other half of the same test, which the field's prompt-written body descriptions cannot
//! pass: **change a hardware condition — add a weight, loosen a joint, knock the camera — and see
//! whether the system NOTICES.** If it cannot notice, this layer does not exist in that system.

// 🔴 MEASURED 2026-08-09, AND IT DOES NOT HOLD: `cargo build --no-default-features` FAILS, with 26
// errors. It failed before this line was audited too -- `git show HEAD:slow/src/probe.rs` already
// used `sort_by` (which lives in `alloc`) and `sqrt`/`hypot`/`powi` (which live in `std`, not
// `core`). So the attribute below, the Cargo comment claiming this "builds the same code for an
// embedded target", and the README's "a layer that cannot build for the target it has to run on is
// not a deliverable" have all been describing something that has never compiled.
//
// It is recorded here, at the attribute, rather than in a note somebody has to find, because this
// is the repository's most expensive recurring bug: a promise kept by a docstring and by no code,
// indistinguishable in the output from a promise that is kept. Fixing it is a decision, not a
// tidy-up -- the float methods need either a dependency (`libm`, against the zero-dependency rule
// that is itself load-bearing here) or a hand-written shim that then needs its own proof.
#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]

// 🔴 NO `alloc`, NO allocator, NO dynamic memory anywhere in this crate.
//
// An earlier draft put the body behind a `Box` for the opaque C handle.  That is one allocation,
// and one allocation is one too many: it drags in a global allocator, which a hard-real-time
// safety layer must not have, and it makes the crate unbuildable on exactly the targets it has to
// run on.  The storage is supplied by the caller instead -- see `bl_sizeof_body` / `bl_init`.
// Everything else was already fixed-size by construction.

/// A panic inside the body layer is a fault, never a recoverable condition: this layer's whole job
/// is to answer "may this body do this", and a layer that is mid-panic cannot answer.
///
/// It spins rather than returning, and that is deliberate: the fast face's watchdog observes the
/// silence and latches a halt, which is the correct outcome.  Returning would let a broken slow
/// face keep issuing answers.
#[cfg(not(feature = "std"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

pub mod abi;
/// 🔴 `std` only, and that boundary is deliberate: reading frames needs a heap sized by the image,
/// while everything a robot must run in its own control loop stays allocation-free. Vision runs on
/// the compute unit; the refusals run on the arm.
#[cfg(feature = "std")]
pub mod blob;
pub mod debt;
pub mod derive;
pub mod execute;
pub mod floor;
#[cfg(feature = "fast")]
pub mod fast;
#[cfg(not(feature = "fast"))]
pub mod faststub;
pub mod hand;
/// 问眼一句「那个名词在画面哪一点」。**眼只准回五个格子**(归一化 u/v · 区域占比 ·
/// 动词 · 粗略力度),用结构化解码锁住 —— 裸问时它会把距离/位姿**编**出来,而编出来的
/// 数在下游和量出来的完全无法区分。
pub mod eye;
pub mod measurement;
pub mod memory;
pub mod persist;
pub mod predict;
pub mod probe;
pub mod refuse;
pub mod schedule;
pub mod touch;
pub mod json;
pub mod store;
/// 画面一点 ↔ 桌面一点。机器人挥自己的手把自己的相机标了,不是填进去的相机参数。
pub mod tabletop;
pub mod verb;

use measurement::{Malformed, Measurement, Quantity};
use refuse::{Ask, Verdict};

/// One physical robot's measured self-knowledge.
///
/// `Body` is deliberately dumb about *how* anything is measured — that is the caller's job, and it
/// differs per rig. What `Body` owns is the part that must be identical on every rig: the
/// provenance, the expiry rules, and the refusal.
#[derive(Clone, Debug)]
pub struct Body {
    slots: [Option<Measurement>; Quantity::COUNT],
    next_epoch: u64,
}

impl Default for Body {
    fn default() -> Self {
        Self::new()
    }
}

/// 新的这一行是不是**每一项都比已存的差** —— 不确定度更大 **且** 量过的地更小。
///
/// 只在两者同时成立时为真。任一项更好都不算 —— 换了身体重标,覆盖的地通常仍可比,
/// 而"又更不准、又量得更少"只有一种读法:这一次采得更差。
fn worse_on_every_axis(new: &Measurement, old: &Measurement) -> bool {
    let dim = new.dim.min(old.dim);
    if dim == 0 {
        return false;
    }
    let mut sigma_worse = false;
    for i in 0..dim {
        if new.uncertainty[i] < old.uncertainty[i] {
            return false; // 有一维更准 ⇒ 不算全面更差
        }
        if new.uncertainty[i] > old.uncertainty[i] {
            sigma_worse = true;
        }
    }
    let mut box_smaller = false;
    for i in 0..dim {
        let (n_lo, n_hi) = (new.valid_lo[i], new.valid_hi[i]);
        let (o_lo, o_hi) = (old.valid_lo[i], old.valid_hi[i]);
        if n_lo < o_lo - 1e-12 || n_hi > o_hi + 1e-12 {
            return false; // 有一维量得更宽 ⇒ 不算全面更差
        }
        if n_lo > o_lo + 1e-12 || n_hi < o_hi - 1e-12 {
            box_smaller = true;
        }
    }
    sigma_worse && box_smaller
}

impl Body {
    /// A body that knows nothing about itself yet.
    ///
    /// Every ask against it is refused with `NeverMeasured`. That is the correct default: a body
    /// layer whose default is "permit" permits whenever somebody forgets to configure it, and
    /// forgetting is the normal case.
    pub fn new() -> Self {
        Body {
            slots: [None; Quantity::COUNT],
            // Epoch 0 is reserved for "no previous version", so real epochs start at 1.
            next_epoch: 1,
        }
    }

    /// Record something the robot measured about itself.
    ///
    /// Rejects a malformed submission outright — see [`Malformed`]. There is deliberately **no**
    /// `force` or `trust_me` path: a body layer that can be told to accept an unverified constant
    /// is a configuration file with extra steps, and one such constant (a gripper bias of `0.145`
    /// whose provenance could not be traced) is exactly what made a previous "zero-shot on a new
    /// body" claim untrue.
    ///
    /// On success the measurement is stamped with a fresh epoch and remembers the one it replaced,
    /// so a consumer can always answer "is what I was measured against still the thing that is
    /// there now".
    pub fn submit(&mut self, mut m: Measurement) -> Result<u64, Malformed> {
        let known = |q: Quantity| self.slots[q as usize].is_some();
        m.validate(&known)?;

        let idx = m.quantity as usize;
        // 🔴 不许用一个【每一项都更差】的行覆盖已存的行。2026-08-13 实测发生过:
        //    54 格 / 残差 0.26 mm / 域 x[-0.60,+0.60] 的地板,被一份 9 格 / 残差 7.5 mm /
        //    域窄 0.3 m 的覆盖掉,全程无人反对;好的那份只能靠原始 rollout 还没删才重建回来。
        //    换了身体当然该重标,但那种情况下**不会两项同时更差** —— 重标一条挪过位置的臂,
        //    覆盖的地仍然可比。**又更不准、又量得更少**,只有一种读法:这一次采得更差。
        if let Some(old) = self.slots[idx] {
            if worse_on_every_axis(&m, &old) {
                return Err(Malformed::WorseThanStored);
            }
        }
        m.prev_epoch = self.slots[idx].map_or(0, |old| old.epoch);
        m.epoch = self.next_epoch;
        self.next_epoch += 1;
        self.slots[idx] = Some(m);
        Ok(m.epoch)
    }

    /// Read the current measurement for a quantity, if there is one.
    pub fn get(&self, q: Quantity) -> Option<Measurement> {
        self.slots[q as usize]
    }

    /// The gate. See [`refuse::admit`] for the ordering of checks and why it is that order.
    pub fn admit(&self, ask: &Ask, now_ns: u64) -> Verdict {
        refuse::admit(ask, now_ns, &|q| self.get(q))
    }

    /// Bitmask of quantities whose self-test currently passes.
    ///
    /// 🔴 The conformance suite feeds every self-test an input that **must** make it fail, and
    /// fails the build if any of them passes it. A guard that has never failed has never been
    /// tested, and in the output it is indistinguishable from a guard that does not exist.
    pub fn selftest_mask(&self) -> u64 {
        let mut mask = 0u64;
        for (i, slot) in self.slots.iter().enumerate() {
            if let Some(m) = slot {
                if m.selftest_passed {
                    mask |= 1u64 << i;
                }
            }
        }
        mask
    }

    /// Which quantities have never been measured on this body. The honest answer to "am I ready".
    pub fn missing(&self) -> impl Iterator<Item = Quantity> + '_ {
        (0..Quantity::COUNT).filter_map(move |i| {
            if self.slots[i].is_none() {
                Quantity::from_u32(i as u32)
            } else {
                None
            }
        })
    }

    /// How many hand-filled constants **entered through this API**: a structural zero.
    ///
    /// 🔴 Read the scope, and never quote this number alone. Nothing can enter through
    /// [`Body::submit`] without a passing self-test, so this counts a set that is empty by
    /// construction. The constants that never came near this API are invisible to it — and on
    /// 2026-08-09 a parameter search found that the single most influential constant on the
    /// deployed stack (`TEACH_HIGH_FRAC`: 32/44 at ≤0.30 against 10/100 above it, p=9.3e-14) was
    /// one this layer had never heard of, alongside 45 environment knobs and a hardcoded camera
    /// matrix against ten declared quantities.
    ///
    /// The honest pair is this **and** [`crate::debt::outstanding`]. A zero here with no second
    /// number is the shape of claim this layer exists to stop other people making.
    pub fn hand_filled_constants(&self) -> usize {
        self.slots
            .iter()
            .flatten()
            .filter(|m| !m.selftest_passed)
            .count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use measurement::{AxisKind, MAX_DEPS, MAX_DIM};

    fn m(q: Quantity, v: f64, sigma: f64, valid_for_ns: u64) -> Measurement {
        let mut x = Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity: q,
            dim: 1,
            value: [0.0; MAX_DIM],
            uncertainty: [0.0; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [0.0; MAX_DIM],
            measured_at_ns: 1_000,
            valid_for_ns,
            deps: [None; MAX_DEPS],
            epoch: 0,
            selftest_passed: true,
            prev_epoch: 0,
        };
        x.value[0] = v;
        x.uncertainty[0] = sigma;
        x.valid_lo[0] = v - 1.0;
        x.valid_hi[0] = v + 1.0;
        x
    }

    /// 🔴 **每一项都更差的行必须被拒。** 2026-08-13 实测发生过一次:
    /// 54 格 / 残差 0.26 mm / 域 x[-0.60,+0.60] 的地板,被 9 格 / 残差 7.5 mm /
    /// 域窄 0.3 m 的覆盖掉,全程无人反对。这个测试就是那次事故。
    #[test]
    fn worse_on_every_axis_is_refused() {
        let mut b = Body::new();
        let mut good = m(Quantity::Floor, 0.92, 0.00026, 0);
        good.valid_lo[0] = -0.60;
        good.valid_hi[0] = 0.60;
        assert!(b.submit(good).is_ok());

        let mut worse = m(Quantity::Floor, 0.93, 0.0075, 0); // 更不准
        worse.valid_lo[0] = -0.60;
        worse.valid_hi[0] = 0.30; // 而且量得更少
        assert_eq!(b.submit(worse), Err(Malformed::WorseThanStored));
        // 存着的仍然是好的那一份
        assert_eq!(b.get(Quantity::Floor).unwrap().uncertainty[0], 0.00026);
    }

    /// 反面:**只要有一维更好就必须放行** —— 换了身体重标,常常是"没那么准但量得更宽",
    /// 那是合法的重标,拒掉它会把这一层变成一个不能重新标定的层。
    #[test]
    fn worse_sigma_but_wider_box_is_admitted() {
        let mut b = Body::new();
        let mut narrow = m(Quantity::Floor, 0.92, 0.00026, 0);
        narrow.valid_lo[0] = -0.10;
        narrow.valid_hi[0] = 0.10;
        assert!(b.submit(narrow).is_ok());

        let mut wide = m(Quantity::Floor, 0.92, 0.0030, 0); // 更不准
        wide.valid_lo[0] = -0.60;
        wide.valid_hi[0] = 0.60; // 但量得更宽 ⇒ 放行
        assert!(b.submit(wide).is_ok());
        assert_eq!(b.get(Quantity::Floor).unwrap().uncertainty[0], 0.0030);
    }

    fn ask_for(q: Quantity) -> Ask {
        let mut a = Ask::EMPTY;
        a.needs[0] = Some(q);
        a
    }

    /// 🔴 A column index is a LABEL. Encoding one as an interval admits the space between two
    /// labels, and the two artefacts that hit this (`arm_id`, `body`) were both real.
    #[test]
    fn a_categorical_axis_refuses_between_its_labels() {
        let mut b = Body::new();
        let mut x = m(Quantity::ToolAxisColumn, 0.0, 0.0, 0);
        x.axis_kind[0] = AxisKind::Categorical;
        x.valid_lo[0] = 0.0;
        x.valid_hi[0] = 2.0;
        b.submit(x).unwrap();

        let mut a = ask_for(Quantity::ToolAxisColumn);
        for col in [0.0, 1.0, 2.0] {
            a.at[0] = Some(col);
            assert!(b.admit(&a, 0).admit, "column {col} was probed and must be admitted");
        }
        a.at[0] = Some(0.5);
        let v = b.admit(&a, 0);
        assert!(!v.admit, "there is no column 0.5, and an interval domain would have admitted it");
        assert_eq!(v.why, refuse::Reason::OutOfRange);
        a.at[0] = Some(3.0);
        assert!(!b.admit(&a, 0).admit, "column 3 is outside the labels that were probed");
    }

    /// 🔴 The asymmetry, and it is measured (`results/bodylayer_aug2026`): crossing a PROBED axis
    /// is a hard refusal, because positive evidence says the ask is outside. Touching an UNPROBED
    /// axis is soft — there is no evidence either way, and hard-refusing it was tried: one
    /// constant with an unmeasured domain made everything downstream unusable and collapsed a
    /// three-level scale to two.
    #[test]
    fn an_unprobed_axis_admits_unverified_instead_of_refusing() {
        let mut b = Body::new();
        let mut x = m(Quantity::StepDelivery, 0.5, 0.0, 0);
        x.axis_kind[0] = AxisKind::Unmeasured;
        b.submit(x).unwrap();

        let mut a = ask_for(Quantity::StepDelivery);
        a.at[0] = Some(1e6); // absurdly far outside anything anyone would probe
        let v = b.admit(&a, 0);
        assert!(v.admit, "an unprobed axis must not hard-refuse");
        assert!(v.unverified, "... but it must not pass silently either");
        assert_eq!(v.why, refuse::Reason::NoEvidence);
        assert_eq!(v.culprit, Some(Quantity::StepDelivery));
    }

    /// "I could not check" and "I checked and it is wrong" must not collapse into one name: the
    /// caller's response differs, re-probe versus abandon.
    #[test]
    fn no_evidence_is_not_self_test_failed() {
        assert_ne!(refuse::Reason::NoEvidence, refuse::Reason::SelfTestFailed);
        assert_eq!(refuse::Reason::NoEvidence.as_str(), "no_evidence");
    }

    /// A real refusal outranks "nobody checked" — otherwise an ask that is BOTH unverified on one
    /// quantity and stale on another would report the softer of the two.
    #[test]
    fn a_hard_refusal_outranks_an_unverified_axis() {
        let mut b = Body::new();
        let mut x = m(Quantity::StepDelivery, 0.5, 0.0, 0);
        x.axis_kind[0] = AxisKind::Unmeasured;
        b.submit(x).unwrap();
        b.submit(m(Quantity::Latency, 1.0, 0.0, 60)).unwrap();

        let mut a = ask_for(Quantity::StepDelivery);
        a.at[0] = Some(1e6);
        a.needs[1] = Some(Quantity::Latency); // fresh for 60 ns, asked about much later
        let v = b.admit(&a, 10_000_000);
        assert!(!v.admit, "the stale quantity must decide");
        assert_eq!(v.why, refuse::Reason::Stale);
    }

    /// 🔴 THE TEST THAT WOULD HAVE CAUGHT IT: assert the DIRECTION, not the magnitude.
    ///
    /// A probe-produced Jacobian was already fed through `execute` by the end-to-end test, which
    /// then asserted `|cmd| == spec.step_m`. The solve normalises to exactly that, so the check
    /// held no matter which gradients went in -- it was on the one quantity the bug could not
    /// affect. Meanwhile the executor indexed `axis * n_joints + j` over three axes while the probe
    /// writes `2j, 2j+1` over two: different count, different order, every command built from
    /// another joint's numbers.
    ///
    /// This rig makes joint 0 move the image in +u ONLY and joint 1 in +v ONLY. Ask to go +u; if
    /// the layout is right, joint 0 carries it and joint 1 stays put. Under the old indexing joint
    /// 0 would have been driven by value[0], value[6], value[12] -- gradients of joints 3 and 6 --
    /// and the assertion below fails.
    #[test]
    fn the_executor_reads_the_jacobian_the_probe_writes() {
        use probe::Sample;
        const NJ: usize = 6;
        // 🔴 Every consecutive PAIR must differ in exactly one joint, because the probe reads
        // finite differences between neighbours. A first version cycled j = k % NJ, so each window
        // changed two joints at once and the probe -- correctly -- attributed part of the v-motion
        // to joint 0. That was the test rig lying, not the probe.
        let mut samples = Vec::new();
        let mut uv = [0.5f64, 0.5];
        let mut k = 0u64;
        for round in 0..4 {
            for j in 0..NJ {
                // rest: all commands zero
                samples.push(Sample { cmd: [0.0; MAX_DIM], n: NJ, uv, at_ns: k * 1_000_000 });
                k += 1;
                // one joint moves
                let mut sm = Sample { cmd: [0.0; MAX_DIM], n: NJ, uv, at_ns: k * 1_000_000 };
                sm.cmd[j] = 0.01;
                if j == 0 {
                    uv[0] += 0.02;
                } else if j == 1 {
                    uv[1] += 0.02;
                }
                let _ = round;
                sm.uv = uv;
                samples.push(sm);
                k += 1;
            }
        }
        let jac = probe::image_jacobian(&samples, NJ, 1_000_000_000, 1e-4)
            .expect("the probe declined on a clean, separable response");
        // The probe's own layout, asserted here so a future edit to either side breaks THIS test
        // rather than silently re-opening the same gap.
        assert!(jac.value[0].abs() > 0.5, "value[0] must be du/dq0: {:?}", &jac.value[..4]);
        assert!(jac.value[3].abs() > 0.5, "value[3] must be dv/dq1: {:?}", &jac.value[..4]);

        let mut body = Body::new();
        body.submit(jac).unwrap();

        let sp = execute::Spec { step_m: 0.004, period_ms: 40, damping: 0.01, n_joints: NJ };
        // 🔴 The SOLVE is asserted directly. It used to be reachable only through `execute`,
        // which needs the proven Ada face -- absent from the default build -- so the standard test
        // run could not reach the code where the defect lived.
        let sp = execute::Spec { step_m: 0.004, period_ms: 40, damping: 0.01, n_joints: NJ };
        let jm = body.get(measurement::Quantity::ImageJacobian).unwrap();

        for (want_axis, other) in [(0usize, 1usize), (1, 0)] {
            let mut dir = [0.0; 3];
            dir[want_axis] = 1.0;
            let cmd = execute::solve(&jm, &sp, &dir);
            assert!(cmd[want_axis].abs() > 10.0 * cmd[other].abs().max(1e-12),
                    "axis {want_axis}: the wrong joint carried the motion: {:?}", &cmd[..NJ]);
            assert!(cmd[want_axis] > 0.0, "axis {want_axis}: sign is inverted: {:?}", &cmd[..NJ]);
        }
    }

    /// 🔴 Every reason must have a name over the C ABI, and this test exists because two did not.
    ///
    /// `NotYet` was unnamed from the day it was added -- `bl_reason_str` held a second, hand-written
    /// copy of the table that stopped at `RateLimit`. Callers in other languages read "unknown" for
    /// the refusal the header documents most carefully, and the Python binding, which walks the enum
    /// until the first "unknown", silently truncated its whole table at the gap.
    #[test]
    fn every_reason_has_a_name_over_the_abi() {
        use refuse::Reason;
        let mut n = 0;
        for v in 0u32..64 {
            let Some(r) = Reason::from_u32(v) else { continue };
            n += 1;
            let c = r.as_cstr();
            assert!(c.ends_with('\0'), "{v}: the C form must be NUL-terminated");
            assert_ne!(r.as_str(), "unknown", "reason {v} has no name");
            assert_eq!(r.as_str(), &c[..c.len() - 1], "the two forms disagree for {v}");
            // and the exported function must agree with the table it now delegates to
            let p = abi::bl_reason_str(v);
            let got = unsafe { core::ffi::CStr::from_ptr(p) }.to_str().unwrap();
            assert_eq!(got, r.as_str(), "bl_reason_str disagrees for {v}");
        }
        assert!(n >= 11, "only {n} reasons round-tripped; from_u32 is behind the enum");
        let p = abi::bl_reason_str(9999);
        let got = unsafe { core::ffi::CStr::from_ptr(p) }.to_str().unwrap();
        assert_eq!(got, "unknown", "an unknown code must say so, not land on a neighbour");
    }

    /// A fresh body must refuse. If this ever passes, the default became permissive.
    #[test]
    fn empty_body_refuses() {
        let b = Body::new();
        let v = b.admit(&ask_for(Quantity::HandPixel), 0);
        assert!(!v.admit);
        assert_eq!(v.why, refuse::Reason::NeverMeasured);
    }

    /// A measurement whose own self-test failed must not be storable at all.
    #[test]
    fn selftest_failure_cannot_be_stored() {
        let mut b = Body::new();
        let mut bad = m(Quantity::ArmWeight, 1.0, 0.01, 0);
        bad.selftest_passed = false;
        assert_eq!(b.submit(bad), Err(Malformed::SelfTestFailed));
        assert!(b.get(Quantity::ArmWeight).is_none());
    }

    /// Staleness must be caught by the clock.
    #[test]
    fn stale_is_refused() {
        let mut b = Body::new();
        b.submit(m(Quantity::ArmWeight, 1.0, 0.01, 100)).unwrap();
        assert!(b.admit(&ask_for(Quantity::ArmWeight), 1_050).admit);
        let v = b.admit(&ask_for(Quantity::ArmWeight), 5_000);
        assert!(!v.admit);
        assert_eq!(v.why, refuse::Reason::Stale);
    }

    /// 🔴 THE GRASP THAT WAS LOST BEFORE THE DESCENT STARTED, CAUGHT BEFORE IT RUNS.
    ///
    /// Conveyor, 4 episodes, every stage reading healthy: image error converged to 7.2–7.9 px,
    /// contact landed within 9–28 mm of the object's own height, the descent drifted sideways by
    /// 1.8–6.9 mm, the tool offset audited to 0.9–2.8 cm. And the hand finished 17–30 cm from the
    /// object with `obj_dz = 0.000`.
    ///
    /// That gap is not error: it is the close-and-lift spent blind, multiplied by the belt. The
    /// numbers below are that episode, and the assertion is that the layer says so **first**.
    #[test]
    fn a_grasp_lost_to_blind_time_is_refused_before_it_runs() {
        use crate::measurement::{AxisKind, MAX_DIM};
use crate::derive;

        let mut b = Body::new();
        b.submit(m(Quantity::Latency, 0.0, 0.5, 0)).unwrap();
        let mut sd = m(Quantity::StepDelivery, 0.9999, 0.00015, 0);
        sd.valid_lo[0] = 0.005;
        sd.valid_hi[0] = 0.01; // the descent commands 1 cm per period
        b.submit(sd).unwrap();
        let mut g = m(Quantity::GripperSpan, 0.0888, 0.001, 0); // measured off the URDF + STLs
        g.valid_lo[0] = 0.0;
        g.valid_hi[0] = 1.0;
        b.submit(g).unwrap();

        // A still object: the blind time costs nothing, and the drift is zero.
        assert_eq!(derive::blind_drift_m(&b, 0.28, 0.01, 0.0).unwrap(), 0.0);

        // The belt: 4 mm per control period, and a 0.28 m descent. This is the episode.
        let v = derive::blind_drift_m(&b, 0.28, 0.01, 0.004).unwrap_err();
        assert_eq!(
            v.why,
            refuse::Reason::NotYet,
            "a drift wider than the jaws means the object cannot be between them when they close, \
             and the layer must say so before the motion rather than after the episode"
        );
        assert_eq!(v.culprit, Some(Quantity::GripperSpan), "the jaws are the tolerance");

        // A belt slow enough that the object is still between the jaws: admitted, with the number.
        let drift = derive::blind_drift_m(&b, 0.28, 0.01, 0.0002).unwrap();
        assert!(drift > 0.0 && drift < 0.0888, "admitted, and it hands back the drift: {drift}");
    }

    /// 🔴 A `BlockedBy` ROW MUST BE BACKED BY A DERIVATION THAT ACTUALLY REFUSES.
    ///
    /// The standing exists so an integrator is told *"you cannot have this number yet, and here is
    /// the one probe that would give it to you"*. That is only worth anything if the derivation is
    /// real and the named quantity is genuinely the thing it stops on — otherwise `BlockedBy` is a
    /// nicer-sounding `Outstanding`, which is the class of decoration this file was written to
    /// stop.
    #[test]
    fn blocked_by_rows_name_the_probe_that_actually_blocks_them() {
        use crate::derive;

        let blocked: Vec<_> = debt::LEDGER
            .iter()
            .filter_map(|c| match c.standing {
                debt::Standing::BlockedBy(q) => Some((c.name, q)),
                _ => None,
            })
            .collect();
        assert!(!blocked.is_empty(), "no BlockedBy rows: either all discharged, or the standing \
                                      stopped being used and the ledger got quieter than the body");

        // A body with everything EXCEPT gripper_span: the approach derivation must stop on exactly
        // that quantity, not on something incidental.
        let mut b = Body::new();
        b.submit(m(Quantity::Latency, 0.0, 0.5, 0)).unwrap();
        let mut sd = m(Quantity::StepDelivery, 0.9999, 0.00015, 0);
        sd.valid_lo[0] = 0.005;
        sd.valid_hi[0] = 0.05;
        b.submit(sd).unwrap();
        let v = derive::approach_clearance_m(&b).unwrap_err();
        assert_eq!(v.culprit, Some(Quantity::GripperSpan),
                   "the approach rows blame gripper_span; the derivation must stop there too");

        // And the same derivation must SUCCEED the moment that one probe reports, or the row is
        // blaming a quantity that would not actually unblock it.
        let mut g = m(Quantity::GripperSpan, 0.0888, 0.001, 0);
        g.valid_lo[0] = 0.0;
        g.valid_hi[0] = 1.0;
        b.submit(g).unwrap();
        let clearance = derive::approach_clearance_m(&b).expect("unblocked by the named probe");
        assert!((clearance - 0.0444).abs() < 1e-9, "half the measured jaw span, got {clearance}");
    }

    /// 🔴 THE SETTLE BUDGET IS SIZED BY THE ARM, AND THE TWO REAL ARMS PROVE IT MATTERS.
    ///
    /// Measured on this project: same harness, same commanded 45 mm step, one arm delivered 0.76
    /// per control period and the other 0.11. A budget set from the first left the second 0.136 m
    /// short on every episode, reading as a planner or reachability fault. The numbers below are
    /// those two arms.
    #[test]
    fn settle_is_sized_by_this_arms_own_delivery() {
        use crate::derive;

        let fast_arm = {
            let mut b = Body::new();
            b.submit(m(Quantity::Latency, 0.0, 0.5, 0)).unwrap();
            let mut sd = m(Quantity::StepDelivery, 0.76, 0.01, 0);
            sd.valid_lo[0] = 0.005;
            sd.valid_hi[0] = 0.045;
            b.submit(sd).unwrap();
            derive::settle_periods(&b, 0.01).unwrap()
        };
        let slow_arm = {
            let mut b = Body::new();
            b.submit(m(Quantity::Latency, 0.0, 0.5, 0)).unwrap();
            let mut sd = m(Quantity::StepDelivery, 0.11, 0.01, 0);
            sd.valid_lo[0] = 0.005;
            sd.valid_hi[0] = 0.045;
            b.submit(sd).unwrap();
            derive::settle_periods(&b, 0.01).unwrap()
        };
        assert!(
            slow_arm > fast_arm * 5,
            "the 0.11-delivery arm needs far longer to land ({slow_arm} vs {fast_arm}); if these              are close, the budget is being set by something other than the arm"
        );

        // And with nothing measured it refuses rather than guessing a budget that looks sensible.
        let empty = Body::new();
        assert_eq!(
            derive::settle_periods(&empty, 0.01).unwrap_err().why,
            refuse::Reason::NeverMeasured
        );

        // A traverse is divided by what ARRIVES, not by what is commanded -- the same bug wearing
        // a different name.
        let mut b = Body::new();
        b.submit(m(Quantity::Latency, 0.0, 0.5, 0)).unwrap();
        let mut sd = m(Quantity::StepDelivery, 0.5, 0.01, 0);
        sd.valid_lo[0] = 0.005;
        sd.valid_hi[0] = 0.04;
        b.submit(sd).unwrap();
        assert_eq!(
            derive::traverse_steps(&b, 0.4).unwrap(),
            20,
            "0.4 m at 0.04 m commanded x 0.5 delivered is 20 steps, not the 10 a commanded-step              count would report"
        );

        // The jaw clearance refuses on a body whose gripper span was never established -- which is
        // the state of the arm this project is running.
        assert_eq!(
            derive::approach_clearance_m(&b).unwrap_err().why,
            refuse::Reason::NeverMeasured
        );
    }

    /// 🔴 THE LAYER'S OWN TWO CONSTANTS COME FROM THE BODY, OR NOTHING COMES OUT.
    ///
    /// `step_m` and `damping` were passed in by the caller until now, which the ledger recorded as
    /// outstanding debt in the middle of the execution path. A default here would have been a
    /// hand-filled constant wearing a function's name, so the guards are tested by making each one
    /// fire.
    #[test]
    fn spec_is_derived_from_measurements_or_refused() {
        use crate::execute::Spec;

        // nothing measured at all
        let b = Body::new();
        let v = Spec::from_body(&b, 20, 7).unwrap_err();
        assert_eq!(v.why, refuse::Reason::NeverMeasured);

        // A probe that established no domain cannot hand over a step size — and it turns out
        // `submit` already refuses to store one, upstream of `from_body`. Asserted here so the
        // two guards cannot drift apart: if `submit` ever loosens, this fails rather than letting
        // `from_body` hand back a zero step.
        let mut b = Body::new();
        let mut sd = m(Quantity::StepDelivery, 0.9999, 0.00015, 0);
        sd.valid_lo[0] = 0.0;
        sd.valid_hi[0] = 0.0;
        assert!(b.submit(sd).is_err(), "an empty probed range must not be storable");

        // a real domain, but the Jacobian has not been measured
        let mut b = Body::new();
        let mut sd = m(Quantity::StepDelivery, 0.9999, 0.00015, 0);
        sd.valid_lo[0] = 0.005;
        sd.valid_hi[0] = 0.05;
        b.submit(sd).unwrap();
        assert_eq!(
            Spec::from_body(&b, 20, 7).unwrap_err().why,
            refuse::Reason::NeverMeasured
        );

        // both present: the values are the measurements, not anything typed in
        let mut jac = m(Quantity::ImageJacobian, -2.18594, 0.26659, 0);
        jac.valid_lo[0] = -0.02;
        jac.valid_hi[0] = 0.02;
        b.submit(jac).unwrap();
        let spec = Spec::from_body(&b, 20, 7).expect("both measured");
        assert_eq!(spec.step_m, 0.05, "the top of the swept domain, not a rating");
        assert_eq!(spec.damping, 0.26659, "the Jacobian's own worst uncertainty");

        // a Jacobian claiming zero uncertainty is not sharp, it is unestablished
        let mut b2 = Body::new();
        let mut sd2 = m(Quantity::StepDelivery, 0.9999, 0.00015, 0);
        sd2.valid_lo[0] = 0.005;
        sd2.valid_hi[0] = 0.05;
        b2.submit(sd2).unwrap();
        b2.submit(m(Quantity::ImageJacobian, -2.0, 0.0, 0)).unwrap();
        assert_eq!(
            Spec::from_body(&b2, 20, 7).unwrap_err().why,
            refuse::Reason::UncertaintyTooHigh,
            "damping by zero is the tuned-to-succeed choice, and it must not be silent"
        );
    }

    /// 🔴 A REACH REFUSAL MUST SAY "NOT YET", NOT "NEVER".
    ///
    /// The band establishes where this body can act right now. It establishes nothing about where
    /// the world will put the ask a moment later, so answering `Unreachable` would be claiming a
    /// future this measurement does not cover — and a caller that hears "unreachable" gives up on
    /// an object that is merely still on its way. Measured instance: a conveyor object 1.3 m away
    /// at t=0 that passes within 0.416 m by step ~320, band 0.134–0.602 m.
    #[test]
    fn outside_the_band_is_not_yet_and_never_unreachable() {
        let mut b = Body::new();
        let mut r = m(Quantity::Reach, 0.0, 0.01, 0);
        r.dim = 2;
        r.value[0] = 0.134;
        r.value[1] = 0.602;
        r.valid_lo[0] = 0.0;
        r.valid_hi[0] = 1.0;
        r.valid_lo[1] = 0.0;
        r.valid_hi[1] = 1.0;
        b.submit(r).unwrap();

        let mut inside = ask_for(Quantity::Reach);
        inside.reach_radius_m = Some(0.416);
        assert!(b.admit(&inside, 1_000).admit, "0.416 m sits inside 0.134-0.602");

        let mut far = ask_for(Quantity::Reach);
        far.reach_radius_m = Some(1.334);
        let v = b.admit(&far, 1_000);
        assert!(!v.admit);
        assert_eq!(v.why, refuse::Reason::NotYet, "a reach refusal may not claim `never`");
        assert_ne!(v.why, refuse::Reason::Unreachable);

        let mut near = ask_for(Quantity::Reach);
        near.reach_radius_m = Some(0.05);
        assert_eq!(b.admit(&near, 1_000).why, refuse::Reason::NotYet, "inside the inner wall too");
    }

    /// 🔴 The case a wall-clock TTL cannot catch, and the reason `deps` exists: the quantity is
    /// perfectly fresh in time, and invalid anyway, because what it was measured against moved.
    #[test]
    fn dependency_change_invalidates_a_fresh_measurement() {
        let mut b = Body::new();
        let jac_epoch = b.submit(m(Quantity::ImageJacobian, 1.0, 0.01, 0)).unwrap();

        let mut hp = m(Quantity::HandPixel, 0.5, 0.001, 0);
        hp.deps[0] = Some((Quantity::ImageJacobian, jac_epoch));
        b.submit(hp).unwrap();
        assert!(b.admit(&ask_for(Quantity::HandPixel), 1_000).admit);

        // The camera got knocked; the Jacobian is re-measured. The hand point's own clock is
        // untouched and it is now worthless.
        b.submit(m(Quantity::ImageJacobian, 1.2, 0.01, 0)).unwrap();
        let v = b.admit(&ask_for(Quantity::HandPixel), 1_000);
        assert!(!v.admit);
        assert_eq!(v.why, refuse::Reason::DependencyChanged);
    }

    /// Asking for more precision than this body has established about itself must be refused, not
    /// answered with the best available number.
    #[test]
    fn insufficient_precision_is_refused() {
        let mut b = Body::new();
        b.submit(m(Quantity::GripperSpan, 0.08, 0.02, 0)).unwrap();
        let mut a = ask_for(Quantity::GripperSpan);
        a.tolerance[0] = Some(0.001);
        let v = b.admit(&a, 0);
        assert!(!v.admit);
        assert_eq!(v.why, refuse::Reason::UncertaintyTooHigh);
    }

    /// 🔴 An ask outside the domain a quantity was actually probed over must be refused, not
    /// extrapolated. Three probes documented this refusal and, until `Ask::at` existed, **nothing
    /// implemented it** — only `hand_pixel` was ever range-checked. A promise kept by a docstring
    /// and by no code is indistinguishable, in the output, from a promise that is kept.
    #[test]
    fn an_ask_outside_the_probed_domain_is_refused() {
        let mut b = Body::new();
        // step_delivery probed over commanded magnitudes 0.020 .. 0.058 m
        let mut sd = m(Quantity::StepDelivery, 0.11, 0.01, 0);
        sd.valid_lo[0] = 0.020;
        sd.valid_hi[0] = 0.058;
        b.submit(sd).unwrap();

        let mut inside = ask_for(Quantity::StepDelivery);
        inside.at[0] = Some(0.045);
        assert!(b.admit(&inside, 0).admit, "a magnitude that was probed must be admitted");

        let mut outside = ask_for(Quantity::StepDelivery);
        outside.at[0] = Some(0.001); // a 1 mm step: a saturating actuator delivers a different
                                     // fraction of it, and nobody probed there
        let v = b.admit(&outside, 0);
        assert!(!v.admit);
        assert_eq!(v.why, refuse::Reason::OutOfRange);
        assert_eq!(v.culprit, Some(Quantity::StepDelivery));
    }

    /// A body layer that refuses everything is also not a body layer. Kept last and kept small.
    #[test]
    fn a_well_formed_ask_is_admitted() {
        let mut b = Body::new();
        b.submit(m(Quantity::ArmWeight, 1.89, 0.05, 0)).unwrap();
        assert!(b.admit(&ask_for(Quantity::ArmWeight), 10_000_000).admit);
        assert_eq!(b.hand_filled_constants(), 0);
    }

    /// 🔴 The anti-elbow branch. Two rigid things both move with the command and are not separable;
    /// the old rule shipped the nearer one with a 0.04 px self-reported error while the truth was
    /// 167 px. This must abstain.
    #[test]
    fn unseparable_candidates_abstain_instead_of_picking() {
        use hand::{Candidate, Config, HandTracker};
        let mut t = HandTracker::new(Config::default());
        let c = |u: f64, gain: f64| Candidate {
            u,
            v: 0.5,
            gain,
            rigidity: 0.9,
            pixels: 500,
            spread: 0.004,
        };
        // gain ratio 1.11 -- exactly the fingertip/elbow depth ratio that fooled the old selector
        let r = t.observe(&[c(0.3, 1.0), c(0.7, 0.9)]);
        assert_eq!(r, Err(hand::Abstain::NotSeparable));
        let (accepted, abstained) = t.counts();
        assert_eq!((accepted, abstained), (0, 1));
    }

    /// And a clearly separable frame must be accepted, or the tracker is just a refuser.
    #[test]
    fn separable_candidates_are_accepted() {
        use hand::{Candidate, Config, HandTracker};
        let mut t = HandTracker::new(Config::default());
        let c = |u: f64, gain: f64| Candidate {
            u,
            v: 0.5,
            gain,
            rigidity: 0.9,
            pixels: 500,
            spread: 0.004,
        };
        let r = t.observe(&[c(0.3, 1.0), c(0.7, 0.2)]);
        assert_eq!(r, Ok((0.3, 0.5)));
    }
}

#[cfg(test)]
mod execute_tests {
    use super::*;
    use execute::{execute, Intent, Outcome, Spec};
    use measurement::{AxisKind, MAX_DEPS, MAX_DIM};

    fn spec() -> Spec {
        Spec {
            step_m: 0.004,      // the rig's own per-period travel, from its rating
            period_ms: 40,
            damping: 0.05,
            n_joints: 6,
        }
    }

    fn unit_x() -> Intent {
        Intent {
            dir: [1.0, 0.0, 0.0],
            drot: [0.0; 3],
            grip: 1.0,
            base: [0.0; 3],
        }
    }

    fn jacobian(epoch_of_hand: &mut u64, b: &mut Body) {
        let mut j = Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity: Quantity::ImageJacobian,
            dim: 18,
            value: [0.0; MAX_DIM],
            uncertainty: [0.01; MAX_DIM],
            valid_lo: [-1.0; MAX_DIM],
            valid_hi: [1.0; MAX_DIM],
            measured_at_ns: 0,
            valid_for_ns: 0,
            deps: [None; MAX_DEPS],
            epoch: 0,
            selftest_passed: true,
            prev_epoch: 0,
        };
        for k in 0..18 {
            j.value[k] = if k % 7 == 0 { 800.0 } else { 5.0 };
        }
        let e = b.submit(j).unwrap();
        *epoch_of_hand = e;
    }

    fn simple(b: &mut Body, q: Quantity, jac_epoch: Option<u64>) {
        let mut m = Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity: q,
            dim: 2,
            value: [0.5; MAX_DIM],
            uncertainty: [0.002; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [1.0; MAX_DIM],
            measured_at_ns: 0,
            valid_for_ns: 0,
            deps: [None; MAX_DEPS],
            epoch: 0,
            selftest_passed: true,
            prev_epoch: 0,
        };
        if let Some(e) = jac_epoch {
            m.deps[0] = Some((Quantity::ImageJacobian, e));
        }
        b.submit(m).unwrap();
    }

    /// A body that knows nothing must refuse to execute -- not move a little, not use a default.
    #[test]
    fn execute_on_an_unmeasured_body_refuses() {
        let b = Body::new();
        let f = execute::Fast_for_test();
        match execute(&b, &f, &spec(), &unit_x(), 1_000) {
            Outcome::Refused(v) => assert_eq!(v.why, refuse::Reason::NeverMeasured),
            other => panic!("expected a refusal, got {other:?}"),
        }
    }

    /// 🔴 The shortcut guard. A policy must not be able to smuggle distance into the magnitude of
    /// its direction vector; a non-unit vector is a malformed intent, never silently normalised.
    #[test]
    fn a_non_unit_direction_is_rejected_not_normalised() {
        let mut b = Body::new();
        let mut e = 0;
        jacobian(&mut e, &mut b);
        simple(&mut b, Quantity::HandPixel, Some(e));
        simple(&mut b, Quantity::Reach, None);
        let f = execute::Fast_for_test();
        let mut i = unit_x();
        i.dir = [3.0, 0.0, 0.0];
        assert!(matches!(
            execute(&b, &f, &spec(), &i, 1_000),
            Outcome::BadIntent(_)
        ));
    }

    /// Grip is an ABSOLUTE opening, so a value outside [0,1] is a contract violation, not a clamp.
    #[test]
    fn grip_outside_the_absolute_range_is_rejected() {
        let b = Body::new();
        let f = execute::Fast_for_test();
        let mut i = unit_x();
        i.grip = 1.7;
        assert!(matches!(
            execute(&b, &f, &spec(), &i, 1_000),
            Outcome::BadIntent(_)
        ));
    }

    /// 🔴 Without the proven fast face linked in, nothing may move. The stub refuses everything,
    /// and this test asserts that -- a build that reached a robot without the proof must be inert,
    /// not silently governed by a second, unproven copy of the rules.
    #[cfg(not(feature = "fast"))]
    #[test]
    fn without_the_proven_core_nothing_moves() {
        let mut b = Body::new();
        let mut e = 0;
        jacobian(&mut e, &mut b);
        simple(&mut b, Quantity::HandPixel, Some(e));
        simple(&mut b, Quantity::Reach, None);
        let f = execute::Fast_for_test();
        assert!(matches!(
            execute(&b, &f, &spec(), &unit_x(), 1_000),
            Outcome::Halted(_)
        ));
    }
}

/// The power-on schedule, and the ledger of what it still does not cover.
#[cfg(test)]
mod schedule_and_debt {
    use super::*;
    use measurement::{AxisKind, MAX_DEPS, MAX_DIM};
    use schedule::{is_ready, plan, prerequisites, Need};

    fn m(q: Quantity, valid_for_ns: u64) -> Measurement {
        let mut x = Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity: q,
            dim: 1,
            value: [1.0; MAX_DIM],
            uncertainty: [0.01; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [2.0; MAX_DIM],
            measured_at_ns: 1_000,
            valid_for_ns,
            deps: [None; MAX_DEPS],
            epoch: 0,
            selftest_passed: true,
            prev_epoch: 0,
        };
        x.value[0] = 1.0;
        x
    }

    /// A body that knows nothing owes everything, and the order must be runnable: no probe may be
    /// scheduled before something it is measured against.
    #[test]
    fn a_fresh_body_owes_everything_in_a_runnable_order() {
        let b = Body::new();
        let p = plan(&b, 0);
        assert_eq!(p.n, Quantity::COUNT, "a fresh body must owe every quantity");
        assert!(!is_ready(&b, 0));

        let mut seen = [false; Quantity::COUNT];
        for (q, why) in p.steps() {
            for pre in prerequisites(*q) {
                assert!(
                    seen[*pre as usize],
                    "{} was scheduled before {}, which it is measured against",
                    q.as_str(),
                    pre.as_str()
                );
            }
            assert_eq!(*why, Need::NeverMeasured);
            seen[*q as usize] = true;
        }
    }

    /// 🔴 The cascade. Re-measuring the Jacobian invalidates everything expressed in terms of it,
    /// and the plan must say so **before** it happens rather than leaving somebody to remember.
    #[test]
    fn re_measuring_a_prerequisite_schedules_everything_built_on_it() {
        let mut b = Body::new();
        // A body that is complete except that the Jacobian has a 60 s life and has just expired.
        let je = b.submit(m(Quantity::ImageJacobian, 60_000_000_000)).unwrap();
        let aw = b.submit(m(Quantity::ArmWeight, 0)).unwrap();
        for q in [
            Quantity::HandPixel,
            Quantity::GripperSpan,
            Quantity::SelfOcclusion,
            Quantity::ToolOffset,
            Quantity::ToolAxisColumn,
        ] {
            let mut x = m(q, 0);
            x.deps[0] = Some((Quantity::ImageJacobian, je));
            b.submit(x).unwrap();
        }
        let mut ct = m(Quantity::ContactThreshold, 0);
        ct.deps[0] = Some((Quantity::ArmWeight, aw));
        b.submit(ct).unwrap();
        for q in [
            Quantity::Latency,
            Quantity::Backlash,
            Quantity::Reach,
            Quantity::StepDelivery,
            Quantity::Floor,
            // 🔴 2026-08-12 加的第 14 格。这条测试因此**先红了一次,而它红得对** ——
            //    "一具完全量过的身体"的定义随着量表变长而变,漏一格就该判不 ready。
            Quantity::HomePose,
            // 🔴 2026-08-17 加的第 15 格,同一条又红了一次,又红得对。
            //    这一次连着抓出加变体时漏改的另外两处:`COUNT` 还是 14(碰到这一格
            //    直接 `index out of bounds`,**整轮自标定当场崩、标定一个字没写出来**),
            //    以及 `from_u32` 不认识它(⇒ `missing()` 报不出它 ⇒ 它**永远不会被
            //    排进上电日程**,连"从没量过"这件事都看不见)。
            //    ⇒ 加一个量要改四处:枚举 · COUNT · from_u32 · 这张表。
            Quantity::Friction,
        ] {
            b.submit(m(q, 0)).unwrap();
        }
        assert!(is_ready(&b, 2_000), "a fully measured body must plan as ready");

        // 61 s later only the Jacobian's own clock has run out. Everything measured against it is
        // still fresh by its own clock and is about to be worthless.
        let p = plan(&b, 61_500_000_000);
        let names: Vec<&str> = p.steps().iter().map(|(q, _)| q.as_str()).collect();
        assert_eq!(names[0], "image_jacobian", "the prerequisite must be first: {names:?}");
        for q in ["hand_pixel", "gripper_span", "self_occlusion", "tool_offset", "tool_axis_column"] {
            assert!(names.contains(&q), "{q} was left off the plan: {names:?}");
        }
        assert!(
            !names.contains(&"arm_weight") && !names.contains(&"contact_threshold"),
            "quantities that answer to nothing the Jacobian touches were dragged in: {names:?}"
        );
        assert_eq!(p.steps()[0].1, Need::Stale);
        assert_eq!(
            p.steps().iter().find(|(q, _)| *q == Quantity::HandPixel).unwrap().1,
            Need::DependencyMoved
        );
    }

    /// 🔴 The correction this whole ledger exists for: the structural zero and the real debt are
    /// two different numbers, and quoting the first alone is the claim this layer punishes.
    #[test]
    fn the_zero_hand_filled_count_is_not_the_debt() {
        let b = Body::new();
        assert_eq!(b.hand_filled_constants(), 0, "structural, by construction");
        assert!(
            debt::outstanding() > 0,
            "the ledger reports no outstanding constants, which would mean the deployed teacher \
             has none -- it has TEACH_HIGH_FRAC, measured at 32/44 against 10/100"
        );
        // The dominant constant must be in the ledger by name, or the ledger is decoration.
        assert!(
            debt::LEDGER.iter().any(|c| c.name == "TEACH_HIGH_FRAC"),
            "the largest measured effect on the stack is not in the ledger"
        );
        // And this layer's own constants must be audited here too. A ledger that audits only other
        // people's code is an advertisement.
        //
        // 2026-08-11: `bl_spec.step_m` and `bl_spec.damping` were DISCHARGED — `Spec::from_body`
        // now derives both from measurements and refuses when they are absent. The guard is kept
        // and inverted rather than deleted: it must still be true that this layer audits itself,
        // so the rows must exist AND each must name the quantity that discharged it. A row that
        // silently went from Outstanding to Measured with no source named is the shape of claim
        // this file exists to punish.
        for (row, by) in [
            ("bl_spec.step_m", Quantity::StepDelivery),
            ("bl_spec.damping", Quantity::ImageJacobian),
        ] {
            let c = debt::LEDGER
                .iter()
                .find(|c| c.name == row)
                .unwrap_or_else(|| panic!("the layer's own {row} is not in its own ledger"));
            assert!(
                matches!(c.standing, debt::Standing::Measured(q) if q == by),
                "{row} must name the quantity that discharged it, not merely claim Measured"
            );
        }
        // And the layer must still be honest about what it has NOT discharged.
        assert!(
            debt::LEDGER
                .iter()
                .any(|c| c.site == debt::SELF && matches!(c.standing, debt::Standing::Outstanding)),
            "every one of this layer's own constants now claims to be measured -- if that is true,              say so deliberately; if it is not, the ledger has stopped auditing its author"
        );
        assert!(debt::total() >= 45, "the census found 45 knobs; the ledger has {}", debt::total());
    }

    /// 🔴 "A named slot in an enum is not a probe." Every quantity must now have one, and this is
    /// the mechanical statement of it -- the count was 5 when the question was first asked.
    #[test]
    fn no_quantity_is_a_name_without_a_probe() {
        assert_eq!(
            debt::declared_only(),
            0,
            "a quantity has a slot and no estimator; it reads as covered and is worth nothing"
        );
        // Every ledger row that names a quantity must name one this build knows.
        for c in debt::LEDGER {
            if let debt::Standing::Measured(q) | debt::Standing::DeclaredOnly(q) = c.standing {
                assert!(
                    Quantity::from_u32(q as u32).is_some(),
                    "{} names a quantity this build does not have",
                    c.name
                );
            }
        }
    }
}

/// The probes, run over **real episode logs** rather than over data a test author imagined.
///
/// 🔴 Why this is a test and not an example. `examples/reach_on_real_data.rs` was written to take a
/// CSV path, and no such CSV was ever committed — so the one real-data check in this repository
/// could not be re-run by anybody, including its author. A validation nobody can re-run is a
/// validation nobody can contradict, which is the same shape as a guard that never fires. These
/// inputs live in `realdata/`, regenerated by `realdata/extract.py`, and the assertions run in
/// `cargo test`.
///
/// It has already earned its keep: the `backlash` control-ratio guard exists because this data
/// produced a **1.01 rad** dead band on an arm that has none, and nothing else in that reading
/// looked wrong.
#[cfg(all(test, feature = "std"))]
mod real_data {
    use crate::probe::{backlash, contact_threshold, Declined, Polarity};

    fn read(name: &str) -> String {
        let p = format!("{}/../realdata/{}", env!("CARGO_MANIFEST_DIR"), name);
        std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"))
    }

    /// A staircase of press depths against a MEASURED touch height, with PhysX contact as ground
    /// truth: 120 rows clear of the surface, 400 pressed into it.
    #[test]
    fn contact_threshold_on_real_press_logs() {
        let text = read("contact_stair.csv");
        let (mut free, mut touch) = (Vec::new(), Vec::new());
        for line in text.lines().skip(1) {
            let f: Vec<&str> = line.trim().split(',').collect();
            if f.len() != 3 {
                continue;
            }
            let (d, force) = (
                f[0].parse::<f64>().expect("depth"),
                f[1].parse::<f64>().expect("force"),
            );
            if d < 0.0 {
                free.push(force);
            } else if d > 0.0 {
                touch.push(force);
            }
        }
        assert_eq!((free.len(), touch.len()), (120, 400), "the log changed shape");

        let m = contact_threshold(&free, &touch, Polarity::HigherOnContact, 1_000_000_000, 3)
            .expect("two physically distinct conditions must be measurable");
        let hi_free = free.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let lo_touch = touch.iter().copied().fold(f64::INFINITY, f64::min);
        eprintln!(
            "contact_threshold: {:.3} ± {:.3} N   free ≤ {:.3}, contact ≥ {:.3}",
            m.value[0], m.uncertainty[0], hi_free, lo_touch
        );
        assert!(
            m.value[0] > hi_free && m.value[0] < lo_touch,
            "threshold {} sits on an observed sample instead of inside the gap [{hi_free}, {lo_touch}]",
            m.value[0]
        );
        // 🔴 This rig's free-space channel reads exactly 0.000 on all 120 rows. Without the
        // half-gap floor the propagated standard error is exactly zero, and the layer would claim
        // the boundary is known perfectly while it could sit anywhere in a 37 N band.
        assert!(
            m.uncertainty[0] >= 0.5 * (lo_touch - hi_free) - 1e-9,
            "sigma {} is tighter than the gap in which every choice is equally consistent",
            m.uncertainty[0]
        );

        // -- must refuse, on the same real data: the two classes are the SAME condition. A probe
        //    that answers this has not been shown to be measuring contact at all.
        let (a, b) = free.split_at(60);
        assert_eq!(
            contact_threshold(a, b, Polarity::HigherOnContact, 1_000_000_000, 3).unwrap_err(),
            Declined::NoResponse
        );

        // -- must refuse: the classes swapped. Contact reading lower than free space is an inverted
        //    convention, and a threshold fitted to it fires in free space and is silent on contact.
        assert_eq!(
            contact_threshold(&touch, &free, Polarity::HigherOnContact, 1_000_000_000, 3).unwrap_err(),
            Declined::Inconsistent
        );
    }

    /// Per-step commanded vs achieved joint motion over three 300-step sweeps of the same arm: one
    /// in free space, two with the leg pressed against a surface.
    #[test]
    fn backlash_on_real_sweep_logs() {
        let text = read("reversals.csv");
        let mut rows: Vec<(String, u32, f64, f64)> = Vec::new();
        for line in text.lines().skip(1) {
            let f: Vec<&str> = line.trim().split(',').collect();
            if f.len() != 4 {
                continue;
            }
            rows.push((
                f[0].to_string(),
                f[1].parse().expect("joint"),
                f[2].parse().expect("cmd"),
                f[3].parse().expect("act"),
            ));
        }
        assert!(rows.len() > 5_000, "the log changed shape: {} rows", rows.len());

        let mut hover_ok = 0usize;
        for leg in ["hover", "near", "press"] {
            for j in 0..7u32 {
                let steps: Vec<(f64, f64)> = rows
                    .iter()
                    .filter(|r| r.0 == leg && r.1 == j)
                    .map(|r| (r.2, r.3))
                    .collect();
                let out = backlash(&steps, 1_000_000_000);
                match out {
                    Ok(m) => {
                        eprintln!(
                            "{leg:<6} j{j}  n={:<4} backlash = {:+.3e} ± {:.1e} rad",
                            steps.len(),
                            m.value[0],
                            m.uncertainty[0]
                        );
                        if leg == "hover" {
                            hover_ok += 1;
                            // A simulated Franka has no gear slop. Anything above a milliradian
                            // here would be the probe inventing one.
                            assert!(
                                m.value[0].abs() < 1e-3,
                                "{leg} j{j} read {:.6} rad of slop on an arm that has none",
                                m.value[0]
                            );
                        }
                    }
                    Err(e) => eprintln!("{leg:<6} j{j}  n={:<4} refused: {e:?}", steps.len()),
                }
            }
        }
        assert!(
            hover_ok >= 5,
            "the free-space sweep was answered on only {hover_ok} of 7 joints -- the probe has \
             become a refuser"
        );

        // 🔴 The reading this guard was written for. On the `near` leg, joint 5's same-direction
        // ratios scatter around 0.00025 with a standard error of 0.279; the unguarded estimator
        // divided by that and reported **1.01 rad** — about 58 degrees of slop. It must refuse.
        let near5: Vec<(f64, f64)> = rows
            .iter()
            .filter(|r| r.0 == "near" && r.1 == 5)
            .map(|r| (r.2, r.3))
            .collect();
        assert_eq!(
            backlash(&near5, 1_000_000_000).unwrap_err(),
            Declined::Inconsistent,
            "the joint that produced a 1.01 rad dead band is being answered again"
        );
    }
}

/// The cross-language conformance run: prove the Rust side is talking to the **proven Ada core**
/// and not to something that merely links.
///
/// 🔴 A binding exercised only on the happy path is a binding whose error codes have never been
/// observed. If the far side were stubbed, misnamed, or resolved to a different symbol, every call
/// would return `Ok` and nothing downstream would notice — which is the exact failure shape this
/// whole layer exists to remove. So the check drives guaranteed **refusals** first.
#[cfg(all(test, feature = "fast"))]
mod fast_conformance {
    use crate::fast::{Fast, FastStatus, HaltReason, MAX_JOINTS};

    /// 🔴 The fast face holds ONE state per process, deliberately: a robot has one body, and an
    /// array of them would invite the "which one am I talking to" class of bug for no gain.  The
    /// consequence is that tests touching it must not run concurrently -- so they take this lock.
    /// Serialising the tests is the right trade; making the production layer multi-instance to
    /// suit a test harness is not.
    pub(crate) static FAST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// The Ada core must reject a NaN limit, halt on an out-of-envelope command rather than clamp
    /// it, keep the safe hold inside the envelope, and still admit a good command.
    #[test]
    fn the_proven_core_refuses_when_it_must() {
        let _g = FAST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let f = Fast::reset();
        f.selftest().expect("the proven fast face did not behave as proved");
    }

    /// The watchdog must latch on silence, and a perfectly good command must NOT walk it back out.
    /// A safety state you can leave by doing nothing is not a safety state.
    #[test]
    fn a_latch_is_a_latch() {
        let _g = FAST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let f = Fast::reset();
        let lo = [-1.0; MAX_JOINTS];
        let hi = [1.0; MAX_JOINTS];
        let hold = [0.0; MAX_JOINTS];
        assert_eq!(f.install(&lo, &hi, &hold, 6, 20.0, 100, 1_000), FastStatus::Ok);

        f.tick(1_500); // deadline was 100 ms; 500 ms of silence
        assert!(f.halted());
        assert_eq!(f.reason(), HaltReason::WatchdogExpired);

        let good = [0.5; MAX_JOINTS];
        assert_eq!(f.admit(&good, 0.0, 1_510).0, FastStatus::Refuse);
        assert!(f.halted(), "a good command cleared a latch");

        // The wrong witness must not open the door either.
        assert_eq!(f.clear(HaltReason::ExternalStop, 1_600), FastStatus::Refuse);
        assert!(f.halted());
        assert_eq!(f.clear(HaltReason::WatchdogExpired, 1_600), FastStatus::Ok);
        assert!(!f.halted());
    }

    /// `MAX_JOINTS` here must equal `Max_Joints` there. A silent disagreement would mis-index every
    /// joint while every call still returned success.
    #[test]
    fn the_two_sides_agree_on_the_joint_count() {
        let _g = FAST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let f = Fast::reset();
        let lo = [-1.0; MAX_JOINTS];
        let hi = [1.0; MAX_JOINTS];
        let hold = [0.0; MAX_JOINTS];
        // One past the shared bound must be rejected by the far side, which is only true if the
        // far side's bound is the same number.
        assert_eq!(
            f.install(&lo, &hi, &hold, (MAX_JOINTS + 1) as u32, 20.0, 100, 1_000),
            FastStatus::Einval
        );
        assert_eq!(
            f.install(&lo, &hi, &hold, MAX_JOINTS as u32, 20.0, 100, 1_000),
            FastStatus::Ok
        );
    }
}

/// End-to-end: a body that knows nothing measures itself and then moves.
///
/// 🔴 This is the test the whole layer exists to pass, and the shape of it *is* the claim:
/// **0 demonstrations, 0 collected episodes, 0 hand-filled numbers.** Nothing below types in a
/// body constant. Every number the robot uses about itself, it obtained by commanding a step and
/// watching what happened.
///
/// It also asserts the order — refuse *before* measuring, move *after* — because a layer that
/// happens to work once it is configured, and permits when it is not, permits every time somebody
/// forgets to configure it. Forgetting is the normal case.
#[cfg(test)]
mod end_to_end {
    use super::*;
    use crate::probe::Polarity;
    use measurement::AxisKind;
    use execute::{execute, Intent, Outcome, Spec};
    use hand::{Candidate, HandTracker};
    use measurement::MAX_DIM;
    use probe::{default_hand_config, Sample};

    const N: usize = 6;

    fn spec() -> Spec {
        Spec {
            step_m: 0.004,
            period_ms: 40,
            damping: 0.05,
            n_joints: N,
        }
    }

    fn forward(cmd: f64, j: usize) -> [f64; 2] {
        // Stand-in for the world: joint j moves the tracked point with a per-joint gain. The
        // numbers do not matter; what matters is that the layer never receives them -- it can only
        // find them out by commanding and looking.
        let gain = 0.02 + 0.004 * j as f64;
        [0.5 + gain * cmd, 0.5 - 0.5 * gain * cmd]
    }

    #[test]
    fn a_body_that_knows_nothing_measures_itself_and_then_moves() {
        #[cfg(feature = "fast")]
        let _g = crate::fast_conformance::FAST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let mut body = Body::new();
        let fast = execute::Fast_for_test();

        // ---- 1. before it has measured anything, it must refuse ------------------------------
        let intent = Intent {
            dir: [1.0, 0.0, 0.0],
            drot: [0.0; 3],
            grip: 1.0,
            base: [0.0; 3],
        };
        assert!(
            matches!(execute(&body, &fast, &spec(), &intent, 1_000), Outcome::Refused(_)),
            "an unmeasured body permitted motion -- the default is permissive"
        );
        assert_eq!(body.hand_filled_constants(), 0);
        assert_eq!(body.missing().count(), measurement::Quantity::COUNT);

        // ---- 2. it probes itself: command a step, watch the image ---------------------------
        let mut samples: [Sample; N + 3] = [Sample {
            cmd: [0.0; MAX_DIM],
            n: N,
            uv: [0.5, 0.5],
            at_ns: 0,
        }; N + 3];
        for (k, s) in samples.iter_mut().enumerate() {
            let c = 0.01 * k as f64;
            for j in 0..N {
                s.cmd[j] = c;
            }
            // superpose every joint's contribution, exactly as a real frame would
            let mut uv = [0.0, 0.0];
            for j in 0..N {
                let p = forward(c, j);
                uv[0] += p[0] / N as f64;
                uv[1] += p[1] / N as f64;
            }
            s.uv = uv;
            s.at_ns = k as u64;
        }
        let jac = probe::image_jacobian(&samples, N, 1_000_000_000, 1e-4)
            .expect("the probe declined on a clean response");
        let jac_epoch = body.submit(jac).expect("a measured Jacobian was rejected");

        // ---- 3. it finds its own hand, and keeps finding it ----------------------------------
        let mut tracker = HandTracker::new(default_hand_config());
        let cands = [
            Candidate { u: 0.62, v: 0.44, gain: 1.0, rigidity: 0.95, pixels: 900, spread: 0.003 },
            // a competitor that moves too, but clearly weaker -- separable, so it may be resolved
            Candidate { u: 0.31, v: 0.70, gain: 0.15, rigidity: 0.9, pixels: 500, spread: 0.004 },
        ];
        // Measured on the SAME clock the executor will use.  A hand point is valid for about as
        // long as the hand has not moved -- 50 ms, one control period or two -- so a test that
        // measured at 1 ms and executed at 1010 ms found the staleness gate instead of the
        // executor.  That was a test bug, and it is asserted deliberately at the end.
        let hp = probe::hand_pixel(&mut tracker, &cands, 1_000_000_000, 2, 0, jac_epoch)
            .expect("the hand probe declined on a separable frame");
        body.submit(hp).expect("a measured hand point was rejected");

        // reach: probed, like everything else
        let mut reach = measurement::Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity: measurement::Quantity::Reach,
            dim: 2,
            value: [0.5; MAX_DIM],
            uncertainty: [0.01; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [1.0; MAX_DIM],
            measured_at_ns: 1_000_000_000,
            valid_for_ns: 0,
            deps: [None; measurement::MAX_DEPS],
            epoch: 0,
            selftest_passed: true,
            prev_epoch: 0,
        };
        reach.deps[0] = Some((measurement::Quantity::ImageJacobian, jac_epoch));
        body.submit(reach).unwrap();

        // ---- 4. now it may move --------------------------------------------------------------
        let out = execute(&body, &fast, &spec(), &intent, 1_010);
        #[cfg(feature = "fast")]
        {
            // The proven core must be installed before it will admit anything -- that is its job.
            let lo = [-1.0; fast::MAX_JOINTS];
            let hi = [1.0; fast::MAX_JOINTS];
            let hold = [0.0; fast::MAX_JOINTS];
            let f = fast::Fast::reset();
            assert_eq!(
                f.install(&lo, &hi, &hold, N as u32, 20.0, 1_000, 1_000),
                fast::FastStatus::Ok
            );
            match execute(&body, &f, &spec(), &intent, 1_010) {
                Outcome::Move(cmd) => {
                    let travelled = cmd[..N].iter().map(|x| x * x).sum::<f64>().sqrt();
                    assert!(
                        (travelled - spec().step_m).abs() < 1e-9,
                        "step came from somewhere other than the spec: {travelled}"
                    );
                }
                other => panic!("a fully measured body still would not move: {other:?}"),
            }
        }
        #[cfg(not(feature = "fast"))]
        assert!(
            matches!(out, Outcome::Halted(_)),
            "without the proven core anything but a halt is wrong"
        );
        let _ = out;

        // ---- 5. and the claim itself ---------------------------------------------------------
        assert_eq!(
            body.hand_filled_constants(),
            0,
            "a body constant was typed in rather than measured"
        );

        // ---- 5b. a hand point goes stale in tens of milliseconds, and that is the point --------
        // The old estimator's whole failure was a fit that stayed trusted for 700 steps after it
        // stopped being true: 2.0 px at fit time, 4.9-14.6 px at the moment that mattered.  A short
        // lifetime is what turns "it drifted" into a refusal instead of a confident wrong answer.
        {
            let mut ask = refuse::Ask::EMPTY;
            ask.needs[0] = Some(measurement::Quantity::HandPixel);
            assert!(body.admit(&ask, 1_000_020_000).admit, "20 ms later it should still hold");
            let v = body.admit(&ask, 3_000_000_000); // 2 s later (1e9 -> 3e9 ns)
            assert!(!v.admit && v.why == refuse::Reason::Stale,
                    "a two-second-old hand point was still being trusted");
        }

        // ---- 6. knock the camera: everything measured against it must go invalid --------------
        // The case a wall-clock TTL cannot catch. Nothing about the hand point's own freshness
        // changed; it is worthless anyway.
        let jac2 = probe::image_jacobian(&samples, N, 1_000_010_000, 1e-4).unwrap();
        body.submit(jac2).unwrap();
        let mut ask = refuse::Ask::EMPTY;
        ask.needs[0] = Some(measurement::Quantity::HandPixel);
        // Note the clock: only 10 ms later, so the hand point is NOT stale.  It is refused purely
        // because what it was measured against moved -- the case a wall-clock TTL cannot catch.
        let v = body.admit(&ask, 1_000_020_000);
        assert!(!v.admit && v.why == refuse::Reason::DependencyChanged,
                "re-measuring the Jacobian did not invalidate what was measured against it");
    }

    /// `step_delivery`: every case here is one the probe **must** refuse, except the last.
    ///
    /// The numbers are the two arms that produced the quantity: a 45 mm commanded step delivered
    /// 0.76 of itself on one and 0.11 on the other, and the step budget had been carried over
    /// from the first.
    #[test]
    fn step_delivery_refuses_what_it_cannot_answer() {
        use probe::{step_delivery, Declined};
        const T: u64 = 1_000_000_000;

        // -- must refuse: not enough evidence to say anything ------------------------------
        assert_eq!(step_delivery(&[(0.045, 0.034); 4], T).unwrap_err(),
                   Declined::NotEnoughSamples);

        // -- must refuse: a step nobody commanded carries no information about delivery, so
        //    after dropping those there is nothing left. NOT "delivery is 0".
        assert_eq!(step_delivery(&[(0.0, 0.001); 20], T).unwrap_err(),
                   Declined::NotEnoughSamples);

        // -- must refuse: commanded repeatedly, body never moved. A dead joint must not be
        //    reported as a merely slow one -- that is the whole point of a separate reason.
        let dead: Vec<(f64, f64)> =
            (0..12).map(|i| (0.010 + 0.003 * f64::from(i), 0.0)).collect();
        assert_eq!(step_delivery(&dead, T).unwrap_err(), Declined::NoResponse);

        // -- must refuse: probed at exactly one magnitude. Delivery varies with step size, so a
        //    single-point "range" would let the gate admit asks it has no basis for -- the same
        //    rule arm_weight applies to a single pose.
        assert_eq!(step_delivery(&[(0.045, 0.034); 20], T).unwrap_err(),
                   Declined::Inconsistent);

        // -- must refuse: NaN in, nothing usable out (they are dropped, not propagated).
        assert_eq!(step_delivery(&[(f64::NAN, 0.03); 20], T).unwrap_err(),
                   Declined::NotEnoughSamples);

        // -- must be ADMITTED: the real reading. A layer that refuses everything is also not a
        //    body layer.  Ratio 0.11 with a handful of contact steps mixed in; the median must
        //    survive them, which a mean would not.
        let mut s: Vec<(f64, f64)> =
            (0..20).map(|i| { let c = 0.020 + 0.002 * f64::from(i); (c, 0.11 * c) }).collect();
        s[3].1 = 0.0;   // hit the table
        s[9].1 = 0.0;   // and again
        let m = step_delivery(&s, T).expect("the real reading must be admitted");
        assert!((m.value[0] - 0.11).abs() < 0.01,
                "two contact steps moved the estimate: {} -- the median did not survive them",
                m.value[0]);
        // Tolerance, because `0.020 + 0.002*19` is 0.057999999999999996 and a bare `>= 0.058`
        // fails on a probe that is behaving perfectly. Recorded rather than silently loosened:
        // the first run of this assertion failed here, and the bug was in the assertion.
        assert!(m.valid_lo[0] <= 0.0201 && m.valid_hi[0] >= 0.0579,
                "the validity range must be the span of commanded magnitudes actually probed, \
                 got [{}, {}]", m.valid_lo[0], m.valid_hi[0]);
        assert!(m.uncertainty[0] >= 0.0 && m.uncertainty[0].is_finite());

        // -- and the other arm, same code path, must come out different. Two bodies reading the
        //    same number here would be the signature of a probe that is not measuring anything.
        let s2: Vec<(f64, f64)> =
            (0..20).map(|i| { let c = 0.020 + 0.002 * f64::from(i); (c, 0.76 * c) }).collect();
        let m2 = step_delivery(&s2, T).unwrap();
        assert!(m2.value[0] > 0.7 && m.value[0] < 0.2,
                "the two arms read {} and {} -- a probe that cannot separate them is not a probe",
                m.value[0], m2.value[0]);
    }

    /// `gripper_span`: every case here is one the probe **must** refuse, except the last two.
    ///
    /// The constant this exists to abolish is real and is still running: `L3_GRIPPER_BIAS = 0.145`,
    /// hand-set in the deployed servo, provenance untraceable. A number nobody can trace cannot be
    /// re-measured on a new machine.
    #[test]
    fn gripper_span_refuses_what_it_cannot_answer() {
        use probe::{gripper_span, Declined};
        const T: u64 = 1_000_000_000;
        const RULER: f64 = 800.0; // image units per metre; the rig's own column norms were 792–904
        const JE: u64 = 7;

        // A gripper whose jaws travel 0.10 m per unit of commanded opening, swept 0.2 -> 1.0.
        let sweep = |slope: f64, n: usize| -> Vec<(f64, f64)> {
            (0..n)
                .map(|i| {
                    let x = 0.2 + 0.8 * (i as f64) / ((n - 1) as f64);
                    (x, RULER * slope * x)
                })
                .collect()
        };

        // -- must refuse: no metric reference. A span "in metres" without a ruler is a number in
        //    image units wearing a unit label.
        assert_eq!(
            gripper_span(&sweep(0.10, 12), 0.0, 1.0, T, JE).unwrap_err(),
            Declined::MissingDependency
        );
        assert_eq!(
            gripper_span(&sweep(0.10, 12), f64::NAN, 1.0, T, JE).unwrap_err(),
            Declined::MissingDependency
        );

        // -- must refuse: four points leave no residual, and a fit without a residual reports an
        //    uncertainty of zero, which is a measurement that cannot be refused on.
        assert_eq!(
            gripper_span(&sweep(0.10, 4), RULER, 1.0, T, JE).unwrap_err(),
            Declined::NotEnoughSamples
        );

        // -- must refuse: every sample at one opening. No range to be valid over.
        assert_eq!(
            gripper_span(&[(0.6, 48.0); 12], RULER, 1.0, T, JE).unwrap_err(),
            Declined::Inconsistent
        );

        // -- must refuse: the jaws did not respond. A jammed gripper and jaws the camera cannot
        //    resolve both land here, and neither may be reported as a very small gripper.
        let stuck: Vec<(f64, f64)> = (0..12)
            .map(|i| (0.2 + 0.8 * f64::from(i) / 11.0, 48.0))
            .collect();
        assert_eq!(
            gripper_span(&stuck, RULER, 1.0, T, JE).unwrap_err(),
            Declined::NoResponse
        );

        // -- must refuse: the jaws CLOSED as the opening was commanded up. Inverted convention, or
        //    the tracked blobs are not the jaws. Taking |slope| would let both through looking well.
        assert_eq!(
            gripper_span(&sweep(-0.10, 12), RULER, 1.0, T, JE).unwrap_err(),
            Declined::Inconsistent
        );

        // -- must be ADMITTED: 0.10 m per unit opening over a 0.8 sweep = 0.080 m of travel, which
        //    is the jaw maximum the deployed teacher currently carries as a hand-set env knob.
        let m = gripper_span(&sweep(0.10, 12), RULER, 1.0, T, JE).expect("a clean sweep must be admitted");
        assert!(
            (m.value[0] - 0.080).abs() < 1e-6,
            "span read {} m, expected 0.080",
            m.value[0]
        );
        assert_eq!(
            (m.valid_lo[0], m.valid_hi[0]),
            (0.2, 1.0),
            "validity must be the openings actually commanded, not [0,1] by assumption"
        );
        assert_eq!(
            m.deps[0].map(|d| d.0),
            Some(measurement::Quantity::ImageJacobian),
            "the ruler comes from the camera; knocking it must invalidate this"
        );

        // -- the ruler's own error must reach the answer. Quoting only the fit's error would make a
        //    span measured with a bad ruler look as precise as one measured with a good one.
        let sharp = gripper_span(&sweep(0.10, 12), RULER, 1.0, T, JE).unwrap();
        let blunt = gripper_span(&sweep(0.10, 12), RULER, 80.0, T, JE).unwrap();
        assert!(
            blunt.uncertainty[0] > 10.0 * sharp.uncertainty[0],
            "a 10% ruler and a 0.1% ruler produced {} and {}",
            blunt.uncertainty[0],
            sharp.uncertainty[0]
        );

        // -- and a partial sweep must describe the partial sweep, not extrapolate to full open.
        let half: Vec<(f64, f64)> = (0..12)
            .map(|i| {
                let x = 0.4 + 0.4 * f64::from(i) / 11.0;
                (x, RULER * 0.10 * x)
            })
            .collect();
        let hm = gripper_span(&half, RULER, 1.0, T, JE).unwrap();
        assert!(
            (hm.value[0] - 0.040).abs() < 1e-6,
            "a sweep of half the range reported {} m -- it extrapolated to jaws nobody opened",
            hm.value[0]
        );
    }

    /// `backlash`: the point of this probe is the **control**, and this test is mostly about that.
    #[test]
    fn backlash_refuses_what_it_cannot_answer() {
        use probe::{backlash, Declined};
        const T: u64 = 1_000_000_000;

        // A body with delivery fraction `f` and dead band `d`: direction flips every two steps, so
        // roughly half the steps are reversals and half are same-direction controls.
        let body = |f: f64, d: f64, n: usize| -> Vec<(f64, f64)> {
            let mut out = Vec::new();
            let mut prev_sign = 1.0f64;
            for i in 0..n {
                let mag = 0.020 + 0.002 * f64::from(i as u32);
                let sign = if (i / 2) % 2 == 0 { 1.0 } else { -1.0 };
                let reversal = i > 0 && sign != prev_sign;
                let eff = if reversal { (mag - d).max(0.0) } else { mag };
                out.push((sign * mag, sign * f * eff));
                prev_sign = sign;
            }
            out
        };

        // -- must refuse: too few steps to classify anything.
        assert_eq!(backlash(&body(0.9, 0.0, 6), T).unwrap_err(), Declined::NotEnoughSamples);

        // -- must refuse: never pushed both ways. A dead band at a reversal cannot be located from
        //    motion in one direction -- the shape of refusal `reach` gives for an unstraddled wall.
        let one_way: Vec<(f64, f64)> = (0..20)
            .map(|i| {
                let c = 0.020 + 0.002 * f64::from(i);
                (c, 0.9 * c)
            })
            .collect();
        assert_eq!(backlash(&one_way, T).unwrap_err(), Declined::Inconsistent);

        // -- must refuse: NO same-direction control (it reverses every single step). Without a
        //    control the post-reversal shortfall is exactly the confound this probe exists to
        //    remove, and reporting it would manufacture a dead band on any slow body.
        let all_rev: Vec<(f64, f64)> = (0..20)
            .map(|i| {
                let c = (0.020 + 0.002 * f64::from(i)) * if i % 2 == 0 { 1.0 } else { -1.0 };
                (c, 0.9 * c)
            })
            .collect();
        assert_eq!(backlash(&all_rev, T).unwrap_err(), Declined::Inconsistent);

        // -- must refuse: reversals at ONE magnitude. A dead band and a reversal-specific delivery
        //    deficit are perfectly confounded there; they separate only across magnitudes.
        let one_mag: Vec<(f64, f64)> = (0..20)
            .map(|i| {
                let s = if (i / 2) % 2 == 0 { 1.0 } else { -1.0 };
                (s * 0.045, s * 0.9 * 0.045)
            })
            .collect();
        assert_eq!(backlash(&one_mag, T).unwrap_err(), Declined::Inconsistent);

        // -- must refuse: commanded both ways and nothing moved. A dead joint is not a huge dead band.
        let dead: Vec<(f64, f64)> = body(0.9, 0.0, 20).iter().map(|&(c, _)| (c, 0.0)).collect();
        assert_eq!(backlash(&dead, T).unwrap_err(), Declined::NoResponse);

        // -- 🔴 must refuse: a control ratio that is not established. THIS CASE CAME FROM REAL DATA.
        //    A joint whose same-direction ratios scattered around 0.00025 with a standard error of
        //    0.279 was divided into, and reported a dead band of 1.01 rad -- 58 degrees, on an arm
        //    that has none. Everything else about the reading looked ordinary.
        let mut scattered = body(0.9, 0.0, 20);
        let mut c = 0usize;
        for (i, s) in scattered.iter_mut().enumerate() {
            let reversal = i > 0 && (i / 2) % 2 != ((i - 1) / 2) % 2;
            if i > 0 && !reversal {
                // Same-direction ratios that swing between 0.0002 and 3.0: a median near 1.5 whose
                // own standard error is wider than itself. Dividing by that is what produced the
                // 1.01 rad reading on the real logs.
                s.1 = s.0 * if c % 2 == 0 { 0.0002 } else { 3.0 };
                c += 1;
            }
        }
        assert_eq!(backlash(&scattered, T).unwrap_err(), Declined::Inconsistent);

        // -- 🔴 must be ADMITTED and read ~ZERO: a body that delivers 0.11 of every step and has no
        //    slop at all. This is the whole design. The naive reading -- post-reversal shortfall
        //    taken directly -- reports an enormous fictional dead band on exactly this body, and
        //    the assertion below computes it so the comparison is on the record rather than claimed.
        let slow_clean = body(0.11, 0.0, 24);
        let m = backlash(&slow_clean, T).expect("a body with no slop must be measurable, not refused");
        let naive: f64 = {
            let mut v: Vec<f64> = slow_clean
                .iter()
                .enumerate()
                .filter(|(i, _)| *i > 0 && (i / 2) % 2 != ((i - 1) / 2) % 2)
                .map(|(_, &(c, o))| c.abs() - o.abs())
                .collect();
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v[v.len() / 2]
        };
        assert!(
            m.value[0].abs() < 1e-9,
            "a slop-free body read a dead band of {} -- the control is not doing its job",
            m.value[0]
        );
        assert!(
            naive > 0.02,
            "the naive estimator was supposed to be badly wrong here; it read {naive}"
        );

        // -- must be ADMITTED and read the real dead band, on the SAME badly-delivering body.
        let slow_sloppy = body(0.11, 0.004, 24);
        let m2 = backlash(&slow_sloppy, T).expect("a real dead band must be measurable");
        assert!(
            (m2.value[0] - 0.004).abs() < 1e-9,
            "dead band read {} on a body built with 0.004",
            m2.value[0]
        );
        assert!(
            m2.valid_lo[0] < m2.valid_hi[0],
            "validity must be the span of reversal magnitudes actually probed"
        );
    }

    /// `tool_offset`: the number that is typed in four times in the live stack, measured instead.
    #[test]
    fn tool_offset_refuses_what_it_cannot_answer() {
        use probe::{tool_offset, Declined};
        const T: u64 = 1_000_000_000;
        const RULER: f64 = 800.0;
        const JE: u64 = 5;

        // A wrist turning through `span` radians with the working point `r_m` metres off the axis.
        let arc = |r_m: f64, span: f64, n: usize| -> Vec<(f64, f64, f64)> {
            (0..n)
                .map(|i| {
                    let a = span * (i as f64) / ((n - 1) as f64);
                    (a, 0.5 + RULER * r_m * a.cos(), 0.5 + RULER * r_m * a.sin())
                })
                .collect()
        };

        // -- must refuse: no metric reference.
        assert_eq!(
            tool_offset(&arc(0.145, 2.0, 12), 0.0, 1.0, T, JE).unwrap_err(),
            Declined::MissingDependency
        );

        // -- must refuse: three points fit a circle exactly and leave no residual, so there is no
        //    uncertainty to report and nothing the gate could refuse on.
        assert_eq!(
            tool_offset(&arc(0.145, 2.0, 4), RULER, 1.0, T, JE).unwrap_err(),
            Declined::NotEnoughSamples
        );

        // -- must refuse: the wrist never turned. Every point at one angle fits every circle, so
        //    any radius could be reported and would be believed.
        let held: Vec<(f64, f64, f64)> = (0..12).map(|_| (0.7, 0.6, 0.55)).collect();
        assert_eq!(
            tool_offset(&held, RULER, 1.0, T, JE).unwrap_err(),
            Declined::Inconsistent
        );

        // -- must refuse: the observed points lie on a LINE. The algebraic fit is singular there,
        //    and dividing by that determinant returns a radius made of rounding error.
        let line: Vec<(f64, f64, f64)> = (0..12)
            .map(|i| (0.1 * f64::from(i), 0.4 + 0.01 * f64::from(i), 0.5))
            .collect();
        assert_eq!(
            tool_offset(&line, RULER, 1.0, T, JE).unwrap_err(),
            Declined::Inconsistent
        );

        // -- 🔴 must be ADMITTED and read the offset. 0.145 m is the ARX x5 value that is currently
        //    hardcoded in four places and defaults onto every other body that forgets to override.
        let m = tool_offset(&arc(0.145, 2.0, 16), RULER, 1.0, T, JE)
            .expect("a wrist that turned must be measurable");
        assert!(
            (m.value[0] - 0.145).abs() < 1e-6,
            "offset read {} m, expected 0.145",
            m.value[0]
        );
        assert_eq!(
            m.deps[0].map(|d| d.0),
            Some(measurement::Quantity::ImageJacobian),
            "the arc is read in the camera's frame"
        );
        assert!(
            m.deps.iter().flatten().all(|d| d.0 != measurement::Quantity::HandPixel),
            "the hand point is re-measured every control step and bumps its epoch every time; \
             depending on it would make this quantity invalid one step after it was taken"
        );
        assert!(
            (m.valid_lo[0] - 0.0).abs() < 1e-12 && (m.valid_hi[0] - 2.0).abs() < 1e-12,
            "validity must be the wrist travel actually swept"
        );

        // -- 🔴 and the two bodies must not read the same. 0.145 against Franka's 0.102 is the
        //    4.3 cm gap that a forgotten environment variable currently hides.
        let franka = tool_offset(&arc(0.102, 2.0, 16), RULER, 1.0, T, JE).unwrap();
        assert!(
            (m.value[0] - franka.value[0]).abs() > 0.04,
            "two bodies 4.3 cm apart read {} and {} -- a probe that cannot separate them is not a \
             probe",
            m.value[0],
            franka.value[0]
        );
    }

    /// `contact_threshold`: it needs both classes and it refuses when they overlap.
    #[test]
    fn contact_threshold_refuses_what_it_cannot_answer() {
        use probe::{contact_threshold, Declined};
        const T: u64 = 1_000_000_000;
        const AW: u64 = 3;
        // Deterministic spreads: no RNG, so a failure here is always reproducible.
        let cls = |centre: f64, n: usize| -> Vec<f64> {
            (0..n).map(|i| centre + f64::from(i as u32 % 5) - 2.0).collect()
        };

        // -- must refuse: not enough of one class.
        assert_eq!(
            contact_threshold(&cls(10.0, 5), &cls(30.0, 20), Polarity::HigherOnContact, T, AW).unwrap_err(),
            Declined::NotEnoughSamples
        );

        // -- must refuse: the channel is stuck, not noiseless.
        assert_eq!(
            contact_threshold(&[7.0; 20], &[7.0; 20], Polarity::HigherOnContact, T, AW).unwrap_err(),
            Declined::NoResponse
        );

        // -- must refuse: contact reads LOWER than free space. The sets were swapped or the sign is
        //    inverted, and a threshold fitted to that fires in free space and stays silent on
        //    contact. Taking |mu_t - mu_f| would let both mistakes through looking healthy.
        assert_eq!(
            contact_threshold(&cls(30.0, 20), &cls(10.0, 20), Polarity::HigherOnContact, T, AW).unwrap_err(),
            Declined::Inconsistent
        );

        // -- must refuse: the two conditions read alike. There is no threshold to report, and
        //    reporting one ships a detector that fires at a rate nobody measured.
        assert_eq!(
            contact_threshold(&cls(10.0, 20), &cls(10.5, 20), Polarity::HigherOnContact, T, AW).unwrap_err(),
            Declined::Inconsistent
        );

        // -- must be ADMITTED: cleanly separated, threshold inside the gap nobody observed.
        let m = contact_threshold(&cls(10.0, 20), &cls(30.0, 20), Polarity::HigherOnContact, T, AW)
            .expect("two separated classes must be measurable");
        assert!(
            m.value[0] > 12.0 && m.value[0] < 28.0,
            "threshold {} landed on an observed sample instead of in the gap",
            m.value[0]
        );
        assert!(
            m.uncertainty[0] >= 7.9,
            "sigma {} is tighter than the gap in which every choice is equally consistent",
            m.uncertainty[0]
        );
        assert_eq!(
            m.deps[0].map(|d| d.0),
            Some(measurement::Quantity::StepDelivery),
            "这个判据读的是位移比,不是关节力矩 —— 它的参照是【自由空间的交付率】。\
             重挂到 ArmWeight 上,就等于让一台没有力矩通道的机器永远量不到接触阈"
        );
    }

    /// `floor`:**"一次都没被挡住"和"支撑面很低"必须是两个不同的答案。**
    #[test]
    fn floor_refuses_what_it_cannot_answer() {
        use probe::{floor, Declined};
        const T: u64 = 1_000_000_000;
        const CE: u64 = 5;
        // 一次完整的"降到碰上、抬起来、再降":自由段交付 0.88,压住段交付 0.03。
        let 一轮 = |底: f64, 起: f64| -> Vec<(f64, f64, f64)> {
            let mut v = Vec::new();
            let mut z = 起;
            while z - 0.01 > 底 {
                v.push((0.01, 0.0088, z));
                z -= 0.0088;
            }
            for _ in 0..4 {
                v.push((0.01, 0.0003, 底));
            }
            v
        };

        // —— 必须拒:整相都在空中。**这跟"支撑面在很低的地方"是两件事**,而"取最小 z"
        //    会把两者读成同一个数,于是一次没碰到桌子的标定会报出一个像模像样的桌面高度。
        let 空中: Vec<(f64, f64, f64)> = (0..30).map(|i| (0.01, 0.0088, 0.5 - 0.0088 * i as f64)).collect();
        assert_eq!(floor(&空中, 0.3, T, CE).unwrap_err(), Declined::NoResponse);

        // —— 必须拒:接触阈没量到。没有阈就没有"被挡住"这件事。
        assert_eq!(floor(&一轮(0.10, 0.30), 0.0, T, CE).unwrap_err(), Declined::MissingDependency);
        assert_eq!(floor(&一轮(0.10, 0.30), f64::NAN, T, CE).unwrap_err(), Declined::MissingDependency);

        // —— 必须拒:只被挡住一两下,给不出散布。
        let mut 少: Vec<(f64, f64, f64)> = 空中.clone();
        少.push((0.01, 0.0003, 0.10));
        少.push((0.01, 0.0003, 0.10));
        assert_eq!(floor(&少, 0.3, T, CE).unwrap_err(), Declined::NotEnoughSamples);

        // —— 必须拒:被挡住的位置比自由走过的最高点还高。探针一起步就顶着东西,或者
        //    交付比例的分母不是这一步的命令 —— 而**平均一下照样给得出一个正常的高度**。
        let 反常: Vec<(f64, f64, f64)> = (0..20)
            .map(|i| if i % 2 == 0 { (0.01, 0.0003, 0.40) } else { (0.01, 0.0088, 0.20) })
            .collect();
        assert_eq!(floor(&反常, 0.3, T, CE).unwrap_err(), Declined::Inconsistent);

        // —— 必须收下:三轮"降—抬—降",每轮都在 0.10 m 处被挡住。
        let mut 三轮 = 一轮(0.10, 0.30);
        三轮.extend(一轮(0.10, 0.30));
        三轮.extend(一轮(0.10, 0.30));
        let m = floor(&三轮, 0.3, T, CE).expect("反复撞到同一个面必须量得出来");
        assert!((m.value[0] - 0.10).abs() < 1e-6, "支撑面读成 {},应当是 0.10", m.value[0]);
        assert_eq!(m.deps[0].map(|d| d.0), Some(measurement::Quantity::ContactThreshold));

        // —— 支撑面必须是**被挡住的位置**,不是**走到过的最低点**。让最后一轮多走一步:
        //    最低点比真实台面低一个步长,而被挡住的那些样本仍然在台面上。
        let mut 走过头 = 三轮.clone();
        走过头.push((0.01, 0.0088, 0.09)); // 一步自由的、比台面还低的位置
        let m2 = floor(&走过头, 0.3, T, CE).expect("多一个自由样本不该让它拒绝");
        assert!(
            (m2.value[0] - 0.10).abs() < 1e-6,
            "支撑面被那个走过头的自由样本拉到了 {} —— 判据必须是【被挡住】,不是【最低】",
            m2.value[0]
        );
    }

    /// `image_ruler_along`:**塌成一条线的手眼必须拒绝,而求逆照样会给出一个数。**
    #[test]
    fn image_ruler_refuses_a_degenerate_hand_eye() {
        use probe::{image_ruler_along, Declined};
        // 一把干净的尺:x 走一米画面挪 400 单位(沿 u),y 走一米挪 400 单位(沿 v)。
        let jac = [400.0, 0.0, 0.0, 400.0];
        let sig = [4.0, 4.0, 4.0, 4.0];
        let (r, s) = image_ruler_along(&jac, &sig, (1.0, 0.0)).expect("正交的手眼必须给得出尺");
        assert!((r - 400.0).abs() < 1e-9, "沿 u 的尺读成 {r}");
        assert!(s > 0.0, "尺必须带自己的 1σ,否则下游会把它当精确值");
        // 斜 45° 方向上,尺仍然是 400(各向同性),这是这个函数的正确性锚点。
        let (r45, _) = image_ruler_along(&jac, &sig, (1.0, 1.0)).unwrap();
        assert!((r45 - 400.0).abs() < 1e-9, "45° 上读成 {r45}");

        // 各向异性:x 走一米挪 400,y 走一米挪 100 ⇒ 沿 v 的尺必须是 100,不是 400。
        // **随便挑一列当尺,这里就会错 4 倍,而两个数看起来同样正常。**
        let 扁 = [400.0, 0.0, 0.0, 100.0];
        let (ru, _) = image_ruler_along(&扁, &sig, (1.0, 0.0)).unwrap();
        let (rv, _) = image_ruler_along(&扁, &sig, (0.0, 1.0)).unwrap();
        assert!((ru - 400.0).abs() < 1e-9 && (rv - 100.0).abs() < 1e-9, "各向异性读成 {ru}/{rv}");

        // —— 必须拒:两列共线 ⇒ 水平面在画面里塌成一条线,这个方向上的尺不存在。
        let 共线 = [400.0, 200.0, 800.0, 400.0];
        assert_eq!(
            image_ruler_along(&共线, &sig, (1.0, 0.0)).unwrap_err(),
            Declined::Inconsistent
        );
        // —— 必须拒:没有方向,就没有"这个方向上的尺"。
        assert_eq!(
            image_ruler_along(&jac, &sig, (0.0, 0.0)).unwrap_err(),
            Declined::MissingDependency
        );
    }

    /// `arm_weight_by_asymmetry`:**没有力矩通道照样量得到胳膊有多重。**
    ///
    /// 撤回记录:我连着三次判"这台机器没有力矩通道 ⇒ 臂重量不了、天花板 14/15"。
    /// 错在把 [`probe::arm_weight`] 的入参当成了那个量的规格。这一层里凡是跟力有关的
    /// 量全从交付比例里读,臂重没有理由例外。
    #[test]
    fn arm_weight_needs_no_torque_channel() {
        use probe::{arm_weight_by_asymmetry, Declined};
        const T: u64 = 1_000_000_000;
        const RE: u64 = 9;
        // 一条"每米力臂亏 0.12 交付比例"的胳膊,外加 0.01 的力臂无关偏置。
        // 上行交付 = 基准 − 亏损,下行 = 基准 + 亏损。
        let 造 = |每米: f64, 偏置: f64, n: usize| -> Vec<(f64, f64, f64)> {
            (0..n)
                .map(|i| {
                    let l = 0.10 + 0.30 * (i as f64) / ((n - 1) as f64);
                    let g = 每米 * l + 偏置;
                    (l, 0.89 - g, 0.89 + g)
                })
                .collect()
        };
        let m = arm_weight_by_asymmetry(&造(0.12, 0.01, 9), T, RE).expect("上下不对称就该量得出来");
        assert!((m.value[0] - 0.12).abs() < 1e-9, "每米亏损读成 {}", m.value[0]);
        assert!((m.value[1] - 0.01).abs() < 1e-9, "力臂无关那一份读成 {}", m.value[1]);
        assert_eq!(m.deps[0].map(|d| d.0), Some(measurement::Quantity::Reach), "力臂从基座算,基座重标它就该失效");

        // —— 必须收下、且是零:重力补偿做得好的胳膊斜率就是零,那是**测量不是拒绝**
        //    (照 backlash 的先例)。
        let z = arm_weight_by_asymmetry(&造(0.0, 0.0, 9), T, RE).expect("没有重力负载是一具真实的身体");
        assert!(z.value[0].abs() < 1e-9, "补偿完好的胳膊读成 {}", z.value[0]);

        // —— 必须拒:所有点在同一个力臂上。**重量和一个力臂无关的偏置在这里完全共线**,
        //    任何观测同时兼容两者。
        let 一个力臂: Vec<(f64, f64, f64)> = (0..9).map(|_| (0.25, 0.85, 0.93)).collect();
        assert_eq!(arm_weight_by_asymmetry(&一个力臂, T, RE).unwrap_err(), Declined::Inconsistent);

        // —— 必须拒:往上反而比往下交付得多。z 反了,或者这根本不是重力。
        //    取绝对值会把它读成一次健康的测量,而它意味着下游每一个"往下压"都是反的。
        assert_eq!(
            arm_weight_by_asymmetry(&造(-0.12, 0.0, 9), T, RE).unwrap_err(),
            Declined::Inconsistent
        );

        // —— 必须拒:样本不够。
        assert_eq!(
            arm_weight_by_asymmetry(&造(0.12, 0.0, 4), T, RE).unwrap_err(),
            Declined::NotEnoughSamples
        );
    }

    /// `latency_from_beats`:**命令发出之前身体就没停住时,"第几拍才动"量的是余振。**
    #[test]
    fn latency_refuses_a_body_that_was_still_ringing() {
        use probe::{latency_from_beats, Declined};
        const T: u64 = 1_000_000_000;
        // 静下来了的身体:静止段逐拍衰减到噪声,命令后第 2 拍动。
        let 静 = [0.004f64, 0.002, 0.001, 0.0002, 0.0001, 0.0001];
        let 动: Vec<(u32, f64)> = vec![(0, 0.0001), (1, 0.0001), (2, 0.016), (3, 0.002), (4, 0.0003)];
        let m = latency_from_beats(&静, &动, T).expect("静下来了就该量得出延迟");
        assert_eq!(m.value[0], 2.0, "延迟读成 {}", m.value[0]);

        // —— 必须拒:命令之前身体还在余振(静止段后一半反而更大)。
        //    🔴 这正是实测那一条:报了 6 拍,而同一相里一拍交付 89%。
        let 余振 = [0.0001f64, 0.0001, 0.0002, 0.003, 0.004, 0.005];
        assert_eq!(latency_from_beats(&余振, &动, T).unwrap_err(), Declined::Inconsistent);

        // —— 必须拒:命令了,什么都没超过静止噪声。这不是"延迟很大",是没反应。
        let 没动: Vec<(u32, f64)> = vec![(0, 0.0001), (1, 0.0001), (2, 0.00012), (3, 0.0001)];
        assert_eq!(latency_from_beats(&静, &没动, T).unwrap_err(), Declined::NoResponse);

        // —— 必须拒:样本不够。
        assert_eq!(latency_from_beats(&静[..2], &动, T).unwrap_err(), Declined::NotEnoughSamples);
    }

    /// `base_from_stalls`:**基座不在观测契约里,但它自己走不动的那几个地方把它交代了。**
    #[test]
    fn base_from_stalls_recovers_a_shoulder_nobody_reported() {
        use probe::{base_from_stalls, Declined};
        const T: u64 = 1_000_000_000;
        let 真基座 = [0.31f64, -0.12, 0.47];
        let 真半径 = 0.62f64;
        // 六条方向不同的射线,各自在同一个球面上走不动。
        let 方向 = [
            [1.0f64, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [0.6, 0.8, 0.0],
        ];
        let 卡住: Vec<([f64; 3], [f64; 3])> = 方向
            .iter()
            .map(|d| {
                let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                let u = [d[0] / n, d[1] / n, d[2] / n];
                ([真基座[0] + 真半径 * u[0], 真基座[1] + 真半径 * u[1], 真基座[2] + 真半径 * u[2]], u)
            })
            .collect();
        let m = base_from_stalls(&卡住, T).expect("同一球面上的六个点必须解得出球心");
        for i in 0..3 {
            assert!((m.value[i] - 真基座[i]).abs() < 1e-6, "球心第 {i} 维读成 {}", m.value[i]);
        }
        assert!((m.value[3] - 真半径).abs() < 1e-6, "半径读成 {}", m.value[3]);

        // —— 必须拒:三个点定不了球。
        assert_eq!(base_from_stalls(&卡住[..3], T).unwrap_err(), Declined::NotEnoughSamples);
        // —— 必须拒:四个点共面 ⇒ 球心不唯一,**而消元照样会给出一个坐标**。
        let e = [0.0f64, 0.0, 1.0];
        let 共面 = [
            ([0.0, 0.0, 0.0], e), ([1.0, 0.0, 0.0], e), ([0.0, 1.0, 0.0], e),
            ([1.0, 1.0, 0.0], e), ([0.5, 0.5, 0.0], e),
        ];
        assert_eq!(base_from_stalls(&共面, T).unwrap_err(), Declined::Inconsistent);
        // —— 必须拒:内边界的点混进来了。它们是**朝着球心走的时候**卡住的
        //    (自碰 / 关节限位),而外边界的定义是"再往外就够不着"。
        //    残差检查放它们过去了:混进三个内点,拟出半径 0.498、残差 0.252,
        //    `残差 < 半径` 照样成立 —— 所以判据必须是方向,不是残差大小。
        let mut 两层 = 卡住.clone();
        for i in 0..3usize {
            let mut p = 真基座;
            let mut d = [0.0f64; 3];
            p[i] += 0.05;
            d[i] = -1.0; // 朝着球心走
            两层.push((p, d));
        }
        assert_eq!(base_from_stalls(&两层, T).unwrap_err(), Declined::Inconsistent);
    }

    /// `image_ruler_forward`:**病态的手眼矩阵不该让一段长度变得不可测。**
    #[test]
    fn a_ruler_that_needs_no_inverse_survives_a_degenerate_hand_eye() {
        use probe::{image_ruler_forward, image_to_plane, Declined};
        // 一副**近乎共线**的手眼:两列几乎指同一个画面方向 ⇒ 逆不成立。
        // 实测这台机器的水平 2×2 行列式 −0.208 与自己的 1σ 分不开,正是这一档。
        let 病 = [400.0, 200.0, 802.0, 401.0];
        let sig = [4.0, 4.0, 4.0, 4.0];
        // 求逆那条路必须拒 —— 而它拒得对。
        assert_eq!(image_to_plane(&病, &sig, (1.0, 0.5)).unwrap_err(), Declined::Inconsistent);
        // 正向那条路照样给得出尺:沿第一列那个方向,一米 = |(400,200)| = 447.2 画面单位。
        let (r, s) = image_ruler_forward(&病, &sig, (400.0, 200.0)).expect("被压扁的方向照样有确定的模长");
        assert!((r - 447.2135955).abs() < 0.5, "尺读成 {r}");
        assert!(s > 0.0, "尺必须带自己的 1σ");

        // —— 必须拒:要的画面方向**垂直于**这具身体在水平面里动得出来的那条线。
        //    那说明配到的两瓣不是钳口。
        assert_eq!(
            image_ruler_forward(&病, &sig, (-200.0, 400.0)).unwrap_err(),
            Declined::Inconsistent
        );
        // 良态的手眼上,两条路必须给出同一个数(锚点)。
        let 好 = [400.0, 0.0, 0.0, 400.0];
        let (rf, _) = image_ruler_forward(&好, &sig, (1.0, 0.0)).unwrap();
        assert!((rf - 400.0).abs() < 1.0, "良态时正向读成 {rf},应当与求逆那条一致");
    }

    /// `image_to_plane`:**斜着看的相机把圆压成椭圆,而一把标量尺读不出那个圆的半径。**
    #[test]
    fn image_to_plane_recovers_a_circle_a_scalar_ruler_would_distort() {
        use probe::{image_ruler_along, image_to_plane};
        // 一台斜着看桌面的相机:x 走一米挪 400 单位,y 走一米只挪 100(前后方向被压扁)。
        let jac = [400.0, 0.0, 0.0, 100.0];
        let sig = [4.0, 4.0, 4.0, 4.0];
        // 水平面里一个半径 0.05 m 的圆,投影成 (0.05*400, 0.05*100) 的椭圆。
        let 半径 = 0.05f64;
        for i in 0..16 {
            let a = std::f64::consts::TAU * i as f64 / 16.0;
            let (uv_u, uv_v) = (半径 * a.cos() * 400.0, 半径 * a.sin() * 100.0);
            let ((x, y), _) = image_to_plane(&jac, &sig, (uv_u, uv_v)).unwrap();
            assert!(
                (x.hypot(y) - 半径).abs() < 1e-9,
                "换回米之后半径读成 {},应当处处是 {}",
                x.hypot(y),
                半径
            );
        }
        // 而一把标量尺在这两个方向上差 4 倍 —— 这就是"先换米、再拟圆"存在的理由。
        let (ru, _) = image_ruler_along(&jac, &sig, (1.0, 0.0)).unwrap();
        let (rv, _) = image_ruler_along(&jac, &sig, (0.0, 1.0)).unwrap();
        assert!(ru / rv > 3.9, "这个反例本身要成立:两个方向的尺必须真的差着倍数");
    }

    /// `tool_axis_column`:**腕根本没转的时候,"散布最小的那一根"是噪声的排序。**
    #[test]
    fn tool_axis_column_refuses_a_wrist_that_did_not_turn() {
        use probe::{tool_axis_column, Declined};
        const T: u64 = 1_000_000_000;
        const JE: u64 = 7;
        // 绕第 `axis` 列转时工作点不动;另外两列扫出半径 `r` 的弧。
        let 弧 = |axis: usize, r: f64, 抖: f64| -> Vec<(u32, f64, f64, f64)> {
            let mut v = Vec::new();
            for c in 0..3usize {
                for k in 0..8u32 {
                    let a = 0.15 * k as f64;
                    let rad = if c == axis { 抖 } else { r };
                    v.push((c as u32, a, 0.5 + rad * a.cos(), 0.5 + rad * a.sin()));
                }
            }
            v
        };

        // —— 必须拒:三列都不动 ⇒ 腕没转,或者工作点根本没被看见。
        assert_eq!(tool_axis_column(&弧(0, 0.0, 0.0), T, JE).unwrap_err(), Declined::NoResponse);
        // —— 必须拒:少一列。
        let mut 缺一列 = 弧(1, 0.05, 0.0);
        缺一列.retain(|s| s.0 != 2);
        assert_eq!(tool_axis_column(&缺一列, T, JE).unwrap_err(), Declined::NotEnoughSamples);
        // —— 必须拒:三列扫出的弧一样大 ⇒ 分不出哪一根是工具轴。**指错一根轴,
        //    每一次"绕工具轴微调"都会把工具尖甩出去。**
        assert_eq!(tool_axis_column(&弧(9, 0.05, 0.0), T, JE).unwrap_err(), Declined::Inconsistent);
        // —— 必须收下:第 1 列不动,另外两列扫出明显的弧。
        let m = tool_axis_column(&弧(1, 0.05, 0.0), T, JE).expect("一列明显不动必须指认得出来");
        assert_eq!(m.value[0], 1.0, "工具轴指成了第 {} 列", m.value[0]);
        assert!(m.uncertainty[0] > 0.0, "余量必须报出来 —— 勉强分开和干脆利落不是一回事");
    }

    /// `self_occlusion`: the all-zero map is the case that matters.
    #[test]
    fn self_occlusion_refuses_what_it_cannot_answer() {
        use probe::{self_occlusion, Declined, OCCLUSION_CELLS};
        const T: u64 = 1_000_000_000;
        const JE: u64 = 11;
        let sweep = |n: usize, f: &dyn Fn(usize) -> u32| -> Vec<(f64, u32)> {
            (0..n).map(|k| (0.1 * k as f64, f(k))).collect()
        };

        // -- must refuse: a frequency over eight poses is a coin flip with a decimal point on it.
        assert_eq!(
            self_occlusion(&sweep(8, &|k| 1u32 << (k % 20)), T, JE).unwrap_err(),
            Declined::NotEnoughSamples
        );

        // -- must refuse: every sample at one pose. The map then describes a pose, not a body.
        let still: Vec<(f64, u32)> = (0..30).map(|k| (0.5, 1u32 << (k % 20))).collect();
        assert_eq!(self_occlusion(&still, T, JE).unwrap_err(), Declined::Inconsistent);

        // -- 🔴 must refuse: nothing occluded anywhere. "No self-occlusion" and "the silhouette
        //    detector never fired" produce the identical output, and the second is far more common.
        //    This project has filed the absence of expected failures as a clean result before.
        assert_eq!(
            self_occlusion(&sweep(30, &|_| 0), T, JE).unwrap_err(),
            Declined::NoResponse
        );

        // -- must refuse: everything occluded in every pose. A detector stuck on.
        let full = (1u32 << OCCLUSION_CELLS) - 1;
        assert_eq!(
            self_occlusion(&sweep(30, &|_| full), T, JE).unwrap_err(),
            Declined::NoResponse
        );

        // -- 🔴 must refuse: the covered region never moved. Whatever is blocking the view is not
        //    moving with the arm -- a bracket, a smudge, a cable -- so it is not self-occlusion, and
        //    attributing it to the body puts a dirty lens into every "can I see that pixel" answer.
        assert_eq!(
            self_occlusion(&sweep(30, &|_| 0b1010_1010), T, JE).unwrap_err(),
            Declined::Inconsistent
        );

        // -- must be ADMITTED: a silhouette that sweeps across the frame.
        let m = self_occlusion(&sweep(30, &|k| 1u32 << (k % 20)), T, JE)
            .expect("a moving silhouette must be measurable");
        assert_eq!(m.dim, OCCLUSION_CELLS);
        assert!(
            (m.value[0] - 2.0 / 30.0).abs() < 1e-12,
            "cell 0 was covered on 2 of 30 poses, read {}",
            m.value[0]
        );
        assert_eq!(m.value[20], 0.0, "cell 20 was never covered");
        assert!(
            m.uncertainty[20] > 0.0,
            "a cell never seen occluded is not a cell KNOWN never to be occluded; a sigma of 0 \
             there claims a certainty 30 observations cannot supply"
        );
        assert!(
            m.valid_lo[0] < m.valid_hi[0] && m.valid_hi[0] > 2.8,
            "validity must be the poses actually swept, got [{}, {}]",
            m.valid_lo[0],
            m.valid_hi[0]
        );
        assert_eq!(
            m.deps[0].map(|d| d.0),
            Some(measurement::Quantity::ImageJacobian),
            "the map is in the camera's frame; knock the camera and every cell is wrong"
        );
    }

    /// `reach` must report a band only where both walls were actually straddled.
    ///
    /// The refusal in the middle of this test is the one that matters. A base-separation sweep run
    /// at 0.75 m produced **no** sample inside the inner limit, so it carries no evidence about
    /// where that limit sits -- and a fit that answered anyway would be indistinguishable
    /// downstream from one that had measured it.
    #[test]
    fn reach_refuses_a_wall_it_never_touched() {
        use probe::{reach, Declined};
        const T: u64 = 1_000_000_000;
        // Attained between 0.33 and 0.60 m from the base; failed outside. `n` samples spread
        // linearly over [lo, hi].
        let band = |lo: f64, hi: f64, n: usize| -> Vec<(f64, bool)> {
            (0..n)
                .map(|i| {
                    let r = lo + (hi - lo) * (i as f64) / ((n - 1) as f64);
                    (r, (0.33..=0.60).contains(&r))
                })
                .collect()
        };

        // -- must refuse: two edges cannot be located from a handful of points.
        assert_eq!(reach(&band(0.10, 0.90, 8), T).unwrap_err(), Declined::NotEnoughSamples);

        // -- must refuse: swept only the far half, so nothing was ever tried inside the inner
        //    wall. This is the 0.75 m sweep, and the estimator must NOT invent its inner edge.
        assert_eq!(reach(&band(0.40, 0.90, 30), T).unwrap_err(), Declined::Inconsistent);

        // -- must refuse: the mirror case -- swept only the near half, outer wall never touched.
        assert_eq!(reach(&band(0.05, 0.50, 30), T).unwrap_err(), Declined::Inconsistent);

        // -- must refuse: attained nothing anywhere. A dead arm is not a narrow band.
        let dead: Vec<(f64, bool)> =
            (0..30).map(|i| (0.05 + 0.03 * f64::from(i), false)).collect();
        assert_eq!(reach(&dead, T).unwrap_err(), Declined::NoResponse);

        // -- must refuse: a FLAT curve. Attained ~85% everywhere, failures scattered rather than
        //    massed at an edge -- the sweep never approached either limit, so it holds no evidence
        //    about where they are. This is the case that actually occurred: 2174 real episodes ran
        //    73–100% attainment with no trend in radius, and an earlier version of this estimator
        //    answered them with crisp-looking bands that were slivers of noise. Left untested, the
        //    probe's most common real input is the one it silently gets wrong.
        let flat: Vec<(f64, bool)> = (0..80)
            .map(|i| (0.10 + 0.008 * f64::from(i), i % 7 != 0))
            .collect();
        assert_eq!(reach(&flat, T).unwrap_err(), Declined::Inconsistent,
                   "a flat attainment curve contains no wall; reporting one is the failure mode \
                    that this probe exists to prevent");

        // -- must be ADMITTED: a sweep that straddles both walls, with one unlucky interior
        //    failure that must NOT truncate the band (a collision is not a wall).
        let mut s = band(0.05, 0.95, 40);
        let mid = s.iter().position(|&(r, _)| r > 0.45).unwrap();
        s[mid].1 = false;
        let m = reach(&s, T).expect("a sweep that straddles both walls must be admitted");
        assert!((m.value[0] - 0.33).abs() < 0.05,
                "inner wall read {} -- expected ~0.33", m.value[0]);
        assert!((m.value[1] - 0.60).abs() < 0.05,
                "outer wall read {} -- one interior failure truncated the band", m.value[1]);
        assert!(m.valid_lo[0] <= 0.06 && m.valid_hi[0] >= 0.94,
                "validity must be the radial span actually swept, got [{}, {}]",
                m.valid_lo[0], m.valid_hi[0]);
        assert!(m.uncertainty[0] > 0.0 && m.uncertainty[1] > 0.0,
                "an edge bracketed by two samples has non-zero width; reporting 0 would claim \
                 a precision the sweep does not have");

        // -- two different mountings must not read the same. A probe that cannot separate them
        //    is measuring the sweep, not the body.
        let s2: Vec<(f64, bool)> = (0..40)
            .map(|i| { let r = 0.05 + 0.9 * (i as f64) / 39.0; (r, (0.15..=0.40).contains(&r)) })
            .collect();
        let m2 = reach(&s2, T).unwrap();
        assert!(m2.value[1] < m.value[1] - 0.1,
                "the two mountings read outer walls {} and {}", m.value[1], m2.value[1]);
    }
}
