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
pub mod execute;
#[cfg(feature = "fast")]
pub mod fast;
#[cfg(not(feature = "fast"))]
pub mod faststub;
pub mod hand;
pub mod measurement;
pub mod persist;
pub mod refuse;

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

    /// How many hand-filled constants this body is carrying: **must be zero**.
    ///
    /// Nothing can enter through [`Body::submit`] without a passing self-test, so the count is a
    /// structural zero rather than a promise. It is exposed anyway, because the number that has to
    /// be reported to an auditor is the number, not the argument for why it must be right.
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
    use measurement::{MAX_DEPS, MAX_DIM};

    fn m(q: Quantity, v: f64, sigma: f64, valid_for_ns: u64) -> Measurement {
        let mut x = Measurement {
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

    fn ask_for(q: Quantity) -> Ask {
        let mut a = Ask::EMPTY;
        a.needs[0] = Some(q);
        a
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
    use measurement::{MAX_DEPS, MAX_DIM};

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
            quantity: Quantity::ImageJacobian,
            dim: 18,
            value: [0.0; MAX_DIM],
            uncertainty: [0.01; MAX_DIM],
            // The probed range must actually contain the measured value; the store rejects it
            // otherwise, which is how this test found its own bug.  800 px/rad is the order the
            // rig really reports (recorded column norms: 792 / 842 / 904 px/m).
            valid_lo: [-2000.0; MAX_DIM],
            valid_hi: [2000.0; MAX_DIM],
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

    /// The Ada core must reject a NaN limit, halt on an out-of-envelope command rather than clamp
    /// it, keep the safe hold inside the envelope, and still admit a good command.
    #[test]
    fn the_proven_core_refuses_when_it_must() {
        let f = Fast::reset();
        f.selftest().expect("the proven fast face did not behave as proved");
    }

    /// The watchdog must latch on silence, and a perfectly good command must NOT walk it back out.
    /// A safety state you can leave by doing nothing is not a safety state.
    #[test]
    fn a_latch_is_a_latch() {
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
