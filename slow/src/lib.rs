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
pub mod probe;
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
