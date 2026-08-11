//! Is this body **touching something**, or has it simply run out of solution?
//!
//! # The measurement this exists because of
//!
//! This body has no torque channel at all, so contact is read from the fraction of a commanded
//! motion that actually arrives: free space **0.9995**, pressed against a surface **0.0055** —
//! 181× apart with zero overlap over 66 samples, threshold **0.29383 ± 0.00039**. That ruler is
//! real and it is this repo's own (`results/contact_aug2026/`).
//!
//! It is also, on its own, **unable to tell contact from having no solution**. Probed at nine
//! points on a flat conveyor (`results/stallwhat_aug2026/`):
//!
//! | point | commanded (down) | reverse (up) | stalled at | what it was |
//! |---|---|---|---|---|
//! | p5 p7 p8 p6 | blocked | 1.000 / 1.000 / 1.000 / 1.000 | 0.9059–0.9278 | the belt |
//! | p0 | blocked | **0.299** | 0.8475 | 🔴 not a surface |
//! | p1 | blocked | **0.572** | 0.8343 | 🔴 not a surface |
//!
//! The four true ones cluster within **2.2 cm on a flat belt**; p0 and p1 stall **7–9 cm below that
//! plane** — there is no belt there to touch — and, decisively, **they cannot lift off**. A hand
//! resting on a surface can always lift off it.
//!
//! 🔴 A first version of this file claimed five of the nine were false, counting three points whose
//! *sideways* delivery was low (0.090–0.589). Friction blocks sideways motion on a perfectly real
//! surface, so that is not evidence of anything. Two, not five. The rule below counts only the
//! reverse direction, which has no such escape.
//!
//! # The rule, and why it generalises
//!
//! **A surface can block a direction. It can never block that direction's opposite.**
//!
//! So contact needs a *witness*, and the witness must be the **reverse** of the commanded axis. Not
//! a heuristic about conveyors: it holds on any body, with or without a force sensor, in any
//! simulator and on real hardware. It costs one extra command.
//!
//! What counts as "free" is not typed in either. It is the midpoint between the two numbers this
//! body has already measured about itself — `step_delivery` (what a command delivers in free space)
//! and `contact_threshold` — so a body that moves sluggishly in free space automatically gets a
//! lower bar, with nobody retuning anything.
//!
//! Without this the failure is the worst kind here: the delivery reading is correct, the threshold
//! is correct, the conclusion is self-consistent, and it is false. The same shape as the LAB's
//! *"a correct probe gives a self-consistent WRONG conclusion when the labels are wrong"*.
//!
//! # What this does NOT decide
//!
//! *What* was touched. Belt, object, or the robot's own body all read the same here. The layer
//! answers "something is in the way, and I am not stuck"; which thing it is belongs to whatever can
//! see.

use crate::measurement::Quantity;
use crate::refuse::{Reason, Verdict};
use crate::Body;

/// What the delivery readings, taken together, say about the hand.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Touch {
    /// Cannot answer. Either the ruler is not admissible, or no second direction was asked.
    Unknown = 0,
    /// The command arrived. Nothing in the way along it.
    Free = 1,
    /// Blocked along the commanded axis, and at least one other direction is still free.
    Contact = 2,
    /// 🔴 Blocked along the commanded axis **and** along every direction that was asked.
    ///
    /// Not contact. This body has no solution at this pose, and the delivery ruler cannot tell the
    /// difference on its own — which is the entire reason this module exists.
    Stuck = 3,
}

impl Touch {
    /// Stable name over the ABI. One table, so a name cannot go missing the way `bl_reason_str`
    /// once truncated an entire enum by hand-mirroring it.
    pub const fn as_str(self) -> &'static str {
        match self {
            Touch::Unknown => "unknown",
            Touch::Free => "free",
            Touch::Contact => "contact",
            Touch::Stuck => "stuck",
        }
    }

    /// Round-trip from the ABI's `u32`.
    pub const fn from_u32(v: u32) -> Option<Touch> {
        match v {
            0 => Some(Touch::Unknown),
            1 => Some(Touch::Free),
            2 => Some(Touch::Contact),
            3 => Some(Touch::Stuck),
            _ => None,
        }
    }
}

/// The answer, and the audit trail for it.
#[derive(Copy, Clone, Debug)]
pub struct Reading {
    /// What the readings say.
    pub touch: Touch,
    /// `admit` is true for [`Touch::Contact`] and [`Touch::Free`]. `why` names what blocked an
    /// answer otherwise, and `unverified` carries the third rung through: answered, and the ruler
    /// was never probed at this command size.
    pub verdict: Verdict,
    /// The bar the reverse direction had to clear, derived from this body's own `step_delivery` and
    /// `contact_threshold`. `NaN` when it was never computed. Recorded so a caller can see how
    /// close the call was rather than only that it was made.
    pub free_bar: f64,
    /// Carried through untouched. Friction blocks sideways motion on a real surface, so this never
    /// decides anything — it is here because it is the reading that made a first version of this
    /// file call three real contacts false.
    pub sideways: Option<f64>,
}

/// 🔴 Delivered fractions are the units here, **not newtons**. Each argument is *how much of a
/// commanded motion arrived, in one control period*. The stored quantity is called
/// `contact_threshold` and a caller would very reasonably assume force; the calibration row says
/// otherwise in its own text, and so does this.
///
/// `delivered_reverse` must come from commanding the **opposite** direction **at the same
/// magnitude**. Same magnitude matters: the threshold was probed at one command size, and comparing
/// a different size against it is the mismatch that already cost this repo three episodes and a 3×
/// regression in closest approach.
///
/// `sideways` is recorded and never decides. Friction blocks sideways motion on a real surface.
pub fn touching(
    body: &Body,
    delivered_along: f64,
    delivered_reverse: Option<f64>,
    sideways: Option<f64>,
    now_ns: u64,
) -> Reading {
    fn out(touch: Touch, verdict: Verdict, free_bar: f64, sideways: Option<f64>) -> Reading {
        Reading {
            touch,
            verdict,
            free_bar,
            sideways,
        }
    }
    let bad = |why: Reason, q: Quantity| Reading {
        touch: Touch::Unknown,
        verdict: Verdict::refuse(why, q),
        free_bar: f64::NAN,
        sideways,
    };

    let finite_nonneg = |x: f64| x.is_finite() && x >= 0.0;
    if !finite_nonneg(delivered_along) {
        return bad(Reason::OutOfRange, Quantity::ContactThreshold);
    }
    if delivered_reverse.is_some_and(|r| !finite_nonneg(r)) {
        return bad(Reason::OutOfRange, Quantity::ContactThreshold);
    }

    // Both rulers go through the ordinary gate: never-measured, stale and a failed self-test refuse
    // here exactly as they would for any other read.
    let Some(thr) = body.get(Quantity::ContactThreshold) else {
        return bad(Reason::NeverMeasured, Quantity::ContactThreshold);
    };
    if !thr.selftest_passed {
        return bad(Reason::SelfTestFailed, Quantity::ContactThreshold);
    }
    if thr.valid_for_ns != 0 && now_ns.saturating_sub(thr.measured_at_ns) > thr.valid_for_ns {
        return bad(Reason::Stale, Quantity::ContactThreshold);
    }
    let t = thr.value[0];
    if !(t.is_finite() && t > 0.0) {
        return bad(Reason::NeverMeasured, Quantity::ContactThreshold);
    }

    // 🔴 Polarity is not assumed and is not a free parameter: for a *delivered fraction*,
    // less-is-more-blocked is the definition of the quantity. The place polarity genuinely varies —
    // a force or current channel — is `probe::contact_threshold`, which takes it as a required
    // argument precisely because hard-coding "contact reads HIGHER" there once refused this body's
    // own validated ruler.
    if delivered_along >= t {
        return out(Touch::Free, Verdict::OK, f64::NAN, sideways);
    }

    let Some(rev) = delivered_reverse else {
        // Blocked, and nobody asked whether this body could still back out. That is the reading
        // that was wrong twice in nine on a flat belt, with nothing in the log disagreeing.
        return bad(Reason::NoEvidence, Quantity::ContactThreshold);
    };
    // "Free" is the midpoint between the two things this body measured about itself. Using the
    // contact threshold alone as the bar would have passed p0 at 0.299 — 0.005 above it — while the
    // arm was moving 30% of a command it should deliver 99.99% of.
    let Some(sd) = body.get(Quantity::StepDelivery) else {
        return bad(Reason::NeverMeasured, Quantity::StepDelivery);
    };
    if !sd.selftest_passed {
        return bad(Reason::SelfTestFailed, Quantity::StepDelivery);
    }
    let f = sd.value[0];
    if !(f.is_finite() && f > t) {
        // Free space delivers no more than a blocked command does: nothing here can discriminate.
        return bad(Reason::SelfTestFailed, Quantity::StepDelivery);
    }
    let bar = 0.5 * (t + f);

    if rev < bar {
        // Blocked going in, and blocked coming back out. A surface cannot do that.
        return out(
            Touch::Stuck,
            Verdict::refuse(Reason::Unreachable, Quantity::ContactThreshold),
            bar,
            sideways,
        );
    }

    // Blocked along the command, free in reverse: something is in the way and this body is not.
    //
    // The third rung survives: the ruler was probed at exactly one command magnitude (marked
    // `AxisKind::Unmeasured`), so any other size is answered and flagged rather than silently
    // agreed with.
    let v = if thr.axis_kind[0] == crate::measurement::AxisKind::Unmeasured {
        Verdict::unverified(Quantity::ContactThreshold)
    } else {
        Verdict::OK
    };
    out(Touch::Contact, v, bar, sideways)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measurement::{AxisKind, Measurement, MAX_DEPS, MAX_DIM};

    const T: f64 = 0.29383; // this body's measured contact threshold
    const SD: f64 = 0.9999; // ... and what it delivers in free space
    const NOW: u64 = 1_000_000_000;

    fn q(quantity: Quantity, v: f64, kind: AxisKind, valid_for_ns: u64) -> Measurement {
        let mut m = Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity,
            dim: 1,
            value: [0.0; MAX_DIM],
            uncertainty: [0.0; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [0.0; MAX_DIM],
            measured_at_ns: 0,
            valid_for_ns,
            deps: [None; MAX_DEPS],
            epoch: 1,
            selftest_passed: true,
            prev_epoch: 0,
        };
        m.axis_kind[0] = kind;
        m.value[0] = v;
        m.valid_hi[0] = 1.0;
        m
    }

    fn body() -> Body {
        let mut b = Body::new();
        b.submit(q(Quantity::ContactThreshold, T, AxisKind::Unmeasured, 0)).unwrap();
        b.submit(q(Quantity::StepDelivery, SD, AxisKind::Interval, 0)).unwrap();
        b
    }

    /// 🔴 THE TWO POINTS THAT WERE NOT TOUCHING ANYTHING, replayed from
    /// `results/stallwhat_aug2026/rollout_g0.jsonl`. Both reported contact under the old rule;
    /// both stalled 7-9 cm BELOW the belt plane the other four points agree on; and neither could
    /// lift off. If this ever goes green on `Contact` again, the bug is back.
    #[test]
    fn the_two_points_that_could_not_lift_off() {
        let b = body();
        for (name, up, sideways) in [("p0", 0.299, 0.698), ("p1", 0.572, 0.001)] {
            let r = touching(&b, 0.0, Some(up), Some(sideways), NOW);
            assert_eq!(r.touch, Touch::Stuck, "{name}");
            assert_eq!(r.verdict.why, Reason::Unreachable);
            assert!(!r.verdict.admit);
        }
    }

    /// 🔴 The correction, welded in. Three points had LOW sideways delivery and a free reverse --
    /// a first version of this file called them false contacts. Friction blocks sideways motion on
    /// a perfectly real surface; only the reverse direction decides.
    #[test]
    fn low_sideways_with_a_free_reverse_is_still_contact() {
        let b = body();
        for (name, up, sideways) in [("p2", 1.000, 0.329), ("p3", 0.919, 0.589), ("p4", 0.972, 0.090)] {
            let r = touching(&b, 0.0, Some(up), Some(sideways), NOW);
            assert_eq!(r.touch, Touch::Contact, "{name}: sideways must not decide");
        }
    }

    /// The four that WERE on the belt must still read contact -- a rule that only says "no" is not
    /// a discriminator.
    #[test]
    fn the_four_points_on_the_belt() {
        let b = body();
        for (name, up) in [("p5", 1.0), ("p6", 1.0), ("p7", 1.0), ("p8", 1.0)] {
            let r = touching(&b, 0.0055, Some(up), Some(0.99), NOW);
            assert_eq!(r.touch, Touch::Contact, "{name}");
            assert!(r.verdict.admit);
            assert!(r.verdict.unverified, "probed at one command size only: flagged, not hidden");
        }
    }

    /// The bar comes from the body, not from this file: halve free-space delivery and the bar
    /// halves with it, so a sluggish body is not declared stuck for being sluggish.
    #[test]
    fn the_free_bar_is_derived_from_this_bodys_own_two_numbers() {
        let b = body();
        let r = touching(&b, 0.0, Some(1.0), None, NOW);
        assert!((r.free_bar - 0.5 * (T + SD)).abs() < 1e-12);

        let mut slow = Body::new();
        slow.submit(q(Quantity::ContactThreshold, T, AxisKind::Unmeasured, 0)).unwrap();
        slow.submit(q(Quantity::StepDelivery, 0.40, AxisKind::Interval, 0)).unwrap();
        // 0.35 is below the fast body's bar (0.647) and above the sluggish one's (0.347)
        assert_eq!(touching(&b, 0.0, Some(0.35), None, NOW).touch, Touch::Stuck);
        assert_eq!(touching(&slow, 0.0, Some(0.35), None, NOW).touch, Touch::Contact);
    }

    #[test]
    fn free_space_is_free() {
        let r = touching(&body(), 0.9995, Some(0.99), None, NOW);
        assert_eq!(r.touch, Touch::Free);
        assert!(r.verdict.admit);
    }

    /// 🔴 No reverse reading is not "probably contact". It is exactly the ask that was wrong twice
    /// in nine, so it refuses.
    #[test]
    fn blocked_with_no_reverse_asked_refuses() {
        let r = touching(&body(), 0.0, None, Some(0.99), NOW);
        assert_eq!(r.touch, Touch::Unknown);
        assert_eq!(r.verdict.why, Reason::NoEvidence);
        assert!(!r.verdict.admit);
    }

    #[test]
    fn an_unmeasured_body_refuses_instead_of_guessing() {
        let r = touching(&Body::new(), 0.0, Some(0.99), None, NOW);
        assert_eq!(r.touch, Touch::Unknown);
        assert_eq!(r.verdict.why, Reason::NeverMeasured);
        assert_eq!(r.verdict.culprit, Some(Quantity::ContactThreshold));
    }

    /// The second ruler is required too, and the refusal must name IT rather than the first.
    #[test]
    fn without_step_delivery_the_bar_cannot_be_built() {
        let mut b = Body::new();
        b.submit(q(Quantity::ContactThreshold, T, AxisKind::Unmeasured, 0)).unwrap();
        let r = touching(&b, 0.0, Some(0.99), None, NOW);
        assert_eq!(r.verdict.why, Reason::NeverMeasured);
        assert_eq!(r.verdict.culprit, Some(Quantity::StepDelivery));
    }

    #[test]
    fn a_stale_ruler_refuses() {
        let mut b = Body::new();
        b.submit(q(Quantity::ContactThreshold, T, AxisKind::Unmeasured, 10)).unwrap();
        b.submit(q(Quantity::StepDelivery, SD, AxisKind::Interval, 0)).unwrap();
        assert_eq!(touching(&b, 0.0, Some(0.99), None, NOW).verdict.why, Reason::Stale);
    }

    #[test]
    fn nonsense_inputs_refuse_rather_than_compare() {
        let b = body();
        assert_eq!(touching(&b, f64::NAN, Some(0.99), None, NOW).touch, Touch::Unknown);
        assert_eq!(touching(&b, -1.0, Some(0.99), None, NOW).touch, Touch::Unknown);
        assert_eq!(touching(&b, 0.0, Some(f64::NAN), None, NOW).touch, Touch::Unknown);
    }

    #[test]
    fn every_touch_has_a_name_and_round_trips() {
        for t in [Touch::Unknown, Touch::Free, Touch::Contact, Touch::Stuck] {
            assert!(!t.as_str().is_empty());
            assert_eq!(Touch::from_u32(t as u32), Some(t));
        }
        assert_eq!(Touch::from_u32(4), None);
    }
}
