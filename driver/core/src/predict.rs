//! The prediction interface: **will that still be the point when I get there?**
//!
//! # The measurement that forced this module to exist
//!
//! Conveyor, four episodes, every stage healthy: the image error converged to **7.2–7.9 px**,
//! contact landed within **9–28 mm** of the object's own height, the descent drifted sideways by
//! **1.8–6.9 mm**, the tool offset audited to within **0.9–2.8 cm**. And the hand finished
//! **17–30 cm** from the object with `obj_dz = 0.000`.
//!
//! That distance is not error. It is 44 control periods of close-and-lift times 4 mm of belt travel
//! per period. **The hand arrived exactly where the object had been when it was last aimed.** Every
//! individual reading was correct and the grasp was lost before the descent began.
//!
//! A layer that can only answer *"can I reach that point"* cannot see this. The missing question is
//! the one this module asks.
//!
//! # 🔴 THE BODY LAYER DOES NOT PREDICT. It supplies the HORIZON and the GATE.
//!
//! *世界靠学,身体靠量.* Predicting where a cup will be is a statement about the world, and the
//! world is learned. What is **measured** here is how long this body will be blind while it acts —
//! [`horizon`], straight out of [`crate::derive::blind_periods`] — and whether a prediction may be
//! acted on — [`admit`].
//!
//! Getting this boundary wrong in the obvious direction (a `bl_predict()` that returns where the
//! cup will be) would put a learned quantity inside the measured layer, where nothing can refuse it
//! and no probe can check it. The split is the same one the whole crate is built on.
//!
//! # 🔴 A PREDICTION CANNOT CARRY A POSE, AND THAT IS STRUCTURAL
//!
//! [`Predicted`] holds a normalised pixel, an extent and a time — the same vocabulary as
//! `bl_world_ref`, deliberately. This is the single most natural place in the entire design for a
//! 3-D pose to enter ("just tell me where it will BE"), and the header's central invariant is that
//! the policy's channel *cannot* express one. A prediction that could return `z` would be a leak
//! with a respectable name.
//!
//! # What this does NOT solve, stated because the gap is already recorded
//!
//! `universal-grounding/README` on dodging a punch: human reaction is **231 ms** and a punch flies
//! for **115–190 ms**, so *"人也躲不掉纯反应的拳,人躲的是起手征兆"* — and then the sharper half:
//! our interface is *"标一个点走过去"*, **and dodging has no point to go to**. This module makes a
//! moving target reachable. It does **not** invent a way to express "get out of where this is
//! going". That needs a verb this vocabulary does not have, and inventing one here without a
//! measurement behind it is exactly the move this crate refuses.

use crate::derive;
use crate::measurement::Quantity;
use crate::refuse::{Reason, Verdict};
use crate::Body;

/// Where a learned model says a reference will be, and how far it can be trusted.
///
/// Every field is the world model's to fill. The body layer only judges it.
#[derive(Copy, Clone, Debug)]
pub struct Predicted {
    /// Normalised image coordinates at `at_period`. No `z`, no pose — see the module docs.
    pub u: f64,
    pub v: f64,
    /// Normalised region size, as in `bl_world_ref`.
    pub extent: f64,
    /// How many control periods from now this describes. `0` means "right now", which is what a
    /// loop with no prediction is implicitly using.
    pub at_period: u32,
    /// 1σ of `(u, v)` at that horizon, in the same normalised units.
    ///
    /// 🔴 Required, and there is no default. A prediction that cannot say how well it knows itself
    /// is the bare `f64` this whole crate exists to abolish, wearing a future tense.
    pub sigma_uv: f64,
    /// 🔴 The largest horizon this model was ACTUALLY VALIDATED over, in periods.
    ///
    /// The same rule as `valid_lo/hi` on a measurement: a model checked to half a second says
    /// nothing about three seconds, and the honest response to being asked beyond it is a refusal
    /// rather than a confident extrapolation. `0` means never validated — admitted **unverified**,
    /// not refused, because refusing it outright collapses the trust scale (see
    /// `measurement::AxisKind::Unmeasured`).
    pub verified_periods: u32,
}

impl Predicted {
    /// What a loop with no prediction is implicitly asserting: *it will still be there.*
    ///
    /// Spelling it out as a constructor is the point — that assertion is normally invisible,
    /// because not predicting looks like not doing anything.
    pub fn none_at(u: f64, v: f64, extent: f64) -> Self {
        Predicted {
            u,
            v,
            extent,
            at_period: 0,
            sigma_uv: 0.0,
            verified_periods: 0,
        }
    }
}

/// How many control periods this body will be **blind** while it covers `distance_m`.
///
/// Settle plus traverse, both from this body's own measured delivery. This is the horizon the
/// caller must predict over; a prediction for any shorter horizon is aimed at a moment that will
/// have passed.
pub fn horizon(body: &Body, distance_m: f64, tol_frac: f64) -> Result<u32, Verdict> {
    derive::blind_periods(body, distance_m, tol_frac)
}

/// May this prediction be acted on, for a motion that leaves the loop blind for `need_periods`?
///
/// * no prediction at all while the body will be blind → **refuse**. This is the conveyor bug, and
///   the refusal names `Latency` because that is what makes the body blind in the first place.
/// * a prediction that stops short of the horizon → **refuse**: aiming at period 5 of a 44-period
///   blind stretch is the same error, smaller.
/// * asked beyond what the model was validated over → **refuse**, never extrapolate.
/// * never validated at all → **admit, unverified** — the third rung.
/// * `sigma_uv` worse than the caller's tolerance → **refuse**.
pub fn admit(
    predicted: &Predicted,
    need_periods: u32,
    tol_uv: Option<f64>,
) -> Verdict {
    if !(predicted.u.is_finite() && predicted.v.is_finite() && predicted.sigma_uv.is_finite()) {
        return Verdict::refuse(Reason::OutOfRange, Quantity::HandPixel);
    }
    if predicted.sigma_uv < 0.0 {
        return Verdict::refuse(Reason::OutOfRange, Quantity::HandPixel);
    }
    if need_periods > 0 && predicted.at_period < need_periods {
        // 🔴 The measured failure, in one branch. The loop aimed at where the object was — a
        // prediction for period 0 — while the close-and-lift left it blind for 44. Every reading
        // was right and the hand still arrived 17–30 cm away.
        return Verdict::refuse(Reason::NotYet, Quantity::Latency);
    }
    if predicted.verified_periods == 0 {
        // Never validated. Not a refusal: hard-refusing an unverified thing was measured to
        // cascade (see `measurement::AxisKind`). Admitted, and the caller is told.
        return Verdict::unverified(Quantity::ImageJacobian);
    }
    if predicted.at_period > predicted.verified_periods {
        return Verdict::refuse(Reason::OutOfRange, Quantity::ImageJacobian);
    }
    if let Some(tol) = tol_uv {
        if predicted.sigma_uv > tol {
            return Verdict::refuse(Reason::UncertaintyTooHigh, Quantity::ImageJacobian);
        }
    }
    Verdict::OK
}

/// The whole question in one call: *may I chase this thing across `distance_m`?*
///
/// Combines the body's own blind horizon with the gate, so a caller cannot accidentally ask the
/// second without the first — which is precisely how the conveyor loop came to aim at a stale
/// point while looking entirely healthy.
pub fn admit_chase(
    body: &Body,
    predicted: &Predicted,
    distance_m: f64,
    tol_frac: f64,
    tol_uv: Option<f64>,
) -> Result<Verdict, Verdict> {
    let n = horizon(body, distance_m, tol_frac)?;
    Ok(admit(predicted, n, tol_uv))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measurement::{AxisKind, Measurement, MAX_DEPS, MAX_DIM};

    fn m(q: Quantity, v: f64, lo: f64, hi: f64) -> Measurement {
        let mut x = Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity: q,
            dim: 1,
            value: [0.0; MAX_DIM],
            uncertainty: [0.0; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [0.0; MAX_DIM],
            measured_at_ns: 0,
            valid_for_ns: 0,
            deps: [None; MAX_DEPS],
            epoch: 0,
            selftest_passed: true,
            prev_epoch: 0,
        };
        x.value[0] = v;
        x.valid_lo[0] = lo;
        x.valid_hi[0] = hi;
        x
    }

    /// A body that delivers most of its commanded step, with one period of dead time.
    fn quick_body() -> Body {
        let mut b = Body::new();
        b.submit(m(Quantity::Latency, 1.0, 0.0, 12.0)).unwrap();
        b.submit(m(Quantity::StepDelivery, 0.9, 0.005, 0.05)).unwrap();
        b
    }

    /// 🔴 THE CONVEYOR, AS A TEST. Not predicting is an assertion, and it is refused.
    #[test]
    fn aiming_at_where_it_is_now_is_refused_when_the_motion_is_long() {
        let now = Predicted::none_at(0.5, 0.5, 0.1);
        let v = admit(&now, 44, None);
        assert!(!v.admit, "a prediction for period 0 cannot serve a 44-period blind stretch");
        assert_eq!(v.why, Reason::NotYet);
        assert_eq!(v.culprit, Some(Quantity::Latency), "blindness is what makes this fail");
    }

    /// The same reference, predicted far enough ahead and validated that far, is admitted.
    #[test]
    fn a_prediction_that_covers_the_blind_stretch_is_admitted() {
        let p = Predicted {
            u: 0.62,
            v: 0.5,
            extent: 0.1,
            at_period: 44,
            sigma_uv: 0.01,
            verified_periods: 60,
        };
        assert!(admit(&p, 44, Some(0.02)).admit);
        // ... and refused when the caller needs better precision than the model has.
        let v = admit(&p, 44, Some(0.005));
        assert!(!v.admit);
        assert_eq!(v.why, Reason::UncertaintyTooHigh);
    }

    /// Beyond what the model was validated over is a refusal, never an extrapolation. Same rule as
    /// a measurement's probed domain, for the same reason.
    #[test]
    fn predicting_past_what_was_validated_refuses() {
        let p = Predicted {
            u: 0.7,
            v: 0.5,
            extent: 0.1,
            at_period: 120,
            sigma_uv: 0.01,
            verified_periods: 60,
        };
        let v = admit(&p, 120, None);
        assert!(!v.admit);
        assert_eq!(v.why, Reason::OutOfRange);
    }

    /// Never validated is the THIRD rung: admitted, and the caller is told nothing checked it.
    #[test]
    fn a_never_validated_model_is_unverified_not_refused() {
        let p = Predicted {
            u: 0.7,
            v: 0.5,
            extent: 0.1,
            at_period: 44,
            sigma_uv: 0.01,
            verified_periods: 0,
        };
        let v = admit(&p, 44, None);
        assert!(v.admit, "hard-refusing this collapses the trust scale");
        assert!(v.unverified);
        assert_eq!(v.why, Reason::NoEvidence);
    }

    /// 🔴 The horizon comes from the BODY, not from the caller's guess — which is the half the
    /// conveyor loop never asked for.
    #[test]
    fn the_horizon_is_measured_not_chosen() {
        let b = quick_body();
        // 0.20 m at 0.045 m of delivered step per period, plus the settle for 1% residual.
        let n = horizon(&b, 0.20, 0.01).unwrap();
        assert!(n >= 5, "a 20 cm move cannot be blind for fewer than a handful of periods: {n}");

        let now = Predicted::none_at(0.5, 0.5, 0.1);
        let v = admit_chase(&b, &now, 0.20, 0.01, None).unwrap();
        assert!(!v.admit, "chasing 20 cm with no prediction must refuse");

        // A body that never measured its delivery cannot state a horizon at all, and says so
        // rather than assuming one.
        let empty = Body::new();
        let e = admit_chase(&empty, &now, 0.20, 0.01, None).unwrap_err();
        assert!(!e.admit);
        assert_eq!(e.why, Reason::NeverMeasured);
    }

    /// A stationary target needs no prediction: `need_periods == 0` admits the present.
    #[test]
    fn a_still_world_needs_no_prediction() {
        let now = Predicted::none_at(0.5, 0.5, 0.1);
        let v = admit(&now, 0, None);
        assert!(v.admit);
        assert!(v.unverified, "still admitted on no evidence, and still said so");
    }

    /// Garbage in is refused, not propagated. A NaN pixel that reaches the servo becomes a motion.
    #[test]
    fn a_non_finite_prediction_is_refused() {
        let mut p = Predicted::none_at(f64::NAN, 0.5, 0.1);
        p.at_period = 44;
        p.verified_periods = 60;
        assert!(!admit(&p, 44, None).admit);

        let mut q = Predicted::none_at(0.5, 0.5, 0.1);
        q.at_period = 44;
        q.verified_periods = 60;
        q.sigma_uv = -1.0;
        assert!(!admit(&q, 44, None).admit, "a negative uncertainty is not a confident model");
    }
}
