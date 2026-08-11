//! Controller constants, handed over instead of typed in.
//!
//! Everything here is a number a robot stack currently hard-codes and this layer can compute from
//! what the body established about itself. The point is not tidiness: each of these is a knob
//! somebody sets once, on one robot, and then carries to the next robot where it is silently
//! wrong.
//!
//! # The rule
//!
//! Every function returns the number **or the refusal**, naming the quantity that blocks it. There
//! are no defaults. A default here would be the same hand-filled constant wearing a function's
//! name — which is precisely how `bl_spec.step_m` sat in the middle of the execution path for
//! months while the ledger counted zero hand-filled constants.
//!
//! # Why `settle` is the one that proves this file earns its keep
//!
//! Two arms, same harness, same commanded 45 mm step: one delivered **0.76** of it per control
//! period, the other **0.11**. The settle budget had been set from the first arm, so the second
//! could never reach a waypoint — **0.136 m of residual on every episode**, surfacing as *"the arm
//! stopped short of the pre-grasp waypoint"*, which reads like a planner, reachability or wrist
//! fault. It was none of those, and every scalar in the log was ordinary. Sizing the budget from
//! the arm's own delivery took the residual to **0.0058 m** with nothing about the robot changed.
//!
//! That is what a typed-in settle costs, and it is why this returns a refusal rather than a
//! sensible-looking default when `step_delivery` has never been measured.

use crate::measurement::Quantity;
use crate::refuse::{Reason, Verdict};
use crate::Body;

/// Control periods to hold a command before the residual is below `tol_frac` of the step.
///
/// `latency` periods pass before anything moves at all; after that each period closes
/// `step_delivery` of what remains, so the residual after `n` moving periods is `(1-f)^n`.
///
/// `tol_frac` is the caller's accuracy requirement, not a body constant — how close is close
/// enough is a property of the task. Everything else comes from the body.
pub fn settle_periods(body: &Body, tol_frac: f64) -> Result<u32, Verdict> {
    if !(tol_frac.is_finite() && tol_frac > 0.0 && tol_frac < 1.0) {
        return Err(Verdict::refuse(Reason::OutOfRange, Quantity::StepDelivery));
    }
    let Some(lat) = body.get(Quantity::Latency) else {
        return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::Latency));
    };
    if !lat.selftest_passed {
        return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::Latency));
    }
    let Some(sd) = body.get(Quantity::StepDelivery) else {
        return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::StepDelivery));
    };
    if !sd.selftest_passed {
        return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::StepDelivery));
    }
    let f = sd.value[0];
    if !(f.is_finite() && f > 0.0) {
        // The body did not move with the command. That is a fault to surface, not a budget to
        // enlarge — an infinite settle would hide a dead joint behind a long wait.
        return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::StepDelivery));
    }
    if f >= 1.0 {
        return Ok(lat.value[0].max(0.0) as u32 + 1);
    }
    let n = (tol_frac.ln() / (1.0 - f).ln()).ceil();
    if !n.is_finite() || n < 0.0 {
        return Err(Verdict::refuse(Reason::UncertaintyTooHigh, Quantity::StepDelivery));
    }
    Ok(lat.value[0].max(0.0) as u32 + n as u32)
}

/// Control steps to traverse `distance_m`, at the largest step this body is known to deliver.
///
/// The step size is `step_delivery`'s own probed ceiling — the same number [`crate::execute::Spec`]
/// scales commands by, so a re-home and a servo step can no longer disagree about how far this arm
/// moves per period.
pub fn traverse_steps(body: &Body, distance_m: f64) -> Result<u32, Verdict> {
    if !(distance_m.is_finite() && distance_m >= 0.0) {
        return Err(Verdict::refuse(Reason::OutOfRange, Quantity::StepDelivery));
    }
    let Some(sd) = body.get(Quantity::StepDelivery) else {
        return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::StepDelivery));
    };
    if !sd.selftest_passed {
        return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::StepDelivery));
    }
    let step = sd.valid_hi[0];
    let f = sd.value[0];
    if !(step.is_finite() && step > 0.0 && f.is_finite() && f > 0.0) {
        return Err(Verdict::refuse(Reason::OutOfRange, Quantity::StepDelivery));
    }
    // Divide by what actually arrives, not by what is commanded. Sizing a traverse by the
    // COMMANDED step is the 0.11-delivery bug in its other form: the count is right, the arm is
    // short, and nothing reports an error.
    Ok((distance_m / (step * f)).ceil() as u32)
}

/// The largest single command this body is known to deliver, in metres.
///
/// Exposed on its own because callers that never touch [`crate::execute::Spec`] still need it, and
/// every one of them currently types it in.
pub fn step_m(body: &Body) -> Result<f64, Verdict> {
    let Some(sd) = body.get(Quantity::StepDelivery) else {
        return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::StepDelivery));
    };
    if !sd.selftest_passed {
        return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::StepDelivery));
    }
    let s = sd.valid_hi[0];
    if !(s.is_finite() && s > 0.0) {
        return Err(Verdict::refuse(Reason::OutOfRange, Quantity::StepDelivery));
    }
    Ok(s)
}

/// Clearance the jaws need above a support surface before closing, in metres.
///
/// 🔴 Refuses on this body today, and the refusal is the useful part: `gripper_span` came back
/// `NoResponse` — the commanded opening did not move the observed signal. An approach height
/// invented while the jaw span is unknown is the constant that quietly decides whether a grasp
/// closes on the object or on the table, and it would be believed because nothing else in the log
/// would look wrong.
pub fn approach_clearance_m(body: &Body) -> Result<f64, Verdict> {
    let Some(g) = body.get(Quantity::GripperSpan) else {
        return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::GripperSpan));
    };
    if !g.selftest_passed {
        return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::GripperSpan));
    }
    let span = g.value[0];
    if !(span.is_finite() && span > 0.0) {
        return Err(Verdict::refuse(Reason::UncertaintyTooHigh, Quantity::GripperSpan));
    }
    // Half the span is the geometric minimum for the jaws to straddle an object of that width;
    // the caller adds whatever margin its own task demands, and that margin is a task property,
    // not a body one.
    Ok(span / 2.0)
}
