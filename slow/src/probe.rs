//! The measuring half. **This is where the name comes from.**
//!
//! Up to here the layer could hold a measurement, judge its provenance, and refuse on it. That is
//! a filing cabinet with good manners. What makes it a *body layer* is that the robot determines
//! these numbers **by acting on itself**, so a new machine costs 0 demonstrations, 0 collected
//! episodes and **0 hand-filled numbers**.
//!
//! # What is in here and what is deliberately not
//!
//! Each probe below is a *schedule* plus an *estimator*: what to command, and how to turn what
//! came back into a [`Measurement`] with honest uncertainty. What is **not** here is any per-robot
//! constant. A probe that needed one would have failed at the first line of its own contract.
//!
//! # The rule every probe obeys, and why it is not obvious
//!
//! **A probe may return `None`.** Abstaining is a legitimate outcome and is counted separately
//! from a bad measurement everywhere downstream. The temptation is always to return the best
//! estimate available — and the archive is unambiguous about where that leads: an estimator that
//! settled on the robot's **elbow**, 167 px from the true fingertip, reporting 0.04–9.3 px of
//! error. It did not fail. It answered confidently, and every downstream number was built on it.
//!
//! # And the rule about probing more
//!
//! "Give the fit more evidence" is a whole family of fixes, and it is **already refuted here**:
//! repainting the robot took usable candidate pixels from 11 to 173 — a 15× increase in exactly
//! the quantity the family says is the bottleneck — and the closed loop stayed **0/9**. So none of
//! these probes tries to win by collecting more; they win by *knowing when they cannot answer*.

use crate::hand::{Candidate, Config, HandTracker};
use crate::measurement::{Measurement, Quantity, MAX_DEPS, MAX_DIM};

/// One commanded step and what the image did about it.
#[derive(Copy, Clone, Debug)]
pub struct Sample {
    /// What was commanded, per joint, in the joint's own units.
    pub cmd: [f64; MAX_DIM],
    /// How many joints were commanded.
    pub n: usize,
    /// Where the tracked point ended up, normalised image coordinates.
    pub uv: [f64; 2],
    /// Monotonic time of the observation.
    pub at_ns: u64,
}

/// Why a probe declined to produce a measurement. Each is a distinct fact and they are never
/// merged: "I have no evidence", "my evidence disagrees with itself" and "the answer is outside
/// what I probed" call for three different actions.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Declined {
    /// Fewer samples than the estimator needs to say anything.
    NotEnoughSamples,
    /// The commanded motion did not move the image enough to be distinguishable from noise.
    NoResponse,
    /// The samples imply mutually inconsistent answers.
    Inconsistent,
    /// A quantity this probe must be measured against has not been measured.
    MissingDependency,
}

/// Measure the **image Jacobian**: "I command δ, the image moves by this."
///
/// This is the single measurement that lets a world-frame intent become joint motion **without a
/// calibrated camera** — no intrinsics, no extrinsics, no hand-eye transform. Those matrices are
/// exactly the numbers a person would otherwise type in, so not needing them is what keeps the
/// zero-hand-filled-numbers line intact.
///
/// The uncertainty reported is the residual spread of the fit, not a constant. A Jacobian that
/// fitted badly must *say so*, because the admit gate refuses on precision and cannot do that
/// against a number somebody chose.
///
/// # The probe amplitude is a pixel-scale contract, not a free knob
///
/// Recorded on one rig: per-axis column norms of 792 / 842 / 904 px per metre, so a 0.06 m probe
/// travels 35–54 px while the tracker's search window was 30 px — **the whole excursion landed
/// outside the window and was discarded**. Amplitude, window and expected travel are one contract;
/// changing any of them alone silently produces a Jacobian fitted to nothing.
pub fn image_jacobian(
    samples: &[Sample],
    n_joints: usize,
    now_ns: u64,
    min_response_px: f64,
) -> Result<Measurement, Declined> {
    if n_joints == 0 || n_joints * 2 > MAX_DIM {
        return Err(Declined::MissingDependency);
    }
    // Two unknowns per joint (du, dv); demand more equations than unknowns, or the "fit" is just
    // an interpolation and its residual is meaningless.
    if samples.len() < n_joints + 2 {
        return Err(Declined::NotEnoughSamples);
    }

    let mut m = blank(Quantity::ImageJacobian, 2 * n_joints, now_ns);
    let mut worst_resid = 0.0_f64;
    let mut any_response = false;

    for j in 0..n_joints {
        // Per-joint least squares of (Δu, Δv) against the commanded δ_j across consecutive pairs.
        let (mut sdd, mut sdu, mut sdv) = (0.0, 0.0, 0.0);
        for w in samples.windows(2) {
            let d = w[1].cmd[j] - w[0].cmd[j];
            sdd += d * d;
            sdu += d * (w[1].uv[0] - w[0].uv[0]);
            sdv += d * (w[1].uv[1] - w[0].uv[1]);
        }
        if sdd <= 0.0 {
            // This joint was never commanded. Reporting a zero column would be a lie that reads
            // as "moving this joint does nothing", which is a very different claim.
            return Err(Declined::NoResponse);
        }
        let (gu, gv) = (sdu / sdd, sdv / sdd);
        if gu.hypot(gv) >= min_response_px {
            any_response = true;
        }

        // Residual spread becomes this column's uncertainty. Honest, and it is what the admit gate
        // will refuse on later.
        let (mut ss, mut cnt) = (0.0, 0.0);
        for w in samples.windows(2) {
            let d = w[1].cmd[j] - w[0].cmd[j];
            let ru = (w[1].uv[0] - w[0].uv[0]) - gu * d;
            let rv = (w[1].uv[1] - w[0].uv[1]) - gv * d;
            ss += ru * ru + rv * rv;
            cnt += 1.0;
        }
        let sigma = if cnt > 1.0 { (ss / (cnt - 1.0)).sqrt() } else { f64::INFINITY };
        if !sigma.is_finite() {
            return Err(Declined::Inconsistent);
        }
        worst_resid = worst_resid.max(sigma);

        m.value[2 * j] = gu;
        m.value[2 * j + 1] = gv;
        m.uncertainty[2 * j] = sigma;
        m.uncertainty[2 * j + 1] = sigma;
        // Probed range = the commands actually issued. Not a range anyone hopes it extrapolates to.
        let (lo, hi) = cmd_range(samples, j);
        m.valid_lo[2 * j] = lo;
        m.valid_hi[2 * j] = hi;
        m.valid_lo[2 * j + 1] = lo;
        m.valid_hi[2 * j + 1] = hi;
    }

    if !any_response {
        // Everything moved less than the floor. A Jacobian fitted to sub-noise motion is a number
        // that will be trusted and is not information.
        return Err(Declined::NoResponse);
    }

    // A Jacobian is a claim about the CURRENT camera and mounting. Bounded lifetime on purpose:
    // one knock and it is wrong, and a quantity that never expires is one nobody re-measures.
    m.valid_for_ns = 60_000_000_000; // 60 s
    m.selftest_passed = worst_resid.is_finite();
    Ok(m)
}

/// Measure **which pixels are my hand**, continuously.
///
/// Delegates to [`HandTracker`], which re-measures every control step and abstains rather than
/// guessing. That design is not a preference; it is what the record demands. Recognising the hand
/// is *done* — 1.7 cm → 0.62 cm across three independent processes — and it bought **nothing**:
/// the latch did not move over 32 paired layouts. The located reason is that fit-time error is
/// 2.0 px while the error *at the moment the hand is closest to the target* is 4.9–14.6 px, at or
/// above the 2.0 cm latch radius. **The version that fits best is the one that drifts worst.**
///
/// So this returns the *current* estimate with a σ that has been growing since the last accepted
/// observation, and `None` once that σ exceeds what the caller can use.
pub fn hand_pixel(
    tracker: &mut HandTracker,
    cands: &[Candidate],
    now_ns: u64,
    epoch: u64,
    prev_epoch: u64,
    jac_epoch: u64,
) -> Result<Measurement, Declined> {
    tracker.observe(cands).map_err(|_| Declined::NoResponse)?;
    tracker
        .publish(now_ns, epoch, prev_epoch, jac_epoch)
        .ok_or(Declined::Inconsistent)
}

/// Measure **what holding still against gravity costs**, from poses the arm visits anyway.
///
/// The estimate is a per-joint mean of the torque needed to hold, and the uncertainty is the
/// spread across the poses actually visited. Compensating it is worth a real number: on this rig,
/// 55–95 N of apparent load became **1.89 N**.
///
/// 🔴 The validity range is the set of poses actually sampled, and that is the load-bearing part.
/// A gravity self-calibration on this project turned out to have its **entire** residual error in
/// *interpolation between the sampled poses* — so asking outside them is precisely where its
/// number stops meaning anything, and the honest answer there is a refusal, not an extrapolation.
pub fn arm_weight(
    holds: &[(f64, f64)], // (joint angle, torque required to hold)
    now_ns: u64,
) -> Result<Measurement, Declined> {
    if holds.len() < 3 {
        return Err(Declined::NotEnoughSamples);
    }
    let n = holds.len() as f64;
    let mean = holds.iter().map(|(_, t)| *t).sum::<f64>() / n;
    let var = holds.iter().map(|(_, t)| (t - mean).powi(2)).sum::<f64>() / (n - 1.0);
    if !mean.is_finite() || !var.is_finite() {
        return Err(Declined::Inconsistent);
    }

    let mut m = blank(Quantity::ArmWeight, 1, now_ns);
    m.value[0] = mean;
    m.uncertainty[0] = var.sqrt();
    let lo = holds.iter().map(|(a, _)| *a).fold(f64::INFINITY, f64::min);
    let hi = holds.iter().map(|(a, _)| *a).fold(f64::NEG_INFINITY, f64::max);
    if !(lo < hi) {
        // Every sample at one pose. A "range" of a single point would let the gate admit asks it
        // has no basis for.
        return Err(Declined::Inconsistent);
    }
    m.valid_lo[0] = lo;
    m.valid_hi[0] = hi;
    // No wall-clock expiry: this changes when the BODY changes, not with time. It is invalidated
    // through dependency epochs instead — pick up a payload and it is wrong immediately, while a
    // timer would still call it fresh.
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

/// Measure **command issued → pixels move**, in control periods.
///
/// Named in the archive as the top unmeasured self-calibration and as the whole bottleneck of
/// dynamic work: public systems fail at it by acting on a stale pose, and **nobody measures it
/// per body**. It is also nearly free — every commanded step is already a probe.
pub fn latency(
    first_motion_step: Option<u32>,
    steps_observed: u32,
    now_ns: u64,
) -> Result<Measurement, Declined> {
    if steps_observed < 3 {
        return Err(Declined::NotEnoughSamples);
    }
    let Some(k) = first_motion_step else {
        // Commanded, and nothing ever moved. That is not "latency is large" -- it is a different
        // fault, and reporting a big number here would hide it.
        return Err(Declined::NoResponse);
    };
    let mut m = blank(Quantity::Latency, 1, now_ns);
    m.value[0] = f64::from(k);
    // Quantised to whole periods, so ±half a period is the honest resolution.
    m.uncertainty[0] = 0.5;
    m.valid_lo[0] = 0.0;
    m.valid_hi[0] = f64::from(steps_observed);
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

/// Measure **step delivery**: I command a step of this size; this fraction of it arrives in one
/// control period.
///
/// # Why this probe exists, in one pair of numbers
///
/// Two arms, same harness, same waypoint controller, same 45 mm commanded step. One delivered
/// **0.76** of it per period; the other **0.11**. The per-waypoint step budget had been set from
/// the first arm, so the second could never reach a waypoint at all — 0.136 m of residual on every
/// episode — and the failure surfaced as *"the arm stopped short of the pre-grasp waypoint"*, which
/// reads like a planning fault, a reachability fault, or a broken wrist convention. It was none of
/// those. Every scalar in the log was ordinary.
///
/// The lesson is the one behind this whole layer: **the budget was a hand-filled number carried
/// over from a different body.** Measured instead, the second arm's residual fell to 0.0058 m —
/// the first arm's own figure — with nothing about the robot changed.
///
/// # Why it is its own quantity and not one of the neighbours
///
/// * Not [`Quantity::Latency`]: that is dead time, "how many periods before anything moves".
///   Both arms above answered **1**. A body can start moving at once and still deliver a tenth.
/// * Not [`Quantity::Backlash`]: that is a dead band around a *reversal*. This shortfall applies
///   to every step, including a long run in one direction.
///
/// # The validity range is the commanded step size, and it is load-bearing
///
/// Delivery is not one number for a body — a saturating actuator delivers a different fraction of
/// a 1 mm step than of a 45 mm one. So `valid_lo/hi` is the span of commanded magnitudes actually
/// probed, and an ask outside it is refused rather than extrapolated. Probing at one magnitude
/// only is an [`Declined::Inconsistent`], for the same reason `arm_weight` refuses a single pose.
///
/// # The median, not the mean
///
/// One step that ends in contact delivers almost nothing and would drag a mean down; the estimate
/// would then quietly describe a body that had bumped into the table. The median survives a
/// minority of contact steps, and the spread is reported so a probe run that *was* mostly contact
/// fails its own precision check downstream instead of being believed.
pub fn step_delivery(
    steps: &[(f64, f64)], // (commanded magnitude, achieved magnitude), same units
    now_ns: u64,
) -> Result<Measurement, Declined> {
    if steps.len() < 5 {
        return Err(Declined::NotEnoughSamples);
    }
    // A step nobody commanded carries no information about delivery; including it would push the
    // ratio wherever the noise floor happens to sit.
    let mut ratio = [0.0f64; 256];
    let mut cmd_lo = f64::INFINITY;
    let mut cmd_hi = f64::NEG_INFINITY;
    let mut k = 0usize;
    for &(c, a) in steps {
        if !c.is_finite() || !a.is_finite() || c <= 0.0 {
            continue;
        }
        if k == ratio.len() {
            break;
        }
        ratio[k] = a / c;
        k += 1;
        cmd_lo = cmd_lo.min(c);
        cmd_hi = cmd_hi.max(c);
    }
    if k < 5 {
        return Err(Declined::NotEnoughSamples);
    }
    let r = &mut ratio[..k];
    r.sort_by(|x, y| x.partial_cmp(y).unwrap());
    let med = if k % 2 == 1 { r[k / 2] } else { 0.5 * (r[k / 2 - 1] + r[k / 2]) };
    if med <= 0.0 {
        // Commanded repeatedly and the body did not move. That is a different fault from "delivery
        // is small", and reporting a near-zero ratio here would let a dead joint pass as a slow one.
        return Err(Declined::NoResponse);
    }
    // Robust spread: median absolute deviation, scaled to the normal-consistent estimator so the
    // number the admit gate compares against a precision ask means the same thing as a std-dev.
    let mut dev = [0.0f64; 256];
    for i in 0..k {
        dev[i] = (r[i] - med).abs();
    }
    let d = &mut dev[..k];
    d.sort_by(|x, y| x.partial_cmp(y).unwrap());
    let mad = if k % 2 == 1 { d[k / 2] } else { 0.5 * (d[k / 2 - 1] + d[k / 2]) };

    if !(cmd_lo < cmd_hi) {
        // Every step at one magnitude: no basis for admitting an ask at any other magnitude, and
        // delivery genuinely varies with it.
        return Err(Declined::Inconsistent);
    }
    let mut m = blank(Quantity::StepDelivery, 1, now_ns);
    m.value[0] = med;
    m.uncertainty[0] = 1.4826 * mad;
    m.valid_lo[0] = cmd_lo;
    m.valid_hi[0] = cmd_hi;
    // No wall-clock expiry: this is a property of the body, invalidated when the body changes.
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

// ------------------------------------------------------------------ helpers

fn blank(q: Quantity, dim: usize, now_ns: u64) -> Measurement {
    Measurement {
        quantity: q,
        dim,
        value: [0.0; MAX_DIM],
        uncertainty: [0.0; MAX_DIM],
        valid_lo: [0.0; MAX_DIM],
        valid_hi: [0.0; MAX_DIM],
        measured_at_ns: now_ns,
        valid_for_ns: 0,
        deps: [None; MAX_DEPS],
        epoch: 0,
        selftest_passed: false,
        prev_epoch: 0,
    }
}

fn cmd_range(samples: &[Sample], j: usize) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for s in samples {
        lo = lo.min(s.cmd[j]);
        hi = hi.max(s.cmd[j]);
    }
    if lo >= hi {
        (lo - 1e-6, hi + 1e-6)
    } else {
        (lo, hi)
    }
}

/// Default tracker configuration. Every field is an **observability** threshold — "can this be
/// read off this image at all" — not a physical constant, so it is identical on every robot.
pub fn default_hand_config() -> Config {
    Config::default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(cmd0: f64, u: f64, v: f64, t: u64) -> Sample {
        let mut c = [0.0; MAX_DIM];
        c[0] = cmd0;
        Sample {
            cmd: c,
            n: 1,
            uv: [u, v],
            at_ns: t,
        }
    }

    /// A joint that was never commanded must not get a zero column. "Moving this does nothing" is
    /// a claim, and it is a different claim from "I never tried".
    #[test]
    fn an_uncommanded_joint_is_declined_not_zeroed() {
        let ss = [
            s(0.0, 0.5, 0.5, 0),
            s(0.0, 0.5, 0.5, 1),
            s(0.0, 0.5, 0.5, 2),
        ];
        assert_eq!(
            image_jacobian(&ss, 1, 10, 0.01).unwrap_err(),
            Declined::NoResponse
        );
    }

    /// Commanded, and nothing moved beyond the floor: declined, not fitted to noise.
    #[test]
    fn sub_noise_motion_is_declined() {
        let ss = [
            s(0.00, 0.5000, 0.5, 0),
            s(0.01, 0.5000, 0.5, 1),
            s(0.02, 0.5001, 0.5, 2),
            s(0.03, 0.5001, 0.5, 3),
        ];
        assert_eq!(
            image_jacobian(&ss, 1, 10, 0.5).unwrap_err(),
            Declined::NoResponse
        );
    }

    /// A clean response is measured, and the reported uncertainty is the fit's own residual —
    /// not a number anybody chose.
    #[test]
    fn a_clean_response_is_measured_with_its_own_residual() {
        let ss = [
            s(0.00, 0.10, 0.5, 0),
            s(0.01, 0.20, 0.5, 1),
            s(0.02, 0.30, 0.5, 2),
            s(0.03, 0.40, 0.5, 3),
        ];
        let m = image_jacobian(&ss, 1, 10, 0.5).expect("should measure");
        assert!((m.value[0] - 10.0).abs() < 1e-9, "du/dcmd = {}", m.value[0]);
        assert!(m.uncertainty[0] < 1e-9, "a perfect line must report ~0 residual");
        // and the probed range is the commands actually issued
        assert!((m.valid_lo[0] - 0.0).abs() < 1e-12 && (m.valid_hi[0] - 0.03).abs() < 1e-12);
    }

    /// 🔴 The validity range must be the poses actually visited. A gravity calibration whose entire
    /// residual was interpolation error between sampled poses is the reason this is not optional.
    #[test]
    fn arm_weight_range_is_the_poses_actually_visited() {
        let holds = [(0.1, 1.90), (0.5, 1.88), (0.9, 1.89)];
        let m = arm_weight(&holds, 10).expect("should measure");
        assert!((m.value[0] - 1.89).abs() < 0.01);
        assert_eq!((m.valid_lo[0], m.valid_hi[0]), (0.1, 0.9));
        assert!(m.uncertainty[0] > 0.0, "spread across poses must be reported");
    }

    /// All samples at one pose: there is no range, so there is nothing to be valid over.
    #[test]
    fn arm_weight_at_a_single_pose_is_declined() {
        let holds = [(0.4, 1.9), (0.4, 1.9), (0.4, 1.9)];
        assert_eq!(arm_weight(&holds, 10).unwrap_err(), Declined::Inconsistent);
    }

    /// Commanded and never moved is not "large latency" — it is a different fault.
    #[test]
    fn latency_with_no_motion_is_declined() {
        assert_eq!(latency(None, 20, 10).unwrap_err(), Declined::NoResponse);
    }

    /// And a real latency is reported with half-a-period resolution, because it is quantised.
    #[test]
    fn latency_reports_its_quantisation() {
        let m = latency(Some(1), 20, 10).expect("should measure");
        assert_eq!(m.value[0], 1.0);
        assert_eq!(m.uncertainty[0], 0.5);
    }
}
