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
use crate::measurement::{AxisKind, MAX_DEPS, MAX_DIM, Measurement, Quantity};

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

/// Measure **reach**: the radial band, from an arm's own base, in which that arm can actually
/// attain a commanded pose.
///
/// # Why this is a band and not a radius
///
/// "Reach" is habitually written down as one number — how far the arm extends. A body that is
/// mounted has *two* limits, and the inner one is the one that gets forgotten: an object close to
/// the shoulder is unreachable for a different reason than one that is far, and no single radius
/// separates them.
///
/// The measurement that forced this: sweeping one robot's two bases apart, task success ran
/// 20% → 46% → 61% → 49% as the separation went 0.30 → 0.55 → 0.65 → 0.75 m, with objects fixed.
/// Nothing about the arm changed; only where its base sat relative to the work. A single-radius
/// reach cannot express that shape, and the separation had been chosen by hand — exactly the kind
/// of number this layer exists to stop anyone from typing in.
///
/// # It refuses when the boundary was never probed — and that is the whole point
///
/// The estimate is only meaningful if the samples straddle both edges. In the 0.75 m sweep above,
/// **zero** samples fell inside the inner limit, so that run contains no evidence about where the
/// inner limit is — yet a naive fit would happily report one, and it would be believed, because a
/// plausible number and a measured number are indistinguishable downstream.
///
/// So both edges must be bracketed by an observed failure, or this returns
/// [`Declined::Inconsistent`]. A caller that wants the answer must probe wider; it may not have a
/// guess dressed up as a measurement.
///
/// # The core is a maximum-subarray, not a threshold
///
/// Attainment is not monotone in radius — a single failure mid-band (a collision, an unlucky
/// orientation) should not truncate the estimate. Scoring attained `+1` / failed `-1` and taking
/// the maximum-sum contiguous run tolerates a minority of interior failures while still stopping at
/// a genuine wall. A threshold on a local rate needs a window size, which is another hand-filled
/// number.
///
/// `value` is `[r_inner, r_outer]`; `uncertainty` is how tightly each edge is bracketed — half the
/// gap between the last failure outside and the first attained sample inside. `valid_lo/hi` is the
/// radial span actually sampled, so an ask about a radius nobody probed is refused, not
/// extrapolated.
pub fn reach(
    samples: &[(f64, bool)], // (radius from this arm's own base, did it attain the pose)
    now_ns: u64,
) -> Result<Measurement, Declined> {
    let mut r = [0.0f64; 256];
    let mut ok = [false; 256];
    let mut k = 0usize;
    for &(radius, attained) in samples {
        if !radius.is_finite() || radius < 0.0 {
            continue;
        }
        if k == r.len() {
            break;
        }
        r[k] = radius;
        ok[k] = attained;
        k += 1;
    }
    // Two edges to locate; fewer than this and the "band" is whichever sample happened to land.
    if k < 12 {
        return Err(Declined::NotEnoughSamples);
    }
    // Sort by radius, carrying the outcome. Insertion sort: k <= 256 and no allocator here.
    for i in 1..k {
        let (rv, ov) = (r[i], ok[i]);
        let mut j = i;
        while j > 0 && r[j - 1] > rv {
            r[j] = r[j - 1];
            ok[j] = ok[j - 1];
            j -= 1;
        }
        r[j] = rv;
        ok[j] = ov;
    }
    if !ok[..k].iter().any(|&a| a) {
        // Commanded across the whole span and attained nothing. That is a dead arm, not a narrow
        // band, and reporting an empty band would let it pass as a reachability limit.
        return Err(Declined::NoResponse);
    }
    // Maximum-sum contiguous run, scored against **this body's own attainment rate** rather than
    // +1/-1. With a flat +/-1 score a body that attains most of what it is asked (0.89 on the real
    // logs) makes the core swallow the whole sweep -- the walls are outvoted rather than found, and
    // the probe then refuses a band it did have the evidence for. Centring on `p` makes the
    // expected score of an arbitrary run zero, so the core stops exactly where attainment falls
    // below what this body manages on average. No window, no threshold, nothing to fill in by hand.
    let p = ok[..k].iter().filter(|&&a| a).count() as f64 / k as f64;
    let (mut best, mut best_lo, mut best_hi) = (f64::NEG_INFINITY, 0usize, 0usize);
    let (mut cur, mut cur_lo) = (0.0f64, 0usize);
    for i in 0..k {
        let w = if ok[i] { 1.0 - p } else { -p };
        if cur <= 0.0 {
            cur = w;
            cur_lo = i;
        } else {
            cur += w;
        }
        if cur > best {
            best = cur;
            best_lo = cur_lo;
            best_hi = i;
        }
    }
    // Both edges must be bracketed by an observed failure, or the boundary is a guess.
    let inner_probed = ok[..best_lo].iter().any(|&a| !a);
    let outer_probed = ok[best_hi + 1..k].iter().any(|&a| !a);
    if !inner_probed || !outer_probed {
        return Err(Declined::Inconsistent);
    }
    // A core exists in any sample; that it exists is not evidence of a wall. Attainment outside it
    // must be lower than inside by more than counting noise, or what has been located is a lucky
    // stretch of a flat curve.
    //
    // This gate is not hypothetical. Fed 2174 real episodes whose attainment ran 73–100% with no
    // trend in radius, an earlier version reported crisp bands (ARX `[0.194, 0.294]`, Franka
    // `[0.358, 0.506]`) that were slivers of noise; they were briefly believed and read as "the two
    // bodies reach different places".
    //
    // 🔴 The flat curve was itself an input bug, and that is the sharper lesson: `attained` had been
    // derived from ONE failure label ("could not reach") while a second label ("stopped short of the
    // pre-grasp waypoint") describes the same event — the hand never arrived — and was being counted
    // as a success. Relabelled, the same episodes show a wall: attainment 12% at 0.15 m rising to
    // 94% at 0.40 m, and it survives a within-configuration control (p=0.0004 at two independent
    // base placements). So this gate must hold even when the caller's signal looks flat: a flat
    // curve is evidence of nothing, and can equally mean the caller measured the wrong thing.
    // Each wall is tested on its OWN side. Pooling the two sides lets a well-evidenced outer wall
    // carry an inner wall that rests on two samples — and the reported band would then name an
    // inner radius nothing established. Testing per side needs no minimum-sample constant: a side
    // holding one or two points has a standard error too wide to separate, and is refused by the
    // same arithmetic that admits a side holding fifty.
    let n_in = best_hi - best_lo + 1;
    let hit_in = ok[best_lo..=best_hi].iter().filter(|&&a| a).count();
    let p_in = hit_in as f64 / n_in as f64;
    let separates = |lo: usize, hi: usize| -> bool {
        let n_s = hi - lo;
        if n_s == 0 {
            return false;
        }
        let hit_s = ok[lo..hi].iter().filter(|&&a| a).count();
        let p_s = hit_s as f64 / n_s as f64;
        let p_pool = (hit_in + hit_s) as f64 / (n_in + n_s) as f64;
        let se = (p_pool * (1.0 - p_pool) * (1.0 / n_in as f64 + 1.0 / n_s as f64)).sqrt();
        se > 0.0 && (p_in - p_s) >= 2.0 * se
    };
    if !separates(0, best_lo) || !separates(best_hi + 1, k) {
        return Err(Declined::Inconsistent);
    }

    // Nearest observed failure outside each edge — that pair brackets the wall.
    let lo_fail = (0..best_lo).rev().find(|&i| !ok[i]).unwrap();
    let hi_fail = (best_hi + 1..k).find(|&i| !ok[i]).unwrap();

    let mut m = blank(Quantity::Reach, 2, now_ns);
    m.value[0] = 0.5 * (r[lo_fail] + r[best_lo]);
    m.value[1] = 0.5 * (r[best_hi] + r[hi_fail]);
    m.uncertainty[0] = 0.5 * (r[best_lo] - r[lo_fail]);
    m.uncertainty[1] = 0.5 * (r[hi_fail] - r[best_hi]);
    m.valid_lo[0] = r[0];
    m.valid_hi[0] = r[k - 1];
    m.valid_lo[1] = r[0];
    m.valid_hi[1] = r[k - 1];
    // A property of the body and its mounting, invalidated when either changes — not by the clock.
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

// --------------------------------------------------------------------- gripper span

/// Measure **gripper span**: how far the jaws travel between the openings actually commanded, in
/// metres.
///
/// # The constant this replaces has a name and a receipt
///
/// A hand-filled gripper constant — `0.145` — sat in this project's stack with **no traceable
/// provenance at all**: nobody could say which body it came off. A constant that cannot be traced
/// cannot be re-measured on a new machine, so the body running on it was never zero-shot. It was
/// configured by hand and reported as zero-shot, which is the failure this whole layer exists to
/// make impossible.
///
/// # It needs a ruler, and the ruler is where the remaining debt lives
///
/// Nothing in an image is in metres. This probe reads the jaw separation **in image units** across
/// a commanded sweep of the opening, and divides by `units_per_m` — image units per metre —
/// obtained by commanding a step of known metric size and watching the image. That ruler carries
/// its own 1σ and it is propagated: a span quoted to the millimetre off a ruler good to the
/// centimetre is a fabricated precision, and the admit gate must be able to refuse on it.
///
/// 🔴 The metric reference is the point where a spec-sheet number still enters this layer
/// (`bl_spec.step_m`). It is not hidden — it is carried as a named outstanding item in
/// [`crate::debt`], with the test that would discharge it.
///
/// # The value describes the sweep that was actually run
///
/// `value` is the jaw travel between the **lowest and highest openings actually commanded**, and
/// `valid_lo/hi` is exactly that pair. A caller that swept `[0.4, 0.8]` gets the span of
/// `[0.4, 0.8]`, and an ask at full open is refused by the gate. Multiplying the fitted slope by a
/// full unit of opening would turn a partial sweep into a claim about jaws nobody opened — the same
/// extrapolation `arm_weight` refuses between poses.
///
/// # What it refuses, and why each one is a distinct fact
///
/// * a ruler that is absent, zero or non-finite ⇒ [`Declined::MissingDependency`] — a span in
///   metres without a metric reference is a number in image units wearing a unit label;
/// * a sweep at one opening ⇒ [`Declined::Inconsistent`] — no range to be valid over;
/// * a separation that does not respond to the command ⇒ [`Declined::NoResponse`] — a jammed
///   gripper and jaws the camera cannot resolve both land here, and neither may be reported as a
///   very small gripper;
/// * jaws that *close* as the opening is commanded up ⇒ [`Declined::Inconsistent`] — the sign
///   convention is inverted, or the tracked blobs are not the jaws. Taking `|slope|` would make
///   both of those read as a healthy measurement.
pub fn gripper_span(
    samples: &[(f64, f64)], // (commanded opening in [0,1], observed jaw separation, image units)
    units_per_m: f64,
    units_per_m_sigma: f64,
    now_ns: u64,
    jac_epoch: u64,
) -> Result<Measurement, Declined> {
    if !units_per_m.is_finite() || units_per_m <= 0.0 || !units_per_m_sigma.is_finite() || units_per_m_sigma < 0.0 {
        return Err(Declined::MissingDependency);
    }
    let (mut n, mut sx, mut sy, mut sxx, mut sxy) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let (mut x_lo, mut x_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut y_lo, mut y_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for &(x, y) in samples {
        if !x.is_finite() || !y.is_finite() || !(0.0..=1.0).contains(&x) {
            continue;
        }
        n += 1.0;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        x_lo = x_lo.min(x);
        x_hi = x_hi.max(x);
        y_lo = y_lo.min(y);
        y_hi = y_hi.max(y);
    }
    // Three points fit a line and leave one degree of freedom for a residual; below that the "fit"
    // has no residual and therefore no uncertainty, and a measurement without uncertainty cannot be
    // refused on.
    if n < 5.0 {
        return Err(Declined::NotEnoughSamples);
    }
    let sxx_c = sxx - sx * sx / n;
    if !(x_lo < x_hi) || sxx_c <= 0.0 {
        return Err(Declined::Inconsistent);
    }
    // 🔴 Exact degeneracy first, before any fitting. FOUND BY THE TEST BELOW, and it is not a
    // rounding nicety: on a perfectly stuck gripper every separation reading is the same number, so
    // the true slope and the true residual are BOTH zero — and in floating point they come out as
    // noise near 1e-14 each, in whatever order the summation happened to land. The statistical
    // response test then becomes a coin flip, and on the losing side the **sign** test decides the
    // outcome from the sign of a rounding error: a jammed gripper reported as "the jaws close as you
    // command them open". Comparing the readings themselves has no such failure mode.
    if !(y_lo < y_hi) {
        return Err(Declined::NoResponse);
    }
    let slope = (sxy - sx * sy / n) / sxx_c;
    let intercept = (sy - slope * sx) / n;
    let mut ss = 0.0f64;
    for &(x, y) in samples {
        if !x.is_finite() || !y.is_finite() || !(0.0..=1.0).contains(&x) {
            continue;
        }
        let r = y - (intercept + slope * x);
        ss += r * r;
    }
    let resid = (ss / (n - 2.0)).sqrt();
    let slope_sigma = resid / sxx_c.sqrt();
    if !slope.is_finite() || !slope_sigma.is_finite() {
        return Err(Declined::Inconsistent);
    }
    // A slope that its own fit cannot separate from zero is not a small gripper; it is no evidence.
    if slope.abs() <= 2.0 * slope_sigma {
        return Err(Declined::NoResponse);
    }
    if slope < 0.0 {
        return Err(Declined::Inconsistent);
    }

    let span_units = slope * (x_hi - x_lo);
    let span_m = span_units / units_per_m;
    // Relative errors add in quadrature: the fit's and the ruler's. Quoting only the fit's would
    // make a span measured with a bad ruler look as precise as one measured with a good one.
    let rel = ((slope_sigma / slope).powi(2) + (units_per_m_sigma / units_per_m).powi(2)).sqrt();

    let mut m = blank(Quantity::GripperSpan, 1, now_ns);
    m.value[0] = span_m;
    m.uncertainty[0] = span_m * rel;
    m.valid_lo[0] = x_lo;
    m.valid_hi[0] = x_hi;
    // Measured against the camera, through the ruler: knock the camera and this is wrong even
    // though the jaws never moved. That is the case a wall-clock TTL cannot catch.
    m.deps[0] = Some((Quantity::ImageJacobian, jac_epoch));
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

/// 爪能张多开 —— **不用相机的第二条测法**:拿合爪停在哪儿当尺子。
///
/// # 为什么要有第二条路
///
/// [`gripper_span`] 那条路是对的,但它要**爪尖在画面里的间距**,而那要一个爪尖检测器。
/// 这具身体上那个检测器从没跑过 ⇒ 喂进去的间距一动不动 ⇒ 探针照规矩答 `NoResponse`,
/// 于是 `GripperSpan` 至今是拒绝态,而 ②a 每一次"这段夹不夹得下"都踩在一个**手填的 0.088** 上。
/// 手填的几何判据判错了,**没有任何一个环节会不一致** —— 这正是这一层存在的理由。
///
/// # 这条路用的是身体已经量到的东西
///
/// 驱动已经有 [`Quantity::ContactThreshold`](crate::measurement::Quantity::ContactThreshold):
/// 命令走了多少 vs 实到走了多少。**把它用在爪子自己这条轴上** —— 合爪合到碰上东西就停,
/// 停住时的**爪指令值**就是那个东西的宽度,只是还没换算成米。
/// 拿几个**宽度已知**的东西各夹一次,就能把爪指令值拟成米。
///
/// 不要相机,不要爪尖检测器,真机上照样能做(一块已知厚度的标定块就够)。
///
/// # 它拒绝什么,每一条都是一件**不同**的事实
///
/// * 样本少于 5 个 ⇒ [`Declined::NotEnoughSamples`]:拟不出带残差的线,而没有不确定度的量无法被拒绝;
/// * 每次都停在**同一个爪指令值** ⇒ [`Declined::NoResponse`]:爪子停住的位置由**限位**决定,
///   不是由物体决定 —— 这不是"一个很小的夹爪",是根本没在量物体。
///   🔴 **这是实测发生过的那一档**:一批离线数据里爪值逐位相同,当时差点被读成"量到了";
/// * 物体越宽、爪子反而停得越紧 ⇒ [`Declined::Inconsistent`]:要么符号约定反了,要么停住的
///   根本不是物体。**取 `|slope|` 会把这两种都读成一次健康的测量**;
/// * 已知宽度本身没有不确定度 ⇒ [`Declined::MissingDependency`]:拿一把不知道多准的尺子
///   量出来的毫米,是编造的精度。
///
/// 约定:爪指令 `0` = 完全合拢,`1` = 完全张开。所以**物体越宽,停住时的爪指令越大**。
pub fn gripper_span_by_stall(
    samples: &[(f64, f64)], // (合爪停住时的爪指令 ∈[0,1], 那个东西夹住处的宽度, 米)
    width_sigma_m: f64,     // 已知宽度自己的 1σ,米
    now_ns: u64,
    contact_epoch: u64,
) -> Result<Measurement, Declined> {
    if !width_sigma_m.is_finite() || width_sigma_m < 0.0 {
        return Err(Declined::MissingDependency);
    }
    let (mut n, mut sx, mut sy, mut sxx, mut sxy) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let (mut x_lo, mut x_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for &(x, y) in samples {
        if !x.is_finite() || !y.is_finite() || !(0.0..=1.0).contains(&x) || y < 0.0 {
            continue;
        }
        n += 1.0;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        x_lo = x_lo.min(x);
        x_hi = x_hi.max(x);
    }
    if n < 5.0 {
        return Err(Declined::NotEnoughSamples);
    }
    // 🔴 先比读数本身,再拟合。爪子每次都停在限位上时,真斜率和真残差**都是零**,
    //    浮点里它们各自落在 1e-14 附近、大小取决于求和顺序 ⇒ 统计检验变成掷硬币,
    //    输的那一面会拿一个舍入误差的符号去判"物体越宽爪子停得越紧"。
    if !(x_lo < x_hi) {
        return Err(Declined::NoResponse);
    }
    let sxx_c = sxx - sx * sx / n;
    if sxx_c <= 0.0 {
        return Err(Declined::Inconsistent);
    }
    let slope = (sxy - sx * sy / n) / sxx_c;
    let intercept = (sy - slope * sx) / n;
    if !(slope > 0.0) {
        // 物体越宽,爪子该停在越张开的地方。反过来 = 符号反了,或者停住的根本不是物体。
        return Err(Declined::Inconsistent);
    }
    let mut ss = 0.0f64;
    for &(x, y) in samples {
        if !x.is_finite() || !y.is_finite() || !(0.0..=1.0).contains(&x) || y < 0.0 {
            continue;
        }
        let r = y - (intercept + slope * x);
        ss += r * r;
    }
    let resid = (ss / (n - 2.0)).sqrt();
    let slope_sigma = resid / sxx_c.sqrt();
    if !slope.is_finite() || !slope_sigma.is_finite() {
        return Err(Declined::Inconsistent);
    }
    // 🔴 一条自己的拟合都分不开零的斜率,**不是一具很小的夹爪,是没有证据**。
    //
    // 这一档照抄 [`gripper_span`] 的同名检查 —— **我第一版漏了它**,于是在一批结构上无效的
    // 对子上给出了 **0.0393 ± 0.0440**(2026-08-12 实测,误差比值本身还大)。
    // 一个"看着像测量"的数比一次拒绝危险得多:没有任何下游环节会因为它而不一致。
    if slope <= 2.0 * slope_sigma {
        return Err(Declined::NoResponse);
    }
    // 全张开时的宽度 = 这条线在爪指令 1.0 处的值。
    let span = intercept + slope;
    if !(span > 0.0) {
        return Err(Declined::Inconsistent);
    }
    // 两项不确定度:拟合自己的,和已知宽度那把尺子的。
    let sigma = (slope_sigma * slope_sigma + width_sigma_m * width_sigma_m).sqrt();
    let mut m = blank(Quantity::GripperSpan, 1, now_ns);
    m.value[0] = span;
    m.uncertainty[0] = sigma;
    // 🔴 有效区间就是**真扫过的那一段**爪指令。全张开处的值是外推,
    //    而外推成不成立由 admit 闸判,不由这里替它决定。
    m.valid_lo[0] = intercept + slope * x_lo;
    m.valid_hi[0] = intercept + slope * x_hi;
    m.deps[0] = Some((Quantity::ContactThreshold, contact_epoch));
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

#[cfg(test)]
mod jaw_stall_tests {
    use super::*;

    #[test]
    fn a_jaw_that_always_stops_at_the_same_place_is_a_refusal_not_a_tiny_gripper() {
        // 🔴 实测发生过的那一档:一批离线数据里爪值**逐位相同**。
        //    含义是"爪子停住的位置由限位决定,不是由物体决定" —— 它根本没在量物体。
        //    读成"一个很小的夹爪"就等于凭空造出一个身体常数。
        let same: Vec<(f64, f64)> = [0.02, 0.03, 0.04, 0.05, 0.06]
            .iter()
            .map(|w| (0.1281, *w))
            .collect();
        assert_eq!(
            gripper_span_by_stall(&same, 0.001, 1, 1).unwrap_err(),
            Declined::NoResponse
        );
    }

    #[test]
    fn wider_object_must_stall_the_jaw_further_open() {
        // 反过来 = 符号约定反了,或者停住的根本不是物体。取绝对值会把这两种都读成健康测量。
        let inverted: Vec<(f64, f64)> =
            vec![(0.10, 0.06), (0.20, 0.05), (0.30, 0.04), (0.40, 0.03), (0.50, 0.02)];
        assert_eq!(
            gripper_span_by_stall(&inverted, 0.001, 1, 1).unwrap_err(),
            Declined::Inconsistent
        );
    }

    #[test]
    fn four_samples_cannot_carry_an_uncertainty() {
        let few: Vec<(f64, f64)> = vec![(0.1, 0.01), (0.2, 0.02), (0.3, 0.03), (0.4, 0.04)];
        assert_eq!(
            gripper_span_by_stall(&few, 0.001, 1, 1).unwrap_err(),
            Declined::NotEnoughSamples
        );
    }

    #[test]
    fn a_span_its_own_fit_cannot_separate_from_zero_is_a_refusal() {
        // 🔴 实测那一批(2026-08-12):对子结构上就是无效的(②a 报的是【打算夹的那一段】,
        //    而爪子实际停在别处),拟合出来 0.0393 ± 0.0440 —— 误差比值还大。
        //    第一版探针把它当成了一个测量结果。补上这一档之后它必须变成拒绝。
        let noisy: Vec<(f64, f64)> = vec![
            (0.0885, 0.0001), (0.1057, 0.0084), (0.1281, 0.0090), (0.1395, 0.0126),
            (0.3005, 0.0048), (0.3017, 0.0351), (0.3020, 0.0048), (0.3020, 0.0048),
        ];
        assert_eq!(
            gripper_span_by_stall(&noisy, 0.001, 1, 1).unwrap_err(),
            Declined::NoResponse
        );
    }

    #[test]
    fn a_clean_sweep_recovers_the_span_and_keeps_the_ruler_error() {
        // 造一具张开度 0.088 的爪子:爪指令 x 处夹得住 0.088x 宽的东西。
        let clean: Vec<(f64, f64)> = (1..=8)
            .map(|i| {
                let x = i as f64 / 10.0;
                (x, 0.088 * x)
            })
            .collect();
        let m = gripper_span_by_stall(&clean, 0.002, 1, 7).unwrap();
        assert!((m.value[0] - 0.088).abs() < 1e-6, "span={}", m.value[0]);
        // 已知宽度那把尺子的误差必须留在里面 —— 只报拟合误差 = 编造的精度。
        assert!(m.uncertainty[0] >= 0.002);
        // 有效区间是**真扫过的那一段**,全张开处是外推,由 admit 闸判。
        assert!(m.valid_hi[0] < m.value[0]);
        // 这条测法建在接触判据上:接触判据一重测,这个数就该跟着过期。
        assert_eq!(m.deps[0], Some((Quantity::ContactThreshold, 7)));
    }
}

/// 这具身体的**原位** —— 归位若干次,把落点和它自己的抖动一起记下来。
///
/// # 为什么必须归位**多次**
///
/// 只归一次拿到的是一个位形,**没有容差**;而"回到原位没有"这个判断,容差是它的全部内容。
/// 一次采样的量无法被拒绝,而这一层的意义就是能拒绝。
/// ⇒ `uncertainty` 记的是**重复性**:同一条归位指令走若干遍,落点自己散多大。
/// 🔴 **那个散布就是容差的下界** —— 比它还紧的容差是编出来的,而编出来的容差会让一具
/// 完全正常的身体永远判"没回到位"。
///
/// # 它拒绝什么
///
/// * 少于 3 次归位 ⇒ [`Declined::NotEnoughSamples`]:两次给不出散布;
/// * 任何一次的姿态四元数不是单位长度 ⇒ [`Declined::Inconsistent`]:那不是一个姿态;
/// * 散布大过 `spread_cap_m` ⇒ [`Declined::Inconsistent`] —— **一个回不到同一处的身体
///   没有"原位"可言**,把它平均一下报出来,等于给一个不存在的点。
pub fn home_pose(
    samples: &[[f64; 7]], // 每次归位后的 [x, y, z, qw, qx, qy, qz]
    spread_cap_m: f64,
    now_ns: u64,
) -> Result<Measurement, Declined> {
    if samples.len() < 3 {
        return Err(Declined::NotEnoughSamples);
    }
    for v in samples {
        if v.iter().any(|c| !c.is_finite()) {
            return Err(Declined::Inconsistent);
        }
        let qn = (v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6]).sqrt();
        if (qn - 1.0).abs() > 1e-3 {
            return Err(Declined::Inconsistent);
        }
    }
    let n = samples.len() as f64;
    let mut mean = [0.0f64; 7];
    for v in samples {
        for i in 0..7 {
            mean[i] += v[i] / n;
        }
    }
    // 姿态取平均之后要重新归一,否则它不再是一个姿态。
    let qn = (mean[3] * mean[3] + mean[4] * mean[4] + mean[5] * mean[5] + mean[6] * mean[6]).sqrt();
    if qn <= 1e-9 {
        // 几次归位的姿态互相抵消 ⇒ 它们指向完全不同的方向,不是同一个原位。
        return Err(Declined::Inconsistent);
    }
    for i in 3..7 {
        mean[i] /= qn;
    }
    // 位置的散布:到均值的距离的 1σ。这一个数就是容差的下界。
    let mut ss = 0.0f64;
    let mut worst = 0.0f64;
    for v in samples {
        let d = ((v[0] - mean[0]).powi(2) + (v[1] - mean[1]).powi(2) + (v[2] - mean[2]).powi(2)).sqrt();
        ss += d * d;
        worst = worst.max(d);
    }
    let spread = (ss / n).sqrt();
    if !(spread_cap_m > 0.0) || spread > spread_cap_m {
        return Err(Declined::Inconsistent);
    }
    let mut m = blank(Quantity::HomePose, 7, now_ns);
    for i in 0..7 {
        m.value[i] = mean[i];
        // 位置三轴带散布;姿态那四位的不确定度这里不给 —— 没量,就不许写一个数进去。
        m.uncertainty[i] = if i < 3 { spread } else { 0.0 };
        m.axis_kind[i] = if i < 3 { AxisKind::Interval } else { AxisKind::Unmeasured };
    }
    // 有效区间 = 真的散到过的范围。判"回没回到位"只能用这个,不能用一个更紧的数。
    m.valid_lo[0] = 0.0;
    m.valid_hi[0] = worst;
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

/// **回到原位了没有** —— 拿量出来的散布当容差,不是拿一个拍出来的数。
///
/// 🔴 `tol_k` 是"几倍散布算到位",默认调用方给 3。**容差的下界是散布本身**:
/// 传一个比 1 还小的 `tol_k`,等于要求身体比它自己的重复性还准,那永远判不过。
pub fn at_home(now: &[f64; 3], home: &Measurement, tol_k: f64) -> Option<bool> {
    if home.quantity as u32 != Quantity::HomePose as u32 || home.dim < 3 {
        return None;
    }
    let spread = home.uncertainty[0];
    if !(spread > 0.0) || !(tol_k >= 1.0) {
        return None;
    }
    let d = ((now[0] - home.value[0]).powi(2)
        + (now[1] - home.value[1]).powi(2)
        + (now[2] - home.value[2]).powi(2))
    .sqrt();
    Some(d <= tol_k * spread)
}

#[cfg(test)]
mod home_tests {
    use super::*;

    fn h(x: f64, y: f64, z: f64) -> [f64; 7] {
        [x, y, z, 1.0, 0.0, 0.0, 0.0]
    }

    #[test]
    fn one_homing_gives_a_pose_but_no_tolerance_so_it_is_refused() {
        // 只归一次(或两次)拿到的是一个位形,**没有容差**;
        // 而"回到原位没有"这个判断,容差就是它的全部内容。
        assert_eq!(
            home_pose(&[h(0.3, 0.0, 0.5), h(0.3, 0.0, 0.5)], 0.01, 1).unwrap_err(),
            Declined::NotEnoughSamples
        );
    }

    #[test]
    fn a_body_that_cannot_return_to_the_same_place_has_no_home() {
        // 三次归位落在相距十几厘米的地方 ⇒ 平均一下报出来,等于给一个**不存在的点**。
        let far = [h(0.30, 0.0, 0.5), h(0.45, 0.0, 0.5), h(0.15, 0.0, 0.5)];
        assert_eq!(home_pose(&far, 0.01, 1).unwrap_err(), Declined::Inconsistent);
    }

    #[test]
    fn a_quaternion_that_is_not_a_rotation_is_refused() {
        let bad = [
            [0.3, 0.0, 0.5, 2.0, 0.0, 0.0, 0.0],
            [0.3, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
            [0.3, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
        ];
        assert_eq!(home_pose(&bad, 0.01, 1).unwrap_err(), Declined::Inconsistent);
    }

    #[test]
    fn the_tolerance_is_the_measured_repeatability_not_a_number_i_picked() {
        // 三次归位散布约 1 mm。
        let ok = [h(0.300, 0.0, 0.5), h(0.301, 0.0, 0.5), h(0.299, 0.0, 0.5)];
        let m = home_pose(&ok, 0.01, 1).expect("这具身体回得去同一处");
        assert!((m.value[0] - 0.300).abs() < 1e-6);
        let spread = m.uncertainty[0];
        assert!(spread > 0.0 && spread < 0.002, "散布该是毫米量级,得到 {spread}");
        // 姿态那四位没量不确定度 ⇒ 必须标成"没量",不许填 0 冒充"很准"。
        assert_eq!(m.axis_kind[3], AxisKind::Unmeasured);

        // 就在原位上 ⇒ 到位
        assert_eq!(at_home(&[0.300, 0.0, 0.5], &m, 3.0), Some(true));
        // 差出十倍散布 ⇒ 没到位
        assert_eq!(at_home(&[0.300 + 10.0 * spread, 0.0, 0.5], &m, 3.0), Some(false));
        // 🔴 要求身体比它自己的重复性还准 ⇒ 拒绝回答,而不是永远判"没到位"
        assert_eq!(at_home(&[0.300, 0.0, 0.5], &m, 0.5), None);
    }
}

// ------------------------------------------------------------------------ backlash

/// Measure **backlash**: the dead band a reversal has to cross before the body starts moving again,
/// in command units.
///
/// # 🔴 It is measured as an EXCESS over a matched same-direction control, never as a raw shortfall
///
/// This is the whole design, and the reason is a body already in this repository. One arm delivered
/// **0.11** of every commanded step (see [`step_delivery`]). Reading the post-reversal shortfall
/// directly on that arm reports a dead band of ~0.89 of the commanded magnitude — an enormous,
/// completely fictional backlash that is really just the arm's ordinary delivery. Same-direction
/// steps are the control, and only the difference is attributable to a reversal.
///
/// So each reversal is scored against this body's own median continuation ratio `f`:
/// `d_i = |cmd_i| · (1 − ratio_i / f)`. If the body has no dead band, `d_i` scatters around zero
/// however badly it delivers.
///
/// # Why it demands reversals at more than one commanded magnitude
///
/// At a single magnitude `c`, a dead band `d` and a reversal-specific delivery deficit are
/// **perfectly confounded** — every observation is consistent with both, and no arithmetic
/// separates them. They separate only across magnitudes: a dead band is a constant number of
/// command units, a delivery deficit is a constant fraction. A sweep at one magnitude is therefore
/// refused, not answered.
///
/// # A zero reading is a measurement, not a refusal
///
/// A body with no slop is a real body. It gets `value ≈ 0` with an honest σ, and a caller that
/// needs the number to a tolerance it does not support is refused by the admit gate's precision
/// check — which is the mechanism that already exists for exactly this. Refusing every backlash-free
/// body would make the probe unable to report the truth.
pub fn backlash(
    steps: &[(f64, f64)], // (SIGNED commanded delta, SIGNED observed delta), time-ordered, same units
    now_ns: u64,
) -> Result<Measurement, Declined> {
    let mut cmd = [0.0f64; 256];
    let mut obs = [0.0f64; 256];
    let mut k = 0usize;
    for &(c, o) in steps {
        if !c.is_finite() || !o.is_finite() || c == 0.0 {
            // A step nobody commanded says nothing about a dead band, and dividing by it would put
            // the answer wherever the noise floor happens to sit.
            continue;
        }
        if k == cmd.len() {
            break;
        }
        cmd[k] = c;
        obs[k] = o;
        k += 1;
    }
    if k < 8 {
        return Err(Declined::NotEnoughSamples);
    }

    let mut cont = [0.0f64; 256]; // delivery ratio on same-direction steps
    let mut rev_ratio = [0.0f64; 256];
    let mut rev_mag = [0.0f64; 256];
    let (mut n_cont, mut n_rev) = (0usize, 0usize);
    for i in 1..k {
        let reversed = (cmd[i] > 0.0) != (cmd[i - 1] > 0.0);
        let ratio = obs[i] / cmd[i]; // signed/signed: positive when it moved the way it was told
        if reversed {
            rev_ratio[n_rev] = ratio;
            rev_mag[n_rev] = cmd[i].abs();
            n_rev += 1;
        } else {
            cont[n_cont] = ratio;
            n_cont += 1;
        }
    }
    if n_rev == 0 {
        // Never pushed both ways. A dead band at a reversal cannot be located from motion in one
        // direction — the same shape of refusal `reach` gives for a wall no sample ever straddled.
        return Err(Declined::Inconsistent);
    }
    if n_cont == 0 {
        // 🔴 No control. Without same-direction steps the post-reversal shortfall is exactly the
        // confound above, and reporting it would manufacture a dead band on any slow body.
        return Err(Declined::Inconsistent);
    }
    if n_rev < 3 || n_cont < 3 {
        return Err(Declined::NotEnoughSamples);
    }

    let f = median_in_place(&mut cont[..n_cont]);
    if f <= 0.0 {
        // Commanded both ways and the body did not move with the command. A dead joint, or a sign
        // convention that is inverted — either way not "the dead band is very large".
        return Err(Declined::NoResponse);
    }
    // 🔴 The control ratio must itself be established, not merely positive. FOUND BY REAL DATA, and
    // it is the difference between a probe and a plausible-looking number.
    //
    // On 300-step sweep logs from a 7-joint arm, one joint's same-direction steps gave
    // `f = 0.00025` with a standard error of **0.279** — the ratios were scattered, not small. The
    // estimator divides by `f`, so that joint came back with a dead band of **1.01 rad**: about
    // 58°, on a simulated arm that has none at all. Nothing else in the reading looked wrong.
    //
    // The guard is the same shape as `reach`'s: a quantity whose own spread does not separate it
    // from zero cannot carry another quantity. It also refuses the right things for the right
    // reason — on the two sweeps where the leg was pressed into a surface, the joints are fighting
    // the contact and have no established free-motion ratio, so those are refused while the
    // free-space sweep is admitted on all six joints that moved.
    let mut cont_scratch = [0.0f64; 256];
    let f_sigma = mad_sigma(&cont[..n_cont], f, &mut cont_scratch);
    let f_se = 1.2533 * f_sigma / (n_cont as f64).sqrt();
    if !(f > 2.0 * f_se) {
        return Err(Declined::Inconsistent);
    }

    let (mut mag_lo, mut mag_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut d = [0.0f64; 256];
    for i in 0..n_rev {
        d[i] = rev_mag[i] * (1.0 - rev_ratio[i] / f);
        mag_lo = mag_lo.min(rev_mag[i]);
        mag_hi = mag_hi.max(rev_mag[i]);
    }
    if !(mag_lo < mag_hi) {
        // Reversals at one magnitude only: a dead band and a reversal-specific delivery deficit are
        // not separable here. See the note above; this is a refusal, not a small correction.
        return Err(Declined::Inconsistent);
    }

    let med = median_in_place(&mut d[..n_rev]);
    let mut scratch = [0.0f64; 256];
    let sigma = mad_sigma(&d[..n_rev], med, &mut scratch);
    // Standard error of a median: σ·1.2533/√n for normal spread. Both constants describe the
    // ESTIMATOR (normal consistency, median efficiency), not this body — move to another robot and
    // neither changes.
    let se = 1.2533 * sigma / (n_rev as f64).sqrt();
    if !med.is_finite() || !se.is_finite() {
        return Err(Declined::Inconsistent);
    }

    let mut m = blank(Quantity::Backlash, 1, now_ns);
    m.value[0] = med;
    m.uncertainty[0] = se;
    m.valid_lo[0] = mag_lo;
    m.valid_hi[0] = mag_hi;
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

// --------------------------------------------------------------- contact threshold

/// Measure **contact threshold**: what "I touched something" reads like *on this body*, on whatever
/// scalar this body actually has (joint current, wrist force, tracking error).
///
/// # It needs both classes, and it refuses when they overlap
///
/// A threshold is only meaningful if free space and contact are separable on this signal. Fitting
/// one to a single class — "take the 99th percentile of free space" — always produces a number, and
/// that number is a detector that fires at a rate nobody measured. This project has already shipped
/// a grasp detector that closed on air and scored it as a grasp; the missing piece was exactly this,
/// a threshold nobody had shown could tell the two conditions apart.
///
/// So the caller must supply both, and if the two do not separate by more than counting noise the
/// answer is a refusal. On a body whose contact signal is genuinely uninformative that refusal is
/// the correct output, and the caller's next move is a different signal, not a lower bar.
///
/// # Measured against the arm's own weight, on purpose
///
/// Any contact signal a joint can produce carries the gravity load in it. Pick up a payload and the
/// free-space distribution moves, so the threshold measured before is wrong while its own clock
/// still says fresh. Declaring [`Quantity::ArmWeight`] as a dependency is what makes the layer
/// *notice* — and "add a weight and see whether the system notices" is the admission test for this
/// whole category.
///
/// # Where the threshold is put between the two classes
///
/// At the point where both classes are the same number of their own standard deviations away:
/// `t = (μ_free·σ_touch + μ_touch·σ_free) / (σ_free + σ_touch)`. A midpoint would sit too close to
/// whichever class is noisier, and any weighting chosen by hand is a per-rig constant.
/// Which way this body's contact signal moves when it touches something.
///
/// 🔴 Not a default. A force channel reads higher on contact; a *did the commanded motion happen*
/// channel reads LOWER, and this project's own validated detector is the second kind (0.18 free vs
/// 0.0001 touching, 289 steps, zero overlap). Guessing costs a detector that fires in free space
/// and stays silent on contact, which is the failure the direction check exists to catch.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Polarity {
    /// Force, current, torque: pressing makes the number bigger.
    HigherOnContact = 0,
    /// Delivered-motion, achieved-step: pressing makes the number smaller.
    LowerOnContact = 1,
}

pub fn contact_threshold(
    free: &[f64],     // signal while moving in free space
    touching: &[f64], // signal while pressed against something
    polarity: Polarity,
    now_ns: u64,
    arm_weight_epoch: u64,
) -> Result<Measurement, Declined> {
    fn moments(x: &[f64]) -> (f64, f64, f64, f64, f64) {
        let (mut n, mut s, mut lo, mut hi) = (0.0f64, 0.0f64, f64::INFINITY, f64::NEG_INFINITY);
        for &v in x {
            if !v.is_finite() {
                continue;
            }
            n += 1.0;
            s += v;
            lo = lo.min(v);
            hi = hi.max(v);
        }
        if n < 2.0 {
            return (n, f64::NAN, f64::NAN, lo, hi);
        }
        let mean = s / n;
        let mut ss = 0.0;
        for &v in x {
            if v.is_finite() {
                ss += (v - mean) * (v - mean);
            }
        }
        (n, mean, (ss / (n - 1.0)).sqrt(), lo, hi)
    }
    let (n_f, mu_f, sd_f, lo_f, hi_f) = moments(free);
    let (n_t, mu_t, sd_t, lo_t, hi_t) = moments(touching);
    // Eight of each: below that the standard error of the mean is wider than most of the gaps this
    // is asked to resolve, and the separation test degenerates into "the two samples happened to
    // differ".
    if n_f < 8.0 || n_t < 8.0 {
        return Err(Declined::NotEnoughSamples);
    }
    if !mu_f.is_finite() || !mu_t.is_finite() || !sd_f.is_finite() || !sd_t.is_finite() {
        return Err(Declined::Inconsistent);
    }
    if sd_f == 0.0 && sd_t == 0.0 && mu_f == mu_t {
        // Every reading identical in both conditions: the channel is stuck, not noiseless.
        return Err(Declined::NoResponse);
    }
    // 🔴 The DIRECTION is checked, and the caller states which direction it expects.
    //
    // This used to hard-code "contact must read HIGHER than free space", which is right for a force
    // channel and BACKWARDS for the ruler this project actually validated: *how much of the
    // commanded downward motion actually happened*, which reads 0.18 in free space and 0.0001 on
    // contact -- 289 steps, zero overlap. So the probe refused the one contact detector here that
    // had been measured to work.
    //
    // Caught by the C client on its first run. Polarity is a property of the SIGNAL, not a law, and
    // the fix is not to take |mu_t - mu_f|: that would let a swapped pair and an inverted sign
    // through looking healthy, which is what the original comment correctly warned about. The
    // caller says which way its signal goes, and a contradiction is still refused.
    let (mu_lo, mu_hi) = match polarity {
        Polarity::HigherOnContact => (mu_f, mu_t),
        Polarity::LowerOnContact => (mu_t, mu_f),
    };
    if mu_hi <= mu_lo {
        return Err(Declined::Inconsistent);
    }
    let se = (sd_f * sd_f / n_f + sd_t * sd_t / n_t).sqrt();
    if !(se > 0.0) || (mu_hi - mu_lo) < 2.0 * se {
        // Free space and contact read alike on this body. There is no threshold to report, and
        // reporting one would ship a detector that fires at a rate nobody measured.
        return Err(Declined::Inconsistent);
    }

    // Propagated standard error of the boundary, whichever branch places it.
    let denom = sd_f + sd_t;
    let (w_f, w_t) = if denom > 0.0 {
        (sd_t / denom, sd_f / denom)
    } else {
        (0.5, 0.5)
    };
    let se_prop = ((w_f * sd_f / n_f.sqrt()).powi(2) + (w_t * sd_t / n_t.sqrt()).powi(2)).sqrt();

    let gap = lo_t - hi_f;
    let (t, mut t_sigma) = if gap > 0.0 {
        // 🔴 A clean gap. Every value inside it classifies every observed sample identically, so the
        // middle is the maximum-margin choice and the only one with no free parameter — and half the
        // gap is exactly how far the boundary could move without contradicting anything that was
        // seen. Both facts are read off the data.
        //
        // This branch is not hypothetical tidiness. On the rig whose real logs this was checked
        // against, the free-space channel reads **exactly 0.000** on all 120 samples: σ_free is 0,
        // the weighted formula below collapses onto μ_free, and the threshold would land precisely
        // on an observed free-space reading with a reported σ of **zero** — a boundary that fires on
        // its own negative class while claiming perfect certainty about where it sits.
        (hi_f + 0.5 * gap, 0.5 * gap)
    } else {
        // Overlap: put it where both classes are the same number of their own σ away. A midpoint
        // would sit too close to whichever class is noisier, and any hand-chosen weighting is a
        // per-rig constant.
        let t = if denom > 0.0 {
            (mu_f * sd_t + mu_t * sd_f) / denom
        } else {
            0.5 * (mu_f + mu_t)
        };
        (t, 0.0)
    };
    t_sigma = t_sigma.max(se_prop);
    // Note what the floor does NOT do: it does not weaken the detector. A caller that only wants
    // "did I touch" passes no tolerance and is admitted. Only a caller claiming the threshold is
    // pinned tighter than the evidence pins it is refused, which is the correct outcome.
    if !t.is_finite() || !t_sigma.is_finite() {
        return Err(Declined::Inconsistent);
    }

    let mut m = blank(Quantity::ContactThreshold, 1, now_ns);
    m.value[0] = t;
    m.uncertainty[0] = t_sigma;
    // The domain is the range of signal values actually seen in either condition. A reading far
    // outside anything this body has ever produced is not a contact decision this measurement can
    // support, and the gate refuses it rather than extrapolating the classifier.
    m.valid_lo[0] = lo_f.min(lo_t);
    m.valid_hi[0] = hi_f.max(hi_t);
    m.deps[0] = Some((Quantity::ArmWeight, arm_weight_epoch));
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

// ---------------------------------------------------------------- self occlusion

/// Columns in the self-occlusion map.
pub const OCCLUSION_COLS: usize = 6;
/// Rows in the self-occlusion map.
pub const OCCLUSION_ROWS: usize = 4;
/// Cells in the self-occlusion map. Sized to fit a `u32` mask and to stay under `MAX_DIM`; the map
/// is coarse on purpose, because a fine map invites treating it as a segmentation rather than as a
/// frequency over poses.
pub const OCCLUSION_CELLS: usize = OCCLUSION_COLS * OCCLUSION_ROWS;

/// Measure **self-occlusion**: how often this body blocks each part of its own view, as it sweeps.
///
/// `value[i]` is the fraction of swept poses in which cell `i` was covered by the body's own
/// silhouette; cells are row-major, [`OCCLUSION_ROWS`] × [`OCCLUSION_COLS`], and the caller passes
/// one `u32` bitmask per pose.
///
/// # 🔴 An all-zero map is refused, and that is the point
///
/// "Zero self-occlusion everywhere" and "the silhouette detector never fired" produce the identical
/// output, and the second is far more common. This repository's canonical version of that mistake
/// read *"0 distraction artifacts"* off a pipeline whose camera was mis-aimed — the **absence** of
/// the expected failures was itself the fingerprint, and it was filed as a clean result. So an
/// all-zero sweep is [`Declined::NoResponse`], not a measurement. A camera that genuinely sees no
/// self-occlusion is a real configuration, and the way to establish it is a positive control that
/// makes the detector fire, not a map that cannot be told apart from a dead one. An all-**ones** map
/// is refused for the mirror reason: a detector stuck on.
///
/// # And a map that does not move with the arm is not self-occlusion
///
/// If the covered cells are identical at every pose, whatever is blocking the view is **not moving
/// with the body** — a bracket, a smudge, a fixed cable. Attributing it to the arm would let a
/// permanently dirty lens be reported as this robot's silhouette, and every downstream decision
/// about "can I see that pixel" would inherit it. This is the same shape as the elbow trap in
/// `hand.rs`: a rule that attributes whatever it finds to the robot does not report an error when
/// the thing it found is not the robot.
pub fn self_occlusion(
    sweep: &[(f64, u32)], // (pose coordinate, bitmask of cells covered by this body's silhouette)
    now_ns: u64,
    jac_epoch: u64,
) -> Result<Measurement, Declined> {
    const FULL: u32 = if OCCLUSION_CELLS >= 32 {
        u32::MAX
    } else {
        (1u32 << OCCLUSION_CELLS) - 1
    };
    let mut count = [0u32; OCCLUSION_CELLS];
    let (mut pose_lo, mut pose_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut n = 0u32;
    let mut first_mask: Option<u32> = None;
    let mut mask_varied = false;
    let mut any_set = false;
    let mut all_full = true;
    for &(pose, mask) in sweep {
        if !pose.is_finite() {
            continue;
        }
        let mask = mask & FULL;
        n += 1;
        pose_lo = pose_lo.min(pose);
        pose_hi = pose_hi.max(pose);
        match first_mask {
            None => first_mask = Some(mask),
            Some(m0) if m0 != mask => mask_varied = true,
            Some(_) => {}
        }
        if mask != 0 {
            any_set = true;
        }
        if mask != FULL {
            all_full = false;
        }
        for (i, c) in count.iter_mut().enumerate() {
            if mask & (1u32 << i) != 0 {
                *c += 1;
            }
        }
    }
    // Twelve poses: a frequency over fewer is a coin flip with a decimal point on it.
    if n < 12 {
        return Err(Declined::NotEnoughSamples);
    }
    if !(pose_lo < pose_hi) {
        // Every sample at one pose. The map then describes a pose, not a body, and there is no
        // domain for it to be valid over.
        return Err(Declined::Inconsistent);
    }
    if !any_set {
        return Err(Declined::NoResponse);
    }
    if all_full {
        return Err(Declined::NoResponse);
    }
    if !mask_varied {
        return Err(Declined::Inconsistent);
    }

    let mut m = blank(Quantity::SelfOcclusion, OCCLUSION_CELLS, now_ns);
    let nf = f64::from(n);
    for i in 0..OCCLUSION_CELLS {
        let p = f64::from(count[i]) / nf;
        m.value[i] = p;
        // Jeffreys-smoothed binomial standard error. A cell never seen occluded is not a cell known
        // never to be occluded, and a σ of 0 there would claim a certainty n observations cannot
        // supply — which is precisely how a map with a few unlucky cells becomes a hard constraint.
        let ps = (f64::from(count[i]) + 0.5) / (nf + 1.0);
        m.uncertainty[i] = (ps * (1.0 - ps) / (nf + 1.0)).sqrt();
        // Every cell was measured over the same sweep, so every cell carries the same domain: the
        // poses actually visited. Asking about a pose outside them is refused, exactly as
        // `arm_weight` refuses a joint angle it never held.
        m.valid_lo[i] = pose_lo;
        m.valid_hi[i] = pose_hi;
    }
    // The map is expressed in the camera's frame; knock the camera and every cell is wrong while
    // the arm has not moved at all.
    m.deps[0] = Some((Quantity::ImageJacobian, jac_epoch));
    m.valid_for_ns = 0;
    m.selftest_passed = true;
    Ok(m)
}

// ------------------------------------------------------------------------ tool offset

/// Measure **tool offset**: how far the working point sits from the mount that is actually
/// commanded, along the tool axis, in metres.
///
/// # This probe exists because one number is typed in four times in the live stack
///
/// * `L3_GRIPPER_BIAS`, default **0.145**, with the comment beside it: *"x5 = 0.145,
///   franka = 0.102"* — 4.3 cm apart between two bodies, to be copied by hand out of
///   `Assets/Robots/<body>/robot_config.yml`. A machine that forgets to pass it does not fail; it
///   executes with **another robot's geometry**.
/// * the same 0.145 hardcoded in the teacher's flange↔TCP transform.
/// * the same 0.145 a third time, as the arithmetic behind a wrist-tilt ceiling.
/// * `tcp_off: 0.1034` on a third rig.
///
/// # How a body measures it on itself
///
/// Turn the wrist about the mount and hold everything else still. The working point sweeps an
/// **arc**, and the radius of that arc *is* the offset. Nothing about the robot's kinematics is
/// needed and no frame has to be declared — the geometry is in the picture.
///
/// A circle is fitted to the observed points (algebraic least squares, centred first for
/// conditioning) and the radius is converted with the same `units_per_m` ruler
/// [`gripper_span`] uses, with the ruler's own error propagated.
///
/// # 🔴 Why [`Quantity::HandPixel`] is deliberately NOT declared as a dependency
///
/// The points come from the hand tracker, so the reflex is to record it as a dependency. That would
/// be wrong here, and the reason is a property of the tracker rather than of the tool: the hand
/// point is **re-measured every control step** and bumps its epoch every time, so a quantity
/// depending on it would be invalid one step after it was taken — permanently, on every body. The
/// dependency that is real is the camera, through the ruler, and that is the one recorded.
///
/// The soundness this gives up is small and named: if the tracker had been wrong, the offset would
/// be wrong. It does not answer when it is unsure — that is the whole of `hand.rs` — so the inputs
/// are either right or absent, and a set that is not mutually consistent fails the fit test below.
///
/// # What it refuses
///
/// * no ruler ⇒ [`Declined::MissingDependency`];
/// * the wrist never turned ⇒ [`Declined::Inconsistent`] — every point at one angle fits every
///   circle, so any radius could be reported and would be believed;
/// * points that lie on a line ⇒ [`Declined::Inconsistent`] — the fit is singular, and the
///   determinant guard catches it before a division does;
/// * a radius the fit cannot separate from zero ⇒ [`Declined::NoResponse`] — the working point did
///   not trace anything distinguishable from a point. That is either a tool with no offset **or**
///   a wrist that did not actually turn, and merging those two is how a body with a 14.5 cm tool
///   comes to believe it has none.
pub fn tool_offset(
    arc: &[(f64, f64, f64)], // (wrist angle rad, u, v) with u,v in image units
    units_per_m: f64,
    units_per_m_sigma: f64,
    now_ns: u64,
    jac_epoch: u64,
) -> Result<Measurement, Declined> {
    if !units_per_m.is_finite()
        || units_per_m <= 0.0
        || !units_per_m_sigma.is_finite()
        || units_per_m_sigma < 0.0
    {
        return Err(Declined::MissingDependency);
    }
    let mut u = [0.0f64; 256];
    let mut v = [0.0f64; 256];
    let (mut a_lo, mut a_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut k = 0usize;
    for &(ang, uu, vv) in arc {
        if !ang.is_finite() || !uu.is_finite() || !vv.is_finite() {
            continue;
        }
        if k == u.len() {
            break;
        }
        u[k] = uu;
        v[k] = vv;
        k += 1;
        a_lo = a_lo.min(ang);
        a_hi = a_hi.max(ang);
    }
    // Three points determine a circle exactly and leave no residual, so there is no uncertainty to
    // report and nothing the admit gate could refuse on. Five is the smallest that leaves two.
    if k < 5 {
        return Err(Declined::NotEnoughSamples);
    }
    if !(a_lo < a_hi) {
        return Err(Declined::Inconsistent);
    }

    // Centre first: the algebraic fit's normal equations are badly conditioned on data far from the
    // origin, and normalised image coordinates sit around 0.5 with a radius of a few hundredths.
    let n = k as f64;
    let (mut cu, mut cv) = (0.0, 0.0);
    for i in 0..k {
        cu += u[i];
        cv += v[i];
    }
    cu /= n;
    cv /= n;

    // Fit u^2 + v^2 = 2a·u + 2b·v + c by least squares on the centred points.
    let (mut suu, mut suv, mut svv, mut su, mut sv) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let (mut szu, mut szv, mut sz) = (0.0, 0.0, 0.0);
    for i in 0..k {
        let (x, y) = (u[i] - cu, v[i] - cv);
        let z = x * x + y * y;
        suu += x * x;
        suv += x * y;
        svv += y * y;
        su += x;
        sv += y;
        szu += z * x;
        szv += z * y;
        sz += z;
    }
    // Normal equations for [2a, 2b, c] with the centred design matrix [x, y, 1].
    let m = [[suu, suv, su], [suv, svv, sv], [su, sv, n]];
    let rhs = [szu, szv, sz];
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    // Scale-free singularity test: compare the determinant against the product of the diagonal,
    // which is what it would be for a well-spread set. Collinear points drive it to zero, and a
    // division by it would return a radius of whatever the rounding error happened to be.
    let scale = (suu * svv * n).abs();
    if !det.is_finite() || scale <= 0.0 || det.abs() <= 1e-12 * scale {
        return Err(Declined::Inconsistent);
    }
    let solve = |col: usize| -> f64 {
        let mut t = m;
        for (r, x) in t.iter_mut().enumerate() {
            x[col] = rhs[r];
        }
        (t[0][0] * (t[1][1] * t[2][2] - t[1][2] * t[2][1])
            - t[0][1] * (t[1][0] * t[2][2] - t[1][2] * t[2][0])
            + t[0][2] * (t[1][0] * t[2][1] - t[1][1] * t[2][0]))
            / det
    };
    let (a2, b2, c) = (solve(0), solve(1), solve(2));
    let (ca, cb) = (0.5 * a2, 0.5 * b2);
    let r2 = ca * ca + cb * cb + c;
    if !r2.is_finite() || r2 <= 0.0 {
        return Err(Declined::Inconsistent);
    }
    let radius = r2.sqrt();

    // Residual: how far each point sits from the fitted circle. This is what the reported σ is made
    // of, and it is what catches an arm that translated while the wrist turned.
    let mut ss = 0.0;
    for i in 0..k {
        let (x, y) = (u[i] - cu, v[i] - cv);
        let d = ((x - ca) * (x - ca) + (y - cb) * (y - cb)).sqrt() - radius;
        ss += d * d;
    }
    let resid = (ss / (n - 3.0)).sqrt();
    let r_sigma = resid / n.sqrt();
    if !r_sigma.is_finite() {
        return Err(Declined::Inconsistent);
    }
    if radius <= 2.0 * r_sigma {
        return Err(Declined::NoResponse);
    }

    let offset_m = radius / units_per_m;
    let rel = ((r_sigma / radius).powi(2) + (units_per_m_sigma / units_per_m).powi(2)).sqrt();

    let mut out = blank(Quantity::ToolOffset, 1, now_ns);
    out.value[0] = offset_m;
    out.uncertainty[0] = offset_m * rel;
    // The domain is the wrist travel actually swept. An offset read off a 20° arc says nothing
    // about a tool that flexes at 90°, and the gate refuses that ask rather than extrapolating.
    out.valid_lo[0] = a_lo;
    out.valid_hi[0] = a_hi;
    out.deps[0] = Some((Quantity::ImageJacobian, jac_epoch));
    out.valid_for_ns = 0;
    out.selftest_passed = true;
    Ok(out)
}

// ------------------------------------------------------------------------- helpers

/// Median of `x`, sorting it in place. `x` must be non-empty and free of NaN — every caller here
/// filters non-finite input before arriving, because a NaN reaching the comparator is a panic, and
/// a panic in this crate spins forever by design (see the panic handler in `lib.rs`).
fn median_in_place(x: &mut [f64]) -> f64 {
    x.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let k = x.len();
    if k % 2 == 1 {
        x[k / 2]
    } else {
        0.5 * (x[k / 2 - 1] + x[k / 2])
    }
}

/// Median absolute deviation about `med`, scaled by 1.4826 to the normal-consistent estimator so
/// the number the admit gate compares against a precision ask means the same thing as a standard
/// deviation. `dev` is scratch, at least `x.len()` long.
fn mad_sigma(x: &[f64], med: f64, dev: &mut [f64]) -> f64 {
    for (i, v) in x.iter().enumerate() {
        dev[i] = (v - med).abs();
    }
    1.4826 * median_in_place(&mut dev[..x.len()])
}

fn blank(q: Quantity, dim: usize, now_ns: u64) -> Measurement {
    Measurement {
        axis_kind: [AxisKind::Interval; MAX_DIM],
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
