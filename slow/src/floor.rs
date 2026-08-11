//! **How low this body can get, as a function of where it is** — and what that tells you the
//! moment a downward motion stops.
//!
//! # The one line this file exists for
//!
//! **A body's limit is a property of the CONFIGURATION and moves with the arm. A surface is a
//! property of the WORLD and stays where it is.**
//!
//! So: measure where downward motion stops across a grid, once. Afterwards every stop is read
//! against that map, and the three cases separate with no joint states, no force sensor, and no
//! extra command per decision:
//!
//! | where the hand stopped | what it is |
//! |---|---|
//! | at the floor | resting on the working surface — nothing on it here |
//! | **above** it | something IS here, and it is `stop − floor` tall |
//! | **below** it | there is no surface here; this is the arm's own limit |
//!
//! # What it replaces, and what that cost
//!
//! The previous answer was [`crate::touch`]: blocked going in, free coming back out. That rule is
//! sound but it is not sufficient, and its own probe refuted it within the hour — the same physical
//! situation read **1.000** to a 5 cm lift command and **0.000** to a 1 cm one, because 1 cm is
//! below this body's dead zone. *"Can I lift off?"* has no absolute answer; it depends on how hard
//! you ask, and `backlash` — the quantity that would say how hard — is REFUSED on this body.
//!
//! It also mis-handles the cases that made this necessary: a joint at its stop blocks one direction
//! and frees its opposite (→ false contact), and a jammed gripper blocks both (→ false free). The
//! floor map is immune to both, because it never asks the arm a question about itself in the
//! moment; it compares against what the arm already established about itself.
//!
//! The measurement that forced this: on a flat conveyor, nine downward probes stopped across
//! **13.9 cm** while the four genuinely on the belt agreed to **2.2 cm**. Every delivery reading
//! was correct, the threshold was correct, and the conveyor loop closed its jaws in mid-air on all
//! nine episodes with nothing in the log disagreeing.
//!
//! # Why a plane, and where it refuses
//!
//! A plane is the smallest model that can hold a tilted table, a belt, or a floor, and it is
//! honest about its own domain: `valid_lo/hi` is the box the grid actually covered, and asking
//! outside it refuses instead of extrapolating over ground nobody drove on. If the stops do not
//! lie on a plane, this **declines** rather than publishing a plane that fits nothing — a surface
//! model that quietly averages a table and the floor beside it is worse than no model.

use crate::measurement::{AxisKind, Measurement, Quantity, MAX_DEPS, MAX_DIM};
use crate::probe::Declined;
use crate::refuse::{Reason, Verdict};
use crate::Body;

/// Least samples that can pin a plane and still leave anything to check it with. Three fix a plane
/// exactly and would report a perfect fit for any three points whatsoever, including three that
/// straddle a table edge — a residual of zero from an exactly-determined fit is not evidence.
pub const MIN_SAMPLES: usize = 6;

/// How much of the grid must survive the robust refit before the region counts as one surface.
/// Below this the stops are not describing a single plane, and the answer is a refusal.
pub const MIN_INLIER_FRACTION: f64 = 0.6;

/// Where a stop sits relative to what this body established it can reach.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Stop {
    /// Resting on the working surface itself. Nothing on it here.
    OnFloor,
    /// Something is here. The payload is its height above the surface, in metres.
    OnSomething(f64),
    /// 🔴 Below the surface — so there is no surface here at all, and the stop is this arm running
    /// out of solution. NOT contact, however much the delivery ruler looks like it.
    ArmLimit(f64),
}

/// The answer plus its audit trail.
#[derive(Copy, Clone, Debug)]
pub struct Reading {
    /// What the stop was.
    pub stop: Option<Stop>,
    /// `admit` is false when the floor could not be consulted at all; `why` names the blocker.
    pub verdict: Verdict,
    /// The floor height this was judged against, and the band around it that counts as "at" the
    /// floor. `NaN` when the floor could not be read.
    pub floor_z: f64,
    /// Half-width of that band, from the plane fit's own residual spread — not a typed-in
    /// tolerance. A rougher surface widens its own band with nobody retuning anything.
    pub band: f64,
}

/// Fit the floor over a grid of "I pressed down here and stopped at this height" samples.
///
/// `xs`, `ys`, `zs` are the probe positions and the height motion stopped at. Robust by design:
/// the cells where the arm ran out of solution before reaching the surface are exactly the ones
/// that must not drag the plane, so the fit is refitted without its outliers and refuses if too
/// few survive.
///
/// `thr_epoch` / `sd_epoch` tie this to the rulers the stops were detected with, so re-measuring
/// either invalidates the map automatically.
pub fn fit(
    xs: &[f64],
    ys: &[f64],
    zs: &[f64],
    tol_m: f64,
    now_ns: u64,
    thr_epoch: u64,
    sd_epoch: u64,
) -> Result<Measurement, Declined> {
    let n = xs.len();
    if n != ys.len() || n != zs.len() {
        return Err(Declined::Inconsistent);
    }
    if n < MIN_SAMPLES {
        return Err(Declined::NotEnoughSamples);
    }
    if xs.iter().chain(ys).chain(zs).any(|v| !v.is_finite()) {
        return Err(Declined::Inconsistent);
    }

    // A grid that is a line in x or in y cannot pin both slopes. Fitting anyway returns a plane
    // that is exact along the line it was given and arbitrary across it — which then reads as a
    // confident answer everywhere off that line.
    let span = |v: &[f64]| {
        v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &x| (lo.min(x), hi.max(x)))
    };
    let (x_lo, x_hi) = span(xs);
    let (y_lo, y_hi) = span(ys);
    if x_hi - x_lo < 1e-6 || y_hi - y_lo < 1e-6 {
        return Err(Declined::NoResponse);
    }

    if !(tol_m.is_finite() && tol_m > 0.0) {
        return Err(Declined::Inconsistent);
    }

    // 🔴 Trim ONE point at a time and refit, capped. Two things were tried and are recorded because
    // both look right and both are wrong:
    //
    //   * a single MAD pass -- the two arm-limited cells (7-9 cm off) inflate the MAD enough to
    //     survive their own test, and the plane comes out at 0.9018 instead of 0.9190;
    //   * clustering the raw heights -- a genuinely TILTED table spans more than the tolerance and
    //     gets split into two surfaces that do not exist.
    //
    // Trimming against the plane handles both. The cap is what stops it "chasing whichever subset
    // fits best": at most `1 - MIN_INLIER_FRACTION` of the grid may ever be discarded.
    let mut keep: Vec<usize> = (0..n).collect();
    let max_drop = n - (MIN_INLIER_FRACTION * n as f64).ceil() as usize;
    let mut plane = solve_plane(xs, ys, zs, &keep).ok_or(Declined::Inconsistent)?;
    for _ in 0..max_drop {
        let worst = keep
            .iter()
            .copied()
            .max_by(|&a, &b| {
                let (ra, rb) = (
                    (zs[a] - eval(&plane, xs[a], ys[a])).abs(),
                    (zs[b] - eval(&plane, xs[b], ys[b])).abs(),
                );
                ra.partial_cmp(&rb).unwrap()
            })
            .unwrap();
        if (zs[worst] - eval(&plane, xs[worst], ys[worst])).abs() <= tol_m {
            break;
        }
        keep.retain(|&i| i != worst);
        plane = solve_plane(xs, ys, zs, &keep).ok_or(Declined::Inconsistent)?;
    }
    // Still not a plane after spending the whole trim budget: the grid is describing more than one
    // surface, and averaging a table with the floor beside it is worse than having no map.
    if keep
        .iter()
        .any(|&i| (zs[i] - eval(&plane, xs[i], ys[i])).abs() > tol_m)
    {
        return Err(Declined::Inconsistent);
    }

    // The band is the inliers' own residual spread. Everything downstream compares against this,
    // so a rough surface automatically gets a wider "at the floor" band.
    let inlier_resid: Vec<f64> = keep.iter().map(|&i| zs[i] - eval(&plane, xs[i], ys[i])).collect();
    let sigma = rms(&inlier_resid);

    let cx = 0.5 * (x_lo + x_hi);
    let cy = 0.5 * (y_lo + y_hi);
    let mut m = Measurement {
        axis_kind: [AxisKind::Interval; MAX_DIM],
        quantity: Quantity::Floor,
        dim: 3,
        value: [0.0; MAX_DIM],
        uncertainty: [0.0; MAX_DIM],
        valid_lo: [0.0; MAX_DIM],
        valid_hi: [0.0; MAX_DIM],
        measured_at_ns: now_ns,
        // 0 = valid until a dependency changes. A floor does not go stale on a clock; it goes
        // stale when somebody moves the table, and that shows up as the next fit disagreeing.
        valid_for_ns: 0,
        deps: [None; MAX_DEPS],
        epoch: 0,
        selftest_passed: true,
        prev_epoch: 0,
    };
    // 🔴 value axes and domain axes are DIFFERENT here; see `Quantity::Floor`.
    m.value[0] = eval(&plane, cx, cy);
    m.value[1] = plane[1];
    m.value[2] = plane[2];
    m.uncertainty[0] = sigma;
    m.uncertainty[1] = 0.0;
    m.uncertainty[2] = 0.0;
    m.valid_lo[0] = x_lo;
    m.valid_hi[0] = x_hi;
    m.valid_lo[1] = y_lo;
    m.valid_hi[1] = y_hi;
    m.axis_kind[2] = AxisKind::Unmeasured;
    m.deps[0] = Some((Quantity::ContactThreshold, thr_epoch));
    m.deps[1] = Some((Quantity::StepDelivery, sd_epoch));
    Ok(m)
}

/// How low this body can get at `(x, y)`, the band around it, and whether the grid actually
/// drove there. `false` = answered by extending the plane; the caller is told and decides.
pub fn at(body: &Body, x: f64, y: f64) -> Result<(f64, f64, bool), Verdict> {
    let Some(f) = body.get(Quantity::Floor) else {
        return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::Floor));
    };
    if !f.selftest_passed {
        return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::Floor));
    }
    if !(x.is_finite() && y.is_finite()) {
        return Err(Verdict::refuse(Reason::OutOfRange, Quantity::Floor));
    }
    // 🔴 OUTSIDE THE PROBED BOX IS THE THIRD RUNG, NOT A REFUSAL.
    //
    // A hard refusal here was measured and it cascades: on the conveyor the hand works over 1.5 m
    // of belt across two arms, a grid can only cover part of it, and refusing everything else
    // meant 163 asks produced 160 refusals and the loop never acted at all -- while the belt was
    // measured to be ONE plane over that whole span (0.9205-0.9219, 1.4 mm across both arms).
    //
    // "I have never driven here, and here is my best answer" is strictly more useful than silence,
    // as long as the caller is told. `measurement::AxisKind` records the same lesson from the
    // other direction: hard-refusing an unprobed axis collapsed everything downstream of one
    // constant into unusable.
    let cx = 0.5 * (f.valid_lo[0] + f.valid_hi[0]);
    let cy = 0.5 * (f.valid_lo[1] + f.valid_hi[1]);
    let z = f.value[0] + f.value[1] * (x - cx) + f.value[2] * (y - cy);
    let outside =
        x < f.valid_lo[0] || x > f.valid_hi[0] || y < f.valid_lo[1] || y > f.valid_hi[1];
    if outside {
        return Ok((z, f.uncertainty[0], false));
    }
    Ok((z, f.uncertainty[0], true))
}

/// 🔴 THE WHOLE QUESTION IN ONE CALL: *the hand stopped here — what stopped it?*
///
/// `band_sigmas` is the caller's: how many residual widths still count as "on the floor". It is a
/// task accuracy requirement, not a body constant — a peg-in-hole wants it tight, a sweep wants it
/// loose — so it is asked for rather than chosen here.
pub fn read_stop(body: &Body, x: f64, y: f64, stop_z: f64, band_sigmas: f64) -> Reading {
    let none = |v: Verdict| Reading {
        stop: None,
        verdict: v,
        floor_z: f64::NAN,
        band: f64::NAN,
    };
    if !stop_z.is_finite() || !(band_sigmas.is_finite() && band_sigmas > 0.0) {
        return none(Verdict::refuse(Reason::OutOfRange, Quantity::Floor));
    }
    let (z, sigma, verified) = match at(body, x, y) {
        Ok(v) => v,
        Err(v) => return none(v),
    };
    // A perfectly flat fit gives sigma = 0, and then every stop is either above or below and
    // nothing is ever "on" the floor. The floor's own uncertainty is the right width; when it is
    // exactly zero the map is claiming to know a surface exactly, which nothing measured does.
    if !(sigma.is_finite() && sigma > 0.0) {
        return none(Verdict::refuse(Reason::UncertaintyTooHigh, Quantity::Floor));
    }
    let band = band_sigmas * sigma;
    let d = stop_z - z;
    let stop = if d.abs() <= band {
        Stop::OnFloor
    } else if d > 0.0 {
        Stop::OnSomething(d)
    } else {
        Stop::ArmLimit(-d)
    };
    Reading {
        stop: Some(stop),
        // Answered by extending the plane past where the grid drove: admitted, and said so.
        verdict: if verified { Verdict::OK } else { Verdict::unverified(Quantity::Floor) },
        floor_z: z,
        band,
    }
}

// ---------------------------------------------------------------- plain arithmetic

/// Least squares for `z = a + b·x + c·y` over `keep`. `None` when the normal equations are
/// singular, which is the degenerate-grid case reaching here by another route.
fn solve_plane(xs: &[f64], ys: &[f64], zs: &[f64], keep: &[usize]) -> Option<[f64; 3]> {
    let n = keep.len() as f64;
    if n < 3.0 {
        return None;
    }
    let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
    for &i in keep {
        sx += xs[i];
        sy += ys[i];
        sz += zs[i];
    }
    let (mx, my, mz) = (sx / n, sy / n, sz / n);
    let (mut sxx, mut sxy, mut syy, mut sxz, mut syz) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for &i in keep {
        let (dx, dy, dz) = (xs[i] - mx, ys[i] - my, zs[i] - mz);
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
        sxz += dx * dz;
        syz += dy * dz;
    }
    let det = sxx * syy - sxy * sxy;
    if det.abs() < 1e-18 {
        return None;
    }
    let b = (sxz * syy - syz * sxy) / det;
    let c = (syz * sxx - sxz * sxy) / det;
    Some([mz - b * mx - c * my, b, c])
}

fn eval(p: &[f64; 3], x: f64, y: f64) -> f64 {
    p[0] + p[1] * x + p[2] * y
}

fn rms(v: &[f64]) -> f64 {
    if v.len() < 2 {
        return 0.0;
    }
    (v.iter().map(|x| x * x).sum::<f64>() / (v.len() as f64 - 1.0)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(f: impl Fn(f64, f64) -> f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let (mut xs, mut ys, mut zs) = (vec![], vec![], vec![]);
        for i in 0..3 {
            for j in 0..3 {
                let (x, y) = (-0.08 + 0.08 * i as f64, -0.08 + 0.08 * j as f64);
                xs.push(x);
                ys.push(y);
                zs.push(f(x, y));
            }
        }
        (xs, ys, zs)
    }

    /// The floor declares the two rulers its stops were detected with, and the store enforces that
    /// -- so a test body has to carry them. That enforcement is the point: re-measure either ruler
    /// and every floor built on it is invalidated without anyone remembering to.
    fn body_with(m: Measurement) -> Body {
        let mut b = Body::new();
        for (q, v) in [(Quantity::ContactThreshold, 0.29383), (Quantity::StepDelivery, 0.9999)] {
            let mut r = Measurement {
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
            r.value[0] = v;
            r.valid_hi[0] = 1.0;
            b.submit(r).expect("ruler must merge");
        }
        b.submit(m).expect("floor must merge");
        b
    }

    /// 🔴 THE CONVEYOR, REPLAYED. Four cells on the belt at 0.9059-0.9278 and two where the arm ran
    /// out of solution 7-9 cm lower. The two outliers must not drag the plane, and the arm-limited
    /// stop must come back as an arm limit rather than as contact -- which is what the old rule
    /// called it, on all nine episodes.
    #[test]
    fn the_cells_where_the_arm_ran_out_must_not_drag_the_plane() {
        let (mut xs, mut ys, mut zs) = grid(|_, _| 0.9190);
        zs[0] = 0.8475; // the two real false readings from results/stallwhat_aug2026
        zs[1] = 0.8343;
        // a little real roughness, so the band is not degenerate
        for (k, z) in zs.iter_mut().enumerate().skip(2) {
            *z += if k % 2 == 0 { 0.0012 } else { -0.0012 };
        }
        let m = fit(&xs, &ys, &zs, 0.01, 1, 1, 1).expect("seven of nine agree: that is a plane");
        assert!((m.value[0] - 0.9190).abs() < 0.005, "plane at {}", m.value[0]);
        assert!(m.uncertainty[0] < 0.01, "band {} must reflect the INLIERS", m.uncertainty[0]);

        let b = body_with(m);
        let r = read_stop(&b, xs[0], ys[0], 0.8475, 3.0);
        match r.stop {
            Some(Stop::ArmLimit(d)) => assert!(d > 0.05, "7 cm below the belt, got {d}"),
            other => panic!("the arm running out of solution is not contact: {other:?}"),
        }
        xs.truncate(0);
        ys.truncate(0);
    }

    #[test]
    fn something_on_the_surface_reads_as_something_and_reports_its_height() {
        let (xs, ys, mut zs) = grid(|_, _| 0.9190);
        for (k, z) in zs.iter_mut().enumerate() {
            *z += if k % 2 == 0 { 0.0010 } else { -0.0010 };
        }
        let b = body_with(fit(&xs, &ys, &zs, 0.01, 1, 1, 1).unwrap());
        // a 2 cm tall object under the hand
        match read_stop(&b, 0.0, 0.0, 0.9190 + 0.020, 3.0).stop {
            Some(Stop::OnSomething(h)) => assert!((h - 0.020).abs() < 0.003, "height {h}"),
            other => panic!("expected an object, got {other:?}"),
        }
        // and the bare surface is the surface
        assert_eq!(read_stop(&b, 0.0, 0.0, 0.9190, 3.0).stop, Some(Stop::OnFloor));
    }

    /// A tilted table must still be one surface -- the whole point of a plane over a constant.
    #[test]
    fn a_tilted_surface_is_still_one_surface() {
        let (xs, ys, mut zs) = grid(|x, y| 0.90 + 0.05 * x - 0.03 * y);
        for (k, z) in zs.iter_mut().enumerate() {
            *z += if k % 2 == 0 { 0.0004 } else { -0.0004 };
        }
        let b = body_with(fit(&xs, &ys, &zs, 0.01, 1, 1, 1).unwrap());
        for (x, y) in [(-0.08, -0.08), (0.08, 0.08), (0.0, 0.08)] {
            let want = 0.90 + 0.05 * x - 0.03 * y;
            assert_eq!(read_stop(&b, x, y, want, 3.0).stop, Some(Stop::OnFloor), "at {x},{y}");
        }
    }

    /// 🔴 Off the probed box it REFUSES. A plane is a local model and extrapolating it is exactly
    /// how a body constant measured on a table came to be used on a conveyor.
    #[test]
    fn outside_the_probed_box_answers_but_says_nothing_verified_it() {
        let (xs, ys, mut zs) = grid(|_, _| 0.9190);
        for (k, z) in zs.iter_mut().enumerate() {
            *z += if k % 2 == 0 { 0.0010 } else { -0.0010 };
        }
        let b = body_with(fit(&xs, &ys, &zs, 0.01, 1, 1, 1).unwrap());
        // 🔴 The third rung, not silence: it answers and says nothing verified it here.
        let r = read_stop(&b, 0.50, 0.0, 0.9190, 3.0);
        assert!(r.stop.is_some(), "refusing here cost 160 of 163 asks on the conveyor");
        assert!(r.verdict.admit);
        assert!(r.verdict.unverified);
        assert_eq!(r.verdict.why, Reason::NoEvidence);
        assert!(!at(&b, 0.0, 0.50).unwrap().2, "and `at` marks it unverified too");
    }

    /// Two surfaces in one grid is not a plane, and averaging them would be worse than nothing.
    #[test]
    fn a_grid_straddling_two_surfaces_declines() {
        let (xs, ys, mut zs) = grid(|_, _| 0.9190);
        // the floor beside the table, reached across the DIAGONAL -- no plane passes through this
        for k in [0, 4, 8, 5] {
            zs[k] = 0.7000;
        }
        assert_eq!(fit(&xs, &ys, &zs, 0.01, 1, 1, 1).unwrap_err(), Declined::Inconsistent);
    }

    /// 🔴 THE AMBIGUITY THIS ESTIMATOR CANNOT RESOLVE, PINNED SO IT IS NEVER MISTAKEN FOR A BUG.
    ///
    /// A step that runs along a grid axis -- the near half of a 3x3 on the table, the far half on
    /// the floor -- **is** a plane through those samples. It is not that the fit is fooled; there
    /// is genuinely no information in nine stop heights that separates "a step" from "a steep
    /// tilt". Both explain the data exactly.
    ///
    /// The mitigation is a denser or offset grid, not a cleverer estimator, and the tell is the
    /// slope: this comes out at 1.37 m per m, which no working surface is. A caller that cares can
    /// read `value[1..3]` and refuse; the layer does not invent a plausibility threshold it never
    /// measured.
    #[test]
    fn an_axis_aligned_step_is_indistinguishable_from_a_tilt_at_this_density() {
        let (xs, ys, mut zs) = grid(|_, _| 0.9190);
        for (k, z) in zs.iter_mut().enumerate() {
            if xs[k] < -0.01 {
                *z = 0.7000;
            }
        }
        let m = fit(&xs, &ys, &zs, 0.01, 1, 1, 1).expect("it fits, and that is the honest answer");
        assert!(m.value[1].abs() > 1.0, "the tell is the slope: {}", m.value[1]);
    }

    #[test]
    fn a_grid_that_is_a_line_cannot_pin_both_slopes() {
        let xs = vec![0.0; 8];
        let ys: Vec<f64> = (0..8).map(|i| 0.01 * i as f64).collect();
        let zs = vec![0.9; 8];
        assert_eq!(fit(&xs, &ys, &zs, 0.01, 1, 1, 1).unwrap_err(), Declined::NoResponse);
    }

    #[test]
    fn three_points_are_not_enough_to_check_a_plane() {
        let xs = vec![0.0, 0.1, 0.0];
        let ys = vec![0.0, 0.0, 0.1];
        let zs = vec![0.9, 0.9, 0.9];
        assert_eq!(fit(&xs, &ys, &zs, 0.01, 1, 1, 1).unwrap_err(), Declined::NotEnoughSamples);
    }

    #[test]
    fn an_unmeasured_floor_refuses_instead_of_guessing() {
        let b = Body::new();
        let r = read_stop(&b, 0.0, 0.0, 0.9, 3.0);
        assert!(r.stop.is_none());
        assert_eq!(r.verdict.why, Reason::NeverMeasured);
        assert_eq!(r.verdict.culprit, Some(Quantity::Floor));
    }

    /// The map declares what it was built on, so re-measuring either ruler invalidates it without
    /// anyone remembering to.
    #[test]
    fn the_map_declares_the_rulers_it_was_built_on() {
        let (xs, ys, mut zs) = grid(|_, _| 0.9190);
        for (k, z) in zs.iter_mut().enumerate() {
            *z += if k % 2 == 0 { 0.0010 } else { -0.0010 };
        }
        let m = fit(&xs, &ys, &zs, 0.01, 1, 7, 9).unwrap();
        assert_eq!(m.deps[0], Some((Quantity::ContactThreshold, 7)));
        assert_eq!(m.deps[1], Some((Quantity::StepDelivery, 9)));
    }
}
