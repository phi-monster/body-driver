//! The gate. **A layer that cannot say REFUSE is not a body layer.**
//!
//! # What this replaces
//!
//! Until now every one of these checks was a hand-welded gate inside one experiment's script.
//! That worked exactly as well as somebody remembering to weld it. Over one night this project
//! recorded **seven** cases of "the apparatus was never actually built, and every reading was
//! green" — and the gates that did fire were the ones somebody had happened to write that day.
//!
//! A REFUSE here is mechanical: it comes from comparing an ask against what this body has actually
//! established about itself, and it fires whether or not anyone remembered.
//!
//! # A REFUSE is an answer, not an error
//!
//! Callers must not retry it away, and must not fold it into "the task failed". *No data*,
//! *not applicable* and *ran and scored zero* are three different things, and a results table in
//! which they look alike is a results table that will be misread. This project has misread one.

use crate::measurement::{Coverage, Measurement, Quantity};

/// Why an ask was refused. Every variant maps to a condition that can be *measured*, never to a
/// judgement call.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Reason {
    /// Nothing was refused.
    None = 0,
    /// The quantity has no value on this body yet.
    NeverMeasured = 1,
    /// Older than its declared validity window.
    Stale = 2,
    /// The ask lies outside the range this quantity was actually probed over.
    OutOfRange = 3,
    /// Something it was measured *against* has since been re-measured.
    DependencyChanged = 4,
    /// Its own self-test does not pass right now.
    SelfTestFailed = 5,
    /// Measured, but not well enough for what is being asked.
    UncertaintyTooHigh = 6,
    /// Geometry says this body cannot get there.
    Unreachable = 7,
    /// The fast face declined: a limit, a force cap, or the watchdog.
    RateLimit = 8,
    /// 🔴 **Not now — and nothing here says never.** The ask is outside what this body can do *at
    /// this instant*, and the refusal is about the state of the WORLD rather than about the body.
    ///
    /// The caller contract is the whole point: the correct response is to let the world advance
    /// and ask again, **not** to abandon the task. Folding this into [`Reason::Unreachable`] tells
    /// a robot to give up on something that is simply still on its way.
    ///
    /// Recorded because it was learned expensively. On a conveyor task this distinction was
    /// hand-welded into an experiment script **three separate times in one night** — a look budget
    /// that ended before the object arrived, a timing gate that fired before it entered frame, and
    /// a reach gate that judged at t=0 an object that would pass within 0.416 m at step ~320 (band
    /// 0.134–0.602 m). Three symptoms, one missing concept, and each patch was invisible to the
    /// next person. It belongs here, once.
    NotYet = 9,
    /// 🔴 **I could not check** — as distinct from [`Self::SelfTestFailed`], which is *I checked
    /// and it is wrong*. Merging the two was tried and is a measured mistake
    /// (`results/bodylayer_aug2026`): the caller's response differs, re-probe versus abandon, so a
    /// single name would send half of them the wrong way. In this crate it arises when an ask
    /// touches an [`AxisKind::Unmeasured`] axis: the verdict ADMITS and carries this reason, which
    /// is the "usable but unverified" rung that a two-state admit/refuse cannot express.
    NoEvidence = 10,
}

impl Reason {
    /// Stable human-readable name. For logs and audit trails; never parsed.
    pub fn as_str(self) -> &'static str {
        use Reason::*;
        match self {
            None => "none",
            NeverMeasured => "never_measured",
            Stale => "stale",
            OutOfRange => "out_of_range",
            NoEvidence => "no_evidence",
            DependencyChanged => "dependency_changed",
            SelfTestFailed => "selftest_failed",
            UncertaintyTooHigh => "uncertainty_too_high",
            Unreachable => "unreachable",
            RateLimit => "rate_limit",
            NotYet => "not_yet",
        }
    }
}

/// The verdict, with enough detail to be actionable and to be audited afterwards.
#[derive(Copy, Clone, Debug)]
pub struct Verdict {
    /// `true` if the ask may proceed.
    pub admit: bool,
    /// 🔴 Admitted, but an axis the ask touched was never probed — the third rung.
    ///
    /// A caller that reads only `admit` behaves exactly as before, which is deliberate: the
    /// alternative was a hard refusal, and that was measured to collapse everything downstream of
    /// one unprobed constant into unusable. But a caller that ignores this is proceeding on a
    /// number nothing has verified over the range it is using, and `why` says so.
    pub unverified: bool,
    /// Why not, when `admit` is false.
    pub why: Reason,
    /// Which quantity carried the refusal, when one did.
    pub culprit: Option<Quantity>,
}

impl Verdict {
    /// An unconditional admit.
    pub const OK: Verdict = Verdict {
        admit: true,
        unverified: false,
        why: Reason::None,
        culprit: None,
    };

    /// A refusal attributed to a specific quantity.
    pub fn refuse(why: Reason, culprit: Quantity) -> Self {
        Verdict {
            admit: false,
            unverified: false,
            why,
            culprit: Some(culprit),
        }
    }

    /// Admitted on a quantity with an axis nobody probed: proceed, but nothing verified this.
    pub fn unverified(culprit: Quantity) -> Self {
        Verdict {
            admit: true,
            unverified: true,
            why: Reason::NoEvidence,
            culprit: Some(culprit),
        }
    }
}

/// What an ask needs from the body, expressed **without naming any body parameter**.
///
/// 🔴 Note what is *not* here: no joint angles, no link lengths, no camera matrix, no gripper span.
/// The caller states what it wants to *do*; translating that into this body's numbers is this
/// layer's job, and keeping that translation on this side of the line is the whole architecture.
#[derive(Copy, Clone, Debug)]
pub struct Ask {
    /// Which quantities this ask will consult.
    pub needs: [Option<Quantity>; 6],
    /// Best precision the ask can tolerate, in that quantity's own units. `None` = no requirement.
    pub tolerance: [Option<f64>; 6],
    /// 🔴 Where in the corresponding quantity's **probed domain** this ask sits — a commanded step
    /// magnitude for `step_delivery`, a commanded opening for `gripper_span`, a joint angle for
    /// `arm_weight`, a pose for `self_occlusion`. Checked against `valid_lo/hi[0]`; outside it the
    /// ask is [`Reason::OutOfRange`].
    ///
    /// This field exists because the refusal it implements was **documented and absent**. Three
    /// probes state in their own docs that an ask outside the range they probed is refused rather
    /// than extrapolated, and until this field there was no mechanism by which that could happen —
    /// only `hand_pixel` was ever range-checked, through `image_point`. A promise in a docstring
    /// that no code keeps is this repository's most expensive recurring bug: a module docstring
    /// once advertised a `--ref` positive control the argument parser never implemented, and
    /// everyone who read it, including its author, believed the control had run for weeks.
    pub at: [Option<f64>; 6],
    /// A point the ask must be able to reach, in normalised image coordinates, if it has one.
    pub image_point: Option<(f64, f64)>,
    /// 🔴 How far from this arm's own measured base the ask wants to act, in metres.
    ///
    /// Checked against the measured `reach` band. Until this field existed **the band was measured
    /// and never consulted** — `Reason::Unreachable` was declared here and produced by nothing, so
    /// every caller that needed a reach check hand-welded one, which is exactly the failure mode
    /// this module's header describes.
    pub reach_radius_m: Option<f64>,
}

impl Ask {
    /// An ask that consults nothing. Useful as a base to fill in.
    pub const EMPTY: Ask = Ask {
        needs: [None; 6],
        tolerance: [None; 6],
        at: [None; 6],
        image_point: None,
        reach_radius_m: None,
    };
}

/// Decide. `get` reads the current measurement for a quantity; `None` means never measured.
///
/// The order of checks is deliberate and is the cheapest-first order that never lets a later check
/// mask an earlier one: existence, then self-test, then staleness, then dependency epochs, then
/// range, then precision. Reordering this can make one failure hide behind another — and a hidden
/// failure is exactly the class of bug this layer exists to eliminate.
pub fn admit(ask: &Ask, now_ns: u64, get: &dyn Fn(Quantity) -> Option<Measurement>) -> Verdict {
    // Set when some axis the ask touched was never probed. A refusal found later still wins: a
    // hard "no" outranks "nobody checked".
    let mut unverified: Option<Quantity> = None;
    for (slot, q) in ask.needs.iter().enumerate() {
        let Some(q) = *q else { continue };

        let Some(m) = get(q) else {
            return Verdict::refuse(Reason::NeverMeasured, q);
        };

        if !m.selftest_passed {
            return Verdict::refuse(Reason::SelfTestFailed, q);
        }

        if m.is_stale(now_ns) {
            return Verdict::refuse(Reason::Stale, q);
        }

        // A quantity is invalid the moment something it was measured AGAINST has moved, even if
        // its own clock says it is fresh. This is the case a wall-clock TTL cannot catch: the
        // camera got knocked, the gripper was swapped, the arm picked up a payload. A hand-written
        // `"my maximum payload is 500 grams"` does not know the arm is sagging today, and nothing
        // in that system will ever notice.
        for dep in m.deps.iter().flatten() {
            let (dq, epoch_at_measure) = *dep;
            match get(dq) {
                None => return Verdict::refuse(Reason::DependencyChanged, dq),
                Some(dm) if dm.epoch != epoch_at_measure => {
                    return Verdict::refuse(Reason::DependencyChanged, dq)
                }
                Some(_) => {}
            }
        }

        if let Some((u, v)) = ask.image_point {
            if q == Quantity::HandPixel
                && [m.covers(0, u), m.covers(1, v)].contains(&Coverage::Unknown)
            {
                unverified = Some(q);
            } else if q == Quantity::HandPixel
                && !(m.covers(0, u) == Coverage::Inside && m.covers(1, v) == Coverage::Inside)
            {
                // Refusing rather than extrapolating. Recorded reason: a self-calibration's whole
                // residual turned out to be interpolation error between the poses it had actually
                // visited, so "outside the probed range" is precisely where its number stops
                // meaning anything.
                return Verdict::refuse(Reason::OutOfRange, q);
            }
        }

        if let Some(x) = ask.at[slot] {
            if m.covers(0, x) == Coverage::Unknown {
                // No evidence either way. Refusing here was tried and collapsed the scale; see
                // `AxisKind::Unmeasured`. Admit, and carry the fact.
                unverified = Some(q);
            } else if m.covers(0, x) == Coverage::Outside {
                // The general form of the refusal above: this quantity was established over a
                // domain, and the ask is outside it. `step_delivery` genuinely differs between a
                // 1 mm and a 45 mm command; `gripper_span` swept from 0.4 to 0.8 says nothing about
                // full open. Extrapolating either produces a number that is believed.
                return Verdict::refuse(Reason::OutOfRange, q);
            }
        }

        if let (Quantity::Reach, Some(r)) = (q, ask.reach_radius_m) {
            let (lo, hi) = (m.value[0], m.value[1]);
            if r < lo || r > hi {
                // 🔴 NotYet, NOT Unreachable — and the difference is a claim about the future that
                // this layer cannot make. The band says where this body can act; it says nothing
                // about whether the world will bring the ask inside it a moment from now. Claiming
                // "never" from a measurement that only establishes "not at this radius" is exactly
                // the over-claim this layer exists to refuse. `Unreachable` stays reserved for
                // statements that are actually established.
                return Verdict::refuse(Reason::NotYet, q);
            }
        }

        if let Some(tol) = ask.tolerance[slot] {
            if m.worst_uncertainty() > tol {
                return Verdict::refuse(Reason::UncertaintyTooHigh, q);
            }
        }
    }
    match unverified {
        Some(q) => Verdict::unverified(q),
        None => Verdict::OK,
    }
}
