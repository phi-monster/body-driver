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

use crate::measurement::{Measurement, Quantity};

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
            DependencyChanged => "dependency_changed",
            SelfTestFailed => "selftest_failed",
            UncertaintyTooHigh => "uncertainty_too_high",
            Unreachable => "unreachable",
            RateLimit => "rate_limit",
        }
    }
}

/// The verdict, with enough detail to be actionable and to be audited afterwards.
#[derive(Copy, Clone, Debug)]
pub struct Verdict {
    /// `true` if the ask may proceed.
    pub admit: bool,
    /// Why not, when `admit` is false.
    pub why: Reason,
    /// Which quantity carried the refusal, when one did.
    pub culprit: Option<Quantity>,
}

impl Verdict {
    /// An unconditional admit.
    pub const OK: Verdict = Verdict {
        admit: true,
        why: Reason::None,
        culprit: None,
    };

    /// A refusal attributed to a specific quantity.
    pub fn refuse(why: Reason, culprit: Quantity) -> Self {
        Verdict {
            admit: false,
            why,
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
    /// A point the ask must be able to reach, in normalised image coordinates, if it has one.
    pub image_point: Option<(f64, f64)>,
}

impl Ask {
    /// An ask that consults nothing. Useful as a base to fill in.
    pub const EMPTY: Ask = Ask {
        needs: [None; 6],
        tolerance: [None; 6],
        image_point: None,
    };
}

/// Decide. `get` reads the current measurement for a quantity; `None` means never measured.
///
/// The order of checks is deliberate and is the cheapest-first order that never lets a later check
/// mask an earlier one: existence, then self-test, then staleness, then dependency epochs, then
/// range, then precision. Reordering this can make one failure hide behind another — and a hidden
/// failure is exactly the class of bug this layer exists to eliminate.
pub fn admit(ask: &Ask, now_ns: u64, get: &dyn Fn(Quantity) -> Option<Measurement>) -> Verdict {
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
            if q == Quantity::HandPixel && !(m.covers(0, u) && m.covers(1, v)) {
                // Refusing rather than extrapolating. Recorded reason: a self-calibration's whole
                // residual turned out to be interpolation error between the poses it had actually
                // visited, so "outside the probed range" is precisely where its number stops
                // meaning anything.
                return Verdict::refuse(Reason::OutOfRange, q);
            }
        }

        if let Some(tol) = ask.tolerance[slot] {
            if m.worst_uncertainty() > tol {
                return Verdict::refuse(Reason::UncertaintyTooHigh, q);
            }
        }
    }
    Verdict::OK
}
