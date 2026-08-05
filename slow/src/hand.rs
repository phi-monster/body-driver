//! "Which pixels are my hand" — measured **continuously**, not fitted once at power-on.
//!
//! # Why this file exists in this exact shape
//!
//! This is the one body quantity that is still open, and the archive is specific about *which*
//! part is open. Recognising the hand is **done**: 1.7 cm → **0.62 cm**, reproduced across three
//! independent processes, 8/8 episodes better. What is not done is keeping it during the servo:
//!
//! * *"three times the localisation reading improved markedly and the closed loop gained nothing"*
//!   — fixing the fit from 1.7 cm to 0.62 cm moved the latch **not at all**, across 32 paired
//!   layouts over two independent runs.
//! * the located cause: fit-time error **2.0 px**, but the error at *the moment the hand is
//!   closest to the target* is **4.9–14.6 px = 1.5–4.6 cm** — at or above the 2.0 cm latch radius.
//!   *"The version that fits best is the one that drifts worst."*
//! * and the whole family of "give the fit more evidence" is already refuted: repainting the robot
//!   raised usable candidate pixels **11 → 173 (15×)** and the loop stayed **0/9**. The verdict
//!   written at the time is the specification for this file: **a deployable fix must act during
//!   the servo — able to re-measure where the hand is on every step, not depending on the template
//!   taken at episode start.**
//!
//! # The trap that is designed against here, explicitly
//!
//! The old selector was *"whichever rigid thing responds most to my command is me"*. That rule was
//! derived when the competitors were **the hand and its shadow**. On a different rig the
//! competitors became **different links of the same arm**, and the elbow — nearer the camera at
//! 0.393 m versus the fingertip at 0.438 m — won the rule. The loop then aimed the elbow at the
//! mark with a self-reported error of **0.04–9.3 px** while the true error was **167 px**.
//!
//! *A selection rule derived for two candidates does not report an error when the candidate set
//! changes. It just quietly picks wrong.*
//!
//! So this estimator does **not** take a maximum. It enumerates candidates, and when they are not
//! separable it returns `None` — a refusal — instead of the best of them. Abstaining is a
//! legitimate answer and is counted separately from a wrong answer everywhere downstream.

use crate::measurement::{Measurement, Quantity, MAX_DEPS, MAX_DIM};

/// One rigid blob that responded to the commanded motion.
#[derive(Copy, Clone, Debug)]
pub struct Candidate {
    /// Centroid in normalised image coordinates.
    pub u: f64,
    /// Centroid in normalised image coordinates.
    pub v: f64,
    /// How strongly it moved with the command, in image units per command unit.
    pub gain: f64,
    /// How well a single rigid transform explains this blob, in `[0, 1]`.
    pub rigidity: f64,
    /// Pixel count. Small blobs are noise; the caller supplies its own floor.
    pub pixels: u32,
    /// How tightly this blob is localised, 1σ in normalised image units.
    ///
    /// 🔴 This is an INPUT, not something this file derives. An earlier draft manufactured it from
    /// the candidate count (`1/(1+n)`), which produced 0.33 — a third of the frame — and the unit
    /// test caught it. A number invented to fill a field is exactly the hand-filled constant this
    /// whole layer exists to abolish; the caller measured the blob, so the caller reports how well.
    pub spread: f64,
}

/// Tunables. Every one is an **observability** threshold — "can this be read off this image at
/// all" — not a physical constant and not a per-robot value. They stay identical when the body
/// changes; only the measurements change. That is the test for whether something belongs here:
/// *move the body layer to another robot and not one line should change.*
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// Below this a blob has no readable shape.
    pub min_pixels: u32,
    /// Below this a blob is not one rigid thing.
    pub min_rigidity: f64,
    /// The winner's gain must exceed the runner-up's by this factor, or the frame is refused.
    ///
    /// This is the anti-elbow rule. On the rig where it failed, fingertip and elbow depths were
    /// 0.438 m and 0.393 m — a gain ratio near **1.11**. Any separation requirement above that
    /// turns a silent mis-pick into a visible refusal.
    pub min_separation: f64,
    /// How fast uncertainty grows per step with no fresh evidence, in normalised image units.
    ///
    /// Non-zero on purpose: an estimate that does not decay is an estimate that never expires, and
    /// never expiring is precisely how the fit-time value stayed trusted while the hand drifted
    /// away from it.
    pub decay_per_step: f64,
    /// Refuse once uncertainty exceeds this.
    pub max_uncertainty: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            min_pixels: 40,
            min_rigidity: 0.60,
            min_separation: 1.50,
            decay_per_step: 0.0025,
            max_uncertainty: 0.030,
        }
    }
}

/// Why a step produced no hand point. Each is counted separately downstream — collapsing them is
/// how "no data" and "ran and got zero" end up looking alike.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Abstain {
    /// Nothing moved enough to be a candidate.
    NoCandidate,
    /// Two or more candidates were too close in gain to tell apart.
    NotSeparable,
    /// Uncertainty has grown past the limit with no fresh evidence.
    TooUncertain,
}

/// The continuous estimator. One per arm.
#[derive(Clone, Debug)]
pub struct HandTracker {
    cfg: Config,
    /// Current best point, if any.
    point: Option<(f64, f64)>,
    /// Current 1σ, in normalised image units. Grows every step without fresh evidence.
    sigma: f64,
    /// Steps since the last accepted observation. Reported, so "it has been coasting" is visible.
    steps_since_evidence: u32,
    /// Accepted observations so far this episode.
    accepted: u32,
    /// Abstentions so far this episode.
    abstained: u32,
    /// The most recent abstention reason, for the audit trail.
    last_abstain: Option<Abstain>,
}

impl HandTracker {
    /// A tracker that has not yet seen anything. It has **no** point, and asking it for one before
    /// it has evidence is refused rather than answered with a default.
    pub fn new(cfg: Config) -> Self {
        HandTracker {
            cfg,
            point: None,
            sigma: f64::INFINITY,
            steps_since_evidence: 0,
            accepted: 0,
            abstained: 0,
            last_abstain: None,
        }
    }

    /// Number of accepted observations and abstentions this episode.
    pub fn counts(&self) -> (u32, u32) {
        (self.accepted, self.abstained)
    }

    /// Most recent abstention reason, if any.
    pub fn last_abstain(&self) -> Option<Abstain> {
        self.last_abstain
    }

    /// Feed one step's candidates. Returns the current point, or the reason there is none.
    ///
    /// Called **every** control step, which is the entire difference from the estimator this
    /// replaces. `residual_px` is how far the accepted candidate sat from the previous estimate,
    /// and it is what shrinks σ — evidence, not time, is what makes the estimate confident.
    pub fn observe(&mut self, cands: &[Candidate]) -> Result<(f64, f64), Abstain> {
        // Uncertainty always grows first. Even on a step that produces good evidence, some time
        // passed and the hand moved; charging that before crediting the observation keeps the
        // arithmetic honest in the direction that costs us, never the direction that flatters us.
        self.sigma += self.cfg.decay_per_step;

        let mut usable = heapless::Vec::new();
        for c in cands {
            if c.pixels >= self.cfg.min_pixels && c.rigidity >= self.cfg.min_rigidity {
                usable.push(*c);
            }
        }

        if usable.is_empty() {
            self.steps_since_evidence += 1;
            self.abstained += 1;
            self.last_abstain = Some(Abstain::NoCandidate);
            return Err(self.report(Abstain::NoCandidate));
        }

        // Sort by gain, descending — but *not* to take the maximum. Only to compare the top two.
        usable.sort_desc_by_gain();

        let Some(best) = usable.get(0) else {
            self.steps_since_evidence += 1;
            self.abstained += 1;
            self.last_abstain = Some(Abstain::NoCandidate);
            return Err(self.report(Abstain::NoCandidate));
        };
        if let Some(second) = usable.get(1) {
            let ratio = if second.gain.abs() > 1e-12 {
                best.gain.abs() / second.gain.abs()
            } else {
                f64::INFINITY
            };
            if ratio < self.cfg.min_separation {
                // 🔴 THE ANTI-ELBOW BRANCH. Two rigid things both moved with the command and we
                // cannot tell which is the hand. The old rule would have shipped the nearer one
                // and reported a 0.04 px error. This ships a refusal.
                self.steps_since_evidence += 1;
                self.abstained += 1;
                self.last_abstain = Some(Abstain::NotSeparable);
                return Err(self.report(Abstain::NotSeparable));
            }
        }

        // Accept. σ becomes the observation's own spread, inflated by how MARGINAL the separation
        // was: a frame that only just cleared the separation test is weak evidence even though it
        // was accepted, and treating it as strong is how a confident wrong answer gets built.
        let margin = match usable.get(1) {
            Some(second) if second.gain.abs() > 1e-12 => best.gain.abs() / second.gain.abs(),
            _ => f64::INFINITY,
        };
        let penalty = if margin.is_finite() {
            1.0 + self.cfg.min_separation / margin
        } else {
            1.0
        };
        let obs_sigma = (best.spread * penalty).max(self.cfg.decay_per_step);
        self.sigma = obs_sigma.min(self.sigma);
        self.point = Some((best.u, best.v));
        self.steps_since_evidence = 0;
        self.accepted += 1;
        self.last_abstain = None;

        if self.sigma > self.cfg.max_uncertainty {
            self.abstained += 1;
            self.last_abstain = Some(Abstain::TooUncertain);
            return Err(Abstain::TooUncertain);
        }
        Ok((best.u, best.v))
    }

    fn report(&mut self, why: Abstain) -> Abstain {
        // Only a tracker that HAS a point can have an old one. Before the first accepted
        // observation, σ is infinite by construction, and reporting that as "too uncertain" would
        // merge two states that must stay apart: "I have no evidence" and "my evidence is stale".
        // The first is normal at episode start; the second means the hand drifted away from a fit
        // that is still being trusted -- the exact failure this file was written against.
        if self.point.is_some() && self.sigma > self.cfg.max_uncertainty {
            self.last_abstain = Some(Abstain::TooUncertain);
            return Abstain::TooUncertain;
        }
        why
    }

    /// Publish the current state as a [`Measurement`], so the admit gate can refuse on it like any
    /// other body quantity.
    ///
    /// `epoch` must be bumped by the caller on every re-measure; `jac_epoch` is the image
    /// Jacobian's current epoch, recorded as a dependency — if the Jacobian is re-measured, every
    /// hand point taken against the old one becomes invalid **even though its own clock is fresh**.
    pub fn publish(&self, now_ns: u64, epoch: u64, prev_epoch: u64, jac_epoch: u64) -> Option<Measurement> {
        let (u, v) = self.point?;
        if self.sigma > self.cfg.max_uncertainty {
            return None;
        }
        let mut m = Measurement {
            quantity: Quantity::HandPixel,
            dim: 2,
            value: [0.0; MAX_DIM],
            uncertainty: [0.0; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [0.0; MAX_DIM],
            measured_at_ns: now_ns,
            // Short on purpose. A hand point is valid for about as long as the hand has not moved,
            // which is one control period, not one episode. The old failure is exactly a hand point
            // that stayed trusted for 700 steps after it stopped being true.
            valid_for_ns: 50_000_000, // 50 ms
            deps: [None; MAX_DEPS],
            epoch,
            selftest_passed: true,
            prev_epoch,
        };
        m.value[0] = u;
        m.value[1] = v;
        m.uncertainty[0] = self.sigma;
        m.uncertainty[1] = self.sigma;
        // The probed range is the frame itself; anything outside it is not a pixel of this image.
        m.valid_lo[0] = 0.0;
        m.valid_hi[0] = 1.0;
        m.valid_lo[1] = 0.0;
        m.valid_hi[1] = 1.0;
        m.deps[0] = Some((Quantity::ImageJacobian, jac_epoch));
        Some(m)
    }
}

/// A tiny fixed-capacity vector, so this crate keeps its zero-dependency property and can build
/// for targets without an allocator.
mod heapless {
    use super::Candidate;

    const CAP: usize = 16;

    #[derive(Copy, Clone, Debug)]
    pub struct Vec {
        buf: [Candidate; CAP],
        len: usize,
    }

    impl Vec {
        pub fn new() -> Self {
            Vec {
                buf: [Candidate {
                    u: 0.0,
                    v: 0.0,
                    gain: 0.0,
                    rigidity: 0.0,
                    pixels: 0,
                    spread: f64::INFINITY,
                }; CAP],
                len: 0,
            }
        }
        /// Silently dropping past capacity would understate how many candidates there were, and
        /// the separation test is exactly a count-sensitive test — so a full buffer keeps the
        /// *strongest* entries rather than the first ones seen.
        pub fn push(&mut self, c: Candidate) {
            if self.len < CAP {
                self.buf[self.len] = c;
                self.len += 1;
            } else if let Some(i) = self.weakest() {
                if self.buf[i].gain.abs() < c.gain.abs() {
                    self.buf[i] = c;
                }
            }
        }
        fn weakest(&self) -> Option<usize> {
            (0..self.len).min_by(|&a, &b| {
                self.buf[a]
                    .gain
                    .abs()
                    .partial_cmp(&self.buf[b].gain.abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
        }
        pub fn is_empty(&self) -> bool {
            self.len == 0
        }
        pub fn get(&self, i: usize) -> Option<Candidate> {
            if i < self.len {
                Some(self.buf[i])
            } else {
                None
            }
        }
        pub fn sort_desc_by_gain(&mut self) {
            let n = self.len;
            for i in 1..n {
                let mut j = i;
                while j > 0 && self.buf[j - 1].gain.abs() < self.buf[j].gain.abs() {
                    self.buf.swap(j - 1, j);
                    j -= 1;
                }
            }
        }
    }
}
