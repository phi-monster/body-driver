//! A measured body quantity, with everything needed to refuse on it later.
//!
//! # Why seven fields and not one number
//!
//! The failure mode this layer exists to eliminate is not "the number is wrong". It is **the
//! number is wrong and nothing in the loop can tell**. Recorded instances, all from this project:
//!
//! * the servo drove `|target − self-estimated hand|` to zero, so the bias in the self-estimate
//!   landed entirely in the *result* and never in the *residual*. Its own visible error read
//!   3.6 px while the true offset was 15.7 px. Algebra, not precision.
//! * on another rig the same estimator settled on the robot's **elbow**, 167 px from the true
//!   fingertip, and reported 0.04–9.3 px. It precisely aimed the wrong point at the mark.
//! * a hand-filled gripper constant, `0.145`. ⚠️ **Corrected 2026-08-09 by reading the deployed
//!   source, and the truth is worse than the earlier note here** (which said its provenance could
//!   not be traced at all). It is traceable, to a line a person writes: it is the tool-axis offset,
//!   copied by hand out of `Assets/Robots/<body>/robot_config.yml`, and the comment beside it in the
//!   running servo reads *"x5 = 0.145, franka = 0.102"* — **4.3 cm apart between two bodies, with
//!   0.145 as the DEFAULT.** A new machine that does not remember to pass it does not fail; it
//!   quietly executes with another robot's geometry. Traceable to a hand-written per-body file is
//!   the same violation as untraceable, and it degrades more quietly. It is now
//!   [`Quantity::ToolOffset`], with a probe.
//!
//! A bare `f64` cannot be refused on. A value that carries its uncertainty, the range it was
//! actually probed over, when it was taken, what it was taken *against*, and a self-test that can
//! be re-run right now — can.
//!
//! # The rule about `valid_lo/hi`
//!
//! They record the **domain actually probed** — not the range someone hopes the value extrapolates
//! to, and 🔴 **not the range of the value itself**. Those are different quantities, usually in
//! different units:
//!
//! | quantity | `value` is | `valid_lo/hi` is |
//! |---|---|---|
//! | image Jacobian | image units per command unit | the **commands** actually issued |
//! | arm weight | torque to hold | the **joint angles** actually visited |
//! | hand pixel | a pixel | the frame — the same units, by coincidence |
//!
//! An earlier draft rejected any measurement whose `value` fell outside its own `valid_lo/hi`.
//! That check reads as obviously correct and is obviously wrong: it is a units error that happens
//! to be satisfied by the one quantity where domain and value share units. The end-to-end test
//! caught it the first time a real Jacobian was submitted.
//!
//! The distinction has been paid for on the other side too: a gravity self-calibration's residual
//! turned out to be *entirely* interpolation error between the poses it had sampled, so the honest
//! statement of its validity is "the poses I visited", and asking outside them must produce a
//! REFUSE rather than a confident extrapolation.

use core::fmt;

/// The body properties this layer knows how to hold.
///
/// These are things a robot can determine **by acting on itself**. They are deliberately not "the
/// fields a URDF happens to contain": a URDF is written by a person, and a hand-written body
/// constant is the same violation as a hand-fed demonstration — somebody touched the new machine.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Quantity {
    /// Which pixels are my hand, in the frame I can see.
    HandPixel = 0,
    /// I move by δ → the image moves by this.
    ImageJacobian = 1,
    /// Full-open to full-closed, in metres, measured off my own jaws.
    GripperSpan = 2,
    /// What holding still against gravity costs.
    ArmWeight = 3,
    /// Command issued → pixels move, in control periods.
    Latency = 4,
    /// Push both ways; the dead band is the slop.
    Backlash = 5,
    /// Where this body can actually put its hand.
    Reach = 6,
    /// What "I touched something" reads like on this body.
    ContactThreshold = 7,
    /// Which parts of my own view I block.
    SelfOcclusion = 8,
    /// I command a step of this size; this fraction of it actually arrives in one control period.
    ///
    /// 🔴 Added 2026-08-09 from a measurement, not from a design meeting. Two arms on the same
    /// harness, same waypoint controller, same step command of 45 mm: one delivered **0.76** of it
    /// per period, the other **0.11**. The step budget per waypoint had been set from the first
    /// arm, so the second could never reach any waypoint — 0.136 m of residual, every episode,
    /// while every scalar in the log looked ordinary.
    ///
    /// It is **not** [`Latency`]: that one is dead time, "how many periods until anything moves",
    /// and both arms answered 1. A body can start moving immediately and still deliver a tenth of
    /// what it was told.
    ///
    /// It is **not** [`Backlash`] either: backlash is a dead band around a reversal, this is a
    /// first-order shortfall that applies to every step in the same direction.
    StepDelivery = 9,
    /// How far my working point sits from the mount I command, along the tool axis, in metres.
    ///
    /// 🔴 Added 2026-08-09 from a **census of the live stack**, not from a design meeting. The same
    /// number is typed in at four places in the deployed system, with three different values for
    /// three bodies:
    ///
    /// * `L3_GRIPPER_BIAS`, default `0.145` — and the comment beside it reads
    ///   *"x5 = 0.145, franka = 0.102 (4.3 cm apart)"*, with the value to be copied by hand from
    ///   `Assets/Robots/<body>/robot_config.yml`. **A body that forgets to pass it silently runs on
    ///   a different robot's number.** That is the quiet degradation this whole layer exists to
    ///   make impossible, live, in production.
    /// * the same `0.145` again, hardcoded in the teacher's `flange_for()`: `tcp = flange + 0.145·R`.
    /// * the same `0.145` a third time, as the justification for a wrist-tilt ceiling — *"the flange
    ///   sits 0.145 m back along the tool axis, so a 60° approach demands 0.126 m of extra lateral
    ///   travel"*.
    /// * and `tcp_off: 0.1034` on a third rig's harness.
    ///
    /// It is measurable by acting on itself: turn the wrist and the working point sweeps an arc
    /// whose radius **is** the offset. See [`crate::probe::tool_offset`].
    ToolOffset = 10,
    /// Which column of my orientation matrix points along my tool, as an index in `{0,1,2}`.
    ///
    /// 🔴 Added 2026-08-11 by MERGING THE SECOND IMPLEMENTATION, not by design. This layer had a C
    /// ABI, 45 tests and no callers; a 224-line Python file with no tests was what every running
    /// script on the box actually used. Collapsing them surfaced this immediately: the Python store
    /// held a twelfth quantity this enum had never heard of, so the merged reader refused a
    /// constant that had been correctly measured for days.
    ///
    /// It travels with [`Self::ToolOffset`] and comes out of the same motion: spin about each
    /// column of R and watch the working point — the column it barely moves about IS the tool axis,
    /// and the arc radius about either of the others is the offset. Two numbers, one probe.
    ///
    /// It is on this list for the same reason as the offset: it was typed in per body
    /// (`L3_TOOL_COL`, `0` for one arm and `2` for another) and a body that forgets to pass it runs
    /// with another robot's tool axis without failing.
    ///
    /// `value[0]` is the column index; `valid_lo/hi` is `[0, 2]`, the columns actually spun about —
    /// here, unusually, the domain and the value do share units, as they do for `HandPixel`.
    ToolAxisColumn = 11,
}

impl Quantity {
    /// Total number of quantities; used to size the store.
    pub const COUNT: usize = 12;

    /// Reconstruct from the ABI's `u32`. Returns `None` for anything unknown — an unknown
    /// quantity is refused, never coerced into a neighbouring one.
    pub fn from_u32(v: u32) -> Option<Self> {
        use Quantity::*;
        Some(match v {
            0 => HandPixel,
            1 => ImageJacobian,
            2 => GripperSpan,
            3 => ArmWeight,
            4 => Latency,
            5 => Backlash,
            6 => Reach,
            7 => ContactThreshold,
            8 => SelfOcclusion,
            9 => StepDelivery,
            10 => ToolOffset,
            11 => ToolAxisColumn,
            _ => return None,
        })
    }

    /// Stable human-readable name. For logs and audit trails; never parsed.
    pub fn as_str(self) -> &'static str {
        use Quantity::*;
        match self {
            HandPixel => "hand_pixel",
            ImageJacobian => "image_jacobian",
            GripperSpan => "gripper_span",
            ArmWeight => "arm_weight",
            Latency => "latency",
            Backlash => "backlash",
            Reach => "reach",
            ContactThreshold => "contact_threshold",
            SelfOcclusion => "self_occlusion",
            StepDelivery => "step_delivery",
            ToolOffset => "tool_offset",
            ToolAxisColumn => "tool_axis_column",
        }
    }
}

/// Maximum dimensionality of a measurement.
///
/// 🔴 Sized as `3 × MAX_JOINTS`, and that is not slack — it is the smallest value that fits the
/// image Jacobian, which is 3 world axes by one column per joint. An earlier draft used 16 "because
/// a body quantity is a small vector", and a unit test caught it immediately: a perfectly ordinary
/// 6-joint arm needs 18. The lesson is the one this layer keeps re-learning — **a constant chosen
/// because it felt about right is a constant nobody measured**, and the fix is to derive it from
/// the thing it has to hold.
///
/// Anything that wants to be an *array of poses* is still not a body quantity and does not belong
/// here; the bound exists to keep that true while actually fitting a Jacobian.
pub const MAX_DIM: usize = 48;
/// Maximum number of quantities one measurement may declare a dependency on.
pub const MAX_DEPS: usize = 8;

/// Why a submitted measurement was rejected. Rejection happens at `submit` time, so a malformed
/// measurement never enters the store and can never be read back as if it were fine.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Malformed {
    /// `dim` is 0 or above [`MAX_DIM`].
    BadDim,
    /// A value, uncertainty or bound is NaN or infinite.
    NonFinite,
    /// Uncertainty is negative.
    NegativeUncertainty,
    /// `valid_lo >= valid_hi` on some axis: an empty validity window.
    EmptyRange,
    /// It declares a dependency on a quantity that has never been measured on this body.
    UnmeasuredDependency,
    /// Its own self-test did not pass at submission time.
    SelfTestFailed,
    /// More dependencies than [`MAX_DEPS`].
    TooManyDeps,
}

impl fmt::Display for Malformed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Malformed::BadDim => "dim out of range",
            Malformed::NonFinite => "non-finite value/uncertainty/bound",
            Malformed::NegativeUncertainty => "negative uncertainty",
            Malformed::EmptyRange => "empty validity range (lo >= hi)",
            Malformed::UnmeasuredDependency => "depends on a quantity never measured here",
            Malformed::SelfTestFailed => "self-test did not pass",
            Malformed::TooManyDeps => "too many dependencies",
        };
        f.write_str(s)
    }
}

/// What kind of thing an axis of `valid_lo/hi` is.
///
/// 🔴 Added 2026-08-11 by absorbing the second schema, and BOTH non-default kinds come from a
/// measured failure rather than from a design meeting (`results/bodylayer_aug2026`):
///
/// * `Categorical` — encoding a label as `Interval(0, 1)` **silently accepts 0.5**. Two real
///   artefacts hit this at once (`arm_id`, `body`). Live example in this repo: `tool_axis_column`
///   is a column index in `{0,1,2}` and its stored domain is `[0, 2]`; as an interval, "spin about
///   column 0.5" is admitted.
/// * `Unmeasured` — an axis nobody probed. The obvious handling, hard-refuse, was tried and is
///   wrong: backfilling a constant whose domain was entirely unmeasured made it unusable, which
///   made everything downstream of it unusable, which collapsed a three-level trust scale back to
///   two. The measured rule is asymmetric and this enum exists to express it: **crossing a probed
///   axis is a hard refusal — you hold positive evidence the ask is outside; touching an unprobed
///   axis is soft — you hold no evidence at all.**
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
#[repr(u32)]
pub enum AxisKind {
    /// Probed continuously between `valid_lo` and `valid_hi`. The default, and what every
    /// measurement taken before this enum existed is.
    #[default]
    Interval = 0,
    /// Probed only at the integer labels in `[valid_lo, valid_hi]`; between them there is nothing.
    Categorical = 1,
    /// Never probed on this axis. An ask that touches it is admitted **unverified**, not refused.
    Unmeasured = 2,
}

/// The answer to "was this ask inside what I actually probed".
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Coverage {
    /// Probed there.
    Inside,
    /// Probed, and the ask is outside it — positive evidence against.
    Outside,
    /// Never probed on this axis — no evidence either way.
    Unknown,
}

/// One measured body quantity, with its provenance.
#[derive(Copy, Clone, Debug)]
pub struct Measurement {
    /// Which quantity this is.
    pub quantity: Quantity,
    /// Used length of the fixed-size arrays below.
    pub dim: usize,
    /// What each axis of `valid_lo/hi` means. Defaults to [`AxisKind::Interval`] everywhere, which
    /// is exactly the behaviour that existed before this field.
    pub axis_kind: [AxisKind; MAX_DIM],
    /// The measured value.
    pub value: [f64; MAX_DIM],
    /// 1σ, in the **same units** as `value`.
    pub uncertainty: [f64; MAX_DIM],
    /// Low end of the range actually probed.
    pub valid_lo: [f64; MAX_DIM],
    /// High end of the range actually probed.
    pub valid_hi: [f64; MAX_DIM],
    /// Monotonic timestamp, nanoseconds, from the caller's clock.
    pub measured_at_ns: u64,
    /// How long this stays valid. `0` means "until a dependency changes" — **not** "forever".
    pub valid_for_ns: u64,
    /// Quantities this was measured *against*.
    pub deps: [Option<(Quantity, u64)>; MAX_DEPS],
    /// Bumped on every re-measure. Consumers compare it to detect that the ground moved.
    pub epoch: u64,
    /// Whether its own self-test passed at submission.
    pub selftest_passed: bool,
    /// The epoch this measurement replaced; `0` if it is the first.
    pub prev_epoch: u64,
}

impl Measurement {
    /// Validate everything that can be checked without looking at the rest of the store.
    ///
    /// `is_measured` answers "has this quantity ever been measured on this body", so that a
    /// declared dependency on something that does not exist is caught here rather than surfacing
    /// later as a confident answer computed against nothing.
    pub fn validate(&self, is_measured: &dyn Fn(Quantity) -> bool) -> Result<(), Malformed> {
        if self.dim == 0 || self.dim > MAX_DIM {
            return Err(Malformed::BadDim);
        }
        if !self.selftest_passed {
            // 🔴 There is no `force` flag anywhere in this API. A body layer that can be told to
            // accept an unverified constant is a configuration file with extra steps.
            return Err(Malformed::SelfTestFailed);
        }
        for i in 0..self.dim {
            let (v, u, lo, hi) = (
                self.value[i],
                self.uncertainty[i],
                self.valid_lo[i],
                self.valid_hi[i],
            );
            if !v.is_finite() || !u.is_finite() || !lo.is_finite() || !hi.is_finite() {
                return Err(Malformed::NonFinite);
            }
            if u < 0.0 {
                return Err(Malformed::NegativeUncertainty);
            }
            match self.axis_kind[i] {
                // Nothing was probed on this axis, so there is no range to be empty. Demanding
                // `lo < hi` here would force whoever stores it to invent a domain, which is the
                // hand-filled constant this layer exists to abolish, wearing a bounds field.
                AxisKind::Unmeasured => {}
                // Labels: `lo == hi` is a legitimate one-label axis, and a fractional bound means
                // somebody is encoding a number as a category or the other way round.
                AxisKind::Categorical => {
                    if lo > hi
                        || (lo - lo.round()).abs() > 1e-9
                        || (hi - hi.round()).abs() > 1e-9
                    {
                        return Err(Malformed::EmptyRange);
                    }
                }
                AxisKind::Interval => {
                    if lo >= hi {
                        return Err(Malformed::EmptyRange);
                    }
                }
            }
            // 🔴 Deliberately NOT `lo <= v <= hi`. `valid_lo/hi` is the DOMAIN this quantity was
            // probed over, and `value` is the quantity itself -- usually different units. See the
            // table in the module docs.
            let _ = v;
        }
        let n_deps = self.deps.iter().filter(|d| d.is_some()).count();
        if n_deps > MAX_DEPS {
            return Err(Malformed::TooManyDeps);
        }
        for d in self.deps.iter().flatten() {
            if !is_measured(d.0) {
                return Err(Malformed::UnmeasuredDependency);
            }
        }
        Ok(())
    }

    /// Has this expired by wall-clock at `now_ns`?
    ///
    /// `valid_for_ns == 0` is **not** "never expires" — it means expiry is governed by dependency
    /// epochs instead of by the clock. The two mechanisms are separate on purpose: a quantity can
    /// be perfectly fresh in time and still invalid because the thing it was measured against
    /// moved (the camera got knocked, the gripper was swapped, the arm now carries a payload).
    pub fn is_stale(&self, now_ns: u64) -> bool {
        if self.valid_for_ns == 0 {
            return false;
        }
        now_ns.saturating_sub(self.measured_at_ns) > self.valid_for_ns
    }

    /// Is `x` inside the range this quantity was actually probed over, on axis `axis`?
    ///
    /// 🔴 Three answers, not two — see [`Coverage`]. "I probed there" / "I probed elsewhere" /
    /// "I never probed this axis at all" are three different facts, and collapsing the third into
    /// either of the others was **measured** to be wrong (`bodylayer_aug2026`).
    pub fn covers(&self, axis: usize, x: f64) -> Coverage {
        if axis >= self.dim {
            return Coverage::Outside;
        }
        let (lo, hi) = (self.valid_lo[axis], self.valid_hi[axis]);
        match self.axis_kind[axis] {
            AxisKind::Unmeasured => Coverage::Unknown,
            AxisKind::Interval => {
                if x >= lo && x <= hi {
                    Coverage::Inside
                } else {
                    Coverage::Outside
                }
            }
            AxisKind::Categorical => {
                // Only the labels themselves were probed. The space between two of them was not
                // visited and does not exist as a value of this quantity.
                if x >= lo && x <= hi && (x - x.round()).abs() <= 1e-9 {
                    Coverage::Inside
                } else {
                    Coverage::Outside
                }
            }
        }
    }

    /// Largest 1σ across the used axes. Used by the admit gate to refuse an ask that needs more
    /// precision than this body has actually established about itself.
    pub fn worst_uncertainty(&self) -> f64 {
        self.uncertainty[..self.dim]
            .iter()
            .copied()
            .fold(0.0_f64, f64::max)
    }
}
