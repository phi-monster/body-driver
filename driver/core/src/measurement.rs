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
    /// 🔴 **How slippery my own fingertips are** — the friction coefficient of THIS gripper
    /// against what it is holding.
    ///
    /// Added 2026-08-17 because it was the last load-bearing number still being typed in.
    /// `contact_gen` needs it to decide whether a grasp slips and **how many contact points are
    /// enough** (owner's counterexample: a person can fire a gun with two fingers — the count is
    /// not the hand's finger count, it is whatever the friction cones can hold). It was being
    /// passed as `0.5` by the caller, i.e. by a human, on every single grasp.
    ///
    /// It needs **no force sensor**: hold something, tilt, and the angle at which it slides is
    /// `atan(mu)`. Both halves of that already exist on this body — the jaws report their own
    /// opening, and "did it slip" is the same readback that already separates a held object from
    /// a closed-on-air one.
    Friction = 14,
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
    /// 🔴 **How low this body can get, as a function of where it is.** The plane a downward motion
    /// stops on, over the region actually probed.
    ///
    /// This is the quantity that separates *"something is in the way"* from *"this arm has run out
    /// of solution"*, and it does so without joint states, without a force sensor, and without
    /// spending an extra command per decision.
    ///
    /// The reasoning is one line: **a body's limit is a property of the CONFIGURATION and moves
    /// with the arm; a surface is a property of the WORLD and stays where it is.** So measure where
    /// motion stops across a grid, once. Afterwards every stop is read against it:
    ///
    /// | where the hand stopped | what it means |
    /// |---|---|
    /// | at the floor | resting on the working surface — nothing on it here |
    /// | **above** the floor | something IS here, and it is `stop - floor` tall |
    /// | **below** the floor | there is no surface here; this is the arm's own limit |
    ///
    /// Measured cost of not having it: on a flat conveyor, nine downward probes stopped across
    /// **13.9 cm** while the four that were genuinely on the belt agreed to **2.2 cm**. The
    /// delivered-motion ruler called all nine "contact", every reading in the log was correct, and
    /// the conveyor loop closed its jaws in mid-air on all nine episodes.
    ///
    /// 🔴 **THE DOMAIN AXES ARE NOT THE VALUE AXES HERE**, and that has bitten this repo twice in
    /// one day, so it is spelled out: `value = [z0, dz/dx, dz/dy]` where `z0` is the height at the
    /// centre of the probed box, while `valid_lo/hi[0]` and `[1]` are the **x and y ranges the grid
    /// actually covered**. Axis 2 carries no domain and is marked [`AxisKind::Unmeasured`]. Asking
    /// outside the box refuses rather than extrapolating a plane over ground nobody drove on.
    Floor = 12,
    /// 这具身体的**原位** —— 每个动作结束时必须回到的那个已知且可重复的位形。
    ///
    /// # 它不是 benchmark 的怪癖,是动作可组合的前提
    ///
    /// `all_robot_back_to_origin` 出现在 RoboDojo **30/42** 个任务里且强制。但真正的理由与
    /// 判据无关:**每个动作结束时身体必须回到一个已知且可重复的位形,否则下一个动作的起点是
    /// 未知的**。真机上同样成立 —— 让开视野、让人能靠近、让下一次从同一处起步。
    ///
    /// # 为什么它是**身体量**而不是配置
    ///
    /// `LAB` 已记两条:*"home 四元数不通用"* 与 *"home 位形是资产文件写死的驱动目标"* ——
    /// 每具身体各不相同,而且是**可以由身体自己测出来的**(上电归位后读一次自己的位形)。
    /// 在此之前它住在策略里:开集时给状态拍个快照、结束时发回去。**能用,但那是土办法** ——
    /// 换一具身体、换一个策略文件,它就跟着漂,而没有任何一处会报不一致。
    ///
    /// `value = [x, y, z, qw, qx, qy, qz]`(工作点位置 + 姿态),
    /// `uncertainty` 是**重复性**:归位若干次,同一个位形自己抖多少。
    /// 🔴 那个抖动就是"回到原位没有"的容差下界 —— **比它还紧的容差是编出来的**。
    HomePose = 13,
}

impl Quantity {
    /// Total number of quantities; used to size the store.
    // 🔴🔴 **加一个量,这个数必须跟着加。**
    // 实测(2026-08-17):加了 `Friction = 14` 而没有动它 ⇒ 存储只有 14 个槽,
    // 一碰这一格就 `index out of bounds: len is 14 but the index is 14`,
    // **整轮自标定当场崩,标定一个字都没写出来**,而前十轮的日志完全正常。
    // ⇒ 谁再加一个量,这里必须跟着改;`ALL` 那张表和这个数对不上时,
    //    下面那条断言会在测试里直接失败,不会拖到线上。
    pub const COUNT: usize = 15;

    /// 全部的量,一个不落。🔴 **加一个变体必须加进这里** —— 下面那条测试会核它与
    /// `COUNT` 是否一致,于是"加了量忘了加槽"在测试里就炸,不会拖到线上崩一整轮标定。
    pub const ALL: [Quantity; Quantity::COUNT] = [
        Quantity::HandPixel,
        Quantity::ImageJacobian,
        Quantity::GripperSpan,
        Quantity::ArmWeight,
        Quantity::Latency,
        Quantity::Backlash,
        Quantity::Reach,
        Quantity::ContactThreshold,
        Quantity::SelfOcclusion,
        Quantity::StepDelivery,
        Quantity::ToolOffset,
        Quantity::ToolAxisColumn,
        Quantity::Floor,
        Quantity::HomePose,
        Quantity::Friction,
    ];


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
            12 => Floor,
            13 => HomePose,
            // 🔴 加一个变体要改**三处**:枚举 · COUNT · 这里。漏了这一处的后果是
            // `missing()` 报不出它 ⇒ 这一格**永远不会被排进上电日程**,
            // 于是它"从来没被量过"这件事本身也看不见。
            14 => Friction,
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
            HomePose => "home_pose",
            Floor => "floor",
            Friction => "friction",
        }
    }

    /// The same name, NUL-terminated for the C ABI. Beside `as_str` on purpose: a new variant that
    /// forgets a name fails to compile here rather than being published as "unknown" forever.
    pub fn name_c(self) -> &'static core::ffi::CStr {
        use Quantity::*;
        match self {
            HandPixel => c"hand_pixel",
            ImageJacobian => c"image_jacobian",
            GripperSpan => c"gripper_span",
            ArmWeight => c"arm_weight",
            Latency => c"latency",
            Backlash => c"backlash",
            Reach => c"reach",
            ContactThreshold => c"contact_threshold",
            SelfOcclusion => c"self_occlusion",
            StepDelivery => c"step_delivery",
            ToolOffset => c"tool_offset",
            ToolAxisColumn => c"tool_axis_column",
            Floor => c"floor",
            HomePose => c"home_pose",
            Friction => c"friction",
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
    /// 🔴 It would replace a stored measurement that is **better on every axis** — wider uncertainty
    /// *and* a strictly smaller probed box.
    ///
    /// Added 2026-08-13 because it happened: a floor measured over 54 cells (residual 0.26 mm, box
    /// `x[-0.60,+0.60]`) was overwritten by one collected from 9 cells (residual **7.5 mm**, box
    /// `x[-0.60,+0.30]` and a y range 0.3 m narrower). Nothing objected. The good one had to be
    /// rebuilt from the original rollouts, and the only reason that was possible is that they had
    /// not been deleted.
    ///
    /// A recalibration is legitimate when the body changed, and then it is normally *not* worse on
    /// both axes at once — a fresh probe of a moved arm still covers a comparable box. Worse
    /// uncertainty **and** less ground covered has one likely reading: this run collected less, and
    /// the stored row was the better measurement. So this refuses, and the caller that really means
    /// it clears the slot first.
    WorseThanStored,
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
            Malformed::WorseThanStored => "worse than the stored row on every axis",
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

#[cfg(test)]
mod 槽位 {
    use super::*;

    /// 🔴 **加一个量而忘了把存储加大 = 一碰那一格就越界。**
    ///
    /// 实测(2026-08-17):`Friction` 加进枚举、`COUNT` 没动 ⇒
    /// `index out of bounds: the len is 14 but the index is 14`,
    /// **整轮自标定当场崩,标定一个字没写出来**,而崩之前十轮的日志完全正常。
    /// 这条测试就是那次事故的形状:**最后一个量的编号必须落在存储里**。
    #[test]
    fn 每一个量都必须有自己的槽() {
        for q in Quantity::ALL {
            assert!(
                (q as usize) < Quantity::COUNT,
                "{:?} 的编号是 {},而存储只有 {} 个槽 —— 加量必须同时加 COUNT",
                q, q as usize, Quantity::COUNT
            );
        }
    }

    /// 反过来也要卡:`COUNT` 比实际的量多,会留下永远读不到的空槽,
    /// 而"这一格没量过"和"这一格根本不存在"读起来一样。
    #[test]
    fn 槽位数不许多于量的个数() {
        assert_eq!(Quantity::ALL.len(), Quantity::COUNT, "ALL 与 COUNT 必须一致");
    }
}
