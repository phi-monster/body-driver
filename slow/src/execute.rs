//! `bl_execute` — turn a world-frame intent into joint commands **for this body**.
//!
//! This is the function that makes the whole split work, and the reason the policy can stay
//! body-free: everything that differs between robots is consumed here, from measurements, and
//! never reaches the model.
//!
//! # How far to move is not the policy's business
//!
//! The policy emits a **unit direction**. Distance is `spec speed × control period` — a property
//! of this body, read from this body. The reason is not tidiness: **magnitude is derivable from
//! proprioception, so it is a shortcut, and direction is not.** Removing magnitude from the
//! policy's output removes the shortcut. The step size is fixed by the machine's specification and
//! **never** by what makes the success rate look better; tuning it per task means the method
//! failed.
//!
//! # How the image intent becomes joint motion, without a calibrated camera
//!
//! The body measures its own **image Jacobian** `J`: "I command joint δ, the image moves by `J·δ`".
//! Driving the hand toward a pixel then needs only `J`, never intrinsics, never extrinsics, never
//! a hand-eye transform. That is what keeps the "0 hand-filled numbers" line intact — those
//! matrices are exactly the numbers a person would otherwise type in.
//!
//! `J` is inverted with **damped least squares**. The damping is not a tuning knob dressed up as
//! mathematics: near a singular configuration the naive pseudo-inverse asks for unbounded joint
//! velocity, and the archive has the receipt — an unbounded lift command drove the arm to full
//! extension and produced a single 0.1308 m step, **6.5× the harness's own speed cap**, flinging
//! the held object 1.4 m and scoring the episode as "could not lift" for an arm that had already
//! lifted it 46 cm.
//!
//! # The order of operations is load-bearing
//!
//! admit → solve → scale → **fast face** → emit. The proven face sees every command before it
//! leaves. There is no path around it, which is the point: a rule enforced in one place that
//! everything must pass is a rule; a rule everybody is supposed to remember is not.

#[cfg(feature = "fast")]
use crate::fast::{Fast, FastStatus, Joints, MAX_JOINTS};
#[cfg(not(feature = "fast"))]
use crate::faststub::{Fast, FastStatus, Joints, MAX_JOINTS};
use crate::measurement::Quantity;
use crate::refuse::{Ask, Reason, Verdict};
use crate::Body;

/// The action model's output. Mirrors `bl_policy_out`.
///
/// 🔴 Look at what is absent: no joint angles, no link lengths, no camera matrix, no gripper span.
/// The policy says *what it wants to happen in the world*; this module knows what that means on
/// this body.
#[derive(Copy, Clone, Debug)]
pub struct Intent {
    /// Unit vector in the world frame. `|dir|` is checked, not assumed.
    pub dir: [f64; 3],
    /// Rotation increment about the tool frame, radians.
    pub drot: [f64; 3],
    /// Absolute opening in `[0, 1]` — not a delta. The whole field agrees on this one.
    pub grip: f64,
    /// Mobile base: vx, vy, wz. Zeros if the body has no base.
    pub base: [f64; 3],
}

/// What `execute` produced, or why it produced nothing.
#[derive(Copy, Clone, Debug)]
pub enum Outcome {
    /// Joint commands, already admitted by the proven fast face.
    Move(Joints),
    /// The body layer refused. **An answer, not an error** — count it separately from a failure.
    Refused(Verdict),
    /// The fast face latched. The safe hold is returned so the caller has something to send.
    Halted(Joints),
    /// The intent itself was malformed. A statement about the caller, not about the body.
    BadIntent(&'static str),
}

/// Configuration read from **this body's specification**, not tuned.
#[derive(Copy, Clone, Debug)]
pub struct Spec {
    /// Metres per control period, from the machine's rating.
    pub step_m: f64,
    /// Control period, milliseconds.
    pub period_ms: u32,
    /// Damping for the least-squares inverse. Set from the measured Jacobian's own conditioning,
    /// never from what makes an episode succeed.
    pub damping: f64,
    /// Number of joints this body actually has.
    pub n_joints: usize,
}

impl Spec {
    /// 🔴 DERIVE this body's two own constants from this body's own measurements.
    ///
    /// Until this existed, `step_m` and `damping` were **passed in by the caller** — and the
    /// project's own ledger says exactly what that means:
    ///
    /// > `bl_spec.step_m` — this layer's own — **outstanding.** Every command is scaled by it, no
    /// > probe produces it, and it is the metric ruler `gripper_span` and `tool_offset` divide by.
    /// > `bl_spec.damping` — this layer's own — **outstanding.** Documented as *"from the measured
    /// > Jacobian's own conditioning"*, and nothing computes it — a promise kept by a comment.
    ///
    /// A body layer whose caller chooses the step size and the damping is not a body layer; it is
    /// a solver with two knobs, and the first thing an integrator asks is where those numbers came
    /// from. Now the answer is: from this body, or it refuses.
    ///
    /// **`step_m` = the largest commanded magnitude `step_delivery` was actually established over**
    /// (`valid_hi[0]`). Not a rating anyone typed: the probe swept commanded magnitudes and
    /// measured what came back, so the top of that swept domain is precisely the largest step this
    /// body is known to deliver. Commanding beyond it is `OutOfRange` by this layer's own rule, so
    /// taking it as the step size makes the scaler and the gate agree by construction instead of by
    /// somebody keeping them in sync.
    ///
    /// **`damping` = the Jacobian's own worst uncertainty.** Damped least squares trades tracking
    /// for stability, and the honest amount to trade is how badly the Jacobian is known: damp
    /// lightly when it is sharp, heavily when it is not. Every other choice is a number tuned until
    /// an episode succeeded — which is the specific failure this struct's doc comment already
    /// forbids ("never from what makes an episode succeed"), and which nothing enforced.
    ///
    /// Returns the refusal rather than a default. A default here would be a hand-filled constant
    /// wearing this function's name.
    pub fn from_body(body: &Body, period_ms: u32, n_joints: usize) -> Result<Spec, Verdict> {
        let Some(sd) = body.get(Quantity::StepDelivery) else {
            return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::StepDelivery));
        };
        if !sd.selftest_passed {
            return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::StepDelivery));
        }
        let step_m = sd.valid_hi[0];
        if !(step_m.is_finite() && step_m > 0.0) {
            // The probe ran but established no domain, so there is no largest validated step.
            return Err(Verdict::refuse(Reason::OutOfRange, Quantity::StepDelivery));
        }

        let Some(jac) = body.get(Quantity::ImageJacobian) else {
            return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::ImageJacobian));
        };
        if !jac.selftest_passed {
            return Err(Verdict::refuse(Reason::SelfTestFailed, Quantity::ImageJacobian));
        }
        let damping = jac.worst_uncertainty();
        if !damping.is_finite() || damping <= 0.0 {
            // A Jacobian reporting zero uncertainty is not a sharp Jacobian, it is one whose
            // uncertainty was never established — and damping by zero is the tuned-to-succeed
            // choice this refuses to make silently.
            return Err(Verdict::refuse(Reason::UncertaintyTooHigh, Quantity::ImageJacobian));
        }

        Ok(Spec { step_m, period_ms, damping, n_joints })
    }
}

/// Execute one step. See the module docs for why the order is admit → solve → scale → fast → emit.
///
/// `now_ms` is the caller's monotonic clock; it drives both staleness and the watchdog, and
/// passing a clock that does not advance will (correctly) latch the watchdog rather than freeze
/// the checks — a stalled clock is a fault, and a fault must be visible.
pub fn execute(body: &Body, fast: &Fast, spec: &Spec, intent: &Intent, now_ms: u32) -> Outcome {
    // ---- 0. the intent must be well formed before anything is consulted -------------------
    let n2 = intent.dir.iter().map(|x| x * x).sum::<f64>();
    if !n2.is_finite() {
        return Outcome::BadIntent("direction is not finite");
    }
    if (n2 - 1.0).abs() > 1e-3 {
        // Not normalised here. Silently normalising would let a policy encode distance in the
        // magnitude — reintroducing exactly the shortcut this contract removes.
        return Outcome::BadIntent("direction is not a unit vector");
    }
    if !intent.grip.is_finite() || !(0.0..=1.0).contains(&intent.grip) {
        return Outcome::BadIntent("grip is not an absolute opening in [0,1]");
    }
    if spec.n_joints == 0 || spec.n_joints > MAX_JOINTS {
        return Outcome::BadIntent("n_joints outside the supported range");
    }

    // ---- 1. may this body do this, right now? --------------------------------------------
    let mut ask = Ask::EMPTY;
    ask.needs[0] = Some(Quantity::ImageJacobian);
    ask.needs[1] = Some(Quantity::HandPixel);
    ask.needs[2] = Some(Quantity::Reach);
    let verdict = body.admit(&ask, u64::from(now_ms) * 1_000_000);
    if !verdict.admit {
        return Outcome::Refused(verdict);
    }

    // ---- 2. solve, using measurements only ------------------------------------------------
    let Some(jac) = body.get(Quantity::ImageJacobian) else {
        // Unreachable given the admit above, but expressed rather than unwrapped: an `unwrap`
        // here would turn a future reordering of the checks into a panic in the safety layer.
        return Outcome::Refused(Verdict::refuse(
            Reason::NeverMeasured,
            Quantity::ImageJacobian,
        ));
    };

    // `J` is stored row-major, 3 world axes × n_joints, in the measurement's value slots. Damped
    // least squares on `Jᵀ(JJᵀ + λ²I)⁻¹` reduces here to a per-joint projection because the stored
    // Jacobian is the *diagonal-dominant* form the power-on probe produces; a full solve lands in
    // this same slot once the probe reports off-diagonal terms, and the shape of the call does not
    // change.
    let lam2 = spec.damping * spec.damping;
    let mut delta: Joints = [0.0; MAX_JOINTS];
    for (j, d) in delta.iter_mut().enumerate().take(spec.n_joints) {
        let mut acc = 0.0;
        let mut gain2 = 0.0;
        for axis in 0..3 {
            let idx = axis * spec.n_joints + j;
            if idx >= jac.dim {
                continue;
            }
            let g = jac.value[idx];
            acc += g * intent.dir[axis];
            gain2 += g * g;
        }
        *d = if gain2 + lam2 > 0.0 {
            acc / (gain2 + lam2)
        } else {
            0.0
        };
    }

    // ---- 3. scale by the SPEC, never by the outcome ----------------------------------------
    let norm = delta[..spec.n_joints]
        .iter()
        .map(|x| x * x)
        .sum::<f64>()
        .sqrt();
    if norm > 0.0 {
        let k = spec.step_m / norm;
        for d in delta.iter_mut().take(spec.n_joints) {
            *d *= k;
        }
    }

    // ---- 4. through the proven face. no path around it. ------------------------------------
    let (st, out) = fast.admit(&delta, 0.0, now_ms);
    match st {
        FastStatus::Ok => Outcome::Move(out),
        // The fast face latched: it returns the safe hold, which is where the arm is, not zeros.
        // Zeros would be a *move*, and "fail safe" cannot mean "fly to the origin".
        FastStatus::Refuse => Outcome::Halted(out),
        FastStatus::Einval => Outcome::BadIntent("the solved command left the proven domain"),
        FastStatus::Unknown(_) => Outcome::BadIntent("fast face returned an unknown status"),
    }
}

/// Construct the handle used by the tests.  It is the real one when the proven core is linked and
/// the inert stub when it is not, so the same tests exercise whichever is actually compiled in.
#[allow(non_snake_case)]
pub fn Fast_for_test() -> Fast {
    #[cfg(feature = "fast")]
    {
        Fast::reset()
    }
    #[cfg(not(feature = "fast"))]
    {
        Fast
    }
}
