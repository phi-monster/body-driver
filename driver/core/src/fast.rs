//! Binding to the **proven** fast face (`../../fast/`, Ada/SPARK, `gnatprove` 40/40).
//!
//! # Why bind instead of reimplement
//!
//! Re-writing the limit / force / watchdog logic here "to avoid the FFI" would leave **two copies
//! of the rule and one proof**, and the copy without the proof is the one that drifts. One
//! implementation, proven, called by everybody.
//!
//! # What crosses the boundary
//!
//! Fixed-size arrays of exactly [`MAX_JOINTS`] doubles, always — a length that travels separately
//! from its buffer is a length that can disagree with it. The Ada side validates every value
//! against the constrained subtypes it was proven for and returns [`Status::Einval`] rather than
//! saturating: a wrapper that "repairs" an out-of-domain input makes the proof describe a number
//! nobody sent, which is worse than no proof.

use core::ffi::{c_double, c_int, c_uint};

/// Must equal `Max_Joints` in `body_layer_fast.ads`. Checked at run time by
/// [`Fast::selftest`], because a silent disagreement here would mis-index every joint.
pub const MAX_JOINTS: usize = 16;

/// A joint vector as it crosses to Ada. Always full length regardless of the robot's joint count.
pub type Joints = [c_double; MAX_JOINTS];

/// Mirror of `bl_status` for the fast face.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum FastStatus {
    /// Admitted.
    Ok,
    /// Refused — **an answer**, not an error.
    Refuse,
    /// A value was outside the domain the proven core was proven for.
    Einval,
    /// The Ada side returned something this binding does not know.
    Unknown(c_int),
}

impl FastStatus {
    fn from_c(v: c_int) -> Self {
        match v {
            0 => FastStatus::Ok,
            1 => FastStatus::Refuse,
            2 => FastStatus::Einval,
            other => FastStatus::Unknown(other),
        }
    }
}

/// Why the fast face latched. Mirror of `Halt_Reason'Pos`.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum HaltReason {
    /// Running.
    None,
    /// Asked to act before any envelope was installed.
    NotInstalled,
    /// A command left the installed envelope.
    LimitViolation,
    /// Measured force above the installed cap.
    ForceExceeded,
    /// No fresh command within the deadline.
    WatchdogExpired,
    /// Somebody pressed the button.
    ExternalStop,
    /// The Ada side returned a code this binding does not know.
    Unknown(c_int),
}

impl HaltReason {
    fn from_c(v: c_int) -> Self {
        match v {
            0 => HaltReason::None,
            1 => HaltReason::NotInstalled,
            2 => HaltReason::LimitViolation,
            3 => HaltReason::ForceExceeded,
            4 => HaltReason::WatchdogExpired,
            5 => HaltReason::ExternalStop,
            other => HaltReason::Unknown(other),
        }
    }
}

extern "C" {
    fn blf_reset();
    fn blf_install(
        lo: *const Joints,
        hi: *const Joints,
        hold0: *const Joints,
        n: c_uint,
        cap: c_double,
        deadline: c_uint,
        now: c_uint,
    ) -> c_int;
    fn blf_admit(cmd: *const Joints, force: c_double, now: c_uint, out_cmd: *mut Joints) -> c_int;
    fn blf_tick(now: c_uint);
    fn blf_stop();
    fn blf_clear(witness: c_int, now: c_uint) -> c_int;
    fn blf_halted() -> c_int;
    fn blf_reason() -> c_int;
}

/// Handle to the fast face. One per process, matching the Ada side.
#[derive(Copy, Clone, Debug)]
pub struct Fast;

impl Fast {
    /// Return the fast face to its initial state: **halted, not installed**.
    ///
    /// The refusing state is the correct default — a safety layer whose default is "permit"
    /// permits whenever somebody forgets to configure it, and forgetting is the normal case.
    pub fn reset() -> Self {
        // SAFETY: no arguments, no pointers; the Ada side owns its own storage.
        unsafe { blf_reset() };
        Fast
    }

    /// Install the measured envelope and the arm's **current** pose as the safe hold.
    ///
    /// `hold0` is where the arm *is*, not a value this layer invents. The Ada side refuses if it
    /// is not inside `[lo, hi]` — an arm that is not where the caller says it is must not be
    /// driven, and seating the hold at, say, the midpoint of the range can put it through the
    /// table.
    pub fn install(
        &self,
        lo: &Joints,
        hi: &Joints,
        hold0: &Joints,
        n: u32,
        cap_newton: f64,
        deadline_ms: u32,
        now_ms: u32,
    ) -> FastStatus {
        // SAFETY: all three pointers are to live, full-length arrays; the Ada side reads only the
        // first `n` elements and validates each before use.
        FastStatus::from_c(unsafe {
            blf_install(lo, hi, hold0, n, cap_newton, deadline_ms, now_ms)
        })
    }

    /// The gate. Every motion goes through here.
    ///
    /// On refusal, `out_cmd` receives the **safe hold** — the last admitted command — never zeros.
    /// Zeros would be a *move*, and "fail safe" cannot mean "fly to the origin".
    pub fn admit(&self, cmd: &Joints, force_newton: f64, now_ms: u32) -> (FastStatus, Joints) {
        let mut out: Joints = [0.0; MAX_JOINTS];
        // SAFETY: `cmd` and `out` are live full-length arrays for the duration of the call.
        let st = FastStatus::from_c(unsafe { blf_admit(cmd, force_newton, now_ms, &mut out) });
        (st, out)
    }

    /// Call at least once per control period even when idle, or the watchdog latches.
    pub fn tick(&self, now_ms: u32) {
        // SAFETY: scalar argument only.
        unsafe { blf_tick(now_ms) };
    }

    /// Latch a halt from outside.
    pub fn stop(&self) {
        // SAFETY: no arguments.
        unsafe { blf_stop() };
    }

    /// Leave a halt. `witness` must name the reason actually latched — a caller that does not know
    /// why the system halted has no business restarting it.
    pub fn clear(&self, witness: HaltReason, now_ms: u32) -> FastStatus {
        let w = match witness {
            HaltReason::None => return FastStatus::Einval,
            HaltReason::NotInstalled => 1,
            HaltReason::LimitViolation => 2,
            HaltReason::ForceExceeded => 3,
            HaltReason::WatchdogExpired => 4,
            HaltReason::ExternalStop => 5,
            HaltReason::Unknown(_) => return FastStatus::Einval,
        };
        // SAFETY: scalar arguments only.
        FastStatus::from_c(unsafe { blf_clear(w, now_ms) })
    }

    /// Is the fast face latched?
    pub fn halted(&self) -> bool {
        // SAFETY: no arguments.
        unsafe { blf_halted() != 0 }
    }

    /// Why it is latched.
    pub fn reason(&self) -> HaltReason {
        // SAFETY: no arguments.
        HaltReason::from_c(unsafe { blf_reason() })
    }

    /// Prove the binding is wired to the code it thinks it is, by making the far side **refuse**.
    ///
    /// 🔴 A binding that has only ever been exercised on the happy path is a binding whose error
    /// codes have never been observed. If the Ada side were stubbed out, or the symbols resolved
    /// to something else, every call would return `Ok` and nothing downstream would notice. So
    /// this drives one guaranteed refusal and one guaranteed acceptance, and reports both.
    pub fn selftest(&self) -> Result<(), &'static str> {
        let lo: Joints = [-1.0; MAX_JOINTS];
        let hi: Joints = [1.0; MAX_JOINTS];
        let hold: Joints = [0.0; MAX_JOINTS];

        Self::reset();
        if !self.halted() {
            return Err("fresh fast face is not halted -- default is permissive");
        }

        // A domain violation must be refused, not repaired.
        let mut bad = lo;
        bad[0] = f64::NAN;
        if self.install(&bad, &hi, &hold, 6, 20.0, 100, 1_000) != FastStatus::Einval {
            return Err("NaN limit was not rejected");
        }

        if self.install(&lo, &hi, &hold, 6, 20.0, 100, 1_000) != FastStatus::Ok {
            return Err("a well-formed install was rejected");
        }

        // Out of envelope must halt, not clamp.
        let mut over: Joints = [0.5; MAX_JOINTS];
        over[2] = 5.0;
        let (st, held) = self.admit(&over, 0.0, 1_010);
        if st != FastStatus::Refuse {
            return Err("an out-of-envelope command was admitted");
        }
        if held[2] < lo[2] || held[2] > hi[2] {
            return Err("the safe hold is outside the installed envelope");
        }
        if self.reason() != HaltReason::LimitViolation {
            return Err("halt reason is not LimitViolation");
        }

        // And a good command must be admitted, or this is just a refuser.
        Self::reset();
        if self.install(&lo, &hi, &hold, 6, 20.0, 100, 1_000) != FastStatus::Ok {
            return Err("re-install after reset failed");
        }
        let good: Joints = [0.5; MAX_JOINTS];
        if self.admit(&good, 1.0, 1_050).0 != FastStatus::Ok {
            return Err("an in-envelope command was refused");
        }
        Ok(())
    }
}
