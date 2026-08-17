//! Stand-in for the proven fast face, used **only** when the `fast` feature is off.
//!
//! 🔴 It exists so the pure-Rust tests can run on a machine without a GNAT toolchain, and for no
//! other reason. It does not implement the limit, force or watchdog rules — it **refuses
//! everything**. That is deliberate: a stub that quietly permitted motion would be a second,
//! unproven copy of the safety logic, and the unproven copy is the one that drifts.
//!
//! If a build reaches a robot with this compiled in, nothing moves. That is the correct failure.

/// Same size as the proven core's `Max_Joints`.
pub const MAX_JOINTS: usize = 16;
/// Same shape as the real binding's joint vector.
pub type Joints = [f64; MAX_JOINTS];

/// Mirror of the real status enum.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum FastStatus {
    /// Never returned by the stub.
    Ok,
    /// Always returned by the stub.
    Refuse,
    /// Never returned by the stub.
    Einval,
    /// Never returned by the stub.
    Unknown(i32),
}

/// The stub handle.
#[derive(Copy, Clone, Debug)]
pub struct Fast;

impl Fast {
    /// Refuses, always, and says so.
    pub fn admit(&self, _cmd: &Joints, _force: f64, _now_ms: u32) -> (FastStatus, Joints) {
        (FastStatus::Refuse, [0.0; MAX_JOINTS])
    }
    /// Always halted: there is no proven core behind this.
    pub fn halted(&self) -> bool {
        true
    }
}
