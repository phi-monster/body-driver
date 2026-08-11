//! The C ABI. `../../abi/body_layer.h` is the contract; this file implements it.
//!
//! # Why the ABI is the audit surface
//!
//! The claim is "anybody's mind + anybody's body". A leaderboard run, or anyone else's stack, must
//! go **through** these functions rather than reaching around them, because reaching around is
//! where cheating hides and nobody — including us — would know it had happened.
//!
//! So the ABI is built to make the cheat *unrepresentable* rather than *forbidden*:
//!
//! * [`bl_world_ref`] — everything any VLM or world model can say — is a normalised pixel, a verb,
//!   and a coarse effort scalar. It has no `z`, no pose, no object id, no task id. **A pointer
//!   that cannot express a pose cannot leak one.**
//! * [`bl_policy_in`] — everything the action model sees — is an image plus that reference. No
//!   joint angles, no link lengths, no camera matrix, no gripper span, no robot name. That absence
//!   *is* the invariant separating this from a body-conditioned policy.
//!
//! An auditor checking for a privileged channel reads the struct definitions. If no member can
//! carry it, no amount of downstream code can.
//!
//! # Safety
//!
//! Every entry point is `unsafe extern "C"` and validates its pointers before use. A null handle
//! or a version mismatch is a hard status, never a best-effort continue: a silently degraded body
//! layer is the exact thing this layer exists to eliminate.

use core::ffi::{c_char, c_void};
use core::mem::{align_of, size_of};

use crate::debt;
use crate::execute::{execute, Intent, Outcome, Spec};
use crate::measurement::{Measurement, Quantity, MAX_DEPS, MAX_DIM};
use crate::persist;
use crate::refuse::{Ask, Reason};
use crate::schedule;
use crate::Body;

#[cfg(feature = "fast")]
use crate::fast::Fast;
#[cfg(not(feature = "fast"))]
use crate::faststub::Fast;

/// Must match `BL_ABI_VERSION` in the header.
pub const BL_ABI_VERSION: u32 = 1;

/// Status codes; mirror of `bl_status`.
#[repr(u32)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Status {
    /// Proceed.
    Ok = 0,
    /// A refusal. **An answer, not an error.**
    Refuse = 1,
    /// Bad argument.
    Einval = 2,
    /// ABI version mismatch — hard failure, never a degrade.
    Eversion = 3,
    /// Output buffer too small.
    Enospace = 4,
    /// Internal invariant broken.
    Einternal = 5,
}

/// C-layout mirror of [`Measurement`]. Field order and types match the header exactly.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct CMeasurement {
    /// `bl_quantity`
    pub quantity: u32,
    /// used length of the arrays below
    pub dim: u32,
    /// measured value
    pub value: [f64; MAX_DIM],
    /// 1σ, same units as `value`
    pub uncertainty: [f64; MAX_DIM],
    /// low end of the range actually probed
    pub valid_lo: [f64; MAX_DIM],
    /// high end of the range actually probed
    pub valid_hi: [f64; MAX_DIM],
    /// monotonic ns
    pub measured_at_ns: u64,
    /// 0 == "until a dependency changes"
    pub valid_for_ns: u64,
    /// used length of `deps`
    pub n_deps: u32,
    /// quantities this was measured against
    pub deps: [u32; MAX_DEPS],
    /// their epoch at measurement time
    pub dep_epoch: [u64; MAX_DEPS],
    /// bumped on every re-measure
    pub epoch: u64,
    /// 0/1
    pub selftest_passed: u32,
    /// the epoch this replaced
    pub prev_epoch: u64,
}

/// C-layout mirror of `bl_world_ref` — **the entire vocabulary any VLM/WM may use**.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct CWorldRef {
    /// normalised pixel
    pub u: f64,
    /// normalised pixel
    pub v: f64,
    /// 0 for a point; >0 = region radius
    pub extent: f64,
    /// `bl_verb`
    pub verb: u32,
    /// `[0,1]` coarse effort. Force is stiffness × deflection — physics, zero data.
    pub manner: f64,
    /// which frame this refers to
    pub frame_id: u64,
}

impl CMeasurement {
    fn to_native(self) -> Option<Measurement> {
        let q = Quantity::from_u32(self.quantity)?;
        if self.dim == 0 || self.dim as usize > MAX_DIM {
            return None;
        }
        if self.n_deps as usize > MAX_DEPS {
            return None;
        }
        let mut deps = [None; MAX_DEPS];
        for i in 0..self.n_deps as usize {
            deps[i] = Some((Quantity::from_u32(self.deps[i])?, self.dep_epoch[i]));
        }
        Some(Measurement {
            quantity: q,
            dim: self.dim as usize,
            value: self.value,
            uncertainty: self.uncertainty,
            valid_lo: self.valid_lo,
            valid_hi: self.valid_hi,
            measured_at_ns: self.measured_at_ns,
            valid_for_ns: self.valid_for_ns,
            deps,
            epoch: self.epoch,
            selftest_passed: self.selftest_passed != 0,
            prev_epoch: self.prev_epoch,
        })
    }

    fn from_native(m: &Measurement) -> Self {
        let mut deps = [0u32; MAX_DEPS];
        let mut dep_epoch = [0u64; MAX_DEPS];
        let mut n = 0usize;
        for d in m.deps.iter().flatten() {
            deps[n] = d.0 as u32;
            dep_epoch[n] = d.1;
            n += 1;
        }
        CMeasurement {
            quantity: m.quantity as u32,
            dim: m.dim as u32,
            value: m.value,
            uncertainty: m.uncertainty,
            valid_lo: m.valid_lo,
            valid_hi: m.valid_hi,
            measured_at_ns: m.measured_at_ns,
            valid_for_ns: m.valid_for_ns,
            n_deps: n as u32,
            deps,
            dep_epoch,
            epoch: m.epoch,
            selftest_passed: u32::from(m.selftest_passed),
            prev_epoch: m.prev_epoch,
        }
    }
}

/// Bytes of storage one body needs. The caller allocates; this crate never does.
#[no_mangle]
pub extern "C" fn bl_sizeof_body() -> usize {
    size_of::<Body>()
}

/// Required alignment of that storage.
#[no_mangle]
pub extern "C" fn bl_alignof_body() -> usize {
    align_of::<Body>()
}

/// Initialise a body layer in caller-supplied storage.
///
/// 🔴 There is no allocation anywhere in this crate, on purpose. A hard-real-time safety layer
/// must not depend on an allocator, and a layer that cannot build for the target it has to run on
/// is not a deliverable. `storage` must be at least [`bl_sizeof_body`] bytes with at least
/// [`bl_alignof_body`] alignment, and must outlive every other call on this handle.
///
/// # Safety
/// `storage` must satisfy the size and alignment above and must not alias another live body.
#[no_mangle]
pub unsafe extern "C" fn bl_init(storage: *mut c_void, len: usize, abi_version: u32) -> Status {
    if storage.is_null() {
        return Status::Einval;
    }
    // A mismatch is refused outright. Accepting a near-enough version is how two components end up
    // disagreeing about what a struct means while both report success.
    if abi_version != BL_ABI_VERSION {
        return Status::Eversion;
    }
    if len < size_of::<Body>() {
        return Status::Enospace;
    }
    if (storage as usize) % align_of::<Body>() != 0 {
        return Status::Einval;
    }
    // SAFETY: size and alignment checked above; the caller promises exclusive ownership.
    unsafe { core::ptr::write(storage as *mut Body, Body::new()) };
    Status::Ok
}

/// Tear down a body layer. The storage itself belongs to the caller and is not freed here.
///
/// # Safety
/// `b` must have been initialised by [`bl_init`] and must not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn bl_close(b: *mut c_void) {
    if b.is_null() {
        return;
    }
    // SAFETY: initialised by `bl_init` per the contract; dropped exactly once.
    unsafe { core::ptr::drop_in_place(b as *mut Body) };
}

/// Submit a measurement the robot made about itself.
///
/// # Safety
/// `b` must come from [`bl_open`]; `m` must point to a valid `bl_measurement`.
#[no_mangle]
pub unsafe extern "C" fn bl_measure(b: *mut c_void, m: *const CMeasurement) -> Status {
    if b.is_null() || m.is_null() {
        return Status::Einval;
    }
    // SAFETY: both checked non-null; the caller guarantees provenance and validity.
    let body = unsafe { &mut *(b as *mut Body) };
    // SAFETY: as above.
    let cm = unsafe { *m };
    let Some(native) = cm.to_native() else {
        return Status::Einval;
    };
    match body.submit(native) {
        Ok(_) => Status::Ok,
        // Malformed is Einval and not Refuse on purpose: REFUSE is a statement about the *body*
        // ("I cannot do this safely"), while a malformed submission is a statement about the
        // *caller*. Merging them would let a caller bug read as a body limitation.
        Err(_) => Status::Einval,
    }
}

/// Read back the current measurement for a quantity.
///
/// # Safety
/// `b` must come from [`bl_open`]; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_get(b: *const c_void, quantity: u32, out: *mut CMeasurement) -> Status {
    if b.is_null() || out.is_null() {
        return Status::Einval;
    }
    let Some(q) = Quantity::from_u32(quantity) else {
        return Status::Einval;
    };
    // SAFETY: checked non-null; provenance guaranteed by the caller.
    let body = unsafe { &*(b as *const Body) };
    match body.get(q) {
        Some(m) => {
            // SAFETY: `out` checked non-null and the caller promises it is writable.
            unsafe { *out = CMeasurement::from_native(&m) };
            Status::Ok
        }
        None => Status::Refuse,
    }
}

/// The gate: may this world reference be executed on this body right now?
///
/// # Safety
/// `b` must come from [`bl_open`]; `refp` must be valid; `why` and `detail` must be writable, and
/// `detail` must have room for `BL_REASON_LEN` bytes.
#[no_mangle]
pub unsafe extern "C" fn bl_admit(
    b: *const c_void,
    refp: *const CWorldRef,
    now_ns: u64,
    why: *mut u32,
    detail: *mut c_char,
) -> Status {
    if b.is_null() || refp.is_null() || why.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null above.
    let body = unsafe { &*(b as *const Body) };
    // SAFETY: as above.
    let r = unsafe { *refp };

    let mut ask = Ask::EMPTY;
    // Executing any reference at all means putting *this* hand on *that* pixel, which needs the
    // hand point and the image Jacobian. Listing them here rather than deep in the servo is what
    // makes the refusal mechanical instead of dependent on somebody remembering.
    ask.needs[0] = Some(Quantity::HandPixel);
    ask.needs[1] = Some(Quantity::ImageJacobian);
    ask.needs[2] = Some(Quantity::Reach);
    ask.image_point = Some((r.u, r.v));

    let v = body.admit(&ask, now_ns);
    // SAFETY: `why` checked non-null.
    unsafe { *why = v.why as u32 };
    if !detail.is_null() {
        write_detail(detail, v.why, v.culprit);
    }
    if v.admit {
        Status::Ok
    } else {
        Status::Refuse
    }
}

/// Bitmask of quantities whose self-test currently passes.
///
/// # Safety
/// `b` must come from [`bl_open`]; `mask` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_selftest(b: *const c_void, mask: *mut u64) -> Status {
    if b.is_null() || mask.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null above.
    let body = unsafe { &*(b as *const Body) };
    // SAFETY: `mask` checked non-null.
    unsafe { *mask = body.selftest_mask() };
    Status::Ok
}

/// Stable human-readable reason. For logs and audit trails; never parsed.
#[no_mangle]
pub extern "C" fn bl_reason_str(why: u32) -> *const c_char {
    let s: &'static str = match why {
        x if x == Reason::None as u32 => "none\0",
        x if x == Reason::NeverMeasured as u32 => "never_measured\0",
        x if x == Reason::Stale as u32 => "stale\0",
        x if x == Reason::OutOfRange as u32 => "out_of_range\0",
        x if x == Reason::DependencyChanged as u32 => "dependency_changed\0",
        x if x == Reason::SelfTestFailed as u32 => "selftest_failed\0",
        x if x == Reason::UncertaintyTooHigh as u32 => "uncertainty_too_high\0",
        x if x == Reason::Unreachable as u32 => "unreachable\0",
        x if x == Reason::RateLimit as u32 => "rate_limit\0",
        _ => "unknown\0",
    };
    s.as_ptr() as *const c_char
}

/// Stable human-readable quantity name.
#[no_mangle]
pub extern "C" fn bl_quantity_str(quantity: u32) -> *const c_char {
    let s: &'static str = match Quantity::from_u32(quantity) {
        Some(Quantity::HandPixel) => "hand_pixel\0",
        Some(Quantity::ImageJacobian) => "image_jacobian\0",
        Some(Quantity::GripperSpan) => "gripper_span\0",
        Some(Quantity::ArmWeight) => "arm_weight\0",
        Some(Quantity::Latency) => "latency\0",
        Some(Quantity::Backlash) => "backlash\0",
        Some(Quantity::Reach) => "reach\0",
        Some(Quantity::ContactThreshold) => "contact_threshold\0",
        Some(Quantity::SelfOcclusion) => "self_occlusion\0",
        Some(Quantity::StepDelivery) => "step_delivery\0",
        Some(Quantity::ToolOffset) => "tool_offset\0",
        None => "unknown\0",
    };
    s.as_ptr() as *const c_char
}

/// Stable human-readable schedule reason.
#[no_mangle]
pub extern "C" fn bl_need_str(need: u32) -> *const c_char {
    let s: &'static str = match need {
        0 => "never_measured\0",
        1 => "stale\0",
        2 => "dependency_moved\0",
        3 => "selftest_failed\0",
        _ => "unknown\0",
    };
    s.as_ptr() as *const c_char
}

/// What this body still has to measure about itself, dependencies first.
///
/// `*n == 0` means every quantity this layer knows how to measure is currently valid. It does
/// **not** mean the body carries no hand-set constants — see [`bl_debt_outstanding`].
///
/// # Safety
/// `b` must come from [`bl_init`]; `quantities` and `needs` must each be writable for `cap`
/// `uint32_t`; `n` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_measure_plan(
    b: *const c_void,
    now_ns: u64,
    quantities: *mut u32,
    needs: *mut u32,
    cap: usize,
    n: *mut usize,
) -> Status {
    if b.is_null() || quantities.is_null() || needs.is_null() || n.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; provenance guaranteed by the caller.
    let body = unsafe { &*(b as *const Body) };
    let p = schedule::plan(body, now_ns);
    if p.n > cap {
        // Refusing rather than truncating: a plan cut short reads as a shorter list of things to
        // measure, and the caller would then believe the body owes less than it does.
        return Status::Enospace;
    }
    for (i, (q, why)) in p.steps().iter().enumerate() {
        // SAFETY: `i < p.n <= cap`, and the caller promises `cap` writable slots in each array.
        unsafe {
            *quantities.add(i) = *q as u32;
            *needs.add(i) = *why as u32;
        }
    }
    // SAFETY: `n` checked non-null.
    unsafe { *n = p.n };
    Status::Ok
}

/// Rows in the hand-set-constant ledger.
#[no_mangle]
pub extern "C" fn bl_debt_total() -> u32 {
    debt::total() as u32
}

/// Body constants this layer **cannot** supply today: no probe, or no slot at all.
///
/// 🔴 The honest counterpart to a body's structural zero. An auditor reading this header should be
/// able to get both numbers without reading any Rust, because reporting only the flattering one is
/// precisely the failure this layer exists to make impossible.
#[no_mangle]
pub extern "C" fn bl_debt_outstanding() -> u32 {
    debt::outstanding() as u32
}

/// Write row `i` of the ledger as `"name\tsite\tstanding\tnote"`, NUL-terminated and truncated.
///
/// # Safety
/// `buf` must be writable for `cap` bytes, `cap >= 1`.
#[no_mangle]
pub unsafe extern "C" fn bl_debt_line(i: u32, buf: *mut c_char, cap: usize) -> Status {
    if buf.is_null() || cap == 0 {
        return Status::Einval;
    }
    let Some(c) = debt::LEDGER.get(i as usize) else {
        return Status::Einval;
    };
    let standing: &str = match c.standing {
        debt::Standing::Measured(_) => "measured",
        debt::Standing::DeclaredOnly(_) => "declared_only",
        // An integrator reading this line needs to tell "nobody wrote the estimator" apart from
        // "the estimator is here and this body cannot feed it" -- the second is one probe away.
        debt::Standing::BlockedBy(_) => "blocked_by",
        debt::Standing::Outstanding => "outstanding",
        debt::Standing::NotABodyConstant => "not_a_body_constant",
    };
    let q: &str = match c.standing {
        debt::Standing::Measured(q) | debt::Standing::DeclaredOnly(q) | debt::Standing::BlockedBy(q) => {
            q.as_str()
        }
        _ => "-",
    };
    let mut at = 0usize;
    // SAFETY of every write below: `at + 1 < cap` is checked before each byte, so the index and the
    // final NUL are both inside the caller's buffer.
    let put = |s: &str, at: &mut usize| {
        for &byte in s.as_bytes() {
            if *at + 1 < cap {
                unsafe { *(buf as *mut u8).add(*at) = byte };
                *at += 1;
            }
        }
    };
    put(c.name, &mut at);
    put("\t", &mut at);
    put(c.site, &mut at);
    put("\t", &mut at);
    put(standing, &mut at);
    put(":", &mut at);
    put(q, &mut at);
    put("\t", &mut at);
    put(c.note, &mut at);
    // SAFETY: `at < cap` by construction above.
    unsafe { *(buf as *mut u8).add(at) = 0 };
    Status::Ok
}

/// Write `"<reason>:<quantity>"` into `detail`, NUL-terminated and truncated to fit.
fn write_detail(detail: *mut c_char, why: Reason, culprit: Option<Quantity>) {
    const CAP: usize = 96; // BL_REASON_LEN
    let mut buf = [0u8; CAP];
    let mut n = 0usize;
    let mut put = |s: &str, n: &mut usize| {
        for &byte in s.as_bytes() {
            if *n + 1 < CAP {
                buf[*n] = byte;
                *n += 1;
            }
        }
    };
    put(why.as_str(), &mut n);
    if let Some(q) = culprit {
        put(":", &mut n);
        put(q.as_str(), &mut n);
    }
    // SAFETY: `detail` was checked non-null by the caller and the contract promises CAP bytes;
    // `n < CAP` holds by construction above, so the NUL fits.
    unsafe {
        core::ptr::copy_nonoverlapping(buf.as_ptr(), detail as *mut u8, n + 1);
    }
}

/// C mirror of `bl_policy_out`.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct CPolicyOut {
    /// unit vector; `|dir|` is checked, never silently normalised
    pub dir: [f64; 3],
    /// rotation increment about the tool frame, rad
    pub drot: [f64; 3],
    /// ABSOLUTE opening in [0,1], not a delta
    pub grip: f64,
    /// vx, vy, wz for a mobile base
    pub base: [f64; 3],
}

/// C mirror of `bl_spec`.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct CSpec {
    /// metres per control period, from the machine's rating
    pub step_m: f64,
    /// control period, ms
    pub period_ms: u32,
    /// least-squares damping
    pub damping: f64,
    /// joints this body actually has
    pub n_joints: u32,
}

/// Mirror of `bl_exec_outcome`.
pub const BL_X_MOVE: u32 = 0;
/// The body layer refused. An **answer**, not an error.
pub const BL_X_REFUSED: u32 = 1;
/// The fast face latched; `joint_cmd` holds the safe hold.
pub const BL_X_HALTED: u32 = 2;
/// The intent was malformed — about the caller, not the body.
pub const BL_X_BAD_INTENT: u32 = 3;

/// Execute one step. See the header for the ordering guarantee.
///
/// # Safety
/// `b` must come from [`bl_init`]; `intent` and `spec` must be valid; `joint_cmd` must have room
/// for `BL_MAX_JOINTS` doubles; `outcome` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_execute(
    b: *mut c_void,
    intent: *const CPolicyOut,
    spec: *const CSpec,
    now_ms: u32,
    joint_cmd: *mut f64,
    outcome: *mut u32,
) -> Status {
    if b.is_null() || intent.is_null() || spec.is_null() || joint_cmd.is_null() || outcome.is_null()
    {
        return Status::Einval;
    }
    // SAFETY: all pointers checked non-null; provenance guaranteed by the caller.
    let body = unsafe { &*(b as *const Body) };
    // SAFETY: as above.
    let (ci, cs) = unsafe { (*intent, *spec) };

    let it = Intent {
        dir: ci.dir,
        drot: ci.drot,
        grip: ci.grip,
        base: ci.base,
    };
    let sp = Spec {
        step_m: cs.step_m,
        period_ms: cs.period_ms,
        damping: cs.damping,
        n_joints: cs.n_joints as usize,
    };

    let fast = fast_handle();
    let (code, cmd) = match execute(body, &fast, &sp, &it, now_ms) {
        Outcome::Move(c) => (BL_X_MOVE, Some(c)),
        Outcome::Halted(c) => (BL_X_HALTED, Some(c)),
        Outcome::Refused(_) => (BL_X_REFUSED, None),
        Outcome::BadIntent(_) => (BL_X_BAD_INTENT, None),
    };
    if let Some(c) = cmd {
        // SAFETY: the contract requires room for BL_MAX_JOINTS doubles.
        unsafe { core::ptr::copy_nonoverlapping(c.as_ptr(), joint_cmd, c.len()) };
    }
    // SAFETY: `outcome` checked non-null.
    unsafe { *outcome = code };
    Status::Ok
}

#[cfg(feature = "fast")]
fn fast_handle() -> Fast {
    Fast
}
#[cfg(not(feature = "fast"))]
fn fast_handle() -> Fast {
    Fast
}

/// Upper bound on the bytes [`bl_save`] can write, for any body.
#[no_mangle]
pub extern "C" fn bl_save_max_bytes() -> usize {
    persist::MAX_BYTES
}

/// Serialize the calibration set, records in dependency order.
///
/// # Safety
/// `b` must come from [`bl_init`]; `buf` must be writable for `cap` bytes; `written` writable.
#[no_mangle]
pub unsafe extern "C" fn bl_save(
    b: *const c_void,
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
) -> Status {
    if b.is_null() || buf.is_null() || written.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null above.
    let body = unsafe { &*(b as *const Body) };
    // SAFETY: the caller promises `cap` writable bytes at `buf`.
    let out = unsafe { core::slice::from_raw_parts_mut(buf, cap) };
    match persist::save(body, out) {
        Some(n) => {
            // SAFETY: `written` checked non-null.
            unsafe { *written = n };
            Status::Ok
        }
        None => Status::Enospace,
    }
}

/// Restore a calibration set. Every record goes through the same validation a live measurement
/// faces — a stored file is not a back door.
///
/// # Safety
/// `b` must come from [`bl_init`]; `buf` must be readable for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn bl_load(b: *mut c_void, buf: *const u8, len: usize) -> Status {
    if b.is_null() || buf.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null above.
    let body = unsafe { &mut *(b as *mut Body) };
    // SAFETY: the caller promises `len` readable bytes at `buf`.
    let src = unsafe { core::slice::from_raw_parts(buf, len) };
    match persist::load(body, src) {
        Ok(_) => Status::Ok,
        // A malformed or corrupt file is a statement about the FILE, so it is Einval rather than
        // Refuse: REFUSE means "this body cannot do that safely", which is a different fact.
        Err(_) => Status::Einval,
    }
}
