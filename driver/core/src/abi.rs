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
use crate::measurement::{AxisKind, Measurement, Quantity, MAX_DEPS, MAX_DIM};
use crate::predict::{self, Predicted};
use crate::execute;
use crate::hand;
use crate::probe::{self, Declined, Polarity};
use crate::memory::{
    Durability, Memory, Opens, PlaceKey, Recognised, Scope, FINGERPRINT_BYTES, SLOT_BYTES,
};
use crate::persist;
use crate::refuse::{Ask, Reason};
use crate::schedule;
use crate::Body;

#[cfg(feature = "fast")]
use crate::fast::Fast;
#[cfg(not(feature = "fast"))]
use crate::faststub::Fast;

/// Must match `BL_ABI_VERSION` in the header.
pub const BL_ABI_VERSION: u32 = 2;

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
    /// `bl_axis_kind` per axis: 0 interval, 1 categorical, 2 unmeasured. Zero is the pre-existing
    /// behaviour, so a caller that memsets its struct gets exactly what it got before this field.
    pub axis_kind: [u32; MAX_DIM],
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
        let mut axis_kind = [AxisKind::Interval; MAX_DIM];
        for (i, k) in axis_kind.iter_mut().enumerate() {
            *k = match self.axis_kind[i] {
                0 => AxisKind::Interval,
                1 => AxisKind::Categorical,
                2 => AxisKind::Unmeasured,
                // An unknown kind is refused, never coerced into a neighbouring one -- the same
                // rule `Quantity::from_u32` follows, for the same reason.
                _ => return None,
            };
        }
        Some(Measurement {
            quantity: q,
            dim: self.dim as usize,
            axis_kind,
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
        let mut axis_kind = [0u32; MAX_DIM];
        for (i, k) in m.axis_kind.iter().enumerate() {
            axis_kind[i] = *k as u32;
        }
        CMeasurement {
            quantity: m.quantity as u32,
            dim: m.dim as u32,
            axis_kind,
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
/// 🔴 THE GATE, FOR ONE QUANTITY.
///
/// `bl_admit` asks whether a whole world reference can be executed, which is the right question for
/// the servo and the wrong one for everybody else: a caller that just wants *"may I use the tool
/// offset, and is it established over the range I am about to ask about"* had no way to ask. So
/// every such caller re-implemented never-measured / stale / out-of-range / self-test-failed on its
/// own side — this project has a Python copy of exactly those four checks, which is how one gate
/// became two implementations that can drift.
///
/// `at` / `tol` are optional: pass `has_at = 0` / `has_tol = 0` to skip. `at` is where in the
/// quantity's own probed domain the ask sits; `tol` is the precision the ask needs.
///
/// Returns `BL_OK` to admit, or `BL_REFUSE` with `*why` and a line in `detail`. A REFUSE is an
/// answer.
#[no_mangle]
pub unsafe extern "C" fn bl_admit_quantity(
    b: *const c_void,
    quantity: u32,
    at: f64,
    has_at: u32,
    tol: f64,
    has_tol: u32,
    now_ns: u64,
    why: *mut u32,
    detail: *mut c_char,
) -> Status {
    if b.is_null() || why.is_null() {
        return Status::Einval;
    }
    let Some(q) = Quantity::from_u32(quantity) else {
        return Status::Einval;
    };
    // SAFETY: checked non-null above.
    let body = unsafe { &*(b as *const Body) };
    let mut ask = Ask::EMPTY;
    ask.needs[0] = Some(q);
    if has_at != 0 {
        ask.at[0] = Some(at);
    }
    if has_tol != 0 {
        ask.tolerance[0] = Some(tol);
    }
    let v = body.admit(&ask, now_ns);
    // SAFETY: checked non-null above.
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
    // Delegates. The hand-written second copy that used to live here fell behind by two reasons and
    // nothing noticed for as long as nobody read a refusal from another language.
    let s: &'static str = match Reason::from_u32(why) {
        Some(r) => r.as_cstr(),
        None => "unknown\0",
    };
    s.as_ptr() as *const c_char
}

/// Stable human-readable quantity name.
#[no_mangle]
pub extern "C" fn bl_quantity_str(quantity: u32) -> *const c_char {
    // 🔴 ONE TABLE. This used to be a second hand-written match mirroring `Quantity::as_str`, and
    // that is precisely the shape that left `bl_reason_str` publishing nine names for an eleven-
    // variant enum -- a new variant is added in one place and the ABI keeps answering for the old
    // set, silently. `name_c` lives next to the enum, so adding a variant without a name does not
    // compile.
    Quantity::from_u32(quantity)
        .map_or(c"unknown".as_ptr(), |q| q.name_c().as_ptr())
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

/* ============================================================ the thin OS's memory ============ */

/// Bytes of storage one memory needs. The caller allocates; this crate never does.
#[no_mangle]
pub extern "C" fn bl_memory_sizeof() -> usize {
    size_of::<Memory>()
}

/// Alignment one memory needs.
#[no_mangle]
pub extern "C" fn bl_memory_alignof() -> usize {
    align_of::<Memory>()
}

/// Initialise a memory in caller-supplied storage. `scope`: 0 task, 1 place.
///
/// # Safety
/// `storage` must be writable for `len` bytes and exclusively owned by the caller.
#[no_mangle]
pub unsafe extern "C" fn bl_memory_init(
    storage: *mut c_void,
    len: usize,
    scope: u32,
    abi_version: u32,
) -> Status {
    if storage.is_null() {
        return Status::Einval;
    }
    if abi_version != BL_ABI_VERSION {
        return Status::Eversion;
    }
    let scope = match scope {
        0 => Scope::Task,
        1 => Scope::Place,
        _ => return Status::Einval,
    };
    if len < size_of::<Memory>() {
        return Status::Enospace;
    }
    if (storage as usize) % align_of::<Memory>() != 0 {
        return Status::Einval;
    }
    // SAFETY: size and alignment checked above; the caller promises exclusive ownership.
    unsafe { core::ptr::write(storage as *mut Memory, Memory::new(scope)) };
    Status::Ok
}

/// Read a NUL-terminated C string, bounded. `None` if null, unterminated within the bound, or not
/// UTF-8 -- a name this layer cannot read is refused, never guessed at.
unsafe fn cstr<'a>(p: *const c_char, max: usize) -> Option<&'a str> {
    if p.is_null() {
        return None;
    }
    let mut n = 0usize;
    while n < max {
        // SAFETY: bounded by `max`, which callers size from BL_SLOT_BYTES.
        if unsafe { *p.add(n) } == 0 {
            // SAFETY: `n` bytes were just walked and found in bounds.
            let bytes = unsafe { core::slice::from_raw_parts(p as *const u8, n) };
            return core::str::from_utf8(bytes).ok();
        }
        n += 1;
    }
    None
}

/// Declare a named slot. `pins != 0` freezes it one observation after it is first written.
///
/// # Safety
/// `m` must come from [`bl_memory_init`]; `name` must be a NUL-terminated string.
#[no_mangle]
pub unsafe extern "C" fn bl_memory_declare(
    m: *mut c_void,
    name: *const c_char,
    pins: u32,
    why: *mut u32,
) -> Status {
    if m.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; the caller owns it.
    let mem = unsafe { &mut *(m as *mut Memory) };
    // SAFETY: bounded read of a caller-provided string.
    let Some(name) = (unsafe { cstr(name, SLOT_BYTES + 1) }) else {
        return Status::Einval;
    };
    match mem.declare(name, pins != 0) {
        Ok(()) => Status::Ok,
        Err(v) => {
            if !why.is_null() {
                // SAFETY: checked non-null.
                unsafe { *why = v.why as u32 };
            }
            Status::Refuse
        }
    }
}

/// 🔴 Advance the observation counter. This is what makes pinning mechanical: nothing in the model
/// can decline to do it, which is the whole reason the previous design's pin never engaged.
///
/// # Safety
/// `m` must come from [`bl_memory_init`].
#[no_mangle]
pub unsafe extern "C" fn bl_memory_observed(m: *mut c_void) -> Status {
    if m.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; the caller owns it.
    unsafe { &mut *(m as *mut Memory) }.observed();
    Status::Ok
}

/// Write a fact. `durability`: 0 perishable, 1 durable.
///
/// 🔴 A perishable fact is REFUSED, not stored — that is rung 1, and it is structural here rather
/// than a rule somebody has to keep at 3am.
///
/// # Safety
/// `m` must come from [`bl_memory_init`]; `name` and `value` must be NUL-terminated.
#[no_mangle]
pub unsafe extern "C" fn bl_memory_write(
    m: *mut c_void,
    name: *const c_char,
    value: *const c_char,
    durability: u32,
    why: *mut u32,
) -> Status {
    if m.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; the caller owns it.
    let mem = unsafe { &mut *(m as *mut Memory) };
    // SAFETY: bounded reads of caller-provided strings.
    let (Some(name), Some(value)) =
        (unsafe { cstr(name, SLOT_BYTES + 1) }, unsafe { cstr(value, SLOT_BYTES + 1) })
    else {
        return Status::Einval;
    };
    let d = match durability {
        0 => Durability::Perishable,
        1 => Durability::Durable,
        _ => return Status::Einval,
    };
    match mem.write(name, value, d) {
        Ok(_) => Status::Ok,
        Err(v) => {
            if !why.is_null() {
                // SAFETY: checked non-null.
                unsafe { *why = v.why as u32 };
            }
            Status::Refuse
        }
    }
}

/// Read a slot into `out` (at least `BL_SLOT_BYTES + 1` bytes). `BL_REFUSE` when the slot does not
/// exist or has never been written — which are two different facts, distinguished by `*why`.
///
/// # Safety
/// `m` must come from [`bl_memory_init`]; `out` must be writable for `cap` bytes.
#[no_mangle]
pub unsafe extern "C" fn bl_memory_get(
    m: *const c_void,
    name: *const c_char,
    out: *mut c_char,
    cap: usize,
    why: *mut u32,
) -> Status {
    if m.is_null() || out.is_null() || cap == 0 {
        return Status::Einval;
    }
    // SAFETY: checked non-null; shared borrow only.
    let mem = unsafe { &*(m as *const Memory) };
    // SAFETY: bounded read of a caller-provided string.
    let Some(name) = (unsafe { cstr(name, SLOT_BYTES + 1) }) else {
        return Status::Einval;
    };
    let Some(v) = mem.get(name) else {
        if !why.is_null() {
            // SAFETY: checked non-null.
            unsafe { *why = Reason::NeverMeasured as u32 };
        }
        return Status::Refuse;
    };
    if v.len() + 1 > cap {
        return Status::Enospace;
    }
    for (i, b) in v.as_bytes().iter().enumerate() {
        // SAFETY: bounds checked immediately above.
        unsafe { *out.add(i) = *b as c_char };
    }
    // SAFETY: `v.len() < cap` checked above.
    unsafe { *out.add(v.len()) = 0 };
    Status::Ok
}

/// Apply a memory-opening event. `event`: 0 new task, 1 unrecognised place, 2 body changed.
/// `*cleared` receives 1 if this memory was cleared.
///
/// # Safety
/// `m` must come from [`bl_memory_init`]; `cleared` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_memory_event(m: *mut c_void, event: u32, cleared: *mut u32) -> Status {
    if m.is_null() {
        return Status::Einval;
    }
    let e = match event {
        0 => Opens::NewTask,
        1 => Opens::UnrecognisedPlace,
        2 => Opens::BodyChanged,
        _ => return Status::Einval,
    };
    // SAFETY: checked non-null; the caller owns it.
    let was = unsafe { &mut *(m as *mut Memory) }.on_event(e);
    if !cleared.is_null() {
        // SAFETY: checked non-null.
        unsafe { *cleared = u32::from(was) };
    }
    Status::Ok
}

/// Counters: observations, unreadable replies, refused perishable facts, filled slots, declared
/// slots. Any pointer may be null.
///
/// `unreadable` is here because a channel failing quietly looks exactly like a world that is merely
/// slow, and only a count separates them.
///
/// # Safety
/// `m` must come from [`bl_memory_init`]; each non-null pointer must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_memory_stats(
    m: *const c_void,
    observations: *mut u64,
    unreadable: *mut u64,
    refused_perishable: *mut u64,
    filled: *mut u32,
    declared: *mut u32,
) -> Status {
    if m.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; shared borrow only.
    let mem = unsafe { &*(m as *const Memory) };
    // SAFETY: each pointer is checked before it is written.
    unsafe {
        if !observations.is_null() {
            *observations = mem.observations;
        }
        if !unreadable.is_null() {
            *unreadable = mem.unreadable;
        }
        if !refused_perishable.is_null() {
            *refused_perishable = mem.refused_perishable;
        }
        if !filled.is_null() {
            *filled = mem.filled() as u32;
        }
        if !declared.is_null() {
            *declared = mem.declared() as u32;
        }
    }
    Status::Ok
}

/// Is this the same place? Returns 0 same, 1 new, 2 **cannot tell**.
///
/// 🔴 The third answer is the point. Misidentifying a place is worse than having no memory at all —
/// you would act on a map of somewhere else, confidently — so "unsure" must not be coerced into
/// either of the others.
///
/// # Safety
/// Both byte pointers must be readable for `BL_FINGERPRINT_BYTES` bytes.
#[no_mangle]
pub unsafe extern "C" fn bl_place_matches(
    a: *const u8,
    a_confidence: f64,
    b: *const u8,
    b_confidence: f64,
    out: *mut u32,
) -> Status {
    if a.is_null() || b.is_null() || out.is_null() {
        return Status::Einval;
    }
    let mut ka = [0u8; FINGERPRINT_BYTES];
    let mut kb = [0u8; FINGERPRINT_BYTES];
    // SAFETY: the caller promises FINGERPRINT_BYTES readable at each pointer.
    unsafe {
        core::ptr::copy_nonoverlapping(a, ka.as_mut_ptr(), FINGERPRINT_BYTES);
        core::ptr::copy_nonoverlapping(b, kb.as_mut_ptr(), FINGERPRINT_BYTES);
        *out = match PlaceKey::new(ka, a_confidence).matches(&PlaceKey::new(kb, b_confidence)) {
            Recognised::Same => 0,
            Recognised::New => 1,
            Recognised::Unsure => 2,
        };
    }
    Status::Ok
}

/* ============================================================ prediction ====================== */

/// What a learned model says about where a reference will be.
///
/// 🔴 Look at what is absent: no `z`, no pose, no object id. The same vocabulary as
/// [`CWorldRef`], and for the same reason — this is the most natural place in the whole design for
/// a 3-D pose to enter ("just tell me where it will BE"), and a prediction that could return one
/// would be a leak with a respectable name.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct CPredicted {
    /// normalised image coordinates at `at_period`
    pub u: f64,
    pub v: f64,
    /// normalised region size
    pub extent: f64,
    /// control periods from now; 0 == "right now", which is what NOT predicting asserts
    pub at_period: u32,
    /// 1σ of (u,v) at that horizon, normalised. Required; there is no default.
    pub sigma_uv: f64,
    /// largest horizon this model was ACTUALLY validated over; 0 == never validated
    pub verified_periods: u32,
}

impl CPredicted {
    fn to_native(self) -> Predicted {
        Predicted {
            u: self.u,
            v: self.v,
            extent: self.extent,
            at_period: self.at_period,
            sigma_uv: self.sigma_uv,
            verified_periods: self.verified_periods,
        }
    }
}

/// How many control periods this body will be BLIND while it covers `distance_m`.
///
/// The horizon a caller must predict over, from this body's own measured delivery — not from a
/// guess. Refuses when the body never measured what it needs.
///
/// # Safety
/// `b` must come from [`bl_open`]; `out` and `why` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_predict_horizon(
    b: *const c_void,
    distance_m: f64,
    tol_frac: f64,
    out: *mut u32,
    why: *mut u32,
) -> Status {
    if b.is_null() || out.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; shared borrow only.
    let body = unsafe { &*(b as *const Body) };
    match predict::horizon(body, distance_m, tol_frac) {
        Ok(n) => {
            // SAFETY: checked non-null.
            unsafe { *out = n };
            Status::Ok
        }
        Err(v) => {
            if !why.is_null() {
                // SAFETY: checked non-null.
                unsafe { *why = v.why as u32 };
            }
            Status::Refuse
        }
    }
}

/// May this prediction be acted on for a motion that leaves the loop blind `need_periods`?
///
/// `has_tol = 0` skips the precision requirement. `BL_OK` with `*why == BL_R_NO_EVIDENCE` is the
/// third rung: admitted, and nothing has validated this model at this horizon.
///
/// # Safety
/// `p` must be a valid `bl_predicted`; `why` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_predict_admit(
    p: *const CPredicted,
    need_periods: u32,
    tol_uv: f64,
    has_tol: u32,
    why: *mut u32,
    detail: *mut c_char,
) -> Status {
    if p.is_null() || why.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null.
    let pred = unsafe { *p }.to_native();
    let v = predict::admit(&pred, need_periods, if has_tol != 0 { Some(tol_uv) } else { None });
    // SAFETY: checked non-null.
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

/// 🔴 THE WHOLE QUESTION IN ONE CALL: *may I chase this thing across `distance_m`?*
///
/// Asks the body for its own blind horizon and then gates the prediction against it, so a caller
/// cannot ask the second half without the first. That is exactly how a conveyor loop came to aim
/// at a stale point while every reading in it looked healthy: image error 7.2–7.9 px, contact
/// within 9–28 mm, descent drift 1.8–6.9 mm — and the hand 17–30 cm from the object, because
/// close-and-lift took 44 periods and the belt moved 4 mm in each of them.
///
/// # Safety
/// `b` must come from [`bl_open`]; `p` must be a valid `bl_predicted`; `why` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_predict_admit_chase(
    b: *const c_void,
    p: *const CPredicted,
    distance_m: f64,
    tol_frac: f64,
    tol_uv: f64,
    has_tol: u32,
    why: *mut u32,
    detail: *mut c_char,
) -> Status {
    if b.is_null() || p.is_null() || why.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; shared borrow only.
    let body = unsafe { &*(b as *const Body) };
    // SAFETY: checked non-null.
    let pred = unsafe { *p }.to_native();
    let tol = if has_tol != 0 { Some(tol_uv) } else { None };
    let v = match predict::admit_chase(body, &pred, distance_m, tol_frac, tol) {
        Ok(v) | Err(v) => v,
    };
    // SAFETY: checked non-null.
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

/* ============================================================ probes ========================== */
/* 🔴 THE MEASURING HALF, WHICH WAS UNREACHABLE FROM C UNTIL 2026-08-11.
 *
 * Eleven probes existed in Rust and `nm` reported ZERO probe symbols exported. So a caller in any
 * other language could read a body constant and could ask whether it may be used -- and could not
 * MEASURE ONE. That is the half this layer is for: *世界靠学,身体靠量*. A robot that can only be
 * handed numbers is a robot with a config file.
 *
 * One function per probe, explicitly typed. A single generic entry taking `params[]` would be
 * shorter and would encode "params[2] is the Jacobian epoch" as a positional convention nobody can
 * check -- and unchecked positional conventions are, specifically, a bug class this repository has
 * paid for. */

/// Why a probe declined. Distinct from [`Reason`]: a probe declines to PRODUCE a measurement, a
/// gate refuses to ADMIT one. Collapsing them would lose which half of the system said no.
#[repr(u32)]
pub enum CDeclined {
    NotEnoughSamples = 0,
    NoResponse = 1,
    Inconsistent = 2,
    MissingDependency = 3,
}

fn decl(d: Declined) -> u32 {
    match d {
        Declined::NotEnoughSamples => CDeclined::NotEnoughSamples as u32,
        Declined::NoResponse => CDeclined::NoResponse as u32,
        Declined::Inconsistent => CDeclined::Inconsistent as u32,
        Declined::MissingDependency => CDeclined::MissingDependency as u32,
    }
}

/// Human-readable, for logs and audit trails. Never parsed.
/// Fit the floor over a grid of "I pressed down here and stopped at this height" samples.
///
/// `tol_m` is the probe's own height resolution — the descent step it took. Two cells that stop
/// within one step of each other are indistinguishable to the probe that produced them, so that is
/// the width the robust trim uses; it is not a tuning knob.
///
/// # Safety
/// `xs`, `ys`, `zs` must each have `n` readable doubles; `out` and `why` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_floor_fit(
    xs: *const f64,
    ys: *const f64,
    zs: *const f64,
    n: usize,
    tol_m: f64,
    now_ns: u64,
    thr_epoch: u64,
    sd_epoch: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    if xs.is_null() || ys.is_null() || zs.is_null() || out.is_null() || why.is_null() {
        return Status::Einval;
    }
    // SAFETY: caller guarantees n readable doubles in each.
    let (x, y, z) = unsafe {
        (
            core::slice::from_raw_parts(xs, n),
            core::slice::from_raw_parts(ys, n),
            core::slice::from_raw_parts(zs, n),
        )
    };
    match crate::floor::fit(x, y, z, tol_m, now_ns, thr_epoch, sd_epoch) {
        Ok(m) => {
            // SAFETY: checked non-null.
            unsafe {
                *out = CMeasurement::from_native(&m);
                *why = 0;
            }
            Status::Ok
        }
        Err(d) => {
            // SAFETY: checked non-null.
            unsafe { *why = d as u32 };
            Status::Refuse
        }
    }
}

/// 🔴 THE HAND STOPPED HERE — WHAT STOPPED IT?
///
/// The one call that separates *something is in the way* from *this arm has no solution here*,
/// with no joint states, no force channel, and no extra command. `*stop` receives a `bl_stop`;
/// `*height_m` receives how far above the floor (for `BL_STOP_ON_SOMETHING`, its height) or below
/// it (`BL_STOP_ARM_LIMIT`).
///
/// `band_sigmas` is the caller's accuracy requirement — how many of the floor's own residual widths
/// still count as "on it". A peg-in-hole wants it tight and a sweep wants it loose, so it is asked
/// for rather than chosen here.
///
/// # Safety
/// `b` must come from [`bl_init`]; `stop`, `why`, `floor_z`, `height_m` writable or null.
#[no_mangle]
pub unsafe extern "C" fn bl_floor_read_stop(
    b: *const c_void,
    x: f64,
    y: f64,
    stop_z: f64,
    band_sigmas: f64,
    stop: *mut u32,
    floor_z: *mut f64,
    height_m: *mut f64,
    why: *mut u32,
    detail: *mut c_char,
) -> Status {
    if b.is_null() || stop.is_null() || why.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; shared borrow only.
    let body = unsafe { &*(b as *const Body) };
    let r = crate::floor::read_stop(body, x, y, stop_z, band_sigmas);
    let (code, h) = match r.stop {
        None => (0u32, f64::NAN),
        Some(crate::floor::Stop::OnFloor) => (1, 0.0),
        Some(crate::floor::Stop::OnSomething(d)) => (2, d),
        Some(crate::floor::Stop::ArmLimit(d)) => (3, d),
    };
    // SAFETY: checked non-null.
    unsafe {
        *stop = code;
        *why = r.verdict.why as u32;
    }
    if !floor_z.is_null() {
        // SAFETY: checked non-null.
        unsafe { *floor_z = r.floor_z };
    }
    if !height_m.is_null() {
        // SAFETY: checked non-null.
        unsafe { *height_m = h };
    }
    if !detail.is_null() {
        write_detail(detail, r.verdict.why, r.verdict.culprit);
    }
    if r.verdict.admit {
        Status::Ok
    } else {
        Status::Refuse
    }
}

/// Name for a `bl_stop`. One table in Rust; see [`bl_quantity_str`] for why that matters.
#[no_mangle]
pub extern "C" fn bl_stop_str(v: u32) -> *const c_char {
    match v {
        0 => c"unknown".as_ptr(),
        1 => c"on_floor".as_ptr(),
        2 => c"on_something".as_ptr(),
        3 => c"arm_limit".as_ptr(),
        _ => c"?".as_ptr(),
    }
}

/// 🔴 AM I TOUCHING SOMETHING, OR HAVE I RUN OUT OF SOLUTION?
///
/// The two look identical to a delivered-motion ruler, which only ever watches the one axis that
/// was commanded. Measured on a flat conveyor: two of nine probe points reported contact while the
/// arm could not lift off — 0.299 and 0.572 of a command it delivers 0.9999 of in free space — and
/// they stalled 7–9 cm below the plane the four true contacts agree on to within 2.2 cm.
///
/// `delivered_reverse` must come from commanding the OPPOSITE direction at the SAME magnitude.
/// `has_reverse = 0` means it was not asked, and this refuses (`BL_R_NO_EVIDENCE`) rather than
/// guessing — guessing is the bug. `sideways` is recorded and never decides: friction blocks
/// sideways motion on a perfectly real surface.
///
/// `*touch` receives a `bl_touch`. `*free_bar` receives the bar the reverse had to clear, derived
/// from this body's own `step_delivery` and `contact_threshold`.
///
/// # Safety
/// `b` must come from [`bl_init`]; `touch`, `why` and `free_bar` must be writable or null.
/* ---- 动词层 (2026-08-12) --------------------------------------------------------------------
 * 在此之前 `bl_world_ref.verb` 在驱动里一次都没被读过,所有招式逻辑住在策略的 Python 里。
 * 这四个入口把它搬回来:策略只能【问】,不能自己算。每一条的判据都是实测换来的,见 verb.rs。
 */

/// "碰到了没有"。推得动的物体不会让手停下来 ⇒ 手停 **或** 世界看得见地动了。
#[no_mangle]
pub unsafe extern "C" fn bl_contact_seen(
    commanded_m: f64,
    achieved_m: f64,
    contact_threshold: f64,
    object_moved_m: f64,
    object_move_eps: f64,
) -> u32 {
    crate::verb::contact_seen(
        commanded_m, achieved_m, contact_threshold, object_moved_m, object_move_eps,
    ) as u32
}

/// "这一段我夹不夹得下"。夹不下就不该试。
#[no_mangle]
pub unsafe extern "C" fn bl_spannable(
    section_width_m: f64,
    jaw_span_m: f64,
    margin_m: f64,
) -> u32 {
    crate::verb::spannable(section_width_m, jaw_span_m, margin_m) as u32
}

/// 合完爪的自查:拿量出来的爪值反推实际夹住多宽,和打算夹的那一段比。
#[no_mangle]
pub unsafe extern "C" fn bl_grasp_check(
    planned_w_m: f64,
    jaw_value: f64,
    jaw_span_m: f64,
    object_moved_m: f64,
    knock_eps_m: f64,
    tol_frac: f64,
) -> u32 {
    crate::verb::classify(
        planned_w_m, jaw_value, jaw_span_m, object_moved_m, knock_eps_m, tol_frac,
    ) as u32
}

/// 自查之后该怎么办。合到底 ⇒ 换招式,不是换地方。
#[no_mangle]
pub unsafe extern "C" fn bl_after_check(verb: u32, check: u32, out_verb: *mut u32) -> u32 {
    let v = match crate::verb::Verb::from_u32(verb) {
        Some(v) => v,
        None => return 255,
    };
    let c = match check {
        0 => crate::verb::Check::AsPlanned,
        1 => crate::verb::Check::ClosedOnAir,
        2 => crate::verb::Check::WrongSection,
        3 => crate::verb::Check::KnockedAway,
        // 劈开的两档:编号往后加,既有 0..3 不动 —— ABI 的既有编号一改,
        // 所有已编译的调用方会静默换语义,而这正是本层存在的理由。
        4 => crate::verb::Check::StoppedWide,
        5 => crate::verb::Check::PinchedThinner,
        _ => return 255,
    };
    match crate::verb::decide(v, c) {
        crate::verb::Next::Proceed => 0,
        crate::verb::Next::NextContact => 1,
        crate::verb::Next::ChangeVerb(nv) => {
            if !out_verb.is_null() {
                unsafe { *out_verb = nv as u32 };
            }
            2
        }
        crate::verb::Next::Relook => 3,
    }
}

#[no_mangle]
pub unsafe extern "C" fn bl_touching(
    b: *const c_void,
    delivered_along: f64,
    delivered_reverse: f64,
    has_reverse: u32,
    sideways: f64,
    has_sideways: u32,
    now_ns: u64,
    touch: *mut u32,
    free_bar: *mut f64,
    why: *mut u32,
    detail: *mut c_char,
) -> Status {
    if b.is_null() || touch.is_null() || why.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; shared borrow only.
    let body = unsafe { &*(b as *const Body) };
    let r = crate::touch::touching(
        body,
        delivered_along,
        if has_reverse != 0 { Some(delivered_reverse) } else { None },
        if has_sideways != 0 { Some(sideways) } else { None },
        now_ns,
    );
    // SAFETY: checked non-null.
    unsafe {
        *touch = r.touch as u32;
        *why = r.verdict.why as u32;
    }
    if !free_bar.is_null() {
        // SAFETY: checked non-null.
        unsafe { *free_bar = r.free_bar };
    }
    if !detail.is_null() {
        write_detail(detail, r.verdict.why, r.verdict.culprit);
    }
    if r.verdict.admit {
        Status::Ok
    } else {
        Status::Refuse
    }
}

/// Name for a `bl_touch`. One table in Rust, so a name cannot go missing the way `bl_reason_str`
/// once did by being hand-mirrored.
#[no_mangle]
pub extern "C" fn bl_touch_str(t: u32) -> *const c_char {
    match crate::touch::Touch::from_u32(t) {
        Some(v) => match v {
            crate::touch::Touch::Unknown => c"unknown".as_ptr(),
            crate::touch::Touch::Free => c"free".as_ptr(),
            crate::touch::Touch::Contact => c"contact".as_ptr(),
            crate::touch::Touch::Stuck => c"stuck".as_ptr(),
        },
        None => c"?".as_ptr(),
    }
}

#[no_mangle]
pub extern "C" fn bl_declined_str(d: u32) -> *const c_char {
    let s: &'static str = match d {
        0 => "not_enough_samples\0",
        1 => "no_response\0",
        2 => "inconsistent\0",
        3 => "missing_dependency\0",
        _ => "unknown\0",
    };
    s.as_ptr() as *const c_char
}

/// SAFETY helper: a caller-owned pair of parallel arrays becomes a slice of tuples in a scratch
/// buffer. Bounded by `PROBE_MAX_SAMPLES`; a longer submission is refused rather than truncated,
/// because a truncated sample set produces a confident measurement of the wrong thing.
const PROBE_MAX_SAMPLES: usize = 4096;
/// Stack budget for the Jacobian probe: each Sample is large, so this is smaller than the pair cap.
const PROBE_SAMPLE_CAP: usize = 256;

unsafe fn pairs(xs: *const f64, ys: *const f64, n: usize, into: &mut [(f64, f64)]) -> bool {
    if xs.is_null() || ys.is_null() || n == 0 || n > into.len() {
        return false;
    }
    for i in 0..n {
        // SAFETY: the caller promises `n` readable doubles at each pointer; `n <= into.len()`.
        into[i] = unsafe { (*xs.add(i), *ys.add(i)) };
    }
    true
}

fn emit(r: Result<Measurement, Declined>, out: *mut CMeasurement, why: *mut u32) -> Status {
    match r {
        Ok(m) => {
            if out.is_null() {
                return Status::Einval;
            }
            // SAFETY: checked non-null; the caller owns `out`.
            unsafe { *out = CMeasurement::from_native(&m) };
            Status::Ok
        }
        Err(d) => {
            if !why.is_null() {
                // SAFETY: checked non-null.
                unsafe { *why = decl(d) };
            }
            Status::Refuse
        }
    }
}

/// 🔴 WHAT HOLDING STILL AGAINST GRAVITY COSTS — the force probe.
///
/// `joint_angle[i]` / `hold_torque[i]`: the arm parks at a pose, touches nothing, and reports the
/// torque needed to stay there. On one rig this turned 55–95 N of apparent load into **1.89 N**.
///
/// The validity range is the set of poses actually visited, and that is load-bearing: a gravity
/// self-calibration on this project had its ENTIRE residual in interpolation between sampled
/// poses, so asking outside them is exactly where the number stops meaning anything.
///
/// # Safety
/// Both arrays must be readable for `n` doubles; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_arm_weight(
    joint_angle: *const f64,
    hold_torque: *const f64,
    n: usize,
    now_ns: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    let mut buf = [(0.0f64, 0.0f64); PROBE_MAX_SAMPLES];
    // SAFETY: bounded copy out of caller-owned arrays.
    if !unsafe { pairs(joint_angle, hold_torque, n, &mut buf) } {
        return Status::Einval;
    }
    emit(probe::arm_weight(&buf[..n], now_ns), out, why)
}

/// 🔴 WHAT "I TOUCHED SOMETHING" READS LIKE ON THIS BODY.
///
/// Two labelled populations of the same signal: `free` while moving through air, `touching` while
/// pressed against something. Depends on [`bl_probe_arm_weight`]: any contact signal a joint can
/// produce has the gravity load in it, so measure the hold torque first or the threshold is a
/// statement about the arm's own weight. Pass its epoch so the layer can invalidate this the
/// moment the weight is re-measured.
///
/// # Safety
/// Both arrays must be readable for their lengths; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_contact_threshold(
    free: *const f64,
    n_free: usize,
    touching: *const f64,
    n_touching: usize,
    polarity: u32,
    now_ns: u64,
    arm_weight_epoch: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    let polarity = match polarity {
        0 => Polarity::HigherOnContact,
        1 => Polarity::LowerOnContact,
        // Not defaulted. Guessing which way a body's contact signal moves is how a detector ends up
        // firing in free space and staying silent on contact.
        _ => return Status::Einval,
    };
    if free.is_null() || touching.is_null() || n_free == 0 || n_touching == 0 {
        return Status::Einval;
    }
    if n_free > PROBE_MAX_SAMPLES || n_touching > PROBE_MAX_SAMPLES {
        return Status::Enospace;
    }
    // SAFETY: non-null and bounded, checked immediately above.
    let (f, t) = unsafe {
        (
            core::slice::from_raw_parts(free, n_free),
            core::slice::from_raw_parts(touching, n_touching),
        )
    };
    emit(probe::contact_threshold(f, t, polarity, now_ns, arm_weight_epoch), out, why)
}

/// How much of a commanded step actually arrives in one control period.
///
/// `commanded[i]` / `achieved[i]`, same units. Two arms on one harness answered **0.76** and
/// **0.11** to the same 45 mm command; a budget set from the first left the second 0.136 m short on
/// every episode while every scalar in its log looked ordinary.
///
/// # Safety
/// Both arrays must be readable for `n` doubles; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_step_delivery(
    commanded: *const f64,
    achieved: *const f64,
    n: usize,
    now_ns: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    let mut buf = [(0.0f64, 0.0f64); PROBE_MAX_SAMPLES];
    // SAFETY: bounded copy out of caller-owned arrays.
    if !unsafe { pairs(commanded, achieved, n, &mut buf) } {
        return Status::Einval;
    }
    emit(probe::step_delivery(&buf[..n], now_ns), out, why)
}

/// Where this body can actually put its hand, as a radial band from its own base.
///
/// `radius[i]` / `attained[i] != 0`: did the arm reach a pose at that distance. A radial band is
/// the shape reach actually has; a hand-typed axis-aligned box rejected a layout 0.409 m from the
/// base while accepting four further ones.
///
/// # Safety
/// Both arrays must be readable for `n` elements; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_reach(
    radius: *const f64,
    attained: *const u32,
    n: usize,
    now_ns: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    if radius.is_null() || attained.is_null() || n == 0 || n > PROBE_MAX_SAMPLES {
        return Status::Einval;
    }
    let mut buf = [(0.0f64, false); PROBE_MAX_SAMPLES];
    for i in 0..n {
        // SAFETY: non-null and bounded, checked above.
        buf[i] = unsafe { (*radius.add(i), *attained.add(i) != 0) };
    }
    emit(probe::reach(&buf[..n], now_ns), out, why)
}

/// Dead time: how many control periods pass before anything moves.
///
/// `first_motion_step < 0` means nothing moved within `steps_observed` — which is a refusal, not a
/// latency of `steps_observed`.
///
/// # Safety
/// `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_latency(
    first_motion_step: i64,
    steps_observed: u32,
    now_ns: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    let first = if first_motion_step < 0 {
        None
    } else {
        Some(first_motion_step as u32)
    };
    emit(probe::latency(first, steps_observed, now_ns), out, why)
}

/// The dead band around a reversal.
///
/// `commanded[i]` / `observed[i]`, SIGNED, time-ordered. Push both ways; what does not arrive on
/// the reversal is the slop. The number-one accuracy killer on cheap hardware, and measurable
/// without any extra sensor.
///
/// # Safety
/// Both arrays must be readable for `n` doubles; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_backlash(
    commanded: *const f64,
    observed: *const f64,
    n: usize,
    now_ns: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    let mut buf = [(0.0f64, 0.0f64); PROBE_MAX_SAMPLES];
    // SAFETY: bounded copy out of caller-owned arrays.
    if !unsafe { pairs(commanded, observed, n, &mut buf) } {
        return Status::Einval;
    }
    emit(probe::backlash(&buf[..n], now_ns), out, why)
}

/// Full-open to full-closed, in metres, measured off this body's own jaws.
///
/// `opening[i]` in `[0,1]` / `separation[i]` in image units, converted by `units_per_m`. Refuses
/// `NoResponse` when the commanded opening does not move the observed signal — which is what this
/// body answers today, and why an approach height cannot be derived on it.
///
/// # Safety
/// Both arrays must be readable for `n` doubles; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_gripper_span(
    opening: *const f64,
    separation: *const f64,
    n: usize,
    units_per_m: f64,
    units_per_m_sigma: f64,
    now_ns: u64,
    jac_epoch: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    let mut buf = [(0.0f64, 0.0f64); PROBE_MAX_SAMPLES];
    // SAFETY: bounded copy out of caller-owned arrays.
    if !unsafe { pairs(opening, separation, n, &mut buf) } {
        return Status::Einval;
    }
    emit(
        probe::gripper_span(&buf[..n], units_per_m, units_per_m_sigma, now_ns, jac_epoch),
        out,
        why,
    )
}

/// How far the working point sits from the mount, along the tool axis, in metres.
///
/// `wrist_angle[i]` / `u[i]` / `v[i]`: turn the wrist and the working point sweeps an arc whose
/// radius IS the offset. This is the constant that was typed in at four places in one live stack,
/// with three values for three bodies and a default that silently used another robot's.
///
/// # Safety
/// All three arrays must be readable for `n` doubles; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_tool_offset(
    wrist_angle: *const f64,
    u: *const f64,
    v: *const f64,
    n: usize,
    units_per_m: f64,
    units_per_m_sigma: f64,
    now_ns: u64,
    jac_epoch: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    if wrist_angle.is_null() || u.is_null() || v.is_null() || n == 0 || n > PROBE_MAX_SAMPLES {
        return Status::Einval;
    }
    let mut buf = [(0.0f64, 0.0f64, 0.0f64); PROBE_MAX_SAMPLES];
    for i in 0..n {
        // SAFETY: non-null and bounded, checked above.
        buf[i] = unsafe { (*wrist_angle.add(i), *u.add(i), *v.add(i)) };
    }
    emit(
        probe::tool_offset(&buf[..n], units_per_m, units_per_m_sigma, now_ns, jac_epoch),
        out,
        why,
    )
}

/// 🔴 THE PROBE THE UNIVERSAL LOOP CANNOT START WITHOUT: how this body's own commands move its own
/// image.
///
/// `cmd` is `n_samples * n_axes`, row-major: the command issued at each sample, in each axis's own
/// units. `uv` is `n_samples * 2`. `at_ns` is `n_samples`.
///
/// 🔴 `n_axes` IS THE ACTUATOR, AND THAT IS WHY THIS LAYER IS ACTUATOR-AGNOSTIC. Probe with six
/// joint commands and the result maps joints to pixels; probe with three end-effector commands and
/// it maps end-effector motion to pixels. `execute::solve` returns deltas in whatever axes were
/// probed. The ONLY requirement is that the probe and the executor use the SAME axes -- which is
/// exactly what the layout defect of 2026-08-11 violated, silently, in a place no test could reach.
///
/// This matters for the claim: RoboDojo and CALVIN take end-effector poses, others take joints. A
/// layer that only spoke one of them would need a different body driver per benchmark.
///
/// # Safety
/// `cmd` must be readable for `n_samples * n_axes` doubles, `uv` for `2 * n_samples`, `at_ns` for
/// `n_samples`; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_image_jacobian(
    cmd: *const f64,
    uv: *const f64,
    at_ns: *const u64,
    n_samples: usize,
    n_axes: usize,
    now_ns: u64,
    min_response_px: f64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    if cmd.is_null() || uv.is_null() || at_ns.is_null() {
        return Status::Einval;
    }
    if n_samples < 2 || n_samples > PROBE_MAX_SAMPLES || n_axes == 0 || n_axes > MAX_DIM {
        return Status::Einval;
    }
    let mut samples = [probe::Sample { cmd: [0.0; MAX_DIM], n: 0, uv: [0.0; 2], at_ns: 0 };
                       PROBE_SAMPLE_CAP];
    if n_samples > PROBE_SAMPLE_CAP {
        return Status::Enospace;
    }
    for (i, s) in samples.iter_mut().enumerate().take(n_samples) {
        s.n = n_axes;
        for a in 0..n_axes {
            // SAFETY: the caller promises n_samples * n_axes readable doubles.
            s.cmd[a] = unsafe { *cmd.add(i * n_axes + a) };
        }
        // SAFETY: the caller promises 2 * n_samples doubles and n_samples timestamps.
        unsafe {
            s.uv = [*uv.add(2 * i), *uv.add(2 * i + 1)];
            s.at_ns = *at_ns.add(i);
        }
    }
    emit(
        probe::image_jacobian(&samples[..n_samples], n_axes, now_ns, min_response_px),
        out,
        why,
    )
}

/// Which pixels are my hand: submit the candidates a detector found and let the tracker decide.
///
/// `u`, `v`, `gain`, `rigidity`, `pixels`, `spread` are parallel arrays of length `n`. The tracker
/// is stateless across calls here -- one shot, from one set of candidates -- which is the form a
/// caller starting up needs.
///
/// # Safety
/// Every array must be readable for `n` elements; `out` must be writable.
#[no_mangle]
pub unsafe extern "C" fn bl_probe_hand_pixel(
    u: *const f64,
    v: *const f64,
    gain: *const f64,
    rigidity: *const f64,
    pixels: *const u32,
    spread: *const f64,
    n: usize,
    now_ns: u64,
    epoch: u64,
    prev_epoch: u64,
    jac_epoch: u64,
    out: *mut CMeasurement,
    why: *mut u32,
) -> Status {
    if u.is_null() || v.is_null() || gain.is_null() || rigidity.is_null() || pixels.is_null()
        || spread.is_null() || n == 0 || n > 64
    {
        return Status::Einval;
    }
    let mut cands = [hand::Candidate { u: 0.0, v: 0.0, gain: 0.0, rigidity: 0.0, pixels: 0,
                                       spread: 0.0 }; 64];
    for (i, c) in cands.iter_mut().enumerate().take(n) {
        // SAFETY: the caller promises n readable elements in each array.
        unsafe {
            *c = hand::Candidate {
                u: *u.add(i),
                v: *v.add(i),
                gain: *gain.add(i),
                rigidity: *rigidity.add(i),
                pixels: *pixels.add(i),
                spread: *spread.add(i),
            };
        }
    }
    let mut tracker = hand::HandTracker::new(probe::default_hand_config());
    emit(
        probe::hand_pixel(&mut tracker, &cands[..n], now_ns, epoch, prev_epoch, jac_epoch),
        out,
        why,
    )
}

/// 🔴 WHICH WAY TO MOVE: an image-plane direction becomes actuator deltas, through the body's own
/// measured Jacobian.
///
/// This is the step that used to live only inside `bl_execute`, behind the proven fast face -- so
/// the default build could not reach it, and the layout defect it contained had nowhere to fail.
/// Exposed on its own because a stack whose actuator is end-effector poses (RoboDojo, CALVIN)
/// cannot use the joint-command path at all, and would otherwise need its own body driver.
///
/// `dir` is a direction in the frame the eye was shown: `dir[0], dir[1]` in the image,
/// `dir[2]` the tool-axis component the image cannot see. `out` receives `spec->n_joints` deltas
/// in **the axes the Jacobian was probed over**, scaled to `spec->step_m`.
///
/// # Safety
/// `b` must come from [`bl_open`]; `out` must have room for `BL_MAX_JOINTS` doubles.
#[no_mangle]
pub unsafe extern "C" fn bl_solve(
    b: *const c_void,
    spec: *const CSpec,
    dir: *const f64,
    out: *mut f64,
    why: *mut u32,
) -> Status {
    if b.is_null() || spec.is_null() || dir.is_null() || out.is_null() {
        return Status::Einval;
    }
    // SAFETY: checked non-null; shared borrow only.
    let body = unsafe { &*(b as *const Body) };
    // SAFETY: checked non-null.
    let sp = unsafe { *spec };
    let Some(jac) = body.get(Quantity::ImageJacobian) else {
        if !why.is_null() {
            // SAFETY: checked non-null.
            unsafe { *why = Reason::NeverMeasured as u32 };
        }
        return Status::Refuse;
    };
    let native = execute::Spec {
        step_m: sp.step_m,
        period_ms: sp.period_ms,
        damping: sp.damping,
        n_joints: sp.n_joints as usize,
    };
    // SAFETY: the caller promises three readable doubles.
    let d = unsafe { [*dir, *dir.add(1), *dir.add(2)] };
    let delta = execute::solve(&jac, &native, &d);
    if sp.n_joints == 0 || sp.n_joints as usize > delta.len() {
        return Status::Einval;
    }
    // scale to the body's own step, the same way `bl_execute` does -- distance is the body's
    // business, never the caller's
    let norm = delta[..native.n_joints].iter().map(|x| x * x).sum::<f64>().sqrt();
    for i in 0..native.n_joints {
        // SAFETY: `n_joints <= MAX_JOINTS` checked above, and `out` has room for MAX_JOINTS.
        unsafe {
            *out.add(i) = if norm > 0.0 { delta[i] / norm * native.step_m } else { 0.0 };
        }
    }
    Status::Ok
}
