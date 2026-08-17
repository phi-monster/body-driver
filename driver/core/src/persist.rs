//! `bl_save` / `bl_load` — the calibration set, with its provenance, on disk.
//!
//! # Why this is not "just serialisation"
//!
//! A stored calibration is a claim about a robot **at a moment**. Reload it a month later, after
//! the gripper was swapped and the camera was knocked, and every number is still there and still
//! looks fine. That is the failure this layer exists to remove, so the format carries the same
//! provenance the in-memory measurement does — timestamps, dependency epochs, probed ranges — and
//! [`load`] **refuses** anything it cannot vouch for rather than trusting it.
//!
//! # Rules the format follows, each paid for
//!
//! * **A checksum over the payload.** A truncated file must fail loudly. This repository has a
//!   recorded case of a scan silently truncating to the first 1,020 files while eight checksums
//!   all passed — because the checksums were over the pieces, not over the whole.
//! * **A version, checked as a hard mismatch.** Never a best-effort degrade: a silently degraded
//!   body layer is precisely the thing this layer exists to eliminate.
//! * **Fixed-width, little-endian, no self-describing framing.** The reader can therefore verify
//!   the length arithmetic before touching a byte, and a wrong length is caught rather than
//!   interpreted.
//! * **No allocation.** Same reason as the rest of the crate: this has to run where there is no
//!   allocator.

use crate::measurement::{AxisKind, MAX_DEPS, MAX_DIM, Malformed, Measurement, Quantity};
use crate::Body;

/// Magic + version. Bumped whenever the layout changes; a mismatch is refused, never coerced.
const MAGIC: [u8; 4] = *b"BLC2";  // BLC1 -> BLC2: records carry AxisKind per axis

/// Bytes per stored measurement. Derived, not written by hand, so it cannot drift from the layout.
const REC: usize = 4          // quantity
    + 4                       // dim
    + 4 * MAX_DIM             // axis_kind
    + 4 * 8 * MAX_DIM         // value, uncertainty, valid_lo, valid_hi
    + 8                       // measured_at_ns
    + 8                       // valid_for_ns
    + 4                       // n_deps
    + MAX_DEPS * (4 + 8)      // dep quantity + epoch
    + 8                       // epoch
    + 1                       // selftest_passed
    + 8; // prev_epoch

/// Header: magic, count, then a checksum over everything after the header.
const HDR: usize = 4 + 4 + 8;

/// Total bytes a full calibration set occupies.
pub const MAX_BYTES: usize = HDR + Quantity::COUNT * REC;

/// Why a stored set was refused on load.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum LoadError {
    /// Not one of ours, or a version this build does not implement.
    BadMagic,
    /// Length does not match what the header claims.
    BadLength,
    /// The payload does not hash to the stored digest.
    Corrupt,
    /// A record inside was malformed. Carries the reason so the operator learns which.
    BadRecord(Malformed),
    /// A record named a quantity this build does not know.
    UnknownQuantity(u32),
}

/// FNV-1a, 64-bit. Chosen because it is four lines and needs no dependency — an auditor of a
/// safety layer should not have to read a hash crate to convince themselves of the framing.
/// It detects truncation and bit-rot, which is the job here; it is not a security primitive and
/// this comment exists so nobody later mistakes it for one.
fn digest(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x1000_0000_01b3);
    }
    h
}

struct Cursor<'a> {
    buf: &'a mut [u8],
    at: usize,
}

impl Cursor<'_> {
    fn put_u32(&mut self, v: u32) {
        self.buf[self.at..self.at + 4].copy_from_slice(&v.to_le_bytes());
        self.at += 4;
    }
    fn put_u64(&mut self, v: u64) {
        self.buf[self.at..self.at + 8].copy_from_slice(&v.to_le_bytes());
        self.at += 8;
    }
    fn put_f64(&mut self, v: f64) {
        self.buf[self.at..self.at + 8].copy_from_slice(&v.to_le_bytes());
        self.at += 8;
    }
}

struct Reader<'a> {
    buf: &'a [u8],
    at: usize,
}

impl Reader<'_> {
    fn u32(&mut self) -> u32 {
        let mut b = [0u8; 4];
        b.copy_from_slice(&self.buf[self.at..self.at + 4]);
        self.at += 4;
        u32::from_le_bytes(b)
    }
    fn u64(&mut self) -> u64 {
        let mut b = [0u8; 8];
        b.copy_from_slice(&self.buf[self.at..self.at + 8]);
        self.at += 8;
        u64::from_le_bytes(b)
    }
    fn f64(&mut self) -> f64 {
        f64::from_bits(self.u64())
    }
}

/// Order the present quantities so every dependency is written **before** whatever depends on it.
///
/// 🔴 Not cosmetic. `HandPixel` is quantity 0 and the `ImageJacobian` it was measured against is
/// quantity 1, so writing in numeric order produces a file whose first record cites a dependency
/// the reader has not seen — and the reader correctly refuses it. A unit test caught exactly that.
///
/// The alternative, a two-pass loader that installs everything and validates afterwards, was
/// rejected: it means a record is briefly in the store before it has been checked, and "briefly
/// unchecked" is how a smuggled constant gets a foothold. Ordering the *file* keeps the loader a
/// single pass through the same door every live measurement uses.
///
/// Returns `None` if the dependencies do not form a DAG. A cycle is a bug in whatever produced the
/// measurements, and writing a file that cannot be read back is not a kindness.
fn dependency_order(body: &Body) -> Option<([Quantity; Quantity::COUNT], usize)> {
    let mut out = [Quantity::HandPixel; Quantity::COUNT];
    let mut done = [false; Quantity::COUNT];
    let mut n = 0usize;

    // At most COUNT passes: each pass must place at least one quantity or the rest form a cycle.
    for _ in 0..Quantity::COUNT {
        let mut placed_any = false;
        for i in 0..Quantity::COUNT {
            if done[i] {
                continue;
            }
            let Some(q) = Quantity::from_u32(i as u32) else {
                continue;
            };
            let Some(m) = body.get(q) else {
                done[i] = true; // absent: nothing to write, nothing to wait for
                continue;
            };
            let ready = m
                .deps
                .iter()
                .flatten()
                .all(|(dq, _)| body.get(*dq).is_none() || done[*dq as usize]);
            if ready {
                out[n] = q;
                n += 1;
                done[i] = true;
                placed_any = true;
            }
        }
        if !placed_any {
            break;
        }
    }
    if done.iter().all(|d| *d) {
        Some((out, n))
    } else {
        None // a cycle
    }
}

/// Write the whole calibration set into `out`. Returns the number of bytes written.
///
/// Only measurements that exist are written; a body that knows five things about itself stores
/// five records, and reloading it leaves the other four **absent** rather than defaulted. Absent
/// is the honest state and the admit gate already knows what to do with it.
pub fn save(body: &Body, out: &mut [u8]) -> Option<usize> {
    if out.len() < HDR {
        return None;
    }
    let (order, present) = dependency_order(body)?;
    let total = HDR + present * REC;
    if out.len() < total {
        return None;
    }

    out[0..4].copy_from_slice(&MAGIC);
    out[4..8].copy_from_slice(&(present as u32).to_le_bytes());
    // digest slot filled last, once the payload exists

    let mut c = Cursor {
        buf: &mut out[HDR..total],
        at: 0,
    };
    for q in order.iter().take(present) {
        let Some(m) = body.get(*q) else { continue };
        c.put_u32(m.quantity as u32);
        c.put_u32(m.dim as u32);
        for k in 0..MAX_DIM {
            c.put_u32(m.axis_kind[k] as u32);
        }
        for k in 0..MAX_DIM {
            c.put_f64(m.value[k]);
        }
        for k in 0..MAX_DIM {
            c.put_f64(m.uncertainty[k]);
        }
        for k in 0..MAX_DIM {
            c.put_f64(m.valid_lo[k]);
        }
        for k in 0..MAX_DIM {
            c.put_f64(m.valid_hi[k]);
        }
        c.put_u64(m.measured_at_ns);
        c.put_u64(m.valid_for_ns);
        let n = m.deps.iter().filter(|d| d.is_some()).count();
        c.put_u32(n as u32);
        for k in 0..MAX_DEPS {
            match m.deps[k] {
                Some((dq, ep)) => {
                    c.put_u32(dq as u32);
                    c.put_u64(ep);
                }
                None => {
                    c.put_u32(0);
                    c.put_u64(0);
                }
            }
        }
        c.put_u64(m.epoch);
        c.buf[c.at] = u8::from(m.selftest_passed);
        c.at += 1;
        c.put_u64(m.prev_epoch);
    }

    let d = digest(&out[HDR..total]);
    out[8..16].copy_from_slice(&d.to_le_bytes());
    Some(total)
}

/// Restore a calibration set into `body`.
///
/// 🔴 Every record goes back in through [`Body::submit`], the **same** door a fresh measurement
/// uses, so it faces the same validation. A loader with its own private path is a loader that can
/// admit what the live path refuses — and then the stored set is a way to smuggle in a constant
/// nobody measured, which is the exact hole this layer exists to close.
///
/// `now_ns` is not used to reject stale records here: staleness is the **admit gate's** job, and
/// deciding it at load time would silently discard a set the operator can see and reason about.
/// What is refused at load is what is *malformed* or *corrupt* — facts about the file, not about
/// the passage of time.
pub fn load(body: &mut Body, buf: &[u8]) -> Result<usize, LoadError> {
    if buf.len() < HDR || buf[0..4] != MAGIC {
        return Err(LoadError::BadMagic);
    }
    let mut n_b = [0u8; 4];
    n_b.copy_from_slice(&buf[4..8]);
    let count = u32::from_le_bytes(n_b) as usize;
    if count > Quantity::COUNT {
        return Err(LoadError::BadLength);
    }
    let total = HDR + count * REC;
    if buf.len() < total {
        return Err(LoadError::BadLength);
    }
    let mut d_b = [0u8; 8];
    d_b.copy_from_slice(&buf[8..16]);
    if u64::from_le_bytes(d_b) != digest(&buf[HDR..total]) {
        return Err(LoadError::Corrupt);
    }

    let mut r = Reader {
        buf: &buf[HDR..total],
        at: 0,
    };
    let mut loaded = 0usize;
    for _ in 0..count {
        let qraw = r.u32();
        let Some(quantity) = Quantity::from_u32(qraw) else {
            return Err(LoadError::UnknownQuantity(qraw));
        };
        let dim = r.u32() as usize;
        let mut axis_kind = [AxisKind::Interval; MAX_DIM];
        for k in axis_kind.iter_mut() {
            *k = match r.u32() {
                0 => AxisKind::Interval,
                1 => AxisKind::Categorical,
                2 => AxisKind::Unmeasured,
                other => return Err(LoadError::UnknownQuantity(other)),
            };
        }
        let mut m = Measurement {
            quantity,
            axis_kind,
            dim,
            value: [0.0; MAX_DIM],
            uncertainty: [0.0; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [0.0; MAX_DIM],
            measured_at_ns: 0,
            valid_for_ns: 0,
            deps: [None; MAX_DEPS],
            epoch: 0,
            selftest_passed: false,
            prev_epoch: 0,
        };
        for k in 0..MAX_DIM {
            m.value[k] = r.f64();
        }
        for k in 0..MAX_DIM {
            m.uncertainty[k] = r.f64();
        }
        for k in 0..MAX_DIM {
            m.valid_lo[k] = r.f64();
        }
        for k in 0..MAX_DIM {
            m.valid_hi[k] = r.f64();
        }
        m.measured_at_ns = r.u64();
        m.valid_for_ns = r.u64();
        let n_deps = r.u32() as usize;
        for k in 0..MAX_DEPS {
            let dq = r.u32();
            let ep = r.u64();
            if k < n_deps {
                let Some(dq) = Quantity::from_u32(dq) else {
                    return Err(LoadError::UnknownQuantity(dq));
                };
                m.deps[k] = Some((dq, ep));
            }
        }
        m.epoch = r.u64();
        m.selftest_passed = r.buf[r.at] != 0;
        r.at += 1;
        m.prev_epoch = r.u64();

        body.submit(m).map_err(LoadError::BadRecord)?;
        loaded += 1;
    }
    Ok(loaded)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measurement::{MAX_DEPS, MAX_DIM};

    fn m(q: Quantity, v: f64) -> Measurement {
        let mut x = Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity: q,
            dim: 1,
            value: [0.0; MAX_DIM],
            uncertainty: [0.01; MAX_DIM],
            valid_lo: [-1.0; MAX_DIM],
            valid_hi: [1.0; MAX_DIM],
            measured_at_ns: 1_234,
            valid_for_ns: 0,
            deps: [None; MAX_DEPS],
            epoch: 0,
            selftest_passed: true,
            prev_epoch: 0,
        };
        x.value[0] = v;
        x
    }

    /// What went in comes out, provenance and all. Without this the rest of the tests are checking
    /// that a corrupt file is rejected while never having shown a good one is accepted.
    #[test]
    fn round_trip_preserves_provenance() {
        let mut a = Body::new();
        let je = a.submit(m(Quantity::ImageJacobian, 800.0)).unwrap();
        let mut hp = m(Quantity::HandPixel, 0.5);
        hp.deps[0] = Some((Quantity::ImageJacobian, je));
        a.submit(hp).unwrap();

        let mut buf = [0u8; MAX_BYTES];
        let n = save(&a, &mut buf).expect("save failed");

        let mut b = Body::new();
        assert_eq!(load(&mut b, &buf[..n]), Ok(2));

        let got = b.get(Quantity::HandPixel).expect("hand pixel missing");
        assert_eq!(got.value[0], 0.5);
        assert_eq!(got.deps[0].map(|d| d.0), Some(Quantity::ImageJacobian));
        assert!(b.get(Quantity::ArmWeight).is_none(), "absent stayed absent");
    }

    /// 🔴 Truncation must fail loudly. A scan in this repository once silently truncated to its
    /// first 1,020 files while eight checksums passed — because they were over the pieces.
    #[test]
    fn truncation_is_refused() {
        let mut a = Body::new();
        a.submit(m(Quantity::ArmWeight, 1.89)).unwrap();
        let mut buf = [0u8; MAX_BYTES];
        let n = save(&a, &mut buf).unwrap();

        let mut b = Body::new();
        assert_eq!(load(&mut b, &buf[..n - 9]), Err(LoadError::BadLength));
        assert!(b.get(Quantity::ArmWeight).is_none());
    }

    /// A single flipped byte must not load.
    #[test]
    fn corruption_is_refused() {
        let mut a = Body::new();
        a.submit(m(Quantity::ArmWeight, 1.89)).unwrap();
        let mut buf = [0u8; MAX_BYTES];
        let n = save(&a, &mut buf).unwrap();
        buf[HDR + 20] ^= 0x01;

        let mut b = Body::new();
        assert_eq!(load(&mut b, &buf[..n]), Err(LoadError::Corrupt));
    }

    /// 🔴 The load path must not be a back door. A record whose self-test did not pass is refused
    /// on load exactly as it would be refused live — otherwise a stored file becomes a way to
    /// smuggle in a constant nobody measured.
    #[test]
    fn a_stored_record_cannot_bypass_validation() {
        let mut a = Body::new();
        a.submit(m(Quantity::ArmWeight, 1.89)).unwrap();
        let mut buf = [0u8; MAX_BYTES];
        let n = save(&a, &mut buf).unwrap();

        // Clear the selftest byte in the stored record, then repair the digest so the file is
        // otherwise perfectly well formed — i.e. exactly what a determined shortcut would look like.
        let off = HDR + REC - 9;
        buf[off] = 0;
        let d = digest(&buf[HDR..n]);
        buf[8..16].copy_from_slice(&d.to_le_bytes());

        let mut b = Body::new();
        assert_eq!(
            load(&mut b, &buf[..n]),
            Err(LoadError::BadRecord(Malformed::SelfTestFailed))
        );
        assert!(b.get(Quantity::ArmWeight).is_none());
    }

    /// 🔴 The dependency-order guarantee, tested from the direction that used to break: the
    /// dependent quantity has the LOWER id, so numeric order would emit it first and the file
    /// would be unreadable by its own loader.
    #[test]
    fn a_dependent_is_written_after_what_it_depends_on() {
        let mut a = Body::new();
        let je = a.submit(m(Quantity::ImageJacobian, 800.0)).unwrap();
        let mut hp = m(Quantity::HandPixel, 0.5); // id 0, depends on id 1
        hp.deps[0] = Some((Quantity::ImageJacobian, je));
        a.submit(hp).unwrap();

        let mut buf = [0u8; MAX_BYTES];
        let n = save(&a, &mut buf).unwrap();
        // The first record in the file must be the Jacobian, not the hand pixel.
        let mut q = [0u8; 4];
        q.copy_from_slice(&buf[HDR..HDR + 4]);
        assert_eq!(
            Quantity::from_u32(u32::from_le_bytes(q)),
            Some(Quantity::ImageJacobian)
        );
        let mut b = Body::new();
        assert_eq!(load(&mut b, &buf[..n]), Ok(2));
    }

    /// A file from another build must be refused outright, not partially interpreted.
    #[test]
    fn a_foreign_or_future_file_is_refused() {
        let mut b = Body::new();
        assert_eq!(load(&mut b, b"XXXX\0\0\0\0"), Err(LoadError::BadMagic));
    }
}
