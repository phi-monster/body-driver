//! What this body still has to measure about itself, and in what order.
//!
//! # What was missing without this file
//!
//! There were probes and there was an executor, and nothing in between. Every rig had to know, by
//! hand, which quantities to probe, in which order, and when one had gone bad. That is the same
//! shape as the hand-welded gates [`crate::refuse`] replaced — it works exactly as well as somebody
//! remembering to do it, and this project recorded **seven** cases in one night of an apparatus
//! that was never actually built while every reading was green.
//!
//! Plugging in a new machine is now: `plan()` → run the probes it names → `submit()` each → repeat
//! until `is_ready()`. Nothing about the order is typed in per robot.
//!
//! # 🔴 The prerequisite table is not a per-robot constant
//!
//! [`prerequisites`] says *which quantity is defined against which* — the hand point is a pixel in
//! the camera's frame, so it cannot be measured before the image Jacobian exists; a contact
//! threshold on a joint signal carries the gravity load, so it cannot be measured before the arm's
//! weight. Those are facts about the **quantities**, not about any body. The test is the one the
//! rest of the layer uses: *move to another robot and not one line changes.*
//!
//! # The cascade is the part that cannot be left to a person
//!
//! Re-measuring the image Jacobian invalidates everything measured against it — the hand point, the
//! gripper span, the occlusion map — **even though none of their own clocks moved**. So a plan that
//! re-measures the Jacobian schedules those too, before they have gone bad, rather than reporting
//! them as fine now and broken later. A rule enforced in one place everything passes through is a
//! rule; a rule everybody is supposed to remember is not.

use crate::measurement::Quantity;
use crate::Body;

/// Why a quantity is on the plan. Each is a distinct fact and they are never merged: "I have never
/// measured this" and "what I measured this against has moved" call for the same probe but mean
/// very different things in an audit trail.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Need {
    /// No value on this body yet.
    NeverMeasured = 0,
    /// Older than its declared validity window.
    Stale = 1,
    /// Something it was measured against has been re-measured, or is about to be.
    DependencyMoved = 2,
    /// Its own self-test does not pass right now.
    SelfTestFailed = 3,
}

impl Need {
    /// Stable human-readable name. For logs and audit trails; never parsed.
    pub fn as_str(self) -> &'static str {
        match self {
            Need::NeverMeasured => "never_measured",
            Need::Stale => "stale",
            Need::DependencyMoved => "dependency_moved",
            Need::SelfTestFailed => "selftest_failed",
        }
    }
}

/// Which quantities must already hold before this one can be measured at all.
///
/// Structural, not physical: it records what a quantity is **expressed in terms of**. Every entry
/// is justified in one clause, and an entry that cannot be justified in one clause is a coupling
/// somebody assumed rather than a dependency that exists.
pub fn prerequisites(q: Quantity) -> &'static [Quantity] {
    use Quantity::*;
    match q {
        // 原位是**这具身体自己**的一个位形:上电归位后读一次就有了。
        // 🔴 它不需要相机、不需要力、不需要任何别的量 —— 空前置是**结论**,不是遗漏:
        //    给它挂一个前置(比如 ImageJacobian),就等于说"没有相机就不知道自己在哪",
        //    而那对一具有关节编码器的身体是假的,并且会让一台没相机的机器永远回不了家。
        HomePose => &[],
        // A hand point is a pixel in the camera's frame; without the Jacobian there is no frame to
        // express it in and no way to tell a hand from an elbow by how it responds.
        HandPixel => &[ImageJacobian],
        // The span is read off the image and converted with a ruler derived from the Jacobian.
        GripperSpan => &[ImageJacobian],
        // The occlusion map is a map of the camera's frame.
        SelfOcclusion => &[ImageJacobian],
        // Any contact signal a joint can produce has the gravity load in it. Measure the hold
        // torque first or the threshold is a statement about the arm's own weight.
        ContactThreshold => &[ArmWeight],
        // The arc the working point sweeps is read in the camera's frame and converted with the
        // same ruler the span uses.
        ToolOffset | ToolAxisColumn => &[ImageJacobian],
        // 🔴 The floor is read as a stop in the delivered-motion signal, so it inherits that
        // ruler's dependency chain -- re-measure the contact threshold and the floor map built on
        // top of it is no longer trustworthy, automatically, without anyone remembering.
        Floor => &[ContactThreshold, StepDelivery],
        // The rest are measured directly off commanded motion and answer to nothing else.
        ImageJacobian | ArmWeight | Latency | Backlash | Reach | StepDelivery => &[],
    }
}

/// An ordered list of what to measure now.
#[derive(Copy, Clone, Debug)]
pub struct Plan {
    /// Quantities to probe, dependencies first.
    pub order: [(Quantity, Need); Quantity::COUNT],
    /// Used length of `order`.
    pub n: usize,
}

impl Plan {
    /// The quantities and reasons, in order.
    pub fn steps(&self) -> &[(Quantity, Need)] {
        &self.order[..self.n]
    }
}

/// Does this quantity need (re-)measuring right now, judged only from what is stored?
///
/// This is the same set of conditions [`crate::refuse::admit`] refuses on, asked ahead of time
/// instead of at the moment of use. Deliberately the same list: a scheduler that used a different
/// rule from the gate would leave a body that plans as ready and refuses in service.
fn direct_need(body: &Body, q: Quantity, now_ns: u64) -> Option<Need> {
    let m = body.get(q)?;
    if !m.selftest_passed {
        return Some(Need::SelfTestFailed);
    }
    if m.is_stale(now_ns) {
        return Some(Need::Stale);
    }
    for dep in m.deps.iter().flatten() {
        let (dq, epoch_at_measure) = *dep;
        match body.get(dq) {
            None => return Some(Need::DependencyMoved),
            Some(dm) if dm.epoch != epoch_at_measure => return Some(Need::DependencyMoved),
            Some(_) => {}
        }
    }
    None
}

/// Everything this body still owes itself, ordered so each probe's prerequisites come first.
///
/// An empty plan means every quantity this layer knows how to measure is currently valid. It does
/// **not** mean the body carries no hand-set constants — see [`crate::debt`], which counts the ones
/// that never came near this API and are therefore invisible to everything here.
pub fn plan(body: &Body, now_ns: u64) -> Plan {
    let mut need: [Option<Need>; Quantity::COUNT] = [None; Quantity::COUNT];
    for i in 0..Quantity::COUNT {
        let Some(q) = Quantity::from_u32(i as u32) else {
            continue;
        };
        need[i] = match body.get(q) {
            None => Some(Need::NeverMeasured),
            Some(_) => direct_need(body, q, now_ns),
        };
    }

    // 🔴 The cascade, as a fixpoint. If a prerequisite is going to be re-measured, everything
    // expressed in terms of it is going to be invalid the moment that happens — so it is scheduled
    // now, while the plan is being made, not discovered later by whoever happens to call `admit`.
    // At most COUNT passes: each pass either marks something new or the set has closed.
    for _ in 0..Quantity::COUNT {
        let mut changed = false;
        for i in 0..Quantity::COUNT {
            if need[i].is_some() {
                continue;
            }
            let Some(q) = Quantity::from_u32(i as u32) else {
                continue;
            };
            if prerequisites(q).iter().any(|p| need[*p as usize].is_some()) {
                need[i] = Some(Need::DependencyMoved);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    // Topological emit: a quantity may go out once every prerequisite that is also on the plan has
    // already gone out. Prerequisites that are fine and not on the plan impose no ordering.
    let mut placed = [false; Quantity::COUNT];
    let mut order = [(Quantity::HandPixel, Need::NeverMeasured); Quantity::COUNT];
    let mut n = 0usize;
    for _ in 0..Quantity::COUNT {
        let mut progressed = false;
        for i in 0..Quantity::COUNT {
            let Some(why) = need[i] else { continue };
            if placed[i] {
                continue;
            }
            let Some(q) = Quantity::from_u32(i as u32) else {
                continue;
            };
            let ready = prerequisites(q)
                .iter()
                .all(|p| need[*p as usize].is_none() || placed[*p as usize]);
            if ready {
                order[n] = (q, why);
                n += 1;
                placed[i] = true;
                progressed = true;
            }
        }
        if !progressed {
            // Unreachable while `prerequisites` is a DAG, and it is one by construction. Stopping
            // rather than looping means a table edited into a cycle produces a SHORT plan, which is
            // visible, instead of a hang in the layer that answers "may this body move".
            break;
        }
    }
    Plan { order, n }
}

/// The next single thing to measure, or `None` if this body is currently complete.
pub fn next(body: &Body, now_ns: u64) -> Option<(Quantity, Need)> {
    let p = plan(body, now_ns);
    if p.n == 0 {
        None
    } else {
        Some(p.order[0])
    }
}

/// Is every quantity this layer knows how to measure currently valid on this body?
///
/// 🔴 Read the name narrowly. It answers "has the measuring half finished", not "is this body free
/// of hand-set constants" — those are different questions and [`crate::debt`] answers the second.
pub fn is_ready(body: &Body, now_ns: u64) -> bool {
    plan(body, now_ns).n == 0
}
