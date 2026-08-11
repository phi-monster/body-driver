//! The thin OS's memory: a **compacting context**, bounded by construction, with no allocator.
//!
//! > owner, 2026-08-11: *"我们的记忆不是长期记忆吧,是类似于 claude code 的上下文 compact 这种"*
//!
//! That framing decides the shape. This is not a database of everything the robot has ever seen.
//! It is the working set a long task needs, kept small on purpose, rewritten as the task runs.
//!
//! # Why this is Rust and not the 529-line Python it replaces
//!
//! The Python version was an experiment and read like one: no tests, no ABI, callable only from
//! Python, and living inside a driver whose whole claim is *anybody's mind + anybody's body*. A
//! memory layer another company is meant to adopt cannot be reachable from exactly one language.
//! The concepts below are the ones that experiment **measured**; the implementation is new.
//!
//! # The one idea: classify by HOW FAST IT GOES STALE
//!
//! | rung | example | dies when | owner |
//! |---|---|---|---|
//! | this frame | where the moving cup is *now* | next frame | **not stored — look again** |
//! | this task | what I am doing, what I have done | the task ends | this module |
//! | this place | the bin is in that corner | you leave the place | this module |
//! | this body | the fingertip is 0.1451 m from the flange | the tool changes | [`crate::measurement`] |
//! | the world | knives are held by the handle | never | the weights |
//!
//! 🔴 **Rung 1 is about the OBJECT, not about positions.** An earlier draft of this rule read
//! "never store a position", which is right on a conveyor — where everything moves — and backwards
//! in a living room, where the sofa and the bin are the most durable facts in the task. The general
//! form, and what [`Durability`] encodes, is **does the thing move by itself**.
//!
//! # What is structural here, and why each one had to be
//!
//! Every rule below was first written as a comment, and each one failed as a comment:
//!
//! * **A perishable fact cannot be stored.** Not "should not": [`Memory::write`] refuses it. In the
//!   Python version a perishable fact *could* be written into a note, and once written it was
//!   indistinguishable from a durable one — so driving on a stale position became a discipline
//!   problem, and discipline is what fails at 3am.
//! * **Pinning is mechanical.** The first version pinned the first-seen object when the model
//!   moved the task out of its "observing" phase — and the model never updated that field, so the
//!   pin never engaged and the protection was decorative. *A guard that only fires when the thing
//!   it guards against cooperates is not a guard.* Here a pinning slot pins when the observation
//!   COUNTER advances, which nothing in the model can decline to do.
//! * **"I could not read the reply" is not "not yet".** One says the world is not ready; the other
//!   says the channel failed. Collapsing them made an unparseable answer look like patience. And
//!   an unreadable update must never CLEAR what it failed to update — a compaction that could not
//!   be parsed silently dropping the history it was compacting is the same bug with a bigger blast
//!   radius.
//! * **Bounded by construction.** Fixed slots, fixed bytes, no allocator — the same discipline as
//!   the rest of this crate, for the same reason: a hard-real-time layer must not depend on one.

use crate::measurement::Quantity;
use crate::refuse::{Reason, Verdict};

/// Bytes one slot can hold. A slot is a short human-readable fact ("a green cup"), not a document.
pub const SLOT_BYTES: usize = 64;
/// How many named slots one memory has. Fixed: the bound IS the compaction.
pub const MAX_SLOTS: usize = 8;
/// Bytes of the place fingerprint, matching the body fingerprint's 16 hex characters.
pub const FINGERPRINT_BYTES: usize = 16;

/// Does this fact move by itself?
///
/// The question that decides whether something may be remembered at all.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Durability {
    /// The thing this describes moves on its own: a cup on a belt, a person, a rolling ball.
    /// **Refused by [`Memory::write`].** Look again instead — that is what rung 1 means.
    Perishable,
    /// The thing does not move unless something moves it: the bin's corner, which object was first,
    /// what the task is. Storable.
    Durable,
}

/// What one observation reported.
///
/// 🔴 Four outcomes, and the last two are the ones a two-state design loses.
#[derive(Clone, Debug)]
pub enum Update<'a> {
    /// A durable fact, to be written into the named slot.
    Fact { slot: &'a str, value: &'a str },
    /// The world is not ready yet. Nothing to write; nothing is wrong.
    NotYet,
    /// 🔴 The reply could not be read. **Distinct from `NotYet`** — the channel failed, the world
    /// did not say "wait". Nothing is written and, critically, nothing is cleared.
    Unreadable,
    /// The observation is about a thing that moves by itself. Carried so the caller can act on it
    /// **this instant**, and refused by [`Memory::write`] so it cannot become a stored belief.
    Perishable { about: &'a str },
}

/// One named slot.
#[derive(Copy, Clone)]
struct Slot {
    name: [u8; SLOT_BYTES],
    name_len: usize,
    value: [u8; SLOT_BYTES],
    value_len: usize,
    /// Written once, then frozen on the next observation. See the module docs on mechanical pins.
    pins: bool,
    pinned: bool,
    /// The observation index at which this slot first got a value.
    first_written_at: Option<u64>,
}

impl Slot {
    const EMPTY: Slot = Slot {
        name: [0; SLOT_BYTES],
        name_len: 0,
        value: [0; SLOT_BYTES],
        value_len: 0,
        pins: false,
        pinned: false,
        first_written_at: None,
    };

    fn name(&self) -> &str {
        core::str::from_utf8(&self.name[..self.name_len]).unwrap_or("")
    }

    /// `None` when the slot has never been written — distinct from written-and-empty.
    pub fn value(&self) -> Option<&str> {
        if self.first_written_at.is_none() {
            return None;
        }
        core::str::from_utf8(&self.value[..self.value_len]).ok()
    }
}

/// What this memory is ABOUT, which decides when it dies.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Scope {
    /// Dies when the task ends. Walking through a door does not end it.
    Task,
    /// Dies when you leave the place. A new task does not end it.
    Place,
}

/// The three events that open a new memory — 🔴 **never a timer**.
///
/// The current stack collapses all three into "one episode, wipe everything", which is why walking
/// out of a room would also forget the errand.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Opens {
    /// A new task was given. Keeps: place memory (same room), body calibration.
    NewTask,
    /// The place was not recognised. Keeps: task memory, body calibration.
    UnrecognisedPlace,
    /// The body or tool changed. Keeps: both memories; it is the body layer that must re-measure.
    BodyChanged,
}

impl Opens {
    /// Does an event of this kind end a memory of that scope?
    ///
    /// 🔴 The table, as code, because as prose it was wrong in the deployed system: everything was
    /// wiped per episode.
    pub fn ends(self, scope: Scope) -> bool {
        match (self, scope) {
            (Opens::NewTask, Scope::Task) => true,
            (Opens::NewTask, Scope::Place) => false,
            (Opens::UnrecognisedPlace, Scope::Place) => true,
            (Opens::UnrecognisedPlace, Scope::Task) => false,
            // A tool change invalidates the BODY's numbers, not what the robot was doing or where
            // it is. Wiping the errand because somebody swapped a gripper is a category error.
            (Opens::BodyChanged, _) => false,
        }
    }
}

/// A place, keyed the way a body is keyed.
///
/// `bl_body` already solved this problem: one calibration per BODY, under a fingerprint computed
/// from what the body itself reports, containing no benchmark name and no task name. A place takes
/// the same shape — one memory per PLACE, keyed by what the place itself looks like.
///
/// 🔴 **And it must be able to refuse.** *"I do not know whether I have been here"* is a legitimate
/// third answer, and misidentifying a place is worse than having no memory at all: you would act on
/// a map of somewhere else, confidently.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct PlaceKey {
    bytes: [u8; FINGERPRINT_BYTES],
    /// How sure the recogniser was, 0..=1.
    confidence: f64,
}

/// Below this, a place is NOT recognised and a fresh memory is opened. Above it and below
/// [`PLACE_CERTAIN`], the recogniser abstains rather than guess.
pub const PLACE_UNKNOWN: f64 = 0.30;
/// At or above this, a place is the same place.
pub const PLACE_CERTAIN: f64 = 0.80;

/// What the recogniser concluded about where we are.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Recognised {
    /// Same place: carry on with the map we had.
    Same,
    /// Never seen: open an empty memory and fill it as we go. Correct outdoors, where every place
    /// is new.
    New,
    /// 🔴 **Cannot tell.** Not a failure — the honest answer, and the one that must not be coerced
    /// into either of the others.
    Unsure,
}

impl PlaceKey {
    pub fn new(bytes: [u8; FINGERPRINT_BYTES], confidence: f64) -> Self {
        PlaceKey { bytes, confidence }
    }

    pub fn matches(&self, other: &PlaceKey) -> Recognised {
        let c = self.confidence.min(other.confidence);
        if !c.is_finite() || c < PLACE_UNKNOWN {
            return Recognised::New;
        }
        if c < PLACE_CERTAIN {
            return Recognised::Unsure;
        }
        if self.bytes == other.bytes {
            Recognised::Same
        } else {
            Recognised::New
        }
    }
}

/// One memory: a fixed set of named slots, rewritten as observations arrive.
pub struct Memory {
    scope: Scope,
    slots: [Slot; MAX_SLOTS],
    n_slots: usize,
    /// Observations seen. The clock the mechanical pin runs on.
    pub observations: u64,
    /// Updates that could not be read. Surfaced because a channel failing quietly looks like a
    /// world that is merely slow.
    pub unreadable: u64,
    /// Perishable facts this memory refused to store. Not an error count — the mechanism working.
    pub refused_perishable: u64,
}

impl Memory {
    /// An empty memory of the given scope, with no slots declared yet.
    pub fn new(scope: Scope) -> Self {
        Memory {
            scope,
            slots: [Slot::EMPTY; MAX_SLOTS],
            n_slots: 0,
            observations: 0,
            unreadable: 0,
            refused_perishable: 0,
        }
    }

    pub fn scope(&self) -> Scope {
        self.scope
    }

    /// Declare a named slot. `pins` freezes it one observation after it is first written.
    ///
    /// Returns a refusal rather than silently dropping the slot when the memory is full: the bound
    /// is the point, and a caller that exceeds it needs to know which fact fell off the end.
    pub fn declare(&mut self, name: &str, pins: bool) -> Result<(), Verdict> {
        if name.is_empty() || name.len() > SLOT_BYTES {
            return Err(Verdict::refuse(Reason::OutOfRange, Quantity::HandPixel));
        }
        if self.find(name).is_some() {
            return Err(Verdict::refuse(Reason::DependencyChanged, Quantity::HandPixel));
        }
        if self.n_slots >= MAX_SLOTS {
            return Err(Verdict::refuse(Reason::RateLimit, Quantity::HandPixel));
        }
        let s = &mut self.slots[self.n_slots];
        *s = Slot::EMPTY;
        s.name[..name.len()].copy_from_slice(name.as_bytes());
        s.name_len = name.len();
        s.pins = pins;
        self.n_slots += 1;
        Ok(())
    }

    fn find(&self, name: &str) -> Option<usize> {
        (0..self.n_slots).find(|&i| self.slots[i].name() == name)
    }

    /// Read a slot. `None` if it does not exist or has never been written.
    pub fn get(&self, name: &str) -> Option<&str> {
        self.find(name).and_then(|i| self.slots[i].value())
    }

    pub fn is_pinned(&self, name: &str) -> bool {
        self.find(name).is_some_and(|i| self.slots[i].pinned)
    }

    /// 🔴 THE MECHANICAL PIN. Advance the observation counter and freeze anything that was written
    /// before now.
    ///
    /// Nothing in the model can decline to do this, which is the entire design: the previous
    /// version pinned on a state transition the model was supposed to perform and never did, so
    /// the protection existed only on paper for as long as it was believed to be working.
    pub fn observed(&mut self) {
        self.observations += 1;
        let now = self.observations;
        for i in 0..self.n_slots {
            let s = &mut self.slots[i];
            if s.pins && !s.pinned {
                if let Some(at) = s.first_written_at {
                    if now > at {
                        // By now the eye has seen a different picture. "The first one" is history,
                        // and any later correction would be overwriting a memory it can no longer
                        // check against the world.
                        s.pinned = true;
                    }
                }
            }
        }
    }

    /// Apply one observation's report.
    ///
    /// Returns `Ok(true)` when something was written, `Ok(false)` when the report correctly wrote
    /// nothing (not-yet, unreadable), and a refusal when the report tried to store something that
    /// must not be stored.
    pub fn apply(&mut self, u: &Update<'_>) -> Result<bool, Verdict> {
        match *u {
            Update::NotYet => Ok(false),
            Update::Unreadable => {
                // Counted, and NOTHING is cleared. An unreadable reply is not evidence that the
                // previous answer stopped being true.
                self.unreadable += 1;
                Ok(false)
            }
            Update::Perishable { .. } => {
                self.refused_perishable += 1;
                Err(Verdict::refuse(Reason::NotYet, Quantity::HandPixel))
            }
            Update::Fact { slot, value } => self.write(slot, value, Durability::Durable),
        }
    }

    /// Write a fact into a named slot.
    ///
    /// Refuses: a perishable fact (rung 1 — look again instead), a pinned slot, an unknown slot,
    /// and a value longer than a slot holds.
    pub fn write(&mut self, name: &str, value: &str, d: Durability) -> Result<bool, Verdict> {
        if d == Durability::Perishable {
            // Structural, not advisory. This is the rule the Python version could only state.
            self.refused_perishable += 1;
            return Err(Verdict::refuse(Reason::NotYet, Quantity::HandPixel));
        }
        let Some(i) = self.find(name) else {
            return Err(Verdict::refuse(Reason::NeverMeasured, Quantity::HandPixel));
        };
        if self.slots[i].pinned {
            return Err(Verdict::refuse(Reason::DependencyChanged, Quantity::HandPixel));
        }
        if value.len() > SLOT_BYTES {
            // Truncating here would produce a fact that reads as complete. Refuse and let the
            // caller shorten it.
            return Err(Verdict::refuse(Reason::OutOfRange, Quantity::HandPixel));
        }
        let now = self.observations;
        let s = &mut self.slots[i];
        s.value[..value.len()].copy_from_slice(value.as_bytes());
        s.value_len = value.len();
        if s.first_written_at.is_none() {
            s.first_written_at = Some(now);
        }
        Ok(true)
    }

    /// Apply a memory-opening event: keep what it keeps, clear what it ends.
    ///
    /// Returns whether this memory was cleared.
    pub fn on_event(&mut self, e: Opens) -> bool {
        if !e.ends(self.scope) {
            return false;
        }
        let scope = self.scope;
        let names: [([u8; SLOT_BYTES], usize, bool); MAX_SLOTS] = core::array::from_fn(|i| {
            (self.slots[i].name, self.slots[i].name_len, self.slots[i].pins)
        });
        let n = self.n_slots;
        *self = Memory::new(scope);
        // The DECLARATIONS survive; only what was learned is cleared. A new errand in the same
        // room does not change which facts the task needs a place for.
        for (name, len, pins) in names.iter().take(n) {
            let s = &mut self.slots[self.n_slots];
            *s = Slot::EMPTY;
            s.name = *name;
            s.name_len = *len;
            s.pins = *pins;
            self.n_slots += 1;
        }
        true
    }

    /// How many declared slots hold a value. The compaction pressure, readable without a dump.
    pub fn filled(&self) -> usize {
        (0..self.n_slots)
            .filter(|&i| self.slots[i].value().is_some())
            .count()
    }

    pub fn declared(&self) -> usize {
        self.n_slots
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn task_memory() -> Memory {
        let mut m = Memory::new(Scope::Task);
        m.declare("first_object", true).unwrap();
        m.declare("already_done", false).unwrap();
        m
    }

    /// 🔴 The pin must not depend on the model doing anything. The first implementation pinned on
    /// a phase transition the model never performed, so the protection was decorative for as long
    /// as anybody believed in it.
    #[test]
    fn the_pin_fires_on_the_observation_counter_not_on_cooperation() {
        let mut m = task_memory();
        m.observed();
        m.write("first_object", "a green cup", Durability::Durable).unwrap();
        assert!(!m.is_pinned("first_object"), "not yet — it was written THIS observation");

        m.observed(); // the eye has now seen a different picture
        assert!(m.is_pinned("first_object"), "the counter advanced; nothing had to agree");

        let v = m.write("first_object", "a red cup", Durability::Durable).unwrap_err();
        assert!(!v.admit);
        assert_eq!(m.get("first_object"), Some("a green cup"), "history must not be rewritten");
    }

    /// Rung 1: a fact about something that moves by itself cannot be stored at all.
    #[test]
    fn a_perishable_fact_cannot_be_stored() {
        let mut m = task_memory();
        m.declare("where_it_is", false).unwrap();
        let v = m.write("where_it_is", "left edge", Durability::Perishable).unwrap_err();
        assert!(!v.admit);
        assert_eq!(m.get("where_it_is"), None);
        assert_eq!(m.refused_perishable, 1);

        // ... and through the observation path too, which is where it would actually arrive.
        let u = Update::Perishable { about: "the cup on the belt" };
        assert!(m.apply(&u).is_err());
        assert_eq!(m.refused_perishable, 2);
    }

    /// "I could not read it" and "not yet" are different facts, and NEITHER may clear a memory.
    #[test]
    fn an_unreadable_reply_is_not_a_not_yet_and_clears_nothing() {
        let mut m = task_memory();
        m.observed();
        m.write("first_object", "a green cup", Durability::Durable).unwrap();

        assert_eq!(m.apply(&Update::NotYet).unwrap(), false);
        assert_eq!(m.unreadable, 0, "patience is not a channel failure");

        assert_eq!(m.apply(&Update::Unreadable).unwrap(), false);
        assert_eq!(m.unreadable, 1);
        assert_eq!(m.get("first_object"), Some("a green cup"), "an unreadable reply cleared it");
    }

    /// 🔴 The table that the deployed stack collapses into "wipe everything each episode".
    #[test]
    fn a_new_errand_in_the_same_room_keeps_the_room() {
        let mut task = Memory::new(Scope::Task);
        task.declare("goal", false).unwrap();
        task.write("goal", "tidy the living room", Durability::Durable).unwrap();

        let mut place = Memory::new(Scope::Place);
        place.declare("bin_corner", false).unwrap();
        place.write("bin_corner", "north-east, behind the sofa", Durability::Durable).unwrap();

        assert!(task.on_event(Opens::NewTask), "a new task ends the task memory");
        assert!(!place.on_event(Opens::NewTask), "... and must not end the place memory");
        assert_eq!(task.get("goal"), None);
        assert_eq!(place.get("bin_corner"), Some("north-east, behind the sofa"));

        // Walking through a door does not change the errand.
        let mut task2 = Memory::new(Scope::Task);
        task2.declare("goal", false).unwrap();
        task2.write("goal", "tidy the living room", Durability::Durable).unwrap();
        assert!(!task2.on_event(Opens::UnrecognisedPlace));
        assert_eq!(task2.get("goal"), Some("tidy the living room"));

        // Swapping a gripper invalidates the BODY's numbers, not what we were doing or where.
        assert!(!task2.on_event(Opens::BodyChanged));
        assert!(!place.on_event(Opens::BodyChanged));
    }

    /// Clearing a memory keeps its SHAPE: the declarations survive, only what was learned goes.
    #[test]
    fn clearing_keeps_the_declarations() {
        let mut m = task_memory();
        m.observed();
        m.write("first_object", "a green cup", Durability::Durable).unwrap();
        m.observed();
        assert!(m.is_pinned("first_object"));

        m.on_event(Opens::NewTask);
        assert_eq!(m.declared(), 2, "the slots a task needs did not change");
        assert_eq!(m.filled(), 0);
        assert!(!m.is_pinned("first_object"), "a cleared slot is writable again");
        m.observed();
        m.write("first_object", "a blue bowl", Durability::Durable).unwrap();
        assert_eq!(m.get("first_object"), Some("a blue bowl"));
    }

    /// 🔴 Misidentifying a place is worse than having no memory: you act on a map of somewhere
    /// else, confidently. So "cannot tell" must be its own answer.
    #[test]
    fn a_place_that_cannot_be_identified_says_so() {
        let here = PlaceKey::new([7; FINGERPRINT_BYTES], 0.95);
        let same = PlaceKey::new([7; FINGERPRINT_BYTES], 0.90);
        let other = PlaceKey::new([9; FINGERPRINT_BYTES], 0.95);
        let murky = PlaceKey::new([7; FINGERPRINT_BYTES], 0.55);
        let outdoors = PlaceKey::new([3; FINGERPRINT_BYTES], 0.05);

        assert_eq!(here.matches(&same), Recognised::Same);
        assert_eq!(here.matches(&other), Recognised::New);
        assert_eq!(here.matches(&murky), Recognised::Unsure, "identical bytes, but not sure");
        assert_eq!(here.matches(&outdoors), Recognised::New, "everything new is correct outdoors");
    }

    /// The bound is the compaction. Exceeding it must name what fell off, not drop it quietly.
    #[test]
    fn the_memory_is_bounded_and_says_so() {
        let mut m = Memory::new(Scope::Task);
        for i in 0..MAX_SLOTS {
            let name = ["s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7"][i];
            m.declare(name, false).unwrap();
        }
        assert!(m.declare("one_too_many", false).is_err());
        assert_eq!(m.declared(), MAX_SLOTS);

        // A value longer than a slot is refused rather than truncated: a truncated fact reads as a
        // complete one.
        let long = "x".repeat(SLOT_BYTES + 1);
        assert!(m.write("s0", &long, Durability::Durable).is_err());
        assert_eq!(m.get("s0"), None);
    }

    /// A slot that was never written and a slot written empty are different facts.
    #[test]
    fn never_written_is_not_empty() {
        let mut m = task_memory();
        assert_eq!(m.get("already_done"), None);
        m.write("already_done", "", Durability::Durable).unwrap();
        assert_eq!(m.get("already_done"), Some(""), "written-empty is a value");
        assert_eq!(m.filled(), 1);
    }

    /// Writing to a slot nobody declared is a refusal, not a new slot. The bound would otherwise
    /// be whatever the model felt like emitting.
    #[test]
    fn an_undeclared_slot_is_refused() {
        let mut m = task_memory();
        let v = m.write("improvised", "something", Durability::Durable).unwrap_err();
        assert_eq!(v.why, Reason::NeverMeasured);
        assert_eq!(m.declared(), 2);
    }
}
