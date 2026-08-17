//! What this body layer still **owes**: every hand-set constant found in the systems it is supposed
//! to replace, one row each, with its disposition.
//!
//! # Why this file exists, stated plainly
//!
//! [`crate::Body::hand_filled_constants`] returns `0`, and that number is true and misleading. It
//! counts constants that entered **through this API**, and nothing can enter through this API
//! without a passing self-test — so it is a structural zero. The constants that never came near the
//! API are invisible to it, and those are the ones that are actually running the robot.
//!
//! The proof is a measurement taken on 2026-08-09. A parameter search over the deployed teacher
//! found a **dominant** constant, `TEACH_HIGH_FRAC`: at `≤ 0.30` the task succeeded **32 of 44**
//! (73%), at `> 0.31` it succeeded **10 of 100** (10%), Fisher p = 9.3e-14. It is the single largest
//! effect anybody has measured on that stack, and this layer **had never heard of it**. A census of
//! the same two files then found **45 environment knobs and a hardcoded camera matrix**, against
//! ten declared quantities.
//!
//! ⇒ *"we removed the hand-filled constants"* was not true. What was true is *"we removed the ones
//! we thought of, and the biggest one was found for us by a search."* This file is the correction:
//! the layer now states its debt as a number it can be held to, instead of a zero that flatters it.
//!
//! # 🔴 What `Measured` does and does not claim
//!
//! [`Standing::Measured`] means **a probe in this crate produces that quantity today**, so the
//! downstream constant is replaceable. It does **not** mean it has been replaced. Nothing outside
//! this directory reads this layer yet — that is the largest open item and it is deliberately not
//! hidden inside a per-row status. Do not read `outstanding() == N` as "N constants left in the
//! stack"; read it as "N constants this layer could not supply even if it were wired in".
//!
//! # Citation discipline
//!
//! The `name` field is the anchor and it is greppable; `site` carries a line number, and line
//! numbers drift the moment anything is inserted above them. A drifted line number is worse than a
//! broken one — it points at a real line with different content, and nothing reports an
//! inconsistency. Grep the name.

use crate::measurement::Quantity;

/// What this layer can do about a constant somebody set by hand.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Standing {
    /// A body constant, and a probe in this crate measures it. **Replaceable, not replaced.**
    Measured(Quantity),
    /// A body constant with a slot in [`Quantity`] and no estimator behind it. A named slot in an
    /// enum is not a probe; this is the worst standing to be in, because it reads as covered.
    DeclaredOnly(Quantity),
    /// 🔴 A derivation for this constant EXISTS in this crate, and it refuses on this body because
    /// the quantity named here is unavailable. Distinct from [`Standing::DeclaredOnly`], which is
    /// about a missing estimator, and distinct from [`Standing::Outstanding`], which is about a
    /// constant this layer cannot address at all.
    ///
    /// This is the standing that makes the ledger useful to an integrator rather than to an
    /// auditor: it answers *"you cannot have this number yet, and here is the one probe that would
    /// give it to you"* instead of leaving them to discover the gap on their robot.
    BlockedBy(Quantity),
    /// A body constant with no slot at all. Hand-set downstream and invisible to everything here.
    Outstanding,
    /// Not a body constant: it describes the task, the world, or the harness. Recorded anyway, and
    /// with its reason, so the classification can be disputed **per row** rather than trusted as a
    /// total — the total is the only part somebody would otherwise have to take on faith.
    NotABodyConstant,
}

/// One constant somebody set by hand, and what this layer can do about it.
#[derive(Copy, Clone, Debug)]
pub struct Constant {
    /// The name as it appears where it is set. Greppable, and the anchor for the citation.
    pub name: &'static str,
    /// Where it is set. Line numbers drift; the name is the anchor.
    pub site: &'static str,
    /// What it currently carries.
    pub value: &'static str,
    /// What this layer can do about it.
    pub standing: Standing,
    /// One line: why this classification, and what would discharge it.
    pub note: &'static str,
}

/// Shorthand for the deployed teacher and executor, both loaded into the same process.
const TEACH: &str = "shot1/scripts/rteach/teach_deploy.py";
const EXEC: &str = "shot1/scripts/rl/uni_exec_deploy.py";
pub const SELF: &str = "abi/body_layer.h";

/// Every hand-set constant found in the systems this layer is meant to replace, plus this layer's
/// own. Complete on purpose: an incomplete ledger is a ledger whose total means nothing.
pub const LEDGER: &[Constant] = &[
    // ---------------------------------------------------------------- the dominant one
    Constant {
        name: "TEACH_HIGH_FRAC",
        site: TEACH,
        value: "0.30",
        standing: Standing::Outstanding,
        note: "🔴 THE LARGEST MEASURED EFFECT ON THIS STACK AND THIS LAYER HAD NO SLOT FOR IT. \
               Shifts the grasp point by HIGH_FRAC x (object long-axis extent) toward the upper \
               end. <=0.30: 32/44 = 73%. >0.31: 10/100 = 10%. Fisher p=9.3e-14 (owner, remote box \
               2026-08-09; the sweep is not in this repository, only the finding). It exists \
               because the jaws (~6 cm open) are wider than the object (2.05 cm) and a support \
               block sits alongside, so a centred grasp closes on the block. DISCHARGE TEST: with \
               gripper_span and tool_offset measured, derive the clearance the jaws need and \
               re-run the sweep -- if the HIGH_FRAC effect disappears it was a body constant in \
               disguise; if it survives, it is a world constant and belongs to the model, not \
               here. NOT declared as a Quantity on purpose: whether a grasp 30% up an object holds \
               depends on the OBJECT, so it has not been shown to be measurable off the body, and \
               a slot with no probe reads as covered while being worth nothing.",
    },
    // ---------------------------------------------------------------- the tool offset, x4
    Constant {
        name: "L3_GRIPPER_BIAS",
        site: EXEC,
        value: "0.145",
        standing: Standing::Measured(Quantity::ToolOffset),
        note: "🔴 Tool-axis offset, copied by hand from Assets/Robots/<body>/robot_config.yml. The \
               comment beside it: 'x5 = 0.145, franka = 0.102' -- 4.3 cm apart, with 0.145 as the \
               DEFAULT, so a body that forgets to pass it silently runs on another robot's \
               geometry. This is the exact number measurement.rs cites as the reason a bare f64 \
               cannot be refused on, still live in the deployed servo.",
    },
    Constant {
        name: "flange_for / tcp = flange + 0.145 * R[:,0]",
        site: TEACH,
        value: "0.145",
        standing: Standing::Measured(Quantity::ToolOffset),
        note: "The same offset again, hardcoded rather than read from the knob above -- so setting \
               L3_GRIPPER_BIAS for a new body fixes one of the two and leaves the other. Two copies \
               of one body constant is worse than one, because they can disagree silently.",
    },
    Constant {
        name: "TEACH_TILT_MAX",
        site: TEACH,
        value: "hand-set; the arithmetic behind it is the 0.145 tool offset",
        standing: Standing::BlockedBy(Quantity::ToolOffset),
        note: "NOT DISCHARGED, and deliberately not invented. The ceiling is where a tilted tool axis swings\
               the working point into a surface or out of the band, which needs the SURFACE as well as\
               tool_offset -- and a surface is not a body constant. Writing a plausible formula here would be\
               exactly the tuned-until-it-worked move this crate refuses. UNBLOCKED BY: stating the surface as\
               a task input, after which the tilt ceiling is arithmetic on tool_offset.",
    },
    Constant {
        name: "tcp_off",
        site: "qcontact/meta.json",
        value: "0.1034",
        standing: Standing::Measured(Quantity::ToolOffset),
        note: "The same quantity on a third rig, a third value, a third hand-set location.",
    },
    // ---------------------------------------------------------------- the camera matrix
    Constant {
        name: "FX",
        site: EXEC,
        value: "288.13",
        standing: Standing::Measured(Quantity::ImageJacobian),
        note: "🔴 A focal length, hardcoded under a comment reading 'FROZEN P1 rig; do not \
               re-derive'. The image Jacobian exists precisely so no intrinsic has to be written \
               down -- README: 'no intrinsics, no extrinsics, no hand-eye transform'. The claim is \
               true of this layer and false of the system it is meant to serve.",
    },
    Constant {
        name: "CX, CY",
        site: EXEC,
        value: "320.0, 240.0",
        standing: Standing::Measured(Quantity::ImageJacobian),
        note: "Principal point, same block. Same disposition as FX.",
    },
    Constant {
        name: "CAM_POS",
        site: EXEC,
        value: "[0.0, -0.41, 1.308]",
        standing: Standing::Measured(Quantity::ImageJacobian),
        note: "Camera extrinsic translation, hardcoded. Knock the camera and nothing notices -- \
               which is the category's admission test, failed.",
    },
    Constant {
        name: "CAM_EULER_DEG",
        site: EXEC,
        value: "[30.0, 0.0, 0.0]",
        standing: Standing::Measured(Quantity::ImageJacobian),
        note: "Camera extrinsic rotation, hardcoded. Same disposition as CAM_POS.",
    },
    Constant {
        name: "W, H",
        site: EXEC,
        value: "640, 480",
        standing: Standing::NotABodyConstant,
        note: "Frame size. A property of the stream, readable from the buffer, not of the body.",
    },
    // ---------------------------------------------------------------- gripper
    Constant {
        name: "TEACH_JAW_MAX",
        site: TEACH,
        value: "0.080",
        standing: Standing::Measured(Quantity::GripperSpan),
        note: "Its own comment calls it 'the VALIDATED jaw ceiling for this arm' -- i.e. per-body, \
               and validated by hand. gripper_span measures exactly this.",
    },
    Constant {
        name: "GRIP_CLOSED",
        site: TEACH,
        value: "0.2",
        standing: Standing::Measured(Quantity::GripperSpan),
        note: "The commanded opening that counts as closed on THIS gripper. Falls out of the same \
               sweep gripper_span already runs; nothing else has to be measured for it.",
    },
    Constant {
        name: "TEACH_GRIPCAL_OFFS",
        site: TEACH,
        value: "-0.12,-0.09,-0.06,-0.03,0,0.03,0.06",
        standing: Standing::NotABodyConstant,
        note: "The offsets of a jaw-closure calibration sweep -- a probe SCHEDULE, not a constant. \
               ⚠️ But note what it means: a gripper probe already exists, in the teacher's own \
               Python, writing gripcal_ep*.json. Same standing as step_delivery when it was added: \
               measured in the experiment's own code rather than through this ABI. Named debt.",
    },
    // ---------------------------------------------------------------- motion / timing
    Constant {
        name: "TEACH_STEP_M",
        site: TEACH,
        value: "derive::step_m -> step_delivery.valid_hi[0]",
        standing: Standing::Measured(Quantity::StepDelivery),
        note: "DISCHARGED 2026-08-11. The top of the domain the probe actually swept: the largest magnitude\
               this body is KNOWN to deliver. Past it the gate answers OutOfRange, so the step the caller takes\
               and the step the gate admits are the same number by construction.",
    },
    Constant {
        name: "TEACH_STEP_M_HOLD",
        site: TEACH,
        value: "0.02",
        standing: Standing::Outstanding,
        note: "The same, while holding an object. Same disposition as TEACH_STEP_M.",
    },
    Constant {
        name: "TEACH_STEP_DEG",
        site: TEACH,
        value: "hand-set angular step",
        standing: Standing::Outstanding,
        note: "NOT DISCHARGED. step_delivery was swept over LINEAR magnitudes only; there is no angular\
               equivalent, and reusing the linear number for degrees is a unit error that would still produce\
               plausible motion. UNBLOCKED BY: an angular step_delivery probe -- the same motion program,\
               commanded in rotation.",
    },
    Constant {
        name: "TEACH_SETTLE",
        site: TEACH,
        value: "derive::settle_periods -> latency + ln(tol)/ln(1-step_delivery)",
        standing: Standing::Measured(Quantity::StepDelivery),
        note: "DISCHARGED 2026-08-11. Latency periods pass before anything moves, then each period closes\
               step_delivery of what remains, so the budget is ln(tol)/ln(1-f) periods on top -- all measured\
               except the accuracy the TASK requires, which is not a body constant. This row is the one that\
               proves the file: two arms on one harness delivered 0.76 and 0.11 of the same 45 mm command; a\
               budget set from the first left the second 0.136 m short EVERY episode, and it read as a planner\
               or reachability fault while every scalar in the log looked ordinary. Sized from the arm's own\
               delivery: 0.136 m -> 0.0058 m, nothing about the robot changed.",
    },
    Constant {
        name: "TEACH_REHOME_STEPS",
        site: TEACH,
        value: "derive::traverse_steps -> distance / (step_m * delivered)",
        standing: Standing::Measured(Quantity::StepDelivery),
        note: "DISCHARGED 2026-08-11. Divided by what ARRIVES, not by what is commanded -- dividing by the\
               commanded step is the 0.11-delivery bug in its other form: the step count is right, the arm\
               stops short, and nothing reports an error. Same step_m the executor scales by, so a re-home and\
               a servo step can no longer disagree about how far this arm moves in one period.",
    },
    // ---------------------------------------------------------------- reach
    Constant {
        name: "BPD_REACH_BOX",
        site: TEACH,
        value: "\"\" (unset)",
        standing: Standing::Measured(Quantity::Reach),
        note: "A hand-drawn reachable box. This is Quantity::Reach typed in; the probe exists and \
               was validated against 2174 real episodes.",
    },
    Constant {
        name: "TEACH_REACH_JSON",
        site: TEACH,
        value: "/root/objlib/reach_ok.json",
        standing: Standing::Measured(Quantity::Reach),
        note: "A file of measured reachable cells -- the right instinct, the wrong home: it has no \
               uncertainty, no probed range, no dependency on the mounting it was measured at, so \
               nothing notices when the base moves. That is the whole argument for Measurement's \
               seven fields.",
    },
    Constant {
        name: "TEACH_APPROACH_H",
        site: TEACH,
        value: "derive::approach_clearance_m -> REFUSES: gripper_span NoResponse on this body",
        standing: Standing::BlockedBy(Quantity::GripperSpan),
        note: "NOT DISCHARGED, AND THE REFUSAL IS THE ANSWER. The derivation exists -- half the jaw span is the\
               geometric minimum for the fingers to straddle an object -- and it refuses here because\
               gripper_span came back NoResponse: the commanded opening did not move the observed signal. An\
               approach height invented while the jaw span is unknown is the constant that silently decides\
               whether a grasp closes on the object or on the table, and nothing else in the log would look\
               wrong. UNBLOCKED BY: a gripper_span probe that responds on this gripper.",
    },
    Constant {
        name: "TEACH_APPROACH_XY",
        site: TEACH,
        value: "same blocker as TEACH_APPROACH_H",
        standing: Standing::BlockedBy(Quantity::GripperSpan),
        note: "NOT DISCHARGED. Lateral standoff has the same blocker: without the jaw span there is no\
               body-derived answer to how far to one side the fingers must start. UNBLOCKED BY: the same\
               gripper_span probe.",
    },
    // ---------------------------------------------------------------- task / world / harness
    Constant {
        name: "TEACH_LIFT_H",
        site: TEACH,
        value: "0.07",
        standing: Standing::NotABodyConstant,
        note: "How high to carry. Set BELOW the task's own scoring predicate (is_lift(0.10)) -- a \
               property of the task, and its comment says so.",
    },
    Constant {
        name: "TEACH_LIFT_MIN",
        site: TEACH,
        value: "0.05",
        standing: Standing::NotABodyConstant,
        note: "Success criterion: physical proof the object came up. Scoring, not body.",
    },
    Constant {
        name: "TEACH_HOLD_TOL",
        site: TEACH,
        value: "0.06",
        standing: Standing::NotABodyConstant,
        note: "Scoring tolerance: the object must track the TCP while held.",
    },
    Constant {
        name: "TEACH_PLACE_TOL",
        site: TEACH,
        value: "0.08",
        standing: Standing::NotABodyConstant,
        note: "Scoring tolerance for placement.",
    },
    Constant {
        name: "TEACH_MOVE_MIN",
        site: TEACH,
        value: "0.05",
        standing: Standing::NotABodyConstant,
        note: "Scoring: it must actually be transported.",
    },
    Constant {
        name: "TEACH_PREGRASP_XY",
        site: TEACH,
        value: "0.06",
        standing: Standing::NotABodyConstant,
        note: "Waypoint arrival tolerance -- a controller tolerance, identical across bodies.",
    },
    Constant {
        name: "TEACH_ENV_STEP_LIM",
        site: TEACH,
        value: "300",
        standing: Standing::NotABodyConstant,
        note: "Episode step budget.",
    },
    Constant {
        name: "L3_MAX_STEPS",
        site: "TEACH=400 / EXEC=260",
        value: "400 and 260",
        standing: Standing::NotABodyConstant,
        note: "⚠️ ONE NAME, TWO DEFAULTS, both files loaded into the same process. Not a body \
               constant, but recorded because whichever module reads it last decides, and nothing \
               reports the disagreement.",
    },
    Constant {
        name: "TEACH_PROBE",
        site: TEACH,
        value: "\"\" (mode flag)",
        standing: Standing::NotABodyConstant,
        note: "Selects the probe routine.",
    },
    Constant {
        name: "TEACH_PROBE_TILT",
        site: TEACH,
        value: "0",
        standing: Standing::NotABodyConstant,
        note: "Probe routine option.",
    },
    Constant {
        name: "TEACH_PROBE_SIGN",
        site: TEACH,
        value: "1",
        standing: Standing::NotABodyConstant,
        note: "Probe routine option.",
    },
    Constant {
        name: "TEACH_PROBE_X0",
        site: TEACH,
        value: "-0.45",
        standing: Standing::NotABodyConstant,
        note: "Extent of the probe sweep -- i.e. the domain the result is valid over. The right \
               idea living in the wrong place: here it would be valid_lo, carried WITH the value.",
    },
    Constant {
        name: "TEACH_PROBE_X1",
        site: TEACH,
        value: "0.45",
        standing: Standing::NotABodyConstant,
        note: "As TEACH_PROBE_X0.",
    },
    Constant {
        name: "TEACH_PROBE_Y0",
        site: TEACH,
        value: "-0.24",
        standing: Standing::NotABodyConstant,
        note: "As TEACH_PROBE_X0.",
    },
    Constant {
        name: "TEACH_PROBE_Y1",
        site: TEACH,
        value: "0.02",
        standing: Standing::NotABodyConstant,
        note: "As TEACH_PROBE_X0.",
    },
    Constant {
        name: "TEACH_PROBE_Z",
        site: TEACH,
        value: "0.79",
        standing: Standing::NotABodyConstant,
        note: "Height of the probe sweep. Table height -- world, not body.",
    },
    Constant {
        name: "TEACH_GRIPCAL",
        site: TEACH,
        value: "\"\" (mode flag)",
        standing: Standing::NotABodyConstant,
        note: "Runs the jaw-closure sweep. See TEACH_GRIPCAL_OFFS.",
    },
    Constant {
        name: "TEACH_REPEAT",
        site: "shot1/scripts/objlib/mklayout.py",
        value: "1",
        standing: Standing::NotABodyConstant,
        note: "Layouts per (object, pose). Sampling, not body.",
    },
    Constant {
        name: "L3_RL_OUT",
        site: TEACH,
        value: "\".\"",
        standing: Standing::NotABodyConstant,
        note: "Output directory.",
    },
    Constant {
        name: "UNI_RD_DOMAIN",
        site: EXEC,
        value: "/root/l3/uni/rd",
        standing: Standing::NotABodyConstant,
        note: "Which domain's normalisation statistics de-normalise the executor output. A model \
               contract; wrong here fails loudly rather than quietly.",
    },
    Constant {
        name: "UNI_ZERO_W1",
        site: EXEC,
        value: "0",
        standing: Standing::NotABodyConstant,
        note: "Ablation flag.",
    },
    Constant {
        name: "L3_COND",
        site: EXEC,
        value: "probe",
        standing: Standing::NotABodyConstant,
        note: "Which experimental condition to run.",
    },
    Constant {
        name: "L3_CKPT",
        site: EXEC,
        value: "\"\"",
        standing: Standing::NotABodyConstant,
        note: "Checkpoint path.",
    },
    Constant {
        name: "L3_GAMMA",
        site: EXEC,
        value: "1.0",
        standing: Standing::NotABodyConstant,
        note: "Model sampling parameter.",
    },
    Constant {
        name: "L3_REL",
        site: EXEC,
        value: "0",
        standing: Standing::NotABodyConstant,
        note: "Action-frame flag (relative vs absolute). A contract, not a body property.",
    },
    Constant {
        name: "L3_DEV",
        site: EXEC,
        value: "cuda:0",
        standing: Standing::NotABodyConstant,
        note: "Compute device.",
    },
    Constant {
        name: "L3_OUT",
        site: EXEC,
        value: "/root/l3/p2/loop.jsonl",
        standing: Standing::NotABodyConstant,
        note: "Output path.",
    },
    Constant {
        name: "L3_DUMP",
        site: EXEC,
        value: "/root/l3/p2/loopdump",
        standing: Standing::NotABodyConstant,
        note: "Output path.",
    },
    Constant {
        name: "L3_SWAP_FRAC",
        site: EXEC,
        value: "0.35",
        standing: Standing::NotABodyConstant,
        note: "Experimental intervention schedule.",
    },
    Constant {
        name: "L3_SWAP_D",
        site: EXEC,
        value: "0.12",
        standing: Standing::NotABodyConstant,
        note: "Phase-matched swap trigger distance. Experimental design.",
    },
    Constant {
        name: "L3_DDIM",
        site: EXEC,
        value: "8",
        standing: Standing::NotABodyConstant,
        note: "Diffusion sampling steps.",
    },
    Constant {
        name: "L3_EXEC_STEPS",
        site: EXEC,
        value: "4",
        standing: Standing::NotABodyConstant,
        note: "Steps of each 16-step chunk executed before re-planning. Its own comment names it \
               'the train<->eval contract knob CLAUDE.md flags (the 44%->63% chunk_size lesson)'. \
               A contract, not a body property -- but the body half of it, dead time, is \
               Quantity::Latency and is measured here.",
    },
    // ---------------------------------------------------------------- this layer's OWN
    Constant {
        name: "bl_spec.step_m",
        site: SELF,
        value: "Spec::from_body -> step_delivery.valid_hi[0]",
        standing: Standing::Measured(Quantity::StepDelivery),
        note: "DISCHARGED 2026-08-11. Was caller-supplied 'from the machine's rating', with every \
               command scaled by it and no probe producing it. Now taken from the TOP OF THE \
               DOMAIN step_delivery was actually swept over: the probe commanded a range of \
               magnitudes and measured what came back, so the largest validated magnitude IS the \
               largest step this body is known to deliver. Commanding past it is OutOfRange by \
               this layer's own rule, so the scaler and the gate now agree by construction rather \
               than by somebody keeping them in sync. Missing or domain-less => REFUSE, never a \
               default: a default here would be the same hand-filled constant wearing a \
               function's name.",
    },
    Constant {
        name: "bl_spec.damping",
        site: SELF,
        value: "Spec::from_body -> image_jacobian.worst_uncertainty()",
        standing: Standing::Measured(Quantity::ImageJacobian),
        note: "DISCHARGED 2026-08-11. Was 'from the measured Jacobian's own conditioning' with \
               nothing computing it -- a promise kept by a comment. Now the Jacobian's OWN WORST \
               UNCERTAINTY: damped least squares trades tracking for stability, and the honest \
               amount to trade is how badly the Jacobian is known -- damp lightly when it is \
               sharp, heavily when it is not. A Jacobian reporting ZERO uncertainty is refused \
               rather than believed: that is not a sharp Jacobian, it is one whose uncertainty was \
               never established, and damping by zero is precisely the tuned-until-it-worked \
               choice this struct's own doc comment forbids.",
    },
    Constant {
        name: "bl_spec.n_joints",
        site: SELF,
        value: "caller-supplied",
        standing: Standing::Outstanding,
        note: "Discoverable by commanding each joint in turn -- image_jacobian already REFUSES a \
               joint that was never commanded -- but nothing derives it, so it is typed in.",
    },
    Constant {
        name: "bl_spec.period_ms",
        site: SELF,
        value: "caller-supplied",
        standing: Standing::NotABodyConstant,
        note: "The controller's own clock, not a property of the body.",
    },
    Constant {
        name: "hand::Config::min_separation",
        site: "slow/src/hand.rs",
        value: "1.50",
        standing: Standing::NotABodyConstant,
        note: "Claimed as an observability threshold, identical on every robot. The claim is \
               falsifiable and the evidence is stated: the fingertip/elbow gain ratio that fooled \
               the old selector was 1.11, so anything above it turns a silent mis-pick into a \
               visible refusal. Recorded here so the claim can be disputed rather than assumed.",
    },
    Constant {
        name: "hand::Config::min_pixels / min_rigidity / decay_per_step / max_uncertainty",
        site: "slow/src/hand.rs",
        value: "40 / 0.60 / 0.0025 / 0.030",
        standing: Standing::NotABodyConstant,
        note: "Same claim, weaker evidence: these are 'can this be read off this image at all' \
               thresholds. If moving to another robot ever requires changing one, the claim is \
               false and it becomes a body constant. Nothing checks that today.",
    },
];

/// Every row in the ledger.
pub fn total() -> usize {
    LEDGER.len()
}

/// Rows classified as body constants, whatever their standing.
pub fn body_constants() -> usize {
    LEDGER
        .iter()
        .filter(|c| !matches!(c.standing, Standing::NotABodyConstant))
        .count()
}

/// Body constants this layer **cannot** supply today: no probe, or no slot at all.
///
/// 🔴 This is the honest counterpart to [`crate::Body::hand_filled_constants`], which is a
/// structural zero. Report both or report neither.
pub fn outstanding() -> usize {
    LEDGER
        .iter()
        .filter(|c| matches!(c.standing, Standing::Outstanding | Standing::DeclaredOnly(_)))
        .count()
}

/// Body constants with a slot in [`Quantity`] and no estimator behind it.
///
/// A named slot in an enum is not a probe, and it is the worst standing to be in because it reads
/// as covered. This was **5** when the question was first asked on 2026-08-09.
pub fn declared_only() -> usize {
    LEDGER
        .iter()
        .filter(|c| matches!(c.standing, Standing::DeclaredOnly(_)))
        .count()
}

/// Body constants a probe in this crate can supply today. **Replaceable, not replaced.**
pub fn measurable() -> usize {
    LEDGER
        .iter()
        .filter(|c| matches!(c.standing, Standing::Measured(_)))
        .count()
}
