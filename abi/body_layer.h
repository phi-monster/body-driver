/* body_layer.h -- the stable C ABI of the body layer.  THIS FILE IS THE CONTRACT.
 *
 *   world model + body layer
 *   世界模型 + 身体层
 *   世界靠学，身体靠量。
 *
 * WHY A C ABI AND NOT A LIBRARY API.  The claim is "anybody's mind + anybody's body".  If plugging
 * in required linking Rust, half the field is excluded on day one; a standard only one language can
 * consume is not a standard.  So: a stable C ABI plus one serialization format, and every other
 * language binds to it.
 *
 * ============================================================================================
 * THE INVARIANT THIS HEADER EXISTS TO ENFORCE (read this before changing any struct)
 * ============================================================================================
 * The nearest ancestor of this design is UP-OSI (Yu/Tan/Liu/Turk, RSS 2017, arXiv 1702.02453):
 * a universal policy plus online identification of body parameters.  The ONE difference, and it
 * carries the whole claim:
 *
 *     UP-OSI feeds the measured body parameters INTO the policy (the policy is body-conditioned).
 *     Here they are fed ONLY to the execution layer.  THE POLICY'S INPUT CONTAINS NO BODY
 *     PARAMETER AT ALL.
 *
 * "Measure the body, then hand it to the policy" is the whole field's reflex and it LOOKS
 * compliant -- the body really was measured, and nothing was baked into the weights.  But once a
 * body parameter enters the policy's input distribution, swapping the body degrades QUIETLY
 * instead of failing loudly.  Lose this and we are 2017.
 *
 * Therefore the enforcement is STRUCTURAL, not procedural:
 *   - `bl_policy_in` has no field through which a body parameter, a camera intrinsic/extrinsic, a
 *     joint angle, a link length, a true object pose, or a task identifier can arrive.  Not
 *     "should not" -- CANNOT.  There is no such member.
 *   - `bl_world_ref` (what any VLM/WM emits) is a normalised PIXEL plus a verb plus a coarse
 *     manner scalar.  It cannot carry 3-D.  A pointer that cannot express a pose cannot leak one.
 *   - Everything a body needs to turn that reference into joint commands lives behind
 *     `bl_execute`, on this side of the measure/learn line.
 *
 * A reviewer auditing for cheating reads exactly this header.  If a leak is possible, it is
 * visible here as a struct member; if no member can carry it, no amount of downstream code can.
 *
 * ============================================================================================
 * THE OTHER HALF: A LAYER THAT CANNOT SAY "REFUSE" IS NOT A BODY LAYER
 * ============================================================================================
 * Every measured quantity carries value / uncertainty / valid-range / timestamp / dependency list
 * / self-test / previous-valid-version (`bl_measurement`).  `bl_admit` consults them and returns
 * BL_REFUSE with a reason when a quantity is missing, stale, out of its valid range, or when a
 * dependency it was measured against has itself been re-measured.  This is the mechanical
 * replacement for a habit that has repeatedly failed here: a hand-written gate per experiment.
 *
 * The failure this prevents is specific and has been measured: a system whose estimate OF ITSELF
 * is wrong, in a loop that cannot see that error.  The servo drives |target - SELF-ESTIMATED hand|
 * to zero, so the bias in the self-estimate lands entirely in the RESULT and never in the RESIDUAL
 * -- algebra, not precision.  Recorded numbers: the loop's own visible error 3.6 px while the true
 * offset was 15.7 px; on another rig the self-estimate sat on the ELBOW, 167 px from the true
 * fingertip, while the loop reported 0.04-9.3 px.  Both read perfectly healthy.
 *
 * ============================================================================================
 * VERSIONING
 * ============================================================================================
 * BL_ABI_VERSION is checked by `bl_init`.  A mismatch is a hard failure, never a best-effort
 * degrade: a silently-degraded body layer is exactly the thing this layer exists to eliminate.
 */

#ifndef BODY_LAYER_H
#define BODY_LAYER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BL_ABI_VERSION 2u

/* ------------------------------------------------------------------ status */

typedef enum {
    BL_OK              = 0,
    BL_REFUSE          = 1,  /* a real answer, not an error -- see bl_reason        */
    BL_EINVAL          = 2,
    BL_EVERSION        = 3,  /* ABI mismatch: hard failure, never degrade           */
    BL_ENOSPACE        = 4,
    BL_EINTERNAL       = 5
} bl_status;

/* Why a request was refused.  Each maps to a measurable condition, never to a mood. */
typedef enum {
    BL_R_NONE                 = 0,
    BL_R_NEVER_MEASURED       = 1,  /* the quantity has no value on this body yet   */
    BL_R_STALE                = 2,  /* older than its declared validity window      */
    BL_R_OUT_OF_RANGE         = 3,  /* request lies outside the measured valid range*/
    BL_R_DEPENDENCY_CHANGED   = 4,  /* something it was measured against moved      */
    BL_R_SELFTEST_FAILED      = 5,  /* its own self-test does not pass right now    */
    BL_R_UNCERTAINTY_TOO_HIGH = 6,  /* measured, but not well enough for this ask   */
    BL_R_UNREACHABLE          = 7,  /* geometry says this body cannot get there     */
    BL_R_RATE_LIMIT           = 8,  /* the fast face declined: limit/watchdog        */
    /* 🔴 NOT NOW -- and nothing here says never.  The ask is outside what this body can do AT THIS
     * INSTANT, and the refusal is about the state of the WORLD, not about the body.
     *
     * CALLER CONTRACT: let the world advance and ask again.  Do NOT abandon the task, and do NOT
     * fold this into BL_R_UNREACHABLE -- that one tells a robot to give up on something that is
     * merely still on its way.  On a conveyor this distinction was hand-welded into an experiment
     * script three separate times in one night before it was noticed that all three were the same
     * missing concept. */
    BL_R_NOT_YET              = 9,
    /* 🔴 I COULD NOT CHECK -- as opposed to BL_R_SELF_TEST_FAILED, which is "I checked and it is
     * wrong".  Merging the two was tried and is a measured mistake: the caller's response differs,
     * re-probe versus abandon, so one name sends half of them the wrong way.
     *
     * CALLER CONTRACT: this reason arrives with BL_OK, not BL_REFUSE.  It is the third rung --
     * proceed, but nothing has verified this number over the range you are using.  It is produced
     * when an ask touches an axis of kind BL_AXIS_UNMEASURED.  Hard-refusing instead was tried:
     * one constant with an entirely unmeasured domain made every constant downstream of it
     * unusable, collapsing the three rungs back to two. */
    BL_R_NO_EVIDENCE          = 10
} bl_reason;

/* ------------------------------------------------------- measured quantities */

/* The identifiers are BODY properties -- things a robot can determine about itself by acting.
 * They are deliberately NOT a list of "parameters a URDF happens to contain": a URDF is written
 * by a person, and a hand-written body constant is the same violation as a hand-fed demonstration.
 */
typedef enum {
    BL_Q_HAND_PIXEL        = 0,  /* which pixels are my hand, in the frame I can see    */
    BL_Q_IMAGE_JACOBIAN    = 1,  /* I move by delta -> the image moves by this          */
    BL_Q_GRIPPER_SPAN      = 2,  /* full-open to full-closed, in metres, measured       */
    BL_Q_ARM_WEIGHT        = 3,  /* what holding still against gravity costs            */
    BL_Q_LATENCY           = 4,  /* command issued -> pixels move, in control periods   */
    BL_Q_BACKLASH          = 5,  /* push both ways; the dead band is the slop           */
    BL_Q_REACH             = 6,  /* where this body can actually put its hand           */
    BL_Q_CONTACT_THRESHOLD = 7,  /* what "I touched something" reads like on this body  */
    BL_Q_SELF_OCCLUSION    = 8,  /* which parts of my own view I block                  */
    BL_Q_STEP_DELIVERY     = 9,  /* I command a step; this fraction of it arrives in    */
                                 /* one control period.  NOT latency (dead time) and    */
                                 /* NOT backlash (dead band at a reversal).             */
    BL_Q_TOOL_OFFSET       = 10, /* how far my working point sits from the mount I    */
                                 /* command, along the tool axis, in metres.  Typed  */
                                 /* in at FOUR places in the live stack, with three  */
                                 /* values for three bodies (0.145 / 0.102 / 0.1034) */
                                 /* and a default that silently uses another robot's.*/
    BL_Q_TOOL_AXIS_COLUMN  = 11, /* which column of R points along my tool, in {0,1,2}. */
                                 /* Same motion as the offset: spin about each column  */
                                 /* and the one the working point barely moves about   */
                                 /* IS the tool axis.  Typed in per body as            */
                                 /* L3_TOOL_COL (0 on one arm, 2 on another).          */
    BL_Q_COUNT             = 12
} bl_quantity;

/* 🔴 3 * BL_MAX_JOINTS, and that is not slack -- it is the smallest value that fits the image
 * Jacobian (3 world axes x one column per joint).  An earlier draft used 16 "because a body
 * quantity is a small vector"; an ordinary 6-joint arm needs 18, and a unit test caught it.
 * A constant chosen because it felt about right is a constant nobody measured. */
#define BL_MAX_DIM   48
#define BL_MAX_DEPS   8
#define BL_REASON_LEN 96

/* 🔴 The seven fields below are not decoration.  A quantity that ships without uncertainty, without
 * a validity window, or without a self-test cannot be refused on -- and a layer that cannot refuse
 * is not a body layer.  `bl_measure` REJECTS a submission missing any of them.
 */
/* What one axis of valid_lo/hi MEANS.  Both non-default kinds come from a measured failure.
 *
 * BL_AXIS_INTERVAL is 0, so a caller that memsets its bl_measurement gets exactly the behaviour
 * that existed before this field. */
typedef enum {
    BL_AXIS_INTERVAL    = 0,  /* probed continuously between valid_lo and valid_hi       */
    BL_AXIS_CATEGORICAL = 1,  /* probed only at the integer labels in [valid_lo,valid_hi];
                               * encoding a label as an interval SILENTLY ACCEPTS 0.5 --
                               * two real artefacts hit this at once                     */
    BL_AXIS_UNMEASURED  = 2   /* never probed on this axis: an ask touching it is admitted
                               * UNVERIFIED (BL_OK + BL_R_NO_EVIDENCE), never refused     */
} bl_axis_kind;

typedef struct {
    uint32_t quantity;                 /* bl_quantity                                    */
    uint32_t dim;                      /* 1..BL_MAX_DIM                                  */
    uint32_t axis_kind[BL_MAX_DIM];    /* bl_axis_kind per axis; 0 == the old behaviour  */
    double   value[BL_MAX_DIM];
    double   uncertainty[BL_MAX_DIM];  /* 1-sigma, SAME units as value. NaN is rejected. */
    double   valid_lo[BL_MAX_DIM];     /* the range over which it was actually probed --  */
    double   valid_hi[BL_MAX_DIM];     /*   NOT the range someone hopes it extrapolates to*/
    uint64_t measured_at_ns;           /* monotonic, from the caller's clock              */
    uint64_t valid_for_ns;             /* 0 == "until a dependency changes", not "forever"*/
    uint32_t n_deps;
    uint32_t deps[BL_MAX_DEPS];        /* bl_quantity values this was measured AGAINST    */
    uint64_t dep_epoch[BL_MAX_DEPS];   /* their epoch at measurement time                 */
    uint64_t epoch;                    /* bumped on every re-measure; consumers compare   */
    uint32_t selftest_passed;          /* 0/1 -- and see the note on bl_selftest below    */
    uint64_t prev_epoch;               /* the version this replaced; 0 if first           */
} bl_measurement;

/* ------------------------------------------------------ the three plug ports */

/* PORT 1 -- WORLD.  What ANY vlm / world model emits.  Deliberately impoverished.
 *
 * u,v are normalised to [0,1] in the frame the eye was shown.  There is no z, no depth, no pose,
 * no object id, no task id.  The eye earns WHERE from pixels; a noun tells it WHAT, and a noun
 * carries no position information -- a person saying "pick up the scissors" transfers none.
 *
 * `verb` and `manner` are the whole of the intent channel.  Widening this struct is the one change
 * that can quietly destroy the embodiment-agnostic property, because "a point" means the same
 * thing on every body while "a whole-body trajectory" does not.  Any proposal to add a field must
 * state, in the same breath, what it costs in body-independence.
 */
typedef enum {
    BL_V_REACH   = 0,
    BL_V_GRASP   = 1,
    BL_V_RELEASE = 2,
    BL_V_PRESS   = 3,
    BL_V_WIPE    = 4,   /* the reference is a region; see bl_world_ref.extent      */
    BL_V_COUNT   = 5
} bl_verb;

typedef struct {
    double   u, v;        /* normalised pixel in the frame the eye was shown          */
    double   extent;      /* 0 for a point; >0 = radius of a region, same units as u,v*/
    uint32_t verb;        /* bl_verb                                                  */
    double   manner;      /* [0,1] coarse effort: the eye says light/medium/firm.     */
                          /* Force is stiffness x deflection -- physics, zero data.   */
    uint64_t frame_id;    /* which frame this refers to; staleness is checkable       */
} bl_world_ref;

/* PORT 2 -- POLICY.  What the action model sees, and what it emits.
 *
 * 🔴 LOOK AT WHAT IS ABSENT.  No joint angles.  No link lengths.  No camera matrix.  No gripper
 * span.  No payload.  No robot name.  No task id.  That absence IS the invariant; it is not an
 * oversight and it is not to be "fixed" by adding a convenience field.
 *
 * The image is passed as an opaque buffer because the body layer does not interpret it -- it only
 * guarantees the buffer the policy sees is the buffer the contract promises (a mismatch here has
 * cost this project a whole experimental arc: the deployed crop did not contain the gripper on
 * 43 of 50 episodes, while training had it in 89.1% -- every downstream number was a constructed
 * zero and looked like an honest one).
 */
typedef struct {
    const uint8_t *image;      /* RGB8, height*width*3                                 */
    uint32_t       width, height;
    bl_world_ref   ref;        /* the eye's answer, already expressed in THIS frame     */
    uint64_t       frame_id;   /* must equal ref.frame_id -- bl_step asserts it         */
} bl_policy_in;

/* World-frame intent.  Direction is a UNIT vector: how far to travel is the body's business
 * (spec speed x control period), never the policy's.  Magnitude is derivable from proprioception
 * -- i.e. it is a shortcut -- while direction is not, so removing magnitude removes the shortcut.
 */
typedef struct {
    double dir[3];      /* unit vector; |dir| is asserted to 1 within tolerance          */
    double drot[3];     /* rotation increment, rad, about the tool frame                 */
    double grip;        /* [0,1] ABSOLUTE opening, not a delta -- the whole field agrees */
    double base[3];     /* vx, vy, wz for a mobile base; zeros if none                   */
} bl_policy_out;

/* PORT 3 -- BODY.  Opaque handle; one per physical robot. */
typedef struct bl_body bl_body;

/* ------------------------------------------------------------------ the API */

/* Storage for one body.  THE CALLER ALLOCATES; this library never does.
 *
 * 🔴 A hard-real-time safety layer must not depend on an allocator, and a layer that cannot build
 * for the target it has to run on is not a deliverable.  An earlier draft returned a heap handle;
 * that one allocation dragged in a global allocator and made the crate unbuildable for exactly the
 * targets it exists to serve.
 */
size_t bl_sizeof_body(void);
size_t bl_alignof_body(void);

/* Initialise a body layer in caller-supplied storage.  `abi_version` MUST be BL_ABI_VERSION;
 * a mismatch is BL_EVERSION, never a best-effort degrade.  `storage` must be at least
 * bl_sizeof_body() bytes, aligned to bl_alignof_body(), and must outlive every other call. */
bl_status bl_init(void *storage, size_t len, uint32_t abi_version);

/* Tear down.  The storage belongs to the caller and is not freed here. */
void      bl_close(bl_body *b);

/* Submit a measurement the robot made about ITSELF.  Rejects (BL_EINVAL) if uncertainty is
 * non-finite, if valid_lo >= valid_hi on any axis, if a declared dependency has never been
 * measured, or if selftest_passed is 0.  There is no "force" flag: a body layer that can be told
 * to accept an unverified constant is a configuration file with extra steps.
 */
bl_status bl_measure(bl_body *b, const bl_measurement *m);

/* Read back the current measurement, including its epoch and provenance. */
bl_status bl_get(const bl_body *b, uint32_t quantity, bl_measurement *out);

/* THE GATE.  Ask whether this world reference can be executed on this body RIGHT NOW.
 * Returns BL_OK, or BL_REFUSE with *why and a human-readable line in `detail`.
 *
 * 🔴 A REFUSE is an ANSWER.  Callers must not treat it as an error to be retried away, and must
 * not collapse it with "the task failed" -- "no data", "not applicable" and "ran and scored zero"
 * are three different things and must never look alike in a results table.
 */
bl_status bl_admit(const bl_body *b, const bl_world_ref *ref,
                   uint32_t *why, char detail[BL_REASON_LEN]);

/* 🔴 THE GATE, FOR ONE QUANTITY.
 *
 * `bl_admit` asks whether a whole world reference may be executed -- the right question for the
 * servo and the wrong one for everybody else.  A caller that only wants "may I use the tool offset,
 * and is it established over the range I am about to ask about" had no way to ask, so every such
 * caller re-implemented never-measured / stale / out-of-range / self-test-failed on its own side.
 * This project grew a Python copy of exactly those four checks, which is how one gate became two
 * implementations that can drift apart, with the untested one deciding.
 *
 * `at` is where in the quantity's own probed domain the ask sits; `tol` is the precision it needs.
 * Pass has_at = 0 / has_tol = 0 to skip either.  A REFUSE is an ANSWER. */
bl_status bl_admit_quantity(const bl_body *b, uint32_t quantity,
                            double at, uint32_t has_at,
                            double tol, uint32_t has_tol,
                            uint64_t now_ns, uint32_t *why, char detail[BL_REASON_LEN]);

/* Per-body specification, read from the machine's rating -- never tuned to make a number look
 * better.  Tuning `step_m` per task means the method failed; see the note on bl_policy_out. */
typedef struct {
    double   step_m;      /* metres per control period, from the machine's rating   */
    uint32_t period_ms;   /* control period                                          */
    double   damping;     /* least-squares damping, from the Jacobian's conditioning */
    uint32_t n_joints;    /* joints this body actually has                           */
} bl_spec;

/* What bl_execute produced, or why it produced nothing.  🔴 BL_X_REFUSED and BL_X_HALTED are
 * ANSWERS, not errors: "not permitted", "the fast face latched" and "ran and scored zero" are
 * three different facts and must never be collapsed in a results table. */
typedef enum {
    BL_X_MOVE       = 0,  /* joint_cmd written and already admitted by the fast face */
    BL_X_REFUSED    = 1,  /* the body layer refused; see bl_admit's reason           */
    BL_X_HALTED     = 2,  /* the fast face latched; joint_cmd holds the SAFE HOLD    */
    BL_X_BAD_INTENT = 3   /* the intent was malformed -- about the CALLER, not the body */
} bl_exec_outcome;

/* Execute one step.  Consumes the policy's world-frame intent, produces joint commands for THIS
 * body using THIS body's measurements.  Order is admit -> solve -> scale -> fast face -> emit;
 * there is no path around the fast face, which is the point.
 *
 * `joint_cmd` must have room for BL_MAX_JOINTS doubles.  On BL_X_HALTED it receives the safe
 * hold -- the last admitted command -- and never zeros: zeros would be a MOVE, and "fail safe"
 * cannot mean "fly to the origin".
 */
bl_status bl_execute(bl_body *b, const bl_policy_out *intent, const bl_spec *spec,
                     uint32_t now_ms, double *joint_cmd, uint32_t *outcome);

/* Run every registered self-test now and report.  `mask` bit i == quantity i passed.
 *
 * 🔴 A guard that has never failed has never been tested.  The conformance suite feeds each
 * self-test an input that MUST make it fail, and refuses to build if any of them passes it.
 */
bl_status bl_selftest(const bl_body *b, uint64_t *mask);

/* Serialize / restore the whole calibration set.
 *
 * The format carries the same provenance the in-memory measurement does, and records are written
 * in DEPENDENCY ORDER so a single-pass reader can validate each one as it arrives.
 *
 * 🔴 bl_load puts every record back through the same door a fresh measurement uses.  A loader with
 * its own private path is a loader that can admit what the live path refuses -- and then a stored
 * file becomes a way to smuggle in a constant nobody measured, which is the hole this layer exists
 * to close.  A truncated or altered file is refused, not partially applied.
 *
 * Buffer size: BL_SAVE_MAX_BYTES is a safe upper bound for any body.
 */
#define BL_MAX_JOINTS 16
size_t    bl_save_max_bytes(void);
bl_status bl_save(const bl_body *b, uint8_t *buf, size_t cap, size_t *written);
bl_status bl_load(bl_body *b, const uint8_t *buf, size_t len);

/* ------------------------------------------------- what this body still owes itself */

/* Why a quantity is on the measurement plan.  Distinct facts, never merged: "I have never measured
 * this" and "what I measured this against has moved" call for the same probe and mean very
 * different things in an audit trail. */
typedef enum {
    BL_N_NEVER_MEASURED   = 0,
    BL_N_STALE            = 1,
    BL_N_DEPENDENCY_MOVED = 2,  /* including: it is ABOUT to move, because a prerequisite is  */
                                /* itself on this plan.  Scheduled before it goes bad, not    */
                                /* discovered afterwards by whoever happens to call bl_admit.  */
    BL_N_SELFTEST_FAILED  = 3
} bl_need;

/* THE POWER-ON SCHEDULE.  Fills `quantities` / `needs` with what to measure now, dependencies
 * first, and writes the count to *n.  *n == 0 means the measuring half has finished.
 *
 * Plugging in a new machine is: bl_measure_plan -> run the probes it names -> bl_measure each ->
 * repeat until *n == 0.  Nothing about the order is typed in per robot.
 *
 * BL_ENOSPACE if cap is too small -- never a truncated plan, because a short list reads as a body
 * that owes less than it does.
 */
bl_status bl_measure_plan(const bl_body *b, uint64_t now_ns,
                          uint32_t *quantities, uint32_t *needs, size_t cap, size_t *n);

/* 🔴 THE DEBT.  Read this next to the fact that nothing can enter bl_measure without a passing
 * self-test -- which makes "hand-filled constants held by this body" a STRUCTURAL zero, true and
 * misleading.  It counts what came through this API.  What never came near it is invisible to it.
 *
 * Measured 2026-08-09: a parameter search over the deployed teacher found its DOMINANT constant,
 * `TEACH_HIGH_FRAC` -- 32/44 (73%) at <= 0.30 against 10/100 (10%) above it, Fisher p = 9.3e-14 --
 * and this layer had no slot for it.  A census of the same two files found 45 environment knobs and
 * a hardcoded camera matrix against ten declared quantities.
 *
 * So the layer publishes its own debt as a number it can be held to.  bl_debt_line() gives one row
 * per constant: name, where it is set, what this layer can do about it, and what would discharge
 * it.  `outstanding` counts the ones this layer could not supply even if it were wired in.
 *
 * ⚠️ "measured" in a row means A PROBE EXISTS, not that the constant has been replaced.  Nothing
 * outside the body-layer tree reads this layer yet.  That gap is stated here rather than hidden
 * inside a per-row status.
 */
uint32_t  bl_debt_total(void);
uint32_t  bl_debt_outstanding(void);
bl_status bl_debt_line(uint32_t i, char *buf, size_t cap);

/* ==================================================== the thin OS's memory ==================== */
/* A COMPACTING CONTEXT, not a memory store.  Bounded by construction, no allocator, caller-owned
 * storage -- the same discipline as the body itself, for the same reason.
 *
 * Memory is classified by HOW FAST IT GOES STALE, and each rung has an owner:
 *
 *   this frame  where the moving cup is now   dies next frame   NOT STORED -- look again
 *   this task   what I am doing               task ends         here, BL_MEM_TASK
 *   this place  the bin is in that corner     you leave         here, BL_MEM_PLACE
 *   this body   fingertip 0.1451 m from flange tool changes     bl_measurement
 *   the world   knives are held by the handle never             the weights
 *
 * 🔴 Rung 1 is about the OBJECT, not about positions.  "Never store a position" is right on a
 * conveyor and BACKWARDS in a living room, where the sofa and the bin are the most durable facts
 * in the task.  The question is whether the thing MOVES BY ITSELF -- that is bl_durability, and a
 * perishable fact is REFUSED by bl_memory_write rather than merely discouraged. */

typedef enum {
    BL_MEM_TASK  = 0,  /* dies when the task ends; a door does not end it       */
    BL_MEM_PLACE = 1   /* dies when you leave the place; a new errand does not  */
} bl_memory_scope;

typedef enum {
    BL_PERISHABLE = 0, /* moves by itself -> REFUSED; look again instead        */
    BL_DURABLE    = 1  /* does not move unless something moves it -> storable   */
} bl_durability;

/* What opens a NEW memory.  🔴 Three events, never a timer.  The stack this replaces collapsed all
 * three into "one episode, wipe everything", which is why walking out of a room would also make
 * the robot forget the errand. */
typedef enum {
    BL_OPENS_NEW_TASK           = 0,  /* keeps place memory + body calibration  */
    BL_OPENS_UNRECOGNISED_PLACE = 1,  /* keeps task memory + body calibration   */
    BL_OPENS_BODY_CHANGED       = 2   /* keeps both memories; the BODY re-measures */
} bl_memory_open;  /* 🔴 NOT `bl_memory_event`: that is the FUNCTION's name, and in C a typedef
                    * and a function share one namespace.  Caught by the C client on its first
                    * compile -- a defect a ctypes binding cannot surface, because it never parses
                    * this file. */

/* Was this place recognised?  🔴 Three answers.  Misidentifying a place is worse than having no
 * memory at all -- you would act on a map of somewhere else, confidently. */
typedef enum {
    BL_PLACE_SAME   = 0,
    BL_PLACE_NEW    = 1,
    BL_PLACE_UNSURE = 2   /* cannot tell, and this must not be coerced into either */
} bl_place_match;

#define BL_SLOT_BYTES        64
#define BL_MAX_SLOTS          8
#define BL_FINGERPRINT_BYTES 16

size_t    bl_memory_sizeof(void);
size_t    bl_memory_alignof(void);
bl_status bl_memory_init(void *storage, size_t len, uint32_t scope, uint32_t abi_version);
bl_status bl_memory_declare(void *m, const char *name, uint32_t pins, uint32_t *why);
/* 🔴 Advance the observation counter.  Pinning runs on THIS, not on the model reporting a state
 * change: the previous design pinned when the model left its "observing" phase, the model never
 * updated that field, and the protection was decorative for as long as it was believed to work.
 * A guard that only fires when the thing it guards against cooperates is not a guard. */
bl_status bl_memory_observed(void *m);
bl_status bl_memory_write(void *m, const char *name, const char *value,
                          uint32_t durability, uint32_t *why);
bl_status bl_memory_get(const void *m, const char *name, char *out, size_t cap, uint32_t *why);
bl_status bl_memory_event(void *m, uint32_t event, uint32_t *cleared);
/* `unreadable` is exposed because a channel failing quietly looks exactly like a world that is
 * merely slow; only a count separates them. */
bl_status bl_memory_stats(const void *m, uint64_t *observations, uint64_t *unreadable,
                          uint64_t *refused_perishable, uint32_t *filled, uint32_t *declared);
bl_status bl_place_matches(const uint8_t *a, double a_confidence,
                           const uint8_t *b, double b_confidence, uint32_t *out);

/* ==================================================== prediction ============================== */
/* 🔴 WILL THAT STILL BE THE POINT WHEN I GET THERE?
 *
 * The measurement that forced this into the ABI.  Conveyor, four episodes, every stage healthy:
 * image error converged to 7.2-7.9 px, contact landed within 9-28 mm of the object's own height,
 * the descent drifted sideways by 1.8-6.9 mm, the tool offset audited to within 0.9-2.8 cm.  And
 * the hand finished 17-30 CM from the object, obj_dz = 0.000.
 *
 * That distance is not error.  It is 44 control periods of close-and-lift times 4 mm of belt
 * travel per period: the hand arrived exactly where the object HAD BEEN when it was last aimed.
 * Every individual reading was correct and the grasp was lost before the descent started.  A layer
 * that can only answer "can I reach that point" cannot see this.
 *
 * 🔴 THE BODY LAYER DOES NOT PREDICT.  Where a cup will be is a statement about the WORLD, and the
 * world is learned.  What is MEASURED here is how long this body will be blind while it acts
 * (bl_predict_horizon) and whether a prediction may be acted on (bl_predict_admit).  A
 * bl_predict() that returned where the cup will be would put a learned quantity inside the
 * measured layer, where nothing can refuse it and no probe can check it. */

typedef struct {
    double   u;                  /* normalised image coordinates AT at_period               */
    double   v;
    double   extent;             /* normalised region size                                  */
    uint32_t at_period;          /* control periods from now; 0 == "right now", which is
                                  * exactly what NOT predicting asserts                     */
    double   sigma_uv;           /* 1-sigma of (u,v) at that horizon.  REQUIRED, no default:
                                  * a prediction that cannot say how well it knows itself is
                                  * the bare double this layer exists to abolish, in the
                                  * future tense                                            */
    uint32_t verified_periods;   /* largest horizon this model was ACTUALLY validated over.
                                  * 0 == never validated -> admitted UNVERIFIED, not refused */
} bl_predicted;

/* 🔴 Note what is absent: no z, no pose, no object id -- the same vocabulary as bl_world_ref.
 * This is the most natural place in the whole design for a 3-D pose to enter ("just tell me where
 * it will BE"), and a prediction that could return one would be a leak with a respectable name. */

bl_status bl_predict_horizon(const void *b, double distance_m, double tol_frac,
                             uint32_t *out, uint32_t *why);
bl_status bl_predict_admit(const bl_predicted *p, uint32_t need_periods,
                           double tol_uv, uint32_t has_tol,
                           uint32_t *why, char detail[BL_REASON_LEN]);
/* The whole question in one call, so a caller cannot ask the gate without asking the horizon --
 * which is precisely how the conveyor loop came to aim at a stale point while looking healthy. */
bl_status bl_predict_admit_chase(const void *b, const bl_predicted *p, double distance_m,
                                 double tol_frac, double tol_uv, uint32_t has_tol,
                                 uint32_t *why, char detail[BL_REASON_LEN]);

/* ==================================================== probes: the MEASURING half ============== */
/* 🔴 UNREACHABLE FROM C UNTIL 2026-08-11, AND THAT WAS THE BIGGEST HOLE IN THIS ABI.
 *
 * Eleven probes existed in the implementation and `nm` reported ZERO probe symbols exported.  A
 * caller in another language could READ a body constant and could ask whether it may be USED --
 * and could not MEASURE one.  That is the half this whole layer is for: 世界靠学,身体靠量.
 * A robot that can only be handed numbers is a robot with a config file.
 *
 * One function per probe, explicitly typed.  A single generic entry taking a params[] array would
 * be shorter and would encode "params[2] is the Jacobian epoch" as a positional convention nobody
 * can check -- a bug class this project has already paid for.
 *
 * ⚠️ STATED GAP: three probes are NOT here yet -- image_jacobian, hand_pixel and self_occlusion --
 * because their inputs are richer than parallel arrays (image candidate lists, a stateful tracker).
 * They exist in the implementation and are reachable only from Rust today.  Saying so is cheaper
 * than a caller discovering it. */

/* Why a PROBE declined.  Distinct from bl_reason on purpose: a probe declines to PRODUCE a
 * measurement, a gate refuses to ADMIT one.  One name for both would lose which half said no. */
typedef enum {
    BL_D_NOT_ENOUGH_SAMPLES = 0,
    BL_D_NO_RESPONSE        = 1,  /* the commanded motion did not move the observed signal */
    BL_D_INCONSISTENT       = 2,
    BL_D_MISSING_DEPENDENCY = 3
} bl_declined;

const char *bl_declined_str(uint32_t d);

/* 🔴 THE FORCE PAIR.
 *
 * arm_weight: the arm parks at a pose, touches nothing, reports the torque needed to stay there.
 * On one rig this turned 55-95 N of apparent load into 1.89 N.  Its validity range is THE SET OF
 * POSES ACTUALLY VISITED, and that is load-bearing: a gravity self-calibration on this project had
 * its ENTIRE residual in interpolation between sampled poses.
 *
 * contact_threshold depends on it: any contact signal a joint can produce has the gravity load in
 * it, so measure the hold torque FIRST or the threshold is a statement about the arm's own weight.
 * Pass arm_weight's epoch so re-measuring the weight invalidates the threshold automatically. */
bl_status bl_probe_arm_weight(const double *joint_angle, const double *hold_torque, size_t n,
                              uint64_t now_ns, bl_measurement *out, uint32_t *why);
/* 🔴 `polarity` is NOT optional and has no default.  A force/current/torque channel reads HIGHER
 * on contact; a "did the commanded motion actually happen" channel reads LOWER -- and this
 * project's own validated detector is the second kind (0.18 free vs 0.0001 touching, 289 steps,
 * zero overlap).  The probe hard-coded the first until 2026-08-11 and therefore REFUSED the one
 * contact detector here that had been measured to work.  Guessing costs a detector that fires in
 * free space and stays silent on contact. */
typedef enum {
    BL_CONTACT_HIGHER = 0,  /* force, current, torque: pressing makes the number bigger  */
    BL_CONTACT_LOWER  = 1   /* delivered-motion: pressing makes the number smaller        */
} bl_contact_polarity;

bl_status bl_probe_contact_threshold(const double *free, size_t n_free,
                                     const double *touching, size_t n_touching,
                                     uint32_t polarity,
                                     uint64_t now_ns, uint64_t arm_weight_epoch,
                                     bl_measurement *out, uint32_t *why);

/* How much of a commanded step actually arrives in one control period.  Two arms on one harness
 * answered 0.76 and 0.11 to the same 45 mm command. */
bl_status bl_probe_step_delivery(const double *commanded, const double *achieved, size_t n,
                                 uint64_t now_ns, bl_measurement *out, uint32_t *why);
/* Where this body can put its hand, as a RADIAL BAND from its own base -- the shape reach actually
 * has.  A hand-typed axis-aligned box rejected a layout 0.409 m from the base while accepting four
 * further ones. */
bl_status bl_probe_reach(const double *radius, const uint32_t *attained, size_t n,
                         uint64_t now_ns, bl_measurement *out, uint32_t *why);
/* Dead time.  first_motion_step < 0 means nothing moved within steps_observed -- a refusal, NOT a
 * latency equal to steps_observed. */
bl_status bl_probe_latency(int64_t first_motion_step, uint32_t steps_observed,
                           uint64_t now_ns, bl_measurement *out, uint32_t *why);
/* The dead band around a reversal: push both ways, what fails to arrive is the slop.  The
 * number-one accuracy killer on cheap hardware, measurable with no extra sensor. */
bl_status bl_probe_backlash(const double *commanded, const double *observed, size_t n,
                            uint64_t now_ns, bl_measurement *out, uint32_t *why);
/* Full-open to full-closed off this body's own jaws.  Refuses BL_D_NO_RESPONSE when the commanded
 * opening does not move the observed signal -- which is what one body in this project answers, and
 * why an approach height cannot be derived on it. */
bl_status bl_probe_gripper_span(const double *opening, const double *separation, size_t n,
                                double units_per_m, double units_per_m_sigma,
                                uint64_t now_ns, uint64_t jac_epoch,
                                bl_measurement *out, uint32_t *why);
/* Turn the wrist; the working point sweeps an arc whose RADIUS is the offset.  This is the
 * constant that was typed in at four places in one live stack, with three values for three bodies
 * and a default that silently used another robot's. */
bl_status bl_probe_tool_offset(const double *wrist_angle, const double *u, const double *v,
                               size_t n, double units_per_m, double units_per_m_sigma,
                               uint64_t now_ns, uint64_t jac_epoch,
                               bl_measurement *out, uint32_t *why);

/* Human-readable, for logs and for the audit trail.  Never parsed. */
const char *bl_reason_str(uint32_t why);
const char *bl_quantity_str(uint32_t quantity);
const char *bl_need_str(uint32_t need);

#ifdef __cplusplus
}
#endif
#endif /* BODY_LAYER_H */
