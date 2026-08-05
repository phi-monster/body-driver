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
 * BL_ABI_VERSION is checked by `bl_open`.  A mismatch is a hard failure, never a best-effort
 * degrade: a silently-degraded body layer is exactly the thing this layer exists to eliminate.
 */

#ifndef BODY_LAYER_H
#define BODY_LAYER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BL_ABI_VERSION 1u

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
    BL_R_RATE_LIMIT           = 8   /* the fast face declined: limit/watchdog        */
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
    BL_Q_COUNT             = 9
} bl_quantity;

#define BL_MAX_DIM   16   /* a measurement is a small vector, never an array of poses */
#define BL_MAX_DEPS   8
#define BL_REASON_LEN 96

/* 🔴 The seven fields below are not decoration.  A quantity that ships without uncertainty, without
 * a validity window, or without a self-test cannot be refused on -- and a layer that cannot refuse
 * is not a body layer.  `bl_measure` REJECTS a submission missing any of them.
 */
typedef struct {
    uint32_t quantity;                 /* bl_quantity                                    */
    uint32_t dim;                      /* 1..BL_MAX_DIM                                  */
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

/* Open a body layer for one robot.  `abi_version` MUST be BL_ABI_VERSION. */
bl_status bl_open(bl_body **out, uint32_t abi_version);
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

/* Execute one step.  Consumes the policy's world-frame intent, produces joint commands for THIS
 * body using THIS body's measurements.  Calls the fast face for limits before emitting anything.
 * `n_joints` in/out: capacity in, count written out.
 */
bl_status bl_execute(bl_body *b, const bl_policy_out *intent,
                     double *joint_cmd, uint32_t *n_joints);

/* Run every registered self-test now and report.  `mask` bit i == quantity i passed.
 *
 * 🔴 A guard that has never failed has never been tested.  The conformance suite feeds each
 * self-test an input that MUST make it fail, and refuses to build if any of them passes it.
 */
bl_status bl_selftest(const bl_body *b, uint64_t *mask);

/* Serialize / restore the whole calibration set.  The format carries provenance, so a stored set
 * that outlives its validity window is refused on load rather than silently trusted.
 */
bl_status bl_save(const bl_body *b, uint8_t *buf, size_t cap, size_t *written);
bl_status bl_load(bl_body *b, const uint8_t *buf, size_t len);

/* Human-readable, for logs and for the audit trail.  Never parsed. */
const char *bl_reason_str(uint32_t why);
const char *bl_quantity_str(uint32_t quantity);

#ifdef __cplusplus
}
#endif
#endif /* BODY_LAYER_H */
