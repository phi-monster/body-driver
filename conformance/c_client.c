/* c_client.c -- the whole body driver, driven from C, compiled against the header itself.
 *
 * ============================================================================================
 * WHY C AND NOT THE PYTHON THAT ALREADY EXISTS
 * ============================================================================================
 * The claim on this header is *anybody's mind + anybody's body*. Until this file, every end-to-end
 * exercise of that claim went through `bind/python/body_layer.py` -- which is a HAND-MIRRORED copy
 * of the structs and signatures below. A ctypes mirror that disagrees with the header does not
 * fail: it reads shifted bytes and reports BL_OK. So the evidence was resting on the one thing
 * this project keeps getting wrong.
 *
 * Two hand-mirror bugs were found in this same tree on 2026-08-11 alone:
 *   * `bl_reason_str` was a second copy of the reason table that stopped at RATE_LIMIT, so
 *     BL_R_NOT_YET had NO NAME over the ABI from the day it was added -- and the Python binding
 *     walks the enum until the first "unknown", so one gap truncated the whole table.
 *   * the Ada fast face hand-mirrors bl_status with a comment saying the numbering "sits visibly
 *     next to the header it has to match", and nothing checked it.
 *
 * A C client cannot have that class of bug. Every call below is type-checked against the header by
 * the compiler; a signature change breaks the BUILD instead of producing a plausible number. And C
 * is the lowest common denominator: if C can drive this, C++/Rust/Ada/Java/Go can.
 *
 * ============================================================================================
 * WHAT IT PROVES, IN ORDER
 * ============================================================================================
 *   1. FORCE      -- measure what holding still against gravity costs, from poses; refuse outside
 *                    the poses actually visited.
 *   2. CONTACT    -- what "I touched something" reads like, keyed to the weight it depends on.
 *   3. MEMORY     -- a bounded compacting context: a durable fact pins mechanically, a perishable
 *                    one cannot be stored at all, a new errand does not erase the room.
 *   4. PREDICTION -- how long this body is blind, and whether a prediction may be acted on.
 *   5. THE WHOLE CHAIN on one body: measure -> store -> gate -> remember -> predict -> admit.
 *
 * Exit 0 only if every check passes. Build and run: ./conformance/c_check.sh
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "body_layer.h"

static int failures = 0;
static int checks = 0;

static void ok(const char *what, int cond, const char *detail)
{
    checks++;
    if (cond) {
        printf("  ok    %-58s %s\n", what, detail ? detail : "");
    } else {
        printf("  FAIL  %-58s %s\n", what, detail ? detail : "");
        failures++;
    }
}

/* A measurement built the way a caller must build one: every field stated, nothing defaulted. */
static bl_measurement blank(uint32_t q, uint32_t dim, uint64_t now_ns)
{
    bl_measurement m;
    memset(&m, 0, sizeof m); /* axis_kind 0 == BL_AXIS_INTERVAL, the pre-existing behaviour */
    m.quantity = q;
    m.dim = dim;
    m.measured_at_ns = now_ns;
    m.valid_for_ns = 0; /* governed by dependency epochs, NOT "forever" */
    m.epoch = 1;
    m.selftest_passed = 1;
    return m;
}

int main(void)
{
    printf("c_client: driving the body layer from C, compiled against abi/body_layer.h\n");
    printf("  ABI version %u\n", (unsigned)BL_ABI_VERSION);

    /* ---------------------------------------------------------------- 1. FORCE */
    /* Poses the arm visits anyway, and the torque needed to hold each one while touching nothing.
     * Shaped like the 72-pose sweep this project actually ran: a smooth gravity curve. */
    enum { NPOSE = 72 };
    double angle[NPOSE], torque[NPOSE];
    for (int i = 0; i < NPOSE; i++) {
        angle[i] = -0.20 + 0.54 * (double)i / (NPOSE - 1);
        torque[i] = -25.0 - 6.0 * sin(angle[i] * 3.0);
    }

    bl_measurement weight;
    uint32_t why = 0;
    bl_status st = bl_probe_arm_weight(angle, torque, NPOSE, 1, &weight, &why);
    char buf[128];
    snprintf(buf, sizeof buf, "mean %.4f N*m, 1sigma %.4f, domain [%.3f, %.3f] rad",
             weight.value[0], weight.uncertainty[0], weight.valid_lo[0], weight.valid_hi[0]);
    ok("FORCE: the arm weighs itself from poses it visits anyway", st == BL_OK, buf);
    ok("...and the domain is the poses ACTUALLY VISITED, not a guess",
       fabs(weight.valid_lo[0] - angle[0]) < 1e-9 && fabs(weight.valid_hi[0] - angle[NPOSE - 1]) < 1e-9,
       "a gravity self-cal here had its ENTIRE residual in interpolation between sampled poses");

    /* Three samples is the floor; two must decline rather than fit a line through nothing. */
    bl_measurement scratch;
    why = 0;
    st = bl_probe_arm_weight(angle, torque, 2, 1, &scratch, &why);
    ok("...and too few samples DECLINE with a name", st == BL_REFUSE,
       (const char *)bl_declined_str(why));

    /* ---------------------------------------------------------------- 2. CONTACT */
    /* Two labelled populations of one signal. Keyed to the weight's epoch: any contact signal a
     * joint produces has the gravity load in it, so re-weighing must invalidate this. */
    double freeair[64], touching[64];
    for (int i = 0; i < 64; i++) {
        freeair[i] = 0.180 + 0.001 * (i % 7);
        touching[i] = 0.0004 + 0.00001 * (i % 5);
    }
    bl_measurement contact;
    why = 0;
    /* This body's signal reads LOWER on contact -- the polarity of the detector this
     * project validated at 289 steps with zero overlap. */
    st = bl_probe_contact_threshold(freeair, 64, touching, 64, BL_CONTACT_LOWER, 1,
                                    weight.epoch, &contact, &why);
    snprintf(buf, sizeof buf, "threshold %.5f", contact.value[0]);
    ok("CONTACT: 'I touched something' has a measured threshold", st == BL_OK, buf);
    ok("...and it declares the weight as a dependency",
       contact.n_deps >= 1 && contact.deps[0] == BL_Q_ARM_WEIGHT,
       "so re-weighing the arm invalidates it automatically");

    /* ---------------------------------------------------------------- 3. THE BODY */
    size_t nb = bl_sizeof_body();
    void *storage = aligned_alloc(bl_alignof_body(), ((nb + 63) / 64) * 64);
    if (!storage) {
        printf("  FAIL  could not allocate %zu bytes for a body\n", nb);
        return 2;
    }
    st = bl_init(storage, ((nb + 63) / 64) * 64, BL_ABI_VERSION);
    ok("a body initialises in caller-owned storage", st == BL_OK, "the library never allocates");
    st = bl_init(storage, ((nb + 63) / 64) * 64, BL_ABI_VERSION + 1);
    ok("...and an ABI mismatch is a HARD failure", st == BL_EVERSION, "never a best-effort degrade");
    bl_init(storage, ((nb + 63) / 64) * 64, BL_ABI_VERSION);

    ok("the weight is admitted into the body", bl_measure(storage, &weight) == BL_OK, NULL);

    why = 0;
    char detail[BL_REASON_LEN];
    st = bl_admit_quantity(storage, BL_Q_ARM_WEIGHT, angle[NPOSE / 2], 1, 0.0, 0, 1, &why, detail);
    ok("...usable INSIDE the poses it visited", st == BL_OK, detail);
    why = 0;
    st = bl_admit_quantity(storage, BL_Q_ARM_WEIGHT, angle[NPOSE - 1] + 1.0, 1, 0.0, 0, 1, &why,
                           detail);
    ok("...and REFUSED outside them", st == BL_REFUSE && why == BL_R_OUT_OF_RANGE, detail);

    why = 0;
    st = bl_admit_quantity(storage, BL_Q_GRIPPER_SPAN, 0.0, 0, 0.0, 0, 1, &why, detail);
    ok("a quantity never measured refuses BY NAME", st == BL_REFUSE && why == BL_R_NEVER_MEASURED,
       detail);

    /* ---------------------------------------------------------------- 4. MEMORY */
    size_t nm = bl_memory_sizeof();
    void *task = aligned_alloc(bl_memory_alignof(), ((nm + 63) / 64) * 64);
    void *place = aligned_alloc(bl_memory_alignof(), ((nm + 63) / 64) * 64);
    if (!task || !place) {
        printf("  FAIL  could not allocate memories\n");
        return 2;
    }
    ok("a TASK memory initialises",
       bl_memory_init(task, ((nm + 63) / 64) * 64, BL_MEM_TASK, BL_ABI_VERSION) == BL_OK, NULL);
    ok("a PLACE memory initialises",
       bl_memory_init(place, ((nm + 63) / 64) * 64, BL_MEM_PLACE, BL_ABI_VERSION) == BL_OK, NULL);

    bl_memory_declare(task, "first_object", 1, &why); /* pins */
    bl_memory_declare(task, "goal", 0, &why);
    bl_memory_declare(place, "bin_corner", 0, &why);

    bl_memory_observed(task);
    ok("MEMORY: a durable fact is stored",
       bl_memory_write(task, "first_object", "a green cup", BL_DURABLE, &why) == BL_OK, NULL);

    why = 0;
    st = bl_memory_write(task, "goal", "it is at the left edge", BL_PERISHABLE, &why);
    ok("...a PERISHABLE fact cannot be stored at all", st == BL_REFUSE,
       "rung 1 is look-again; storing it made a stale position indistinguishable from a fact");

    bl_memory_observed(task); /* the eye has now seen a different picture */
    why = 0;
    st = bl_memory_write(task, "first_object", "a red cup", BL_DURABLE, &why);
    ok("...and the pin fires on the OBSERVATION COUNTER, not on cooperation", st == BL_REFUSE,
       "the previous design pinned on a state the model never updated");

    char slot[BL_SLOT_BYTES + 1];
    bl_memory_get(task, "first_object", slot, sizeof slot, &why);
    ok("...so history cannot be rewritten", strcmp(slot, "a green cup") == 0, slot);

    bl_memory_write(place, "bin_corner", "north-east, behind the sofa", BL_DURABLE, &why);
    bl_memory_write(task, "goal", "tidy the living room", BL_DURABLE, &why);
    uint32_t cleared = 0;
    bl_memory_event(task, BL_OPENS_NEW_TASK, &cleared);
    ok("a NEW ERRAND clears the task memory", cleared == 1, NULL);
    bl_memory_event(place, BL_OPENS_NEW_TASK, &cleared);
    ok("...and does NOT clear the room", cleared == 0, NULL);
    bl_memory_get(place, "bin_corner", slot, sizeof slot, &why);
    ok("...the room is still known", strcmp(slot, "north-east, behind the sofa") == 0, slot);
    bl_memory_event(place, BL_OPENS_BODY_CHANGED, &cleared);
    ok("swapping a gripper erases NEITHER memory", cleared == 0,
       "it invalidates the BODY's numbers, not what we were doing or where");

    uint8_t here[BL_FINGERPRINT_BYTES], elsewhere[BL_FINGERPRINT_BYTES];
    memset(here, 7, sizeof here);
    memset(elsewhere, 9, sizeof elsewhere);
    uint32_t match = 99;
    bl_place_matches(here, 0.95, here, 0.92, &match);
    ok("a place recognised is the same place", match == BL_PLACE_SAME, NULL);
    bl_place_matches(here, 0.95, elsewhere, 0.95, &match);
    ok("a place never seen opens a new memory", match == BL_PLACE_NEW, "correct outdoors");
    bl_place_matches(here, 0.95, here, 0.55, &match);
    ok("...and 'I cannot tell' is its OWN answer", match == BL_PLACE_UNSURE,
       "acting on a map of somewhere else, confidently, is worse than having no map");

    /* ---------------------------------------------------------------- 5. PREDICTION */
    /* This body needs to know how long it is blind, and that comes from ITS OWN delivery. */
    double cmd[16], got[16];
    for (int i = 0; i < 16; i++) {
        cmd[i] = 0.005 + 0.003 * i;
        got[i] = cmd[i] * 0.90;
    }
    bl_measurement delivery;
    why = 0;
    st = bl_probe_step_delivery(cmd, got, 16, 1, &delivery, &why);
    ok("PREDICT: this body measures how much of a step arrives", st == BL_OK, NULL);
    bl_measure(storage, &delivery);

    bl_measurement lat = blank(BL_Q_LATENCY, 1, 1);
    lat.value[0] = 1.0;
    lat.valid_lo[0] = 0.0;
    lat.valid_hi[0] = 12.0;
    bl_measure(storage, &lat);

    uint32_t horizon = 0;
    why = 0;
    st = bl_predict_horizon(storage, 0.20, 0.01, &horizon, &why);
    snprintf(buf, sizeof buf, "%u control periods to cover 0.20 m", horizon);
    ok("...how long it will be BLIND, from its own delivery", st == BL_OK && horizon > 0, buf);

    bl_predicted now = {0.5, 0.5, 0.1, 0, 0.0, 0}; /* no prediction: "it will still be there" */
    why = 0;
    st = bl_predict_admit_chase(storage, &now, 0.20, 0.01, 0.0, 0, &why, detail);
    ok("...chasing a moving thing with NO prediction is refused", st == BL_REFUSE, detail);

    bl_predicted ahead = {0.62, 0.5, 0.1, horizon, 0.01, horizon * 2};
    why = 0;
    st = bl_predict_admit_chase(storage, &ahead, 0.20, 0.01, 0.02, 1, &why, detail);
    ok("...a prediction that covers the blind stretch is admitted", st == BL_OK, detail);

    bl_predicted unvalidated = {0.62, 0.5, 0.1, horizon, 0.01, 0};
    why = 0;
    st = bl_predict_admit_chase(storage, &unvalidated, 0.20, 0.01, 0.0, 0, &why, detail);
    ok("...never-validated is ADMITTED-UNVERIFIED, the third rung",
       st == BL_OK && why == BL_R_NO_EVIDENCE, detail);

    bl_predicted too_far = {0.62, 0.5, 0.1, horizon * 4, 0.01, horizon};
    why = 0;
    st = bl_predict_admit(&too_far, horizon * 4, 0.0, 0, &why, detail);
    ok("...past what was validated REFUSES, never extrapolates",
       st == BL_REFUSE && why == BL_R_OUT_OF_RANGE, detail);

    /* ------------------------------------------------- TOUCHING, or out of solution?
     *
     * The two are indistinguishable to a delivered-motion ruler on its own.  Replayed here from
     * results/stallwhat_aug2026: on a flat conveyor, two of nine probe points reported contact
     * while the arm could not lift off.  A surface can block a direction; it can never block that
     * direction's opposite. */
    bl_measure(storage, &contact);
    uint32_t touch = 99;
    double free_bar = -1.0;
    why = 0;
    st = bl_touching(storage, 0.0, 1.0, 1, 0.99, 1, 1, &touch, &free_bar, &why, detail);
    snprintf(buf, sizeof buf, "%s, reverse had to clear %.4f", bl_touch_str(touch), free_bar);
    ok("TOUCH: blocked going in, free coming back out = contact",
       st == BL_OK && touch == BL_TOUCH_CONTACT, buf);
    ok("...and the bar came from this body, not from a constant",
       free_bar > 0.0 && free_bar < 1.0, buf);

    /* p0 on the belt probe: it reported contact and could not lift off. */
    why = 0;
    st = bl_touching(storage, 0.0, 0.299, 1, 0.698, 1, 1, &touch, &free_bar, &why, detail);
    ok("...blocked BOTH ways is NOT contact -- it is no solution here",
       st == BL_REFUSE && touch == BL_TOUCH_STUCK && why == BL_R_UNREACHABLE, detail);

    /* p4: sideways nearly dead, reverse free.  Friction does that on a real surface. */
    why = 0;
    st = bl_touching(storage, 0.0, 0.972, 1, 0.090, 1, 1, &touch, &free_bar, &why, detail);
    ok("...sideways must NOT decide (friction blocks it on a real surface)",
       st == BL_OK && touch == BL_TOUCH_CONTACT, detail);

    why = 0;
    st = bl_touching(storage, 0.0, 0.0, 0, 0.0, 0, 1, &touch, &free_bar, &why, detail);
    ok("...no reverse asked REFUSES rather than guessing",
       st == BL_REFUSE && touch == BL_TOUCH_UNKNOWN && why == BL_R_NO_EVIDENCE, detail);

    why = 0;
    st = bl_touching(storage, 0.9, 1.0, 1, 0.0, 0, 1, &touch, &free_bar, &why, detail);
    ok("...a command that arrived is free space", st == BL_OK && touch == BL_TOUCH_FREE, detail);

    for (uint32_t t = 0; t <= 3; t++) {
        ok("...every bl_touch has a name over the ABI", bl_touch_str(t)[0] != '?',
           bl_touch_str(t));
    }

    /* ------------------------------------------------------------- THE FLOOR, FROM C
     *
     * Replayed from results/floormap_aug2026: a 3x3 grid on the belt with two cells where the arm
     * ran out of solution 7-9 cm lower.  Those two must not drag the plane, and a stop at one of
     * them must come back as an ARM LIMIT rather than as contact -- which is what the delivery
     * ruler alone called it, on all nine conveyor episodes. */
    double fx[9], fy[9], fz[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int k = i * 3 + j;
            fx[k] = -0.06 + 0.06 * i;
            fy[k] = -0.06 + 0.06 * j;
            fz[k] = 0.9190 + ((k % 2) ? -0.0012 : 0.0012);
        }
    }
    fz[0] = 0.8475;   /* the arm ran out of solution here */
    fz[1] = 0.8343;
    bl_measurement fl;
    why = 0;
    st = bl_floor_fit(fx, fy, fz, 9, 0.01, 1, contact.epoch, delivery.epoch, &fl, &why);
    snprintf(buf, sizeof buf, "plane %.4f, band %.4f", fl.value[0], fl.uncertainty[0]);
    ok("FLOOR: a grid of stops becomes a plane", st == BL_OK, buf);
    ok("...and the cells where the arm ran out did NOT drag it",
       st == BL_OK && fl.value[0] > 0.914 && fl.value[0] < 0.924, buf);
    bl_measure(storage, &fl);

    uint32_t what = 99;
    double floor_z = 0.0, height = 0.0;
    why = 0;
    st = bl_floor_read_stop(storage, 0.0, 0.0, 0.9190, 3.0, &what, &floor_z, &height, &why, detail);
    ok("...a stop AT the floor is the working surface",
       st == BL_OK && what == BL_STOP_ON_FLOOR, bl_stop_str(what));

    why = 0;
    st = bl_floor_read_stop(storage, 0.0, 0.0, 0.9190 + 0.02, 3.0, &what, &floor_z, &height, &why,
                            detail);
    snprintf(buf, sizeof buf, "%s, %.4f m tall", bl_stop_str(what), height);
    ok("...a stop ABOVE it is an object, and its height comes back",
       st == BL_OK && what == BL_STOP_ON_SOMETHING && height > 0.015 && height < 0.025, buf);

    why = 0;
    st = bl_floor_read_stop(storage, fx[0], fy[0], 0.8475, 3.0, &what, &floor_z, &height, &why,
                            detail);
    snprintf(buf, sizeof buf, "%s, %.4f m below", bl_stop_str(what), height);
    ok("...a stop BELOW it is the arm's own limit, NOT contact",
       st == BL_OK && what == BL_STOP_ARM_LIMIT && height > 0.05, buf);

    /* 🔴 Off the probed box is the THIRD RUNG, not silence. Refusing here was measured: on the
     * conveyor the hand works over 1.5 m of belt across two arms, and hard-refusing everything
     * outside one grid turned 163 asks into 160 refusals -- the loop never acted at all, while
     * the belt had been measured as ONE plane over that whole span (0.9205-0.9219). */
    why = 0;
    st = bl_floor_read_stop(storage, 0.60, 0.0, 0.9190, 3.0, &what, &floor_z, &height, &why, detail);
    ok("...off the probed box it ANSWERS and says nothing verified it",
       st == BL_OK && what != BL_STOP_UNKNOWN && why == BL_R_NO_EVIDENCE, detail);

    /* ---------------------------------------------------------------- the ledger */
    uint32_t total = bl_debt_total(), outstanding = bl_debt_outstanding();
    snprintf(buf, sizeof buf, "%u rows, %u outstanding", (unsigned)total, (unsigned)outstanding);
    ok("the layer publishes its own debt as a number", total > 0, buf);
    ok("...and outstanding is NOT zero", outstanding > 0,
       "a layer claiming zero debt is a layer that stopped counting");

    free(storage);
    free(task);
    free(place);

    printf("\nc_client: %d checks, %d failures\n", checks, failures);
    if (failures == 0) {
        printf("c_client: PASS -- force, touch, floor, memory and prediction all drive from C\n");
    } else {
        printf("c_client: FAIL\n");
    }
    return failures ? 1 : 0;
}
