/* servobank.c: simulator-side rig code. One step of bodies.ServoBank for every servo of a robot at once: the same
   arithmetic, in the same order, as its numpy (each element as its scalar), built without fused multiply-adds
   (-ffp-contract=off) so the commands are the same to the bit (controltest.py). The robot's Python loop of a step was
   most of RB-Y1's world (10-28: 15.8 ms a beat joint by joint, 5.5 ms as numpy over its few joints). */
#include <math.h>

void servo_bank_step(int n, double dt, const unsigned char *planned, const unsigned char *gravity,
                     const unsigned char *has_v, const unsigned char *has_a, const double *vmax, const double *amax,
                     const double *kp, const double *kv_kp, const double *refs, const double *qfrc_bias,
                     const int *dof, double *p0, double *p1, double *ctrl, const int *ids)
{
    for (int i = 0; i < n; i++) {
        const double ref = refs[i];
        if (!planned[i]) {               /* an unplanned servo is sent its target itself */
            ctrl[ids[i]] = ref;
            continue;
        }
        double q0 = p0[i], q1 = p1[i];
        if (!has_v[i]) {                 /* a joint without limits is at its target at once */
            q0 = ref;
            q1 = 0.0;
        } else {
            const double err = ref - q0;
            double mag;
            if (has_a[i]) {
                const double r = sqrt(2.0 * amax[i] * fabs(err));
                mag = r < vmax[i] ? r : vmax[i];               /* np.minimum (vmax, r) */
            } else {
                mag = vmax[i];
            }
            const double want = copysign(mag, err);
            if (has_a[i]) {
                const double step = amax[i] * dt, d = want - q1;
                const double lo = -amax[i] * dt;
                const double m = d < step ? d : step;          /* np.minimum (amax * dt, want - p1) */
                q1 = q1 + (m > lo ? m : lo);                   /* np.maximum (-amax * dt, ...) */
            } else {
                q1 = want;
            }
            double nxt = q0 + q1 * dt;
            if ((ref - nxt) * err <= 0.0) {                    /* the path reaches its target this step */
                nxt = ref;
                q1 = 0.0;
            }
            q0 = nxt;
        }
        p0[i] = q0;
        p1[i] = q1;
        double u = q0 + kv_kp[i] * q1;
        if (gravity[i])
            u = u + qfrc_bias[dof[i]] / kp[i];
        ctrl[ids[i]] = u;
    }
}
