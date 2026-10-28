/* sensor.c: simulator-side rig code. A camera's sensor over a rendered picture, for robot.py, in one pass: each value x
   (0 .. 255) becomes x + sd[x] n, rounded to the nearest (half to even, as numpy's rint) and held to 0 .. 255, n a
   standard normal draw of its own and sd[x] the sensor's noise at that level (its shot and read noise, the robot's
   table). The draws: Marsaglia and Tsang's ziggurat (2000), 128 layers, a draw's layer and its value from bits of one
   64-bit draw apart (the paper took both from the same bits), over xoshiro256** seeded by splitmix64 from the picture's
   own key (the robot's seed, the camera, the beat), so every picture's noise is its own whatever thread makes it. numpy
   drew a picture's 230 thousand in 2.5 ms on the big machine (10-28), an indirect call a draw, and then 0.34 ms more to
   add them; the robots' noise was a quarter of their CPU. */
#include <math.h>
#include <stdint.h>

static uint32_t kn[128];
static float wn[128], fn[128];

/* The ziggurat's tables (the paper's zigset): called once, before any picture. */
void sensor_setup(void)
{
    const double m1 = 2147483648.0;                     /* 2^31: a signed 32-bit draw's magnitude */
    double dn = 3.442619855899, tn = dn, vn = 9.91256303526217e-3;
    const double q = vn / exp(-0.5 * dn * dn);
    kn[0] = (uint32_t) ((dn / q) * m1);
    kn[1] = 0;
    wn[0] = (float) (q / m1);
    wn[127] = (float) (dn / m1);
    fn[0] = 1.0f;
    fn[127] = (float) exp(-0.5 * dn * dn);
    for (int i = 126; i >= 1; i--) {
        dn = sqrt(-2.0 * log(vn / dn + exp(-0.5 * dn * dn)));
        kn[i + 1] = (uint32_t) ((dn / tn) * m1);
        tn = dn;
        fn[i] = (float) exp(-0.5 * dn * dn);
        wn[i] = (float) (dn / m1);
    }
}

typedef struct { uint64_t s[4]; } rng;

static inline uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

static inline uint64_t next(rng *g)
{
    const uint64_t r = rotl(g->s[1] * 5, 7) * 9;
    const uint64_t t = g->s[1] << 17;
    g->s[2] ^= g->s[0];
    g->s[3] ^= g->s[1];
    g->s[1] ^= g->s[2];
    g->s[0] ^= g->s[3];
    g->s[2] ^= t;
    g->s[3] = rotl(g->s[3], 45);
    return r;
}

static uint64_t splitmix(uint64_t *x)
{
    uint64_t z = (*x += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

/* A uniform draw in (0, 1), never 0 (a logarithm's argument). */
static inline double uni(rng *g) { return ((double) (next(g) >> 11) + 0.5) * (1.0 / 9007199254740992.0); }

/* A draw that left the fast path: the base strip's tail beyond r, or a wedge's test under the curve, else drawn again
   (the paper's nfix). Apart from the fast path, which it would crowd. */
static float __attribute__((noinline)) rnor_slow(rng *g, int32_t hz, uint32_t iz)
{
    for (;;) {
        if (iz == 0) {                                  /* the base strip: a draw from the tail beyond r */
            const double r = 3.442619855899;
            double x, y;
            do {
                x = -log(uni(g)) / r;
                y = -log(uni(g));
            } while (y + y < x * x);
            return (float) (hz > 0 ? r + x : -r - x);
        }
        {                                               /* a wedge: under the curve or drawn again */
            const float x = (float) hz * wn[iz];
            if (fn[iz] + (float) uni(g) * (fn[iz - 1] - fn[iz]) < (float) exp(-0.5 * (double) x * (double) x))
                return x;
        }
        {
            const uint64_t u = next(g);
            hz = (int32_t) (uint32_t) (u >> 32);
            iz = (uint32_t) u & 127u;
            const uint32_t az = hz < 0 ? (uint32_t) (-(int64_t) hz) : (uint32_t) hz;
            if (az < kn[iz])
                return (float) hz * wn[iz];
        }
    }
}

static inline float rnor(rng *g)
{
    const uint64_t u = next(g);
    const int32_t hz = (int32_t) (uint32_t) (u >> 32);
    const uint32_t iz = (uint32_t) u & 127u;
    const uint32_t az = hz < 0 ? (uint32_t) (-(int64_t) hz) : (uint32_t) hz;
    if (__builtin_expect(az < kn[iz], 1))
        return (float) hz * wn[iz];
    return rnor_slow(g, hz, iz);
}

/* To the nearest, half to even: 1.5 * 2^23 added and taken away again leaves a float within 2^22 of nought rounded
   to the nearest whole number by the processor's own rounding (rintf was a library call a pixel without SSE4.1). */
static inline float nearest(float v)
{
    const float big = 12582912.0f;
    return (v + big) - big;
}

void sensor_sense(const uint8_t *in, uint8_t *out, long n, const float *sd, uint64_t key)
{
    rng g;
    uint64_t x = key;
    for (int i = 0; i < 4; i++)
        g.s[i] = splitmix(&x);
    for (long i = 0; i < n; i++) {
        const float v = nearest((float) in[i] + sd[in[i]] * rnor(&g));
        out[i] = v <= 0.0f ? 0 : (v >= 255.0f ? 255 : (uint8_t) v);
    }
}
