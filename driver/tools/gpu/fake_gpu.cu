// The fake robot's pictures on a GPU, one camera's picture a call: the renderer (Fake_Render: a ray through every
// pixel's centre, the nearest solid, its light and texture), the settle's accumulation, and the camera
// (Fake_Camera.Expose: exposure, the sensor's noise, MJPEG's 8x8 blocks coded and back), in single precision. The
// fake loads it at run time (tools/fake_gpu.adb, FAKE_GPU naming this library) and uses its own processors without
// it. Build: nvcc -O3 -shared -Xcompiler -fPIC -arch=sm_86 fake_gpu.cu -o libfakegpu.so
#include <cuda_runtime.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define MAX_SOLIDS 512
#define MAX_CAMERAS 16

extern "C" {

typedef struct {
  int kind;           // 0 a box, 1 a plane
  int tex;            // 0 flat, 1 noise, 2 stripes, 3 grain
  float half[3];
  float rot[9];       // a world direction into the solid's frame, row by row
  float o[3];         // the eye in the solid's frame
  float albedo[3];
  float contrast, scale, gloss;
} fg_solid;

}

typedef struct {
  int w, h, used;
  float *ray;            // per pixel: where its ray meets the plane a unit in front (x, y)
  float *scratch;        // the last render, RGB
  float *shown;          // the settle's accumulation, RGB
  float *r, *g, *b;      // exposed
  float *y, *cb, *cr;    // MJPEG's planes
  unsigned char *bytes;
  double *sum;           // the picture's luminance summed
  fg_solid *solids;
  int fresh;
} cam_t;

static cam_t cams[MAX_CAMERAS];
static int device_ok = 0;

__constant__ float c_basis[64];   // C (K, N) = c(K) cos((2N + 1) K pi / 16)
__constant__ float c_light[3];

__device__ float fg_hash(int x, int y) {
  uint64_t h = (uint64_t)(uint32_t)x * 374761393ull + (uint64_t)(uint32_t)y * 668265263ull;
  h = (h ^ (h >> 13)) * 1274126177ull;
  h = h ^ (h >> 16);
  return (float)(h % 65536ull) / 32768.0f - 1.0f;
}

__device__ float fg_smooth(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

__device__ float fg_noise2(float x, float y) {
  int x0 = (int)floorf(x), y0 = (int)floorf(y);
  float fx = fg_smooth(x - (float)x0), fy = fg_smooth(y - (float)y0);
  float a = fg_hash(x0, y0), b = fg_hash(x0 + 1, y0), c = fg_hash(x0, y0 + 1), d = fg_hash(x0 + 1, y0 + 1);
  float top = a + (b - a) * fx, bottom = c + (d - c) * fx;
  return top + (bottom - top) * fy;
}

__device__ float fg_shade(const fg_solid *m, float u, float v) {
  const float pi = 3.14159265358979f;
  float su = u * m->scale, sv = v * m->scale, t;
  switch (m->tex) {
    case 0: return 1.0f;
    case 1: t = 0.6f * fg_noise2(su, sv) + 0.3f * fg_noise2(2.0f * su + 17.0f, 2.0f * sv + 3.0f)
              + 0.1f * fg_noise2(4.0f * su + 5.0f, 4.0f * sv + 11.0f); break;
    case 2: t = sinf(2.0f * pi * su) * 0.7f + 0.3f * fg_noise2(3.0f * su, 3.0f * sv); break;
    default:
      t = 0.35f * sinf(2.0f * pi * (0.3f * su + 0.8f * fg_noise2(0.1f * su, 0.6f * sv)))
        + 0.25f * sinf(2.0f * pi * (0.73f * su + 1.1f * fg_noise2(0.17f * su + 3.0f, 0.4f * sv)))
        + 0.4f * fg_noise2(su, 6.0f * sv) + 0.3f * fg_noise2(3.0f * su + 7.0f, 18.0f * sv);
  }
  return 1.0f + m->contrast * t;
}

__global__ void render_kernel(int w, int h, const float *ray, const float *R, int n, const fg_solid *solids,
                              float *out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= w * h) return;
  float px = ray[2 * i], py = ray[2 * i + 1];
  float pn = sqrtf(px * px + py * py + 1.0f);
  float cx = px / pn, cy = py / pn, cz = 1.0f / pn;
  float dx = R[0] * cx + R[1] * cy + R[2] * cz, dy = R[3] * cx + R[4] * cy + R[5] * cz,
        dz = R[6] * cx + R[7] * cy + R[8] * cz;
  const float near = 1.0e-6f;
  float best_t = 3.0e38f; int best = -1;
  for (int s = 0; s < n; s++) {
    const fg_solid *q = &solids[s];
    float lx = q->rot[0] * dx + q->rot[1] * dy + q->rot[2] * dz;
    float ly = q->rot[3] * dx + q->rot[4] * dy + q->rot[5] * dz;
    float lz = q->rot[6] * dx + q->rot[7] * dy + q->rot[8] * dz;
    float t;
    if (q->kind == 1) {
      if (fabsf(lz) < 1.0e-12f) continue;
      t = -q->o[2] / lz;
      if (!(t > near)) continue;
    } else {
      float tin = -3.0e38f, tout = 3.0e38f;
      float o3[3] = {q->o[0], q->o[1], q->o[2]}, l3[3] = {lx, ly, lz};
      bool miss = false;
      for (int a = 0; a < 3; a++) {
        if (fabsf(l3[a]) < 1.0e-12f) { if (fabsf(o3[a]) > q->half[a]) miss = true; }
        else {
          float t1 = (-q->half[a] - o3[a]) / l3[a], t2 = (q->half[a] - o3[a]) / l3[a];
          if (t1 < t2) { tin = fmaxf(tin, t1); tout = fminf(tout, t2); }
          else { tin = fmaxf(tin, t2); tout = fminf(tout, t1); }
        }
      }
      if (miss || !(tin <= tout) || !(tin > near)) continue;
      t = tin;
    }
    if (t < best_t) { best_t = t; best = s; }
  }
  float rgb[3];
  if (best < 0) { rgb[0] = rgb[1] = rgb[2] = 110.0f; }
  else {
    const fg_solid *q = &solids[best];
    float lx = q->rot[0] * dx + q->rot[1] * dy + q->rot[2] * dz;
    float ly = q->rot[3] * dx + q->rot[4] * dy + q->rot[5] * dz;
    float lz = q->rot[6] * dx + q->rot[7] * dy + q->rot[8] * dz;
    float hx = q->o[0] + best_t * lx, hy = q->o[1] + best_t * ly, hz = q->o[2] + best_t * lz;
    float nx = 0.0f, ny = 0.0f, nz = 0.0f, u, v;
    if (q->kind == 1) { nz = (lz < 0.0f) ? 1.0f : -1.0f; u = hx; v = hy; }
    else {
      float ex = fabsf(fabsf(hx) - q->half[0]), ey = fabsf(fabsf(hy) - q->half[1]), ez = fabsf(fabsf(hz) - q->half[2]);
      if (ex <= ey && ex <= ez) { nx = (hx > 0.0f) ? 1.0f : -1.0f; u = hy; v = hz; }
      else if (ey <= ez) { ny = (hy > 0.0f) ? 1.0f : -1.0f; u = hx; v = hz; }
      else { nz = (hz > 0.0f) ? 1.0f : -1.0f; u = hx; v = hy; }
    }
    float wx = q->rot[0] * nx + q->rot[3] * ny + q->rot[6] * nz;
    float wy = q->rot[1] * nx + q->rot[4] * ny + q->rot[7] * nz;
    float wz = q->rot[2] * nx + q->rot[5] * ny + q->rot[8] * nz;
    float diffuse = fmaxf(0.0f, wx * c_light[0] + wy * c_light[1] + wz * c_light[2]);
    float hhx = c_light[0] - dx, hhy = c_light[1] - dy, hhz = c_light[2] - dz;
    float hn = sqrtf(hhx * hhx + hhy * hhy + hhz * hhz);
    float sc = fmaxf(0.0f, (wx * hhx + wy * hhy + wz * hhz) / hn);
    float spec = 0.0f;
    if (q->gloss > 0.0f) { float s2 = sc * sc, s4 = s2 * s2, s8 = s4 * s4, s16 = s8 * s8, s32 = s16 * s16; spec = q->gloss * s8 * s32; }
    float facing = 0.92f + 0.08f * fabsf(wx * dx + wy * dy + wz * dz);
    float tex = fg_shade(q, u, v);
    float level = (0.35f + 0.65f * diffuse) * tex * facing;
    for (int c = 0; c < 3; c++) rgb[c] = fmaxf(0.0f, fminf(255.0f, 255.0f * (q->albedo[c] * level + spec)));
  }
  out[3 * i] = rgb[0]; out[3 * i + 1] = rgb[1]; out[3 * i + 2] = rgb[2];
}

__device__ uint64_t fg_mix(uint64_t z) {
  z += 0x9E3779B97F4A7C15ull;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

// The sensor's Gaussian: four uniforms of 24 bits summed, centred and scaled to a unit variance, as Fake_Camera's.
__device__ float fg_gauss(uint64_t seed, uint64_t k) {
  uint64_t a = fg_mix(seed ^ (k * 0xD1B54A32D192ED03ull)), b = fg_mix(a);
  float s = (float)(a & 0xFFFFFF) / 16777216.0f + (float)((a >> 32) & 0xFFFFFF) / 16777216.0f
          + (float)(b & 0xFFFFFF) / 16777216.0f + (float)((b >> 32) & 0xFFFFFF) / 16777216.0f;
  return (s - 2.0f) * 1.7320508075688772f;
}

__device__ float fg_clip(float v) { return fmaxf(0.0f, fminf(255.0f, v)); }

__global__ void expose_kernel(int n, const float *scratch, float *shown, float share, float gain, float noise_level,
                              uint64_t seed, float *r, float *g, float *b, double *sum) {
  __shared__ double part[256];
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  double lum = 0.0;
  if (i < n) {
    for (int c = 0; c < 3; c++) shown[3 * i + c] = (1.0f - share) * shown[3 * i + c] + share * scratch[3 * i + c];
    r[i] = fg_clip(shown[3 * i] * gain + noise_level * fg_gauss(seed, 3ull * i));
    g[i] = fg_clip(shown[3 * i + 1] * gain + noise_level * fg_gauss(seed, 3ull * i + 1));
    b[i] = fg_clip(shown[3 * i + 2] * gain + noise_level * fg_gauss(seed, 3ull * i + 2));
    lum = 0.299 * r[i] + 0.587 * g[i] + 0.114 * b[i];
  }
  part[threadIdx.x] = lum;
  __syncthreads();
  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s) part[threadIdx.x] += part[threadIdx.x + s];
    __syncthreads();
  }
  if (threadIdx.x == 0) atomicAdd(sum, part[0]);
}

// JFIF's colour transform, the chroma averaged over 2 x 2 (4:2:0): one chroma sample a thread.
__global__ void ycc_kernel(int w, int h, const float *r, const float *g, const float *b, float *y, float *cb, float *cr) {
  int cw = (w + 1) / 2, ch = (h + 1) / 2;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (j >= cw * ch) return;
  int crow = j / cw, ccol = j % cw;
  float sb = 0.0f, sr = 0.0f, count = 0.0f;
  for (int row = 2 * crow; row <= 2 * crow + 1 && row < h; row++)
    for (int col = 2 * ccol; col <= 2 * ccol + 1 && col < w; col++) {
      int i = row * w + col;
      y[i] = 0.299f * r[i] + 0.587f * g[i] + 0.114f * b[i];
      sb += 128.0f - 0.168736f * r[i] - 0.331264f * g[i] + 0.5f * b[i];
      sr += 128.0f + 0.5f * r[i] - 0.418688f * g[i] - 0.081312f * b[i];
      count += 1.0f;
    }
  cb[j] = sb / count; cr[j] = sr / count;
}

// One 8x8 block a thread: forward DCT, quantized by q, back.
__global__ void dct_kernel(float *plane, int w, int h, const float *q) {
  int bw = (w + 7) / 8, bh = (h + 7) / 8;
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= bw * bh) return;
  int by = k / bw, bx = k % bw;
  float blk[64], tmp[64];
  for (int yy = 0; yy < 8; yy++)
    for (int xx = 0; xx < 8; xx++) {
      int py = min(h - 1, by * 8 + yy), px = min(w - 1, bx * 8 + xx);
      blk[yy * 8 + xx] = plane[py * w + px] - 128.0f;
    }
  for (int yy = 0; yy < 8; yy++)
    for (int kk = 0; kk < 8; kk++) { float s = 0.0f; for (int xx = 0; xx < 8; xx++) s += c_basis[kk * 8 + xx] * blk[yy * 8 + xx]; tmp[yy * 8 + kk] = s; }
  float coef[64];
  for (int kk = 0; kk < 8; kk++)
    for (int l = 0; l < 8; l++) {
      float s = 0.0f; for (int yy = 0; yy < 8; yy++) s += c_basis[kk * 8 + yy] * tmp[yy * 8 + l];
      coef[kk * 8 + l] = roundf(s / q[kk * 8 + l]) * q[kk * 8 + l];
    }
  for (int yy = 0; yy < 8; yy++)
    for (int l = 0; l < 8; l++) { float s = 0.0f; for (int kk = 0; kk < 8; kk++) s += c_basis[kk * 8 + yy] * coef[kk * 8 + l]; tmp[yy * 8 + l] = s; }
  for (int yy = 0; yy < 8; yy++)
    for (int xx = 0; xx < 8; xx++) {
      float s = 0.0f; for (int l = 0; l < 8; l++) s += c_basis[l * 8 + xx] * tmp[yy * 8 + l];
      if (by * 8 + yy < h && bx * 8 + xx < w) plane[(by * 8 + yy) * w + bx * 8 + xx] = s + 128.0f;
    }
}

__global__ void bytes_kernel(int w, int h, int mjpeg, const float *r, const float *g, const float *b,
                             const float *y, const float *cb, const float *cr, unsigned char *out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= w * h) return;
  float rr, gg, bb;
  if (mjpeg) {
    int cw = (w + 1) / 2, row = i / w, col = i % w, j = (row / 2) * cw + col / 2;
    float pb = cb[j] - 128.0f, pr = cr[j] - 128.0f;
    rr = y[i] + 1.402f * pr; gg = y[i] - 0.344136f * pb - 0.714136f * pr; bb = y[i] + 1.772f * pb;
  } else { rr = r[i]; gg = g[i]; bb = b[i]; }
  out[3 * i] = (unsigned char)(int)floorf(fg_clip(rr) + 0.5f);
  out[3 * i + 1] = (unsigned char)(int)floorf(fg_clip(gg) + 0.5f);
  out[3 * i + 2] = (unsigned char)(int)floorf(fg_clip(bb) + 0.5f);
}

extern "C" {

// The device: the given one of those there are; the constants once. 0 when ready.
int fg_init(int which) {
  int count = 0;
  if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0) return 1;
  if (cudaSetDevice(which % count) != cudaSuccess) return 2;
  float basis[64];
  for (int k = 0; k < 8; k++)
    for (int n = 0; n < 8; n++)
      basis[k * 8 + n] = (k == 0 ? sqrtf(1.0f / 8.0f) : sqrtf(2.0f / 8.0f)) * cosf((float)((2 * n + 1) * k) * 3.14159265358979f / 16.0f);
  float l[3] = {0.3f, -0.5f, 0.8f};
  float ln = sqrtf(l[0] * l[0] + l[1] * l[1] + l[2] * l[2]);
  for (int c = 0; c < 3; c++) l[c] /= ln;
  if (cudaMemcpyToSymbol(c_basis, basis, sizeof basis) != cudaSuccess) return 3;
  if (cudaMemcpyToSymbol(c_light, l, sizeof l) != cudaSuccess) return 4;
  device_ok = 1;
  return 0;
}

// Camera cam's picture size and rays (two floats a pixel, rows top to bottom). 0 when ready.
int fg_camera(int cam, int w, int h, const float *ray) {
  if (!device_ok || cam < 0 || cam >= MAX_CAMERAS) return 1;
  cam_t *c = &cams[cam];
  size_t n = (size_t)w * h;
  if (cudaMalloc(&c->ray, 2 * n * sizeof(float)) || cudaMalloc(&c->scratch, 3 * n * sizeof(float))
      || cudaMalloc(&c->shown, 3 * n * sizeof(float)) || cudaMalloc(&c->r, n * sizeof(float))
      || cudaMalloc(&c->g, n * sizeof(float)) || cudaMalloc(&c->b, n * sizeof(float))
      || cudaMalloc(&c->y, n * sizeof(float)) || cudaMalloc(&c->cb, n * sizeof(float))
      || cudaMalloc(&c->cr, n * sizeof(float)) || cudaMalloc(&c->bytes, 3 * n)
      || cudaMalloc(&c->sum, sizeof(double)) || cudaMalloc(&c->solids, MAX_SOLIDS * sizeof(fg_solid)))
    return 2;
  if (cudaMemcpy(c->ray, ray, 2 * n * sizeof(float), cudaMemcpyHostToDevice)) return 3;
  c->w = w; c->h = h; c->used = 1; c->fresh = 1;
  return 0;
}

// One picture of camera cam: rendered anew when render is set (rot: the camera's rotation, world from camera, row by
// row), accumulated by share, exposed at gain with the sensor's noise drawn from seed, MJPEG with the quality tables
// (64 steps each, natural order) when mjpeg is set; the bytes RGB into out, the exposed picture's mean luminance into
// mean. 0 when made.
int fg_frame(int cam, int render, const float *rot, int nsolids, const fg_solid *solids, float share, float gain,
             float noise_level, unsigned long long seed, int mjpeg, const float *qluma, const float *qchroma,
             unsigned char *out, double *mean) {
  if (!device_ok || cam < 0 || cam >= MAX_CAMERAS || !cams[cam].used || nsolids > MAX_SOLIDS) return 1;
  cam_t *c = &cams[cam];
  int n = c->w * c->h, threads = 256, blocks = (n + threads - 1) / threads;
  static float *d_rot = 0, *d_ql = 0, *d_qc = 0;
  if (!d_rot) { cudaMalloc(&d_rot, 9 * sizeof(float)); cudaMalloc(&d_ql, 64 * sizeof(float)); cudaMalloc(&d_qc, 64 * sizeof(float)); }
  if (render) {
    cudaMemcpy(d_rot, rot, 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(c->solids, solids, nsolids * sizeof(fg_solid), cudaMemcpyHostToDevice);
    render_kernel<<<blocks, threads>>>(c->w, c->h, c->ray, d_rot, nsolids, c->solids, c->scratch);
  }
  if (c->fresh) { cudaMemset(c->shown, 0, 3 * (size_t)n * sizeof(float)); share = 1.0f; c->fresh = 0; }
  cudaMemset(c->sum, 0, sizeof(double));
  expose_kernel<<<blocks, threads>>>(n, c->scratch, c->shown, share, gain, noise_level, (uint64_t)seed, c->r, c->g, c->b, c->sum);
  if (mjpeg) {
    int cw = (c->w + 1) / 2, ch = (c->h + 1) / 2;
    cudaMemcpy(d_ql, qluma, 64 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_qc, qchroma, 64 * sizeof(float), cudaMemcpyHostToDevice);
    ycc_kernel<<<(cw * ch + 255) / 256, 256>>>(c->w, c->h, c->r, c->g, c->b, c->y, c->cb, c->cr);
    int by = ((c->w + 7) / 8) * ((c->h + 7) / 8), bc = ((cw + 7) / 8) * ((ch + 7) / 8);
    dct_kernel<<<(by + 63) / 64, 64>>>(c->y, c->w, c->h, d_ql);
    dct_kernel<<<(bc + 63) / 64, 64>>>(c->cb, cw, ch, d_qc);
    dct_kernel<<<(bc + 63) / 64, 64>>>(c->cr, cw, ch, d_qc);
  }
  bytes_kernel<<<blocks, threads>>>(c->w, c->h, mjpeg, c->r, c->g, c->b, c->y, c->cb, c->cr, c->bytes);
  double s = 0.0;
  if (cudaMemcpy(&s, c->sum, sizeof(double), cudaMemcpyDeviceToHost)) return 2;
  if (cudaMemcpy(out, c->bytes, 3 * (size_t)n, cudaMemcpyDeviceToHost)) return 3;
  *mean = s / n;
  return cudaGetLastError() == cudaSuccess ? 0 : 4;
}

}
