# The instrument service

Beside the driver runs a small HTTP service, the instrument. With a learned,
task-free model it measures one thing in pictures and hands back numbers:
which pixels of a picture are the thing a box or a few points pick out
(segmentation). The driver trusts only the numbers, never the model: every
answer is checked where it is used, and the model's own confidence is
recorded but never used as a gate. Changing the model changes only the
service. Where a pixel of one picture is in another the driver tells by
itself, with its own aligner: it places what its eyes show from the pictures
they took from other places, and finds a thing again from one picture to the
next ([`design/alignment.md`](design/alignment.md)).

The reference service is [`harness/instruments/serve.py`](../harness/instruments/serve.py)
(SAM 2.1, weights pinned by sha256). It picks CUDA, else Apple's MPS, else the CPU
(`INST_DEVICE` overrides), and loads the checkpoint `SAM_CKPT` with the config
`SAM_CFG`: on a Mac or a Raspberry Pi the tiny model (`sam2.1_hiera_tiny.pt`,
`configs/sam2.1/sam2.1_hiera_t.yaml`). A path it does not serve answers 404. It
is one of the two programs outside the driver that may be Python, because its
models only run in PyTorch; the driver itself talks to it like to any other
service.

## 1. Where the driver connects

```
body_driver --listen PORT --inst HOST:PORT ...
```

Every question is one HTTP/1.1 `POST http://HOST:PORT/<path>` with a JSON
body, answered with JSON. Without `--inst`, every question fails with "no
address was given for the instrument service", and what needs the instrument
is reported as not measured.

- Pictures travel as a 24-bit BMP file encoded in base64 (no `data:` prefix),
  at the picture's own size.
- Pixel coordinates: the origin is the top left corner of the top left
  pixel, u grows to the right and v downwards, in pixels, with fractions.
  The centre of the top left pixel is (0.5, 0.5).
- `"ok": false` with `"err"` means the service could not answer; the reason
  goes into the log.

The driver never waits on the instrument in its main loop: the parts of the
driver that estimate submit a question and read the reply on a later beat;
only the parts that decide (booting, binding a name) ask and wait.

## 2. `POST /segment`: the pixels of a thing

```json
{"image": "<BMP base64>", "box": [x0, y0, x1, y1], "points": [[u, v, 1], [u, v, 0], ...]}
```

`box` is in pixels; each point is on the thing (1) or off it (0). At least
one of the two is given. The name binder sends the box the brain's eye gave
for a name ([`language.md`](language.md), section 7) and no points.

```json
{"ok": true, "w": 640, "h": 480, "score": 0.93, "runs": [r0, r1, r2, ...]}
```

- `runs` are run lengths over the picture read row by row from the top: a
  run of pixels off the thing, then a run on it, alternately, starting off
  it. They must add up to `w x h`.
- `w` and `h` must be the picture's size.
- `score` is the model's own estimate of its quality; it is recorded only.
- A reply for another picture size, or whose runs run past the picture or
  do not cover it, is refused.

## 3. Where the driver uses it

| step | questions |
|---|---|
| binding a name in an eye that stands still in the world: which pixels the brain's eye boxed (an eye the body held at several places tells them by itself) | `/segment` |
| a thing seen anew in another eye, where its points fall there, or found again in one after it moved | `/segment` |

Without the instrument, a step that needs it says so and measures nothing.
