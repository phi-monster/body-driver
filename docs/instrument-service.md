# The instrument service

Beside the driver runs a small HTTP service, the instrument. With learned,
task-free models it measures a few things in pictures and hands back numbers:
where a pixel of one picture is in another (matching), and which pixels of a
picture are the thing a box or a few points pick out (segmentation). The
driver trusts only the numbers, never the model: every answer is checked by
geometry where it is used, and the model's own confidence is recorded but
never used as a gate. Changing the model changes only the service.

The reference service is [`harness/instruments/serve.py`](../harness/instruments/serve.py)
(RoMa for matching, SAM 2.1 for segmentation, weights pinned by sha256). It
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

## 2. `POST /frame`: keep a picture

```json
{"image": "<BMP base64>"}
```

answered with

```json
{"ok": true, "id": 17}
```

The service keeps the picture, and later questions name it by its number
instead of sending it again (`a_id`, `b_id` below). A reply without a
non-negative number is refused.

## 3. `POST /match`: where points of one picture are in another

```json
{"a": "<BMP base64>", "b": "<BMP base64>", "num": 0, "points": [[u, v], ...], "back": true}
```

`"a_id"` and `"b_id"` (numbers from `/frame`) may stand for `"a"` and `"b"`.
The driver always sends `"num": 0`: it asks only about the points it names.

```json
{"ok": true,
 "points": [[ub, vb, certainty], ...],
 "back":   [[ua, va], ...]}
```

- `points` answers the request's points one for one, in order: where each
  point of picture a is in picture b, and the model's certainty from 0 to 1.
  A point the service cannot answer is written with a negative coordinate.
- `back`, when asked for: where each answer in b matches back to in a. The
  distance between a point and its round trip is how well the match agrees
  with itself, and that, not the certainty, is what the driver judges.
- A reply with another number of answers than points asked, or without a
  round trip for every point when one was asked, is refused whole: nothing
  of it is used.

## 4. `POST /segment`: the pixels of a thing

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

## 5. Where the driver uses it

| step | questions |
|---|---|
| booting: matching what an eye saw before and after a push, mounting the eyes | `/frame`, `/match` with round trips |
| binding a name: which pixels the brain's eye boxed | `/segment` |

Without the instrument, a step that needs it says so and measures nothing.
