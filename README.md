# Body Driver

One driver for any robot body. A robot connects with nothing but its joint
readings, its commands and its camera pictures. The driver measures the body
from these alone: which group of numbers moves what, the kinematics, the
lenses and where every eye is mounted, the hands and their fingertips. It then
lets a brain that speaks only the body language (`driver/LANGUAGE.md`) make
the body do things. No URDF, no vendor calibration, no units, no calibration
board, no tuning numbers.

## Status

The driver is being rewritten from scratch in `driver/` (Ada 2022). It boots a
two-arm robot in simulation and measures its arms and lenses; the hands are not
yet within acceptance. The plan, in Chinese, is `驱动重写.md`.

## Documents

- Connecting a robot: [`docs/body-protocol.md`](docs/body-protocol.md)
- The brain service: [`docs/brain-service.md`](docs/brain-service.md)
- The instrument service: [`docs/instrument-service.md`](docs/instrument-service.md)
- The body file: [`docs/body-file.md`](docs/body-file.md)
- The log lines: [`docs/log-lines.md`](docs/log-lines.md)
- How the driver is put together: [`docs/design/architecture.md`](docs/design/architecture.md)
- The language the brain speaks: [`driver/LANGUAGE.md`](driver/LANGUAGE.md)

## Building

```
cd driver && alr build
```

`tools/check.sh` runs every gate (build, numbers, names, no Python, English
sources, the self test, dead code). `driver/bin/selftest` runs the self test
alone; `driver/bin/replay` feeds a recording through the estimators and
`driver/bin/score` scores the result against simulator truth.

## License

AGPL (`LICENSE`), or a commercial license (`LICENSE-COMMERCIAL.md`). See
`CONTRIBUTING.md` and `CLA.md` before sending changes.
