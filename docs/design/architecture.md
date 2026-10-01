# Architecture of the body driver

This note explains how the driver is put together and why. It is written for
people who change the driver; people who only connect a robot or a brain need
the documents next to it (body protocol, brain service, instrument service,
body file, log lines), not this one.

## The job

A robot connects with nothing but its readings, its commands and its camera
images. The driver measures the body from these alone, keeps that measurement
up to date, and lets a brain that speaks only the body language
(`driver/LANGUAGE.md`) make it do things. Nothing about a body is declared by
its user: no kinematics, no calibration, no units, no calibration board.

## Layers

Each layer depends only on the layers below it.

| Layer | Package | Responsibility |
|---|---|---|
| 1 | `Driver.*` (core) | protocol, numerics, statistics, uncertainty, recording, services, the beat channel |
| 2 | `Driver.Robot` | the body: groups and roles, response and noise, stillness, kinematics, lenses and mounts, link shapes, motion primitives, the body file, boot; `Driver.Robot.Hand` measures the hands |
| 3 | `Driver.World` | things, the surfaces they rest on, places, the remembered scene |
| 4 | `Driver.Action` | from a wanted change of the world to motion, and back to one ending word |
| 5 | `Driver.Brain` | rounds with the brain: keyboard, prompt, names, program execution |

The public specification of each layer is the contract with the layers above.
Changing what another layer uses goes through whoever owns the interfaces.

## Estimation is separate from decision

Estimators are pure functions of what crossed the boundary: observations, the
commands that were sent, and service replies. They never choose anything.
Deciders (boot, the motion primitives, action, the brain rounds) choose
commands and read the estimates.

This split is what makes every estimator testable offline: a recording of
another driver's run feeds the same estimators the same stream, and their
output can be scored against simulator truth (`driver/tools/replay`).

Causality: the estimators receive each observation together with the last
command sent before it arrived, the one in effect while it was captured.

## One beat

The main loop owns the connection to the robot. For every observation:

1. it is parsed against the layout learned from the first one;
2. every estimator observes it, with the command in effect;
3. the beat is offered to the decider task;
4. the reply carries the decider's command, or Hold if the decider is busy.

A decider is ordinary sequential code that calls `Driver.Beats.Next` and
`Driver.Beats.Send` (through `Driver.Robot.Motion`, the only place that may).
It may read and change the models only between `Next` returning and `Send`;
the main loop touches them only outside that window. This ownership rule is
what lets the models stay plain objects without locks. A decider that is
waiting for the brain is simply not in `Next`, and the robot holds meanwhile.

Holding has one rule for every group: the last target sent in this episode;
if none, this beat's reading; if no reading, the last value sent; if nothing
was ever sent, the key is left out. No value is ever invented. Stopping a push
against an obstacle is a decision, made by setting the target to the reading.

## Uncertainty and the one test

Every measured quantity carries its standard deviation. Every gate in the
driver is the same test: a difference is significant when it exceeds Z times
its own measured sigma (`Driver.Uncertain.Significant`), with Z = 3 the only
confidence level (`Driver.Conventions`). Vector differences are tested along
their own direction, so points, directions and poses use the same rule.

There is one estimator per quantity, computed in one place, fed by every
observation the body gives. An optional sensor (motor current, an IMU, depth)
is one more observation for the same estimator: it shrinks sigma and with it
every gate; without it sigma is larger, or the quantity is reported unknown.
There are no primary and backup paths and no branches on which body it is.

## Numbers

Every numeric literal in the driver has a stated origin in
`driver/numbers.tsv`: structure, mathematics, numerics, statistics or format.
A number that describes a body, a scene or a behavior is a tuning number and
is rejected by the gate; such a quantity has to be measured. The only chosen
numbers are the two conventions in `Driver.Conventions`.

## Gates

`tools/check.sh` runs on every merge and install: numbers, no benchmark or
robot names in driver code, no Python in the driver, no CJK characters in
driver sources, no action words in the contact set, no tutorial sentences in
what the brain is shown, commands only through `Driver.Robot.Motion`, a build
without warnings, and the self test. Every gate is checked to fail on a
planted violation.

## Recording and replay

`harness/record/wire_proxy.py` records the conversation between a robot and
any driver at the protocol boundary, unchanged. The driver itself can record
everything it exchanges, service replies included (`--record`). The format is
described in `Driver.Recording`. `driver/tools/replay` feeds a recording
through the estimators and writes the measured body for scoring; truth comes
only from the simulator side and is never read by the driver.
