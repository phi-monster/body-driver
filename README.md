# body-driver

**A robot's body layer.** It measures what *this* body is — reach, jaw span, tool offset, delivery,
latency, backlash, friction — by moving that body and watching it. Nothing is typed in, nothing is
read out of a URDF or a camera matrix, and anything it has not measured it **refuses to answer**.

It is not a library you query. It is the thing you install once per robot, before anything else.

```
install → plug in → it measures itself → it will now answer questions about this body
```

---

## Why a body layer at all

Every stack that drives a robot ends up carrying a handful of numbers about the machine: how far the
fingertip is from the flange, how wide the jaws open, how far it can reach, how much of a commanded
step actually arrives. Those numbers are almost always **typed in**.

A typed-in number cannot be re-measured on a new machine, so a stack built on one was never portable
— it was configured by hand and described as portable. It also never notices when the robot changes:
a heavier battery, a bent camera mount, a worn gearbox, a different gripper. The number stays right
in the file and wrong in the world.

This layer exists to make that impossible:

* **Measured or refused, never guessed.** Values carry their provenance. Handing it a number that is
  not a measurement is rejected at the door.
* **Refusals are answers.** "Never measured", "stale", "the thing it depended on moved", "asked
  outside the range I probed" are four different facts and are never merged into one.
* **Zero dependencies, no allocator.** The driver builds for targets that have no allocator story.
  Anything needing a dependency (a WebSocket handshake, msgpack) lives in a *plug*, outside.
* **No robot names anywhere.** A mechanical check (`driver/check_purity.sh`) fails the build if the
  driver source mentions a specific robot, simulator, or benchmark. A rule that is only in a comment
  is not a rule.

---

## Install

Requires a Rust toolchain. Nothing else.

```bash
git clone https://github.com/phi-monster/body-driver
cd body-driver
./install.sh
```

That builds the driver, runs its test suite and its purity check, and puts `bl-calibrate` on your
path. It does not touch your robot.

---

## Calibrate — the only command you run per robot

Start your robot's controller so it is speaking, then:

```bash
bl-calibrate --listen 9077          # your controller connects to us
```

You write **no code and no config**. The plug listens to one frame and works out the shape of what
this machine reports — which arrays are joint angles, which is an end-effector pose, which is the
gripper, which is a camera — **from the shapes and value ranges, never from key names**. Key names
belong to one machine; shapes do not. It prints what it worked out:

```
[认] joints: state.left_arm_joint_state · state.right_arm_joint_state
[认] pose:   state.left_ee_pose · state.right_ee_pose
[认] jaw:    state.left_ee_joint_state · state.right_ee_joint_state
[认] camera: vision.cam_head.color
```

If two candidates cannot be told apart, it **refuses and names them**. A layout that was guessed
wrong lets a whole calibration run to completion and produce numbers that look perfectly normal —
and numbers that look normal are worse than no numbers, because they get used.

Then it runs the power-on schedule: ask the driver what it still owes itself, perform the motions
that quantity needs, hand the raw samples back, repeat until nothing is owed. **Dependency order is
computed, not configured** — measure the jaw span before friction, because "it is still between the
fingers" is read off the jaws.

Each quantity ends as a value with an uncertainty, or as a **named refusal**:

```
[量] round 3: gripper_span — because NeverMeasured
      🟢 measured: [0.0803]
[量] round 4: friction — because NeverMeasured
      🔴 declined: Inconsistent — it never slipped across the whole sweep, so all this establishes
         is mu > tan(theta_max): a lower bound, not a value. Probe steeper.
```

A refusal is an output. Omitting it would make the body look like it owes less than it does.

---

## What it measures

| quantity | how the body finds it out for itself |
|---|---|
| image jacobian | move a known step, see how far the picture moves |
| hand pixel | which pixels are mine — waggle the jaws, keep what responds |
| gripper span | commanded opening vs the separation its own camera sees |
| tool offset & axis | spin the wrist; the working point sweeps an arc whose radius *is* the offset |
| reach | targets at growing radius from its own measured base; attained or not |
| step delivery | commanded magnitude vs achieved, across magnitudes |
| latency | one step from rest; how many control periods before anything moves |
| backlash | push both ways; the dead band is the slop |
| contact threshold | how "I touched something" reads on this body — **with no force sensor** |
| **friction** | hold something, tilt, and the angle it starts sliding at is `atan(mu)` |
| arm weight | hold still in many poses; what it costs to do nothing is the weight |
| home pose | where "back where I started" is, to its own repeatability |
| floor | the supporting surface, read as a stop in the delivered-motion signal |

---

## Layout

```
driver/            zero dependencies, no allocator, no robot names
  core/            measure · store with provenance · judge expiry · REFUSE
  selfcal/         the motions each quantity needs, and in what order
  contact-set/     what a task IS: which points to touch, which way to push, what the object does
  contact-gen/     given a point cloud and a body, where this shape can be taken hold of
  contact-exec/    a contact set becomes waypoints for whatever hand is present
  point-gen/       pixels become surface points — depth, two cameras, one camera and motion, touch
  abi/             the C ABI; the driver is callable from anything
  conformance/     the same checks from C, Ada and Python
plug/
  ws/              one plug: a robot that speaks msgpack over a WebSocket
```

A different robot means a different **plug** — three methods: read a frame, send a command, report
identity. The calibration program, the phases, the dependency order and the refusal rules do not
change.

---

## License

AGPL-3.0-or-later.
