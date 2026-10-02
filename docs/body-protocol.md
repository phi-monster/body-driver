# The body protocol

This is how a robot, or a simulator, connects to the body driver. The driver
needs no drawing of the robot and no configuration file. The robot reports
its readings and its pictures; the driver works out what each group of
numbers is by pushing it and looking, and from then on it sends the robot
commands in the robot's own shape.

## 1. The connection

```
body_driver --listen PORT [--eye HOST:PORT] [--inst HOST:PORT] [--body FILE] [--record FILE]
```

- The driver is a WebSocket server on `PORT`; the robot is the client. One
  robot at a time.
- Every message is one binary frame holding a msgpack map (text frames are
  read the same way). Pings are answered.
- When the connection drops, the driver waits for the robot on the same
  port. Nothing measured is lost, and a reconnection is not a new episode.
- `--eye` is the brain ([`brain-service.md`](brain-service.md)), `--inst` the
  instrument ([`instrument-service.md`](instrument-service.md)), `--body` the
  body file ([`body-file.md`](body-file.md)). `--record FILE` writes
  everything that crosses the driver's boundary (robot messages, replies,
  service calls), so a run can be replayed (`driver/bin/replay`).

## 2. What the robot sends

Every message is a map:

| key | what it is |
|---|---|
| `message_type` | `hello`, `prepare_case`, `reset`, `call`, `infer`, `trial_end` or `heartbeat` |
| `payload` | a map: see below |
| `message_id`, `evaluation_id`, `action_case_id`, `trial_id`, `repeat_index`, `sent_at` | optional; returned unchanged |
| `step` | optional; returned unchanged (0 when absent) |

- `payload.obs` (or `payload.observation`) is this beat's observation
  (section 3). Any message may carry one; each observation is a new beat.
- To ask for an action: `message_type` `call` with `payload.func_name`
  `get_action`, usually with this beat's observation.
- `reset` starts a new episode: the driver forgets this world (the names the
  brain gave, the places, which pictures showed what) and keeps what it
  measured about the body.

## 3. The observation: recognized by shape, not by key names

An observation is a map, nested as deep as you like. The driver flattens it
into leaves, each with the path of keys from the root, and recognizes each
leaf by its shape:

| leaf | recognized as |
|---|---|
| bytes (`u1` or `i1`) shaped height x width x 3 | a colour camera, RGB |
| floats shaped height x width, the size of a camera | that camera's depth, paired with the camera whose key path shares the longest prefix with it |
| any other float grid (intrinsics, extrinsics, ...) | recognized and left unused: the driver measures its eyes itself |
| a string `instruction` at the top level | the person's words: the task, handed to the brain |
| any other numeric array of one dimension (or a number) | a group of readings |

Arrays may be msgpack arrays or msgpack-numpy maps
(`{"nd": true, "type": "<f4", "shape": [480, 640], "data": <bin>}`).

The driver starts once an observation has at least one camera and one group
of readings; until then it says what it lacks and waits for the next one.
Keep every camera in every observation: a camera missing from one beat is a
gap at its place, not a shift of the others.

### Command keys

The driver commands a group by the last key of its path. When the robot
reports back the command it received under the same last key as the reading
(two groups with the same last key and the same size: one the reading, one
its echo), that key is the group's command key. When the robot echoes
nothing anywhere, every group is tried as commandable under its last key.

### What each group is

The driver does not guess a group's meaning from its size or its values.
At boot it pushes each group a little and watches its readings and every
eye: an arm moves an eye it carries; what closes a hand shows only a patch
moving; a base moves every eye at once. What a robot reports that the
driver must not use (a pose of the hand computed by the robot, true object
poses) may be in the observation: the driver reads only joint readings,
commands and images.

### The porting contract

A body runs every program of the language when it does three things
([`driver/LANGUAGE.md`](../driver/LANGUAGE.md), section 10):

1. Each group can be pushed on its own.
2. When it is pushed, something changes in some picture: the body's own
   part, or the whole world as seen by an eye the group carries.
3. When it was pushed and did not move, its reading says so.

A body that breaks one of them is told which, in the log, and the driver
holds still. A part that moves while no eye sees anything change is mute:
add an eye or a mirror, no code can fix it.

## 4. What the driver answers

Every request gets one reply. Its `message_type` is the request's with a
suffix: `hello_ack`, `prepare_case_ack`, `reset_result`, `call_result`,
`infer_result`, `trial_end_ack`, `heartbeat_ack` (and `error` for an unknown
type). The optional identifiers and `step` come back unchanged.

| request | payload of the reply |
|---|---|
| `hello` | `{"ok": true, "server": "xpolicylab_policy_server", "server_instance_id": "body-driver"}` |
| `call` `get_action` | `{"result": [<action>]}`, or `{"result": []}` before any observation was recognized |
| any other | `{"ok": true}` (`false` for an unknown type) |

### The action

A map from command key to a list of numbers, as many as the group's reading:
`{"<command key>": [x1, x2, ...], ...}`.

- A group the driver moves this beat gets its target.
- A group it does not move holds: it gets the last target the driver sent it
  this episode; if it was never commanded this episode, its reading of this
  beat; if this beat has no reading for it, what was last sent for it; and if
  nothing was ever sent, the key is left out. No number is ever made up.
- What closes a hand gets the last target given to it this episode, not its
  reading, since an outside push changes the reading and holding the reading
  would lock the push in.

## 5. Time

- One beat is one observation. The driver counts beats as they arrive, and an
  episode counts from its `reset`.
- Report every beat's readings and pictures together, as they are at that
  beat.
- While the driver is busy (asking the brain, which takes seconds) every
  reply holds, so the robot always gets an answer in time.
