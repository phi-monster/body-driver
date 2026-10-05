# What the log says

The driver writes one line per event to standard output, flushed at once, so
a log read live is never behind:

```
[topic] text
```

The topic is one of `core` (the connection, the protocol, the main loop),
`robot` (measuring the body: groups, eyes, hands), `world` (things and
surfaces), `action` (stretches) and `brain` (rounds with the brain). A line
that carries several lines of its own continues them below it, the brain's
indented with `  | `. Numbers in angle brackets below stand for what the
driver fills in.

This page lists every line the driver writes now. Lines of layers still
being written are added here as those layers land.

## core

| line | meaning |
|---|---|
| `listening on port <n>` | the WebSocket server is up; the robot may connect ([`body-protocol.md`](body-protocol.md)) |
| `cannot listen on port <n>` | the port is taken or not allowed; the driver exits |
| `usage: body_driver --listen PORT [--eye HOST:PORT] [--inst HOST:PORT] [--body FILE] [--record FILE]` | `--listen` is missing; the driver exits |
| `the robot connected` | a client finished the handshake |
| `the robot disconnected; waiting for it on the same port` | the client went away; nothing measured is lost, and a client that connects again is served on |
| `the robot reports:` | the first observation was recognized; one line per camera, group and unused leaf follows (below) |
| `waiting: the driver needs at least one camera and one group of readings, got <layout>` | the observation lacks a camera or a group; the next one is looked at again |
| `a robot message that is not a protocol message was ignored` | a message that is not msgpack, or not a map |
| `the body could not be measured; holding still` | the boot found the body breaking the porting contract; the reason was logged by the boot; every reply holds |

The layout lines after `the robot reports:`:

```
camera <k>: <path> <width> x <height>[, depth <path>]
group <k>: <path>, <n> values, command key <key>[, echoed at <path>]
group <k>: <path>, <n> values, not commandable
unused: <path>
instruction: present
```

## robot

The boot's body file (`--body`, [`body-file.md`](body-file.md)):

| line | meaning |
|---|---|
| `boot: the body is kept in <file> after <stage>` | the body measured so far was written to the file, after recognizing the groups and after sweeping the arms; the line `the body as measured:` and its table follow the hands, the last write |
| `the body file <file> cannot be written` | a write failed; the boot goes on |

The hand (measured at boot):

| line | meaning |
|---|---|
| `hand: closer group <g> on arm <a> is watched in eye <e>[, its own]` | the group that closes a hand, and the eye that watches it |
| `hand: closer group <g> channel <c> seen at both ends in eye <e>; asking where <n> pixels went` | the closing was seen open and shut; the instrument is asked to match pixels between the two |
| `hand: the instrument did not answer for closer group <g> channel <c>: <why>` | that channel's sweep is not used |
| `hand: closer group <g> channel <c> has no visible step measured; not swept` | the channel's response is unmeasured, so it is not swept |
| `hand <n>: a press at the <opening> opening, <k> kept; it agrees with the others` (or `does not agree`) | a fingertip press on a surface was recorded, and whether the fit of all the presses so far keeps it |
| `hand <n>: its arm was fitted again; the <k> presses kept take their poses from the new fit` | the arm's table moved, so every press kept took its pose again from the arm's readings it kept |
| `hand <n>: a press with the closer at neither measured opening is not used` | the hand was neither at its measured open nor at its shut opening |
| `hand <n>: pressing lobe <l> at <opening>, aimed by turning the hand <a> rad, <h> +- <s> above the surface the presses so far fixed` (or `nothing yet predicting the surface below its tip: doubling from <x> until blocked`) | a press begins, in the arm's own frame and unit: how far the hand turned to point the lobe down, and the tip's predicted height above the table, or none |
| `hand <n>: press of lobe <l> at <opening>: <k> pushes, <f> fast and <b> within Z sigma of the contact its presses predict, <d> doubling from <x> with nothing predicting it; blocked, the last push by <y> after lowering <t>` | how the press went down, in the arm's unit |
| `hand <n>: up, the arm's pose or the eye's mount is unmeasured; no press` | a press cannot be aimed yet: the arm is not fitted, or its eye saw no table |
| `hand <n>: cannot aim a press: <why>` / `hand <n>: cannot press lower: <why>` | the motion planner refused the press |
| `hand: measured` | what was measured about every hand, one line each |

## brain

### Settings and episodes

| line | meaning |
|---|---|
| `BL_BRAIN_SAMPLING: both questions carry it as it is: <json>` | the deployment's sampling is merged into every request ([`brain-service.md`](brain-service.md), section 2) |
| `BL_BRAIN_SAMPLING: it is not set, so the service samples by its own defaults` | nothing is merged |
| `BL_BRAIN_SAMPLING: it is not one JSON object (<why>), so it is not used: <text>` | the variable is not usable |
| `BL_BRAIN_SAMPLING: it sets "<member>", which the driver writes itself, so it is not used: <text>` | the variable sets model, messages, stream, structured_outputs or response_format |
| `a new episode: <instruction>` | the rounds begin, with the person's words |
| `the last episode called the brain <n> times (<a> programs, <b> where-is-it questions)` | the count of the episode that ended |
| `the brain said done after <n> rounds` | the task is finished; the rounds end |
| `the rounds stopped on <exception>: <message>; holding still until the next episode` | a failure inside the rounds; the body holds and the next episode starts over |

### A round

```
[brain] round 2: the brain looks through eye 1, quantity keyboard
[brain] call 3 of this episode: write a program, 1.4 s, complete (line 2 is done outside every block), 2 lines kept
[brain] the program:
[brain]   | do the scissors height up until free
[brain]   | done
[brain] call 4 of this episode: where is "the scissors", 0.9 s, box from (472, 176) to (507, 262)
[brain] "the scissors": thing 1 (eye 1 boxed it)
[brain] what happened:
[brain]   | line 1: do the scissors height up until free -- ended free: ...
[brain]   | line 2: done came before you saw how the lines above it ended, so I ask you again.
```

| line | meaning |
|---|---|
| `round <n>: the brain looks through eye <k>, <quantity \| full \| speech-only> keyboard` | a round begins: the large picture's eye, and the keyboard this round ([`language.md`](language.md), section 2) |
| `call <n> of this episode: write a program, <s> s, <how> (<why>), <k> lines kept` | the brain answered; how reading ended (`ended by itself`, `complete`, `ran away`, `stopped by the service`, `failed`) and why; how many finished lines are kept |
| `call <n> of this episode: where is "<name>", <s> s, box from (<u>, <v>) to (<u>, <v>)` | an eye pointed the name out, in that eye's pixels |
| `call <n> of this episode: where is "<name>", <s> s, not here (<why>)` | that eye cannot see it; the next eye is asked |
| `call <n> of this episode: where is "<name>", <s> s, no answer (<why>)` | the service failed; no other eye is asked for the name |
| `the program:` | the kept lines, as written |
| `"<name>": thing <n> (<account>)` / `place <n> (...)` / `not bound (<account>)` | how each name of the program was bound, with every step that led there ([`language.md`](language.md), section 7) |
| `refused before anything moved:` | the program was refused: the line, why, and what to write instead |
| `the brain says: <sentence>` | a `say` line ran |
| `what happened:` | what the next round will tell the brain, line by line |
| `the brain looks through eye <k> from the next round on` | a `say look = <k>` ran |
| `new words from the person: <words>` | the task changed; the next round carries the new words |
| `no eye has a picture this beat; looking again` | the round waits for a picture |
| `the episode ended while the brain was writing` | the answer is dropped; nothing of it runs |
| `no program: <why>` | the brain service could not be read; nothing moves this round |

## Tools

`driver/bin/replay`, `frame`, `score`, `feed`, `proxy`, `brain_measure` and
`fake_brain` print their own lines; each tool's first comment says what it
prints.
