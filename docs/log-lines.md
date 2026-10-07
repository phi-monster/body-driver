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

Recognizing the groups (every commandable channel probed, then each group pushed channel by channel):

| line | meaning |
|---|---|
| `boot: every commandable channel moved together is first seen at <x> reading units, after <n> doublings` | every commandable channel was moved away from its hold by one amount, doubled from the smallest step the readings can tell, until some eye saw the body move; each channel's own probe starts from that amount |
| `boot: group <g> channel <c> is seen when moved by <x> reading units` | an eye saw this channel move at that amount, confirmed by moves back and forth; the group's pushes use it |
| `boot: group <g> channel <c> is at its end <upwards or downwards>: it delivered nothing that way up to <x> reading units, while it answered the other way` | a limit on one side (a closer resting at its upper limit): that way is not asked further and not pushed again |
| `boot: group <g> channel <c> moves nothing any eye sees, up to where it stops following` | its reading followed each way and then went no further, and no eye saw it move; it is not pushed |
| `boot: group <g> channel <c> answers neither way up to <x> reading units, at every level it was asked either way: dead or disconnected for this boot; it is left alone` | its reading followed no ask either way, at every level, as many levels each way as a float has bits of precision (a deadband yields to a large enough one); nothing the other channels needed bounds it; it is not pushed |
| `boot: group <g> channel <c> moves nothing any eye sees, and its reading's noise is not measured, so how its reading followed is not known` | the model had no noise for the channel even after measuring again, so nothing tells a reading that followed from one that did not; the eyes' evidence alone was asked, up to every level; it is not called dead |

The kinematics (an arm's joints and the lens of the eye it carries, fitted from its keyframes):

| line | meaning |
|---|---|
| `kinematics: arm <a> fitted from <k> sightings of <f> keyframes, <u> fit, median <m> px, noise <s> px; focal <fx> x <fy> px; errors, px: a sighting's own <e>, a point's in every keyframe <p> (half as alike at <h> px apart), a keyframe's added for its points <q>; the clip took <c>` | the fit of arm <a>, of <u> of the <k> sightings; the errors its residuals show, which its covariance rests on: a sighting's own, the part of it a point has in every keyframe (and the distance at which two points have half as much of it alike), what a keyframe adds for its points, and the share of the covariance's spread the clip to positive semi-definiteness took away (0 when none) |
| `kinematics: arm <a> not fitted (stage <n>): <why>` | the sightings do not determine the arm; the last fit that held is kept |

The hand (measured at boot):

| line | meaning |
|---|---|
| `hand: closer group <g> on arm <a> is watched in eye <e>[, its own]` | the group that closes a hand, and the eye that watches it |
| `hand: closer group <g> channel <c> seen at both ends in eye <e>; asking where <n> pixels went` | the closing was seen open and shut; the instrument is asked to match pixels between the two |
| `hand: the instrument did not answer for closer group <g> channel <c>: <why>` | that channel's sweep is not used |
| `hand: closer group <g> channel <c> has no visible step measured; not swept` | the channel's response is unmeasured, so it is not swept |
| `hand: closer group <g> of arm <a> is not swept: no eye on its arm watches it, so no hand is made of it` | a closer of the body that no arm-carried eye sees move |
| `hand: closer group <g> channel <c> in eye <e>: <n> lobes, closed at the high reading` (or `at the low reading`, or `closing direction not significant: their distances changed by <v> +- <s> pixels between the ends`, or `closing direction not known: there is nothing to compare their distances with`) | what the channel's sweep found in that eye |
| `hand: no hand was found, so nothing is pressed; below, what became of each closer` | no closer gave a hand; the lines of `hand: measured` say what became of each |
| `hand <n>: lobe <l> at <opening> is not pressed: its tip is not seen in the hand's eye at this opening` | the lobe has no tip pixel at that opening |
| `hand <n> is not pressed: its closer group is no longer a closer of its arm` | the body re-read its roles |
| `hand <n>: lobe <l> at <opening> pressed once, straight, and not tilted: no other line of sight of the hand is known to tilt away from` | a lone lobe: no tilt to try |
| `hand <n>: lobe <l> at <opening> tilted one way (the other):<k> presses, then <why>` | how far the tilts went on that side: to a right angle, until a press could not be made, or until the latest did not agree with the others |
| `hand <n>: a press at the <opening> opening, <k> kept; it agrees with the others` (or `does not agree`) | a fingertip press on a surface was recorded, and whether the fit of all the presses so far keeps it |
| `hand <n>: its arm was fitted again; the <k> presses kept take their poses from the new fit` | the arm's table moved, so every press kept took its pose again from the arm's readings it kept |
| `hand <n>: a press with the closer at neither measured opening is not used` | the hand was neither at its measured open nor at its shut opening |
| `hand <n>: pressing lobe <l> at <opening>, aimed by turning the hand <a> rad, <h> +- <s> above the surface the presses so far fixed` (or `nothing yet predicting the surface below its tip: doubling from <x> until blocked`) | a press begins, in the arm's own frame and unit: how far the hand turned to point the lobe down, and the tip's predicted height above the table, or none |
| `hand <n>: press of lobe <l> at <opening>: <k> pushes, <f> fast and <b> within Z sigma of the contact its presses predict, <d> doubling from <x> with nothing predicting it; blocked, the last push by <y> after lowering <t>` | how the press went down, in the arm's unit |
| `hand <n>: up, the arm's pose or the eye's mount is unmeasured; no press` | a press cannot be aimed yet: the arm is not fitted, or its eye saw no table |
| `hand <n>: cannot aim a press: <why>` / `hand <n>: cannot press lower: <why>` | the motion planner refused the press |
| `hand: measured` | what was measured about every hand, one line each; for each closer an eye watches that gave no hand: `closer group <g> on arm <a> in eye <e>, its own: no hand; channel <c>: <account>;` (or `not its own, no hand is made of it;`), where the account says where the channel's sweep stands: its ends not both seen still, nothing yet asked of the instrument, its answer not come, the instrument could not (or can never) answer, nothing in the eye moves between its ends, half of the eye's picture or more changes between them (what moved cannot be told from what did not), or the lobes found (as above) |

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
