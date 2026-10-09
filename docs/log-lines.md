# What the log says

The driver writes one line per event to standard output, flushed at once, so
a log read live is never behind:

```
@<beat> [topic] text
```

`@<beat>` is the number of observations the robot had sent when the line was
written (the replay of a recording gives the same number); lines written
before the first observation have none.

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
| `body: group <g> channel <c> stopped at <r> asked <t> from <s>, the push that began at beat <b>: noted; it is an end of the channel upwards (or downwards) when the channel stops at this reading from another pose` | a decider said that the group's latest push, ended Blocked, was stopped by the body itself and not by a surface (an aim, or a descent whose tip stood above the contact the presses predict), and the channel whose part of the shortfall along the ask (its share of the ask times its own shortfall) is more than Z deviations above zero and above every other channel's, from the readings' noise and the scatter of the free pushes, has the reading it stopped at kept with the group's readings there (`Driver.Robot.Motion.Note_Stopped`); one stop is a contact until another pose shows the same reading, and no plan is refused for it |
| `body: group <g> channel <c> stopped at <r> asked <t> from <s>, the push that began at beat <b>: found again from another pose (<w> before): an end of the channel upwards` (or `downwards`) | the channel stopped at the same reading (within what two independent contacts would not meet by chance, and no less than Z deviations of the difference of two readings) with some other channel standing somewhere else: it is an end, the furthest of the two readings; plans past it are refused (`Driver.Robot.Motion.Plan_Reach`), and a press whose aim stopped there is planned once more past it (`hand <n>: the aim of a press ... stopped on an end the arm showed by it`) |
| `body: group <g> channel <c> stopped at <r> ...: the same reading from the same pose as before, the same contact` or `...: at the end the channel has` | the stop is the one already kept (nothing in another channel stood elsewhere), or the channel already has an end in that sense, which explains it: nothing is kept |
| `body: group <g> channel <c> stopped at <r> ...: beyond the end found at <e>: the reading widens it` | the channel stopped beyond the end it has by more than two readings of it can differ: the end was nearer than the joint's limit (two contacts that agreed by chance), and the reading itself widens it (`Driver.Robot.End_Of` is never nearer than the readings seen); nothing is kept |

Recognizing the groups (every commandable channel probed, then each group pushed channel by channel):

| line | meaning |
|---|---|
| `boot: every commandable channel moved together is first seen at <x> reading units, after <n> doublings` | every commandable channel was moved away from its hold by one amount, doubled from the smallest step the readings can tell, until some eye saw the body move; each channel's own probe starts from that amount |
| `boot: group <g> channel <c> is seen when moved by <x> reading units` | an eye saw this channel move at that amount, confirmed by moves back and forth; the group's pushes use it |
| `group <g> answers a push within <n> beats, the longest wait of its pushes from their start to its readings' first motion` | the group's response delay as measured from the stream, logged when it changes; a probe looks that many beats after each push |
| `estimated from <n> beats in <s> s (channels <a>, lags <b>, lock-in <c>, graph <d>, kinematics <e>)` | the heavier estimates were redone from the stream so far (each time the evidence doubles, or when a decider asks), and how long each stage of them took |
| `boot: group <g> channel <c> is at its end <upwards or downwards>: it delivered nothing that way up to <x> reading units, while it answered the other way` | a limit on one side (a closer resting at its upper limit): that way is not asked further and not pushed again |
| `boot: group <g> channel <c> moves nothing any eye sees, up to where it stops following` | its reading followed each way and then went no further, and no eye saw it move; it is not pushed |
| `boot: group <g> channel <c> answers neither way up to <x> reading units, at every level it was asked either way: dead or disconnected for this boot; it is left alone` | its reading followed no ask either way, at every level, as many levels each way as a float has bits of precision (a deadband yields to a large enough one); nothing the other channels needed bounds it; it is not pushed |
| `boot: group <g> channel <c> moves nothing any eye sees, and its reading's noise is not measured, so how its reading followed is not known` | the model had no noise for the channel even after measuring again, so nothing tells a reading that followed from one that did not; the eyes' evidence alone was asked, up to every level; it is not called dead |
| `boot: group <g> eye <e> is <a patch or undecided> at <k> times the amounts: <n> of <t> cells respond[, <m> before], the rest move together by <z> sigmas` | after a round of the group's pushes (at <k> times the amounts its probes found, 1 in the first round), eye <e>'s verdict on the group is not an answer yet: it shows a patch (some cells move, the rest do not) or cannot tell; <n> of its <t> cells that can show a displacement respond, and <m> did after the round before; <z> is how many sigmas the cells that did not respond move together (about a standard normal when none moves) |
| `boot: group <g> leaves an eye undecided or showing a patch that may grow; pushed again at <k> times its amounts` | some eye is open about the group (undecided, or a patch) and either it is the first round or the cells responding grew by more than the false alarms of the cells that did not respond could make (at Z): a push at the smallest amount an eye saw shows only the cells that move most of an eye whose view moves by different amounts in different cells, so the group is pushed again at twice the amounts of its last round; a patch that is one does not grow, and ends the rounds |

The kinematics (an arm's joints and the lens of the eye it carries, fitted from its keyframes):

| line | meaning |
|---|---|
| `kinematics: arm <a> fitted from <k> sightings of <f> keyframes, <u> fit, median <m> px, noise <s> px; focal <fx> x <fy> px; errors, px: a sighting's own <e>, a point's in every keyframe <p> (half as alike at <h> px apart), a keyframe's added for its points <q>; the clip took <c>` | the fit of arm <a>, of <u> of the <k> sightings; the errors its residuals show, which its covariance rests on: a sighting's own, the part of it a point has in every keyframe (and the distance at which two points have half as much of it alike), what a keyframe adds for its points, and the share of the covariance's spread the clip to positive semi-definiteness took away (0 when none) |
| `kinematics: arm <a> not fitted (stage <n>): <why>` | the sightings do not determine the arm; the last fit that held is kept |
| `kinematics: eye <e> fixed in the world: <placed or not placed> from <k> answers of <q> points, <u> fit, noise <s> px[, its cost quadratic over <l> of <z> sigmas]; focal <fx> +-<sfx> x <fy> +-<sfy> px, centre <cx> +-<scx>, <cy> +-<scy>, <distortion>; in the world at <x> +-<sx>, <y> +-<sy>, <z> +-<sz>, turned <rx> +-<srx>, <ry> +-<sry>, <rz> +-<srz>[: <why>]` | the lens and place of the eye <e>, which stands still in the world (the first arm's frame, in its unit), measured from the first arm's tracked points and the <k> answers of the eye to where they are that return to their query, of which <u> fit; <l> says how far the fit's cost is the quadratic its covariance rests on: the smallest, over the principal directions of the covariance and both ways along each, of the displacement in sigmas that the cost's rise corresponds to for a displacement of <z> (Z) of them (<z> where the cost is quadratic; below <z> less one sigma the covariance does not stand and the eye is not placed); with the standard deviations of its terms (the turn is the camera's rotation as a rotation vector, its sigmas about its own axes), `no distortion` or the two distortion terms when significant as a pair; when not placed, <why> says what the points leave out, and every answer about the eye (pose, projection, lines of sight) is unknown |
| `kinematics: eye <e> is fixed in the world, and not placed: <why>` | the points give the eye no start (all of them on the first arm's table, too few off it, too few answers), or its picture has no size, or the first arm's fit has no covariance of its points' depths; said once for each change of what it rests on |

The hand (measured at boot):

| line | meaning |
|---|---|
| `hand: closer group <g> on arm <a> is watched in eye <e>, its own` | the group that closes a hand, and the eye on its arm that watches it |
| `hand: closer group <g> channel <c> has no visible step measured; not swept` | the channel's response is unmeasured, or its reading's noise is (a visible step is one the reading tells from its noise as well), so it is not swept |
| `hand: closer group <g> of arm <a> is not swept: no eye on its arm watches it, so no hand is made of it` | a closer of the body that no arm-carried eye sees move |
| `hand: closer group <g> channel <c> in eye <e>: <account>` | what the channel's sweep made of its new ends, said once for the ends, from the pictures alone (the hand asks no instrument). The account is one of: `its two ends were not both seen still`; `nothing in this eye moves between its ends`; `half of this eye's picture or more changes between its ends, so what moved cannot be told from what did not`; `the arm has not moved the eye against its surroundings at either end's readings (seen from <a> poses at the low reading, <b> at the high one; two are needed): <n> pixels changed, beyond <x> levels of a spread of <y>`; `the <n> pixels changed, ..., do not fall in two groups by how much they vary over the arm's poses (...)`; `the <n> pixels changed, ...; of them <h> went to the low end, <t> to the high end, <u> to neither; the parts they made that are attached to the picture's border and larger than the doubt of <d> pixels are <p> at the low end and <q> at the high end`; or `<k> lobes, ` then `closed at the high reading` (or `at the low reading`, or `closing direction not significant: their distances changed by <v> +- <s> pixels between the ends`, or `closing direction not known: there is nothing to compare their distances with`) and `(<n> pixels changed, beyond <x> levels of a spread of <y>; <h> given to the low end, <t> to the high end, <u> to neither, of doubt <d> pixels, after <r> rounds; its parts: <p> at the low end, <q> at the high end)` |
| `hand: closer group <g> channel <c> in eye <e>: lobe <i>: <n> pixels at the low reading, tip (<u>, <v>)[ from the border]; <m> at the high reading, tip (...)` | each lobe of a measured channel, its pixels and tip (or `no tip`) at each end |
| `hand: the hand of closer group <g> is dropped: its ends were measured again and gave no lobes` | new ends of a channel gave no lobes, so the hand made of its older ends no longer stands |
| `hand: no hand was found, so nothing is pressed; below, what became of each closer` | no closer gave a hand; the lines of `hand: measured` say what became of each |
| `hand <n>: lobe <l> at <opening> is not pressed: its tip is not seen in the hand's eye at this opening` | the lobe has no tip pixel at that opening |
| `hand: the hand of closer group <g> is not pressed: it is gone, its group is no longer a closer of its arm` | the body re-read its roles before the hand's turn came; the hand is found again at each beat by its closer group, never by an index kept from an earlier beat |
| `hand <n>: the hand this press of lobe <l> measures is gone: its closer group, group <g>, is no longer a closer of arm <a> as the estimators have it between two beats of the press; the press ends here[, the arm let go and taken back to where the descent began]` | the estimators read the roles again between two held beats (a recompute of the heavier estimates), Find_Pairs found no hand of the closer and dropped it, under a press in progress (A29); the press ends, the arm is let go and taken back to where the descent began when it had moved, and the hand's other lobes are not pressed |
| `hand <n>: lobe <l> is not pressed: the hand is gone (its closer group is no longer a closer of its arm)` and `hand <n>: lobe <l> is pressed no more: the hand is gone` | the same, found gone where the decider reads the lobe's tip, before or after a press |
| `hand <n>: the arm did not reach the aim of a press of lobe <l> at <opening>, turning the hand <t> rad (<group>: delivered <f> of <length>, blocked; ): no press is made from where it stopped, and the arm is taken back to where the aim began` | the aim, a turn of the hand about its eye, ended Blocked or Short: something of the arm's own stopped it (A27's hand 2: the third joint at +0.07 for the -0.05 asked); no descent is made from where it stopped and the tilt is taken for one the arm cannot make from there (half of it is tried) |
| `hand <n>: no press of lobe <l> was found at the rest after the let-go: the arm did not rest in <k> beats, the stream's length; the arm is taken back to where the descent began` | the hand was let go (the arm held at the readings the block left) and the decider waited for the watcher to find the press, at the arm's own rest; none was found in as many beats as the stream has (a creep that does not decay, a hold that took no effect), so the press is not made and the tilts of that side end; a press the watcher found and could not use (the closer at no measured opening: `a press with the closer at neither measured opening is not used`) is a press made |
| `hand <n>: the aim of a press of lobe <l> at <opening> stopped on an end the arm showed by it: the same aim is planned again past it` | the arm did not reach the aim and what stopped it was noted as a new end (`body:` below): the aim is planned once more with the end known, turned about the way down past it, or not planned at all (the next line) and no arm move is spent |
| `hand <n>: no aim of a press of lobe <l> at <opening> is planned, the hand turned about the way down by any eighth of a turn: <why>` | the arm cannot be taken to the aim, or from it down to the contact the presses so far predict, by the least rotation or any turn of the hand about the way down (an end a joint showed is in the way, `body:` below); nothing was moved, the tilt is taken for one the arm cannot make from there and half of it is tried |
| `hand <n>: the aim of a press of lobe <l> at the least rotation is not planned (<why>); the hand turned about the way down by <y> rad it is` | the aim was refused (an end in the way of the arm's joints), and another pose that points the line of sight down was not: the turn is taken |
| `hand <n>: lobe <l> at <opening> pressed once, straight, and not tilted: no other line of sight of the hand is known to tilt away from` | a lone lobe: no tilt to try |
| `hand <n>: lobe <l> at <opening> tilted one way (the other):<k> presses, then <why>` | how far the tilts went on that side: to a right angle (the first tilt is the hand's own angle and no less than the least that tells the tip from a stop), until a press could not be made, until the lobe's tip was confirmed, or until a press stopped short of the table at a tilt and halving it came under the least tilt that tells the tip from a stop that does not move with the tilt |
| `hand <n>: lobe <l> at <opening> is not pressed: its tip is confirmed already` | presses made for other lobes had already landed on this lobe's tip from two poses |
| `hand <n>: a press at the <opening> opening, at beat <b>, <k> kept; a tip rests on it` (or `no tip rests on it (it stopped short of the table, or no tip is fixed)`)`; the tips at this opening: lobe <l> <confirmed|provisional> <d> +- <s> along its sight, on <j> presses, its tip region <w> wide across it, left <a> and <b> along the axes the tilts of the presses told most and least (tested across the sight|not tested across the sight), ...` (or `none`) | a fingertip press on a surface was recorded, at the rest after the push let go, whether a tip's fit keeps it (the tip stopped it) or left it out (the arm stopped on something else), and each lobe's tip at this opening: how far along its line of sight from the eye, in the arm's unit, and whether a second press from another pose landed on it (confirmed) or only one fixes it (provisional); the tip is the finger as it stood under the press (loaded: `Tip_Beat` is the press it rests on), and the width of its tip region across the line is added to the tip's covariance |
| `hand <n>: under the press at the <opening> opening, at beat <b>, the fingers stand from where the reading puts them: lobe <l> slid <s> +- <e> pixels, <p> % of the <d> it closes (cost <c>, where it stood <c0>, typical <ct>)` (or `not found at <s> pixels (cost ...)`, or `has no patch to look for`) | how far each lobe's finger stood from where the closer's reading puts it when a press was kept, found in the picture of that beat as the shift along the way the finger closes in the picture (positive inward) at which the finger's tip pixels, as the sweep saw them free, are likeliest in the picture; its sigma is how far the two halves of the tip region put it from each other, and the three costs say how clear the minimum is (mean negative log-likelihood a pixel) |
| `hand <n>: its arm was fitted again; the <k> presses kept take their poses from the new fit` | the arm's table moved, so every press kept took its pose again from the arm's readings it kept |
| `hand <n>: a press with the closer at neither measured opening is not used` | the hand was neither at its measured open nor at its shut opening |
| `hand <n>: the closer of lobe <l> does not come to its <opening> opening: no press is made at it` | after the aim the closer's readings were not those of the opening, and asked for once more they still were not: the finger is held (A17: against the table at the sweep's pose), so no press is made at that opening |
| `hand: closer group <g> channel <c>, asked back to <t>, reads <r>: it is held; the hand is raised by <y> (<q> before) of the <h> the eye stands above the table` | a closer asked back to a reading it has been at, in its sweep, stayed short of it: something holds it (a finger resting on the table, A17's), and the hand is raised along the way up, in steps that double from the least move of its tool: y this one, q the raises so far, h the eye's height above the table, which they do not pass. Nothing is said when the closer arrives |
| `hand: closer group <g>: its eye <e> on arm <a> has seen its readings from <n> poses of the rest of the body, and 2 are needed to tell the robot from its surroundings; the arm raises the eye by <y>, the least that its readings show` (or `, twice the raise before`) | the eye had seen the closer's readings from fewer poses than a deviation needs (a body reloaded from a file, or a hand measured again, has the boot's none), so before the closer's sweep, or after it when its lobes were not placed for want of poses, the arm raises the eye along the way up: y this raise, the least whose readings the body's one test of motion sees, then twice the one before. Nothing is said when the eye has the poses |
| `hand: closer group <g>: its eye <e> on arm <a> cannot be raised: <why>` | no raise was made: where up is, in the arm's frame, is not measured, the plan for it could not be made (the motion's own reason), or no raise within the arm's reach moves its readings by what an eye sees |
| `hand: closer group <g>: after <r> raises its eye has seen its readings from <n> poses, of the <w> asked` (and `; it cannot be raised further, or its picture did not rest to keep a frame`) | what the raises gave: the poses now, those asked, and when short, that the arm made as many raises as it could (the line before says why the next was not), or that the eye's picture did not come to rest within as many beats as a view takes to form |
| `hand: the tip has gone past the contact its presses predict and the band of its sigma about it, and met nothing (it is predicted <g> above it now): the steps double again` | a descent went through the band Z sigma either side of the contact its presses predict without being blocked: the prediction was a bound (a press that stopped in the air fixed it), and the steps double from the band's own, as with nothing predicting it |
| `hand <n>: pressing lobe <l> at <opening>, its line of sight tilted <t> rad from straight down, aimed by turning the hand <a> rad, <h> +- <s> above the surface the presses so far fixed` (or `nothing yet predicting the surface below its tip: doubling from <x> until blocked`), then `; the eye <e> above the table` (or `the eye's height above the table unknown`) | a press begins, in the arm's own frame and unit: how far the lobe's line of sight is tilted from the way down (0 for the straight press, the tilt of the others), how far the hand turned to point it down, the tip's predicted height above the table, or none, and the eye's height above the table its arm's own eye saw, which no step takes it below |
| `hand <n>: press of lobe <l> at <opening>: <k> pushes, <f> fast and <b> within Z sigma of the contact its presses predict, <d> doubling from <x> with nothing predicting it (before a prediction, or past the band of one), <c> cut to the eye's room above the table; blocked, the last push by <y> after lowering <t>` (or `stalled, the arm followed the last push by <y> and the hand did not go down with it, after lowering <t>`, or `then the eye has no room left above the table, or the steps that cover it were made, and nothing was met, after lowering <t>`) | how the press went down, in the arm's unit; "stalled" is a press like a blocked one (the hand lies on what it met, and the arm went on following), the last form is a descent that reached the eye's room, or made the steps a doubling schedule needs to reach it (so that steps that were reached and lowered nothing, without a stall being told, end on the schedule), without the arm being blocked, which makes no press; the hand goes back to where the descent began |
| `hand <n>: the arm followed the push that began at beat <b> and the hand did not go down with it:` then, for each measure that stopped it, ` its readings stopped short of the push's target by <p> % of its length, where the <k> pushes of this descent before it stopped short by at most <q> %;` and/or ` asked to take <the tool's origin, or the tip of lobe l> down <a>, it went down <w>, short by <s> % of the ask, where the pushes before it fell short by at most <r> %;` then ` it has stopped lowering the hand` | the arm's push, read from the stream (Driver.Robot.Hand.Lowering), stopped short of what it asked by a larger share than any push of the descent before it by Z times, in the readings (the whole vector to the target, when that is a motion the one test of motion sees: the hand lying on what it met is pushed off the ask, and the part along the ask stays most of the way) or at a point of the hand (the tool's origin, or a tip the hand has measured, asked to go down by what the tool's noise tells and short of it by as much): the hand is on what it met, and the arm is sliding it or turning it. The same push is a block for the press found in the stream; the descent that made it ends |
| `hand <n>: up, the arm's pose or the eye's mount is unmeasured; no press` | a press cannot be aimed yet: the arm is not fitted, or its eye saw no table |
| `hand <n>: cannot aim a press: <why>` / `hand <n>: cannot press lower: <why>` | the motion planner refused the press |
| `hand: measured` | what was measured about every hand, one line each; for each closer an eye watches that gave no hand: `closer group <g> on arm <a> in eye <e>, its own: no hand; channel <c>: <account>;`, where the account says where the channel's sweep stands: its ends not both seen still, nothing yet asked of the instrument, its answer not come, the instrument could not (or can never) answer, nothing in the eye moves between its ends, half of the eye's picture or more changes between them (what moved cannot be told from what did not), or the lobes found (as above) |

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
