# The hand

This note explains how `Driver.Robot.Hand` measures a robot's hands and why it
is built that way. It is for people who change the hand; the measured hand
reaches the layers above through the specification `driver-robot-hand.ads`.

A hand is a closer group: a group whose push moves a patch inside an eye that
rides on an arm (path A's role `Closer`). Pushing one of its channels moves
one or more *lobes*, the pieces of the hand that move. A two-finger gripper
has two lobes on one channel, a five-finger hand several channels with a lobe
each, a suction cup one lobe. Nothing in the hand knows which of these it is
looking at.

## What is measured, and from what

Everything comes from the recorded stream: readings, commands, images and the
instrument's replies, with path A's measured geometry. The estimators
(`Hands.Observe`) run on every beat whoever drives the robot, so a replay of
any run measures the same hands. The decider (`Hands.Measure`, in
`driver-robot-hand-measure.adb`) only chooses commands: it sweeps and presses,
and the estimators find what it did in the stream.

| Package | Responsibility |
|---|---|
| `Hand.Views` | per channel, the still views at the two ends of its travel, with the rest of the body unchanged between them |
| `Hand.Sweep` | per closer and eye, the correspondence request between the ends and the lobes from the reply, as plain data |
| `Hand.Lobes` | the lobes from the matcher's correspondences, their masks and tips in the image, which end is closed |
| `Hand.Frames` | points and lines of sight taken into the tool frame with their uncertainty |
| `Hand.Presses` | the presses an arm made, found in the stream |
| `Hand.Touch` | tips and the surface pressed on, fitted together |
| `Hand.Tips` | a hand's presses, each given to the lobe that touched, and the tips they measure |
| `Hand.Aims` | the turn about the eye that aims a press |

## Lobes from two still views

A channel is seen at the two ends of its travel while everything else holds
still: the readings of every other group must not change between the two
views, or the background moves and the comparison means nothing. Each view
gathers frames while the body is still; two frames at least, so every pixel
has its own measured variance (with the floor 1/12 of an 8-bit level).

The instrument's matcher gives, for every pixel of the box the change covers,
where it went in the other view and where that point matches back to, both
ways round. Pixels whose intensity did not change measure the matcher's own
noise (robust spread, degrees of freedom from the median absolute deviation's
efficiency). A pixel moves when its displacement passes the two-dimensional
gate and its round trip does not.

**Lesson: a lobe needs a seed that passes the family gate.** With about 19 000
pixels tested per view at the single rate, dozens of still pixels pass by
chance, and some land on one another across the views and make a lobe. A
component counts only when one of its pixels passes `Vector_Gate (2, Dof,
Tests => every correspondence tested)`. Measured: 100 still synthetic scenes,
10 414 still pixels pass the single test, 0 scenes give a lobe; without the
seed rule 18 of 100 do.

**Lesson: the tip is the farthest seed.** The tip is the lobe pixel farthest,
along the lobe, from where it is attached (the image border or the robot's
still pixels). A still pixel next to a finger that passed the single test by
chance becomes the farthest pixel by one row; seeds pass the family test, so
taking the farthest seed removes it.

## Closers whose reading echoes the command

On the x5 the gripper's reading is the last command. Commanded past its travel
the reading goes on while the fingers stand still. A view at a reading beyond
an end therefore replaces that end only when the eye sees something the end's
view does not. The sweep pushes outwards with doubling steps, from the
smallest step path A says the eyes see, for as long as each step shows the
eye something new; a channel whose push the eye never sees change is reported
as moving nothing. `Closer_Reading` is then where the hand really is open or
closed, not where a command went.

## The tip in the tool frame

A tip's pixel gives a line of sight. The hand keeps it in the tool frame of
the eye's arm (`Eye_In_Tool` composed with `Eye_Ray`): composing the world
ray with the inverse tool pose would count the arm's kinematics twice, since
the arm carries both the eye and the tool. Where along that line the tip is
comes only from touching.

## Presses

A press is a run of beats at which the arm was blocked, ended by the first
still beat after the push let go. The tool's pose at that beat is where the
hand rests on what it pressed. Reading it while the push still drives the hand
in would be wrong: in the legacy driver's runs (V1B21) the finger sank 6.6 mm
into the table while pushed and came back to 2.5 mm once the command stopped.

Each press goes to the lobe that leads into the surface: before anything is
fitted, the lobe whose line of sight lies closest to the way the tool was
pressing; once the surface is fitted, the lobe whose fitted tip is foremost
along its normal. All presses are refitted and given out again until none
moves, so a press that came in sideways ends up with the lobe that touched.

## Tips and the surface, fitted together

A press says the tip, fixed in the tool frame, lies on the surface:
n . (R x + t) = d. The tip lies on its line of sight (one unknown) or anywhere
(three, the check); a surface is either measured before (a prior) or unknown
(three unknowns: offset and two tilts). Everything is solved together by
weighted least squares, so the surface's uncertainty is in every tip's. The
presses' noise is the one predicted from the arm's pose, raised to the scatter
the agreeing presses show when that is larger; a press whose deleted residual
is significant is dropped (stopped above the surface: something else touched;
sunk below: a yielding contact).

**Lesson: on an unknown surface the tilts must differ in size.** With the tool
turned by a tilt about the eye, the tip's distance s along its line enters
the equations only as s (1 - cos tilt): presses tilted by the same amount in
different directions leave s and the surface's offset confounded, exactly as
presses from one orientation do. The fit reports that as an unknown it cannot
fix. With tilts of 0, 0.3 and 0.6 rad at six places per lobe, contacts
scattering 0.5 mm and the surface unknown, a tip's distance came out with a
sigma of 2.4 mm, its errors calibrated (rms of error over sigma 1.21 against
the t distribution's 1.18 for 7 degrees of freedom). A surface measured by the
eyes would fix the offset directly; the hand, in layer 2, has none.

**Lesson: test the surface with a slope.** A table at right angles to the
presses lets a wrong update of the surface's tilt pass unnoticed, since the
start already holds the right normal. A table sloped 0.15 rad found a sign
error in the tilt update at once.

## How the decider aims a press

A press turns the hand about the eye, so the eye keeps its view, until the
lobe's line of sight, tilted, points along gravity (path A's `Up`), then
lowers the hand with doubling steps from the smallest move that tells from
the arm's own noise until the arm is blocked, lets go, and lifts back. The
first press is straight along the line of sight; then the line is tilted away
from the other lobes, on either side, by doubling multiples of the angle to
the nearest other lobe's line (a lone lobe: the angle it travels through
between the openings), until a press stops agreeing with the others or cannot
be reached. Tilting away from the other lobes keeps them behind the aimed tip;
in the legacy driver's runs (V1B79), tilting towards a finger's own body made
the body touch first.

## Open

- A closer group is one hand. A five-finger hand that reports all its fingers
  as one group yields one hand with a lobe per channel; the graspers that are
  subsets of its channels are not separated yet.
- A gripper that no eye on its arm sees is classified by path A as a part, not
  a closer, and gets no hand.
- The descent doubles its step towards a surface it does not know; the last
  step can push up to as far beyond the contact as the hand travelled before
  it, which is why the press reads the pose only after letting go.
