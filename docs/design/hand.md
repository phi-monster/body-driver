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
| `Hand.Frames` | points and lines of sight taken into the tool frame, and a tip into the world, with their uncertainty |
| `Hand.Presses` | the presses an arm made, found in the stream |
| `Hand.Touch` | tips and the surface pressed on, fitted together |
| `Hand.Tips` | a hand's presses, each given to the lobe that touched, and the tips they measure |
| `Hand.Aims` | the turn about the eye that aims a press |
| `Hand.Pressing` | what one press asks of its arm, in the arm's own frame: the aim, the lowering, the way back, the tip's gap above the table |

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

**Lesson: a refusal that can never pass ends the sweep.** A refused pair of
ends is not asked again, but new ends are. While the boot moves the arm, the
closer's views begin again at new beats, so the ends are new on nearly every
still beat. Path A's rig, run without an instrument address, asked about 6000
times. The service now says when its failure is lasting
(`Driver.Services.Reply.Lasting`: no address was given). Then every channel
of that sweep is `Unanswerable` with the reason, nothing is asked again,
`Measure` does not sweep the channels left and says why, and `Describe`
states it. A reply that does not fit, or a failed request, may pass, and new
ends are asked again. There is no retry count.

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

**Lesson: a hand is measured in its arm's own frame.** The pose of a press is
`Tool_In_Arm`, the table is `Table_In_Arm`, down is `Up_In_Arm`, and every move
is planned with `Plan_Reach_In_Arm`: the frame of the arm's eye at its
reference readings, in the arm's own unit. They need the arm's fit and
nothing of where the arm stands in the world. Three things went wrong in the
world's frame. An arm the body has not placed (A11's second arm failed the F
test of its link) has no pose there, so it pressed nothing. An arm placed a
little wrong turned every press by the placement's error, since down was the
first arm's up. And a tip is in the arm's unit, which a placed arm's pose does
not share (its place is in world lengths): fitted in the world's frame, on a
rig whose arm has a unit of 1.4 world lengths, the widest tip sigma was 0.034
arm units on tips 0.13 long, against 0.00043 in the arm's frame, and `Tip` put
a tip 0.078 from where the placement puts it. Only `Tip`, `Tip_Now` and
`Grip_Centre` go to the world, through the placement and the arm's unit
(`Frames.Into_World`).

**Lesson: the arm's frame moves while the hand presses.** A body that did not
reload its kinematics goes on taking a keyframe at every pose the arm rests in
that no keyframe was taken at, and fits the arm again: the arm's unit (the root
mean square of its eye's positions over its keyframes) and with it every
length in its frame move with each fit. A press kept as a pose of the frame it
was made in would disagree with the table of the frame the arm has later. So a
press keeps the arm's readings its pose came from (`Presses.Event.Arm`), and
when the table the arm gives is not the one the presses were fitted with, all
take their poses again from their readings (`Tips.Set_Frame`); a unit grown by
a tenth moved the tips 12 sigma when it did not. A press never carries a pose
from one beat to the next either: its way back is to the readings its descent
began from.

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
the t distribution's 1.18 for 7 degrees of freedom). The table the arm's own
eye saw fixes the offset directly: it is the surface's prior
(`Tips.Set_Surface`), so a lobe's tip is fixed by two presses, not by the six
an unknown table takes, and the first presses that follow predict their
contact.

**Lesson: the fit may not wait for the hand's other tips.** The start of the
fit asked for as many presses as the hand has tips and surfaces, though a tip
nobody pressed is pinned and costs no press. A hand of four tips (two lobes at
two openings) could be fitted from its fifth press on, whatever the surface:
the first lobe's presses all went blind, and `Latest_Agrees` (no fit, so no
agreement) ended its tilts after one press a side.

**Lesson: test the surface with a slope.** A table at right angles to the
presses lets a wrong update of the surface's tilt pass unnoticed, since the
start already holds the right normal. A table sloped 0.15 rad found a sign
error in the tilt update at once.

## How the decider aims a press

A press turns the hand about the eye, so the eye keeps its view, until the
lobe's line of sight, tilted, points into the table its arm's own eye saw
(`Up_In_Arm`), then lowers the hand with doubling steps from the smallest move
that tells from the arm's own noise until the arm is blocked, lets go, and
lifts back. Once the presses so far fix the lobe's tip, the steps stop
doubling Z sigma above the contact they predict and go on by that sigma. The
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
