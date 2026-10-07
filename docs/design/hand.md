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

Everything comes from the recorded stream: readings, commands and images, with
path A's measured geometry; the hand asks no instrument. The estimators
(`Hands.Observe`) run on every beat whoever drives the robot, so a replay of
any run measures the same hands. The decider (`Hands.Measure`, in
`driver-robot-hand-measure.adb`) only chooses commands: it sweeps and presses,
and the estimators find what it did in the stream.

| Package | Responsibility |
|---|---|
| `Hand.Views` | per channel, the still views at the two ends of its travel, with the rest of the body unchanged between them |
| `Hand.Selfsight` | what an eye that rides on an arm shows of the robot itself, from the arm's own motion: the eye's still frames at each setting of the closer, one for each pose of the rest of the body (a plain-data memory that moves into the model behind `Self_Mask`) |
| `Hand.Sweep` | per closer and own eye, the change between the ends of each channel and the lobes found from it and the arm's poses, as plain data |
| `Hand.Lobes` | the lobes from that change, their masks and tips in the image, which end is closed (and, kept for the shape's moves, the matcher-based finding) |
| `Hand.Frames` | points and lines of sight taken into the tool frame, and a tip into the world, with their uncertainty |
| `Hand.Presses` | the presses an arm made, found in the stream |
| `Hand.Touch` | tips and the surface pressed on, fitted together |
| `Hand.Tips` | a hand's presses, each given to the lobe that touched, and the tips they measure |
| `Hand.Aims` | the turn about the eye that aims a press |
| `Hand.Pressing` | what one press asks of its arm, in the arm's own frame: the aim, the lowering, the way back, the tip's gap above the table |

## Lobes from the change and the arm's poses

A channel is seen at the two ends of its travel while everything else holds
still: the readings of every other group must not change between the two
views, or the background moves and the comparison means nothing. Each view
gathers frames while the body is still; two frames at least, since the first
alone cannot show that the body held still.

**What changed.** Two views of one scene differ a little everywhere: A14's
ends differed by one to three levels at most pixels (the median absolute
difference was 2.0 levels in the wrist eye and 1.0 in the head's: the
renderer's lighting moves with the fingers) while the frames within a view
agreed to the quantization, and the test of each pixel's difference against
the frames' own variance (Welch) called 262 474 of 307 200 pixels changed in
the wrist eye (85 %) and 258 310 in the head's. `Driver.Pixels.Compare` takes
the null from the difference itself: its median is the typical difference (a
lighting shift of the whole view is no change), its spread is found from the
nearer half of the pixels and then from the pixels within the gate of that
spread until no more drop out, never below the quantization of two means, and
every pixel is one test of a family of as many as the view has pixels. It
assumes that most of the view does not change, and says so when half of it or
more does (no pixel marked, and the sweep's channel is `Everything_Moves`). On
the A14 ends, one frame each, it marks 89 148 pixels (29.0 %) in the wrist
eye, beyond 16.3 levels of a spread of 2.8, the solid union of both fingers'
two positions; 4 705 (1.5 %) in the head's. A trusted answer is one consistent
with the assumption: a view most of which changed in one way is not told from
a view whose smaller part did.

**Lesson: a matcher cannot follow a smooth finger.** A14's fingers were two
black wedges, each a quarter of the wrist eye's picture, moving 150 pixels.
The instrument's matcher, asked where every pixel of the box went and where
that point matches back to, answered for no pixel of the changed set by more
than 64 pixels, with round trips that failed inside the wedges; 7 583 pixels
forward and 10 724 backward passed as moving, more than half of them outside the changed
set (the matcher's warp carried the fingers' motion into the plain wood beside
them), the rest in patches at the fan's base, the pens and the keys, not along
the fingers' edges. Its noise, taken from the pixels that had not changed, was
measured on the quietest 15 % of the picture (0.067 px). Twenty lobes were
found in the wrist eye of the replay (eleven in the live run) and not one was a finger; the head's eye said the hand
was closed at the high reading, and by eye it was closed at the low one.

**Which end a changed pixel belongs to.** The change alone does not say: a
pixel that changed shows a part at one end and the world at the other, and the
picture is the same with the ends exchanged. The cue is what the eye shows of
the robot itself as its arm moves (`Hand.Selfsight`): the parts that go with
the eye stay where they are in its picture and the world slides across it. For
each setting of the closer's readings the memory keeps the eye's still frames,
one for each pose of the rest of the body it was seen still in; a setting the
closer leaves after one pose is forgotten, and no more than `Capacity` of two
poses or more are kept. In A14 the closer sat at 1.000 from beat 1193 to the
sweep's start at 6283: 269 poses, and the mean over them shows both fingers as
crisp black wedges at the borders and the world ghosted.

The deviation over the poses is a seed and not a mask. A14's left finger
varied by 11.4 levels over them (median; 8.1 to 17.0), the right one, which
reflects more, by 16.1 (11.5 to 23.8), the table by 29.5 (13.1 at the tenth
percentile). Among the changed pixels, 33 948 showed a finger at the starting
end and 57 609 did not, and a cut at 20 levels (Otsu's cut inside the set
chooses 22) put 89.5 % of the fingers' below it and 15.1 % of the others:
86.6 % right, the errors in clusters, a wedge of the flat dark world beside the
right finger's other place and the right finger's reflecting patches.

`Lobes.From_Change` tells the changed pixels in a mixture of two kinds: the
robot at the starting end and the world at the other, or the robot at the
other end and the world at the starting end. A pixel of either kind has a
brightness at each end, and the kinds are told by how bright the robot and the
world are where they changed, each a histogram (every one begins with one
pixel a bin), found together with every pixel's share in each kind by
expectation and maximisation (`Tell_Ends`). A pixel is given to a kind only
when the other kind's share of it is below what one test alarms at (Z); the
rest are left unassigned, never guessed, and the log counts them.

**Lesson: where the rounds begin decides where they end.** Histograms this
free keep any labelling they are begun from, if each group of pixels with a
brightness of its own supports it. The first form began from the seeds, the
pixels whose deviation over the poses is low, and the seeds are wrong where
the world is flat. A14's final ends (89 294 changed pixels, the seeds 86.6 %
right) came out 91.8 % right. A15's final ends (86 392 pixels; the closed
fingers touch and cover the grey table beside them) had seeds 69 % right and
came out with 47 % of the pixels given the end that shows them darker, three
lobes and a tip where no finger was: the table's grey held as the robot's and
the finger's black as the world's, each group at the label its seeds gave it
(the likelihood of that state was higher than of the right one, by 2 600 in
the log, so the likelihood cannot choose). Begun instead from which end shows
the pixel darker where the two ends differ most (the half of the pixels above
their median difference; elsewhere, and where the levels are equal, from the
seed) the same pixels came out 91 % (`hand.lobes.order`: a scene of six
groups, 58 % of its seeds right, 3 136 of 10 600 pixels given the wrong end
from the seeds and none from the order). Over A15's stages of 7 657 changed
pixels and more the agreement went 18 to 85 %, 86 to 86, 83 to 83, 78 to 79,
94 to 94, 47 to 91 and 94 to 95 %. The seeds say which kind is which when the
rounds are over: more of them in the kind with the robot at the anchored end
than not, or the kinds are exchanged. Not the order for every pixel: of two
levels that differ little the order is noise, which histograms begun from it
sort into a robot and a world that are not there, and the neighbours cannot
decide them (`hand.lobes.blank`, pixels dark at both ends, begun from the
order: 332 of 2 522 right; begun from their seeds 2 267). The seeds are wrong
where the world is flat, and a flat patch is wrong as a whole and explains
itself: when the rounds have settled every connected part of one label is
tried the other way round against the histograms of all the rest, and turned
when that is the likelier by more than Z squared over two in the log of the
likelihood. Last the eight neighbours are heard (how many of them are of each
kind, another histogram).

The rounds end by the convention for an iterative estimate: it has stopped
when one more round moves it by less than `Unchanged_Fraction` of its own
size, and what is estimated differs between the two phases. While only the
brightness speaks the estimate is the labels, and the phase ends when fewer
than that fraction of them change in a round (run on, the brightness alone
drifts to one kind for everything); the parts are then turned, and the phase
begins again unless none was turned or a pass turned no fewer than the pass
before it (two sets of parts that each explain the other better turned can be
turned back and forth for ever: A14's stage of 62 570 changed pixels turned
four to nine parts at every pass for 1 120 rounds, fourteen seconds, and left
38 % of its pixels to neither; it now ends in 33 rounds, 0.23 s, with 6 %;
`hand.lobes.cycle`). Once the neighbours are heard the estimate is the doubt,
the expected error, which goes on sharpening for many rounds after the labels
have stopped, a pixel at a round from the clear ones around it: on A14's final
ends 12 962 pixels were left to neither when the rounds ended with the labels
(nine rounds), 3 840 when they end with the doubt (23 rounds, 0.24 s). The
doubt can wait on a plateau for a round before it falls, so it has stopped
when two rounds in a row moved it by less than the fraction (pixels dark at
both ends: 331 of 2 421 right and 2 090 to neither after 6 rounds when one
round sufficed, 2 088 and 333 after 24 when two had to, `hand.lobes.blank`),
or when it came back within the fraction of its value two rounds before,
twice in a row (two pixels whose neighbours each tell them to take the
other's label swap it every round: the 112 rounds a label can cross the
picture in, against 32 with the rule; `hand.lobes.cycle`), or when it and the
doubt before it are what they were in an earlier round. On A15's three last
stages: 29, 26 and 40 rounds, 0.21 to 0.36 s on the Mac.

**Lesson: the parts of a mixture's sets are not all lobes.** Each end's
pixels are cleaned of what is one pixel across and split in their
eight-connected parts. The first form took every part attached to the image's
border for a lobe, and A14's low end held, beside its two fingers of 29 219 and
19 664 pixels, parts of 899, 400, 334, 149 ... pixels (islands of keyboard
keys, dark stripes on dark stripes, that the mixture cannot tell from the
finger): six lobes, and the 400-pixel fragment at the bottom left, nearest to
the open finger's wedge, took its place, so the finger it belonged to was left
without a partner and dropped. Three rules decide which parts count. (1) A
part must be attached, to the border or to the robot's pixels that did not
change. (2) It must hold more pixels than the mixture's doubt, the number of
pixels it expects to have given to the wrong end (the sum over the pixels of
the smaller of the two shares); a part no larger could be made of nothing
else. A14's doubt was 381, which the mixture's confidence made too small to
matter, since it is confidently wrong at the keys. (3) It must not be a
fragment by its size: the log sizes of an end's parts are parted in two groups,
and the parting stands when the groups' mean log sizes differ by more than Z
standard errors, from the spread within the groups, and the biggest part is
more than N times the size parted at, N the number of parts; what is left out
is then smaller than the share 1 / N the biggest would have if all the parts
were of its size. A finger half the size of the others is kept (a ratio of 2
against N of 3), and so are fingers that touch at one end and show as one part
twice as large, since each end is judged by its own parts and the end with too
few parts takes the other's threshold. The lobes are then the parts of the end
that has the more of them, and each part of the other goes to the lobe whose
part is nearest by their centres (pixel by pixel when fewer parts remain than
lobes: fingers that touched, each taking what is nearest).

**Lesson: the tip is the farthest pixel along the lobe's reach.** The tip
was the lobe pixel farthest by the paths through the lobe from where it is
attached (the image border or the robot's still pixels). A14's open wedge had
a strip of keyboard pixels hanging on it along the bottom of the picture: 143
pixels from the border by the paths, where the wedge's own apex is 118 from
it, so the strip's end was the tip (160.8 against 127.9), 200 pixels from where
a press must land. The tip is now the pixel farthest along the lobe's reach,
the direction from the centre of where it is attached to the centre of its
pixels: the strip is 110 along it and the apex 157. (The matcher-based
finding, `Lobes.Find`, took the farthest seed that passed the family gate; it
is kept for the moves the shape is fitted from, which the hand does not ask
for yet.)

The hand asks no instrument: its lobes, tips and closed end come from the
stream alone, so the same estimators run on a recording, in a test and live,
and a recording of a run needs no replies to measure its hands.

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
that no keyframe was taken at, and fits the arm again, and each fit moves the
poses the arm gives and the table its eye saw. Once the arm's unit is its
first fit's, they move a little; before that the unit, the root mean square of
the eye's positions over every keyframe, moved with each one, a fixed eye pose
by 0.24 of a unit after six far keyframes. A press kept as a pose of the frame
it was made in would disagree with the table of the frame the arm has later.
So a press keeps the arm's readings its pose came from
(`Presses.Event.Arm`), and when the table the arm gives is not the one the
presses were fitted with, all take their poses again from their readings
(`Tips.Set_Frame`); a unit grown by a tenth moved the tips 12 sigma when it
did not. A press never carries a pose from one beat to the next either: its
way back is to the readings its descent began from.

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
- The hand's sizes (width, thickness, face, depth) are unmeasured: they were
  fitted from the matcher's moves of a lobe's pixels, and the hand asks no
  instrument now. The matcher is for later, asked only about the pixels of
  lobes already found; `Lobes.Find` and the shape's tests are kept for that.
- The eye must have seen the closer's readings from two poses of the arm, as
  the boot gives it (A14: 269 at the starting reading). A body whose arm does
  not move before the closer is swept gets `Unlocated` for the channel, with
  the poses it has; the decider does not yet move the arm to make them.
- A lobe is attached to the image's border, or to the robot's own pixels that
  did not change, which the model does not yet give (`Self_Mask` is empty): a
  hand whose fingers do not reach the border of its own eye's picture gets no
  lobes. Path A takes the memory of the eye's poses (`Hand.Selfsight`, written
  so that the move is a rename) into the model behind `Self_Mask`.
- Pixels the mixture cannot tell (the other kind's share of them is above what
  one test alarms at) stay unassigned; the log counts them (about 4 % of
  A14's changed pixels). Dark stripes on dark stripes, the keyboard keys under
  A14's open finger, are where it is wrong: 91 % of the assigned pixels agree
  with the dark region of the starting view, and what is wrong forms
  fragments that the size rule leaves out of the lobes.
- A finger much smaller than the others (under the share 1 / N of the biggest
  part, N the parts of its end) is taken for a fragment, and one cut in several
  parts by a gap wider than a pixel or two is several lobes; the closing
  direction needs two lobes, or one and the robot's still pixels (empty until
  the model gives them).
- The subprograms that fitted a lobe's shape from the matcher's moves
  (`Lobes.Find`, `Shape.From_Moves`, `Shape.Fit`) are not reached from the
  driver now; the deadcode report lists them.
