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
| `Hand.Lowering` | whether a push that asked the hand to go down took it down: the readings' whole shortfall and the hand's points', against what free pushes of the descent fell short by |
| `Hand.Slide` | how far a finger slid under a press, read at the edge of its mask in the picture under the press |

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

**Lesson: the seeds are evidence in every brightness round, not only where
the rounds begin.** A pixel whose two brightnesses belong to the robot as much
as to the world (a closed finger's lit face against the table's grey at the
other end) is told by nothing the histograms hold, and the histograms drift
from the order of the levels to whatever labelling explains itself. A16's
final ends of the second hand (88 090 changed pixels, eye 3): the lit face of
the closed left finger, 10 177 pixels of a grey a little darker than the
table, began at the end that shows it darker, right, and went in nine rounds
(1 074 of them by the third, 8 727 by the ninth) to the kind with the robot at
the anchored end; 9 229 of them were there when the rounds ended, the part was
never turned back (the labelling it had made was the likelier by 15 427 in the
log of the whole mixture, and by 38 706 for the part against the rest), and the
lobes' tips were 60 and 125 pixels low (the face, 10 216 pixels of it, took
the place of the open finger as the lobe's open end, and the closed right
finger's inner face and outer face were two parts). The poses had called
almost none of that face the robot's, and they are right about the robot: 53 %
of the pixels given the kind with the robot at the anchored end are seeds and
3 % of the others, the robot there holding still in the eye's picture while the
eye moves and the world not. Each kind now has seeds at a rate of its own,
found with the histograms from the shares, and a pixel's seed or none is
heard in every brightness round: the face stays at the closed end (0 of the
window's pixels to the anchored end, from 9 229), the lobes' tips on A16's last
stages are at the fingers' tips but one, the closed left finger's, which is 30
pixels low (the baseball's shadow, unchanged, cuts the tip off the finger),
and A15's final ends, three lobes of two fingers and closed tips 100 and 175
pixels low, are two lobes with their tips (`hand.lobes.lit` and
`hand.lobes.keys`: the changed pixels of the two stages at every tenth column
and row, 76 of 107 and 63 of 67 pixels of a window of the closed finger given
to the anchored end without the seeds, none with). Three forms did worse and
are not the one. Rates for the pixels the ends differ most at and for the
others apart return the failure, for the face is most of the others and its
own labelling sets their rate. Seeds in the neighbours' rounds too leave 364 of
the 2 522 pixels dark at both ends to the right end and 2 158 to neither
(`hand.lobes.blank`): a flat dark world is as still over the poses as the
robot is, and the seeds there are a coin, the neighbours' to decide. Seeds in
the turning of parts count a part's thousands of seeds as thousands of
independent tests, and turned the final fingers of A15 and of A16's first hand
over (their closed tips 80 to 160 pixels low).

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
fragment by its size: it must hold at least the share 1 / N of the biggest
part's pixels, N the number of parts the end has (from three parts on; the
share a part would hold if all were as big as the biggest). The first form
parted the log sizes in two groups and let the parting stand when the groups'
mean log sizes differed by more than Z standard errors and the biggest part
was more than N times the size parted at; at A16's open end, of 18 032 and
17 036 pixels (two fingers) and 2 114 and 354 (pieces of the finger the jeans
lay on), the sizes parted in the middle of their gap, where the biggest was 3.0
times the size and had to be 4, so (the lit face being at the right end) the
2 114 pixels nearest the closed finger took the open finger's place and its tip
was found 160 pixels from where it is (`hand.lobes.pieces`: parts of 1 600,
1 600, 192 and 32 pixels, four lobes where two fingers are); another view of
it, 17 699, 14 354, 3 197 and 213, had a piece of 3 197 that no parting of two
groups could leave out. Now the thresholds are 4 508 and 4 425. A finger half the size of the others is kept (a
ratio of 2 against N of 3), and so are fingers that touch at one end and show
as one part twice as large, since each end is judged by its own parts and the
end with too few parts takes the other's threshold. The lobes are then the
parts of the end that has the more of them, and each part of the other goes to
the lobe whose part is nearest by their centres (pixel by pixel when fewer
parts remain than lobes: fingers that touched, each taking what is nearest).

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

A press is the arm blocked and then at rest once the push that drove it in has
let go. The tool's pose at that rest is where the hand rests on what it
pressed. Reading it while the push still drives the hand in would be wrong: in
the legacy driver's runs (V1B21) the finger sank 6.6 mm into the table while
pushed and came back to 2.5 mm once the command stopped; in A16 the first
press's arm eased back 0.52 mrad over the 83 beats after the let-go, and its
line of sight met the table at 4.798 units from the stop (the true table along
that sight lies at 4.82 to 4.84) and at 4.826 from the rest.

**Lesson: the rest after the let-go, not the stop and not the first rest after
the block (A16).** The watcher waited for the first beat at which the whole
body was still and its verdict was not Blocked. A16's three presses ended at
beats 10275, 10377 and 10546, each at the end of the retreat, where the tool is
at the arm's origin (the aim pose): the stops were at 10178, 10348 and 10529,
the tool 5.19, 0.46 and 4.47 units from the aim. Two things kept the rest from
being seen. The let-go is a push that asks nothing (4e-7 rad, the arm's own
reading as its target) and the arm eased back from the table during it, so
the step tracker judged it Blocked (it fell short of a target it had no
distance to); the verdict stands until the next push, so at the rest (10264)
the body was still but Blocked, and the watcher went on to the retreat. And in
two presses of three the pictures of the eyes were still settling at the rest
(10353 to 10355, 10533 to 10535) and the retreat began after three beats.
Now a press is found at the first beat after the push that followed the block
has ended at which the arm's own readings are still, whatever that push was
judged (`Presses.Observe`, which settles after a press and is free again at
the next rest unblocked, so that the let-go's verdict cannot begin another
press). It needs the let-go: a driver that retreats at once has the rest of its
retreat. The stop needs no let-go, but it is read under the push: the last step
of a descent that doubles with nothing to stop it can drive the hand in as far
as the whole descent before it was long, so what the arm gives under it differs
press to press (the shortfalls of A16's three stops: 0.0036 rad, 0.0055 rad,
4.9 rad). The rest has no push to give under. Measured in A16 the rest is the
pose within 0.3 mm of the truth and the stop within 0.9 mm; the reason for the
rest is the first, not the second.

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

A press the tip stopped says the tip, fixed in the tool frame, lies on the
surface: n . (R x + t) = d. The tip lies on its line of sight (one unknown) or
anywhere (three, the check); a surface is either measured before (a prior) or
unknown (three unknowns: offset and two tilts). Everything is solved together
by weighted least squares, so the surface's uncertainty is in every tip's. A
press the tip did not stop says only that the tip is above the surface (see the
lesson below), and is left out. The presses' noise is the one predicted from
the arm's pose and the surface's uncertainty.

**Lesson: a tip cannot be below the table, and an arm is stopped by much
besides the tip (A16).** The fit raised the presses' noise to the scatter
they showed, and dropped a press whose residual, predicted from the others,
was significant against that noise. A16's hand had three presses on its first
lobe: one the tip stopped (its line of sight meets the table 4.80 units from
the eye at that press's pose) and two the arm stopped on itself (the lines meet
it 16.77 and 9.78 units from the eye at theirs; at the second, link 2 was
within 6.3 mm of link 4, at the third within 2.0 mm of link 5 and 4.8 mm over
the table; the tip was 86 and 45 mm above the table). With the poses read at
the stops, the three hits disagreed by 12 units and the noise was raised
520-fold (the scatter 275128), so the test had no power left and all three
agreed: the tip came out at 14.2 +- 3.3 units, where the finger's vertex
nearest the line of sight is 4.70 from the eye, and the fourth press, a true
contact of the other lobe, went to the first lobe because its far tip led.
With the poses read at the aim (the old watcher) it was 17.3 +- 0.06, the two
stops being one pose twice. A scatter taken from a few presses cannot tell a
contact less repeatable than the arm from a stop on something else, and the
wider it is allowed to grow the more stops it takes in. The fit now reads each
press as a bound: the tip is no farther along its line than the hit, the
distance at which the line of sight meets the surface from the press's pose,
and only a press the tip stopped gives the hit itself. The tip is the lowest
hit; a press that leaves the tip above the surface by more than the predicted
noise is left out (Stopped), the most above first, one at a time; a press that
leaves it below stays and the ones above it go (A16's three: the first). One
press with the surface measured before fixes a tip and nothing checks it:
provisional (A16: 4.80 units, against the 4.70 of the vertex nearest the sight).
It is Confirmed when a second press, from a pose distinct from the first's,
lands on it within the noise; the same stop twice from one pose is not a second
press (`Touch.Distinct`: the positions or the turns apart by more than the
poses' uncertainty tells apart). The noise is not raised any more: a contact
less repeatable than the arm reads as the lowest of its presses, its error
bounded by the scatter, and a tip that two such contacts land on within the
arm's noise is rarer than it was. What real contacts scatter is measured
against the truth (A17), not assumed.

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

**Lesson: a pressed finger slides, and what a press gives is the finger as it
stood under it (A16).** The scorer set A16's two tips against the free finger
(the support point of its collision mesh at the closer's reading): along the
press they were 2.75 and 8.06 mm short of it, 4.1 and 10.4 times the sigma the
driver stated. Against the finger as it stood at the press both were within it,
along the press's normal (-0.53 and -0.41 mm): the fingers of the true robot, at
the press of lobe 1 (beat 10264), stood 5.1 mm (link 7) and 1.8 mm (link 8)
inward along their own axis from where the closer's reading puts them, and at
the press of lobe 2 (beat 10654) link 8 stood 17.5 mm in, the closer reading
1.0000 throughout: it reads what it was asked, not where the finger is. So a tip
a press gives is the loaded finger's (`Tip_Kind`: Loaded), at the press the tip
rests on (`Tip_Beat`), and the free finger's (Free) is unknown until the slide of
the presses it rests on is measured and the tips at both openings are known (the
slide is along the finger's travel, which is the difference of the two). The
tip's covariance holds what a press does not tell: across its line of sight the
spread of the lobe's tip region (its pixels in the cap within one mean width of
the tip, about the tip pixel: A16 lobe 1's, 0.351 units across a tip 4.83
along), not the 0.026 mm the pixel alone gives.

**Lesson: a slide is read at the finger's edge, not in its pixels (A16, A17).**
`Slide` measures the shift, along the way the lobe closes in its eye's picture,
at which the picture under a press shows the free finger's edge: the points of
the lobe's mask edge near its tip, each scored by the step of luma across it
(weighted by how nearly across the way it lies), the greatest score the shift,
from where the same score was greatest in the free picture (the points are read
a pixel off the edge) and as far from the truth as half what the patch's two
halves, taken across the way, put it apart. Three things that look right failed,
each on the data. (1) The free finger's pixels matched in the picture under the
press (the mean absolute difference of luma): a finger that enters the picture
from its side and is of one tone has the same pixels at every shift along it, so
the cost is flat on one side and the least is anywhere on it; a patch of the
pixels the finger would come to cover (a ring) put a floor under the other side,
but its Gaussian classes were wrong in the picture under the press: the black
finger that was 8 to 10 in the free view stands at 41 to 66 there (the room it
reflects had changed), the table beside it that was 120 to 140 is 40 to 77, and
A16's and A17's lobes at no slide gave -26 pixels twice, with a sigma of 14. (2)
The same with the least misclassified pixels over every threshold on luma (a
threshold per shift, so that a change of light moves it): the table's dark wood
against the finger's grey, and the least at -12 pixels for a finger that had
not moved. (3) The edge, scored by the step of luma across it of the free sign
(the finger is the darker side), weighted by the way across the edge against the
way of the shift, with the greatest score taken from the free picture's own
greatest: twelve measurements, five presses of A16 and one of A17 at two lobes
each, against the truth's finger positions (4.55 pixels to the millimetre at the
tip, 200 pixels of travel for 44 mm):

| press (beat) | lobe | measured, pixels | truth, pixels (mm) | sigma, pixels |
| --- | --- | --- | --- | --- |
| A16 10264 | 1 (link 7) | 23.11 | 23.2 (5.1) | 0.30 |
| A16 10264 | 2 (link 8) | 8.31 | 8.2 (1.8) | 4.52 |
| A16 10353 | 1 / 2 | -0.07 / -0.02 | 0 / 0 | 0.30 / 0.42 |
| A16 10533 | 1 / 2 | -0.23 / 0.05 | 0 / 0 | 3.78 / 4.12 |
| A16 10654 | 1 / 2 | 0.05 / 78.48 | 0 / 79.6 (17.5) | 0.30 / 4.65 |
| A16 10687 | 1 / 2 | -0.27 / 0.04 | 0 / 0 | 0.29 / 0.29 |
| A17 6998 | 1 / 2 | -0.22 / -0.13 | 0 / 0 | 0.30 / 0.54 |

The least score of several that stand out of the others is the one nearest where
the finger stood free: a slide is the smallest the picture lets it be (A16's
press 10533 had a second, stronger edge 196 pixels in).

## How the decider aims a press

A press turns the hand about the eye, so the eye keeps its view, until the
lobe's line of sight, tilted, points into the table its arm's own eye saw
(`Up_In_Arm`), then lowers the hand with doubling steps from the smallest move
that tells from the arm's own noise until the arm is blocked, lets go, and
lifts back. Once the presses so far fix the lobe's tip, the steps stop
doubling Z sigma above the contact they predict and go on by that sigma. The
first press is straight along the line of sight; then the line is tilted away
from the other lobes, on either side, by the angle to the nearest other lobe's
line (a lone lobe: the angle it travels through between the openings), doubled
while the presses are ones the tip rests on and halved when one stops short of
the table, until the lobe's tip is confirmed, or the tilts are past a right
angle or past the least that tells a tip from a stop that does not move with
the tilt (`Aims.Least_Tilt`), or a press cannot be made. Tilting away from the
other lobes keeps them behind the aimed tip; in the
legacy driver's runs (V1B79), tilting towards a finger's own body made the
body touch first. No step takes the eye below the table its arm's own eye saw,
the height of the eye above it less Z of its sigma.

**Lesson: a doubling step is bounded by the eye's height above the table, and
the planner's answer to a step beyond reach was a turn the long way (A16).**
The third press of A16's first lobe doubled to 5.85 units after 5.80, with
the eye 10.0 units above the table: a lowering to 11.65, the eye 1.65 units
under it. The step asked of the arm 5.68 rad of joint motion (the one before it,
2.9 units, 0.33 rad), of which it made 13.6 per cent before link 2 lay on the
table and the arm folded on itself. Replayed at the readings of that push,
`Plan_Reach_In_Arm` for the same lowering ends at joint 2 = -3.677 and joint 4
= -4.310; planned in twenty steps of 0.29 units, each from the readings the
last ended at, it ends at 2.610 and 1.977: the same pose, each joint a full
turn apart (6.287 and 6.288 rad against 2 pi), the second 3.26 rad from the
readings and the first 5.68. The planner's solve ends at a representative of
the joint angle that is not the nearest to where the arm is, and the arm is
asked to turn the long way round (path A's: `Motion.Plan_Reach_In_Arm`). The
step capped to the eye's room (4.2 units less Z sigma) ends 1.18 rad away.

**Lesson: the smallest push is read where the aim leaves the tool, and the
step tracker calls a push short for a visible step.** A15 was the first live
run that pressed, and its presses failed in two ways, both in the log. (1) The
first press of the first lobe lowered from a least push too small to print: 34
pushes, the first 33 asking of the arm less than five millionths of a radian,
"blocked after lowering 0.0000". The arm's frame is the eye at its reference
readings, where the tool's place is known exactly, and the arm stood there
before the aim; the least push was read there, not where the aim leaves the
tool (0.06 to 0.2 two presses later). It is now read at the readings the aim's
plan ends at (`hand.pressing.least`). (2) After the first press that met the
table (6 pushes, the last of 3.26, "delivered 0.716" of its ask) every push of
every later press was judged blocked at once, with the whole of the ask
delivered: the step tracker calls a push short when a joint stops short of its
ask by its visible step, 1 to 5 millionths of a radian, and from the end of
that press on it was true of every step whatever its size or direction. Of the
46 steps from the first press's end on, 31 pushes of 0.005 to 0.02 radian and
15 aims and retreats of 0.1 to 0.9, every one was judged blocked, having
delivered 0.9995 or more of its ask, and each press ended at its first push,
in the air. The press reads the verdict as every decider does
(`Report.Outcome`); the defect lives in the step tracker (`Steps`, path A's):
a push that delivered 0.9995 of its ask is not blocked, and the push that met
A15's table, 0.716 of its ask against a noise of 0.0001, is. The presses found
in the stream were not misled: from all of it the estimators kept one press,
the one that met the table.

**Lesson: the band about a predicted contact has two edges, and a press that
stopped in the air fixes only a bound (A17).** A17's first press that the hand
kept (beat 6998, the closer at its open reading) stopped the arm in the air: at
the rest after its let-go the lowest vertex of link 7 was 86.2 mm above the table
(92.1 mm at beat 6975, where the arm stopped) and that of link 8 187.4 mm, and
link 2 lay within 8 mm of link 4, as A16's stops had (A16: link 2 within 6.3 mm
of link 4). The press before it, the one the closer was not at an opening for,
stopped with link 7 105 mm above the table (beat 6921). The line of sight through the tip meets
the table 17.87 units from the eye from that pose (266 mm: the arm's unit was
14.9 mm in A17), and the finger was not on the table there. One press fixes a
provisional tip, and a provisional tip is the hit, the farthest the tip can be:
a bound, not the tip. The next press (tilted by 0.28 rad, aimed at "0.3416 +-
0.0437 above the surface the presses so far fixed") took that bound for the
contact: from the first push it was within Z sigma of it, so every push was the
larger of the sigma and the least, 0.0437 units, 0.6 mm, at 8 beats a push, and
the band had no far edge: the hand went down 0.15 m in 1,200 beats (truth: the
finger's link 7 from 0.936 to 0.786 m, beats 7021 to 8259), met the table, and
went on as it slid along it for 4,600 beats (8 cm in y; the finger's z 0.7829 to
0.7813), each push reached: the joints followed 83 to 97 per cent of each, one
of them 22 per cent, and the step tracker, which judges a group, said reached.
Nothing ended it: not Blocked, and not Spent, because the eye's room above the
table never ran out for a hand that stayed on it. The log said nothing for its
last 25 minutes because a press says how it went down only when it ends.

The band is now as wide as its prediction says, Z sigma either side of the
contact, and past it (nothing met) the steps double again, from the band's own,
counted as blind: the prediction was a bound. The descent also ends, Spent, when
it has made the steps a doubling schedule needs to cover the eye's room above the
table (doubling from Least to the room, a step cut to it, one that finds none;
with a prediction, its band and the doubling past it): steps that were reached
and lowered nothing end on the schedule, not on the clock. A hand that ends Spent
goes back to where the descent began, for the presses after it begin there.
(`hand.measure.press` (e), `hand.measure.room` (d): the surface 10 below the
predicted contact is met by a doubling, and steps that lower nothing end at the
schedule's ten.)

**Lesson: the closer is at the opening before the press is made at it (A17).**
The first press of A17's first lobe (beats 6913 to 6920) was blocked, and "a
press with the closer at neither measured opening is not used": the closer read
0.6859 where its openings read 1.0 and 0.0. The closer had been asked for 1.0
since before beat 6840; its reading had crept from 0.59 to 0.686 over 70 beats
while the arm stood at the sweep's pose, and at that pose the truth has the left
finger's lowest vertex on the table (0.0 mm, at (-0.274, -0.261), from beat 6860
to the aim's first move at 6913; link 8 was 20 mm above it): the finger could not
open against it. The aim's turn took the hand off the table and the reading
jumped to 1.0 at beat 6918, five beats into a press that had begun without it. A
press now waits for it: after the aim, the closer's readings are those of the
opening or are asked for once more (the finger is free then), and when they are
still not there no press is made at that opening (`Closer_At`).

**Lesson: a descent ends when the hand stops going down, whatever the arm does
(A17).** The third press of A17 did not stop at the table, and it did not end
as a press, because "the arm followed every push" and "the hand went down" are
two things. From the truth (beats 6990 to 13600): the free pushes
lowered the tool's last link, link 6, 1.0 to 1.1 mm each, at 8 to 9 beats a push;
the lowest vertex of link 7, the finger, reached the table at beat 8208 (0.7 mm
above it at 8200, 0.0 at 8208) and stayed within 0.4 mm of it until beat 13,200
while it slid along the table 0.8 mm a push (y -267.9 to -267.1 mm at the first
push after contact). The tool's origin and the camera did not stop at once: the
push that brought the finger to the table lowered link 6 by 1.0 mm and the next
three by 1.0, 0.9 and 0.8 mm (a hand pivoting on its finger while the finger
slides), the following ones by less and less, 9.9 mm in all by beat 8920 (link 6
118.1 to 108.2 mm above the table, the camera 55.9 to 49.2 mm), and then the
camera rose again, to 61.6 mm at beat 11,120 and 79.8 at 13,600, as the hand
rolled up on it. So what stops at the first push after contact is the contact,
and it is a point of the finger; the tool's origin, which the plan moves as it
asks, tells only after the pivot has spent itself.

The arm's readings show the difference: a free push (A17 beats 7044 to 7076:
5.8 mrad of joint motion each) was delivered 100.0 per cent along its ask with
1.8 microradians across it (0.03 per cent of its length); the crawl's push at
beat 9009, 11.6 mrad, 91.9 per cent along it and 3.4 mrad across it, 29 per cent
of its length (joint 2 delivered 81 per cent of its share, joint 3 26, joint 4
108, joints 1, 5 and 6 whatever the contact let them). The step tracker judges a
group by the part of the delivery along the ask, 8 per cent short here, and said
reached; the 29 per cent across the ask is the hand turning and sliding on what
it touches.

`Driver.Robot.Hand.Lowering` judges a push by how far it fell short of its ask,
in two places, and either is a stall. In the readings: the whole vector from
where the joints stopped to the push's target, as a share of the ask's length,
no geometry read, so it is the same for any arm. At the hand's points
(`Tool_In_Arm` at the push's start, its target and its end): the tool's origin
and the tips the hand has measured at the opening the closer stands at, each a
share of the lowering it was asked (a tip is a point of the tool frame, a
provisional one a bound beyond the finger, which rises when the hand turns about
the finger on the table and so shows the stall the origin does not). A share is
a stall when it is larger than Z times the largest share any push of the descent
before it fell short by (free pushes fall short by one share of their length,
however long: a joint held against gravity settles short by a share of its push;
three pushes are the least that have a scatter), and the push asked what the one
test of motion sees (`Channels.Visible` of its ask) and fell short of it by what
that test sees (of the shortfall vector): that test is the floor under both. The
first version had the tool's `Least_Push` for a floor, the tool's uncertainty at
the pose it stands at; it is the uncertainty of a fit, common to the two poses a
push is the difference of, and it grew with the descent: A17's pushes were 0.0561
units, and from beat 7502 the lowered tool's was more, so nothing was judged after
it, the contact included. The pushes are forgotten when a push asks the hand
nothing down (the mean of its points: a retreat, a hold, a let-go that raises one
side as it lowers the other), so a descent is compared with itself, and a stall
among the first three pushes is taken for what free pushes do. A share is of the
farthest any point was asked to go, not of what was asked down: A22's aims at
presses 2 and 5 turned the hand about the way down, asked every point of it 2e-16
down by the rounding of the turn, and 4.7e-8 came back up, a share of 2.1e8 that
stood as the largest free share of the descent and blinded the points' measure
for the pushes after it (the readings' measure, which has its own, went on); the
let-go after press 5 (one point asked up 0.0525 that went up 0.139, the other
asked down) was judged a descent push and a stall by the readings. Only a point
asked to go down can have stopped going down. The verdict is read from the
stream by the estimators as the press is, so the same push is a block for the
watcher that finds the press (the verdict stands until the next push begins, as a
Blocked one does), and it is read with the heights before the next step, not by a
beat of its own, so the descent that made it ends as a press, not Spent: a bound
or a contact, whichever the presses tell, and not thrown away.
(`hand.lowering.*`, `hand.measure.stall`.)

**Lesson: a closer the table holds is freed by raising the hand (A17).** A
finger resting on the table cannot slide along it, and the closer's reading,
asked back to a reading it has been at, stays short of it: A17's, asked back to
1.0 at the end of its sweep, stood between 0.59 and 0.686 for seventy beats and
read 1.0 four beats after the aim lifted the hand. The sweep's returns to a
reading the closer has been at (`Return_Channel`) check that it arrived (within
its noise, or a step no eye tells from it), and if it did not, the hand is
raised along the way up, in steps that double from the least move of its tool,
while each raise sets the closer moving; a raise after which it has not moved
was not what held it, and the hand is raised no more, nor higher than the eye
stands above the table (`Free_Closer`; `hand.measure.held`). The hand stays
where it was raised to: the presses aim from wherever the arm stands. A19
(8597f62) lost its first press the same way (`a press with the closer at neither
measured opening is not used`, blocked after lowering 5.1 units).

**Lesson: a hand is measured from its own arm's motion, not from what the boot
did (A25h).** A closer's lobes are found from what the arm's motion does to its
eye's picture: the robot stays where it is in the picture while the world moves,
so the deviation of a pixel over the poses of the rest of the body is small
where the robot is (`Hand.Selfsight`). A22's closers had the poses from the
boot, which moved both arms for fifteen minutes with the closers at their start
readings; a run that reloads a body file goes from the file to the hands with no
arm motion, and A25h logged "the arm has not moved the eye against its
surroundings ... 0 poses, two are needed" at every round. `Measure` now sees to
the poses itself (`Gather_Own_Poses`), before a closer is moved and again after
its sweep when its lobes were not placed for want of them (Unlocated, or Unplaced
with the changed pixels Unseparated by the poses): the arm raises the eye along
the way up (`Up_In_Arm`), first by the least move whose readings the body's one
test of motion sees (the last readings of the plan, tried before anything moves,
doubled until they show), then by twice that, a pose each, until the eye has
seen the closer's readings from the poses asked (`Selfsight.Needed`, two; after
a sweep that did not place, twice what it has) or cannot be raised. A pose's
frame is kept once the eye's picture has rested, and the wait for it is as long
as a view takes to form. Nothing is raised when the eye has the poses, so a run
from zero is as it was. The poses are taken at the closer's start reading, which
is an end of its travel in every body met (A17, A19, A22 start open); a closer
that starts between its ends would need them at an end. (`Gather_Poses`,
`hand.measure.poses`.)

**Lesson: the free finger is the loaded fit with each press taken with the tip
it had slid to.** A press loads its finger, and the finger gives way along its
own axis (A16: 5 and 17.5 mm inward, the closer's reading exactly where it was).
What a press fixes is the contact of the finger as it stood under it (the
loaded tip), and the finger as it stands free at the closer's reading (the free
tip, which is what a hand opened to that reading brings to a surface) is that
less the slide. The slide of a press is a share of the lobe's travel between its
two openings (`Slide.Measure`: positive inward, towards the closed end), and the
travel is the difference of the lobe's tips at the two openings, so the free tip
of a lobe is unknown until the lobe has a tip at each opening and until the
presses its tip rests on have their slide measured (A16, A17 and A22 pressed the
open opening first, the closed after all the open ones).

The free fit is the loaded one with a vector on each press (`Touch.Press.Slide`,
`Slide_Covariance`): the press's height above the surface is that of the tip the
finger had slid to, `n . (R (x + s) + t)`, with `x` the free tip on its line of
sight and `s` the share times the travel; `s` is in the equation's constant, in
the equation's variance (`Lift' Cov Lift`, Cov the share's variance times the
travel twice over plus the share squared times the travel's own), and in the hit
the press gives. The travel comes from the tips of the loaded fit, each of which
holds its own slides, and then from the free tips of the first pass: two passes,
since the shares are of the free travel. The presses of a lobe with nothing
measured of their slide are left out of the free fit, not taken for unslid ones.
The free tip is confirmed (`Confirmed (Free)`) when two presses at poses apart,
each taken with its slide, land on it within the noise the slides' uncertainty
makes.

`hand.tips.free` on two fingers that slide inward 12 and 28 per cent of their
travel (3 and 9 mm) under every press, 12 presses a tip, at both openings: the
loaded tips are 2.0 to 7.3 mm off the free fingers, the free tips 0.0007 to
0.06 mm off (their sigma 0.12 to 0.16 mm), and confirmed; pressed at one
opening only, or with nothing measured of the slides, there is a loaded tip and
no free one. The API: `Tip_In_Tool`, `Tip_Beat` and `Tip_Confirmed` of a hand
take `Kind` (`Loaded`, the default, or `Free`).

**Lesson: a line of sight that meets the surface behind the eye is not a tip
(A19).** A press aimed at lobe 1 (0.956 rad) and given by its direction to lobe
2 fitted lobe 2's tip -79.4 units along its sight, a point behind the eye (`a
press at the OPEN opening ... lobe 2 provisional -79.3757 +- 0.5164 along its
sight`), and the next prediction took it for the surface. A tip is in front of
the eye: a fit that puts it at or behind it is not a tip, and the presses fitted
to it stopped on something else or belong to another tip. The tip is not Ok
then and the presses on it do not agree (`hand.touch.behind`).

## Open

- A closer that follows neither way at its first push (a finger pressed so
  hard on the table that its small first pushes do not move it) is left
  stuck by the sweep; the hand does not raise the arm and sweep again. A17 and
  A19 had fingers on the table at the sweep's pose, and both closers did start,
  and were held only on the way back.
- A stall among the first three pushes of a descent is taken for what free
  pushes do (nothing is compared with fewer than three) and enters the
  largest share the descent's later pushes are compared with, so that the
  descent judges nothing more; the step tracker's block still ends it.

- A contact that is less repeatable than the arm's predicted noise is not
  modelled (the fit no longer raises its noise to the scatter it sees, which a
  few presses cannot tell from stops on something else). Such contacts read as
  the lowest of them, and the tips two presses land on within the arm's noise
  are fewer. What the contacts of the rigs scatter, at different tilts, is to be
  measured against the truth (A17 has it); a measured term goes in the
  presses' noise then, not an assumed one.
- The tilts of a lobe's presses start at the hand's own angle (to the other
  lobe), as they were. A16's two tilted presses, 0.96 and 1.23 rad, both
  stopped on the arm itself, at 0.46 and 4.5 of the 7.2 units of lowering the
  table asked, and their aims stood the wrist's fifth joint at 0.743 rad,
  0.002 under the largest reading it had in the run; the straight press's aim
  had it at 0.258. A tilt that stops short is now followed by half of it, down
  to the least that tells a tip from a stop that does not move with the tilt
  (0.239 rad for A16's first tip). Whether the half reaches the table, and the
  second press of a tip is a contact, is A17's measurement.
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
