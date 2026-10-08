# The body file

At boot the driver measures the body: how each group responds, how much its
readings shake at rest, the kinematics of each arm, each eye's lens and where
it is mounted, the shapes of the links, and each hand. What it measured can
be kept in a file that travels with this robot, so the next boot does not
start from zero.

The file holds the body only. Nothing about the world, the task or the
things on the table is kept in it.

## 1. Use

```
body_driver --listen 9080 --body body.dat     # reload from body.dat if it is there, store into it
body_driver --listen 9080                     # measure from zero and keep nothing
```

With `--body`, the boot reloads the file when it exists and writes what it
measures back into the same file as it measures: after recognizing the groups,
after sweeping the arms, and when the hands are measured. A run that fails
later keeps what it had measured, and the next boot reloads it. A write
replaces the file whole (it goes to `body.dat.part` first), so a run killed
in the middle of a write leaves the file the stage before wrote. Delete the
file to measure from zero again.

## 2. What a reload does

Every quantity is stored with its uncertainty and the version of the method
that measured it.

- A quantity whose measuring method has changed since it was stored is
  measured again, and so is every quantity measured from it (section 4); the
  others are reloaded.
- A file whose key (the size of every group, which groups take commands,
  the size of every image) is not what the robot reports belongs to another
  body: nothing is reloaded from it.
- What is reloaded stands for the session: the boot skips the stages that
  measured it, and the estimators do not measure it again. The readings'
  travel and the step responses go on growing from what was reloaded.
- For that reason the kinematics are reloaded only when every arm that carries
  an eye is fitted in them. A file written while the arms were being swept
  holds the fits there were and the arms that had none; its recognition
  stands, and the next boot sweeps all the arms again.
- Quantities are never patched by hand, and a stored number is never taken
  over a measurement that disagrees with it.

`driver/bin/replay RECORDING --body FILE` writes the body measured from a
recording, so its estimates can be scored against the truth of the run
(`driver/bin/score`).
A run that reloaded a body file replays as it ran: the driver records the
text it reloaded (`Driver.Recording`, kind F), and replay reloads that text
where the run read it, even after the run rewrote the file.

## 3. Do not edit it

Every number in the file was measured by this robot. A number edited by hand
is a number nobody measured, which the driver's rules forbid. To measure
again, delete the file or give another path.

## 4. What it holds

A JSON object. `key` is the body's shape; `beats` how many beats measured it.
Every other member is one quantity with its `method`, listed with what it is
measured from:

| Member | What | Measured from |
|---|---|---|
| `noise` | every channel's reading noise and its degrees of freedom | the readings |
| `travel` | every channel's lowest and highest reading so far | the readings |
| `steps` | every group's longest wait for an answer, its free pushes' shortfalls | the noise |
| `lags` | every eye's image lag, and whether it is known | the noise |
| `responses` | every eye's cell noise at rest, what each channel's push does to each cell, each group's effect on the eye | the noise, the lags |
| `graph` | every group's role and arm, the arms, the carrier, every eye's mount | the responses |
| `kinematics` | every arm's fit (joints, lens, uncertainty), its table, where it stands in the world; the lens and place of every eye fixed in the world | the graph, the responses |

`Driver.Robot.Load_Body` reloads a file into a model by the same rules; a
model that has seen no robot takes the file's groups and eyes, so a measured
body can be loaded and planned on with no robot connected.

### The kinematics (method 6)

Per arm, `kinematics.arms` holds:

- `joints`, `lens` and `covariance`: the fit of the arm and of the eye it
  carries, in the eye's frame at the arm's reference readings (`reference`):
  the arm's own frame, in the arm's own unit. The unit is the first fit's:
  the root mean square of the eye's positions over the keyframes of that fit.
  A keyframe taken later refines every term in that unit and moves no
  length, and a reloaded arm is not fitted again, so what is stored stays in
  the unit it was stored in.
- `table`: the plane most of the arm's tracked points lie on, each point
  judged by its own depth uncertainty, in the arm's own frame, with its whole
  uncertainty: its points' scatter about it, what the fit moves every depth
  by together, and the lens's lines of sight. `normal` points towards the
  eye and is the arm's up. `centre` is the point of the plane whose height is
  least uncertain: there `offset_sigma` is its height's sigma along the
  normal, uncorrelated with its tilt, whose covariance `tilt` gives (in
  radians squared, towards `tangent` and towards the normal crossed with
  `tangent`, and between the two). `points` is how many points it rests on,
  `scatter` their residuals' chi square per degree of freedom. The first
  arm's table gives the world its up.
- `placed` and `placement`: where the arm's frame stands in the world, which
  is the first arm's frame: a point X of the arm's frame is at
  `rotation * (scale * X) + centre` there. `covariance` (6 x 6) is that of
  the turn and the centre, `scale_sigma` the scale's. `through` names the eye
  that placed it: an eye whose view shows both arms move and holds both arms'
  tables as one plane. It is a fixed eye that sees both arms, or the arm's
  own eye when that eye shows the first arm. `points` is how many of the two
  arms' table points that eye's view tied together, and `px` the noise of
  that tie in the eye's units. The first arm stands as it is (`through` 0).

Per eye fixed in the world (the graph mounts it `world_fixed`), `kinematics.fixed` holds the lens and the place the
eye was measured to have, from the first arm's tracked points and the eye's answers to where they are:

- `known` says whether the points determine them. When they do not (all the points on the table, too few
  answers, answers that leave a focal length or the turn or the centre unsure), `why` says what they leave out,
  the eye is unknown (every answer about it carries an unknown pose and unknown lines of sight) and the numbers
  are the fit's last state, not a result. `arm` is the arm whose points measured it (the first: the world).
- `lens` holds the focal lengths and the principal point, in pixels, and the distortion (zero when the two
  terms are not significant as a pair). `rotation` (row by row) and `centre` are its camera frame in the world,
  which is the first arm's frame, in that arm's unit.
- `covariance` (12 x 12, row by row) is that of the lens's six terms, in the order of the arms' (the logarithms
  of the two focal lengths, the principal point, the two distortion terms; the distortion rows are zero when not
  kept), the camera's turn about its own axes (three) and its centre in the world (three). It is the sandwich
  of the normal equations around the spread of the gradient, as the arms' (the spatial kernel of the residuals),
  with the spread the points' own uncertainty adds: each point's depth, and what the first arm's fit moves every
  point by together (its covariance of 36 terms carried through the gains of every depth and, for a point on the
  table, through the table's response, with the table's scatter). A point on the table is where its line of sight
  meets the table, not at the depth of its own refinement: the error of a depth along the table's normal would
  pass for the structure that fixes the lens, and a plane alone fixes none.
- `used` of `offered` answers fit (the answers whose round trip comes back to the query, within the noise of all
  the round trips, and whose point the arm has); `sigma_px` is the noise they show, `distorted` whether the
  distortion terms are kept. `from_matches` and `from_sets` say what the fit rests on (the first arm's keyframes
  with matches, the sets of its reference matched into other eyes): it is measured again when either changed.

Method 4's covariance of the fit counted the sightings of a keyframe as one
share of the gradient and the keyframes as independent. The matcher's errors
are not: a point errs alike in every keyframe (where the matcher finds it is
the same in every view) and so do points near each other, the points of a
keyframe err alike, nearer more, and a sighting has errors of its own. The
covariance of method 5 reads from the fit's residuals how the errors of two
sightings depend on each other, as a function of the distance between their
points in the reference picture (separately for two sightings of one
keyframe and of two keyframes, across, down and across with down; the fall
with distance and the floor at zero are all that is assumed, an isotonic
fit), and weights every pair of rows of the gradient by it (a spatial-kernel
sandwich, Conley's), made positive semi-definite. The lens it gives is
within Z's tail of the truth where method 4's sigmas were 3 to 4 times too
small (A11: a chi square of 94 and 51 on the lens's six terms against the
truth, 15 and 10 now, Z's tail 21). The stored `covariance` is that matrix.

What the lens's sigmas do not cover. The covariance describes the random
errors the residuals show, about a fit taken to be at its minimum (the final
refinement runs until a step cannot move any combination of the parameters by
more than 1 % of its standard error). It does not cover a bias the fit
absorbs. On A10 and A11 the lens came out 2 to 4 sigmas off the truth along
one axis of its covariance, the weakest, a mix of the focal length and the two
distortion terms (its sigma is some 1.5e-4 in the terms' own units), with the
same sign in both arms and in both runs of the one scene, while the five other
axes lay within their sigmas (chi squares of 4 to 8 on five terms). The sum of
the sightings' true errors, each weighted by its influence on that axis,
reproduces the lens's error along it to a few per cent, and little of it is a
point's error in every keyframe or a keyframe's shift (4 % and 2 % on one arm,
31 % and 23 % on the other): the rest is what is left of each sighting when
those are taken out, and it is no error proportional to the displacement (a
scale of 1e-4 of it) nor one that belongs to the place in the target picture
(1 % of the variance). A reader of the lens must therefore take the sigma of the
focal length, of the distortion terms and of any quantity that leans on their
difference (the scale of a view, the position of a point far from the picture's
centre) as a lower bound, not as the whole error. Taking the distortion terms
as zero does not remove it: it moves the same error onto the focal length, 0.2
to 0.8 px too long on the recorded fits, 1.3 to 4.6 of that fit's smaller sigma.
The principal point and the joints' terms are not on that axis.

Method 3 kept only the table's normal, its offset and two scalar sigmas: a
plane of measured covariance, which pressing a hand onto it needs, cannot be
rebuilt from them. Method 2 placed the arm from its own eye's view of the
first arm's points. When the two arms' eyes do not share a view, that is
wrong: a dense matcher answers points an eye does not show, and those
answers then read as an eye at the first eye's centre. A file of method 2
or 3 is measured again, and so are files of methods 4 and 5 (they have no fixed eyes).
