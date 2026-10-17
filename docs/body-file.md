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
| `noise` | every channel's reading noise and its degrees of freedom, and its creep (`drift`: its steady change a beat at rest, 0 where none stands out) | the readings |
| `travel` | every channel's lowest and highest reading so far | the readings |
| `ends` | every channel's end down and up, found where the body stopped a push of it at the same reading from two poses of the other channels (`low`, `high`, with `has_low` and `has_high` saying whether there is one), and the `stops` not yet found again, each with the channel, the sense, the reading and the group's readings it was made from | the noise |
| `steps` | every group's longest wait for an answer, its free pushes' shortfalls | the noise |
| `lags` | every eye's image lag, and whether it is known, and the longest it took to settle after a move (`settling`, beats) | the noise |
| `responses` | every eye's cell noise at rest, what each channel's push does to each cell, each group's effect on the eye | the noise, the lags |
| `graph` | every group's role and arm, the turns at which the recognition saw each of its channels move an eye (`seen_at`, 0 where none did; the sweeps begin there), the arms, the carrier, every eye's mount | the responses |
| `kinematics` | every arm's fit (joints, lens, uncertainty), its table, where it stands in the world; the lens and place of every eye fixed in the world | the graph, the responses |

The `ends` are what the arm showed by being stopped, not by being seen
(`Driver.Robot.Motion.Note_Stopped`, `Driver.Robot.End_Of`): a decider that knows a push of the arm was stopped by the body
itself and not by a surface it pressed says so, and the channel the stop is the shortfall of, by a significance test of each
asked channel's part of the shortfall against its own noise, has the reading it stopped at noted in the sense it was asked,
with the group's readings there. A joint's end is a property of that joint, which no pose of the others changes: a stop is an
end when the channel stops at the same reading from another pose (some other channel standing somewhere else), the two
readings within what two independent contacts would not meet by chance (the false alarm rate of Z over the channel's travel,
and no less than Z deviations of the difference of two readings); until then it is a stop kept with its pose, and plans stay
free. An end is no nearer than the readings the channel has been seen at, and a plan that passes it
(`Driver.Robot.Motion.Plan_Reach`) is refused. The ends and the stops are reloaded with the noise they rest on and are never
reset by a recompute; a file of the first method (ends from one stop) is measured again. A boot that moves one joint at a time
from the rest pose confirms no end by itself: its stops share one pose.

`Driver.Robot.Load_Body` reloads a file into a model by the same rules; a
model that has seen no robot takes the file's groups and eyes, so a measured
body can be loaded and planned on with no robot connected.

### The kinematics (method 10)

Per arm, `kinematics.arms` holds:

- `shifts`: per channel, how far the arm's eye's view moves per reading unit of it, in pixels, as the
  keyframes measured it (per point, its displacements regressed on the changes of every channel that moved; the
  median over the points), 0 where not measured; `shift_sigmas` the standard deviation of each (0 where not
  measured, with `shifts_known` false), `shift_points` how many points each rests on, `shifts_seen` whether the
  view moves with the channel (false for a joint that moves a part of the body in the view, such as the hand's),
  and `shift_told` how many points told it. They are kept when the arm's sweep begins anew, and a body begun
  from the file sweeps from them.
- `chain`: the arm's joints along it, from its base to the eye, each by its
  reading's place in the group (1 for the first). The robot reports its
  joints in any order; the fit finds the order from the keyframes that moved
  several joints at once (method 7; method 6 took the readings' order, as
  the x5 gives it). Each joint's `c` is its turn (radians) or slide (the
  arm's unit) per reading unit, measured: the readings may be in any unit.
- `joints`, `lens` and `covariance`: the fit of the arm and of the eye it
  carries, in the eye's frame at the arm's reference readings (`reference`):
  the arm's own frame, in the arm's own unit. The unit is the first fit's:
  the root mean square of the eye's positions over the keyframes of that fit.
  A keyframe taken later refines every term in that unit and moves no
  length, and a reloaded arm is not fitted again, so what is stored stays in
  the unit it was stored in.
- `unit_changes`: the readings of the keyframes that define the unit, less the reference's, a row of the
  arm's channels each, so that a body begun from the file, which keeps no keyframes, still has the unit's
  uncertainty.
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
  `rotation * (scale * X) + centre` there. `covariance` (7 x 7) is that of
  the turn, the centre and the scale's logarithm, `scale_sigma` the scale's.
  `through` names the eye that placed it: an eye fixed in the world that
  watched both this arm and the first move, fitted to both (its pose in each
  arm's frame), and to which the two arms' tables are one plane. `points` is
  how many sightings that eye's fit rests on, and `px` the noise they show,
  in its pixels. The first arm stands as it is (`through` 0).

Per eye fixed in the world (the graph mounts it `world_fixed`), `kinematics.fixed` holds the lens and the place the
eye was measured to have from the arms it watched move: its picture is kept at every keyframe of an arm whose push
moves part of it (the lock-in), the points of its first such picture that respond to the push are followed through
the others by the same aligner as the arms' own keyframes, and the eye is fitted to the arms' fitted motion
(`Kinematics.Hand_Eye`): its lens, its pose in each arm's frame, and which link of the arm, and at what depth, each
point rides on.

- `known` says whether that determines them and the first arm (the world) is among the arms it was fitted to.
  When not, `why` says what is left out and the numbers are the fit's last state, not a result. `arm` is the
  arm whose frame is the world.
- `lens` holds the focal lengths and the principal point, in pixels, and the distortion (zero when the two
  terms are not significant as a pair). `rotation` (row by row) and `centre` are its camera frame in the world,
  which is the first arm's frame, in that arm's unit.
- `covariance` (12 x 12, row by row) is that of the lens's six terms, in the order of the arms' (the logarithms
  of the two focal lengths, the principal point, the two distortion terms; the distortion rows are zero when not
  kept), the camera's turn about its own axes (three) and its centre in the world (three), the arms' joints
  carried as their fits state them.
- `used` of `offered` sightings fit (those the model has a place for); `sigma_px` is the noise they show,
  `distorted` whether the distortion terms are kept. `from` counts the arms' keyframes aligned in the eye's
  views (and aligned again) that the fit rests on: it is measured again when that changed.

Method 8 keeps each arm's `shifts`, the view's motion per reading unit as the keyframes measured it; method 9
keeps their sigmas, the points they rest on and which channels the view moves with. Method 10 measures the fixed
eyes and the arms' placement from the arms the fixed eyes watch move (`Kinematics.Hand_Eye`), where method 9
matched the first arm's reference points into the other eyes' pictures by an instrument (its `from_matches` and
`from_sets` are now `from`, and the placement's covariance takes the scale's logarithm in).

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
