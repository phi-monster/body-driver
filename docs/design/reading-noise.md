# Reading noise and what counts as motion

`Driver.Robot.Channels` decides, beat by beat, whether a group's reading
moved, whether a target asks for motion, and when a push is over. Every one
of those judgments compares a change with the reading's own noise at rest,
so the noise estimate decides what the body thinks is motion.

## A resting reading is a mixture

A reading at rest does one of two things from one beat to the next:

- it **repeats exactly**: a simulator between physics updates, an encoder
  that has not ticked, an echo of the last command;
- it **changes by its jitter**: a physics step that settles the joint by a
  hair, an encoder flicking one count, sensor noise.

The noise that answers "is this change motion?" is the size of the second
kind. Exact repeats say nothing about how far a resting reading moves when it
does.

The median absolute deviation of all rest changes is the obvious robust
scale, and it fails here: once more than half the rest changes are exact
repeats, it is zero. A sigma of zero makes every jitter significant, so the
reading looks as if it is always moving.

## The measurement that showed it

On the x5 boot recording (RX5, 634 beats), every arm channel measured noise
exactly zero. Per arm channel, out of about 400 rest beats:

- about 250 changes were exact repeats (between 224 and 276 per channel);
- the other 125 to 172 changes had a median magnitude of about 3e-5
  (between 6e-6 and 8e-5 per channel), which is simulator jitter;
- the largest nonzero rest change was about 0.2: a joint still settling
  after a push whose target had stopped changing.

The two grippers repeat exactly at every rest beat (494 and 496 of 494 and
496): their reading is an echo of the command.

With sigma zero, jitter of a few 1e-5 passed for motion. During the lockstep
sweep, where both arms are commanded together, a jitter-level move away from
its target ended one arm's push a beat before the other's. On that beat the
left arm was still moving and seen by its wrist eye (mean displacement
750 px) while only the right arm counted as pushed. The lock-in then
credited the right arm with the left wrist eye's whole image. Eye 2 was
mounted on the wrong arm, and the left gripper became the right arm's closer.

## The rule

- A channel that repeats exactly at every rest beat has noise zero: any
  change of it is motion. This is right for an echo.
- Otherwise its noise is the robust sigma (median absolute deviation) of
  the rest changes that are not exact repeats, divided by the square root
  of two, since a change is the difference of two readings. The degrees of
  freedom are those of the MAD over that many changes.
- Rest beats are beats where the target did not change and no push of the
  group is under way. Finding pushes takes the noise, so the two are
  measured in turn until the pushes found stop changing.

The test `robot.channels` holds a joint that repeats exactly two beats in
three and jitters by 1e-5 on the third. Its noise must come out at the
jitter's scale, no jitter may count as motion, and a push of 0.1 must.
Counting the exact repeats again makes the test fail.
