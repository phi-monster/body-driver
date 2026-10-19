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

## A step is not jitter

A reading that comes in whole units of its wire moves in steps: a servo's
ticks, or a joint whose ticks the body reports as four units each, which
steps by four. Its step is the least change it has shown
(`Channels.Step_Unit`). A joint at rest between two of its values flips to
the other and back. That flip is the reading's rounding, not jitter, and it
says nothing about how far a resting reading moves beyond it.

Counted as jitter, the flips made the body blind to its own motion. On w11
(servo seed 11 of the fake robot, 10-19), arm 1's fourth channel steps by
four. By beat 2540 its rest held 24 changes of four units each, the joint
flipping one tick and back in the truth. Its noise came out at 4.19 units,
and a change had to pass 17.8 units in a beat, more than four ticks, to
count as motion.

On w12, the same body, the flips also left the noise resting on very few
changes. The estimate at beat 5940 found arm 1 pushed at 1076 beats in its
first round, 766 in its second and none in its third. The noise of every
channel then rested on one to four degrees of freedom. The test of a change
widened its gate by the fewest of them (an F quantile on one degree of
freedom is hundreds wide), so no push passed. The arm was then taken to be
still while it turned 53 units a beat. Keyframes were taken while it moved,
and its wrist eye dropped out of the lock-in.

The noise of such a reading is therefore measured beyond its step. A flip
of one tick is no motion, and two ticks in a beat are. Several channels
flipping a tick in the same beat still add up in the test of a group's
change, and the gate passes two of them together. In four runs of the fake
robot (w11, w13, q10, u20), beats where two or more of an arm's channels
flipped a tick together while the arm rested in the truth were at most
0.24 % of its rest beats (64 of 26574, u20's arm 2; none in w13's first
1432 beats). That is about the gate's own rate at Z = 3 (0.27 %): two flips
score 12 against its 11.83. A push begins only where a target changed, so
such a beat at most ends a push one beat later.

## The rule

- A channel that never changes by more than one step at rest has the noise
  of its rounding alone. For a float that repeats exactly that noise is
  zero, and any change of it is motion; this is right for an echo. For a
  reading in whole units it is the step over the root of twelve, the
  standard deviation of a uniform spread over one step.
- Otherwise its noise is the robust sigma (median absolute deviation) of
  the rest changes beyond one step, divided by the square root of two,
  since a change is the difference of two readings, and never less than
  its rounding's. The degrees of freedom are those of the MAD over that
  many changes.
- A test of a change takes the noise as measured. Its degrees of freedom
  tell the boot how far to trust a rest (a doubling of the rest's beats
  must repeat it), not how wide the test of a change is.
- Rest beats are beats where the target did not change and no push of the
  group is under way. Finding pushes takes the noise, so the two are
  measured in turn until the pushes found stop changing.
- The first round takes every change for motion (noise zero). The other
  starting point, noise from every beat whose target did not change, fails
  for a reading without jitter. There the only nonzero "rest" changes are
  the tails of its own slow responses: the beats after a push where the
  target is held and the reading is still on its way. In the self test's
  rig, an arm that takes three beats to arrive moved 1.2 per beat with
  noise measured from those tails, so its push ended after one beat. The
  wrong noise then never changed, because the push marks it produced kept
  the tails in the rest beats. Starting from zero, a push lasts as long as
  its reading changes at all. The tails are never rest, and the noise
  measured outside the pushes is the jitter alone, or zero.

The test `robot.channels` holds a joint that repeats exactly two beats in
three and jitters by 1e-5 on the third. Its noise must come out at the
jitter's scale, no jitter may count as motion, and a push of 0.1 must.
Counting the exact repeats again makes the test fail.

The test `robot.channels.flips` holds a reading that steps by four and flips
one step at rest. Its noise must come out at four over the root of twelve,
a flip must not count as motion, and a change of two steps must. The test
`robot.channels.gate` gives a group two channels whose noises rest on one
degree of freedom each, and a change of 20 units against their 2.1 must
count as motion. Both fail with the flips counted as jitter and the gate
widened by the fewest degrees of freedom.
