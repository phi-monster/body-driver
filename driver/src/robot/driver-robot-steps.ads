--  How each push of a commandable group went: its step response.
--
--  Every beat the push under way of every group is followed. A push starts
--  where the target in effect asks for motion (Channels.Asked). It waits
--  for the reading to move, at most the longest delay any earlier push of
--  the body took, whichever group it moved: a command travels one path to
--  every group (or, before any push was answered, as many beats as the
--  stream had before it: no wait is longer than all the waiting so far). It
--  then lasts until the reading is still, or until the next push cuts it
--  short, or until it is plainly going nowhere: a joint chattering against
--  what stops it moves every beat, by many steps an eye can see, and never
--  comes to rest. A push comes closer to its target when it advances along
--  its ask by a step the one test of motion would see (Channels.Visible)
--  and by more than its own chatter makes: held against something, a push
--  moves about, and a stick-slip against a table sets a new extreme of
--  that movement, a visible step each, at rarer and rarer beats for as long
--  as it is watched, which is no progress. The chatter is the scatter of the
--  push's own progress since it last came closer (that beat included), and
--  a gain is the difference of two beats' progress: it comes closer when
--  that is significant against the scatter, with the degrees of freedom of
--  the beats it rests on (a push that comes closer at every beat has fewer
--  than two, and no chatter to tell a gain from). One still short of its
--  target by such a step, that has not come closer for as long as it took
--  to come as close as it did (and no less than the wait above), ends
--  there, moving, and is judged like any other: a push that moves about
--  against an obstacle ends within as many beats of its last closest point
--  as it took to come that close.
--
--  A push that came to rest is judged by its shortfall: how far short of
--  its target it stopped, along its ask. Free motion falls short too (a
--  joint held against gravity settles short of its target; a simulator's
--  joint closes all but a hair of a push and stops, a few millionths of a
--  radian, which is far beyond its readings' noise and as much as the step
--  an eye can see, and more once the arm has met something than before): a
--  push is short only when it falls short by more than the group's own free
--  pushes did, by more than Driver.Conventions.Z times the most any of them
--  fell short by, either way, and than its readings can tell from none
--  (their noise, and where they repeat exactly the float's resolution,
--  Channels.Resolution). The first pushes of a group have nothing to be
--  compared with and count as free. That is not enough: the shortfall must
--  be a change the one test of motion sees (Channels.Visible), spread over
--  the channels along the ask as the push was, so that where an eye watches
--  a joint it is at least the step that eye can see (less than that no eye
--  can tell from where it was asked to be, however exactly the readings tell
--  it) and the readings' noise can tell it. The two guard each other as in
--  the one test of motion: the step alone would call every free push blocked
--  where the joints' own error is as large as the step, and the free pushes
--  alone would call a push blocked that fell short by a hair no eye could
--  tell from the target. The shortfall weighs each channel by its share of
--  the ask, so that a joint standing a few millionths of a radian off a
--  target it was given before is nothing beside one stopped by something
--  while the others move. A push answers when its reading moves
--  (Channels.Moving); one that asks for more than the one test of motion
--  sees and was not answered is blocked.
--
--  A push that asks for less than that test sees is not judged at all: it
--  moves nothing any eye can see, so that it did not seem to answer says
--  nothing, and whatever the group did meanwhile was not what it asked (a
--  press let go, the arm relaxing from the surface it pressed, a few
--  millionths of a radian asked and half a thousandth moved the other
--  way). There is no delivery to fall short of: its delivered fraction and
--  its shortfall are unknown, it is not blocked, and it is not one of the
--  group's free pushes, whose shortfalls it would otherwise widen. A push
--  that does ask what that test sees, and is moved against its ask by a
--  motion it sees, delivered less than nothing (its delivered fraction is
--  below zero) and is blocked: something outside it, a surface the arm
--  pressed, moved the group more than the push did.
--
--  The judgment is made from the stream alone, as the pushes happen, so the
--  same verdicts follow from a recording (Driver.Robot.Blocked) as from the
--  run that made it (Driver.Robot.Motion.Step reads them).

private package Driver.Robot.Steps is

   procedure Track (M : in out Model; Beat : Natural);
   --  Follows every commandable group's push through the beat just
   --  appended (Channels.Append).

   function Episodes (M : Model; G : Group_Id) return Natural;

   function Latest (M : Model; G : Group_Id) return Episode
     with Pre => Episodes (M, G) > 0;

   function Limiter (M : Model; G : Group_Id; E : Episode) return Natural;
   --  The channel of the group that stopped the push E, 0 when none did. A push that ended at rest and was
   --  blocked fell short along its ask by the sum, over the channels, of each one's share of the ask (its ask over
   --  the ask's length, as Finish weighs them) times its own shortfall. The limiter is the channel whose part of
   --  that is more than Z standard deviations above zero and above every other channel's, each difference against
   --  the two variances together (a channel's own noise over its share, two readings of it, and the scatter of the
   --  group's free pushes along their ask): one channel far short of the rest. A shortfall spread over the
   --  channels in equal parts has none; a channel asked nothing the one test of motion sees contributes none,
   --  however it moved. Only the caller knows what kind of stop it asked about (a table's and a joint's end both
   --  leave a push short, the table's with its channels short by different shares, many sigmas of their noise
   --  apart, as A27's push at 6277 was, 16, 28 and 22 per cent): this tells which channel, not whether the body
   --  stopped it. The push's readings are read where Driver.Robot.Hand.Judge_Push reads them (the Episode keeps
   --  no vectors).

end Driver.Robot.Steps;
