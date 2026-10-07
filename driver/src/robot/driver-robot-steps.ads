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
--  its ask by a step the one test of motion would see (Channels.Visible);
--  one still short of its target by such a step, that has not come closer
--  for as long as it took to come as close as it did, ends there, moving,
--  and is judged like any other.
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
--  (Channels.Moving); one that asks for less than the one test of motion
--  sees moves nothing any eye can see, so that it did not seem to answer
--  says nothing, and is judged by its shortfall alone; one that asks for
--  more and was not answered is blocked.
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

end Driver.Robot.Steps;
