--  How each push of a commandable group went: its step response.
--
--  Every beat the push under way of every group is followed. A push starts
--  where the target in effect asks for motion (Channels.Asked). It waits
--  for the reading to move, at most the longest delay any earlier push of
--  the group took (or, before any push was answered, as many beats as the
--  stream had before it: no wait is longer than all the waiting so far). It
--  then lasts until the reading is still, or until the next push cuts it
--  short.
--
--  A push that came to rest is judged by its shortfall: how far short of
--  its target it stopped, along the direction it was asked to move. Free
--  motion falls short too (a joint held against gravity settles short of
--  its target), so the shortfall is compared with that of the last push
--  that moved freely. A push is blocked when it falls short by
--  significantly more, against the readings' noise and the spread of the
--  free pushes' shortfalls, and by more than a negligible fraction of the
--  step (Driver.Conventions.Unchanged_Fraction): pushing further did not
--  get it there. A push the reading never answered is blocked. The first
--  push of a group has nothing to be compared with and counts as free.
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
