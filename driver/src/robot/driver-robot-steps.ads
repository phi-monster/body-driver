--  How each push of a commandable group went: its step response.
--
--  Every beat the push under way of every group is followed. A push starts
--  where the target in effect asks for motion (Channels.Asked). It waits
--  for the reading to move, at most the longest delay any earlier push of
--  the group took (or, before any push was answered, as many beats as the
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
--  its target it stopped. A channel an eye watches (one with a Visible_Step)
--  is short when it stopped short of its ask by at least that step: less
--  than that no eye can tell from where it was asked to be, however exactly
--  the readings tell it (a simulator's joint closes all but a percent of a
--  push and stops, which is far beyond its readings' noise). The channels no
--  eye watches are judged together, along the direction they were asked to
--  move: free motion falls short too (a joint held against gravity settles
--  short of its target), so their shortfall is compared with that of the
--  last push that moved them freely, and they are short when they fall
--  short by significantly more, against the spread of the free pushes'
--  shortfalls (the first pushes have nothing to be compared with and count
--  as free). A push is blocked when a channel is short, or when nothing
--  answered the channels no eye watches. A push answers when its reading
--  moves (Channels.Moving); one that asks a watched channel for less than
--  its visible step moves nothing any eye can see, so that it did not seem
--  to answer says nothing, and it is judged by its shortfall alone.
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
