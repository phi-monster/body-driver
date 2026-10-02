--  What each group's motion does to what each eye sees.
--
--  For every cell of an eye, the displacement from one beat to the next is
--  regressed on the reading change of every channel of every commandable
--  group, read the measured image lag earlier: for small motions the
--  displacement is linear in the reading changes (the image Jacobian), so
--  one regression separates groups that moved at different times, gives
--  no weight to a group's tiny incidental motion (a reaction to another
--  group), and lets a swaying body or a running mouse only add noise. A
--  group moves a cell when its block of the Jacobian is significant.
--
--  A group whose push moves the whole image of an eye carries that eye;
--  one that moves a patch is seen by it. Whole means a significant majority
--  of the cells that can show a displacement respond (an eye mostly sees
--  the world, not the body parts that ride with it); a patch is a
--  significant minority; anything in between is undecided. That a group
--  moves anything at all is itself tested: the responding cells must
--  outnumber the false alarms the per-cell test makes on its own.

private package Driver.Robot.Lockin is

   procedure Measure (M : in out Model);
   --  Re-measures every group's effect on every eye, and each cell's
   --  displacement noise at rest, from the whole stream. Uses the image
   --  lags (Driver.Robot.Lag) and the channel noise (Driver.Robot.Channels).

   function Moved (M : Model; E : Eye_Id; Beat : Natural) return Boolean;
   --  The eye's image moved at that beat as the lock-in can tell: more of its
   --  textured cells resolved a displacement beyond their noise (as the last
   --  lock-in measured it, never below their floor), or
   --  moved too far to be resolved, than that per-cell test alarms on by
   --  chance. The step that makes an eye move this way is one the lock-in
   --  can measure; a change the stillness judgment sees (a pixel's rounding
   --  flipping) can be far smaller.

   function Shift (M : Model; E : Eye_Id; G : Group_Id; Channel : Positive) return Real;
   --  How many pixels the eye's image moves per reading unit of the channel:
   --  the median over the cells that respond to its group; zero when none
   --  does or it was not measured.

end Driver.Robot.Lockin;
