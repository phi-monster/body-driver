--  How many beats an eye's images trail the readings.
--
--  The eye's motion (the mean displacement of its cells from one beat to
--  the next; a cell whose content moved too far to be resolved counts as
--  far as one can be resolved, so a violent beat still ranks high) is
--  compared with each commandable group's reading speed. Both
--  are differenced in time first, so a long motion contributes a sharp edge
--  where it starts and another where it stops instead of a broad plateau
--  that matches many lags. Both are then replaced by their ranks, so a few
--  violent beats do not dominate. The lag is the shift at which the rank
--  correlation, as a z-score over the beats the two series share, is the
--  highest for any group; every shift the recording allows is tried, and
--  the lag counts as measured only when that best one stands out of the
--  whole family. Telling a lag from another group's pushes takes push
--  trains that do not repeat one another: with every group pushed in the
--  same rhythm, an eye that sees one group lines up with the next group's
--  pushes as well as with its own.

private package Driver.Robot.Lag is

   procedure Measure (M : in out Model);
   --  Re-measures the lag of every eye from the whole stream; an eye whose
   --  motion follows no push stays unmeasured, with lag 0.

end Driver.Robot.Lag;
