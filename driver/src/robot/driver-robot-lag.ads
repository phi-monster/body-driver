--  How many beats an eye's images trail the readings.
--
--  The eye's motion (the mean displacement of its cells from one beat to
--  the next) is compared with each commandable group's reading speed. Both
--  are differenced in time first, so a long motion contributes a sharp edge
--  where it starts and another where it stops instead of a broad plateau
--  that matches many lags. Both are then replaced by their ranks, so a few
--  violent beats do not dominate. The lag is the shift at which the rank
--  correlation, as a z-score over the beats the two series share, is the
--  highest for any group; every shift the recording allows is tried.

private package Driver.Robot.Lag is

   procedure Measure (M : in out Model);
   --  Re-measures the lag of every eye from the whole stream; an eye whose
   --  motion follows no group gets lag 0.

end Driver.Robot.Lag;
