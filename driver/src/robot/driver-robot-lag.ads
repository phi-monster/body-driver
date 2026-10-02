--  How many beats an eye's images trail the readings.
--
--  The eye's motion (the mean displacement of its cells from one beat to
--  the next; a cell whose content moved too far to be resolved counts as
--  far as one can be resolved, so a violent beat still ranks high) is
--  compared with what the body was pushed to do: every commandable group's
--  reading speed while it was pushed. Both are differenced in time first,
--  so a long motion contributes a sharp edge where it starts and another
--  where it stops instead of a broad plateau that matches many lags, and
--  then replaced by their ranks, so a few violent beats do not dominate.
--  At every shift tried, the eye's ranks are fitted on all groups' ranks at
--  once, every coefficient non-negative (a group's motion can only add image
--  motion, and one beat off the true lag the edges anti-correlate); the lag
--  is the shift whose fit noise explains least often (its F tail), and it
--  counts as measured only when that best one stands out of the whole family
--  of shifts tried.
--
--  The shifts tried are those a lag can be told at: a push's response is
--  told from the next push's only when it shows before that one starts, so
--  only shifts up to the median stretch between one burst of pushes and the
--  next (a ramp of targets is one burst). Beyond it a shift lines one push's
--  response up with another push of the same pattern, as a sweep repeated
--  for every joint does.

private package Driver.Robot.Lag is

   procedure Measure (M : in out Model);
   --  Re-measures the lag of every eye from the whole stream; an eye whose
   --  motion follows no push stays unmeasured, with lag 0.

end Driver.Robot.Lag;
