--  The one stillness judgment.
--
--  A group is still at a beat when no channel's reading changed
--  significantly against its own noise since the beat before. An eye is
--  still when its cells' displacements since the beat before, each divided
--  by that cell's own displacement noise at rest, add up to a chi-square
--  that is not significant: one test per eye, so the many cells of an image
--  do not make it restless by chance. An eye whose noise is not measured
--  yet, or that has no displacement this beat, gives no evidence either
--  way; so does a group without readings for the two beats. The body is
--  still when nothing that gives evidence moves.

private package Driver.Robot.Stillness is

   function Group_Still (M : Model; G : Group_Id; Beat : Natural) return Boolean;
   function Eye_Still (M : Model; E : Eye_Id; Beat : Natural) return Boolean;

   function All_Still (M : Model; Beat : Natural) return Boolean;
   --  Every group and every eye still at Beat.

end Driver.Robot.Stillness;
