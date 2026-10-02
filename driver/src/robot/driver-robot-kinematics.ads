--  The evidence the kinematics are fitted from, collected as the body moves.
--
--  For every arm whose eye rides on it (Driver.Robot.Graph), a keyframe is
--  kept whenever the arm and its eye are both at rest and the arm's readings
--  differ from every keyframe kept so far: the readings, and what the eye
--  saw. The first keyframe is the reference. The instrument is asked where
--  the reference's query points went in every later keyframe, both ways
--  round, and the answers are kept with the keyframe they belong to. The
--  query points are the centres of the reference eye's cells that respond
--  to the arm's own push (the lock-in): those show the world, not the hand
--  that rides with the eye.
--
--  Every request is submitted and its reply read on a later beat
--  (Driver.Services), so the same evidence follows from a recording.

private package Driver.Robot.Kinematics is

   procedure Observe (M : in out Model; O : Observation);
   --  One beat, after the readings and the eyes were taken in.

   function Matched (M : Model; A : Arm_Id) return Natural;
   --  How many keyframes of the arm have their matches back.

end Driver.Robot.Kinematics;
