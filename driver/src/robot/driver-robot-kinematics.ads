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

   function Pending (M : Model) return Natural;
   --  Match requests of every arm not answered yet.

   procedure Refit (M : in out Model);
   --  Fits every arm whose matches changed since its last fit
   --  (Driver.Robot.Kinematics.Fit). Its sightings are the matches whose
   --  round trip comes back to where it started, within the noise of all the
   --  round trips.

   function Eye_In_Reference (M : Model; A : Arm_Id; Readings : Real_Array) return Rigid;
   --  The arm's eye at those readings, in the frame of its eye at the
   --  reference keyframe; the identity until the arm is fitted.

   function Fitted (M : Model; A : Arm_Id) return Boolean;

   function Angle_Sigma (M : Model; A : Arm_Id) return Real;
   --  The angle one pixel of the fit's measured noise subtends at the arm's
   --  eye: the uncertainty of a line of sight, and of the eye's turn.

   function Ray_In_Eye (M : Model; A : Arm_Id; U, V : Real) return Vec3;
   --  The unit line of sight through pixel (U, V) of the arm's eye.

   procedure Project_In_Eye (M : Model; A : Arm_Id; P : Vec3; U, V : out Real; In_Front : out Boolean);
   --  Where a point of the arm's eye frame lands in its image.

end Driver.Robot.Kinematics;
