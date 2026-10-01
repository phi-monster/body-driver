--  Fingertips measured by touching a surface.
--
--  When a press onto a surface is blocked, the point of the hand that leads
--  along the press direction rests on that surface. For a press aimed at a
--  lobe (its tip's line of sight pointed into the surface) that point is the
--  lobe's tip x, fixed in the arm's tool frame, so n . (R x + t) = d with
--  (R, t) the tool's pose at that beat and n . y = d the surface. One press
--  is one equation; the tip is the point all of them agree on.
--
--  A press can be wrong in two ways the equations cannot hide: something
--  else touched first (the tip stays above the surface), or the hand pressed
--  into a yielding contact (the tip appears below it). Each press is checked
--  against the others: its residual, predicted from all the other presses,
--  must not be significant against the presses' own noise. That noise is the
--  one predicted from the pose and surface uncertainties, raised to the
--  scatter the agreeing presses actually show when that is larger, so a
--  contact that is less repeatable than the kinematics says what it is.

with Driver.Geometry;

package Driver.Robot.Hand.Touch is

   type Press is record
      Tool    : Pose_Estimate;                 --  the arm's last link, world frame, at the blocked beat
      Surface : Geometry.Plane_Estimate;       --  the surface pressed on, world frame
   end record;

   type Press_Array is array (Positive range <>) of Press;

   type Fit_Result is record
      Ok          : Boolean := False;
      Tip         : Point_Estimate;            --  tool frame
      Distance    : Estimate;                  --  On_Ray: along the line of sight from its origin
      Used        : Natural := 0;              --  presses that agree
      Stopped     : Natural := 0;              --  presses left out because the tip stayed above the surface
      Sunk        : Natural := 0;              --  presses left out because the tip went below it
      Scatter     : Real := Real'Last;         --  the agreeing presses' scatter against their predicted noise
   end record;

   function Fit_On_Ray (Presses : Press_Array; Origin : Point_Estimate; Direction : Direction_Estimate)
     return Fit_Result;
   --  The tip on the line of sight Origin + s * Direction (tool frame). Ok is
   --  False with fewer than two agreeing presses or when no press faces the
   --  surface along the line.

   function Fit_Free (Presses : Press_Array) return Fit_Result;
   --  Ok is False with fewer than four agreeing presses, or when their
   --  orientations leave a direction of the tip undetermined to working
   --  precision; a direction they hardly fix shows as a large covariance.

   function Residual (P : Press; Tip : Vec3) return Estimate;
   --  How far above the surface the tip is at that press (negative: below),
   --  with the predicted uncertainty of that height.

end Driver.Robot.Hand.Touch;
