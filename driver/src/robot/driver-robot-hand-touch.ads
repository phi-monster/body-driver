--  Fingertips and the surfaces they were pressed on, measured together.
--
--  When a press onto a surface is blocked and the arm has come to rest, the
--  point of the hand that leads towards the surface rests on it. For a press
--  aimed at a lobe that point is the lobe's tip x, fixed in the arm's tool
--  frame, so n . (R x + t) = d with (R, t) the tool's pose at that beat and
--  n . y = d the surface. One press is one equation. A tip lies on its line
--  of sight from the eye (one unknown, how far along it) or anywhere (three);
--  a surface is either measured before, and its estimate is a prior, or
--  unknown, and the presses on it find it: a surface takes presses at three
--  places at least, and a tip on its line presses at two tool orientations,
--  or the distance along the line and the surface's offset cannot be told
--  apart. Everything is solved together, so the surface's uncertainty is in
--  every tip's.
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

   type Sight_Array is array (Positive range <>) of Ray_Estimate;
   --  Lines of sight to the tips, tool frame.

   type Surface_Prior (Measured : Boolean := False) is record
      case Measured is
         when True  => Plane : Geometry.Plane_Estimate;   --  in the frame of the presses' poses
         when False => null;
      end case;
   end record;

   type Surface_Prior_Array is array (Positive range <>) of Surface_Prior;

   type Press is record
      Tool    : Pose_Estimate;   --  the arm's last link at rest after the block, in the surfaces' frame
      Sight   : Positive;        --  the tip that touched
      Surface : Positive;        --  what it touched
   end record;

   type Press_Array is array (Positive range <>) of Press;

   type Model is (On_Sight, Free);
   --  On_Sight  each tip on its line of sight
   --  Free      each tip anywhere: the check that the tip is where the eye saw it

   type Tip_Fit is record
      Ok       : Boolean := False;
      Tip      : Point_Estimate;    --  tool frame
      Distance : Estimate;          --  On_Sight: along the line from its origin
      Used     : Natural := 0;      --  presses that agree
      Stopped  : Natural := 0;      --  presses left out because the tip stayed above the surface
      Sunk     : Natural := 0;      --  presses left out because the tip went below it
   end record;

   type Tip_Fit_Array is array (Positive range <>) of Tip_Fit;

   type Plane_Array is array (Positive range <>) of Geometry.Plane_Estimate;

   type Agreement is array (Positive range <>) of Boolean;

   type Fit_Result (Presses, Sights, Surfaces : Natural) is record
      Ok      : Boolean := False;
      Tips    : Tip_Fit_Array (1 .. Sights);
      Planes  : Plane_Array (1 .. Surfaces);   --  each surface as the presses and its prior found it
      Scatter : Real := Real'Last;              --  the agreeing presses' scatter against their predicted noise
      Agrees  : Agreement (1 .. Presses) := [others => False];   --  each press, in order, agrees with the others
   end record;

   function Fit
     (Presses  : Press_Array;
      Sights   : Sight_Array;
      Surfaces : Surface_Prior_Array;
      As       : Model := On_Sight) return Fit_Result
     with Pre => (for all P of Presses => P.Sight in Sights'Range and then P.Surface in Surfaces'Range);
   --  Ok is False when the agreeing presses leave some unknown undetermined
   --  to working precision, or fewer of them remain than the unknowns and
   --  one more; a tip no agreeing press touched is not Ok. Free starts from
   --  the On_Sight solution.

end Driver.Robot.Hand.Touch;
