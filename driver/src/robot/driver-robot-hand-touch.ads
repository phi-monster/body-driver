--  Fingertips and the surfaces they were pressed on, measured together.
--
--  When a press onto a surface is blocked and the arm has come to rest, the
--  point of the hand that leads towards the surface rests on it, if that
--  point is what stopped the arm. For a press aimed at a lobe that point is
--  the lobe's tip x, fixed in the arm's tool frame, so n . (R x + t) = d with
--  (R, t) the tool's pose at that beat and n . y = d the surface. A tip lies
--  on its line of sight from the eye (one unknown, how far along it) or
--  anywhere (three); a surface is either measured before, and its estimate is
--  a prior, or unknown, and the presses on it find it: a surface takes
--  presses at three places at least, and a tip on its line presses at two
--  tool orientations, or the distance along the line and the surface's offset
--  cannot be told apart. Everything is solved together, so the surface's
--  uncertainty is in every tip's.
--
--  A tip cannot lie below the surface it was pressed on, and an arm is
--  stopped by much besides the tip: itself, the other finger, a joint's end.
--  So a press says only that the tip is at the surface or above it, and a
--  press bounds the tip's distance from above, by the distance at which its
--  line of sight meets the surface from the pose the press was made at (its
--  hit): only a press the tip stopped gives that distance itself. The tip is
--  the lowest hit. A press whose tip, fitted to the others, stays above the
--  surface by more than the noise predicts stopped on something else, and is
--  left out (Stopped), the most above first and one at a time. A press that
--  leaves the tip below the surface does not leave itself out: the others
--  are the ones that are above. A tip that one press fixes (the surface
--  measured before, and one equation for the tip) has nothing to check it:
--  it is provisional. It is Confirmed when a second press at a pose distinct
--  from the first's, within what the poses' uncertainty tells apart, lands
--  on it within the noise: two stops on something else do not land together
--  from two poses, and from one pose they do whatever stopped them.
--
--  The noise is the one predicted from the pose and surface uncertainties,
--  and it is not raised by what the presses are seen to scatter: a scatter
--  taken from a few presses cannot tell a contact less repeatable than the
--  arm from a stop on something else, and the wider it was allowed to grow
--  the more of the stops it took in (A16: hits at 4.80, 9.78 and 16.77 units
--  agreed within a noise raised 520 times, and the tip came out at 14.2).
--  A contact less repeatable than the arm reads as the lowest of its presses.

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
      Ok        : Boolean := False;
      Tip       : Point_Estimate;    --  tool frame
      Distance  : Estimate;          --  On_Sight: along the line from its origin
      Used      : Natural := 0;      --  presses it rests on
      Confirmed : Boolean := False;  --  two of them, at poses distinct from each other, land on it within the noise
      Stopped   : Natural := 0;      --  presses left out because the tip stayed above the surface
      Sunk      : Natural := 0;      --  presses it rests on that left it below the surface
   end record;

   type Tip_Fit_Array is array (Positive range <>) of Tip_Fit;

   type Plane_Array is array (Positive range <>) of Geometry.Plane_Estimate;

   type Agreement is array (Positive range <>) of Boolean;

   type Hit_Array is array (Positive range <>) of Real;

   type Fit_Result (Presses, Sights, Surfaces : Natural) is record
      Ok      : Boolean := False;
      Tips    : Tip_Fit_Array (1 .. Sights);
      Planes  : Plane_Array (1 .. Surfaces);   --  each surface as the presses and its prior found it
      Agrees  : Agreement (1 .. Presses) := [others => False];   --  each press, in order, is one a tip rests on
      Hits    : Hit_Array (1 .. Presses) := [others => 0.0];
      --  each press, in order, left out or not (On_Sight): how far along its
      --  tip's line the line meets the surface from the press's pose; zero
      --  when its tip is not fitted
   end record;

   function Fit
     (Presses  : Press_Array;
      Sights   : Sight_Array;
      Surfaces : Surface_Prior_Array;
      As       : Model := On_Sight) return Fit_Result
     with Pre => (for all P of Presses => P.Sight in Sights'Range and then P.Surface in Surfaces'Range);
   --  Ok is False when the presses that remain leave fewer equations than
   --  unknowns (a measured surface's prior counts as equations: as many
   --  fix the unknowns, one more checks them), or leave some unknown fixed
   --  only to working precision; a tip no press remains for is not Ok. Free
   --  starts from the On_Sight solution.

   function Distinct (A, B : Pose_Estimate) return Boolean;
   --  The poses are apart by more than their uncertainty tells them from:
   --  their positions, or their turns (the rotation taking one to the other,
   --  in the parent frame, against the sum of the two rotations'
   --  covariances).

end Driver.Robot.Hand.Touch;
