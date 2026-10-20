--  An arm other than the first, put into the world from its closer's flicks seen by an eye whose lens and place are known.
--
--  A pinhole is scale invariant: a point of the arm seen along a ray might be near and the arm's unit small, or far and
--  the unit large, and the eye alone cannot tell which (the arm's unit is its own: the root mean square of its eye's
--  places over its first keyframes). The table is what both arms and the eye have in common: the plane that the arm's
--  own eye saw, in its frame, is the plane the first arm's eye saw, in the world, and with the rays of the flicks it
--  fixes the arm's turn, its place and its unit (X_world = Placement * (Scale * X_arm), as Arm_Fit.Placement has it).
--  Without the table the flicks of one eye would leave a line of arms, one at each scale.

with Ada.Strings.Unbounded;
with Driver.Numerics;
with Driver.Robot.Kinematics.Beacon;
with Driver.Robot.Kinematics.Fit;

package Driver.Robot.Kinematics.Placement is

   --  A plane N * X = Distance, N a unit vector (towards what rests on the plane), each uncertain by a sigma: the
   --  normal in radians, the distance in the frame's own length.
   type Plane is record
      Normal         : Driver.Numerics.Vec3 := [0.0, 0.0, 1.0];
      Distance       : Real := 0.0;
      Sigma_Normal   : Real := Real'Last;
      Sigma_Distance : Real := Real'Last;
   end record;

   Terms  : constant := 10;   --  the arm's turn about the world's axes, its place, the logarithm of its unit, the beacon
   Tables : constant := 3;    --  the residuals that follow the pixels': the table's two tilts and its distance

   --  The fewest sightings that leave the terms a freedom: two residuals a sighting and the table's, more than the terms.
   Least_Sightings : constant := (Terms - Tables) / 2 + 1;

   type Report is record
      Determined : Boolean := False;
      Why        : Ada.Strings.Unbounded.Unbounded_String;
      Placement  : Driver.Numerics.Rigid := Driver.Numerics.Identity;
      Scale      : Real := 1.0;
      Beacon     : Driver.Numerics.Vec3 := [0.0, 0.0, 0.0];   --  in the arm's eye's frame, in the arm's unit
      Covariance : Driver.Numerics.Arrays.Real_Matrix (1 .. Terms, 1 .. Terms) := [others => [others => 0.0]];
      Scatter    : Real := 1.0;      --  the residuals' spread over their stated sigmas, never below one
      Used       : Natural := 0;     --  the sightings kept
      Offered    : Natural := 0;     --  the sightings given
   end record;

   procedure Fit_Arm
     (Seen   : Beacon.Sighting_Array;
      Lens   : Fit.Lens;
      Eye    : Driver.Numerics.Rigid;
      World  : Plane;
      Arm    : Plane;
      Result : out Report);
   --  Seen: the arm's flicks as the eye saw them, each carrying the arm's eye in the arm's own frame (the arm's unit).
   --  Eye: the camera's frame in the world (the first arm's frame, in its unit). World: the table there; Arm: the table
   --  in the arm's own frame. Determined when the placement's turn is within a quarter turn at the sigmas the freedom
   --  allows and the arm's unit is told from nought.

end Driver.Robot.Kinematics.Placement;
