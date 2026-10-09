--  What one press asks of its arm, in the arm's own frame.
--
--  A press lowers the hand onto the table below it until the arm is blocked.
--  Where the tool is, which way is down and how far the table lies below the
--  tip are all taken in the frame of the arm's eye at its reference readings,
--  in the arm's own unit (Driver.Robot.Tool_In_Arm, Up_In_Arm and Table_In_Arm):
--  they need only that arm's own fit. A press so does not wait for its arm to
--  be placed in the world, and an error in the placement turns nothing of it:
--  the direction of a press is the table the arm's own eye saw, and the
--  moves are planned in that frame (Driver.Robot.Motion.Plan_Reach_In_Arm).
--  A frame mixed in (the tool in the world with the arm's own up, say) would
--  turn the press by the whole placement.
--
--  The arm goes on being fitted as it moves, and its frame and unit with it:
--  what is read here is read at one beat and used at that beat, and nothing
--  of a pose is carried from one beat to the next (the way back of a press is
--  to the readings it began from, not to a pose).
--
--  Plain geometry on what the body measured; the decider that moves the arm
--  is Driver.Robot.Hand.Measure. Read inside a held beat, as it does.

with Ada.Strings.Unbounded;
with Driver.Geometry;
with Driver.Robot.Motion;

package Driver.Robot.Hand.Pressing is

   type Aimed is record
      Ok    : Boolean := False;   --  the arm's pose, its table and the eye's mount are measured
      Above : Rigid := Driver.Numerics.Identity;   --  the tool aimed, arm frame
      Turn  : Real := 0.0;        --  how far the aim turns the tool, radians
      Into  : Vec3 := Zero3;      --  down: into the table, unit, arm frame
      Least : Real := Real'Last;  --  the smallest move of the tool that tells from its noise, where the aim leaves it
      Plan  : Driver.Robot.Motion.Plan;   --  from the tool as at O to Above; unset unless Ok
   end record;

   procedure Aim
     (M : Model; Arm : Arm_Id; Eye : Eye_Id; O : Observation; Along : Vec3; Result : out Aimed; Yaw : Real := 0.0);
   --  The tool turned about the eye (a point of the tool frame, so that the
   --  eye keeps its view) by the least rotation that points Along, a
   --  direction of the tool frame, into the table, and a plan to get there.
   --  Yaw turns the hand further about the way down, through the eye, which
   --  keeps Along pointing down and the eye where it is: the poses that point
   --  Along down are a circle of them, and the arm's joints reach some of the
   --  circle and not others (A22's lobe 2 aimed straight stands 0.117 outside
   --  the readings the arm has shown at the least rotation, 0.103 inside them
   --  at 30 degrees).

   procedure Aim_Reaching
     (M       : Model;
      Arm     : Arm_Id;
      Eye     : Eye_Id;
      O       : Observation;
      Along   : Vec3;
      Tip     : Point_Estimate;
      Surface : Driver.Geometry.Plane_Estimate;
      Result  : out Aimed;
      Yaw     : out Real;
      Reaches : out Boolean;
      Unmeasured : out Boolean;
      Why     : out Ada.Strings.Unbounded.Unbounded_String);
   --  The first aim of Along that the arm can be taken to, and from which it can be taken down to the contact the
   --  presses so far predict (Tip in the tool frame, Surface in the arm's: nothing is asked of the descent when they
   --  do not predict it): the least rotation first, then the hand turned about the way down by a quarter either way,
   --  by half a turn, by three quarters, and by the whole of one (eight poses of the circle). A path past an end a
   --  channel showed is not planned (Driver.Robot.Motion.Plan_Reach); Yaw is the turn that was taken. Reaches is
   --  False when none is planned, Result then the last tried, and Why says why the least rotation was not, with the
   --  descent when it is that; Unmeasured is True when that was for want of a measured arm.

   function Least_Push (M : Model; Arm : Arm_Id; O : Observation) return Real;
   --  The smallest move of the tool that tells from its noise, where the tool
   --  is at O: Z times the root of the largest variance of its position there.
   --  Real'Last where the tool's place is not measured.
   --  Aim gives it where the aim leaves the tool (Aimed.Least), which is where
   --  a press lowers it from, not where it stood before the aim: the arm's
   --  frame is the eye at its reference readings, where the tool's place is
   --  known exactly and a push of any size tells from the noise (A15's first
   --  press stood there: 34 pushes, the first 33 of them asking less than
   --  five millionths of a radian; two presses later the least push was 0.06
   --  to 0.2).

   function Lowered
     (M : Model; Arm : Arm_Id; O : Observation; Into : Vec3; By : Real) return Driver.Robot.Motion.Plan;
   --  A plan to the tool as at O moved By along Into.

   function Gap
     (M       : Model;
      Arm     : Arm_Id;
      O       : Observation;
      Tip     : Point_Estimate;
      Surface : Driver.Geometry.Plane_Estimate;
      Into    : Vec3) return Estimate;
   --  How far the tip (a point of the tool frame) is above the surface along
   --  Into, tool as at O, its sigma that of the surface there, the tip and
   --  the tool's pose; unknown when any of them is, or when Into does not
   --  point into the surface. The surface is in the arm's frame, as the
   --  presses that fit it were (Driver.Robot.Hand.Tips.Surface).

end Driver.Robot.Hand.Pressing;
