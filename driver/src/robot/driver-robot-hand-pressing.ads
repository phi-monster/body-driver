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

with Driver.Geometry;
with Driver.Robot.Motion;

package Driver.Robot.Hand.Pressing is

   type Aimed is record
      Ok    : Boolean := False;   --  the arm's pose, its table and the eye's mount are measured
      Above : Rigid := Driver.Numerics.Identity;   --  the tool aimed, arm frame
      Into  : Vec3 := Zero3;      --  down: into the table, unit, arm frame
      Least : Real := Real'Last;  --  the smallest move of the tool that tells from its noise
      Plan  : Driver.Robot.Motion.Plan;   --  from the tool as at O to Above; unset unless Ok
   end record;

   procedure Aim (M : Model; Arm : Arm_Id; Eye : Eye_Id; O : Observation; Along : Vec3; Result : out Aimed);
   --  The tool turned about the eye (a point of the tool frame, so that the
   --  eye keeps its view) by the least rotation that points Along, a
   --  direction of the tool frame, into the table, and a plan to get there.

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
