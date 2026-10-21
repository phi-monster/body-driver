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
with Driver.Uncertain;

package Driver.Robot.Hand.Pressing is

   type Aimed is record
      Ok    : Boolean := False;   --  the arm's pose, its table and the eye's mount are measured
      Above : Rigid := Driver.Numerics.Identity;   --  the tool aimed, arm frame
      Turn  : Real := 0.0;        --  how far the aim turns the tool, radians
      Into  : Vec3 := Zero3;      --  down: into the table, unit, arm frame
      Least : Real := Real'Last;  --  the smallest move of the tool that tells from its noise, where the aim leaves it
      Plan  : Driver.Robot.Motion.Plan;   --  from the tool as at O to Above; unset unless Ok
      Yawed : Real := 0.0;        --  the turn about the way down the plan took from the least rotation, radians
   end record;

   --  How far the tool stands off where an aim put it, and what tells it from there.
   type Aim_Offset is record
      Place      : Real := Real'Last;   --  how far its place is off the aim's,
      Place_Sigma : Real := Real'Last;  --  and the sigma it is told against
      Turn       : Real := Real'Last;   --  how far its turn is off the aim's, radians,
      Turn_Sigma : Real := Real'Last;   --  and the sigma it is told against
      --  How far off the aim the arm's own landing leaves the tool: its place and turn at the readings a free push of
      --  the arm falls short by (Steps.Free_Landing), along the way the arm stands short of the aim's readings.
      Allow_Place : Real := 0.0;
      Allow_Turn  : Real := 0.0;
   end record;

   function Off_Aim (M : Model; Arm : Arm_Id; O : Observation; A : Aimed) return Aim_Offset;
   --  The tool as at O against A.Above, each told against the sigma the aim was planned to (the fit's angle sigma,
   --  as Driver.Robot.Motion.Plan_Reach_In_Arm plans it, place and turn alike) with the sigma of what the tool's
   --  pose is told from there: its place's, the root of the largest variance of its position as at O, and the turn
   --  of the way down the aim pointed along. Unknown where any of them is.

   function At_Aim (Off : Aim_Offset) return Boolean is
     (Off.Place_Sigma < Real'Last and then Off.Turn_Sigma < Real'Last
      and then not Driver.Uncertain.Significant (Real'Max (0.0, Off.Place - Off.Allow_Place), Off.Place_Sigma)
      and then not Driver.Uncertain.Significant (Real'Max (0.0, Off.Turn - Off.Allow_Turn), Off.Turn_Sigma));
   --  The tool stands where the aim put it, as far as anything tells: neither its place nor its turn significantly
   --  off. Beyond what the arm's own landing leaves: a servo stops anywhere within its dead band of its target,
   --  and a tool told to a tenth of a millimetre was judged short of every tilted aim the dead band left a millimetre or
   --  two off (u9, servo seed 3 of the fake robot, both arms, 10-19: hand 1's aims "delivered 0.987 of 604" and
   --  "0.989 of 557", the tool 0.172 from the aim against 0.046, turned 0.006 rad against 0.004, and no tilted press
   --  of its tip was made, its tip never tested across its line of sight). What the press needs of an aim is the
   --  tool's pose, not the joints':
   --  a large aim that ends within a few ten-thousandths of its joints' targets, the arm's own sag under its load
   --  where it stands, is judged short against the free pushes of the boot that never carried that load (A89's hand
   --  2: every aim "delivered 1.000 of" 1.36 to 1.46, "blocked", and no press was made in the whole boot).

   procedure Aim
     (M : Model; Arm : Arm_Id; Eye : Eye_Id; O : Observation; Along : Vec3; Result : out Aimed; Yaw : Real := 0.0;
      Shift : Vec3 := [0.0, 0.0, 0.0]);
   --  Shift moves the whole aim by that much, in the arm's frame (Aim_Reaching: across the table, to where nothing
   --  stands).
   --  The tool turned about the eye (a point of the tool frame, so that the
   --  eye keeps its view) by the least rotation that points Along, a
   --  direction of the tool frame, into the table, and a plan to get there.
   --  Yaw turns the hand further about the way down, through the eye, which
   --  keeps Along pointing down and the eye where it is: the poses that point
   --  Along down are a circle of them, and the arm's joints reach some of the
   --  circle and not others (A22's lobe 2 aimed straight stands 0.117 outside
   --  the readings the arm has shown at the least rotation, 0.103 inside them
   --  at 30 degrees).
   --  Along is pointed down exactly and the turn about the way down is Yaw
   --  where the arm reaches the whole pose, else that of the nearest pose
   --  pointing Along down (Motion.Pose_Goal Free_About): an arm of five
   --  joints reaches a pose with Along down at one turn of the circle at most,
   --  and h20 (servo seed 3, 10-17) planned none of its sixteen aims, each left
   --  0.036 rad off its turn at a fifth of the way, its conditioning 1e-6.
   --  Above is then where the plan takes the tool, Yawed the turn it took.

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
      Why     : out Ada.Strings.Unbounded.Unbounded_String;
      Footprint : Real := 0.0);
   --  The aim of Along that the arm can be taken to, and from which it can be taken down to the contact the presses
   --  so far predict (Tip in the tool frame, Surface in the arm's: nothing is asked of the descent when they do not
   --  predict it), of the least rotation and the hand turned about the way down by an eighth of a turn either way, a
   --  quarter, three eighths, and half a turn (eight poses of the circle), where the arm's fit places the tool most
   --  surely (Least_Push there; the least rotation of those as sure): the fit is surest where its sweep moved the arm,
   --  and a joint turned far past what the sweep saw is placed by axes it never pinned (f16u's press aims turned a
   --  joint 273 units from its reference, the sweep 47 at most, and the tool there stood 49 mm uncertain for the
   --  0.19 mm its truth was off at the poses it saw: the presses halved spans of that). A path past an end a
   --  channel showed is not planned (Driver.Robot.Motion.Plan_Reach); Yaw is the turn that was taken. Reaches is
   --  False when none is planned, Result then the last tried, and Why says why the least rotation was not, with the
   --  descent when it is that; Unmeasured is True when that was for want of a measured arm. An aim whose tip would
   --  land where something stands above the table (the arm's eye's tracks, Kinematics.Standing_Near, within what the
   --  landing is uncertain by and Footprint, the radius of the tip's region there in the arm's unit) is not taken
   --  either: the press would meet that, not the table (A79's hand 1, item 7); a tip nothing predicts yet lands
   --  where nothing can be looked at.

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
     (M : Model; Arm : Arm_Id; O : Observation; Into : Vec3; By : Real; Turn_Free : Boolean := False)
      return Driver.Robot.Motion.Plan;
   --  A plan to the tool as at O moved By along Into: turned as it is, or,
   --  Turn_Free, turned as the arm reaches the place (a move made for the
   --  new poses it gives, where the turn does not matter).

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
