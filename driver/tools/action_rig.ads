--  A rig for the action layer on a measured body (action_matrix): every arm
--  move goes through the real motion layer, Plan_Reach and Follow on the
--  body's fitted kinematics, and the arm's estimates are the body's own; the
--  things, the table and the hand's lobes are the simulated world's
--  (Driver.Action.Plants.Tests), which takes the tool wherever the body's
--  kinematics put the readings. The robot side is Robot_Beat: it takes the
--  arm's joints where the motion layer sends them, in pieces, and stops them
--  where the simulated world stops the tool.
--
--  Like the live plant, the rig reads the models only in a beat's window
--  (Driver.Beats), and a reach, a view or a prediction asked outside one
--  raises Program_Error.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Action.Plants;
with Driver.Action.Plants.Tests;
with Driver.Action.Snapshots;
with Driver.Commands;
with Driver.Numerics;
with Driver.Observations;
with Driver.Robot;
with Driver.Uncertain;

package Action_Rig is

   use Driver.Numerics;

   package Plants renames Driver.Action.Plants;
   package Sim renames Driver.Action.Plants.Tests;

   type Move_Record is record
      Asked   : Rigid;                                  --  the tool pose the action layer asked for
      Reached : Rigid;                                  --  where the readings after the move put the tool
      Outcome : Plants.Step_Outcome := Plants.Refused;
      Why     : Ada.Strings.Unbounded.Unbounded_String;
   end record;

   package Move_Vectors is new Ada.Containers.Vectors (Positive, Move_Record);

   type Rig (Robot : not null access Driver.Robot.Model; World : not null access Sim.World)
     is limited new Plants.Plant with record
      Last    : Driver.Observations.Observation;   --  of the latest window
      Inside  : Boolean := False;
      Episode : Natural := 0;
      Started : Boolean := False;
      Moves   : Move_Vectors.Vector;               --  every arm move, in order
   end record;

   overriding procedure Look (P : in out Rig; S : out Driver.Action.Snapshots.Snapshot);
   overriding procedure Within (P : in out Rig; During : not null access procedure);
   overriding function Reach (P : Rig; Goal : Plants.Arm_Goal) return Plants.Reach_Answer;
   overriding procedure Move (P : in out Rig; O : Plants.Order; R : out Plants.Report);
   overriding function Predicted (P : Rig; T : Driver.Action.Snapshots.Thing_Id; Beats : Natural)
     return Driver.Uncertain.Point_Estimate;
   overriding procedure Learn (P : in out Rig; L : Plants.Lesson);
   overriding function Episode_Over (P : Rig) return Boolean;
   overriding function In_View (P : Rig; Point : Vec3) return Boolean;

   function Observation_Of (M : Driver.Robot.Model; Readings : Driver.Observations.Reading_Vectors.Vector;
                            Beat : Natural) return Driver.Observations.Observation;
   --  What the robot reports at that beat: the readings, and no image.

   procedure Robot_Beat
     (M        : Driver.Robot.Model;
      W        : in out Sim.World;
      Arm      : Driver.Robot.Arm_Id;
      Readings : in out Driver.Observations.Reading_Vectors.Vector;
      Command  : Driver.Commands.Command);
   --  One beat of the robot: every commandable group takes the target the
   --  command carries; the arm's joints go there in pieces, each piece's tool
   --  (the body's kinematics at its readings) put into the simulated world,
   --  and stop at the last piece the world did not stop. Then the world's
   --  beat: drift, and the closers toward their goals.

end Action_Rig;
