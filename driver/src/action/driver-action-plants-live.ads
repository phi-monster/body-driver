--  The plant over the lower layers: what Driver.Robot, Driver.Robot.Hand and
--  Driver.World measure, read within one beat's window, and every motion
--  through Driver.Robot.Motion.
--
--  What a lower layer does not measure yet stays unknown in the snapshot,
--  and the action layer then says so instead of acting on a guess: thing
--  surfaces and the planes things rest on (Driver.World), the lobes' section
--  and the depth of a hand (Driver.Robot.Hand), each arm's step response and
--  resolution in its tool frame (Driver.Robot).

with Driver.Action.Snapshots;
with Driver.Observations;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.World;

package Driver.Action.Plants.Live is

   function Snapshot_Of
     (Robot : Driver.Robot.Model;
      Hands : Driver.Robot.Hand.Hands;
      Scene : Driver.World.Scene;
      O     : Driver.Observations.Observation;
      Learned : Driver.Action.Snapshots.Thing_Vectors.Vector := Driver.Action.Snapshots.Thing_Vectors.Empty_Vector)
     return Driver.Action.Snapshots.Snapshot;
   --  The estimates at the beat of O; Learned carries friction bounds handed
   --  back earlier, by thing.

   type Live
     (Robot : not null access Driver.Robot.Model;
      Hands : not null access Driver.Robot.Hand.Hands;
      Scene : not null access Driver.World.Scene) is limited new Plant with private;

   overriding procedure Look (P : in out Live; S : out Driver.Action.Snapshots.Snapshot);
   overriding function Reach (P : Live; Goal : Arm_Goal) return Reach_Answer;
   overriding procedure Move (P : in out Live; O : Order; R : out Report);
   overriding function Predicted (P : Live; T : Driver.Action.Snapshots.Thing_Id; Beats : Natural)
     return Driver.Uncertain.Point_Estimate;
   overriding procedure Learn (P : in out Live; L : Lesson);
   overriding function Episode_Over (P : Live) return Boolean;
   overriding function In_View (P : Live; Point : Vec3) return Boolean;

private

   type Live
     (Robot : not null access Driver.Robot.Model;
      Hands : not null access Driver.Robot.Hand.Hands;
      Scene : not null access Driver.World.Scene) is limited new Plant with
   record
      Last    : Driver.Observations.Observation;   --  a copy of the latest look's observation
      Looked  : Boolean := False;
      Episode : Natural := 0;
      Started : Boolean := False;
      Learned : Driver.Action.Snapshots.Thing_Vectors.Vector;
   end record;

end Driver.Action.Plants.Live;
