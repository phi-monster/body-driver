--  The plant over the lower layers: what Driver.Robot, Driver.Robot.Hand and
--  Driver.World measure, read within one beat's window, and every motion
--  through Driver.Robot.Motion.
--
--  The motion layer's verdicts are taken as it gives them: a step that ends
--  with no push blocked reached what it could (short of the goal when it
--  delivered significantly less than the whole step), and a blocked push,
--  whether it moved first or not, is blocked. What the lower layers do not
--  measure yet stays unknown in the snapshot, and the action layer says so
--  instead of acting on a guess: the lobes' section and the depth of a hand
--  (Driver.Robot.Hand), each arm's step response (Driver.Robot).

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
      O     : Driver.Observations.Observation) return Driver.Action.Snapshots.Snapshot;
   --  The estimates at the beat of O.

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
      Episode : Natural := 0;
      Started : Boolean := False;
   end record;

end Driver.Action.Plants.Live;
