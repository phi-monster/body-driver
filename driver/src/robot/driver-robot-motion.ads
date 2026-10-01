--  The motion primitives every decider uses. Each exists once: there is one
--  way to wait for the body to settle, one way to take a step and judge what
--  happened, and one solver for reaching a pose, used both to ask whether a
--  pose is reachable and to move there.
--
--  These are decider operations: they exchange beats with the main loop
--  through Driver.Beats and must only be called from the decider task.

with Ada.Strings.Unbounded;
with Driver.Commands;

package Driver.Robot.Motion is

   use Ada.Strings.Unbounded;

   procedure Settle (M : in out Model; Beats_Waited : out Natural);
   --  Holds until Still (M) and returns how many beats that took.

   type Step_Outcome is (Reached, Blocked, Short);
   --  Reached  every targeted channel arrived within its noise
   --  Blocked  a channel stopped short and pushing further does not move it
   --  Short    the body moved but delivered only part of the step

   type Step_Report is record
      Outcome   : Step_Outcome := Short;
      Beats     : Natural := 0;
      Delivered : Estimate;          --  delivered fraction of the commanded step
      Detail    : Unbounded_String;  --  which channels, and by how much, for the log
   end record;

   procedure Step (M : in out Model; Targets : Driver.Commands.Command; Report : out Step_Report);
   --  Sends the targets, waits until the body settles, and judges the step.

   type Pose_Goal is record
      Pose          : Rigid;
      Position_Only : Boolean := False;   --  orientation free
   end record;

   type Plan_Status is (Planned, Unreachable, Unmeasured);

   type Plan is private;

   function Plan_Reach (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal) return Plan;
   --  A joint path from the arm's configuration at O to the goal, solved
   --  along the way so it never jumps between solution branches.

   function Status (P : Plan) return Plan_Status;
   function Why (P : Plan) return String;
   --  For Unreachable: which limit, joint or distance stopped it.

   procedure Follow (M : in out Model; P : Plan; Report : out Step_Report)
     with Pre => Status (P) = Planned;
   --  Moves along a planned path, step by step, judging every step.

private

   type Plan is record
      State  : Plan_Status := Unmeasured;
      Reason : Unbounded_String;
   end record;

end Driver.Robot.Motion;
