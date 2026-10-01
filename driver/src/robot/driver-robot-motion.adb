package body Driver.Robot.Motion is

   --  Path A replaces these placeholders.

   Unbuilt : exception;

   procedure Settle (M : in out Model; Beats_Waited : out Natural) is
   begin
      raise Unbuilt with "Settle";
   end Settle;

   procedure Step (M : in out Model; Targets : Driver.Commands.Command; Report : out Step_Report) is
   begin
      raise Unbuilt with "Step";
   end Step;

   function Plan_Reach (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal) return Plan is
     (raise Unbuilt with "Plan_Reach");

   function Status (P : Plan) return Plan_Status is (P.State);
   function Why (P : Plan) return String is (To_String (P.Reason));

   procedure Follow (M : in out Model; P : Plan; Report : out Step_Report) is
   begin
      raise Unbuilt with "Follow";
   end Follow;

end Driver.Robot.Motion;
